// Offline functional experiment. Generator truth is used only for constructing
// mixtures and scoring; the online SIC receives public templates and coarse ROI.
#include <gnuradio/uwb/uwb_sic_unified.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>
#include <stdexcept>
#include <vector>

using namespace gr::uwb;
using cf = std::complex<float>;
constexpr size_t sps = 1016, q = 512;
constexpr double fs = 998.4e6, pi = 3.14159265358979323846;

struct Packet {
    mod::HrpModConfig cfg;
    mod::HrpModScratch scratch;
    std::vector<uint8_t> bytes;
    std::vector<cf> wave, sync, prefix;
    Packet(size_t code, unsigned seed, size_t repetitions=64, size_t psdu_bytes=32) {
        cfg.code_index = code; cfg.sync_repetitions = repetitions;
        cfg.sfd_mode = "4z2"; cfg.peak_amplitude = .8f;
        if(psdu_bytes<2 || psdu_bytes>127) throw std::runtime_error("PSDU must include FCS and fit 127 bytes");
        bytes = mod::make_random_psdu(psdu_bytes-2, seed, true);
        const size_t cap = mod::packet_samples_998p4(repetitions, 8, bytes.size(), false, &cfg.pulse) + 512;
        scratch.reserve(127, cap); wave.resize(cap); prefix.resize(cap);
        size_t len = 0;
        if (!mod::modulate_one(bytes.data(), bytes.size(), cfg, scratch, wave.data(), cap, len))
            throw std::runtime_error("packet generation failed");
        wave.resize(len); sync.assign(wave.begin(), wave.begin()+sps);
        if (!mod::modulate_prefix_waveform(cfg, scratch, prefix.data(), cap, len))
            throw std::runtime_error("prefix generation failed");
        prefix.resize(len);
    }
};

struct Observation {
    int status; bool sfd, fcs, bytes;
    int radar_status; int64_t detected;
    size_t peak; double amplitude;
    std::vector<cf> cir;
};

Observation observe(const std::vector<cf>& y, const Packet& target, int64_t predicted=q) {
    auto profile = demod::Qm35825Profile::Default();
    profile.code_index = target.cfg.code_index;
    profile.preamble_repetitions = target.cfg.sync_repetitions;
    profile.cir_repetitions=target.cfg.sync_repetitions-profile.cir_skip_initial_repetitions;
    profile.timing_coarse_stride = 1;
    profile.rx_fast_sfd = true; profile.rx_fast_cfo_rotation = true;
    profile.rx_seed_refine_tail = true; profile.rx_softchip_on_demand = true;
    profile.rx_fused_payload_cmf = true; profile.rx_fused_phr_cmf = true;
    demod::core::DemodScratch ds; ds.reserve(y.size());
    auto d = demod::core::demodulate_one(y.data(), y.size(), profile, 0, predicted, 0, target.sync, ds);
    radar::RadarCirConfig cfg; cfg.code_index = target.cfg.code_index;
    cfg.sync_repetitions=target.cfg.sync_repetitions;
    cfg.cir_repetitions=cfg.sync_repetitions-cfg.cir_skip_initial;
    radar::RadarCirCoreScratch scratch;
    if (!radar::prepare_radar_cir_core(cfg, target.sync.data(), sps, cfg.cir_pre, cfg.cir_post, scratch))
        throw std::runtime_error("radar preparation failed");
    radar::RadarCirResult r;
    radar::radar_cir_one(y.data(), y.size(), predicted + cfg.sync_repetitions*sps, cfg, scratch, r);
    return {int(d.status), d.sfd.ok, d.payload.fcs_pass, d.payload.bytes == target.bytes,
            int(r.status), r.preamble_start_sample, r.peak_tap, r.peak_abs,
            std::vector<cf>(scratch.cir.raw_taps.begin(), scratch.cir.raw_taps.begin()+r.tap_count)};
}

double energy(const std::vector<cf>& x) {
    double e=0; for (auto a:x) e+=std::norm(std::complex<double>(a)); return e;
}

void emit(const char* role, const char* scenario, unsigned seed, double isr, double cfo,
          const char* path, const Observation& o, const Observation& clean,
          bool committed, const char* reason, double suppression) {
    double err=0, ref=energy(clean.cir);
    double relative=-1, amp=-999, shift=-999;
    if (o.radar_status==0 && clean.radar_status==0 && o.cir.size()==clean.cir.size()) {
        for(size_t j=0;j<o.cir.size();++j) err+=std::norm(std::complex<double>(o.cir[j]-clean.cir[j]));
        relative=std::sqrt(err/std::max(ref,1e-30));
        amp=20*std::log10(std::max(o.amplitude,1e-30)/std::max(clean.amplitude,1e-30));
        shift=double(o.peak)-double(clean.peak);
    }
    std::cout<<role<<','<<scenario<<','<<seed<<','<<isr<<','<<cfo<<','<<path<<','
        <<o.status<<','<<o.sfd<<','<<o.fcs<<','<<o.bytes<<','<<o.radar_status<<','
        <<o.detected<<','<<shift<<','<<amp<<','<<relative<<','<<committed<<','
        <<reason<<','<<suppression<<'\n';
}

int main(int argc,char** argv) {
    try {
        unsigned seeds=argc>1?std::stoul(argv[1]):8;
        const std::string mode=argc>2?argv[2]:"work";
        const std::string directory=argc>3?argv[3]:"";
        const size_t target_reps=mode=="dump" && argc>5?std::stoull(argv[5]):64;
        const size_t psdu_bytes=mode=="dump" && argc>6?std::stoull(argv[6]):32;
        const size_t interference_reps=mode=="dump" && argc>7?std::stoull(argv[7]):64;
        std::vector<float> taps;
        if(mode=="native") {
            if(argc!=5) throw std::runtime_error("native mode requires directory and tap file");
            std::ifstream f(argv[4],std::ios::binary|std::ios::ate);
            if(!f) throw std::runtime_error("cannot open taps");
            const size_t bytes=f.tellg();
            if(bytes==0 || bytes%sizeof(float)) throw std::runtime_error("invalid tap bytes");
            taps.resize(bytes/sizeof(float)); f.seekg(0);
            f.read(reinterpret_cast<char*>(taps.data()),bytes);
        } else if(mode!="dump" && mode!="work") throw std::runtime_error("unknown mode");
        std::cout<<"role,scenario,seed,isr_db,cfo_hz,path,demod_status,sfd,fcs,psdu_exact,radar_status,detected_start,cir_peak_shift,cir_peak_gain_db,cir_relative_l2,sic_committed,sic_reason,residual_improvement_db\n";
        struct Scenario { const char* name; double isr,cfo; size_t offset; bool interference,noise; };
        const Scenario scenarios[] = {
            {"clean_control",0,0,0,false,false},
            {"noise_control",0,0,0,false,true},
            {"sync_isr0",0,0,0,true,true},
            {"sync_isr10",10,0,0,true,true},
            {"sync_isr20",20,0,0,true,true},
            {"sync_isr20_cfo50k",20,50000,0,true,true},
            // SYNC+SFD ends at 72*sps, PHR at about 94*sps; start
            // interference at 104*sps so this cell overlaps actual payload.
            {"payload_isr20",20,0,104*sps,true,true}
        };
        for(size_t code:{size_t(9),size_t(10)}) for(unsigned seed=0;seed<seeds;++seed) {
            // Sweep 16 configured work-grid origins in native mode, exposing
            // different fractional phases on the 48/65 synthesis grid.
            const size_t q=::q+(mode=="work"?0:seed%16);
            Packet target(code,1000+seed,target_reps,psdu_bytes),
                   interferer(code==9?10:9,2000+seed,interference_reps,psdu_bytes);
            if(mode=="dump") {
                std::ofstream sync(directory+"/sync_"+std::to_string(code)+".bin",std::ios::binary);
                sync.write(reinterpret_cast<const char*>(target.sync.data()),target.sync.size()*sizeof(cf));
                std::ofstream psdu(directory+"/psdu_"+std::to_string(code)+"_"+std::to_string(seed)+".bin",std::ios::binary);
                psdu.write(reinterpret_cast<const char*>(target.bytes.data()),target.bytes.size());
            }
            for(const auto& sc:scenarios) {
                if(mode=="dump" && argc>4 && std::string(sc.name)!=argv[4]) continue;
                const size_t iq=q+sc.offset;
                size_t n=std::max(q+target.wave.size(),iq+interferer.prefix.size())+1024;
                std::vector<cf> clean(n), raw(n), out(n), ihat(n);
                std::copy(target.wave.begin(),target.wave.end(),clean.begin()+q);
                // 20 dB SNR referenced to desired packet mean power, identical
                // noise in clean/raw/SIC. No per-path normalization or oracle fit.
                std::mt19937 rng(3000+seed); std::normal_distribution<float> normal(0,1);
                double sigma=sc.noise?std::sqrt(energy(target.wave)/target.wave.size()*.01/2):0;
                for(auto& z:clean) z+=cf(normal(rng)*sigma,normal(rng)*sigma);
                raw=clean;
                double es=0; for(size_t j=0;j<64*sps;++j) es+=std::norm(target.wave[j]);
                double ei=0; for(size_t j=0;j<64*sps;++j) ei+=std::norm(interferer.prefix[j]);
                double gain=std::sqrt(es/ei*std::pow(10.,sc.isr/10));
                if(sc.interference) for(size_t j=0;j<interferer.prefix.size();++j)
                    raw[iq+j]+=interferer.prefix[j]*cf(gain*std::cos(2*pi*sc.cfo*j/fs+.4),gain*std::sin(2*pi*sc.cfo*j/fs+.4));
                const std::string stem=directory+"/"+std::to_string(code)+"_"+std::to_string(seed)+"_"+sc.name;
                if(mode=="dump") {
                    std::ofstream file(stem+".cf32",std::ios::binary);
                    file.write(reinterpret_cast<const char*>(clean.data()),clean.size()*sizeof(cf));
                    file.write(reinterpret_cast<const char*>(raw.data()),raw.size()*sizeof(cf));
                    if(!file) throw std::runtime_error("source dump failed");
                    continue;
                }
                int64_t predicted=q, interference_predicted=iq;
                Observation ideal;
                if(mode=="native") {
                    ideal=observe(clean,target,q);
                    std::ifstream file(stem+".sc16",std::ios::binary);
                    uint64_t count=0; double scale=0;
                    file.read(reinterpret_cast<char*>(&count),sizeof(count));
                    file.read(reinterpret_cast<char*>(&scale),sizeof(scale));
                    if(!file || count==0 || count>10000000 || !(scale>0))
                        throw std::runtime_error("invalid native header: "+stem);
                    core::RationalResampler65_48Core resampler(taps);
                    resampler.set_num_workers(1); resampler.set_fixed_phase_schedule(true);
                    auto convert=[&](std::vector<cf>& output) {
                        std::vector<std::complex<int16_t>> input(count);
                        file.read(reinterpret_cast<char*>(input.data()),count*sizeof(input[0]));
                        if(!file) throw std::runtime_error("truncated native input");
                        const size_t expected=resampler.expected_output_length(count);
                        output.resize(expected); resampler.reset();
                        auto r=resampler.process_sc16(input.data(),count,float(scale),output.data(),output.size());
                        if(r.consumed!=count) throw std::runtime_error("native input not fully consumed");
                        size_t produced=r.produced;
                        while(produced<expected) {
                            size_t z=resampler.flush(output.data()+produced,expected-produced);
                            if(!z) break;
                            produced+=z;
                        }
                        if(produced!=expected) throw std::runtime_error("resampler output length mismatch");
                    };
                    convert(clean); convert(raw);
                    n=clean.size(); out.resize(n); ihat.resize(n);
                    // Keep actual output samples untouched. Shift public nominal
                    // coordinates by the known FIR delay; acquisition refines
                    // the fractional residual, without oracle alignment.
                    const double delay=(taps.size()-1)/96.;
                    predicted=std::llround(q+delay);
                    interference_predicted=std::llround(iq+delay);
                }
                sic::SicUnifiedConfig cfg; cfg.policy=sic::SicPolicy::PreambleOnly;
                cfg.backend=sic::SicEstimatorBackend::Ipatov;
                cfg.comm_code=interferer.cfg.code_index; cfg.samples_per_symbol=sps;
                cfg.sync_reps=64; cfg.sample_rate=fs;
                cfg.roi_lo=interference_predicted-64; cfg.roi_hi=interference_predicted+64;
                cfg.sub_roi_lo=0; cfg.sub_roi_hi=n;
                cfg.max_window_samples=n; cfg.max_packet_samples=n;
                sic::SicUnifiedTemplates t;
                t.sync=interferer.sync.data(); t.sync_len=sps;
                t.preamble=interferer.prefix.data(); t.preamble_len=interferer.prefix.size();
                sic::SicUnifiedScratch scratch; std::string error;
                if(!sic::prepare_sic_unified(cfg,t,scratch,&error)) throw std::runtime_error(error);
                sic::SicUnifiedResult result; result.out=out.data(); result.i_hat=ihat.data();
                if(!sic::sic_process_unified(raw.data(),n,t,cfg,scratch,result)) throw std::runtime_error("SIC API failure");
                // Offline error to the paired desired+noise reference, not total
                // power reduction (which could reward deleting the desired signal).
                double before=0,after=0;
                for(size_t j=0;j<n;++j) {
                    before+=std::norm(std::complex<double>(raw[j]-clean[j]));
                    after+=std::norm(std::complex<double>(out[j]-clean[j]));
                }
                double suppression=sc.interference?10*std::log10(std::max(before,1e-30)/std::max(after,1e-30)):0;
                const char* role=code==9?"sensing9":"comm10";
                auto baseline=observe(clean,target,predicted);
                if(mode=="native") emit(role,sc.name,seed,sc.isr,sc.cfo,"ideal",ideal,ideal,false,"not_run",0);
                emit(role,sc.name,seed,sc.isr,sc.cfo,"clean",baseline,mode=="native"?ideal:baseline,false,"not_run",0);
                emit(role,sc.name,seed,sc.isr,sc.cfo,"raw",observe(raw,target,predicted),baseline,false,"not_run",0);
                emit(role,sc.name,seed,sc.isr,sc.cfo,"sic",observe(out,target,predicted),baseline,result.committed,result.reason?result.reason:"unknown",suppression);
            }
        }
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
