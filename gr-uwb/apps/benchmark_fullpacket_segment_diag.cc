// Isolated CFO-kernel replay. Unchecked variants are diagnostics only.
#include "fullpacket_segment_diagnostics.h"
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace fs = std::filesystem;
using namespace gr::uwb;
namespace diag = gr::uwb::experiment::segment_diag;
using cf = std::complex<float>;
using cd = std::complex<double>;

namespace {
std::vector<cf> read_iq(const fs::path& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot read " + path.string());
    const auto bytes = f.tellg();
    if (bytes <= 0 || bytes % sizeof(cf)) throw std::runtime_error("invalid CF32 size");
    std::vector<cf> out(size_t(bytes) / sizeof(cf));
    f.seekg(0); f.read(reinterpret_cast<char*>(out.data()), bytes);
    if (!f) throw std::runtime_error("short CF32 read");
    return out;
}
bool finite(cf value)
{
    return std::isfinite(value.real()) && std::isfinite(value.imag());
}
struct Snapshot {
    size_t source=0;
    sic::SicModel model;
    std::vector<cf> iq,expected;
    std::vector<uint8_t> phr,psdu;
};
std::vector<Snapshot> capture(const fs::path& input_case)
{
    boost::property_tree::ptree pub;
    boost::property_tree::read_json((input_case / "public.json").string(), pub);
    const size_t n = pub.get<size_t>("n");
    if (n < 9000 || n > 2000000 || pub.get<double>("sample_rate") != 998400000. ||
        pub.get<int>("comm_code", 10) != 10 || pub.get<int>("sync_reps", 64) != 64 ||
        pub.get<std::string>("sfd_mode", "4z2") != "4z2" ||
        pub.get<std::string>("pulse_shape", "legacy") != "legacy")
        throw std::runtime_error("unsupported public PHY/context");
    const auto sync = read_iq(input_case / "comm_sync.cf32");
    const auto preamble = read_iq(input_case / "comm_preamble.cf32");
    sic::SicConfig cfg;
    cfg.mode = sic::SicMode::Estimate;
    cfg.roi_lo = pub.get<int64_t>("roi_lo"); cfg.roi_hi = pub.get<int64_t>("roi_hi");
    cfg.observed_end_sample = pub.get<int64_t>("observed_end");
    cfg.detect_threshold = .4; cfg.validate_threshold = .25;
    cfg.online_validation_reps = 0; cfg.k_paths = 4; cfg.coarse_stride = 1;
    cfg.sub_roi_lo = pub.get<int64_t>("consumer_lo", 0);
    cfg.sub_roi_hi = pub.get<int64_t>("consumer_hi", n);
    if (cfg.sub_roi_lo != 0 || cfg.sub_roi_hi != int64_t(n) ||
        cfg.observed_end_sample <= 0 || cfg.observed_end_sample > int64_t(n))
        throw std::runtime_error("diagnostic requires complete consumer context");
    sic::SicScratch estimator;
    std::string error;
    if (!sic::prepare_sic_core(cfg, sync.data(), sync.size(), preamble.data(),
                               preamble.size(), estimator, &error))
        throw std::runtime_error(error);
    sic::SicTemplates templates;
    templates.sync = sync.data(); templates.sync_len = sync.size();
    templates.preamble = preamble.data(); templates.preamble_len = preamble.size();
    demod::Qm35825Profile profile;
    profile.code_index = 10; profile.timing_search_margin = 256;
    profile.timing_coarse_stride = 1;
    demod::core::DemodScratch decode_scratch; decode_scratch.reserve(n);
    mod::HrpModConfig tx;
    tx.code_index = 10; tx.sync_repetitions = 64; tx.sfd_mode = "4z2";
    tx.peak_amplitude = .8f;
    sic::FullPacketScratch full; full.reserve(n);
    if (!full.prepare_prefix_cache()) throw std::runtime_error("prefix preparation failed");
    std::vector<cf> output(n), reconstructed(n);
    std::vector<fs::path> windows;
    for (const auto& entry : fs::directory_iterator(input_case / "windows"))
        if (entry.is_directory() && fs::exists(entry.path() / "raw.cf32"))
            windows.push_back(entry.path());
    std::sort(windows.begin(), windows.end());
    if (windows.empty()) throw std::runtime_error("no source windows");
    if (windows.size() > 16) windows.resize(16);
    std::vector<Snapshot> snapshots;
    for (size_t index = 0; index < windows.size(); ++index) {
        const auto iq = read_iq(windows[index] / "raw.cf32");
        if (iq.size() != n) throw std::runtime_error("raw/public size mismatch");
        sic::SicModel model;
        if (!sic::estimate_model(iq.data(), n, cfg.observed_end_sample, templates,
                                 cfg, estimator, model) || !model.valid)
            throw std::runtime_error("source model failed: " + windows[index].string());
        const auto decoded = demod::core::demodulate_one(
            iq.data(), n, profile, index, model.origin, 0, sync, decode_scratch);
        if (!decoded.payload.fcs_pass)
            throw std::runtime_error("source FCS failed: " + windows[index].string());
        const auto result = sic::fullpacket_cancel(
            iq.data(), n, decoded.phr.phr_bits, decoded.payload.bytes, true, tx,
            model, sync.data(), sync.size(), 0, n, full, output.data(), reconstructed.data());
        if (!result.committed || result.sub_lo < 0 || result.sub_hi <= result.sub_lo ||
            result.sub_hi > int64_t(n))
            throw std::runtime_error(std::string("source reconstruction failed: ") + result.reason);
        Snapshot snap;
        snap.source=index;snap.model=model;snap.iq=iq;snap.expected=reconstructed;
        snap.phr=decoded.phr.phr_bits;snap.psdu=decoded.payload.bytes;
        snapshots.push_back(std::move(snap));
    }
    return snapshots;
}

void self_test() {
    for(size_t length:{size_t(16),size_t(32),size_t(127)}) {
        const size_t cap=mod::packet_samples_998p4(64,8,length,false)+512;
        sic::FullPacketScratch original,segmented;
        original.reserve(cap);segmented.reserve(cap);
        if(!original.prepare_prefix_cache()||!segmented.prepare_prefix_cache())throw std::runtime_error("cache init");
        mod::HrpModConfig tx;tx.code_index=10;tx.sync_repetitions=64;tx.sfd_mode="4z2";tx.peak_amplitude=.8f;
        std::vector<uint8_t> bytes(length-2,37);mod::append_ieee_fcs(bytes);
        std::vector<int8_t> bits;mod::detail::encode_phr19(length,64,false,bits);
        std::vector<uint8_t> phr(bits.begin(),bits.end());
        size_t count=0;
        if(!mod::modulate_one(bytes.data(),bytes.size(),tx,original.modulator,original.waveform.data(),cap,count))throw std::runtime_error("modulate");
        std::vector<cf> sync(original.waveform.begin(),original.waveform.begin()+1016);
        std::vector<cf> iq(cap,cf(.001f,-.002f)),out(cap),other(cap),a(cap),b(cap);
        sic::SicModel m;m.valid=true;m.state=sic::SicState::Committed;m.n_paths=2;
        m.tau_abs[0]=128.99;m.tau_abs[1]=151.4;m.gain[0]={.8f,-.2f};m.gain[1]={.1f,.2f};
        m.cfo_hz=-50000;m.sample_rate=998.4e6;m.samples_per_symbol=1016;m.nominal_sync_reps=64;m.phase_reference_sample=2048;
        for(bool pass:{true,true,false}) {
            auto x=sic::fullpacket_cancel(iq.data(),cap,phr,bytes,pass,tx,m,sync.data(),sync.size(),0,cap,original,out.data(),a.data());
            auto y=diag::run(iq.data(),cap,phr,bytes,pass,tx,m,sync.data(),sync.size(),0,cap,segmented,other.data(),b.data());
            if(x.committed!=y.committed||std::string(x.reason)!=y.reason)throw std::runtime_error("self-test state mismatch");
            for(size_t j=0;j<cap;++j) {
                if(!finite(b[j])||std::abs(a[j]-b[j])>2e-6f||std::abs(out[j]-other[j])>2e-6f)throw std::runtime_error("self-test IQ mismatch");
                if(!pass&&(other[j]!=iq[j]||b[j]!=cf(0)))throw std::runtime_error("self-test bypass mismatch");
            }
        }
    }
    std::cout<<"self-test: lengths16/32/127, cold/hot cache, fractional multipath, negative phase, failed FCS passed\n";
}
} // namespace

int main(int argc,char** argv) try {
    if(argc==2&&std::string(argv[1])=="--self-test"){self_test();return 0;}
    fs::path input_case,outdir;size_t observations=128,rounds=2,warmup=16;
    for(int k=1;k<argc;++k){std::string key=argv[k];auto next=[&](){if(++k>=argc)throw std::runtime_error("missing argument");return std::string(argv[k]);};
        if(key=="--case")input_case=next();else if(key=="--out")outdir=next();
        else if(key=="--observations")observations=std::stoull(next());else if(key=="--rounds")rounds=std::stoull(next());
        else if(key=="--warmup")warmup=std::stoull(next());else throw std::runtime_error("unknown option");}
    if(input_case.empty()||outdir.empty()||!observations||!rounds||observations>100000||rounds>100||warmup>10000)throw std::runtime_error("invalid arguments");
    if(fs::exists(outdir)&&!fs::is_empty(outdir))throw std::runtime_error("choose new output");
    fs::create_directories(outdir);
    const auto snapshots=capture(input_case);const auto sync=read_iq(input_case/"comm_sync.cf32");const size_t n=snapshots.front().iq.size();
    sic::FullPacketScratch original,segmented;original.reserve(n);segmented.reserve(n);
    if(!original.prepare_prefix_cache()||!segmented.prepare_prefix_cache())throw std::runtime_error("cache init");
    mod::HrpModConfig tx;tx.code_index=10;tx.sync_repetitions=64;tx.sfd_mode="4z2";tx.peak_amplitude=.8f;
    std::vector<cf> out(n),ihat(n);double worst=0,relative=0;size_t checks=0,exact=0;
    std::ofstream log(outdir/"windows.jsonl");log<<std::setprecision(15);
    auto replay=[&](const Snapshot& source,bool split,const char* temperature,size_t round,size_t index,bool emit){
        // Timing excludes file I/O, model estimation, decode and verification.
        diag::Result measured;
        if(split)measured=diag::run(source.iq.data(),n,source.phr,source.psdu,true,tx,source.model,sync.data(),sync.size(),0,n,segmented,out.data(),ihat.data());
        else static_cast<sic::FullPacketResult&>(measured)=sic::fullpacket_cancel(source.iq.data(),n,source.phr,source.psdu,true,tx,source.model,sync.data(),sync.size(),0,n,original,out.data(),ihat.data());
        if(!measured.committed||measured.packet_samples!=133568)throw std::runtime_error("replay reconstruction failed or unexpected length");
        double err=0,energy=0;
        for(size_t j=0;j<n;++j){
            if(!finite(ihat[j])||!finite(out[j]))throw std::runtime_error("nonfinite output");
            worst=std::max(worst,double(std::abs(ihat[j]-source.expected[j])));
            const cd d=cd(ihat[j])-cd(source.expected[j]);err+=std::norm(d);energy+=std::norm(cd(source.expected[j]));
            if(std::abs(ihat[j]-source.expected[j])>2e-6f || std::abs(out[j]-(source.iq[j]-source.expected[j]))>2e-6f)throw std::runtime_error("whole model/output mismatch");
        }
        relative=std::max(relative,std::sqrt(err/std::max(energy,1e-30)));++checks;
        exact+=std::memcmp(ihat.data(),source.expected.data(),n*sizeof(cf))==0;
        if(!emit)return;
        log<<"{\"variant\":\""<<(split?"segmented":"production")<<"\",\"temperature\":\""<<temperature<<"\",\"round\":"<<round<<",\"index\":"<<index
           <<",\"source_window\":"<<source.source<<",\"model_id\":"<<source.model.model_id<<",\"paths\":"<<source.model.n_paths
           <<",\"channel_us\":"<<measured.channel_us<<",\"common_us\":"<<measured.common_us
           <<",\"phase_total_us\":"<<measured.phase_filter_us<<",\"path_total_us\":"<<measured.path_accum_us<<",\"cfo_total_us\":"<<measured.cfo_check_us
           <<",\"cache_hits\":"<<measured.prefix_cache_hits<<",\"cache_misses\":"<<measured.prefix_cache_misses<<",\"fields\":[";
        double sum=measured.common_us;
        for(size_t f=0;f<3;++f){if(f)log<<',';
            log<<"{\"phase_us\":"<<measured.phase_us[f]<<",\"path_us\":"<<measured.path_us[f]<<",\"cfo_us\":"<<measured.cfo_us[f]<<'}';
            sum+=measured.phase_us[f]+measured.path_us[f]+measured.cfo_us[f];}
        log<<"]}\n";
        if(split&&(measured.common_us<0||std::abs(sum-measured.channel_us)>1e-6))throw std::runtime_error("timing accounting mismatch");
    };
    // Cold means reset the phase mask/key before each call, allocation excluded.
    for(size_t j=0;j<snapshots.size();++j)for(bool split:{false,true}){
        auto& s=split?segmented:original;s.prefix_key_valid=false;s.prefix_phase_mask=0;
        replay(snapshots[j],split,"cold",0,j,true);
    }
    // Start each variant's steady phase from the same empty single-key cache.
    original.prefix_key_valid=segmented.prefix_key_valid=false;original.prefix_phase_mask=segmented.prefix_phase_mask=0;
    for(size_t j=0;j<warmup;++j)for(bool split:{false,true})replay(snapshots[j%snapshots.size()],split,"warmup",0,j,false);
    for(size_t round=0;round<rounds;++round)for(size_t j=0;j<observations;++j){
        std::array<bool,2> order{false,true};if((round+j)%2)std::reverse(order.begin(),order.end());
        for(bool split:order)replay(snapshots[j%snapshots.size()],split,"steady",round+1,j,true);
    }
    std::ofstream summary(outdir/"summary.json");summary<<std::setprecision(15)
       <<"{\"schema\":\"fullpacket-segment-diagnostic/1\",\"sources\":"<<snapshots.size()<<",\"observations_per_variant_per_round\":"<<observations<<",\"rounds\":"<<rounds
       <<",\"warmup_per_variant\":"<<warmup<<",\"verified_outputs_including_cold_and_warmup\":"<<checks<<",\"bit_exact_models\":"<<exact<<",\"max_model_abs_error\":"<<worst<<",\"max_model_relative_l2_error\":"<<relative
       <<",\"waveform_cuts\":[0,73152,94656,133568],\"all_outputs_valid\":true}\n";
    std::cout<<"sources="<<snapshots.size()<<" verified="<<checks<<" worst="<<worst<<" relative="<<relative<<'\n';return 0;
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
