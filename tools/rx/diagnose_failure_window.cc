// Offline, single-window diagnostics. Uses a private instrumented header copy;
// the production header and receiver are not modified.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <string>
#include <complex>
#include <cmath>
namespace window_diag {
struct Local { int call; long long want, radius, pos; float metric, threshold; };
struct Sfd { int call; long long expected, center, pos; float metric, threshold; };
struct Nsfd { float metric, threshold; long long position; };
inline int timing_call=0, sfd_call=0;
inline double force_cfo=0;
inline bool dense_sfd=false, nominal_period=false;
inline std::vector<Local> locals;
inline std::vector<Sfd> sfds;
inline std::vector<Nsfd> nsfds;
inline void local(long long w,long long r,float m,long long p,float t) {
    locals.push_back({timing_call,w,r,p,m,t});
}
inline void sfd(long long e,long long c,float m,long long p,float t) {
    sfds.push_back({sfd_call,e,c,p,m,t});
}
}
#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>
using namespace gr::uwb;
template<class T> std::vector<T> load(const std::string& path) {
    std::ifstream f(path,std::ios::binary|std::ios::ate);
    if(!f) throw std::runtime_error(path);
    size_t bytes=f.tellg();
    std::vector<T> x(bytes/sizeof(T));f.seekg(0);
    f.read(reinterpret_cast<char*>(x.data()),bytes);return x;
}
std::vector<std::complex<int16_t>> read(const std::string& path,int64_t start,size_t n) {
    if(start<0) throw std::runtime_error("negative start");
    std::ifstream f(path,std::ios::binary);f.seekg(start*4);
    std::vector<std::complex<int16_t>> x(n);
    if(!f.read(reinterpret_cast<char*>(x.data()),n*4)) throw std::runtime_error("read");
    return x;
}
int main(int argc,char**argv) try {
    if(argc!=8) throw std::runtime_error("SOURCE WINDOW_START COUNT SEED MODE OUT_PREFIX RESOURCES");
    std::string source=argv[1], mode=argv[5], out=argv[6], res=argv[7];
    int64_t ws=std::stoll(argv[2]), seed=std::stoll(argv[4]);size_t count=std::stoull(argv[3]);
    auto x=read(source,ws,count);
    // Reproduce the same epoch's 0.5 ms training and saturating SC16 subtraction.
    // Exported examples are away from update crossfades; hard-switch tables
    // are identical there. Handle any 20 ms boundary explicitly.
    core::FixedToneCancelCore tone;core::FixedToneCancelConfig cfg;cfg.enable=true;
    cfg.crossfade_samples=0;std::string error;
    if(!tone.prepare(cfg,&error)) throw std::runtime_error(error);
    size_t done=0;
    while(done<count) {
        int64_t epoch=(ws+done)/tone.period_length_samples()*tone.period_length_samples();
        auto training=read(source,epoch,tone.training_length_samples());
        tone.begin_training(epoch);tone.add_training_samples(reinterpret_cast<int16_t*>(training.data()),training.size());
        if(tone.finish_training()!=core::FixedToneTrainingStatus::Ok) throw std::runtime_error("tone");
        size_t n=std::min<size_t>(count-done,epoch+tone.period_length_samples()-ws-done);
        tone.cancel_in_place(reinterpret_cast<int16_t*>(x.data()+done),n,ws+done);done+=n;
    }
    auto taps=load<float>(res+"/resampler_65_48/taps_realtime_eq16.txt");
    auto tmpl=load<std::complex<float>>(res+"/reference_preamble_code10_998p4.cf32");
    core::RationalResampler65_48Core rs(taps);rs.set_fixed_phase_schedule(true);
    std::vector<std::complex<float>> y(rs.expected_output_length(count));
    size_t made=rs.process_sc16(x.data(),count,1.f/32768,y.data(),y.size()).produced;
    while(made<y.size()) {size_t n=rs.flush(y.data()+made,y.size()-made);if(!n)break;made+=n;}
    demod::Qm35825Profile p;p.code_index=10;p.preamble_repetitions=256;p.cir_repetitions=246;
    p.sfd_mode="4z2";p.cir_soft_chip_mode=demod::CirSoftChipMode::Full;
    p.rx_fast_sfd=true;p.rx_fast_cfo_rotation=true;p.rx_seed_refine_tail=true;
    p.rx_softchip_on_demand=true;p.rx_fused_payload_cmf=true;p.rx_fused_phr_cmf=true;
    p.fused_gate_ppm=demod::core::kIntegerGridCeilingPpm;
    p.sync_strategy=demod::SyncStrategy::LockedBounded;p.locked_roi_samples=256;
    p.locked_allow_backtrack=false;p.timing_search_margin=0;p.timing_max_backtrack_symbols=0;
    if(mode=="acquire") {p.sync_strategy=demod::SyncStrategy::AcquireLegacy;p.timing_search_margin=131072;p.timing_max_backtrack_symbols=128;}
    if(mode=="timing020") p.radar_verification_threshold=.20f;
    if(mode=="sfd025") p.sfd_detection_threshold=.25f;
    if(mode=="both") {p.radar_verification_threshold=.20f;p.sfd_detection_threshold=.25f;}
    if(mode.rfind("cfo=",0)==0) window_diag::force_cfo=std::stod(mode.substr(4));
    if(mode=="dense") window_diag::dense_sfd=true;
    if(mode=="nominal") window_diag::nominal_period=true;
    const int64_t rel=std::llround((seed-ws)*65.0/48);
    demod::core::DemodScratch scratch;
    auto r=demod::core::demodulate_one(y.data(),made,p,0,rel,0,tmpl,scratch);
    std::ofstream local(out+".local.csv");local<<"call,want,radius,position,metric,threshold\n";
    for(auto &a:window_diag::locals) local<<a.call<<','<<a.want<<','<<a.radius<<','<<a.pos<<','<<a.metric<<','<<a.threshold<<'\n';
    std::ofstream sfd(out+".sfd.csv");sfd<<"call,expected,center,position,metric,threshold\n";
    for(auto &a:window_diag::sfds) sfd<<a.call<<','<<a.expected<<','<<a.center<<','<<a.pos<<','<<a.metric<<','<<a.threshold<<'\n';
    std::ofstream nsfd(out+".nsfd.csv");nsfd<<"metric,threshold,position\n";
    for(auto &a:window_diag::nsfds) nsfd<<a.metric<<','<<a.threshold<<','<<a.position<<'\n';
    // Independent full-preamble inspection: probe every known symbol, including
    // after the production tracker's first miss. Zero threshold is DIAGNOSTIC
    // ONLY and is not used to declare packet success.
    auto probe=p;probe.sync_strategy=demod::SyncStrategy::LockedBounded;
    probe.locked_inner_symbols=0;probe.locked_roi_samples=8;
    probe.radar_verification_threshold=0;probe.timing_track_radius=8;
    demod::TimingResult timing;demod::core::DemodScratch ps;
    const int64_t probe_seed=rel+std::llround((taps.size()-1)*0.5/48.0);
    demod::core::stage_timing(y.data(),made,probe,tmpl,probe_seed,timing,ps);
    std::ofstream grid(out+".grid.csv");grid<<"symbol,position,metric,corr_re,corr_im\n";
    for(size_t k=0;k<timing.peak_samples.size();++k)
        grid<<k<<','<<timing.peak_samples[k]<<','<<timing.peak_metrics[k]<<','
            <<timing.peak_corr[k].real()<<','<<timing.peak_corr[k].imag()<<'\n';
    const auto &initial=scratch.initial_timing.peak_samples.empty()?r.timing:scratch.initial_timing;
    std::cout<<std::setprecision(12)<<"{\"status\":"<<int(r.status)<<",\"sync\":"<<r.timing.detected_peaks
        <<",\"initial_sync\":"<<initial.detected_peaks<<",\"initial_period\":"<<initial.measured_period
        <<",\"initial_start\":"<<initial.preamble_start_sample
        <<",\"fit\":"<<r.cfo.peaks_used<<",\"cfo_hz\":"<<r.cfo.cfo_hz
        <<",\"sfd\":"<<r.sfd.ok<<",\"phr\":"<<r.phr.ok<<",\"fcs\":"<<r.payload.fcs_pass
        <<",\"sfdmetric\":"<<r.sfd.metric<<",\"sfd_start\":"<<r.sfd.sfd_start_sample
        <<",\"sfd_expected\":"<<r.sfd.expected_start_sample
        <<",\"timing_start\":"<<r.timing.preamble_start_sample
        <<",\"measured_period\":"<<r.timing.measured_period<<",\"hex\":\"";
    for(auto b:r.payload.bytes)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);
    std::cout<<"\"}\n";
    std::ofstream yy(out+".cf32",std::ios::binary);yy.write(reinterpret_cast<char*>(y.data()),made*8);
    return 0;
}catch(std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
