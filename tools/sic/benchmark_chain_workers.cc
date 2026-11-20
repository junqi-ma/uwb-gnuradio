// Offline capacity benchmark of full DSP jobs. No production RX/queue is
// emulated here; use benchmark_sc16_pipeline_steady for those measurements.
#include <gnuradio/uwb/uwb_sic_unified.h>
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_demod_core.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <memory>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <array>

using namespace openuwb;
using cf=std::complex<float>;
using sc16=std::complex<int16_t>;
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point a,Clock::time_point b) {
    return std::chrono::duration<double,std::micro>(b-a).count();
}
template<typename T> std::vector<T> load(const std::string& path) {
    std::ifstream f(path,std::ios::binary|std::ios::ate);
    if(!f) throw std::runtime_error("cannot load "+path);
    const size_t bytes=f.tellg();
    if(bytes%sizeof(T)) throw std::runtime_error("invalid file size");
    std::vector<T> out(bytes/sizeof(T)); f.seekg(0);
    f.read(reinterpret_cast<char*>(out.data()),bytes);
    if(!f) throw std::runtime_error("read failure");
    return out;
}
const std::vector<std::string> detail_names={
    "sic_estimate_us","sic_prefix_rx_us","sic_reconstruct_us","sic_subtract_us","sic_other_us",
    "est_detect_us","est_cfo_us","est_cir_us","est_solve_us","est_fit_us","est_other_us",
    "sic_rx_timing_us","sic_rx_cfo_us","sic_rx_sfd_us","sic_rx_other_us",
    "recon_scale_us","recon_conv_us","recon_phase_us","recon_other_us",
    "sense_sfd_us","sense_refine_us","sense_estimate_us","sense_other_us",
    "comm_timing_us","comm_cfo_us","comm_sfd_us","comm_cir_us","comm_ns_sfd_us","comm_phr_us","comm_payload_us","comm_soft_fallback_us","comm_other_us",
    "comm_cir_estimate_us","comm_cir_fir_us","comm_cir_post_us","comm_cir_other_us",
    "resample_reset_us","resample_process_us","resample_flush_us",
    // Diagnostic CHILDREN of comm_sfd_us / comm_cfo_us (never add to parents).
    "comm_sfd_bootstrap_us","comm_sfd_tail_retrack_us","comm_sfd_cfo_refit_us",
    "comm_sfd_final_us","comm_sfd_anchor_other_us","comm_cfo_fit_us","comm_cfo_rotate_us"
};
struct Record {
    size_t id=0; int worker=0;
    double start=0,end=0,resample=0,sic=0,sensing=0,comm=0,total=0;
    bool committed=false,radar=false,fcs=false,exact=false;
    double cfo_est_hz=0,detected_start=-1;
    std::vector<double> details;
    unsigned soft_fallbacks=0,payload_fallbacks=0,phr_fallbacks=0;
};

// Success-path decomposition using exactly the production radar primitives.
// Warmup compares the complete result and raw/norm taps against radar_cir_one.
radar::RadarCirResult detailed_radar(const cf* data,size_t n,int64_t predicted,
        const radar::RadarCirConfig& cfg,radar::RadarCirCoreScratch& scratch,
        std::array<double,3>& times) {
    radar::RadarCirResult out;out.predicted_sfd_start=predicted;
    radar::RadarSfdResult sfd;auto t0=Clock::now();
    if(!radar::search_sfd(data,n,predicted,cfg.sfd_search_margin,cfg.sfd_threshold,sfd,scratch.sfd))
        throw std::runtime_error("detailed radar SFD failure");
    auto t1=Clock::now();radar::RadarTimingResult timing;
    if(!radar::refine_sync_origin(data,n,sfd.sfd_start_sample,cfg.sync_repetitions,1016,
            cfg.sync_refine_margin,cfg.sync_refine_threshold,scratch.sync_template.data(),
            scratch.sync_template.size(),timing)) throw std::runtime_error("detailed timing failure");
    auto t2=Clock::now();radar::RadarCirEstimate cir;
    const int64_t origin=radar::nominal_preamble_start(predicted,cfg.sync_repetitions,1016);
    if(!radar::estimate_radar_cir(data,n,origin,1016,cfg.cir_pre,cfg.cir_post,cfg.cir_skip_initial,
            cfg.cir_repetitions,cfg.sync_repetitions,cir,scratch.cir)) throw std::runtime_error("detailed CIR failure");
    auto t3=Clock::now();times={elapsed(t0,t1),elapsed(t1,t2),elapsed(t2,t3)};
    out.status=radar::RadarCirStatus::Ok;out.sfd_start_sample=sfd.sfd_start_sample;
    out.sfd_metric=sfd.metric;out.preamble_start_sample=timing.preamble_start_sample;out.sync_metric=timing.metric;
    out.cir_origin_sample=origin;out.tap_count=cir.tap_count;out.peak_tap=cir.peak_tap;
    out.peak_abs=cir.peak_abs;out.raw_l2_norm=cir.raw_l2_norm;out.valid_repetitions=cir.valid_repetitions;
    return out;
}
struct Worker {
    core::RationalResampler65_48Core resampler;
    sic::SicUnifiedConfig cfg;
    sic::SicUnifiedTemplates tmpl;
    sic::SicUnifiedScratch sic_scratch;
    radar::RadarCirConfig radar_cfg;
    radar::RadarCirCoreScratch radar_scratch;
    demod::Qm35825Profile profile=demod::Qm35825Profile::Default();
    demod::core::DemodScratch demod_scratch;
    std::vector<cf> target_sync,interferer_sync,preamble,work,out,ihat;
    size_t expected;
    std::string task;
    Worker(const std::vector<float>& taps,size_t native_count,const std::string& directory,const std::string& requested="all")
        :resampler(taps),task(requested) {
        expected=resampler.expected_output_length(native_count);
        work.resize(expected);out.resize(expected);ihat.resize(expected);
        resampler.set_num_workers(1);resampler.set_fixed_phase_schedule(true);
        target_sync=load<cf>(directory+"/sync_9.bin");
        if(target_sync.size()!=1016) throw std::runtime_error("invalid sync");
        size_t target_reps=64,interference_reps=64;
        std::ifstream geometry(directory+"/geometry.txt");
        if(geometry && !(geometry>>target_reps>>interference_reps)) throw std::runtime_error("invalid geometry");
        // clean_comm measures the receive chain of one interference-free packet:
        // no SIC and no sensing job is prepared or executed.
        if(task!="clean_comm") {
            interferer_sync=load<cf>(directory+"/sync_10.bin");
            if(interferer_sync.size()!=1016) throw std::runtime_error("invalid sync");
            for(size_t r=0;r<interference_reps;++r) preamble.insert(preamble.end(),interferer_sync.begin(),interferer_sync.end());
            tmpl.sync=interferer_sync.data();tmpl.sync_len=1016;
            tmpl.preamble=preamble.data();tmpl.preamble_len=preamble.size();
            cfg.policy=sic::SicPolicy::PreambleOnly;cfg.backend=sic::SicEstimatorBackend::Ipatov;
            cfg.comm_code=10;cfg.samples_per_symbol=1016;cfg.sync_reps=interference_reps;cfg.sample_rate=998.4e6;
            // Same public geometry as the production pipeline: pre-guard 400 native
            // samples -> nominal output origin 542, coarse ROI +/-64.
            cfg.roi_lo=478;cfg.roi_hi=606;cfg.sub_roi_lo=0;cfg.sub_roi_hi=expected;
            cfg.max_window_samples=expected+512;cfg.max_packet_samples=expected+512;
            std::string error;
            if(!sic::prepare_sic_unified(cfg,tmpl,sic_scratch,&error)) throw std::runtime_error(error);
            radar_cfg.sync_repetitions=target_reps;
            radar_cfg.cir_repetitions=target_reps-radar_cfg.cir_skip_initial;
            if(!radar::prepare_radar_cir_core(radar_cfg,target_sync.data(),1016,16,100,radar_scratch))
                throw std::runtime_error("radar preparation failed");
        }
        profile.code_index=9;profile.timing_coarse_stride=1;
        profile.preamble_repetitions=target_reps;
        profile.cir_repetitions=target_reps-profile.cir_skip_initial_repetitions;
        profile.rx_fast_sfd=true;profile.rx_fast_cfo_rotation=true;
        profile.rx_seed_refine_tail=true;profile.rx_softchip_on_demand=true;
        profile.rx_fused_payload_cmf=true;profile.rx_fused_phr_cmf=true;
        demod_scratch.reserve(expected+512);
    }
    Record run(const sc16* input,size_t count,const std::vector<uint8_t>& psdu,bool check_reference=false) {
        Record r;
        auto t0=Clock::now();resampler.reset();
        auto tr=Clock::now();
        auto converted=resampler.process_sc16(input,count,1.f/32768,work.data(),expected);
        auto tp=Clock::now();
        if(converted.consumed!=count) throw std::runtime_error("input consumption mismatch");
        size_t produced=converted.produced;
        while(produced<expected) {
            size_t n=resampler.flush(work.data()+produced,expected-produced);
            if(!n) break;
            produced+=n;
        }
        if(produced!=expected) throw std::runtime_error("output length mismatch");
        auto t1=Clock::now();
        sic::SicUnifiedResult result;result.out=out.data();result.i_hat=ihat.data();
        if(task!="clean_comm") {
            if(!sic::sic_process_unified(work.data(),expected,tmpl,cfg,sic_scratch,result))
                throw std::runtime_error("SIC API failure");
        }
        auto t2=Clock::now();
        const cf* data=task=="clean_comm"?work.data():(result.committed?out.data():work.data());
        radar::RadarCirResult radar_result;
        std::array<double,3> sense_times{};
        if(task=="sensing_detail") radar_result=detailed_radar(data,expected,542+radar_cfg.sync_repetitions*1016,radar_cfg,radar_scratch,sense_times);
        else if(task!="comm" && task!="clean_comm") radar::radar_cir_one(data,expected,542+radar_cfg.sync_repetitions*1016,radar_cfg,radar_scratch,radar_result);
        auto t3=Clock::now();
        demod::DemodResult decoded;
        if(task=="all"||task=="comm"||task=="clean_comm") decoded=demod::core::demodulate_one(data,expected,profile,0,542,0,target_sync,demod_scratch);
        auto t4=Clock::now();
        r.start=elapsed(Clock::time_point{},t0);r.end=elapsed(Clock::time_point{},t4);
        r.resample=elapsed(t0,t1);r.sic=elapsed(t1,t2);r.sensing=elapsed(t2,t3);
        r.comm=(task=="all"||task=="comm"||task=="clean_comm")?elapsed(t3,t4):0;
        if(task=="comm"||task=="clean_comm") r.sensing=0;
        r.total=elapsed(t0,t4);
        r.committed=task=="clean_comm"?true:result.committed;r.radar=radar_result.status==radar::RadarCirStatus::Ok;
        r.fcs=decoded.payload.fcs_pass;r.exact=decoded.payload.bytes==psdu;
        r.cfo_est_hz=decoded.cfo.cfo_hz;r.detected_start=double(decoded.sfd.sfd_start_sample);
        const auto& t=result.timings;
        const double est_children=t.est_detect_us+t.est_cfo_us+t.est_cir_us+t.est_solve_us+t.est_fit_check_us;
        const double rx_children=t.rx_timing_us+t.rx_cfo_us+t.rx_sfd_us;
        const double recon_children=t.recon_scale_us+t.recon_conv_us+t.recon_cfo_rot_us;
        const double comm_children=decoded.stage_timing_us+decoded.stage_cfo_us+decoded.stage_sfd_us+decoded.stage_cir_us+
            decoded.stage_ns_sfd_us+decoded.stage_phr_us+decoded.stage_payload_us+decoded.stage_softchip_fallback_us;
        r.details={t.estimate_us,t.comm_rx_us,t.reconstruct_us,t.subtract_us,
            r.sic-t.estimate_us-t.comm_rx_us-t.reconstruct_us-t.subtract_us,
            t.est_detect_us,t.est_cfo_us,t.est_cir_us,t.est_solve_us,t.est_fit_check_us,t.estimate_us-est_children,
            t.rx_timing_us,t.rx_cfo_us,t.rx_sfd_us,t.comm_rx_us-rx_children,
            t.recon_scale_us,t.recon_conv_us,t.recon_cfo_rot_us,t.reconstruct_us-recon_children,
            sense_times[0],sense_times[1],sense_times[2],r.sensing-sense_times[0]-sense_times[1]-sense_times[2],
            double(decoded.stage_timing_us),double(decoded.stage_cfo_us),double(decoded.stage_sfd_us),double(decoded.stage_cir_us),
            double(decoded.stage_ns_sfd_us),double(decoded.stage_phr_us),double(decoded.stage_payload_us),double(decoded.stage_softchip_fallback_us),r.comm-comm_children,
            double(decoded.cir.cir_estimate_us),double(decoded.cir.soft_fir_us),double(decoded.cir.postprocess_us),
            double(decoded.stage_cir_us)-decoded.cir.cir_estimate_us-decoded.cir.soft_fir_us-decoded.cir.postprocess_us,
            elapsed(t0,tr),elapsed(tr,tp),elapsed(tp,t1),
            double(decoded.stage_sfd_bootstrap_us),double(decoded.stage_sfd_tail_retrack_us),
            double(decoded.stage_sfd_cfo_refit_us),double(decoded.stage_sfd_final_us),
            double(decoded.stage_sfd_anchor_other_us),double(decoded.stage_cfo_fit_us),
            double(decoded.stage_cfo_rotate_us)};
        r.soft_fallbacks=decoded.stage_softchip_fallbacks;r.payload_fallbacks=decoded.stage_fused_fallbacks;r.phr_fallbacks=decoded.stage_phr_fused_fallbacks;
        if(task=="sensing_detail" && check_reference) {
            const auto raw=radar_scratch.cir.raw_taps,norm=radar_scratch.cir.norm_taps;
            radar::RadarCirResult ref;
            if(!radar::radar_cir_one(data,expected,542+radar_cfg.sync_repetitions*1016,radar_cfg,radar_scratch,ref) ||
                ref.status!=radar_result.status || ref.sfd_start_sample!=radar_result.sfd_start_sample ||
                ref.preamble_start_sample!=radar_result.preamble_start_sample || ref.peak_tap!=radar_result.peak_tap ||
                ref.peak_abs!=radar_result.peak_abs || ref.valid_repetitions!=radar_result.valid_repetitions ||
                ref.sfd_metric!=radar_result.sfd_metric || ref.sync_metric!=radar_result.sync_metric ||
                ref.cir_origin_sample!=radar_result.cir_origin_sample || ref.tap_count!=radar_result.tap_count ||
                raw!=radar_scratch.cir.raw_taps || norm!=radar_scratch.cir.norm_taps)
                throw std::runtime_error("radar decomposition differs from production result");
        }
        return r;
    }
};
int main(int argc,char** argv) {
    try {
        if(argc!=8 && argc!=9) throw std::runtime_error("usage: binary directory taps native_count cores_csv warmup_per_worker measure_jobs source_pool_size [all|sensing|sensing_detail|comm|clean_comm]");
        const std::string task=argc==9?argv[8]:"all";
        if(task!="all" && task!="sensing" && task!="sensing_detail" && task!="comm" && task!="clean_comm") throw std::runtime_error("unknown task");
        const std::string directory=argv[1];
        const auto taps=load<float>(argv[2]);const size_t count=std::stoull(argv[3]);
        std::vector<int> cores;std::stringstream stream(argv[4]);std::string token;
        while(std::getline(stream,token,',')) cores.push_back(std::stoi(token));
        const size_t warmup=std::stoull(argv[5]),jobs=std::stoull(argv[6]);
        // argv[7] identifies the source pool size; keep it explicit in commands.
        const size_t pool_size=std::stoull(argv[7]);
        auto input=load<sc16>(directory+"/mixture.sc16");
        if(cores.empty()||!count||input.size()!=pool_size*count||!jobs) throw std::runtime_error("invalid workload size");
        std::vector<std::vector<uint8_t>> psdus;
        for(size_t i=0;i<pool_size;++i) psdus.push_back(load<uint8_t>(directory+"/psdu_9_"+std::to_string(i)+".bin"));
        std::vector<Record> records(jobs);std::atomic<size_t> next{0};
        std::mutex mutex;std::condition_variable cv;size_t ready=0;bool start=false;
        std::string failure;
        auto fail=[&](const std::string& error) {std::lock_guard<std::mutex> guard(mutex);if(failure.empty())failure=error;};
        auto bad=[&](const Record& r) {return !r.committed || (task!="comm" && task!="clean_comm" && !r.radar) || ((task=="all"||task=="comm"||task=="clean_comm") && (!r.fcs||!r.exact));};
        std::vector<std::thread> threads;
        for(size_t id=0;id<cores.size();++id) threads.emplace_back([&,id] {
            std::unique_ptr<Worker> worker;
            try {
                cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(cores[id],&mask);
                if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask)) throw std::runtime_error("affinity failed");
                worker=std::make_unique<Worker>(taps,count,directory,task);
                for(size_t k=0;k<warmup;++k) {
                    size_t item=(k+id)%pool_size;
                    if(bad(worker->run(input.data()+item*count,count,psdus[item],k<pool_size))) throw std::runtime_error("warmup functional failure");
                }
            } catch(const std::exception& e) {fail(e.what());}
            {std::unique_lock<std::mutex> guard(mutex);++ready;cv.notify_all();cv.wait(guard,[&]{return start;});}
            if(!worker) return;
            try {
                for(;;) {
                    size_t job=next.fetch_add(1);if(job>=jobs) break;
                    const size_t item=job%pool_size;
                    auto r=worker->run(input.data()+item*count,count,psdus[item]);
                    r.id=job;r.worker=id;records[job]=r;
                    if(bad(r)) throw std::runtime_error("measured functional failure");
                }
            } catch(const std::exception& e) {fail(e.what());}
        });
        {std::unique_lock<std::mutex> guard(mutex);cv.wait(guard,[&]{return ready==cores.size();});start=true;cv.notify_all();}
        for(auto& thread:threads) thread.join();
        if(!failure.empty()) throw std::runtime_error(failure);
        std::cout<<std::setprecision(16)<<"job,worker,start_us,end_us,resample_us,sic_us,sensing_us,comm_us,total_us,committed,radar_ok,fcs,psdu_exact,cfo_est_hz,detected_start";
        for(const auto& name:detail_names) std::cout<<','<<name;
        std::cout<<",soft_fallbacks,payload_fallbacks,phr_fallbacks\n";
        for(const auto& r:records) {
            std::cout<<r.id<<','<<r.worker<<','<<r.start<<','<<r.end<<','<<r.resample<<','<<r.sic<<','<<r.sensing<<','<<r.comm<<','<<r.total<<','<<r.committed<<','<<r.radar<<','<<r.fcs<<','<<r.exact<<','<<r.cfo_est_hz<<','<<r.detected_start;
            for(auto value:r.details) std::cout<<','<<value;
            std::cout<<','<<r.soft_fallbacks<<','<<r.payload_fallbacks<<','<<r.phr_fallbacks<<'\n';
        }
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
