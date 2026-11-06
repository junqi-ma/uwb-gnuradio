// Offline paired SIC and bounded, real-clock arrival replay. No GNU Radio block.
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>
#include "fullpacket_rx_pipeline.h"
#include <gnuradio/uwb/uwb_radar_cir_core.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <ctime>
#include <memory>
#include <array>
using namespace gr::uwb;
using C = std::complex<float>;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;
double cpu_us() { timespec t{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1e6+t.tv_nsec/1e3; }
double us(Clock::time_point a, Clock::time_point b) { return std::chrono::duration<double,std::micro>(b-a).count(); }
std::vector<C> read_iq(const fs::path& p) {
 std::ifstream f(p,std::ios::binary|std::ios::ate); if(!f) throw std::runtime_error("read "+p.string());
 auto size=f.tellg(); if(size<=0 || size%sizeof(C)) throw std::runtime_error("invalid IQ");
 std::vector<C> a(size/sizeof(C)); f.seekg(0); f.read(reinterpret_cast<char*>(a.data()),size); return a;
}
void write_iq(const fs::path& p,const std::vector<C>& a) { std::ofstream f(p,std::ios::binary); f.write(reinterpret_cast<const char*>(a.data()),a.size()*sizeof(C)); if(!f) throw std::runtime_error("write IQ"); }
std::string hex(const std::vector<uint8_t>& b) { std::ostringstream o; o<<std::hex<<std::setfill('0'); for(auto x:b)o<<std::setw(2)<<int(x);return o.str(); }
std::string quote(const std::string& s) { std::ostringstream o;o<<'"';for(char c:s){if(c=='"'||c=='\\')o<<'\\';if(c=='\n')o<<"\\n";else o<<c;}o<<'"';return o.str(); }
struct Options {fs::path input,out; size_t repeat=0,warmup=0; double rate=0,seconds=10;std::string mode="both",reconstruction="full";bool dump=false;
 double lambda=1e-6,peak_floor=0.15;int64_t lag_lo=-8;size_t tap_count=64,peak_min_spacing=5,active_radius=1;};
struct Row {
 size_t id=0,source=0,n=0;std::string mode,reason="not_run",reconstruction="full";
 sic::FullPacketResult recon_stats; bool committed=false,valid=false,rejected=false;
 sic::DiscreteCirModel discrete_model; sic::DiscreteCirResult discrete_result;
 sic::DiscreteCirTimings discrete_timings;
 uint64_t model_id=0; int64_t observed=0,required=0,lo=0,hi=0,origin=-1;
 bool comm_fcs=false,sense_fcs=false;std::string comm_hex,sense_hex;int sense_status=-1,comm_status=-1,cir_status=-1;
 std::array<double,7> comm_stages{};
 double thread_cpu=0,estimate=0,comm=0,validate=0,tx=0,channel=0,subtract=0,copy=0,sic=0,cir=0,sense=0,service=0;
 double submit=0,start=0,finish=0,result=0,first=0,ready=0,scheduled=0,lateness=0;
};
void emit(std::ostream& o,const Row&r){
 o<<std::setprecision(12)<<"{\"window_id\":"<<r.id<<",\"source_window\":"<<r.source<<",\"mode\":"<<quote(r.mode)
 <<",\"reconstruction\":"<<quote(r.reconstruction)
 <<",\"rx_pipeline\":"<<quote(experiment::FastRxPipeline::avx2_enabled()?"fast":"portable")
 <<",\"rx_avx2\":"<<(experiment::FastRxPipeline::avx2_enabled()?"true":"false")
 <<",\"n\":"<<r.n<<",\"committed\":"<<(r.committed?"true":"false")<<",\"reason\":"<<quote(r.reason)
 <<",\"model_valid\":"<<(r.valid?"true":"false")<<",\"model_id\":"<<r.model_id<<",\"origin\":"<<r.origin
 <<",\"observed_end\":"<<r.observed<<",\"support\":["<<r.lo<<","<<r.hi<<"],\"rejected\":"<<(r.rejected?"true":"false")
 <<",\"comm_fcs\":"<<(r.mode=="preamble"?"null":r.comm_fcs?"true":"false")<<",\"comm_status\":"<<r.comm_status
 <<",\"comm_psdu_hex\":"<<quote(r.comm_hex)<<",\"sensing_fcs\":"<<(r.sense_fcs?"true":"false")
 <<",\"sensing_psdu_hex\":"<<quote(r.sense_hex)<<",\"sensing_status\":"<<r.sense_status<<",\"cir_status\":"<<r.cir_status
 <<",\"reconstruction_stats\":{";
 if(r.mode=="full" && !r.rejected) {
  const auto& f=r.recon_stats;
  o<<"\"phase_filter_us\":"<<f.phase_filter_us<<",\"path_accum_us\":"<<f.path_accum_us
   <<",\"cfo_check_us\":"<<f.cfo_check_us<<",\"prefix_cache_used\":"<<(f.prefix_cache_used?"true":"false")
   <<",\"key_changed\":"<<(f.prefix_key_changed?"true":"false")<<",\"cached_samples\":"<<f.prefix_cached_samples
   <<",\"phase_calls\":"<<f.phase_calls<<",\"nonzero_phase_calls\":"<<f.nonzero_phase_calls
   <<",\"cache_hits\":"<<f.prefix_cache_hits<<",\"cache_misses\":"<<f.prefix_cache_misses
   <<",\"fir_output_samples\":"<<f.fir_output_samples<<",\"reference_fir_output_samples\":"<<f.reference_fir_output_samples;
 }
 o<<"}";
 if(r.reconstruction=="discrete_cir") {
   const auto&d=r.discrete_model;const auto&f=r.discrete_result;const auto&t=r.discrete_timings;
   o<<",\"discrete_cir\":{\"model_id\":"<<d.model_id<<",\"q\":"<<d.q
    <<",\"cfo_hz\":"<<d.cfo_hz<<",\"lag_lo\":"<<d.lag_lo<<",\"tap_count\":"<<d.tap_count
    <<",\"active_count\":"<<d.active_count
    <<",\"condition\":"<<d.gain_condition<<",\"effective_rank\":"<<d.effective_rank
    <<",\"fit_residual\":"<<d.fit_residual<<",\"waveform_scale\":"<<f.waveform_scale
    <<",\"support\":["<<r.lo<<","<<r.hi<<"]"
    <<",\"sub\":["<<(f.committed?f.sub_lo:0)<<","<<(f.committed?f.sub_hi:0)<<"]"
    <<",\"times_us\":{\"detect\":"<<t.detect_us<<",\"cfo\":"<<t.cfo_us
    <<",\"support\":"<<t.support_us<<",\"projection\":"<<t.projection_us
    <<",\"solve\":"<<t.solve_us<<",\"fit_check\":"<<t.fit_check_us
    <<",\"estimate_total\":"<<t.estimate_total_us<<",\"validate\":"<<f.validate_us
    <<",\"tx\":"<<f.tx_us<<",\"scale\":"<<f.scale_us<<",\"convolution\":"<<f.convolution_us
    <<",\"cfo_rot\":"<<f.cfo_us<<",\"subtract\":"<<f.subtract_us<<"}}";
 }
 o<<",\"comm_stage_us\":{";
 if(r.mode=="full" && !r.rejected) {
  const char* names[]={"timing","cfo","sfd","cir","ns_sfd","phr","payload"};
  for(size_t k=0;k<7;++k){if(k)o<<',';o<<quote(names[k])<<':'<<r.comm_stages[k];}
 }
 o<<"},\"times_us\":{\"thread_cpu\":"<<r.thread_cpu<<",\"estimate\":"<<r.estimate<<",\"comm_demod\":"<<r.comm<<",\"validate\":"<<r.validate
 <<",\"tx\":"<<r.tx<<",\"channel\":"<<r.channel<<",\"subtract\":"<<r.subtract<<",\"copy\":"<<r.copy
 <<",\"sic_total\":"<<r.sic<<",\"cir\":"<<r.cir<<",\"sensing_demod\":"<<r.sense<<",\"service\":"<<r.service
 <<"},\"events\":{\"required_sample\":"<<r.required<<",\"submit_us\":"<<r.submit<<",\"start_us\":"<<r.start
 <<",\"finish_us\":"<<r.finish<<",\"result_us\":"<<r.result<<",\"first_clean_us\":"<<r.first
 <<",\"sample_ready_us\":"<<r.ready<<",\"scheduled_us\":"<<r.scheduled<<",\"submit_late_us\":"<<r.lateness<<"}}\n";
}
struct Engine {
 boost::property_tree::ptree pub;
 sic::SicConfig cfg; sic::SicScratch scratch; sic::SicTemplates templ;
 sic::FullPacketScratch full; mod::HrpModConfig modcfg;
 sic::DiscreteCirConfig dcfg; sic::DiscreteCirScratch dscratch; sic::DiscreteCirTemplates dtempl;
 std::vector<C> sync,pre,ssync,out,ihat;
 std::unique_ptr<experiment::FastRxPipeline> comm_rx,sensing_rx;
 radar::RadarCirConfig rc;radar::RadarCirCoreScratch rs;
 int64_t sense_seed;size_t n;double fsamp=998400000.;bool cached_prefix=false;
 bool discrete=false,old_ready=false;
 Engine(const fs::path& p,bool use_prefix_cache=false,bool use_discrete=false,bool need_old=true,
        double dlambda=1e-6,double dpeak_floor=0.15,int64_t dlag_lo=-8,size_t dtap_count=64,
        size_t dpeak_min_spacing=5,size_t dactive_radius=1) {
  boost::property_tree::read_json((p/"public.json").string(),pub);
  if(pub.get<int>("comm_code",10)!=10 || pub.get<int>("sensing_code",9)!=9 || pub.get<int>("sync_reps",64)!=64 || pub.get<std::string>("sfd_mode","4z2")!="4z2" || pub.get<std::string>("pulse_shape","legacy")!="legacy")throw std::runtime_error("unsupported public PHY");
  n=pub.get<size_t>("n");fsamp=pub.get<double>("sample_rate");
  if(n>2000000 || n<9000 || fsamp!=998400000.)throw std::runtime_error("unsupported context/profile");
  sync=read_iq(p/"comm_sync.cf32");pre=read_iq(p/"comm_preamble.cf32");ssync=read_iq(p/"sensing_sync.cf32");
  const int64_t observed=pub.get<int64_t>("observed_end");
  const int64_t roi_lo=pub.get<int64_t>("roi_lo"),roi_hi=pub.get<int64_t>("roi_hi");
  if(observed<=0 || observed>int64_t(n))throw std::runtime_error("invalid observation limit");
  discrete=use_discrete;
  cfg.observed_end_sample=observed;cfg.roi_lo=roi_lo;cfg.roi_hi=roi_hi;
  cfg.sub_roi_lo=pub.get<int64_t>("consumer_lo",0);cfg.sub_roi_hi=pub.get<int64_t>("consumer_hi",n);
  if(need_old) {
   cfg.mode=sic::SicMode::Estimate;
   cfg.detect_threshold=.40;cfg.validate_threshold=.25;
   cfg.online_validation_reps=0;cfg.k_paths=4;cfg.coarse_stride=1;
   std::string err;if(!sic::prepare_sic_core(cfg,sync.data(),sync.size(),pre.data(),pre.size(),scratch,&err))throw std::runtime_error(err);
   templ.sync=sync.data();templ.sync_len=sync.size();templ.preamble=pre.data();templ.preamble_len=pre.size();
   full.reserve(n);cached_prefix=use_prefix_cache;
   if(cached_prefix && !full.prepare_prefix_cache())throw std::runtime_error("prefix cache preparation failed");
   old_ready=true;
  }
  out.resize(n);ihat.resize(n);
  if(discrete) {
   dcfg.roi_lo=roi_lo;dcfg.roi_hi=roi_hi;
   dcfg.coarse_stride=1;dcfg.detect_threshold=.40;
   dcfg.lag_lo=dlag_lo;dcfg.tap_count=dtap_count;
   dcfg.peak_floor=dpeak_floor;dcfg.peak_min_spacing=dpeak_min_spacing;
   dcfg.active_radius=dactive_radius;dcfg.lambda=dlambda;
   dcfg.fit_residual_max=.8;dcfg.gain_condition_max=1e8;
   dcfg.max_window_samples=n;
   dscratch.reserve(n);
   dtempl.sync=sync.data();dtempl.sync_len=sync.size();
   dtempl.train=pre.data();dtempl.train_len=pre.size();
   std::string err;if(!sic::prepare_discrete_cir(dcfg,dtempl,dscratch,&err))throw std::runtime_error(err);
  }
  comm_rx=std::make_unique<experiment::FastRxPipeline>(n,10);
  sensing_rx=std::make_unique<experiment::FastRxPipeline>(n,9);
  sense_seed=pub.get<int64_t>("sensing_seed");
  modcfg.code_index=10;modcfg.sync_repetitions=64;modcfg.sfd_mode="4z2";modcfg.peak_amplitude=.8f;
  rc.use_predicted_timing=true;
  if(!radar::prepare_radar_cir_core(rc,ssync.data(),ssync.size(),rc.cir_pre,rc.cir_post,rs))throw std::runtime_error("radar prepare");
 }
 Row process(const std::vector<C>& y,size_t id,size_t source,const std::string& mode,
             Clock::time_point epoch,Clock::time_point arrival,bool realtime,double submit,double scheduled) {
  if(y.size()!=n)throw std::runtime_error("input length mismatch");
  const double cpu_begin=cpu_us();
  Row r;r.reconstruction=discrete?"discrete_cir":(cached_prefix?"payload_cached":"full");r.id=id;r.source=source;r.mode=mode;r.n=n;r.observed=cfg.observed_end_sample;r.submit=submit;r.scheduled=scheduled;
  r.lateness=std::max(0.,submit-scheduled);
  r.required=(mode=="preamble"?cfg.observed_end_sample:int64_t(n));
  auto wait_sample=[&](size_t k){if(realtime)std::this_thread::sleep_until(arrival+std::chrono::nanoseconds(int64_t(k*1e9/fsamp)));};
  r.start=us(epoch,Clock::now());double service=0;
  auto t=Clock::now();
  if(mode=="raw" || mode=="clean") {
   wait_sample(n);t=Clock::now();std::copy(y.begin(),y.end(),out.begin());r.copy=us(t,Clock::now());
   r.reason="reference";r.sic=r.copy;r.first=us(epoch,Clock::now());
  } else if(discrete) {
   if(mode!="full")throw std::runtime_error("discrete_cir supports full mode only");
   wait_sample(cfg.observed_end_sample);
   t=Clock::now();sic::DiscreteCirModel dm;sic::DiscreteCirTimings dt;
   sic::estimate_discrete_cir(y.data(),n,cfg.observed_end_sample,dtempl,dcfg,dscratch,dm,&dt);
   r.estimate=us(t,Clock::now());r.valid=dm.valid;r.model_id=dm.model_id;r.origin=dm.q;
   r.discrete_model=dm;r.discrete_timings=dt;
   if(!dm.valid) {
    t=Clock::now();std::copy(y.begin(),y.end(),out.begin());std::fill(ihat.begin(),ihat.end(),C(0));
    r.copy=us(t,Clock::now());r.committed=false;r.reason=dm.reason?dm.reason:dm.status;
    r.lo=0;r.hi=0;r.first=us(epoch,Clock::now());
   } else {
    wait_sample(n);t=Clock::now();demod::DemodResult decoded;
    decoded=comm_rx->run(y.data(),n,id,dm.q,sync);
    r.comm=us(t,Clock::now());
    r.comm_stages={double(decoded.stage_timing_us),double(decoded.stage_cfo_us),double(decoded.stage_sfd_us),double(decoded.stage_cir_us),double(decoded.stage_ns_sfd_us),double(decoded.stage_phr_us),double(decoded.stage_payload_us)};
    r.comm_status=int(decoded.status);r.comm_fcs=decoded.payload.fcs_pass;r.comm_hex=hex(decoded.payload.bytes);
    t=Clock::now();auto f=sic::fullpacket_cancel_discrete(y.data(),n,decoded.phr.phr_bits,decoded.payload.bytes,r.comm_fcs,modcfg,dm,sync.data(),sync.size(),cfg.sub_roi_lo,cfg.sub_roi_hi,dscratch,out.data(),ihat.data());
    r.discrete_result=f;double full_wall=us(t,Clock::now());r.validate=f.validate_us;r.tx=f.tx_us;r.channel=f.channel_us;r.subtract=f.subtract_us;
    r.copy=std::max(0.,full_wall-r.validate-r.tx-r.channel-r.subtract);
    r.committed=f.committed;r.reason=f.reason;r.lo=f.support_lo;r.hi=f.support_hi;
    if(!f.committed){r.lo=0;r.hi=0;}
    r.first=us(epoch,Clock::now());
   }
   r.sic=r.estimate+r.comm+r.validate+r.tx+r.channel+r.subtract+r.copy;
  } else {
  wait_sample(cfg.observed_end_sample);
  t=Clock::now();sic::SicModel model;sic::SicTimings st;
  sic::estimate_model(y.data(),n,cfg.observed_end_sample,templ,cfg,scratch,model,&st);
  r.estimate=us(t,Clock::now());r.valid=model.valid;r.model_id=model.model_id;r.origin=model.origin;
  if(mode=="preamble") {
   // First output chunk contains only available samples. Future chunks are applied after arrival.
   t=Clock::now();size_t prefix=cfg.observed_end_sample;
   sic::SicTimings back;
   sic::apply_model(model,y.data(),prefix,0,templ,cfg,scratch,out.data(),ihat.data(),&back);
   double back_wall=us(t,Clock::now());r.channel+=back.reconstruct_us;
   r.subtract+=std::max(0.,back_wall-double(back.reconstruct_us));r.first=us(epoch,Clock::now());
   wait_sample(n);
   sic::SicTimings forward;t=Clock::now();sic::apply_model(model,y.data()+prefix,n-prefix,prefix,templ,cfg,scratch,out.data()+prefix,ihat.data()+prefix,&forward);
   double forward_wall=us(t,Clock::now());r.channel+=forward.reconstruct_us;
   r.subtract+=std::max(0.,forward_wall-double(forward.reconstruct_us));r.committed=model.valid;r.reason=model.reason?model.reason:model.status;
   r.lo=model.valid?model.sub_lo:0;r.hi=model.valid?model.sub_hi:0;
  } else {
   wait_sample(n);t=Clock::now();demod::DemodResult decoded;
   if(model.valid)decoded=comm_rx->run(y.data(),n,id,model.origin,sync);
   else decoded.status=demod::DemodStatus::InvalidInput;
   r.comm=us(t,Clock::now());
   r.comm_stages={double(decoded.stage_timing_us),double(decoded.stage_cfo_us),double(decoded.stage_sfd_us),double(decoded.stage_cir_us),double(decoded.stage_ns_sfd_us),double(decoded.stage_phr_us),double(decoded.stage_payload_us)};
   r.comm_status=int(decoded.status);r.comm_fcs=model.valid&&decoded.payload.fcs_pass;r.comm_hex=hex(decoded.payload.bytes);
   t=Clock::now();auto f=sic::fullpacket_cancel(y.data(),n,decoded.phr.phr_bits,decoded.payload.bytes,r.comm_fcs,modcfg,model,sync.data(),sync.size(),cfg.sub_roi_lo,cfg.sub_roi_hi,full,out.data(),ihat.data());
   r.recon_stats=f;double full_wall=us(t,Clock::now());r.validate=f.validate_us;r.tx=f.tx_us;r.channel=f.channel_us;r.subtract=f.subtract_us;
   r.copy=std::max(0.,full_wall-r.validate-r.tx-r.channel-r.subtract);
   r.committed=f.committed;r.reason=f.reason;r.lo=f.sub_lo;r.hi=f.sub_hi;r.first=us(epoch,Clock::now());
  }
  r.sic=r.estimate+r.comm+r.validate+r.tx+r.channel+r.subtract+r.copy;
  }
  r.finish=us(epoch,Clock::now());r.ready=us(epoch,arrival)+(realtime?r.required*1e6/fsamp:0);
  t=Clock::now();radar::RadarCirResult cr;radar::radar_cir_one(out.data(),n,sense_seed+64*1016,rc,rs,cr);r.cir=us(t,Clock::now());r.cir_status=int(cr.status);
  t=Clock::now();auto sd=sensing_rx->run(out.data(),n,id,sense_seed,ssync);r.sense=us(t,Clock::now());r.sense_status=int(sd.status);r.sense_fcs=sd.payload.fcs_pass;r.sense_hex=hex(sd.payload.bytes);
  service=r.sic+r.cir+r.sense;r.service=service;r.result=us(epoch,Clock::now());r.thread_cpu=cpu_us()-cpu_begin;return r;
 }
};
int main(int argc,char**argv)try {
 Options a;for(int k=1;k<argc;++k){std::string v=argv[k];auto next=[&](){if(++k>=argc)throw std::runtime_error("missing argument");return std::string(argv[k]);};
 if(v=="--case")a.input=next();else if(v=="--out")a.out=next();else if(v=="--repeat")a.repeat=std::stoull(next());else if(v=="--warmup")a.warmup=std::stoull(next());else if(v=="--mode")a.mode=next();else if(v=="--reconstruction")a.reconstruction=next();else if(v=="--paced-rate")a.rate=std::stod(next());else if(v=="--paced-seconds")a.seconds=std::stod(next());else if(v=="--lambda")a.lambda=std::stod(next());else if(v=="--peak-floor")a.peak_floor=std::stod(next());else if(v=="--lag-lo")a.lag_lo=std::stoll(next());else if(v=="--tap-count")a.tap_count=std::stoull(next());else if(v=="--peak-min-spacing")a.peak_min_spacing=std::stoull(next());else if(v=="--active-radius")a.active_radius=std::stoull(next());else if(v=="--dump-iq")a.dump=true;else throw std::runtime_error("unknown argument "+v);}
 if(a.reconstruction=="path_fractional")a.reconstruction="full";
 if(!(a.reconstruction=="full"||a.reconstruction=="payload_cached"||a.reconstruction=="discrete_cir"))throw std::runtime_error("invalid reconstruction mode");
 if(a.reconstruction=="discrete_cir" && (a.mode!="full"||a.rate>0))throw std::runtime_error("discrete_cir supports unpaced full mode only");
 if(a.input.empty()||a.out.empty()||!(a.mode=="both"||a.mode=="full"||a.mode=="preamble"||a.mode=="raw"||a.mode=="clean")||a.rate<0||a.seconds<=0)throw std::runtime_error("invalid arguments");
 if(fs::exists(a.out/"windows.jsonl"))throw std::runtime_error("output exists; choose new run");
 fs::create_directories(a.out);
 std::vector<fs::path>windows;for(auto& d:fs::directory_iterator(a.input/"windows"))if(d.is_directory()&&fs::exists(d.path()/"raw.cf32"))windows.push_back(d.path());std::sort(windows.begin(),windows.end());if(windows.empty())throw std::runtime_error("no input windows");
 const bool discrete=a.reconstruction=="discrete_cir";
 const bool need_old=!discrete || a.mode=="preamble" || a.mode=="both";
 auto init_t0=Clock::now();Engine e(a.input,a.reconstruction=="payload_cached",discrete,need_old,a.lambda,a.peak_floor,a.lag_lo,a.tap_count,a.peak_min_spacing,a.active_radius);double engine_init_us=us(init_t0,Clock::now());std::ofstream log(a.out/"windows.jsonl");std::vector<std::string> modes=a.mode=="both"?std::vector<std::string>{"preamble","full"}:std::vector<std::string>{a.mode};
 auto t0=Clock::now();auto first=read_iq(windows.front()/"raw.cf32");for(size_t j=0;j<a.warmup;++j)for(auto&m:modes)e.process(first,0,0,m,t0,t0,false,0,0);
 t0=Clock::now();size_t submitted=0,completed=0,rejected=0,source_late=0;
 if(a.rate==0){size_t count=a.repeat?a.repeat:windows.size();for(size_t j=0;j<count;++j){size_t src=j%windows.size();auto y=read_iq(windows[src]/(a.mode=="clean"?"clean.cf32":"raw.cf32"));
  // Alternate processing order across observations, with independent estimation in each group.
  auto order=modes;if(j%2)std::reverse(order.begin(),order.end());
  for(auto&m:order){auto arrival=Clock::now();auto r=e.process(y,j,src,m,t0,arrival,false,us(t0,arrival),us(t0,arrival));emit(log,r);if(a.dump){std::ostringstream p;p<<'w'<<std::setfill('0')<<std::setw(6)<<j<<'_'<<m<<".cf32";write_iq(a.out/p.str(),e.out);}++submitted;++completed;}
 }} else {
  if(a.mode!="preamble" && a.mode!="full")throw std::runtime_error("paced modes must be preamble or full separately");
  // Only raw input pool preloaded. Online worker sees no truth or decoded packets.
  size_t pool=std::min<size_t>(16,windows.size());std::vector<std::vector<C>> inputs;for(size_t j=0;j<pool;++j)inputs.push_back(read_iq(windows[j]/"raw.cf32"));
  struct Job {size_t id,src;Clock::time_point arrival;double submit,scheduled;};std::deque<Job> q;std::mutex mu,output;std::condition_variable cv;bool done=false;size_t bound=2;
  t0=Clock::now()+std::chrono::milliseconds(20);
  std::exception_ptr worker_error;
  std::thread worker([&](){try {while(true){Job j;{std::unique_lock<std::mutex>lk(mu);cv.wait(lk,[&](){return done||!q.empty();});if(q.empty()&&done)break;j=q.front();q.pop_front();}
    auto r=e.process(inputs[j.src],j.id,j.src,a.mode,t0,j.arrival,true,j.submit,j.scheduled);{std::lock_guard<std::mutex>lk(output);emit(log,r);++completed;}
   }}catch(...){worker_error=std::current_exception();}});
  size_t total=size_t(a.rate*a.seconds);for(size_t j=0;j<total;++j){auto arrival=t0+std::chrono::nanoseconds(int64_t(j*1e9/a.rate));auto admission=arrival+std::chrono::nanoseconds(int64_t(e.cfg.observed_end_sample*1e9/e.fsamp));std::this_thread::sleep_until(admission);auto now=Clock::now();double scheduled=us(t0,admission),submit=us(t0,now);if(submit-scheduled>1e6/a.rate)++source_late;
   bool full;{std::lock_guard<std::mutex>lk(mu);full=q.size()>=bound;if(!full)q.push_back({j,j%pool,arrival,submit,scheduled});}++submitted;
   if(full){Row r;r.reconstruction=a.reconstruction;r.id=j;r.source=j%pool;r.mode=a.mode;r.n=e.n;r.observed=e.cfg.observed_end_sample;r.required=a.mode=="preamble"?r.observed:e.n;r.reason="queue_full";r.rejected=true;r.submit=r.start=r.finish=r.result=r.first=submit;r.scheduled=scheduled;r.lateness=submit-scheduled;r.ready=us(t0,arrival)+r.required*1e6/e.fsamp;std::lock_guard<std::mutex>lk(output);emit(log,r);++rejected;}else cv.notify_one();
  }{std::lock_guard<std::mutex>lk(mu);done=true;}cv.notify_one();worker.join();if(worker_error)std::rethrow_exception(worker_error);
 }
 log.close();std::ofstream summary(a.out/"runner_summary.json");summary<<"{\"schema\":\"fullpacket-compare/1\",\"reconstruction\":"<<quote(a.reconstruction)<<",\"engine_init_us\":"<<engine_init_us<<",\"prefix_cache_capacity_bytes\":"<<8*(e.full.prefix_key.capacity()+e.full.prefix_rows.capacity())<<",\"rx_pipeline\":"<<quote(experiment::FastRxPipeline::avx2_enabled()?"fast":"portable")<<",\"rx_avx2\":"<<(experiment::FastRxPipeline::avx2_enabled()?"true":"false")<<",\"submitted\":"<<submitted<<",\"completed\":"<<completed<<",\"rejected\":"<<rejected<<",\"source_late\":"<<source_late<<",\"paced_rate\":"<<a.rate<<",\"full_wait_policy\":\"whole_public_context\",\"queue_capacity\":2,\"input_pool\":"<<(a.rate?std::min<size_t>(16,windows.size()):windows.size())<<",\"first_output\":\"first_clean_us\",\"time_reference\":\"run_steady_clock_epoch\"}\n";
 std::cout<<"submitted="<<submitted<<" completed="<<completed<<" rejected="<<rejected<<"\n";return 0;
}catch(const std::exception& ex){std::cerr<<ex.what()<<'\n';return 1;}
