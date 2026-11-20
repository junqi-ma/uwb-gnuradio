// Offline tap-count cost experiment; production algorithm unchanged.
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace openuwb;
using cf=std::complex<float>;
using cd=std::complex<double>;
using Clock=std::chrono::steady_clock;
int main() {
 try {
  mod::HrpModConfig profile;
  profile.code_index=10; profile.sync_repetitions=64;
  profile.sfd_mode="4z2"; profile.peak_amplitude=0.8f;
  auto bytes=mod::make_random_psdu(30,20261106,true);
  std::vector<int8_t> bits;
  mod::detail::encode_phr19(bytes.size(),64,false,bits);
  std::vector<uint8_t> phr(bits.begin(),bits.end());
  const size_t cap=mod::packet_samples_998p4(64,8,bytes.size(),false,&profile.pulse)+64;
  mod::HrpModScratch ms; ms.reserve(127,cap);
  std::vector<cf> wave(cap); size_t plen=0;
  if(!mod::modulate_one(bytes.data(),bytes.size(),profile,ms,wave.data(),cap,plen))
   throw std::runtime_error("modulation failed");
  wave.resize(plen);
  const size_t n=plen+4096;
  std::vector<cf> ref(wave.begin(),wave.begin()+1016),y(n,cf(.01f,.02f)),out(n),ihat(n);
  sic::DiscreteCirScratch scratch; scratch.reserve(n);
  scratch.prepared=true; scratch.sps=1016; scratch.max_packet_samples=cap;
  auto model=[](int k) {
   sic::DiscreteCirModel m; m.valid=true;m.q=1024;m.phase_reference_sample=m.q;
   m.lag_lo=0;m.tap_count=64;m.active_count=k;m.cfo_hz=50000;
   m.observed_end_sample=m.q+8*1016;m.model_id=k;
   for(int a=0;a<k;++a) {
    int lag=k==1?0:a*63/(k-1);
    m.active_lags[a]=lag;
    m.taps[lag]=std::polar(float(1/std::sqrt(double(k))),float(.37*a));
   }
   return m;
  };
  auto run=[&](const sic::DiscreteCirModel&m) {
   auto r=sic::fullpacket_cancel_discrete(y.data(),n,phr,bytes,true,profile,m,
                  ref.data(),ref.size(),0,n,scratch,out.data(),ihat.data());
   if(!r.committed) throw std::runtime_error(r.reason);
   return r;
  };
  std::vector<int> counts={1,3,6,12,24,48,64};
  // Independent dense double convolution, all output samples, outside timing.
  double worst=0;
  for(int k:counts) {
   auto m=model(k);run(m);double err=0,power=0;
   for(size_t j=0;j<n;++j) {
    cd sum=0;
    for(int lag=0;lag<64;++lag) {
     auto ix=int64_t(j)-m.q-lag;
     if(ix>=0&&ix<int64_t(plen))sum+=cd(m.taps[lag])*cd(wave[ix]);
    }
    sum*=std::polar(1.,2*std::acos(-1.)*m.cfo_hz/998.4e6*(double(j)-m.q));
    err+=std::norm(cd(ihat[j])-sum);power+=std::norm(sum);
    if(!std::isfinite(out[j].real())||!std::isfinite(out[j].imag()) ||
       std::abs(out[j]-(y[j]-ihat[j]))>1e-6f)throw std::runtime_error("subtract mismatch");
   }
   worst=std::max(worst,std::sqrt(err/power));
  }
  if(worst>2e-5)throw std::runtime_error("reference error");
  std::cerr<<std::setprecision(12)<<"packet_samples="<<plen<<" window_samples="<<n
           <<" max_relative_l2="<<worst<<"\n";
  std::cout<<"round,taps,iteration,convolution_us,channel_us,wrapper_us,tx_us,validate_us\n"<<std::setprecision(12);
  for(int round=0;round<2;++round) {
   if(round)std::reverse(counts.begin(),counts.end());
   for(int k:counts) {
    auto m=model(k);
    for(int w=0;w<16;++w)run(m);
    for(int i=0;i<128;++i) {
     auto start=Clock::now();auto r=run(m);
     double wall=std::chrono::duration<double,std::micro>(Clock::now()-start).count();
     std::cout<<round<<','<<k<<','<<i<<','<<r.convolution_us<<','<<r.channel_us<<','
              <<wall<<','<<r.tx_us<<','<<r.validate_us<<'\n';
    }
   }
  }
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
