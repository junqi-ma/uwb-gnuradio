#define main decode_unused_main
#include "decode_windows.cc"
#undef main
int main(){auto x=load<std::complex<float>>("testdata/x410_fixed_tone_live_20261115/first_window.cf32"),t=load<std::complex<float>>("testdata/reference_preamble_code10_998p4.cf32");demod::Qm35825Profile p;p.code_index=10;p.preamble_repetitions=256;demod::core::DemodScratch s;demod::TimingResult r;demod::CfoResult f;demod::core::stage_timing(x.data(),x.size(),p,t,19968,r,s);demod::core::fit_cfo(x.data(),x.size(),p,r,f,s);std::cout<<"cfo "<<f.cfo_hz<<" peaks "<<r.peak_samples.size()<<"\n";for(int i=0;i<12;i++)std::cout<<r.peak_samples[i]<<" "<<std::arg(r.peak_corr[i])<<"\n";}
