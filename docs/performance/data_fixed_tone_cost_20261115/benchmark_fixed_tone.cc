// Standalone cost prototype; not a production pipeline implementation.
#include <immintrin.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
constexpr size_t P=288,N=14745600,T=368640;
constexpr double W=-2*3.14159265358979323846*91/288;
volatile double sink=0;
struct alignas(32) Table { std::array<int16_t,2*P> iq; };
std::complex<double> estimate(const int16_t*x,size_t n){
 // At most ceil(n/P)*32768 per component: int32 is safe for this 0.5 ms.
 alignas(32) std::array<int32_t,2*P> fold{};
 size_t j=0;for(;j+P<=n;j+=P){for(size_t k=0;k<2*P;k++)fold[k]+=x[2*j+k];}
 for(size_t k=0;k<2*(n-j);k++)fold[k]+=x[2*j+k];
 std::complex<double> c(0,0);for(size_t k=0;k<P;k++){
  double co=std::cos(W*k),si=std::sin(W*k);
  c+=std::complex<double>(fold[2*k]*co+fold[2*k+1]*si,fold[2*k+1]*co-fold[2*k]*si);
 }return c/double(n);
}
Table make_table(std::complex<double>c){Table t;for(size_t k=0;k<P;k++){
 auto v=c*std::complex<double>(std::cos(W*k),std::sin(W*k));
 // Saturating subtraction below exactly implements subtract-rounded-tone.
 // This narrow prototype only accepts a tone fitting int16; general code
 // must handle larger models with int32 or bypass, never silently clip it.
 t.iq[2*k]=int16_t(std::lrint(v.real()));t.iq[2*k+1]=int16_t(std::lrint(v.imag()));
 }return t;}
void cancel(const int16_t*src,int16_t*dst,size_t n,const Table&t,size_t phase=0){
 size_t a=0;
 // Period 288 is divisible by 8 complex samples / AVX2 vector.
 while(a<n){size_t cnt=std::min(n-a,P-phase);size_t j=0;
  for(;j+8<=cnt;j+=8){auto x=_mm256_loadu_si256((const __m256i*)(src+2*(a+j)));auto v=_mm256_loadu_si256((const __m256i*)(t.iq.data()+2*(phase+j)));_mm256_storeu_si256((__m256i*)(dst+2*(a+j)),_mm256_subs_epi16(x,v));}
  for(;j<cnt;j++)for(int iq=0;iq<2;iq++){int z=int(src[2*(a+j)+iq])-int(t.iq[2*(phase+j)+iq]);dst[2*(a+j)+iq]=int16_t(std::clamp(z,-32768,32767));}
  a+=cnt;phase=0;
 }
}
template<class F>double timed(F f){auto start=Clock::now();f();return std::chrono::duration<double,std::micro>(Clock::now()-start).count();}
void stats(const char*name,std::vector<double>v){std::sort(v.begin(),v.end());std::cout<<name<<" median_us="<<v[v.size()/2]<<" min_us="<<v.front()<<" max_us="<<v.back()<<"\n";}
int main(){int lock=open("/tmp/uwb-sic-benchmark.lock",O_CREAT|O_RDWR,0666);if(lock<0||flock(lock,LOCK_EX|LOCK_NB)){std::cerr<<"benchmark lock busy\n";return 2;}
 std::cout<<std::setprecision(10)<<"sample_rate=737280000 tone_hz=-232960000 period_samples="<<P<<" train_samples="<<T<<" period_samples_20ms="<<N<<"\n";
 std::vector<int16_t>src(2*N),dst(2*N);std::mt19937 rng(42);for(auto&v:src)v=int16_t(rng());
 auto lut=make_table({-342.1,35.7});cancel(src.data(),dst.data(),N,lut);
 size_t bad=0;for(size_t i=0;i<2*N;i++){int ref=int(src[i])-int(lut.iq[i%(2*P)]);if(dst[i]!=std::clamp(ref,-32768,32767))bad++;}
 // Chunking check includes arbitrary offsets, tails and period wrap.
 std::vector<int16_t>chunked(2*P*7);size_t nn=chunked.size()/2,pos=0;
 for(size_t span:{1u,7u,288u,1023u,697u}){size_t n=std::min(span,nn-pos);cancel(src.data()+2*pos,chunked.data()+2*pos,n,lut,pos%P);pos+=n;}
 if(pos<nn)cancel(src.data()+2*pos,chunked.data()+2*pos,nn-pos,lut,pos%P);
 for(size_t i=0;i<chunked.size();i++)if(chunked[i]!=dst[i])bad++;
 std::complex<double> scalar(0,0);for(size_t j=0;j<T;j++){size_t k=j%P;scalar+=std::complex<double>(src[2*j],src[2*j+1])*std::complex<double>(std::cos(W*k),-std::sin(W*k));}scalar/=double(T);
 auto folded=estimate(src.data(),T);std::cout<<"integer_reference_mismatches="<<bad<<" folding_coefficient_error_adc="<<std::abs(scalar-folded)<<"\n";
 if(bad||std::abs(scalar-folded)>1e-8)return 3;
 std::vector<double>fit,table,copy,oop,inplace,full,stream;
 for(int rep=0;rep<17;rep++){
  std::complex<double>c;Table tmp;
  fit.push_back(timed([&]{c=estimate(src.data(),T);}));sink=abs(c);
  table.push_back(timed([&]{tmp=make_table(c);}));sink=tmp.iq[rep];
  // ABBA copy and out-of-place candidate, alternating pair order each round.
  auto cp=[&]{copy.push_back(timed([&]{std::memcpy(dst.data(),src.data(),src.size()*2);}));sink=dst[rep];};
  auto op=[&]{oop.push_back(timed([&]{cancel(src.data(),dst.data(),N,lut);}));sink=dst[rep];};
  if(rep%2){op();cp();cp();op();}else{cp();op();op();cp();}
  std::memcpy(dst.data(),src.data(),src.size()*2);
  inplace.push_back(timed([&]{cancel(dst.data(),dst.data(),N,lut);}));sink=dst[rep];
  full.push_back(timed([&]{auto cc=estimate(src.data(),T);auto tt=make_table(cc);cancel(src.data(),dst.data(),N,tt);}));sink=dst[rep];
  stream.push_back(timed([&]{for(size_t a=0;a<N;a+=16384){auto count=std::min(size_t(16384),N-a);cancel(src.data()+2*a,dst.data()+2*a,count,lut,a%P);}}));sink=dst[rep];
 }
 stats("estimate_0p5ms",fit);stats("build_288_sample_lut",table);stats("memcpy_20ms",copy);stats("cancel_20ms_out_of_place",oop);stats("cancel_20ms_in_place",inplace);stats("estimate_build_cancel_20ms",full);stats("cancel_20ms_16k_chunks",stream);
 close(lock);
}
