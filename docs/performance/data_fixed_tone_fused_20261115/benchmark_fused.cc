#include "fixed_tone_sc16_kernel.h"
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <random>
#include <sched.h>
#include <sys/file.h>
#include <unistd.h>
#include <vector>
using fixed_tone_prototype::Kernel;
constexpr size_t N=14745600,T=368640;
volatile double sink=0;
template<class F>double timed(F f){auto s=std::chrono::steady_clock::now();f();return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-s).count();}
void stats(const char*name,std::vector<double>v){std::sort(v.begin(),v.end());std::cout<<name<<" median_us="<<v[v.size()/2]<<" min_us="<<v.front()<<" max_us="<<v.back()<<"\n";}
int main(){int fd=open("/tmp/uwb-sic-benchmark.lock",O_CREAT|O_RDWR,0666);if(fd<0||flock(fd,LOCK_EX|LOCK_NB)){std::cerr<<"benchmark lock busy\n";return 2;}
 std::cout<<std::setprecision(10)<<"CPU"<<sched_getcpu()<<" single-thread AVX2; 737.28MS/s; -232.96MHz; 20ms; session groups=9 ABBA\n";
 std::vector<int16_t>src(2*N),dst(2*N);std::mt19937 gen(42);for(auto&v:src)v=int16_t(gen());
 Kernel k; if(!k.set_coefficient({-342.1,35.7}))return 3;
 // Bit-exact paths: same model, all samples, arbitrary chunk boundaries.
 std::vector<int16_t>reference(2*N);std::memcpy(reference.data(),src.data(),src.size()*2);k.cancel_in_place(reference.data(),N);
 for(size_t chunk:{N,size_t(16384),size_t(65536)}) {
  for(size_t a=0;a<N;a+=chunk)k.copy_cancel(src.data()+2*a,dst.data()+2*a,std::min(chunk,N-a),a%Kernel::period);
  if(reference!=dst){std::cerr<<"functional mismatch\n";return 4;}
 }
 std::cout<<"functional PASS: 20ms x3 chunk geometries vs memcpy+inplace; 0 differing components\n";
 reference.clear();reference.shrink_to_fit();
 std::cout<<"chunk_samples,session,group,A1_us,B1_us,B2_us,A2_us,ratio_B_over_A\n";
 for(size_t chunk:{N,size_t(16384),size_t(65536)}) {
  auto a=[&]{for(size_t off=0;off<N;off+=chunk){size_t count=std::min(chunk,N-off);std::memcpy(dst.data()+2*off,src.data()+2*off,count*4);k.cancel_in_place(dst.data()+2*off,count,off%Kernel::period);}};
  auto b=[&]{for(size_t off=0;off<N;off+=chunk){size_t count=std::min(chunk,N-off);k.copy_cancel(src.data()+2*off,dst.data()+2*off,count,off%Kernel::period);}};
  a();b();std::vector<double>av,bv;
  for(int session=0;session<3;session++)for(int group=0;group<9;group++) {
   double a1=timed(a);sink=dst[group];double b1=timed(b);sink=dst[group];double b2=timed(b);sink=dst[group];double a2=timed(a);sink=dst[group];
   av.push_back((a1+a2)/2);bv.push_back((b1+b2)/2);
   std::cout<<chunk<<","<<session<<","<<group<<","<<a1<<","<<b1<<","<<b2<<","<<a2<<","<<(b1+b2)/(a1+a2)<<"\n";
  }
  std::cout<<"summary chunk="<<chunk<<"\n";stats("copy_then_cancel",av);stats("fused_copy_cancel",bv);
 }
 std::vector<double>est,table,ip;
 for(int rep=0;rep<25;rep++) {
  std::complex<double>c;
  est.push_back(timed([&]{if(!k.estimate(src.data(),T,c))std::abort();}));sink=abs(c);
  table.push_back(timed([&]{if(!k.set_coefficient(c))std::abort();}));
  std::memcpy(dst.data(),src.data(),src.size()*2);
  ip.push_back(timed([&]{k.cancel_in_place(dst.data(),N);}));sink=dst[rep];
 }
 stats("estimate_precomputed_basis_0p5ms",est);stats("table_update_no_sincos",table);stats("inplace_owned_buffer_20ms",ip);
 close(fd);
}
