#include "fixed_tone_sc16_kernel.h"
#include <cfenv>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
using fixed_tone_prototype::Kernel;
static void check(bool ok, const char* what) { if(!ok) throw std::runtime_error(what); }
static int16_t qref(double v) {
    double a=std::floor(v), d=v-a;
    if(d>.5 || (d==.5 && std::fmod(a,2.)!=0)) a+=1;
    return int16_t(a);
}
int main() {
    Kernel k; std::complex<double> c{-342.1,35.7}; check(k.set_coefficient(c),"model");
    std::mt19937 gen(42); size_t compared=0;
    for(size_t offset:{0u,1u,7u}) for(size_t phase:{0u,1u,7u,287u,288u,1000000u})
    for(size_t n:{0u,1u,7u,8u,9u,287u,288u,289u,1023u,4097u}) {
        std::vector<int16_t> src(2*n+offset+16), out(2*n+offset+16,12345),ref(2*n);
        for(auto&v:src)v=int16_t(gen());
        for(size_t i=0;i<n;i++) {
            auto z=c*std::exp(std::complex<double>(0,Kernel::omega*((phase+i)%Kernel::period)));
            for(size_t iq=0;iq<2;iq++) {
                int val=int(src[offset+2*i+iq])-int(qref(iq?z.imag():z.real()));
                ref[2*i+iq]=int16_t(std::clamp(val,-32768,32767));
            }
        }
        k.copy_cancel(src.data()+offset,out.data()+offset,n,phase);
        for(size_t i=0;i<2*n;i++){check(out[offset+i]==ref[i],"scalar reference");compared++;}
        for(size_t i=0;i<offset;i++)check(out[i]==12345,"prefix guard");
        for(size_t i=offset+2*n;i<out.size();i++)check(out[i]==12345,"suffix guard");
        auto ip=src;k.cancel_in_place(ip.data()+offset,n,phase);
        for(size_t i=0;i<2*n;i++)check(ip[offset+i]==ref[i],"in place");
        std::fill(out.begin(),out.end(),12345);size_t pos=0;
        while(pos<n){size_t count=std::min(n-pos,size_t(1+gen()%319));k.copy_cancel(src.data()+offset+2*pos,out.data()+offset+2*pos,count,(phase+pos)%Kernel::period);pos+=count;}
        for(size_t i=0;i<2*n;i++)check(out[offset+i]==ref[i],"chunk phase");
    }
    // Independently test folding against direct complex exponential projection.
    for(size_t n:{1u,287u,288u,289u,368640u}) {
        std::vector<int16_t> x(2*n);for(auto&v:x)v=int16_t(gen());
        std::complex<double> scalar{},coefficient;
        for(size_t j=0;j<n;j++)scalar+=std::complex<double>(x[2*j],x[2*j+1])*std::exp(std::complex<double>(0,-Kernel::omega*(j%Kernel::period)));
        scalar/=double(n);check(k.estimate(x.data(),n,coefficient),"estimate");check(abs(scalar-coefficient)<1e-8,"folding reference");
    }
    std::vector<int16_t> zeros(2*288,0),old(2*288),after(2*288);
    k.copy_cancel(zeros.data(),old.data(),288);
    for(auto bad:{std::complex<double>(1e9,0),std::complex<double>(std::numeric_limits<double>::quiet_NaN(),0),std::complex<double>(0,std::numeric_limits<double>::infinity())}) {
        check(!k.set_coefficient(bad),"invalid rejected");k.copy_cancel(zeros.data(),after.data(),288);check(old==after,"model retained");
    }
    std::complex<double> unused;check(!k.estimate(nullptr,0,unused),"empty estimate");
    // Ties-even LUT conversion is independent of fenv at phase 0.
    int prior=std::fegetround();
    for(int mode:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}) {
        std::fesetround(mode);check(k.set_coefficient({.5,1.5}),"tie model");
        int16_t input[2]={0,0},output[2];k.copy_cancel(input,output,1);check(output[0]==0&&output[1]==-2,"ties even");
    }
    std::fesetround(prior);
    std::cout<<"PASS scalar_components="<<compared<<" inplace/chunk/guards/folding/invalid/ties\n";
}
