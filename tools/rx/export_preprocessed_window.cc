// Export a known raw window through production tone and resampler cores.
#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <complex>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace gr::uwb;
template<class T> std::vector<T> load(const std::string& path) {
    std::ifstream f(path,std::ios::binary|std::ios::ate);
    if(!f)throw std::runtime_error(path);
    size_t bytes=f.tellg();if(bytes%sizeof(T))throw std::runtime_error("size");
    std::vector<T> x(bytes/sizeof(T));f.seekg(0);
    if(!f.read(reinterpret_cast<char*>(x.data()),bytes))throw std::runtime_error("read");
    return x;
}
std::vector<std::complex<int16_t>> read(const std::string& path,int64_t start,size_t n) {
    if(start<0)throw std::runtime_error("negative offset");
    std::ifstream f(path,std::ios::binary);f.seekg(start*4);
    std::vector<std::complex<int16_t>> x(n);
    if(!f.read(reinterpret_cast<char*>(x.data()),n*4))throw std::runtime_error("read window");
    return x;
}
int main(int argc,char**argv)try {
    if(argc!=6)throw std::runtime_error("SOURCE START COUNT PREFIX RESOURCE_ROOT");
    const std::string source=argv[1],prefix=argv[4],res=argv[5];
    int64_t start=std::stoll(argv[2]);size_t count=std::stoull(argv[3]);
    auto x=read(source,start,count);
    core::FixedToneCancelCore tone;core::FixedToneCancelConfig cfg;cfg.enable=true;
    std::string error;if(!tone.prepare(cfg,&error))throw std::runtime_error(error);
    size_t done=0;
    while(done<count) {
        int64_t epoch=(start+done)/tone.period_length_samples()*tone.period_length_samples();
        // Load the previous epoch first so the production crossfade is also
        // defined correctly if a window intersects an update boundary.
        tone.reset();
        for(int back=(epoch>0?1:0);back>=0;--back) {
            int64_t at=epoch-back*tone.period_length_samples();
            auto training=read(source,at,tone.training_length_samples());
            if(!tone.begin_training(at)||!tone.add_training_samples(
                reinterpret_cast<int16_t*>(training.data()),training.size())||
                tone.finish_training()!=core::FixedToneTrainingStatus::Ok)
                throw std::runtime_error("training");
        }
        size_t n=std::min<size_t>(count-done,epoch+tone.period_length_samples()-start-done);
        auto c=tone.cancel_in_place(reinterpret_cast<int16_t*>(x.data()+done),n,start+done);
        if(c.cancelled!=n||c.bypassed)throw std::runtime_error("uncancelled samples");
        done+=n;
    }
    std::ofstream native(prefix+"_tone_cancelled.sc16",std::ios::binary);
    native.write(reinterpret_cast<char*>(x.data()),x.size()*4);
    auto taps=load<float>(res+"/resampler_65_48/taps_realtime_eq16.txt");
    core::RationalResampler65_48Core rs(taps);rs.set_fixed_phase_schedule(true);
    std::vector<std::complex<float>> y(rs.expected_output_length(count));
    size_t made=rs.process_sc16(x.data(),count,1.f/32768,y.data(),y.size()).produced;
    while(made<y.size()) {size_t n=rs.flush(y.data()+made,y.size()-made);if(!n)break;made+=n;}
    std::ofstream work(prefix+"_tone_cancelled_resampled.cf32",std::ios::binary);
    work.write(reinterpret_cast<char*>(y.data()),made*8);
    if(!native||!work)throw std::runtime_error("write");
    std::cout<<"{\"native_samples\":"<<count<<",\"work_samples\":"<<made
             <<",\"filter_taps\":"<<taps.size()<<"}\n";
    return 0;
}catch(std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
