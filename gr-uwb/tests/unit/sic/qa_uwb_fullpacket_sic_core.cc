/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <boost/test/unit_test.hpp>
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <cstring>
#include <array>
#include <limits>

namespace {
using namespace gr::uwb;
using cf=std::complex<float>;
struct Fixture {
    mod::HrpModConfig p;
    mod::HrpModScratch ms;
    std::vector<uint8_t> bytes,phr;
    std::vector<cf> wave,sync,y,out,ihat;
    sic::SicModel m;
    sic::FullPacketScratch s;
    Fixture(size_t len=32) {
        p.code_index=10;p.sync_repetitions=64;p.sfd_mode="4z2";p.peak_amplitude=.8f;
        bytes.resize(len-2);
        for(size_t i=0;i<bytes.size();++i) bytes[i]=uint8_t(17*i+7);
        mod::append_ieee_fcs(bytes);
        std::vector<int8_t> pb;
        mod::detail::encode_phr19(bytes.size(),64,false,pb);
        phr.assign(pb.begin(),pb.end());
        const size_t cap=mod::packet_samples_998p4(64,8,bytes.size(),false,&p.pulse)+512;
        ms.reserve(127,cap);wave.resize(cap);size_t count=0;
        BOOST_REQUIRE(mod::modulate_one(bytes.data(),bytes.size(),p,ms,wave.data(),cap,count));
        wave.resize(count);sync.assign(wave.begin(),wave.begin()+1016);
        for(auto&v:sync)v*=.37f;
        y.assign(count+512,cf(.001f,-.002f));out.resize(y.size());ihat.resize(y.size());
        s.reserve(y.size());
        m.valid=true;m.state=sic::SicState::Committed;m.n_paths=1;
        m.tau_abs[0]=128;m.gain[0]=cf(.8f,-.2f);m.cfo_hz=1700;
        m.sample_rate=998.4e6;m.samples_per_symbol=1016;m.nominal_sync_reps=64;
        m.phase_reference_sample=128;
    }
    sic::FullPacketResult run(int64_t lo=0,int64_t hi=-1,bool fcs=true) {
        return sic::fullpacket_cancel(y.data(),y.size(),phr,bytes,fcs,p,m,sync.data(),sync.size(),
                                     lo,hi<0?int64_t(y.size()):hi,s,out.data(),ihat.data());
    }
    void exact_bypass() const {
        BOOST_CHECK_EQUAL(std::memcmp(y.data(),out.data(),y.size()*sizeof(cf)),0);
        BOOST_CHECK(std::all_of(ihat.begin(),ihat.end(),[](cf z){return z==cf(0);}));
    }
};
}

BOOST_AUTO_TEST_CASE(fullpacket_integer_cfo_scale_and_length_edges) {
    for(size_t length:{size_t(16),size_t(127)}) {
        Fixture f(length);
        for(size_t j=0;j<f.wave.size();++j) {
            const cf rot(std::polar(1.0,2*3.14159265358979323846*f.m.cfo_hz*j/f.m.sample_rate));
            f.y[j+128]+=f.wave[j]*f.m.gain[0]*.37f*rot;
        }
        const auto r=f.run();
        BOOST_REQUIRE_MESSAGE(r.committed,r.reason);
        BOOST_CHECK_CLOSE(r.waveform_scale,.37,0.001);
        BOOST_CHECK_EQUAL(r.packet_samples,f.wave.size());
        float worst=0;
        for(size_t i=0;i<f.y.size();++i) worst=std::max(worst,std::abs(f.out[i]-cf(.001f,-.002f)));
        BOOST_CHECK_LT(worst,5e-7f);
    }
}

BOOST_AUTO_TEST_CASE(fullpacket_rejects_fcs_phr_and_nonfinite_transactionally) {
    Fixture f;
    BOOST_CHECK(!f.run(0,-1,false).committed);f.exact_bypass();
    f.bytes[0]^=1;
    BOOST_CHECK(!f.run().committed);f.exact_bypass();
    f.bytes[0]^=1;f.phr[9]^=1;
    BOOST_CHECK(!f.run().committed);f.exact_bypass();
    f.phr[9]^=1;f.m.gain[0]={std::numeric_limits<float>::infinity(),0};
    BOOST_CHECK(!f.run().committed);f.exact_bypass();
    f.m.gain[0]={.8f,-.2f};f.y.back()={std::numeric_limits<float>::quiet_NaN(),0};
    BOOST_CHECK(!f.run().committed);f.exact_bypass();
}

BOOST_AUTO_TEST_CASE(fullpacket_fractional_operator_matches_frozen_core_and_roi) {
    Fixture f;
    f.m.n_paths=2;f.m.tau_abs[0]=128.4;f.m.tau_abs[1]=151.99;
    f.m.gain[1]={.2f,.1f};
    // Reuse the already validated frozen preamble operator as a regression
    // witness, passing the same finite packet as its template. This tests the
    // fractional/CFO extension, not the shared encoder's correctness.
    std::vector<cf> scaled=f.wave;
    for(auto&v:scaled)v*=.37f;
    sic::SicConfig cfg;cfg.mode=sic::SicMode::Estimate;cfg.roi_lo=64;cfg.roi_hi=192;
    sic::SicScratch scratch;
    std::string err;
    BOOST_REQUIRE_MESSAGE(sic::prepare_sic_core(cfg,f.sync.data(),f.sync.size(),scaled.data(),scaled.size(),scratch,&err),err);
    sic::SicTemplates templates;
    templates.sync=f.sync.data();templates.sync_len=f.sync.size();
    templates.preamble=scaled.data();templates.preamble_len=scaled.size();
    f.m.sub_lo=200;f.m.sub_hi=int64_t(f.y.size())-32;
    f.m.predicted_support_lo=128;f.m.predicted_support_hi=int64_t(f.wave.size())+152;
    std::vector<cf> expected(f.y.size()),expected_i(f.y.size());
    BOOST_REQUIRE(sic::apply_model(f.m,f.y.data(),f.y.size(),0,templates,cfg,scratch,expected.data(),expected_i.data()));
    const auto r=f.run(f.m.sub_lo,f.m.sub_hi);
    BOOST_REQUIRE_MESSAGE(r.committed,r.reason);
    float worst=0;
    for(size_t i=0;i<f.y.size();++i) {
        worst=std::max(worst,std::abs(f.out[i]-expected[i]));
        if(i<size_t(f.m.sub_lo)||i>=size_t(f.m.sub_hi)) BOOST_CHECK_EQUAL(f.out[i],f.y[i]);
    }
    BOOST_CHECK_LT(worst,2e-6f);
}

BOOST_AUTO_TEST_CASE(fullpacket_truncated_tail_and_wrong_template_bypass) {
    Fixture f;
    f.m.tau_abs[0]=double(f.y.size()-64);
    const auto truncated=f.run();
    BOOST_CHECK(!truncated.committed);
    BOOST_CHECK_EQUAL(std::string(truncated.reason),"context_truncated");f.exact_bypass();
    f.m.tau_abs[0]=128;f.sync[80]+=.25f;
    const auto mismatch=f.run();
    BOOST_CHECK(!mismatch.committed);
    BOOST_CHECK_EQUAL(std::string(mismatch.reason),"template_scale_mismatch");f.exact_bypass();
}

BOOST_AUTO_TEST_CASE(fullpacket_cfo_matches_scalar_absolute_phase) {
    // Cover negative relative phase, block edges and fractional multipath.
    for(double cfo:{0.0,-50000.0,50000.0}) {
        Fixture f;
        f.m.cfo_hz=cfo;f.m.phase_reference_sample=2048;
        f.m.n_paths=2;f.m.tau_abs[0]=128.4;f.m.tau_abs[1]=151.99;
        f.m.gain[1]={.2f,.1f};
        const auto r=f.run();BOOST_REQUIRE_MESSAGE(r.committed,r.reason);
        std::array<cf,1024> base;
        const double w=2*3.14159265358979323846*cfo/f.m.sample_rate;
        std::complex<double> cur=1,step=std::exp(std::complex<double>(0,w));
        for(auto& v:base){v=cf(cur);cur*=step;}
        float worst=0;
        for(size_t j=0;j<f.y.size();++j) {
            const int64_t p=int64_t(j)-f.m.phase_reference_sample;
            const int64_t b=p/1024-((p%1024)<0?1:0);
            const cf block(std::exp(std::complex<double>(0,w*double(b*1024))));
            const cf expected=cf(f.s.accum[j])*(base[size_t(p-b*1024)]*block);
            worst=std::max(worst,std::abs(f.ihat[j]-expected));
        }
        BOOST_CHECK_LT(worst,2e-7f);
    }
}

BOOST_AUTO_TEST_CASE(fullpacket_conversion_and_subtraction_overflow_bypass) {
    Fixture conversion;
    for(auto& v:conversion.sync)v*=1e30f;
    conversion.m.gain[0]={1e30f,0};
    const auto a=conversion.run();
    BOOST_CHECK(!a.committed);
    BOOST_CHECK_EQUAL(std::string(a.reason),"nonfinite_reconstruction");
    conversion.exact_bypass();
    Fixture trial;
    trial.m.cfo_hz=0;
    trial.m.gain[0]={std::numeric_limits<float>::max()*.8f,0};
    const auto peak=std::max_element(trial.wave.begin(),trial.wave.end(),
        [](cf a,cf b){return a.real()<b.real();});
    trial.y[128+size_t(peak-trial.wave.begin())]={-std::numeric_limits<float>::max(),0};
    const auto b=trial.run();
    BOOST_CHECK(!b.committed);
    BOOST_CHECK_EQUAL(std::string(b.reason),"nonfinite_reconstruction");
    trial.exact_bypass();
}
