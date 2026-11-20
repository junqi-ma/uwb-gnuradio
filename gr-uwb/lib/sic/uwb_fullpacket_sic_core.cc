/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <openuwb/sic/fullpacket_sic_core.h>
#include <array>
#include <volk/volk.h>
#include <chrono>
#include <cstring>
#include <limits>

namespace gr { namespace uwb { namespace sic {
namespace {
using cf = std::complex<float>;
using cd = std::complex<double>;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
constexpr size_t phases = 32, half = 16, taps_count = 33, rot_block = 1024;
// The continuous convolutional encoder feeds PHR19 then payload bits, but
// emits21 physical PHR symbols. Its last two symbols therefore depend on the
// first payload bits. Keep those symbols, a conservative shaping guard and the
// fractional FIR's future halo in the dynamic tail.
constexpr size_t variable_phr_start = 64 * 1016 + 8 * 1016 + 19 * 1024;
constexpr size_t prefix_input = variable_phr_start - 64;
constexpr size_t prefix_output = prefix_input - half;
double us(Clock::time_point t) {
    return std::chrono::duration<double, std::micro>(Clock::now() - t).count();
}
bool finite(cf v) { return std::isfinite(v.real()) && std::isfinite(v.imag()); }
uint16_t crc(const uint8_t* p, size_t n) {
    uint16_t c = 0;
    for (size_t i=0;i<n;++i) {
        c ^= p[i];
        for (int b=0;b<8;++b) c = (c&1) ? (c>>1)^0x8408 : c>>1;
    }
    return c;
}
// Same normalized Hamming-windowed 33-tap phase rows as preamble core.
// Each selected row is rounded once to float32 before phase blending.
void phase_row(const cf* wave, size_t n, size_t g, cf* row, cd* tmp) {
    if (!g) { std::copy(wave,wave+n,row); return; }
    double taps[taps_count], sum=0;
    const double tau=double(g)/phases;
    for (size_t p=0;p<taps_count;++p) {
        const double j=double(p)-half, x=j-tau;
        taps[p] = (std::abs(x)<1e-12 ? 1.0 : std::sin(pi*x)/(pi*x)) *
            (0.54-0.46*std::cos(2*pi*p/(taps_count-1)));
        sum += taps[p];
    }
    for (auto& t:taps) t *= 1.0/sum;
    std::fill(tmp,tmp+n,cd(0));
    for (size_t p=0;p<taps_count;++p) {
        const int64_t shift=int64_t(p)-half;
        const int64_t lo=std::max<int64_t>(0,shift);
        const int64_t hi=std::min<int64_t>(n,int64_t(n)+shift);
        for(int64_t j=lo;j<hi;++j) tmp[j] += cd(wave[j-shift])*taps[p];
    }
    for(size_t j=0;j<n;++j) row[j]=cf(float(tmp[j].real()),float(tmp[j].imag()));
}
// Compute the SAME output samples as phase_row, but only [first,last).
// Source indices remain packet-global: the dynamic tail sees both sides of the
// split, rather than treating the payload boundary as a new zero-padded signal.
void phase_row_range(const cf* wave, size_t n, size_t g, cf* row, cd* tmp,
                     size_t first, size_t last) {
    if (!g) { std::copy(wave+first,wave+last,row+first); return; }
    double taps[taps_count], sum=0;
    const double tau=double(g)/phases;
    for (size_t p=0;p<taps_count;++p) {
        const double j=double(p)-half, x=j-tau;
        taps[p] = (std::abs(x)<1e-12 ? 1.0 : std::sin(pi*x)/(pi*x)) *
            (0.54-0.46*std::cos(2*pi*p/(taps_count-1)));
        sum += taps[p];
    }
    for (auto& t:taps) t *= 1.0/sum;
    std::fill(tmp+first,tmp+last,cd(0));
    for (size_t p=0;p<taps_count;++p) {
        const int64_t shift=int64_t(p)-half;
        const int64_t lo=std::max<int64_t>(int64_t(first),std::max<int64_t>(0,shift));
        const int64_t hi=std::min<int64_t>(int64_t(last),std::min<int64_t>(n,int64_t(n)+shift));
        for(int64_t j=lo;j<hi;++j) tmp[j] += cd(wave[j-shift])*taps[p];
    }
    // Preserve double accumulation and the original float32 row rounding.
    for(size_t j=first;j<last;++j) row[j]=cf(float(tmp[j].real()),float(tmp[j].imag()));
}

int64_t floordiv(int64_t a,int64_t b) {
    return a/b-((a%b)<0 ? 1 : 0);
}
} // namespace

void FullPacketScratch::reserve(size_t max_samples) {
    capacity=max_samples;
    modulator.reserve(127,max_samples);
    waveform.resize(max_samples); model.resize(max_samples);
    phase0.resize(max_samples); phase1.resize(max_samples);
    accum.resize(max_samples); phase_tmp.resize(max_samples);
    expected_phr.reserve(19);
    prefix_cache_enabled=false;prefix_key_valid=false;prefix_phase_mask=0;
    prefix_input_len=0;prefix_output_len=0;
}

bool FullPacketScratch::prepare_prefix_cache() {
    prefix_cache_enabled=false;prefix_key_valid=false;prefix_phase_mask=0;
    prefix_input_len=0;prefix_output_len=0;
    if(capacity<prefix_input || waveform.size()<capacity || phase_tmp.size()<capacity)
        return false;
    prefix_key.resize(prefix_input);
    prefix_rows.resize(phases*prefix_output);
    prefix_input_len=prefix_input;prefix_output_len=prefix_output;
    prefix_cache_enabled=true;
    return true;
}

FullPacketResult fullpacket_cancel(
 const cf* y,size_t n,const std::vector<uint8_t>& phr,
 const std::vector<uint8_t>& psdu,bool fcs_pass,const mod::HrpModConfig& profile,
 const SicModel& m,const cf* reference_sync,size_t sync_n,
 int64_t roi_lo,int64_t roi_hi,FullPacketScratch& s,cf* out,cf* ihat) {
    FullPacketResult r;
    const auto tv=Clock::now();
    auto fail=[&](const char* reason) {r.reason=reason; r.validate_us=us(tv); return r;};
    if(!y || !out || !n || (ihat && (ihat==y || ihat==out))) return fail("invalid_buffers");
    std::copy(y,y+n,out);
    if(ihat) std::fill(ihat,ihat+n,cf(0));
    if(n>s.capacity || s.waveform.size()<s.capacity ||
       s.model.size()<n || s.accum.size()<n || s.phase_tmp.size()<s.capacity ||
       s.phase0.size()<s.capacity || s.phase1.size()<s.capacity)
        return fail("scratch_capacity");
    if(!fcs_pass) return fail("fcs_failed");
    if(profile.code_index!=10 || profile.sync_repetitions!=64 || profile.insert_sts ||
       !profile.sfd_mode || std::strcmp(profile.sfd_mode,"4z2") ||
       profile.ranging || profile.pulse.shape!=mod::PulseShape::Legacy)
        return fail("unsupported_profile");
    if(phr.size()!=19 || psdu.size()<2 || psdu.size()>127)
        return fail("invalid_frame_length");
    if(crc(psdu.data(),psdu.size()-2)!=(uint16_t(psdu[psdu.size()-2])|(uint16_t(psdu.back())<<8)))
        return fail("fcs_failed");
    mod::detail::encode_phr19(psdu.size(),profile.sync_repetitions,profile.ranging,s.expected_phr);
    for(size_t i=0;i<19;++i)
        if(phr[i]>1 || phr[i]!=uint8_t(s.expected_phr[i])) return fail("unsupported_phr");
    if(!m.valid || m.state!=SicState::Committed || !m.n_paths || m.n_paths>kSicMaxPaths ||
       m.sample_rate!=998.4e6 || m.samples_per_symbol!=1016 ||
       m.nominal_sync_reps!=64 || !std::isfinite(m.cfo_hz) ||
       m.phase_reference_sample<0 || m.phase_reference_sample>int64_t(n) ||
       !reference_sync || sync_n!=1016 || roi_lo<0 || roi_hi<=roi_lo || roi_hi>int64_t(n))
        return fail("invalid_model_or_roi");
    for(size_t k=0;k<m.n_paths;++k)
        if(!std::isfinite(m.tau_abs[k]) || m.tau_abs[k]<0 || m.tau_abs[k]>double(n) || !finite(m.gain[k]))
            return fail("nonfinite_or_invalid_path");
    for(size_t i=0;i<n;++i) if(!finite(y[i])) return fail("nonfinite_input");
    r.validate_us=us(tv);
    const auto tt=Clock::now();
    size_t plen=0;
    if(!mod::modulate_one(psdu.data(),psdu.size(),profile,s.modulator,
                         s.waveform.data(),s.capacity,plen) || plen<sync_n) {
        r.reason="tx_failed";r.tx_us=us(tt);return r;
    }
    // Determine exact shaped-waveform convention relative to estimator template.
    cd cross=0;double energy=0,refenergy=0;
    for(size_t i=0;i<sync_n;++i) {
        if(!finite(reference_sync[i])) {r.reason="nonfinite_template";r.tx_us=us(tt);return r;}
        cross+=std::conj(cd(s.waveform[i]))*cd(reference_sync[i]);
        energy+=std::norm(cd(s.waveform[i]));refenergy+=std::norm(cd(reference_sync[i]));
    }
    const cd scale=energy>0 ? cross/energy : cd(0);
    double error=0;
    for(size_t i=0;i<sync_n;++i) error+=std::norm(cd(s.waveform[i])*scale-cd(reference_sync[i]));
    if(!(scale.real()>0) || !std::isfinite(scale.real()) ||
       std::abs(scale.imag())>1e-6*scale.real() || !(refenergy>0) || error>1e-10*refenergy) {
        r.reason="template_scale_mismatch";r.tx_us=us(tt);return r;
    }
    r.waveform_scale=scale.real();r.packet_samples=plen;
    // Scale gains rather than waveform: identical convention, one multiplication/path.
    r.tx_us=us(tt);
    const auto tc=Clock::now();
    int64_t lo=int64_t(std::floor(m.tau_abs[0])),hi=int64_t(std::ceil(m.tau_abs[0]))+int64_t(plen);
    for(size_t k=1;k<m.n_paths;++k) {
        lo=std::min(lo,int64_t(std::floor(m.tau_abs[k])));
        hi=std::max(hi,int64_t(std::ceil(m.tau_abs[k]))+int64_t(plen));
    }
    // A complete observed packet is mandatory; truncation never silently clips.
    if(lo<0 || hi>int64_t(n)) {r.reason="context_truncated";r.channel_us=us(tc);return r;}
    r.sub_lo=std::max(lo,roi_lo);r.sub_hi=std::min(hi,roi_hi);
    if(r.sub_hi<=r.sub_lo) {r.reason="no_overlap";r.channel_us=us(tc);return r;}
    if(s.prefix_cache_enabled) {
        if(s.prefix_input_len!=prefix_input || s.prefix_output_len!=prefix_output ||
           s.prefix_key.size()<prefix_input || s.prefix_rows.size()<phases*prefix_output ||
           plen<prefix_input) {
            r.reason="prefix_cache_unprepared";r.channel_us=us(tc);return r;
        }
        r.prefix_cache_used=true;r.prefix_cached_samples=prefix_output;
        // Exact bytes include actual packet normalization and PHR. A new
        // length, amplitude or prefix waveform invalidates all phase rows.
        if(!s.prefix_key_valid ||
           std::memcmp(s.prefix_key.data(),s.waveform.data(),prefix_input*sizeof(cf))) {
            std::copy(s.waveform.begin(),s.waveform.begin()+prefix_input,s.prefix_key.begin());
            s.prefix_phase_mask=0;s.prefix_key_valid=true;r.prefix_key_changed=true;
        }
    }
    auto build_phase=[&](size_t g,cf* row) {
        const auto tp=Clock::now();
        ++r.phase_calls;
        if(g) {
            ++r.nonzero_phase_calls;r.reference_fir_output_samples+=plen;
        }
        if(!s.prefix_cache_enabled || !g) {
            phase_row(s.waveform.data(),plen,g,row,s.phase_tmp.data());
            if(g) r.fir_output_samples+=plen;
        } else {
            const uint32_t bit=uint32_t(1)<<g;
            cf* const cached=s.prefix_rows.data()+g*prefix_output;
            if(s.prefix_phase_mask&bit) {
                ++r.prefix_cache_hits;
            } else {
                phase_row_range(s.waveform.data(),plen,g,cached,s.phase_tmp.data(),0,prefix_output);
                s.prefix_phase_mask|=bit;
                ++r.prefix_cache_misses;r.fir_output_samples+=prefix_output;
            }
            std::copy(cached,cached+prefix_output,row);
            phase_row_range(s.waveform.data(),plen,g,row,s.phase_tmp.data(),prefix_output,plen);
            r.fir_output_samples+=plen-prefix_output;
        }
        r.phase_filter_us+=us(tp);
    };
    std::fill(s.accum.begin(),s.accum.begin()+n,cd(0));
    for(size_t k=0;k<m.n_paths;++k) {
        const int64_t ni=int64_t(std::floor(m.tau_abs[k]));
        const double beta=(m.tau_abs[k]-double(ni))*phases;
        const size_t g=std::min<size_t>(phases-1,size_t(beta));
        const double alpha=std::clamp(beta-double(g),0.0,1.0);
        build_phase(g,s.phase0.data());
        if(alpha!=0) build_phase((g+1)%phases,s.phase1.data());
        const auto ta=Clock::now();
        const cd gain=cd(m.gain[k])*r.waveform_scale;
        // Match preamble core's double row-AXPY accumulation ordering.
        for(int64_t j=lo;j<hi;++j) {
            const int64_t q=j-ni;
            if(q>=0 && q<int64_t(plen)) s.accum[j]+=cd(s.phase0[q])*gain*(1-alpha);
        }
        if(alpha!=0) for(int64_t j=lo;j<hi;++j) {
            const int64_t q=j-ni-(g+1==phases ? 1 : 0);
            if(q>=0 && q<int64_t(plen)) s.accum[j]+=cd(s.phase1[q])*gain*alpha;
        }
        r.path_accum_us+=us(ta);
    }
    const auto t_cfo=Clock::now();
    std::array<cf,rot_block> base, rotation;
    const double omega=2*pi*m.cfo_hz/m.sample_rate;
    const cd step=std::exp(cd(0,omega));cd cur=1;
    for(size_t j=0;j<rot_block;++j){base[j]=cf(cur);cur*=step;}
    for(int64_t j=lo;j<hi;) {
        const int64_t p=j-m.phase_reference_sample,b=floordiv(p,rot_block);
        const size_t j0=size_t(p-b*rot_block), count=std::min<size_t>(rot_block-j0,hi-j);
        const cf block(std::exp(cd(0,omega*double(b*int64_t(rot_block)))));
        // Keep the absolute 1024-sample phase grid; use the same dispatched
        // VOLK kernels as the preamble operator, without stateful float drift.
        // Stack storage and existing model scratch avoid hot-path allocation.
        volk_32fc_s32fc_multiply_32fc(rotation.data(),base.data()+j0,block,
                                    static_cast<unsigned>(count));
        volk_64f_convert_32f(reinterpret_cast<float*>(s.model.data()+j),
                            reinterpret_cast<const double*>(s.accum.data()+j),
                            static_cast<unsigned>(2*count));
        volk_32fc_x2_multiply_32fc(s.model.data()+j,s.model.data()+j,rotation.data(),
                                  static_cast<unsigned>(count));
        for(size_t z=0;z<count;++z) {
            if(!finite(s.model[j+z]) || !finite(y[j+z]-s.model[j+z])) {
                r.reason="nonfinite_reconstruction";r.cfo_check_us=us(t_cfo);r.channel_us=us(tc);return r;
            }
        }
        j+=count;
    }
    r.cfo_check_us=us(t_cfo);
    r.channel_us=us(tc);
    const auto ts=Clock::now();
    for(int64_t j=r.sub_lo;j<r.sub_hi;++j) out[j]=y[j]-s.model[j];
    if(ihat) std::copy(s.model.begin()+lo,s.model.begin()+hi,ihat+lo);
    r.subtract_us=us(ts);r.committed=true;r.reason="committed";return r;
}
}}}
