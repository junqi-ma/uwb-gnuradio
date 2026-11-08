/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * QA for the discrete equivalent CIR full-packet reconstruction.
 *
 * The oracle here is an INDEPENDENT double-precision direct convolution with
 * an absolute-phase CFO loop written from the header math, not a golden file
 * and not the estimator.  Packets are built locally from the public modulator
 * (or, where the modulator would hide the arithmetic, from a deterministic
 * synthetic waveform).  No sidecar and no injected truth is used.
 */
#include <boost/test/unit_test.hpp>
#include <gnuradio/uwb/uwb_discrete_cir_sic.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace gr::uwb;
using cf = std::complex<float>;
using cd = std::complex<double>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kWorkRateHz = 998.4e6;
constexpr size_t kSps = 1016;
constexpr size_t kSyncSamples = 1016; // one shaped SYNC on the work grid
constexpr float kRefScale = 0.5f;     // exactly representable -> exact template

// Independent reconstruction: y_hat[j] = exp(j*w*(j-r)) * sum_l h[l]*x[j-q-l]
// with the single waveform scale applied to x in double.  It walks the DENSE
// hull (inactive taps are zero), so it also checks that the active set matches
// the hull the model declares.
std::vector<cf> independent_reconstruction(const std::vector<cf>& wave,
                                           const sic::DiscreteCirModel& m,
                                           double scale,
                                           size_t n)
{
    std::vector<cf> ref(n, cf(0.f, 0.f));
    const int64_t lo = m.q + m.lag_lo;
    const int64_t hi = lo + int64_t(m.tap_count) + int64_t(wave.size());
    const double omega = 2.0 * kPi * m.cfo_hz / kWorkRateHz;
    for (int64_t j = lo; j < hi; ++j) {
        if (j < 0 || j >= int64_t(n))
            continue;
        cd acc(0.0, 0.0);
        for (size_t i = 0; i < m.tap_count; ++i) {
            const int64_t x = j - m.q - (m.lag_lo + int64_t(i));
            if (x < 0 || x >= int64_t(wave.size()))
                continue;
            acc += cd(m.taps[i]) * (cd(wave[static_cast<size_t>(x)]) * scale);
        }
        const cd rot =
            std::exp(cd(0.0, omega * static_cast<double>(j - m.phase_reference_sample)));
        ref[static_cast<size_t>(j)] = cf(acc * rot);
    }
    return ref;
}

sic::DiscreteCirModel make_model(int64_t q, int64_t lag_lo, size_t taps)
{
    sic::DiscreteCirModel m;
    m.valid = true;
    m.status = "committed";
    m.model_id = 0x5eed;
    m.q = q;
    m.phase_reference_sample = q;
    m.observed_end_sample = q + int64_t(8 * kSps);
    m.lag_lo = lag_lo;
    m.tap_count = taps;
    m.fit_residual = 0.01;
    m.gain_condition = 12.0;
    m.effective_rank = taps;
    m.support_source = "coarse_cir_peak_groups";
    return m;
}

// Active set in increasing lag order; every lag must lie inside the dense hull.
void set_active(sic::DiscreteCirModel& m,
                std::initializer_list<std::pair<int64_t, cf>> entries)
{
    m.active_count = 0;
    for (const auto& e : entries) {
        const int64_t idx = e.first - m.lag_lo;
        BOOST_REQUIRE_GE(idx, 0);
        BOOST_REQUIRE_LT(idx, int64_t(m.tap_count));
        BOOST_REQUIRE_LT(m.active_count, sic::kDiscreteCirMaxActive);
        m.active_lags[m.active_count++] = static_cast<int16_t>(e.first);
        m.taps[static_cast<size_t>(idx)] = e.second;
    }
}

size_t count_nonzero(const std::vector<cf>& v, int64_t lo, int64_t hi)
{
    size_t bad = 0;
    for (int64_t j = 0; j < int64_t(v.size()); ++j)
        if ((j < lo || j >= hi) && v[static_cast<size_t>(j)] != cf(0.f, 0.f))
            ++bad;
    return bad;
}

// A locally modulated packet.  `reference` is the public one-SYNC reference
// scaled by kRefScale, so the exact waveform scale is kRefScale.
struct Packet {
    mod::HrpModConfig profile;
    std::vector<uint8_t> psdu; // includes the FCS appended by the QA
    std::vector<uint8_t> phr;
    std::vector<cf> wave;

    Packet(size_t wire_len, uint32_t seed)
    {
        profile.code_index = 10;
        profile.sync_repetitions = 64;
        profile.sfd_mode = "4z2";
        profile.peak_amplitude = 0.8f;
        BOOST_REQUIRE_GT(wire_len, 2);
        psdu = mod::make_random_psdu(wire_len - 2, seed, true);
        BOOST_REQUIRE_EQUAL(psdu.size(), wire_len);
        std::vector<int8_t> bits;
        mod::detail::encode_phr19(psdu.size(), profile.sync_repetitions, false, bits);
        phr.assign(bits.begin(), bits.end());
        mod::HrpModScratch ms;
        const size_t cap =
            mod::packet_samples_998p4(64, 8, psdu.size(), false, &profile.pulse) + 64;
        ms.reserve(127, cap);
        wave.resize(cap);
        size_t produced = 0;
        BOOST_REQUIRE(mod::modulate_one(psdu.data(), psdu.size(), profile, ms,
                                        wave.data(), cap, produced));
        BOOST_REQUIRE_GT(produced, kSyncSamples);
        wave.resize(produced);
    }

    std::vector<cf> reference(float k = kRefScale) const
    {
        std::vector<cf> r(wave.begin(), wave.begin() + kSyncSamples);
        for (auto& v : r)
            v *= k;
        return r;
    }
};

// Apply-level rig with a synthetic (modulator-free) observation window.
struct ApplyRig {
    size_t n;
    int64_t q;
    sic::DiscreteCirModel model;
    std::vector<cf> wave, ref, y, out, ihat;
    sic::DiscreteCirScratch scratch;

    ApplyRig(size_t wave_n, int64_t origin)
        : n(wave_n + 4096), q(origin), model(make_model(origin, -1, 3))
    {
        wave.assign(wave_n, cf(0.8f, 0.f));
        ref.assign(wave.begin(), wave.begin() + kSyncSamples);
        for (auto& v : ref)
            v *= kRefScale;
        y.assign(n, cf(0.01f, 0.f));
        out.assign(n, cf(0.f, 0.f));
        ihat.assign(n, cf(0.f, 0.f));
        set_active(model, { { -1, cf(1.f, 0.f) },
                            { 0, cf(0.5f, 0.f) },
                            { 1, cf(0.25f, 0.f) } });
        reset_scratch();
    }

    void reset_scratch()
    {
        scratch.prepared = true;
        scratch.sps = kSps;
        scratch.max_window_samples = n;
        scratch.accumulator.assign(n, cf(0.f, 0.f));
        scratch.fir_tmp.assign(n, cf(0.f, 0.f));
        scratch.max_packet_samples = wave.size() + 64;
        scratch.scaled_waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));
    }

    bool run(sic::DiscreteCirResult& r)
    {
        return sic::apply_discrete_cir(y.data(), n, wave.data(), wave.size(),
                                       ref.data(), ref.size(), model, 0,
                                       int64_t(n), scratch, out.data(),
                                       ihat.data(), &r);
    }

    void expect_bypass() const
    {
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }
};

// Transactional rig: the observation is the remodulated packet through one
// integer-lag tap plus a CFO, so a committed run must actually cancel it.
struct TransactionRig {
    Packet pkt;
    size_t n;
    int64_t q;
    sic::DiscreteCirModel model;
    std::vector<cf> ref, y, out, ihat;
    sic::DiscreteCirScratch scratch;

    TransactionRig(size_t wire_len, uint32_t seed)
        : pkt(wire_len, seed),
          n(pkt.wave.size() + 4096),
          q(2048),
          model(make_model(2048, 3, 1))
    {
        set_active(model, { { 3, cf(0.5f, 0.25f) } });
        model.cfo_hz = 1500.0;
        ref = pkt.reference();
        y.assign(n, cf(0.002f, -0.001f));
        const double omega = 2.0 * kPi * model.cfo_hz / kWorkRateHz;
        for (size_t i = 0; i < pkt.wave.size(); ++i) {
            // CFO phase is anchored at r == q, so the observed packet sample at
            // output index q+3+i carries exp(j*omega*(3+i)).
            const double rel = 3.0 + double(i);
            const cf rot = cf(static_cast<float>(std::cos(omega * rel)),
                              static_cast<float>(std::sin(omega * rel)));
            // ihat = (kRefScale * wave) * h, h = 0.5 + 0.25j
            y[static_cast<size_t>(q + 3 + int64_t(i))] +=
                pkt.wave[i] * cf(kRefScale * 0.5f, kRefScale * 0.25f) * rot;
        }
        out.assign(n, cf(0.f, 0.f));
        ihat.assign(n, cf(0.f, 0.f));
        scratch.prepared = true;
        scratch.sps = kSps;
        scratch.max_window_samples = n;
        scratch.accumulator.assign(n, cf(0.f, 0.f));
        scratch.fir_tmp.assign(n, cf(0.f, 0.f));
        scratch.max_packet_samples = pkt.wave.size() + 64;
        scratch.waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));
        scratch.scaled_waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));
        scratch.expected_phr.reserve(19);
        scratch.modulator.reserve(127, scratch.max_packet_samples);
    }

    sic::DiscreteCirResult cancel(bool fcs = true,
                                  const mod::HrpModConfig* prof = nullptr,
                                  const std::vector<uint8_t>* phr = nullptr,
                                  const std::vector<uint8_t>* psdu = nullptr)
    {
        return sic::fullpacket_cancel_discrete(
            y.data(), n, phr ? *phr : pkt.phr, psdu ? *psdu : pkt.psdu, fcs,
            prof ? *prof : pkt.profile, model, ref.data(), ref.size(), 0,
            int64_t(n), scratch, out.data(), ihat.data());
    }

    void expect_bypass() const
    {
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }
};

} // namespace

BOOST_AUTO_TEST_CASE(discrete_apply_matches_independent_direct_convolution)
{
    Packet pkt(32, 11);
    const std::vector<cf> ref = pkt.reference();
    const int64_t q = 2048;
    const size_t n = pkt.wave.size() + 4096;

    for (double cfo : { 0.0, -50000.0, 50000.0 }) {
        sic::DiscreteCirModel m = make_model(q, -2, 10);
        m.cfo_hz = cfo;
        set_active(m, { { -2, cf(0.9f, -0.2f) },  // lowest lag, negative
                        { 0, cf(0.4f, 0.25f) },   // packet origin
                        { 7, cf(-0.15f, 0.1f) } }); // inside the hull, far right

        sic::DiscreteCirScratch scratch;
        scratch.prepared = true;
        scratch.sps = kSps;
        scratch.max_window_samples = n;
        scratch.accumulator.assign(n, cf(0.f, 0.f));
        scratch.fir_tmp.assign(n, cf(0.f, 0.f));
        scratch.max_packet_samples = pkt.wave.size() + 64;
        scratch.scaled_waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));

        std::vector<cf> y(n, cf(0.001f, -0.002f)), out(n), ihat(n);
        const double omega = 2.0 * kPi * cfo / kWorkRateHz;
        for (size_t i = 0; i < pkt.wave.size(); ++i) {
            const cf rot = cf(static_cast<float>(std::cos(omega * double(i))),
                              static_cast<float>(std::sin(omega * double(i))));
            y[static_cast<size_t>(q + int64_t(i))] += pkt.wave[i] * 0.5f * rot;
        }

        sic::DiscreteCirResult r;
        BOOST_REQUIRE_MESSAGE(
            sic::apply_discrete_cir(y.data(), n, pkt.wave.data(), pkt.wave.size(),
                                    ref.data(), ref.size(), m, 0, int64_t(n),
                                    scratch, out.data(), ihat.data(), &r),
            r.reason);
        BOOST_CHECK(r.committed);
        BOOST_CHECK_EQUAL(std::string(r.reason), "committed");
        BOOST_CHECK_CLOSE(r.waveform_scale, double(kRefScale), 1e-6);
        BOOST_CHECK_EQUAL(r.packet_samples, pkt.wave.size());
        BOOST_CHECK_EQUAL(r.model_id, m.model_id);
        BOOST_CHECK_EQUAL(r.q, q);
        BOOST_CHECK_EQUAL(r.tap_count, m.tap_count);
        BOOST_CHECK(r.scale_us >= 0.0);
        BOOST_CHECK(r.convolution_us >= 0.0);
        BOOST_CHECK(r.cfo_us >= 0.0);
        BOOST_CHECK(r.subtract_us >= 0.0);
        BOOST_CHECK_GE(r.channel_us, r.scale_us + r.convolution_us + r.cfo_us);
        BOOST_CHECK_GE(r.apply_total_us, r.channel_us + r.subtract_us);

        const std::vector<cf> want =
            independent_reconstruction(pkt.wave, m, double(kRefScale), n);
        const int64_t lo = q + m.lag_lo;
        const int64_t hi = lo + int64_t(m.tap_count) + int64_t(pkt.wave.size());
        BOOST_CHECK_EQUAL(r.support_lo, lo);
        BOOST_CHECK_EQUAL(r.support_hi, hi);
        BOOST_CHECK_EQUAL(r.sub_lo, lo);
        BOOST_CHECK_EQUAL(r.sub_hi, hi);

        float worst = 0.f;
        size_t leaks = 0, out_bad = 0, sub_bad = 0;
        for (int64_t j = 0; j < int64_t(n); ++j) {
            const size_t s = static_cast<size_t>(j);
            worst = std::max(worst, std::abs(ihat[s] - want[s]));
            if ((j < lo || j >= hi) && ihat[s] != cf(0.f, 0.f))
                ++leaks;
            if (j < r.sub_lo || j >= r.sub_hi) {
                if (!(out[s] == y[s]))
                    ++out_bad;
            } else if (std::abs(out[s] - (y[s] - ihat[s])) > 2e-6f) {
                ++sub_bad;
            }
        }
        // float FIR + float absolute-grid rotation vs a double oracle.  The
        // measured worst case over the three CFO cells is ~9e-8, so 1e-5 leaves
        // more than 100x margin for compiler/vectorization differences.
        BOOST_CHECK_LT(worst, 1e-5f);
        BOOST_CHECK_EQUAL(leaks, 0);
        BOOST_CHECK_EQUAL(out_bad, 0);
        BOOST_CHECK_EQUAL(sub_bad, 0);
    }
}

BOOST_AUTO_TEST_CASE(discrete_apply_tail_and_unaligned_offsets)
{
    // 1019 = 254 four-wide vectors + one two-wide + one scalar remainder.
    // Lags -1/0/1 shift src/dst off the 16-byte vector boundary, so this
    // checks the fused AXPY remainder path and unaligned loads/stores against
    // the independent double convolution.
    ApplyRig rig(1019, 2048);
    sic::DiscreteCirResult r;
    BOOST_REQUIRE_MESSAGE(rig.run(r), r.reason);
    BOOST_CHECK(r.committed);
    const std::vector<cf> want =
        independent_reconstruction(rig.wave, rig.model, double(kRefScale), rig.n);
    float worst = 0.f;
    for (size_t j = 0; j < rig.n; ++j)
        worst = std::max(worst, std::abs(rig.ihat[j] - want[j]));
    BOOST_CHECK_LT(worst, 1e-6f);
}

BOOST_AUTO_TEST_CASE(discrete_apply_truncation_roi_and_packet_tail)
{
    Packet pkt(24, 13);
    const std::vector<cf> ref = pkt.reference();
    const size_t n = pkt.wave.size() + 1024;
    std::vector<cf> y(n, cf(0.01f, 0.02f)), out(n), ihat(n);

    sic::DiscreteCirScratch scratch;
    scratch.prepared = true;
    scratch.sps = kSps;
    scratch.max_window_samples = n;
    scratch.accumulator.assign(n, cf(0.f, 0.f));
    scratch.fir_tmp.assign(n, cf(0.f, 0.f));
    scratch.max_packet_samples = pkt.wave.size() + 64;
    scratch.scaled_waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));

    // q near the end: a complete observed packet is mandatory, never clipped.
    {
        sic::DiscreteCirModel m = make_model(int64_t(n) - 512, -2, 10);
        set_active(m, { { 0, cf(0.5f, 0.f) } });
        sic::DiscreteCirResult r;
        BOOST_CHECK(!sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                             pkt.wave.size(), ref.data(),
                                             ref.size(), m, 0, int64_t(n),
                                             scratch, out.data(), ihat.data(), &r));
        BOOST_CHECK(!r.committed);
        BOOST_CHECK_EQUAL(std::string(r.reason), "context_truncated");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }

    // Negative lowest lag with a packet origin smaller than |lag_lo| would put
    // the support before sample 0: also a truncation, not a silent clip.
    {
        sic::DiscreteCirModel m = make_model(1, -8, 10);
        set_active(m, { { 0, cf(0.5f, 0.f) } });
        sic::DiscreteCirResult r;
        BOOST_CHECK(!sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                             pkt.wave.size(), ref.data(),
                                             ref.size(), m, 0, int64_t(n),
                                             scratch, out.data(), ihat.data(), &r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "context_truncated");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }

    // Consumer ROI that ends at the support start has no overlap.
    {
        sic::DiscreteCirModel m = make_model(64, -2, 10);
        set_active(m, { { 0, cf(0.5f, 0.f) } });
        const int64_t lo = m.q + m.lag_lo;
        sic::DiscreteCirResult r;
        BOOST_CHECK(!sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                             pkt.wave.size(), ref.data(),
                                             ref.size(), m, 0, lo,
                                             scratch, out.data(), ihat.data(), &r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "no_overlap");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }

    // A waveform shorter than the reference SYNC cannot be scaled and must not
    // read past its end.
    {
        sic::DiscreteCirModel m = make_model(64, -2, 10);
        set_active(m, { { 0, cf(0.5f, 0.f) } });
        sic::DiscreteCirResult r;
        BOOST_CHECK(!sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                             kSyncSamples - 1, ref.data(),
                                             ref.size(), m, 0, int64_t(n),
                                             scratch, out.data(), ihat.data(), &r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "invalid_buffers");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(), out.data(), n * sizeof(cf)), 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, 0, 0), 0);
    }

    // Packet tail exactly at the window end commits, and a narrower ROI still
    // subtracts only inside its own interval.
    {
        const int64_t wave_n = int64_t(pkt.wave.size());
        // hi = q + lag_lo + tap_count + wave_n = q - 2 + 10 + wave_n.
        const int64_t q = int64_t(n) - 8 - wave_n; // hi == n
        sic::DiscreteCirModel m = make_model(q, -2, 10);
        set_active(m, { { 0, cf(0.5f, 0.f) } });
        sic::DiscreteCirResult r;
        const int64_t roi_hi = int64_t(n) - 500;
        BOOST_REQUIRE_MESSAGE(sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                                      pkt.wave.size(), ref.data(),
                                                      ref.size(), m, 0, roi_hi,
                                                      scratch, out.data(), ihat.data(),
                                                      &r),
                              r.reason);
        BOOST_CHECK_EQUAL(r.support_hi, int64_t(n));
        BOOST_CHECK_EQUAL(r.sub_hi, roi_hi);
        BOOST_CHECK_LT(r.sub_hi, r.support_hi);
        size_t out_bad = 0;
        for (int64_t j = 0; j < int64_t(n); ++j)
            if ((j < r.sub_lo || j >= r.sub_hi) &&
                !(out[static_cast<size_t>(j)] == y[static_cast<size_t>(j)]))
                ++out_bad;
        BOOST_CHECK_EQUAL(out_bad, 0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat, r.support_lo, r.support_hi), 0);
    }
}

BOOST_AUTO_TEST_CASE(discrete_apply_scale_once_and_wrong_reference_bypass)
{
    Packet pkt(32, 17);
    const std::vector<cf> ref = pkt.reference();
    const size_t n = pkt.wave.size() + 2048;

    sic::DiscreteCirModel m = make_model(1024, -1, 5);
    set_active(m, { { -1, cf(0.7f, 0.1f) },
                    { 0, cf(0.2f, -0.05f) },
                    { 3, cf(0.05f, 0.f) } });
    m.cfo_hz = 12000.0;

    sic::DiscreteCirScratch scratch;
    scratch.prepared = true;
    scratch.sps = kSps;
    scratch.max_window_samples = n;
    scratch.accumulator.assign(n, cf(0.f, 0.f));
    scratch.fir_tmp.assign(n, cf(0.f, 0.f));
    scratch.max_packet_samples = pkt.wave.size() + 64;
    scratch.scaled_waveform.assign(scratch.max_packet_samples, cf(0.f, 0.f));

    std::vector<cf> y(n, cf(0.0f, 0.0f));
    std::vector<cf> out1(n), ihat1(n), out3(n), ihat3(n);

    sic::DiscreteCirResult r1;
    BOOST_REQUIRE_MESSAGE(sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                                  pkt.wave.size(), ref.data(),
                                                  ref.size(), m, 0, int64_t(n),
                                                  scratch, out1.data(), ihat1.data(),
                                                  &r1),
                          r1.reason);

    // Same packet observed three times louder: h is fitted against the public
    // template, so the recovered scale must drop by 3 and the reconstruction
    // must be unchanged (taps are not scaled a second time).
    std::vector<cf> wave3 = pkt.wave;
    for (auto& v : wave3)
        v *= 3.f;
    sic::DiscreteCirResult r3;
    BOOST_REQUIRE_MESSAGE(sic::apply_discrete_cir(y.data(), n, wave3.data(),
                                                  wave3.size(), ref.data(),
                                                  ref.size(), m, 0, int64_t(n),
                                                  scratch, out3.data(), ihat3.data(),
                                                  &r3),
                          r3.reason);
    BOOST_CHECK_CLOSE(r1.waveform_scale, 3.0 * r3.waveform_scale, 1e-3);

    float worst = 0.f, worst_out = 0.f;
    for (size_t j = 0; j < n; ++j) {
        worst = std::max(worst, std::abs(ihat1[j] - ihat3[j]));
        worst_out = std::max(worst_out, std::abs(out1[j] - out3[j]));
    }
    BOOST_CHECK_LT(worst, 1e-6f);
    // Same reconstruction, but the pre-scaled float waveform differs by its
    // last bits, so out matches to float rounding rather than bit-for-bit.
    BOOST_CHECK_LT(worst_out, 2e-6f);

    // A perturbed public reference is a template mismatch, never a silent
    // rescale of a different waveform.
    std::vector<cf> bad_ref = ref;
    bad_ref[80] += 0.25f;
    std::vector<cf> outb(n, cf(0.f, 0.f)), ihatb(n, cf(0.f, 0.f));
    sic::DiscreteCirResult rb;
    BOOST_CHECK(!sic::apply_discrete_cir(y.data(), n, pkt.wave.data(),
                                         pkt.wave.size(), bad_ref.data(),
                                         bad_ref.size(), m, 0, int64_t(n),
                                         scratch, outb.data(), ihatb.data(), &rb));
    BOOST_CHECK(!rb.committed);
    BOOST_CHECK_EQUAL(std::string(rb.reason), "template_scale_mismatch");
    BOOST_CHECK_EQUAL(std::memcmp(y.data(), outb.data(), n * sizeof(cf)), 0);
    BOOST_CHECK_EQUAL(count_nonzero(ihatb, 0, 0), 0);
}

BOOST_AUTO_TEST_CASE(discrete_transaction_validates_fcs_phr_profile_and_length)
{
    TransactionRig rig(32, 19);
    auto r = rig.cancel();
    BOOST_REQUIRE_MESSAGE(r.committed, r.reason);
    BOOST_CHECK_EQUAL(std::string(r.reason), "committed");
    BOOST_CHECK_CLOSE(r.waveform_scale, double(kRefScale), 1e-6);
    BOOST_CHECK_EQUAL(r.packet_samples, rig.pkt.wave.size());
    BOOST_CHECK_GT(r.validate_us, 0.0);
    BOOST_CHECK_GT(r.tx_us, 0.0);
    BOOST_CHECK_GE(r.apply_total_us, r.channel_us + r.subtract_us);

    // The committed transaction must actually cancel the matching observation
    // inside its ROI and leave the floor untouched elsewhere.
    float worst_residual = 0.f;
    size_t out_bad = 0;
    for (int64_t j = 0; j < int64_t(rig.n); ++j) {
        const size_t s = static_cast<size_t>(j);
        if (j < r.sub_lo || j >= r.sub_hi) {
            if (!(rig.out[s] == rig.y[s]))
                ++out_bad;
        } else if (j >= rig.q + 3 &&
                   j < rig.q + 3 + int64_t(rig.pkt.wave.size())) {
            worst_residual =
                std::max(worst_residual, std::abs(rig.out[s] - cf(0.002f, -0.001f)));
        }
    }
    BOOST_CHECK_EQUAL(out_bad, 0);
    // Measured ~3.4e-8: the observation term and the reconstruction differ only
    // by float rounding of the same product, so 1e-6 is a ~30x margin.
    BOOST_CHECK_LT(worst_residual, 1e-6f);
    BOOST_CHECK_EQUAL(count_nonzero(rig.ihat, r.support_lo, r.support_hi), 0);

    // The wrapper's remodulation capacity is the real packet buffer length; a
    // stale planning bound must not reject a packet that fits.
    {
        const size_t saved = rig.scratch.max_packet_samples;
        rig.scratch.max_packet_samples = 1024;
        auto again = rig.cancel();
        BOOST_REQUIRE_MESSAGE(again.committed, again.reason);
        BOOST_CHECK_EQUAL(again.packet_samples, rig.pkt.wave.size());
        rig.scratch.max_packet_samples = saved;
    }

    // FCS flag.
    BOOST_CHECK_EQUAL(std::string(rig.cancel(false).reason), "fcs_failed");
    rig.expect_bypass();

    // PHR bit flip and non-binary bit.
    {
        auto phr = rig.pkt.phr;
        phr[9] ^= 1;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, &phr).reason),
                          "unsupported_phr");
        rig.expect_bypass();
        phr[9] ^= 1;
        phr[9] = 2;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, &phr).reason),
                          "unsupported_phr");
        rig.expect_bypass();
    }

    // Unsupported profiles (same gate as the frozen full-packet core).
    {
        mod::HrpModConfig p = rig.pkt.profile;
        p.sfd_mode = "ieee";
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
        p = rig.pkt.profile;
        p.sync_repetitions = 16;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
        p = rig.pkt.profile;
        p.code_index = 9;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
        p = rig.pkt.profile;
        p.insert_sts = true;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
        p = rig.pkt.profile;
        p.ranging = true;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
        p = rig.pkt.profile;
        p.pulse.shape = mod::PulseShape::Gaussian;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, &p).reason),
                          "unsupported_profile");
        rig.expect_bypass();
    }

    // Wire length outside [2, 127].
    {
        std::vector<uint8_t> one(1, 0);
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, nullptr, &one).reason),
                          "invalid_frame_length");
        rig.expect_bypass();
        std::vector<uint8_t> big(128, 0);
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, nullptr, &big).reason),
                          "invalid_frame_length");
        rig.expect_bypass();
        // Correct length but a zeroed PHR: the encoder-derived PHR cannot match.
        std::vector<uint8_t> zero_phr(19, 0);
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, &zero_phr).reason),
                          "unsupported_phr");
        rig.expect_bypass();
    }

    // CRC mismatch inside a valid length.
    {
        auto psdu = rig.pkt.psdu;
        psdu[0] ^= 1;
        BOOST_CHECK_EQUAL(std::string(rig.cancel(true, nullptr, nullptr, &psdu).reason),
                          "fcs_failed");
        rig.expect_bypass();
    }

    // Model gate: anchor, active set and finiteness.
    {
        rig.model.phase_reference_sample = rig.model.q + 1;
        BOOST_CHECK_EQUAL(std::string(rig.cancel().reason), "invalid_model");
        rig.expect_bypass();
        rig.model.phase_reference_sample = rig.model.q;

        rig.model.active_count = 0;
        BOOST_CHECK_EQUAL(std::string(rig.cancel().reason), "invalid_model");
        rig.expect_bypass();
        rig.model.active_count = 1;

        const int16_t saved_lag = rig.model.active_lags[0];
        rig.model.active_lags[0] = static_cast<int16_t>(rig.model.lag_lo + 1);
        BOOST_CHECK_EQUAL(std::string(rig.cancel().reason), "invalid_model");
        rig.expect_bypass();
        rig.model.active_lags[0] = saved_lag;

        rig.model.taps[0] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
        BOOST_CHECK_EQUAL(std::string(rig.cancel().reason), "invalid_model");
        rig.expect_bypass();
        rig.model.taps[0] = cf(0.5f, 0.25f);

        rig.model.valid = false;
        BOOST_CHECK_EQUAL(std::string(rig.cancel().reason), "invalid_model");
        rig.expect_bypass();
        rig.model.valid = true;
    }

    // ROI and template gates.
    {
        BOOST_CHECK_EQUAL(std::string(sic::fullpacket_cancel_discrete(
                                          rig.y.data(), rig.n, rig.pkt.phr, rig.pkt.psdu,
                                          true, rig.pkt.profile, rig.model, rig.ref.data(),
                                          rig.ref.size(), 10, 10, rig.scratch,
                                          rig.out.data(), rig.ihat.data())
                                          .reason),
                          "invalid_roi");
        rig.expect_bypass();
        BOOST_CHECK_EQUAL(std::string(sic::fullpacket_cancel_discrete(
                                          rig.y.data(), rig.n, rig.pkt.phr, rig.pkt.psdu,
                                          true, rig.pkt.profile, rig.model, rig.ref.data(),
                                          rig.ref.size() - 1, 0, int64_t(rig.n),
                                          rig.scratch, rig.out.data(), rig.ihat.data())
                                          .reason),
                          "invalid_template");
        rig.expect_bypass();
    }
}

BOOST_AUTO_TEST_CASE(discrete_transaction_length_edges_16_and_127)
{
    // Shortest and longest accepted wire PSDU.  The FCS is appended by the QA
    // and the PHR derives from the same length, so a 16 B PHR must not validate
    // the 127 B frame.
    Packet small(16, 23);
    TransactionRig short_rig(16, 23);
    auto r16 = short_rig.cancel();
    BOOST_REQUIRE_MESSAGE(r16.committed, r16.reason);
    BOOST_CHECK_EQUAL(r16.packet_samples, short_rig.pkt.wave.size());

    TransactionRig long_rig(127, 29);
    auto r127 = long_rig.cancel();
    BOOST_REQUIRE_MESSAGE(r127.committed, r127.reason);
    BOOST_CHECK_EQUAL(r127.packet_samples, long_rig.pkt.wave.size());
    BOOST_CHECK_GT(r127.packet_samples, r16.packet_samples);

    // Same wire length, but the PHR of the other frame length.
    auto cross = long_rig.cancel(true, nullptr, &small.phr);
    BOOST_CHECK(!cross.committed);
    BOOST_CHECK_EQUAL(std::string(cross.reason), "unsupported_phr");
    long_rig.expect_bypass();
}

BOOST_AUTO_TEST_CASE(sparse_pulse_apply_matches_approximate_cfo_reference)
{
    auto wire=mod::make_random_psdu(30,909,true);
    mod::HrpModConfig unit;
    unit.code_index=10; unit.sync_repetitions=64; unit.sfd_mode="4z2";
    unit.peak_amplitude=1.f;
    unit.pulse.shape=mod::PulseShape::External;
    unit.pulse.external_n_taps=2; unit.external_taps={1.f,0.f};
    mod::HrpModScratch ms;
    const size_t cap=300000;
    ms.reserve(127,cap);
    mod::PulseList pl;
    BOOST_REQUIRE(mod::modulate_pulse_list(wire.data(),wire.size(),unit,ms,cap,pl));
    std::vector<mod::PulseEvent> events(pl.events,pl.events+pl.count);
    pl.events=events.data();

    const int64_t q=2048;
    const size_t n=pl.packet_samples+4096;
    const int8_t* code=demod::GetPreambleCode(10);
    for (double cfo : {0.0,-50000.0,50000.0}) {
        sic::DiscreteCirModel m=make_model(q,-2,10);
        m.impulse_template=true;
        m.cfo_hz=cfo;
        set_active(m,{{-2,cf(0.9f,-0.2f)},{0,cf(0.4f,0.25f)},
                      {7,cf(-0.15f,0.1f)}});
        sic::DiscreteCirScratch s;
        s.prepared=true; s.sps=kSps; s.max_window_samples=n;
        s.ipatov_correlation=true; s.sparse_pulse_reconstruct=true;
        s.max_packet_samples=pl.packet_samples+64;
        s.waveform.assign(s.max_packet_samples,cf{});
        s.scaled_waveform.assign(s.max_packet_samples,cf{});
        s.accumulator.assign(n,cf{});
        s.fir_tmp.assign(n,cf{});
        s.impulse_sync.assign(kSps,cf{});
        for (size_t c=0;c<demod::kQm35CodeLength;++c)
            s.impulse_sync[8*c]=cf(float(code[c]),0.f);

        std::vector<cf> y(n,cf(0.01f,-0.02f)),out(n),ihat(n);
        sic::DiscreteCirResult r;
        BOOST_REQUIRE_MESSAGE(sic::apply_discrete_cir_pulses(
                                  y.data(),n,pl,s.impulse_sync.data(),
                                  s.impulse_sync.size(),m,0,int64_t(n),s,
                                  out.data(),ihat.data(),&r),
                              r.reason);
        BOOST_CHECK(r.committed);
        BOOST_CHECK_EQUAL(std::string(r.reason),"committed");
        BOOST_CHECK_EQUAL(r.cfo_us,0.0);
        BOOST_CHECK_EQUAL(r.pulse_event_count,pl.count);
        BOOST_CHECK_EQUAL(r.pulse_packet_samples,pl.packet_samples);

        // Independent double scalar implementation of the approximate form
        // c_i = sign_i*exp(j*w*(q+m_i-anchor)); yhat[q+m_i+lag] += c_i*taps.
        std::vector<cd> want(n,cd{});
        const double omega=2.0*kPi*cfo/kWorkRateHz;
        for (const auto& e : events) {
            const cd c=double(e.sign)*std::polar(
                1.0,omega*double(q+int64_t(e.sample_offset)-
                                 m.phase_reference_sample));
            for (size_t d=0;d<m.tap_count;++d) {
                const int64_t j=q+int64_t(e.sample_offset)+m.lag_lo+
                               int64_t(d);
                if (j>=0 && j<int64_t(n))
                    want[static_cast<size_t>(j)] += c*cd(m.taps[d]);
            }
        }
        float worst=0.f;
        for (size_t j=0;j<n;++j)
            worst=std::max(worst,std::abs(ihat[j]-cf(want[j])));
        BOOST_CHECK_LT(worst,2e-5f);
        const int64_t lo=q+m.lag_lo;
        const int64_t hi=lo+int64_t(m.tap_count)+int64_t(pl.packet_samples);
        for (int64_t j=0;j<int64_t(n);++j) {
            if (j<lo || j>=hi) {
                BOOST_CHECK_EQUAL(ihat[static_cast<size_t>(j)],cf{});
            }
            if (j<lo || j>=hi)
                BOOST_CHECK_EQUAL(out[static_cast<size_t>(j)],y[static_cast<size_t>(j)]);
            else
                BOOST_CHECK_LT(std::abs(out[static_cast<size_t>(j)]-
                                        (y[static_cast<size_t>(j)]-
                                         ihat[static_cast<size_t>(j)])),1e-6f);
        }

        // Optional ihat: the same reconstruction must be produced and the
        // support flanks outside the consumer ROI must still equal y.
        std::vector<cf> out_only(n);
        sic::DiscreteCirResult rn;
        BOOST_REQUIRE_MESSAGE(sic::apply_discrete_cir_pulses(
                                  y.data(),n,pl,s.impulse_sync.data(),
                                  s.impulse_sync.size(),m,0,int64_t(n),s,
                                  out_only.data(),nullptr,&rn),
                              rn.reason);
        BOOST_CHECK(rn.committed);
        for (int64_t j=0;j<int64_t(n);++j) {
            const size_t u=static_cast<size_t>(j);
            if (j<lo || j>=hi)
                BOOST_CHECK_EQUAL(out_only[u],y[u]);
            else
                BOOST_CHECK_LT(std::abs(out_only[u]-
                                        (y[u]-ihat[u])),1e-6f);
        }
    }

    // Representation failures must bypass transactionally.
    {
        sic::DiscreteCirModel m=make_model(q,-2,10);
        m.impulse_template=true;
        set_active(m,{{0,cf(0.5f,0.0f)}});
        sic::DiscreteCirScratch s;
        s.prepared=true; s.sps=kSps; s.max_window_samples=n;
        s.ipatov_correlation=true; s.sparse_pulse_reconstruct=true;
        s.impulse_sync.assign(kSps,cf{});
        for (size_t c=0;c<demod::kQm35CodeLength;++c)
            s.impulse_sync[8*c]=cf(float(code[c]),0.f);
        std::vector<cf> y(n,cf(0.02f,0.01f)),out(n),ihat(n);
        auto check_fail=[&](const mod::PulseList& bad,const char* reason){
            sic::DiscreteCirResult r;
            BOOST_CHECK(!sic::apply_discrete_cir_pulses(
                y.data(),n,bad,s.impulse_sync.data(),s.impulse_sync.size(),
                m,0,int64_t(n),s,out.data(),ihat.data(),&r));
            BOOST_CHECK_EQUAL(std::string(r.reason),reason);
            BOOST_CHECK_EQUAL(std::memcmp(y.data(),out.data(),n*sizeof(cf)),0);
            BOOST_CHECK_EQUAL(count_nonzero(ihat,0,0),0);
        };
        auto duplicate=events;
        if (duplicate.size()>1) duplicate[1].sample_offset=duplicate[0].sample_offset;
        mod::PulseList bad{duplicate.data(),duplicate.size(),pl.packet_samples};
        check_fail(bad,"invalid_pulse_list");

        auto wrong_sync=events;
        wrong_sync[0].sign=static_cast<int8_t>(-wrong_sync[0].sign);
        mod::PulseList bad_sync{wrong_sync.data(),wrong_sync.size(),
                                pl.packet_samples};
        check_fail(bad_sync,"pulse_template_mismatch");

        auto far=m;
        far.q=int64_t(n)-1;
        far.phase_reference_sample=far.q;
        sic::DiscreteCirResult rf;
        BOOST_CHECK(!sic::apply_discrete_cir_pulses(
            y.data(),n,pl,s.impulse_sync.data(),s.impulse_sync.size(),far,
            0,int64_t(n),s,out.data(),ihat.data(),&rf));
        BOOST_CHECK_EQUAL(std::string(rf.reason),"context_truncated");
        BOOST_CHECK_EQUAL(std::memcmp(y.data(),out.data(),n*sizeof(cf)),0);
        BOOST_CHECK_EQUAL(count_nonzero(ihat,0,0),0);
    }
}

BOOST_AUTO_TEST_CASE(discrete_numeric_safety_and_scratch_capacity_bypass)
{
    ApplyRig rig(4096, 1024);
    sic::DiscreteCirResult r;

    // Sanity: the unmodified synthetic rig commits, so the bypass checks below
    // are not vacuous.
    BOOST_REQUIRE_MESSAGE(rig.run(r), r.reason);
    BOOST_CHECK_EQUAL(r.reason, "committed");

    // max_packet_samples is the estimator's planning bound only: a zero or
    // stale value must not reject a waveform that fits the real buffers.
    {
        const std::vector<cf> reference_out = rig.out;
        const size_t saved = rig.scratch.max_packet_samples;
        rig.scratch.max_packet_samples = 0;
        BOOST_REQUIRE_MESSAGE(rig.run(r), r.reason);
        BOOST_CHECK_EQUAL(std::string(r.reason), "committed");
        BOOST_CHECK_EQUAL(std::memcmp(rig.out.data(), reference_out.data(),
                                      rig.n * sizeof(cf)),
                          0);
        rig.scratch.max_packet_samples = 1024; // far below the packet length
        BOOST_REQUIRE_MESSAGE(rig.run(r), r.reason);
        BOOST_CHECK_EQUAL(std::memcmp(rig.out.data(), reference_out.data(),
                                      rig.n * sizeof(cf)),
                          0);
        rig.scratch.max_packet_samples = saved;
    }
    // Committed reference image; the failure cases below intentionally reset
    // out to y, so keep a copy instead of re-reading rig.out.
    const std::vector<cf> committed_out = rig.out;

    // NaN in the observation.
    {
        std::vector<cf> saved = rig.y;
        rig.y[3] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "nonfinite_input");
        rig.expect_bypass();
        rig.y = saved;
    }

    // Non-finite waveform outside the reference prefix (the scale pass only
    // sees the first SYNC, so this exercises the full pre-scale check).
    {
        std::vector<cf> saved = rig.wave;
        rig.wave.back() = cf(std::numeric_limits<float>::infinity(), 0.f);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "nonfinite_input");
        rig.expect_bypass();
        rig.wave = saved;
    }

    // Non-finite taps are a model failure, not a silent truncation.
    {
        rig.model.taps[2] = cf(std::numeric_limits<float>::quiet_NaN(), 0.f);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "invalid_model");
        rig.expect_bypass();
        rig.model.taps[2] = cf(0.25f, 0.f);
    }

    // Finite taps whose accumulated reconstruction overflows float.
    {
        const float huge = std::numeric_limits<float>::max();
        rig.model.taps[0] = cf(huge, 0.f);
        rig.model.taps[1] = cf(huge, 0.f);
        rig.model.taps[2] = cf(huge, 0.f);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "nonfinite_reconstruction");
        rig.expect_bypass();
        rig.model.taps[0] = cf(1.f, 0.f);
        rig.model.taps[1] = cf(0.5f, 0.f);
        rig.model.taps[2] = cf(0.25f, 0.f);
    }

    // Optional ihat: the same reconstruction must still be produced, out must
    // still equal y outside the subtraction interval, and a failure must still
    // restore out exactly.
    {
        std::vector<cf> out_only(rig.n, cf(0.f, 0.f));
        sic::DiscreteCirResult rn;
        BOOST_REQUIRE(sic::apply_discrete_cir(rig.y.data(), rig.n, rig.wave.data(),
                                              rig.wave.size(), rig.ref.data(),
                                              rig.ref.size(), rig.model, 0,
                                              int64_t(rig.n), rig.scratch,
                                              out_only.data(), nullptr, &rn));
        BOOST_CHECK(rn.committed);
        BOOST_CHECK_EQUAL(std::memcmp(out_only.data(), committed_out.data(),
                                      rig.n * sizeof(cf)),
                          0);
        // With ihat == nullptr the trial reconstruction lives in `out`, so the
        // flanks of the support outside the ROI must be restored to y.
        size_t flank_bad = 0;
        for (int64_t j = rn.support_lo; j < rn.sub_lo; ++j)
            if (!(out_only[static_cast<size_t>(j)] == rig.y[static_cast<size_t>(j)]))
                ++flank_bad;
        for (int64_t j = rn.sub_hi; j < rn.support_hi; ++j)
            if (!(out_only[static_cast<size_t>(j)] == rig.y[static_cast<size_t>(j)]))
                ++flank_bad;
        BOOST_CHECK_EQUAL(flank_bad, 0);

        rig.model.taps[0] = cf(std::numeric_limits<float>::max(), 0.f);
        rig.model.taps[1] = cf(std::numeric_limits<float>::max(), 0.f);
        rig.model.taps[2] = cf(std::numeric_limits<float>::max(), 0.f);
        std::vector<cf> out_fail(rig.n, cf(0.f, 0.f));
        BOOST_CHECK(!sic::apply_discrete_cir(rig.y.data(), rig.n, rig.wave.data(),
                                             rig.wave.size(), rig.ref.data(),
                                             rig.ref.size(), rig.model, 0,
                                             int64_t(rig.n), rig.scratch,
                                             out_fail.data(), nullptr, &rn));
        BOOST_CHECK_EQUAL(std::string(rn.reason), "nonfinite_reconstruction");
        BOOST_CHECK_EQUAL(std::memcmp(rig.y.data(), out_fail.data(),
                                      rig.n * sizeof(cf)),
                          0);
        rig.model.taps[0] = cf(1.f, 0.f);
        rig.model.taps[1] = cf(0.5f, 0.f);
        rig.model.taps[2] = cf(0.25f, 0.f);
    }

    // Scratch capacity contract.
    {
        rig.scratch.prepared = false;
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "scratch_unprepared");
        rig.expect_bypass();
        rig.scratch.prepared = true;

        const size_t saved_window = rig.scratch.max_window_samples;
        rig.scratch.max_window_samples = rig.n - 1;
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "scratch_capacity");
        rig.expect_bypass();
        rig.scratch.max_window_samples = saved_window;

        rig.scratch.scaled_waveform.resize(1024);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "scratch_capacity");
        rig.expect_bypass();
        rig.scratch.scaled_waveform.resize(rig.scratch.max_packet_samples);

        // The tap-major FIR accumulates in scratch.accumulator, which must
        // cover the whole window.
        rig.scratch.accumulator.resize(rig.n - 1);
        BOOST_CHECK(!rig.run(r));
        BOOST_CHECK_EQUAL(std::string(r.reason), "scratch_capacity");
        rig.expect_bypass();
        rig.scratch.accumulator.assign(rig.n, cf(0.f, 0.f));
    }

    // Template gate still holds for the synthetic rig.
    {
        std::vector<cf> bad = rig.ref;
        bad.back() += 0.5f;
        sic::DiscreteCirResult rt;
        BOOST_CHECK(!sic::apply_discrete_cir(rig.y.data(), rig.n, rig.wave.data(),
                                             rig.wave.size(), bad.data(), bad.size(),
                                             rig.model, 0, int64_t(rig.n),
                                             rig.scratch, rig.out.data(),
                                             rig.ihat.data(), &rt));
        BOOST_CHECK_EQUAL(std::string(rt.reason), "template_scale_mismatch");
        rig.expect_bypass();
    }
}

BOOST_AUTO_TEST_CASE(b1_avoid_out_double_write_and_ihat_semantics)
{
    ApplyRig rig(4096, 1024);
    const int64_t roi_lo = 1500;
    const int64_t roi_hi = 3000;
    sic::DiscreteCirResult r;
    BOOST_REQUIRE(sic::apply_discrete_cir(
        rig.y.data(), rig.n, rig.wave.data(), rig.wave.size(),
        rig.ref.data(), rig.ref.size(), rig.model, roi_lo, roi_hi,
        rig.scratch, rig.out.data(), rig.ihat.data(), &r));
    BOOST_CHECK(r.committed);
    BOOST_CHECK_EQUAL(r.sub_lo, roi_lo);
    BOOST_CHECK_EQUAL(r.sub_hi, roi_hi);

    // 1. Outside [sub_lo, sub_hi), out must be byte-identical to y.
    BOOST_CHECK_EQUAL(std::memcmp(rig.out.data(), rig.y.data(), roi_lo * sizeof(cf)), 0);
    BOOST_CHECK_EQUAL(std::memcmp(rig.out.data() + roi_hi, rig.y.data() + roi_hi,
                                  (rig.n - roi_hi) * sizeof(cf)), 0);

    // 2. Inside [sub_lo, sub_hi), out == y - ihat.
    for (int64_t j = roi_lo; j < roi_hi; ++j) {
        BOOST_CHECK_EQUAL(rig.out[j], rig.y[j] - rig.ihat[j]);
    }

    // 3. ihat must be strictly zero outside [support_lo, support_hi).
    for (int64_t j = 0; j < r.support_lo; ++j) {
        BOOST_CHECK_EQUAL(rig.ihat[j], cf(0.f, 0.f));
    }
    for (int64_t j = r.support_hi; j < int64_t(rig.n); ++j) {
        BOOST_CHECK_EQUAL(rig.ihat[j], cf(0.f, 0.f));
    }

    // 4. In-place y == out test:
    std::vector<cf> in_place = rig.y;
    sic::DiscreteCirResult r_ip;
    BOOST_REQUIRE(sic::apply_discrete_cir(
        in_place.data(), rig.n, rig.wave.data(), rig.wave.size(),
        rig.ref.data(), rig.ref.size(), rig.model, roi_lo, roi_hi,
        rig.scratch, in_place.data(), nullptr, &r_ip));
    BOOST_CHECK(r_ip.committed);
    BOOST_CHECK_EQUAL(std::memcmp(in_place.data(), rig.out.data(), rig.n * sizeof(cf)), 0);

    // 5. Failure restore test:
    std::vector<cf> out_fail(rig.n, cf(999.f, -999.f));
    std::vector<cf> ihat_fail(rig.n, cf(999.f, -999.f));
    rig.model.cfo_hz = 998.4e6; // beyond Nyquist -> invalid_model
    BOOST_CHECK(!sic::apply_discrete_cir(
        rig.y.data(), rig.n, rig.wave.data(), rig.wave.size(),
        rig.ref.data(), rig.ref.size(), rig.model, roi_lo, roi_hi,
        rig.scratch, out_fail.data(), ihat_fail.data(), &r));
    BOOST_CHECK(!r.committed);
    BOOST_CHECK_EQUAL(std::memcmp(out_fail.data(), rig.y.data(), rig.n * sizeof(cf)), 0);
    BOOST_CHECK_EQUAL(count_nonzero(ihat_fail, 0, 0), 0);
}

BOOST_AUTO_TEST_CASE(b2_b3_sparse_pulse_avx2_fma_20tap_and_fallback_parity)
{
    auto wire = mod::make_random_psdu(30, 909, true);
    mod::HrpModConfig unit;
    unit.code_index = 10;
    unit.sync_repetitions = 64;
    unit.sfd_mode = "4z2";
    unit.peak_amplitude = 1.f;
    unit.pulse.shape = mod::PulseShape::External;
    unit.pulse.external_n_taps = 2;
    unit.external_taps = { 1.f, 0.f };
    mod::HrpModScratch ms;
    const size_t cap = 300000;
    ms.reserve(127, cap);
    mod::PulseList pl;
    BOOST_REQUIRE(mod::modulate_pulse_list(wire.data(), wire.size(), unit, ms, cap, pl));
    std::vector<mod::PulseEvent> events(pl.events, pl.events + pl.count);
    pl.events = events.data();

    const int64_t q = 2048;
    const size_t n = pl.packet_samples + 4096;
    const int8_t* code = demod::GetPreambleCode(10);

    sic::DiscreteCirScratch s;
    s.reserve(n);
    s.prepared = true;
    s.sps = kSps;
    s.ipatov_correlation = true;
    s.sparse_pulse_reconstruct = true;
    s.impulse_sync.assign(kSps, cf{});
    for (size_t c = 0; c < demod::kQm35CodeLength; ++c)
        s.impulse_sync[8 * c] = cf(float(code[c]), 0.f);

    std::vector<cf> y(n, cf(0.01f, -0.02f));

    // Test both 20-tap (B3 fast path) and 19-tap (generic fallback)
    for (size_t tap_count : { 20UL, 19UL }) {
        for (double cfo : { 0.0, 35000.0 }) {
            sic::DiscreteCirModel m = make_model(q, -1, tap_count);
            m.impulse_template = true;
            m.cfo_hz = cfo;
            for (size_t t = 0; t < tap_count; ++t) {
                m.taps[t] = cf(0.1f * float(t + 1), -0.05f * float(t));
            }
            m.active_count = tap_count;
            for (size_t t = 0; t < tap_count; ++t) {
                m.active_lags[t] = static_cast<int16_t>(m.lag_lo + int64_t(t));
            }

            // Run with B3 (default)
            ::unsetenv("UWB_SIC_DISABLE_B3");
            std::vector<cf> out_opt(n), ihat_opt(n);
            sic::DiscreteCirResult r_opt;
            BOOST_REQUIRE(sic::apply_discrete_cir_pulses(
                y.data(), n, pl, s.impulse_sync.data(), s.impulse_sync.size(),
                m, 0, int64_t(n), s, out_opt.data(), ihat_opt.data(), &r_opt));
            BOOST_CHECK(r_opt.committed);

            // Run with B3 disabled (generic fallback path)
            ::setenv("UWB_SIC_DISABLE_B3", "1", 1);
            std::vector<cf> out_fallback(n), ihat_fallback(n);
            sic::DiscreteCirResult r_fallback;
            BOOST_REQUIRE(sic::apply_discrete_cir_pulses(
                y.data(), n, pl, s.impulse_sync.data(), s.impulse_sync.size(),
                m, 0, int64_t(n), s, out_fallback.data(), ihat_fallback.data(), &r_fallback));
            BOOST_CHECK(r_fallback.committed);
            ::unsetenv("UWB_SIC_DISABLE_B3");

            if (cfo == 0.0) {
                // Zero CFO must be bit-for-bit identical between AVX2 and fallback!
                BOOST_CHECK_EQUAL(std::memcmp(out_opt.data(), out_fallback.data(), n * sizeof(cf)), 0);
                BOOST_CHECK_EQUAL(std::memcmp(ihat_opt.data(), ihat_fallback.data(), n * sizeof(cf)), 0);
            } else {
                // Non-zero CFO uses FMA vs non-FMA, should agree within float numerical precision
                float max_err = 0.f;
                for (size_t j = 0; j < n; ++j) {
                    max_err = std::max(max_err, std::abs(out_opt[j] - out_fallback[j]));
                }
                BOOST_CHECK_LT(max_err, 1e-5f);
            }
        }
    }
}
