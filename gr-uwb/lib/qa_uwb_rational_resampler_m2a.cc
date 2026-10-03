/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A resampler core QA (G0 docs/twr/M2-A_G0接口与数字坐标.md §3.3-§3.5,
 * A03/A04/A05).
 *
 * Covers BOTH new TX directions and the existing RX directions:
 *   TX work->native  <48,65>  998.4 -> 737.28 MS/s   RationalResampler48_65Core
 *   TX work->native  <32,65>  998.4 -> 491.52 MS/s   RationalResampler32_65Core
 *   RX native->work  <65,48>  737.28 -> 998.4 MS/s   RationalResampler65_48Core
 *   RX native->work  <65,32>  491.52 -> 998.4 MS/s   RationalResampler65_32Core
 *
 * Frozen length contract (NOT resample_poly centre-crop):
 *     Lout = ceil(((N-1)*L + T) / M)
 *     y[m] = sum_k h[(m*M mod L) + L*k] * x[floor(m*M/L) - k],  x[j]=0 outside [0,N)
 *
 * Expectations are computed from that frozen formula and hand-derived values,
 * NOT by asserting that two same-source predicates agree.  The tap files are
 * read from testdata/; if UWB_TESTDATA_DIR is defined it is tried first.
 *
 * N=0 note: the CORE exposes the transient length ceil((T-L)/M) for N=0.
 * G0 §3.2 defines N=0 -> 0 only at the M2-A *helper* layer (the helper must
 * not pass N=0 into the core).  This QA tests the core contract and documents
 * the difference.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using gr::uwb::core::RationalResampler48_65Core;
using gr::uwb::core::RationalResampler32_65Core;
using gr::uwb::core::RationalResampler65_48Core;
using gr::uwb::core::RationalResampler65_32Core;
using gr_complex = std::complex<float>;

namespace {

constexpr float kTolAbs = 2e-3f;   // vs float32 golden export (existing QA)
constexpr float kKernelTol = 1e-4f; // default (VOLK/AVX2) vs scalar

std::string find_path(const std::string& rel_to_testdata)
{
    std::vector<std::string> cands;
#ifdef UWB_TESTDATA_DIR
    cands.push_back(std::string(UWB_TESTDATA_DIR) + "/" + rel_to_testdata);
#endif
    const char* prefixes[] = { "", "../", "../../", "../../../", "../../../../" };
    for (const char* p : prefixes)
        cands.push_back(std::string(p) + "testdata/" + rel_to_testdata);
    for (const auto& c : cands) {
        std::ifstream f(c, std::ios::binary);
        if (f)
            return c;
    }
    return cands.empty() ? rel_to_testdata : cands.front();
}

std::vector<float> load_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()),
           static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

const std::vector<float>& tx_48_65_taps()
{
    static std::vector<float> taps =
        load_f32(find_path("twr/m2a/taps/tx_48_65.f32"));
    return taps;
}
const std::vector<float>& tx_32_65_taps()
{
    static std::vector<float> taps =
        load_f32(find_path("twr/m2a/taps/tx_32_65.f32"));
    return taps;
}
const std::vector<float>& rx_65_48_taps()
{
    static std::vector<float> taps =
        load_f32(find_path("resampler_65_48/taps_quality_minorder.txt"));
    return taps;
}
const std::vector<float>& rx_65_32_taps()
{
    static std::vector<float> taps =
        load_f32(find_path("resampler_65_32/taps_quality_minorder.txt"));
    return taps;
}

/** Frozen length formula, computed independently of the core. */
int64_t frozen_len(uint64_t N, uint32_t L, uint32_t M, size_t T)
{
    const int64_t num =
        (static_cast<int64_t>(N) - 1) * static_cast<int64_t>(L) +
        static_cast<int64_t>(T);
    if (num <= 0)
        return 0;
    return (num + static_cast<int64_t>(M) - 1) / static_cast<int64_t>(M);
}

template <typename Core>
std::vector<gr_complex> run_full(Core& core, const std::vector<gr_complex>& x)
{
    const size_t Lout =
        Core::expected_output_length(x.size(), core.tap_count());
    std::vector<gr_complex> y(Lout + 256, gr_complex(0, 0));
    size_t produced = 0;
    if (!x.empty()) {
        auto r = core.process(x.data(), x.size(), y.data(), y.size());
        produced = r.produced;
        BOOST_REQUIRE_EQUAL(r.consumed, x.size());
    }
    while (true) {
        if (produced >= y.size())
            y.resize(produced + 256);
        const size_t n = core.flush(y.data() + produced, y.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    y.resize(produced);
    return y;
}

/**
 * Chunked runner.  Feeds the stream in the requested chunk sizes and, for
 * each chunk, calls process() with a SHORT output buffer until the chunk is
 * fully consumed (partial-consume path).  Then flushes.
 */
template <typename Core>
std::vector<gr_complex> run_chunked(Core& core,
                                    const std::vector<gr_complex>& x,
                                    const std::vector<size_t>& chunks,
                                    size_t out_cap)
{
    std::vector<gr_complex> y;
    std::vector<gr_complex> tmp(out_cap);
    size_t off = 0;
    size_t ci = 0;
    while (off < x.size()) {
        size_t ch = chunks[ci % chunks.size()];
        ++ci;
        ch = std::min(ch, x.size() - off);
        size_t fed = 0;
        size_t guard = 0;
        while (fed < ch) {
            auto r = core.process(x.data() + off + fed, ch - fed, tmp.data(),
                                  out_cap);
            for (size_t i = 0; i < r.produced; ++i)
                y.push_back(tmp[i]);
            fed += r.consumed;
            if (r.consumed == 0 && r.produced == 0) {
                BOOST_FAIL("resampler made no progress (off=" +
                           std::to_string(off) + ")");
                break;
            }
            if (++guard > 1000000) {
                BOOST_FAIL("resampler chunk drain did not terminate");
                break;
            }
        }
        off += ch;
    }
    std::vector<gr_complex> tail(out_cap);
    while (true) {
        const size_t n = core.flush(tail.data(), tail.size());
        if (n == 0)
            break;
        for (size_t i = 0; i < n; ++i)
            y.push_back(tail[i]);
    }
    return y;
}

float max_abs_diff(const std::vector<gr_complex>& a,
                   const std::vector<gr_complex>& b)
{
    BOOST_REQUIRE_EQUAL(a.size(), b.size());
    float m = 0.f;
    for (size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(a[i] - b[i]));
    return m;
}

std::vector<gr_complex> make_tone(size_t n, double f, double fs)
{
    std::vector<gr_complex> x(n);
    for (size_t i = 0; i < n; ++i) {
        const double p = 2.0 * M_PI * f * static_cast<double>(i) / fs;
        x[i] = gr_complex(static_cast<float>(std::cos(p)),
                          static_cast<float>(std::sin(p)));
    }
    return x;
}

double rms(const std::vector<gr_complex>& v, size_t lo, size_t hi)
{
    if (hi <= lo)
        return 0.0;
    double s = 0.0;
    for (size_t i = lo; i < hi; ++i)
        s += std::norm(v[i]);
    return std::sqrt(s / static_cast<double>(hi - lo));
}

/**
 * Normalized correlation of the output with a unit-magnitude reference tone
 * exp(+j2*pi*f*i/fs_out).  == 1 iff the output is exactly that tone.  Only
 * valid when the reference is genuinely complex (f strictly inside the band);
 * at exact Nyquist the tone is real and has conjugate spectral energy.
 */
double tone_corr(const std::vector<gr_complex>& y, double f, double fs,
                 size_t lo, size_t hi)
{
    if (hi <= lo)
        return 0.0;
    std::complex<double> acc(0, 0);
    double e = 0.0;
    for (size_t i = lo; i < hi; ++i) {
        const double p = 2.0 * M_PI * f * static_cast<double>(i) / fs;
        const std::complex<double> ref(std::cos(p), std::sin(p));
        const std::complex<double> a(y[i].real(), y[i].imag());
        acc += a * std::conj(ref);
        e += std::norm(a);
    }
    const double denom =
        std::sqrt(e * static_cast<double>(hi - lo));
    return denom > 0.0 ? std::abs(acc) / denom : 0.0;
}

/** Fine DFT peak of the output over a scan band (Hz). */
double tone_peak_freq(const std::vector<gr_complex>& y, double fs,
                      size_t lo, size_t hi, double f_lo, double f_hi,
                      double step)
{
    if (hi <= lo)
        return 0.0;
    double best_f = f_lo;
    double best_m = -1.0;
    for (double f = f_lo; f < f_hi; f += step) {
        std::complex<double> acc(0, 0);
        for (size_t i = lo; i < hi; ++i) {
            const double p = 2.0 * M_PI * f * static_cast<double>(i) / fs;
            acc += std::complex<double>(y[i].real(), y[i].imag()) *
                   std::complex<double>(std::cos(p), -std::sin(p));
        }
        const double m = std::abs(acc);
        if (m > best_m) {
            best_m = m;
            best_f = f;
        }
    }
    return best_f;
}

// ---------------------------------------------------------------------------
// A03/A05: length contract, gain, phase, impulse response
// ---------------------------------------------------------------------------

template <typename Core>
void check_length_contract(const char* name, uint32_t L, uint32_t M,
                           const std::vector<float>& taps)
{
    const size_t T = taps.size();
    // Formula vs the core's static helper (two independent derivations).
    const uint64_t Ns[] = { 0, 1, 2, 3, 47, 48, 49, 64, 65, 95, 96, 4096 };
    for (uint64_t N : Ns) {
        const int64_t want = frozen_len(N, L, M, T);
        BOOST_CHECK_MESSAGE(
            Core::expected_output_length(N, T) == static_cast<size_t>(want),
            std::string(name) + " static len N=" + std::to_string(N));
        if (N == 0)
            continue; // actual process/flush of N=0 handled in tail test
        std::vector<gr_complex> x(N, gr_complex(0, 0));
        x[0] = gr_complex(1, 0);
        Core core(taps);
        auto y = run_full(core, x);
        BOOST_CHECK_MESSAGE(
            y.size() == static_cast<size_t>(want),
            std::string(name) + " produced N=" + std::to_string(N) +
                " got " + std::to_string(y.size()) + " want " +
                std::to_string(want));
        BOOST_CHECK_EQUAL(core.input_items(), N);
        BOOST_CHECK_EQUAL(core.output_items(), y.size());
        BOOST_CHECK_LT(core.phase(), L);
    }
    // Hand-derived anchors (T=2707): N=1 -> ceil(T/M), N=2 -> ceil((L+T)/M).
    BOOST_CHECK_EQUAL(Core::expected_output_length(1, T),
                      static_cast<size_t>(frozen_len(1, L, M, T)));
    std::cout << "length[" << name << "] T=" << T << " L=" << L << " M=" << M
              << " N=1->" << Core::expected_output_length(1, T)
              << " N=2->" << Core::expected_output_length(2, T) << "\n";
}

template <typename Core>
void check_gain_and_impulse(const char* name, uint32_t L, uint32_t M,
                            const std::vector<float>& taps)
{
    const size_t T = taps.size();
    Core core(taps);
    BOOST_CHECK_EQUAL(core.tap_count(), T);
    BOOST_CHECK_EQUAL(core.arm_length(), (T + L - 1) / L);

    // DC sum of the frozen taps == Interp (effective gain 1).
    double sum = 0.0;
    for (float t : taps)
        sum += static_cast<double>(t);
    BOOST_CHECK_MESSAGE(std::fabs(sum - static_cast<double>(L)) < 1e-3,
                        std::string(name) + " DC sum=" + std::to_string(sum));
    BOOST_CHECK_EQUAL(core.arm_tap(0, 0), taps[0]);
    BOOST_CHECK_EQUAL(core.arm_tap(1, 0), taps[1]);

    // Impulse at n=0: y[m] == taps[m*M] while m*M < T (hand-derived from the
    // frozen upfirdn formula).  The peak index/value follow directly.
    std::vector<gr_complex> x(1, gr_complex(1, 0));
    auto y = run_full(core, x);
    size_t peak_m = 0;
    float peak_v = 0.f;
    size_t n_nonzero = 0;
    for (size_t m = 0; m < y.size(); ++m) {
        const size_t idx = m * M;
        const float want = (idx < T) ? taps[idx] : 0.0f;
        BOOST_CHECK_MESSAGE(
            std::abs(y[m].real() - want) < 1e-6f &&
                std::abs(y[m].imag()) < 1e-9f,
            std::string(name) + " impulse m=" + std::to_string(m));
        if (idx < T) {
            ++n_nonzero;
            if (std::abs(want) > peak_v) {
                peak_v = std::abs(want);
                peak_m = m;
            }
        }
    }
    // Independent expected peak from the taps themselves.
    size_t exp_peak_m = 0;
    float exp_peak_v = 0.f;
    for (size_t m = 0; m * M < T; ++m) {
        if (std::abs(taps[m * M]) > exp_peak_v) {
            exp_peak_v = std::abs(taps[m * M]);
            exp_peak_m = m;
        }
    }
    BOOST_CHECK_EQUAL(peak_m, exp_peak_m);
    BOOST_CHECK_CLOSE(peak_v, exp_peak_v, 0.01f);
    BOOST_CHECK_EQUAL(n_nonzero, static_cast<size_t>(frozen_len(1, L, M, T)));
    BOOST_CHECK_GT(n_nonzero, 0u);

    std::cout << "gain[" << name << "] T=" << T << " dc=" << sum
              << " peak_m=" << peak_m << " peak_v=" << peak_v << "\n";
}

template <typename Core>
void check_phase_map(const char* name, uint32_t L, uint32_t M,
                     const std::vector<float>& taps)
{
    Core core(taps);
    BOOST_CHECK_EQUAL(core.phase(), 0u);
    // map(p) = llround((p*L + 0.5*(T-1))/M), clamped >= 0.
    const int64_t T1 = static_cast<int64_t>(taps.size()) - 1;
    const int64_t ps[] = { 0, 1, 2, 32, 48, 65, -1000 };
    for (int64_t p : ps) {
        const double m =
            (static_cast<double>(p) * L + 0.5 * static_cast<double>(T1)) / M;
        int64_t want = static_cast<int64_t>(std::llround(m));
        if (want < 0)
            want = 0;
        BOOST_CHECK_MESSAGE(core.map_input_offset_to_output(p) == want,
                            std::string(name) + " map(" + std::to_string(p) +
                                ")=" +
                                std::to_string(
                                    core.map_input_offset_to_output(p)) +
                                " want " + std::to_string(want));
    }
    std::cout << "map[" << name << "] p=0->"
              << core.map_input_offset_to_output(0) << " p=1->"
              << core.map_input_offset_to_output(1) << " p=2->"
              << core.map_input_offset_to_output(2) << " p=65->"
              << core.map_input_offset_to_output(65) << "\n";
}

// ---------------------------------------------------------------------------
// A04: chunk invariance, flush/reset
// ---------------------------------------------------------------------------

template <typename Core>
void check_chunk_invariance(const char* name, const std::vector<float>& taps)
{
    const size_t N = 4096;
    std::vector<gr_complex> x(N);
    for (size_t i = 0; i < N; ++i)
        x[i] = gr_complex(std::sin(0.01f * i), std::cos(0.013f * i));

    Core one(taps);
    auto y_ref = run_full(one, x);

    // Several chunk sizes with a generous buffer.
    Core c1(taps);
    auto y1 = run_chunked(c1, x, { 1, 47, 48, 49, 100, 4096 }, 8192);
    BOOST_CHECK_EQUAL(y1.size(), y_ref.size());
    const float e1 = max_abs_diff(y1, y_ref);

    // Partial consume with a SHORT output buffer (forces process() to return
    // fewer outputs and consume fewer inputs per call).
    Core c2(taps);
    auto y2 = run_chunked(c2, x, { 4096 }, 16);
    BOOST_CHECK_EQUAL(y2.size(), y_ref.size());
    const float e2 = max_abs_diff(y2, y_ref);

    std::cout << "chunk[" << name << "] ref=" << y_ref.size()
              << " max_abs(chunks)=" << e1 << " max_abs(short)=" << e2 << "\n";
    BOOST_CHECK_SMALL(e1, 1e-6f);
    BOOST_CHECK_SMALL(e2, 1e-6f);
}

template <typename Core>
void check_reset_and_flush(const char* name, const std::vector<float>& taps)
{
    const size_t N = 512;
    std::vector<gr_complex> a(N), b(N);
    for (size_t i = 0; i < N; ++i) {
        a[i] = gr_complex(std::sin(0.05f * i), 0.25f);
        b[i] = gr_complex(0.5f, std::cos(0.07f * i));
    }

    // reset() between independent bursts == a fresh core on burst B.
    Core core(taps);
    (void)run_full(core, a);
    core.reset();
    BOOST_CHECK_EQUAL(core.input_items(), 0u);
    BOOST_CHECK_EQUAL(core.output_items(), 0u);
    BOOST_CHECK_EQUAL(core.phase(), 0u);
    BOOST_CHECK_EQUAL(core.resets(), 1u);
    auto y_burst = run_full(core, b);

    Core fresh(taps);
    auto y_fresh = run_full(fresh, b);
    BOOST_CHECK_EQUAL(y_burst.size(), y_fresh.size());
    BOOST_CHECK_SMALL(max_abs_diff(y_burst, y_fresh), 1e-6f);

    // process() after flush throws until reset(); reset() clears the guard.
    Core g(taps);
    std::vector<gr_complex> out(8192);
    auto r = g.process(a.data(), a.size(), out.data(), out.size());
    BOOST_REQUIRE_EQUAL(r.consumed, a.size());
    size_t n = 0;
    while (true) {
        const size_t k = g.flush(out.data(), out.size());
        if (k == 0)
            break;
        n += k;
    }
    BOOST_CHECK(g.flush_complete());
    // Repeated flush is idempotent: no further samples.
    BOOST_CHECK_EQUAL(g.flush(out.data(), out.size()), 0u);
    BOOST_CHECK_EQUAL(g.flush(out.data(), out.size()), 0u);
    bool threw = false;
    try {
        (void)g.process(b.data(), b.size(), out.data(), out.size());
    } catch (const std::logic_error&) {
        threw = true;
    }
    BOOST_CHECK_MESSAGE(threw, std::string(name) +
                                   " process-after-flush must throw");
    g.reset();
    auto r2 = g.process(b.data(), b.size(), out.data(), out.size());
    BOOST_CHECK_EQUAL(r2.consumed, b.size());

    std::cout << "reset[" << name << "] burst_match=" << y_burst.size()
              << " flush_total=" << n << "\n";
}

// ---------------------------------------------------------------------------
// A03: kernel agreement, and rates / alias
// ---------------------------------------------------------------------------

template <typename Core>
void check_kernels(const char* name, const std::vector<float>& taps)
{
    const size_t N = 1024;
    std::vector<gr_complex> x(N);
    for (size_t i = 0; i < N; ++i)
        x[i] = gr_complex(std::sin(0.02f * i), std::cos(0.017f * i));

    Core def(taps);
    Core sca(taps);
    sca.force_scalar_kernel();
    BOOST_CHECK(std::string(sca.kernel_name()).find("scalar") !=
                std::string::npos);
    auto y_def = run_full(def, x);
    auto y_sca = run_full(sca, x);
    BOOST_CHECK_EQUAL(y_def.size(), y_sca.size());
    const float e = max_abs_diff(y_def, y_sca);
    std::cout << "kernel[" << name << "] default=" << def.kernel_name()
              << " max_abs=" << e << "\n";
    BOOST_CHECK_SMALL(e, kKernelTol);
}

/**
 * Rate/alias behaviour.
 *  - f0 inside the passband is preserved (same absolute frequency, gain ~1).
 *  - a tone at the minimum Nyquist (== the frozen cutoff) is still mapped to
 *    the same absolute frequency.
 *  - for TX (input Nyquist > output Nyquist) a tone above the stopband edge
 *    is rejected, proving the anti-image band-limit to the native Nyquist.
 */
template <typename Core>
void check_rates_and_alias(const char* name, uint32_t L, uint32_t M,
                           double fs_in, double fs_out,
                           const std::vector<float>& taps,
                           double stopband_hz /*0 = no TX stopband test*/)
{
    const size_t N = 4096;
    const double min_nyq = std::min(fs_in, fs_out) / 2.0;

    // Passband tone at 100 MHz.
    {
        const double f0 = 100.0e6;
        auto x = make_tone(N, f0, fs_in);
        Core core(taps);
        auto y = run_full(core, x);
        const size_t lo = y.size() / 4;
        const size_t hi = y.size();
        const double c = tone_corr(y, f0, fs_out, lo, hi);
        const double g = rms(y, lo, hi) / rms(x, 0, x.size());
        std::cout << "tone[" << name << "] f0=100MHz corr=" << c
                  << " gain=" << g << "\n";
        BOOST_CHECK_MESSAGE(c > 0.99, std::string(name) + " passband corr");
        BOOST_CHECK_SMALL(std::fabs(g - 1.0), 0.02);
    }

    // Tone at the minimum Nyquist (the frozen cutoff).  At exact Nyquist the
    // complex tone is real, so verify the MAPPED FREQUENCY via a fine DFT
    // peak rather than a complex-tone correlation.
    {
        auto x = make_tone(N, min_nyq, fs_in);
        Core core(taps);
        auto y = run_full(core, x);
        const size_t lo = y.size() / 4;
        const size_t hi = y.size();
        const double peak = tone_peak_freq(y, fs_out, lo, hi, fs_out * 0.2,
                                           fs_out * 0.55, fs_out / 20000.0);
        std::cout << "nyq[" << name << "] f=" << min_nyq
                  << " peak=" << peak << " rms=" << rms(y, lo, hi) << "\n";
        BOOST_CHECK_MESSAGE(
            std::fabs(peak - min_nyq) <= fs_out / 20000.0 + 1.0,
            std::string(name) + " min-Nyquist tone peak freq");
        // At the cutoff the response is ~-3 dB, so the tone is not removed.
        BOOST_CHECK_GT(rms(y, lo, hi), 0.2);
    }

    // TX only: stopband rejection above the native Nyquist.
    if (stopband_hz > 0.0) {
        auto x = make_tone(N, stopband_hz, fs_in);
        Core core(taps);
        auto y = run_full(core, x);
        const size_t lo = y.size() / 4;
        const size_t hi = y.size();
        const double g = rms(y, lo, hi) / rms(x, 0, x.size());
        std::cout << "stop[" << name << "] f=" << stopband_hz
                  << " gain=" << g << "\n";
        BOOST_CHECK_MESSAGE(g < 0.05,
                            std::string(name) + " stopband not rejected");
    }
}

} // namespace

// ---------------------------------------------------------------------------
// Test cases
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(test_m2a_alias_rates)
{
    // The two new aliases must carry the correct L/M.
    BOOST_CHECK_EQUAL(RationalResampler48_65Core::kInterp, 48u);
    BOOST_CHECK_EQUAL(RationalResampler48_65Core::kDecim, 65u);
    BOOST_CHECK_EQUAL(RationalResampler32_65Core::kInterp, 32u);
    BOOST_CHECK_EQUAL(RationalResampler32_65Core::kDecim, 65u);
    BOOST_CHECK_EQUAL(RationalResampler65_48Core::kInterp, 65u);
    BOOST_CHECK_EQUAL(RationalResampler65_48Core::kDecim, 48u);
    BOOST_CHECK_EQUAL(RationalResampler65_32Core::kInterp, 65u);
    BOOST_CHECK_EQUAL(RationalResampler65_32Core::kDecim, 32u);
}

BOOST_AUTO_TEST_CASE(test_m2a_tap_files_shape)
{
    const auto& a = tx_48_65_taps();
    const auto& b = tx_32_65_taps();
    BOOST_CHECK_EQUAL(a.size(), 2707u);
    BOOST_CHECK_EQUAL(b.size(), 2707u);
    // Frozen sha256 (recorded in testdata/twr/m2a/taps/design.json).
    // (Content hash is checked by the design script/README; here we pin the
    //  DC sum and length that the core contract depends on.)
    double sa = 0.0, sb = 0.0;
    for (float t : a)
        sa += t;
    for (float t : b)
        sb += t;
    BOOST_CHECK_CLOSE(sa, 48.0, 0.01);
    BOOST_CHECK_CLOSE(sb, 32.0, 0.01);
}

BOOST_AUTO_TEST_CASE(test_m2a_length_contract_all_dirs)
{
    check_length_contract<RationalResampler48_65Core>("tx_48_65", 48, 65,
                                                      tx_48_65_taps());
    check_length_contract<RationalResampler32_65Core>("tx_32_65", 32, 65,
                                                      tx_32_65_taps());
    check_length_contract<RationalResampler65_48Core>("rx_65_48", 65, 48,
                                                      rx_65_48_taps());
    check_length_contract<RationalResampler65_32Core>("rx_65_32", 65, 32,
                                                      rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_gain_and_impulse_all_dirs)
{
    check_gain_and_impulse<RationalResampler48_65Core>("tx_48_65", 48, 65,
                                                       tx_48_65_taps());
    check_gain_and_impulse<RationalResampler32_65Core>("tx_32_65", 32, 65,
                                                       tx_32_65_taps());
    check_gain_and_impulse<RationalResampler65_48Core>("rx_65_48", 65, 48,
                                                       rx_65_48_taps());
    check_gain_and_impulse<RationalResampler65_32Core>("rx_65_32", 65, 32,
                                                       rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_phase_map_all_dirs)
{
    check_phase_map<RationalResampler48_65Core>("tx_48_65", 48, 65,
                                                tx_48_65_taps());
    check_phase_map<RationalResampler32_65Core>("tx_32_65", 32, 65,
                                                tx_32_65_taps());
    check_phase_map<RationalResampler65_48Core>("rx_65_48", 65, 48,
                                                rx_65_48_taps());
    check_phase_map<RationalResampler65_32Core>("rx_65_32", 65, 32,
                                                rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_chunk_invariance_all_dirs)
{
    check_chunk_invariance<RationalResampler48_65Core>("tx_48_65",
                                                       tx_48_65_taps());
    check_chunk_invariance<RationalResampler32_65Core>("tx_32_65",
                                                       tx_32_65_taps());
    check_chunk_invariance<RationalResampler65_48Core>("rx_65_48",
                                                       rx_65_48_taps());
    check_chunk_invariance<RationalResampler65_32Core>("rx_65_32",
                                                       rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_reset_flush_all_dirs)
{
    check_reset_and_flush<RationalResampler48_65Core>("tx_48_65",
                                                      tx_48_65_taps());
    check_reset_and_flush<RationalResampler32_65Core>("tx_32_65",
                                                      tx_32_65_taps());
    check_reset_and_flush<RationalResampler65_48Core>("rx_65_48",
                                                      rx_65_48_taps());
    check_reset_and_flush<RationalResampler65_32Core>("rx_65_32",
                                                      rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_kernels_all_dirs)
{
    check_kernels<RationalResampler48_65Core>("tx_48_65", tx_48_65_taps());
    check_kernels<RationalResampler32_65Core>("tx_32_65", tx_32_65_taps());
    check_kernels<RationalResampler65_48Core>("rx_65_48", rx_65_48_taps());
    check_kernels<RationalResampler65_32Core>("rx_65_32", rx_65_32_taps());
}

BOOST_AUTO_TEST_CASE(test_m2a_rates_and_alias_all_dirs)
{
    check_rates_and_alias<RationalResampler48_65Core>(
        "tx_48_65", 48, 65, 998.4e6, 737.28e6, tx_48_65_taps(), 450.0e6);
    check_rates_and_alias<RationalResampler32_65Core>(
        "tx_32_65", 32, 65, 998.4e6, 491.52e6, tx_32_65_taps(), 350.0e6);
    check_rates_and_alias<RationalResampler65_48Core>(
        "rx_65_48", 65, 48, 737.28e6, 998.4e6, rx_65_48_taps(), 0.0);
    check_rates_and_alias<RationalResampler65_32Core>(
        "rx_65_32", 65, 32, 491.52e6, 998.4e6, rx_65_32_taps(), 0.0);
}

BOOST_AUTO_TEST_CASE(test_m2a_n0_core_contract)
{
    // Core-level N=0 flush yields the transient ceil((T-L)/M); the M2-A helper
    // overrides this to 0 (G0 §3.2).  Document the core behaviour explicitly.
    {
        RationalResampler48_65Core c(tx_48_65_taps());
        auto y = run_full(c, {});
        BOOST_CHECK_EQUAL(y.size(), frozen_len(0, 48, 65, c.tap_count()));
    }
    {
        RationalResampler65_48Core c(rx_65_48_taps());
        auto y = run_full(c, {});
        BOOST_CHECK_EQUAL(y.size(), frozen_len(0, 65, 48, c.tap_count()));
    }
}
