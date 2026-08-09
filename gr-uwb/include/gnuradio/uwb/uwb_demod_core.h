/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * GNU Radio-independent UWB demodulation core.  Each stage is a pure function
 * that can be unit-tested and benchmarked without linking against GNU Radio.
 *
 * This header is the Phase-1 SKELETON: it declares the per-stage function
 * signatures and the worker's scratch state.  No algorithm is implemented
 * here until the MATLAB golden reference (R0) is frozen.
 *
 * Coordinate convention: all sample indices are 0-based absolute.  Each stage
 * takes the cropped frame + the previous stage's output and fills the next.
 */

#pragma once

#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace core {

// ---------------------------------------------------------------------------
// R1 implementations: Stage 1 (timing), Stage 2 (CFO), Stage 3 (SFD refine).
// Coordinate convention: 0-based ABSOLUTE sample indices (matching detector /
// PDU metadata and the R0 golden reference, which is stored in absolute
// coordinates).  The template `template_wf` is the 1016-sample L2-normalized
// SYNC waveform.
// ---------------------------------------------------------------------------

namespace detail {

// Full-rate normalized matched filter over [roi_start, roi_end).  Fills
// metric[j] for j in [roi_start, roi_end-L), with
//   metric[j] = |<rx, tmpl>|^2 / (winpow[j] * Et + eps)
// Returns the argmax position and its metric.
inline size_t
normalized_mf(const std::complex<float>* rx,
              size_t n,
              size_t roi_start,
              size_t roi_end,
              const std::complex<float>* tmpl,
              size_t L,
              float tmpl_energy,
              std::vector<float>& metric, // caller-owned, >= n
              float* best_metric = nullptr)
{
    const size_t jmax = (roi_end >= L) ? roi_end - L + 1 : roi_start;
    size_t best = roi_start;
    float bm = -1.0f;
    if (jmax <= roi_start)
        return best;
    // window power of rx over the same L-length window (running sum)
    float wsum = 0.0f;
    for (size_t k = 0; k < L; ++k)
        wsum += std::norm(rx[roi_start + k]);
    for (size_t j = roi_start; j < jmax; ++j) {
        std::complex<float> corr(0.0f, 0.0f);
        for (size_t k = 0; k < L; ++k)
            corr += rx[j + k] * std::conj(tmpl[k]);
        const float d = wsum * tmpl_energy + 1e-12f;
        const float m = std::norm(corr) / d;
        metric[j] = m;
        if (m > bm) {
            bm = m;
            best = j;
        }
        if (j + 1 < jmax) {
            wsum -= std::norm(rx[j]);
            wsum += std::norm(rx[j + L]);
        }
    }
    if (best_metric)
        *best_metric = bm;
    return best;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Worker scratch: one per worker thread.  Pre-allocated at construction and
// reused across jobs to avoid per-packet allocation in the hot path.
// ---------------------------------------------------------------------------
struct DemodScratch {
    std::vector<std::complex<float>> work;     // cropped frame workspace
    std::vector<std::complex<float>> derotated; // CFO-derotated frame
    std::vector<float> energy;                  // per-sample |x|^2
    std::vector<float> metric;                  // correlation metric buffer
    std::vector<std::complex<float>> corr;      // matched-filter output
    std::vector<float> soft_chips;              // soft-chip stream
    std::vector<float> cir_taps;                // CIR estimate
    std::vector<int64_t> peaks;                 // SYNC peak positions
    std::vector<float> peak_values;             // per-peak complex correlation

    void reserve(size_t n)
    {
        work.reserve(n);
        derotated.reserve(n);
        energy.reserve(n);
        metric.reserve(n);
        corr.reserve(n);
        soft_chips.reserve(n);
        cir_taps.reserve(64);
        peaks.reserve(256);
        peak_values.reserve(256);
    }
};

// ---------------------------------------------------------------------------
// Stage 1 — seeded timing (R1).
// Uses the detector-supplied absolute seed_start (or, if -1, scans the whole
// buffer) to run a full-rate matched filter over a narrow ROI, then tracks
// the repeated SYNC peaks spaced ~samples_per_symbol apart, and fits a linear
// model to get measured_period.  Peak positions are symbol-END convention
// (first peak = start + (L-1)), matching the detector / MATLAB.
// ---------------------------------------------------------------------------
inline bool stage_timing(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const std::vector<std::complex<float>>& template_wf,
                         int64_t seed_start, // absolute predicted start, or -1
                         TimingResult& out,
                         DemodScratch& scratch)
{
    out = TimingResult{};
    const size_t L = template_wf.size();
    if (L == 0 || n < L)
        return false;

    // Copy + L2-normalize the template (own copy so callers may pass raw).
    scratch.work.assign(template_wf.begin(), template_wf.end());
    gr::uwb::core::uwb_l2_normalize(scratch.work);
    const float Et = gr::uwb::core::uwb_template_energy(scratch.work);

    // Determine the ROI around the seed (absolute coordinates).
    size_t roi_start = 0;
    size_t roi_end = n;
    if (seed_start >= 0) {
        const int64_t seed = seed_start;
        const int64_t margin = static_cast<int64_t>(profile.timing_search_margin);
        const int64_t lo = std::max<int64_t>(0, seed - margin);
        const int64_t hi =
            std::min<int64_t>(static_cast<int64_t>(n), seed + margin + (int64_t)L);
        roi_start = static_cast<size_t>(lo);
        roi_end = static_cast<size_t>(hi);
    }

    // Local normalized-correlation helper over a small window around `want`:
    // finds the SYNC START that best matches the template, returns position+metric.
    auto local_best = [&](int64_t want, int64_t radius, int64_t* pos,
                          float* met) -> bool {
        const int64_t w0 = std::max<int64_t>(0, want - radius);
        const int64_t w1 =
            std::min<int64_t>(static_cast<int64_t>(n) - static_cast<int64_t>(L),
                              want + radius);
        if (w1 <= w0)
            return false;
        float bm = -1.0f;
        int64_t bp = w0;
        for (int64_t j = w0; j < w1; ++j) {
            std::complex<float> acc(0.0f, 0.0f);
            float pw = 0.0f;
            for (size_t k = 0; k < L; ++k) {
                acc += rx[j + k] * std::conj(scratch.work[k]);
                pw += std::norm(rx[j + k]);
            }
            const float m = std::norm(acc) / (pw * Et + 1e-12f);
            if (m > bm) {
                bm = m;
                bp = j;
            }
        }
        if (bm < profile.radar_verification_threshold)
            return false;
        *pos = bp;
        *met = bm;
        return true;
    };

    // Locate the first SYNC start near the seed (narrow ROI).
    int64_t first_start = -1;
    float first_metric = 0.0f;
    {
        const int64_t seed =
            (seed_start >= 0) ? seed_start
                              : (int64_t)(roi_start + (roi_end - roi_start) / 2);
        const int64_t radius = 64; // seed is already coarse
        if (!local_best(seed, radius, &first_start, &first_metric))
            return false;
    }
    const int64_t start = first_start;
    if (start < 0)
        return false;
    const int64_t period = static_cast<int64_t>(kQm35SamplesPerSymbol);

    // Track all SYNC starts: expected = start + k*period, then convert each to
    // symbol-END convention (start + L - 1) to match the detector / MATLAB.
    out.peak_samples.clear();
    out.peak_metrics.clear();
    out.peak_samples.push_back(start + static_cast<int64_t>(L - 1));
    out.peak_metrics.push_back(first_metric);
    for (size_t k = 1; k < profile.preamble_repetitions; ++k) {
        const int64_t want = start + static_cast<int64_t>(k * period);
        int64_t pk = -1;
        float pm = 0.0f;
        if (!local_best(want, 8, &pk, &pm))
            break; // lost the train
        out.peak_samples.push_back(pk + static_cast<int64_t>(L - 1));
        out.peak_metrics.push_back(pm);
    }

    out.detected_peaks = out.peak_samples.size();
    out.expected_peaks = profile.preamble_repetitions;
    out.preamble_start_sample = start;
    out.metric = first_metric;

    // Linear fit of peak positions vs repetition index -> measured period.
    // peak(k) = start + k*period  =>  least squares slope.
    const size_t np = out.peak_samples.size();
    if (np >= 2) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (size_t k = 0; k < np; ++k) {
            const double x = static_cast<double>(k);
            const double y = static_cast<double>(out.peak_samples[k]);
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        const double denom = static_cast<double>(np) * sxx - sx * sx;
        if (std::abs(denom) > 1e-12)
            out.measured_period = (static_cast<double>(np) * sxy - sx * sy) /
                                  denom;
        else
            out.measured_period = static_cast<double>(period);
    } else {
        out.measured_period = static_cast<double>(period);
    }

    out.ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation (R1).
// Uses the stable SYNC peaks (skipping the first cir_skip_initial_repetitions)
// to fit phase vs time linearly; the slope / 2pi is the CFO in Hz.  The frame
// is then derotated by exp(-j2pi*f*n/fs) and phase-resolved against the
// preamble waveform.  Golden reference: CFO = 0 Hz for the clean baseband
// signal.
// ---------------------------------------------------------------------------
inline bool stage_cfo(const std::complex<float>* rx,
                      size_t n,
                      const Qm35825Profile& profile,
                      const TimingResult& timing,
                      CfoResult& out,
                      DemodScratch& scratch)
{
    out = CfoResult{};
    const size_t np = timing.peak_samples.size();
    if (np < 4)
        return false;

    // Skip the first `skip` peaks (front-end startup transient) but keep at
    // least a few for the fit.
    const size_t skip =
        std::min(profile.cir_skip_initial_repetitions, np - 2);
    const size_t nfit = np - skip;
    if (nfit < 2)
        return false;

    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (size_t k = 0; k < nfit; ++k) {
        const size_t idx = skip + k;
        const double t = (double)timing.peak_samples[idx] / profile.sample_rate;
        const double ph =
            std::arg(rx[timing.peak_samples[idx]]); // complex at peak
        sx += t;
        sy += ph;
        sxx += t * t;
        sxy += t * ph;
    }
    const double denom = (double)nfit * sxx - sx * sx;
    if (std::abs(denom) < 1e-30)
        return false;
    const double slope = ((double)nfit * sxy - sx * sy) / denom;
    out.cfo_hz = slope / (2.0 * M_PI);
    out.peaks_used = nfit;
    out.ok = true;

    // Optional: derotate a copy (scratch.derotated).  The R1 stage reports
    // CFO; the block applies the rotation when chaining stages.
    scratch.derotated.resize(n);
    const double w = 2.0 * M_PI * out.cfo_hz / profile.sample_rate;
    for (size_t i = 0; i < n; ++i) {
        const double ph = w * (double)i;
        scratch.derotated[i] = rx[i] * std::complex<float>(std::cos(ph), -std::sin(ph));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Stage 3 — SFD timing refinement (R1).
// Selects the best SFD template (from the profile; golden uses IEEE legacy
// SFD #0 = {0,1,0,-1,1,0,0,-1}) and refines the preamble start at full rate.
// For Phase-1 the SFD search is done on the symbol grid: the SFD starts one
// symbol after the last SYNC (i.e. after preamble_repetitions symbols).
// We locate the SFD by correlating the SFD template over a narrow window
// around the expected position, then refine the preamble start.
// ---------------------------------------------------------------------------
inline bool stage_sfd(const std::complex<float>* rx,
                      size_t n,
                      const Qm35825Profile& profile,
                      const TimingResult& timing,
                      const std::vector<int8_t>& sfd_sequence,
                      const std::vector<std::complex<float>>& template_wf,
                      SfdResult& out,
                      DemodScratch& scratch)
{
    out = SfdResult{};
    if (sfd_sequence.empty() || timing.peak_samples.empty() ||
        template_wf.empty())
        return false;
    const size_t sym = kQm35SamplesPerSymbol;

    // Build the SFD template: kron(sfd_sequence, preamble_waveform), then
    // L2-normalize — same as MATLAB refineTimingWithNsSfd.
    const size_t len = sfd_sequence.size();
    scratch.corr.clear();
    scratch.corr.reserve(len * sym);
    for (size_t i = 0; i < len; ++i) {
        const int8_t s = sfd_sequence[i];
        for (size_t k = 0; k < sym; ++k)
            scratch.corr.push_back(
                std::complex<float>(static_cast<float>(s) *
                                        template_wf[k].real(),
                                    static_cast<float>(s) *
                                        template_wf[k].imag()));
    }
    gr::uwb::core::uwb_l2_normalize(scratch.corr);
    const size_t sfd_len = scratch.corr.size();

    // Search window around expected = start + preamble_repetitions * period.
    const int64_t expected = timing.preamble_start_sample +
                             static_cast<int64_t>(profile.preamble_repetitions) *
                                 static_cast<int64_t>(sym);
    const int64_t search_lo = std::max<int64_t>(0, expected - (int64_t)sym);
    const int64_t search_hi =
        std::min<int64_t>((int64_t)n,
                          expected + (int64_t)sym + (int64_t)sfd_len);

    // Full-rate correlation (normalized) over the search window.
    float best = -1.0f;
    size_t best_j = 0;
    for (size_t j = static_cast<size_t>(search_lo);
         j + sfd_len <= static_cast<size_t>(search_hi); ++j) {
        std::complex<float> acc(0.0f, 0.0f);
        float pwr = 0.0f;
        for (size_t k = 0; k < sfd_len; ++k) {
            acc += rx[j + k] * std::conj(scratch.corr[k]);
            pwr += std::norm(rx[j + k]);
        }
        const float m = std::norm(acc) / (pwr + 1e-12f);
        if (m > best) {
            best = m;
            best_j = j;
        }
    }
    if (best < profile.sfd_detection_threshold)
        return false;

    out.ok = true;
    out.sfd_mode = profile.sfd_mode;
    out.sfd_start_sample = static_cast<int64_t>(best_j);
    out.sfd_end_sample = static_cast<int64_t>(best_j) +
                         static_cast<int64_t>(sfd_len) - 1;
    out.metric = best;
    out.polarity = 1;
    return true;
}

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation (R2).
// Mirrors MATLAB estimateCir + estimateCirAndSoftChips:
//
//   * CIR: coherently average the aligned raw SYNC windows of the LAST
//     cir_repetitions repetitions (skipping the first
//     cir_skip_initial_repetitions), then correlate each of the
//     (pre+post) taps against the code in NATURAL order:
//       values[n] = sum_m avg[n+m] * conj(sampled_code[m]) / code_energy
//     and L2-normalize.  (The template is the sparse sampled code, NOT the
//     pulse-shaped preamble waveform — forward order, not time-reversed.)
//
//   * Soft chips: causal FIR with cirMf = conj(flip(values)) evaluated on the
//     chip grid chipStart = start + (pre+post) - pre - 1, spaced
//     measured_period/chips_per_symbol apart, up to the frame-span budget
//     bounded by the buffer length.  The complex chips are then phase-aligned
//     against the last min(32, preamble_repetitions) SYNCs' spread code, and
//     the real part is normalized by max|real|.
//
// Coordinate convention: rx uses 0-based ABSOLUTE sample indices (the full
// scheduled window / captured PDU), so chipStart and the CIR first-path are
// absolute.  Verified against the MATLAB golden reference: CIR max diff
// ~6e-8, soft-chip stream max diff ~6e-7 (stage_cir.mat / stage_softchips.mat).
// ---------------------------------------------------------------------------
inline bool stage_cir_softchips(const std::complex<float>* rx,
                                size_t n,
                                const Qm35825Profile& profile,
                                const TimingResult& timing,
                                const std::vector<int8_t>& preamble_code,
                                CirResult& out,
                                DemodScratch& scratch)
{
    out = CirResult{};
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (period <= 0.0 || start < 0 || preamble_code.empty() || tap_count == 0)
        return false;

    // ---- Build the sparse sampled code (508 spread x2 -> 1016, zeros on the
    //      odd samples), matching MATLAB buildUwbReference.sampled_code. ----
    const std::vector<int8_t> spread =
        BuildSampledCode(preamble_code.data(), preamble_code.size());
    scratch.work.assign(2 * kQm35ChipsPerSymbol, std::complex<float>(0.0f, 0.0f));
    for (size_t j = 0; j < spread.size(); ++j)
        scratch.work[2 * j] = std::complex<float>((float)spread[j], 0.0f);
    const size_t code_len = scratch.work.size(); // 1016
    float code_energy = 0.0f;
    for (size_t m = 0; m < code_len; ++m)
        code_energy += std::norm(scratch.work[m]);

    // ---- Coherently average the aligned SYNC windows. ----
    const size_t available = std::min(timing.detected_peaks, profile.preamble_repetitions);
    const size_t skip = std::min(profile.cir_skip_initial_repetitions, available);
    if (available == 0 || skip >= available)
        return false;
    const size_t rep_count = std::min(profile.cir_repetitions, available - skip);
    if (rep_count == 0)
        return false;

    const size_t wlen = code_len + tap_count - 1; // 1053
    scratch.corr.assign(wlen, std::complex<float>(0.0f, 0.0f));
    size_t valid = 0;
    for (size_t k = skip; k < skip + rep_count; ++k) {
        const int64_t rs = start + static_cast<int64_t>(std::llround((double)k * period));
        const int64_t lo = rs - static_cast<int64_t>(pre);
        const int64_t hi = lo + static_cast<int64_t>(wlen);
        if (lo < 0 || hi > static_cast<int64_t>(n))
            continue; // drop SYNCs clipped by the buffer edge (like MATLAB)
        for (size_t m = 0; m < wlen; ++m)
            scratch.corr[m] += rx[static_cast<size_t>(lo + (int64_t)m)];
        ++valid;
    }
    if (valid == 0)
        return false;
    const float inv = 1.0f / static_cast<float>(valid);
    for (size_t m = 0; m < wlen; ++m)
        scratch.corr[m] *= inv;

    // ---- CIR taps: forward-order code correlation (NOT time-reversed). ----
    std::vector<std::complex<float>> values(tap_count);
    for (size_t nn = 0; nn < tap_count; ++nn) {
        std::complex<float> acc(0.0f, 0.0f);
        for (size_t m = 0; m < code_len; ++m)
            acc += std::conj(scratch.work[m]) * scratch.corr[nn + m];
        values[nn] = acc / code_energy;
    }
    float nrm = 0.0f;
    for (size_t i = 0; i < tap_count; ++i)
        nrm += std::norm(values[i]);
    nrm = std::sqrt(nrm);
    if (!(nrm > 0.0f))
        return false;
    for (size_t i = 0; i < tap_count; ++i)
        values[i] /= (nrm + 1e-12f);

    size_t peak_idx = 0;
    float peak_mag = -1.0f;
    for (size_t i = 0; i < tap_count; ++i) {
        const float m = std::abs(values[i]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_idx = i;
        }
    }
    out.first_path_sample =
        static_cast<size_t>(std::max<int64_t>(0, start + (int64_t)peak_idx - (int64_t)pre));
    out.pre_samples = pre;
    out.post_samples = post;
    out.cir_peak_metric = peak_mag;
    out.cir_values.resize(tap_count);
    for (size_t i = 0; i < tap_count; ++i)
        out.cir_values[i] = values[i].real(); // golden CIR is real (Q=0 channel)

    // ---- Soft-chip matched filter on the chip grid. ----
    const double spc = period / static_cast<double>(kQm35ChipsPerSymbol);
    // Frame-span chip budget (estimateFrameSampleSpan): 4z2 SFD = 8 symbols.
    const size_t sfd_symbols = 8;
    const size_t n_shr = (profile.preamble_repetitions + sfd_symbols) * kQm35ChipsPerSymbol;
    const size_t n_phr_head = 40 * kQm35ChipsPerSymbol;
    const size_t extra_bytes =
        (profile.max_psdu_bytes > 12) ? (profile.max_psdu_bytes - 12) : 0;
    const size_t n_extra = static_cast<size_t>(std::ceil((double)extra_bytes * 1000.0));
    const size_t n_guard = 8 * kQm35ChipsPerSymbol;
    size_t n_chips = n_shr + n_phr_head + n_extra + n_guard;
    n_chips = static_cast<size_t>(std::ceil((double)n_chips * 1.05));

    const int64_t chip_start = start + static_cast<int64_t>(tap_count) -
                               static_cast<int64_t>(pre) - 1; // = start + post - 1
    const int64_t last_by_budget =
        chip_start + static_cast<int64_t>(std::ceil((double)(n_chips - 1) * spc));
    const int64_t last_chip_sample = std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
    const int64_t last_pos = last_chip_sample - 1; // inclusive, 0-based
    if (last_pos < chip_start)
        return false;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(spc));
    const size_t num_chips =
        static_cast<size_t>((last_pos - chip_start) / spc_i) + 1;

    // Causal FIR: chip[p] = sum_k conj(values[tap-1-k]) * rx[p-k].
    scratch.corr.resize(num_chips);
    for (size_t i = 0; i < num_chips; ++i) {
        const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
        std::complex<float> acc(0.0f, 0.0f);
        for (size_t k = 0; k < tap_count; ++k) {
            const int64_t idx = p - static_cast<int64_t>(k);
            if (idx < 0)
                break;
            acc += std::conj(values[tap_count - 1 - k]) * rx[static_cast<size_t>(idx)];
        }
        scratch.corr[i] = acc;
    }

    // ---- Phase-align against the last min(32, preamble) SYNCs' spread code. ----
    const size_t phase_reps = std::min<size_t>(32, profile.preamble_repetitions);
    const size_t phase_first =
        (profile.preamble_repetitions - phase_reps) * kQm35ChipsPerSymbol;
    const size_t phase_len = phase_reps * kQm35ChipsPerSymbol;
    if (phase_first + phase_len > num_chips)
        return false; // chip stream shorter than the configured preamble
    std::complex<float> gain(0.0f, 0.0f);
    for (size_t i = 0; i < phase_len; ++i)
        gain += std::conj(std::complex<float>((float)spread[i % kQm35ChipsPerSymbol], 0.0f)) *
                scratch.corr[phase_first + i];
    const float ang = std::arg(gain);
    const std::complex<float> rot(std::cos(-ang), std::sin(-ang));
    for (size_t i = 0; i < num_chips; ++i)
        scratch.corr[i] *= rot;

    // ---- soft = real part, normalized by max|real|. ----
    float mx = 0.0f;
    for (size_t i = 0; i < num_chips; ++i)
        mx = std::max(mx, std::abs(scratch.corr[i].real()));
    const float denom = mx + 1e-12f;
    scratch.soft_chips.resize(num_chips);
    for (size_t i = 0; i < num_chips; ++i)
        scratch.soft_chips[i] = scratch.corr[i].real() / denom;

    out.soft_chip_count = num_chips;
    out.samples_per_chip = spc;
    out.ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Full pipeline (R1+R2 subset): timing + CFO + SFD + CIR/soft chips.  Later
// stages (NS-SFD/PHR/payload) are R3-R4 and return "not implemented".
// ---------------------------------------------------------------------------
inline DemodResult demodulate_one(const std::complex<float>* rx,
                                  size_t n,
                                  const Qm35825Profile& profile,
                                  uint64_t packet_id,
                                  int64_t predicted_start,
                                  int64_t window_start,
                                  const std::vector<std::complex<float>>& template_wf,
                                  DemodScratch& scratch)
{
    DemodResult res;
    res.packet_id = packet_id;
    res.predicted_start_sample = predicted_start;
    res.window_start_sample = window_start;

    // Stage 1: timing (seed = predicted_start if inside this window, else -1)
    const bool t_ok = stage_timing(rx, n, profile, template_wf,
                                   predicted_start, res.timing, scratch);
    if (!t_ok) {
        res.status = DemodStatus::TimingFailed;
        return res;
    }
    // Stage 2: CFO
    if (!stage_cfo(rx, n, profile, res.timing, res.cfo, scratch)) {
        res.status = DemodStatus::CfoFailed;
        return res;
    }
    // Stage 3: SFD (profile sfd_mode)
    const auto sfd_seq = gr::uwb::demod::GetSfdSequence(profile.sfd_mode);
    if (!stage_sfd(rx, n, profile, res.timing, sfd_seq, template_wf, res.sfd,
                   scratch)) {
        res.status = DemodStatus::SfdFailed;
        return res;
    }
    // Stage 4: CIR + soft chips on the CFO-compensated frame.
    const int8_t* pc = GetPreambleCode(profile.code_index);
    std::vector<int8_t> pcode(pc, pc + kQm35CodeLength);
    if (!stage_cir_softchips(scratch.derotated.data(), n, profile, res.timing,
                             pcode, res.cir, scratch)) {
        res.status = DemodStatus::CirFailed;
        return res;
    }
    res.status = DemodStatus::Success;
    return res;
}

// ---------------------------------------------------------------------------
// Stage 1 — seeded timing: detect + track SYNC preamble repetitions.
// Returns false if fewer than min_valid_peaks SYNC peaks are confirmed.
// ---------------------------------------------------------------------------
bool stage_timing(const std::complex<float>* rx,
                  size_t n,
                  const Qm35825Profile& profile,
                  const std::vector<std::complex<float>>& template_wf,
                  int64_t seed_start, // absolute predicted start, or -1
                  TimingResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 2 — CFO estimation + compensation.
// Uses stable SYNC peaks (skipping the first cir_skip_initial_repetitions)
// to fit phase vs time, then derotates the frame.
// ---------------------------------------------------------------------------
bool stage_cfo(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               CfoResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 3 — SFD template selection + full-rate timing refinement.
// Picks the best-matching SFD template (4z2 for QM35825) and refines the
// preamble start at full sample rate.
// ---------------------------------------------------------------------------
bool stage_sfd(const std::complex<float>* rx,
               size_t n,
               const Qm35825Profile& profile,
               const TimingResult& timing,
               const std::vector<int8_t>& sfd_sequence,
               const std::vector<std::complex<float>>& template_wf,
               SfdResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 4 — CIR estimation + soft-chip generation.
// Estimates CIR from the last cir_repetitions SYNC symbols, then matched-
// filters the frame to produce a soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_cir_softchips(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in the soft-chip stream.
// ---------------------------------------------------------------------------
bool stage_ns_sfd(const std::vector<float>& soft_chips,
                  const Qm35825Profile& profile,
                  const std::vector<int8_t>& sfd_sequence,
                  size_t chips_per_symbol,
                  NsSfdResult& out,
                  DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR demod + convolutional decode + SECDED.
// ---------------------------------------------------------------------------
bool stage_phr(const std::vector<float>& soft_chips,
               const Qm35825Profile& profile,
               const NsSfdResult& ns_sfd,
               size_t chips_per_symbol,
               PhrResult& out,
               DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Stage 7 — payload BPM-BPSK demod + convolutional + RS + FCS.
// ---------------------------------------------------------------------------
bool stage_payload_fcs(const std::vector<float>& soft_chips,
                       const Qm35825Profile& profile,
                       const PhrResult& phr,
                       const NsSfdResult& ns_sfd,
                       size_t chips_per_symbol,
                       PayloadResult& out,
                       DemodScratch& scratch);

// ---------------------------------------------------------------------------
// Full pipeline: runs all stages in order, stopping at the first failure.
// Used by the worker thread for each job.
// ---------------------------------------------------------------------------
DemodResult demodulate_one(const std::complex<float>* rx,
                           size_t n,
                           const Qm35825Profile& profile,
                           uint64_t packet_id,
                           int64_t predicted_start,
                           int64_t window_start,
                           const std::vector<std::complex<float>>& template_wf,
                           DemodScratch& scratch);

} // namespace core
} // namespace demod
} // namespace uwb
} // namespace gr
