// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_demod_scratch.h>
#include <gnuradio/uwb/uwb_demod_grid.h>
#include <gnuradio/uwb/uwb_rx_softchip_opt.h>

namespace gr::uwb::demod::core {
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
// Coordinates are relative to rx, the supplied scheduled/captured window.
// The complete packet entry point rebases the published CIR first-path.  Verified against the MATLAB golden reference: CIR max diff
// ~6e-8, soft-chip stream max diff ~6e-7 (stage_cir.mat / stage_softchips.mat).
// ---------------------------------------------------------------------------

// Standalone CIR estimation stage (runs after SFD, independent of CMF/despreading).
inline bool estimate_cir(const std::complex<float>* rx,
                         size_t n,
                         const Qm35825Profile& profile,
                         const TimingResult& timing,
                         const std::vector<int8_t>& preamble_code,
                         CirResult& out,
                         DemodScratch& scratch)
{
    out = CirResult{};
    auto t_cir0 = std::chrono::steady_clock::now();
    const size_t pre = profile.cir_pre_samples;
    const size_t post = profile.cir_post_samples;
    const size_t tap_count = pre + post;
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (period <= 0.0 || start < 0 || preamble_code.empty() || tap_count == 0)
        return false;

    // ---- Build (or reuse) the sparse sampled code (508 spread x2 -> 1016, zeros on the
    //      odd samples), matching MATLAB buildUwbReference.sampled_code. ----
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    rxopt::ensure_sparse_cir_template(
        preamble_code.data(), preamble_code.size(), scratch.cir_template);
    const size_t code_len = 2 * kQm35ChipsPerSymbol; // 1016

    // ---- Coherently average the aligned SYNC windows. ----
    const size_t available = std::min(
        timing.cir_grid_repetitions ? timing.cir_grid_repetitions : timing.detected_peaks,
        profile.preamble_repetitions);
    if (available == 0 || profile.cir_skip_initial_repetitions >= available)
        return false;
    const size_t first_rep = profile.cir_skip_initial_repetitions;
    const size_t rep_count = std::min(profile.cir_repetitions,
                                      available - first_rep);
    if (rep_count == 0)
        return false;
    // Match acceleration/estimateCir.m's explicit-skip branch: begin after
    // cir_skip_initial_repetitions and consume at most cir_repetitions.
    // The runtime profile sets count=preamble-skip when all remaining SYNCs
    // should participate.

    const size_t wlen = code_len + tap_count - 1; // 1053
    scratch.corr.assign(wlen, std::complex<float>(0.0f, 0.0f));
    size_t valid = 0;
    for (size_t k = first_rep; k < first_rep + rep_count; ++k) {
        const int64_t rs = start + static_cast<int64_t>(std::llround((double)k * period));
        const int64_t lo = rs - static_cast<int64_t>(pre);
        const int64_t hi = lo + static_cast<int64_t>(wlen);
        if (lo < 0 || hi > static_cast<int64_t>(n))
            continue; // drop SYNCs clipped by the buffer edge (like MATLAB)
        rxopt::vector_accumulate_complex(
            scratch.corr.data(),
            rx + static_cast<size_t>(lo),
            wlen);
        ++valid;
    }
    if (valid == 0)
        return false;
    const float inv = 1.0f / static_cast<float>(valid);
    rxopt::vector_scale_complex(scratch.corr.data(), inv, wlen);

    // ---- CIR taps: sparse ternary template correlation (Task A2). ----
    std::vector<std::complex<float>> values;
    rxopt::correlate_cir_sparse(scratch.corr.data(), scratch.cir_template,
                                tap_count, values);
    const float nrm = std::sqrt(
        gr::uwb::core::sample_power_sum(values.data(), values.size()));
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
    out.cir_filter_taps.resize(tap_count);
    for (size_t i = 0; i < tap_count; ++i) {
        out.cir_values[i] = values[i].real(); // golden CIR is real (Q=0 channel)
        out.cir_filter_taps[i] = std::conj(values[i]);
    }
    out.cir_estimate_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_cir0)
            .count());
    out.ok = true;
    return true;
}

// Soft-chip matched-filter FIR stage consuming a precomputed CIR.
inline bool stage_softchips_from_cir(const std::complex<float>* rx,
                                     size_t n,
                                     const Qm35825Profile& profile,
                                     const TimingResult& timing,
                                     CirResult& out,
                                     DemodScratch& scratch,
                                     size_t max_chips = 0,
                                     size_t begin_chip = 0)
{
    const bool on_demand =
        (profile.rx_softchip_on_demand || profile.rx_fused_payload_cmf ||
         profile.rx_fused_phr_cmf) &&
        max_chips != 0;
    const bool extending = on_demand && begin_chip != 0;
    if (extending && !scratch.softchip.prepared)
        return false;
    if (on_demand && !extending)
        scratch.softchip.reset();

    const size_t pre = out.pre_samples;
    const size_t post = out.post_samples;
    const size_t tap_count = out.cir_filter_taps.size();
    const double period = timing.measured_period;
    const int64_t start = timing.preamble_start_sample;
    if (!out.ok || period <= 0.0 || start < 0 || tap_count == 0 ||
        tap_count != pre + post)
        return false;

    auto t_fir0 = std::chrono::steady_clock::now();

    // ---- Soft-chip matched filter on the chip grid. ----
    const double spc = period / static_cast<double>(kQm35ChipsPerSymbol);
    size_t n_chips = rxopt::legacy_chip_budget(profile);
    if (on_demand) {
        if (max_chips < 2)
            return false;
        if (n_chips > max_chips)
            n_chips = max_chips;
    }

    const int64_t chip_start = start + static_cast<int64_t>(tap_count) -
                               static_cast<int64_t>(pre) - 1; // = start + post - 1
    const int64_t last_by_budget =
        chip_start + static_cast<int64_t>(std::ceil((double)(n_chips - 1) * spc));
    const int64_t last_chip_sample = std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
    const int64_t last_pos = last_chip_sample - 1; // inclusive, 0-based
    if (last_pos < chip_start)
        return false;
    const int64_t spc_i = std::max<int64_t>(1, std::llround(spc));
    // Integer grid unless the clock offset would drift the grid past the
    // correlation tolerance across the longest frame (see integer_chip_grid).
    const bool fractional_grid =
        !integer_chip_grid(spc, profile.fused_gate_ppm * 1e-6);
    const size_t num_chips =
        static_cast<size_t>(std::floor(
            static_cast<double>(last_pos - chip_start) / spc)) + 1;
    if (on_demand &&
        num_chips > static_cast<size_t>(std::numeric_limits<uint32_t>::max()))
        return false; // result chip counters are uint32_t

    if (extending && num_chips <= begin_chip) {
        // The PHR-derived range already fits in the prefix.  Nothing to do;
        // the existing normalized prefix remains the final stream.
        out.soft_chip_count = begin_chip;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(begin_chip);
        out.on_demand_extended = 0;
        out.samples_per_chip = spc;
        out.ok = true;
        return true;
    }

    // Causal FIR on the chip grid, rewritten for contiguous forward access:
    //   chip = sum_k conj(values[T-1-k]) * rx[base-k]
    //        = sum_q conj(values[q])     * rx[base-(T-1)+q]
    // Pre-conjugated taps h[q]=conj(values[q]) once outside the chip loop.
    // Kernels: multi-acc / VOLK / fixed-38 AVX2 (see uwb_cir_fir_simd.h).
    //
    // On-demand uses a separate output buffer: the prefix must survive the
    // extension call's CIR re-estimate (which temporarily uses scratch.corr).
    std::vector<std::complex<float>>& fir_out =
        on_demand ? scratch.softchip_corr : scratch.corr;
    fir_out.resize(num_chips);
    const int64_t fwd = static_cast<int64_t>(tap_count) - 1;

    // Stack buffers — no heap alloc in the FIR hot path.
    // h_conj[q] = conj(values[q]) for multi-acc / AVX2; values_nat for VOLK.
    alignas(32) std::complex<float> h_conj[64];
    alignas(32) std::complex<float> values_nat[64];
    if (tap_count > 64)
        return false;
    for (size_t q = 0; q < tap_count; ++q) {
        h_conj[q] = out.cir_filter_taps[q];
        values_nat[q] = std::conj(h_conj[q]);
    }
    size_t peak_idx = 0;
    float peak_mag = -1.0f;
    for (size_t q = 0; q < tap_count; ++q) {
        const float m = std::abs(values_nat[q]);
        if (m > peak_mag) {
            peak_mag = m;
            peak_idx = q;
        }
    }
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    const std::vector<int8_t>& spread = scratch.cached_seq.sampled_pcode;

    // Optional sparse RAKE: choose the strongest complex CIR taps once per
    // packet. Keep indices in ascending delay order for predictable loads;
    // weights retain their estimated complex phases for coherent combining.
    alignas(32) uint8_t rake_indices[64];
    alignas(32) std::complex<float> rake_weights[64];
    const bool bypass_filter =
        profile.cir_soft_chip_mode == CirSoftChipMode::Bypass;
    const bool use_rake =
        profile.cir_soft_chip_mode == CirSoftChipMode::Rake ||
        (profile.cir_soft_chip_mode == CirSoftChipMode::Auto &&
         profile.cir_rake_top_k > 0);
    const size_t rake_k = use_rake
        ? std::min(profile.cir_rake_top_k, tap_count)
        : size_t(0);
    if (rake_k > 0 && rake_k < tap_count) {
        std::array<uint8_t, 64> order{};
        for (size_t q = 0; q < tap_count; ++q)
            order[q] = static_cast<uint8_t>(q);
        std::partial_sort(order.begin(), order.begin() + rake_k,
                          order.begin() + tap_count,
                          [&](uint8_t a, uint8_t b) {
                              return std::norm(values_nat[a]) > std::norm(values_nat[b]);
                          });
        std::sort(order.begin(), order.begin() + rake_k);
        for (size_t i = 0; i < rake_k; ++i) {
            rake_indices[i] = order[i];
            rake_weights[i] = h_conj[order[i]];
        }
    }
    // The on-demand experiment is limited to the production integer-grid,
    // full-tap kernel.  Fractional SFO grids, sparse RAKE and bypass keep the
    // complete legacy path, so their semantics are not changed or rounded.
    if (on_demand && (fractional_grid || bypass_filter || rake_k > 0))
        return false;

    // Only NS-SFD still consumes a soft-chip stream on the fused PHR path.
    // Keep its -8-chip search halo, not a filtered copy of every SYNC.
    const bool skip_sync_fir = on_demand && !extending &&
        profile.rx_fused_phr_cmf && profile.rx_fused_payload_cmf && spc_i == 2;
    const size_t sync_end = profile.preamble_repetitions * kQm35ChipsPerSymbol;
    const size_t fir_first = skip_sync_fir && sync_end > 8 ? sync_end - 8 : 0;
    if (fir_first >= num_chips)
        return false;
    const bool sparse_rake = rake_k > 0 && rake_k < tap_count;
    auto clipped_chip = [&](int64_t p) {
        const int64_t window_start = p - fwd;
        if (bypass_filter) {
            const int64_t idx = window_start + static_cast<int64_t>(peak_idx);
            return idx >= 0 && idx < static_cast<int64_t>(n)
                ? rx[static_cast<size_t>(idx)] : std::complex<float>{};
        }
        return cir_fir::dot_clipped(rx, n, window_start, h_conj,
                                    sparse_rake ? rake_k : tap_count,
                                    sparse_rake ? rake_indices : nullptr);
    };
    size_t checked = 0;
    if (fractional_grid) {
        // MATLAB keeps measured_period/chips_per_symbol as a fractional
        // position and linearly interpolates the CIR-filtered stream.  Keep
        // the integer positions as FIR inputs and interpolate the two outputs.
        auto dot_at = [&](int64_t p) {
            if (bypass_filter)
                return clipped_chip(p);
            if (p >= fwd && p < static_cast<int64_t>(n)) {
                const auto* window = rx + (p - fwd);
                if (sparse_rake)
                    return cir_fir::dot_topk(window, rake_indices, rake_weights, rake_k);
                return cir_fir::dot(cir_fir::kDefaultKernel, window, h_conj,
                                    values_nat, tap_count);
            }
            return clipped_chip(p);
        };
        for (size_t i = 0; i < num_chips; ++i) {
            const double pd = static_cast<double>(chip_start) +
                              static_cast<double>(i) * spc;
            const int64_t p0 = static_cast<int64_t>(std::floor(pd));
            const float frac = static_cast<float>(pd - static_cast<double>(p0));
            const std::complex<float> y0 = dot_at(p0);
            const std::complex<float> y1 = dot_at(p0 + 1);
            fir_out[i] = y0 + frac * (y1 - y0);
        }
        checked = num_chips; // fractional path completed the whole stream
    } else if (chip_start < fwd) {
        const int64_t gap = fwd - chip_start;
        checked = std::min(num_chips,
                           static_cast<size_t>((gap + spc_i - 1) / spc_i));
    }
    if (extending && begin_chip < checked)
        return false; // [begin_chip, checked) would never be generated
    if (extending)
        checked = std::min(num_chips, begin_chip);
    // Edge chips: FIR window may clip below sample 0 (bounds-checked scalar).
    // A prefix call handles them once; an extension starts after them.
    for (size_t i = extending ? checked : 0;
         !fractional_grid && !skip_sync_fir && i < checked; ++i) {
        const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
        fir_out[i] = clipped_chip(p);
    }
    if (skip_sync_fir)
        checked = std::max(checked, fir_first);
    out.soft_fir_first_chip = static_cast<uint32_t>(checked);
    out.soft_fir_computed_chips = static_cast<uint32_t>(
        num_chips - (skip_sync_fir ? fir_first : (extending ? begin_chip : 0)));
    // Main body: full tap window is in-bounds.  Call the selected kernel
    // directly (no per-chip switch) so the fixed-38 AVX2 path fully inlines.
    {
        const cir_fir::Kernel fir_k = cir_fir::kDefaultKernel;
        const std::complex<float>* rx0 = rx;
        if (bypass_filter) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p =
                    chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t idx = static_cast<size_t>(
                    p - fwd + static_cast<int64_t>(peak_idx));
                fir_out[i] = rx0[idx];
            }
        } else if (rake_k > 0 && rake_k < tap_count) {
            size_t i = checked;
#if UWB_CIR_FIR_HAVE_AVX2
            // Fixed Top-4/Top-8 fast path: vectorize across four outputs. The
            // load helper reads through max_q+7; keep the final boundary on
            // the scalar path when that speculative contiguous load is unsafe.
            if ((rake_k == 4 || rake_k == 8) && spc_i == 2) {
                const size_t max_q = rake_indices[rake_k - 1];
                for (; i + 4 <= num_chips; i += 4) {
                    const int64_t p =
                        chip_start + static_cast<int64_t>(i) * spc_i;
                    const size_t win = static_cast<size_t>(p - fwd);
                    if (win + max_q + 7 >= n)
                        break;
                    if (rake_k == 4)
                        cir_fir::dot_topk_x4_avx2<4>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                    else
                        cir_fir::dot_topk_x4_avx2<8>(
                            rx0 + win, rake_indices, rake_weights,
                            fir_out.data() + i);
                }
            }
#endif
            for (; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] = cir_fir::dot_topk(
                    rx0 + win, rake_indices, rake_weights, rake_k);
            }
        } else if (fir_k == cir_fir::Kernel::Avx2Fixed &&
            tap_count == cir_fir::kDefaultTapCount) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_avx2_fixed38(rx0 + win, h_conj, tap_count);
            }
        } else if (fir_k == cir_fir::Kernel::Volk) {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_volk(rx0 + win, values_nat, tap_count);
            }
        } else {
            for (size_t i = checked; i < num_chips; ++i) {
                const int64_t p = chip_start + static_cast<int64_t>(i) * spc_i;
                const size_t win = static_cast<size_t>(p - fwd);
                fir_out[i] =
                    cir_fir::dot_multi_acc8(rx0 + win, h_conj, tap_count);
            }
        }
    }
    out.soft_fir_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_fir0)
            .count());
    auto t_post0 = std::chrono::steady_clock::now();

    // ---- Phase-align against the last min(32, preamble) SYNCs' spread code. ----
    const size_t phase_reps = std::min<size_t>(32, profile.preamble_repetitions);
    const size_t phase_first =
        (profile.preamble_repetitions - phase_reps) * kQm35ChipsPerSymbol;
    const size_t phase_len = phase_reps * kQm35ChipsPerSymbol;
    if (phase_first + phase_len > num_chips)
        return false; // chip stream shorter than the configured preamble
    // spread[] is real ±1/0, so conj(spread)*corr = spread*corr.  Avoid % by
    // walking the 508-chip period with a running index.
    std::complex<float> rot(1.0f, 0.0f);
    if (extending) {
        // Frozen by the prefix pass: phase alignment runs once per packet.
        rot = scratch.softchip.rot;
    } else if (skip_sync_fir) {
        const auto phase_begin = std::chrono::steady_clock::now();
        std::complex<float> gain{};
        if (!rxopt::sync_phase_gain(rx, n,
                chip_start - fwd + static_cast<int64_t>(2 * phase_first),
                h_conj, tap_count, spread, phase_reps, scratch.work, gain))
            return false;
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        scratch.softchip.rot = rot;
        out.sync_phase_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - phase_begin).count());
    } else {
        std::complex<float> gain(0.0f, 0.0f);
        size_t s = 0;
        for (size_t i = 0; i < phase_len; ++i) {
            gain += static_cast<float>(spread[s]) *
                    fir_out[phase_first + i];
            if (++s == kQm35ChipsPerSymbol)
                s = 0;
        }
        const float ang = std::arg(gain);
        rot = std::complex<float>(std::cos(-ang), std::sin(-ang));
        if (on_demand)
            scratch.softchip.rot = rot;
    }

    // Merged postprocess: compute only the rotated real component and track
    // max|real|, then normalize into soft_chips. No later stage consumes the
    // rotated complex corr stream, so writing it back wastes ~1.2 MB/packet.
    scratch.soft_chips.resize(num_chips);
    if (!extending) {
        // Preserve absolute chip coordinates; the skipped prefix is not data.
        // NS-SFD uses normalized correlation, invariant to this common scale.
        const size_t post_first = skip_sync_fir ? fir_first : 0;
        std::fill(scratch.soft_chips.begin(),
                  scratch.soft_chips.begin() + post_first, 0.0f);
        const float mx = rxopt::project_soft_chips(
            fir_out.data() + post_first, scratch.soft_chips.data() + post_first,
            num_chips - post_first, rot);
        const float inv_denom = 1.0f / (mx + 1e-12f);
        for (size_t i = post_first; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        if (on_demand) {
            scratch.softchip.prefix_max = mx;
            scratch.softchip.generated = num_chips;
            scratch.softchip.prepared = true;
            out.initial_chip_count = static_cast<uint32_t>(num_chips);
            out.extended_chip_count = 0;
            out.on_demand_extended = 0;
        }
    } else {
        // Extension: rotate only the new chips, then put the whole stream on
        // one denominator.  rescale_prefix() multiplies the already-normalized
        // prefix by (prefix_max+eps)/(final_max+eps); the new tail is scaled by
        // 1/(final_max+eps).  This equals the one-shot raw/final_max result up
        // to one float rounding on the prefix.
        const float ext_mx = rxopt::project_soft_chips(
            fir_out.data() + begin_chip, scratch.soft_chips.data() + begin_chip,
            num_chips - begin_chip, rot);
        const float final_max = std::max(scratch.softchip.prefix_max, ext_mx);
        rxopt::rescale_prefix(scratch.soft_chips, begin_chip,
                              scratch.softchip.prefix_max, final_max);
        const float inv_denom = 1.0f / (final_max + 1e-12f);
        for (size_t i = begin_chip; i < num_chips; ++i)
            scratch.soft_chips[i] *= inv_denom;
        scratch.softchip.prefix_max = final_max;
        scratch.softchip.generated = num_chips;
        out.initial_chip_count = static_cast<uint32_t>(begin_chip);
        out.extended_chip_count = static_cast<uint32_t>(num_chips);
        out.on_demand_extended = 1;
    }
    out.postprocess_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t_post0)
            .count());

    out.soft_chip_count = num_chips;
    out.samples_per_chip = spc;
    out.ok = true;
    return true;
}

inline bool stage_cir_softchips(const std::complex<float>* rx,
                                size_t n,
                                const Qm35825Profile& profile,
                                const TimingResult& timing,
                                const std::vector<int8_t>& preamble_code,
                                CirResult& out,
                                DemodScratch& scratch,
                                size_t max_chips = 0,
                                size_t begin_chip = 0)
{
    if (!estimate_cir(rx, n, profile, timing, preamble_code, out, scratch))
        return false;
    return stage_softchips_from_cir(rx, n, profile, timing, out, scratch,
                                    max_chips, begin_chip);
}


} // namespace gr::uwb::demod::core
