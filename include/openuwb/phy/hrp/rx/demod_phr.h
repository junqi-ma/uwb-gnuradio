// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_fec.h>
#include <openuwb/phy/hrp/rx/demod_scratch.h>
#include <openuwb/phy/hrp/rx/demod_grid.h>
#include <openuwb/phy/hrp/rx/rx_softchip_opt.h>

namespace openuwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 5 — NS-SFD location in the soft-chip stream (R3).
// Matches MATLAB locateNsSfd: spread = kron(sfd_sequence, spread_code), search
// a +/-8-chip window around expected = preamble_repetitions*chips_per_symbol
// for the best normalized correlation.  Golden: start_chip=32512,
// end_chip=36575, polarity=1, correlation=1.0 (chip coordinates, 0-based).
// ---------------------------------------------------------------------------
inline bool stage_ns_sfd(const std::vector<float>& soft_chips,
                         const Qm35825Profile& profile,
                         const std::vector<int8_t>& sfd_sequence,
                         size_t chips_per_symbol,
                         NsSfdResult& out,
                         DemodScratch& scratch)
{
    out = NsSfdResult{};
    if (soft_chips.empty() || sfd_sequence.empty() || chips_per_symbol == 0)
        return false;

    // spread = kron(sfd_sequence, spread_code); both real ternary -> real.
    const char* cur_mode = profile.sfd_mode ? profile.sfd_mode : "";
    if (scratch.cached_seq.code_index != profile.code_index ||
        scratch.cached_seq.sfd_mode != cur_mode ||
        scratch.cached_seq.sampled_pcode.empty()) {
        scratch.cached_seq.update(profile.code_index, profile.sfd_mode);
    }
    if (scratch.cached_seq.sfd_seq != sfd_sequence ||
        scratch.cached_seq.ns_sfd_sparse.empty()) {
        scratch.cached_seq.sfd_seq = sfd_sequence;
        scratch.cached_seq.ns_sfd_sparse.clear();
        const std::vector<int8_t>& sc = scratch.cached_seq.sampled_pcode;
        scratch.cached_seq.ns_sfd_total_chips = sfd_sequence.size() * sc.size();
        for (size_t i = 0; i < sfd_sequence.size(); ++i) {
            const int8_t si = sfd_sequence[i];
            if (si == 0) continue;
            for (size_t j = 0; j < sc.size(); ++j) {
                const int8_t pj = sc[j];
                if (pj == 0) continue;
                const int8_t val = static_cast<int8_t>(si * pj);
                scratch.cached_seq.ns_sfd_sparse.push_back({ static_cast<uint32_t>(i * sc.size() + j), val });
            }
        }
        scratch.cached_seq.ns_sfd_norm = std::sqrt(static_cast<float>(scratch.cached_seq.ns_sfd_sparse.size()));
    }

    const size_t sfd_len = sfd_sequence.size() * chips_per_symbol;
    const int64_t expected =
        static_cast<int64_t>(profile.preamble_repetitions) *
        static_cast<int64_t>(chips_per_symbol);
    const int64_t half = 8;
    const int64_t lo = std::max<int64_t>(0, expected - half);
    const int64_t hi = std::min<int64_t>(
        static_cast<int64_t>(soft_chips.size()) - static_cast<int64_t>(sfd_len),
        expected + half);
    if (hi < lo)
        return false;

    float best = -1.0f;
    int64_t best_k = lo;
    int pol = 1;

    if (chips_per_symbol == scratch.cached_seq.sampled_pcode.size() &&
        scratch.cached_seq.ns_sfd_total_chips == sfd_len &&
        !scratch.cached_seq.ns_sfd_sparse.empty()) {
        const float spread_norm = scratch.cached_seq.ns_sfd_norm;
        double win_energy = 0.0;
        for (size_t i = 0; i < sfd_len; ++i) {
            const double v = static_cast<double>(
                soft_chips[static_cast<size_t>(lo + static_cast<int64_t>(i))]);
            win_energy += v * v;
        }
        if (!std::isfinite(win_energy) || win_energy < 0.0)
            return false;

        for (int64_t k = lo; k <= hi; ++k) {
            if (k > lo) {
                const double v_in = static_cast<double>(
                    soft_chips[static_cast<size_t>(k + static_cast<int64_t>(sfd_len) - 1)]);
                const double v_out = static_cast<double>(
                    soft_chips[static_cast<size_t>(k - 1)]);
                win_energy += v_in * v_in - v_out * v_out;
                if (!std::isfinite(win_energy))
                    return false;
            }
            const float seg_norm = std::sqrt(std::max(0.0, win_energy));
            float corr = 0.0f;
            for (const auto& entry : scratch.cached_seq.ns_sfd_sparse) {
                const float v = soft_chips[static_cast<size_t>(k + static_cast<int64_t>(entry.offset))];
                // The sparse sign pattern is fixed across the 17 lag trials, so
                // the branch predictor learns it; add/sub is cheaper than the
                // branchless `sign * v` multiply.
                if (entry.sign > 0)
                    corr += v;
                else
                    corr -= v;
            }
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    } else {
        const std::vector<int8_t>& spread_code = scratch.cached_seq.sampled_pcode;
        scratch.work.assign(sfd_len, std::complex<float>(0.0f, 0.0f));
        for (size_t i = 0; i < sfd_sequence.size(); ++i)
            for (size_t j = 0; j < spread_code.size(); ++j)
                scratch.work[i * spread_code.size() + j] = std::complex<float>(
                    static_cast<float>(sfd_sequence[i] * spread_code[j]), 0.0f);
        float spread_norm = 0.0f;
        for (size_t i = 0; i < sfd_len; ++i)
            spread_norm += std::norm(scratch.work[i]);
        spread_norm = std::sqrt(spread_norm);

        for (int64_t k = lo; k <= hi; ++k) {
            float seg_norm = 0.0f, corr = 0.0f;
            for (size_t i = 0; i < sfd_len; ++i) {
                const float v = soft_chips[static_cast<size_t>(k + (int64_t)i)];
                seg_norm += v * v;
                corr += scratch.work[i].real() * v;
            }
            seg_norm = std::sqrt(seg_norm);
            const float m = std::abs(corr) / (spread_norm * seg_norm + 1e-12f);
            if (m > best) {
                best = m;
                best_k = k;
                pol = (corr >= 0.0f) ? 1 : -1;
            }
        }
    }

    if (best < profile.sfd_detection_threshold)
        return false;

    out.ok = true;
    out.sfd_start_chip = best_k;
    out.sfd_end_chip = best_k + static_cast<int64_t>(sfd_len) - 1;
    out.polarity = pol;
    out.metric = best;
    return true;
}

// ---------------------------------------------------------------------------
// Stage 6 — BPRF PHR demod + convolutional decode + SECDED (R3).
// Matches helperUWBBPRFDemod + helperUWBConvDec + helperUWBPHRDecode:
//   * 21 BPM-BPSK symbols (0.85 Mbps PHR: 64 chips/burst, 512 chips/symbol),
//     de-spread with the scrambler LFSR (offset 0).
//   * rate-1/2 CL-3 Viterbi (trunc), keep the 19 PHR bits.
//   * SECDED single-bit correction / double-bit detection -> PSDU length.
// Golden: coded/decoded PHR match MATLAB, psdu_length=127, secded_pass=1.
// ---------------------------------------------------------------------------
// Shared PHR SECDED/parser tail.  The legacy stage_phr(), the fused PHR kernel
// and the soft-decision variants all feed their 21 decoded codeword bits here,
// so the FEC convention and the SECDED tail-energy guard are identical and are
// implemented exactly once.  `soft_chips` is only read by the guard when a
// corrected length grows; callers with a short prefix therefore keep the same
// conservative "return the corrected result and let the caller retry the full
// stream once" behaviour as the existing on-demand path.
// ---------------------------------------------------------------------------
namespace detail {

inline bool phr_secded_parse(const std::vector<int8_t>& decoded,
                             const std::vector<float>& soft_chips,
                             const NsSfdResult& ns_sfd,
                             PhrResult& out)
{
    out = PhrResult{};
    if (decoded.size() < 19)
        return false;

    std::vector<int8_t> sys13(decoded.begin(), decoded.begin() + 13);
    std::vector<int8_t> recv_par(decoded.begin() + 13, decoded.begin() + 19);
    std::vector<int8_t> phr19;
    detail::hrp_secded(sys13, phr19);
    int8_t synd[6];
    for (int i = 0; i < 6; ++i)
        synd[i] = static_cast<int8_t>(recv_par[i] ^ phr19[13 + i]);

    bool secded_pass = true;
    int idx = 0;
    for (int i = 1; i < 6; ++i)
        idx |= (synd[i] << (i - 1));
    if (idx > 0) {
        static const int kAddresses[13] = { 3, 5, 6, 7, 9, 10, 11,
                                            12, 13, 14, 15, 17, 18 };
        for (int j = 0; j < 13; ++j) {
            if (kAddresses[j] == idx) {
                phr19[j] ^= 1;
                out.secded_corrected = true;
                break;
            }
        }
        if (!synd[0])
            secded_pass = false; // DED: second error cannot be corrected
    }
    out.secded_uncorrectable = !secded_pass;

    // MATLAB bit2int default is MSB-first (bit2int(x,n)); phr(1:2) -> data
    // rate, phr(3:9) -> PSDU length.
    //
    // Guard against SECDED mis-correction on real RF.  A false single-bit
    // flip can inflate PSDU length (e.g. 13 -> 45 on DW1000 captures) past
    // the RF energy while the soft-chip *buffer* is still long enough.  If
    // the corrected length needs a long tail whose soft-chip energy is near
    // the noise floor relative to the uncorrected body, keep uncorrected.
    auto length_of = [](const std::vector<int8_t>& b) -> uint32_t {
        uint32_t psdu = 0;
        for (int i = 0; i < 7; ++i)
            psdu |= (static_cast<uint32_t>(b[2 + i]) << (6 - i));
        return psdu;
    };
    const uint32_t len_uncorr = length_of(sys13); // sys13 is never flipped
    const uint32_t len_corr = length_of(phr19);
    if (out.secded_corrected && len_corr > len_uncorr &&
        ns_sfd.sfd_end_chip >= 0) {
        const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
        const size_t need_c = rxopt::payload_chips_needed(len_corr);
        const size_t need_u = rxopt::payload_chips_needed(len_uncorr);
        if (phr_start >= 0 && need_c > need_u + 64) {
            const size_t base = static_cast<size_t>(phr_start);
            const size_t end_u = base + need_u;
            const size_t end_c = base + need_c;
            if (end_c <= soft_chips.size() && end_u < soft_chips.size()) {
                double e_body = 0.0, e_tail = 0.0;
                for (size_t i = base; i < end_u; ++i) {
                    const double v = soft_chips[i];
                    e_body += v * v;
                }
                for (size_t i = end_u; i < end_c; ++i) {
                    const double v = soft_chips[i];
                    e_tail += v * v;
                }
                const double n_body = static_cast<double>(end_u - base);
                const double n_tail = static_cast<double>(end_c - end_u);
                const double mean_body = e_body / (n_body + 1e-12);
                const double mean_tail = e_tail / (n_tail + 1e-12);
                // Tail is noise-like if its mean energy is << body.
                if (mean_tail < 0.15 * mean_body) {
                    for (int i = 0; i < 13; ++i)
                        phr19[i] = sys13[i];
                    out.secded_corrected = false;
                }
            }
        }
    }

    const int dr = (phr19[0] << 1) | phr19[1];
    static const float kDataRates[4] = { 0.11f, 0.85f, 6.81f, 27.24f };
    out.data_rate_mbps = kDataRates[dr & 3];
    out.psdu_length = length_of(phr19);
    out.phr_bits.reserve(19);
    for (int i = 0; i < 19; ++i)
        out.phr_bits.push_back(static_cast<uint8_t>(phr19[i]));
    out.ok = true;
    return true;
}

} // namespace detail

// Hard-decision PHR entry point: 21 raw g0/g1 pairs -> CL-3 Viterbi -> the
// shared SECDED/parser tail.
inline bool decode_phr_decisions(const std::vector<int8_t>& g0,
                                 const std::vector<int8_t>& g1,
                                 const std::vector<float>& soft_chips,
                                 const NsSfdResult& ns_sfd,
                                 PhrResult& out)
{
    out = PhrResult{};
    const size_t nsym = 21;
    if (g0.size() != nsym || g1.size() != nsym)
        return false;

    std::vector<int8_t> rx(2 * nsym);
    for (size_t s = 0; s < nsym; ++s) {
        rx[2 * s] = g0[s];
        rx[2 * s + 1] = g1[s];
    }
    std::vector<int8_t> decoded;
    detail::vitdec_cl3(rx.data(), nsym, decoded);
    return detail::phr_secded_parse(decoded, soft_chips, ns_sfd, out);
}

// Soft-decision PHR entry point: the same 21 codeword symbols as per-symbol
// LLRs (positive favours bit 0, exactly the bprf_demod_soft()/
// rxfusion::decide_bpm_soft() convention) -> soft CL-3 Viterbi -> the same
// shared SECDED/parser tail.  Nothing else differs from the hard path.
inline bool decode_phr_decisions_soft(const std::vector<float>& llr0,
                                      const std::vector<float>& llr1,
                                      const std::vector<float>& soft_chips,
                                      const NsSfdResult& ns_sfd,
                                      PhrResult& out)
{
    out = PhrResult{};
    const size_t nsym = 21;
    if (llr0.size() != nsym || llr1.size() != nsym)
        return false;

    std::vector<int8_t> decoded;
    detail::vitdec_cl3_soft(llr0.data(), llr1.data(), nsym, decoded);
    return detail::phr_secded_parse(decoded, soft_chips, ns_sfd, out);
}

inline bool stage_phr(const std::vector<float>& soft_chips,
                      const Qm35825Profile& profile,
                      const NsSfdResult& ns_sfd,
                      size_t chips_per_symbol,
                      PhrResult& out,
                      DemodScratch& scratch)
{
    out = PhrResult{};
    if (soft_chips.empty() || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const size_t cpb = 64, cps = 512, nsym = 21;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start + static_cast<int64_t>(cps * nsym)) >
            soft_chips.size())
        return false;
    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;

    // Soft-decision PHR: bprf_demod_soft() produces the same hard decisions as
    // bprf_demod() plus the per-symbol LLRs, and the soft CL-3 Viterbi is
    // strictly better informed than the hard one.  The SECDED/parse tail is
    // shared, so the PHR bit/length/rate contract is unchanged.
    std::vector<int8_t> g0, g1;
    std::vector<float> llr0, llr1;
    if (!detail::bprf_demod_soft(soft_chips, phr_start, profile.code_index,
                                 nsym, cpb, cps, 0, pol, g0, g1, llr0, llr1))
        return false;
    return decode_phr_decisions_soft(llr0, llr1, soft_chips, ns_sfd, out);
}

// Build (or reuse) the PHR spreading stream.  The PN depends only on the code
// index and the fixed 21-symbol geometry; the channel template is rebuilt per
// packet by stage_phr_fused_cmf().
inline bool prepare_fused_phr_pn(DemodScratch& scratch, size_t code_index)
{
    const size_t nbits =
        rxfusion::kPhrSymbols * rxfusion::kPhrChipsPerBurst;
    if (scratch.fused_phr.code_index == code_index &&
        scratch.fused_phr.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused_phr.pn, code_index,
                                rxfusion::kPhrScramOffset, nbits)) {
        scratch.fused_phr.code_index = 0;
        scratch.fused_phr.pn.clear();
        scratch.fused_phr.reset_decisions();
        return false;
    }
    scratch.fused_phr.code_index = code_index;
    scratch.fused_phr.reset_decisions();
    return true;
}

// Experimental fused PHR metrics + the shared PHR FEC/SECDED parser.  The PHR
// field is generated directly from CFO-compensated IQ with 164-point templates
// (38 taps), so no PHR soft-chip FIR is required.  The 21 raw g0/g1 decisions
// are cached for the payload joint Viterbi.  Returns false on unsupported
// geometry, range violations, non-finite metrics or decode failure; the caller
// retries the complete legacy stream once.  Timings are always reported when
// the corresponding work was performed.
inline bool stage_phr_fused_cmf(const std::complex<float>* rx,
                                size_t n,
                                const Qm35825Profile& profile,
                                const TimingResult& timing,
                                const CirResult& cir,
                                const std::vector<float>& prefix_soft_chips,
                                const NsSfdResult& ns_sfd,
                                PhrResult& out,
                                DemodScratch& scratch,
                                uint64_t& template_us,
                                uint64_t& correlate_us,
                                uint64_t& decode_us,
                                uint64_t& decode_ns,
                                size_t& symbols_out)
{
    out = PhrResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !ns_sfd.ok || ns_sfd.sfd_end_chip < 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0 ||
        static_cast<size_t>(phr_start) > prefix_soft_chips.size())
        return false;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    if (!profile.rx_fused_sfo_track &&
        !integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    // Under SFO the chip grid accumulates (spc - 2) samples per chip, so the
    // PHR window origin computed on the nominal 2-sample grid is off by
    // phr_start*(spc-2) samples.  Shift the integer origin by the rounded
    // drift; the residual (< 0.5 sample) is well inside the 64-chip PHR burst
    // tolerance.  The payload uses the exact fractional bank instead.
    int64_t phr_chip_start = chip_start;
    if (profile.rx_fused_sfo_track && cir.samples_per_chip > 0.0) {
        const double drift =
            static_cast<double>(phr_start) * (cir.samples_per_chip - 2.0);
        phr_chip_start += static_cast<int64_t>(std::llround(drift));
    }
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;

    const auto t_plan0 = std::chrono::steady_clock::now();
    if (!prepare_fused_phr_pn(scratch, profile.code_index))
        return false;
    const auto t_plan1 = std::chrono::steady_clock::now();
    const uint64_t plan_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 - t_plan0)
            .count());

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    const double spc_phr =
        (cir.samples_per_chip > 0.0) ? cir.samples_per_chip
                                     : static_cast<double>(rxfusion::kSamplesPerChip);
    rxfusion::Timings timings;
    rxfusion::Metrics2& pm = scratch.fused_phr.metrics2;
    if (profile.rx_bpm_known_hop) {
        // Known-hop 2-candidate PHR: the hop comes from pn[64*s], so only the
        // two BPM halves are correlated.  z0/z1 are raw complex values on the
        // same phase basis as Metrics::complex_value, ready for the pilot
        // refinement below.
        if (!rxfusion::correlate_phr_known_hop(
                scratch.derotated.data(), n, cir.cir_filter_taps.data(),
                tap_count, phr_chip_start, rxfusion::kSamplesPerChip, phr_start,
                scratch.fused_phr.pn.data(), pol, scratch.softchip.rot, pm,
                scratch.fused_phr.kernel, &timings))
            return false;
        scratch.fused_phr.metrics2_valid = true;
    } else {
        if (!rxfusion::correlate_phr(
                scratch.derotated.data(), n, cir.cir_filter_taps.data(),
                tap_count, phr_chip_start, rxfusion::kSamplesPerChip, phr_start,
                scratch.fused_phr.pn.data(), pol, scratch.softchip.rot,
                scratch.fused_phr.metrics, scratch.fused_phr.kernel,
                &timings))
            return false;
        for (float v : scratch.fused_phr.metrics.signed_value) {
            if (!std::isfinite(v))
                return false;
        }
        // Project the four candidates onto the two known-hop metrics by
        // picking the hop with the larger half energy.  The 21 raw complex
        // values are kept for the pilot refinement.
        pm.reset(rxfusion::kPhrSymbols, true);
        const auto& sv = scratch.fused_phr.metrics.signed_value;
        const auto& cv = scratch.fused_phr.metrics.complex_value;
        for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
            const float h0 = std::max(std::abs(sv[4 * s + 0]),
                                      std::abs(sv[4 * s + 1]));
            const float h1 = std::max(std::abs(sv[4 * s + 2]),
                                      std::abs(sv[4 * s + 3]));
            const size_t h = (h1 > h0) ? 1 : 0;
            pm.m0[s] = sv[4 * s + 2 * h + 0];
            pm.m1[s] = sv[4 * s + 2 * h + 1];
            pm.z0[s] = cv[4 * s + 2 * h + 0];
            pm.z1[s] = cv[4 * s + 2 * h + 1];
        }
        scratch.fused_phr.metrics2_valid = true;
    }
    template_us = plan_us + timings.template_update_us;
    correlate_us = timings.correlate_us;

    // Decisions and LLRs from the two known-hop metrics.
    auto& pg0 = scratch.fused_phr.g0;
    auto& pg1 = scratch.fused_phr.g1;
    auto& pll0 = scratch.fused_phr.llr0;
    auto& pll1 = scratch.fused_phr.llr1;
    pg0.resize(rxfusion::kPhrSymbols);
    pg1.resize(rxfusion::kPhrSymbols);
    pll0.resize(rxfusion::kPhrSymbols);
    pll1.resize(rxfusion::kPhrSymbols);
    for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
        const float a = pm.m0[s];
        const float b = pm.m1[s];
        pg0[s] = (std::abs(a) >= std::abs(b)) ? 0 : 1;
        const float best = (pg0[s] == 0) ? a : b;
        pg1[s] = (best < 0.0f) ? 1 : 0;
        pll0[s] = std::abs(a) - std::abs(b);
        pll1[s] = std::max(a, b) - std::max(-a, -b);
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    const bool decoded =
        profile.rx_bpm_joint_ml
            ? detail::decode_phr_biorth_soft(
                  pm.m0.data(), pm.m1.data(), prefix_soft_chips, ns_sfd, out)
            : decode_phr_decisions_soft(pll0, pll1, prefix_soft_chips, ns_sfd,
                                        out);

    // PHR pilot refinement: only after a valid PHR decode, and only when the
    // 21 winning complex pilot values are available.  A failed coherence gate
    // leaves `applied=false` and the payload keeps the original rotation.
    if (decoded && profile.rx_phr_phase_refine &&
        scratch.fused_phr.metrics2_valid) {
        auto& z0 = pm.z0;
        auto& z1 = pm.z1;
        for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
            const std::complex<float> win =
                (pg0[s] == 0) ? z0[s] : z1[s];
            z0[s] = win; // alias the winning half into the pilot slot
        }
        const double phr_symbol_samples =
            static_cast<double>(rxfusion::kPhrChipsPerSymbol) * spc_phr;
        // Last PHR symbol window start -> payload s=0 window start is one PHR
        // symbol; the refine helper adds the centre-symbol offset itself.
        scratch.phr_refine = rxopt::refine_phase_cfo_from_phr(
            z0.data(), rxfusion::kPhrSymbols, pg1.data(), pol,
            scratch.softchip.rot, phr_symbol_samples, phr_symbol_samples);
        // Reliability gate: the lag-1 phase-difference slope is extrapolated
        // over ~11 PHR symbols, so a noisy estimate hurts marginal packets.
        // Only accept the correction when the polarity-stripped adjacent
        // products are nearly collinear, i.e. the pilot really follows one
        // phase ramp.  UWB_BPM_REFINE_PROD overrides the 0.997 default.
        double prod_re = 0.0, prod_abs = 0.0;
        for (size_t k = 0; k + 1 < rxfusion::kPhrSymbols; ++k) {
            const float s0 = static_cast<float>(
                (1 - 2 * static_cast<int>(pg1[k])) *
                static_cast<int>(pol));
            const float s1 = static_cast<float>(
                (1 - 2 * static_cast<int>(pg1[k + 1])) *
                static_cast<int>(pol));
            const std::complex<float> p = (s1 * z0[k + 1]) *
                                          std::conj(s0 * z0[k]);
            prod_re += p.real();
            prod_abs += std::abs(p);
        }
        const double prod = (prod_abs > 0.0) ? prod_re / prod_abs : 0.0;
        double prod_gate = 0.997;
        if (const char* g = std::getenv("UWB_BPM_REFINE_PROD"))
            prod_gate = std::atof(g);
        if (prod < prod_gate) {
            scratch.phr_refine.applied = false;
            scratch.phr_refine.rot_at_payload_start = scratch.softchip.rot;
            scratch.phr_refine.delta_cfo_w_rad = 0.0;
        }
        if (scratch.phr_refine.applied) {
            // Re-express the 21 PHR metrics in the refined payload phase so
            // the joint ML Viterbi sees one consistent frame.
            const std::complex<float> corr =
                std::conj(scratch.softchip.rot) *
                scratch.phr_refine.rot_at_payload_start;
            for (size_t s = 0; s < rxfusion::kPhrSymbols; ++s) {
                const std::complex<float> y0 =
                    scratch.softchip.rot * pm.z0[s];
                const std::complex<float> y1 =
                    scratch.softchip.rot * pm.z1[s];
                const float r0 = (corr * y0).real();
                const float r1 = (corr * y1).real();
                pm.m0[s] = (pol < 0) ? -r0 : r0;
                pm.m1[s] = (pol < 0) ? -r1 : r1;
            }
        }
    }

    // Only publish the cache once the decode succeeded: a failed PHR must not
    // leave decisions/LLRs that a later stage could mistake for this packet's.
    scratch.fused_phr.decisions_valid = decoded;
    decode_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - t_decode0)
            .count());
    decode_us = decode_ns / 1000;
    symbols_out = rxfusion::kPhrSymbols;
    if (!decoded) {
        scratch.fused_phr.reset_decisions();
        return false;
    }
    return true;
}

} // namespace openuwb::demod::core
