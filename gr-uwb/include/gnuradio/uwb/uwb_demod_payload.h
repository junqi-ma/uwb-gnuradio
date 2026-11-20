// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_phr.h>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// Stage 7 — payload BPM-BPSK + RS + FCS (R4).
// Mirrors helperUWBPayloadDecode + lrwpan.internal.hrpRS + ieee802154CRC16:
//   * payload field at 6.81 Mbps (8 chips/burst, 64 chips/symbol), scrambler
//     LFSR offset = 21 PHR symbols * 64 = 1344.
//   * joint rate-1/2 CL-3 Viterbi decode of [PHR | payload] codewords, then
//     slice out the RS-coded stream (drop the first 19 PHR bits + 2 tails).
//   * RS(63,55) decode (detail::rs_decode_stream) -> PSDU bits.
//   * pack bits to bytes LSB-first, compute IEEE 802.15.4 CRC-16 FCS.
// Golden: 127 payload bytes, fcs_pass=1, received==calculated==0x584b.
// ---------------------------------------------------------------------------

inline bool stage_payload_fcs(const std::vector<float>& soft_chips,
                              const Qm35825Profile& profile,
                              const PhrResult& phr,
                              const NsSfdResult& ns_sfd,
                              size_t chips_per_symbol,
                              PayloadResult& out,
                              DemodScratch& scratch,
                              PayloadSubTimers* sub = nullptr)
{
    // Diagnostic child timing of `stage_payload_us`.  `sub_guard` attributes the
    // remainder of this function to `other_us`; the named buckets are filled
    // around the exact calls that produce them.  Passing `sub == nullptr`
    // (e.g. the full-stream fallback retry, which is accounted in its own
    // bucket) disables all of it.
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->legacy_calls;
    out = PayloadResult{};
    if (soft_chips.empty() || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0)
        return false;
    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    if (phr_start < 0)
        return false;
    // The payload field begins right after the 21 PHR symbols (512 chips each).
    const int64_t payload_start = phr_start + static_cast<int64_t>(512 * 21);

    bool any_ok = false;

    // First run the MATLAB-equivalent production path: reported PHR length,
    // NS-SFD polarity, hard BPM decisions and hard truncated Viterbi.  The
    // RF experiments below (length neighborhood, opposite polarity and soft
    // Viterbi) are fallbacks; they must not hide a valid fixed-length decode
    // or replace it with a shorter accidental RS codeword.
    {
        const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
        const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        std::vector<int8_t> g0p, g1p, g0, g1;
        const auto t_soft = std::chrono::steady_clock::now();
        const bool demod_ok =
            detail::bprf_demod(soft_chips, phr_start, profile.code_index, 21,
                               64, 512, 0, pol, g0p, g1p) &&
            detail::bprf_demod(soft_chips, payload_start, profile.code_index,
                               nsym, 8, 64, 1344, pol, g0, g1);
        if (sub)
            sub->soft_us += detail::payload_elapsed_us(t_soft);
        if (demod_ok) {
            if (detail::decode_payload_hard(g0p, g1p, g0, g1, psdu_bits, out,
                                            sub)) {
                any_ok = true;
                if (out.fcs_pass)
                    return true;
            }
        }
    }

    // Candidate PSDU lengths: reported PHR length, then a small neighborhood
    // (±2) that covers common SECDED single-bit length slips on real RF.
    uint32_t lens[5];
    size_t n_lens = 0;
    auto add_len = [&](uint32_t L) {
        if (L == 0 || L > profile.max_psdu_bytes)
            return;
        for (size_t i = 0; i < n_lens; ++i)
            if (lens[i] == L)
                return;
        if (n_lens < 5)
            lens[n_lens++] = L;
    };
    add_len(phr.psdu_length);
    if (phr.psdu_length > 2)
        add_len(phr.psdu_length - 1);
    add_len(phr.psdu_length + 1);
    if (phr.psdu_length > 3)
        add_len(phr.psdu_length - 2);
    add_len(phr.psdu_length + 2);

    // Try reported SFD polarity first, then the opposite.  On real RF the
    // NS-SFD sign can flip relative to the payload field even when the
    // magnitude metric is high; one polarity yields RS+FCS, the other fails.
    const int8_t pol0 = (ns_sfd.polarity < 0) ? -1 : 1;
    const int8_t pols[2] = { pol0, static_cast<int8_t>(-pol0) };
    for (size_t li = 0; li < n_lens; ++li) {
        const uint32_t L = lens[li];
        const size_t psdu_bits = static_cast<size_t>(L) * 8;
        const size_t num_blocks = (psdu_bits + 329) / 330;
        const size_t nsym = psdu_bits + 48 * num_blocks;
        for (int8_t pol : pols) {
            std::vector<int8_t> g0p, g1p, g0, g1;
            std::vector<float> llr0p, llr1p, llr0, llr1;
            const auto t_soft = std::chrono::steady_clock::now();
            const bool soft_ok =
                detail::bprf_demod_soft(soft_chips, phr_start,
                                        profile.code_index, 21, 64, 512, 0,
                                        pol, g0p, g1p, llr0p, llr1p) &&
                detail::bprf_demod_soft(soft_chips, payload_start,
                                        profile.code_index, nsym, 8, 64,
                                        1344, pol, g0, g1, llr0, llr1);
            if (sub)
                sub->soft_us += detail::payload_elapsed_us(t_soft);
            if (!soft_ok)
                continue;

            const size_t total = 21 + nsym;
            std::vector<float> llr0j(total), llr1j(total);
            for (size_t s = 0; s < 21; ++s) {
                llr0j[s] = llr0p[s];
                llr1j[s] = llr1p[s];
            }
            for (size_t s = 0; s < nsym; ++s) {
                llr0j[s + 21] = llr0[s];
                llr1j[s + 21] = llr1[s];
            }
            std::vector<int8_t> decoded;
            const auto t_vit = std::chrono::steady_clock::now();
            detail::vitdec_cl3_soft(llr0j.data(), llr1j.data(), total, decoded);
            if (sub)
                sub->viterbi_us += detail::payload_elapsed_us(t_vit);
            if (decoded.size() < total)
                continue;

            std::vector<int8_t> rs_cw(decoded.begin() + 19,
                                      decoded.begin() + (total - 2));
            if (rs_cw.size() != nsym)
                continue;

            std::vector<int8_t> psdu_bits_out;
            const auto t_rs = std::chrono::steady_clock::now();
            const bool rs_ok =
                detail::rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
            if (sub)
                sub->rs_us += detail::payload_elapsed_us(t_rs);
            if (!rs_ok)
                continue;

            // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
            const auto t_crc = std::chrono::steady_clock::now();
            const size_t nbytes = psdu_bits_out.size() / 8;
            PayloadResult cand;
            cand.bytes.resize(nbytes);
            for (size_t i = 0; i < nbytes; ++i) {
                uint8_t b = 0;
                for (int bit = 0; bit < 8; ++bit)
                    b |= static_cast<uint8_t>(psdu_bits_out[8 * i + bit])
                         << bit;
                cand.bytes[i] = b;
            }
            cand.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
            if (nbytes >= 2) {
                cand.calculated_fcs =
                    detail::crc16_802154(cand.bytes.data(), nbytes - 2);
                cand.received_fcs =
                    static_cast<uint16_t>(cand.bytes[nbytes - 2]) |
                    (static_cast<uint16_t>(cand.bytes[nbytes - 1]) << 8);
                cand.fcs_pass =
                    (cand.received_fcs == cand.calculated_fcs);
            }
            cand.ok = true;
            if (sub)
                sub->crc_us += detail::payload_elapsed_us(t_crc);
            if (!any_ok || cand.fcs_pass) {
                out = cand;
                any_ok = true;
            }
            if (out.fcs_pass)
                return true;
        }
    }
    return any_ok;
}

// Build (or reuse) the BPRF payload spreading stream for the fused kernel.
// The PN sequence depends only on the code index and the decoded symbol count,
// so it is cached across packets; the channel template is still rebuilt for
// every packet and counted separately.
inline bool prepare_fused_payload_pn(DemodScratch& scratch,
                                     size_t code_index,
                                     size_t symbols)
{
    const size_t nbits = symbols * rxfusion::kChipsPerBurst;
    if (scratch.fused.code_index == code_index &&
        scratch.fused.symbols == symbols &&
        scratch.fused.pn.size() == nbits)
        return true;
    if (!detail::bprf_spreading(scratch.fused.pn, code_index, 1344, nbits)) {
        scratch.fused.code_index = 0;
        scratch.fused.symbols = 0;
        scratch.fused.pn.clear();
        return false;
    }
    scratch.fused.code_index = code_index;
    scratch.fused.symbols = symbols;
    return true;
}

// Experimental fused payload metrics + FEC/FCS decode.  Reuses the PHR
// decisions and the CIR/phase frozen by the prefix pass.  When `phr_g0`/
// `phr_g1` are supplied by the fused PHR stage, the joint Viterbi consumes
// those raw 21 decisions directly and the prefix does not need PHR soft chips;
// otherwise it re-runs bprf_demod() from the legacy prefix.  The optional
// `phr_llr0`/`phr_llr1` pair carries the same 21 PHR symbols as soft LLRs and
// is only consumed by the in-place soft retry (attempt 2).  Returns false on
// any unsupported configuration, range/short-window violation, fused-decode
// failure or FCS failure; the caller then retries the complete legacy stream
// once.
// `template_us`, `correlate_us` and `decode_us` are populated even when a later
// stage fails so the attempted work is not hidden.
inline bool stage_payload_fused_cmf(const std::complex<float>* rx,
                                    size_t n,
                                    const Qm35825Profile& profile,
                                    const TimingResult& timing,
                                    const CirResult& cir,
                                    const std::vector<float>& prefix_soft_chips,
                                    const NsSfdResult& ns_sfd,
                                    const PhrResult& phr,
                                    PayloadResult& out,
                                    DemodScratch& scratch,
                                    uint64_t& template_us,
                                    uint64_t& correlate_us,
                                    uint64_t& decode_us,
                                    uint64_t& decode_ns,
                                    size_t& symbols_out,
                                    const std::vector<int8_t>* phr_g0 = nullptr,
                                    const std::vector<int8_t>* phr_g1 = nullptr,
                                    PayloadSubTimers* sub = nullptr,
                                    const std::vector<float>* phr_llr0 = nullptr,
                                    const std::vector<float>* phr_llr1 = nullptr,
                                    double cfo_w_rad = 0.0)
{
    // Diagnostic child timing of `stage_payload_us`; see stage_payload_fcs().
    detail::PayloadSubTimer sub_guard(sub);
    if (sub)
        ++sub->fused_calls;
    out = PayloadResult{};
    template_us = 0;
    correlate_us = 0;
    decode_us = 0;
    decode_ns = 0;
    symbols_out = 0;
    if (!rx || !phr.ok || !ns_sfd.ok || phr.psdu_length == 0 ||
        phr.psdu_length > profile.max_psdu_bytes)
        return false;

    const int64_t phr_start = ns_sfd.sfd_end_chip + 1;
    const int64_t payload_start =
        phr_start + static_cast<int64_t>(rxopt::kPhrSymbols) *
                       static_cast<int64_t>(rxopt::kPhrChipsPerSymbol);
    if (phr_start < 0 || payload_start < 0)
        return false;
    // When the fused PHR stage supplies the raw 21 g0/g1 decisions, the
    // payload path does not need the PHR soft chips in the prefix.  Otherwise
    // it must reproduce the legacy bprf_demod() admission exactly.
    const bool have_phr_decisions = phr_g0 && phr_g1;
    if (!have_phr_decisions &&
        static_cast<size_t>(phr_start) +
                rxopt::kPhrSymbols * rxopt::kPhrChipsPerSymbol >
            prefix_soft_chips.size())
        return false;

    const size_t psdu_bits = static_cast<size_t>(phr.psdu_length) * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    const size_t symbols = psdu_bits + 48 * num_blocks;
    if (symbols == 0)
        return false;
    symbols_out = symbols;

    const size_t tap_count = cir.cir_filter_taps.size();
    if (tap_count == 0 || tap_count > rxfusion::kMaxTapCount)
        return false;
    if (tap_count != profile.cir_pre_samples + profile.cir_post_samples)
        return false;
    if (!integer_chip_grid_at(cir.samples_per_chip,
                              rxfusion::kSamplesPerChip,
                              profile.fused_gate_ppm * 1e-6))
        return false;
    if (profile.cir_pre_samples > tap_count)
        return false;
    if (!scratch.softchip.prepared || std::norm(scratch.softchip.rot) <= 0.0f ||
        !std::isfinite(scratch.softchip.rot.real()) ||
        !std::isfinite(scratch.softchip.rot.imag()))
        return false;

    const int64_t chip_start =
        timing.preamble_start_sample + static_cast<int64_t>(tap_count) -
        static_cast<int64_t>(profile.cir_pre_samples) - 1;
    if (chip_start < 0 || static_cast<int64_t>(n) <= chip_start)
        return false;
    const int64_t max_chip =
        (static_cast<int64_t>(n) - 1 - chip_start) /
        static_cast<int64_t>(rxfusion::kSamplesPerChip);
    if (max_chip < 0)
        return false;

    // Reproduce the exact length of the soft-chip stream that the legacy
    // on-demand path would have produced (prefix + optional extension).  The
    // legacy bprf_demod() admission is `payload_start + 64*nsym <= stream`,
    // which is 16 chips stricter than the last actually used chip.  Using only
    // the raw-IQ length here would be a different contract if the chip budget
    // ever became the limiting factor.
    const size_t prefix_chips = cir.soft_chip_count;
    size_t stream_chips = prefix_chips;
    const size_t final_budget = rxopt::final_chip_budget(
        profile, ns_sfd.sfd_end_chip, phr.psdu_length);
    if (final_budget > prefix_chips) {
        const double spc = cir.samples_per_chip;
        const int64_t last_by_budget =
            chip_start + static_cast<int64_t>(std::ceil(
                             static_cast<double>(final_budget - 1) * spc));
        const int64_t last_chip_sample =
            std::min<int64_t>(static_cast<int64_t>(n), last_by_budget);
        const int64_t last_pos = last_chip_sample - 1;
        if (last_pos < chip_start)
            return false;
        stream_chips = static_cast<size_t>(
                           std::floor(static_cast<double>(last_pos - chip_start) /
                                      spc)) +
                       1;
    }
    if (payload_start + static_cast<int64_t>(symbols) *
                            static_cast<int64_t>(rxopt::kPayloadChipsPerSymbol) >
        static_cast<int64_t>(stream_chips))
        return false;

    const int8_t pol = (ns_sfd.polarity < 0) ? -1 : 1;
    rxfusion::Timings timings;
    {
        // PN build + four-candidate correlation is the fused "soft bit"
        // generation; it is the soft bucket of this path.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        const auto t_plan0 = std::chrono::steady_clock::now();
        if (!prepare_fused_payload_pn(scratch, profile.code_index, symbols))
            return false;
        const auto t_plan1 = std::chrono::steady_clock::now();
        const uint64_t plan_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t_plan1 -
                                                                  t_plan0)
                .count());

        if (!rxfusion::correlate_payload_fused_cfo(
                rx, n, cir.cir_filter_taps.data(), tap_count, chip_start,
                rxfusion::kSamplesPerChip, payload_start,
                scratch.fused.pn.data(), symbols, pol, scratch.softchip.rot,
                cfo_w_rad, scratch.fused.metrics, scratch.fused.kernel,
                &timings, /*write_complex=*/false))
            return false;
        template_us = plan_us + timings.template_update_us;
        correlate_us = timings.correlate_us;
        for (float v : scratch.fused.metrics.signed_value) {
            if (!std::isfinite(v))
                return false;
        }
    }

    const auto t_decode0 = std::chrono::steady_clock::now();
    auto stamp_decode = [&]() {
        decode_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - t_decode0)
                .count());
        decode_us = decode_ns / 1000;
    };
    std::vector<int8_t> local_g0p, local_g1p;
    std::vector<int8_t>& g0 = scratch.fused.g0;
    std::vector<int8_t>& g1 = scratch.fused.g1;
    const std::vector<int8_t>* g0p = phr_g0;
    const std::vector<int8_t>* g1p = phr_g1;
    {
        // PHR re-demod (only when the fused PHR stage supplied no decisions) and
        // the BPM decisions belong to the metric-generation bucket.
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        if (g0p || g1p) {
            if (!g0p || !g1p || g0p->size() != 21 || g1p->size() != 21) {
                stamp_decode();
                return false;
            }
        } else {
            if (!detail::bprf_demod(prefix_soft_chips, phr_start,
                                    profile.code_index, 21, 64, 512, 0, pol,
                                    local_g0p, local_g1p)) {
                stamp_decode();
                return false;
            }
            g0p = &local_g0p;
            g1p = &local_g1p;
        }
        rxfusion::decide_bpm(scratch.fused.metrics, g0, g1);
    }
    // Attempt 1 — the unchanged hard-decision path.  On success this is the
    // whole cost of a high-SNR packet: nothing below runs.
    const bool decoded = detail::decode_payload_hard(
        *g0p, *g1p, g0, g1, psdu_bits, out, sub, &scratch.fused.decode);
    if (decoded && out.fcs_pass) {
        stamp_decode();
        return true;
    }

    // Attempt 2 — in-place soft-decision retry on the SAME four signed metrics.
    // No extra correlation and no extra derotation: the LLRs are a by-product
    // of the metrics the fused kernel already produced.  The PHR LLRs come from
    // the fused PHR cache when it is valid, otherwise from one bprf_demod_soft()
    // pass over the legacy prefix.  This is what keeps a marginal packet out of
    // the ~15 ms full-stream stage_softchip_fallbacks retry.
    std::vector<int8_t>& soft_g0 = scratch.fused.soft_g0;
    std::vector<int8_t>& soft_g1 = scratch.fused.soft_g1;
    std::vector<float>& llr0 = scratch.fused.llr0;
    std::vector<float>& llr1 = scratch.fused.llr1;
    std::vector<int8_t> local_soft_g0p, local_soft_g1p;
    std::vector<float> local_llr0p, local_llr1p;
    const std::vector<float>* llr0p = phr_llr0;
    const std::vector<float>* llr1p = phr_llr1;
    {
        detail::PayloadSubBucket soft_bucket(sub, &PayloadSubTimers::soft_us);
        rxfusion::decide_bpm_soft(scratch.fused.metrics, soft_g0, soft_g1,
                                  llr0, llr1);
        if (!llr0p || !llr1p || llr0p->size() != 21 || llr1p->size() != 21) {
            llr0p = nullptr;
            llr1p = nullptr;
            if (detail::bprf_demod_soft(prefix_soft_chips, phr_start,
                                        profile.code_index, 21, 64, 512, 0,
                                        pol, local_soft_g0p, local_soft_g1p,
                                        local_llr0p, local_llr1p)) {
                llr0p = &local_llr0p;
                llr1p = &local_llr1p;
            }
        }
    }
    if (llr0p && llr1p) {
        const bool soft_decoded = detail::decode_payload_soft(
            *llr0p, *llr1p, llr0, llr1, psdu_bits, out, sub,
            &scratch.fused.decode);
        if (soft_decoded && out.fcs_pass) {
            stamp_decode();
            return true;
        }
    }
    stamp_decode();
    return false;
}

} // namespace gr::uwb::demod::core
