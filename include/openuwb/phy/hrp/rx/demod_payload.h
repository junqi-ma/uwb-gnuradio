// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_phr.h>

namespace openuwb::demod::core {
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

bool stage_payload_fcs(const std::vector<float>& soft_chips,
                       const Qm35825Profile& profile,
                       const PhrResult& phr,
                       const NsSfdResult& ns_sfd,
                       size_t chips_per_symbol,
                       PayloadResult& out,
                       DemodScratch& scratch,
                       PayloadSubTimers* sub = nullptr);

// Build (or reuse) the BPRF payload spreading stream for the fused kernel.
// The PN sequence depends only on the code index and the decoded symbol count,
// so it is cached across packets; the channel template is still rebuilt for
// every packet and counted separately.
bool prepare_fused_payload_pn(DemodScratch& scratch,
                              size_t code_index,
                              size_t symbols);

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
bool stage_payload_fused_cmf(const std::complex<float>* rx,
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
                             double cfo_w_rad = 0.0);

} // namespace openuwb::demod::core
