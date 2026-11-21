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
bool stage_ns_sfd(const std::vector<float>& soft_chips,
                  const Qm35825Profile& profile,
                  const std::vector<int8_t>& sfd_sequence,
                  size_t chips_per_symbol,
                  NsSfdResult& out,
                  DemodScratch& scratch);

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

bool phr_secded_parse(const std::vector<int8_t>& decoded,
                      const std::vector<float>& soft_chips,
                      const NsSfdResult& ns_sfd,
                      PhrResult& out);

} // namespace detail

// Hard-decision PHR entry point: 21 raw g0/g1 pairs -> CL-3 Viterbi -> the
// shared SECDED/parser tail.
bool decode_phr_decisions(const std::vector<int8_t>& g0,
                          const std::vector<int8_t>& g1,
                          const std::vector<float>& soft_chips,
                          const NsSfdResult& ns_sfd,
                          PhrResult& out);

// Soft-decision PHR entry point: the same 21 codeword symbols as per-symbol
// LLRs (positive favours bit 0, exactly the bprf_demod_soft()/
// rxfusion::decide_bpm_soft() convention) -> soft CL-3 Viterbi -> the same
// shared SECDED/parser tail.  Nothing else differs from the hard path.
bool decode_phr_decisions_soft(const std::vector<float>& llr0,
                               const std::vector<float>& llr1,
                               const std::vector<float>& soft_chips,
                               const NsSfdResult& ns_sfd,
                               PhrResult& out);

bool stage_phr(const std::vector<float>& soft_chips,
               const Qm35825Profile& profile,
               const NsSfdResult& ns_sfd,
               size_t chips_per_symbol,
               PhrResult& out,
               DemodScratch& scratch);

// Build (or reuse) the PHR spreading stream.  The PN depends only on the code
// index and the fixed 21-symbol geometry; the channel template is rebuilt per
// packet by stage_phr_fused_cmf().
bool prepare_fused_phr_pn(DemodScratch& scratch, size_t code_index);

// Experimental fused PHR metrics + the shared PHR FEC/SECDED parser.  The PHR
// field is generated directly from CFO-compensated IQ with 164-point templates
// (38 taps), so no PHR soft-chip FIR is required.  The 21 raw g0/g1 decisions
// are cached for the payload joint Viterbi.  Returns false on unsupported
// geometry, range violations, non-finite metrics or decode failure; the caller
// retries the complete legacy stream once.  Timings are always reported when
// the corresponding work was performed.
bool stage_phr_fused_cmf(const std::complex<float>* rx,
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
                         size_t& symbols_out);

} // namespace openuwb::demod::core
