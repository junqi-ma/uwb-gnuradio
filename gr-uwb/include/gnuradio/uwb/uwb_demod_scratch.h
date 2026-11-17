// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_cir_fir_simd.h>
#include <gnuradio/uwb/uwb_demod_result.h>
#include <gnuradio/uwb/uwb_detector_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_rx_cfo_opt.h>
#include <gnuradio/uwb/uwb_rx_sync_opt.h>
#include <gnuradio/uwb/uwb_rx_softchip_opt.h>
#include <gnuradio/uwb/uwb_rx_fused_cmf.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <vector>


namespace gr::uwb::demod::core {
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
    // On-demand soft-chip FIR output.  Kept separate from `corr` because the
    // extension call re-runs the (cheap) CIR estimate, which temporarily uses
    // `corr` for the coherent SYNC average while the prefix FIR values must
    // survive.  Unused by the legacy path.
    std::vector<std::complex<float>> softchip_corr;

    // Diagnostic-only replay of the legacy soft-chip FIR, segmented at the
    // decoded SFD/PHR/payload boundaries.  Never part of a formal timing run
    // and NOT part of the public Qm35825Profile production configuration:
    // only the experiment wrappers set this on their private scratch.
    bool diagnose_fir_segments = false;

    // ---- Experimental RX fast-path state ---------------------------------
    // Cached normalized SFD waveform (kron(sfd_sequence, template_wf)).
    // The key covers the template + SFD content, so the cache does not depend
    // on the temporary contents of work/corr surviving between calls.
    std::vector<std::complex<float>> sfd_template;
    uint64_t sfd_template_key = 0;

    // Cached sparse ternary CIR template (Task A2).
    rxopt::SparseCirTemplate cir_template;

    // Cached pre-duplicated real SYNC template for AVX2 fast dot.
    rxopt::RealSyncTemplate real_sync_template;

    // Cached sequence data for fixed preamble code and SFD mode (Task B2).
    struct CachedSequenceState {
        size_t code_index = std::numeric_limits<size_t>::max();
        std::string sfd_mode;
        std::vector<int8_t> pcode;
        std::vector<int8_t> sfd_seq;
        std::vector<int8_t> sampled_pcode;

        struct SparseSfdEntry {
            uint32_t offset;
            int8_t sign;
        };
        std::vector<SparseSfdEntry> ns_sfd_sparse;
        size_t ns_sfd_total_chips = 0;
        float ns_sfd_norm = 0.0f;

        void update(size_t new_code_index, const char* new_sfd_mode)
        {
            const char* cur = new_sfd_mode ? new_sfd_mode : "";
            const bool code_changed = (code_index != new_code_index);
            const bool mode_changed = (sfd_mode != cur);
            if (code_changed) {
                code_index = new_code_index;
                const int8_t* pc = GetPreambleCode(code_index);
                pcode.assign(pc, pc + kQm35CodeLength);
                sampled_pcode = BuildSampledCode(pcode.data(), pcode.size());
            }
            if (mode_changed) {
                sfd_mode = cur;
                sfd_seq = gr::uwb::demod::GetSfdSequence(new_sfd_mode);
            }
            if (code_changed || mode_changed || ns_sfd_sparse.empty()) {
                ns_sfd_sparse.clear();
                ns_sfd_total_chips = sfd_seq.size() * sampled_pcode.size();
                for (size_t i = 0; i < sfd_seq.size(); ++i) {
                    const int8_t si = sfd_seq[i];
                    if (si == 0) continue;
                    for (size_t j = 0; j < sampled_pcode.size(); ++j) {
                        const int8_t pj = sampled_pcode[j];
                        if (pj == 0) continue;
                        const int8_t val = static_cast<int8_t>(si * pj);
                        ns_sfd_sparse.push_back({ static_cast<uint32_t>(i * sampled_pcode.size() + j), val });
                    }
                }
                ns_sfd_norm = std::sqrt(static_cast<float>(ns_sfd_sparse.size()));
            }
        }

        void invalidate()
        {
            code_index = std::numeric_limits<size_t>::max();
            sfd_mode.clear();
            pcode.clear();
            sfd_seq.clear();
            sampled_pcode.clear();
            ns_sfd_sparse.clear();
            ns_sfd_total_chips = 0;
            ns_sfd_norm = 0.0f;
        }
    } cached_seq;

    // Buffer for SFD window pre-folding.
    std::vector<std::complex<float>> sfd_folded;

    // Retained initial timing for tail re-track conditional equivalence.
    TimingResult initial_timing;

    // Two-phase soft-chip state.  Only populated when
    // profile.rx_softchip_on_demand is enabled; the legacy full-stream path
    // never reads it.  The extension re-runs the cheap CIR estimate and
    // deterministically rebuilds the same taps/grid, so only the prefix
    // boundary and its normalization maximum must persist.
    struct SoftChipState {
        bool prepared = false;
        size_t generated = 0;
        float prefix_max = 0.0f;
        // Phase rotation frozen by the prefix pass: the phase alignment is
        // estimated once per packet and reused by the extension.
        std::complex<float> rot{ 1.0f, 0.0f };

        void reset() { *this = SoftChipState{}; }
    } softchip;

    // Experimental fused payload CMF/despread state.  PN depends only on the
    // selected code and the decoded payload length, so it survives packets;
    // the per-packet channel templates live in kernel.templates and are
    // rebuilt for every packet (counted in stage_fused_template_us).
    struct FusedCmfState {
        size_t code_index = 0;
        size_t symbols = 0;
        std::vector<int8_t> pn;
        rxfusion::Metrics metrics;
        rxfusion::Scratch kernel;

        void reset_metrics()
        {
            rxfusion::Metrics empty;
            metrics = std::move(empty);
        }
    } fused;

    // Experimental fused PHR CMF/despread state.  PN depends only on the
    // selected code (fixed 21 symbols, scrambler offset 0); the per-packet
    // channel templates live in kernel.templates.  The 21 raw g0/g1 decisions
    // are cached for the joint payload Viterbi, which must not re-run the
    // soft-chip PHR demod when the PHR was produced by the fused kernel.  The
    // per-symbol LLRs are cached alongside them so a soft-decision payload
    // retry can reuse the PHR soft information instead of re-demodulating it.
    struct FusedPhrState {
        size_t code_index = 0;
        std::vector<int8_t> pn;
        std::vector<int8_t> g0;
        std::vector<int8_t> g1;
        std::vector<float> llr0;
        std::vector<float> llr1;
        bool decisions_valid = false;
        rxfusion::Metrics metrics;
        rxfusion::Scratch kernel;

        void reset_decisions()
        {
            decisions_valid = false;
            g0.clear();
            g1.clear();
            llr0.clear();
            llr1.clear();
        }
    } fused_phr;

    // Packet decisions cannot survive a retry/new packet. Template caches and
    // allocated workspace capacities intentionally survive. Other phase state
    // is reset by its owning stage, preserving legacy initialization order.
    void begin_packet() { fused_phr.reset_decisions(); }

    void reserve(size_t n)
    {
        work.reserve(n);
        derotated.reserve(n);
        energy.reserve(n);
        metric.reserve(n);
        corr.reserve(n);
        softchip_corr.reserve(n);
        soft_chips.reserve(n);
        cir_taps.reserve(64);
        peaks.reserve(256);
        peak_values.reserve(256);
        sfd_template.reserve(8 * kQm35SamplesPerSymbol);
        cached_seq.pcode.reserve(kQm35CodeLength);
        cached_seq.sfd_seq.reserve(64);
        cached_seq.sampled_pcode.reserve(2 * kQm35ChipsPerSymbol);
        fused.pn.reserve(8 * 2048);
        fused.metrics.complex_value.reserve(
            rxfusion::kCandidateCount * 2048);
        fused.metrics.signed_value.reserve(
            rxfusion::kCandidateCount * 2048);
        fused.kernel.templates.reserve(
            rxfusion::kBlockSymbols * (rxfusion::kMaxTapCount +
                                       rxfusion::kSamplesPerChip *
                                           (rxfusion::kChipsPerBurst - 1)));
        fused_phr.pn.reserve(rxfusion::kPhrSymbols *
                             rxfusion::kPhrChipsPerBurst);
        fused_phr.g0.reserve(rxfusion::kPhrSymbols);
        fused_phr.g1.reserve(rxfusion::kPhrSymbols);
        fused_phr.metrics.complex_value.reserve(
            rxfusion::kCandidateCount * rxfusion::kPhrSymbols);
        fused_phr.metrics.signed_value.reserve(
            rxfusion::kCandidateCount * rxfusion::kPhrSymbols);
        fused_phr.kernel.templates.reserve(
            rxfusion::kBlockSymbols *
            (rxfusion::kMaxTapCount +
             rxfusion::kSamplesPerChip *
                 (rxfusion::kPhrChipsPerBurst - 1)));
    }
};


} // namespace gr::uwb::demod::core
