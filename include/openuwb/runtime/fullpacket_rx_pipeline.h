/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <openuwb/phy/hrp/rx/demod_pipeline.h>

#include <stdexcept>

namespace gr {
namespace uwb {
namespace experiment {

// Reuses the mature RX core, including its timing/CFO/SFD refinements and
// soft-chip FIR. Native and portable executables compile this SAME pipeline;
// the existing CIR FIR header selects AVX2/FMA only when actually enabled.
// The caller supplies the observed IQ and a public/estimated seed. This layer
// neither crops the input nor consumes injected timing, payload or channel.
class FastRxPipeline {
public:
    // Explicit experimental RX options.  Every flag defaults to false so the
    // existing caller path (and all historical runs) keep the legacy kernels.
    struct Options {
        bool fast_sfd;
        bool fast_cfo_rotation;
        bool seed_refine_tail;
        bool softchip_on_demand;
        bool fused_payload_cmf;
        bool fused_phr_cmf;
        bool diagnose_fir_segments;

        Options()
            : fast_sfd(false), fast_cfo_rotation(false),
              seed_refine_tail(false), softchip_on_demand(false),
              fused_payload_cmf(false), fused_phr_cmf(false),
              diagnose_fir_segments(false)
        {
        }
        Options(bool sfd, bool cfo, bool seed, bool soft,
                bool fused_payload = false, bool fused_phr = false,
                bool diagnose_fir = false)
            : fast_sfd(sfd), fast_cfo_rotation(cfo),
              seed_refine_tail(seed), softchip_on_demand(soft),
              fused_payload_cmf(fused_payload), fused_phr_cmf(fused_phr),
              diagnose_fir_segments(diagnose_fir)
        {
        }

        // The fused paths consume the on-demand prefix.  Payload fusion needs
        // SYNC/SFD/PHR soft chips; PHR fusion stops the prefix at the NS-SFD
        // end and also requires payload fusion.  Return the effective
        // configuration so callers can log exactly what the profile will do
        // rather than a potentially contradictory set of request switches.
        Options normalized() const
        {
            Options out = *this;
            out.softchip_on_demand = softchip_on_demand ||
                                     fused_payload_cmf || fused_phr_cmf;
            out.fused_payload_cmf = fused_payload_cmf || fused_phr_cmf;
            return out;
        }
    };

    FastRxPipeline(size_t max_samples, size_t code,
                   const Options& options = Options{})
        : capacity_(max_samples), profile_(demod::Qm35825Profile::Default())
    {
        if (max_samples == 0 || code < 9 || code > 12)
            throw std::invalid_argument("unsupported RX capacity/code");
        const Options effective = options.normalized();
        profile_.code_index = code;
        profile_.timing_search_margin = 256;
        profile_.timing_coarse_stride = 1;
        profile_.rx_fast_sfd = effective.fast_sfd;
        profile_.rx_fast_cfo_rotation = effective.fast_cfo_rotation;
        profile_.rx_seed_refine_tail = effective.seed_refine_tail;
        // The fused payload/PHR paths consume the on-demand prefix.  Selecting
        // either therefore implies the prefix path; PHR fusion additionally
        // implies payload fusion.  The other options retain their values.
        profile_.rx_softchip_on_demand = effective.softchip_on_demand;
        profile_.rx_fused_payload_cmf = effective.fused_payload_cmf;
        profile_.rx_fused_phr_cmf = effective.fused_phr_cmf;
        // Diagnostic hook lives on the core scratch, not on the public
        // production profile (moved out of Qm35825Profile).
        scratch_.diagnose_fir_segments = effective.diagnose_fir_segments;
        scratch_.reserve(max_samples);
    }

    demod::DemodResult run(const std::complex<float>* iq,
                          size_t n,
                          uint64_t packet_id,
                          int64_t seed,
                          const std::vector<std::complex<float>>& sync)
    {
        if (!iq || n == 0 || n > capacity_ || seed < 0 ||
            seed >= static_cast<int64_t>(n) || sync.size() != 1016) {
            demod::DemodResult result;
            result.status = demod::DemodStatus::InvalidInput;
            result.packet_id = packet_id;
            result.predicted_start_sample = seed;
            result.window_start_sample = 0;
            result.input_sample_count = n;
            return result;
        }
        return demod::core::demodulate_one(
            iq, n, profile_, packet_id, seed, 0, sync, scratch_);
    }

    static constexpr bool avx2_enabled()
    {
        return UWB_CIR_FIR_HAVE_AVX2 != 0;
    }

    static constexpr const char* pipeline_name()
    {
        return avx2_enabled() ? "mature_rx_avx2_fma" : "mature_rx_portable";
    }

private:
    size_t capacity_;
    demod::Qm35825Profile profile_;
    demod::core::DemodScratch scratch_;
};

} // namespace experiment
} // namespace uwb
} // namespace gr
