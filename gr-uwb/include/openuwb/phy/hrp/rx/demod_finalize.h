// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_result.h>
namespace gr::uwb::demod::core {
// Called exactly once on each pipeline exit. Stages use window-local sample
// positions; only the fields historically published as absolute are rebased.
// Soft-chip positions and other local diagnostic fields are left unchanged.
inline void finalize_demod_result(DemodResult& res, const PayloadSubTimers& payload_sub,
                                   int64_t window_start) {

    res.stage_payload_soft_us = payload_sub.soft_us;
    res.stage_payload_viterbi_us = payload_sub.viterbi_us;
    res.stage_payload_rs_us = payload_sub.rs_us;
    res.stage_payload_crc_us = payload_sub.crc_us;
    res.stage_payload_other_us = payload_sub.other_us;
    res.stage_payload_derotate_us = payload_sub.other_derotate_us;
    res.stage_payload_fused_calls = payload_sub.fused_calls;
    res.stage_payload_legacy_calls = payload_sub.legacy_calls;
    if (window_start == 0)
        return;
    if (res.timing.ok) {
        res.timing.preamble_start_sample += window_start;
        res.timing.preamble_start_uncropped += window_start;
        for (auto& p : res.timing.peak_samples)
            p += window_start;
    }
    if (res.cfo.ok) {
        res.cfo.fit_first_peak_sample += window_start;
        res.cfo.fit_last_peak_sample += window_start;
    }
    if (res.sfd.ok) {
        res.sfd.sfd_start_sample += window_start;
        res.sfd.sfd_end_sample += window_start;
    }
    if (res.sfd_initial_predicted_sample >= 0)
        res.sfd_initial_predicted_sample += window_start;
    if (res.sfd_bootstrap_detected_sample >= 0)
        res.sfd_bootstrap_detected_sample += window_start;
    if (res.cir.ok)
        res.cir.first_path_sample += window_start;
}
} // namespace gr::uwb::demod::core
