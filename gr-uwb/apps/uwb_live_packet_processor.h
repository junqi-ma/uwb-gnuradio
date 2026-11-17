// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_pipeline.h>
#include <gnuradio/uwb/uwb_qm35_acquisition_tracker.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_resample_window.h>
#include <gnuradio/uwb/uwb_target_lock_core.h>
#include <chrono>

namespace gr::uwb::live {
// Identity and coordinates carried from planning through completion. Native
// absolute coordinates belong to the scheduler; predicted_rel_work belongs to
// the resampled window. Slot/generation are provenance, not a completion order.
struct PacketRequest {
    uint64_t id = 0;
    int64_t seed = 0;
    int64_t window_start = 0;
    int64_t predicted_rel_work = 0;
    size_t guard = 0;
    uint64_t slot_index = std::numeric_limits<uint64_t>::max();
    uint64_t generation = 0;
    bool target_slot = false;
    bool feeds_lock = false;
};

// Independent CRC verification intentionally remains separate from codec FCS.
inline bool verify_packet_crc(const std::vector<uint8_t>& b) {
    if (b.size() < 2) return false;
    uint16_t crc = 0;
    for (size_t i = 0; i + 2 < b.size(); ++i) {
        crc ^= b[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
    }
    return crc == uint16_t(b[b.size()-2] | uint16_t(b.back()) << 8);
}

inline bool should_retry_full_cir(const demod::DemodResult& decoded,
    const demod::Qm35825Profile& profile, bool auto_cir) {
    return !profile.rx_stop_after_sfd && auto_cir &&
        decoded.timing.ok && decoded.sfd.ok &&
        (decoded.status != demod::DemodStatus::Success ||
         !decoded.payload.fcs_pass || !verify_packet_crc(decoded.payload.bytes));
}

struct PacketProcessingResult {
    demod::DemodResult decoded;
    bool retried = false;
    std::chrono::steady_clock::time_point resample_end, demod_end, compute_end;
};

// One instance per worker. Owns all mutable DSP state; borrows the immutable
// template for its lifetime. No Reporter, queue or TargetLockCore reference.
class PacketProcessor {
public:
    PacketProcessor(const std::vector<float>& taps,
                    const std::vector<std::complex<float>>& tmpl,
                    size_t max_native_samples)
        : rs_(taps), template_(tmpl),
          work_(rs_.expected_output_length(max_native_samples)) {
        rs_.set_fixed_phase_schedule(true);
        scratch_.reserve(work_.size());
    }

    PacketProcessingResult process(const PacketRequest& request,
        const std::complex<int16_t>* iq, size_t count,
        const demod::Qm35825Profile& profile, bool auto_cir) {
        PacketProcessingResult out;
        const size_t made = core::resample_sc16_window(rs_, iq, count,
                                                       1.f/32768, work_);
        out.resample_end = std::chrono::steady_clock::now();
        out.decoded = demod::core::demodulate_one(work_.data(), made, profile,
            request.id, request.predicted_rel_work, 0, template_, scratch_);
        out.retried = should_retry_full_cir(out.decoded, profile, auto_cir);
        out.demod_end = std::chrono::steady_clock::now();
        if (out.retried) {
            auto full = profile;
            full.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
            out.decoded = demod::core::demodulate_one(work_.data(), made, full,
                request.id, request.predicted_rel_work, 0, template_, scratch_);
        }
        out.compute_end = std::chrono::steady_clock::now();
        return out;
    }
private:
    core::RationalResampler65_48Core rs_;
    const std::vector<std::complex<float>>& template_;
    std::vector<std::complex<float>> work_;
    demod::core::DemodScratch scratch_;
};

// Pure conversion of a completed packet to a lock observation. Publishing it
// remains the worker's responsibility; application remains the RX owner's.
inline core::TargetObservation make_target_observation(
    const PacketRequest& request, const demod::DemodResult& decoded,
    double work_rate, double native_rate, double group_delay)
{
    core::TargetObservation obs;
    obs.packet_id = request.id;
    obs.schedule_generation = request.generation;
    obs.slot_index = request.slot_index;
    obs.window_origin = request.window_start;
    obs.predicted_origin = request.seed;
    obs.timing_ok = decoded.timing.ok;
    obs.sfd_ok = decoded.sfd.ok;
    obs.phr_ok = decoded.phr.ok;
    obs.fcs_pass = decoded.payload.fcs_pass && verify_packet_crc(decoded.payload.bytes);
    obs.code_index = 10;
    obs.preamble_repetitions = 256;
    obs.sfd_mode = "4z2";
    obs.quality = decoded.timing.metric;
    obs.from_target_window = request.target_slot;
    if (decoded.timing.ok) {
        obs.observed_origin = request.window_start + core::map_obs_sample_to_native(
            decoded.timing.preamble_start_sample, work_rate, native_rate, group_delay);
        obs.residual = obs.observed_origin - request.seed;
    }
    return obs;
}
} // namespace gr::uwb::live
