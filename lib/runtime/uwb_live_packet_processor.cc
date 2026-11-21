// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Out-of-line implementations of the SIC-enabled live packet path
 * (declarations in include/openuwb/runtime/live_packet_processor.h).
 *
 * Moved bodies: the PacketProcessor resample -> optional SIC -> demodulate
 * entry point (constructor and process()) and the PacketSlotPool stop/submit
 * hand-offs.  The zero-allocation hot loop is unchanged: process() only
 * touches work_/sic_out_/scratch_, which the constructor sizes once.
 *
 * The header keeps PacketRequest/PacketProcessingResult/PacketSicOptions, the
 * PacketSlotPool worker loop template (it must be instantiated by the caller)
 * and the small inline predicates.
 */

#include <openuwb/runtime/live_packet_processor.h>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace openuwb {
namespace live {

PacketProcessor::PacketProcessor(const std::vector<float>& taps,
                                 const std::vector<std::complex<float>>& tmpl,
                                 size_t max_native_samples,
                                 PacketSicOptions sic)
    : rs_(taps), template_(tmpl),
      work_(rs_.expected_output_length(max_native_samples)),
      sic_(std::move(sic)) {
    rs_.set_kernel("transposed");
    scratch_.reserve(work_.size());
    if (sic_.enabled) {
        // Buffer sizing is owned by the processor: the caller only
        // supplies the PHY/policy.  Prepare once here, never on the hot
        // path.
        if (sic_.config.max_window_samples == 0)
            sic_.config.max_window_samples = work_.size();
        if (sic_.config.max_packet_samples == 0)
            sic_.config.max_packet_samples = work_.size();
        // Blind acquisition: center the coarse search on the window and
        // let its margin cover the whole buffer.  The interferer position
        // is not known in a live target window, so a zero/zero ROI must
        // not mean "search nowhere".
        if (sic_.config.roi_hi <= sic_.config.roi_lo) {
            sic_.config.roi_lo = 0;
            sic_.config.roi_hi =
                static_cast<int64_t>(sic_.config.max_window_samples);
        }
        if (sic_.config.sub_roi_hi <= 0)
            sic_.config.sub_roi_hi =
                static_cast<int64_t>(sic_.config.max_window_samples);
        std::string err;
        if (!sic::prepare_sic_unified(sic_.config, sic_.templates,
                                      sic_scratch_, &err)) {
            throw std::runtime_error("SIC initialization failed: " + err);
        }
        sic_out_.assign(work_.size(), std::complex<float>(0.f, 0.f));
        sic_prepared_ = true;
    }
}

PacketProcessingResult PacketProcessor::process(
    const PacketRequest& request, const std::complex<int16_t>* iq, size_t count,
    const demod::Qm35825Profile& profile, bool auto_cir) {
    PacketProcessingResult out;
    const size_t made = core::resample_sc16_window(rs_, iq, count,
                                                   1.f/32768, work_);
    out.resample_end = std::chrono::steady_clock::now();
    out.sic_end = out.resample_end;
    const std::complex<float>* demod_in = work_.data();

    // Fail-open is the contract: the unmodified work window is demodulated
    // unless SIC both succeeded AND reached L1/L2 with a committed
    // transaction.  A failed/L0 SIC never replaces the target samples.
    if (sic_prepared_ && made > 0) {
        out.sic_executed = true;
        // ROI bounds are window-relative and the prepared buffers are sized
        // for the largest window.  Clamp the per-call copy to the samples
        // actually produced: the reconstruction transaction rejects a
        // sub-ROI that reaches past `made`, and a centered ROI whose high
        // edge is inside the window keeps the Ipatov acquisition scan
        // bounded to real data.  The copy is a flat struct (no heap).
        sic::SicUnifiedConfig call_cfg = sic_.config;
        call_cfg.roi_lo = std::max<int64_t>(
            0, std::min<int64_t>(call_cfg.roi_lo,
                                 static_cast<int64_t>(made)));
        call_cfg.roi_hi = std::min<int64_t>(
            static_cast<int64_t>(made), call_cfg.roi_hi);
        if (call_cfg.roi_hi <= call_cfg.roi_lo) {
            call_cfg.roi_lo = 0;
            call_cfg.roi_hi = static_cast<int64_t>(made);
        }
        call_cfg.sub_roi_lo = std::max<int64_t>(
            0, std::min<int64_t>(call_cfg.sub_roi_lo,
                                 static_cast<int64_t>(made)));
        call_cfg.sub_roi_hi = std::min<int64_t>(
            static_cast<int64_t>(made), call_cfg.sub_roi_hi);
        if (call_cfg.sub_roi_hi <= call_cfg.sub_roi_lo) {
            call_cfg.sub_roi_lo = 0;
            call_cfg.sub_roi_hi = static_cast<int64_t>(made);
        }
        call_cfg.observed_end_sample = 0;
        sic::SicUnifiedResult sic_result;
        sic_result.out = sic_out_.data();
        // The live path consumes `out` only on committed L1/L2.  Skipping
        // the L0/failure full-window copy and the unused i_hat zero-fill
        // removes ~0.58 ms/candidate of pure memory traffic on the common
        // no-interferer path.
        sic_result.copy_out_on_bypass = false;
        sic_result.i_hat = nullptr;
        out.sic_ok = sic::sic_process_unified(
            work_.data(), made, sic_.templates, call_cfg,
            sic_scratch_, sic_result);
        out.sic_end = std::chrono::steady_clock::now();
        out.sic_committed = out.sic_ok && sic_result.committed;
        out.sic_level = sic_result.level;
        out.sic_reason = sic_result.reason ? sic_result.reason : "";
        out.sic_suppression_db = sic_result.suppression_db;
        if (out.sic_committed &&
            (out.sic_level == sic::SicCancellationLevel::L1_Preamble ||
             out.sic_level == sic::SicCancellationLevel::L2_FullPacket)) {
            demod_in = sic_out_.data();
        }
    }

    out.decoded = demod::core::demodulate_one(demod_in, made, profile,
        request.id, request.predicted_rel_work, 0, template_, scratch_);
    out.retried = should_retry_full_cir(out.decoded, profile, auto_cir);
    out.demod_end = std::chrono::steady_clock::now();
    if (out.retried) {
        auto full = profile;
        full.cir_soft_chip_mode = demod::CirSoftChipMode::Full;
        out.decoded = demod::core::demodulate_one(demod_in, made, full,
            request.id, request.predicted_rel_work, 0, template_, scratch_);
    }
    out.compute_end = std::chrono::steady_clock::now();
    return out;
}

void PacketSlotPool::stop()
{
    {
        std::lock_guard<std::mutex> l(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    for (auto& t : threads_) {
        if (t.joinable())
            t.join();
    }
    threads_.clear();
}

bool PacketSlotPool::submit(const PacketRequest& req, size_t count,
                            const std::vector<std::complex<int16_t>>& ring)
{
    size_t slot = 0;
    {
        std::lock_guard<std::mutex> l(mutex_);
        if (stopping_ || free_.empty())
            return false;
        slot = free_.front();
        free_.pop_front();
    }
    auto& job = jobs_[slot];
    static_cast<PacketRequest&>(job) = req;
    job.count = std::min(count, max_window_);
    job.ready = std::chrono::steady_clock::now();
    if (job.count > 0 && !ring.empty()) {
        const size_t mask = ring.size() - 1;
        const size_t offset = static_cast<size_t>(req.window_start) & mask;
        const size_t first = std::min(job.count, ring.size() - offset);
        std::copy_n(ring.data() + offset, first, job.iq.data());
        if (job.count > first)
            std::copy_n(ring.data(), job.count - first, job.iq.data() + first);
    }
    {
        std::lock_guard<std::mutex> l(mutex_);
        pending_.push_back(slot);
    }
    cv_.notify_one();
    return true;
}

} // namespace live
} // namespace openuwb
