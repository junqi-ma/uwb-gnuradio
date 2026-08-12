/* -*- c++ -*- */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_scheduled_extractor_sc16.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace gr {
namespace uwb {

UwbScheduledExtractorSc16::UwbScheduledExtractorSc16(
    double sample_rate,
    double packet_interval_s,
    uint64_t first_packet_sample,
    size_t pre_guard_samples,
    size_t capture_samples,
    size_t post_guard_samples,
    size_t pool_size,
    EmitPolicy emit_policy)
    : gr::sync_block("uwb_scheduled_extractor_sc16",
                     gr::io_signature::make(1, 1, 2 * sizeof(int16_t)),
                     gr::io_signature::make(0, 0, 0))
{
    if (sample_rate <= 0.0 || packet_interval_s <= 0.0 ||
        pre_guard_samples + capture_samples + post_guard_samples == 0) {
        throw std::invalid_argument("UwbScheduledExtractorSc16: invalid schedule");
    }
    if (emit_policy != EmitPolicy::EverySlot) {
        throw std::invalid_argument(
            "UwbScheduledExtractorSc16: only EverySlot is supported");
    }
    d_cfg_.sample_rate = sample_rate;
    d_cfg_.packet_interval_s = packet_interval_s;
    d_cfg_.first_packet_sample_exact = static_cast<double>(first_packet_sample);
    d_cfg_.pre_guard_samples = pre_guard_samples;
    d_cfg_.capture_samples = capture_samples;
    d_cfg_.post_guard_samples = post_guard_samples;
    d_cfg_.pool_size = std::clamp<size_t>(pool_size, 1, kMaxPoolSize);
    d_cfg_.emit_policy = emit_policy;
    const size_t n_i16 = 2 * d_cfg_.window_capacity();
    for (size_t i = 0; i < d_cfg_.pool_size; ++i)
        d_slots_[i].iq.resize(n_i16);
    set_max_noutput_items(1048576);
    message_port_register_out(pmt::mp("packet"));
}

UwbScheduledExtractorSc16::sptr UwbScheduledExtractorSc16::make(
    double sample_rate, double packet_interval_s, uint64_t first_packet_sample,
    size_t pre_guard_samples, size_t capture_samples, size_t post_guard_samples,
    size_t pool_size, EmitPolicy emit_policy)
{
    return gnuradio::get_initial_sptr(new UwbScheduledExtractorSc16(
        sample_rate, packet_interval_s, first_packet_sample, pre_guard_samples,
        capture_samples, post_guard_samples, pool_size, emit_policy));
}

UwbScheduledExtractorSc16::~UwbScheduledExtractorSc16() { shutdown_worker(); }

uint64_t UwbScheduledExtractorSc16::scheduled_windows() const { return d_scheduled_; }
uint64_t UwbScheduledExtractorSc16::emitted_windows() const { return d_emitted_; }
uint64_t UwbScheduledExtractorSc16::dropped_windows() const { return d_dropped_; }
uint64_t UwbScheduledExtractorSc16::completed_windows() const { return d_completed_; }
uint64_t UwbScheduledExtractorSc16::process_total_us() const { return d_process_total_us_; }
uint64_t UwbScheduledExtractorSc16::copy_total_us() const { return d_copy_total_us_; }
uint64_t UwbScheduledExtractorSc16::publish_total_us() const { return d_publish_total_us_; }

bool UwbScheduledExtractorSc16::start()
{
    std::lock_guard<std::mutex> lock(d_mutex_);
    d_current_sample_ = 0;
    d_next_schedule_index_ = 0;
    d_active_slot_ = -1;
    d_job_head_ = d_job_tail_ = d_job_count_ = d_jobs_in_flight_ = 0;
    d_stop_worker_ = false;
    d_scheduled_ = d_completed_ = d_emitted_ = d_dropped_ = 0;
    d_process_total_us_ = d_copy_total_us_ = d_publish_total_us_ = 0;
    for (size_t i = 0; i < d_cfg_.pool_size; ++i) {
        d_slots_[i].in_use.store(false, std::memory_order_relaxed);
        d_slots_[i].filled = 0;
        d_slots_[i].meta = core::WindowMeta{};
    }
    d_worker_ = std::thread(&UwbScheduledExtractorSc16::worker_loop, this);
    return true;
}

void UwbScheduledExtractorSc16::begin_next_window()
{
    if (d_active_slot_ >= 0)
        return;
    const auto b = core::window_bounds_for_slot(
        d_cfg_.first_packet_sample_exact, d_cfg_.period_samples_exact(),
        d_next_schedule_index_, d_cfg_.pre_guard_samples, d_cfg_.capture_samples,
        d_cfg_.post_guard_samples);
    if (d_current_sample_ < static_cast<uint64_t>(std::max<int64_t>(0, b.window_start)))
        return;
    if (d_current_sample_ >= static_cast<uint64_t>(std::max<int64_t>(0, b.window_end))) {
        ++d_next_schedule_index_;
        ++d_dropped_;
        return;
    }
    int free_slot = -1;
    for (size_t i = 0; i < d_cfg_.pool_size; ++i) {
        if (!d_slots_[i].in_use.load(std::memory_order_acquire)) {
            free_slot = static_cast<int>(i);
            break;
        }
    }
    ++d_scheduled_;
    if (free_slot < 0) {
        ++d_dropped_;
        ++d_next_schedule_index_;
        return;
    }
    Slot& s = d_slots_[static_cast<size_t>(free_slot)];
    s.in_use.store(true, std::memory_order_release);
    s.filled = 0;
    s.meta = core::WindowMeta{};
    s.meta.schedule_index = d_next_schedule_index_;
    s.meta.predicted_start_sample = b.predicted_start;
    s.meta.window_start_sample = b.window_start;
    s.meta.window_end_sample = b.window_end;
    d_active_slot_ = free_slot;
}

void UwbScheduledExtractorSc16::finish_active_window()
{
    if (d_active_slot_ < 0)
        return;
    const size_t idx = static_cast<size_t>(d_active_slot_);
    Slot& s = d_slots_[idx];
    s.meta.sample_count = s.filled;
    if (s.filled != d_cfg_.window_capacity()) {
        s.in_use.store(false, std::memory_order_release);
        ++d_dropped_;
    } else {
        std::lock_guard<std::mutex> lock(d_mutex_);
        if (d_job_count_ < d_cfg_.pool_size) {
            d_jobs_[d_job_tail_] = idx;
            d_job_tail_ = (d_job_tail_ + 1) % d_cfg_.pool_size;
            ++d_job_count_;
            ++d_completed_;
            d_cv_.notify_one();
        } else {
            s.in_use.store(false, std::memory_order_release);
            ++d_dropped_;
        }
    }
    d_active_slot_ = -1;
    ++d_next_schedule_index_;
}

int UwbScheduledExtractorSc16::work(int noutput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star&)
{
    const auto process_begin = std::chrono::steady_clock::now();
    const auto* in = static_cast<const int16_t*>(input_items[0]);
    const int consumed = noutput_items > 1 ? noutput_items - 1 : noutput_items;
    size_t offset = 0;
    while (offset < static_cast<size_t>(consumed)) {
        begin_next_window();
        const auto b = core::window_bounds_for_slot(
            d_cfg_.first_packet_sample_exact, d_cfg_.period_samples_exact(),
            d_next_schedule_index_, d_cfg_.pre_guard_samples, d_cfg_.capture_samples,
            d_cfg_.post_guard_samples);
        const uint64_t ws = static_cast<uint64_t>(std::max<int64_t>(0, b.window_start));
        const uint64_t we = static_cast<uint64_t>(std::max<int64_t>(0, b.window_end));
        const uint64_t chunk_end = d_current_sample_ + (static_cast<size_t>(consumed) - offset);
        if (d_current_sample_ < ws) {
            const size_t skip = static_cast<size_t>(std::min<uint64_t>(chunk_end, ws) - d_current_sample_);
            d_current_sample_ += skip;
            offset += skip;
            continue;
        }
        if (d_active_slot_ < 0) {
            const size_t skip = static_cast<size_t>(std::min<uint64_t>(chunk_end, we) - d_current_sample_);
            d_current_sample_ += skip;
            offset += skip;
            continue;
        }
        const size_t take = static_cast<size_t>(std::min<uint64_t>(chunk_end, we) - d_current_sample_);
        Slot& s = d_slots_[static_cast<size_t>(d_active_slot_)];
        const auto copy_begin = std::chrono::steady_clock::now();
        std::memcpy(s.iq.data() + 2 * s.filled, in + 2 * offset,
                    2 * take * sizeof(int16_t));
        d_copy_total_us_.fetch_add(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - copy_begin).count()),
            std::memory_order_relaxed);
        s.filled += take;
        d_current_sample_ += take;
        offset += take;
        if (d_current_sample_ == we)
            finish_active_window();
    }
    d_process_total_us_.fetch_add(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - process_begin).count()),
        std::memory_order_relaxed);
    // Preserve GNU Radio's one-sample EOS sentinel convention: let the
    // message worker publish completed windows while downstream ports are
    // still active, rather than draining only from stop().
    if (noutput_items == 1) {
        std::unique_lock<std::mutex> lock(d_mutex_);
        d_cv_.wait(lock, [this] {
            return d_job_count_ == 0 && d_jobs_in_flight_ == 0;
        });
    }
    return consumed;
}

void UwbScheduledExtractorSc16::publish_slot(size_t idx)
{
    const auto publish_begin = std::chrono::steady_clock::now();
    Slot& s = d_slots_[idx];
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("packet_id"), pmt::from_uint64(s.meta.schedule_index));
    meta = pmt::dict_add(meta, pmt::mp("schedule_index"), pmt::from_uint64(s.meta.schedule_index));
    meta = pmt::dict_add(meta, pmt::mp("predicted_start_sample"), pmt::from_long(s.meta.predicted_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("window_start_sample"), pmt::from_long(s.meta.window_start_sample));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"), pmt::from_long(static_cast<long>(s.filled)));
    meta = pmt::dict_add(meta, pmt::mp("pre_guard_samples"), pmt::from_long(static_cast<long>(d_cfg_.pre_guard_samples)));
    meta = pmt::dict_add(meta, pmt::mp("capture_samples"), pmt::from_long(static_cast<long>(d_cfg_.capture_samples)));
    meta = pmt::dict_add(meta, pmt::mp("post_guard_samples"), pmt::from_long(static_cast<long>(d_cfg_.post_guard_samples)));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"), pmt::from_double(d_cfg_.sample_rate));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"), pmt::mp("sc16"));
    message_port_pub(pmt::mp("packet"), pmt::cons(meta, pmt::init_s16vector(2 * s.filled, s.iq.data())));
    d_publish_total_us_.fetch_add(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - publish_begin).count()),
        std::memory_order_relaxed);
    ++d_emitted_;
}

void UwbScheduledExtractorSc16::worker_loop()
{
    for (;;) {
        size_t idx = 0;
        {
            std::unique_lock<std::mutex> lock(d_mutex_);
            d_cv_.wait(lock, [this] { return d_stop_worker_ || d_job_count_ > 0; });
            if (d_job_count_ == 0 && d_stop_worker_)
                return;
            idx = d_jobs_[d_job_head_];
            d_job_head_ = (d_job_head_ + 1) % d_cfg_.pool_size;
            --d_job_count_;
            ++d_jobs_in_flight_;
        }
        publish_slot(idx);
        {
            std::lock_guard<std::mutex> lock(d_mutex_);
            d_slots_[idx].in_use.store(false, std::memory_order_release);
            --d_jobs_in_flight_;
        }
        d_cv_.notify_all();
    }
}

void UwbScheduledExtractorSc16::shutdown_worker()
{
    if (!d_worker_.joinable()) return;
    { std::lock_guard<std::mutex> lock(d_mutex_); d_stop_worker_ = true; }
    d_cv_.notify_all();
    d_worker_.join();
}

bool UwbScheduledExtractorSc16::stop()
{
    finish_active_window();
    shutdown_worker();
    return true;
}

} // namespace uwb
} // namespace gr
