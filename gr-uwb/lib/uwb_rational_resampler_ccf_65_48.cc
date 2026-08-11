/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRationalResamplerCcf65_48 — fixed 65/48 CF32 resampler adapter.
 *
 * Block type: gr::block
 * Scheduler semantics:
 *   - set_relative_rate(65, 48)     integer contract
 *   - set_output_multiple(65)       macroblock-aligned grants when possible
 *   - set_max_noutput_items(1<<20)
 *   - set_tag_propagation_policy(TPP_DONT) + manual tag mapping
 *   - forecast: macroblocks*48 + (H-1); integer only; 0 while flushing
 *   - general_work: core.process / core.flush; consume_each(exact consumed);
 *     return produced.  No alloc / PMT dict / logging in the hot path.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/block_detail.h>
#include <gnuradio/buffer_reader.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h>

#include <fstream>
#include <stdexcept>

namespace gr {
namespace uwb {

namespace {

std::string find_testdata_taps(const char* filename)
{
    const char* prefixes[] = {
        "testdata/resampler_65_48/",
        "../testdata/resampler_65_48/",
        "../../testdata/resampler_65_48/",
        "../../../testdata/resampler_65_48/",
        "../../../../testdata/resampler_65_48/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + filename;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    throw std::runtime_error(
        std::string("UwbRationalResamplerCcf65_48: cannot find ") + filename +
        " under testdata/resampler_65_48/ (tried several cwd depths)");
}

std::vector<float> load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_48: cannot open taps file: " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0) {
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_48: taps file size not a positive "
            "multiple of float32: " +
            path);
    }
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    if (!f) {
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_48: failed reading taps: " + path);
    }
    return taps;
}

} // namespace

std::vector<float> UwbRationalResamplerCcf65_48::load_taps_from_profile_or_path(
    const std::string& taps_file_or_profile)
{
    if (taps_file_or_profile.empty() || taps_file_or_profile == "quality") {
        return load_taps_f32(find_testdata_taps("taps_quality.txt"));
    }
    if (taps_file_or_profile == "realtime") {
        return load_taps_f32(find_testdata_taps("taps_realtime.txt"));
    }
    return load_taps_f32(taps_file_or_profile);
}

UwbRationalResamplerCcf65_48::sptr
UwbRationalResamplerCcf65_48::make(const std::string& taps_file_or_profile,
                                   bool tag_propagation_enable,
                                   bool reset_on_discontinuity,
                                   int num_workers)
{
    auto taps = load_taps_from_profile_or_path(taps_file_or_profile);
    return gnuradio::make_block_sptr<UwbRationalResamplerCcf65_48>(
        taps, tag_propagation_enable, reset_on_discontinuity, num_workers);
}

UwbRationalResamplerCcf65_48::sptr
UwbRationalResamplerCcf65_48::make_from_taps(const std::vector<float>& taps,
                                             bool tag_propagation_enable,
                                             bool reset_on_discontinuity,
                                             int num_workers)
{
    return gnuradio::make_block_sptr<UwbRationalResamplerCcf65_48>(
        taps, tag_propagation_enable, reset_on_discontinuity, num_workers);
}

UwbRationalResamplerCcf65_48::UwbRationalResamplerCcf65_48(
    const std::vector<float>& taps,
    bool tag_propagation_enable,
    bool reset_on_discontinuity,
    int num_workers)
    : gr::block("uwb_rational_resampler_ccf_65_48",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
      d_taps_(taps),
      d_core_(std::make_unique<core::RationalResampler65_48Core>(taps)),
      d_tag_prop_(tag_propagation_enable),
      d_reset_on_disc_(reset_on_discontinuity),
      d_key_rx_time_(pmt::intern("rx_time")),
      d_key_rx_rate_(pmt::intern("rx_rate")),
      d_key_sample_rate_(pmt::intern("sample_rate")),
      d_key_rx_freq_(pmt::intern("rx_freq")),
      d_key_overflow_(pmt::intern("overflow")),
      d_key_discontinuity_(pmt::intern("discontinuity")),
      d_key_resampler_reset_(pmt::intern("resampler_reset"))
{
    if (d_taps_.empty()) {
        throw std::invalid_argument(
            "UwbRationalResamplerCcf65_48: taps must be non-empty");
    }

    set_relative_rate(static_cast<uint64_t>(kInterp),
                      static_cast<uint64_t>(kDecim));
    // Steady-state macroblock alignment.  Final upfirdn length is not always
    // a multiple of 65; general_work may return fewer than the grant, and
    // the done()-aware forecast path drains the FIR tail at EOS.
    set_output_multiple(static_cast<int>(kInterp));
    set_max_noutput_items(1048576);
    set_tag_propagation_policy(TPP_DONT);
    d_core_->set_num_workers(num_workers < 1 ? 1 : num_workers);
}

void UwbRationalResamplerCcf65_48::set_num_workers(int n)
{
    d_core_->set_num_workers(n);
}

int UwbRationalResamplerCcf65_48::num_workers() const
{
    return d_core_->num_workers();
}

void UwbRationalResamplerCcf65_48::set_kernel(const std::string& name)
{
    d_core_->set_kernel(name);
}

UwbRationalResamplerCcf65_48::~UwbRationalResamplerCcf65_48() = default;

void UwbRationalResamplerCcf65_48::reset()
{
    d_core_->reset();
    d_flushing_ = false;
    d_finished_ = false;
    d_abs_in_ = 0;
    d_abs_out_ = 0;
    d_stat_resets_.fetch_add(1, std::memory_order_relaxed);
}

void UwbRationalResamplerCcf65_48::forecast(
    int noutput_items, gr_vector_int& ninput_items_required)
{
    if (d_flushing_ || d_finished_) {
        ninput_items_required[0] = 0;
        return;
    }

    // If the upstream reader is already marked done, request at most 1 so
    // any residual buffered samples are drained; when 0 arrive, general_work
    // enters flush.  This is required for upfirdn EOS equivalence — a hard
    // macroblock*48 forecast would strand the short final grant.
    if (detail()) {
        auto reader = detail()->input(0);
        if (reader && reader->done()) {
            ninput_items_required[0] = 1;
            return;
        }
    }

    // Steady state: macroblocks = ceil(noutput_items / 65)
    const int macroblocks =
        (noutput_items + static_cast<int>(kInterp) - 1) /
        static_cast<int>(kInterp);
    // Delay line is internal (no set_history) — do not add (H-1).
    int nreq = macroblocks * static_cast<int>(kDecim);
    if (nreq < 1)
        nreq = 1;
    ninput_items_required[0] = nreq;
}

void UwbRationalResamplerCcf65_48::propagate_tags(int ninput_consumed,
                                                  int noutput_produced,
                                                  uint64_t abs_in_before,
                                                  uint64_t abs_out_before)
{
    if (!d_tag_prop_ || ninput_consumed <= 0)
        return;

    std::vector<tag_t> tags;
    const uint64_t abs_start = nitems_read(0);
    const uint64_t abs_end = abs_start + static_cast<uint64_t>(ninput_consumed);
    get_tags_in_range(tags, 0, abs_start, abs_end);

    for (const tag_t& t : tags) {
        const int64_t local_in =
            static_cast<int64_t>(t.offset) - static_cast<int64_t>(abs_start);
        if (local_in < 0 || local_in >= ninput_consumed) {
            d_stat_tag_err_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        const uint64_t abs_in_sample =
            abs_in_before + static_cast<uint64_t>(local_in);
        const int64_t abs_out_sample = d_core_->map_input_offset_to_output(
            static_cast<int64_t>(abs_in_sample));
        int64_t local_out =
            abs_out_sample - static_cast<int64_t>(abs_out_before);
        if (local_out < 0)
            local_out = 0;
        if (noutput_produced <= 0) {
            // Still handle discontinuity resets even if no output yet.
            if ((pmt::eq(t.key, d_key_overflow_) ||
                 pmt::eq(t.key, d_key_discontinuity_)) &&
                d_reset_on_disc_) {
                d_core_->reset();
                d_flushing_ = false;
                d_finished_ = false;
                d_stat_disc_.fetch_add(1, std::memory_order_relaxed);
                d_stat_resets_.fetch_add(1, std::memory_order_relaxed);
                ++d_disc_epoch_;
            }
            continue;
        }
        if (local_out >= noutput_produced)
            local_out = noutput_produced - 1;

        const uint64_t out_offset =
            nitems_written(0) + static_cast<uint64_t>(local_out);

        if (pmt::eq(t.key, d_key_rx_time_)) {
            add_item_tag(0, out_offset, t.key, t.value, t.srcid);
        } else if (pmt::eq(t.key, d_key_rx_rate_) ||
                   pmt::eq(t.key, d_key_sample_rate_)) {
            add_item_tag(0,
                         out_offset,
                         t.key,
                         pmt::from_double(kOutputRateHz),
                         t.srcid);
        } else if (pmt::eq(t.key, d_key_rx_freq_)) {
            add_item_tag(0, out_offset, t.key, t.value, t.srcid);
        } else if (pmt::eq(t.key, d_key_overflow_) ||
                   pmt::eq(t.key, d_key_discontinuity_)) {
            if (d_reset_on_disc_) {
                d_core_->reset();
                d_flushing_ = false;
                d_finished_ = false;
                d_stat_disc_.fetch_add(1, std::memory_order_relaxed);
                d_stat_resets_.fetch_add(1, std::memory_order_relaxed);
                ++d_disc_epoch_;
                add_item_tag(0,
                             out_offset,
                             d_key_resampler_reset_,
                             pmt::from_uint64(d_disc_epoch_),
                             alias_pmt());
            }
            add_item_tag(0, out_offset, t.key, t.value, t.srcid);
        } else {
            add_item_tag(0, out_offset, t.key, t.value, t.srcid);
        }
    }
}

int UwbRationalResamplerCcf65_48::general_work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    if (d_finished_)
        return WORK_DONE;

    auto* out = static_cast<gr_complex*>(output_items[0]);
    const auto* in = static_cast<const gr_complex*>(input_items[0]);
    const int nin = ninput_items[0];
    const size_t max_out = static_cast<size_t>(noutput_items);

    // Upstream finished? (finite source / head).  Prefer explicit done()
    // so we can flush the upfirdn tail without waiting for a zero-input call
    // that some scheduler paths never deliver when forecast > 0.
    bool upstream_done = false;
    if (detail()) {
        auto reader = detail()->input(0);
        if (reader)
            upstream_done = reader->done();
    }

    // EOS with no remaining input: drain FIR tail.
    if (nin == 0 || (upstream_done && nin == 0)) {
        d_flushing_ = true;
        const size_t produced = d_core_->flush(out, max_out);
        if (produced > 0) {
            d_abs_out_ += produced;
            d_stat_out_.store(d_core_->output_items(),
                              std::memory_order_relaxed);
            return static_cast<int>(produced);
        }
        d_finished_ = true;
        return WORK_DONE;
    }

    const uint64_t abs_in_before = d_abs_in_;
    const uint64_t abs_out_before = d_abs_out_;

    auto r = d_core_->process(
        in, static_cast<size_t>(nin), out, max_out, d_core_->num_workers());

    // Tags before consume so nitems_read() is still the grant start.
    if (d_tag_prop_) {
        propagate_tags(static_cast<int>(r.consumed),
                       static_cast<int>(r.produced),
                       abs_in_before,
                       abs_out_before);
    }

    if (r.consumed > 0)
        consume_each(static_cast<int>(r.consumed));

    d_abs_in_ += r.consumed;
    size_t produced = r.produced;

    // If the upstream is done and we have consumed everything available,
    // continue into flush within this grant so the FIR tail is emitted
    // (upfirdn contract).  Continuous streams never set upstream_done.
    if (upstream_done && r.consumed == static_cast<size_t>(nin) &&
        produced < max_out) {
        d_flushing_ = true;
        produced += d_core_->flush(out + produced, max_out - produced);
        if (d_core_->flush_complete()) {
            // More flush may still be needed on subsequent calls if the
            // output grant was too small; leave d_flushing_ set.
        }
    }

    d_abs_out_ += produced;
    d_stat_in_.store(d_core_->input_items(), std::memory_order_relaxed);
    d_stat_out_.store(d_core_->output_items(), std::memory_order_relaxed);

    if (d_flushing_ && d_core_->flush_complete()) {
        d_finished_ = true;
        // Return produced this call; next entry returns WORK_DONE.
    }

    return static_cast<int>(produced);
}

} // namespace uwb
} // namespace gr
