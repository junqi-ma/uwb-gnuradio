/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPduWindowCrop — message-only native-rate crop before 65/48.
 *
 * Scheduler: packet handler only; no forecast / general_work.
 * Hot path: copy the cropped slice into reserved scratch, then wrap a
 * new PMT vector.  Acquisition / missing-pred PDUs are republished.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_pdu_window_crop.h>

#include <algorithm>
#include <cstring>

namespace gr {
namespace uwb {

namespace {

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def));
    if (pmt::is_uint64(v))
        return static_cast<int64_t>(pmt::to_uint64(v));
    if (pmt::is_integer(v))
        return static_cast<int64_t>(pmt::to_long(v));
    return def;
}

std::string dict_str(pmt::pmt_t dict, const char* key)
{
    if (!pmt::is_dict(dict))
        return {};
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_symbol(v))
        return pmt::symbol_to_string(v);
    if (pmt::is_bool(v))
        return pmt::to_bool(v) ? "true" : "false";
    return {};
}

} // namespace

UwbPduWindowCrop::sptr
UwbPduWindowCrop::make(size_t pre_samples,
                       size_t capture_samples,
                       size_t post_samples)
{
    return gnuradio::make_block_sptr<UwbPduWindowCrop>(
        pre_samples, capture_samples, post_samples);
}

UwbPduWindowCrop::UwbPduWindowCrop(size_t pre_samples,
                                   size_t capture_samples,
                                   size_t post_samples)
    : gr::block("uwb_pdu_window_crop",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0))
{
    set_geometry(pre_samples, capture_samples, post_samples);
    const size_t typical = defaults::kNativeScheduledPreGuard +
                           defaults::kNativeScheduledCapture +
                           defaults::kNativeScheduledPostGuard;
    d_s16_scratch_.reserve(typical * 2);
    d_c32_scratch_.reserve(typical);

    message_port_register_in(pmt::mp("packet"));
    message_port_register_out(pmt::mp("packet"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("packet"),
                    [this](pmt::pmt_t msg) { handle_packet(msg); });
}

UwbPduWindowCrop::~UwbPduWindowCrop() = default;

void
UwbPduWindowCrop::set_geometry(size_t pre, size_t capture, size_t post)
{
    d_geom_.pre = static_cast<int64_t>(pre);
    d_geom_.capture = static_cast<int64_t>(capture);
    d_geom_.post = static_cast<int64_t>(post);
}

void
UwbPduWindowCrop::reset_stats()
{
    d_pdus_received_.store(0, std::memory_order_relaxed);
    d_pdus_emitted_.store(0, std::memory_order_relaxed);
    d_pdus_passthrough_.store(0, std::memory_order_relaxed);
    d_pdus_cropped_.store(0, std::memory_order_relaxed);
    d_pdus_clamped_.store(0, std::memory_order_relaxed);
    d_pdus_dropped_.store(0, std::memory_order_relaxed);
    d_total_in_.store(0, std::memory_order_relaxed);
    d_total_out_.store(0, std::memory_order_relaxed);
    d_clamp_logged_ = false;
}

void
UwbPduWindowCrop::publish_status(const char* event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    meta = pmt::dict_add(meta, pmt::mp("pdus_received"),
                         pmt::from_uint64(pdus_received()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_emitted"),
                         pmt::from_uint64(pdus_emitted()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_passthrough"),
                         pmt::from_uint64(pdus_passthrough()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_cropped"),
                         pmt::from_uint64(pdus_cropped()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_clamped"),
                         pmt::from_uint64(pdus_clamped()));
    if (pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

void
UwbPduWindowCrop::handle_packet(pmt::pmt_t msg)
{
    d_pdus_received_.fetch_add(1, std::memory_order_relaxed);
    if (!pmt::is_pair(msg)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    pmt::pmt_t meta_in = pmt::car(msg);
    pmt::pmt_t data_in = pmt::cdr(msg);
    if (!pmt::is_dict(meta_in))
        meta_in = pmt::make_dict();
    const bool input_sc16 = pmt::is_s16vector(data_in);
    const bool input_c32 = pmt::is_c32vector(data_in);
    if (!input_sc16 && !input_c32) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    const size_t payload_items = pmt::length(data_in);
    if (payload_items == 0 || (input_sc16 && (payload_items & 1u) != 0)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    const size_t n_in = input_sc16 ? payload_items / 2 : payload_items;
    d_total_in_.fetch_add(n_in, std::memory_order_relaxed);

    const int64_t ws = dict_i64(meta_in, "window_start_sample",
                                dict_i64(meta_in, "start_sample", 0));
    const int64_t pred = dict_i64(meta_in, "predicted_start_sample", -1);
    const std::string mode = dict_str(meta_in, "capture_mode");
    const bool acquisition = (mode == "acquisition");

    const core::WindowCropPlan plan =
        core::plan_window_crop(ws, static_cast<int64_t>(n_in), pred, d_geom_,
                               acquisition);

    if (plan.empty) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status(plan.reason);
        return;
    }

    if (plan.passthrough) {
        d_pdus_passthrough_.fetch_add(1, std::memory_order_relaxed);
        d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
        d_total_out_.fetch_add(n_in, std::memory_order_relaxed);
        message_port_pub(pmt::mp("packet"), msg);
        if (std::strcmp(plan.reason, "acquisition_passthrough") == 0 ||
            std::strcmp(plan.reason, "missing_predicted_start") == 0) {
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("reason"),
                                  pmt::mp(plan.reason));
            extra = pmt::dict_add(extra, pmt::mp("sample_count"),
                                  pmt::from_long(static_cast<long>(n_in)));
            publish_status("passthrough", extra);
        }
        return;
    }

    const size_t off = static_cast<size_t>(plan.in_offset);
    const size_t n_out = static_cast<size_t>(plan.out_count);
    pmt::pmt_t data_out;
    if (input_sc16) {
        size_t n_elem = 0;
        const int16_t* s16 = pmt::s16vector_elements(data_in, n_elem);
        if (s16 == nullptr || n_elem != payload_items ||
            off + n_out > n_in) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
        if (d_s16_scratch_.size() < n_out * 2)
            d_s16_scratch_.resize(n_out * 2);
        std::memcpy(d_s16_scratch_.data(), s16 + off * 2,
                    n_out * 2 * sizeof(int16_t));
        data_out = pmt::init_s16vector(n_out * 2, d_s16_scratch_.data());
    } else {
        size_t n_elem = 0;
        const gr_complex* c32 = pmt::c32vector_elements(data_in, n_elem);
        if (c32 == nullptr || n_elem != n_in || off + n_out > n_in) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_input");
            return;
        }
        if (d_c32_scratch_.size() < n_out)
            d_c32_scratch_.resize(n_out);
        std::memcpy(d_c32_scratch_.data(), c32 + off,
                    n_out * sizeof(gr_complex));
        data_out = pmt::init_c32vector(n_out, d_c32_scratch_.data());
    }

    pmt::pmt_t meta_out = meta_in;
    meta_out = pmt::dict_add(meta_out, pmt::mp("window_start_sample"),
                             pmt::from_long(plan.window_start));
    meta_out = pmt::dict_add(meta_out, pmt::mp("start_sample"),
                             pmt::from_long(plan.window_start));
    meta_out = pmt::dict_add(meta_out, pmt::mp("pre_guard_samples"),
                             pmt::from_long(plan.pre));
    meta_out = pmt::dict_add(meta_out, pmt::mp("pre_trigger_samples"),
                             pmt::from_long(plan.pre));
    meta_out = pmt::dict_add(meta_out, pmt::mp("capture_samples"),
                             pmt::from_long(plan.capture));
    meta_out = pmt::dict_add(meta_out, pmt::mp("post_guard_samples"),
                             pmt::from_long(plan.post));
    meta_out = pmt::dict_add(meta_out, pmt::mp("sample_count"),
                             pmt::from_long(static_cast<long>(n_out)));
    meta_out = pmt::dict_add(meta_out, pmt::mp("native_dump_window_start"),
                             pmt::from_long(ws));
    meta_out = pmt::dict_add(meta_out, pmt::mp("native_dump_sample_count"),
                             pmt::from_long(static_cast<long>(n_in)));

    d_pdus_cropped_.fetch_add(1, std::memory_order_relaxed);
    if (plan.clamped_start || plan.clamped_end) {
        d_pdus_clamped_.fetch_add(1, std::memory_order_relaxed);
        if (!d_clamp_logged_) {
            d_clamp_logged_ = true;
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("window_start_sample"),
                                  pmt::from_long(plan.window_start));
            extra = pmt::dict_add(extra, pmt::mp("pre_guard_samples"),
                                  pmt::from_long(plan.pre));
            extra = pmt::dict_add(extra, pmt::mp("sample_count"),
                                  pmt::from_long(static_cast<long>(n_out)));
            extra = pmt::dict_add(extra, pmt::mp("clamped_start"),
                                  pmt::from_bool(plan.clamped_start));
            extra = pmt::dict_add(extra, pmt::mp("clamped_end"),
                                  pmt::from_bool(plan.clamped_end));
            publish_status("clamped", extra);
        }
    }

    d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
    d_total_out_.fetch_add(n_out, std::memory_order_relaxed);
    message_port_pub(pmt::mp("packet"), pmt::cons(meta_out, data_out));
}

} // namespace uwb
} // namespace gr
