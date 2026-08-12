/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbPduRationalResamplerCcf65_48 — PDU-level fixed 65/48 CF32 resampler.
 *
 * Block type: gr::block, no stream IO.
 * Scheduler: message-driven only; no forecast / general_work.
 * Hot path: one-shot core process+flush into preallocated scratch; no per-PDU
 * heap growth once the window length is steady (extractor geometry is fixed).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace gr {
namespace uwb {

namespace {

constexpr double kRateTolRel = 1e-6; // relative tolerance on sample_rate check

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
        std::string("UwbPduRationalResamplerCcf65_48: cannot find ") +
        filename + " under testdata/resampler_65_48/");
}

std::vector<float> load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        throw std::runtime_error(
            "UwbPduRationalResamplerCcf65_48: cannot open taps file: " + path);
    }
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0) {
        throw std::runtime_error(
            "UwbPduRationalResamplerCcf65_48: taps file size not a positive "
            "multiple of float32: " +
            path);
    }
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    if (!f) {
        throw std::runtime_error(
            "UwbPduRationalResamplerCcf65_48: failed reading taps: " + path);
    }
    return taps;
}

bool rates_close(double a, double b)
{
    if (a <= 0.0 || b <= 0.0)
        return false;
    return std::abs(a - b) <= kRateTolRel * std::max(std::abs(a), std::abs(b));
}

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

double dict_f64(pmt::pmt_t dict, const char* key, double def)
{
    if (!pmt::is_dict(dict))
        return def;
    pmt::pmt_t v = pmt::dict_ref(dict, pmt::mp(key), pmt::from_double(def));
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

bool dict_has(pmt::pmt_t dict, const char* key)
{
    return pmt::is_dict(dict) && pmt::dict_has_key(dict, pmt::mp(key));
}

} // namespace

// ---------------------------------------------------------------------------
// Taps loading
// ---------------------------------------------------------------------------

std::vector<float>
UwbPduRationalResamplerCcf65_48::load_taps_from_profile_or_path(
    const std::string& taps_file_or_profile)
{
    if (taps_file_or_profile.empty() || taps_file_or_profile == "quality") {
        return load_taps_f32(find_testdata_taps("taps_quality.txt"));
    }
    if (taps_file_or_profile == "realtime") {
        return load_taps_f32(find_testdata_taps("taps_realtime.txt"));
    }
    if (taps_file_or_profile == "quality_minorder") {
        return load_taps_f32(find_testdata_taps("taps_quality_minorder.txt"));
    }
    if (taps_file_or_profile == "realtime_minorder") {
        return load_taps_f32(find_testdata_taps("taps_realtime_minorder.txt"));
    }
    return load_taps_f32(taps_file_or_profile);
}

// ---------------------------------------------------------------------------
// Factories / construction
// ---------------------------------------------------------------------------

UwbPduRationalResamplerCcf65_48::sptr
UwbPduRationalResamplerCcf65_48::make(const std::string& taps_file_or_profile,
                                      double output_sample_rate,
                                      bool validate_input_rate,
                                      EmitPolicy emit_policy)
{
    auto taps = load_taps_from_profile_or_path(taps_file_or_profile);
    return gnuradio::make_block_sptr<UwbPduRationalResamplerCcf65_48>(
        taps, output_sample_rate, validate_input_rate, emit_policy);
}

UwbPduRationalResamplerCcf65_48::sptr
UwbPduRationalResamplerCcf65_48::make_from_taps(const std::vector<float>& taps,
                                                double output_sample_rate,
                                                bool validate_input_rate,
                                                EmitPolicy emit_policy)
{
    return gnuradio::make_block_sptr<UwbPduRationalResamplerCcf65_48>(
        taps, output_sample_rate, validate_input_rate, emit_policy);
}

UwbPduRationalResamplerCcf65_48::UwbPduRationalResamplerCcf65_48(
    const std::vector<float>& taps,
    double output_sample_rate,
    bool validate_input_rate,
    EmitPolicy emit_policy)
    : gr::block("uwb_pdu_rational_resampler_ccf_65_48",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_taps_(taps),
      d_core_(std::make_unique<core::RationalResampler65_48Core>(taps)),
      d_output_rate_(output_sample_rate),
      d_validate_rate_(validate_input_rate),
      d_emit_policy_(emit_policy)
{
    if (d_taps_.empty()) {
        throw std::invalid_argument(
            "UwbPduRationalResamplerCcf65_48: taps must be non-empty");
    }
    if (d_output_rate_ <= 0.0) {
        throw std::invalid_argument(
            "UwbPduRationalResamplerCcf65_48: output_sample_rate must be > 0");
    }

    // Typical scheduled window at 737.28 (~pre+capture+post) → ~280k out.
    // Reserve once so steady-state PDUs never reallocate.
    const size_t typical_in = 220000;
    const size_t typical_out =
        core::RationalResampler65_48Core::expected_output_length(
            typical_in, d_taps_.size());
    d_scratch_.reserve(typical_out + 256);
    d_input_scratch_.reserve(typical_in);

    message_port_register_in(pmt::mp("packet"));
    message_port_register_out(pmt::mp("packet"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("packet"),
                    [this](pmt::pmt_t msg) { handle_packet(msg); });
}

UwbPduRationalResamplerCcf65_48::~UwbPduRationalResamplerCcf65_48() = default;

void
UwbPduRationalResamplerCcf65_48::reset_stats()
{
    d_pdus_received_.store(0, std::memory_order_relaxed);
    d_pdus_emitted_.store(0, std::memory_order_relaxed);
    d_pdus_dropped_.store(0, std::memory_order_relaxed);
    d_total_in_.store(0, std::memory_order_relaxed);
    d_total_out_.store(0, std::memory_order_relaxed);
    d_resets_.store(0, std::memory_order_relaxed);
    d_short_guard_.store(0, std::memory_order_relaxed);
    d_resample_total_us_.store(0, std::memory_order_relaxed);
    d_resample_max_us_.store(0, std::memory_order_relaxed);
    d_handler_total_us_.store(0, std::memory_order_relaxed);
    d_input_convert_total_us_.store(0, std::memory_order_relaxed);
    d_publish_total_us_.store(0, std::memory_order_relaxed);
    d_short_guard_logged_ = false;
}

// ---------------------------------------------------------------------------
// Scratch / one-shot resample
// ---------------------------------------------------------------------------

void
UwbPduRationalResamplerCcf65_48::ensure_scratch(size_t n_out)
{
    if (d_scratch_.size() < n_out)
        d_scratch_.resize(n_out);
}

void
UwbPduRationalResamplerCcf65_48::resample_oneshot(const gr_complex* in,
                                                   size_t n_in,
                                                   size_t* n_out)
{
    // Fresh FIR state per PDU (each window is an independent capture).
    d_core_->reset();
    d_resets_.fetch_add(1, std::memory_order_relaxed);

    const size_t Lout =
        core::RationalResampler65_48Core::expected_output_length(
            n_in, d_taps_.size());
    ensure_scratch(Lout + 64);

    size_t produced = 0;
    if (n_in > 0) {
        auto r = d_core_->process(in, n_in, d_scratch_.data(), d_scratch_.size());
        produced = r.produced;
    }
    while (produced < Lout) {
        if (produced >= d_scratch_.size())
            d_scratch_.resize(produced + 256);
        const size_t n =
            d_core_->flush(d_scratch_.data() + produced,
                           d_scratch_.size() - produced);
        if (n == 0)
            break;
        produced += n;
    }
    *n_out = produced;
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

void
UwbPduRationalResamplerCcf65_48::publish_status(const std::string& event,
                                                 pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    meta = pmt::dict_add(meta, pmt::mp("pdus_received"),
                         pmt::from_uint64(pdus_received()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_emitted"),
                         pmt::from_uint64(pdus_emitted()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_dropped"),
                         pmt::from_uint64(pdus_dropped()));
    meta = pmt::dict_add(meta, pmt::mp("total_input_samples"),
                         pmt::from_uint64(total_input_samples()));
    meta = pmt::dict_add(meta, pmt::mp("total_output_samples"),
                         pmt::from_uint64(total_output_samples()));
    if (pmt::is_dict(extra)) {
        pmt::pmt_t items = pmt::dict_items(extra);
        for (size_t i = 0; i < pmt::length(items); ++i) {
            pmt::pmt_t kv = pmt::nth(i, items);
            meta = pmt::dict_add(meta, pmt::car(kv), pmt::cdr(kv));
        }
    }
    message_port_pub(pmt::mp("status"), meta);
}

// ---------------------------------------------------------------------------
// Message handler
// ---------------------------------------------------------------------------

void
UwbPduRationalResamplerCcf65_48::handle_packet(pmt::pmt_t msg)
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
    if (!pmt::is_c32vector(data_in) && !pmt::is_s16vector(data_in)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    const bool input_sc16 = pmt::is_s16vector(data_in);
    const auto handler_begin = std::chrono::steady_clock::now();
    const size_t payload_items = pmt::length(data_in);
    if (payload_items == 0 || (input_sc16 && (payload_items & 1u) != 0)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }

    const double in_rate = dict_f64(meta_in, "sample_rate", 0.0);
    if (d_validate_rate_ && !rates_close(in_rate, kInputRateHz)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        pmt::pmt_t extra = pmt::make_dict();
        extra = pmt::dict_add(extra, pmt::mp("sample_rate"),
                              pmt::from_double(in_rate));
        extra = pmt::dict_add(extra, pmt::mp("expected_sample_rate"),
                              pmt::from_double(kInputRateHz));
        publish_status("bad_input_rate", extra);
        return;
    }

    const size_t n_in = input_sc16 ? payload_items / 2 : payload_items;
    size_t n_out = 0;
    size_t n_elem = 0;
    const gr_complex* in_ptr = nullptr;
    if (!input_sc16) {
        in_ptr = pmt::c32vector_elements(data_in, n_elem);
    } else {
        const auto convert_begin = std::chrono::steady_clock::now();
        const int16_t* s16 = pmt::s16vector_elements(data_in, n_elem);
        if (s16 != nullptr && n_elem == payload_items) {
            if (d_input_scratch_.size() < n_in)
                d_input_scratch_.resize(n_in);
            for (size_t i = 0; i < n_in; ++i) {
                d_input_scratch_[i] = gr_complex(
                    static_cast<float>(s16[2 * i]),
                    static_cast<float>(s16[2 * i + 1]));
            }
            in_ptr = d_input_scratch_.data();
            n_elem = n_in;
        }
        d_input_convert_total_us_.fetch_add(static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - convert_begin).count()),
            std::memory_order_relaxed);
    }
    if (in_ptr == nullptr || n_elem != n_in) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_input");
        return;
    }
    const auto resample_begin = std::chrono::steady_clock::now();
    resample_oneshot(in_ptr, n_in, &n_out);
    const uint64_t resample_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - resample_begin).count());
    d_resample_total_us_.fetch_add(resample_us, std::memory_order_relaxed);
    uint64_t prior_max = d_resample_max_us_.load(std::memory_order_relaxed);
    while (prior_max < resample_us &&
           !d_resample_max_us_.compare_exchange_weak(
               prior_max, resample_us, std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
    if (n_out == 0) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("empty_output");
        return;
    }

    d_total_in_.fetch_add(n_in, std::memory_order_relaxed);

    // ---- coordinate mapping (absolute sample domain) ----
    const int64_t ws = dict_i64(meta_in, "window_start_sample", 0);
    const int64_t pre_in = dict_i64(meta_in, "pre_guard_samples", 0);
    const int64_t cap_in = dict_i64(meta_in, "capture_samples",
                                    static_cast<int64_t>(n_in));
    // post may be absent from extractor meta; recover from sample_count.
    int64_t post_in = dict_i64(meta_in, "post_guard_samples", -1);
    if (post_in < 0) {
        const int64_t sc =
            dict_i64(meta_in, "sample_count", static_cast<int64_t>(n_in));
        post_in = std::max<int64_t>(0, sc - pre_in - cap_in);
    }

    const int64_t ws_out = map_input_offset_to_output(ws);
    int64_t pre_out =
        map_input_offset_to_output(ws + pre_in) - map_input_offset_to_output(ws);
    int64_t cap_out = map_input_offset_to_output(ws + pre_in + cap_in) -
                      map_input_offset_to_output(ws + pre_in);
    if (pre_out < 0)
        pre_out = 0;
    if (cap_out < 0)
        cap_out = 0;

    // Crop bounds: clamp pre+capture into [0, Lout]; short guards → status.
    bool short_guard = false;
    if (static_cast<size_t>(pre_out) > n_out) {
        pre_out = static_cast<int64_t>(n_out);
        short_guard = true;
    }
    if (static_cast<size_t>(pre_out + cap_out) > n_out) {
        cap_out = static_cast<int64_t>(n_out) - pre_out;
        if (cap_out < 0)
            cap_out = 0;
        short_guard = true;
    }
    int64_t post_out =
        static_cast<int64_t>(n_out) - pre_out - cap_out;
    if (post_out < 0) {
        post_out = 0;
        short_guard = true;
    }
    // Also flag if input guards cannot cover the FIR group-delay transient.
    const double d_virt =
        0.5 * static_cast<double>(d_taps_.size() > 0 ? d_taps_.size() - 1 : 0);
    const int64_t gd_in =
        static_cast<int64_t>(std::llround(d_virt / static_cast<double>(kInterp)));
    if (pre_in < gd_in || post_in < gd_in)
        short_guard = true;

    if (short_guard) {
        d_short_guard_.fetch_add(1, std::memory_order_relaxed);
        if (!d_short_guard_logged_) {
            d_short_guard_logged_ = true;
            pmt::pmt_t extra = pmt::make_dict();
            extra = pmt::dict_add(extra, pmt::mp("pre_guard_in"),
                                  pmt::from_long(pre_in));
            extra = pmt::dict_add(extra, pmt::mp("post_guard_in"),
                                  pmt::from_long(post_in));
            extra = pmt::dict_add(extra, pmt::mp("Lout"),
                                  pmt::from_long(static_cast<long>(n_out)));
            publish_status("short_guard", extra);
        }
    }

    // ---- emit payload ----
    size_t emit_off = 0;
    size_t emit_len = n_out;
    int64_t emit_pre = pre_out;
    int64_t emit_cap = cap_out;
    int64_t emit_post = post_out;
    int64_t emit_ws = ws_out;

    if (d_emit_policy_ == EmitPolicy::CaptureOnly) {
        emit_off = static_cast<size_t>(pre_out);
        emit_len = static_cast<size_t>(cap_out);
        emit_pre = 0;
        emit_cap = static_cast<int64_t>(emit_len);
        emit_post = 0;
        emit_ws = map_input_offset_to_output(ws + pre_in);
    }

    pmt::pmt_t meta_out = pmt::make_dict();
    // Preserve packet identity / schedule lineage.
    if (dict_has(meta_in, "packet_id")) {
        meta_out = pmt::dict_add(meta_out, pmt::mp("packet_id"),
                                 pmt::dict_ref(meta_in, pmt::mp("packet_id"),
                                               pmt::from_uint64(0)));
    }
    if (dict_has(meta_in, "schedule_index")) {
        meta_out =
            pmt::dict_add(meta_out, pmt::mp("schedule_index"),
                          pmt::dict_ref(meta_in, pmt::mp("schedule_index"),
                                        pmt::from_uint64(0)));
    }
    if (dict_has(meta_in, "schedule_generation")) {
        meta_out = pmt::dict_add(
            meta_out, pmt::mp("schedule_generation"),
            pmt::dict_ref(meta_in, pmt::mp("schedule_generation"),
                          pmt::from_uint64(0)));
    }

    meta_out = pmt::dict_add(meta_out, pmt::mp("sample_rate"),
                             pmt::from_double(d_output_rate_));
    meta_out = pmt::dict_add(meta_out, pmt::mp("window_start_sample"),
                             pmt::from_long(emit_ws));
    meta_out = pmt::dict_add(meta_out, pmt::mp("pre_guard_samples"),
                             pmt::from_long(emit_pre));
    meta_out = pmt::dict_add(meta_out, pmt::mp("capture_samples"),
                             pmt::from_long(emit_cap));
    meta_out = pmt::dict_add(meta_out, pmt::mp("post_guard_samples"),
                             pmt::from_long(emit_post));
    meta_out = pmt::dict_add(meta_out, pmt::mp("sample_count"),
                             pmt::from_long(static_cast<long>(emit_len)));

    if (dict_has(meta_in, "predicted_start_sample")) {
        const int64_t p = dict_i64(meta_in, "predicted_start_sample", -1);
        if (p >= 0) {
            meta_out = pmt::dict_add(meta_out, pmt::mp("predicted_start_sample"),
                                     pmt::from_long(map_input_offset_to_output(p)));
        }
    }
    if (dict_has(meta_in, "detected_start_sample")) {
        const int64_t p = dict_i64(meta_in, "detected_start_sample", -1);
        if (p >= 0) {
            meta_out = pmt::dict_add(meta_out, pmt::mp("detected_start_sample"),
                                     pmt::from_long(map_input_offset_to_output(p)));
        }
    }

    // Resample provenance.
    meta_out = pmt::dict_add(meta_out, pmt::mp("resample_interp"),
                             pmt::from_long(static_cast<long>(kInterp)));
    meta_out = pmt::dict_add(meta_out, pmt::mp("resample_decim"),
                             pmt::from_long(static_cast<long>(kDecim)));
    const double filter_delay =
        0.5 * static_cast<double>(d_taps_.size() > 0 ? d_taps_.size() - 1 : 0);
    meta_out = pmt::dict_add(meta_out, pmt::mp("resample_filter_delay"),
                             pmt::from_double(filter_delay));
    meta_out = pmt::dict_add(meta_out, pmt::mp("input_sample_rate"),
                             pmt::from_double(kInputRateHz));
    meta_out = pmt::dict_add(meta_out, pmt::mp("output_sample_rate"),
                             pmt::from_double(d_output_rate_));
    meta_out = pmt::dict_add(meta_out, pmt::mp("input_sample_count"),
                             pmt::from_long(static_cast<long>(n_in)));
    meta_out = pmt::dict_add(meta_out, pmt::mp("input_sample_format"),
                             pmt::mp(input_sc16 ? "sc16" : "fc32"));
    meta_out = pmt::dict_add(meta_out, pmt::mp("sample_format"),
                             pmt::mp("fc32"));
    meta_out = pmt::dict_add(meta_out, pmt::mp("full_output_sample_count"),
                             pmt::from_long(static_cast<long>(n_out)));
    meta_out = pmt::dict_add(meta_out, pmt::mp("resample_us"),
                             pmt::from_uint64(resample_us));

    const auto publish_begin = std::chrono::steady_clock::now();
    pmt::pmt_t vec = pmt::init_c32vector(
        emit_len, d_scratch_.data() + emit_off);
    message_port_pub(pmt::mp("packet"), pmt::cons(meta_out, vec));
    d_publish_total_us_.fetch_add(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - publish_begin).count()),
        std::memory_order_relaxed);
    d_handler_total_us_.fetch_add(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - handler_begin).count()),
        std::memory_order_relaxed);

    d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
    d_total_out_.fetch_add(emit_len, std::memory_order_relaxed);
}

} // namespace uwb
} // namespace gr
