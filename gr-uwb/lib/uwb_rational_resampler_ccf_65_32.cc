/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRationalResamplerCcf65_32 — fixed 65/32 CF32 tagged-stream resampler.
 *
 * One "packet_len"-delimited window per work() call: reset + process + flush
 * reproduces the one-shot PDU resampler exactly.  Radar metadata tags are
 * mapped from the native to the work domain with the same law as the PDU
 * resampler so the downstream CIR estimator can stay a message block fed by
 * blocks.tagged_stream_to_pdu.
 *
 * Hot path: no allocation except the (small, per-window) tag vectors.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_radar_checked_math.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>
#include <gnuradio/uwb/uwb_rational_resampler_ccf_65_32.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace gr {
namespace uwb {

namespace {

std::string find_testdata_taps(const char* filename)
{
    const char* prefixes[] = {
        "testdata/resampler_65_32/",
        "../testdata/resampler_65_32/",
        "../../testdata/resampler_65_32/",
        "../../../testdata/resampler_65_32/",
        "../../../../testdata/resampler_65_32/",
    };
    for (const char* p : prefixes) {
        const std::string path = std::string(p) + filename;
        std::ifstream f(path, std::ios::binary);
        if (f)
            return path;
    }
    throw std::runtime_error(
        std::string("UwbRationalResamplerCcf65_32: cannot find ") + filename +
        " under testdata/resampler_65_32/ (tried several cwd depths)");
}

std::vector<float> load_taps_f32(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_32: cannot open taps file: " + path);
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0 || bytes % sizeof(float) != 0) {
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_32: taps file size not a positive "
            "multiple of float32: " +
            path);
    }
    f.seekg(0);
    std::vector<float> taps(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(taps.data()),
           static_cast<std::streamsize>(bytes));
    if (!f)
        throw std::runtime_error(
            "UwbRationalResamplerCcf65_32: failed reading taps: " + path);
    return taps;
}

inline int64_t tag_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    if (!radar_meta::dict_has(dict, key))
        return def;
    return radar_meta::to_i64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def)), def);
}

} // namespace

std::vector<float>
UwbRationalResamplerCcf65_32::load_taps_from_profile_or_path(
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

UwbRationalResamplerCcf65_32::sptr
UwbRationalResamplerCcf65_32::make(const std::string& taps_file_or_profile,
                                   bool map_radar_tags,
                                   int num_workers,
                                   const std::string& lengthtagname)
{
    auto taps = load_taps_from_profile_or_path(taps_file_or_profile);
    return gnuradio::make_block_sptr<UwbRationalResamplerCcf65_32>(
        taps, map_radar_tags, num_workers, lengthtagname);
}

UwbRationalResamplerCcf65_32::sptr
UwbRationalResamplerCcf65_32::make_from_taps(const std::vector<float>& taps,
                                             bool map_radar_tags,
                                             int num_workers,
                                             const std::string& lengthtagname)
{
    return gnuradio::make_block_sptr<UwbRationalResamplerCcf65_32>(
        taps, map_radar_tags, num_workers, lengthtagname);
}

UwbRationalResamplerCcf65_32::UwbRationalResamplerCcf65_32(
    const std::vector<float>& taps,
    bool map_radar_tags,
    int num_workers,
    const std::string& lengthtagname)
    : gr::tagged_stream_block("uwb_rational_resampler_ccf_65_32",
                              gr::io_signature::make(1, 1, sizeof(gr_complex)),
                              gr::io_signature::make(1, 1, sizeof(gr_complex)),
                              lengthtagname),
      d_taps_(taps),
      d_core_(std::make_unique<core::RationalResampler65_32Core>(taps)),
      d_map_tags_(map_radar_tags)
{
    if (d_taps_.empty())
        throw std::invalid_argument(
            "UwbRationalResamplerCcf65_32: taps must be non-empty");
    set_tag_propagation_policy(TPP_DONT);
    // The tagged-stream base sets min_noutput_items to the mapped window
    // length; a full CG400 window (preamble 128) maps to ~287k work samples,
    // so the output buffer must be able to hold at least one window.
    set_min_output_buffer(0, 1u << 20);
    d_core_->set_num_workers(num_workers < 1 ? 1 : num_workers);
}

UwbRationalResamplerCcf65_32::~UwbRationalResamplerCcf65_32() = default;

void UwbRationalResamplerCcf65_32::set_num_workers(int n)
{
    d_core_->set_num_workers(n);
}

int UwbRationalResamplerCcf65_32::num_workers() const
{
    return d_core_->num_workers();
}

void UwbRationalResamplerCcf65_32::set_kernel(const std::string& name)
{
    d_core_->set_kernel(name);
}

int UwbRationalResamplerCcf65_32::calculate_output_stream_length(
    const gr_vector_int& ninput_items)
{
    const int nin = *std::max_element(ninput_items.begin(), ninput_items.end());
    if (nin <= 0)
        return 0;
    return static_cast<int>(d_core_->expected_output_length(
        static_cast<uint64_t>(nin)));
}

void UwbRationalResamplerCcf65_32::emit_mapped_tags(int ninput_items,
                                                    size_t produced)
{
    const uint64_t abs = nitems_read(0);
    std::vector<tag_t> tags;
    get_tags_in_range(tags, 0, abs, abs + static_cast<uint64_t>(ninput_items));

    pmt::pmt_t src = pmt::make_dict();
    for (const tag_t& t : tags)
        src = pmt::dict_add(src, t.key, t.value);

    auto map_index = [this](int64_t p, int64_t& mapped) {
        mapped = d_core_->map_input_offset_to_output(p);
        if (mapped < 0)
            mapped = 0;
        return true;
    };

    pmt::pmt_t dst = pmt::make_dict();
    if (!radar_meta::apply_radar_whitelist(
            dst, src, map_index, kInterp, kDecim)) {
        d_stat_tag_err_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Full-window emit geometry, identical to the PDU resampler.
    using radar::radar_i64_add;
    using radar::radar_i64_sub;
    const int64_t n_in = ninput_items;
    const int64_t ws = tag_i64(src, "window_start_sample", 0);
    const int64_t pre_in = tag_i64(src, "pre_guard_samples", 0);
    const int64_t cap_in = tag_i64(src, "capture_samples", n_in);
    const int64_t sc = tag_i64(src, "sample_count", n_in);
    int64_t post_in = tag_i64(src, "post_guard_samples", -1);
    if (post_in < 0) {
        int64_t tmp = 0;
        if (!radar_i64_sub(sc, pre_in + cap_in, tmp) || tmp < 0)
            tmp = 0;
        post_in = tmp;
    }

    int64_t ws_pre = 0, ws_pre_cap = 0;
    if (!radar_i64_add(ws, pre_in, ws_pre) ||
        !radar_i64_add(ws_pre, cap_in, ws_pre_cap)) {
        d_stat_tag_err_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const int64_t ws_out = d_core_->map_input_offset_to_output(ws);
    const int64_t ws_pre_out = d_core_->map_input_offset_to_output(ws_pre);
    const int64_t ws_pre_cap_out =
        d_core_->map_input_offset_to_output(ws_pre_cap);
    int64_t pre_out = ws_pre_out - ws_out;
    int64_t cap_out = ws_pre_cap_out - ws_pre_out;
    if (pre_out < 0)
        pre_out = 0;
    if (cap_out < 0)
        cap_out = 0;
    int64_t post_out = static_cast<int64_t>(produced) - pre_out - cap_out;
    if (post_out < 0)
        post_out = 0;

    dst = pmt::dict_add(dst, pmt::mp("sample_rate"),
                        pmt::from_double(kOutputRateHz));
    dst = pmt::dict_add(dst, pmt::mp("window_start_sample"),
                        pmt::from_long(ws_out));
    dst = pmt::dict_add(dst, pmt::mp("pre_guard_samples"),
                        pmt::from_long(pre_out));
    dst = pmt::dict_add(dst, pmt::mp("capture_samples"),
                        pmt::from_long(cap_out));
    dst = pmt::dict_add(dst, pmt::mp("post_guard_samples"),
                        pmt::from_long(post_out));
    dst = pmt::dict_add(dst, pmt::mp("sample_count"),
                        pmt::from_long(static_cast<long>(produced)));

    const uint64_t out0 = nitems_written(0);
    pmt::pmt_t keys = pmt::dict_keys(dst);
    const size_t nk = pmt::length(keys);
    for (size_t i = 0; i < nk; ++i) {
        pmt::pmt_t k = pmt::nth(i, keys);
        pmt::pmt_t v = pmt::dict_ref(dst, k, pmt::PMT_NIL);
        add_item_tag(0, out0, k, v, alias_pmt());
    }
}

int UwbRationalResamplerCcf65_32::work(
    int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const int nin = ninput_items[0];
    if (nin <= 0)
        return 0;
    auto* out = static_cast<gr_complex*>(output_items[0]);
    const auto* in = static_cast<const gr_complex*>(input_items[0]);
    const size_t max_out = static_cast<size_t>(noutput_items);

    d_core_->reset();
    auto r = d_core_->process(in, static_cast<size_t>(nin), out, max_out);
    size_t produced = r.produced;
    if (produced < max_out)
        produced += d_core_->flush(out + produced, max_out - produced);

    if (d_map_tags_)
        emit_mapped_tags(nin, produced);

    d_stat_windows_.fetch_add(1, std::memory_order_relaxed);
    d_stat_in_.fetch_add(static_cast<uint64_t>(nin), std::memory_order_relaxed);
    d_stat_out_.fetch_add(static_cast<uint64_t>(produced),
                          std::memory_order_relaxed);
    return static_cast<int>(produced);
}

} // namespace uwb
} // namespace gr
