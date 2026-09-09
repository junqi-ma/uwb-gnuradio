/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * UwbRadarPacketSource — message-only complete-packet loader.
 *
 * Scheduler: emit handler only.  Waveform is loaded at construction and
 * republished; IQ is not rebuilt per pulse.  PRI is not generated here.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_packet_source.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace gr {
namespace uwb {

namespace {

int64_t dict_i64(pmt::pmt_t dict, const char* key, int64_t def)
{
    return radar_meta::to_i64(
        pmt::dict_ref(dict, pmt::mp(key), pmt::from_long(def)), def);
}

std::string dirname_of(const std::string& path)
{
    const auto pos = path.find_last_of("/\\");
    if (pos == std::string::npos)
        return ".";
    if (pos == 0)
        return path.substr(0, 1);
    return path.substr(0, pos);
}

bool file_exists(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return static_cast<bool>(f);
}

std::string read_text_file(const std::string& path)
{
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error(
            "UwbRadarPacketSource: cannot open descriptor: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool json_find_value(const std::string& text, const char* key, size_t& pos)
{
    const std::string pat = std::string("\"") + key + "\"";
    size_t p = 0;
    while ((p = text.find(pat, p)) != std::string::npos) {
        size_t i = p + pat.size();
        while (i < text.size() && (text[i] == ' ' || text[i] == '\t' ||
                                   text[i] == '\n' || text[i] == '\r'))
            ++i;
        if (i < text.size() && text[i] == ':') {
            ++i;
            while (i < text.size() && (text[i] == ' ' || text[i] == '\t' ||
                                       text[i] == '\n' || text[i] == '\r'))
                ++i;
            pos = i;
            return true;
        }
        p += pat.size();
    }
    return false;
}

bool json_get_number(const std::string& text, const char* key, double& out)
{
    size_t pos = 0;
    if (!json_find_value(text, key, pos))
        return false;
    char* end = nullptr;
    const double v = std::strtod(text.c_str() + pos, &end);
    if (end == text.c_str() + pos || !std::isfinite(v))
        return false;
    out = v;
    return true;
}

bool json_get_int(const std::string& text, const char* key, int64_t& out)
{
    double v = 0.0;
    if (!json_get_number(text, key, v))
        return false;
    if (v > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
        v < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        std::trunc(v) != v)
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

bool json_get_string(const std::string& text, const char* key, std::string& out)
{
    size_t pos = 0;
    if (!json_find_value(text, key, pos))
        return false;
    if (pos >= text.size() || text[pos] != '"')
        return false;
    ++pos;
    std::string s;
    while (pos < text.size() && text[pos] != '"') {
        if (text[pos] == '\\' && pos + 1 < text.size()) {
            s.push_back(text[pos + 1]);
            pos += 2;
            continue;
        }
        s.push_back(text[pos++]);
    }
    if (pos >= text.size() || text[pos] != '"')
        return false;
    out = s;
    return true;
}

void apply_packet_descriptor(const std::string& desc_path,
                             double sample_rate,
                             const std::string& sample_format,
                             size_t sync_repetitions,
                             const std::string& sfd_mode,
                             size_t code_index,
                             size_t num_samples)
{
    const std::string text = read_text_file(desc_path);
    int64_t sync = 0, code = 0, len = -1;
    int64_t bytes_per_complex = 0;
    std::string sfd, descriptor_format, dtype, byte_order, layout;
    if (!json_get_int(text, "sync_repetitions", sync) ||
        !json_get_int(text, "code_index", code) ||
        !json_get_string(text, "sfd_mode", sfd)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: descriptor missing "
            "sync_repetitions/sfd_mode/code_index: " +
            desc_path);
    }
    if (static_cast<size_t>(sync) != sync_repetitions ||
        static_cast<size_t>(code) != code_index || sfd != sfd_mode) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: profile does not match descriptor " +
            desc_path);
    }

    // The descriptor is the byte-level contract for a complete packet, not
    // merely a geometry sidecar.  Loading a four-byte SC16 complex sample as
    // part of an eight-byte FC32 sample (or vice versa) can otherwise pass a
    // coincidental sample-count check and silently corrupt the TX waveform.
    const char* expected_dtype =
        sample_format == "fc32" ? "complex64" : "int16";
    const int64_t expected_bytes_per_complex =
        sample_format == "fc32" ? 8 : 4;
    if (!json_get_string(text, "sample_format", descriptor_format) ||
        !json_get_string(text, "dtype", dtype) ||
        !json_get_string(text, "byte_order", byte_order) ||
        !json_get_string(text, "layout", layout) ||
        !json_get_int(text, "bytes_per_complex", bytes_per_complex)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: descriptor missing "
            "sample_format/dtype/byte_order/layout/bytes_per_complex: " +
            desc_path);
    }
    if (descriptor_format != sample_format || dtype != expected_dtype ||
        byte_order != "little-endian" || layout != "interleaved_iq" ||
        bytes_per_complex != expected_bytes_per_complex) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: descriptor sample format mismatch: " +
            desc_path);
    }

    const bool work = radar_meta::is_work_rate(sample_rate);
    const bool cg400 = radar_meta::is_cg400_native_rate(sample_rate);
    double sidecar_rate = 0.0;
    const char* rate_key = work ? "rate_work_hz"
                                : (cg400 ? "rate_native_cg400_hz"
                                         : "rate_native_hz");
    const char* len_key = work ? "tx_length_998p4"
                               : (cg400 ? "tx_length_491p52"
                                        : "tx_length_737p28");
    if (!json_get_number(text, rate_key, sidecar_rate)) {
        // CG400 length may live in a combined sidecar that still names
        // the UC200 rate as rate_native_hz; accept rate_native_hz if it
        // actually matches 491.52.
        if (cg400 && json_get_number(text, "rate_native_hz", sidecar_rate) &&
            radar_meta::is_cg400_native_rate(sidecar_rate)) {
            rate_key = "rate_native_hz";
        } else {
            throw std::invalid_argument(
                std::string("UwbRadarPacketSource: descriptor missing ") +
                rate_key + ": " + desc_path);
        }
    }
    if (!json_get_int(text, len_key, len)) {
        throw std::invalid_argument(
            std::string("UwbRadarPacketSource: descriptor missing ") +
            len_key + ": " + desc_path);
    }
    if (!radar_meta::rates_close(sidecar_rate, sample_rate)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: descriptor sample_rate mismatch: " +
            desc_path);
    }
    if (len < 0 || static_cast<size_t>(len) != num_samples) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: file length does not match descriptor " +
            desc_path);
    }
}

} // namespace

UwbRadarPacketSource::sptr
UwbRadarPacketSource::make(const std::string& path,
                           double sample_rate,
                           const std::string& sample_format,
                           size_t sync_repetitions,
                           const std::string& sfd_mode,
                           size_t code_index,
                           double pri_s,
                           bool auto_emit,
                           const std::string& descriptor_path)
{
    return gnuradio::make_block_sptr<UwbRadarPacketSource>(
        path,
        sample_rate,
        sample_format,
        sync_repetitions,
        sfd_mode,
        code_index,
        pri_s,
        auto_emit,
        descriptor_path);
}

UwbRadarPacketSource::UwbRadarPacketSource(const std::string& path,
                                           double sample_rate,
                                           const std::string& sample_format,
                                           size_t sync_repetitions,
                                           const std::string& sfd_mode,
                                           size_t code_index,
                                           double pri_s,
                                           bool auto_emit,
                                           const std::string& descriptor_path)
    : gr::block("uwb_radar_packet_source",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
      d_path_(path),
      d_sample_rate_(sample_rate),
      d_sample_format_(sample_format),
      d_sync_repetitions_(sync_repetitions),
      d_sfd_mode_(sfd_mode),
      d_code_index_(code_index),
      d_pri_s_(pri_s),
      d_auto_emit_(auto_emit),
      d_descriptor_path_(descriptor_path)
{
    if (path.empty()) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: path must be non-empty");
    }
    if (!radar_meta::is_work_rate(sample_rate) &&
        !radar_meta::is_native_rate(sample_rate)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: sample_rate must be 998.4e6, 737.28e6, "
            "or 491.52e6");
    }
    if (sample_format != "fc32" && sample_format != "sc16") {
        throw std::invalid_argument(
            "UwbRadarPacketSource: sample_format must be fc32 or sc16");
    }
    if (!radar_meta::code_index_supported(code_index)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: code_index must be 9..12");
    }
    if (!radar_meta::sync_reps_supported(sync_repetitions)) {
        throw std::invalid_argument(
            std::string("UwbRadarPacketSource: sync_repetitions must be ") +
            radar_meta::sync_reps_supported_list());
    }
    if (!(pri_s > 0.0)) {
        throw std::invalid_argument("UwbRadarPacketSource: pri_s must be > 0");
    }

    const auto sfd = demod::GetSfdSequence(sfd_mode.c_str());
    if (sfd.empty()) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: unknown sfd_mode (GetSfdSequence)");
    }

    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        throw std::runtime_error(
            "UwbRadarPacketSource: cannot open packet file: " + path);
    }
    const auto bytes = static_cast<size_t>(f.tellg());
    if (bytes == 0) {
        throw std::runtime_error("UwbRadarPacketSource: empty file: " + path);
    }
    f.seekg(0);

    if (sample_format == "fc32") {
        if (bytes % sizeof(gr_complex) != 0) {
            throw std::runtime_error(
                "UwbRadarPacketSource: fc32 file size is not a multiple of "
                "8 bytes: " +
                path);
        }
        d_num_samples_ = bytes / sizeof(gr_complex);
        d_fc32_.resize(d_num_samples_);
        f.read(reinterpret_cast<char*>(d_fc32_.data()),
               static_cast<std::streamsize>(bytes));
        if (!f) {
            throw std::runtime_error(
                "UwbRadarPacketSource: failed reading fc32 file: " + path);
        }
        d_iq_pmt_ = pmt::init_c32vector(d_fc32_.size(), d_fc32_.data());
    } else {
        if (bytes % (2 * sizeof(int16_t)) != 0) {
            throw std::runtime_error(
                "UwbRadarPacketSource: sc16 file size is not a multiple of "
                "4 bytes: " +
                path);
        }
        d_sc16_.resize(bytes / sizeof(int16_t));
        d_num_samples_ = d_sc16_.size() / 2;
        f.read(reinterpret_cast<char*>(d_sc16_.data()),
               static_cast<std::streamsize>(bytes));
        if (!f) {
            throw std::runtime_error(
                "UwbRadarPacketSource: failed reading sc16 file: " + path);
        }
        d_iq_pmt_ = pmt::init_s16vector(d_sc16_.size(), d_sc16_.data());
    }

    if (d_num_samples_ == 0) {
        throw std::runtime_error(
            "UwbRadarPacketSource: file has no samples: " + path);
    }

    if (radar_meta::is_work_rate(sample_rate)) {
        const size_t min_len =
            sync_repetitions * demod::kQm35SamplesPerSymbol +
            sfd.size() * demod::kQm35SamplesPerSymbol;
        if (d_num_samples_ < min_len) {
            throw std::invalid_argument(
                "UwbRadarPacketSource: packet shorter than "
                "sync_repetitions*1016 + sfd*1016");
        }
        d_sync_samples_ = static_cast<int64_t>(sync_repetitions *
                                               demod::kQm35SamplesPerSymbol);
        d_sfd_samples_ =
            static_cast<int64_t>(sfd.size() * demod::kQm35SamplesPerSymbol);
    }

    if (d_descriptor_path_.empty()) {
        const std::string dir = dirname_of(path);
        // A format-specific sidecar takes precedence, allowing FC32 and
        // SC16 representations of the same packet to coexist safely.
        const std::string format_sidecar =
            dir + "/metadata." + sample_format + ".json";
        const std::string sidecar = dir + "/metadata.json";
        if (file_exists(format_sidecar))
            d_descriptor_path_ = format_sidecar;
        else if (file_exists(sidecar))
            d_descriptor_path_ = sidecar;
    }
    if (radar_meta::is_native_rate(sample_rate) && d_descriptor_path_.empty()) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: native 737.28e6 / 491.52e6 packets "
            "require a metadata sidecar or descriptor_path (empty file "
            "is not a complete packet)");
    }
    if (!d_descriptor_path_.empty()) {
        apply_packet_descriptor(d_descriptor_path_,
                                sample_rate,
                                sample_format,
                                sync_repetitions,
                                sfd_mode,
                                code_index,
                                d_num_samples_);
    }

    const double duration = static_cast<double>(d_num_samples_) / sample_rate;
    if (!(duration < pri_s)) {
        throw std::invalid_argument(
            "UwbRadarPacketSource: packet duration must be < pri_s");
    }

    message_port_register_in(pmt::mp("emit"));
    message_port_register_out(pmt::mp("tx"));
    message_port_register_out(pmt::mp("status"));
    set_msg_handler(pmt::mp("emit"),
                    [this](pmt::pmt_t msg) { handle_emit(msg); });
}

UwbRadarPacketSource::~UwbRadarPacketSource() = default;

bool
UwbRadarPacketSource::start()
{
    if (d_auto_emit_)
        publish_tx(d_next_pulse_id_++);
    return true;
}

void
UwbRadarPacketSource::publish_status(const std::string& event, pmt::pmt_t extra)
{
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("event"), pmt::mp(event));
    meta = pmt::dict_add(meta, pmt::mp("pdus_emitted"),
                         pmt::from_uint64(pdus_emitted()));
    meta = pmt::dict_add(meta, pmt::mp("pdus_dropped"),
                         pmt::from_uint64(pdus_dropped()));
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
UwbRadarPacketSource::publish_tx(uint64_t pulse_id)
{
    if (pmt::is_null(d_iq_pmt_)) {
        d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
        publish_status("invalid_waveform");
        return;
    }

    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kPulseId),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kScheduleIndex),
                         pmt::from_uint64(pulse_id));
    meta = pmt::dict_add(meta, pmt::mp("sample_rate"),
                         pmt::from_double(d_sample_rate_));
    meta = pmt::dict_add(meta, pmt::mp("sample_format"),
                         pmt::mp(d_sample_format_));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncRepetitions),
                         pmt::from_long(static_cast<long>(d_sync_repetitions_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdMode),
                         pmt::mp(d_sfd_mode_));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kCodeIndex),
                         pmt::from_long(static_cast<long>(d_code_index_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kTxPacketSamples),
                         pmt::from_long(static_cast<long>(d_num_samples_)));
    meta = pmt::dict_add(meta, pmt::mp("sample_count"),
                         pmt::from_long(static_cast<long>(d_num_samples_)));
    meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSource),
                         pmt::mp("packet_source"));
    if (d_sync_samples_ >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSyncSamples),
                             pmt::from_long(d_sync_samples_));
    }
    if (d_sfd_samples_ >= 0) {
        meta = pmt::dict_add(meta, pmt::mp(radar_meta::kSfdSamples),
                             pmt::from_long(d_sfd_samples_));
    }

    message_port_pub(pmt::mp("tx"), pmt::cons(meta, d_iq_pmt_));
    d_pdus_emitted_.fetch_add(1, std::memory_order_relaxed);
}

void
UwbRadarPacketSource::handle_emit(pmt::pmt_t msg)
{
    d_emits_received_.fetch_add(1, std::memory_order_relaxed);

    uint64_t pulse_id = d_next_pulse_id_;
    pmt::pmt_t dict = msg;
    if (pmt::is_pair(msg) && pmt::is_dict(pmt::car(msg)))
        dict = pmt::car(msg);
    if (pmt::is_dict(dict) && radar_meta::dict_has(dict, radar_meta::kPulseId)) {
        const int64_t v = dict_i64(dict, radar_meta::kPulseId, 0);
        if (v < 0) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_emit");
            return;
        }
        pulse_id = static_cast<uint64_t>(v);
        d_next_pulse_id_ = pulse_id + 1;
    } else if (pmt::is_uint64(msg) || pmt::is_integer(msg)) {
        const int64_t v = radar_meta::to_i64(msg, 0);
        if (v < 0) {
            d_pdus_dropped_.fetch_add(1, std::memory_order_relaxed);
            publish_status("invalid_emit");
            return;
        }
        pulse_id = static_cast<uint64_t>(v);
        d_next_pulse_id_ = pulse_id + 1;
    } else {
        d_next_pulse_id_ = pulse_id + 1;
    }

    publish_tx(pulse_id);
}

} // namespace uwb
} // namespace gr
