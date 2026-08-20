/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Minimal JSONL + interleaved SC16 IO for scheduled native dumps.
 * No GNU Radio / PMT.  Field set matches UwbPacketWriter and the
 * demod_results.jsonl sidecar from offline_qm35_auto_lock.py.
 */

#pragma once

#include <cctype>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace gr {
namespace uwb {
namespace core {

struct DumpWindowMeta {
    int64_t packet_id = -1;
    int64_t schedule_index = -1;
    int64_t sample_count = 0;
    int64_t file_offset_samples = 0;
    int64_t window_start_sample = -1;
    int64_t predicted_start_sample = -1;
    int64_t pre_guard_samples = -1;
    int64_t capture_samples = -1;
    int64_t post_guard_samples = -1;
    int64_t start_sample = -1;
    double sample_rate = 737280000.0;
    double iq_scale = 32768.0;
    std::string capture_mode;
    std::string lock_state;
    std::string sample_format = "sc16";
};

struct DemodResultMeta {
    int64_t packet_id = -1;
    int64_t schedule_index = -1;
    std::string status;
    bool fcs_pass = false;
    int64_t detected_start_sample = -1;
    int64_t predicted_start_sample = -1;
    int64_t native_predicted_start = -1;
    int64_t native_window_start = -1;
    double resample_us = 0.0 / 0.0;
    double t_total_us = 0.0 / 0.0;
    bool has_detected_start = false;
};

namespace json_min {

inline bool find_key(const std::string& s, const char* key, size_t& val)
{
    const std::string pat = std::string("\"") + key + "\"";
    size_t pos = 0;
    while (true) {
        pos = s.find(pat, pos);
        if (pos == std::string::npos)
            return false;
        size_t i = pos + pat.size();
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
            ++i;
        if (i < s.size() && s[i] == ':') {
            ++i;
            while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])))
                ++i;
            val = i;
            return true;
        }
        pos += pat.size();
    }
}

inline int64_t as_i64(const std::string& s, const char* key, int64_t def)
{
    size_t i = 0;
    if (!find_key(s, key, i))
        return def;
    if (i < s.size() && s.compare(i, 4, "null") == 0)
        return def;
    try {
        return static_cast<int64_t>(std::stoll(s.substr(i)));
    } catch (...) {
        return def;
    }
}

inline double as_f64(const std::string& s, const char* key, double def)
{
    size_t i = 0;
    if (!find_key(s, key, i))
        return def;
    if (i < s.size() && s.compare(i, 4, "null") == 0)
        return def;
    try {
        return std::stod(s.substr(i));
    } catch (...) {
        return def;
    }
}

inline bool as_bool(const std::string& s, const char* key, bool def)
{
    size_t i = 0;
    if (!find_key(s, key, i))
        return def;
    if (s.compare(i, 4, "true") == 0)
        return true;
    if (s.compare(i, 5, "false") == 0)
        return false;
    if (s.compare(i, 1, "1") == 0)
        return true;
    if (s.compare(i, 1, "0") == 0)
        return false;
    return def;
}

inline std::string as_str(const std::string& s, const char* key,
                          const std::string& def = {})
{
    size_t i = 0;
    if (!find_key(s, key, i))
        return def;
    if (i >= s.size() || s[i] != '"')
        return def;
    ++i;
    std::string out;
    while (i < s.size() && s[i] != '"') {
        if (s[i] == '\\' && i + 1 < s.size()) {
            ++i;
            out.push_back(s[i]);
        } else {
            out.push_back(s[i]);
        }
        ++i;
    }
    return out;
}

inline bool has_key(const std::string& s, const char* key)
{
    size_t i = 0;
    return find_key(s, key, i);
}

} // namespace json_min

inline bool parse_dump_jsonl_line(const std::string& line, DumpWindowMeta& m)
{
    if (line.empty() || line[0] != '{')
        return false;
    m.packet_id = json_min::as_i64(line, "packet_id", -1);
    m.schedule_index = json_min::as_i64(line, "schedule_index", -1);
    m.sample_count = json_min::as_i64(line, "sample_count", 0);
    m.file_offset_samples = json_min::as_i64(line, "file_offset_samples", 0);
    m.window_start_sample = json_min::as_i64(line, "window_start_sample", -1);
    if (m.window_start_sample < 0)
        m.window_start_sample = json_min::as_i64(line, "start_sample", -1);
    m.predicted_start_sample =
        json_min::as_i64(line, "predicted_start_sample", -1);
    m.pre_guard_samples = json_min::as_i64(line, "pre_guard_samples", -1);
    m.capture_samples = json_min::as_i64(line, "capture_samples", -1);
    m.post_guard_samples = json_min::as_i64(line, "post_guard_samples", -1);
    m.start_sample = json_min::as_i64(line, "start_sample", -1);
    m.sample_rate = json_min::as_f64(line, "sample_rate", 737280000.0);
    m.iq_scale = json_min::as_f64(line, "iq_scale", 32768.0);
    m.capture_mode = json_min::as_str(line, "capture_mode");
    m.lock_state = json_min::as_str(line, "lock_state");
    m.sample_format = json_min::as_str(line, "sample_format", "sc16");
    return m.sample_count > 0;
}

inline bool parse_demod_jsonl_line(const std::string& line, DemodResultMeta& m)
{
    if (line.empty() || line[0] != '{')
        return false;
    m.packet_id = json_min::as_i64(line, "packet_id", -1);
    m.schedule_index = json_min::as_i64(line, "schedule_index", -1);
    m.status = json_min::as_str(line, "status");
    m.fcs_pass = json_min::as_bool(line, "fcs_pass", false);
    m.detected_start_sample =
        json_min::as_i64(line, "detected_start_sample", -1);
    m.has_detected_start =
        json_min::has_key(line, "detected_start_sample") &&
        m.detected_start_sample >= 0;
    m.predicted_start_sample =
        json_min::as_i64(line, "predicted_start_sample", -1);
    m.native_predicted_start =
        json_min::as_i64(line, "native_predicted_start", -1);
    m.native_window_start = json_min::as_i64(line, "native_window_start", -1);
    m.resample_us = json_min::as_f64(line, "resample_us", 0.0 / 0.0);
    m.t_total_us = json_min::as_f64(line, "t_total_us", 0.0 / 0.0);
    return m.packet_id >= 0;
}

template <typename T>
inline std::vector<T> load_jsonl(const std::string& path,
                                 bool (*parse)(const std::string&, T&))
{
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("cannot open " + path);
    std::vector<T> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty())
            continue;
        T rec;
        if (parse(line, rec))
            out.push_back(std::move(rec));
    }
    return out;
}

inline std::vector<DumpWindowMeta> load_capture_jsonl(const std::string& path)
{
    return load_jsonl<DumpWindowMeta>(path, parse_dump_jsonl_line);
}

inline std::vector<DemodResultMeta> load_demod_jsonl(const std::string& path)
{
    return load_jsonl<DemodResultMeta>(path, parse_demod_jsonl_line);
}

inline bool read_sc16_window(std::istream& iq,
                             int64_t file_offset_samples,
                             size_t n,
                             std::vector<int16_t>& interleaved)
{
    interleaved.resize(n * 2);
    if (n == 0)
        return true;
    const std::streamoff byte_off =
        static_cast<std::streamoff>(file_offset_samples) *
        static_cast<std::streamoff>(4);
    iq.clear();
    iq.seekg(byte_off, std::ios::beg);
    if (!iq)
        return false;
    iq.read(reinterpret_cast<char*>(interleaved.data()),
            static_cast<std::streamsize>(n * 2 * sizeof(int16_t)));
    return static_cast<size_t>(iq.gcount()) == n * 2 * sizeof(int16_t);
}

inline bool write_sc16_window(std::ostream& iq, const int16_t* interleaved,
                              size_t n)
{
    iq.write(reinterpret_cast<const char*>(interleaved),
             static_cast<std::streamsize>(n * 2 * sizeof(int16_t)));
    return static_cast<bool>(iq);
}

inline bool write_cf32_window(std::ostream& out,
                              const std::complex<float>* x,
                              size_t n)
{
    out.write(reinterpret_cast<const char*>(x),
              static_cast<std::streamsize>(n * sizeof(std::complex<float>)));
    return static_cast<bool>(out);
}

struct JsonField {
    std::string key;
    std::string value; // already encoded
};

inline void write_json_object(std::ostream& os, const std::vector<JsonField>& f)
{
    os << '{';
    for (size_t i = 0; i < f.size(); ++i) {
        if (i)
            os << ',';
        os << '"' << f[i].key << "\":" << f[i].value;
    }
    os << "}\n";
}

inline std::string json_str(const std::string& s)
{
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\')
            o.push_back('\\');
        o.push_back(c);
    }
    o.push_back('"');
    return o;
}

inline std::string json_i64(int64_t v)
{
    return std::to_string(v);
}

inline std::string json_u64(uint64_t v)
{
    return std::to_string(v);
}

inline std::string json_f64(double v)
{
    if (!std::isfinite(v))
        return "null";
    std::ostringstream ss;
    ss.setf(std::ios::fmtflags(0), std::ios::floatfield);
    ss.precision(17);
    ss << v;
    return ss.str();
}

inline std::string json_bool(bool v)
{
    return v ? "true" : "false";
}

} // namespace core
} // namespace uwb
} // namespace gr
