/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Radar PDU metadata keys and copy/map helpers for the 65/48 adapter.
 *
 * Identity/time/profile fields are copied as-is. Native sample *indices*
 * and *lengths* used on the 998.4 work grid are mapped with the caller's
 * group-delay-centered map(p). Existing scheduled-capture keys keep their
 * current semantics; this header only adds the radar whitelist.
 *
 * The pure C++ constants/validation (rates, spans, key names, whitelist
 * arrays) live in uwb_radar_meta_core.h, which does NOT include <pmt/pmt.h>;
 * this header adds the PMT dictionary mapping helpers on top of it and is
 * the adapter-side entry point.
 */

#pragma once

#include <gnuradio/uwb/uwb_radar_meta_core.h>
#include <pmt/pmt.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace gr {
namespace uwb {
namespace radar_meta {

inline bool dict_has(pmt::pmt_t dict, const char* key)
{
    return pmt::is_dict(dict) && pmt::dict_has_key(dict, pmt::mp(key));
}

inline void copy_if_present(pmt::pmt_t& dst, pmt::pmt_t src, const char* key)
{
    if (!dict_has(src, key) || dict_has(dst, key))
        return;
    dst = pmt::dict_add(dst, pmt::mp(key),
                        pmt::dict_ref(src, pmt::mp(key), pmt::PMT_NIL));
}

inline bool try_to_i64(pmt::pmt_t v, int64_t& out)
{
    if (pmt::is_uint64(v))
        return radar::radar_i64_from_u64(pmt::to_uint64(v), out);
    if (pmt::is_integer(v)) {
        out = static_cast<int64_t>(pmt::to_long(v));
        return true;
    }
    if (pmt::is_real(v)) {
        const double d = pmt::to_double(v);
        if (!std::isfinite(d) ||
            d > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
            d < static_cast<double>(std::numeric_limits<int64_t>::min()))
            return false;
        out = static_cast<int64_t>(std::llround(d));
        return true;
    }
    return false;
}

inline int64_t to_i64(pmt::pmt_t v, int64_t def)
{
    int64_t out = def;
    if (!try_to_i64(v, out))
        return def;
    return out;
}

inline double to_f64(pmt::pmt_t v, double def)
{
    if (pmt::is_real(v) || pmt::is_integer(v) || pmt::is_uint64(v))
        return pmt::to_double(v);
    return def;
}

// MapFn: bool(int64_t native_index, int64_t& mapped). Returns false if any
// present Radar index/length field cannot be converted or mapped.
template <typename MapFn>
inline bool apply_radar_whitelist(pmt::pmt_t& dst,
                                  pmt::pmt_t src,
                                  MapFn map_index,
                                  uint32_t interp = kResampleInterp,
                                  uint32_t decim = kUc200NativeDecim)
{
    if (!pmt::is_dict(src))
        return true;

    for (const char* key : kPassthrough)
        copy_if_present(dst, src, key);

    if (dict_has(src, kCalDelayNative) && !dict_has(dst, kCalDelayWork)) {
        const double native = to_f64(
            pmt::dict_ref(src, pmt::mp(kCalDelayNative), pmt::from_double(0)),
            0.0);
        if (!std::isfinite(native))
            return false;
        if (interp == 0 || decim == 0)
            return false;
        dst = pmt::dict_add(dst, pmt::mp(kCalDelayWork),
                            pmt::from_double(native * static_cast<double>(interp) /
                                             static_cast<double>(decim)));
    }

    auto add_native_alias = [&](const char* key, pmt::pmt_t value) {
        std::string nk = std::string(key) + "_native";
        if (!dict_has(dst, nk.c_str()))
            dst = pmt::dict_add(dst, pmt::mp(nk.c_str()), value);
    };

    for (const char* key : kIndexKeys) {
        if (!dict_has(src, key))
            continue;
        const pmt::pmt_t raw =
            pmt::dict_ref(src, pmt::mp(key), pmt::from_long(-1));
        int64_t p = 0;
        if (!try_to_i64(raw, p))
            return false;
        add_native_alias(key, raw);
        if (p < 0)
            return false;
        int64_t mapped = 0;
        if (!map_index(p, mapped))
            return false;
        if (!dict_has(dst, key))
            dst = pmt::dict_add(dst, pmt::mp(key), pmt::from_long(mapped));
    }

    for (const char* key : kLengthKeys) {
        if (!dict_has(src, key))
            continue;
        const pmt::pmt_t raw =
            pmt::dict_ref(src, pmt::mp(key), pmt::from_long(0));
        int64_t n = 0;
        if (!try_to_i64(raw, n))
            return false;
        add_native_alias(key, raw);
        if (n < 0)
            return false;
        int64_t mapped_n = 0;
        int64_t mapped_0 = 0;
        if (!map_index(n, mapped_n) || !map_index(0, mapped_0))
            return false;
        int64_t mapped = 0;
        if (!radar::radar_i64_sub(mapped_n, mapped_0, mapped))
            return false;
        if (mapped < 0)
            mapped = 0;
        if (!dict_has(dst, key))
            dst = pmt::dict_add(dst, pmt::mp(key), pmt::from_long(mapped));
    }
    return true;
}

} // namespace radar_meta
} // namespace uwb
} // namespace gr
