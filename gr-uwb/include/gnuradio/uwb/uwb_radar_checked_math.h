/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace gr {
namespace uwb {
namespace radar {

// Checked integer coordinate operations shared by the header-only radar
// cores. A failed conversion or operation is an InvalidInput condition at
// the caller boundary; never rely on signed wraparound for sample indices.
inline bool radar_i64_from_size(size_t v, int64_t& out)
{
    if (v > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

inline bool radar_i64_from_u64(uint64_t v, int64_t& out)
{
    if (v > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        return false;
    out = static_cast<int64_t>(v);
    return true;
}

inline bool radar_i64_mul(int64_t a, int64_t b, int64_t& out)
{
    if (a == 0 || b == 0) {
        out = 0;
        return true;
    }
    if (a > 0 && b > 0) {
        if (a > std::numeric_limits<int64_t>::max() / b)
            return false;
    } else if (a > 0 && b < 0) {
        if (b < std::numeric_limits<int64_t>::min() / a)
            return false;
    } else if (a < 0 && b > 0) {
        if (a < std::numeric_limits<int64_t>::min() / b)
            return false;
    } else {
        if (a < 0 && b < 0 &&
            a < std::numeric_limits<int64_t>::max() / b)
            return false;
    }
    out = a * b;
    return true;
}

inline bool radar_i64_add(int64_t a, int64_t b, int64_t& out)
{
    if (b > 0 && a > std::numeric_limits<int64_t>::max() - b)
        return false;
    if (b < 0 && a < std::numeric_limits<int64_t>::min() - b)
        return false;
    out = a + b;
    return true;
}

inline bool radar_i64_sub(int64_t a, int64_t b, int64_t& out)
{
    if (b > 0 && a < std::numeric_limits<int64_t>::min() + b)
        return false;
    if (b < 0 && a > std::numeric_limits<int64_t>::max() + b)
        return false;
    out = a - b;
    return true;
}

} // namespace radar
} // namespace uwb
} // namespace gr
