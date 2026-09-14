/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Sc16ScalePolicy — SC16 input amplitude contract shared by the fixed-ratio
 * PDU resamplers (65/32 and 65/48).  Keeping one definition means the Python
 * binding registers a single enum type usable by both blocks.
 *
 *  RawInteger: fc32 = float(int16)          (legacy scheduled-capture chain).
 *  UnitRange : fc32 = float(int16) / 32768  (matches UHD Python FC32 and the
 *              legacy radar chain; required by the cpp-pdu echo backend,
 *              which publishes the raw UHD SC16 payload).
 */

#pragma once

namespace gr {
namespace uwb {

enum class Sc16ScalePolicy {
    RawInteger = 0,
    UnitRange = 1,
};

} // namespace uwb
} // namespace gr
