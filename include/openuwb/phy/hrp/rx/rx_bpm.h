// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace gr::uwb::demod::rxopt {

struct BpmDecision {
    int8_t position;
    int8_t polarity;
    float position_llr;
};

// Candidates are in hop-major order: [half0, half1] for each hop.
// The chip-stream search starts at a finite sentinel and ignores NaN
// candidates. The fused search initializes each half with its first hop.
inline BpmDecision decide_bpm_symbol(const float* metrics, bool chip_stream)
{
    float magnitude[2] = { std::abs(metrics[0]), std::abs(metrics[1]) };
    float best[2] = { metrics[0], metrics[1] };
    for (int half = 0; half < 2; ++half) {
        if (chip_stream && !(magnitude[half] > -1e30f)) {
            magnitude[half] = -1e30f;
            best[half] = 0.0f;
        }
        const float second = std::abs(metrics[half + 2]);
        if (second > magnitude[half]) {
            magnitude[half] = second;
            best[half] = metrics[half + 2];
        }
    }
    const int8_t position = magnitude[0] >= magnitude[1] ? 0 : 1;
    return { position, static_cast<int8_t>(best[position] < 0.0f),
             magnitude[0] - magnitude[1] };
}

inline float bpm_polarity_llr(const float* metrics)
{
    const float positive = std::max(std::max(metrics[0], metrics[1]),
                                    std::max(metrics[2], metrics[3]));
    const float negative = std::max(std::max(-metrics[0], -metrics[1]),
                                    std::max(-metrics[2], -metrics[3]));
    return positive - negative;
}

} // namespace gr::uwb::demod::rxopt
