// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace openuwb::core {
// One independent native SC16 window -> work-rate CF32, including FIR tail.
// Worker owns resampler/output. Resets filter history between windows, keeps
// output capacity across calls. Caller chooses scaling and resampler policy.
// No scheduling, timestamp mapping, feedback, or queue ownership here.
template<class Resampler>
size_t resample_sc16_window(Resampler& rs, const std::complex<int16_t>* input,
                            size_t count, float scale,
                            std::vector<std::complex<float>>& output)
{
    const size_t expected = rs.expected_output_length(count);
    if (output.size() < expected)
        output.resize(expected);
    // RS-1: one-shot zero-copy whole-window entry (reset + head + macroblocks
    // + tail/flush inside the core).  It returns the complete window; the
    // loop below only exists for a resampler type that predates the entry.
    const size_t made =
        rs.process_window_sc16(input, count, scale, output.data(), output.size())
            .produced;
    if (made >= expected)
        return made;
    // Defensive fallback for a truncated grant: finish the stream exactly as
    // the previous implementation did.
    size_t produced = made;
    while (produced < expected) {
        const size_t n = rs.flush(output.data() + produced, output.size() - produced);
        if (!n) break;
        produced += n;
    }
    return produced;
}
} // namespace openuwb::core
