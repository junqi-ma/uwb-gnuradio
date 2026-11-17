// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr::uwb::core {
// One independent native SC16 window -> work-rate CF32, including FIR tail.
// Worker owns resampler/output. Resets filter history between windows, keeps
// output capacity across calls. Caller chooses scaling and resampler policy.
// No scheduling, timestamp mapping, feedback, or queue ownership here.
template<class Resampler>
size_t resample_sc16_window(Resampler& rs, const std::complex<int16_t>* input,
                            size_t count, float scale,
                            std::vector<std::complex<float>>& output)
{
    rs.reset();
    const size_t expected = rs.expected_output_length(count);
    if (output.size() < expected)
        output.resize(expected);
    size_t made = rs.process_sc16(input, count, scale, output.data(),
                                  output.size()).produced;
    while (made < expected) {
        const size_t n = rs.flush(output.data() + made, output.size() - made);
        if (!n) break;
        made += n;
    }
    return made;
}
} // namespace gr::uwb::core
