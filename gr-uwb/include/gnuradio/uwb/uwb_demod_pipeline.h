// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_scratch.h>

namespace gr::uwb::demod::core {
// Full-packet entry point. rx uses work-rate samples. predicted_start and
// window_start are absolute work coordinates; stages use window-relative
// coordinates internally. Published origin fields are rebased once; soft-chip
// and local diagnostic positions retain their stage coordinate conventions.
DemodResult demodulate_one(const std::complex<float>* rx, size_t n,
    const Qm35825Profile& profile, uint64_t packet_id, int64_t predicted_start,
    int64_t window_start, const std::vector<std::complex<float>>& template_wf,
    DemodScratch& scratch);

} // namespace gr::uwb::demod::core
