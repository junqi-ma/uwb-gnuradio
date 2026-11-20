// SPDX-License-Identifier: GPL-3.0-or-later
// Offline-only segmented copy of fullpacket reconstruction at 1adf401.
#pragma once
#include <gnuradio/uwb/uwb_fullpacket_sic_core.h>
#include <array>
namespace gr::uwb::experiment::segment_diag {
struct Result : sic::FullPacketResult {
    std::array<double,3> phase_us{}, path_us{}, cfo_us{};
    double common_us=0;
};
// Intervals in waveform coordinates: [0,73152), [73152,94656), tail.
// Accumulation/CFO output intervals use floor(min(tau_abs)) + these cuts.
// Every filter reads the complete waveform including its boundary halo.
Result run(const std::complex<float>* y,size_t n,
 const std::vector<uint8_t>& phr,const std::vector<uint8_t>& psdu,bool fcs_pass,
 const mod::HrpModConfig& profile,const sic::SicModel& m,
 const std::complex<float>* reference_sync,size_t sync_n,
 int64_t roi_lo,int64_t roi_hi,sic::FullPacketScratch& s,
 std::complex<float>* out,std::complex<float>* ihat);
}
