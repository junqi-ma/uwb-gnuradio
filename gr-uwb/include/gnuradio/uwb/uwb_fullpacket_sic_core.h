#pragma once
#include <gnuradio/uwb/uwb_preamble_sic_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
namespace gr { namespace uwb { namespace sic {
struct FullPacketScratch {
    mod::HrpModScratch modulator;
    std::vector<std::complex<float>> waveform, model;
    std::vector<std::complex<float>> phase0, phase1;
    std::vector<std::complex<double>> accum, phase_tmp;
    std::vector<int8_t> expected_phr;
    size_t capacity = 0;
    UWB_API void reserve(size_t max_samples);
};
struct FullPacketResult {
    bool committed = false;
    const char* reason = "not_run";
    size_t packet_samples = 0;
    int64_t sub_lo = 0, sub_hi = 0;
    double waveform_scale = 0;
    double validate_us = 0, tx_us = 0, channel_us = 0, subtract_us = 0;
};
// Independent apply support; never changes the frozen preamble model.
// Input wire PSDU includes FCS; only supported PHR/profile is accepted.
// reference_sync is the EXACT shaped SYNC used by estimate_model.
// roi is consumer allowed region; out/ihat size n, failure leaves out==y.
UWB_API FullPacketResult fullpacket_cancel(
 const std::complex<float>* y, size_t n,
 const std::vector<uint8_t>& phr_bits, const std::vector<uint8_t>& wire_psdu,
 bool fcs_pass, const mod::HrpModConfig& profile,
 const SicModel& frozen, const std::complex<float>* reference_sync, size_t sync_n,
 int64_t roi_lo, int64_t roi_hi, FullPacketScratch& scratch,
 std::complex<float>* out, std::complex<float>* ihat);
}}}
