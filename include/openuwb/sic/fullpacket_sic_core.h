#pragma once
#include <openuwb/sic/preamble_sic_core.h>
#include <openuwb/phy/hrp/tx/hrp_mod_core.h>
namespace openuwb { namespace sic {
struct FullPacketScratch {
    mod::HrpModScratch modulator;
    std::vector<std::complex<float>> waveform, model;
    std::vector<std::complex<float>> phase0, phase1;
    std::vector<std::complex<double>> accum, phase_tmp;
    std::vector<int8_t> expected_phr;
    size_t capacity = 0;
    // Unified SIC sets this after preparing its configured public profile.
    // Direct callers keep the historical code10/64/4z2 gate.
    bool allow_configured_profile = false;
    UWB_API void reserve(size_t max_samples);
    // Optional exact prefix phase cache. Allocated explicitly before hot calls.
    bool prefix_cache_enabled = false, prefix_key_valid = false;
    size_t prefix_input_len = 0, prefix_output_len = 0;
    uint32_t prefix_phase_mask = 0;
    std::vector<std::complex<float>> prefix_key, prefix_rows;
    UWB_API bool prepare_prefix_cache();
};
struct FullPacketResult {
    bool committed = false;
    const char* reason = "not_run";
    size_t packet_samples = 0;
    int64_t sub_lo = 0, sub_hi = 0;
    double waveform_scale = 0;
    double validate_us = 0, tx_us = 0, channel_us = 0, subtract_us = 0;
    double phase_filter_us = 0, path_accum_us = 0, cfo_check_us = 0;
    bool prefix_cache_used = false, prefix_key_changed = false;
    size_t prefix_cached_samples = 0, phase_calls = 0, nonzero_phase_calls = 0;
    size_t prefix_cache_hits = 0, prefix_cache_misses = 0;
    uint64_t fir_output_samples = 0, reference_fir_output_samples = 0;
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
}}
