/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * IEEE 802.15.4a HRP BPRF modulator (work grid 998.4 MS/s).
 *
 * Inverse of demod::core for the frozen P1 profile: no STS, BPM-BPSK
 * 0.85 Mb/s PHR + 6.81 Mb/s payload.  MATLAB lrwpanWaveformGenerator is
 * the sample-level golden (testdata/realtime_demod_golden/window.cfile).
 *
 * No GNU Radio scheduler.  modulate_one is a pure function; callers
 * reserve HrpModScratch once and never grow it on the hot path.
 *
 * P3 hot path: cache pulse-shaped SYNC/SFD/STS, sparse 48-tap chip
 * placement for PHR/payload, VOLK scale + real/imag interleave.
 * 64-SYNC PSDU<=127 B target is mean <= 1 ms/packet (cached prefix
 * and per-packet unique PSDU).  2048-SYNC is not a 1000 pps gate.
 *
 * Declaration-only header; the implementation bodies (the STS DRBG
 * table, the pulse-shaping lookup tables kStsDrbgBprf/kPulse48 and all
 * modulator code) live in lib/phy/hrp/uwb_hrp_mod_core.cc
 * (openuwb_phy_hrp).  Consumers include this header unchanged.
 */

#pragma once

#include <openuwb/phy/hrp/common/hrp_codec.h>
#include <openuwb/phy/hrp/common/profile.h>
#include <openuwb/sensing/radar_meta_core.h>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace openuwb {
namespace mod {

inline constexpr size_t kPulseTaps = 48;
inline constexpr size_t kPhrSymbols = 21;
inline constexpr size_t kPhrChipsPerBurst = 64;
inline constexpr size_t kPhrChipsPerSymbol = 512;
inline constexpr size_t kPayloadChipsPerBurst = 8;
inline constexpr size_t kPayloadChipsPerSymbol = 64;
inline constexpr size_t kPayloadScramblerOffset = 1344; // 21 * 64
inline constexpr float kDefaultPeakAmplitude = 0.8f;
inline constexpr size_t kMaxHrpTxSamples = 4194304; // 1<<22, 2048-SYNC+STS+127B
inline constexpr size_t kStsGapChips = 512;
inline constexpr size_t kStsActiveChipsBprf = 64 * 512; // 1 segment
inline constexpr size_t kStsChipsBprf =
    kStsGapChips + kStsActiveChipsBprf + kStsGapChips; // 33792
inline constexpr size_t kStsSamplesBprf =
    kStsChipsBprf * demod::kQm35SamplesPerChip; // 67584
inline constexpr size_t kStsSpreadingBprf = 8;
inline constexpr size_t kStsDrbgBits = 128;
inline constexpr size_t kStsDrbgBlocksBprf =
    kStsActiveChipsBprf / (kStsDrbgBits * kStsSpreadingBprf); // 32

inline constexpr size_t kGaussianPulseTaps = 49;
inline constexpr size_t kGaussianPulseCenter = 12;
inline constexpr size_t kBlackmanPulseTaps = 129;
inline constexpr size_t kBlackmanPulseCenter = 64;
inline constexpr size_t kMaxExternalPulseTaps = 4096;

// MATLAB allDRBG_STS.mat rows 0..31 (BPRF one segment).  Key/nonce are
// the toolbox defaults used by lrwpanWaveformGenerator.  The table lives
// in lib/phy/hrp/uwb_hrp_mod_core.cc.

enum class PulseShape : int {
    Legacy = 0,
    Gaussian = 1,
    Blackman = 2,
    External = 3, // precomputed taps (see design_tx_pulse.py)
};

struct PulseSpec {
    PulseShape shape = PulseShape::Legacy;
    float gaussian_sigma_ns = 2.5f;  // -67 dB tails at 10..100 ns
    float blackman_bw_mhz = 200.0f;  // target one-sided spectral edge
    size_t external_n_taps = 0;      // valid when shape == External
    size_t external_center = 0;      // peak tap, metadata only
};

struct HrpModConfig {
    size_t code_index = 9;
    size_t sync_repetitions = 64;
    const char* sfd_mode = "ieee";
    bool insert_sts = false;
    float peak_amplitude = kDefaultPeakAmplitude;
    bool ranging = false;
    PulseSpec pulse;
    // Owned taps for PulseShape::External; must outlive modulate_one.
    std::vector<float> external_taps;
};

// One non-zero unit impulse of the BPRF chip sequence.  Positions are relative
// to the packet origin on the work grid; every impulse is exactly +/-1 and the
// expansion places it at `sample_offset`, never at an interpolated position.
struct PulseEvent {
    uint32_t sample_offset = 0;
    int8_t sign = 1;
};

// Read-only view returned by modulate_pulse_list().  The storage is owned by
// HrpModScratch and stays valid until the next modulation call on that scratch.
// packet_samples is the complete packet length (including any trailing zero
// region), NOT the last event offset.
struct PulseList {
    const PulseEvent* events = nullptr;
    size_t count = 0;
    size_t packet_samples = 0;
};

struct HrpPrefixCache {
    bool valid = false;
    size_t code_index = 0;
    size_t sync_repetitions = 0;
    size_t n_sfd = 0;
    size_t n_samples = 0; // includes pulse tail into PHR
    bool insert_sts = false;
    PulseSpec pulse;
    uint64_t pulse_generation = 0;
    char sfd_mode[16] = {};
    std::vector<float> samples;
    // Sparse form of the same prefix.  It shares the key fields above; the
    // waveform and event caches are valid independently so a caller may use
    // either representation without forcing a regeneration of the other.
    std::vector<PulseEvent> events;
    size_t event_count = 0;
    bool events_valid = false;
};

struct HrpModScratch {
    std::vector<int8_t> psdu_bits;
    std::vector<int8_t> rs_cw;
    std::vector<int8_t> conv_in;
    std::vector<int8_t> coded;
    std::vector<int8_t> phr19;
    std::vector<int8_t> spread;
    size_t pn_code_index = 0;
    bool pn_valid = false;
    std::vector<int8_t> g0;
    std::vector<int8_t> g1;
    std::vector<float> iq_re;
    std::vector<float> iq_im; // zeros for VOLK interleave
    HrpPrefixCache prefix;
    // Sparse output of modulate_pulse_list().  Capacity is reserved once by
    // reserve(); the hot call only writes into the existing storage.
    std::vector<PulseEvent> pulse_events;
    size_t pulse_count = 0;
    size_t pulse_packet_samples = 0;

    // Active pulse taps.  Empty for Legacy (uses the static kPulse48 table).
    std::vector<float> pulse_taps;
    PulseSpec pulse_spec;
    bool pulse_valid = false;
    uint64_t pulse_generation = 0;
    const float* external_src = nullptr;

    const float* pulse_data() const;
    size_t pulse_len() const;
    void invalidate_pn();
    void reserve(size_t max_psdu_bytes, size_t max_samples);
};

// append_ieee_fcs is an inline definition by design: the TWR QA
// (qa_uwb_twr_frame.cc) counts a "void append_ieee_fcs" definition in this
// header as the single production FCS owner.
inline void append_ieee_fcs(std::vector<uint8_t>& bytes)
{
    const uint16_t crc =
        codec::crc16_802154(bytes.data(), bytes.size());
    bytes.push_back(static_cast<uint8_t>(crc & 0xff));
    bytes.push_back(static_cast<uint8_t>((crc >> 8) & 0xff));
}

inline std::vector<uint8_t> make_random_psdu(size_t n_data,
                                             uint32_t seed,
                                             bool append_fcs)
{
    std::vector<uint8_t> out(n_data);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> dist(0, 255);
    for (size_t i = 0; i < n_data; ++i)
        out[i] = static_cast<uint8_t>(dist(rng));
    if (append_fcs && n_data > 0)
        append_ieee_fcs(out);
    return out;
}

// Declarations of the public entry points; bodies are in
// lib/phy/hrp/uwb_hrp_mod_core.cc.
std::vector<float> make_pulse_taps(const PulseSpec& spec);
bool ensure_pulse(const HrpModConfig& cfg, HrpModScratch& scratch);
size_t packet_samples_998p4(size_t sync_repetitions,
                            size_t n_sfd,
                            size_t psdu_bytes,
                            bool insert_sts = false,
                            const PulseSpec* pulse = nullptr);

namespace detail {

struct HrpCodedGeometry {
    size_t n_base = 0;
    size_t n_out = 0;
    size_t nsym = 0;
    size_t n_enc_sym = 0;
    size_t sync_chips = 0;
    size_t sfd_chips = 0;
    size_t sts_chips = 0;
    size_t phr_chips = 0;
    size_t pay_chips = 0;
    size_t n_chips = 0;
    size_t phr_chip0 = 0;
};

using codec::fill_bprf_spread;

bool sfd_lookup(const char* mode, const int8_t*& data, size_t& n);
void encode_phr19(size_t psdu_bytes,
                  size_t sync_repetitions,
                  bool ranging,
                  std::vector<int8_t>& phr19);
void bytes_to_lsb_bits(const uint8_t* bytes,
                       size_t n,
                       std::vector<int8_t>& bits);
bool rs_encode_stream(const int8_t* data_bits,
                      size_t data_bits_n,
                      std::vector<int8_t>& coded);
bool encode_hrp_packet(const uint8_t* psdu,
                       size_t psdu_len,
                       const HrpModConfig& cfg,
                       HrpModScratch& scratch,
                       const int8_t* sfd,
                       size_t n_sfd,
                       HrpCodedGeometry& g);

} // namespace detail

bool modulate_one(const uint8_t* psdu, size_t psdu_len,
                  const HrpModConfig& cfg, HrpModScratch& scratch,
                  std::complex<float>* out, size_t out_cap, size_t& out_n);

bool modulate_unit_impulses(const uint8_t* psdu, size_t psdu_len,
                            const HrpModConfig& cfg, HrpModScratch& scratch,
                            size_t out_cap, size_t& out_n);

bool modulate_pulse_list(const uint8_t* psdu, size_t psdu_len,
                         const HrpModConfig& cfg, HrpModScratch& scratch,
                         size_t sample_cap, PulseList& out);

bool modulate_prefix_pulse_list(const HrpModConfig& cfg,
                                HrpModScratch& scratch,
                                size_t sample_cap,
                                PulseList& out);

bool modulate_prefix_waveform(const HrpModConfig& cfg,
                              HrpModScratch& scratch,
                              std::complex<float>* out,
                              size_t out_cap,
                              size_t& out_n);

} // namespace mod
} // namespace openuwb
