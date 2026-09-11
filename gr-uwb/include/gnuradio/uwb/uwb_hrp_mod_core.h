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
 */

#pragma once

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_phy_profile.h>
#include <gnuradio/uwb/uwb_radar_pdu_meta.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <volk/volk.h>

namespace gr {
namespace uwb {
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

// MATLAB allDRBG_STS.mat rows 0..31 (BPRF one segment).  Key/nonce are
// the toolbox defaults used by lrwpanWaveformGenerator.
inline constexpr uint8_t kStsDrbgBprf[32][16] = {
    { 246, 130, 32, 6, 84, 10, 193, 14, 222, 60, 34, 239, 74, 8, 201, 56 },
    { 108, 57, 171, 188, 232, 141, 62, 232, 193, 230, 253, 121, 118, 172, 104,
      175 },
    { 122, 154, 133, 112, 205, 186, 107, 200, 197, 55, 97, 150, 231, 141, 31,
      50 },
    { 150, 62, 123, 13, 235, 203, 184, 107, 166, 19, 67, 58, 189, 1, 244, 61 },
    { 212, 0, 90, 50, 99, 240, 152, 200, 159, 38, 90, 222, 75, 148, 29, 228 },
    { 90, 167, 133, 60, 84, 36, 46, 224, 170, 222, 204, 96, 239, 63, 92, 212 },
    { 169, 115, 125, 184, 23, 32, 237, 112, 201, 127, 187, 122, 112, 162, 50,
      20 },
    { 35, 29, 125, 227, 71, 172, 249, 248, 165, 154, 238, 90, 135, 130, 57, 31 },
    { 228, 68, 154, 177, 255, 26, 149, 216, 179, 44, 125, 167, 16, 253, 220,
      206 },
    { 74, 184, 231, 45, 59, 159, 136, 60, 9, 157, 127, 64, 42, 80, 233, 110 },
    { 163, 5, 144, 202, 49, 4, 183, 174, 69, 179, 59, 239, 126, 105, 110, 46 },
    { 91, 221, 100, 141, 237, 1, 229, 48, 108, 52, 106, 209, 46, 144, 238, 58 },
    { 234, 48, 171, 85, 225, 129, 28, 68, 177, 239, 162, 38, 111, 227, 179,
      130 },
    { 158, 109, 17, 7, 68, 98, 35, 62, 47, 70, 191, 129, 139, 58, 0, 197 },
    { 29, 51, 180, 85, 90, 119, 245, 230, 56, 9, 2, 168, 175, 172, 42, 137 },
    { 99, 99, 42, 255, 14, 12, 14, 33, 240, 59, 0, 114, 250, 39, 186, 102 },
    { 19, 84, 60, 146, 243, 103, 46, 61, 66, 102, 66, 48, 91, 129, 169, 193 },
    { 225, 124, 31, 102, 52, 216, 21, 106, 186, 223, 127, 200, 220, 240, 192,
      97 },
    { 24, 208, 190, 73, 150, 86, 159, 91, 184, 47, 74, 88, 121, 102, 167, 56 },
    { 152, 125, 127, 199, 120, 29, 81, 125, 234, 207, 232, 73, 144, 203, 195,
      114 },
    { 66, 147, 77, 120, 113, 0, 35, 79, 56, 138, 100, 93, 113, 255, 249, 55 },
    { 54, 251, 179, 165, 151, 76, 3, 25, 181, 117, 83, 167, 78, 0, 72, 162 },
    { 244, 31, 125, 33, 235, 62, 72, 136, 121, 126, 66, 39, 200, 116, 94, 171 },
    { 28, 68, 76, 98, 75, 181, 160, 233, 112, 182, 65, 181, 176, 81, 190, 157 },
    { 173, 176, 206, 16, 145, 222, 29, 143, 132, 80, 227, 18, 93, 199, 154,
      209 },
    { 103, 167, 25, 72, 231, 169, 236, 248, 242, 203, 234, 197, 0, 248, 19,
      161 },
    { 164, 187, 97, 96, 215, 219, 31, 138, 34, 124, 63, 133, 217, 139, 190, 58 },
    { 213, 73, 197, 21, 107, 247, 102, 242, 25, 19, 103, 174, 247, 255, 84, 22 },
    { 5, 121, 59, 67, 35, 228, 84, 142, 210, 216, 113, 164, 11, 61, 0, 229 },
    { 182, 140, 16, 206, 6, 151, 2, 4, 195, 222, 7, 94, 213, 241, 80, 29 },
    { 112, 147, 118, 127, 190, 103, 239, 77, 138, 125, 140, 187, 128, 118, 162,
      205 },
    { 143, 84, 231, 164, 23, 232, 219, 76, 162, 19, 163, 239, 164, 45, 253, 17 },
};

// Causal FIR fitted to testdata/reference_preamble.bin vs sparse sampled_code.
inline constexpr float kPulse48[kPulseTaps] = {
    1.661402360e-02f,  6.645609438e-02f,  9.160924703e-02f,  3.415651619e-02f,
    -2.820419334e-02f, -1.777498424e-02f,  1.208979171e-02f,  8.035786450e-03f,
    -5.377765745e-03f, -3.591632470e-03f,  2.400185680e-03f,  1.603686251e-03f,
    -1.071562059e-03f, -7.159923553e-04f,  4.784113553e-04f,  3.196640464e-04f,
    -2.135928808e-04f, -1.427182142e-04f,  9.536153084e-05f,  6.371849304e-05f,
    -4.257548426e-05f, -2.844783739e-05f,  1.900822826e-05f,  1.270086977e-05f,
    -8.486479601e-06f, -5.670523933e-06f,  3.788995400e-06f,  2.531475729e-06f,
    -1.691563853e-06f, -1.130281248e-06f,  7.552291663e-07f,  5.046135243e-07f,
    -3.372759068e-07f, -2.249929452e-07f,  1.505134719e-07f,  1.006561163e-07f,
    -6.698117261e-08f, -4.486743066e-08f,  3.000025472e-08f,  1.999612564e-08f,
    -1.334855870e-08f, -9.090527442e-09f,  5.908431078e-09f,  4.061501357e-09f,
    -2.305225122e-09f, -1.765908753e-09f,  1.236934999e-09f,  7.909589650e-10f,
};

// TX pulse shaping on the 998.4 MS/s work grid (2 samples/chip).
// Legacy keeps the 48-tap core fitted to the 737.28 MS/s reference; the
// 491.52 MS/s USRP cannot reproduce its spectrum near +/-245.76 MHz and
// rings, so Gaussian/Blackman band-limited cores are offered instead.
// See docs/固定491p52采样率_发射脉冲低拖尾方案.md.
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

inline const char* pulse_shape_name(PulseShape shape)
{
    switch (shape) {
    case PulseShape::Gaussian:
        return "gaussian";
    case PulseShape::Blackman:
        return "blackman";
    case PulseShape::External:
        return "external";
    default:
        return "legacy";
    }
}

inline bool parse_pulse_shape(const std::string& name, PulseShape& out)
{
    if (name.empty() || name == "legacy" || name == "butter") {
        out = PulseShape::Legacy;
        return true;
    }
    if (name == "gaussian" || name == "gauss") {
        out = PulseShape::Gaussian;
        return true;
    }
    if (name == "blackman") {
        out = PulseShape::Blackman;
        return true;
    }
    if (name == "external" || name == "file") {
        out = PulseShape::External;
        return true;
    }
    return false;
}

inline constexpr size_t kGaussianPulseTaps = 49;
inline constexpr size_t kGaussianPulseCenter = 12;
inline constexpr size_t kBlackmanPulseTaps = 129;
inline constexpr size_t kBlackmanPulseCenter = 64;
inline constexpr size_t kMaxExternalPulseTaps = 4096;

inline size_t pulse_n_taps(const PulseSpec& spec)
{
    switch (spec.shape) {
    case PulseShape::Gaussian:
        return kGaussianPulseTaps;
    case PulseShape::Blackman:
        return kBlackmanPulseTaps;
    case PulseShape::External:
        return spec.external_n_taps;
    default:
        return kPulseTaps;
    }
}

inline size_t pulse_center_tap(const PulseSpec& spec)
{
    switch (spec.shape) {
    case PulseShape::Gaussian:
        return kGaussianPulseCenter;
    case PulseShape::Blackman:
        return kBlackmanPulseCenter;
    case PulseShape::External:
        return spec.external_center;
    default:
        return 2;
    }
}

inline bool pulse_spec_valid(const PulseSpec& spec)
{
    switch (spec.shape) {
    case PulseShape::Gaussian:
        return std::isfinite(spec.gaussian_sigma_ns) &&
               spec.gaussian_sigma_ns > 0.f;
    case PulseShape::Blackman:
        return std::isfinite(spec.blackman_bw_mhz) &&
               spec.blackman_bw_mhz > 0.f &&
               spec.blackman_bw_mhz < 499.2f; // work-grid Nyquist, MHz
    case PulseShape::External:
        return spec.external_n_taps >= 2 &&
               spec.external_n_taps <= kMaxExternalPulseTaps;
    default:
        return true;
    }
}

// Causal real taps at kWorkRateHz.  Empty vector on invalid spec.
inline std::vector<float> make_pulse_taps(const PulseSpec& spec)
{
    if (!pulse_spec_valid(spec))
        return {};
    if (spec.shape == PulseShape::Legacy)
        return std::vector<float>(kPulse48, kPulse48 + kPulseTaps);
    if (spec.shape == PulseShape::External)
        return {};
    const double rate = radar_meta::kWorkRateHz;
    const double dt = 1.0 / rate;
    const size_t n = pulse_n_taps(spec);
    const size_t center = pulse_center_tap(spec);
    std::vector<float> taps(n, 0.f);
    if (spec.shape == PulseShape::Gaussian) {
        const double sigma = spec.gaussian_sigma_ns * 1e-9;
        for (size_t i = 0; i < n; ++i) {
            const double x = (static_cast<double>(i) -
                              static_cast<double>(center)) * dt / sigma;
            taps[i] = static_cast<float>(std::exp(-0.5 * x * x));
        }
        return taps;
    }
    const double B = spec.blackman_bw_mhz * 1e6;
    auto sinc = [](double v) {
        if (v == 0.0)
            return 1.0;
        const double p = M_PI * v;
        return std::sin(p) / p;
    };
    double peak = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = (static_cast<double>(i) -
                          static_cast<double>(center)) * dt;
        const double x = 2.0 * B * t;
        const double h = 2.0 * B *
                         (0.42 * sinc(x) +
                          0.25 * (sinc(x - 0.5) + sinc(x + 0.5)) +
                          0.04 * (sinc(x - 1.0) + sinc(x + 1.0)));
        taps[i] = static_cast<float>(h);
        peak = std::max(peak, std::fabs(h));
    }
    if (peak > 0.0) {
        for (auto& v : taps)
            v = static_cast<float>(v / peak);
    }
    return taps;
}

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
};

struct HrpModScratch {
    std::vector<int8_t> psdu_bits;
    std::vector<int8_t> rs_cw;
    std::vector<int8_t> conv_in;
    std::vector<int8_t> coded;
    std::vector<int8_t> phr19;
    std::vector<int8_t> spread;
    std::vector<int8_t> g0;
    std::vector<int8_t> g1;
    std::vector<float> iq_re;
    std::vector<float> iq_im; // zeros for VOLK interleave
    HrpPrefixCache prefix;

    // Active pulse taps.  Empty for Legacy (uses static kPulse48).
    std::vector<float> pulse_taps;
    PulseSpec pulse_spec;
    bool pulse_valid = false;
    uint64_t pulse_generation = 0;
    const float* external_src = nullptr;

    const float* pulse_data() const
    {
        return pulse_taps.empty() ? kPulse48 : pulse_taps.data();
    }
    size_t pulse_len() const
    {
        return pulse_taps.empty() ? kPulseTaps : pulse_taps.size();
    }

    void reserve(size_t max_psdu_bytes, size_t max_samples)
    {
        const size_t max_bits = max_psdu_bytes * 8;
        const size_t max_blocks = (max_bits + 329) / 330;
        const size_t max_nsym = max_bits + 48 * max_blocks;
        const size_t max_conv = 19 + max_nsym + 2;
        psdu_bits.reserve(max_bits);
        rs_cw.reserve(max_nsym);
        conv_in.reserve(max_conv);
        coded.reserve(2 * max_conv);
        phr19.reserve(19);
        g0.reserve(max_conv);
        g1.reserve(max_conv);
        spread.reserve(std::max(kPhrChipsPerBurst * kPhrSymbols,
                                kPayloadChipsPerBurst * max_nsym));
        iq_re.reserve(max_samples);
        iq_im.assign(max_samples, 0.f);
        prefix.samples.reserve(max_samples);
    }
};

inline bool ensure_pulse(const HrpModConfig& cfg, HrpModScratch& scratch)
{
    const PulseSpec& spec = cfg.pulse;
    if (!pulse_spec_valid(spec)) {
        scratch.pulse_valid = false;
        return false;
    }
    if (spec.shape == PulseShape::External) {
        const float* src = cfg.external_taps.data();
        if (cfg.external_taps.size() != spec.external_n_taps || !src)
            return false;
        if (scratch.pulse_valid && scratch.pulse_spec.shape == spec.shape &&
            scratch.external_src == src)
            return true;
        scratch.pulse_taps = cfg.external_taps;
        scratch.pulse_spec = spec;
        scratch.external_src = src;
        scratch.pulse_valid = true;
        ++scratch.pulse_generation;
        return true;
    }
    if (scratch.pulse_valid && scratch.pulse_spec.shape == spec.shape &&
        scratch.pulse_spec.gaussian_sigma_ns == spec.gaussian_sigma_ns &&
        scratch.pulse_spec.blackman_bw_mhz == spec.blackman_bw_mhz)
        return true;
    if (spec.shape == PulseShape::Legacy) {
        scratch.pulse_taps.clear();
        scratch.external_src = nullptr;
        scratch.pulse_spec = spec;
        scratch.pulse_valid = true;
        ++scratch.pulse_generation;
        return true;
    }
    std::vector<float> taps = make_pulse_taps(spec);
    if (taps.empty()) {
        scratch.pulse_valid = false;
        return false;
    }
    scratch.pulse_taps.swap(taps);
    scratch.external_src = nullptr;
    scratch.pulse_spec = spec;
    scratch.pulse_valid = true;
    ++scratch.pulse_generation;
    return true;
}

inline size_t pulse_tail_extra(const PulseSpec& spec)
{
    if (spec.shape == PulseShape::Legacy)
        return 0;
    return pulse_n_taps(spec) - demod::kQm35SamplesPerChip;
}

inline size_t payload_bpm_symbols(size_t psdu_bytes)
{
    if (psdu_bytes == 0)
        return 0;
    const size_t psdu_bits = psdu_bytes * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    return psdu_bits + 48 * num_blocks;
}

inline size_t packet_samples_998p4(size_t sync_repetitions,
                                   size_t n_sfd,
                                   size_t psdu_bytes,
                                   bool insert_sts = false,
                                   const PulseSpec* pulse = nullptr)
{
    const size_t nsym = payload_bpm_symbols(psdu_bytes);
    const size_t sts = insert_sts ? kStsSamplesBprf : 0;
    size_t n = sync_repetitions * demod::kQm35SamplesPerSymbol +
               n_sfd * demod::kQm35SamplesPerSymbol + sts +
               kPhrSymbols * kPhrChipsPerSymbol * demod::kQm35SamplesPerChip +
               nsym * kPayloadChipsPerSymbol * demod::kQm35SamplesPerChip;
    if (pulse)
        n += pulse_tail_extra(*pulse);
    return n;
}

inline bool sfd_mode_is_4z(const char* sfd_mode)
{
    if (!sfd_mode)
        return false;
    const std::string s(sfd_mode);
    return s == "4z1" || s == "4z2" || s == "4z3" || s == "4z4";
}

inline void append_ieee_fcs(std::vector<uint8_t>& bytes)
{
    const uint16_t crc =
        demod::core::detail::crc16_802154(bytes.data(), bytes.size());
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

namespace detail {

inline const int* rs_generator()
{
    // g(x) = Π_{i=1..8} (x + α^i), α=2, low-first, deg 8.
    static int g[9];
    static bool ready = false;
    if (!ready) {
        std::fill(g, g + 9, 0);
        g[0] = 1;
        int deg = 0;
        for (int i = 1; i <= 8; ++i) {
            const int root = demod::core::detail::rs_gf_pow(2, i);
            int ng[10] = {};
            for (int j = 0; j <= deg; ++j) {
                ng[j] ^= demod::core::detail::rs_gf_mul(g[j], root);
                ng[j + 1] ^= g[j];
            }
            ++deg;
            for (int j = 0; j <= deg; ++j)
                g[j] = ng[j];
        }
        ready = true;
    }
    return g;
}

inline void bits_to_nsyms(const int8_t* bits, int nsym, int* syms)
{
    for (int i = 0; i < nsym; ++i) {
        int s = 0;
        for (int b = 0; b < 6; ++b)
            s = (s << 1) | (bits[i * 6 + b] & 1);
        syms[i] = s;
    }
}

inline void rs_encode_330(const int8_t* data330, int8_t* coded378)
{
    int dsym[55];
    bits_to_nsyms(data330, 55, dsym);
    const int* g = rs_generator();
    int reg[8] = {};
    for (int i = 54; i >= 0; --i) {
        const int fb = dsym[i] ^ reg[7];
        int nreg[8];
        nreg[0] = demod::core::detail::rs_gf_mul(fb, g[0]);
        for (int k = 1; k < 8; ++k)
            nreg[k] = reg[k - 1] ^ demod::core::detail::rs_gf_mul(fb, g[k]);
        for (int k = 0; k < 8; ++k)
            reg[k] = nreg[k];
    }
    int cw[63];
    for (int i = 0; i < 55; ++i)
        cw[i] = dsym[i];
    for (int i = 0; i < 8; ++i)
        cw[55 + i] = reg[i];
    demod::core::detail::rs_syms_to_bits(cw, 63, coded378);
}

inline bool rs_encode_stream(const int8_t* data_bits,
                             size_t data_bits_n,
                             std::vector<int8_t>& coded)
{
    coded.clear();
    if (data_bits_n == 0)
        return true;
    const size_t num_blocks = (data_bits_n + 329) / 330;
    coded.reserve(data_bits_n + 48 * num_blocks);
    size_t off = 0;
    for (size_t b = 0; b < num_blocks; ++b) {
        const size_t this_data = std::min<size_t>(330, data_bits_n - off);
        int8_t block330[330];
        std::memset(block330, 0, sizeof(block330));
        const size_t data_start = 330 - this_data;
        for (size_t i = 0; i < this_data; ++i)
            block330[data_start + i] = static_cast<int8_t>(data_bits[off + i] & 1);
        int8_t block378[378];
        rs_encode_330(block330, block378);
        for (size_t i = 0; i < this_data; ++i)
            coded.push_back(block378[data_start + i]);
        for (size_t i = 0; i < 48; ++i)
            coded.push_back(block378[330 + i]);
        off += this_data;
    }
    return coded.size() == data_bits_n + 48 * num_blocks;
}

inline void convenc_cl3(const int8_t* bits, size_t n, std::vector<int8_t>& coded)
{
    int s1 = 0, s0 = 0;
    coded.resize(2 * n);
    for (size_t t = 0; t < n; ++t) {
        const int u = bits[t] & 1;
        coded[2 * t] = static_cast<int8_t>(s1);
        coded[2 * t + 1] = static_cast<int8_t>(u ^ s0);
        const int ns1 = u;
        const int ns0 = s1;
        s1 = ns1;
        s0 = ns0;
    }
}

inline void bytes_to_lsb_bits(const uint8_t* bytes,
                              size_t n,
                              std::vector<int8_t>& bits)
{
    bits.resize(n * 8);
    for (size_t i = 0; i < n; ++i)
        for (int b = 0; b < 8; ++b)
            bits[8 * i + b] = static_cast<int8_t>((bytes[i] >> b) & 1);
}

inline void encode_phr19(size_t psdu_bytes,
                         size_t sync_repetitions,
                         bool ranging,
                         std::vector<int8_t>& phr19)
{
    std::vector<int8_t> sys13(13, 0);
    // Data rate 6.81 Mb/s = index 2, MSB-first.
    sys13[0] = 1;
    sys13[1] = 0;
    for (int i = 0; i < 7; ++i)
        sys13[2 + i] = static_cast<int8_t>((psdu_bytes >> (6 - i)) & 1);
    sys13[9] = ranging ? 1 : 0;
    sys13[10] = 0;
    // Preamble duration: 16/64/1024/4096.  Non-native SYNC lengths still
    // advertise the nearest native (64 or 1024) like MATLAB BPRF.
    size_t idx = 1; // 64
    if (sync_repetitions >= 1024)
        idx = 2;
    else if (sync_repetitions <= 16)
        idx = 0;
    sys13[11] = static_cast<int8_t>((idx >> 1) & 1);
    sys13[12] = static_cast<int8_t>(idx & 1);
    demod::core::detail::hrp_secded(sys13, phr19);
}

inline void bpm_place(float* chips,
                      size_t nsym,
                      size_t cpb,
                      size_t cps,
                      const int8_t* spread,
                      const int8_t* g0,
                      const int8_t* g1)
{
    const size_t total = nsym * cps;
    std::fill(chips, chips + total, 0.f);
    for (size_t s = 0; s < nsym; ++s) {
        const int8_t* sp = spread + s * cpb;
        const size_t hop = (sp[0] < 0) ? 1u : 0u;
        const size_t half = (g0[s] & 1) ? (cps / 2) : 0u;
        const size_t pos = s * cps + half + hop * cpb;
        const float pol = (g1[s] & 1) ? -1.f : 1.f;
        for (size_t c = 0; c < cpb; ++c)
            chips[pos + c] = pol * static_cast<float>(sp[c]);
    }
}

inline void fill_sts_bprf(float* chips)
{
    std::fill(chips, chips + kStsChipsBprf, 0.f);
    size_t off = kStsGapChips;
    for (size_t blk = 0; blk < kStsDrbgBlocksBprf; ++blk) {
        for (size_t byte = 0; byte < 16; ++byte) {
            const uint8_t v = kStsDrbgBprf[blk][byte];
            for (int b = 7; b >= 0; --b) {
                const int bit = (v >> b) & 1;
                chips[off] = bit ? -1.f : 1.f;
                off += kStsSpreadingBprf;
            }
        }
    }
}

inline void add_shaped_chip(float* y,
                            size_t n_out,
                            size_t chip,
                            float amp,
                            const float* taps,
                            size_t n_taps)
{
    const size_t n0 = chip * demod::kQm35SamplesPerChip;
    if (n0 >= n_out || amp == 0.f)
        return;
    const size_t nmax = std::min(n_out, n0 + n_taps);
    float* dst = y + n0;
    for (size_t k = 0; k < nmax - n0; ++k)
        dst[k] += amp * taps[k];
}

inline bool sfd_lookup(const char* mode, const int8_t*& data, size_t& n)
{
    if (!mode)
        return false;
    if (std::strcmp(mode, "ieee") == 0) {
        data = demod::kSfdIeee.data();
        n = demod::kSfdIeee.size();
        return true;
    }
    if (std::strcmp(mode, "decawave") == 0) {
        data = demod::kSfdDecawave.data();
        n = demod::kSfdDecawave.size();
        return true;
    }
    if (std::strcmp(mode, "4z1") == 0) {
        data = demod::kSfd4z1.data();
        n = demod::kSfd4z1.size();
        return true;
    }
    if (std::strcmp(mode, "4z2") == 0) {
        data = demod::kSfd4z2.data();
        n = demod::kSfd4z2.size();
        return true;
    }
    if (std::strcmp(mode, "4z3") == 0) {
        data = demod::kSfd4z3.data();
        n = demod::kSfd4z3.size();
        return true;
    }
    if (std::strcmp(mode, "4z4") == 0) {
        data = demod::kSfd4z4.data();
        n = demod::kSfd4z4.size();
        return true;
    }
    return false;
}

inline void copy_sfd_key(char* dst, size_t dst_n, const char* src)
{
    if (dst_n == 0)
        return;
    if (!src) {
        dst[0] = 0;
        return;
    }
    size_t n = std::strlen(src);
    if (n >= dst_n)
        n = dst_n - 1;
    std::memcpy(dst, src, n);
    dst[n] = 0;
}

inline bool fill_bprf_spread(int8_t* out,
                             size_t nbits,
                             size_t code_index,
                             size_t start_bit)
{
    static const int8_t kInit9[15] = {
        0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1
    };
    static const int8_t kInit10[15] = {
        0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1
    };
    static const int8_t kInit11[15] = {
        1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1
    };
    static const int8_t kInit12[15] = {
        1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0
    };
    const int8_t* init = nullptr;
    if (code_index == 9)
        init = kInit9;
    else if (code_index == 10)
        init = kInit10;
    else if (code_index == 11)
        init = kInit11;
    else if (code_index == 12)
        init = kInit12;
    else
        return false;
    int8_t r[15];
    std::memcpy(r, init, 15);
    size_t t = 0;
    auto next_bit = [&]() -> int8_t {
        if (t < 15)
            return r[t++];
        const int8_t b = static_cast<int8_t>(r[1] ^ r[0]);
        std::memmove(r, r + 1, 14);
        r[14] = b;
        ++t;
        return b;
    };
    for (size_t i = 0; i < start_bit; ++i)
        (void)next_bit();
    for (size_t i = 0; i < nbits; ++i)
        out[i] = next_bit() ? int8_t(-1) : int8_t(1);
    return true;
}

inline void add_sts_pulses(float* y,
                           size_t n_out,
                           size_t chip0,
                           const float* taps,
                           size_t n_taps)
{
    size_t off = chip0 + kStsGapChips;
    for (size_t blk = 0; blk < kStsDrbgBlocksBprf; ++blk) {
        for (size_t byte = 0; byte < 16; ++byte) {
            const uint8_t v = kStsDrbgBprf[blk][byte];
            for (int b = 7; b >= 0; --b) {
                const float amp = ((v >> b) & 1) ? -1.f : 1.f;
                add_shaped_chip(y, n_out, off, amp, taps, n_taps);
                off += kStsSpreadingBprf;
            }
        }
    }
}

inline void add_sync_symbol(float* y,
                            size_t n_out,
                            size_t symbol_index,
                            int8_t polarity,
                            const int8_t* pc,
                            const float* taps,
                            size_t n_taps)
{
    if (polarity == 0)
        return;
    const size_t chip0 = symbol_index * demod::kQm35ChipsPerSymbol;
    for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
        if (pc[c] == 0)
            continue;
        add_shaped_chip(y, n_out, chip0 + c * demod::kQm35SpreadingFactor,
                        static_cast<float>(polarity * pc[c]), taps, n_taps);
    }
}

inline void bpm_add_pulses(float* y,
                           size_t n_out,
                           size_t chip0,
                           size_t nsym,
                           size_t cpb,
                           size_t cps,
                           const int8_t* spread,
                           const int8_t* g0,
                           const int8_t* g1,
                           const float* taps,
                           size_t n_taps)
{
    for (size_t s = 0; s < nsym; ++s) {
        const int8_t* sp = spread + s * cpb;
        const size_t hop = (sp[0] < 0) ? 1u : 0u;
        const size_t half = (g0[s] & 1) ? (cps / 2) : 0u;
        const size_t pos = chip0 + s * cps + half + hop * cpb;
        const float pol = (g1[s] & 1) ? -1.f : 1.f;
        for (size_t c = 0; c < cpb; ++c)
            add_shaped_chip(y, n_out, pos + c, pol * static_cast<float>(sp[c]),
                            taps, n_taps);
    }
}

inline float abs_peak(const float* y, size_t n)
{
    float peak = 0.f;
    size_t i = 0;
    for (; i + 4 <= n; i += 4) {
        peak = std::max(peak, std::fabs(y[i]));
        peak = std::max(peak, std::fabs(y[i + 1]));
        peak = std::max(peak, std::fabs(y[i + 2]));
        peak = std::max(peak, std::fabs(y[i + 3]));
    }
    for (; i < n; ++i)
        peak = std::max(peak, std::fabs(y[i]));
    return peak;
}

inline bool prefix_matches(const HrpPrefixCache& p,
                           const HrpModConfig& cfg,
                           size_t n_sfd)
{
    if (!p.valid)
        return false;
    if (p.code_index != cfg.code_index ||
        p.sync_repetitions != cfg.sync_repetitions ||
        p.insert_sts != cfg.insert_sts || p.n_sfd != n_sfd)
        return false;
    if (p.pulse.shape != cfg.pulse.shape ||
        p.pulse.gaussian_sigma_ns != cfg.pulse.gaussian_sigma_ns ||
        p.pulse.blackman_bw_mhz != cfg.pulse.blackman_bw_mhz ||
        p.pulse.external_n_taps != cfg.pulse.external_n_taps)
        return false;
    const char* mode = cfg.sfd_mode ? cfg.sfd_mode : "";
    return std::strcmp(p.sfd_mode, mode) == 0;
}

inline bool ensure_prefix(const HrpModConfig& cfg,
                          const int8_t* sfd,
                          size_t n_sfd,
                          HrpModScratch& scratch,
                          const float* taps,
                          size_t n_taps)
{
    if (prefix_matches(scratch.prefix, cfg, n_sfd) &&
        scratch.prefix.pulse_generation == scratch.pulse_generation)
        return true;
    const int8_t* pc = demod::GetPreambleCode(cfg.code_index);
    const size_t sync_chips =
        cfg.sync_repetitions * demod::kQm35ChipsPerSymbol;
    const size_t sfd_chips = n_sfd * demod::kQm35ChipsPerSymbol;
    const size_t sts_chips = cfg.insert_sts ? kStsChipsBprf : 0;
    const size_t prefix_samples =
        (sync_chips + sfd_chips + sts_chips) * demod::kQm35SamplesPerChip;
    const size_t shaped = prefix_samples + n_taps;
    if (shaped == 0 || shaped > kMaxHrpTxSamples)
        return false;
    scratch.prefix.samples.resize(shaped);
    std::memset(scratch.prefix.samples.data(), 0, shaped * sizeof(float));
    float* y = scratch.prefix.samples.data();
    for (size_t r = 0; r < cfg.sync_repetitions; ++r)
        add_sync_symbol(y, shaped, r, 1, pc, taps, n_taps);
    for (size_t s = 0; s < n_sfd; ++s)
        add_sync_symbol(y, shaped, cfg.sync_repetitions + s, sfd[s], pc,
                        taps, n_taps);
    if (cfg.insert_sts)
        add_sts_pulses(y, shaped, sync_chips + sfd_chips, taps, n_taps);
    scratch.prefix.valid = true;
    scratch.prefix.code_index = cfg.code_index;
    scratch.prefix.sync_repetitions = cfg.sync_repetitions;
    scratch.prefix.insert_sts = cfg.insert_sts;
    scratch.prefix.pulse = cfg.pulse;
    scratch.prefix.pulse_generation = scratch.pulse_generation;
    scratch.prefix.n_sfd = n_sfd;
    scratch.prefix.n_samples = shaped;
    copy_sfd_key(scratch.prefix.sfd_mode, sizeof(scratch.prefix.sfd_mode),
                 cfg.sfd_mode);
    return true;
}

} // namespace detail

inline bool modulate_one(const uint8_t* psdu,
                         size_t psdu_len,
                         const HrpModConfig& cfg,
                         HrpModScratch& scratch,
                         std::complex<float>* out,
                         size_t out_cap,
                         size_t& out_n)
{
    out_n = 0;
    if (!out || (psdu_len > 0 && !psdu))
        return false;
    if (psdu_len > radar_meta::kMaxPsduBytes)
        return false;
    if (!radar_meta::code_index_supported(cfg.code_index))
        return false;
    if (!radar_meta::sync_reps_supported(cfg.sync_repetitions))
        return false;
    if (cfg.insert_sts && !sfd_mode_is_4z(cfg.sfd_mode))
        return false;
    if (!(cfg.peak_amplitude > 0.f) || !std::isfinite(cfg.peak_amplitude))
        return false;
    if (!pulse_spec_valid(cfg.pulse))
        return false;

    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    if (!detail::sfd_lookup(cfg.sfd_mode, sfd, n_sfd) || n_sfd == 0)
        return false;

    if (!ensure_pulse(cfg, scratch))
        return false;
    const float* taps = scratch.pulse_data();
    const size_t n_taps = scratch.pulse_len();

    const size_t n_base = packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, psdu_len, cfg.insert_sts);
    const size_t n_out = packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, psdu_len, cfg.insert_sts, &cfg.pulse);
    if (n_base == 0 || n_out == 0 || n_out > out_cap ||
        n_out > kMaxHrpTxSamples)
        return false;

    detail::bytes_to_lsb_bits(psdu, psdu_len, scratch.psdu_bits);
    if (!detail::rs_encode_stream(scratch.psdu_bits.data(),
                                  scratch.psdu_bits.size(),
                                  scratch.rs_cw))
        return false;

    detail::encode_phr19(psdu_len, cfg.sync_repetitions, cfg.ranging,
                         scratch.phr19);
    scratch.conv_in.clear();
    scratch.conv_in.insert(scratch.conv_in.end(),
                           scratch.phr19.begin(),
                           scratch.phr19.end());
    scratch.conv_in.insert(scratch.conv_in.end(),
                           scratch.rs_cw.begin(),
                           scratch.rs_cw.end());
    scratch.conv_in.push_back(0);
    scratch.conv_in.push_back(0);
    detail::convenc_cl3(scratch.conv_in.data(), scratch.conv_in.size(),
                        scratch.coded);

    const size_t nsym = scratch.rs_cw.size();
    const size_t n_enc_sym = scratch.conv_in.size(); // 19+nsym+2
    if (n_enc_sym != kPhrSymbols + nsym)
        return false;

    const size_t sync_chips =
        cfg.sync_repetitions * demod::kQm35ChipsPerSymbol;
    const size_t sfd_chips = n_sfd * demod::kQm35ChipsPerSymbol;
    const size_t sts_chips = cfg.insert_sts ? kStsChipsBprf : 0;
    const size_t phr_chips = kPhrSymbols * kPhrChipsPerSymbol;
    const size_t pay_chips = nsym * kPayloadChipsPerSymbol;
    const size_t n_chips =
        sync_chips + sfd_chips + sts_chips + phr_chips + pay_chips;
    if (n_chips * demod::kQm35SamplesPerChip != n_base)
        return false;
    if (n_base + pulse_tail_extra(cfg.pulse) != n_out)
        return false;

    if (!detail::ensure_prefix(cfg, sfd, n_sfd, scratch, taps, n_taps))
        return false;
    const size_t prefix_n =
        std::min(scratch.prefix.n_samples, n_out);
    if (prefix_n == 0)
        return false;

    if (scratch.iq_re.size() < n_out)
        scratch.iq_re.resize(n_out);
    if (scratch.iq_im.size() < n_out)
        scratch.iq_im.assign(n_out, 0.f);
    float* y = scratch.iq_re.data();
    std::memcpy(y, scratch.prefix.samples.data(), prefix_n * sizeof(float));
    if (prefix_n < n_out)
        std::memset(y + prefix_n, 0, (n_out - prefix_n) * sizeof(float));

    scratch.g0.resize(n_enc_sym);
    scratch.g1.resize(n_enc_sym);
    for (size_t s = 0; s < n_enc_sym; ++s) {
        scratch.g0[s] = scratch.coded[2 * s];
        scratch.g1[s] = scratch.coded[2 * s + 1];
    }

    const size_t phr_chip0 = sync_chips + sfd_chips + sts_chips;
    scratch.spread.resize(kPhrChipsPerBurst * kPhrSymbols);
    if (!detail::fill_bprf_spread(scratch.spread.data(),
                                  kPhrChipsPerBurst * kPhrSymbols,
                                  cfg.code_index, 0))
        return false;
    detail::bpm_add_pulses(y, n_out, phr_chip0, kPhrSymbols,
                           kPhrChipsPerBurst, kPhrChipsPerSymbol,
                           scratch.spread.data(), scratch.g0.data(),
                           scratch.g1.data(), taps, n_taps);
    if (nsym > 0) {
        const size_t n_spread = kPayloadChipsPerBurst * nsym;
        scratch.spread.resize(n_spread);
        if (!detail::fill_bprf_spread(scratch.spread.data(), n_spread,
                                      cfg.code_index, kPayloadScramblerOffset))
            return false;
        detail::bpm_add_pulses(y, n_out, phr_chip0 + phr_chips, nsym,
                               kPayloadChipsPerBurst, kPayloadChipsPerSymbol,
                               scratch.spread.data(),
                               scratch.g0.data() + kPhrSymbols,
                               scratch.g1.data() + kPhrSymbols, taps, n_taps);
    }

    const float peak = detail::abs_peak(y, n_out);
    if (!(peak > 0.f))
        return false;
    const float scale = cfg.peak_amplitude / peak;
    volk_32f_s32f_multiply_32f(y, y, scale,
                               static_cast<unsigned int>(n_out));
    volk_32f_x2_interleave_32fc(reinterpret_cast<lv_32fc_t*>(out), y,
                                scratch.iq_im.data(),
                                static_cast<unsigned int>(n_out));
    out_n = n_out;
    return true;
}

} // namespace mod
} // namespace uwb
} // namespace gr
