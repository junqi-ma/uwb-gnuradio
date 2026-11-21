/*
 * openuwb::mod implementation: the STS DRBG table and the pulse-shaping
 * lookup tables (kStsDrbgBprf, kPulse48), the RS/PHR/convolution/placement
 * helpers and all modulator entry points.  These bodies were moved out of
 * include/openuwb/phy/hrp/tx/hrp_mod_core.h; the header now declares the
 * public API and structs, so consumers compile unchanged.
 */

#include <openuwb/phy/hrp/tx/hrp_mod_core.h>

#include <openuwb/sensing/radar_meta_core.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <volk/volk.h>

namespace openuwb {
namespace mod {

constexpr uint8_t kStsDrbgBprf[32][16] = {
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

constexpr float kPulse48[kPulseTaps] = {
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

// ---- public utility bodies ----
const char* pulse_shape_name(PulseShape shape)
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

bool parse_pulse_shape(const std::string& name, PulseShape& out)
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

size_t pulse_n_taps(const PulseSpec& spec)
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

size_t pulse_center_tap(const PulseSpec& spec)
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

bool pulse_spec_valid(const PulseSpec& spec)
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

std::vector<float> make_pulse_taps(const PulseSpec& spec)
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

bool ensure_pulse(const HrpModConfig& cfg, HrpModScratch& scratch)
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

size_t pulse_tail_extra(const PulseSpec& spec)
{
    if (spec.shape == PulseShape::Legacy)
        return 0;
    return pulse_n_taps(spec) - demod::kQm35SamplesPerChip;
}

size_t payload_bpm_symbols(size_t psdu_bytes)
{
    if (psdu_bytes == 0)
        return 0;
    const size_t psdu_bits = psdu_bytes * 8;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    return psdu_bits + 48 * num_blocks;
}

size_t packet_samples_998p4(size_t sync_repetitions,
                            size_t n_sfd,
                            size_t psdu_bytes,
                            bool insert_sts,
                            const PulseSpec* pulse)
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

bool sfd_mode_is_4z(const char* sfd_mode)
{
    if (!sfd_mode)
        return false;
    const std::string s(sfd_mode);
    return s == "4z1" || s == "4z2" || s == "4z3" || s == "4z4";
}

// ---- detail namespace bodies ----

namespace detail {

const int* rs_generator()
{
    // g(x) = Π_{i=1..8} (x + α^i), α=2, low-first, deg 8.
    static int g[9];
    static bool ready = false;
    if (!ready) {
        std::fill(g, g + 9, 0);
        g[0] = 1;
        int deg = 0;
        for (int i = 1; i <= 8; ++i) {
            const int root = codec::rs_gf_pow(2, i);
            int ng[10] = {};
            for (int j = 0; j <= deg; ++j) {
                ng[j] ^= codec::rs_gf_mul(g[j], root);
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

void bits_to_nsyms(const int8_t* bits, int nsym, int* syms)
{
    codec::rs_bits_to_syms(bits, nsym, syms);
}

void rs_encode_330(const int8_t* data330, int8_t* coded378)
{
    int dsym[55];
    bits_to_nsyms(data330, 55, dsym);
    const int* g = rs_generator();
    int reg[8] = {};
    for (int i = 54; i >= 0; --i) {
        const int fb = dsym[i] ^ reg[7];
        int nreg[8];
        nreg[0] = codec::rs_gf_mul(fb, g[0]);
        for (int k = 1; k < 8; ++k)
            nreg[k] = reg[k - 1] ^ codec::rs_gf_mul(fb, g[k]);
        for (int k = 0; k < 8; ++k)
            reg[k] = nreg[k];
    }
    int cw[63];
    for (int i = 0; i < 55; ++i)
        cw[i] = dsym[i];
    for (int i = 0; i < 8; ++i)
        cw[55 + i] = reg[i];
    codec::rs_syms_to_bits(cw, 63, coded378);
}

bool rs_encode_stream(const int8_t* data_bits,
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

void convenc_cl3(const int8_t* bits, size_t n, std::vector<int8_t>& coded)
{
    int state = 0;
    coded.resize(2 * n);
    for (size_t t = 0; t < n; ++t) {
        const auto step = codec::cl3_transition(state, bits[t] & 1);
        coded[2 * t] = static_cast<int8_t>(step.g0);
        coded[2 * t + 1] = static_cast<int8_t>(step.g1);
        state = step.next_state;
    }
}

void bytes_to_lsb_bits(const uint8_t* bytes,
                              size_t n,
                              std::vector<int8_t>& bits)
{
    bits.resize(n * 8);
    for (size_t i = 0; i < n; ++i)
        for (int b = 0; b < 8; ++b)
            bits[8 * i + b] = static_cast<int8_t>((bytes[i] >> b) & 1);
}

void encode_phr19(size_t psdu_bytes,
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
    codec::hrp_secded(sys13, phr19);
}

void bpm_place(float* chips,
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

void fill_sts_bprf(float* chips)
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

void add_shaped_chip(float* y,
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

bool sfd_lookup(const char* mode, const int8_t*& data, size_t& n)
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

void copy_sfd_key(char* dst, size_t dst_n, const char* src)
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

using codec::fill_bprf_spread;

bool ensure_pn(HrpModScratch& scratch, size_t code_index, size_t min_len)
{
    if (scratch.pn_valid && scratch.pn_code_index == code_index &&
        scratch.spread.size() >= min_len) {
        return true;
    }
    const size_t target_len = std::max(scratch.spread.capacity(), min_len);
    if (scratch.spread.size() < target_len)
        scratch.spread.resize(target_len);
    if (!fill_bprf_spread(scratch.spread.data(), scratch.spread.size(),
                          code_index, 0)) {
        scratch.pn_valid = false;
        scratch.pn_code_index = 0;
        return false;
    }
    scratch.pn_code_index = code_index;
    scratch.pn_valid = true;
    return true;
}

void add_sts_pulses(float* y,
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

void add_sync_symbol(float* y,
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

void bpm_add_pulses(float* y,
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

bool add_pulse_event(PulseEvent* events,
                            size_t capacity,
                            size_t& count,
                            size_t sample_offset,
                            int sign)
{
    if (!events || count >= capacity || sample_offset > 0xffffffffu)
        return false;
    if (sign != 1 && sign != -1)
        return false;
    if (count && sample_offset <= events[count - 1].sample_offset)
        return false;
    events[count].sample_offset = static_cast<uint32_t>(sample_offset);
    events[count].sign = static_cast<int8_t>(sign);
    ++count;
    return true;
}

bool add_sync_symbol_events(PulseEvent* events,
                                   size_t capacity,
                                   size_t& count,
                                   size_t symbol_index,
                                   int8_t polarity,
                                   const int8_t* pc)
{
    if (polarity == 0)
        return true;
    const size_t chip0 = symbol_index * demod::kQm35ChipsPerSymbol;
    for (size_t c = 0; c < demod::kQm35CodeLength; ++c) {
        if (pc[c] == 0)
            continue;
        const size_t chip = chip0 + c * demod::kQm35SpreadingFactor;
        if (!add_pulse_event(events, capacity, count,
                             chip * demod::kQm35SamplesPerChip,
                             static_cast<int>(polarity) *
                                 static_cast<int>(pc[c])))
            return false;
    }
    return true;
}

bool add_sts_events(PulseEvent* events,
                           size_t capacity,
                           size_t& count,
                           size_t chip0)
{
    size_t off = chip0 + kStsGapChips;
    for (size_t blk = 0; blk < kStsDrbgBlocksBprf; ++blk) {
        for (size_t byte = 0; byte < 16; ++byte) {
            const uint8_t v = kStsDrbgBprf[blk][byte];
            for (int b = 7; b >= 0; --b) {
                const int sign = ((v >> b) & 1) ? -1 : 1;
                if (!add_pulse_event(events, capacity, count,
                                     off * demod::kQm35SamplesPerChip, sign))
                    return false;
                off += kStsSpreadingBprf;
            }
        }
    }
    return true;
}

bool bpm_add_pulse_events(PulseEvent* events,
                                 size_t capacity,
                                 size_t& count,
                                 size_t chip0,
                                 size_t nsym,
                                 size_t cpb,
                                 size_t cps,
                                 const int8_t* spread,
                                 const int8_t* g0,
                                 const int8_t* g1)
{
    for (size_t s = 0; s < nsym; ++s) {
        const int8_t* sp = spread + s * cpb;
        const size_t hop = (sp[0] < 0) ? 1u : 0u;
        const size_t half = (g0[s] & 1) ? (cps / 2) : 0u;
        const size_t pos = chip0 + s * cps + half + hop * cpb;
        const int pol = (g1[s] & 1) ? -1 : 1;
        for (size_t c = 0; c < cpb; ++c) {
            if (!add_pulse_event(events, capacity, count,
                                 (pos + c) * demod::kQm35SamplesPerChip,
                                 pol * static_cast<int>(sp[c])))
                return false;
        }
    }
    return true;
}

float abs_peak(const float* y, size_t n)
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

bool prefix_key_matches(const HrpPrefixCache& p,
                               const HrpModConfig& cfg,
                               size_t n_sfd,
                               uint64_t pulse_generation)
{
    if (p.pulse_generation != pulse_generation)
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

void prefix_key_update(HrpPrefixCache& p,
                              const HrpModConfig& cfg,
                              size_t n_sfd,
                              uint64_t pulse_generation)
{
    p.code_index = cfg.code_index;
    p.sync_repetitions = cfg.sync_repetitions;
    p.insert_sts = cfg.insert_sts;
    p.pulse = cfg.pulse;
    p.pulse_generation = pulse_generation;
    p.n_sfd = n_sfd;
    copy_sfd_key(p.sfd_mode, sizeof(p.sfd_mode), cfg.sfd_mode);
    p.valid = false;
    p.n_samples = 0;
    p.events_valid = false;
    p.event_count = 0;
}

bool ensure_prefix(const HrpModConfig& cfg,
                          const int8_t* sfd,
                          size_t n_sfd,
                          HrpModScratch& scratch,
                          const float* taps,
                          size_t n_taps)
{
    if (!prefix_key_matches(scratch.prefix, cfg, n_sfd,
                            scratch.pulse_generation))
        prefix_key_update(scratch.prefix, cfg, n_sfd,
                          scratch.pulse_generation);
    if (scratch.prefix.valid)
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
    scratch.prefix.n_samples = shaped;
    return true;
}

bool ensure_prefix_events(const HrpModConfig& cfg,
                                 const int8_t* sfd,
                                 size_t n_sfd,
                                 HrpModScratch& scratch)
{
    if (!prefix_key_matches(scratch.prefix, cfg, n_sfd,
                            scratch.pulse_generation))
        prefix_key_update(scratch.prefix, cfg, n_sfd,
                          scratch.pulse_generation);
    if (scratch.prefix.events_valid)
        return true;
    const int8_t* pc = demod::GetPreambleCode(cfg.code_index);
    const size_t sync_chips =
        cfg.sync_repetitions * demod::kQm35ChipsPerSymbol;
    const size_t sfd_chips = n_sfd * demod::kQm35ChipsPerSymbol;
    const size_t sts_chips = cfg.insert_sts ? kStsChipsBprf : 0;
    const size_t prefix_samples =
        (sync_chips + sfd_chips + sts_chips) * demod::kQm35SamplesPerChip;
    const size_t capacity = scratch.prefix.events.capacity();
    if (prefix_samples == 0 || capacity == 0 ||
        prefix_samples / demod::kQm35SamplesPerChip > capacity)
        return false;
    scratch.prefix.events.resize(prefix_samples /
                                 demod::kQm35SamplesPerChip);
    size_t count = 0;
    PulseEvent* const events = scratch.prefix.events.data();
    for (size_t r = 0; r < cfg.sync_repetitions; ++r)
        if (!add_sync_symbol_events(events, capacity, count, r, 1, pc))
            return false;
    for (size_t s = 0; s < n_sfd; ++s)
        if (!add_sync_symbol_events(events, capacity, count,
                                    cfg.sync_repetitions + s, sfd[s], pc))
            return false;
    if (cfg.insert_sts &&
        !add_sts_events(events, capacity, count, sync_chips + sfd_chips))
        return false;
    scratch.prefix.event_count = count;
    scratch.prefix.events_valid = true;
    scratch.prefix.events.resize(count);
    return true;
}

bool encode_hrp_packet(const uint8_t* psdu,
                              size_t psdu_len,
                              const HrpModConfig& cfg,
                              HrpModScratch& scratch,
                              const int8_t* sfd,
                              size_t n_sfd,
                              HrpCodedGeometry& g)
{
    g = HrpCodedGeometry{};
    g.n_base = packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, psdu_len, cfg.insert_sts);
    g.n_out = packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, psdu_len, cfg.insert_sts, &cfg.pulse);
    if (g.n_base == 0 || g.n_out == 0 || g.n_out > kMaxHrpTxSamples)
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

    g.nsym = scratch.rs_cw.size();
    g.n_enc_sym = scratch.conv_in.size(); // 19+nsym+2
    if (g.n_enc_sym != kPhrSymbols + g.nsym)
        return false;

    g.sync_chips = cfg.sync_repetitions * demod::kQm35ChipsPerSymbol;
    g.sfd_chips = n_sfd * demod::kQm35ChipsPerSymbol;
    g.sts_chips = cfg.insert_sts ? kStsChipsBprf : 0;
    g.phr_chips = kPhrSymbols * kPhrChipsPerSymbol;
    g.pay_chips = g.nsym * kPayloadChipsPerSymbol;
    g.n_chips = g.sync_chips + g.sfd_chips + g.sts_chips + g.phr_chips +
                g.pay_chips;
    if (g.n_chips * demod::kQm35SamplesPerChip != g.n_base)
        return false;
    if (g.n_base + pulse_tail_extra(cfg.pulse) != g.n_out)
        return false;
    g.phr_chip0 = g.sync_chips + g.sfd_chips + g.sts_chips;

    scratch.g0.resize(g.n_enc_sym);
    scratch.g1.resize(g.n_enc_sym);
    for (size_t s = 0; s < g.n_enc_sym; ++s) {
        scratch.g0[s] = scratch.coded[2 * s];
        scratch.g1[s] = scratch.coded[2 * s + 1];
    }
    return true;
}

bool modulate_one_impl(const uint8_t* psdu,
                         size_t psdu_len,
                         const HrpModConfig& cfg,
                         HrpModScratch& scratch,
                         std::complex<float>* out,
                         size_t out_cap,
                         size_t& out_n,
                         bool unit_real)
{
    out_n = 0;
    if ((!out && !unit_real) || (psdu_len > 0 && !psdu))
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
    // The real-only SIC path is valid only for unit impulses.  Its absolute
    // receive amplitude and pulse shaping already live in the estimated CIR.
    if (unit_real && (cfg.peak_amplitude != 1.f ||
        cfg.pulse.shape != PulseShape::External ||
        cfg.pulse.external_n_taps != 2 || cfg.external_taps.size() != 2 ||
        cfg.external_taps[0] != 1.f || cfg.external_taps[1] != 0.f))
        return false;

    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    if (!detail::sfd_lookup(cfg.sfd_mode, sfd, n_sfd) || n_sfd == 0)
        return false;

    if (!ensure_pulse(cfg, scratch))
        return false;
    const float* taps = scratch.pulse_data();
    const size_t n_taps = scratch.pulse_len();

    HrpCodedGeometry g;
    if (!encode_hrp_packet(psdu, psdu_len, cfg, scratch, sfd, n_sfd, g))
        return false;
    const size_t n_out = g.n_out;
    if (n_out > out_cap)
        return false;
    const size_t nsym = g.nsym;

    if (!detail::ensure_prefix(cfg, sfd, n_sfd, scratch, taps, n_taps))
        return false;
    const size_t prefix_n =
        std::min(scratch.prefix.n_samples, n_out);
    if (prefix_n == 0)
        return false;

    if (scratch.iq_re.size() < n_out)
        scratch.iq_re.resize(n_out);
    if (!unit_real && scratch.iq_im.size() < n_out)
        scratch.iq_im.assign(n_out, 0.f);
    float* y = scratch.iq_re.data();
    std::memcpy(y, scratch.prefix.samples.data(), prefix_n * sizeof(float));
    if (prefix_n < n_out)
        std::memset(y + prefix_n, 0, (n_out - prefix_n) * sizeof(float));

    const size_t n_pay_spread = (nsym > 0) ? (kPayloadChipsPerBurst * nsym) : 0;
    const size_t total_spread_needed = kPayloadScramblerOffset + n_pay_spread;
    if (!detail::ensure_pn(scratch, cfg.code_index, total_spread_needed))
        return false;
    detail::bpm_add_pulses(y, n_out, g.phr_chip0, kPhrSymbols,
                           kPhrChipsPerBurst, kPhrChipsPerSymbol,
                           scratch.spread.data(), scratch.g0.data(),
                           scratch.g1.data(), taps, n_taps);
    if (nsym > 0) {
        detail::bpm_add_pulses(y, n_out, g.phr_chip0 + g.phr_chips, nsym,
                               kPayloadChipsPerBurst, kPayloadChipsPerSymbol,
                               scratch.spread.data() + kPayloadScramblerOffset,
                               scratch.g0.data() + kPhrSymbols,
                               scratch.g1.data() + kPhrSymbols, taps, n_taps);
    }

    if (unit_real) {
        // BPRF places one +/-1 impulse per selected chip, without overlap.
        // Keep the real scratch buffer; no peak scan, normalization or IQ
        // interleave. The caller consumes iq_re[0:out_n] directly.
        out_n = n_out;
        return true;
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

bool modulate_pulse_list_impl(const uint8_t* psdu,
                                     size_t psdu_len,
                                     const HrpModConfig& cfg,
                                     HrpModScratch& scratch,
                                     size_t sample_cap,
                                     PulseList& out)
{
    out = PulseList{};
    scratch.pulse_count = 0;
    scratch.pulse_packet_samples = 0;
    if (psdu_len > 0 && !psdu)
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
    // The sparse event list is the unit-impulse representation only.  Any
    // transmit shaping belongs to the estimated CIR, not to remodulation.
    if (cfg.peak_amplitude != 1.f ||
        cfg.pulse.shape != PulseShape::External ||
        cfg.pulse.external_n_taps != 2 || cfg.external_taps.size() != 2 ||
        cfg.external_taps[0] != 1.f || cfg.external_taps[1] != 0.f)
        return false;

    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    if (!detail::sfd_lookup(cfg.sfd_mode, sfd, n_sfd) || n_sfd == 0)
        return false;
    if (!ensure_pulse(cfg, scratch))
        return false;

    HrpCodedGeometry g;
    if (!encode_hrp_packet(psdu, psdu_len, cfg, scratch, sfd, n_sfd, g))
        return false;
    if (g.n_out > sample_cap || g.n_out > kMaxHrpTxSamples)
        return false;
    if (!detail::ensure_prefix_events(cfg, sfd, n_sfd, scratch))
        return false;

    const size_t phr_events = kPhrSymbols * kPhrChipsPerBurst;
    const size_t pay_events = g.nsym * kPayloadChipsPerBurst;
    const size_t total_events = scratch.prefix.event_count + phr_events +
                                pay_events;
    if (total_events > scratch.pulse_events.capacity())
        return false;
    if (scratch.prefix.event_count == 0)
        return false;

    scratch.pulse_events.resize(total_events);
    PulseEvent* const events = scratch.pulse_events.data();
    std::memcpy(events, scratch.prefix.events.data(),
                scratch.prefix.event_count * sizeof(PulseEvent));
    size_t count = scratch.prefix.event_count;

    // PHR: same BPM hop/half/polarity placement as the waveform path.
    const size_t n_pay_spread = (g.nsym > 0) ? (kPayloadChipsPerBurst * g.nsym) : 0;
    const size_t total_spread_needed = kPayloadScramblerOffset + n_pay_spread;
    if (!detail::ensure_pn(scratch, cfg.code_index, total_spread_needed))
        return false;
    if (!detail::bpm_add_pulse_events(events, total_events, count,
                                      g.phr_chip0, kPhrSymbols,
                                      kPhrChipsPerBurst,
                                      kPhrChipsPerSymbol,
                                      scratch.spread.data(),
                                      scratch.g0.data(), scratch.g1.data()))
        return false;

    // payload: same spreading decision, appended after the PHR segment.
    if (g.nsym > 0) {
        if (!detail::bpm_add_pulse_events(
                events, total_events, count,
                g.phr_chip0 + g.phr_chips, g.nsym,
                kPayloadChipsPerBurst, kPayloadChipsPerSymbol,
                scratch.spread.data() + kPayloadScramblerOffset,
                scratch.g0.data() + kPhrSymbols,
                scratch.g1.data() + kPhrSymbols))
            return false;
    }
    if (count != total_events)
        return false;

    scratch.pulse_count = count;
    scratch.pulse_packet_samples = g.n_out;
    out.events = events;
    out.count = count;
    out.packet_samples = g.n_out;
    return true;
}

} // namespace detail

// ---- public modulate bodies ----

bool modulate_one(const uint8_t* psdu, size_t psdu_len,
                         const HrpModConfig& cfg, HrpModScratch& scratch,
                         std::complex<float>* out, size_t out_cap, size_t& out_n)
{
    return detail::modulate_one_impl(psdu, psdu_len, cfg, scratch,
                                     out, out_cap, out_n, false);
}

bool modulate_unit_impulses(const uint8_t* psdu, size_t psdu_len,
                                  const HrpModConfig& cfg, HrpModScratch& scratch,
                                  size_t out_cap, size_t& out_n)
{
    return detail::modulate_one_impl(psdu, psdu_len, cfg, scratch,
                                     nullptr, out_cap, out_n, true);
}

bool modulate_pulse_list(const uint8_t* psdu, size_t psdu_len,
                                const HrpModConfig& cfg,
                                HrpModScratch& scratch, size_t sample_cap,
                                PulseList& out)
{
    return detail::modulate_pulse_list_impl(psdu, psdu_len, cfg, scratch,
                                            sample_cap, out);
}

bool modulate_prefix_pulse_list(const HrpModConfig& cfg,
                                       HrpModScratch& scratch,
                                       size_t sample_cap,
                                       PulseList& out)
{
    out = PulseList{};
    if (cfg.code_index < 9 || cfg.code_index > 12)
        return false;
    if (cfg.sync_repetitions == 0)
        return false;

    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    if (!detail::sfd_lookup(cfg.sfd_mode, sfd, n_sfd))
        return false;

    if (!ensure_pulse(cfg, scratch))
        return false;
    if (!detail::ensure_prefix_events(cfg, sfd, n_sfd, scratch))
        return false;

    const size_t sync_chips =
        cfg.sync_repetitions * demod::kQm35ChipsPerSymbol;
    const size_t sfd_chips = n_sfd * demod::kQm35ChipsPerSymbol;
    const size_t sts_chips = cfg.insert_sts ? kStsChipsBprf : 0;
    const size_t prefix_samples =
        (sync_chips + sfd_chips + sts_chips) * demod::kQm35SamplesPerChip;
    if (prefix_samples == 0 || prefix_samples > sample_cap ||
        prefix_samples > kMaxHrpTxSamples)
        return false;
    // The event list must fit strictly inside the declared span.
    if (scratch.prefix.event_count == 0 ||
        scratch.prefix.events[scratch.prefix.event_count - 1].sample_offset >=
            prefix_samples)
        return false;

    out.events = scratch.prefix.events.data();
    out.count = scratch.prefix.event_count;
    out.packet_samples = prefix_samples;
    return true;
}

bool modulate_prefix_waveform(const HrpModConfig& cfg,
                                     HrpModScratch& scratch,
                                     std::complex<float>* out,
                                     size_t out_cap,
                                     size_t& out_n)
{
    out_n = 0;
    if (!out || cfg.code_index < 9 || cfg.code_index > 12 || cfg.sync_repetitions == 0)
        return false;

    const int8_t* sfd = nullptr;
    size_t n_sfd = 0;
    if (!detail::sfd_lookup(cfg.sfd_mode, sfd, n_sfd))
        return false;

    if (!ensure_pulse(cfg, scratch))
        return false;
    if (!detail::ensure_prefix(cfg, sfd, n_sfd, scratch, scratch.pulse_data(), scratch.pulse_len()))
        return false;

    const size_t prefix_samples = scratch.prefix.n_samples;
    if (prefix_samples > out_cap)
        return false;

    const float* const re = scratch.prefix.samples.data();
    for (size_t i = 0; i < prefix_samples; ++i) {
        out[i] = std::complex<float>(re[i], 0.f);
    }
    out_n = prefix_samples;
    return true;
}

// ---- HrpModScratch members ----

const float* HrpModScratch::pulse_data() const
{
    return pulse_taps.empty() ? kPulse48 : pulse_taps.data();
}

size_t HrpModScratch::pulse_len() const
{
    return pulse_taps.empty() ? kPulseTaps : pulse_taps.size();
}

void HrpModScratch::invalidate_pn()
{
    pn_valid = false;
    pn_code_index = 0;
}

void HrpModScratch::reserve(size_t max_psdu_bytes, size_t max_samples)
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
    const size_t max_spread =
        kPayloadScramblerOffset + kPayloadChipsPerBurst * max_nsym;
    spread.reserve(max_spread);
    spread.resize(max_spread);
    pn_code_index = 0;
    pn_valid = false;
    iq_re.reserve(max_samples);
    iq_im.assign(max_samples, 0.f);
    prefix.samples.reserve(max_samples);
    const size_t max_events =
        max_samples / demod::kQm35SamplesPerChip + 2;
    pulse_events.reserve(max_events);
    prefix.events.reserve(max_events);
}

} // namespace mod
} // namespace openuwb
