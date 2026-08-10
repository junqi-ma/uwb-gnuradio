/* -*- c++ -*- */
/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_UWB_TX_RECONSTRUCTOR_H
#define INCLUDED_UWB_TX_RECONSTRUCTOR_H

#include <gnuradio/uwb/uwb_phy_profile.h>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace gr {
namespace uwb {
namespace sic {

enum class ReconstructStatus {
    Success,
    FcsFailed,
    InvalidInput,
    RsEncodeFailed
};

struct FieldBounds {
    size_t begin = 0;
    size_t end = 0; // zero-based, end-exclusive
};

struct TxReconstruction {
    ReconstructStatus status = ReconstructStatus::InvalidInput;
    std::vector<float> pulse_impulses;
    std::vector<std::complex<float>> replica;
    FieldBounds sync;
    FieldBounds sfd;
    FieldBounds phr;
    FieldBounds payload;

    void reserve(size_t max_samples)
    {
        pulse_impulses.reserve(max_samples);
        replica.reserve(max_samples);
    }
};

struct TxReconstructionScratch {
    std::vector<int8_t> psdu_bits;
    std::vector<int8_t> rs_bits;
    std::vector<int8_t> encoder_input;
    std::vector<int8_t> g0;
    std::vector<int8_t> g1;
    std::vector<int8_t> spread;
    std::vector<int8_t> lfsr_sequence;

    void reserve(size_t max_psdu_bytes)
    {
        const size_t data_bits = max_psdu_bytes * 8;
        const size_t blocks = (data_bits + 329) / 330;
        const size_t rs_count = data_bits + blocks * 48;
        psdu_bits.reserve(data_bits);
        rs_bits.reserve(rs_count);
        encoder_input.reserve(19 + rs_count + 2);
        g0.reserve(19 + rs_count + 2);
        g1.reserve(19 + rs_count + 2);
        spread.reserve(std::max<size_t>(21 * 64, rs_count * 8));
        lfsr_sequence.reserve(21 * 64 + rs_count * 8);
    }
};

namespace tx_detail {

inline uint16_t crc16_802154(const uint8_t* data, size_t length)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) :
                              static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

inline int gf_mul(int a, int b)
{
    int p = 0;
    a &= 0x3f;
    b &= 0x3f;
    for (int i = 0; i < 6; ++i) {
        if (b & 1)
            p ^= a;
        const bool carry = (a & 0x20) != 0;
        a = (a << 1) & 0x3f;
        if (carry)
            a ^= 0x21;
        b >>= 1;
    }
    return p & 0x3f;
}

inline int gf_pow(int a, int n)
{
    int result = 1;
    n %= 63;
    if (n < 0)
        n += 63;
    while (n) {
        if (n & 1)
            result = gf_mul(result, a);
        a = gf_mul(a, a);
        n >>= 1;
    }
    return result;
}

inline int gf_inv(int a) { return a ? gf_pow(a, 62) : 0; }

// Solve the eight parity symbols at codeword positions 55..62 such that
// r(alpha^i)=0, i=1..8. The systematic data symbols occupy positions 0..54,
// matching the decoder's low-first RS polynomial convention.
inline bool rs_parity(const int data[55], int parity[8])
{
    int matrix[8][9] = {};
    for (int row = 0; row < 8; ++row) {
        const int root = gf_pow(2, row + 1);
        int power = 1;
        int syndrome = 0;
        for (int position = 0; position < 63; ++position) {
            if (position < 55)
                syndrome ^= gf_mul(data[position], power);
            else
                matrix[row][position - 55] = power;
            power = gf_mul(power, root);
        }
        matrix[row][8] = syndrome;
    }

    for (int column = 0; column < 8; ++column) {
        int pivot = column;
        while (pivot < 8 && matrix[pivot][column] == 0)
            ++pivot;
        if (pivot == 8)
            return false;
        if (pivot != column)
            for (int j = column; j <= 8; ++j)
                std::swap(matrix[pivot][j], matrix[column][j]);
        const int inverse = gf_inv(matrix[column][column]);
        for (int j = column; j <= 8; ++j)
            matrix[column][j] = gf_mul(matrix[column][j], inverse);
        for (int row = 0; row < 8; ++row) {
            if (row == column || matrix[row][column] == 0)
                continue;
            const int scale = matrix[row][column];
            for (int j = column; j <= 8; ++j)
                matrix[row][j] ^= gf_mul(scale, matrix[column][j]);
        }
    }
    for (int i = 0; i < 8; ++i)
        parity[i] = matrix[i][8];
    return true;
}

inline bool rs_encode_stream(const std::vector<uint8_t>& psdu,
                             std::vector<int8_t>& coded,
                             std::vector<int8_t>& bits)
{
    const size_t data_bits = psdu.size() * 8;
    if (data_bits == 0)
        return false;
    bits.resize(data_bits);
    for (size_t byte = 0; byte < psdu.size(); ++byte)
        for (size_t bit = 0; bit < 8; ++bit)
            bits[byte * 8 + bit] =
                static_cast<int8_t>((psdu[byte] >> bit) & 1u); // LSB-first

    const size_t blocks = (data_bits + 329) / 330;
    coded.clear();
    coded.reserve(data_bits + blocks * 48);
    size_t offset = 0;
    for (size_t block = 0; block < blocks; ++block) {
        const size_t count = std::min<size_t>(330, data_bits - offset);
        int8_t padded[330] = {};
        const size_t first = 330 - count;
        for (size_t i = 0; i < count; ++i)
            padded[first + i] = bits[offset + i];

        int data_symbols[55] = {};
        for (int symbol = 0; symbol < 55; ++symbol)
            for (int bit = 0; bit < 6; ++bit)
                data_symbols[symbol] =
                    (data_symbols[symbol] << 1) |
                    padded[symbol * 6 + bit];
        int parity[8] = {};
        if (!rs_parity(data_symbols, parity))
            return false;

        coded.insert(coded.end(), bits.begin() + offset,
                     bits.begin() + offset + count);
        for (int symbol = 0; symbol < 8; ++symbol)
            for (int bit = 5; bit >= 0; --bit)
                coded.push_back(
                    static_cast<int8_t>((parity[symbol] >> bit) & 1));
        offset += count;
    }
    return true;
}

inline void convolutional_encode(const std::vector<int8_t>& input,
                                 std::vector<int8_t>& g0,
                                 std::vector<int8_t>& g1)
{
    g0.resize(input.size());
    g1.resize(input.size());
    int state = 0;
    for (size_t i = 0; i < input.size(); ++i) {
        const int u = input[i] & 1;
        const int s1 = state >> 1;
        const int s0 = state & 1;
        g0[i] = static_cast<int8_t>(s1);
        g1[i] = static_cast<int8_t>(u ^ s0);
        state = (u << 1) | s1;
    }
}

inline void spreading(std::vector<int8_t>& chips,
                      std::vector<int8_t>& sequence,
                      size_t code_index,
                      size_t start,
                      size_t count)
{
    static const int8_t init9[15] =
        { 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1 };
    static const int8_t init10[15] =
        { 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1 };
    static const int8_t init11[15] =
        { 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1 };
    const int8_t* init = code_index == 11 ? init11 :
                         (code_index == 10 ? init10 : init9);
    sequence.resize(start + count);
    for (size_t i = 0; i < 15 && i < sequence.size(); ++i)
        sequence[i] = init[i];
    for (size_t i = 15; i < sequence.size(); ++i)
        sequence[i] = sequence[i - 14] ^ sequence[i - 15];
    chips.resize(count);
    for (size_t i = 0; i < count; ++i)
        chips[i] = sequence[start + i] ? int8_t(-1) : int8_t(1);
}

inline void emit_bprf(std::vector<float>& impulses,
                      size_t field_begin,
                      const std::vector<int8_t>& g0,
                      const std::vector<int8_t>& g1,
                      size_t first_symbol,
                      size_t symbol_count,
                      size_t chips_per_burst,
                      size_t chips_per_symbol,
                      size_t code_index,
                      size_t scrambler_offset,
                      std::vector<int8_t>& spread,
                      std::vector<int8_t>& sequence)
{
    spreading(spread, sequence, code_index, scrambler_offset,
              symbol_count * chips_per_burst);
    for (size_t symbol = 0; symbol < symbol_count; ++symbol) {
        const size_t spread_first = symbol * chips_per_burst;
        const size_t half = g0[first_symbol + symbol] ?
                                chips_per_symbol / 2 : 0;
        // IEEE BPRF burst-position hopping uses the first scrambling bit of
        // the burst: bit 1 (spread=-1) selects the second candidate burst.
        const size_t hop = spread[spread_first] < 0 ? chips_per_burst : 0;
        const float polarity = g1[first_symbol + symbol] ? -1.0f : 1.0f;
        for (size_t chip = 0; chip < chips_per_burst; ++chip) {
            const size_t pulse_symbol =
                symbol * chips_per_symbol + half + hop + chip;
            impulses[field_begin + 2 * pulse_symbol] =
                polarity * static_cast<float>(spread[spread_first + chip]);
        }
    }
}

} // namespace tx_detail

// Production first-version DW1000 reconstruction on the preprocessed
// 998.4 MHz grid. `phr_bits` are the 19 decoded SECDED bits retained by the
// demodulator; `psdu` includes its two FCS bytes. The complex CIR replaces
// the transmitter shaping pulse exactly as in apply_estimated_cir_to_uwb.m.
inline bool reconstruct_dw1000(const std::vector<uint8_t>& phr_bits,
                               const std::vector<uint8_t>& psdu,
                               bool fcs_pass,
                               const std::vector<std::complex<float>>& cir,
                               size_t cir_pre_samples,
                               TxReconstruction& out,
                               TxReconstructionScratch& scratch)
{
    out.status = ReconstructStatus::InvalidInput;
    out.pulse_impulses.clear();
    out.replica.clear();
    out.sync = {};
    out.sfd = {};
    out.phr = {};
    out.payload = {};
    if (!fcs_pass) {
        out.status = ReconstructStatus::FcsFailed;
        return false;
    }
    if (phr_bits.size() != 19 || psdu.size() < 2 || cir.empty() ||
        cir_pre_samples >= cir.size())
        return false;
    for (uint8_t bit : phr_bits)
        if (bit > 1)
            return false;
    size_t phr_psdu_length = 0;
    for (size_t bit = 2; bit < 9; ++bit)
        phr_psdu_length = (phr_psdu_length << 1) | phr_bits[bit];
    if (phr_psdu_length != psdu.size())
        return false;
    const uint16_t received_fcs = static_cast<uint16_t>(psdu[psdu.size() - 2]) |
                                  (static_cast<uint16_t>(psdu.back()) << 8);
    if (received_fcs != tx_detail::crc16_802154(psdu.data(), psdu.size() - 2)) {
        out.status = ReconstructStatus::FcsFailed;
        return false;
    }
    for (const auto& tap : cir)
        if (!std::isfinite(tap.real()) || !std::isfinite(tap.imag()))
            return false;

    if (!tx_detail::rs_encode_stream(psdu, scratch.rs_bits,
                                     scratch.psdu_bits)) {
        out.status = ReconstructStatus::RsEncodeFailed;
        return false;
    }
    scratch.encoder_input.clear();
    for (uint8_t bit : phr_bits)
        scratch.encoder_input.push_back(static_cast<int8_t>(bit & 1u));
    scratch.encoder_input.insert(scratch.encoder_input.end(),
                                 scratch.rs_bits.begin(), scratch.rs_bits.end());
    scratch.encoder_input.push_back(0);
    scratch.encoder_input.push_back(0);
    tx_detail::convolutional_encode(scratch.encoder_input, scratch.g0,
                                    scratch.g1);
    if (scratch.g0.size() < 21)
        return false;

    constexpr size_t sync_samples = 128 * demod::kQm35SamplesPerSymbol;
    constexpr size_t sfd_samples = 8 * demod::kQm35SamplesPerSymbol;
    constexpr size_t phr_samples = 21 * 512 * 2;
    const size_t payload_symbols = scratch.rs_bits.size();
    const size_t payload_samples = payload_symbols * 64 * 2;
    const size_t total = sync_samples + sfd_samples + phr_samples + payload_samples;
    out.pulse_impulses.assign(total, 0.0f);
    out.sync = { 0, sync_samples };
    out.sfd = { out.sync.end, out.sync.end + sfd_samples };
    out.phr = { out.sfd.end, out.sfd.end + phr_samples };
    out.payload = { out.phr.end, total };

    const int8_t* code = demod::GetPreambleCode(11);
    const auto emit_code = [&](size_t begin, int8_t scale) {
        for (size_t i = 0; i < demod::kQm35CodeLength; ++i)
            out.pulse_impulses[begin + i * 8] =
                static_cast<float>(scale * code[i]);
    };
    for (size_t repetition = 0; repetition < 128; ++repetition)
        emit_code(repetition * demod::kQm35SamplesPerSymbol, 1);
    const int8_t sfd[8] = { -1, -1, -1, -1, 1, -1, 0, 0 };
    for (size_t symbol = 0; symbol < 8; ++symbol)
        emit_code(out.sfd.begin + symbol * demod::kQm35SamplesPerSymbol,
                  sfd[symbol]);

    tx_detail::emit_bprf(out.pulse_impulses, out.phr.begin, scratch.g0,
                         scratch.g1, 0, 21, 64, 512, 11, 0,
                         scratch.spread, scratch.lfsr_sequence);
    tx_detail::emit_bprf(out.pulse_impulses, out.payload.begin, scratch.g0,
                         scratch.g1, 21, payload_symbols, 8, 64, 11, 21 * 64,
                         scratch.spread, scratch.lfsr_sequence);

    out.replica.assign(total, std::complex<float>(0.0f, 0.0f));
    for (size_t sample = 0; sample < total; ++sample) {
        const float impulse = out.pulse_impulses[sample];
        if (impulse == 0.0f)
            continue;
        for (size_t tap = 0; tap < cir.size(); ++tap) {
            const int64_t destination = static_cast<int64_t>(sample) +
                                        static_cast<int64_t>(tap) -
                                        static_cast<int64_t>(cir_pre_samples);
            if (destination >= 0 && static_cast<size_t>(destination) < total)
                out.replica[static_cast<size_t>(destination)] +=
                    impulse * cir[tap];
        }
    }
    out.status = ReconstructStatus::Success;
    return true;
}

} // namespace sic
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_UWB_TX_RECONSTRUCTOR_H */
