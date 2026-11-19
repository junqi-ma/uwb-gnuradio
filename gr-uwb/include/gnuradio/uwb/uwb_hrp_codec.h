// SPDX-License-Identifier: GPL-3.0-or-later
// Shared HRP encoding primitives. No receiver, SIMD, GR or UHD dependency.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace gr::uwb::codec {
// Bit 0 is the next chip; bit 14 is the newest bit of the 15-bit LFSR.
// Initial states are the IEEE HRP BPRF sequences for preamble codes 9–12.
inline uint16_t bprf_spreading_seed(size_t code_index)
{
    constexpr uint16_t initial[] = {
        0b101111001000010,
        0b111100001001100,
        0b110011001001111,
        0b001110011111001
    };
    return code_index >= 9 && code_index <= 12 ? initial[code_index - 9] : 0;
}

inline void generate_bprf_spreading(int8_t* out, size_t nbits,
                                    size_t start_bit, uint16_t state)
{
    auto next_bit = [&]() {
        const unsigned bit = state & 1u;
        const unsigned feedback = (state ^ (state >> 1)) & 1u;
        state = static_cast<uint16_t>((state >> 1) | (feedback << 14));
        return bit;
    };
    for (size_t i = 0; i < start_bit; ++i)
        (void)next_bit();
    for (size_t i = 0; i < nbits; ++i)
        out[i] = next_bit() ? int8_t(-1) : int8_t(1);
}

inline bool fill_bprf_spread(int8_t* out, size_t nbits, size_t code_index,
                             size_t start_bit)
{
    const uint16_t seed = bprf_spreading_seed(code_index);
    if (seed == 0)
        return false;
    generate_bprf_spreading(out, nbits, start_bit, seed);
    return true;
}

inline bool bprf_spreading(std::vector<int8_t>& out, size_t code_index,
                           size_t start_bit, size_t nbits)
{
    const uint16_t seed = bprf_spreading_seed(code_index);
    if (seed == 0)
        return false;
    out.resize(nbits);
    generate_bprf_spreading(out.data(), nbits, start_bit, seed);
    return true;
}

struct Cl3Transition {
    int g0;
    int g1;
    int next_state;
};

inline constexpr Cl3Transition cl3_transition(int state, int input)
{
    const int s1 = state >> 1;
    const int s0 = state & 1;
    return { s1, input ^ s0, (input << 1) | s1 };
}

inline void hrp_secded(const std::vector<int8_t>& sys13,
                       std::vector<int8_t>& phr19)
{
    static const uint16_t kParity[6] = {
        0b1110110100111, // p1
        0b0000000000011, // p2
        0b0000111111100, // p3
        0b0111000111100, // p4
        0b1011011001101, // p5
        0b1101101010110, // p6
    };
    phr19.resize(19);
    for (int i = 0; i < 13; ++i)
        phr19[i] = sys13[i];
    for (int r = 0; r < 6; ++r) {
        int p = 0;
        for (int i = 0; i < 13; ++i)
            if (kParity[r] & (1u << (12 - i)))
                p ^= sys13[i];
        phr19[13 + r] = static_cast<int8_t>(p);
    }
}

inline uint16_t crc16_802154(const uint8_t* data, size_t len)
{
    uint16_t crc = 0;
    const uint16_t poly = 0x8408;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1)
                crc = static_cast<uint16_t>((crc >> 1) ^ poly);
            else
                crc = static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

inline int rs_gf_mul(int a, int b)
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
            a ^= 0x21; // lower 6 bits of 0x61 (x^6 = x^5 + 1)
        b >>= 1;
    }
    return p & 0x3f;
}

inline int rs_gf_pow(int a, int n)
{
    int r = 1;
    a &= 0x3f;
    n %= 63;
    if (n < 0)
        n += 63;
    while (n) {
        if (n & 1)
            r = rs_gf_mul(r, a);
        a = rs_gf_mul(a, a);
        n >>= 1;
    }
    return r;
}

inline int rs_gf_inv(int a)
{
    return a ? rs_gf_pow(a, 62) : 0;
}

inline void rs_bits_to_syms(const int8_t* bits, int count, int* symbols)
{
    for (int i = 0; i < count; ++i) {
        int s = 0;
        for (int b = 0; b < 6; ++b)
            s = (s << 1) | (bits[i * 6 + b] & 1);
        symbols[i] = s;
    }
}

inline void rs_bits_to_syms(const int8_t* bits378, int* syms63)
{
    rs_bits_to_syms(bits378, 63, syms63);
}

inline void rs_syms_to_bits(const int* syms, int nsym, int8_t* bits)
{
    for (int i = 0; i < nsym; ++i) {
        for (int b = 0; b < 6; ++b)
            bits[i * 6 + b] =
                static_cast<int8_t>((syms[i] >> (5 - b)) & 1);
    }
}
} // namespace gr::uwb::codec
