// SPDX-License-Identifier: GPL-3.0-or-later
// Shared HRP encoding primitives. No receiver, SIMD, GR or UHD dependency.
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
namespace openuwb::codec {
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

// ---------------------------------------------------------------------------
// GF(2^6) arithmetic tables.
//
// Primitive polynomial x^6 + x^5 + 1 (0x61) with alpha = x = 2.  The tables
// are generated at compile time (`constexpr`) from the branchy bit-serial
// multiply, so every entry is bit-identical to the shift-and-XOR definition
// by construction: mul[a][b] == mul_ref(a, b) for all 64 x 64 (a, b).
// `powers_are_primitive()` proves that alpha generates GF(2^6)*, i.e. the
// 63-entry power table is a complete permutation of 1..63 (needed by the
// log/exp based rs_gf_pow()).
// ---------------------------------------------------------------------------
namespace gf64_detail {

// Reference bit-serial GF(2^6) multiply (kept as the table generator).
constexpr uint8_t mul_ref(uint8_t a, uint8_t b)
{
    uint8_t p = 0;
    a &= 0x3fu;
    b &= 0x3fu;
    for (int i = 0; i < 6; ++i) {
        if (b & 1u)
            p ^= a;
        const bool carry = (a & 0x20u) != 0;
        a = static_cast<uint8_t>((a << 1) & 0x3fu);
        if (carry)
            a ^= 0x21u; // lower 6 bits of 0x61 (x^6 = x^5 + 1)
        b = static_cast<uint8_t>(b >> 1);
    }
    return static_cast<uint8_t>(p & 0x3fu);
}

struct Tables {
    uint8_t mul[64][64];       // mul[a][b]
    uint8_t exp[63];           // exp[k] = alpha^k
    uint8_t log[64];           // log[alpha^k] = k; log[0] unused (0)
    uint8_t inv[64];           // inv[0] = 0 (matches pow(0, 62) == 0)
    uint8_t alpha_pow[63];     // alpha^i
    uint8_t alpha_inv_pow[63]; // alpha^{-j}
};

constexpr Tables make_tables()
{
    Tables t{};
    for (int a = 0; a < 64; ++a)
        for (int b = 0; b < 64; ++b)
            t.mul[a][b] = mul_ref(static_cast<uint8_t>(a), static_cast<uint8_t>(b));
    uint8_t x = 1;
    for (int k = 0; k < 63; ++k) {
        t.exp[k] = x;
        t.alpha_pow[k] = x;
        t.alpha_inv_pow[(63 - k) % 63] = x; // alpha^{-j} with j = (63 - k) % 63
        t.log[x] = static_cast<uint8_t>(k);
        x = mul_ref(x, 2);
    }
    for (int a = 1; a < 64; ++a)
        t.inv[a] = t.exp[(63 - t.log[a]) % 63];
    return t;
}

inline constexpr Tables kTables = make_tables();

constexpr bool powers_are_primitive()
{
    bool seen[64] = {};
    for (int k = 0; k < 63; ++k) {
        const uint8_t v = kTables.alpha_pow[k];
        if (v == 0 || seen[v])
            return false;
        seen[v] = true;
    }
    return true;
}
static_assert(powers_are_primitive(), "alpha = 2 must generate GF(2^6)*");

} // namespace gf64_detail

// GF(2^6) multiply (poly 0x61).  Bit-exact with the original bit-serial
// routine, including the `a &= 0x3f` / `b &= 0x3f` input masking.
inline int rs_gf_mul(int a, int b)
{
    return gf64_detail::kTables.mul[a & 0x3f][b & 0x3f];
}

// GF(2^6) power a^n (n may be negative via modular reduction mod 63).
inline int rs_gf_pow(int a, int n)
{
    a &= 0x3f;
    n %= 63;
    if (n < 0)
        n += 63;
    if (a == 0)
        return n == 0 ? 1 : 0;
    return gf64_detail::kTables.exp[(gf64_detail::kTables.log[a] * n) % 63];
}

inline int rs_gf_inv(int a)
{
    return gf64_detail::kTables.inv[a & 0x3f];
}

// Kernel fast paths for alpha^i / alpha^{-j} lookups (j, i reduced mod 63).
inline int rs_gf_alpha_pow(int i)
{
    i %= 63;
    if (i < 0)
        i += 63;
    return gf64_detail::kTables.alpha_pow[i];
}

inline int rs_gf_alpha_inv_pow(int j)
{
    j %= 63;
    if (j < 0)
        j += 63;
    return gf64_detail::kTables.alpha_inv_pow[j];
}

inline void rs_bits_to_syms(const int8_t* bits, int count, int* symbols)
{
    for (int i = 0; i < count; ++i) {
        const int8_t* b = bits + i * 6;
        symbols[i] = ((b[0] & 1) << 5) | ((b[1] & 1) << 4) |
                     ((b[2] & 1) << 3) | ((b[3] & 1) << 2) |
                     ((b[4] & 1) << 1) | (b[5] & 1);
    }
}

inline void rs_bits_to_syms(const int8_t* bits378, int* syms63)
{
    rs_bits_to_syms(bits378, 63, syms63);
}

inline void rs_syms_to_bits(const int* syms, int nsym, int8_t* bits)
{
    for (int i = 0; i < nsym; ++i) {
        const int s = syms[i];
        int8_t* b = bits + i * 6;
        b[0] = static_cast<int8_t>((s >> 5) & 1);
        b[1] = static_cast<int8_t>((s >> 4) & 1);
        b[2] = static_cast<int8_t>((s >> 3) & 1);
        b[3] = static_cast<int8_t>((s >> 2) & 1);
        b[4] = static_cast<int8_t>((s >> 1) & 1);
        b[5] = static_cast<int8_t>(s & 1);
    }
}
} // namespace openuwb::codec
