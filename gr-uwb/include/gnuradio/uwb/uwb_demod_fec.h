// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <gnuradio/uwb/uwb_demod_result.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <gnuradio/uwb/uwb_hrp_codec.h>

namespace gr::uwb::demod::core {
// ---------------------------------------------------------------------------
// R3 helpers: BPRF spreading LFSR, CL-3 Viterbi, and SECDED parity.
// All verified against the MATLAB golden reference (phr_ref2.mat / decode_uwb).
// ---------------------------------------------------------------------------
namespace detail {

// ---------------------------------------------------------------------------
// Payload sub-stage timing helpers (diagnostic only).
//
// `PayloadSubTimer` is an RAII scope guard over the whole payload stage: it
// snapshots the four named child buckets on construction and, on destruction,
// adds whatever is left of the enclosing interval to `other_us`.  The five
// buckets are therefore exactly additive whatever early return the enclosing
// function takes, and the guard costs two steady_clock reads per payload stage
// with no allocation and no change to any algorithm path.
//
// `PayloadSubBucket` attributes the elapsed time of a nested scope to one named
// bucket (e.g. the fused CMF plan + correlation).  Nested buckets are children
// of the same struct, so the outer guard's remainder automatically shrinks by
// the same amount: no double counting.
// ---------------------------------------------------------------------------
inline uint64_t payload_sub_total(const PayloadSubTimers& s)
{
    // Includes `other_us` so nested guards stay additive: an inner guard adds
    // its remainder to `other_us`, and the outer guard subtracts everything
    // already attributed (including that remainder) from its own interval.
    return s.soft_us + s.viterbi_us + s.rs_us + s.crc_us + s.other_us;
}

// Elapsed microseconds since `t0`, truncated exactly like the parent laps.
inline uint64_t payload_elapsed_us(std::chrono::steady_clock::time_point t0)
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0)
            .count());
}

class PayloadSubTimer
{
public:
    explicit PayloadSubTimer(PayloadSubTimers* s)
        : s_(s),
          t0_(s ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point()),
          base_(s ? payload_sub_total(*s) : 0)
    {
    }
    ~PayloadSubTimer()
    {
        if (!s_)
            return;
        const uint64_t total = payload_elapsed_us(t0_);
        const uint64_t spent = payload_sub_total(*s_) - base_;
        if (total > spent)
            s_->other_us += total - spent;
    }
    PayloadSubTimer(const PayloadSubTimer&) = delete;
    PayloadSubTimer& operator=(const PayloadSubTimer&) = delete;

private:
    PayloadSubTimers* s_;
    std::chrono::steady_clock::time_point t0_;
    uint64_t base_;
};

class PayloadSubBucket
{
public:
    PayloadSubBucket(PayloadSubTimers* s, uint64_t PayloadSubTimers::*member)
        : s_(s),
          member_(member),
          t0_(s ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point())
    {
    }
    ~PayloadSubBucket()
    {
        if (s_)
            s_->*member_ += payload_elapsed_us(t0_);
    }
    PayloadSubBucket(const PayloadSubBucket&) = delete;
    PayloadSubBucket& operator=(const PayloadSubBucket&) = delete;

private:
    PayloadSubTimers* s_;
    uint64_t PayloadSubTimers::*member_;
    std::chrono::steady_clock::time_point t0_;
};

// IEEE 802.15.4a BPRF data-scrambler spreading (lrwpan.internal.createScrambler):
// a 15-bit LFSR with s[i] = s[i-14] ^ s[i-15].  The initial state depends
// on the preamble code index; values below were exported from MATLAB R2025b
// lrwpan.internal.createScrambler for codes 9–12. This distinction is required
// for DW1000 PHR/payload data.
// Returns the +/-1 spreading stream over [start_bit, start_bit+nbits).
// The PHR field uses offset 0; the payload field is offset by the number of
// PHR active chips (21 symbols x 64 chips/burst = 1344).
using gr::uwb::codec::bprf_spreading;

// Rate-1/2 constraint-length-3 Viterbi decoder for poly2trellis(3,[2 5])
// (IEEE 802.15.4a PHR).  Hard decision, 'trunc' mode.  `rx` is the interleaved
// received code bits (length 2*N); output is the N decoded systematic bits.
inline void vitdec_cl3(const int8_t* rx, size_t N, std::vector<int8_t>& out)
{
    // octal 2 = 010 -> g0 = [0 1 0]; octal 5 = 101 -> g1 = [1 0 1].
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        const int s1 = st >> 1, s0 = st & 1;
        for (int u = 0; u < 2; ++u) {
            o0[st][u] = s1;
            o1[st][u] = u ^ s0;
            nxt[st][u] = (u << 1) | s1;
        }
    }
    const int INF = 1000000;
    int surf[4] = { 0, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        int ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const int d = (rx[2 * t] ^ o0[st][u]) + (rx[2 * t + 1] ^ o1[st][u]);
                const int m = surf[st] + d;
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

// SECDED encode: 13 systematic bits -> 19-bit codeword (13 data + 6 parity).
// Parity matrix rows from lrwpan.internal.hrpSECDED (verified against golden).
using gr::uwb::codec::hrp_secded;

// ---------------------------------------------------------------------------
// Shared BPM-BPSK BPRF demod kernel (used by both the PHR and payload fields).
// soft: chip-rate soft chips (0-based absolute).  start: field start chip.
// nsym: number of BPM-BPSK symbols.  cpb: chips per burst.  cps: chips per
// symbol (BPM half-symbol offset = cps/2).  scram_offset: LFSR start bit
// (PHR=0, payload=1344).  pol: sfd polarity.  Fills g0 (BPM position bit) and
// g1 (BPSK polarity bit), each nsym entries.  Mirrors helperUWBBPRFDemodKernel.
// ---------------------------------------------------------------------------
inline bool bprf_demod(const std::vector<float>& soft, int64_t start,
                       size_t code_index, size_t nsym, size_t cpb, size_t cps,
                       size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                       std::vector<int8_t>& g1)
{
    if (start < 0 ||
        static_cast<size_t>(start + static_cast<int64_t>(cps * nsym)) >
            soft.size())
        return false;
    std::vector<int8_t> spread;
    if (!bprf_spreading(spread, code_index, scram_offset, cpb * nsym))
        return false;
    g0.resize(nsym);
    g1.resize(nsym);
    const size_t third = cps / 2;
    for (size_t s = 0; s < nsym; ++s) {
        const size_t base = static_cast<size_t>(start) + s * cps;
        const int8_t* sp = &spread[s * cpb];
        float b0 = -1e30f, b1 = -1e30f, m0 = 0.0f, m1 = 0.0f;
        for (size_t hop = 0; hop < 2; ++hop) {
            const size_t f0 = hop * cpb;
            float metric0 = 0.0f, metric1 = 0.0f;
            for (size_t c = 0; c < cpb; ++c) {
                metric0 += pol * soft[base + f0 + c] * static_cast<float>(sp[c]);
                metric1 += pol * soft[base + third + f0 + c] *
                           static_cast<float>(sp[c]);
            }
            if (std::abs(metric0) > b0) {
                b0 = std::abs(metric0);
                m0 = metric0;
            }
            if (std::abs(metric1) > b1) {
                b1 = std::abs(metric1);
                m1 = metric1;
            }
        }
        float best;
        if (b0 >= b1) {
            g0[s] = 0;
            best = m0;
        } else {
            g0[s] = 1;
            best = m1;
        }
        g1[s] = (best < 0.0f) ? 1 : 0;
    }
    return true;
}

// Soft-output variant of bprf_demod.  llr_g0 / llr_g1 > 0 favour bit value 0.
//   llr_g0 = |metric_half0| - |metric_half1|         (BPM position)
//   llr_g1 = max_c(+metric_c) - max_c(-metric_c)     (BPSK polarity; + => bit 0)
// Hard decisions still written to g0/g1 for callers that need them.
inline bool bprf_demod_soft(const std::vector<float>& soft, int64_t start,
                            size_t code_index, size_t nsym, size_t cpb, size_t cps,
                            size_t scram_offset, int8_t pol,
                            std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                            std::vector<float>& llr_g0, std::vector<float>& llr_g1)
{
    if (start < 0 ||
        static_cast<size_t>(start + static_cast<int64_t>(cps * nsym)) >
            soft.size())
        return false;
    std::vector<int8_t> spread;
    if (!bprf_spreading(spread, code_index, scram_offset, cpb * nsym))
        return false;
    g0.resize(nsym);
    g1.resize(nsym);
    llr_g0.resize(nsym);
    llr_g1.resize(nsym);
    const size_t third = cps / 2;
    for (size_t s = 0; s < nsym; ++s) {
        const size_t base = static_cast<size_t>(start) + s * cps;
        const int8_t* sp = &spread[s * cpb];
        float b0 = -1e30f, b1 = -1e30f, m0 = 0.0f, m1 = 0.0f;
        float mc[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        for (size_t hop = 0; hop < 2; ++hop) {
            const size_t f0 = hop * cpb;
            float metric0 = 0.0f, metric1 = 0.0f;
            for (size_t c = 0; c < cpb; ++c) {
                metric0 += pol * soft[base + f0 + c] * static_cast<float>(sp[c]);
                metric1 += pol * soft[base + third + f0 + c] *
                           static_cast<float>(sp[c]);
            }
            mc[2 * hop + 0] = metric0;
            mc[2 * hop + 1] = metric1;
            if (std::abs(metric0) > b0) {
                b0 = std::abs(metric0);
                m0 = metric0;
            }
            if (std::abs(metric1) > b1) {
                b1 = std::abs(metric1);
                m1 = metric1;
            }
        }
        float best;
        if (b0 >= b1) {
            g0[s] = 0;
            best = m0;
        } else {
            g0[s] = 1;
            best = m1;
        }
        const float p_pos =
            std::max(std::max(mc[0], mc[1]), std::max(mc[2], mc[3]));
        const float p_neg =
            std::max(std::max(-mc[0], -mc[1]), std::max(-mc[2], -mc[3]));
        g1[s] = (best < 0.0f) ? 1 : 0;
        llr_g0[s] = b0 - b1;        // >0 prefers g0=0
        llr_g1[s] = p_pos - p_neg;  // >0 prefers g1=0
    }
    return true;
}

// Soft-decision rate-1/2 CL-3 Viterbi (same trellis as vitdec_cl3).
// llr0/llr1: per-symbol LLRs for the two coded bits (positive favours bit 0).
inline void vitdec_cl3_soft(const float* llr0, const float* llr1, size_t N,
                            std::vector<int8_t>& out)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    for (int st = 0; st < 4; ++st) {
        const int s1 = st >> 1, s0 = st & 1;
        for (int u = 0; u < 2; ++u) {
            o0[st][u] = s1;
            o1[st][u] = u ^ s0;
            nxt[st][u] = (u << 1) | s1;
        }
    }
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                // cost = (2*o-1)*llr : o=0 gets -llr, o=1 gets +llr
                const float c0 =
                    (2.0f * static_cast<float>(o0[st][u]) - 1.0f) * llr0[t];
                const float c1 =
                    (2.0f * static_cast<float>(o1[st][u]) - 1.0f) * llr1[t];
                const float m = surf[st] + c0 + c1;
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int i = 0; i < 4; ++i)
            surf[i] = ns[i];
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    out.resize(N);
    int cur = best;
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        out[t] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

// IEEE 802.15.4 CRC-16 (reflected polynomial 0x8408, init 0).  FCS algorithm
// used by DW1000 / QM35 frames (matches uwbdecoder.ieee802154CRC16).
using gr::uwb::codec::crc16_802154;

// ---------------------------------------------------------------------------
// IEEE 802.15.4a RS(63,55) over GF(2^6) — matches lrwpan.internal.hrpRS.
// Field primitive poly: x^6 + x^5 + 1 (0x61).  Generator roots alpha^1..alpha^8
// with alpha = 2 (the element x).  Symbols are 6-bit, packed MSB-first.
// Systematic codeword layout (low-first poly): [55 data][8 parity] symbols =
// 330 data bits + 48 parity bits = 378 bits.  Partial blocks are formed by
// LEADING zero-padding the data field to 330 bits (verified on golden vectors).
// ---------------------------------------------------------------------------

// GF(2^6) multiply (poly 0x61).
using gr::uwb::codec::rs_gf_mul;

// GF(2^6) power a^n (n may be negative via modular reduction mod 63).
using gr::uwb::codec::rs_gf_pow;

using gr::uwb::codec::rs_gf_inv;

// Pack 378 MSB-first bits -> 63 GF(2^6) symbols.
using gr::uwb::codec::rs_bits_to_syms;

// Unpack nsym symbols -> 6*nsym MSB-first bits.
using gr::uwb::codec::rs_syms_to_bits;

// Syndromes S_i = r(alpha^i), i=1..8.  Low-first: r(x) = sum_j r[j] x^j.
inline void rs_syndromes(const int* r63, int* S8)
{
    for (int i = 1; i <= 8; ++i) {
        const int ai = rs_gf_pow(2, i);
        int s = 0;
        int ap = 1;
        for (int j = 0; j < 63; ++j) {
            s ^= rs_gf_mul(r63[j], ap);
            ap = rs_gf_mul(ap, ai);
        }
        S8[i - 1] = s;
    }
}

// Berlekamp-Massey over GF(2^6) on `count` syndromes S[0..count-1].
// Returns L; Lambda[0..L] with Lambda[0]=1.
inline int rs_berlekamp_massey_len(const int* S, int count, int* Lambda /* len >= 9 */)
{
    int C[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int B[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    int L = 0;
    int m = 1;
    int b = 1;
    for (int n = 0; n < count; ++n) {
        int delta = S[n];
        for (int i = 1; i <= L; ++i)
            delta ^= rs_gf_mul(C[i], S[n - i]);
        if (delta == 0) {
            ++m;
        } else {
            int T[9];
            for (int i = 0; i < 9; ++i)
                T[i] = C[i];
            const int scale = rs_gf_mul(delta, rs_gf_inv(b));
            for (int i = 0; i < 9 - m; ++i)
                C[i + m] ^= rs_gf_mul(scale, B[i]);
            if (2 * L <= n) {
                L = n + 1 - L;
                for (int i = 0; i < 9; ++i)
                    B[i] = T[i];
                b = delta;
                m = 1;
            } else {
                ++m;
            }
        }
    }
    for (int i = 0; i < 9; ++i)
        Lambda[i] = C[i];
    return L;
}

// Berlekamp-Massey on S1..S8.  Returns L; Lambda[0..L] with Lambda[0]=1.
inline int rs_berlekamp_massey(const int* S8, int* Lambda /* len >= 9 */)
{
    return rs_berlekamp_massey_len(S8, 8, Lambda);
}

// Chien search: roots of Lambda at alpha^{-j} => error at position j.
// Returns number of roots (may exceed L if uncorrectable).
inline int rs_chien_search(const int* Lambda, int L, int* err_pos /* <=4 */)
{
    int nerr = 0;
    for (int j = 0; j < 63; ++j) {
        const int xinv = rs_gf_pow(2, (63 - j) % 63); // alpha^{-j}
        int v = 0;
        int xp = 1;
        for (int i = 0; i <= L; ++i) {
            v ^= rs_gf_mul(Lambda[i], xp);
            xp = rs_gf_mul(xp, xinv);
        }
        if (v == 0) {
            if (nerr < 4)
                err_pos[nerr] = j;
            ++nerr;
        }
    }
    return nerr;
}

// Forney (b=1, roots alpha^1..): e_j = Omega(X^{-1}) / Lambda'(X^{-1}).
inline void rs_forney(const int* S8, const int* Lambda, int L,
                      const int* err_pos, int nerr, int* err_val)
{
    int Omega[8] = {};
    for (int i = 0; i < 8; ++i) {
        int o = 0;
        for (int j = 0; j <= L && j <= i; ++j)
            o ^= rs_gf_mul(Lambda[j], S8[i - j]);
        Omega[i] = o;
    }
    for (int k = 0; k < nerr; ++k) {
        const int j = err_pos[k];
        const int Xinv = rs_gf_pow(2, (63 - j) % 63);
        int num = 0;
        int xp = 1;
        for (int i = 0; i < 8; ++i) {
            num ^= rs_gf_mul(Omega[i], xp);
            xp = rs_gf_mul(xp, Xinv);
        }
        // Lambda' in char 2: only odd powers, Lambda_i * x^{i-1}
        int den = 0;
        xp = 1; // Xinv^0 for i=1
        for (int i = 1; i <= L; i += 2) {
            den ^= rs_gf_mul(Lambda[i], xp);
            xp = rs_gf_mul(xp, rs_gf_mul(Xinv, Xinv));
        }
        err_val[k] = den ? rs_gf_mul(num, rs_gf_inv(den)) : 0;
    }
}

// Decode one RS(63,55) block: 378 coded bits -> 330 data bits (MSB-first).
// Returns false if the block is uncorrectable (more than t=4 symbol errors).
inline bool rs_decode_block(const int8_t* coded378, int8_t* data330)
{
    int r[63];
    rs_bits_to_syms(coded378, r);

    int S[8];
    rs_syndromes(r, S);
    bool all0 = true;
    for (int i = 0; i < 8; ++i)
        if (S[i])
            all0 = false;

    if (!all0) {
        int Lambda[9] = {};
        const int L = rs_berlekamp_massey(S, Lambda);
        if (L <= 0 || L > 4)
            return false;
        int err_pos[4] = {};
        const int nerr = rs_chien_search(Lambda, L, err_pos);
        if (nerr != L)
            return false;
        int err_val[4] = {};
        rs_forney(S, Lambda, L, err_pos, nerr, err_val);
        for (int k = 0; k < nerr; ++k)
            r[err_pos[k]] ^= err_val[k];
        // Verify correction
        rs_syndromes(r, S);
        for (int i = 0; i < 8; ++i)
            if (S[i])
                return false;
    }

    rs_syms_to_bits(r, 55, data330);
    return true;
}

// Full-stream RS decoder for a PSDU of `data_bits` bits (e.g. 8*L).
// Splits the coded bit stream into num_blocks = ceil(data_bits/330) blocks,
// each carrying min(330, remaining) data bits + 48 parity bits.  Partial
// blocks are leading-zero-padded to 330 data bits before decode.
inline bool rs_decode_stream(const std::vector<int8_t>& coded,
                             size_t data_bits,
                             std::vector<int8_t>& data_out)
{
    data_out.clear();
    data_out.reserve(data_bits);
    if (data_bits == 0)
        return true;

    const size_t num_blocks = (data_bits + 329) / 330;
    size_t coded_off = 0;
    size_t data_off = 0;
    for (size_t b = 0; b < num_blocks; ++b) {
        const size_t this_data =
            std::min<size_t>(330, data_bits - data_off);
        const size_t this_coded = this_data + 48;
        if (coded_off + this_coded > coded.size())
            return false;

        int8_t block378[378];
        for (int i = 0; i < 378; ++i)
            block378[i] = 0;
        // Leading-zero pad: real data occupies the last this_data bits of
        // the 330-bit systematic data field (MATLAB hrpRS convention).
        const size_t data_start = 330 - this_data;
        for (size_t i = 0; i < this_data; ++i)
            block378[data_start + i] =
                static_cast<int8_t>(coded[coded_off + i] & 1);
        for (size_t i = 0; i < 48; ++i)
            block378[330 + i] = static_cast<int8_t>(
                coded[coded_off + this_data + i] & 1);

        int8_t data330[330];
        if (!rs_decode_block(block378, data330))
            return false;

        for (size_t i = 0; i < this_data; ++i)
            data_out.push_back(data330[data_start + i]);

        coded_off += this_coded;
        data_off += this_data;
    }
    return data_out.size() == data_bits;
}

// Shared RS + PSDU assembly + FCS tail of the payload decoder.  Both the
// hard-decision and the soft-decision front ends hand their RS codeword stream
// here, so the RS(63,55) decode, the LSB-first bit packing, the IEEE 802.15.4
// CRC-16 and the FCS comparison exist exactly once.  `out` is reset first;
// `out.ok` is set only when the RS decode succeeded, and `out.fcs_pass` then
// reports the CRC verdict (a false FCS is a decoded-but-rejected frame, not a
// decoder failure).
inline bool finish_rs_crc_payload(const std::vector<int8_t>& rs_cw,
                                  size_t psdu_bits,
                                  PayloadResult& out,
                                  PayloadSubTimers* sub = nullptr)
{
    out = PayloadResult{};
    std::vector<int8_t> psdu_bits_out;
    const auto t_rs = std::chrono::steady_clock::now();
    const bool rs_ok = rs_decode_stream(rs_cw, psdu_bits, psdu_bits_out);
    if (sub)
        sub->rs_us += payload_elapsed_us(t_rs);
    if (!rs_ok)
        return false;

    // PSDU assembly + FCS: bit packing, CRC-16 and the comparison.
    const auto t_crc = std::chrono::steady_clock::now();
    out.bits.assign(psdu_bits_out.begin(), psdu_bits_out.end());
    out.bytes.resize(psdu_bits_out.size() / 8);
    for (size_t i = 0; i < out.bytes.size(); ++i) {
        uint8_t b = 0;
        for (int bit = 0; bit < 8; ++bit)
            b |= static_cast<uint8_t>(psdu_bits_out[8 * i + bit]) << bit;
        out.bytes[i] = b;
    }
    if (out.bytes.size() >= 2) {
        out.calculated_fcs =
            crc16_802154(out.bytes.data(), out.bytes.size() - 2);
        out.received_fcs = static_cast<uint16_t>(
                               out.bytes[out.bytes.size() - 2]) |
                           (static_cast<uint16_t>(
                                out.bytes[out.bytes.size() - 1])
                            << 8);
        out.fcs_pass = (out.received_fcs == out.calculated_fcs);
    }
    if (sub)
        sub->crc_us += payload_elapsed_us(t_crc);
    out.ok = true;
    return true;
}

// Shared hard-decision payload backend: concatenate the 21 PHR symbols and the
// payload symbols, run the truncated CL-3 Viterbi, remove the 19 systematic PHR
// bits and the two tail bits, RS-decode, pack LSB-first and compute the FCS.
// This is the exact first attempt of stage_payload_fcs(); the fused payload
// path calls it with g0/g1 produced by rxfusion, so both paths share the same
// FEC/FCS implementation and length conventions.
inline bool decode_payload_hard(const std::vector<int8_t>& g0p,
                                const std::vector<int8_t>& g1p,
                                const std::vector<int8_t>& g0,
                                const std::vector<int8_t>& g1,
                                size_t psdu_bits,
                                PayloadResult& out,
                                PayloadSubTimers* sub = nullptr)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (g0p.size() != 21 || g1p.size() != 21 || g0.empty() ||
        g0.size() != g1.size())
        return false;

    const size_t total = 21 + g0.size();
    std::vector<int8_t> rx_bits(2 * total);
    for (size_t s = 0; s < 21; ++s) {
        rx_bits[2 * s] = g0p[s];
        rx_bits[2 * s + 1] = g1p[s];
    }
    for (size_t s = 0; s < g0.size(); ++s) {
        rx_bits[2 * (s + 21)] = g0[s];
        rx_bits[2 * (s + 21) + 1] = g1[s];
    }
    std::vector<int8_t> decoded;
    const auto t_vit = std::chrono::steady_clock::now();
    vitdec_cl3(rx_bits.data(), total, decoded);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (decoded.size() < total)
        return false;

    std::vector<int8_t> rs_cw(decoded.begin() + 19,
                              decoded.begin() + (total - 2));
    if (rs_cw.size() != g0.size())
        return false;
    return finish_rs_crc_payload(rs_cw, psdu_bits, out, sub);
}

struct Cl3TrellisTrace {
    std::vector<int8_t> bits;
    std::vector<int8_t> states;
    std::vector<float> margins;
    std::vector<uint8_t> lose_u;
};

inline void cl3_trellis_tables(int o0[4][2], int o1[4][2], int nxt[4][2])
{
    for (int st = 0; st < 4; ++st) {
        const int s1 = st >> 1, s0 = st & 1;
        for (int u = 0; u < 2; ++u) {
            o0[st][u] = s1;
            o1[st][u] = u ^ s0;
            nxt[st][u] = (u << 1) | s1;
        }
    }
}

// Soft CL-3 Viterbi recording per-step path metric margins and competing edges
// (invoked only when single-pass soft decode + RS/CRC fails).
inline void vitdec_cl3_soft_trace(const float* llr0,
                                  const float* llr1,
                                  size_t N,
                                  Cl3TrellisTrace& tr)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    cl3_trellis_tables(o0, o1, nxt);
    const float INF = 1e30f;
    float surf[4] = { 0.0f, INF, INF, INF };
    std::vector<int8_t> back(N * 4);
    std::vector<float> diff(N * 4, INF);
    std::vector<uint8_t> alt(N * 4, 0);
    for (size_t t = 0; t < N; ++t) {
        float ns[4] = { INF, INF, INF, INF };
        float ns2[4] = { INF, INF, INF, INF };
        uint8_t nalt[4] = { 0, 0, 0, 0 };
        for (int st = 0; st < 4; ++st) {
            for (int u = 0; u < 2; ++u) {
                const float c0 =
                    (2.0f * static_cast<float>(o0[st][u]) - 1.0f) * llr0[t];
                const float c1 =
                    (2.0f * static_cast<float>(o1[st][u]) - 1.0f) * llr1[t];
                const float m = surf[st] + c0 + c1;
                const int nst = nxt[st][u];
                const uint8_t code = static_cast<uint8_t>((u << 2) | st);
                if (m < ns[nst]) {
                    ns2[nst] = ns[nst];
                    nalt[nst] = static_cast<uint8_t>(back[t * 4 + nst]);
                    ns[nst] = m;
                    back[t * 4 + nst] = static_cast<int8_t>(code);
                } else if (m < ns2[nst]) {
                    ns2[nst] = m;
                    nalt[nst] = code;
                }
            }
        }
        for (int i = 0; i < 4; ++i) {
            surf[i] = ns[i];
            diff[t * 4 + i] = ns2[i] - ns[i];
            alt[t * 4 + i] = nalt[i];
        }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (surf[i] < surf[best])
            best = i;
    tr.bits.resize(N);
    tr.states.resize(N + 1);
    tr.margins.resize(N);
    tr.lose_u.resize(N);
    int cur = best;
    tr.states[N] = static_cast<int8_t>(cur);
    for (size_t t = N; t-- > 0;) {
        const int v = back[t * 4 + cur];
        tr.bits[t] = static_cast<int8_t>(v >> 2);
        tr.margins[t] = diff[t * 4 + cur];
        tr.lose_u[t] = static_cast<uint8_t>(alt[t * 4 + cur] >> 2);
        cur = v & 3;
        tr.states[t] = static_cast<int8_t>(cur);
    }
}

// Re-solve a local CL-3 trellis window [lo, hi) between fixed endpoint states
// while forcing the loser input bit at one or two target steps.
inline void local_cl3_trellis_flip(const float* llr0,
                                   const float* llr1,
                                   size_t lo,
                                   size_t hi,
                                   int start_st,
                                   int end_st,
                                   const size_t* force_t,
                                   const uint8_t* force_u,
                                   size_t n_force,
                                   std::vector<int8_t>& out_bits)
{
    int o0[4][2], o1[4][2], nxt[4][2];
    cl3_trellis_tables(o0, o1, nxt);
    const size_t len = hi - lo;
    const float INF = 1e30f;
    float surf[4] = { INF, INF, INF, INF };
    surf[start_st] = 0.0f;
    std::vector<int8_t> back(len * 4, 0);
    for (size_t i = 0; i < len; ++i) {
        const size_t t = lo + i;
        int forced = -1;
        for (size_t f = 0; f < n_force; ++f) {
            if (force_t[f] == t) {
                forced = static_cast<int>(force_u[f]);
                break;
            }
        }
        float ns[4] = { INF, INF, INF, INF };
        for (int st = 0; st < 4; ++st) {
            if (surf[st] > 1e20f)
                continue;
            for (int u = 0; u < 2; ++u) {
                if (forced >= 0 && u != forced)
                    continue;
                const float c0 =
                    (2.0f * static_cast<float>(o0[st][u]) - 1.0f) * llr0[t];
                const float c1 =
                    (2.0f * static_cast<float>(o1[st][u]) - 1.0f) * llr1[t];
                const float m = surf[st] + c0 + c1;
                const int nst = nxt[st][u];
                if (m < ns[nst]) {
                    ns[nst] = m;
                    back[i * 4 + nst] = static_cast<int8_t>((u << 2) | st);
                }
            }
        }
        for (int s = 0; s < 4; ++s)
            surf[s] = ns[s];
    }
    int cur = end_st;
    if (surf[cur] > 1e20f) {
        for (int s = 0; s < 4; ++s)
            if (surf[s] < surf[cur])
                cur = s;
    }
    for (size_t i = len; i-- > 0;) {
        const int v = back[i * 4 + cur];
        out_bits[lo + i] = static_cast<int8_t>(v >> 2);
        cur = v & 3;
    }
}

// RS(63,55) Errors-and-Erasures decoder (`v` erasures + `e` unknown errors,
// 2*e + v <= 8, capped at v <= 6 so at least 2 residual parity checks remain).
inline bool rs_decode_block_erasures(const int8_t* coded378,
                                     const int* eras_pos,
                                     int v,
                                     int8_t* data330)
{
    if (v <= 0 || v > 6)
        return rs_decode_block(coded378, data330);
    int r[63];
    rs_bits_to_syms(coded378, r);
    int S[8];
    rs_syndromes(r, S);
    bool all0 = true;
    for (int i = 0; i < 8; ++i)
        if (S[i])
            all0 = false;
    if (all0) {
        rs_syms_to_bits(r, 55, data330);
        return true;
    }

    int Gamma[9] = { 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int m = 0; m < v; ++m) {
        const int a = rs_gf_pow(2, eras_pos[m]);
        for (int d = m + 1; d >= 1; --d)
            Gamma[d] ^= rs_gf_mul(Gamma[d - 1], a);
    }
    const int rem = 8 - v;
    int T[8] = {};
    for (int k = 0; k < rem; ++k) {
        int val = 0;
        for (int j = 0; j <= v; ++j)
            val ^= rs_gf_mul(Gamma[j], S[v + k - j]);
        T[k] = val;
    }
    int C[9] = {};
    const int Le = rs_berlekamp_massey_len(T, rem, C);
    if (2 * Le + v > 8)
        return false;
    int err_pos[4] = {};
    int nerr = 0;
    if (Le > 0) {
        nerr = rs_chien_search(C, Le, err_pos);
        if (nerr != Le)
            return false;
    }
    int Psi[9] = {};
    for (int i = 0; i <= v; ++i)
        for (int j = 0; j <= Le; ++j)
            if (i + j <= 8)
                Psi[i + j] ^= rs_gf_mul(Gamma[i], C[j]);
    int all_pos[8] = {};
    int all_val[8] = {};
    int n_tot = 0;
    for (int i = 0; i < v; ++i)
        all_pos[n_tot++] = eras_pos[i];
    for (int i = 0; i < nerr; ++i) {
        for (int j = 0; j < v; ++j)
            if (err_pos[i] == eras_pos[j])
                return false;
        all_pos[n_tot++] = err_pos[i];
    }
    rs_forney(S, Psi, Le + v, all_pos, n_tot, all_val);
    for (int k = 0; k < n_tot; ++k)
        r[all_pos[k]] ^= all_val[k];
    int S_chk[8];
    rs_syndromes(r, S_chk);
    for (int i = 0; i < 8; ++i)
        if (S_chk[i])
            return false;
    rs_syms_to_bits(r, 55, data330);
    return true;
}

// Failure-path-only per-RS-block rescue.  Two phases, ordered so the cheapest
// candidate generator runs first:
//
//   Phase 1 (zero Viterbi cost): every block whose trellis-decoded codeword
//   fails RS gets up to three RS(63,55) errors-and-erasures attempts (v = 2, 4,
//   6) built from the six least-reliable RS symbols of that block.  If every
//   block produced at least one candidate, one combined CRC-16 check decides
//   the packet immediately -- the 21 local Trellis-Chase re-decodes are then
//   never run.
//
//   Phase 2 (only if phase 1 could not close the frame): 6 single and 15 pair
//   local trellis flips on the weakest merges of each still-failing block.
//
// Only a full-frame CRC-16 match is accepted.  Never invoked when standard soft
// Viterbi + RS/CRC already passes.
//
// `tr` is the caller's single soft-Viterbi trace: passing it in removes the
// second full-length Viterbi pass this function used to run itself.
inline bool rescue_payload_soft(const float* llr0,
                                const float* llr1,
                                size_t total,
                                size_t psdu_bits,
                                const Cl3TrellisTrace& tr,
                                PayloadResult& out,
                                PayloadSubTimers* sub = nullptr)
{
    if (psdu_bits == 0 || (psdu_bits % 8) != 0)
        return false;
    if (tr.bits.size() < total)
        return false;
    const size_t num_blocks = (psdu_bits + 329) / 330;
    struct BlockCands {
        std::vector<std::array<int8_t, 330>> cands;
    };
    std::vector<BlockCands> bc(num_blocks);
    // Per-block geometry, hoisted so both phases can extract a codeword.
    std::vector<size_t> blk_data_start(num_blocks, 0);
    std::vector<size_t> blk_coded_start(num_blocks, 0);
    std::vector<size_t> blk_this_data(num_blocks, 0);
    std::vector<size_t> blk_this_coded(num_blocks, 0);
    std::vector<uint8_t> need_flips(num_blocks, 0);
    {
        size_t coded_off = 0;
        size_t data_off = 0;
        for (size_t b = 0; b < num_blocks; ++b) {
            const size_t this_data = std::min<size_t>(330, psdu_bits - data_off);
            const size_t this_coded = this_data + 48;
            if (19 + coded_off + this_coded > total)
                return false;
            blk_data_start[b] = 330 - this_data;
            blk_coded_start[b] = 19 + coded_off;
            blk_this_data[b] = this_data;
            blk_this_coded[b] = this_coded;
            coded_off += this_coded;
            data_off += this_data;
        }
    }

    auto extract_block = [&](size_t b, const std::vector<int8_t>& dec,
                             int8_t* block378) {
        std::fill(block378, block378 + 378, 0);
        const size_t data_start = blk_data_start[b];
        const size_t this_data = blk_this_data[b];
        const size_t coded_start = blk_coded_start[b];
        for (size_t i = 0; i < this_data; ++i)
            block378[data_start + i] = dec[coded_start + i] & 1;
        for (size_t i = 0; i < 48; ++i)
            block378[330 + i] = dec[coded_start + this_data + i] & 1;
    };
    auto try_append = [&](size_t b, const std::array<int8_t, 330>& cand) {
        for (const auto& ex : bc[b].cands)
            if (ex == cand)
                return;
        bc[b].cands.push_back(cand);
    };

    // ---- Phase 1: RS only (no Trellis-Chase) ------------------------------
    for (size_t b = 0; b < num_blocks; ++b) {
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        extract_block(b, tr.bits, blk);
        if (rs_decode_block(blk, d330.data())) {
            bc[b].cands.push_back(d330);
            continue;
        }
        need_flips[b] = 1;
        // Rank the RS symbols (6 bits each) by their least-reliable bit and
        // erase the worst 2/4/6.  `data_start`-relative bits that fall in the
        // 48 parity bits map to the parity region of the codeword.
        const size_t data_start = blk_data_start[b];
        const size_t this_data = blk_this_data[b];
        const size_t t_b = blk_coded_start[b];
        struct SymRel {
            int sym;
            float min_m;
        };
        std::vector<SymRel> srel;
        const int first_sym = static_cast<int>(data_start / 6);
        for (int sy = first_sym; sy < 63; ++sy) {
            float mm = 1e30f;
            for (int bit = 0; bit < 6; ++bit) {
                const size_t pos = static_cast<size_t>(sy * 6 + bit);
                size_t vt = 0;
                if (pos >= data_start && pos < 330)
                    vt = t_b + (pos - data_start);
                else if (pos >= 330)
                    vt = t_b + this_data + (pos - 330);
                else
                    continue;
                mm = std::min(mm, tr.margins[vt]);
            }
            srel.push_back({ sy, mm });
        }
        std::sort(srel.begin(), srel.end(),
                  [](const SymRel& a, const SymRel& c) {
                      return a.min_m < c.min_m;
                  });
        for (int v : { 2, 4, 6 }) {
            if (static_cast<int>(srel.size()) < v)
                break;
            int ep[6];
            for (int k = 0; k < v; ++k)
                ep[k] = srel[k].sym;
            if (rs_decode_block_erasures(blk, ep, v, d330.data())) {
                try_append(b, d330);
                break;
            }
        }
    }

    std::vector<size_t> idx(num_blocks, 0);
    std::vector<int8_t> psdu_out(psdu_bits);
    std::vector<uint8_t> bytes(psdu_bits / 8);
    // Combined-candidate CRC search.  Returns true and fills `out` on a match.
    auto combine_and_crc = [&](size_t max_iter) -> bool {
        std::fill(idx.begin(), idx.end(), 0);
        size_t iter = 0;
        while (iter < max_iter) {
            size_t doff = 0;
            for (size_t b = 0; b < num_blocks; ++b) {
                const size_t this_data = blk_this_data[b];
                const size_t data_start = blk_data_start[b];
                const auto& c = bc[b].cands[idx[b]];
                for (size_t i = 0; i < this_data; ++i)
                    psdu_out[doff + i] = c[data_start + i];
                doff += this_data;
            }
            for (size_t i = 0; i < bytes.size(); ++i) {
                uint8_t byte = 0;
                for (int bit = 0; bit < 8; ++bit)
                    byte |= static_cast<uint8_t>(psdu_out[8 * i + bit]) << bit;
                bytes[i] = byte;
            }
            if (bytes.size() >= 2) {
                const uint16_t calc =
                    crc16_802154(bytes.data(), bytes.size() - 2);
                const uint16_t rx_fcs =
                    static_cast<uint16_t>(bytes[bytes.size() - 2]) |
                    (static_cast<uint16_t>(bytes[bytes.size() - 1]) << 8);
                if (calc == rx_fcs) {
                    out = PayloadResult{};
                    out.ok = true;
                    out.fcs_pass = true;
                    out.bytes = bytes;
                    out.bits.assign(psdu_out.begin(), psdu_out.end());
                    out.calculated_fcs = calc;
                    out.received_fcs = rx_fcs;
                    return true;
                }
            }
            ++iter;
            size_t pos = num_blocks;
            while (pos-- > 0) {
                if (++idx[pos] < bc[pos].cands.size())
                    break;
                idx[pos] = 0;
            }
            if (pos == static_cast<size_t>(-1))
                break;
        }
        return false;
    };

    bool all_have_cands = true;
    for (size_t b = 0; b < num_blocks; ++b)
        if (bc[b].cands.empty())
            all_have_cands = false;
    if (all_have_cands && combine_and_crc(64))
        return true;

    // ---- Phase 2: local Trellis-Chase on the still-failing blocks ---------
    std::vector<int8_t> work_bits;
    for (size_t b = 0; b < num_blocks; ++b) {
        if (!need_flips[b])
            continue;
        int8_t blk[378];
        std::array<int8_t, 330> d330{};
        const size_t t_b = blk_coded_start[b];
        const size_t t_e = t_b + blk_this_coded[b];
        struct StepM {
            size_t t;
            float m;
        };
        std::vector<StepM> sm;
        sm.reserve(blk_this_coded[b]);
        for (size_t t = t_b; t < t_e; ++t)
            sm.push_back({ t, tr.margins[t] });
        std::sort(sm.begin(), sm.end(),
                  [](const StepM& a, const StepM& c) { return a.m < c.m; });
        std::vector<size_t> weak;
        for (const auto& s : sm) {
            bool near = false;
            for (size_t w : weak) {
                if (std::llabs(static_cast<int64_t>(s.t) -
                               static_cast<int64_t>(w)) < 3) {
                    near = true;
                    break;
                }
            }
            if (!near) {
                weak.push_back(s.t);
                if (weak.size() >= 6)
                    break;
            }
        }
        if (work_bits.empty())
            work_bits = tr.bits;
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            const size_t t = weak[i];
            const size_t lo = (t > 12) ? (t - 12) : 0;
            const size_t hi = std::min(total, t + 13);
            std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                      work_bits.begin() + lo);
            const uint8_t fu = tr.lose_u[t];
            local_cl3_trellis_flip(llr0, llr1, lo, hi, tr.states[lo],
                                   tr.states[hi], &t, &fu, 1, work_bits);
            extract_block(b, work_bits, blk);
            if (rs_decode_block(blk, d330.data()))
                try_append(b, d330);
        }
        for (size_t i = 0; i < weak.size() && bc[b].cands.size() < 4; ++i) {
            for (size_t j = i + 1;
                 j < weak.size() && bc[b].cands.size() < 4; ++j) {
                const size_t t0 = std::min(weak[i], weak[j]);
                const size_t t1 = std::max(weak[i], weak[j]);
                const size_t lo = (t0 > 12) ? (t0 - 12) : 0;
                const size_t hi = std::min(total, t1 + 13);
                std::copy(tr.bits.begin() + lo, tr.bits.begin() + hi,
                          work_bits.begin() + lo);
                const size_t ft[2] = { t0, t1 };
                const uint8_t fu[2] = { tr.lose_u[t0], tr.lose_u[t1] };
                local_cl3_trellis_flip(llr0, llr1, lo, hi, tr.states[lo],
                                       tr.states[hi], ft, fu, 2, work_bits);
                extract_block(b, work_bits, blk);
                if (rs_decode_block(blk, d330.data()))
                    try_append(b, d330);
            }
        }
        if (bc[b].cands.empty())
            return false;
    }

    if (combine_and_crc(64))
        return true;
    return false;
}

// Soft-decision twin of decode_payload_hard(): the same joint [21 PHR | payload]
// symbol stream, but each of the two coded bits arrives as an LLR (positive
// favours bit 0, the bprf_demod_soft()/rxfusion::decide_bpm_soft() convention)
// instead of a hard bit.  Everything after the Viterbi is the shared
// finish_rs_crc_payload(), so the RS/CRC/FCS contract cannot diverge, and the
// diagnostic buckets are the same (viterbi_us / rs_us / crc_us).
inline bool decode_payload_soft(const std::vector<float>& llr0p,
                                const std::vector<float>& llr1p,
                                const std::vector<float>& llr0,
                                const std::vector<float>& llr1,
                                size_t psdu_bits,
                                PayloadResult& out,
                                PayloadSubTimers* sub = nullptr)
{
    PayloadSubTimer sub_guard(sub);
    out = PayloadResult{};
    if (llr0p.size() != 21 || llr1p.size() != 21 || llr0.empty() ||
        llr0.size() != llr1.size())
        return false;

    const size_t total = 21 + llr0.size();
    std::vector<float> llr0j(total), llr1j(total);
    for (size_t s = 0; s < 21; ++s) {
        llr0j[s] = llr0p[s];
        llr1j[s] = llr1p[s];
    }
    for (size_t s = 0; s < llr0.size(); ++s) {
        llr0j[s + 21] = llr0[s];
        llr1j[s + 21] = llr1[s];
    }
    std::vector<int8_t> decoded;
    Cl3TrellisTrace tr;
    const auto t_vit = std::chrono::steady_clock::now();
    // One tracing Viterbi serves both the first attempt and the rescue.  The
    // rescue needs the per-step path-metric margins, and recomputing them with
    // a second full-length pass was pure duplication: `tr.bits` is bit-identical
    // to what vitdec_cl3_soft() produces (same state/back-pointer updates).
    vitdec_cl3_soft_trace(llr0j.data(), llr1j.data(), total, tr);
    if (sub)
        sub->viterbi_us += payload_elapsed_us(t_vit);
    if (tr.bits.size() < total)
        return false;

    std::vector<int8_t> rs_cw(tr.bits.begin() + 19,
                              tr.bits.begin() + (total - 2));
    if (rs_cw.size() != llr0.size())
        return false;
    if (finish_rs_crc_payload(rs_cw, psdu_bits, out, sub) && out.fcs_pass)
        return true;

    PayloadResult rescued;
    if (rescue_payload_soft(llr0j.data(), llr1j.data(), total, psdu_bits, tr,
                            rescued, sub)) {
        out = std::move(rescued);
        return true;
    }
    return out.ok;
}

} // namespace detail


} // namespace gr::uwb::demod::core
