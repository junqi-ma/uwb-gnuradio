// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <openuwb/phy/hrp/rx/demod_result.h>
#include <openuwb/phy/hrp/rx/rx_bpm.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <openuwb/phy/hrp/common/hrp_codec.h>

namespace openuwb::demod::core {
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
using openuwb::codec::bprf_spreading;

inline void cl3_trellis_tables(int o0[4][2], int o1[4][2], int nxt[4][2])
{
    for (int st = 0; st < 4; ++st) {
        for (int u = 0; u < 2; ++u) {
            const auto step = openuwb::codec::cl3_transition(st, u);
            o0[st][u] = step.g0;
            o1[st][u] = step.g1;
            nxt[st][u] = step.next_state;
        }
    }
}


// ---------------------------------------------------------------------------
// Unrolled 4-state CL-3 ACS (poly2trellis(3,[2 5])).
//
// next = (u << 1) | (st >> 1), g0 = st >> 1, g1 = u ^ (st & 1).  Each target
// state therefore has exactly two predecessor edges, and both share the same
// u (so the old `for (st) for (u)` nested loop visited them in ascending st
// order):
//
//   nst = 0: (st=0,u=0) code 0, (st=1,u=0) code 1
//   nst = 1: (st=2,u=0) code 2, (st=3,u=0) code 3
//   nst = 2: (st=0,u=1) code 4, (st=1,u=1) code 5
//   nst = 3: (st=2,u=1) code 6, (st=3,u=1) code 7
//
// `code = (u << 2) | st` is exactly the survivor byte the reference loop
// stored.  The eight edges below are emitted as explicit statements with the
// same comparison (`strictly less`, i.e. the smaller (st, u) keeps an exact
// tie) and the same per-target order as the reference, so the decoded bits are
// bit-identical.  Targets that receive no improving edge keep the
// zero-initialised back pointer, exactly like the reference.
// ---------------------------------------------------------------------------
inline constexpr size_t kCl3StackTraceMax = 2048;

inline void cl3_acs_update(int m, int nst, int8_t code, int8_t* bslot, int* ns)
{
    if (m < ns[nst]) {
        ns[nst] = m;
        *bslot = code;
    }
}

inline void cl3_acs_update_soft(float m, int nst, int8_t code, int8_t* bslot,
                                float* ns)
{
    if (m < ns[nst]) {
        ns[nst] = m;
        *bslot = code;
    }
}

// Tracing-ACS update: keeps the runner-up metric (`ns2`) and the competing
// edge (`nalt`) alongside the survivor, matching the reference loop exactly.
inline void cl3_trace_update(float m, int nst, int8_t code, int8_t* bslot,
                             float* ns, float* ns2, uint8_t* nalt)
{
    if (m < ns[nst]) {
        ns2[nst] = ns[nst];
        nalt[nst] = static_cast<uint8_t>(*bslot);
        ns[nst] = m;
        *bslot = code;
    } else if (m < ns2[nst]) {
        ns2[nst] = m;
        nalt[nst] = static_cast<uint8_t>(code);
    }
}

// Traceback buffer: 8 KiB of stack covers every PSDU length used in practice
// (N <= 2048); longer inputs fall back to a heap vector.  Both are
// zero-filled, matching std::vector's value initialisation.
struct Cl3BackBuffer {
    int8_t stack[kCl3StackTraceMax * 4];
    std::vector<int8_t> heap;
    int8_t* get(size_t N)
    {
        const size_t n = N * 4;
        if (n > sizeof(stack)) {
            heap.assign(n, int8_t{0});
            return heap.data();
        }
        std::fill_n(stack, n, int8_t{0});
        return stack;
    }
};

void vitdec_cl3(const int8_t* rx, size_t N, std::vector<int8_t>& out);

// SECDED encode: 13 systematic bits -> 19-bit codeword (13 data + 6 parity).
// Parity matrix rows from lrwpan.internal.hrpSECDED (verified against golden).
using openuwb::codec::hrp_secded;

// ---------------------------------------------------------------------------
// Shared BPM-BPSK BPRF demod kernel (used by both the PHR and payload fields).
// soft: chip-rate soft chips (0-based absolute).  start: field start chip.
// nsym: number of BPM-BPSK symbols.  cpb: chips per burst.  cps: chips per
// symbol (BPM half-symbol offset = cps/2).  scram_offset: LFSR start bit
// (PHR=0, payload=1344).  pol: sfd polarity.  Fills g0 (BPM position bit) and
// g1 (BPSK polarity bit), each nsym entries.  Mirrors helperUWBBPRFDemodKernel.
// ---------------------------------------------------------------------------
bool bprf_demod_impl(const std::vector<float>& soft, int64_t start,
                     size_t code_index, size_t nsym, size_t cpb, size_t cps,
                     size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                     std::vector<int8_t>& g1, std::vector<float>* llr_g0,
                     std::vector<float>* llr_g1);

bool bprf_demod(const std::vector<float>& soft, int64_t start,
                size_t code_index, size_t nsym, size_t cpb, size_t cps,
                size_t scram_offset, int8_t pol, std::vector<int8_t>& g0,
                std::vector<int8_t>& g1);

// Positive LLRs favour bit 0 for BPM position and BPSK polarity.
bool bprf_demod_soft(const std::vector<float>& soft, int64_t start,
                     size_t code_index, size_t nsym, size_t cpb, size_t cps,
                     size_t scram_offset, int8_t pol,
                     std::vector<int8_t>& g0, std::vector<int8_t>& g1,
                     std::vector<float>& llr_g0, std::vector<float>& llr_g1);

inline float cl3_soft_path_metric(float path, int o0, int o1,
                                  float llr0, float llr1)
{
    const float c0 = (2.0f * static_cast<float>(o0) - 1.0f) * llr0;
    const float c1 = (2.0f * static_cast<float>(o1) - 1.0f) * llr1;
    return path + c0 + c1;
}

// Soft-decision rate-1/2 CL-3 Viterbi (same trellis as vitdec_cl3).
// llr0/llr1: per-symbol LLRs for the two coded bits (positive favours bit 0).
void vitdec_cl3_soft(const float* llr0, const float* llr1, size_t N,
                     std::vector<int8_t>& out);

// IEEE 802.15.4 CRC-16 (reflected polynomial 0x8408, init 0).  FCS algorithm
// used by DW1000 / QM35 frames (matches uwbdecoder.ieee802154CRC16).
using openuwb::codec::crc16_802154;

// ---------------------------------------------------------------------------
// IEEE 802.15.4a RS(63,55) over GF(2^6) — matches lrwpan.internal.hrpRS.
// Field primitive poly: x^6 + x^5 + 1 (0x61).  Generator roots alpha^1..alpha^8
// with alpha = 2 (the element x).  Symbols are 6-bit, packed MSB-first.
// Systematic codeword layout (low-first poly): [55 data][8 parity] symbols =
// 330 data bits + 48 parity bits = 378 bits.  Partial blocks are formed by
// LEADING zero-padding the data field to 330 bits (verified on golden vectors).
// ---------------------------------------------------------------------------

// GF(2^6) multiply (poly 0x61).
using openuwb::codec::rs_gf_mul;

// GF(2^6) power a^n (n may be negative via modular reduction mod 63).
using openuwb::codec::rs_gf_pow;

using openuwb::codec::rs_gf_inv;

// Precomputed alpha^i / alpha^{-j} kernel lookups (GF(2^6), alpha = 2).
using openuwb::codec::rs_gf_alpha_inv_pow;
using openuwb::codec::rs_gf_alpha_pow;

// Pack 378 MSB-first bits -> 63 GF(2^6) symbols.
using openuwb::codec::rs_bits_to_syms;

// Unpack nsym symbols -> 6*nsym MSB-first bits.
using openuwb::codec::rs_syms_to_bits;

// Syndromes S_i = r(alpha^i), i=1..8.  Low-first: r(x) = sum_j r[j] x^j.
// Horner: s = (((r62 * ai) ^ r61) * ai ^ ...) ^ r0, one table multiply per
// coefficient (the old loop maintained alpha^j separately with a second
// multiply per coefficient).
void rs_syndromes(const int* r63, int* S8);

// Berlekamp-Massey over GF(2^6) on `count` syndromes S[0..count-1].
// Returns L; Lambda[0..L] with Lambda[0]=1.
int rs_berlekamp_massey_len(const int* S, int count, int* Lambda /* len >= 9 */);

// Berlekamp-Massey on S1..S8.  Returns L; Lambda[0..L] with Lambda[0]=1.
int rs_berlekamp_massey(const int* S8, int* Lambda /* len >= 9 */);

// Chien search: roots of Lambda at alpha^{-j} => error at position j.
// Returns number of roots (may exceed L if uncorrectable).
// alpha^{-j} comes from the precomputed table and Lambda(alpha^{-j}) is
// evaluated by Horner from the highest coefficient.
int rs_chien_search(const int* Lambda, int L, int* err_pos /* <=4 */);

// Forney (b=1, roots alpha^1..): e_j = Omega(X^{-1}) / Lambda'(X^{-1}).
void rs_forney(const int* S8, const int* Lambda, int L,
               const int* err_pos, int nerr, int* err_val);

inline bool rs_zero_syndromes(const int* syndromes)
{
    for (int i = 0; i < 8; ++i)
        if (syndromes[i] != 0)
            return false;
    return true;
}

bool rs_apply_corrections(int* symbols, const int* syndromes,
                          const int* locator, int degree,
                          const int* positions, int count);

// Reinsert the leading zero bits removed from shortened systematic blocks.
void expand_rs_codeword(const int8_t* coded, size_t data_bits, int8_t* block);

// Decode one RS(63,55) block: 378 coded bits -> 330 data bits (MSB-first).
// Returns false if the block is uncorrectable (more than t=4 symbol errors).
bool rs_decode_block(const int8_t* coded378, int8_t* data330);

// Full-stream RS decoder for a PSDU of `data_bits` bits (e.g. 8*L).
// Splits the coded bit stream into num_blocks = ceil(data_bits/330) blocks,
// each carrying min(330, remaining) data bits + 48 parity bits.  Partial
// blocks are leading-zero-padded to 330 data bits before decode.
bool rs_decode_stream(const std::vector<int8_t>& coded,
                      size_t data_bits,
                      std::vector<int8_t>& data_out);

void pack_psdu_bytes(const std::vector<int8_t>& bits, std::vector<uint8_t>& bytes);

bool check_psdu_fcs(const std::vector<uint8_t>& bytes,
                    uint16_t& calculated, uint16_t& received);

// Reusable buffers for the payload hard/soft decoders.  The fused payload
// stage owns one of these in DemodScratch, so the per-packet work vectors keep
// their capacity instead of being reallocated for every frame.  Passing
// nullptr keeps the original self-allocating behaviour (legacy callers and QA).
struct PayloadDecodeWorkspace {
    std::vector<int8_t> rx_bits;
    std::vector<int8_t> decoded;
    std::vector<int8_t> rs_cw;
    std::vector<int8_t> psdu_bits;
    std::vector<float> llr0_joint;
    std::vector<float> llr1_joint;
};

// Shared RS + PSDU assembly + FCS tail for the ordinary hard and soft
// decoder paths.  `out` is reset first;
// `out.ok` is set only when the RS decode succeeded, and `out.fcs_pass` then
// reports the CRC verdict (a false FCS is a decoded-but-rejected frame, not a
// decoder failure).
bool finish_rs_crc_payload(const std::vector<int8_t>& rs_cw,
                           size_t psdu_bits,
                           PayloadResult& out,
                           PayloadSubTimers* sub = nullptr,
                           PayloadDecodeWorkspace* ws = nullptr);

// Shared hard-decision payload backend: concatenate the 21 PHR symbols and the
// payload symbols, run the truncated CL-3 Viterbi, remove the 19 systematic PHR
// bits and the two tail bits, RS-decode, pack LSB-first and compute the FCS.
// This is the exact first attempt of stage_payload_fcs(); the fused payload
// path calls it with g0/g1 produced by rxfusion, so both paths share the same
// FEC/FCS implementation and length conventions.
bool decode_payload_hard(const std::vector<int8_t>& g0p,
                         const std::vector<int8_t>& g1p,
                         const std::vector<int8_t>& g0,
                         const std::vector<int8_t>& g1,
                         size_t psdu_bits,
                         PayloadResult& out,
                         PayloadSubTimers* sub = nullptr,
                         PayloadDecodeWorkspace* ws = nullptr);

struct Cl3TrellisTrace {
    std::vector<int8_t> bits;
    std::vector<int8_t> states;
    std::vector<float> margins;
    std::vector<uint8_t> lose_u;
};

// Soft CL-3 Viterbi recording per-step path metric margins and competing edges
// (invoked only when single-pass soft decode + RS/CRC fails).
void vitdec_cl3_soft_trace(const float* llr0,
                           const float* llr1,
                           size_t N,
                           Cl3TrellisTrace& tr);

// Re-solve a local CL-3 trellis window [lo, hi) between fixed endpoint states
// while forcing the loser input bit at one or two target steps.
void local_cl3_trellis_flip(const float* llr0,
                            const float* llr1,
                            size_t lo,
                            size_t hi,
                            int start_st,
                            int end_st,
                            const size_t* force_t,
                            const uint8_t* force_u,
                            size_t n_force,
                            std::vector<int8_t>& out_bits);

// RS(63,55) Errors-and-Erasures decoder (`v` erasures + `e` unknown errors,
// 2*e + v <= 8, capped at v <= 6 so at least 2 residual parity checks remain).
bool rs_decode_block_erasures(const int8_t* coded378,
                              const int* eras_pos,
                              int v,
                              int8_t* data330);

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
bool rescue_payload_soft(const float* llr0,
                         const float* llr1,
                         size_t total,
                         size_t psdu_bits,
                         const Cl3TrellisTrace& tr,
                         PayloadResult& out,
                         PayloadSubTimers* sub = nullptr);

// Soft-decision twin of decode_payload_hard(): the same joint [21 PHR | payload]
// symbol stream, but each of the two coded bits arrives as an LLR (positive
// favours bit 0, the bprf_demod_soft()/rxfusion::decide_bpm_soft() convention)
// instead of a hard bit.  Everything after the Viterbi is the shared
// finish_rs_crc_payload(), so the RS/CRC/FCS contract cannot diverge, and the
// diagnostic buckets are the same (viterbi_us / rs_us / crc_us).
bool decode_payload_soft(const std::vector<float>& llr0p,
                         const std::vector<float>& llr1p,
                         const std::vector<float>& llr0,
                         const std::vector<float>& llr1,
                         size_t psdu_bits,
                         PayloadResult& out,
                         PayloadSubTimers* sub = nullptr,
                         PayloadDecodeWorkspace* ws = nullptr);

// ===========================================================================
// Joint bi-orthogonal ML CL-3 decoding (Wave 1 / JOINT-ML-VITERBI).
//
// IEEE 802.15.4 HRP BPM-BPSK maps the two CL-3 coded bits (o0, o1) onto the
// four bi-orthogonal points (+A,0), (-A,0), (0,+A), (0,-A):
//
//   (0,0) -> (+A, 0)     (0,1) -> (-A, 0)
//   (1,0) -> (0, +A)     (1,1) -> (0, -A)
//
// With (m0, m1) the two half-symbol matched-filter outputs, the exact negative
// log-likelihood (up to a symbol-common constant) is
//
//   branch(o0, o1, t) = (o0 == 0) ? (o1 ? +m0[t] : -m0[t])
//                                 : (o1 ? +m1[t] : -m1[t])
//
// and Viterbi minimises the path sum.  This keeps the BPM position and the
// BPSK polarity coupled.  The legacy split soft metric
// (llr0 = |m0| - |m1|, llr1 = max(m) - max(-m)) throws that coupling away:
// for m0 = +1.0, m1 = -0.8 the true runner-up branch (1,1) costs -0.8, but the
// split LLR turns it into the *worst* branch (+0.4).  Everything below uses
// this branch metric with the same 4-state 8-edge unrolled kernels, the same
// edge order and the same strict `<` tie break as the legacy functions.
// ===========================================================================
inline float cl3_biorth_branch(float m0, float m1, int o0, int o1)
{
    return (o0 == 0) ? (o1 ? m0 : -m0) : (o1 ? m1 : -m1);
}

// ML counterpart of vitdec_cl3_soft(): same trellis, same edge order, same
// traceback.  `terminated == true` forces the final state to 0 (the standard
// appends two zero tail bits after the last RS codeword).
void vitdec_cl3_biorth(const float* m0, const float* m1, size_t N,
                       std::vector<int8_t>& out,
                       bool terminated = false);

// Tracing ML Viterbi: bits/states/margins/lose_u have exactly the same shape
// and semantics as vitdec_cl3_soft_trace().  `margins[t]` is the winning path
// metric minus the best competing merged edge (`ns2 - ns`, including the
// INF-INF == 0 behaviour of unreachable states); `lose_u[t]` is the input bit
// of that competing edge.  The eight edges are emitted in the reference
// nested-loop (st, u) order with the same strict `<` tie break.
void vitdec_cl3_biorth_trace(const float* m0, const float* m1, size_t N,
                             Cl3TrellisTrace& tr,
                             bool terminated = false);

// Bi-orthogonal twin of local_cl3_trellis_flip(): re-solve [lo, hi) between
// fixed endpoint states while forcing the loser input bit at up to two steps.
void local_cl3_trellis_flip_biorth(const float* m0,
                                   const float* m1,
                                   size_t lo,
                                   size_t hi,
                                   int start_st,
                                   int end_st,
                                   const size_t* force_t,
                                   const uint8_t* force_u,
                                   size_t n_force,
                                   std::vector<int8_t>& out_bits);

// Pulse-interference guard for the bi-orthogonal metrics.  The per-symbol
// amplitude max(|m0|, |m1|) is summarised by a deterministic strided sample
// (<= 1024 points, fixed stack array, zero heap allocation) and clamped to
// +/- clip_factor * median.  Sign patterns are preserved, but a single 50x
// burst can no longer outvote hundreds of clean symbols in the Viterbi path.
// `N < 16` or a non-positive median (e.g. an all-zero buffer) skips safely.
void clip_biorth_metrics(float* m0, float* m1, size_t N,
                         float clip_factor = 4.0f);

// Bi-orthogonal twin of rescue_payload_soft(): identical two-phase logic
// (Phase 1 RS errors-and-erasures v = 2/4/6, Phase 2 six single + fifteen pair
// trellis flips), only the local re-decode uses the ML branch metric.  Only a
// full-frame CRC-16 match is accepted.
bool rescue_payload_biorth(const float* m0,
                           const float* m1,
                           size_t total,
                           size_t psdu_bits,
                           const Cl3TrellisTrace& tr,
                           PayloadResult& out,
                           PayloadSubTimers* sub = nullptr);

// End-to-end bi-orthogonal payload entry point.  Same contract as
// decode_payload_soft(): the 21 PHR symbols are prepended to the payload
// symbols, one tracing Viterbi decodes the joint stream, the first 19
// systematic PHR bits and the two tail bits are removed, and the shared
// finish_rs_crc_payload() performs RS + PSDU assembly + FCS.  The rescue runs
// only when that first attempt does not pass the FCS.  A false FCS therefore
// still returns `ok == true, fcs_pass == false`, exactly like the soft path.
//
// `ws->llr0_joint` / `llr1_joint` are reused as the joined m0/m1 buffers (they
// are plain float workspaces; the name is historical).  With a workspace the
// steady state performs no heap allocation apart from the trace vectors.
bool decode_payload_biorth_soft(const float* m0p,
                                const float* m1p,
                                const float* m0,
                                const float* m1,
                                size_t payload_symbols,
                                size_t psdu_bits,
                                PayloadResult& out,
                                PayloadSubTimers* sub = nullptr,
                                PayloadDecodeWorkspace* ws = nullptr,
                                bool terminated = false,
                                bool enable_clip = false);

// Bi-orthogonal PHR entry point (21 symbols, fixed `terminated = false`
// because the PHR is followed by the payload in the same continuous CL-3
// trellis).  uwb_demod_fec.h cannot include uwb_demod_phr.h (that header
// includes this one), so the SECDED/parser tail is implemented here from the
// same hrp_secded() primitive and the same 13 corrected-address table.  Note:
// the phr_secded_parse() "long length tail-energy rollback" heuristic is NOT
// replicated; Wave 2 re-checks this decode against decode_phr_decisions() and
// the soft-chip energy at the uwb_demod_phr.h layer.  `soft_chips` and
// `ns_sfd` are accepted only to keep the call sites symmetric.
bool decode_phr_biorth_soft(const float* m0p,
                            const float* m1p,
                            const std::vector<float>& soft_chips,
                            const NsSfdResult& ns_sfd,
                            PhrResult& out);

} // namespace detail


} // namespace openuwb::demod::core
