/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Multi-TX (dual-channel jammer) contract for the UWB EchoTimer
 * (X410 C++ dual-TX high-performance Radar CIR).
 *
 * Header-only, UHD-free.  Implements planning §5.1–§5.4 shared by the
 * EchoTimer worker, the Fake backend (QA) and the real UHD backend:
 *
 *   - kEchoMaxTxChannels = 4 fixed upper bound;
 *   - TxBurstFragment: one fragment carrying N equal-length buffer
 *     pointers (one per TX channel);
 *   - deterministic PCG/xorshift PRNG for per-pulse jammer delay
 *     (no std::uniform_int_distribution — cross-STL consistency is not
 *     guaranteed);
 *   - zero-copy multi-channel layout planner: sense is parked at D,
 *     jammer at D + delay, L = max(D+sense_len, 2D+jam_len) fixed at arm
 *     time; each burst only recomputes integer boundaries and pointers
 *     into per-channel waveforms or a preallocated zero scratch —
 *     never a (2, L) fill+copy.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_ECHO_MULTITX_H
#define INCLUDED_GNURADIO_UWB_UWB_ECHO_MULTITX_H

#include <gnuradio/uwb/uwb_echo_scheduler_core.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace echo {

// Fixed upper bound on TX channels (§5.1).  All hot-path arrays are
// sized by this constant; configured counts are 1..kEchoMaxTxChannels.
inline constexpr size_t kEchoMaxTxChannels = 4;

// One multi-channel TX fragment.  All configured channels carry the
// SAME count for the SAME offset: every UHD send uses
// buffers.size() == tx_channel_count with identical nsamps.
struct TxBurstFragment {
    const int16_t* tx_data[kEchoMaxTxChannels] = {};
    uint64_t offset = 0; // element offset inside the physical burst [0, L)
    uint64_t count = 0;  // elements in this fragment (per channel)
    unsigned flags = 0;  // kFlag* bits (same contract as BurstFragment)
    int64_t device_ticks = 0; // command time when kFlagTimeSpec
};

// Jammer delay mode (schedule PDU "jam_delay_mode": 0 = fixed, 1 = uniform).
enum class JamDelayMode : uint8_t { Fixed = 0, Uniform = 1 };

// ---------------------------------------------------------------------------
// Deterministic PRNG (§5.4): PCG-XSH-RR 32-bit output, 64-bit LCG state.
// Fixed algorithm, reproducible across STL implementations, no allocation.
// ---------------------------------------------------------------------------
class JamDelayRng
{
public:
    JamDelayRng() = default;
    explicit JamDelayRng(uint64_t seed) { seed_rng(seed); }

    void seed_rng(uint64_t seed)
    {
        // PCG seeding: state = 0, stream = fixed odd increment, then mix
        // the user seed in.  Stream is fixed so a seed fully determines
        // the sequence.
        d_state = 0u;
        d_inc = 1442695040888963407ULL; // fixed odd stream selector
        next_u32();
        d_state += seed;
        next_u32();
    }

    uint32_t next_u32()
    {
        const uint64_t old = d_state;
        d_state = old * 6364136223846793005ULL + d_inc;
        const uint32_t xorshifted =
            static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
        const uint32_t rot = static_cast<uint32_t>(old >> 59u);
        return (xorshifted >> rot) |
               (xorshifted << ((32u - rot) & 31u));
    }

    // Unbiased integer in [lo, hi] inclusive (Lemire multiply-high with
    // rejection; deterministic for a given stream).  lo may be negative.
    int64_t next_range_inclusive(int64_t lo, int64_t hi)
    {
        if (hi <= lo)
            return lo;
        const uint64_t span =
            static_cast<uint64_t>(hi - lo) + 1u; // hi > lo: no overflow
        // Rejection threshold: largest multiple of span fitting in 2^32.
        const uint64_t threshold =
            (0x100000000ULL % span == 0) ? 0 : (0x100000000ULL % span);
        for (;;) {
            const uint32_t r = next_u32();
            const uint64_t m =
                (static_cast<uint64_t>(r) * span) >> 32;
            if (static_cast<uint64_t>(r) - m * span >= threshold ||
                threshold == 0) {
                // m in [0, span): exact multiply-high mapping.
                return lo + static_cast<int64_t>(m);
            }
            // Rare rejection: draw again (bounded expected iterations).
        }
    }

private:
    uint64_t d_state = 0;
    uint64_t d_inc = 1442695040888963407ULL;
};

// ---------------------------------------------------------------------------
// Zero-copy layout planner (§5.4).
//
// Arm-time geometry (fixed for the whole grid):
//   sense_begin = D, sense_end = D + sense_len
//   L = max(D + sense_len, 2D + jam_len)
// Per-pulse jammer placement:
//   jam_begin = D + delay, jam_end = jam_begin + jam_len, delay in [-D, +D]
// Per burst the planner emits the sorted unique boundary set
// {0, sense_begin, sense_end, jam_begin, jam_end, L} (2..5 entries) and
// the caller resolves each fragment's per-channel pointer to either the
// channel waveform slice or the shared zero scratch.
// ---------------------------------------------------------------------------
struct MultiTxGeometry {
    uint64_t sense_len = 0;
    uint64_t jam_len = 0;   // 0 = jammer silent (row all zero)
    uint64_t half_span_D = 0; // D >= 0 (0 = fixed-delay legacy placement)
    uint64_t phys_len_L = 0;  // arm-time fixed L
    uint64_t sense_begin = 0; // == D
};

inline bool
prepare_multitx_geometry(uint64_t sense_len,
                         uint64_t jam_len,
                         uint64_t half_span_D,
                         MultiTxGeometry& out,
                         std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    out = MultiTxGeometry{};
    if (sense_len == 0)
        return fail("sense waveform must be non-empty");
    // L must fit the fixed fragment-array contract; the caller enforces
    // the block sample caps separately.
    uint64_t L = 0;
    if (jam_len == 0) {
        // No jammer: legacy single-row length (no pad) when D == 0,
        // else parked at D (radio worker keeps ROI semantics).
        if (half_span_D == 0)
            L = sense_len;
        else {
            if (half_span_D >
                static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
                sense_len >
                    static_cast<uint64_t>(
                        std::numeric_limits<uint64_t>::max()) -
                        half_span_D)
                return fail("geometry overflow");
            L = half_span_D + sense_len;
        }
    } else {
        if (half_span_D >
                static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
            return fail("geometry overflow");
        const uint64_t a = half_span_D + sense_len;
        // 2D + jam_len, checked.
        if (half_span_D >
            (std::numeric_limits<uint64_t>::max() - jam_len) / 2)
            return fail("geometry overflow");
        const uint64_t b = 2 * half_span_D + jam_len;
        L = a > b ? a : b;
    }
    if (L == 0)
        return fail("geometry overflow");
    out.sense_len = sense_len;
    out.jam_len = jam_len;
    out.half_span_D = half_span_D;
    out.phys_len_L = L;
    out.sense_begin = half_span_D;
    return true;
}

// Per-burst boundary set.  Returns the sorted unique boundaries in
// `bounds[0..*count)` with bounds[0] == 0 and bounds[count-1] == L.
// Count is 2..5.  delay_native is in [-D, +D] (0 when D == 0).
inline size_t
multitx_burst_bounds(const MultiTxGeometry& g,
                     int64_t delay_native,
                     uint64_t* bounds,
                     size_t cap)
{
    // Caller guarantees cap >= 5.
    uint64_t tmp[5];
    size_t n = 0;
    tmp[n++] = 0;
    const uint64_t sb = g.sense_begin;
    const uint64_t se = sb + g.sense_len;
    tmp[n++] = sb;
    tmp[n++] = se;
    if (g.jam_len > 0) {
        const int64_t jb_signed =
            static_cast<int64_t>(g.half_span_D) + delay_native;
        const uint64_t jb = static_cast<uint64_t>(jb_signed);
        const uint64_t je = jb + g.jam_len;
        tmp[n++] = jb;
        tmp[n++] = je;
    }
    // Insertion sort (n <= 5) + dedup.
    for (size_t i = 1; i < n; ++i) {
        uint64_t k = tmp[i];
        size_t j = i;
        while (j > 0 && tmp[j - 1] > k) {
            tmp[j] = tmp[j - 1];
            --j;
        }
        tmp[j] = k;
    }
    size_t m = 0;
    for (size_t i = 0; i < n && m < cap; ++i) {
        if (i == 0 || tmp[i] != tmp[m - 1])
            bounds[m++] = tmp[i];
    }
    // Guarantee trailing L (arithmetic above always includes se or je
    // == L at the extreme, but clamp defensively).
    if (m == 0 || bounds[m - 1] != g.phys_len_L) {
        if (m < cap)
            bounds[m++] = g.phys_len_L;
    }
    return m;
}

// Resolve one channel's pointer for fragment [off, off+count):
// pointer into `wave` when the fragment overlaps [begin, begin+len),
// else the shared zero scratch at the same offset.  `wave` is the
// channel's effective waveform base; `zero` is the preallocated all-zero
// scratch (capacity >= L).  Returns nullptr only when count == 0.
inline const int16_t*
multitx_channel_ptr(const int16_t* wave,
                    uint64_t wave_begin,
                    uint64_t wave_len,
                    const int16_t* zero,
                    uint64_t off,
                    uint64_t count)
{
    if (count == 0)
        return nullptr;
    if (wave != nullptr && wave_len > 0 && off < wave_begin + wave_len &&
        off + count > wave_begin) {
        // Fragment must lie fully inside one interval between planner
        // boundaries for a single slice pointer to be valid.  The worker
        // splits fragments at every boundary first, so a fragment that
        // straddles a waveform edge here is a planner bug: fall back to
        // per-interval splitting upstream.  For robustness, handle the
        // fully-inside fast path and clamp partial overlaps to zero for
        // the non-overlapping part is NOT representable by one pointer —
        // so report the waveform slice only when fully contained.
        if (off >= wave_begin && off + count <= wave_begin + wave_len)
            return wave + (off - wave_begin) * 2;
        // Straddling fragment: cannot represent with one pointer.
        // The worker guarantees this never happens (it splits at all
        // boundaries before resolving); return zero so a bug fails
        // loudly in QA sample checks rather than reading OOB.
        return zero + off * 2;
    }
    return zero + off * 2;
}

// ---------------------------------------------------------------------------
// Contiguous multi-TX window bank (2026-09-20 remediation).
//
// Production send path no longer splits at sensing/jammer begin/end.
// Arm-time materialization builds one dense row per non-jam channel of
// length L, plus a jammer backing of length L+2D (uniform) so a per-pulse
// delay only slides the jammer read pointer.  Hot path: one fragment of
// count L; TX0 pointer/length/sample positions are identical every burst.
//
//   jam_backing_length(L, D) = L + 2D
//   jam_wave_begin(D)        = 2D
//   jam_window_begin(D, d)   = D - delta,  delta in [-D, +D]
// ---------------------------------------------------------------------------
struct MultiTxWindowBank {
    uint64_t tx_len = 0;                 // L, samples/channel
    uint64_t sense_offset = 0;           // D
    uint64_t jam_backing_wave_begin = 0; // 2D (uniform); unused in fixed
    size_t channel_count = 0;
    size_t jam_channel = 1;
    JamDelayMode delay_mode = JamDelayMode::Fixed;

    std::array<std::vector<int16_t>, kEchoMaxTxChannels> dense_rows;
    std::vector<int16_t> jam_backing; // empty in fixed mode
};

inline bool
jam_backing_length(uint64_t L, uint64_t D, uint64_t& out,
                   std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (D > (std::numeric_limits<uint64_t>::max() / 2))
        return fail("geometry overflow");
    const uint64_t two_d = 2u * D;
    if (L > std::numeric_limits<uint64_t>::max() - two_d)
        return fail("geometry overflow");
    out = L + two_d;
    return true;
}

inline bool
jam_wave_begin(uint64_t D, uint64_t& out, std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (D > (std::numeric_limits<uint64_t>::max() / 2))
        return fail("geometry overflow");
    out = 2u * D;
    return true;
}

inline bool
jam_window_begin(uint64_t D, int64_t delta, uint64_t& out,
                 std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (D > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        return fail("geometry overflow");
    const int64_t Di = static_cast<int64_t>(D);
    if (delta < -Di || delta > Di)
        return fail("jam_delay_out_of_span");
    // Di - delta ∈ [0, 2D] for delta ∈ [-D, +D]; no signed overflow.
    out = static_cast<uint64_t>(Di - delta);
    return true;
}

// Bytes occupied by dense rows + jam backing (diagnostics / memory report).
inline uint64_t
multitx_window_bank_bytes(const MultiTxWindowBank& bank)
{
    uint64_t n = bank.jam_backing.size();
    for (size_t c = 0; c < kEchoMaxTxChannels; ++c)
        n += bank.dense_rows[c].size();
    if (n > std::numeric_limits<uint64_t>::max() / sizeof(int16_t))
        return std::numeric_limits<uint64_t>::max();
    return n * static_cast<uint64_t>(sizeof(int16_t));
}

// Logical L-sample jammer window for delay `delta` (zeros + jam at D+delta).
// `out` must hold L SC16 pairs (2L int16).  Returns false on overflow.
inline bool
compose_jam_window(const int16_t* jam,
                   uint64_t jam_len,
                   uint64_t L,
                   uint64_t D,
                   int64_t delta,
                   int16_t* out,
                   std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        return false;
    };
    if (out == nullptr || L == 0)
        return fail("compose_jam_window bad args");
    if (L > std::numeric_limits<size_t>::max() / 2)
        return fail("geometry overflow");
    std::memset(out, 0, static_cast<size_t>(L) * 2u * sizeof(int16_t));
    if (jam == nullptr || jam_len == 0)
        return true;
    if (D > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
        return fail("geometry overflow");
    const int64_t begin = static_cast<int64_t>(D) + delta;
    if (begin < 0)
        return fail("jam_window_underflow");
    const uint64_t ubegin = static_cast<uint64_t>(begin);
    if (ubegin > L || jam_len > L - ubegin)
        return fail("jam_window_out_of_bounds");
    std::memcpy(out + ubegin * 2, jam,
                static_cast<size_t>(jam_len) * 2u * sizeof(int16_t));
    return true;
}

// Materialize dense rows / jam backing.  Control path only (schedule
// handler); never called from the per-burst hot path.
//
// waves[c] / wave_lens[c] are the effective SC16 waveforms (pairs).
// bases[c] is the frozen base offset inside L (sense D = bases[0]).
inline bool
materialize_multitx_window_bank(const int16_t* const waves[],
                                const uint64_t* wave_lens,
                                const uint64_t* bases,
                                size_t nch,
                                size_t jam_ch,
                                uint64_t L,
                                JamDelayMode mode,
                                uint64_t max_tx_samples,
                                MultiTxWindowBank& out,
                                std::string* error = nullptr)
{
    auto fail = [&](const char* what) {
        if (error)
            *error = what;
        out = MultiTxWindowBank{};
        return false;
    };
    if (waves == nullptr || wave_lens == nullptr || bases == nullptr)
        return fail("materialize null args");
    if (nch < 2 || nch > kEchoMaxTxChannels)
        return fail("bad_channel_count");
    if (jam_ch < 1 || jam_ch >= nch)
        return fail("jam_logical_channel_out_of_range");
    if (L == 0 || L > max_tx_samples)
        return fail("tx_len_over_cap");
    if (L > std::numeric_limits<size_t>::max() / 2)
        return fail("geometry overflow");
    const uint64_t D = bases[0];
    if (wave_lens[0] == 0)
        return fail("sense_waveform_empty");
    if (D > L || wave_lens[0] > L - D)
        return fail("channel_geometry_out_of_bounds");

    out = MultiTxWindowBank{};
    out.tx_len = L;
    out.sense_offset = D;
    out.channel_count = nch;
    out.jam_channel = jam_ch;
    out.delay_mode = mode;

    const size_t pair_elems = static_cast<size_t>(L) * 2u;
    auto place = [&](std::vector<int16_t>& row, const int16_t* wave,
                     uint64_t wlen, uint64_t begin) -> bool {
        row.assign(pair_elems, static_cast<int16_t>(0));
        if (wave == nullptr || wlen == 0)
            return true;
        if (begin > L || wlen > L - begin)
            return false;
        std::memcpy(row.data() + static_cast<size_t>(begin) * 2, wave,
                    static_cast<size_t>(wlen) * 2u * sizeof(int16_t));
        return true;
    };

    for (size_t c = 0; c < nch; ++c) {
        if (bases[c] > L || wave_lens[c] > L - bases[c])
            return fail("channel_geometry_out_of_bounds");
        // Uniform jammer lives in jam_backing; its dense row stays zeros
        // so a bug that forgets to slide the pointer still sends silence
        // rather than a delay-dependent TX0-adjacent slice.
        const bool skip_jam_dense =
            (mode == JamDelayMode::Uniform && c == jam_ch);
        if (skip_jam_dense) {
            out.dense_rows[c].assign(pair_elems, static_cast<int16_t>(0));
            continue;
        }
        if (!place(out.dense_rows[c], waves[c], wave_lens[c], bases[c]))
            return fail("channel_geometry_out_of_bounds");
    }

    if (mode == JamDelayMode::Uniform) {
        uint64_t backing_len = 0;
        uint64_t wave_begin = 0;
        if (!jam_backing_length(L, D, backing_len, error) ||
            !jam_wave_begin(D, wave_begin, error))
            return false;
        if (backing_len > std::numeric_limits<size_t>::max() / 2)
            return fail("geometry overflow");
        // Bound backing by 2 * max_tx_samples so a large D cannot explode
        // memory after L itself already passed the cap.
        const uint64_t backing_cap =
            (max_tx_samples > std::numeric_limits<uint64_t>::max() / 2)
                ? std::numeric_limits<uint64_t>::max()
                : max_tx_samples * 2u;
        if (backing_len > backing_cap)
            return fail("backing_over_cap");
        out.jam_backing_wave_begin = wave_begin;
        out.jam_backing.assign(static_cast<size_t>(backing_len) * 2u,
                               static_cast<int16_t>(0));
        const uint64_t Nj = wave_lens[jam_ch];
        if (waves[jam_ch] != nullptr && Nj > 0) {
            if (wave_begin > backing_len || Nj > backing_len - wave_begin)
                return fail("jam_backing_out_of_bounds");
            std::memcpy(out.jam_backing.data() +
                            static_cast<size_t>(wave_begin) * 2,
                        waves[jam_ch],
                        static_cast<size_t>(Nj) * 2u * sizeof(int16_t));
        }
    }
    return true;
}

// Per-burst jammer pointer: uniform slides the backing window; fixed
// returns the already-placed dense row.  Never allocates.
inline const int16_t*
multitx_jam_ptr(const MultiTxWindowBank& bank, int64_t delta,
                std::string* error = nullptr)
{
    if (bank.delay_mode != JamDelayMode::Uniform) {
        if (bank.jam_channel >= kEchoMaxTxChannels)
            return nullptr;
        const auto& row = bank.dense_rows[bank.jam_channel];
        return row.empty() ? nullptr : row.data();
    }
    uint64_t win = 0;
    if (!jam_window_begin(bank.sense_offset, delta, win, error))
        return nullptr;
    const uint64_t backing_pairs = bank.jam_backing.size() / 2;
    if (win > backing_pairs || bank.tx_len > backing_pairs - win)
        return nullptr;
    return bank.jam_backing.data() + static_cast<size_t>(win) * 2;
}

} // namespace echo
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_ECHO_MULTITX_H */
