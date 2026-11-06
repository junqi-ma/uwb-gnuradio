/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Experimental RX fast-path helpers: SFD template caching and the
 * SFD-anchored first-SYNC refine radius.  Header-only and GNU-Radio-runtime
 * independent, mirroring uwb_demod_core.h.
 *
 * Every helper is opt-in through Qm35825Profile.  With the experimental flags
 * disabled the callers keep the legacy algorithms unchanged.
 *
 * Threading: the cache lives in DemodScratch, which is per worker.  Nothing
 * here is shared between workers or allocated per candidate.
 */

#pragma once

#include <gnuradio/uwb/uwb_detector_core.h> // gr::uwb::core::uwb_l2_normalize
#include <gnuradio/uwb/uwb_phy_profile.h>   // kQm35SamplesPerSymbol

#include <volk/volk.h>

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace gr {
namespace uwb {
namespace demod {
namespace rxopt {

// ---------------------------------------------------------------------------
// Long complex conjugate dot product, identical math to
// demod::core::detail::timing_conjugate_dot:
//
//   returns sum_{k < count} input[k] * conj(taps[k])
//
// VOLK runtime-dispatches to the fastest kernel on the host.  The SFD search
// evaluates 8128-point dots, so dispatch overhead is amortized.  Reduction
// order differs from the scalar loop, so the fast path is only enabled
// explicitly and its output is verified against the legacy search.
// ---------------------------------------------------------------------------
inline std::complex<float> conjugate_dot(const std::complex<float>* input,
                                         const std::complex<float>* taps,
                                         size_t count)
{
    std::complex<float> acc(0.0f, 0.0f);
    if (count == 0)
        return acc;
    volk_32fc_x2_conjugate_dot_prod_32fc(
        &acc,
        reinterpret_cast<const lv_32fc_t*>(input),
        reinterpret_cast<const lv_32fc_t*>(taps),
        static_cast<unsigned int>(count));
    return acc;
}

namespace detail {

// FNV-1a (64-bit).  Multi-byte values are fed little-endian so the key is
// reproducible across hosts and does not depend on struct padding.
inline constexpr uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
inline constexpr uint64_t kFnvPrime = 1099511628211ULL;

inline uint64_t fnv1a_bytes(uint64_t h, const void* data, size_t count)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < count; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= kFnvPrime;
    }
    return h;
}

inline uint64_t fnv1a_u32(uint64_t h, uint32_t v)
{
    uint8_t buf[4];
    for (size_t i = 0; i < sizeof(buf); ++i)
        buf[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xffu);
    return fnv1a_bytes(h, buf, sizeof(buf));
}

inline uint64_t fnv1a_u64(uint64_t h, uint64_t v)
{
    uint8_t buf[8];
    for (size_t i = 0; i < sizeof(buf); ++i)
        buf[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xffu);
    return fnv1a_bytes(h, buf, sizeof(buf));
}

} // namespace detail

// ---------------------------------------------------------------------------
// Cache key for the normalized SFD template.
//
// Covers, in order: template length, every template sample's raw float bit
// pattern, SFD sequence length, and the int8 SFD values.  It deliberately does
// NOT look at DemodScratch work/corr contents or at any pointer identity, so a
// cached template stays valid across jobs and scratch reallocations.
//
// Callers must also require a non-empty cached template before reuse: a key of
// 0 is also the "never built" state.
// ---------------------------------------------------------------------------
inline uint64_t
sfd_template_key(const std::vector<std::complex<float>>& template_wf,
                 const std::vector<int8_t>& sfd_sequence)
{
    static_assert(sizeof(float) == 4, "SFD cache key assumes 32-bit float");
    uint64_t h = detail::kFnvOffsetBasis;
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(template_wf.size()));
    for (const std::complex<float>& v : template_wf) {
        uint32_t re_bits = 0;
        uint32_t im_bits = 0;
        const float re = v.real();
        const float im = v.imag();
        std::memcpy(&re_bits, &re, sizeof(re_bits));
        std::memcpy(&im_bits, &im, sizeof(im_bits));
        h = detail::fnv1a_u32(h, re_bits);
        h = detail::fnv1a_u32(h, im_bits);
    }
    h = detail::fnv1a_u64(h, static_cast<uint64_t>(sfd_sequence.size()));
    if (!sfd_sequence.empty())
        h = detail::fnv1a_bytes(h, sfd_sequence.data(), sfd_sequence.size());
    return h;
}

// ---------------------------------------------------------------------------
// Build the normalized SFD waveform: kron(sfd_sequence, template_wf), then
// L2-normalize.  This reproduces the legacy stage_sfd construction exactly,
// including its per-symbol length of kQm35SamplesPerSymbol samples (the legacy
// loop reads template_wf[0..kQm35SamplesPerSymbol), not template_wf.size()).
//
// May allocate (a cache miss only).  On invalid input the output is cleared so
// callers can fail closed; the cache key must only be committed when the
// output is non-empty.
// ---------------------------------------------------------------------------
inline void
build_sfd_template(const std::vector<std::complex<float>>& template_wf,
                   const std::vector<int8_t>& sfd_sequence,
                   std::vector<std::complex<float>>& out)
{
    out.clear();
    const size_t sym = kQm35SamplesPerSymbol;
    if (sfd_sequence.empty() || template_wf.size() < sym)
        return; // the legacy loop would read past the template: fail closed
    if (sfd_sequence.size() > std::numeric_limits<size_t>::max() / sym)
        return;
    out.resize(sfd_sequence.size() * sym);
    for (size_t i = 0; i < sfd_sequence.size(); ++i) {
        const int8_t s = sfd_sequence[i];
        for (size_t k = 0; k < sym; ++k)
            out[i * sym + k] =
                std::complex<float>(static_cast<float>(s) * template_wf[k].real(),
                                    static_cast<float>(s) * template_wf[k].imag());
    }
    gr::uwb::core::uwb_l2_normalize(out);
}

// ---------------------------------------------------------------------------
// Half-width of the first seeded SYNC refine.
//
// configured_radius == 0 keeps the legacy max(256, timing_coarse_stride)
// behaviour; a nonzero configured radius is the opt-in narrow search and is
// clamped to at least 1 sample.  Mirrors the legacy integer promotion exactly
// so the fallback radius is unchanged.
// ---------------------------------------------------------------------------
inline int64_t seed_refine_radius(size_t configured_radius, size_t stride)
{
    if (configured_radius != 0)
        return std::max<int64_t>(1, static_cast<int64_t>(configured_radius));
    return std::max<int64_t>(256, static_cast<int64_t>(stride));
}

} // namespace rxopt
} // namespace demod
} // namespace uwb
} // namespace gr
