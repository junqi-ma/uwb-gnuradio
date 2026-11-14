/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Pure C++ UCR4 wire format.
 *
 * This header is the on-disk contract of the asynchronous pipeline output
 * writer.  It is deliberately free of GNU Radio and PMT so that
 * `uwb_pipeline_output_writer.cc` can be compiled and unit-tested without
 * linking `gnuradio-runtime`.
 *
 * The layout below is field-for-field identical to `gr::uwb::Ucr4Header` in
 * <gnuradio/uwb/uwb_cir_writer.h> (the GNU Radio block).  Both headers assert
 * `sizeof(Ucr4Header) == 52`; the pipeline writer writes the same bytes so a
 * file produced by either writer is readable by the other, by
 * `testdata/uwb_radar/read_uwb_cir.m` and by the independent Python verifier.
 *
 *   magic "UCR4" | pulse_id u32 | status u16 | tap_count u16
 *   | repetition_index u16 | repetition_count u16
 *   | sfd_metric f32 | cir_peak_metric f32 | peak_tap i32 | estimator_us u32
 *   | freq_hz f64 | freq_offset_hz f64 | cir_scale f32
 *
 * followed by `tap_count` interleaved little-endian int16 (I, Q) taps.
 * Reconstruction is FC32 = SC16 * cir_scale (block floating point).
 *
 * Owner: Track 1 of
 * docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_UCR4_FORMAT_H
#define INCLUDED_GNURADIO_UWB_UWB_UCR4_FORMAT_H

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gr {
namespace uwb {
namespace pipeline {

// On-disk / UDP UCR4 header.  Packed little-endian, 52 bytes.
struct Ucr4Header {
    char magic[4];
    uint32_t pulse_id;
    uint16_t status;
    uint16_t tap_count;
    uint16_t repetition_index;
    uint16_t repetition_count;
    float sfd_metric;
    float cir_peak_metric;
    int32_t peak_tap;
    uint32_t estimator_us;
    double freq_hz;
    double freq_offset_hz;
    float cir_scale;
} __attribute__((packed));
static_assert(sizeof(Ucr4Header) == 52, "UCR4 header must be 52 bytes");

// Total bytes of one record: header + tap_count interleaved SC16 taps.
inline size_t
ucr4_record_bytes(size_t tap_count)
{
    return sizeof(Ucr4Header) + tap_count * 4;
}

// Block-floating SC16, matching CirUdpSink / UCR4 / UwbCirWriter:
//   scale = max(|I|,|Q|) / 32767;  sc16 = round(fc32 / scale) clipped.
// A non-finite or non-positive peak zero-fills the taps and returns scale 0,
// so a degenerate frame can never produce NaN in the file (the reader then
// reconstructs an all-zero record rather than NaN).
//
// NOTE: this is deliberately stricter than `gr::uwb::encode_cir_sc16` in
// uwb_cir_writer.h.  There, `std::max` does not propagate NaN, so a single
// non-finite sample in the middle of an otherwise finite frame slips through
// and is clipped to -32767 instead of triggering the documented zero-fill.
// For every all-finite input the two implementations are bit-identical; for a
// non-finite input this one does what the contract says.
inline float
encode_cir_sc16(const std::complex<float>* in,
                size_t n,
                int16_t* interleaved_iq)
{
    float peak = 0.0f;
    bool all_finite = true;
    for (size_t i = 0; i < n; ++i) {
        const float re = in[i].real();
        const float im = in[i].imag();
        if (!std::isfinite(re) || !std::isfinite(im)) {
            all_finite = false;
            break;
        }
        peak = std::max(peak, std::fabs(re));
        peak = std::max(peak, std::fabs(im));
    }
    if (!all_finite || peak <= 0.0f) {
        if (n > 0 && interleaved_iq != nullptr)
            std::memset(interleaved_iq, 0, n * 2 * sizeof(int16_t));
        return 0.0f;
    }
    constexpr float kPeak = 32767.0f;
    const float scale = peak / kPeak;
    for (size_t i = 0; i < n; ++i) {
        float re = std::nearbyint(in[i].real() / scale);
        float im = std::nearbyint(in[i].imag() / scale);
        re = std::min(kPeak, std::max(-kPeak, re));
        im = std::min(kPeak, std::max(-kPeak, im));
        interleaved_iq[2 * i] = static_cast<int16_t>(re);
        interleaved_iq[2 * i + 1] = static_cast<int16_t>(im);
    }
    return scale;
}

// Inverse of encode_cir_sc16: out[i] = SC16[i] * scale.
inline void
decode_cir_sc16(const int16_t* interleaved_iq,
                size_t n,
                float scale,
                std::complex<float>* out)
{
    for (size_t i = 0; i < n; ++i) {
        out[i] = std::complex<float>(
            static_cast<float>(interleaved_iq[2 * i]) * scale,
            static_cast<float>(interleaved_iq[2 * i + 1]) * scale);
    }
}

// Same numeric codes and names as UwbCirWriter / RadarCirStatus:
//   0 ok, 1 sfd_failed, 2 timing_failed, 3 cir_failed, 4 invalid_input.
// Anything outside the domain fails closed to "internal_error" -- an unknown
// code is never silently reported as "ok".
inline const char*
cir_status_name(uint16_t code)
{
    switch (code) {
    case 0:
        return "ok";
    case 1:
        return "sfd_failed";
    case 2:
        return "timing_failed";
    case 3:
        return "cir_failed";
    case 4:
        return "invalid_input";
    default:
        return "internal_error";
    }
}

} // namespace pipeline
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_UCR4_FORMAT_H */
