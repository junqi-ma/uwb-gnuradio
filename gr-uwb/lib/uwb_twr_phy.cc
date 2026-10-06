/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * M2-A: offline native-rate PHY closed loop for TWR frames.
 *
 * Implementation of every declaration in
 * `include/gnuradio/uwb/uwb_twr_phy.h` (owned by the coordinator).  This file
 * is the ONE production pipeline; it never re-implements a core.  Each stage
 * calls the repository's existing header-only core:
 *
 *   hrp_mod      : gr::uwb::mod::modulate_one (+ append_ieee_fcs exactly once)
 *   tx_resample  : gr::uwb::core::RationalResampler48_65Core / <32,65>
 *   rx_resample  : gr::uwb::core::RationalResampler65_48Core / <65,32>
 *   demod        : gr::uwb::demod::core::demodulate_one
 *   codec        : gr::uwb::twr::encode_into / mac_payload_from_psdu / decode
 *
 * Numeric contract: docs/twr/M2-A_G0接口与数字坐标.md (G0) §3 (causal upfirdn,
 * N==0 -> 0, Lout = ceil(((N-1)*L+T)/M), reset between bursts), §5 (integer
 * sample-grid bookkeeping; never a RMARKER), §6 (the ONLY success criterion),
 * §7 (SC16 clamp(llround), round-half-away-from-zero, saturation counted,
 * NaN/Inf refused), and appendix A.1/A.2 (prepared context, stage coordinates).
 *
 * EVIDENCE BOUNDARY: `measurement_valid` is always false.  Nothing here
 * produces a distance, an RMARKER, a first path or a hardware timestamp.
 *
 * ---- demod acquisition, in one paragraph ---------------------------------
 * G0 §5 / task §3 require the decoder NOT be handed the exact packet start.
 * `m2a_demod_work` therefore calls `demodulate_one(..., predicted_start = -1,
 * window_start = 0, ...)`.  The frozen demod core seeds its bounded acquisition
 * at the buffer MIDPOINT (see `stage_timing`: seed = n/2 when predicted_start
 * < 0) and searches seed +/- timing_search_margin.  A long TWR frame is wider
 * than that window, so the frame is placed at a RECORDED offset inside a
 * zero-guarded search buffer: `[Gf zeros][work][Gb zeros]` with the guard
 * chosen so the core's midpoint seed lands inside the frame's preamble.  The
 * core still performs the actual correlation/SFD/CFO/decode; no start, no
 * payload and no propagation truth is passed as a detection result.  A small
 * set of bounded placements is tried (recorded in the stage trace) so leading
 * whitespace is covered.  The winning placement's guards and search window are
 * recorded in `M2aStageTrace` (G0 A.2) and in `M2aResult::detail`.
 *
 * ---- prepared context (G0 A.1) -------------------------------------------
 * `M2aContext` freezes the profile/taps/capacity/format and OWNS the TX/RX
 * resampler cores, the modulation scratch, the demod scratch, the demod
 * template and a reusable search workspace.  Every buffer is reserved to the
 * frozen capacity in `prepare()`; the per-frame path never constructs a core
 * and never grows a buffer (the demod core's own internal temporaries are the
 * documented exception -- they are not refactored).  `prepare()` is atomic:
 * on failure the context is left UNPREPARED and `run()` refuses rather than
 * running half a pipeline.  `reset()` clears filter history/phase and the
 * previous frame's bytes without rebuilding a core.  `m2a_native_roundtrip()`
 * is a COLD wrapper that prepares a context and runs exactly one frame.
 *
 * Enum handling follows the M0.1 N07 rule: every enum has an `xxx_is_known()`
 * written as a switch with NO `default`, and a domain test runs before the
 * switch that consumes the value.
 */

#include <gnuradio/uwb/uwb_twr_phy.h>

#include <gnuradio/uwb/uwb_demod_core.h>
#include <gnuradio/uwb/uwb_hrp_mod_core.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace gr {
namespace uwb {
namespace twr {

// The ONE FCS append site in this file (defined below, after the anonymous
// helpers that use it).  Declared here so the stage body can call it.
static std::vector<uint8_t> m2a_psdu_with_fcs(const uint8_t* mac, size_t mac_len);

namespace {

// ---------------------------------------------------------------------------
// Small total helpers
// ---------------------------------------------------------------------------

bool all_finite(const std::vector<float>& v)
{
    for (float x : v)
        if (!std::isfinite(x))
            return false;
    return true;
}

// DC sum contract of G0 §3.3: taps sum to `interp` so the effective gain is 1.
bool taps_dc_ok(const std::vector<float>& taps, double interp, std::string& why)
{
    if (taps.size() < 2) {
        why = "taps must hold at least 2 coefficients";
        return false;
    }
    if (!all_finite(taps)) {
        why = "taps contain a non-finite coefficient";
        return false;
    }
    double sum = 0.0;
    for (float t : taps)
        sum += static_cast<double>(t);
    if (std::fabs(sum - interp) > 1e-2 * interp) {
        why = "taps DC sum is not " + std::to_string(interp) + " (got " +
              std::to_string(sum) + ")";
        return false;
    }
    return true;
}

std::string hex_bytes(const std::vector<uint8_t>& v)
{
    static const char* kHex = "0123456789abcdef";
    std::string s;
    s.reserve(v.size() * 2);
    for (uint8_t b : v) {
        s.push_back(kHex[(b >> 4) & 0xf]);
        s.push_back(kHex[b & 0xf]);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Stage-coordinate fill (G0 A.2)
// ---------------------------------------------------------------------------
//
// A resampler stage's trace is described in its OUTPUT (interpolated) grid:
// origin 0, trim 0 (causal full convolution, no crop), implicit zero padding
// T-1 split symmetrically by the linear-phase group delay, and the group delay
// `0.5*(T-1)` in the INTERPOLATED domain (equivalent to `(T-1)/(2L)` input
// samples).  These are sample-grid bookkeeping, never a hardware constant.
void fill_resample_trace(M2aStageTrace& t, const char* name, double rate_hz,
                         uint32_t l, uint32_t m, size_t n_in, size_t n_out,
                         size_t taps)
{
    t = M2aStageTrace{};
    t.name = name;
    t.unit = "samples";
    t.rate_hz = rate_hz;
    t.interp = l;
    t.decim = m;
    t.origin = 0;
    t.phase = 0;
    t.trim = 0;
    const int64_t span = static_cast<int64_t>(taps) - 1;
    t.pad_front = span / 2;
    t.pad_back = span - t.pad_front;
    t.padding = span; // == pad_front + pad_back
    t.filter_delay = 0.5 * static_cast<double>(span);
    t.in_count = n_in;
    t.out_count = n_out;
    t.valid_from = 0;
    t.valid_to = n_out;
}

// Causal full-convolution resample through an EXISTING core (G0 §3).  The core
// is reset first so every call is an independent burst regardless of what the
// previous frame did; process() is then followed by repeated flush() until
// flush_complete().  No core is constructed here (the prepared context reuses
// its owned cores) and `out` is only ever assigned within its reserved
// capacity, so the hot path never reallocates.
template <class Core>
bool resample_with_core(Core& core,
                        const std::complex<float>* in,
                        size_t n,
                        size_t expected_out,
                        std::vector<std::complex<float>>& out,
                        std::string& why)
{
    if (!in && n > 0) {
        why = "null input pointer";
        return false;
    }
    try {
        out.assign(expected_out, std::complex<float>(0.0f, 0.0f));
        core.reset();
        const size_t produced0 =
            core.process(in, n, out.data(), expected_out).produced;
        if (produced0 > expected_out) {
            why = "resampler produced more than the expected length";
            return false;
        }
        size_t produced = produced0;
        size_t flushes = 0;
        while (!core.flush_complete()) {
            const size_t room = expected_out - produced;
            if (room == 0)
                break;
            const size_t got = core.flush(out.data() + produced, room);
            produced += got;
            if (got == 0)
                break;
            if (++flushes > expected_out + 8) {
                why = "resampler flush did not complete";
                return false;
            }
        }
        if (produced != expected_out) {
            why = "resampler produced " + std::to_string(produced) + " of " +
                  std::to_string(expected_out) + " samples";
            return false;
        }
        out.resize(produced);
        return true;
    } catch (const std::exception& e) {
        out.clear();
        why = std::string("resampler threw: ") + e.what();
        return false;
    } catch (...) {
        out.clear();
        why = "resampler threw an unknown exception";
        return false;
    }
}

// One-shot stage helper: constructs a fresh core (an independent burst, G0
// §3.4).  The prepared context does NOT use this path.
template <class Core>
bool resample_causal(const std::complex<float>* in,
                     size_t n,
                     const std::vector<float>& taps,
                     size_t expected_out,
                     std::vector<std::complex<float>>& out,
                     std::string& why)
{
    try {
        Core core(taps.data(), taps.size());
        if (!resample_with_core(core, in, n, expected_out, out, why))
            return false;
        core.reset();
        return true;
    } catch (const std::exception& e) {
        out.clear();
        why = std::string("resampler threw: ") + e.what();
        return false;
    } catch (...) {
        out.clear();
        why = "resampler threw an unknown exception";
        return false;
    }
}

// The known preamble shape used as the demod matched-filter template: the first
// SYNC symbol of a modulated (preamble + PHR) waveform.  A zero-length PSDU is
// valid for modulate_one and yields the same SYNC/SFD preamble as any frame, so
// the template is derived from the frozen modulator, not from the received
// buffer (task §3: "first SYNC symbol of the modulated work waveform").
bool build_sync_template(const M2aConfig& cfg,
                         std::vector<std::complex<float>>& tmpl,
                         std::string& why)
{
    const size_t n_sfd = m2a_sfd_symbols(cfg.sfd_mode);
    if (n_sfd == 0) {
        why = "unknown SFD mode '" + cfg.sfd_mode + "'";
        return false;
    }
    if (!radar_meta::code_index_supported(cfg.code_index)) {
        why = "code_index is not supported by this build";
        return false;
    }
    if (!radar_meta::sync_reps_supported(cfg.sync_repetitions)) {
        why = "sync_repetitions is not supported by this build";
        return false;
    }
    if (cfg.insert_sts) {
        why = "insert_sts is out of scope for M2-A";
        return false;
    }
    if (!(cfg.peak_amplitude > 0.0f) || !std::isfinite(cfg.peak_amplitude)) {
        why = "peak_amplitude must be finite and > 0";
        return false;
    }

    mod::HrpModConfig mc = cfg.hrp_config();
    const size_t n_out = mod::packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, 0, cfg.insert_sts, &mc.pulse);
    if (n_out < demod::kQm35SamplesPerSymbol) {
        why = "template waveform shorter than one SYNC symbol";
        return false;
    }
    if (n_out > kM2aMaxSamples) {
        why = "capacity_exceeded: template waveform exceeds kM2aMaxSamples";
        return false;
    }

    mod::HrpModScratch scratch;
    scratch.reserve(0, n_out);
    std::vector<std::complex<float>> wf(n_out);
    size_t got = 0;
    if (!mod::modulate_one(nullptr, 0, mc, scratch, wf.data(), wf.size(), got) ||
        got < demod::kQm35SamplesPerSymbol) {
        why = "modulate_one refused the template preamble";
        return false;
    }
    tmpl.assign(wf.begin(),
                wf.begin() + static_cast<std::ptrdiff_t>(demod::kQm35SamplesPerSymbol));
    return true;
}

// Progress rank of a demod status, used only to keep the most advanced failed
// attempt's DemodResult for diagnostics when no placement decodes.
int demod_progress(demod::DemodStatus s)
{
    switch (s) {
    case demod::DemodStatus::Success:
        return 100;
    case demod::DemodStatus::FcsFailed:
        return 9;
    case demod::DemodStatus::PayloadFailed:
        return 8;
    case demod::DemodStatus::PhrFailed:
        return 7;
    case demod::DemodStatus::SfdFailed:
        return 6;
    case demod::DemodStatus::CirFailed:
        return 5;
    case demod::DemodStatus::CfoFailed:
        return 4;
    case demod::DemodStatus::TimingFailed:
        return 3;
    case demod::DemodStatus::InvalidInput:
        return 1;
    case demod::DemodStatus::QueueFull:
        return 0;
    case demod::DemodStatus::InternalError:
        return 0;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Stage bodies that accept caller-owned resources (G0 A.1).  The public
// one-shot stage functions below are thin wrappers that build those resources
// once; the prepared context passes its frozen, reserved resources instead.
// ---------------------------------------------------------------------------

bool modulate_with_scratch(const uint8_t* mac_bytes,
                           size_t mac_len,
                           const M2aConfig& cfg,
                           mod::HrpModScratch& scratch,
                           std::vector<std::complex<float>>& out,
                           M2aStageTrace& trace,
                           std::string& why)
{
    out.clear();
    trace = M2aStageTrace{};
    trace.name = "hrp_mod";
    // MIXED stage: the PSDU input is BYTES, the work waveform is SAMPLES.
    trace.in_unit = "bytes";
    trace.out_unit = "samples";
    trace.unit = "samples"; // == out_unit (compatibility alias)
    trace.rate_hz = cfg.tx_rate_hz();
    trace.interp = 1;
    trace.decim = 1;
    trace.origin = 0;

    if (!mac_bytes && mac_len > 0) {
        why = "null MAC payload pointer";
        return false;
    }
    if (!radar_meta::code_index_supported(cfg.code_index)) {
        why = "code_index is not supported by this build";
        return false;
    }
    if (!radar_meta::sync_reps_supported(cfg.sync_repetitions)) {
        why = "sync_repetitions is not supported by this build";
        return false;
    }
    if (cfg.insert_sts) {
        why = "insert_sts is out of scope for M2-A";
        return false;
    }
    if (!(cfg.peak_amplitude > 0.0f) || !std::isfinite(cfg.peak_amplitude)) {
        why = "peak_amplitude must be finite and > 0";
        return false;
    }
    const size_t n_sfd = m2a_sfd_symbols(cfg.sfd_mode);
    if (n_sfd == 0) {
        why = "unknown SFD mode '" + cfg.sfd_mode + "'";
        return false;
    }

    // The HRP layer appends the ONE FCS.  The codec never writes it (G0 §4).
    const std::vector<uint8_t> psdu = m2a_psdu_with_fcs(mac_bytes, mac_len);

    mod::HrpModConfig mc = cfg.hrp_config();
    const size_t n_out = mod::packet_samples_998p4(
        cfg.sync_repetitions, n_sfd, psdu.size(), cfg.insert_sts, &mc.pulse);
    if (n_out == 0) {
        why = "modulated packet length is zero";
        return false;
    }
    if (n_out > kM2aMaxSamples) {
        why = "capacity_exceeded: modulated length " + std::to_string(n_out) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    // Reserve only grows; a context scratch pre-provisioned to the frozen
    // capacity is left untouched, so no allocation happens in the hot path.
    scratch.reserve(psdu.size(), n_out);
    out.resize(n_out);
    size_t got = 0;
    if (!mod::modulate_one(psdu.data(), psdu.size(), mc, scratch, out.data(),
                           out.size(), got) ||
        got != n_out) {
        out.clear();
        why = "modulate_one refused the frame";
        return false;
    }
    out.resize(got);
    trace.in_count = psdu.size();
    trace.out_count = got;
    trace.valid_from = 0;
    trace.valid_to = got;
    return true;
}

bool demod_with_resources(const std::complex<float>* work,
                          size_t n,
                          const M2aConfig& cfg,
                          const std::vector<std::complex<float>>& tmpl,
                          demod::core::DemodScratch& scratch,
                          std::vector<std::complex<float>>& search,
                          demod::DemodResult& out,
                          M2aStageTrace& trace,
                          std::string& why)
{
    out = demod::DemodResult{};
    trace = M2aStageTrace{};
    trace.name = "demod";
    trace.unit = "samples";
    trace.rate_hz = cfg.tx_rate_hz();
    trace.interp = 1;
    trace.decim = 1;
    trace.origin = 0;

    if (!work || n == 0) {
        why = "empty demod input";
        return false;
    }
    if (n > kM2aMaxSamples) {
        why = "capacity_exceeded: demod input " + std::to_string(n) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    demod::Qm35825Profile profile = cfg.demod_profile();
    const size_t margin = profile.timing_search_margin;

    // Candidate seed positions within the input.  The buffer
    //   [Gf zeros][work][Gb zeros]
    // has midpoint seed = (Gf + n + Gb)/2; to place the seed at input offset p
    // we need Gf - Gb = n - 2p.  p == 0 is the primary placement (seed exactly
    // at the input start); the direct buffer (seed at n/2) and a bounded sweep
    // of p cover leading whitespace.  Every placement is recorded.  This is a
    // fixed-size list (no per-frame allocation): at most 2 + 24 entries.
    constexpr size_t kStep = 2048;
    size_t seed_buf[32];
    size_t seed_n = 0;
    seed_buf[seed_n++] = 0;
    if (n >= 2)
        seed_buf[seed_n++] = n / 2;
    const size_t max_seed = std::min<size_t>(n > 0 ? n - 1 : 0, 24 * kStep);
    for (size_t p = kStep; p <= max_seed; p += kStep)
        seed_buf[seed_n++] = p;
    std::sort(seed_buf, seed_buf + seed_n);
    seed_n = static_cast<size_t>(std::unique(seed_buf, seed_buf + seed_n) -
                                 seed_buf);

    // Largest guarded placement; reserve the workspace once (a no-op for the
    // prepared context, which already reserved to the frozen capacity).  Every
    // guard length is counted against kM2aMaxSamples.
    size_t max_buf = 0;
    for (size_t i = 0; i < seed_n; ++i) {
        const size_t p = seed_buf[i];
        const size_t gsum = (2 * p <= n) ? (n - 2 * p) : (2 * p - n);
        const size_t buf_n = gsum + n;
        if (buf_n <= kM2aMaxSamples && buf_n > max_buf)
            max_buf = buf_n;
    }
    if (max_buf == 0) {
        why = "capacity_exceeded: no bounded demod placement fit kM2aMaxSamples";
        return false;
    }
    if (search.capacity() < max_buf)
        search.reserve(max_buf);

    bool have_result = false;
    int best_progress = -1;
    size_t best_gf = 0;
    size_t best_gb = 0;
    size_t best_buf_n = 0;

    for (size_t i = 0; i < seed_n; ++i) {
        const size_t p = seed_buf[i];
        size_t gf = 0;
        size_t gb = 0;
        if (2 * p <= n)
            gf = n - 2 * p;
        else
            gb = 2 * p - n;
        const size_t buf_n = gf + n + gb;
        if (buf_n > kM2aMaxSamples)
            continue;

        search.assign(buf_n, std::complex<float>(0.0f, 0.0f));
        std::memcpy(search.data() + gf, work, n * sizeof(std::complex<float>));
        scratch.reserve(buf_n);
        demod::DemodResult r;
        try {
            r = demod::core::demodulate_one(search.data(), buf_n, profile,
                                            /*packet_id=*/1,
                                            /*predicted_start=*/-1,
                                            /*window_start=*/0, tmpl, scratch);
        } catch (const std::exception& e) {
            why = std::string("demodulator threw: ") + e.what();
            return false;
        } catch (...) {
            why = "demodulator threw an unknown exception";
            return false;
        }

        const int prog = demod_progress(r.status);
        if (!have_result || prog > best_progress) {
            have_result = true;
            best_progress = prog;
            out = std::move(r);
            best_gf = gf;
            best_gb = gb;
            best_buf_n = buf_n;
        }
        if (out.status == demod::DemodStatus::Success)
            break;
    }

    if (!have_result) {
        why = "capacity_exceeded: no bounded demod placement fit kM2aMaxSamples";
        return false;
    }

    // The core reports coordinates inside the zero-guarded search buffer.  The
    // input `work` starts at offset `best_gf` there.  Re-base the DIAGNOSTIC
    // decode coordinates onto the UN-GUARDED work_rx grid (G0 §5, A.2:
    // sample-grid bookkeeping only, never a RMARKER / first path).  A
    // coordinate that lands in either guard, or a failed decode, is reported as
    // invalid (-1) rather than as a padded index mistaken for an input index.
    auto rebase = [&](int64_t v) -> int64_t {
        if (v < 0)
            return -1;
        if (v < static_cast<int64_t>(best_gf))
            return -1; // inside the front guard
        const int64_t r = v - static_cast<int64_t>(best_gf);
        if (r >= static_cast<int64_t>(n))
            return -1; // inside the back guard / past the input
        return r;
    };
    if (out.status == demod::DemodStatus::Success) {
        out.timing.preamble_start_sample =
            rebase(out.timing.preamble_start_sample);
        out.sfd.sfd_start_sample = rebase(out.sfd.sfd_start_sample);
        // ... and the SFD END, which is a guarded index too (review E).
        out.sfd.sfd_end_sample = rebase(out.sfd.sfd_end_sample);
    } else {
        out.timing.preamble_start_sample = -1;
        out.sfd.sfd_start_sample = -1;
        out.sfd.sfd_end_sample = -1;
    }

    const size_t seed = best_buf_n / 2;
    const size_t roi_lo = (seed > margin) ? seed - margin : 0;
    const size_t roi_hi = std::min<size_t>(best_buf_n, seed + margin);

    // The stage's own grid is the un-guarded work_rx grid: origin 0, in_count
    // is the input length (the guards are the pad_front/pad_back).  The search
    // window is reported separately in the guarded search-buffer grid.
    trace.in_unit = "samples";
    trace.out_unit = "bytes"; // decoded PSDU length
    trace.unit = "bytes";
    trace.in_count = n;
    trace.out_count = out.payload.bytes.size();
    trace.pad_front = static_cast<int64_t>(best_gf);
    trace.pad_back = static_cast<int64_t>(best_gb);
    trace.padding = static_cast<int64_t>(best_gf + best_gb);
    trace.search_guard_front = static_cast<int64_t>(best_gf);
    trace.search_guard_back = static_cast<int64_t>(best_gb);
    trace.search_roi_from = roi_lo;
    trace.search_roi_to = roi_hi;
    trace.valid_from = roi_lo;
    trace.valid_to = roi_hi;

    // On success the caller reconstructs the search-window summary from the
    // trace; keep `why` empty.
    why.clear();
    return true;
}

} // namespace

// ===========================================================================
// to_string()
// ===========================================================================

std::string M2aStageTrace::to_string() const
{
    std::ostringstream os;
    os << name << "{unit=" << unit << ",rate=" << rate_hz << ",L=" << interp
       << ",M=" << decim << ",origin=" << origin << ",in=" << in_count
       << ",out=" << out_count << ",phase=" << phase << ",trim=" << trim
       << ",pad_front=" << pad_front << ",pad_back=" << pad_back
       << ",padding=" << padding << ",filter_delay=" << filter_delay
       << ",valid=[" << valid_from << "," << valid_to << ")"
       << ",guard=[" << search_guard_front << "," << search_guard_back
       << "),roi=[" << search_roi_from << "," << search_roi_to << ")}";
    return os.str();
}

std::string M2aResult::to_string() const
{
    std::ostringstream os;
    os << "M2aResult{ok=" << (ok ? "true" : "false")
       << ",status=" << m2a_status_to_string(status)
       << ",rate=" << m2a_native_rate_to_string(native_rate)
       << ",fmt=" << m2a_iq_format_to_string(iq_format)
       << ",fn=" << static_cast<int>(function_code)
       << ",mac=" << hex_bytes(mac_bytes) << ",psdu_len=" << psdu_bytes.size()
       << ",fcs=0x" << std::hex << fcs << std::dec
       << ",work_tx=" << work_tx_samples << ",native=" << native_samples
       << ",work_rx=" << work_rx_samples << ",sc16_sat=" << sc16_saturated
       << ",demod=" << static_cast<int>(demod_status)
       << ",fcs_pass=" << (fcs_pass ? "true" : "false")
       << ",bytes_exact=" << (bytes_exact ? "true" : "false")
       << ",measurement_valid=" << (measurement_valid ? "true" : "false")
       << "}";
    return os.str();
}

// ===========================================================================
// The ONE FCS append site in this file (G0 §4).
// ===========================================================================

// `modulate_one` consumes a complete PSDU and never appends the FCS itself, so
// the HRP layer owns it -- and it must be appended exactly once per buffer.
// Both the modulated PSDU and the PSDU recorded in the report are built here,
// so the structural guard in qa_uwb_twr_frame.cc still sees a single
// production site.
static std::vector<uint8_t> m2a_psdu_with_fcs(const uint8_t* mac, size_t mac_len)
{
    std::vector<uint8_t> psdu(mac, mac + mac_len);
    mod::append_ieee_fcs(psdu);
    return psdu;
}

// ===========================================================================
// Stage: codec MAC bytes -> append FCS once -> modulate_one -> work IQ
// ===========================================================================

bool m2a_modulate_to_work(const uint8_t* mac_bytes,
                          size_t mac_len,
                          const M2aConfig& cfg,
                          std::vector<std::complex<float>>& out,
                          M2aStageTrace& trace,
                          std::string& why)
{
    mod::HrpModScratch scratch;
    return modulate_with_scratch(mac_bytes, mac_len, cfg, scratch, out, trace,
                                 why);
}

// ===========================================================================
// Stage: work IQ -> native IQ (TX <L,65>)
// ===========================================================================

bool m2a_tx_resample(const std::complex<float>* work,
                     size_t n,
                     const M2aConfig& cfg,
                     std::vector<std::complex<float>>& native,
                     M2aStageTrace& trace,
                     std::string& why)
{
    native.clear();
    trace = M2aStageTrace{};
    trace.name = "tx_resample";
    trace.unit = "samples";
    trace.rate_hz = cfg.native_rate_hz();
    trace.origin = 0;

    uint32_t l = 0;
    uint32_t m = 0;
    if (!m2a_rate_tx_lm(cfg.native_rate, l, m)) {
        why = "native_rate is not a member of M2aNativeRate";
        return false;
    }
    trace.interp = l;
    trace.decim = m;

    const size_t taps = cfg.tx_taps.size();
    if (!taps_dc_ok(cfg.tx_taps, static_cast<double>(l), why))
        return false;
    if (n > kM2aMaxSamples) {
        why = "capacity_exceeded: work input " + std::to_string(n) +
              " exceeds kM2aMaxSamples";
        return false;
    }
    trace.in_count = n;
    if (n == 0) {
        // G0 §3.2: N == 0 is ZERO output with no state change.
        trace.out_count = 0;
        return true;
    }
    const size_t lout = m2a_resampled_length(n, taps, l, m);
    if (lout == 0) {
        why = "resampler output length is zero";
        return false;
    }
    if (lout > kM2aMaxSamples) {
        why = "capacity_exceeded: native output " + std::to_string(lout) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    bool ok = false;
    switch (cfg.native_rate) {
    case M2aNativeRate::Uc200_737280000:
        ok = resample_causal<core::RationalResampler48_65Core>(
            work, n, cfg.tx_taps, lout, native, why);
        break;
    case M2aNativeRate::Cg400_491520000:
        ok = resample_causal<core::RationalResampler32_65Core>(
            work, n, cfg.tx_taps, lout, native, why);
        break;
    }
    if (!ok) {
        native.clear();
        return false;
    }

    fill_resample_trace(trace, "tx_resample", cfg.native_rate_hz(), l, m, n,
                        lout, taps);
    return true;
}

// ===========================================================================
// Stage: native IQ -> work IQ (RX <65,L>)
// ===========================================================================

bool m2a_rx_resample(const std::complex<float>* native,
                     size_t n,
                     const M2aConfig& cfg,
                     std::vector<std::complex<float>>& work,
                     M2aStageTrace& trace,
                     std::string& why)
{
    work.clear();
    trace = M2aStageTrace{};
    trace.name = "rx_resample";
    trace.unit = "samples";
    trace.rate_hz = cfg.tx_rate_hz();
    trace.origin = 0;

    uint32_t l = 0;
    uint32_t m = 0;
    if (!m2a_rate_tx_lm(cfg.native_rate, l, m)) {
        why = "native_rate is not a member of M2aNativeRate";
        return false;
    }
    // RX is the reciprocal direction: <65, L>.
    const uint32_t rl = 65;
    const uint32_t rm = l;
    trace.interp = rl;
    trace.decim = rm;

    const size_t taps = cfg.rx_taps.size();
    if (!taps_dc_ok(cfg.rx_taps, 65.0, why))
        return false;
    if (n > kM2aMaxSamples) {
        why = "capacity_exceeded: native input " + std::to_string(n) +
              " exceeds kM2aMaxSamples";
        return false;
    }
    trace.in_count = n;
    if (n == 0) {
        trace.out_count = 0;
        return true;
    }
    const size_t lout = m2a_resampled_length(n, taps, rl, rm);
    if (lout == 0) {
        why = "resampler output length is zero";
        return false;
    }
    if (lout > kM2aMaxSamples) {
        why = "capacity_exceeded: work output " + std::to_string(lout) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    bool ok = false;
    switch (cfg.native_rate) {
    case M2aNativeRate::Uc200_737280000:
        ok = resample_causal<core::RationalResampler65_48Core>(
            native, n, cfg.rx_taps, lout, work, why);
        break;
    case M2aNativeRate::Cg400_491520000:
        ok = resample_causal<core::RationalResampler65_32Core>(
            native, n, cfg.rx_taps, lout, work, why);
        break;
    }
    if (!ok) {
        work.clear();
        return false;
    }

    fill_resample_trace(trace, "rx_resample", cfg.tx_rate_hz(), rl, rm, n, lout,
                        taps);
    return true;
}

// ===========================================================================
// Stage: CF32 -> SC16 -> CF32 (G0 §7)
// ===========================================================================

bool m2a_sc16_roundtrip(const std::complex<float>* in,
                        size_t n,
                        float scale,
                        std::vector<std::complex<float>>& out,
                        size_t& saturated,
                        std::string& why)
{
    out.clear();
    saturated = 0;
    if (!in && n > 0) {
        why = "null input pointer";
        return false;
    }
    if (!(scale > 0.0f) || !std::isfinite(scale)) {
        why = "SC16 scale must be finite and > 0";
        return false;
    }
    if (n > kM2aMaxSamples) {
        why = "capacity_exceeded: SC16 input " + std::to_string(n) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    out.resize(n);
    const double s = static_cast<double>(scale);
    const float pos = static_cast<float>(32767.0 / s);
    const float neg = static_cast<float>(-32768.0 / s);
    auto quantise = [&](float v) -> float {
        // clamp(llround(x*S), -32768, 32767), round-half-away-from-zero.
        const double t = static_cast<double>(v) * s;
        long long r;
        if (t >= 32768.0)
            r = 32768;
        else if (t < -32769.0)
            r = -32769;
        else
            r = std::llround(t);
        if (r > 32767) {
            ++saturated;
            return pos;
        }
        if (r < -32768) {
            ++saturated;
            return neg;
        }
        return static_cast<float>(static_cast<double>(r) / s);
    };

    for (size_t i = 0; i < n; ++i) {
        const std::complex<float> x = in[i];
        if (!std::isfinite(x.real()) || !std::isfinite(x.imag())) {
            out.clear();
            saturated = 0;
            why = "non-finite input sample refused by SC16";
            return false;
        }
        out[i] = std::complex<float>(quantise(x.real()), quantise(x.imag()));
    }
    return true;
}

// ===========================================================================
// Stage: work IQ -> demodulate_one
// ===========================================================================

bool m2a_demod_work(const std::complex<float>* work,
                    size_t n,
                    const M2aConfig& cfg,
                    demod::DemodResult& out,
                    M2aStageTrace& trace,
                    std::string& why)
{
    out = demod::DemodResult{};
    trace = M2aStageTrace{};
    trace.name = "demod";
    trace.unit = "samples";
    trace.rate_hz = cfg.tx_rate_hz();
    trace.interp = 1;
    trace.decim = 1;
    trace.origin = 0;

    if (!work || n == 0) {
        why = "empty demod input";
        return false;
    }
    if (n > kM2aMaxSamples) {
        why = "capacity_exceeded: demod input " + std::to_string(n) +
              " exceeds kM2aMaxSamples";
        return false;
    }

    std::vector<std::complex<float>> tmpl;
    if (!build_sync_template(cfg, tmpl, why))
        return false;

    demod::core::DemodScratch scratch;
    std::vector<std::complex<float>> search;
    return demod_with_resources(work, n, cfg, tmpl, scratch, search, out, trace,
                                why);
}

// ===========================================================================
// Prepared context (G0 A.1)
// ===========================================================================

struct M2aContext::Impl {
    bool prepared = false;
    M2aConfig cfg;

    // Owned cores.  Exactly one TX and one RX are non-null after prepare();
    // both directions are the SAME causal upfirdn core, only the template
    // parameters differ.
    std::unique_ptr<core::RationalResampler48_65Core> tx_uc;
    std::unique_ptr<core::RationalResampler32_65Core> tx_cg;
    std::unique_ptr<core::RationalResampler65_48Core> rx_uc;
    std::unique_ptr<core::RationalResampler65_32Core> rx_cg;

    // Owned scratch + template, built once.
    mod::HrpModScratch mod_scratch;
    demod::core::DemodScratch demod_scratch;
    std::vector<std::complex<float>> tmpl;

    // Owned buffers, reserved to the frozen capacity in prepare().
    std::vector<std::complex<float>> work_tx;
    std::vector<std::complex<float>> native;
    std::vector<std::complex<float>> quant;
    std::vector<std::complex<float>> work_rx;
    std::vector<std::complex<float>> search;
    FrameScratch frame_scratch;

    // Frozen capacities (G0 A.1 rule 4).
    size_t cap_work_tx = 0;
    size_t cap_native = 0;
    size_t cap_work_rx = 0;
    size_t cap_search = 0;

    bool build(const M2aConfig& c, std::string& why);
    bool run(const Frame& frame, const FrameProfile& profile, M2aResult& out,
             std::string& why);
    void reset();
};

bool M2aContext::Impl::build(const M2aConfig& c, std::string& why)
{
    prepared = false;

    std::string vwhy;
    if (!c.is_valid(vwhy)) {
        why = vwhy;
        return false;
    }

    uint32_t l = 0;
    uint32_t m = 0;
    if (!m2a_rate_tx_lm(c.native_rate, l, m)) {
        why = "no TX ratio for this native rate";
        return false;
    }

    // Build the matched-filter template once.  This also validates the SFD mode
    // (which M2aConfig::is_valid does not) before any capacity arithmetic.
    std::vector<std::complex<float>> tmpl_local;
    if (!build_sync_template(c, tmpl_local, why))
        return false;

    // Capacity: every guard, tail and buffer length is counted against the
    // frozen kM2aMaxSamples bound, not just the raw frame (G0 A.1 rule 4).
    const size_t max_psdu = radar_meta::kMaxPsduBytes;
    const size_t work_tx_max = m2a_expected_work_samples(max_psdu, c);
    if (work_tx_max == 0 || work_tx_max > kM2aMaxSamples) {
        why = "capacity_exceeded: max work waveform " +
              std::to_string(work_tx_max) + " exceeds kM2aMaxSamples";
        return false;
    }
    const size_t native_max =
        m2a_resampled_length(work_tx_max, c.tx_taps.size(), l, m);
    if (native_max == 0 || native_max > kM2aMaxSamples) {
        why = "capacity_exceeded: max native waveform " +
              std::to_string(native_max) + " exceeds kM2aMaxSamples";
        return false;
    }
    const size_t work_rx_max =
        m2a_resampled_length(native_max, c.rx_taps.size(), 65, l);
    if (work_rx_max == 0 || work_rx_max > kM2aMaxSamples) {
        why = "capacity_exceeded: max work_rx waveform " +
              std::to_string(work_rx_max) + " exceeds kM2aMaxSamples";
        return false;
    }
    // Search buffer = [front guard][work_rx][back guard].  The largest bounded
    // placement (p == 0) is exactly 2*work_rx, so the guards are counted here.
    const size_t search_max = 2 * work_rx_max;
    if (search_max > kM2aMaxSamples) {
        why = "capacity_exceeded: guarded demod search buffer " +
              std::to_string(search_max) + " exceeds kM2aMaxSamples";
        return false;
    }

    // Cores (same template parameters as the one-shot stage helpers).
    try {
        if (c.native_rate == M2aNativeRate::Uc200_737280000) {
            tx_uc.reset(new core::RationalResampler48_65Core(c.tx_taps.data(),
                                                             c.tx_taps.size()));
            rx_uc.reset(new core::RationalResampler65_48Core(c.rx_taps.data(),
                                                             c.rx_taps.size()));
        } else {
            tx_cg.reset(new core::RationalResampler32_65Core(c.tx_taps.data(),
                                                             c.tx_taps.size()));
            rx_cg.reset(new core::RationalResampler65_32Core(c.rx_taps.data(),
                                                             c.rx_taps.size()));
        }
    } catch (const std::exception& e) {
        why = std::string("resampler construction failed: ") + e.what();
        return false;
    } catch (...) {
        why = "resampler construction failed";
        return false;
    }

    // Reserve every owned buffer/scratch to its frozen maximum ONCE.
    mod_scratch.reserve(max_psdu, work_tx_max);
    demod_scratch.reserve(search_max);
    work_tx.reserve(work_tx_max);
    native.reserve(native_max);
    quant.reserve(native_max);
    work_rx.reserve(work_rx_max);
    search.reserve(search_max);

    cfg = c;
    tmpl = std::move(tmpl_local);
    cap_work_tx = work_tx_max;
    cap_native = native_max;
    cap_work_rx = work_rx_max;
    cap_search = search_max;
    prepared = true;
    return true;
}

void M2aContext::Impl::reset()
{
    // Clear filter history / phase and the previous frame's bytes WITHOUT
    // rebuilding a core or freeing a buffer (G0 A.1 rule 5).
    if (tx_uc)
        tx_uc->reset();
    if (tx_cg)
        tx_cg->reset();
    if (rx_uc)
        rx_uc->reset();
    if (rx_cg)
        rx_cg->reset();
    std::memset(frame_scratch.bytes, 0, sizeof(frame_scratch.bytes));
    work_tx.clear();
    native.clear();
    quant.clear();
    work_rx.clear();
    search.clear();
}

bool M2aContext::Impl::run(const Frame& frame,
                           const FrameProfile& profile,
                           M2aResult& out,
                           std::string& why)
{
    out = M2aResult{};
    out.native_rate = cfg.native_rate;
    out.iq_format = cfg.iq_format;
    out.measurement_valid = false;

    if (!prepared) {
        out.status = M2aStatus::InvalidConfig;
        out.detail = "M2aContext is not prepared";
        why = out.detail;
        return false;
    }

    // --- 1. config / frame validation (fail-closed) ------------------------
    {
        std::string cwhy;
        if (!cfg.is_valid(cwhy)) {
            out.status = M2aStatus::InvalidConfig;
            out.detail = cwhy;
            why = cwhy;
            return false;
        }
    }
    {
        std::string fwhy;
        if (!frame_profile_validate(profile, fwhy)) {
            out.status = M2aStatus::InvalidFrame;
            out.detail = fwhy;
            why = fwhy;
            return false;
        }
    }
    if (!frame_type_is_implemented(frame.function_code)) {
        out.status = M2aStatus::InvalidFrame;
        out.detail = "frame type is not implemented in frame v1";
        why = out.detail;
        return false;
    }

    // --- 2. codec: MAC bytes (NO FCS) --------------------------------------
    size_t mac_len = 0;
    std::string ferr;
    FrameError fcode = FrameError::None;
    if (!encode_into(frame, profile, frame_scratch.bytes, FrameScratch::capacity,
                     mac_len, ferr, &fcode)) {
        out.status = M2aStatus::InvalidFrame;
        out.detail = ferr;
        why = ferr;
        return false;
    }
    out.function_code = static_cast<uint8_t>(frame.function_code);
    out.mac_bytes.assign(frame_scratch.bytes, frame_scratch.bytes + mac_len);

    // The PSDU for the report is MAC + the same single FCS the HRP layer
    // appends inside modulate_with_scratch(): both go through the one helper,
    // so the modulator receives exactly one FCS (G0 §4).
    out.psdu_bytes = m2a_psdu_with_fcs(out.mac_bytes.data(), out.mac_bytes.size());
    out.fcs = static_cast<uint16_t>(
        static_cast<uint16_t>(out.psdu_bytes[mac_len]) |
        static_cast<uint16_t>(static_cast<uint16_t>(out.psdu_bytes[mac_len + 1])
                              << 8));
    out.expected_psdu_length = out.psdu_bytes.size();
    out.stages.clear();

    // Per-frame capacity guard: the frame must fit the frozen work capacity.
    {
        const size_t n_work = m2a_expected_work_samples(out.psdu_bytes.size(), cfg);
        if (n_work == 0 || n_work > cap_work_tx) {
            out.status = M2aStatus::CapacityExceeded;
            out.detail = "capacity_exceeded: frame needs " + std::to_string(n_work) +
                         " work samples, prepared capacity is " +
                         std::to_string(cap_work_tx);
            why = out.detail;
            return false;
        }
    }

    // --- 3. modulate to work grid (owned scratch/buffer) -------------------
    {
        M2aStageTrace t;
        std::string swhy;
        if (!modulate_with_scratch(out.mac_bytes.data(), out.mac_bytes.size(),
                                   cfg, mod_scratch, work_tx, t, swhy)) {
            out.status = swhy.compare(0, 17, "capacity_exceeded") == 0
                             ? M2aStatus::CapacityExceeded
                             : M2aStatus::ModulateFailed;
            out.detail = swhy;
            why = swhy;
            return false;
        }
        out.work_tx_samples = work_tx.size();
        out.stages.push_back(t);
    }

    // --- 4. TX resample (work -> native, owned core) -----------------------
    uint32_t l = 0;
    uint32_t m = 0;
    (void)m2a_rate_tx_lm(cfg.native_rate, l, m);
    {
        const size_t lout =
            m2a_resampled_length(work_tx.size(), cfg.tx_taps.size(), l, m);
        if (lout == 0 || lout > cap_native) {
            out.status = M2aStatus::CapacityExceeded;
            out.detail = "capacity_exceeded: native length " + std::to_string(lout) +
                         " exceeds prepared capacity " + std::to_string(cap_native);
            why = out.detail;
            return false;
        }
        std::string swhy;
        bool ok = false;
        switch (cfg.native_rate) {
        case M2aNativeRate::Uc200_737280000:
            ok = resample_with_core(*tx_uc, work_tx.data(), work_tx.size(), lout,
                                    native, swhy);
            break;
        case M2aNativeRate::Cg400_491520000:
            ok = resample_with_core(*tx_cg, work_tx.data(), work_tx.size(), lout,
                                    native, swhy);
            break;
        }
        if (!ok) {
            out.status = swhy.compare(0, 17, "capacity_exceeded") == 0
                             ? M2aStatus::CapacityExceeded
                             : M2aStatus::TxResampleFailed;
            out.detail = swhy;
            why = swhy;
            return false;
        }
        M2aStageTrace t;
        fill_resample_trace(t, "tx_resample", cfg.native_rate_hz(), l, m,
                            work_tx.size(), native.size(), cfg.tx_taps.size());
        out.native_samples = native.size();
        out.stages.push_back(t);
    }

    // --- 5. optional CF32 -> SC16 -> CF32 on the NATIVE grid ---------------
    if (cfg.iq_format == M2aIqFormat::Sc16) {
        size_t saturated = 0;
        std::string swhy;
        if (!m2a_sc16_roundtrip(native.data(), native.size(),
                                cfg.effective_sc16_scale(), quant, saturated,
                                swhy)) {
            out.status = M2aStatus::QuantiseFailed;
            out.detail = swhy;
            why = swhy;
            return false;
        }
        out.sc16_saturated = saturated;
        native.swap(quant);
    }

    // --- 6. RX resample (native -> work, owned core) -----------------------
    {
        const uint32_t rl = 65;
        const uint32_t rm = l;
        const size_t lout =
            m2a_resampled_length(native.size(), cfg.rx_taps.size(), rl, rm);
        if (lout == 0 || lout > cap_work_rx) {
            out.status = M2aStatus::CapacityExceeded;
            out.detail = "capacity_exceeded: work_rx length " + std::to_string(lout) +
                         " exceeds prepared capacity " + std::to_string(cap_work_rx);
            why = out.detail;
            return false;
        }
        std::string swhy;
        bool ok = false;
        switch (cfg.native_rate) {
        case M2aNativeRate::Uc200_737280000:
            ok = resample_with_core(*rx_uc, native.data(), native.size(), lout,
                                    work_rx, swhy);
            break;
        case M2aNativeRate::Cg400_491520000:
            ok = resample_with_core(*rx_cg, native.data(), native.size(), lout,
                                    work_rx, swhy);
            break;
        }
        if (!ok) {
            out.status = swhy.compare(0, 17, "capacity_exceeded") == 0
                             ? M2aStatus::CapacityExceeded
                             : M2aStatus::RxResampleFailed;
            out.detail = swhy;
            why = swhy;
            return false;
        }
        M2aStageTrace t;
        fill_resample_trace(t, "rx_resample", cfg.tx_rate_hz(), rl, rm,
                            native.size(), work_rx.size(), cfg.rx_taps.size());
        out.work_rx_samples = work_rx.size();
        out.stages.push_back(t);
    }

    // --- 7. demodulate (owned template/scratch/search) ---------------------
    demod::DemodResult dr;
    M2aStageTrace dt;
    {
        std::string swhy;
        if (!demod_with_resources(work_rx.data(), work_rx.size(), cfg, tmpl,
                                  demod_scratch, search, dr, dt, swhy)) {
            out.status = swhy.compare(0, 17, "capacity_exceeded") == 0
                             ? M2aStatus::CapacityExceeded
                             : M2aStatus::DemodFailed;
            out.detail = swhy;
            why = swhy;
            return false;
        }
        out.stages.push_back(dt);
    }

    out.demod_status = dr.status;
    out.fcs_pass = dr.payload.fcs_pass;
    out.decoded_bytes = dr.payload.bytes;
    out.sfd_start_sample = dr.sfd.sfd_start_sample;
    out.packet_start_sample = dr.timing.preamble_start_sample;

    {
        std::ostringstream os;
        os << "demod_search{guard_front=" << dt.search_guard_front
           << ",guard_back=" << dt.search_guard_back
           << ",buffer=" << (dt.in_count + dt.padding) << ",roi=["
           << dt.search_roi_from << "," << dt.search_roi_to
           << "),predicted_start=-1,window_start=0}";
        out.detail = os.str();
    }

    // --- 8. the ONLY success criterion (G0 §6 / review B3) -----------------
    auto fail = [&](M2aStatus s, const std::string& d) {
        out.ok = false;
        out.status = s;
        out.detail = out.detail.empty() ? d : (d + "; " + out.detail);
        why = d;
        return false;
    };

    if (dr.status == demod::DemodStatus::FcsFailed) {
        return fail(M2aStatus::FcsFailed,
                    "demod decoded a frame whose FCS failed");
    }
    if (dr.status != demod::DemodStatus::Success) {
        return fail(M2aStatus::DemodFailed,
                    std::string("demod status is not Success: ") +
                        std::to_string(static_cast<int>(dr.status)));
    }
    if (!dr.payload.fcs_pass)
        return fail(M2aStatus::FcsFailed, "demod FCS check failed");
    if (dr.payload.bytes.size() != out.expected_psdu_length)
        return fail(M2aStatus::LengthMismatch,
                    "decoded PSDU length " +
                        std::to_string(dr.payload.bytes.size()) +
                        " != expected " +
                        std::to_string(out.expected_psdu_length));
    if (dr.payload.bytes != out.psdu_bytes)
        return fail(M2aStatus::BytesMismatch,
                    "decoded PSDU bytes differ from the input PSDU");

    // --- 9. codec decode of the MAC payload (FCS stripped) -----------------
    const uint8_t* mac = nullptr;
    size_t mac_n = 0;
    std::string derr;
    FrameError dcode = FrameError::None;
    if (!mac_payload_from_psdu(dr.payload.bytes.data(), dr.payload.bytes.size(),
                               profile, mac, mac_n, derr, &dcode)) {
        return fail(M2aStatus::InternalError,
                    "mac_payload_from_psdu refused the decoded PSDU: " + derr);
    }
    Frame decoded;
    if (!decode(mac, mac_n, profile, decoded, derr, &dcode)) {
        return fail(M2aStatus::InternalError,
                    "codec decode refused the decoded MAC payload: " + derr);
    }

    // --- 10. success --------------------------------------------------------
    out.ok = true;
    out.status = M2aStatus::Ok;
    out.bytes_exact = true;
    out.max_abs_error = 0.0;
    out.relative_l2 = 0.0;
    out.measurement_valid = false; // ALWAYS false in M2-A
    why.clear();
    return true;
}

M2aContext::M2aContext()
    : d_impl(new Impl())
{
}

M2aContext::~M2aContext()
{
    delete d_impl;
    d_impl = nullptr;
}

bool M2aContext::prepare(const M2aConfig& cfg, std::string& why)
{
    std::unique_ptr<Impl> next(new Impl());
    if (!next->build(cfg, why)) {
        // Atomic failure: the context becomes UNPREPARED and is not left with a
        // half-updated config; a following run() refuses instead of running.
        if (d_impl)
            d_impl->prepared = false;
        return false;
    }
    delete d_impl;
    d_impl = next.release();
    return true;
}

bool M2aContext::prepared() const
{
    return d_impl != nullptr && d_impl->prepared;
}

void M2aContext::reset()
{
    if (d_impl)
        d_impl->reset();
}

const M2aConfig& M2aContext::config() const
{
    return d_impl->cfg;
}

bool M2aContext::run(const Frame& frame,
                     const FrameProfile& profile,
                     M2aResult& out,
                     std::string& why)
{
    if (!d_impl) {
        out = M2aResult{};
        out.status = M2aStatus::InvalidConfig;
        out.detail = "M2aContext is not prepared";
        out.measurement_valid = false;
        why = out.detail;
        return false;
    }
    return d_impl->run(frame, profile, out, why);
}

// ===========================================================================
// The closed loop (COLD wrapper, G0 A.1)
// ===========================================================================
//
// Encodes `frame` with `profile`, runs the full chain above, and reports
// whether the bytes survived.  `ok` is true ONLY under the G0 §6 / review B3
// criterion:
//
//     demod status == Success AND fcs_pass AND
//     decoded PSDU length == expected AND bytes byte-exact
//
// Any payload FALLBACK candidate, FcsFailed, or length mismatch yields
// ok == false with the corresponding M2aStatus.  The result never claims a
// distance, a RMARKER, a first path, or hardware evidence.

bool m2a_native_roundtrip(const Frame& frame,
                          const FrameProfile& profile,
                          const M2aConfig& cfg,
                          M2aResult& out,
                          std::string& why)
{
    out = M2aResult{};
    out.native_rate = cfg.native_rate;
    out.iq_format = cfg.iq_format;
    out.measurement_valid = false;

    M2aContext ctx;
    std::string pwhy;
    if (!ctx.prepare(cfg, pwhy)) {
        out.status = pwhy.compare(0, 17, "capacity_exceeded") == 0
                         ? M2aStatus::CapacityExceeded
                         : M2aStatus::InvalidConfig;
        out.detail = pwhy;
        why = pwhy;
        return false;
    }
    return ctx.run(frame, profile, out, why);
}

} // namespace twr
} // namespace uwb
} // namespace gr
