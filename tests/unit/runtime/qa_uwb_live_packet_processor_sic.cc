/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * QA for the optional SIC stage inside live::PacketProcessor.
 *
 * Coverage:
 *   1. --sic off is bit-for-bit the pre-SIC packet path (no SIC state, no
 *      timing split change);
 *   2. fail-open: a missing / too-weak interferer degrades to L0_Bypass and
 *      the unmodified work window still decodes the target packet;
 *   3. a synthetic target (QM35825 code 9, 64 SYNC, 4z2) + strong DW1000
 *      interferer (code 11, 128 SYNC, DW-8) collision window: SIC off fails,
 *      preamble/full/auto recover the target FCS (both dense and shaped-sparse
 *      timing kernels), and repeated windows leave no state behind.
 */

#include <boost/test/unit_test.hpp>

#include <openuwb/phy/hrp/common/profile.h>
#include <openuwb/phy/hrp/tx/hrp_mod_core.h>
#include <openuwb/runtime/live_packet_processor.h>
#include <openuwb/sic/sic_unified.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef UWB_TESTDATA_DIR
#define UWB_TESTDATA_DIR "testdata"
#endif

namespace {

using namespace openuwb;
using cf = std::complex<float>;
using sc16 = std::complex<int16_t>;

constexpr size_t kSps = 1016;

std::string testdata_path(const std::string& rel)
{
    return std::string(UWB_TESTDATA_DIR) + "/" + rel;
}

std::vector<float> load_f32(const std::string& rel)
{
    std::ifstream f(testdata_path(rel), std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + rel);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(float), 0u);
    f.seekg(0);
    std::vector<float> v(bytes / sizeof(float));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

const std::vector<float>& rx_taps()
{
    static const auto taps =
        load_f32("resampler_65_48/taps_realtime_eq16.txt");
    return taps;
}

const std::vector<float>& tx_taps()
{
    static const auto taps = load_f32("twr/m2a/taps/tx_48_65.f32");
    return taps;
}

std::vector<cf> load_cf32(const std::string& rel)
{
    std::ifstream f(testdata_path(rel), std::ios::binary | std::ios::ate);
    BOOST_REQUIRE_MESSAGE(f, "cannot open " + rel);
    const auto bytes = static_cast<size_t>(f.tellg());
    BOOST_REQUIRE_EQUAL(bytes % sizeof(cf), 0u);
    f.seekg(0);
    std::vector<cf> v(bytes / sizeof(cf));
    f.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(bytes));
    BOOST_REQUIRE(f);
    return v;
}

const std::vector<cf>& reference_sync()
{
    static const auto sync = load_cf32("reference_preamble.bin");
    BOOST_REQUIRE_EQUAL(sync.size(), kSps);
    return sync;
}

struct PacketSpec {
    size_t code;
    size_t reps;
    const char* sfd;
    size_t psdu_bytes;
    uint32_t seed;
};

struct WavePacket {
    std::vector<uint8_t> wire;
    std::vector<cf> wave;
};

size_t sfd_symbols(const char* sfd)
{
    const auto seq = demod::GetSfdSequence(sfd);
    BOOST_REQUIRE(!seq.empty());
    return seq.size();
}

WavePacket make_packet(const PacketSpec& spec)
{
    WavePacket p;
    p.wire = mod::make_random_psdu(spec.psdu_bytes - 2, spec.seed, true);

    mod::HrpModConfig cfg;
    cfg.code_index = spec.code;
    cfg.sync_repetitions = spec.reps;
    cfg.sfd_mode = spec.sfd;
    cfg.peak_amplitude = 0.8f;

    const size_t count = mod::packet_samples_998p4(
        spec.reps, sfd_symbols(spec.sfd), p.wire.size(), false, &cfg.pulse);
    mod::HrpModScratch scratch;
    scratch.reserve(127, count);
    p.wave.resize(count);
    size_t written = 0;
    BOOST_REQUIRE(mod::modulate_one(p.wire.data(), p.wire.size(), cfg, scratch,
                                    p.wave.data(), p.wave.size(), written));
    BOOST_REQUIRE_EQUAL(written, count);
    return p;
}

// Work-grid CF32 -> 737.28 MS/s native SC16 using the frozen TX taps.  This is
// the inverse of the RX 65/48 resampler the PacketProcessor runs, so the test
// exercises the real live path instead of injecting work-rate samples.
std::vector<sc16> to_native_sc16(const std::vector<cf>& work)
{
    core::RationalResampler48_65Core tx(tx_taps());
    tx.reset();
    std::vector<cf> out(tx.expected_output_length(work.size()));
    auto r = tx.process(work.data(), work.size(), out.data(), out.size());
    size_t made = r.produced;
    while (made < out.size()) {
        const size_t n = tx.flush(out.data() + made, out.size() - made);
        if (!n)
            break;
        made += n;
    }
    out.resize(made);

    std::vector<sc16> native(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        const float re = std::max(-32768.f, std::min(32767.f,
            std::round(out[i].real() * 32767.f)));
        const float im = std::max(-32768.f, std::min(32767.f,
            std::round(out[i].imag() * 32767.f)));
        native[i] = sc16(static_cast<int16_t>(re), static_cast<int16_t>(im));
    }
    return native;
}

demod::Qm35825Profile target_profile()
{
    demod::Qm35825Profile p;
    p.code_index = 9;
    p.preamble_repetitions = 64;
    p.sfd_mode = "4z2";
    p.cir_repetitions = 54;
    p.rx_timing_sparse = false;
    return p;
}

live::PacketSicOptions make_sic_options(sic::SicPolicy policy,
                                        bool shaped_sparse = false)
{
    live::PacketSicOptions options;
    options.enabled = true;
    auto& cfg = options.config;
    cfg.policy = policy;
    cfg.backend = sic::SicEstimatorBackend::Ipatov;
    cfg.comm_code = 11;       // DW1000 interferer
    cfg.sync_reps = 128;
    cfg.samples_per_symbol = kSps;
    cfg.sample_rate = 998.4e6;
    cfg.sfd_mode = "decawave"; // operator name dw-8
    cfg.fit_reps = 8;
    cfg.rx_timing_shaped_sparse = shaped_sparse;
    // Exercise the same whole-window probe openuwb_rx uses for --sic.
    cfg.rx_wide_interval_probe = true;
    cfg.rx_wide_probe_step_symbols = 42;
    cfg.rx_wide_probe_fold_symbols = 4;
    cfg.rx_wide_probe_fallback = false;
    cfg.roi_lo = 0;
    cfg.roi_hi = 0;
    cfg.sub_roi_lo = 0;
    cfg.sub_roi_hi = 0;
    cfg.observed_end_sample = 0;
    return options;
}

// One SYNC symbol of the interferer, tiled into a preamble template.  Kept
// local to the test so the PacketSicOptions contract does not depend on the
// application's resource-root resolver.
void attach_sic_templates(live::PacketSicOptions& options)
{
    static std::vector<cf> sync;
    static std::vector<cf> preamble;
    static bool ready = false;
    if (!ready) {
        mod::HrpModConfig cfg;
        cfg.code_index = options.config.comm_code;
        cfg.sync_repetitions = 1;
        cfg.sfd_mode = options.config.sfd_mode;
        cfg.peak_amplitude = 0.8f;
        mod::HrpModScratch scratch;
        const size_t cap = 64 * kSps;
        scratch.reserve(1, cap);
        std::vector<cf> prefix(cap);
        size_t n = 0;
        BOOST_REQUIRE(mod::modulate_prefix_waveform(cfg, scratch, prefix.data(),
                                                    prefix.size(), n));
        BOOST_REQUIRE_GE(n, kSps);
        sync.assign(prefix.begin(), prefix.begin() + kSps);
        preamble.reserve(options.config.sync_reps * sync.size());
        for (size_t rep = 0; rep < options.config.sync_reps; ++rep)
            preamble.insert(preamble.end(), sync.begin(), sync.end());
        ready = true;
    }
    options.templates.sync = sync.data();
    options.templates.sync_len = sync.size();
    options.templates.preamble = preamble.data();
    options.templates.preamble_len = preamble.size();
}

live::PacketRequest request_at(size_t target_work_sample, uint64_t id)
{
    live::PacketRequest req;
    req.id = id;
    req.seed = static_cast<int64_t>(target_work_sample);
    req.window_start = 0;
    req.predicted_rel_work = static_cast<int64_t>(target_work_sample);
    req.guard = 0;
    return req;
}

struct MixedWindow {
    std::vector<sc16> native;
    size_t target_work = 0;
    std::vector<uint8_t> target_wire;
};

// The target starts 512 samples after a strong DW1000 preamble and is short
// enough (16 B PSDU) that the COMPLETE target packet sits inside the
// interferer's 128-SYNC + DW-8 prefix.  L1 therefore subtracts every sample
// that corrupts the target, while L2 additionally removes the interferer
// payload.  The interferer is ~6x the target amplitude and the mixture is
// normalized before SC16 quantization.
//
// `jammer_offset` places the interferer at a different absolute work sample
// (early, mid or late in the search window) and `cfo_hz` applies an
// independent carrier offset to the interferer, so the K-symbol coherent
// probe is exercised with both a phase ramp and different probe-grid phases.
MixedWindow make_mixed_window_at(uint32_t target_seed, float interferer_gain,
                                 size_t jammer_offset, double cfo_hz)
{
    const auto target = make_packet({9, 64, "4z2", 16, target_seed});
    const auto jammer = make_packet({11, 128, "decawave", 32, 0x44u});

    const size_t q_i = jammer_offset;
    // Put the whole target packet inside the interferer's SYNC+SFD span.  L1
    // then subtracts exactly the samples that corrupt the target preamble AND
    // payload, while an L2 transaction additionally removes the interferer
    // payload.  Both policies are therefore able to recover the target.
    const size_t q_t = q_i + 512;
    const size_t n = std::max(q_t + target.wave.size(),
                              q_i + jammer.wave.size()) + 4096;

    std::vector<cf> work(n, cf(0.f, 0.f));
    for (size_t i = 0; i < target.wave.size(); ++i)
        work[q_t + i] += target.wave[i];
    const double cfo_rad_per_sample =
        2.0 * M_PI * cfo_hz / 998.4e6;
    for (size_t i = 0; i < jammer.wave.size(); ++i) {
        const double phase = cfo_rad_per_sample * static_cast<double>(i);
        const cf rot(static_cast<float>(std::cos(phase)),
                     static_cast<float>(std::sin(phase)));
        work[q_i + i] += jammer.wave[i] * interferer_gain * rot;
    }

    // Keep the SC16 quantization unclipped.
    float peak = 0.f;
    for (const auto& v : work)
        peak = std::max(peak, std::max(std::abs(v.real()), std::abs(v.imag())));
    if (peak > 0.95f) {
        const float s = 0.95f / peak;
        for (auto& v : work)
            v *= s;
    }

    MixedWindow out;
    out.native = to_native_sc16(work);
    out.target_work = q_t;
    out.target_wire = target.wire;
    return out;
}

MixedWindow make_mixed_window(uint32_t target_seed, float interferer_gain)
{
    return make_mixed_window_at(target_seed, interferer_gain, 4096, 0.0);
}

std::vector<sc16> make_target_only_native(uint32_t seed)
{
    const auto target = make_packet({9, 64, "4z2", 32, seed});
    const size_t q_t = 4096;
    std::vector<cf> work(q_t + target.wave.size() + 4096, cf(0.f, 0.f));
    for (size_t i = 0; i < target.wave.size(); ++i)
        work[q_t + i] = 0.32f * target.wave[i];
    return to_native_sc16(work);
}

bool decoded_ok(const demod::DemodResult& r)
{
    return r.status == demod::DemodStatus::Success && r.payload.fcs_pass &&
           live::verify_packet_crc(r.payload.bytes);
}

// Everything a deterministic run must reproduce.  Stage timings are excluded.
void require_same_decode(const demod::DemodResult& a,
                         const demod::DemodResult& b)
{
    BOOST_REQUIRE_EQUAL(int(a.status), int(b.status));
    BOOST_REQUIRE_EQUAL(a.payload.fcs_pass, b.payload.fcs_pass);
    BOOST_REQUIRE(a.payload.bytes == b.payload.bytes);
    BOOST_REQUIRE_EQUAL(a.phr.ok, b.phr.ok);
    BOOST_REQUIRE_EQUAL(a.phr.psdu_length, b.phr.psdu_length);
    BOOST_REQUIRE(a.phr.phr_bits == b.phr.phr_bits);
    BOOST_REQUIRE_EQUAL(a.timing.ok, b.timing.ok);
    BOOST_REQUIRE_EQUAL(a.timing.preamble_start_sample, b.timing.preamble_start_sample);
    BOOST_REQUIRE_EQUAL(a.timing.detected_peaks, b.timing.detected_peaks);
    BOOST_REQUIRE_EQUAL(a.sfd.ok, b.sfd.ok);
    BOOST_REQUIRE_EQUAL(a.sfd.sfd_start_sample, b.sfd.sfd_start_sample);
    BOOST_REQUIRE_EQUAL(a.cir.first_path_sample, b.cir.first_path_sample);
    BOOST_REQUIRE_EQUAL(a.cir.soft_chip_count, b.cir.soft_chip_count);
    BOOST_REQUIRE_CLOSE(a.cfo.cfo_hz, b.cfo.cfo_hz, 1e-6);
}

} // namespace

BOOST_AUTO_TEST_SUITE(qa_uwb_live_packet_processor_sic)

BOOST_AUTO_TEST_CASE(disabled_sic_is_packet_equivalent)
{
    const auto native = make_target_only_native(0x1234u);
    const auto profile = target_profile();

    live::PacketProcessor baseline(rx_taps(), reference_sync(), 450000);
    live::PacketProcessor off(rx_taps(), reference_sync(), 450000,
                              live::PacketSicOptions{});

    for (uint64_t id = 1; id <= 3; ++id) {
        const auto req = request_at(4096, id);
        const auto a = baseline.process(req, native.data(), native.size(),
                                        profile, false);
        const auto b = off.process(req, native.data(), native.size(),
                                   profile, false);
        require_same_decode(a.decoded, b.decoded);
        BOOST_CHECK(!b.sic_executed);
        BOOST_CHECK(b.sic_end == b.resample_end);
        BOOST_CHECK_EQUAL(std::string(b.sic_reason), "off");
    }
}

BOOST_AUTO_TEST_CASE(fail_open_without_interferer_keeps_target)
{
    const auto native = make_target_only_native(0x2345u);
    const auto profile = target_profile();
    auto options = make_sic_options(sic::SicPolicy::AutoFallback);
    attach_sic_templates(options);

    live::PacketProcessor baseline(rx_taps(), reference_sync(), 450000);
    live::PacketProcessor sic(rx_taps(), reference_sync(), 450000,
                              std::move(options));

    const auto req = request_at(4096, 11);
    const auto a = baseline.process(req, native.data(), native.size(), profile,
                                    false);
    const auto b = sic.process(req, native.data(), native.size(), profile, false);

    BOOST_REQUIRE(decoded_ok(a.decoded));
    require_same_decode(a.decoded, b.decoded);
    BOOST_CHECK(b.sic_executed);
    BOOST_CHECK(b.sic_ok);
    // No code-11 packet exists: SIC must not claim a cancellation level and
    // the unmodified work window must be the demodulated one.
    BOOST_CHECK_EQUAL(int(b.sic_level),
                      int(sic::SicCancellationLevel::L0_Bypass));
    BOOST_CHECK(!b.sic_committed);
}

BOOST_AUTO_TEST_CASE(collision_requires_sic_and_recovers_target)
{
    const auto mixed = make_mixed_window(0x3456u, 6.0f);
    const auto profile = target_profile();

    live::PacketProcessor baseline(rx_taps(), reference_sync(), 450000);
    const auto req = request_at(mixed.target_work, 21);
    const auto off = baseline.process(req, mixed.native.data(),
                                      mixed.native.size(), profile, false);
    BOOST_TEST_MESSAGE("sic off: status=" << int(off.decoded.status)
                       << " fcs=" << off.decoded.payload.fcs_pass);
    BOOST_CHECK(!decoded_ok(off.decoded));

    for (const bool shaped_sparse : {false, true}) {
        for (const auto policy : {sic::SicPolicy::PreambleOnly,
                                  sic::SicPolicy::FullPacket,
                                  sic::SicPolicy::AutoFallback}) {
            auto options = make_sic_options(policy, shaped_sparse);
            attach_sic_templates(options);
            live::PacketProcessor processor(rx_taps(), reference_sync(),
                                            450000, std::move(options));
            const auto r = processor.process(req, mixed.native.data(),
                                             mixed.native.size(), profile, false);
            BOOST_TEST_MESSAGE("sic policy=" << int(policy)
                               << " shaped=" << shaped_sparse
                               << " level=" << int(r.sic_level)
                               << " ok=" << r.sic_ok
                               << " reason=" << r.sic_reason
                               << " fcs=" << r.decoded.payload.fcs_pass
                               << " status=" << int(r.decoded.status));
            BOOST_REQUIRE(r.sic_executed);
            BOOST_REQUIRE(r.sic_ok);
            BOOST_REQUIRE(r.sic_committed);
            BOOST_REQUIRE_GE(int(r.sic_level),
                             int(sic::SicCancellationLevel::L1_Preamble));
            BOOST_REQUIRE(decoded_ok(r.decoded));
            BOOST_CHECK(r.decoded.payload.bytes == mixed.target_wire);
        }
    }
}

BOOST_AUTO_TEST_CASE(wide_probe_collision_offsets_and_cfo_recover)
{
    const auto profile = target_profile();
    // Different absolute offsets exercise different probe-grid phases and
    // backtrack distances; 50/100 kHz CFO exercise the K=4 coherent fold with
    // a real inter-symbol phase ramp.  Each geometry is reused by every policy
    // so packet synthesis is not repeated per configuration.
    const size_t offsets[3] = {4096, 98304, 196608};
    const double cfos[2] = {50e3, 100e3};
    unsigned cases = 0;
    for (size_t oi = 0; oi < 3; ++oi) {
        for (size_t ci = 0; ci < 2; ++ci) {
            const uint32_t seed = 0x7000u + static_cast<uint32_t>(oi * 10 + ci);
            const auto mixed = make_mixed_window_at(seed, 6.0f, offsets[oi],
                                                    cfos[ci]);
            for (const bool shaped_sparse : {false, true}) {
                for (const auto policy : {sic::SicPolicy::PreambleOnly,
                                          sic::SicPolicy::AutoFallback}) {
                    auto options = make_sic_options(policy, shaped_sparse);
                    attach_sic_templates(options);
                    live::PacketProcessor processor(
                        rx_taps(), reference_sync(), 450000,
                        std::move(options));
                    const auto req = request_at(
                        mixed.target_work, 300 + cases);
                    const auto r = processor.process(
                        req, mixed.native.data(), mixed.native.size(),
                        profile, false);
                    BOOST_TEST_MESSAGE("offset=" << offsets[oi]
                                       << " cfo=" << cfos[ci]
                                       << " shaped=" << shaped_sparse
                                       << " policy=" << int(policy)
                                       << " level=" << int(r.sic_level)
                                       << " committed=" << r.sic_committed
                                       << " reason=" << r.sic_reason
                                       << " fcs=" << r.decoded.payload.fcs_pass);
                    BOOST_REQUIRE_MESSAGE(r.sic_executed,
                                          "case " << cases << " SIC did not run");
                    BOOST_REQUIRE_MESSAGE(r.sic_ok,
                                          "case " << cases << " SIC failed: "
                                          << r.sic_reason);
                    BOOST_REQUIRE_MESSAGE(r.sic_committed,
                                          "case " << cases << " SIC did not commit");
                    BOOST_REQUIRE_MESSAGE(
                        int(r.sic_level) >= int(sic::SicCancellationLevel::L1_Preamble),
                        "case " << cases << " level below L1");
                    BOOST_REQUIRE_MESSAGE(decoded_ok(r.decoded),
                                          "case " << cases << " target FCS failed");
                    BOOST_CHECK(r.decoded.payload.bytes == mixed.target_wire);
                    ++cases;
                }
            }
        }
    }
    BOOST_CHECK_EQUAL(cases, 24u);
}

BOOST_AUTO_TEST_CASE(shaped_sparse_collision_recovers_repeated_targets)
{
    const auto profile = target_profile();
    auto options = make_sic_options(sic::SicPolicy::AutoFallback, true);
    attach_sic_templates(options);
    live::PacketProcessor processor(rx_taps(), reference_sync(), 450000,
                                    std::move(options));

    // Same geometry as the dense collision test, but with the shaped-sparse
    // timing kernel enabled for the whole-window blind search.  Eight distinct
    // targets must recover 8/8 FCS; this is the openuwb_rx configuration.
    unsigned recovered = 0;
    for (uint32_t k = 0; k < 8; ++k) {
        const auto mixed = make_mixed_window(0x6000u + k, 6.0f);
        const auto req = request_at(mixed.target_work, 200 + k);
        const auto r = processor.process(req, mixed.native.data(),
                                         mixed.native.size(), profile, false);
        BOOST_TEST_MESSAGE("shaped sparse packet " << k
                           << " level=" << int(r.sic_level)
                           << " ok=" << r.sic_ok
                           << " reason=" << r.sic_reason
                           << " fcs=" << r.decoded.payload.fcs_pass);
        BOOST_REQUIRE_MESSAGE(r.sic_executed, "packet " << k
                                << " SIC did not run");
        BOOST_REQUIRE_MESSAGE(r.sic_ok, "packet " << k << " SIC failed: "
                                << r.sic_reason);
        BOOST_REQUIRE_MESSAGE(r.sic_committed, "packet " << k
                                << " SIC did not commit");
        BOOST_REQUIRE_MESSAGE(decoded_ok(r.decoded), "packet " << k
                                << " target FCS failed");
        BOOST_CHECK(r.decoded.payload.bytes == mixed.target_wire);
        ++recovered;
    }
    BOOST_CHECK_EQUAL(recovered, 8u);
}

BOOST_AUTO_TEST_CASE(repeated_sic_windows_have_no_state_residue)
{
    const auto profile = target_profile();
    auto options = make_sic_options(sic::SicPolicy::AutoFallback);
    attach_sic_templates(options);
    live::PacketProcessor processor(rx_taps(), reference_sync(), 450000,
                                    std::move(options));
    live::PacketProcessor clean_processor(rx_taps(), reference_sync(), 450000);

    // Different payloads on every repetition so a stale PSDU/pulse cache would
    // show up as an FCS mismatch, not just a re-decoded old packet.
    for (uint32_t k = 0; k < 8; ++k) {
        const auto clean = make_target_only_native(0x5000u + k);
        const auto clean_req = request_at(4096, 90 + k);
        const auto cr = clean_processor.process(clean_req, clean.data(),
                                                clean.size(), profile, false);
        BOOST_REQUIRE_MESSAGE(decoded_ok(cr.decoded),
                              "clean target fixture " << k << " does not decode");
        const auto mixed = make_mixed_window(0x5000u + k, 6.0f);
        const auto req = request_at(mixed.target_work, 100 + k);
        const auto r = processor.process(req, mixed.native.data(),
                                         mixed.native.size(), profile, false);
        BOOST_REQUIRE_MESSAGE(r.sic_ok, "packet " << k << " SIC failed: "
                                << r.sic_reason);
        BOOST_REQUIRE_MESSAGE(r.sic_committed, "packet " << k
                                << " SIC did not commit");
        BOOST_REQUIRE_MESSAGE(decoded_ok(r.decoded), "packet " << k
                                << " target FCS failed");
        BOOST_CHECK(r.decoded.payload.bytes == mixed.target_wire);
    }
}

BOOST_AUTO_TEST_SUITE_END()
