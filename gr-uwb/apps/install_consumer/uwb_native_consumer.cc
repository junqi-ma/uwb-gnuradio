/* -*- c++ -*- */
/*
 * 2026-11-15 native-cleanup installed-consumer proof.
 *
 * This program includes ONLY installed gr-uwb public headers and links ONLY
 * the installed `uwb_native` / `uwb_core` static archives (+ VOLK/Threads).
 * It must compile and run against an install prefix with NO GNU Radio
 * development headers, NO source tree and NO build tree on the include or
 * library path -- the wrapper additionally proves with readelf/ldd that the
 * resulting binary has no libgnuradio / pmt NEEDED entry.
 *
 * It exercises, on synthetic data only:
 *   - the pure resampler core (length formula + process_sc16 round trip),
 *   - the native pipeline ledger conservation contract,
 *   - the minimal SC16 stream-source contract (ReplaySc16Source over a
 *     temporary file) through the bounded pipeline with a fake result,
 *   - the fixed-tone-cancel core + decorator header inclusion,
 *   - the target-lock core state machine entry.
 *
 * Exit 0 only when every check passes.
 */

#include <gnuradio/uwb/uwb_fixed_tone_cancel_core.h>
#include <gnuradio/uwb/uwb_fixed_tone_cancel_source.h>
#include <gnuradio/uwb/uwb_rational_resampler_core.h>
#include <gnuradio/uwb/uwb_sc16_stream_source.h>
#include <gnuradio/uwb/uwb_target_lock_core.h>
#include <gnuradio/uwb/uwb_uhd_sc16_pipeline.h>

#include <cmath>
#include <complex>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace gr::uwb;

namespace {

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("%-64s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok)
        ++failures;
}

// Write a small synthetic SC16 file and read it back through the replay
// source contract.
bool replay_source_roundtrip()
{
    const std::string path = "uwb_native_consumer_fixture.sc16";
    {
        std::ofstream f(path, std::ios::binary);
        if (!f)
            return false;
        std::vector<std::complex<int16_t>> v(4096);
        for (size_t i = 0; i < v.size(); ++i)
            v[i] = {static_cast<int16_t>((i * 37) % 4000 - 2000),
                    static_cast<int16_t>((i * 91) % 4000 - 2000)};
        f.write(reinterpret_cast<const char*>(v.data()),
                static_cast<std::streamsize>(v.size() * sizeof(v[0])));
    }
    pipeline::ReplaySc16Source src(path, 737.28e6, /*paced=*/false);
    check(src.open(), "ReplaySc16Source::open");
    check(src.start_stream(), "ReplaySc16Source::start_stream");
    std::vector<std::complex<int16_t>> buf(4096);
    pipeline::StreamReadMeta meta;
    const size_t n = src.read_samples(buf.data(), buf.size(), meta);
    check(n == 4096, "ReplaySc16Source reads 4096 samples");
    const size_t n2 = src.read_samples(buf.data(), buf.size(), meta);
    check(n2 == 0 && src.is_eof(), "ReplaySc16Source reaches EOF after drain");
    src.close();
    std::remove(path.c_str());
    return n == 4096;
}

} // namespace

int main()
{
    // ---- pure resampler core: length formula + SC16 round trip ----------
    {
        // 65/48-style taps: a flat boxcar is fine for a contract check; the
        // point is the length formula and produced/consumed accounting.
        std::vector<float> taps(65 * 48, 1.0f / 48.0f);
        core::RationalResampler65_48Core rs(taps);
        const size_t N = 4800;
        const size_t expect =
            core::RationalResampler65_48Core::expected_output_length(N, taps.size());
        // (N-1)*65 + T taps, rounded up to whole Decim=48 groups.
        const size_t expect_ref =
            (static_cast<size_t>((N - 1) * 65 + taps.size()) + 47) / 48;
        check(expect == expect_ref, "expected_output_length formula");

        std::vector<std::complex<int16_t>> in(N);
        for (size_t i = 0; i < N; ++i)
            in[i] = {static_cast<int16_t>(i % 97), static_cast<int16_t>(-(int)(i % 89))};
        std::vector<std::complex<float>> out(expect + 64, std::complex<float>(0.0f, 0.0f));
        auto r = rs.process_sc16(in.data(), N, 1.0f / 32768.0f, out.data(), out.size());
        const size_t tail =
            rs.flush(out.data() + r.produced, out.size() - r.produced);
        check(r.produced > 0 && r.consumed <= N, "process_sc16 produced/consumed");
        check(r.produced + tail == expect, "process+flush == expected_output_length");
        check(rs.sc16_kernel_variant() != nullptr, "sc16_kernel_variant reports");
    }

    // ---- pipeline ledger conservation contract ---------------------------
    {
        pipeline::PipelineLedger led;
        led.offered_windows = 10;
        led.admitted_windows = 8;
        led.completed_windows = 6;
        led.processing_failed = 1;
        led.invalid_gap_after_admission = 1;
        led.dropped_admission = 1;
        led.invalid_gap_before_admission = 1;
        led.cancelled_windows = 0;
        std::string err;
        check(led.verify_invariants(&err), "PipelineLedger conservation invariant");
        if (!err.empty())
            std::printf("  ledger error: %s\n", err.c_str());
        pipeline::PipelineLedger bad;
        bad.offered_windows = 5;
        bad.admitted_windows = 9; // inconsistent
        check(!bad.verify_invariants(), "PipelineLedger rejects inconsistent state");
    }

    // ---- minimal source contract: tone decorator wraps any source --------
    {
        const std::string path = "uwb_native_consumer_tone.sc16";
        {
            std::ofstream f(path, std::ios::binary);
            std::vector<std::complex<int16_t>> v(2048, {1000, -1000});
            f.write(reinterpret_cast<const char*>(v.data()),
                    static_cast<std::streamsize>(v.size() * sizeof(v[0])));
        }
        auto inner = std::make_unique<pipeline::ReplaySc16Source>(path, 737.28e6);
        core::FixedToneCancelConfig cfg;
        cfg.enable = true;
        cfg.sample_rate = 737.28e6;
        cfg.tone_frequency_hz = -232.96e6;
        cfg.period_s = 20.0e-3;
        cfg.training_s = 0.5e-3;
        pipeline::FixedToneCancelSc16Source dec(std::move(inner), cfg, 65536, 32);
        check(dec.open(), "FixedToneCancelSc16Source::open (decorator header usable standalone)");
        dec.close();
        std::remove(path.c_str());
    }

    // ---- target-lock core entry ------------------------------------------
    {
        core::TargetLockConfig lc;
        lc.sample_rate = 737.28e6;
        lc.nominal_period_s = 5.0e-3;
        lc.verify_observations = 3;
        lc.holdover_miss_count = 3;
        lc.reacquire_miss_count = 8;
        lc.code_index = 10;
        lc.preamble_repetitions = 256;
        core::TargetLockCore lock(lc);
        check(lock.state() == core::TargetLockState::Acquire,
              "TargetLockCore starts in Acquire");
    }

    // ---- replay source over temp file (ISc16StreamSource contract) -------
    check(replay_source_roundtrip(), "replay source round trip");

    if (failures == 0) {
        std::printf("uwb_native_consumer: ALL OK (no GNU Radio, no UHD device)\n");
        return 0;
    }
    std::printf("uwb_native_consumer: %d FAILURES\n", failures);
    return 1;
}
