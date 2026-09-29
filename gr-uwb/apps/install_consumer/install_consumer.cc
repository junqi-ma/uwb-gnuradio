/* -*- c++ -*- */
/*
 * External consumer for the INSTALLED gr-uwb TWR API.
 *
 * Purpose
 * -------
 * M0 review R8 / P2 proved that the TWR headers and the Python module were
 * never installed, so `ctest` passing said nothing about the installed API.
 * A test that compiles inside the source tree cannot detect that: it sees the
 * files where they already are.
 *
 * So this program is deliberately built OUTSIDE the project, against nothing
 * but the install prefix:
 *
 *   - its include path is the prefix's include directory only, never the
 *     source tree;
 *   - its Python path is the prefix's configured Python install directory
 *     only (dist-packages on a stock local install), never site-packages by
 *     assumption;
 *   - it does not link the OOT module's build directory, so the header-only
 *     API is exercised on its own, exactly as an external consumer would.
 *
 * It checks the things that were actually missing:
 *   1. every public TWR header is includable from the prefix, and the contract
 *      they describe is usable (build a config, look it up in the capability
 *      whitelist, encode a frame, do timestamp arithmetic);
 *   2. the ranging-admission entry point (`uwb_twr_tof_input.h`, review defect
 *      N05) is not merely includable but USABLE: the consumer builds a fully
 *      evidenced RMARKER pair, shows `admit_range_capable_time()` admits it,
 *      shows `admit_ranging_interval()` yields the exact interval, and shows
 *      the default admission context admits NOTHING (default deny);
 *   3. `import uwb.twr_config` works from the prefix and validates the same
 *      profile the C++ side does, against the CURRENT schema (`twr-config/2`).
 *
 * If a header is missing from install(FILES ...) or twr_config.py is missing
 * from GR_PYTHON_INSTALL(...), this fails to compile / fails at import, which
 * is the point.
 *
 * Build (see run_install_consumer.sh):
 *   c++ -std=c++17 -I<prefix>/include install_consumer.cc -o consumer
 *   PYTHONPATH=<prefix>/<configured python dir> python3 install_consumer.py
 * (the configured Python dir is dist-packages on a stock local install, not
 * the site-packages this comment used to hardcode -- review defect N05).
 */

#include <gnuradio/uwb/uwb_twr_capability_evidence.h>
#include <gnuradio/uwb/uwb_twr_config.h>
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_math.h>
#include <gnuradio/uwb/uwb_twr_timestamp.h>
#include <gnuradio/uwb/uwb_twr_tof_input.h>
#include <gnuradio/uwb/uwb_twr_types.h>

#include <cstdio>
#include <string>

namespace twr = gr::uwb::twr;
namespace ev = gr::uwb::twr::evidence;

// The exact correction masks the TWR contract requires for each marker
// family.  Written out here from the requirement text (REQ-TIME-02 / 03)
// rather than read back from the header, so the consumer proves the contract
// and not merely its own reflection.
constexpr uint32_t kRmarkerRxBits = twr::kCorrectionRxSampleToFirstPath |
                                    twr::kCorrectionWindowCrop |
                                    twr::kCorrectionSampleRateConversion |
                                    twr::kCorrectionFirstPathFraction |
                                    twr::kCorrectionWaveformGeometry |
                                    twr::kCorrectionRmarkerOffset |
                                    twr::kCorrectionFirstPathQualityGate;
constexpr uint32_t kRmarkerTxBits = twr::kCorrectionWaveformGeometry |
                                    twr::kCorrectionTxCommandToAir |
                                    twr::kCorrectionDelayedTxQuantization;

static int g_failures = 0;

static void
check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok)
        ++g_failures;
}

int
main()
{
    std::printf("gr-uwb installed TWR API consumer\n");

    // ---- 1. the capability whitelist is reachable and is DEFAULT DENY -----
    const auto& caps = twr::capabilities();
    std::printf("  schema=%s profile=%s phy_rows=%zu\n",
                caps.schema_version.c_str(),
                caps.profile_version.c_str(),
                caps.phy_matrix.size());
    // twr-config/2 is the CURRENT schema: frame.frame_profile became required
    // and the insufficient-phr-rate constraint was removed (M0.1).  A stale
    // "twr-config/1" expectation here is exactly review defect N05.
    check(caps.schema_version == "twr-config/2", "schema_version is current (twr-config/2)");
    check(!caps.phy_matrix.empty(), "capability whitelist is populated");
    // The M0 profile: ch5 / 64 MHz PRF / 6.81 Mb/s / ranging bit required.
    check(caps.channels.size() == 1 && caps.channels[0] == 5,
          "channel whitelist is {5}");
    check(!caps.sts_supported, "STS is out of scope (fail-closed)");
    check(caps.ranging_bit_required, "ranging bit is required");

    // A 64 SYNC / 4z2 / code 9 combination must be admitted ...
    const auto ok_lookup = caps.lookup_phy(737280000.0, 9, 64,
                                           twr::SfdMode::R4z2, 127, true);
    check(ok_lookup.allowed, "measured 64-SYNC/4z2/code9 row is admitted");
    // ... and 128 SYNC must be rejected, with a reason, not defaulted.
    const auto bad_lookup = caps.lookup_phy(737280000.0, 9, 128,
                                            twr::SfdMode::R4z2, 127, true);
    check(!bad_lookup.allowed, "128 SYNC is rejected (DEFAULT DENY)");
    check(!bad_lookup.reason.empty(), "the rejection carries a reason");
    std::printf("        reason: %s\n", bad_lookup.reason.c_str());

    // ---- 2. the evidence model (M0.1) is installed and usable -------------
    // The M0 matrix establishes work_decode_verified and nothing else.  An
    // installed consumer must be able to SEE that, and must not be able to
    // use such a row to produce a range.
    ev::SourceRecord rec;
    rec.path = "work_direct_998p4";
    rec.result = "PASS";
    rec.whitelist = "supported";
    rec.sync_repetitions = 64;
    const ev::RowEvidence e = ev::derive_row_evidence(rec);
    check(e.established == ev::Level::WorkDecodeVerified,
          "a supported M0 row is work_decode_verified");
    check(!ev::allows(e.established, ev::Use::Ranging),
          "work-decode evidence does NOT license ranging (fail-closed)");
    check(!ev::allows(e.established, ev::Use::VendorInterop),
          "work-decode evidence does NOT license vendor interop");
    check(ev::not_established_list(e.established).find("toa_verified") !=
              std::string::npos,
          "the absent ToA level is listed explicitly");
    // 128 SYNC is a limit of THIS build's software, not of the hardware.
    ev::SourceRecord len_rec;
    len_rec.path = "work_direct_998p4";
    len_rec.result = "PASS";
    len_rec.whitelist = "unsupported";
    len_rec.sync_repetitions = 128;
    const ev::RowEvidence len = ev::derive_row_evidence(len_rec);
    check(len.reject_scope == ev::reject_scope_current_software(),
          "128 SYNC rejection is scoped to this build's software");
    check(len.reject_scope_note.find("not_a_hardware_claim") !=
              std::string::npos,
          "the rejection explicitly disclaims a hardware verdict");
    // The provenance hasher is installed and correct.
    check(ev::Sha256::hex_of("abc") ==
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f"
              "20015ad",
          "SHA-256 matches the FIPS vector (provenance output hash works)");

    // ---- 3. the frame codec is installed and usable -----------------------
    // The frame length comes from the codec, not from a geometry the caller
    // supplies: frame_psdu_bytes(FrameGeometry, ...) deliberately IGNORES its
    // geometry argument and delegates to FrameProfileGeometry /
    // frame_length_for().  So the numbers are asked of the two-argument form
    // and cross-checked against the codec, and a caller-supplied geometry that
    // disagrees does NOT change the answer.
    const uint32_t poll = twr::frame_psdu_bytes(twr::FrameType::Poll,
                                                twr::FcsAppender::PhyLayer);
    const uint32_t resp = twr::frame_psdu_bytes(twr::FrameType::Response,
                                                twr::FcsAppender::PhyLayer);
    const uint32_t fin = twr::frame_psdu_bytes(twr::FrameType::Final,
                                               twr::FcsAppender::PhyLayer);
    std::printf("  frame v1 MAC payload: Poll=%u Response=%u Final=%u B\n",
                poll, resp, fin);
    check(poll == 14 && resp == 24 && fin == 29,
          "frame v1 MAC payload is 14 B header + 40-bit timestamps");
    // The FCS is appended by exactly ONE layer, so the on-air PSDU is +2.
    check(twr::frame_psdu_bytes(twr::FrameType::Poll,
                                twr::FcsAppender::MacLayer) == poll + 2,
          "the FCS belongs to exactly one layer (2 bytes, not two)");

    // A caller-supplied geometry must not be able to change a frame length.
    {
        twr::FrameGeometry bogus;
        bogus.mac_header_bytes = 7; // the M0 7-byte header that was wrong
        bogus.timestamp_bytes = 5;
        bogus.phr_bytes = 2;
        check(twr::frame_psdu_bytes(bogus, twr::FrameType::Poll,
                                    twr::FcsAppender::PhyLayer) == poll,
              "a caller-supplied geometry cannot override the codec length");
    }

    // The codec itself, reached through the installed frame header.
    {
        twr::FrameProfile profile;
        check(twr::frame_profile_for(twr::FrameProfileId::TwrV1, profile),
              "the frame v1 profile is resolvable");
        twr::FrameProfileGeometry pg;
        pg.id = twr::FrameProfileId::TwrV1;
        pg.fcs_owner = twr::FcsOwner::PhyLayer;
        const size_t from_codec =
            pg.mac_payload_bytes(twr::FrameType::Response, profile);
        check(from_codec == resp, "config and codec agree on the Response length");
        std::printf("  frame v1 profile=%s version=%u timestamp=%u bits"
                    " (unit %g Hz), max_psdu=%zu B\n",
                    twr::frame_profile_id_to_string(twr::FrameProfileId::TwrV1),
                    profile.version, profile.timestamp_bits,
                    profile.timestamp_unit_hz, profile.max_psdu_bytes);
        check(profile.timestamp_bits == 40,
              "frame v1 carries 40-bit timestamps");
        check(twr::timestamp_bytes(profile) == 5,
              "a 40-bit timestamp is 5 bytes on the wire");
        // R3: the 14-byte header is the codec's own constant, returned by a
        // METHOD so no second geometry can exist elsewhere in the system.
        check(pg.mac_header_bytes() == 14,
              "the frame v1 MAC header is 14 B, owned by the codec");
        check(pg.mac_footer_bytes() == 0,
              "frame v1 has no MAC footer");
        check(pg.mac_fcs_bytes() == 0,
              "frame v1 reserves no FCS inside the MAC PSDU");
    }
    check(twr::frame_type_timestamp_count(twr::FrameType::Poll) == 0 &&
              twr::frame_type_timestamp_count(twr::FrameType::Response) == 2 &&
              twr::frame_type_timestamp_count(twr::FrameType::Final) == 3,
          "timestamp counts per frame type");

    // ---- 4. the timestamp / ToF arithmetic is installed and usable --------
    // 737.28 MHz -> 1 tick = 1.356336805... ns, i.e. a tick is NOT an integer
    // number of nanoseconds.  The installed header must be able to say so.
    const double tick_ns = 1e9 / 737280000.0;
    std::printf("  737.28 MHz tick = %.9f ns\n", tick_ns);
    check(tick_ns > 1.356 && tick_ns < 1.357, "tick period is ~1.356 ns");
    check(tick_ns != std::floor(tick_ns),
          "a tick is NOT an integer ns (so a lossy ns projection is wrong)");
    check(twr::is_allowed_native_rate(737280000.0) &&
              twr::is_allowed_native_rate(491520000.0),
          "the two native rates are recognised");
    check(!twr::is_allowed_native_rate(998.4e6),
          "the work grid is not a native rate");

    // ---- 5. the ranging-admission entry point (M0.1 / R6) ----------------
    // uwb_twr_tof_input.h was the one public TWR header the install rules
    // omitted.  It is EXERCISED here, not merely included: a fully evidenced
    // RMARKER pair must be admitted and yield the exact tick interval, while
    // a default admission context (no calibration, no reference instant, no
    // first-path decision) must admit nothing.
    {
        twr::ClockDomain domain;
        check(twr::ClockDomain::make("consumer_dw1000_ch5_uus", 499.2e6 * 128.0, 7u,
                                     40u, domain),
              "the admission header builds a clock domain");

        twr::Timestamp rx;
        twr::Timestamp tx;
        check(twr::Timestamp::from_ticks(9000,
                                         domain,
                                         twr::TimestampMarker::RmarkerRx,
                                         twr::TimestampSource::HardwareMeasured,
                                         kRmarkerRxBits,
                                         rx),
              "an RX RMARKER timestamp is constructible");
        check(twr::Timestamp::from_ticks(5000,
                                         domain,
                                         twr::TimestampMarker::RmarkerTx,
                                         twr::TimestampSource::HardwareMeasured,
                                         kRmarkerTxBits,
                                         tx),
              "a TX RMARKER timestamp is constructible");

        const std::string cal_id = "cal-consumer-dw1000-ch5-uus-r1";
        check(twr::apply_calibration_ticks(rx, cal_id, 0, 0, 0u, 0u) ==
                  twr::CalibrationResult::Applied,
              "the RX timestamp is calibrated exactly once");
        check(twr::apply_calibration_ticks(tx, cal_id, 0, 0, 0u, 0u) ==
                  twr::CalibrationResult::Applied,
              "the TX timestamp is calibrated exactly once");

        twr::CalibrationStamp cal;
        cal.id = cal_id;
        cal.calibrated_epoch = 7u;
        cal.valid_from_ticks = 1000;
        cal.valid_until_ticks = 2000;
        twr::CalibrationApplication app;
        app.calibration_id = cal_id;
        app.result = twr::CalibrationResult::Applied;
        cal.applications.push_back(app);

        twr::RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = twr::FirstPathQuality::passed(18.5, 9.0, 0.82);

        // DEFAULT DENY: the whole point of the gate.  A context that was
        // never filled in carries no calibration, no reference instant and no
        // first-path decision, and must admit nothing rather than defaulting
        // to "fine".
        twr::RangeAdmissionContext empty;
        const twr::RangeAdmission denied = twr::admit_range_capable_time(rx, empty);
        check(!denied.admitted && !denied.value.has_value(),
              "a default admission context admits NOTHING (default deny)");

        // R6: a host capture coordinate is refused however many correction
        // bits it claims, because it is not an on-air instant.
        twr::Timestamp raw;
        const bool raw_made =
            twr::Timestamp::from_ticks(1000,
                                       domain,
                                       twr::TimestampMarker::UhdRxFirstIqSample,
                                       twr::TimestampSource::HardwareMeasured,
                                       twr::kCorrectionAll,
                                       raw);
        check(raw_made && !twr::admit_range_capable_time(raw, ctx).admitted,
              "a raw UhdRxFirstIqSample is refused even with all bits set (R6)");

        const twr::RangeAdmission one = twr::admit_range_capable_time(rx, ctx);
        check(one.admitted && one.value.has_value(),
              "a fully-evidenced RX RMARKER is admitted");

        const twr::RangingIntervalAdmission pair =
            twr::admit_ranging_interval(rx, tx, ctx);
        check(pair.admitted && pair.value.has_value() &&
                  pair.value->interval().ticks == 4000,
              "admit_ranging_interval yields the exact 4000-tick interval");
        int64_t num = 0;
        int64_t den = 0;
        check(pair.value.has_value() && pair.value->exact_ratio(num, den) && num == 4000 &&
                  den == 1,
              "the admitted interval is an exact tick-space rational");
        std::printf("        admission header: %s\n",
                    pair.value.has_value() ? pair.value->to_string().c_str()
                                           : "<refused>");
    }

    // ---- 6. the SS/DS ToF mathematics (M1-A) ------------------------------
    // uwb_twr_math.h is INCLUDED and EXERCISED.  A common clock (k = 1 stated
    // explicitly, never assumed) with RA = 2000 and DB = 1000 ticks must give
    // exactly 500 A ticks.  The intervals are built through the real ranging
    // gate, so this also proves the install ships the gate the math needs.
    {
        twr::ClockDomain domain;
        check(twr::ClockDomain::make("consumer_m1a_ch5_uus", 499.2e6 * 128.0, 7u, 40u,
                                     domain),
              "the math header builds a clock domain");

        const std::string cal_id = "cal-consumer-m1a";
        auto make = [&](int64_t ticks, twr::TimestampMarker m, uint32_t bits,
                        twr::Timestamp& out) {
            if (!twr::Timestamp::from_ticks(ticks, domain, m,
                                            twr::TimestampSource::HardwareMeasured, bits, out))
                return false;
            return twr::apply_calibration_ticks(out, cal_id, 0, 0, 0u, 0u) ==
                   twr::CalibrationResult::Applied;
        };
        twr::Timestamp ra_l;
        twr::Timestamp ra_e;
        twr::Timestamp db_l;
        twr::Timestamp db_e;
        check(make(2000, twr::TimestampMarker::RmarkerRx, kRmarkerRxBits, ra_l) &&
                  make(0, twr::TimestampMarker::RmarkerTx, kRmarkerTxBits, ra_e) &&
                  make(1000, twr::TimestampMarker::RmarkerTx, kRmarkerTxBits, db_l) &&
                  make(0, twr::TimestampMarker::RmarkerRx, kRmarkerRxBits, db_e),
              "the M1-A timestamps are constructible");

        twr::CalibrationStamp cal;
        cal.id = cal_id;
        cal.calibrated_epoch = 7u;
        cal.valid_from_ticks = 1000;
        cal.valid_until_ticks = 2000;
        twr::CalibrationApplication app;
        app.calibration_id = cal_id;
        app.result = twr::CalibrationResult::Applied;
        cal.applications.push_back(app);

        twr::RangeAdmissionContext ctx;
        ctx.calibration = &cal;
        ctx.reference_ticks = 1500;
        ctx.reference_ticks_recorded = true;
        ctx.rx_first_path = twr::FirstPathQuality::passed(18.5, 9.0, 0.82);

        const twr::RangingIntervalAdmission ra =
            twr::admit_ranging_interval(ra_l, ra_e, ctx);
        const twr::RangingIntervalAdmission db =
            twr::admit_ranging_interval(db_l, db_e, ctx);
        check(ra.admitted && db.admitted, "the M1-A intervals pass the ranging gate");

        twr::ClockRatio k;
        check(twr::ClockRatio::unity_same_clock(domain, k),
              "a common clock is STATED (k=1) rather than defaulted");
        const twr::TofResult tof = twr::compute_ss_tof(*ra.value, *db.value, k);
        check(tof.ok && tof.tof.num == 500 && tof.tof.den == 1,
              "compute_ss_tof gives exactly 500 A ticks for RA=2000, DB=1000");
        std::printf("        math header: compute_ss_tof -> %s A ticks (%s)\n",
                    tof.tof.to_string().c_str(), twr::tof_status_to_string(tof.status));
    }

    std::printf("%s\n", g_failures == 0 ? "CONSUMER OK"
                                        : "CONSUMER FAILED");
    return g_failures == 0 ? 0 : 1;
}
