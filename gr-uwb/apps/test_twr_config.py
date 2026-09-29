#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""QA for :mod:`uwb.twr_config` -- the pure-Python TWR configuration layer.

Run:
    python3 gr-uwb/apps/test_twr_config.py

The module under test is importable straight from a source checkout: no
pybind11, no C++ extension, no built OOT module, no radio and no GNU Radio
import is required.  That is the point of the pure-Python design -- pybind
exposure is deferred to M1 -- so this suite is what proves REQ-API-01's Python
half actually works before anything is bound.

What this suite is FOR
----------------------
  1. prove a config that satisfies the MEASURED whitelist validates, and that
     ``effective_config()`` records requested vs effective faithfully;
  2. prove DEFAULT DENY -- the central requirement.  Every combination absent
     from ``testdata/twr/phy_matrix_whitelist_737280000.csv`` is rejected with a
     specific, machine-readable reason, and nothing is ever silently defaulted
     to the radar / QM35 behaviour (REQ-SCOPE-01);
  3. encode the MEASURED reasons for the rejected preamble lengths, so the CFO
     fit defect and the PHR self-description defect cannot be quietly forgotten;
  4. prove NaN/Inf, index range, channel/frequency conflict, resource conflict,
     over-capacity, timing-precision and calibration rules each produce their
     own reason;
  5. prove the JSON round trip is stable and keeps integer precision
     (REQ-OUT-01);
  6. cross-check the vocabulary against the frozen C++ header and the shipped
     whitelist against the measured CSV, so the two implementations cannot
     drift apart silently.

Two tests are conditional on repository files that are generated rather than
committed (the C++ header and the measured CSVs are build products of other
milestone work).  When a file is absent those tests SKIP with an explicit
message; everything else runs unconditionally.
"""

from __future__ import annotations

import copy
import dataclasses
import importlib
import inspect
import json
import math
import os
import sys
import unittest

# --------------------------------------------------------------------------
# Import the module under test from the source checkout, with no C++ library.
# --------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))             # gr-uwb/apps
_GR_UWB = os.path.dirname(_HERE)                                # gr-uwb
_PYTHON_DIR = os.path.join(_GR_UWB, "python")                   # gr-uwb/python
_REPO_ROOT = os.path.dirname(_GR_UWB)                           # repository root
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

try:
    T = importlib.import_module("uwb.twr_config")
except ModuleNotFoundError as exc:  # pragma: no cover - setup guard
    raise SystemExit("cannot import uwb.twr_config from %s: %s"
                     % (_PYTHON_DIR, exc))

if "gnuradio" in sys.modules:  # pragma: no cover - setup guard
    raise SystemExit("twr_config must import with NO GNU Radio present")

_CXX_HEADER = os.path.join(_REPO_ROOT, "gr-uwb", "include", "gnuradio", "uwb",
                           "uwb_twr_config.h")
_CXX_TYPES = os.path.join(_REPO_ROOT, "gr-uwb", "include", "gnuradio", "uwb",
                          "uwb_twr_types.h")
_WHITELIST_CSV = os.path.join(_REPO_ROOT, "testdata", "twr",
                             "phy_matrix_whitelist_737280000.csv")


def _read(path):
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _read_whitelist_csv():
    """The measured whitelist rows, skipping the leading '#' provenance note."""
    import csv
    with open(_WHITELIST_CSV, "r", encoding="utf-8") as handle:
        lines = [ln for ln in handle.read().splitlines() if not ln.startswith("#")]
    return list(csv.DictReader(lines))


# ==========================================================================
# Shared fixtures
# ==========================================================================


def _minimal(protocol=T.Protocol.SS, role=T.Role.INITIATOR):
    """A minimal but COMPLETE config: every field is stated, nothing assumed.

    THE FRAME GEOMETRY IS DERIVED, NEVER TYPED (M0.1, R3)
    ----------------------------------------------------
    The layout comes from ``frame_geometry_of_profile(FrameProfileId.TWR_V1)``
    -- the codec's own authority -- and not from a literal.  M0's fixture wrote
    a 7-byte header and a 12-byte PHR into the config and then asserted 9/19/24
    on-air bytes, while the codec had emitted a 14-byte header and 16/26/31
    since it was frozen.  Both suites were green, because nothing ever asked
    whether an ACCEPTED configuration could build the real frame.  A fixture
    that derives its numbers from the codec cannot drift from it, and the tests
    below assert 14/2/16/26/31 as the consequence rather than as a constant
    they happen to agree on.

    ``session_id`` is inside the 16-bit wire range for the same reason the
    geometry is: 0x11223344 was a 32-bit number in a 16-bit field.
    """
    ds = protocol == T.Protocol.DS
    initiator = role == T.Role.INITIATOR
    ch = 0 if initiator else 1

    c = T.TwrConfig()
    c.meta.schema_version = T.capabilities().schema_version
    c.meta.profile_version = "qa-synthetic-frame/0"
    c.meta.calibration_version = "qa-synthetic-cal/0"
    c.meta.label = "test_twr_config minimal"

    # ---- session -------------------------------------------------------
    s = c.session
    s.protocol = protocol
    s.role = role
    s.local_address = 0x0001 if initiator else 0x0002
    s.peer_address = 0x0002 if initiator else 0x0001
    s.pan_id = 0xCAFE
    s.session_id = 0x1234
    s.exchange_id = 1
    s.sequence = 7
    s.sequence_modulus = 256
    s.measurement_count = 1
    s.measurement_interval = T.Duration(20_000_000)
    s.max_attempts_per_exchange = 1
    s.retry_backoff = T.Duration(0)
    s.max_in_flight_exchanges = T.MAX_IN_FLIGHT_EXCHANGES
    s.require_pan_match = True
    s.require_address_match = True

    # ---- phy: the MEASURED whitelist row (ch5 / 64 SYNC / 4z2 / code 9) --
    p = c.phy
    p.channel = 5
    p.center_frequency_hz = 6489.6e6
    p.tx_preamble_code = 9
    p.rx_preamble_code = 9
    p.preamble_symbols = 64
    p.prf_class = T.PrfClass.BPRF64
    p.data_rate = T.DataRate.R6P8M
    p.phr_rate = T.PhrRate.STANDARD_850K

    # ---- frame format ---------------------------------------------------
    f = c.frame
    f.frame_profile = T.FrameProfileId.TWR_V1
    f.sfd_mode = T.SfdMode.R4Z2
    f.sfd_symbols = 8
    f.sfd_timeout = T.device_field(20_000, T.TimeReferenceEvent.RX_ENABLE,
                                   T.TimestampMarker.UHD_RX_FIRST_IQ_SAMPLE)
    f.phr_mode = T.PhrMode.STANDARD
    f.ranging_bit = True
    f.geometry = T.frame_geometry_of_profile(T.FrameProfileId.TWR_V1)
    f.mac_psdu_bytes = 32
    f.mac_psdu_includes_fcs = False
    f.fcs_append = T.FcsAppender.PHY
    f.fcs_bytes = 2
    f.application_payload_bytes = 8
    f.sts_mode = T.StsMode.OFF
    f.sts_length_symbols = 0

    # ---- transmit: gain dB is the authority; IQ amplitude and dBm absent --
    c.tx.port = ch
    c.tx.gain_db = 20.0
    c.tx.power_policy = T.TxPowerPolicy.MANUAL_GAIN_DB
    c.tx.pulse_shaping = T.PulseShaping.EXISTING_HRP

    # ---- receive --------------------------------------------------------
    c.rx.port = ch
    c.rx.gain_db = 30.0
    c.rx.agc = T.AgcMode.MANUAL
    c.rx.bandwidth_hz = 500.0e6
    c.rx.detection_threshold = 0.30
    c.rx.correlation_threshold = 0.35
    c.rx.first_path_threshold = 0.25
    c.rx.first_path_index = 8
    c.rx.first_path_window = 30

    # ---- radio ----------------------------------------------------------
    r = c.radio
    r.device_args = "addr=192.168.10.2"
    r.tx_channel = ch
    r.rx_channel = ch
    r.native_sample_rate_hz = T.NATIVE_RATE_UC200_HZ
    r.clock_source = "internal"
    r.time_source = "internal"
    peer = T.EndpointBinding(
        id="B" if initiator else "A",
        role=T.Role.RESPONDER if initiator else T.Role.INITIATOR,
        tx_channel=1 if initiator else 0,
        rx_channel=1 if initiator else 0,
        native_sample_rate_hz=T.NATIVE_RATE_UC200_HZ,
        occupies_resources=True)
    r.peers.append(peer)
    r.readback = T.RadioReadback(present=True,
                                 sample_rate_hz=T.NATIVE_RATE_UC200_HZ,
                                 center_freq_hz=6489.6e6, tx_channel=ch,
                                 rx_channel=ch, mpm_string="X410", fpga_image="",
                                 uhd_version="4.x", clock_source="internal",
                                 time_source="internal")
    r.require_readback = True

    # ---- per-message timing ---------------------------------------------
    t = c.timing
    t.poll_start = T.device_field(
        40_000_000_000 if initiator else 0,
        T.TimeReferenceEvent.POLL_TX_RMARKER, T.TimestampMarker.RMARKER_TX)
    if initiator:
        # The initiator owns NO reply delay.
        t.poll_to_response = T.device_field(
            0, T.TimeReferenceEvent.POLL_RX_RMARKER, T.TimestampMarker.RMARKER_RX)
        t.response_to_final = T.device_field(
            600_000 if ds else 0,
            T.TimeReferenceEvent.RESPONSE_RX_RMARKER if ds
            else T.TimeReferenceEvent.FINAL_TX_RMARKER,
            T.TimestampMarker.RMARKER_RX)
    else:
        t.poll_to_response = T.device_field(
            600_000, T.TimeReferenceEvent.POLL_RX_RMARKER,
            T.TimestampMarker.RMARKER_RX)
        t.response_to_final = T.device_field(
            0, T.TimeReferenceEvent.FINAL_TX_RMARKER, T.TimestampMarker.RMARKER_RX)
    t.final_to_report = T.device_field(
        0, T.TimeReferenceEvent.REPORT_TX_RMARKER, T.TimestampMarker.RMARKER_TX)
    # A third, unrelated quantity: arm RX after the end of our own frame.
    t.post_tx_rx_enable = T.device_field(
        2_000, T.TimeReferenceEvent.FRAME_TAIL, T.TimestampMarker.PREAMBLE_START)
    # NOTE: 20 us is a QA PLACEHOLDER, not a measurement.
    # testdata/twr/timing_budget_turnaround_estimate.csv records
    # uhd_min_lead_time as UNMEASURED and min_feasible_reply_delay as
    # BLOCKED_ON_HOST_MEASUREMENT for all four reply scopes, so no default lead
    # time exists and this value must not be read as a promise of a short
    # turnaround.
    t.min_tx_lead_time = T.device_field(
        20_000, T.TimeReferenceEvent.RESPONSE_TX_RMARKER,
        T.TimestampMarker.RMARKER_TX)

    # ---- timeouts --------------------------------------------------------
    to = c.timeouts
    rx = T.TimestampMarker.UHD_RX_FIRST_IQ_SAMPLE
    to.poll_rx_window = T.device_field(0 if initiator else 20_000,
                                       T.TimeReferenceEvent.RX_ENABLE, rx)
    to.response_rx_window = T.device_field(20_000 if initiator else 0,
                                           T.TimeReferenceEvent.RX_ENABLE, rx)
    to.final_rx_window = T.device_field(
        20_000 if (ds and not initiator) else 0, T.TimeReferenceEvent.RX_ENABLE,
        rx)
    to.report_rx_window = T.device_field(0, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.rx_timeout = T.device_field(40_000, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.exchange_timeout = T.host_field(50_000_000)
    to.retry_interval = T.host_field(0)

    # ---- calibration -----------------------------------------------------
    k = c.calibration
    k.tx_link_delay = T.Duration(120)
    k.rx_link_delay = T.Duration(180)
    k.antenna_delay = T.Duration(35)
    k.cable_delay = T.Duration(90)
    k.native_sample_rate_hz = T.NATIVE_RATE_UC200_HZ
    k.link_delay_unit = T.TimeUnit.NANOSECONDS
    k.cfo_compensation = T.CompensationFlag.REQUIRED
    k.sfo_compensation = T.CompensationFlag.ON
    k.calibration_id = "qa-synthetic-cal/0"
    k.record = T.CalibrationRecord(
        calibration_id="qa-synthetic-cal/0", device_serial="X410-QA",
        channel=5, native_sample_rate_hz=T.NATIVE_RATE_UC200_HZ,
        profile_version="qa-synthetic-frame/0", gain_db=20.0,
        valid_until_monotonic_ns=0)

    # ---- diagnostics -----------------------------------------------------
    d = c.diagnostics
    d.result_queue_capacity = 256
    d.event_queue_capacity = 512
    d.stats_cadence = T.host_field(1_000_000_000)
    return c


class TwrConfigTestBase(unittest.TestCase):
    """Assertions every rejection in this suite must satisfy (REQ-ERR-01)."""

    def assert_machine_readable(self, report):
        """A bare bool is never an acceptable rejection outcome."""
        self.assertFalse(report.ok(), "expected a rejection, got: %s"
                         % report.to_string())
        self.assertTrue(report.violations)
        for v in report.violations:
            self.assertTrue(v.message, "violation with no message: %r" % (v,))
            self.assertIsNot(v.reason, T.ConfigReason.NONE)
            self.assertTrue(v.reason.value and v.reason.value != "invalid")
            self.assertTrue(v.status.value and v.status.value != "invalid")
            self.assertNotEqual(T.exchange_status_family(v.status), "invalid")
            self.assertTrue(v.requirement,
                            "violation with no requirement tag: %r" % (v,))

    def assert_has(self, report, field, reason):
        self.assert_machine_readable(report)
        v = report.find(field, reason)
        self.assertIsNotNone(
            v, "expected %s / %s, got: %s" % (field, reason, report.to_string()))
        return v


# ==========================================================================
# A. Frozen vocabulary and mirrored constants
# ==========================================================================


class TestVocabulary(TwrConfigTestBase):

    def test_enum_spellings_are_exact(self):
        # These spellings are part of the wire schema (REQ-OUT-01): changing
        # one silently breaks every stored config and every recorded result.
        self.assertEqual(
            [m.value for m in T.Protocol], ["ss", "ds"])
        self.assertEqual(
            [m.value for m in T.Role], ["initiator", "responder"])
        self.assertEqual(
            [m.value for m in T.FrameType],
            ["poll", "response", "final", "report"])
        self.assertEqual(
            [m.value for m in T.PrfClass], ["bprf64", "hprf64", "hprf400"])
        self.assertEqual(
            [m.value for m in T.DataRate],
            ["850k", "6p8m", "27m", "7p8m", "27p2m", "6p8m_hprf"])
        self.assertEqual(
            [m.value for m in T.SfdMode],
            ["4z1", "4z2", "4z3", "4z4", "decawave", "ieee"])
        self.assertEqual(
            [m.value for m in T.PhrMode], ["standard", "extended", "none"])
        self.assertEqual(
            [m.value for m in T.StsMode],
            ["off", "sp64", "sp128", "sp256", "sp512", "sp1024"])
        self.assertEqual(
            [m.value for m in T.FcsAppender], ["none", "mac", "phy"])
        self.assertEqual(
            [m.value for m in T.TxPowerPolicy],
            ["leave_untouched", "manual_gain_db", "iq_amplitude",
             "calibrated_dbm"])
        self.assertEqual(
            [m.value for m in T.PulseShaping],
            ["existing_hrp", "rectangular", "root_raised_cosine"])
        self.assertEqual(
            [m.value for m in T.AgcMode], ["manual", "disabled", "vendor_default"])
        self.assertEqual(
            [m.value for m in T.FirstPathAlgorithm],
            ["leading_edge", "peak", "energy_centroid", "interpolated_peak",
             "strongest_cluster"])
        self.assertEqual(
            [m.value for m in T.CompensationFlag], ["off", "on", "required"])
        self.assertEqual(
            [m.value for m in T.TimeReferenceEvent],
            ["poll_tx_rmarker", "poll_rx_rmarker", "response_tx_rmarker",
             "response_rx_rmarker", "final_tx_rmarker", "final_rx_rmarker",
             "report_tx_rmarker", "report_rx_rmarker", "frame_tail", "rx_enable",
             "host_monotonic"])
        self.assertEqual(
            [m.value for m in T.TimeDomain],
            ["unspecified", "device_ticks", "monotonic_host"])
        self.assertEqual(
            [m.value for m in T.TimeUnit], ["ns", "s", "native_ticks"])
        self.assertEqual(
            [m.value for m in T.TimestampMarker],
            ["uhd_rx_first_iq_sample", "preamble_start", "sfd_start", "phr_start",
             "rmarker_tx", "rmarker_rx", "antenna_plane"])

    def test_enum_helpers_round_trip_and_reject_nonsense(self):
        for cls in (T.Protocol, T.Role, T.PrfClass, T.DataRate, T.SfdMode,
                    T.PhrMode, T.StsMode, T.FcsAppender, T.TxPowerPolicy,
                    T.PulseShaping, T.AgcMode, T.FirstPathAlgorithm,
                    T.CompensationFlag, T.TimeReferenceEvent, T.TimeDomain,
                    T.TimeUnit, T.TimestampMarker, T.ExchangeStatus,
                    T.CapabilityStatus, T.FrameType):
            for member in cls:
                self.assertIs(cls.from_string(member.value), member,
                              "%s round trip" % cls.__name__)
            self.assertIsNone(cls.from_string("nope"))
        # A str-valued enum compares and prints as its wire spelling.
        self.assertEqual(T.Protocol.SS, "ss")
        self.assertEqual(str(T.SfdMode.DWT8), "decawave")
        self.assertEqual("%s" % T.TimeUnit.NATIVE_TICKS, "native_ticks")
        # The C++ protocol reader also accepts SS/DS and single/double.
        self.assertIs(T.Protocol.from_string("SS"), T.Protocol.SS)
        self.assertIs(T.Protocol.from_string("double"), T.Protocol.DS)
        self.assertIsNone(T.Protocol.from_string("sideways"))

    def test_config_reason_taxonomy_is_complete_and_unique(self):
        reasons = [m.value for m in T.ConfigReason]
        self.assertEqual(len(reasons), len(set(reasons)))
        for expected in ("none", "not_finite", "out_of_range",
                         "index_out_of_range", "negative_value", "empty_value",
                         "over_capacity", "zero_value", "field_conflict",
                         "channel_frequency_mismatch", "duplicate_resource",
                         "timing_order_violation", "timing_budget_infeasible",
                         "timing_precision_insufficient",
                         "quantisation_unsupported", "unit_mismatch",
                         "frame_length_overflow", "unsupported", "out_of_scope",
                         "in_flight_not_supported", "calibration_missing",
                         "calibration_expired", "calibration_already_applied",
                         "calibration_mismatch", "mid_exchange_mutation",
                         "exchange_already_in_flight", "malformed_json",
                         "unknown_key", "missing_key", "type_mismatch",
                         "unknown_enum_value", "integer_precision_loss"):
            self.assertIn(expected, reasons)
        self.assertEqual(len(reasons), 32, "expected the full C++ taxonomy")

    def test_exchange_status_taxonomy_and_families(self):
        for name in ("ok", "config_rejected", "unsupported", "cancelled",
                     "queue_full", "internal_error", "phy_fcs_failed",
                     "phy_decode_failed", "wrong_peer", "unexpected_frame_type",
                     "stale_session", "rx_timeout", "rx_overflow",
                     "rx_chain_broken", "tx_late", "tx_underflow", "tx_seq_error",
                     "tx_chain_broken", "invalid_time_domain",
                     "clock_estimate_invalid", "first_path_unreliable",
                     "calibration_missing", "calibration_expired",
                     "protocol_timeout", "deadline_missed"):
            status = T.ExchangeStatus.from_string(name)
            self.assertIsNotNone(status, name)
            self.assertNotEqual(T.exchange_status_family(status), "invalid")
        self.assertTrue(T.exchange_status_is_ok(T.ExchangeStatus.OK))
        self.assertTrue(T.exchange_status_yields_range(T.ExchangeStatus.OK))
        # Only a fully successful exchange yields a range (REQ-TIME-05).
        self.assertFalse(T.exchange_status_yields_range(
            T.ExchangeStatus.FIRST_PATH_UNRELIABLE))

    def test_mirrored_constants(self):
        # Mirrors radar_meta::code_index_supported / kMaxPsduBytes and
        # is_allowed_uhd_native_rate.
        self.assertEqual((T.CODE_INDEX_MIN, T.CODE_INDEX_MAX), (9, 12))
        self.assertTrue(T.code_index_supported(9))
        self.assertTrue(T.code_index_supported(12))
        self.assertFalse(T.code_index_supported(8))
        self.assertFalse(T.code_index_supported(13))
        self.assertEqual(T.MAX_PSDU_BYTES, 127)
        self.assertEqual(T.NATIVE_RATE_UC200_HZ, 737280000.0)
        self.assertEqual(T.NATIVE_RATE_CG400_HZ, 491520000.0)
        for hz in (737280000.0, 491520000.0):
            self.assertTrue(T.is_allowed_native_rate(hz))
        for hz in (1e6, 64e6, 100e6, 500e6, 998.4e6, 1e9, 2e9):
            self.assertFalse(T.is_allowed_native_rate(hz))
        # REQ-PHY-02: work rate, native rates, symbol rate and bandwidth are
        # four different quantities.
        self.assertEqual(T.WORK_SAMPLE_RATE_HZ, 998.4e6)
        self.assertEqual(T.WORK_SAMPLES_PER_SYMBOL, 1016)
        self.assertEqual(T.EXISTING_MEAN_PRF_HZ, 62.4e6)
        self.assertEqual(T.NOMINAL_MEAN_PRF_HZ, 64.0e6)
        self.assertEqual(T.SCHEMA_VERSION, "twr-config/2")
        self.assertEqual(T.JSON_MAX_SAFE_INTEGER, 2 ** 53 - 1)
        self.assertEqual(T.MAX_IN_FLIGHT_EXCHANGES, 1)
        self.assertEqual(T.RADAR_SYNC_REPETITIONS,
                         (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048))
        self.assertEqual(T.IEEE_802154A_PREAMBLE_SYMBOLS, (16, 64, 1024, 4096))

    def test_allowed_quantisation_rates(self):
        for hz in (737280000.0, 491520000.0, 998400000.0, 1.0e9):
            self.assertTrue(T.is_allowed_quantisation_hz(hz), str(hz))
        for hz in (0.0, -1.0, 1.0, 2.0e9, 1e18, float("nan"),
                   float("inf"), 737.28e6 + 5000.0):
            self.assertFalse(T.is_allowed_quantisation_hz(hz), str(hz))

    def test_channel_plan_is_ieee_and_channel5_is_6489p6(self):
        self.assertEqual(T.uwb_channel_center_frequency_hz(5), 6489.6e6)
        self.assertEqual(T.uwb_channel_center_frequency_hz(9), 6988.8e6)
        self.assertEqual(T.uwb_channel_center_frequency_hz(0), 4835.2e6)
        self.assertEqual(T.uwb_channel_center_frequency_hz(16), 7852.2e6)
        self.assertEqual(T.uwb_channel_center_frequency_hz(17), 0.0)
        self.assertEqual(T.uwb_channel_center_frequency_hz(-1), 0.0)

    def test_duration_arithmetic_and_tick_round_trip(self):
        d = T.Duration(600_000)
        self.assertEqual(d.nanos(), 600_000)
        self.assertFalse(d.is_zero())
        self.assertFalse(d.negative())
        self.assertEqual(d.ticks_at(T.NATIVE_RATE_UC200_HZ), 442368)
        self.assertIsNone(d.ticks_at(0.0))
        self.assertIsNone(d.ticks_at(float("nan")))
        back = T.Duration.from_ticks(442368, T.NATIVE_RATE_UC200_HZ)
        self.assertIsNotNone(back)
        self.assertLessEqual(abs(back.nanos() - 600_000), 1)
        self.assertEqual(T.Duration.from_seconds(float("nan")), None)
        self.assertEqual(T.Duration.from_seconds(float("inf")), None)
        self.assertEqual((T.Duration(10) + T.Duration(5)).nanos(), 15)
        self.assertEqual((T.Duration(10) - T.Duration(15)).nanos(), -5)
        self.assertLess(T.Duration(1), T.Duration(2))
        self.assertEqual(T.Duration(0), T.Duration(0))
        self.assertEqual(T.duration_to_text(T.Duration(120)), "120ns")
        with self.assertRaises(TypeError):
            T.Duration(1.5)          # type: ignore[arg-type]
        with self.assertRaises(OverflowError):
            T.Duration(2 ** 64)

    @unittest.skipUnless(os.path.isfile(_CXX_HEADER) and os.path.isfile(_CXX_TYPES),
                         "frozen C++ TWR headers are not in this checkout")
    def test_python_vocabulary_matches_the_frozen_cxx_header(self):
        """The Python layer must agree with uwb_twr_config.h field-for-field.

        A text cross-check, not a build: every enum spelling, every
        ConfigReason name and every dotted field path the C++ header emits must
        also exist on the Python side, so the two cannot drift apart silently
        while either is frozen.
        """
        header = _read(_CXX_HEADER) + _read(_CXX_TYPES)

        def in_header(needle, what):
            if needle not in header:
                self.fail("%s is not spelled the same in the C++ header: %r"
                          % (what, needle))

        for cls in (T.Protocol, T.Role, T.FrameType, T.PrfClass, T.DataRate,
                    T.SfdMode, T.PhrMode, T.StsMode, T.FcsAppender,
                    T.TxPowerPolicy, T.PulseShaping, T.AgcMode,
                    T.FirstPathAlgorithm, T.CompensationFlag,
                    T.TimeReferenceEvent, T.TimeDomain, T.TimeUnit,
                    T.TimestampMarker, T.ExchangeStatus, T.CapabilityStatus):
            for member in cls:
                in_header('"%s"' % member.value,
                          "%s::%s" % (cls.__name__, member.value))
        for reason in T.ConfigReason:
            in_header('"%s"' % reason.value, "ConfigReason::%s" % reason.value)
        # Dotted field paths used by validate() exist on the Python side too.
        paths = [f.path for f in T.flatten_fields(_minimal())]
        for needle in ("session.protocol", "phy.channel", "phy.preamble_symbols",
                       "frame.sfd_mode", "frame.mac_psdu_bytes[poll]",
                       "frame.geometry.mac_header_bytes", "radio.peers[0].tx_channel",
                       "timing.poll_to_response.ns", "timeouts.rx_timeout.marker",
                       "calibration.tx_link_delay_native_ticks",
                       "calibration.record.valid_until_monotonic_ns",
                       "diagnostics.stats_cadence.quantisation_hz",
                       "tx.vendor_power_word_backend", "rx.vendor_pac_applied_step"):
            prefix = needle.split(".")[0]
            self.assertTrue(any(p.startswith(prefix + ".") for p in paths),
                            "no flattened field under %s" % prefix)
        # The mirrored numbers are spelled the same way in both languages.
        for needle in ("737280000.0", "491520000.0", "998.4e6", "1016",
                       "62.4e6", "9007199254740991"):
            in_header(needle, "constant %s" % needle)

    @unittest.skipUnless(os.path.isfile(_CXX_HEADER),
                         "frozen C++ TWR header is not in this checkout")
    def test_capability_whitelist_matches_the_frozen_cxx_header(self):
        """The shipped whitelist must equal uwb_twr_config.h's, item for item.

        A text cross-check, not a build.  The C++ header splits the row-reason
        literals across source lines, so the comparison is made against a
        normalised copy of the header (whitespace and literal quotes removed):
        the reason strings contain neither, so a whitespace-insensitive match
        is exactly the byte comparison we want.
        """
        header = _read(_CXX_HEADER)
        flat = "".join(header.split()).replace('"', "")
        caps = T.capabilities()

        def in_header(needle, what=None):
            if "".join(needle.split()) not in flat:
                self.fail("%s is not spelled the same in the C++ header: %r"
                          % (what or needle, needle))

        def in_header_exact(needle, what=None):
            if needle not in header:
                self.fail("%s is not spelled the same in the C++ header: %r"
                          % (what or needle, needle))

        # The admitted SYNC lengths, in the C++ order.
        in_header_exact("c.sync_repetitions.push_back(16);")
        in_header_exact("c.sync_repetitions.push_back(64);")
        self.assertEqual(caps.sync_repetitions, [16, 64])

        # The measured row loop: 2 lengths x 4 codes x 6 SFD modes.
        in_header_exact("const uint16_t kSyncs[] = { 16, 64 };")
        in_header_exact("for (uint8_t code = kTwrCodeIndexMin; "
                        "code <= kTwrCodeIndexMax;")
        self.assertEqual(len(caps.phy_matrix), 2 * 4 * 6)
        self.assertEqual(len(caps.phy_matrix), 48)
        self.assertEqual(len(caps.phy_matrix), 2 * len(caps.code_indices) *
                         len(caps.sfd_modes))

        # The two capability-row reason strings, byte for byte.
        in_header(T.MEASURED_ROW_REASON_TOA_UNVERIFIED,
                  "the ToA-unverified row reason")
        in_header(T.MEASURED_ROW_REASON, "the plain row reason")
        self.assertNotEqual(T.MEASURED_ROW_REASON_TOA_UNVERIFIED,
                            T.MEASURED_ROW_REASON)

        # The profile version.
        in_header_exact('c.profile_version = "%s";'
                        % T.MEASURED_PROFILE_VERSION)
        self.assertEqual(caps.profile_version, T.MEASURED_PROFILE_VERSION)

        # The ToA-validation predicate and its table.
        in_header_exact("kTwrSyncRepsNeedingToaValidation[] = { 16 }")
        in_header_exact("twr_sync_reps_needs_toa_validation")
        in_header("twr_sync_reps_needs_toa_validation(sync)",
                  "the predicate call site")
        self.assertEqual(T.SYNC_REPS_NEEDING_TOA_VALIDATION, (16,))

        # The measured data rate and SFD set.  C++ spells the enum members
        # differently from the Python identifiers, so map them explicitly
        # rather than deriving the C++ name from the Python one.
        in_header_exact("c.data_rates.push_back(DataRate::R6p8M);")
        self.assertEqual(caps.data_rates, [T.DataRate.R6P8M])
        cpp_sfd_names = {T.SfdMode.R4Z1: "R4z1", T.SfdMode.R4Z2: "R4z2",
                         T.SfdMode.R4Z3: "R4z3", T.SfdMode.R4Z4: "R4z4",
                         T.SfdMode.DWT8: "Dwt8", T.SfdMode.IEEE8: "Ieee8"}
        self.assertEqual(sorted(caps.sfd_modes, key=str),
                         sorted(cpp_sfd_names, key=str))
        for sfd in caps.sfd_modes:
            in_header_exact("c.sfd_modes.push_back(SfdMode::%s);"
                            % cpp_sfd_names[sfd],
                            "the %s SFD push" % sfd.value)
        # Channel 5 only, bprf64 only, ranging bit required, no STS.
        in_header_exact("c.channels.push_back(5);")
        in_header_exact("c.prf_classes.push_back(PrfClass::Bprf64);")
        in_header_exact("c.ranging_bit_required = true;")
        in_header_exact("c.sts_supported = false;")
        self.assertEqual(caps.channels, [5])
        self.assertEqual(caps.prf_classes, [T.PrfClass.BPRF64])
        self.assertTrue(caps.ranging_bit_required)
        self.assertFalse(caps.sts_supported)

# ==========================================================================
# B. Valid configurations and the frozen effective config
# ==========================================================================


class TestValidConfigs(TwrConfigTestBase):

    def test_valid_ss_and_ds_for_both_roles(self):
        for protocol in (T.Protocol.SS, T.Protocol.DS):
            for role in (T.Role.INITIATOR, T.Role.RESPONDER):
                c = _minimal(protocol, role)
                report = T.validate(c)
                self.assertTrue(
                    report.ok(),
                    "%s/%s rejected: %s" % (protocol, role, report.to_string()))

    def test_effective_config_snapshot_is_correct(self):
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())

        # versions (REQ-API-01)
        self.assertEqual(e.schema_version, T.capabilities().schema_version)
        self.assertEqual(e.profile_version, c.meta.profile_version)
        self.assertEqual(e.calibration_version, c.meta.calibration_version)
        self.assertTrue(e.config_hash.startswith("fnv1a64:"))
        self.assertEqual(len(e.config_hash), len("fnv1a64:") + 16)

        # requested vs effective: nothing is invented.
        self.assertEqual(e.requested.calibration.tx_link_delay.nanos(), 120)
        self.assertEqual(e.effective.calibration.tx_link_delay.nanos(), 120)
        self.assertEqual(T.config_hash(e.requested), T.config_hash(e.effective))
        for ch in e.changes:
            if not ch.adjusted():
                continue
            self.assertEqual(ch.path, "timing.quantised_delay")
            self.assertNotIn("quantised at", ch.effective)

        # readback is preserved verbatim (REQ-PHY-02)
        self.assertTrue(e.readback.present)
        self.assertAlmostEqual(e.readback.sample_rate_hz,
                               T.NATIVE_RATE_UC200_HZ, places=3)
        self.assertEqual(e.readback.mpm_string, "X410")

        # quantised timing: 600 us at 737.28 MS/s
        self.assertAlmostEqual(e.tick_rate_hz, T.NATIVE_RATE_UC200_HZ, places=3)
        self.assertEqual(e.response_to_final_ticks, 442368)
        self.assertEqual(e.poll_to_response_ticks, 0)
        self.assertEqual(e.poll_to_response_effective.nanos(), 0)
        # post-TX RX enable is a SEPARATE quantity, quantised on its own.
        self.assertEqual(e.post_tx_rx_enable_ticks, 1475)
        self.assertNotEqual(e.poll_to_response_ticks, e.post_tx_rx_enable_ticks)

        # Frame budget: the CODEC's numbers, and the assertion is written
        # against the profile's own constants so it cannot drift from them.
        # M0 asserted 9/19/24 here, from a 7-byte header the codec has never
        # emitted; the real on-air sizes are 16/26/31.
        auth = T.frame_geometry_for(T.FrameProfileId.TWR_V1)
        self.assertEqual(e.poll_bytes, T.FRAME_HEADER_BYTES + 0 * 5 + 2)
        self.assertEqual(e.response_bytes, T.FRAME_HEADER_BYTES + 2 * 5 + 2)
        self.assertEqual(e.final_bytes, T.FRAME_HEADER_BYTES + 3 * 5 + 2)
        self.assertEqual((e.poll_bytes, e.response_bytes, e.final_bytes),
                         (16, 26, 31))
        self.assertEqual((e.poll_bytes, e.response_bytes, e.final_bytes),
                         (auth.on_air_bytes(T.FrameType.POLL),
                          auth.on_air_bytes(T.FrameType.RESPONSE),
                          auth.on_air_bytes(T.FrameType.FINAL)))
        self.assertEqual(e.max_psdu_bytes, T.MAX_PSDU_BYTES)
        self.assertEqual(e.max_timestamp_count, T.FRAME_MAX_TIMESTAMPS)
        # The PHR of this profile, recorded so no consumer recomputes it.
        self.assertEqual(e.phr_bytes, T.PHR_STANDARD_INFO_BYTES)
        self.assertEqual(e.phr_coded_bits, T.PHR_STANDARD_CODED_BITS)

    def test_effective_config_records_requested_vs_effective_quantisation(self):
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        # 600002 ns is not an exact number of 737.28 MS/s device ticks.
        c.timing.response_to_final.value = T.Duration(600_002)
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        found = [ch for ch in e.changes if ch.path == "timing.quantised_delay"]
        self.assertEqual(len(found), 1)
        ch = found[0]
        self.assertTrue(ch.adjusted())
        self.assertEqual(ch.requested, "600002ns")
        self.assertNotEqual(ch.effective, ch.requested)
        self.assertIn("quantised at", ch.note)
        self.assertIn("ticks=442369", ch.note)
        # The device gets the tick value, the operator sees both numbers.
        self.assertEqual(e.response_to_final_ticks, 442369)
        self.assertEqual(e.response_to_final_effective.nanos(), 600_001)

    def test_rejected_config_yields_no_effective_values(self):
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        c.frame.sts_mode = T.StsMode.SP128
        c.frame.sts_length_symbols = 128
        e = T.effective_config(c)
        self.assertFalse(e.ok)
        self.assertFalse(e.validation.ok())
        # The hash of the REQUESTED config is recorded so a failure is
        # traceable to the exact input that caused it (REQ-OUT-01).
        self.assertTrue(e.config_hash.startswith("fnv1a64:"))
        self.assertEqual(e.config_hash, T.config_hash(c))
        # ... and there is no way to accidentally use a value-initialised
        # "effective" config as if it were a profile.
        self.assertEqual(e.effective.meta.profile_version, "")
        self.assertIs(e.effective.frame.sts_mode, T.StsMode.OFF)
        self.assertEqual(e.effective.frame.sts_length_symbols, 0)
        self.assertEqual(e.effective.calibration.calibration_id, "")
        self.assertEqual(e.effective.timing.response_to_final.value.nanos(), 0)
        self.assertEqual(e.changes, [])
        self.assertEqual(e.tick_rate_hz, 0.0)
        self.assertEqual(e.poll_bytes, 0)

    def test_config_hash_tracks_every_field(self):
        a = _minimal()
        b = _minimal()
        self.assertEqual(T.config_hash(a), T.config_hash(b))
        b.session.pan_id = 0xCBFF
        self.assertNotEqual(T.config_hash(a), T.config_hash(b))
        d = T.diff_fields(a, b)
        self.assertEqual(len(d), 1)
        self.assertEqual(d[0].path, "session.pan_id")
        self.assertTrue(d[0].adjusted())
        # A deep change is tracked too.
        deep = _minimal()
        deep.diagnostics.stats_cadence.value = T.Duration(2_000_000_000)
        self.assertNotEqual(T.config_hash(a), T.config_hash(deep))

    def test_canonical_text_covers_every_group(self):
        canon = T.canonical_text(T.flatten_fields(_minimal()))
        for needle in ("session.protocol=ss\n", "phy.channel=5\n",
                       "phy.phr_rate=850k\n",
                       "frame.frame_profile=frame_v1\n",
                       "frame.geometry.mac_header_bytes=14\n",
                       "frame.geometry.phr_bytes=2\n",
                       "timing.poll_to_response.ns=0\n",
                       "timing.post_tx_rx_enable.marker=preamble_start\n",
                       "timeouts.exchange_timeout.domain=monotonic_host\n",
                       "timeouts.exchange_timeout.marker=null\n",
                       "timeouts.rx_timeout.reference=rx_enable\n",
                       "diagnostics.io_on_realtime_thread=false\n",
                       "calibration.record.calibration_id=qa-synthetic-cal/0\n",
                       "radio.peers[0].id=B\n", "tx.gain_db=20\n",
                       "rx.vendor_pac_value=null\n"):
            self.assertIn(needle, canon, "canonical text is missing %r" % needle)

    def test_timed_field_marker_rule_is_enforced(self):
        # A device-tick field MUST name its RF marker.
        c = _minimal()
        c.timing.post_tx_rx_enable.marker = None
        self.assert_has(T.validate(c), "timing.post_tx_rx_enable.marker",
                        T.ConfigReason.EMPTY_VALUE)
        # A host-monotonic field MUST NOT carry one: that would invite a
        # cross-domain subtraction (REQ-TIME-01).
        d = _minimal()
        d.timeouts.exchange_timeout.marker = T.TimestampMarker.RMARKER_TX
        self.assert_has(T.validate(d), "timeouts.exchange_timeout.marker",
                        T.ConfigReason.FIELD_CONFLICT)
        # ... and must be quantised at integer nanoseconds.
        e = _minimal()
        e.timeouts.exchange_timeout.required_quantisation_hz = 1.0e6
        self.assert_has(T.validate(e), "timeouts.exchange_timeout.quantisation_hz",
                        T.ConfigReason.QUANTISATION_UNSUPPORTED)
        # The clock domain must be stated at all.
        f = _minimal()
        f.timing.post_tx_rx_enable.domain = T.TimeDomain.UNSPECIFIED
        self.assert_has(T.validate(f), "timing.post_tx_rx_enable.domain",
                        T.ConfigReason.EMPTY_VALUE)
        # The first RX IQ sample is not the end of a frame.
        g = _minimal()
        g.timing.post_tx_rx_enable.marker = \
            T.TimestampMarker.UHD_RX_FIRST_IQ_SAMPLE
        self.assert_has(T.validate(g), "timing.post_tx_rx_enable.marker",
                        T.ConfigReason.FIELD_CONFLICT)


# ==========================================================================
# C. DEFAULT DENY: the capability whitelist
# ==========================================================================


class TestDefaultDeny(TwrConfigTestBase):

    def test_shipped_whitelist_enables_only_the_measured_phy_row(self):
        caps = T.capabilities()
        self.assertEqual(caps.schema_version, T.SCHEMA_VERSION)
        self.assertEqual(caps.phy_matrix_source, T.PHY_MATRIX_WHITELIST_CSV)
        # 48 measured rows: 2 lengths x 4 codes x 6 SFD modes.
        self.assertEqual(len(caps.phy_matrix), 48)
        self.assertEqual(sorted(caps.code_indices), [9, 10, 11, 12])
        # BOTH 16 and 64 round trip; 64 is the phase-1 ranging profile.
        self.assertEqual(caps.sync_repetitions, [16, 64])
        self.assertEqual(len(caps.sfd_modes), 6)
        self.assertEqual(caps.data_rates, [T.DataRate.R6P8M])
        # C++ parity: the profile version and the two capability-row reason
        # strings are byte-identical to uwb_twr_config.h.
        self.assertEqual(caps.profile_version, "m0-ch5-64sync-4z2")
        self.assertEqual(caps.profile_version, T.MEASURED_PROFILE_VERSION)
        # REQ-PHY-01 / REQ-PHY-03: channel 5 only, 64 MHz PRF class only,
        # ranging bit required.
        self.assertEqual(caps.channels, [5])
        self.assertEqual(caps.prf_classes, [T.PrfClass.BPRF64])
        self.assertTrue(caps.ranging_bit_required)
        # REQ-SCOPE-04: no STS at all.
        self.assertFalse(caps.sts_supported)
        self.assertEqual(caps.sts_modes, [])
        for row in caps.phy_matrix:
            self.assertEqual(row.native_rate_hz, T.NATIVE_RATE_UC200_HZ)
            self.assertIn(row.sync_repetitions, (16, 64))
            self.assertEqual(row.max_psdu_bytes, T.MAX_PSDU_BYTES)
            self.assertTrue(row.ranging)
            self.assertIs(row.status, T.CapabilityStatus.MEASURED)
        # The 16 SYNC rows carry the extra first-path / ToA caveat token; the
        # 64 SYNC rows do not, and nothing else differs between the two sets.
        by_sync = {16: set(), 64: set()}
        for row in caps.phy_matrix:
            by_sync[row.sync_repetitions].add(row.reason)
        self.assertEqual(by_sync[16], {T.MEASURED_ROW_REASON_TOA_UNVERIFIED})
        self.assertEqual(by_sync[64], {T.MEASURED_ROW_REASON})
        self.assertIn("first_path_toa_accuracy_unverified",
                      T.MEASURED_ROW_REASON_TOA_UNVERIFIED)
        self.assertNotIn("first_path_toa_accuracy_unverified",
                         T.MEASURED_ROW_REASON)
        # C++ row order: sync, then code, then sfd.
        self.assertEqual([(r.sync_repetitions, r.code_index, str(r.sfd_mode))
                          for r in caps.phy_matrix[:7]],
                         [(16, 9, "4z1"), (16, 9, "4z2"), (16, 9, "4z3"),
                          (16, 9, "4z4"), (16, 9, "decawave"), (16, 9, "ieee"),
                          (16, 10, "4z1")])

    @unittest.skipUnless(os.path.isfile(_WHITELIST_CSV),
                         "measured PHY whitelist CSV is not in this checkout")
    def test_whitelist_mirrors_the_measured_csv_exactly(self):
        """Mirror the CSV's supported rows and add no combination it lacks."""
        rows = _read_whitelist_csv()
        self.assertEqual(len(rows), 84, "the measured matrix changed shape")
        supported = {(float(r["native_rate_hz"]), int(r["code_index"]),
                      int(r["sync_repetitions"]), r["sfd_mode"])
                     for r in rows if r["whitelist"] == "supported"}
        self.assertEqual(len(supported), 48)

        caps = T.capabilities()
        mine = {(row.native_rate_hz, row.code_index, row.sync_repetitions,
                 str(row.sfd_mode)) for row in caps.phy_matrix}
        self.assertEqual(mine, supported,
                         "the Python whitelist is not the CSV's supported set")
        self.assertEqual(mine - supported, set(),
                         "the Python whitelist invented rows the CSV lacks")

        # Every row the CSV does NOT mark supported must be refused.  That
        # covers the explicit `unsupported` rows, including the one combination
        # that is not even a valid SFD mode and so cannot be named here.
        accepted = set()
        for r in rows:
            if r["whitelist"] == "supported":
                continue
            sfd_mode = T.SfdMode.from_string(r["sfd_mode"])
            if sfd_mode is None:
                continue  # e.g. the `bogus_sfd` row: not an enum value at all
            if caps.lookup_phy(float(r["native_rate_hz"]),
                               int(r["code_index"]),
                               int(r["sync_repetitions"]), sfd_mode, 32,
                               True).allowed:
                accepted.add((r["code_index"], r["sync_repetitions"],
                              r["sfd_mode"]))
        self.assertEqual(accepted, set(),
                         "a combination the CSV rejects was accepted: %s"
                         % sorted(accepted))
        # And the admitted SYNC set is exactly the CSV's supported lengths.
        csv_syncs = sorted({int(r["sync_repetitions"]) for r in rows
                            if r["whitelist"] == "supported"})
        self.assertEqual(csv_syncs, [16, 64])
        self.assertEqual(sorted(caps.sync_repetitions), csv_syncs)

    @unittest.skipUnless(os.path.isfile(_WHITELIST_CSV),
                         "measured PHY whitelist CSV is not in this checkout")
    def test_rejection_reasons_are_the_measured_csv_cells_verbatim(self):
        """A Python refusal and a measured CSV cell must be greppable as one.

        The reason strings are copied byte for byte, including the QA's
        capital-Y typo in "syMBOLs" and the whitelist CSV's SHORT stage-failure
        tokens, precisely so an operator can grep a config rejection message
        straight to the measurement that caused it.  If the QA rewords a token,
        regenerate this table rather than loosening the comparison.
        """
        rows = _read_whitelist_csv()
        csv_rej = {}
        for r in rows:
            if r["whitelist"] == "supported":
                continue
            if T.SfdMode.from_string(r["sfd_mode"]) is None:
                continue
            csv_rej[(int(r["code_index"]), int(r["sync_repetitions"]),
                     r["sfd_mode"])] = r["reason"]
        mine_rej = {(row.code_index, row.sync_repetitions, str(row.sfd_mode)):
                    row.reason
                    for row in T.capabilities().phy_matrix_rejections}
        self.assertEqual(len(csv_rej), 35, "the CSV's rejected row set changed")
        self.assertEqual(set(mine_rej), set(csv_rej),
                         "the rejection keys drifted from the CSV")
        mismatched = {k: (csv_rej[k], mine_rej[k]) for k in csv_rej
                      if csv_rej[k] != mine_rej[k]}
        self.assertEqual(mismatched, {},
                         "rejection reasons drifted from the measured CSV; "
                         "update the tables in twr_config.py to match "
                         "qa_uwb_twr_phy_matrix.cc: %s"
                         % sorted(mismatched.items())[:2])
        # Spot-check that each measured cause is the one we expect to find.
        self.assertEqual(csv_rej[(9, 1, "4z2")], T.STAGE_CFO_FAILED_REASON)
        self.assertEqual(csv_rej[(9, 2, "4z2")], T.STAGE_CFO_FAILED_REASON)
        self.assertEqual(csv_rej[(9, 4, "4z2")], T.STAGE_CIR_FAILED_REASON)
        self.assertEqual(csv_rej[(9, 8, "4z2")], T.STAGE_CIR_FAILED_REASON)
        self.assertEqual(csv_rej[(9, 100, "4z2")], T.API_LIST_REASON)
        self.assertEqual(csv_rej[(13, 64, "4z2")], T.CODE_INDEX_REASON)
        # 32: the PHR token alone -- it decodes and is CFO-exact.
        self.assertEqual(csv_rej[(9, 32, "4z2")], T.phr_self_description_reason(32))
        # 1024: the CFO token ALONE.  It must NOT carry a PHR reason, because
        # encode_phr19 advertises index 2 = 1024, which is exactly right.
        self.assertEqual(csv_rej[(9, 1024, "4z2")], T.cfo_fit_leak_reason(1024))
        self.assertNotIn("phr_preamble_duration",
                         csv_rej[(9, 1024, "4z2")])
    @unittest.skipUnless(os.path.isfile(_WHITELIST_CSV),
                         "measured PHY whitelist CSV is not in this checkout")
    def test_sixteen_sync_is_admitted_but_not_yet_fit_for_ranging(self):
        """16 SYNC is measured and ADMITTED; its ToA accuracy is still open.

        The re-measurement against a freshly rebuilt library is done and the
        verdict is positive on every measured axis, so 16 SYNC validates.  What
        remains unverified is the first-path / ToA accuracy, and ranging
        accuracy is a function of exactly that -- so 16 is flagged, not claimed.
        """
        rows = _read_whitelist_csv()
        sixteen = [r for r in rows if int(r["sync_repetitions"]) == 16]
        self.assertEqual(len(sixteen), 24, "the CSV's 16 SYNC row set changed")
        for r in sixteen:
            self.assertEqual(r["whitelist"], "supported")
            # Its own cell carries the caveat, byte for byte.
            self.assertEqual(r["reason"], T.SYNC_16_CAVEAT_REASON)

        # The config layer ACCEPTS it: no structural objection remains.
        c = _minimal()
        c.phy.preamble_symbols = 16
        report = T.validate(c)
        self.assertTrue(report.ok(),
                        "16 SYNC was rejected: %s" % report.to_string())
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        self.assertEqual(T.sync_repetition_reasons(16), [])
        self.assertTrue(T.capabilities().sync_repetitions_supported(16))
        # 16 is SELF-DESCRIBING: encode_phr19 maps <= 16 to index 0 = 16, a legal
        # 802.15.4a BPRF duration, so no PHR objection is raised.
        self.assertEqual(T.phr_preamble_duration_index(16), 0)
        self.assertEqual(T.phr_advertised_sync_symbols(0), 16)

        # ... but it is FLAGGED as not fit for ranging, and the reason is the
        # measured caveat, not a guess.
        self.assertTrue(T.sync_reps_needing_toa_validation(16))
        self.assertIn("first_path_toa_accuracy_unverified",
                      T.SYNC_16_CAVEAT_REASON)
        sixteen_rows = [r for r in T.capabilities().phy_matrix
                        if r.sync_repetitions == 16]
        self.assertEqual(len(sixteen_rows), 24)
        for row in sixteen_rows:
            self.assertEqual(row.reason,
                             T.MEASURED_ROW_REASON_TOA_UNVERIFIED)
        # The module-level statement must be the FINAL one, not a
        # "still re-measuring" placeholder.
        self.assertIn("measured_and_admitted",
                      T.SYNC_16_TOA_UNVERIFIED_REASON)
        self.assertIn("UNVERIFIED", T.SYNC_16_TOA_UNVERIFIED_REASON)
        self.assertIn("until_m2_measures_it", T.SYNC_16_TOA_UNVERIFIED_REASON)
        self.assertNotIn("pending_re_measurement",
                         T.SYNC_16_TOA_UNVERIFIED_REASON)

    def test_toa_validation_predicate(self):
        """Mirrors C++ twr_sync_reps_needing_toa_validation()."""
        self.assertEqual(T.SYNC_REPS_NEEDING_TOA_VALIDATION, (16,))
        self.assertTrue(T.sync_reps_needing_toa_validation(16))
        self.assertFalse(T.sync_reps_needing_toa_validation(64))
        # Exactly one length needs it; every other API length does not.
        for n in (1, 2, 4, 8, 16, 32, 64, 100, 128, 256, 512, 1024, 2048, 4096):
            self.assertEqual(T.sync_reps_needing_toa_validation(n), n == 16,
                             "sync_reps_needing_toa_validation(%d)" % n)
        for n in (-1, 0, 3, 5, 17, 1000, 100000):
            self.assertFalse(T.sync_reps_needing_toa_validation(n))
        # The predicate is about RANGING FITNESS, not about the whitelist: 16 is
        # both admitted and flagged, while 128 is refused by the whitelist alone.
        self.assertTrue(T.capabilities().sync_repetitions_supported(16))
        self.assertTrue(T.sync_reps_needing_toa_validation(16))
        self.assertFalse(T.capabilities().sync_repetitions_supported(128))
        self.assertFalse(T.sync_reps_needing_toa_validation(128))
        # A predicate that returned True for everything would be useless: it has
        # to discriminate.
        self.assertLess(len(T.SYNC_REPS_NEEDING_TOA_VALIDATION),
                        len(T.capabilities().sync_repetitions))

    def test_both_measured_sync_lengths_are_accepted(self):
        # 64 SYNC is the phase-1 ranging profile.
        c = _minimal()
        c.phy.preamble_symbols = 64
        self.assertTrue(T.validate(c).ok(), T.validate(c).to_string())
        # All four API codes are measured at 64 SYNC.
        for code in (9, 10, 11, 12):
            d = _minimal()
            d.phy.tx_preamble_code = code
            d.phy.rx_preamble_code = code
            self.assertTrue(T.validate(d).ok(),
                            "code %d at 64 SYNC was rejected: %s"
                            % (code, T.validate(d).to_string()))
        # All six measured SFD modes, with the matching symbol count.
        for sfd in T.SfdMode:
            d = _minimal()
            d.frame.sfd_mode = sfd
            d.frame.sfd_symbols = T.sfd_mode_symbols(sfd)
            self.assertTrue(T.validate(d).ok(),
                            "SFD %s at 64 SYNC was rejected: %s"
                            % (sfd, T.validate(d).to_string()))
        # ... and 16 SYNC is accepted too, across all four codes.  Its
        # first-path / ToA caveat is a ranging-fitness flag, not a rejection.
        e = _minimal()
        e.phy.preamble_symbols = 16
        self.assertTrue(T.validate(e).ok(), T.validate(e).to_string())
        for code in (9, 10, 11, 12):
            d = _minimal()
            d.phy.preamble_symbols = 16
            d.phy.tx_preamble_code = code
            d.phy.rx_preamble_code = code
            self.assertTrue(T.validate(d).ok(),
                            "code %d at 16 SYNC was rejected: %s"
                            % (code, T.validate(d).to_string()))
        self.assertFalse(T.sync_reps_needing_toa_validation(64))

    def test_sync_lengths_above_64_are_rejected_with_measured_reasons(self):
        """Encode the two MEASURED defects, so they cannot be forgotten.

        (a) the carrier-offset least-squares fit synthesises the repetitions
            before the last 40 with zero matched-filter phase, and the hardcoded
            cfo_skip_initial_repetitions = 24 coincides with 64 - 40 ONLY at
            64 SYNC, so above 64 zero-phase points leak into the fit;
        (b) encode_phr19 maps 128/256/512 to preamble-duration index 1 (= 64)
            and 2048 to index 2 (= 1024), so the PHR does not self-describe;
        (c) 128/256/512/2048 are not legal IEEE 802.15.4a BPRF preamble
            durations (legal: 16/64/1024/4096) -- a DERIVED, additional reason.
        """
        # (a) applies to every length above 64.
        for n in (128, 256, 512, 1024, 2048):
            reasons = T.sync_repetition_reasons(n)
            cfo = [r for r in reasons if r.startswith("cfo_fit_includes_")]
            self.assertEqual(len(cfo), 1, "no CFO reason for %d SYNC" % n)
            self.assertEqual(cfo[0], T.cfo_fit_leak_reason(n))
            self.assertIn("_zero_peak_corr_generated_peaks", cfo[0])

        # (b) 128/256/512 -> index 1 (= 64); 2048 -> index 2 (= 1024).
        for n, idx, advertised in ((128, 1, 64), (256, 1, 64), (512, 1, 64),
                                   (2048, 2, 1024)):
            reasons = T.sync_repetition_reasons(n)
            phr = [r for r in reasons if r.startswith("phr_preamble_duration_")]
            self.assertEqual(len(phr), 1, "no PHR reason for %d SYNC" % n)
            self.assertEqual(phr[0], T.phr_self_description_reason(n))
            self.assertEqual(phr[0], T.PHR_DURATION_REASON_TEMPLATE.format(
                idx=idx, advertised=advertised, requested=n))
        # 1024 is the exception and MUST stay one: the PHR DOES describe it
        # (index 2 = 1024), so its refusal rests solely on the CFO-fit bias.
        self.assertEqual(T.phr_self_description_reason(1024), "")
        self.assertEqual([r for r in T.sync_repetition_reasons(1024)
                          if r.startswith("phr_preamble_duration_")], [])
        self.assertEqual(T.phr_preamble_duration_index(1024), 2)
        self.assertEqual(T.phr_advertised_sync_symbols(2), 1024)

        # (c) the derived illegal-length reason, and the 1024/16/64 exemptions.
        for n in (128, 256, 512, 2048):
            self.assertIn(T.ILLEGAL_LENGTH_REASON_TEMPLATE.format(n=n),
                          T.sync_repetition_reasons(n))
        for n in (16, 64, 1024, 4096):
            self.assertNotIn(T.ILLEGAL_LENGTH_REASON_TEMPLATE.format(n=n),
                             T.sync_repetition_reasons(n))
        # 16 and 64 are the two measured lengths with no structural objection.
        self.assertEqual(T.sync_repetition_reasons(64), [])
        self.assertEqual(T.sync_repetition_reasons(16), [])
        # 64 is clean precisely because 64 - 40 == cfo_skip_initial_repetitions;
        # 16 is cleaner still, because its measured tail covers the WHOLE
        # preamble, so no peak is synthesised at all.
        self.assertEqual(64 - T.CFO_TAIL_REPETITIONS,
                         T.CFO_SKIP_INITIAL_REPETITIONS)
        self.assertNotEqual(128 - T.CFO_TAIL_REPETITIONS,
                            T.CFO_SKIP_INITIAL_REPETITIONS)
        self.assertEqual(T.cfo_fit_leak_reason(64), "")
        self.assertEqual(T.cfo_fit_leak_reason(16), "")

        # And the config layer refuses each of them, quoting the measured reason.
        for n in (128, 256, 512, 1024, 2048):
            c = _minimal()
            c.phy.preamble_symbols = n
            report = T.validate(c)
            v = self.assert_has(report, "phy.preamble_symbols",
                                T.ConfigReason.UNSUPPORTED)
            self.assertIn("cfo_fit_includes_", v.message)
            self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
            self.assertEqual(v.requirement, "REQ-PHY-01")
            self.assertFalse(effective_config_ok(c))
        # 1024's message must not blame the PHR.
        c = _minimal()
        c.phy.preamble_symbols = 1024
        v = self.assert_has(T.validate(c), "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        self.assertNotIn("phr_preamble_duration", v.message)

    def test_thirty_two_sync_is_rejected_for_the_phr_alone(self):
        """32 SYNC decodes byte-exactly and is CFO-exact -- and is still refused.

        Its measured tail covers the whole preamble (32 measured peaks, zero
        synthesised), so neither the CFO stage nor the CIR stage objects.  The
        ONLY measured cause is that encode_phr19 advertises preamble-duration
        index 1 (= 64), so the PHR does not self-describe.
        """
        reasons = T.sync_repetition_reasons(32)
        self.assertEqual(len([r for r in reasons
                              if r.startswith("cfo_fit_includes_")]), 0)
        self.assertEqual([r for r in reasons
                          if r.startswith("stage")], [])
        self.assertIn(T.phr_self_description_reason(32), reasons)
        self.assertEqual(T.phr_preamble_duration_index(32), 1)
        self.assertEqual(T.phr_advertised_sync_symbols(1), 64)
        c = _minimal()
        c.phy.preamble_symbols = 32
        report = T.validate(c)
        v = self.assert_has(report, "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        # The FIRST refusal is the PreambleLength gate: 32 is not a length this
        # decoder/CFO path implements, and the reason says WHY in its own
        # words -- the PHR does not self-describe it.  The measured CSV cell is
        # still the module's second, greppable view of the same fact
        # (sync_repetition_reasons above); it is simply not the first refusal.
        self.assertEqual(v.message,
                         T.preamble_length_unsupported_reason(32))
        self.assertIn("preamble-duration index 1 (= 64)", v.message)
        self.assertNotIn("cfo_fit_includes_", v.message)
        self.assertNotIn("stage", v.message)
        # Every one of the three gates refuses it, and none of them is silent.
        self.assertEqual(len([x for x in report.violations
                              if x.field == "phy.preamble_symbols"]), 3)
        self.assertFalse(effective_config_ok(c))

    def test_every_other_sync_length_is_refused_too(self):
        for n in (1, 2, 4, 8, 32, 100, 4096):
            c = _minimal()
            c.phy.preamble_symbols = n
            self.assert_has(T.validate(c), "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        # 0 is "not stated": a missing value, not an unsupported length.
        c = _minimal()
        c.phy.preamble_symbols = 0
        self.assert_has(T.validate(c), "phy.preamble_symbols",
                        T.ConfigReason.ZERO_VALUE)
        # 16 is NOT in that list: it is measured and admitted.
        d = _minimal()
        d.phy.preamble_symbols = 16
        self.assertTrue(T.validate(d).ok())

    def test_preambles_too_short_for_the_measured_chain_are_refused(self):
        """Measured: the chain itself fails below 16 SYNC.

        1 and 2 SYNC leave the CFO stage with fewer than four measured peaks
        (measured peaks == 1 and 2), and 4 and 8 leave the CIR estimator fewer
        repetitions than its 10-repetition skip (measured peaks == 4 and 8).
        """
        for n in (1, 2):
            reasons = T.sync_repetition_reasons(n)
            self.assertIn(T.CFO_MIN_PEAKS_REASON_TEMPLATE.format(
                min=T.CFO_MIN_MEASURED_PEAKS), reasons)
            c = _minimal()
            c.phy.preamble_symbols = n
            v = self.assert_has(T.validate(c), "phy.preamble_symbols",
                                T.ConfigReason.UNSUPPORTED)
            self.assertEqual(v.message,
                             T.preamble_length_unsupported_reason(n))
            self.assertIn("fewer than the 4 measured peaks", v.message)
            # The whitelist CSV's SHORT cell is still what the measured table
            # carries for this length, and it is still the module's second,
            # greppable view of the same measured fact.
            self.assertEqual(T.STAGE_CFO_FAILED_REASON, "stage2_cfo_failed")
            self.assertIn(T.STAGE_CFO_FAILED_REASON,
                          T.capabilities().phy_matrix_rejections[0].reason
                          + T.capabilities().phy_matrix_rejections[1].reason)
        for n in (4, 8):
            reasons = T.sync_repetition_reasons(n)
            self.assertIn(T.CIR_SKIP_REASON_TEMPLATE.format(
                skip=T.CIR_SKIP_INITIAL_REPETITIONS, n=n), reasons)
            c = _minimal()
            c.phy.preamble_symbols = n
            v = self.assert_has(T.validate(c), "phy.preamble_symbols",
                                T.ConfigReason.UNSUPPORTED)
            self.assertEqual(v.message,
                             T.preamble_length_unsupported_reason(n))
            self.assertIn("10 initial repetitions", v.message)
            self.assertEqual(T.STAGE_CIR_FAILED_REASON, "stage4_cir_failed")
        self.assertEqual(T.CFO_MIN_MEASURED_PEAKS, 4)
        self.assertEqual(T.CIR_SKIP_INITIAL_REPETITIONS, 10)
        # A non-power-of-two length is outside the modulator API list.
        c = _minimal()
        c.phy.preamble_symbols = 100
        v = self.assert_has(T.validate(c), "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        self.assertEqual(v.message,
                         T.preamble_length_unsupported_reason(100))
        self.assertIn("not a SYNC repetition count this build implements",
                      v.message)
        self.assertIn(T.API_LIST_REASON,
                      "; ".join(T.sync_repetition_reasons(100)))
        self.assertFalse(T.sync_reps_supported(100))

    def test_sync_lengths_above_64_are_rejected_with_measured_reasons(self):
        """Encode the two MEASURED defects, so they cannot be forgotten.

        (a) the CFO least-squares fit synthesises the repetitions before the
            last 40 with zero matched-filter phase, and the hardcoded
            cfo_skip_initial_repetitions = 24 coincides with 64 - 40 ONLY at
            64 SYNC, so above 64 zero-phase points leak into the fit;
        (b) encode_phr19 maps 128/256/512 to preamble-duration index 1 (= 64)
            and 2048 to index 2 (= 1024), so the PHR does not self-describe;
        (c) 128/256/512/2048 are not legal IEEE 802.15.4a BPRF preamble
            durations (legal: 16/64/1024/4096).
        """
        # (a) applies to every length above 64.
        for n in (128, 256, 512, 1024, 2048):
            reasons = T.sync_repetition_reasons(n)
            cfo = [r for r in reasons if r.startswith("cfo_fit_includes_")]
            self.assertEqual(len(cfo), 1, "no CFO reason for %d SYNC" % n)
            self.assertEqual(
                cfo[0], T.CFO_FIT_REASON_TEMPLATE.format(
                    tail_first=n - T.CFO_TAIL_REPETITIONS,
                    skip=T.CFO_SKIP_INITIAL_REPETITIONS))
            self.assertIn("_zero_peak_corr_generated_peaks", cfo[0])

        # (b) 128/256/512 -> index 1 (= 64); 2048 -> index 2 (= 1024).
        for n, idx, advertised in ((128, 1, 64), (256, 1, 64), (512, 1, 64),
                                   (2048, 2, 1024)):
            reasons = T.sync_repetition_reasons(n)
            phr = [r for r in reasons if r.startswith("phr_preamble_duration_")]
            self.assertEqual(len(phr), 1, "no PHR reason for %d SYNC" % n)
            self.assertEqual(phr[0], T.PHR_DURATION_REASON_TEMPLATE.format(
                idx=idx, advertised=advertised, requested=n))
        # 1024 is the exception: the PHR DOES describe it (index 2 = 1024), so
        # the refusal must NOT claim otherwise -- it rests on the CFO-fit
        # defect and on the fact that 1024 was never swept by the matrix.
        self.assertEqual([r for r in T.sync_repetition_reasons(1024)
                          if r.startswith("phr_preamble_duration_")], [])
        self.assertEqual(T.phr_preamble_duration_index(1024), 2)
        self.assertEqual(T.phr_advertised_sync_symbols(2), 1024)

        # (c) the illegal-length reason, and the 64 SYNC exemption.
        for n in (128, 256, 512, 2048):
            self.assertIn(T.ILLEGAL_LENGTH_REASON_TEMPLATE.format(n=n),
                          T.sync_repetition_reasons(n))
        self.assertNotIn(T.ILLEGAL_LENGTH_REASON_TEMPLATE.format(n=1024),
                         T.sync_repetition_reasons(1024))
        # 64 is the ONLY length with no structural objection at all.
        self.assertEqual(T.sync_repetition_reasons(64), [])
        # ... precisely because 64 - 40 == cfo_skip_initial_repetitions there,
        # and nowhere else.
        self.assertEqual(64 - T.CFO_TAIL_REPETITIONS,
                         T.CFO_SKIP_INITIAL_REPETITIONS)
        self.assertNotEqual(128 - T.CFO_TAIL_REPETITIONS,
                            T.CFO_SKIP_INITIAL_REPETITIONS)

        # And the config layer refuses each of them, in words that name THIS
        # software and cite the vendor -- never "the chip cannot do it".
        for n in (128, 256, 512, 1024, 2048):
            c = _minimal()
            c.phy.preamble_symbols = n
            report = T.validate(c)
            v = self.assert_has(report, "phy.preamble_symbols",
                                T.ConfigReason.UNSUPPORTED)
            self.assertEqual(v.message,
                             T.preamble_length_unsupported_reason(n))
            self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
            self.assertEqual(v.requirement, "REQ-PHY-01")
            if n in (128, 256, 512):
                self.assertIn("max(0, reps - 64)", v.message)
                self.assertIn("measured at 128 SYNC", v.message)
                self.assertIn("Qorvo", v.message)
            elif n == 1024:
                self.assertIn("same CFO-fit bias as 128/256/512", v.message)
                self.assertIn("a software limit, not a hardware one",
                              v.message)
            else:
                self.assertIn("work-grid TX buffer", v.message)
                self.assertIn("not a hardware capability statement", v.message)
            self.assertNotIn("the hardware cannot", v.message)
            self.assertNotIn("structurally impossible", v.message)
            self.assertFalse(effective_config_ok(c))


    def test_data_rates_outside_the_measured_one_are_refused(self):
        for rate in T.DataRate:
            c = _minimal()
            c.phy.data_rate = rate
            # The PHR rate is a SEPARATE axis and is NOT set to the payload
            # rate: M0 did that, and required the two to be equal, which is
            # the inverse of what uwb_hrp_mod_core.h transmits.
            report = T.validate(c)
            if rate is T.DataRate.R6P8M:
                self.assertTrue(report.ok(), report.to_string())
                continue
            v = self.assert_has(report, "phy.data_rate",
                                T.ConfigReason.UNSUPPORTED)
            self.assertIn(str(rate), v.message)
            self.assertIn("6_81_mbps", v.message)

    def test_phr_rate_is_not_compared_with_the_payload_rate(self):
        """M0 review R2: the rule was backwards and is GONE.

        M0 typed ``phr_rate`` as a ``DataRate`` and required it to EQUAL
        ``data_rate``.  That accepted the 6.81 Mb/s PHR this modulator cannot
        produce and rejected the 0.85 Mb/s PHR it does produce.  The PHR is a
        different modulation at a different rate: 21 SEC-DED-coded symbols of
        512 chips ahead of an 8-chips-per-burst-of-64 payload.  The 2-bit
        data-rate field INSIDE the PHR describes the payload that follows it and
        says nothing about how the PHR itself is transmitted.
        """
        # The exact pair M0 rejected must now be accepted.
        c = _minimal()
        c.phy.phr_rate = T.PhrRate.STANDARD_850K
        c.phy.data_rate = T.DataRate.R6P8M
        self.assertNotEqual(str(c.phy.phr_rate), str(c.phy.data_rate))
        self.assertTrue(T.validate(c).ok(), T.validate(c).to_string())

        # A payload-rate spelling is no longer a legal PHR rate: the two axes
        # no longer share a value domain.
        self.assertIsNone(T.PhrRate.from_string("6p8m"))
        self.assertIsNone(T.PhrRate.from_string("27m"))
        self.assertEqual([m.value for m in T.PhrRate],
                         ["850k", "same_as_data"])

        # same_as_data is a legal 802.15.4 / DW3xxx option this build has no
        # code path for, and the refusal must say so about THIS SOFTWARE.
        d = _minimal()
        d.phy.phr_rate = T.PhrRate.SAME_AS_DATA
        v = self.assert_has(T.validate(d), "phy.phr_rate",
                            T.ConfigReason.UNSUPPORTED)
        self.assertEqual(v.requirement, "REQ-PHY-02")
        self.assertIn("exactly ONE PHR code path", v.message)
        self.assertIn("not a claim about any Qorvo part", v.message)
        self.assertIn("It is NOT compared with phy.data_rate", v.message)
        # Only the implemented rate may be selected.
        self.assertTrue(T.phr_rate_is_implemented(T.PhrRate.STANDARD_850K))
        self.assertFalse(T.phr_rate_is_implemented(T.PhrRate.SAME_AS_DATA))
        self.assertEqual(T.phr_rate_symbols(T.PhrRate.STANDARD_850K), 21)
        self.assertEqual(T.phr_rate_symbols(T.PhrRate.SAME_AS_DATA), 0)
        self.assertIn("851.2 kb/s",
                      T.phr_rate_unsupported_reason(T.PhrRate.STANDARD_850K))

    def test_channels_outside_channel_5_are_refused(self):
        for ch in (0, 4, 6, 8, 9, 16):
            c = _minimal()
            c.phy.channel = ch
            c.phy.center_frequency_hz = T.uwb_channel_center_frequency_hz(ch)
            c.calibration.record.channel = ch
            v = self.assert_has(T.validate(c), "phy.channel",
                                T.ConfigReason.UNSUPPORTED)
            self.assertIn("channel 5", v.message)
        for ch in (17, 200):
            c = _minimal()
            c.phy.channel = ch
            self.assert_has(T.validate(c), "phy.channel",
                            T.ConfigReason.INDEX_OUT_OF_RANGE)

    def test_prf_class_other_than_bprf64_is_refused(self):
        for prf in (T.PrfClass.HPRF64, T.PrfClass.HPRF400):
            c = _minimal()
            c.phy.prf_class = prf
            self.assert_has(T.validate(c), "phy.prf_class",
                            T.ConfigReason.UNSUPPORTED)

    def test_code_index_outside_9_to_12_is_refused(self):
        for code in (1, 8, 13, 32, 255):
            c = _minimal()
            c.phy.tx_preamble_code = code
            c.phy.rx_preamble_code = code
            report = T.validate(c)
            self.assert_has(report, "phy.tx_preamble_code",
                            T.ConfigReason.UNSUPPORTED)
            self.assert_has(report, "phy.rx_preamble_code",
                            T.ConfigReason.UNSUPPORTED)
        # 0 is "not stated", which is a missing value.
        c = _minimal()
        c.phy.tx_preamble_code = 0
        c.phy.rx_preamble_code = 0
        self.assert_has(T.validate(c), "phy.tx_preamble_code",
                        T.ConfigReason.ZERO_VALUE)
        # TX and RX codes must agree: one profile uses one code.
        d = _minimal()
        d.phy.tx_preamble_code = 9
        d.phy.rx_preamble_code = 10
        self.assert_has(T.validate(d), "phy.rx_preamble_code",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_joint_lookup_is_all_or_nothing(self):
        caps = T.capabilities()
        ok = caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 64, T.SfdMode.R4Z2,
                             127, True)
        self.assertTrue(ok.allowed)
        self.assertIs(ok.status, T.CapabilityStatus.MEASURED)
        # Any single axis off, and the whole combination is refused.
        self.assertFalse(caps.lookup_phy(T.NATIVE_RATE_CG400_HZ, 9, 64,
                                          T.SfdMode.R4Z2, 127, True).allowed)
        self.assertFalse(caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 13, 64,
                                          T.SfdMode.R4Z2, 127, True).allowed)
        self.assertFalse(caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 128,
                                          T.SfdMode.R4Z2, 127, True).allowed)
        self.assertFalse(caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 64,
                                          T.SfdMode.R4Z2, 128, True).allowed)
        self.assertFalse(caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 64,
                                          T.SfdMode.R4Z2, 127, False).allowed)

    def test_unmeasured_whitelist_refuses_every_phy_axis(self):
        """The PRE-matrix default-deny state is still reproducible here.

        ``build_unmeasured_capabilities()`` keeps the empty measured lists the
        C++ header used to ship, so the default-deny MECHANISM stays testable:
        the same config is refused there and accepted against the measured
        whitelist, and nothing in between.  It is no longer the C++ default.
        """
        caps = T.unmeasured_capabilities()
        self.assertEqual(caps.data_rates, [])
        self.assertEqual(caps.sync_repetitions, [])
        self.assertEqual(caps.sfd_modes, [])
        self.assertEqual(caps.phy_matrix, [])
        self.assertEqual(caps.phy_matrix_rejections, [])
        # ... and it is exactly the C++ header's former state, not the measured
        # one.
        self.assertEqual(caps.profile_version, "unfrozen-m0")
        self.assertEqual(len(T.capabilities().phy_matrix), 48)
        self.assertIsNot(T.default_deny_capabilities(), T.capabilities())

        c = _minimal()
        report = T.validate(c, caps)
        self.assert_has(report, "phy.data_rate", T.ConfigReason.UNSUPPORTED)
        self.assert_has(report, "phy.preamble_symbols",
                        T.ConfigReason.UNSUPPORTED)
        self.assert_has(report, "frame.sfd_mode", T.ConfigReason.UNSUPPORTED)
        for v in report.violations:
            if v.reason is T.ConfigReason.UNSUPPORTED:
                self.assertIn("phy_matrix", v.message)
        # ... and the measured whitelist accepts the very same config.
        self.assertTrue(T.validate(c, T.capabilities()).ok())
        # 16 SYNC is admitted by the measured whitelist and refused by the empty
        # one, which is exactly the difference the matrix made possible.
        d = _minimal()
        d.phy.preamble_symbols = 16
        self.assertFalse(T.validate(d, caps).ok())
        self.assertTrue(T.validate(d).ok())

    def test_capabilities_builder_only_ever_relaxes(self):
        caps = T.capabilities()
        # Adding a measured row for 128 SYNC turns that refusal into an
        # acceptance, and changes nothing else.
        row = T.PhyCapabilityRow(
            native_rate_hz=T.NATIVE_RATE_UC200_HZ, code_index=9,
            sync_repetitions=128, sfd_mode=T.SfdMode.R4Z2,
            max_psdu_bytes=T.MAX_PSDU_BYTES, ranging=True,
            status=T.CapabilityStatus.MEASURED, reason="SYNTHETIC, not measured")
        wider = T.CapabilitiesBuilder(caps).add_sync_repetitions(128) \
            .add_phy_row(row).build()
        c = _minimal()
        c.phy.preamble_symbols = 128
        self.assertFalse(T.validate(c, caps).ok())
        # A capability table can only relax the WHITELIST gate.  It cannot
        # relax the PreambleLength gate, which is a property of this
        # repository's demodulator / CFO path and is not a measurement at all:
        # 128 SYNC still produces a biased carrier-offset estimate here, and
        # adding a row to a table does not change that.  So the builder turns
        # the "not measured" refusal into the "measured and refused" one, and
        # the config stays rejected -- with the SAME field and reason.
        report = T.validate(c, wider)
        self.assertFalse(report.ok(), report.to_string())
        # The whitelist gate stops firing (the joint lookup now finds a row),
        # but the PreambleLength gate does not, and it is the same field and
        # the same reason: the config is still refused, one violation fewer.
        narrow = [v for v in report.violations
                  if v.field == "phy.preamble_symbols"]
        base_v = [v for v in T.validate(c, caps).violations
                  if v.field == "phy.preamble_symbols"]
        # Three gates in all, all on the same field and the same reason: the
        # PreambleLength gate, the whitelist gate and the joint PHY lookup.
        # Adding a row silences the last two and leaves the first.
        self.assertEqual(len(base_v), 3)
        self.assertEqual(len(narrow), 1)
        self.assertEqual([v.reason for v in base_v],
                         [T.ConfigReason.UNSUPPORTED] * 3)
        self.assertEqual([v.reason for v in narrow], [T.ConfigReason.UNSUPPORTED])
        self.assertEqual(narrow[0].message,
                         T.preamble_length_unsupported_reason(128))
        # A 16- or 64-SYNC length is a different story: there the builder's
        # row IS enough, because the PreambleLength gate already passes.
        ok_row = T.PhyCapabilityRow(
            native_rate_hz=T.NATIVE_RATE_UC200_HZ, code_index=9,
            sync_repetitions=64, sfd_mode=T.SfdMode.R4Z2,
            max_psdu_bytes=T.MAX_PSDU_BYTES, ranging=True,
            status=T.CapabilityStatus.MEASURED, reason="SYNTHETIC, not measured")
        wider64 = T.CapabilitiesBuilder(caps).add_phy_row(ok_row).build()
        self.assertTrue(T.validate(_minimal(), wider64).ok())
        # The original is untouched: a builder copies.
        self.assertEqual(caps.sync_repetitions, [16, 64])
        self.assertEqual(len(caps.phy_matrix), 48)
        self.assertEqual(len(wider.phy_matrix), 49)
        # A rejection row can only improve a message, never admit anything.  Use
        # a length with no measured row of its own, so the row is not shadowed
        # by a real one.
        c3 = _minimal()
        c3.phy.preamble_symbols = 3
        self.assertFalse(T.validate(c3, caps).ok())
        # The MEASURED-cell view is the JOINT LOOKUP's message, which is the
        # last of the three refusals on this field; the first is the
        # PreambleLength gate and the second the whitelist gate.
        def joint_message(report):
            msgs = [v.message for v in report.violations
                    if v.field == "phy.preamble_symbols"]
            self.assertEqual(len(msgs), 3)
            return msgs[-1]
        derived = joint_message(T.validate(c3, caps))
        self.assertIn("outside_api_list", derived)
        with_row = T.CapabilitiesBuilder(caps).add_phy_rejection_row(
            T.PhyCapabilityRow(
                native_rate_hz=T.NATIVE_RATE_UC200_HZ, code_index=9,
                sync_repetitions=3, sfd_mode=T.SfdMode.R4Z2,
                max_psdu_bytes=T.MAX_PSDU_BYTES, ranging=True,
                status=T.CapabilityStatus.MEASURED,
                reason="measured_cell_quoted_instead_of_the_derived_analysis"
            )).build()
        v = self.assert_has(T.validate(c3, with_row), "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        self.assertIn("measured_cell_quoted_instead_of_the_derived_analysis",
                      joint_message(T.validate(c3, with_row)))
        self.assertFalse(T.validate(c3, with_row).ok(),
                         "a rejection row must never admit a combination")
        # The shipped instance is unaffected by the builder.
        self.assertEqual(len(T.capabilities().phy_matrix_rejections), 35)

    def test_ranging_bit_false_is_refused(self):
        c = _minimal()
        c.frame.ranging_bit = False
        v = self.assert_has(T.validate(c), "frame.ranging_bit",
                            T.ConfigReason.FIELD_CONFLICT)
        self.assertEqual(v.requirement, "REQ-PROTO-05")
        # The joint capability lookup refuses it too, since no measured row
        # has ranging = false.
        self.assert_has(T.validate(c), "phy.preamble_symbols",
                        T.ConfigReason.UNSUPPORTED)

    def test_schema_version_mismatch_is_refused(self):
        c = _minimal()
        c.meta.schema_version = "twr-config/99"
        self.assert_has(T.validate(c), "meta.schema_version",
                        T.ConfigReason.UNSUPPORTED)
        d = _minimal()
        d.meta.schema_version = ""
        self.assert_has(T.validate(d), "meta.schema_version",
                        T.ConfigReason.EMPTY_VALUE)
        e = _minimal()
        e.meta.profile_version = ""
        self.assert_has(T.validate(e), "meta.profile_version",
                        T.ConfigReason.EMPTY_VALUE)


def effective_config_ok(c):
    return T.effective_config(c).ok


# ==========================================================================
# D. STS and the reserved Report frame
# ==========================================================================


class TestOutOfScope(TwrConfigTestBase):

    def test_sts_is_refused_for_every_non_off_mode(self):
        for mode in T.StsMode:
            if mode is T.StsMode.OFF:
                continue
            c = _minimal()
            c.frame.sts_mode = mode
            c.frame.sts_length_symbols = 128
            report = T.validate(c)
            v = self.assert_has(report, "frame.sts_mode",
                                T.ConfigReason.OUT_OF_SCOPE)
            self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
            self.assertIn("phase 1", v.message)
            self.assertEqual(v.requirement, "REQ-SCOPE-04")
            # The length is a separate conflict, not silently ignored.
            self.assert_has(report, "frame.sts_length_symbols",
                            T.ConfigReason.FIELD_CONFLICT)
        # off is fine.
        c = _minimal()
        c.frame.sts_mode = T.StsMode.OFF
        c.frame.sts_length_symbols = 0
        self.assertTrue(T.validate(c).ok())
        # A length with sts_mode off is still a conflict.
        d = _minimal()
        d.frame.sts_length_symbols = 128
        self.assert_has(T.validate(d), "frame.sts_length_symbols",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_capabilities_claim_no_sts_at_all(self):
        caps = T.capabilities()
        self.assertFalse(caps.sts_supported)
        self.assertEqual(caps.sts_modes, [])
        self.assertIn("REQ-SCOPE-04", caps.unsupported_sts_reason)

    def test_report_frame_type_is_reserved_and_not_implemented(self):
        # The code point exists so a future profile can claim it, but phase 1
        # rejects both places a Report could appear (REQ-PROTO-05).
        self.assertEqual(T.frame_type_timestamp_count(T.FrameType.REPORT), 0)
        c = _minimal()
        c.timing.final_to_report.value = T.Duration(600_000)
        v = self.assert_has(T.validate(c), "timing.final_to_report",
                            T.ConfigReason.OUT_OF_SCOPE)
        self.assertIn("REPORT", v.message)
        self.assertEqual(v.requirement, "REQ-PROTO-05")

        d = _minimal()
        d.timeouts.report_rx_window.value = T.Duration(20_000)
        v2 = self.assert_has(T.validate(d), "timeouts.report_rx_window",
                             T.ConfigReason.OUT_OF_SCOPE)
        self.assertEqual(v2.requirement, "REQ-PROTO-05")

        # None of the three implemented frame budgets counts a Report: a
        # Report has no codec at all, so there is no length to report.
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.POLL,
                                             T.FcsAppender.PHY), 16)
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.RESPONSE,
                                             T.FcsAppender.PHY), 26)
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.FINAL,
                                             T.FcsAppender.PHY), 31)
        self.assertEqual(T.frame_geometry_for(T.FrameProfileId.TWR_V1).on_air_bytes(
            T.FrameType.REPORT), 0 if False else
            T.FRAME_HEADER_BYTES + 0 * 5 + T.FRAME_FCS_BYTES)

    def test_phr_mode_none_is_refused(self):
        c = _minimal()
        c.frame.phr_mode = T.PhrMode.NONE
        v = self.assert_has(T.validate(c), "frame.phr_mode",
                            T.ConfigReason.OUT_OF_SCOPE)
        self.assertIn("RANGING", v.message)


# ==========================================================================
# E. NaN / Inf
# ==========================================================================


class TestNonFinite(TwrConfigTestBase):

    def _mutators(self):
        return (
            ("phy.center_frequency_hz",
             lambda c, v: setattr(c.phy, "center_frequency_hz", v)),
            ("radio.native_sample_rate_hz",
             lambda c, v: setattr(c.radio, "native_sample_rate_hz", v)),
            ("calibration.native_sample_rate_hz",
             lambda c, v: setattr(c.calibration, "native_sample_rate_hz", v)),
            ("tx.gain_db", lambda c, v: setattr(c.tx, "gain_db", v)),
            ("tx.iq_amplitude", lambda c, v: setattr(c.tx, "iq_amplitude", v)),
            ("tx.calibrated_tx_power_dbm",
             lambda c, v: setattr(c.tx, "calibrated_tx_power_dbm", v)),
            ("rx.gain_db", lambda c, v: setattr(c.rx, "gain_db", v)),
            ("rx.bandwidth_hz", lambda c, v: setattr(c.rx, "bandwidth_hz", v)),
            ("rx.detection_threshold",
             lambda c, v: setattr(c.rx, "detection_threshold", v)),
            ("rx.correlation_threshold",
             lambda c, v: setattr(c.rx, "correlation_threshold", v)),
            ("rx.first_path_threshold",
             lambda c, v: setattr(c.rx, "first_path_threshold", v)),
            ("rx.vendor_pac_applied_step",
             lambda c, v: setattr(c.rx, "vendor_pac_applied_step", v)),
            ("calibration.record.gain_db",
             lambda c, v: setattr(c.calibration.record, "gain_db", v)),
            ("timing.poll_to_response.quantisation_hz",
             lambda c, v: setattr(c.timing.poll_to_response,
                                 "required_quantisation_hz", v)),
            ("timeouts.rx_timeout.quantisation_hz",
             lambda c, v: setattr(c.timeouts.rx_timeout,
                                 "required_quantisation_hz", v)),
            ("timeouts.exchange_timeout.quantisation_hz",
             lambda c, v: setattr(c.timeouts.exchange_timeout,
                                 "required_quantisation_hz", v)),
            ("diagnostics.stats_cadence.quantisation_hz",
             lambda c, v: setattr(c.diagnostics.stats_cadence,
                                 "required_quantisation_hz", v)),
            ("radio.readback.sample_rate_hz",
             lambda c, v: setattr(c.radio.readback, "sample_rate_hz", v)),
            ("radio.readback.center_freq_hz",
             lambda c, v: setattr(c.radio.readback, "center_freq_hz", v)),
            ("radio.peers[0].native_sample_rate_hz",
             lambda c, v: setattr(c.radio.peers[0], "native_sample_rate_hz", v)),
        )

    def test_non_finite_is_refused_in_every_float_field(self):
        bad = (float("nan"), float("inf"), float("-inf"))
        for name, apply in self._mutators():
            for v in bad:
                c = _minimal()
                apply(c, v)
                report = T.validate(c)
                self.assert_machine_readable(report)
                self.assertTrue(
                    report.has(T.ConfigReason.NOT_FINITE),
                    "%s accepted %r: %s" % (name, v, report.to_string()))

    def test_non_finite_cannot_be_exported_to_json(self):
        for name, apply in self._mutators():
            c = _minimal()
            apply(c, float("nan"))
            with self.assertRaises(T.ConfigJsonError):
                T.to_json_string(c)
        # The error message names the field, and no NaN token is ever emitted.
        c = _minimal()
        c.rx.detection_threshold = float("inf")
        with self.assertRaises(T.ConfigJsonError) as ctx:
            T.to_json_string(c)
        self.assertIn("not finite", str(ctx.exception))
        self.assertIn("rx.detection_threshold", str(ctx.exception))

    def test_negative_and_out_of_range_floats(self):
        c = _minimal()
        c.rx.first_path_threshold = 1.5
        self.assert_has(T.validate(c), "rx.first_path_threshold",
                        T.ConfigReason.OUT_OF_RANGE)
        d = _minimal()
        d.rx.correlation_threshold = 0.1   # looser than the coarse gate
        self.assert_has(T.validate(d), "rx.correlation_threshold",
                        T.ConfigReason.FIELD_CONFLICT)
        e = _minimal()
        e.tx.iq_amplitude = 0.0
        e.tx.power_policy = T.TxPowerPolicy.IQ_AMPLITUDE
        e.tx.gain_db = None
        self.assert_has(T.validate(e), "tx.iq_amplitude",
                        T.ConfigReason.OUT_OF_RANGE)


# ==========================================================================
# F. Conflicts, indices, capacities
# ==========================================================================


class TestConflicts(TwrConfigTestBase):

    def test_channel_versus_center_frequency_conflict(self):
        c = _minimal()
        c.phy.center_frequency_hz = 6489.6e6 + 5000.0   # 5 kHz off
        report = T.validate(c)
        self.assert_has(report, "phy.center_frequency_hz",
                        T.ConfigReason.CHANNEL_FREQUENCY_MISMATCH)
        self.assert_has(report, "radio.readback.center_freq_hz",
                        T.ConfigReason.CHANNEL_FREQUENCY_MISMATCH)
        # A different channel's frequency for the same channel number.
        d = _minimal()
        d.phy.center_frequency_hz = T.uwb_channel_center_frequency_hz(9)
        self.assert_has(T.validate(d), "phy.center_frequency_hz",
                        T.ConfigReason.CHANNEL_FREQUENCY_MISMATCH)
        # Within tolerance: accepted.
        e = _minimal()
        e.phy.center_frequency_hz = 6489.6e6 + 10.0
        self.assertTrue(T.validate(e).ok(), T.validate(e).to_string())

    def test_peer_resource_and_role_conflicts(self):
        # Same physical TX channel as this endpoint.
        c = _minimal()
        c.radio.peers[0].tx_channel = c.radio.tx_channel
        v = self.assert_has(T.validate(c), "radio.peers[0].tx_channel",
                            T.ConfigReason.DUPLICATE_RESOURCE)
        self.assertEqual(v.requirement, "REQ-BASE-01")
        # Same physical RX channel.
        d = _minimal()
        d.radio.peers[0].rx_channel = d.radio.rx_channel
        self.assert_has(T.validate(d), "radio.peers[0].rx_channel",
                        T.ConfigReason.DUPLICATE_RESOURCE)
        # Both endpoints claim the same role.
        e = _minimal()
        e.radio.peers[0].role = e.session.role
        self.assert_has(T.validate(e), "radio.peers[0].role",
                        T.ConfigReason.FIELD_CONFLICT)
        # A peer that does not occupy resources is not a conflict.
        f = _minimal()
        f.radio.peers[0].tx_channel = f.radio.tx_channel
        f.radio.peers[0].rx_channel = f.radio.rx_channel
        f.radio.peers[0].role = f.session.role
        f.radio.peers[0].occupies_resources = False
        self.assertTrue(T.validate(f).ok(), T.validate(f).to_string())
        # Duplicate endpoint id.
        g = _minimal()
        g.radio.peers.append(copy.deepcopy(g.radio.peers[0]))
        report = T.validate(g)
        self.assertTrue(report.has(T.ConfigReason.DUPLICATE_RESOURCE))
        # Over capacity.
        h = _minimal()
        h.radio.peers = [T.EndpointBinding(id="p%d" % i, role=T.Role.RESPONDER,
                                          tx_channel=i, rx_channel=i,
                                          native_sample_rate_hz=(
                                              T.NATIVE_RATE_UC200_HZ),
                                          occupies_resources=False)
                         for i in range(T.MAX_PEERS_PER_CONFIG + 1)]
        self.assert_has(T.validate(h), "radio.peers",
                        T.ConfigReason.OVER_CAPACITY)

    def test_endpoints_sharing_a_device_share_the_native_rate(self):
        c = _minimal()
        c.radio.peers[0].native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        v = self.assert_has(T.validate(c), "radio.peers[0].native_sample_rate_hz",
                            T.ConfigReason.FIELD_CONFLICT)
        self.assertEqual(v.requirement, "REQ-PHY-02")

    def test_bad_native_rates_are_refused(self):
        for hz in (1.0e6, 64.0e6, 100.0e6, 500.0e6, 998.4e6, 1.0e9, 2.0e9):
            c = _minimal()
            c.radio.native_sample_rate_hz = hz
            self.assert_has(T.validate(c), "radio.native_sample_rate_hz",
                            T.ConfigReason.UNSUPPORTED)
        # 737.28 MS/s is accepted.
        c = _minimal()
        self.assertTrue(T.validate(c).ok())
        # 491.52 MS/s IS a supported native rate, but the only measured PHY
        # row is for 737.28 MS/s, so the combination is refused -- and the
        # refusal says so instead of quietly re-running the other rate.
        d = _minimal()
        d.radio.native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        d.radio.readback.sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        d.radio.peers[0].native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        d.calibration.native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        d.calibration.record.native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        report = T.validate(d)
        self.assertFalse(report.has(T.ConfigReason.UNSUPPORTED,
                                    "radio.native_sample_rate_hz"))
        v = self.assert_has(report, "phy.preamble_symbols",
                            T.ConfigReason.UNSUPPORTED)
        self.assertIn("native_rate=491520000", v.message)

    def test_psdu_over_127_bytes_is_refused(self):
        for n in (128, 200, 1000):
            c = _minimal()
            c.frame.mac_psdu_bytes = n
            self.assert_has(T.validate(c), "frame.mac_psdu_bytes",
                            T.ConfigReason.FRAME_LENGTH_OVERFLOW)
        # 127 is exactly the measured maximum and is fine.
        ok = _minimal()
        ok.frame.mac_psdu_bytes = 127
        self.assertTrue(T.validate(ok).ok(), T.validate(ok).to_string())
        # The joint capability lookup also caps the PSDU at 127.
        self.assertTrue(T.capabilities().lookup_phy(
            T.NATIVE_RATE_UC200_HZ, 9, 64, T.SfdMode.R4Z2, 127, True).allowed)
        self.assertFalse(T.capabilities().lookup_phy(
            T.NATIVE_RATE_UC200_HZ, 9, 64, T.SfdMode.R4Z2, 128, True).allowed)

    def test_poll_response_final_byte_lengths_must_fit(self):
        # The MAC PSDU the CODEC builds: header 14 + 3x5 timestamps = 29 B for
        # the Final.  The 2 FCS bytes the PHY appends are NOT part of the MAC
        # PSDU, which is the whole point of the two-fields rule: comparing the
        # on-air length against mac_psdu_bytes would understate the requirement
        # by exactly the FCS.
        need_final = T.FRAME_HEADER_BYTES + 3 * 5
        self.assertEqual(need_final, 29)
        tight = _minimal()
        tight.frame.mac_psdu_bytes = need_final - 1
        report = T.validate(tight)
        v = self.assert_has(report, "frame.mac_psdu_bytes[final]",
                            T.ConfigReason.FRAME_LENGTH_OVERFLOW)
        self.assertIn("29", v.message)
        self.assertIn("header 14 + 3x5 timestamps", v.message)
        self.assertIn("the PHY appends the FCS separately", v.message)
        self.assertIsNone(report.find("frame.mac_psdu_bytes[poll]",
                                      T.ConfigReason.FRAME_LENGTH_OVERFLOW))
        self.assertIsNone(report.find("frame.mac_psdu_bytes[response]",
                                      T.ConfigReason.FRAME_LENGTH_OVERFLOW))
        # Below the Poll's 14 bytes even the Poll does not fit.
        too_small = _minimal()
        too_small.frame.mac_psdu_bytes = T.FRAME_HEADER_BYTES - 1
        small = T.validate(too_small)
        for name in ("poll", "response", "final"):
            self.assert_has(small, "frame.mac_psdu_bytes[%s]" % name,
                            T.ConfigReason.FRAME_LENGTH_OVERFLOW)
        # Exactly 29 fits all three.
        exact = _minimal()
        exact.frame.mac_psdu_bytes = need_final
        self.assertTrue(T.validate(exact).ok(), T.validate(exact).to_string())

    def test_frame_psdu_helper_counts_the_fcs_exactly_once(self):
        # A claim, and DELIBERATELY IGNORED: a claim does not decide a length.
        # The retained three-argument call shape must keep working while losing
        # the ability to pass a number of its own -- that is the point of the
        # delegation, and it is why a 7-byte header can no longer produce a
        # 9-byte Poll.
        g = T.FrameGeometry(mac_header_bytes=7, timestamp_bytes=5,
                            mac_fcs_bytes=2, phr_bytes=12)
        # MAC appends the FCS: it is inside the MAC PSDU.
        self.assertEqual(T.frame_psdu_bytes(g, T.FrameType.POLL,
                                            T.FcsAppender.MAC), 16)
        self.assertEqual(T.frame_psdu_bytes(g, T.FrameType.RESPONSE,
                                            T.FcsAppender.MAC), 26)
        self.assertEqual(T.frame_psdu_bytes(g, T.FrameType.FINAL,
                                            T.FcsAppender.MAC), 31)
        # PHY appends it: the MAC PSDU stops before the CRC.
        self.assertEqual(T.frame_psdu_bytes(g, T.FrameType.POLL,
                                            T.FcsAppender.PHY), 14)
        self.assertEqual(T.frame_psdu_bytes(g, T.FrameType.FINAL,
                                            T.FcsAppender.PHY), 29)
        # Either way exactly one layer adds the 2 bytes on the air.
        self.assertEqual(T.frame_bytes_on_air(g, T.FrameType.FINAL,
                                              T.FcsAppender.MAC, 2), 31)
        self.assertEqual(T.frame_bytes_on_air(g, T.FrameType.FINAL,
                                              T.FcsAppender.PHY, 2), 31)
        # The two-argument form is the real one, and it is a delegation: no
        # second `header + n * timestamp` sum exists anywhere in the module.
        self.assertEqual(T.frame_psdu_bytes(T.FrameType.FINAL,
                                            T.FcsAppender.PHY), 29)
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.FINAL,
                                              T.FcsAppender.PHY), 31)

    def test_application_payload_macsdu_and_fcs_are_distinct(self):
        c = _minimal()
        c.frame.mac_psdu_bytes = 32
        c.frame.application_payload_bytes = 64
        self.assert_has(T.validate(c), "frame.application_payload_bytes",
                        T.ConfigReason.FRAME_LENGTH_OVERFLOW)
        # A payload is not required.
        ok = _minimal()
        ok.frame.application_payload_bytes = 0
        self.assertTrue(T.validate(ok).ok())
        # mac_psdu_bytes is independent of the payload: an operator may fill
        # the PSDU without carrying application bytes at all.
        g = _minimal()
        g.frame.application_payload_bytes = 20
        self.assertTrue(T.validate(g).ok(), T.validate(g).to_string())

    def test_exactly_one_layer_appends_the_fcs(self):
        c = _minimal()
        c.frame.mac_psdu_includes_fcs = True   # but the PHY is the appender
        self.assert_has(T.validate(c), "frame.mac_psdu_includes_fcs",
                        T.ConfigReason.FIELD_CONFLICT)
        d = _minimal()
        d.frame.fcs_append = T.FcsAppender.NONE
        self.assert_has(T.validate(d), "frame.fcs_append",
                        T.ConfigReason.FIELD_CONFLICT)
        # fcs_append=mac is DESCRIBABLE and NOT ENCODABLE.  The geometry
        # authority has a variant for it, so a MAC-appends claim that forgets
        # to reserve the 2 FCS bytes is caught as the FIELD disagreement it
        # is -- and the layout is separately refused as unencodable here.
        e = _minimal()
        e.frame.fcs_append = T.FcsAppender.MAC
        e.frame.mac_psdu_includes_fcs = True
        rep = T.validate(e)
        v = self.assert_has(rep, "frame.fcs_append", T.ConfigReason.UNSUPPORTED)
        self.assertEqual(v.requirement, "REQ-API-01")
        self.assertIn("only encodes the PHY-appends form", v.message)
        self.assert_has(rep, "frame.geometry.mac_fcs_bytes",
                        T.ConfigReason.FIELD_CONFLICT)
        # And with the FCS reserved the geometry agrees -- the ONLY remaining
        # complaint is that this codec cannot build the layout at all.
        ok = _minimal()
        ok.frame.fcs_append = T.FcsAppender.MAC
        ok.frame.mac_psdu_includes_fcs = True
        ok.frame.geometry = T.frame_geometry_from_authority(
            T.frame_geometry_mac_appends_fcs(T.FrameProfileId.TWR_V1))
        self.assertEqual(ok.frame.geometry.mac_fcs_bytes, 2)
        rep2 = T.validate(ok)
        self.assertIsNone(rep2.find("frame.geometry.mac_fcs_bytes",
                                    T.ConfigReason.FIELD_CONFLICT))
        self.assert_has(rep2, "frame.fcs_append", T.ConfigReason.UNSUPPORTED)
        # The on-air length does not change: 31 B either way.
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.FINAL,
                                              T.FcsAppender.MAC), 31)
        self.assertEqual(T.frame_bytes_on_air(T.FrameType.FINAL,
                                              T.FcsAppender.PHY), 31)

    def test_sfd_symbol_count_must_match_the_mode(self):
        c = _minimal()
        c.frame.sfd_symbols = 4   # 4z2 is 8 symbols
        self.assert_has(T.validate(c), "frame.sfd_symbols",
                        T.ConfigReason.FIELD_CONFLICT)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.R4Z1), 4)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.R4Z2), 8)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.R4Z3), 16)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.R4Z4), 32)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.DWT8), 8)
        self.assertEqual(T.sfd_mode_symbols(T.SfdMode.IEEE8), 8)

    def test_at_most_one_in_flight_exchange(self):
        for n in (0, 2, 4, 16):
            c = _minimal()
            c.session.max_in_flight_exchanges = n
            v = self.assert_has(T.validate(c), "session.max_in_flight_exchanges",
                                T.ConfigReason.IN_FLIGHT_NOT_SUPPORTED)
            self.assertIn("phase 1", v.message)
        # Exactly one is the only legal value.
        c = _minimal()
        self.assertEqual(c.session.max_in_flight_exchanges, 1)
        self.assertTrue(T.validate(c).ok())
        # Consecutive measurements may not overlap: with one exchange in
        # flight, the interval must cover a whole exchange timeout.
        d = _minimal()
        d.session.measurement_count = 5
        d.session.measurement_interval = T.Duration(1_000_000)  # 1 ms
        self.assert_has(T.validate(d), "session.measurement_interval.ns",
                        T.ConfigReason.TIMING_ORDER_VIOLATION)
        e = _minimal()
        e.session.measurement_count = 5
        e.session.measurement_interval = T.Duration(50_000_000)
        self.assertTrue(T.validate(e).ok(), T.validate(e).to_string())

    def test_exchange_gate_refuses_a_second_concurrent_exchange(self):
        gate = T.ExchangeGate()
        self.assertTrue(gate.try_begin().ok())
        self.assertEqual(gate.in_flight(), 1)
        second = gate.try_begin()
        self.assert_machine_readable(second)
        v = second.violations[0]
        self.assertIs(v.reason, T.ConfigReason.EXCHANGE_ALREADY_IN_FLIGHT)
        self.assertIs(v.status, T.ExchangeStatus.QUEUE_FULL)
        # A refused request must not consume a slot, or a run of refusals
        # would deadlock the endpoint.
        self.assertEqual(gate.in_flight(), 1)
        gate.end()
        self.assertTrue(gate.try_begin().ok())
        self.assertEqual(gate.in_flight(), 1)

    def test_addresses_pan_session_and_sequence_are_checked(self):
        c = _minimal()
        c.session.local_address = c.session.peer_address
        self.assert_has(T.validate(c), "session.peer_address",
                        T.ConfigReason.FIELD_CONFLICT)
        d = _minimal()
        d.session.local_address = 0x0000
        self.assert_has(T.validate(d), "session.local_address",
                        T.ConfigReason.FIELD_CONFLICT)
        e = _minimal()
        e.session.peer_address = 0xFFFF
        self.assert_has(T.validate(e), "session.peer_address",
                        T.ConfigReason.FIELD_CONFLICT)
        f = _minimal()
        f.session.pan_id = 0x0000
        self.assert_has(T.validate(f), "session.pan_id",
                        T.ConfigReason.EMPTY_VALUE)
        g = _minimal()
        g.session.session_id = 0
        self.assert_has(T.validate(g), "session.session_id",
                        T.ConfigReason.ZERO_VALUE)
        # The sequence wrap modulus is independent of the session id.
        h = _minimal()
        h.session.sequence_modulus = 5
        self.assert_has(T.validate(h), "session.sequence_modulus",
                        T.ConfigReason.OUT_OF_RANGE)
        i = _minimal()
        i.session.sequence_modulus = 4
        i.session.sequence = 4
        self.assert_has(T.validate(i), "session.sequence",
                        T.ConfigReason.INDEX_OUT_OF_RANGE)
        j = _minimal()
        j.session.sequence_modulus = 4
        j.session.sequence = 3
        j.session.session_id = 0xBEEF   # unchanged, still legal
        self.assertTrue(T.validate(j).ok(), T.validate(j).to_string())

    def test_session_id_must_fit_the_16_bit_wire_field(self):
        """REQ-PROTO-01: identity-or-refuse, never fold, mask or hash.

        The wire session field of frame v1 is 2 bytes, so the wire session
        space is 2**16 and that is the WHOLE space.  A lossy local->wire map
        would make two different local sessions produce byte-identical session
        fields, and the frame match would then accept the wrong session's reply
        as this one's -- a wrong-peer frame silently folded into a range.  A
        refusal is visible and attributable; a collision is neither.

        This is the defect the M0 fixture carried: it stated 0x11223344 in a
        16-bit field and the config validated, because nothing ever asked
        whether the value could reach the air.
        """
        for good in (1, 2, 0x7FFF, 0x8000, T.SESSION_ID_WIRE_MAX):
            c = _minimal()
            c.session.session_id = good
            self.assertTrue(T.validate(c).ok(),
                            "session_id %d should be legal: %s"
                            % (good, T.validate(c).to_string()))
        for bad in (0x10000, 0x10001, 0xDEADBEEF, 0xFFFFFFFF):
            c = _minimal()
            c.session.session_id = bad
            v = self.assert_has(T.validate(c), "session.session_id",
                                T.ConfigReason.OUT_OF_RANGE)
            self.assertIs(v.status, T.ExchangeStatus.CONFIG_REJECTED)
            self.assertEqual(v.requirement, "REQ-PROTO-01")
            self.assertIn("refused, not truncated", v.message)
        # The mapping itself: in range -> exact; out of range -> refused, and
        # never a plausible-looking value left behind.
        self.assertEqual(T.session_id_to_wire(0x1234)[:1], (0x1234,))
        wire, text, err = T.session_id_to_wire(0x10000)
        self.assertEqual(wire, 0)
        self.assertIs(err, T.SessionIdError.OUT_OF_WIRE_RANGE)
        self.assertIn("not truncated to 0", text)
        self.assertEqual(T.SESSION_ID_WIRE_BITS, 16)
        self.assertEqual(T.SESSION_ID_WIRE_MAX, 0xFFFF)
        self.assertEqual(T.SESSION_ID_RESERVED_LOCAL, 0)
        self.assertTrue(T.session_id_fits_wire(0xFFFF))
        self.assertFalse(T.session_id_fits_wire(0x10000))
        self.assertFalse(T.session_id_fits_wire(-1))
        # Injective over everything that can reach the air: the only collision
        # is the same integer, because an out-of-range id never goes on the air
        # at all.
        self.assertTrue(T.wire_session_id_collides(5, 5))
        self.assertFalse(T.wire_session_id_collides(5, 6))
        self.assertFalse(T.wire_session_id_collides(0x10000, 0x0000))
        self.assertEqual(T.SESSION_ID_BIRTHDAY_SESSIONS_50PCT, 256.0)

    def test_retry_policy_is_consistent(self):
        c = _minimal()
        c.session.max_attempts_per_exchange = 3
        self.assert_has(T.validate(c), "session.retry_backoff",
                        T.ConfigReason.FIELD_CONFLICT)
        d = _minimal()
        d.session.max_attempts_per_exchange = 3
        d.session.retry_backoff = T.Duration(1_000_000)
        self.assertTrue(T.validate(d).ok(), T.validate(d).to_string())
        e = _minimal()
        e.session.max_attempts_per_exchange = 0
        self.assert_has(T.validate(e), "session.max_attempts_per_exchange",
                        T.ConfigReason.OUT_OF_RANGE)
        f = _minimal()
        f.session.retry_backoff = T.Duration(1_000)
        self.assert_has(T.validate(f), "session.retry_backoff",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_readback_mismatch_is_a_startup_failure(self):
        c = _minimal()
        c.radio.readback.sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        self.assert_has(T.validate(c), "radio.readback.sample_rate_hz",
                        T.ConfigReason.FIELD_CONFLICT)
        d = _minimal()
        d.radio.readback.tx_channel = 3
        self.assert_has(T.validate(d), "radio.readback.tx_channel",
                        T.ConfigReason.FIELD_CONFLICT)
        e = _minimal()
        e.radio.readback.clock_source = "external"
        self.assert_has(T.validate(e), "radio.readback.clock_source",
                        T.ConfigReason.FIELD_CONFLICT)
        f = _minimal()
        f.radio.readback.sample_rate_hz = 1.0e6
        self.assert_has(T.validate(f), "radio.readback.sample_rate_hz",
                        T.ConfigReason.OUT_OF_RANGE)
        # require_readback with no readback recorded is calibration_missing.
        g = _minimal()
        g.radio.readback.present = False
        v = self.assert_has(T.validate(g), "radio.readback.present",
                            T.ConfigReason.CALIBRATION_MISSING)
        self.assertIs(v.status, T.ExchangeStatus.CALIBRATION_MISSING)
        # A dry-run may opt out.
        h = _minimal()
        h.radio.readback.present = False
        h.radio.require_readback = False
        self.assertTrue(T.validate(h).ok(), T.validate(h).to_string())

    def test_diagnostics_are_bounded(self):
        c = _minimal()
        c.diagnostics.cir_capture_enabled = True
        self.assert_has(T.validate(c), "diagnostics.cir_capture_max_bytes",
                        T.ConfigReason.ZERO_VALUE)
        d = _minimal()
        d.diagnostics.cir_capture_enabled = True
        d.diagnostics.cir_capture_max_bytes = 1 << 20
        d.diagnostics.cir_capture_stride = 0
        self.assert_has(T.validate(d), "diagnostics.cir_capture_stride",
                        T.ConfigReason.ZERO_VALUE)
        e = _minimal()
        e.diagnostics.cir_capture_max_bytes = T.MAX_DIAGNOSTIC_BYTES + 1
        self.assert_has(T.validate(e), "diagnostics.cir_capture_max_bytes",
                        T.ConfigReason.OVER_CAPACITY)
        f = _minimal()
        f.diagnostics.result_queue_capacity = 0
        self.assert_has(T.validate(f), "diagnostics.result_queue_capacity",
                        T.ConfigReason.ZERO_VALUE)
        g = _minimal()
        g.diagnostics.event_queue_capacity = T.MAX_QUEUE_ENTRIES + 1
        self.assert_has(T.validate(g), "diagnostics.event_queue_capacity",
                        T.ConfigReason.OVER_CAPACITY)
        # Diagnostic I/O must not run on the realtime processing thread
        # (REQ-GR-02).
        h = _minimal()
        h.diagnostics.io_on_realtime_thread = True
        v = self.assert_has(T.validate(h), "diagnostics.io_on_realtime_thread",
                            T.ConfigReason.FIELD_CONFLICT)
        self.assertEqual(v.requirement, "REQ-GR-02")

    def test_three_timing_quantities_stay_distinct(self):
        """reply delay, post-TX RX enable and RX timeout are three numbers."""
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        self.assertNotEqual(e.response_to_final_ticks, e.post_tx_rx_enable_ticks)
        self.assertNotEqual(e.post_tx_rx_enable_ticks, 0)
        # Giving the post-TX RX enable the reply delay's reference is wrong.
        d = _minimal(T.Protocol.DS, T.Role.RESPONDER)
        d.timing.post_tx_rx_enable.reference = \
            T.TimeReferenceEvent.RESPONSE_RX_RMARKER
        self.assert_has(T.validate(d), "timing.post_tx_rx_enable.reference",
                        T.ConfigReason.FIELD_CONFLICT)
        # The RX timeout is measured from RX enable, not from an RMARKER.
        f = _minimal()
        f.timeouts.rx_timeout.reference = \
            T.TimeReferenceEvent.POLL_TX_RMARKER
        self.assert_has(T.validate(f), "timeouts.rx_timeout.reference",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_roles_own_different_delays(self):
        # The initiator never sends a Response.
        c = _minimal(T.Protocol.SS, T.Role.INITIATOR)
        c.timing.poll_to_response.value = T.Duration(600_000)
        self.assert_has(T.validate(c), "timing.poll_to_response",
                        T.ConfigReason.FIELD_CONFLICT)
        # SS has no Final at all: neither the delay nor the RX window may exist.
        d = _minimal(T.Protocol.SS, T.Role.INITIATOR)
        d.timing.response_to_final.value = T.Duration(600_000)
        d.timeouts.final_rx_window.value = T.Duration(20_000)
        report = T.validate(d)
        self.assert_has(report, "timing.response_to_final",
                        T.ConfigReason.FIELD_CONFLICT)
        self.assert_has(report, "timeouts.final_rx_window.ns",
                        T.ConfigReason.FIELD_CONFLICT)
        # The same Final RX window is legal for a DS responder, which waits for
        # it.
        ds = _minimal(T.Protocol.DS, T.Role.RESPONDER)
        self.assertEqual(ds.timeouts.final_rx_window.value.nanos(), 20_000)
        self.assertTrue(T.validate(ds).ok(), T.validate(ds).to_string())
        # The responder never sends a Final.
        e = _minimal(T.Protocol.DS, T.Role.RESPONDER)
        e.timing.response_to_final.value = T.Duration(600_000)
        self.assert_has(T.validate(e), "timing.response_to_final",
                        T.ConfigReason.FIELD_CONFLICT)
        # The responder transmits no Poll, so its Poll start time is zero.
        f = _minimal(T.Protocol.SS, T.Role.RESPONDER)
        f.timing.poll_start.value = T.Duration(40_000_000_000)
        self.assert_has(T.validate(f), "timing.poll_start.ns",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_reply_delay_must_cover_the_measured_tx_lead_time(self):
        # A scheduled TX whose deadline cannot be met must FAIL, not be sent
        # late (REQ-GR-04).
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        c.timing.response_to_final.value = T.Duration(10_000)
        v = self.assert_has(T.validate(c), "timing.response_to_final.ns",
                            T.ConfigReason.TIMING_BUDGET_INFEASIBLE)
        self.assertIs(v.status, T.ExchangeStatus.DEADLINE_MISSED)
        self.assertEqual(v.requirement, "REQ-GR-04")
        # There is NO default lead time: an unstated one is rejected.
        d = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        d.timing.min_tx_lead_time = T.TimedField()
        self.assert_has(T.validate(d), "timing.min_tx_lead_time.domain",
                        T.ConfigReason.EMPTY_VALUE)

    def test_rx_window_must_fit_inside_the_rx_timeout(self):
        c = _minimal()
        c.timeouts.response_rx_window.value = T.Duration(50_000)
        c.timeouts.rx_timeout.value = T.Duration(40_000)
        self.assert_has(T.validate(c), "timeouts.response_rx_window.ns",
                        T.ConfigReason.TIMING_ORDER_VIOLATION)
        d = _minimal()
        d.timeouts.exchange_timeout = T.host_field(1_000)   # 1 us
        self.assert_has(T.validate(d), "timeouts.exchange_timeout.ns",
                        T.ConfigReason.TIMING_ORDER_VIOLATION)


# ==========================================================================
# G. Insufficient timing precision
# ==========================================================================


class TestTimingPrecision(TwrConfigTestBase):

    def test_quantisation_rate_must_be_a_rate_this_build_expresses(self):
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        c.timing.response_to_final.required_quantisation_hz = 4.0e9  # 0.25 ns
        self.assert_has(T.validate(c), "timing.response_to_final.quantisation_hz",
                        T.ConfigReason.QUANTISATION_UNSUPPORTED)

    def test_zero_nanosecond_budget_is_insufficient(self):
        # An ALLOWED rate, but the caller demands an exact nanosecond.
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        c.timing.response_to_final.max_quantisation_error_ns = 0
        self.assert_has(T.validate(c), "timing.response_to_final.ns",
                        T.ConfigReason.TIMING_PRECISION_INSUFFICIENT)
        # The 1 ns budget is exactly what the 1 GHz, work and 737.28 MS/s
        # rates offer; nothing finer is claimed anywhere.
        for rate in (1.0e9, T.WORK_SAMPLE_RATE_HZ, T.NATIVE_RATE_UC200_HZ):
            self.assertTrue(T.Duration(600_000).representable_at(rate, 1),
                            "%.17g" % rate)
            self.assertFalse(T.Duration(600_000).representable_at(rate, 0),
                             "%.17g" % rate)

    def test_cg400_two_point_zero_three_nanosecond_tick_needs_two_or_three_ns(self):
        """The MEASURED fact: at 491.52 MS/s one device tick is 2.0345 ns.

        A 1 ns error budget is therefore UNACHIEVABLE there and must be
        rejected as insufficient timing precision, not silently rounded.  Two
        or 3 ns is required.
        """
        tick_ns = 1.0e9 / T.NATIVE_RATE_CG400_HZ
        self.assertAlmostEqual(tick_ns, 2.0345, places=4)
        self.assertAlmostEqual(T.NATIVE_TICK_NS[T.NATIVE_RATE_CG400_HZ], 2.0345,
                               places=4)
        self.assertFalse(T.Duration(600_000).representable_at(
            T.NATIVE_RATE_CG400_HZ, 1))
        self.assertTrue(T.Duration(600_000).representable_at(
            T.NATIVE_RATE_CG400_HZ, 2))
        self.assertTrue(T.Duration(600_000).representable_at(
            T.NATIVE_RATE_CG400_HZ, 3))

        c = _minimal()
        c.timeouts.rx_timeout.required_quantisation_hz = T.NATIVE_RATE_CG400_HZ
        v = self.assert_has(T.validate(c), "timeouts.rx_timeout.ns",
                            T.ConfigReason.TIMING_PRECISION_INSUFFICIENT)
        self.assertIn("2.0345", v.message)
        # Asking for the tick's true resolution instead is accepted.
        c.timeouts.rx_timeout.max_quantisation_error_ns = 3
        report = T.validate(c)
        self.assertTrue(report.ok(), report.to_string())
        c.timeouts.rx_timeout.max_quantisation_error_ns = 2
        self.assertTrue(T.validate(c).ok(), T.validate(c).to_string())
        # 1 ns is still refused at that rate.
        c.timeouts.rx_timeout.max_quantisation_error_ns = 1
        self.assert_has(T.validate(c), "timeouts.rx_timeout.ns",
                        T.ConfigReason.TIMING_PRECISION_INSUFFICIENT)

    def test_uc200_one_point_three_five_nanosecond_tick_accepts_one_ns(self):
        tick_ns = 1.0e9 / T.NATIVE_RATE_UC200_HZ
        self.assertAlmostEqual(tick_ns, 1.3563, places=4)
        self.assertTrue(T.Duration(600_000).representable_at(
            T.NATIVE_RATE_UC200_HZ, 1))
        self.assertFalse(T.Duration(600_000).representable_at(
            T.NATIVE_RATE_UC200_HZ, 0))

    def test_negative_quantisation_budget_is_refused(self):
        c = _minimal()
        c.timeouts.rx_timeout.max_quantisation_error_ns = -1
        self.assert_has(T.validate(c), "timeouts.rx_timeout.max_quantisation_error_ns",
                        T.ConfigReason.OUT_OF_RANGE)

    def test_negative_delays_are_never_delays(self):
        c = _minimal()
        c.timing.post_tx_rx_enable.value = T.Duration(-1)
        report = T.validate(c)
        self.assert_has(report, "timing.post_tx_rx_enable.ns",
                        T.ConfigReason.NEGATIVE_VALUE)
        self.assert_has(report, "timing.post_tx_rx_enable",
                        T.ConfigReason.NEGATIVE_VALUE)


# ==========================================================================
# H. JSON import / export (REQ-OUT-01)
# ==========================================================================


class TestJson(TwrConfigTestBase):

    def test_round_trip_is_stable_for_every_protocol_and_role(self):
        for protocol in (T.Protocol.SS, T.Protocol.DS):
            for role in (T.Role.INITIATOR, T.Role.RESPONDER):
                c = _minimal(protocol, role)
                text = T.to_json_string(c)
                self.assertTrue(text)
                back, report = T.from_json_string(text)
                self.assertTrue(report.ok(), report.to_string())
                text2 = T.to_json_string(back)
                self.assertEqual(text, text2, "the second write differs")
                self.assertEqual(T.config_hash(c), T.config_hash(back))
                # The re-imported config still validates against the measured
                # whitelist.
                again = T.validate(back)
                self.assertTrue(again.ok(), again.to_string())

    def test_round_trip_preserves_absent_optional_fields(self):
        c = _minimal()
        c.tx.gain_db = None
        c.tx.iq_amplitude = None
        c.tx.calibrated_tx_power_dbm = None
        c.rx.vendor_pac_value = None
        c.tx.vendor_power_word = None
        c.calibration.record.gain_db = None
        doc = json.loads(T.to_json_string(c))
        self.assertIsNone(doc["tx"]["gain_db"])
        self.assertIsNone(doc["tx"]["iq_amplitude"])
        self.assertIsNone(doc["tx"]["calibrated_tx_power_dbm"])
        self.assertIsNone(doc["tx"]["vendor_power_word"])
        self.assertIsNone(doc["rx"]["vendor_pac_value"])
        self.assertIsNone(doc["calibration"]["record"]["gain_db"])
        back, report = T.from_json_string(T.to_json_string(c))
        self.assertTrue(report.ok(), report.to_string())
        self.assertIsNone(back.tx.gain_db)
        self.assertIsNone(back.rx.vendor_pac_value)
        self.assertEqual(T.config_hash(c), T.config_hash(back))

    def test_unit_and_key_suffix_conventions(self):
        doc = json.loads(T.to_json_string(_minimal()))
        # Durations are plain integer NANOSECONDS under a _ns key.
        self.assertEqual(doc["session"]["measurement_interval_ns"], 20_000_000)
        self.assertEqual(doc["session"]["retry_backoff_ns"], 0)
        self.assertIsInstance(doc["session"]["measurement_interval_ns"], int)
        self.assertIsInstance(doc["timing"]["post_tx_rx_enable"]["ns"], int)
        # Native ticks carry a _native_ticks key and never mix with ns.
        self.assertIn("tx_link_delay_native_ticks", doc["calibration"])
        self.assertIn("rx_link_delay_native_ticks", doc["calibration"])
        self.assertEqual(doc["calibration"]["tx_link_delay_ns"], 120)
        self.assertEqual(doc["calibration"]["tx_link_delay_native_ticks"], 0)
        # ... and the unit is stated explicitly.
        self.assertEqual(doc["calibration"]["link_delay_unit"], "ns")
        # A frame-length key is a plain byte count, not a duration.
        self.assertEqual(doc["frame"]["mac_psdu_bytes"], 32)
        self.assertEqual(doc["frame"]["fcs_bytes"], 2)

    def test_int64_precision_survives_the_round_trip(self):
        c = _minimal()
        # Beyond 2**53, so a naive double round trip would corrupt these.
        c.timeouts.exchange_timeout.value = T.Duration(4_000_000_000_000_000_001)
        c.calibration.tx_link_delay = T.Duration(9_007_199_254_740_993)
        c.calibration.record.valid_until_monotonic_ns = \
            9_223_372_036_854_775_000
        c.session.session_id = 0xFFFFFFFF

        text = T.to_json_string(c)
        # Large integers are written as decimal STRINGS, so no JSON reader can
        # silently round them through a double.
        self.assertIn('"4000000000000000001"', text)
        self.assertIn('"9007199254740993"', text)
        self.assertIn('"9223372036854775000"', text)
        # Small integers stay unquoted numbers.
        self.assertIn('"session_id": 4294967295', text)

        back, report = T.from_json_string(text)
        self.assertTrue(report.ok(), report.to_string())
        self.assertEqual(back.timeouts.exchange_timeout.value.nanos(),
                         4_000_000_000_000_000_001)
        self.assertEqual(back.calibration.tx_link_delay.nanos(),
                         9_007_199_254_740_993)
        self.assertEqual(back.calibration.record.valid_until_monotonic_ns,
                         9_223_372_036_854_775_000)
        self.assertEqual(back.session.session_id, 0xFFFFFFFF)
        self.assertEqual(T.config_hash(c), T.config_hash(back))

    def test_parse_int64_strict_rejects_everything_that_is_not_a_decimal(self):
        self.assertEqual(T.parse_int64_strict("4000000000000000001"),
                         4_000_000_000_000_000_001)
        self.assertEqual(T.parse_int64_strict("-42"), -42)
        for bad in ("42.0", "4e9", "", "0x10", " 42", "42 ", "+42", "1_000",
                    "99999999999999999999999", "nan", "NaN"):
            self.assertIsNone(T.parse_int64_strict(bad), repr(bad))
        # A huge int64 literal in a document is a type error on an int field,
        # not a silently wrapped value.
        back, report = T.from_json_string('{"session": {"session_id": '
                                          '99999999999999999999999}}')
        self.assertTrue(report.has(T.ConfigReason.TYPE_MISMATCH))

    def test_doubles_round_trip_exactly(self):
        c = _minimal()
        c.phy.center_frequency_hz = 6489.6e6
        c.rx.detection_threshold = 0.1 + 0.2      # 0.30000000000000004
        text = T.to_json_string(c)
        back, report = T.from_json_string(text)
        self.assertTrue(report.ok(), report.to_string())
        self.assertEqual(back.rx.detection_threshold, 0.1 + 0.2)
        self.assertEqual(back.phy.center_frequency_hz, 6489.6e6)
        self.assertEqual(T.config_hash(c), T.config_hash(back))

    def test_unknown_key_is_refused(self):
        doc = json.loads(T.to_json_string(_minimal()))
        doc["phy"]["turbo_mode"] = 1
        _back, report = T.from_json_string(json.dumps(doc))
        self.assert_machine_readable(report)
        self.assertIsNotNone(report.find("phy.turbo_mode",
                                        T.ConfigReason.UNKNOWN_KEY))
        # Also in a nested group and in a peer.
        doc2 = json.loads(T.to_json_string(_minimal()))
        doc2["calibration"]["record"]["oops"] = 1
        _b2, r2 = T.from_json_string(json.dumps(doc2))
        self.assertIsNotNone(r2.find("calibration.record.oops",
                                    T.ConfigReason.UNKNOWN_KEY))
        doc3 = json.loads(T.to_json_string(_minimal()))
        doc3["radio"]["peers"][0]["oops"] = 1
        _b3, r3 = T.from_json_string(json.dumps(doc3))
        self.assertIsNotNone(r3.find("radio.peers[0].oops",
                                    T.ConfigReason.UNKNOWN_KEY))

    def test_missing_key_is_refused(self):
        doc = json.loads(T.to_json_string(_minimal()))
        del doc["phy"]["channel"]
        _back, report = T.from_json_string(json.dumps(doc))
        self.assert_machine_readable(report)
        self.assertIsNotNone(report.find("phy.channel", T.ConfigReason.MISSING_KEY))
        # A missing group, a missing nested group and a missing array.
        doc2 = json.loads(T.to_json_string(_minimal()))
        del doc2["frame"]["geometry"]
        _b2, r2 = T.from_json_string(json.dumps(doc2))
        self.assertIsNotNone(r2.find("frame.geometry", T.ConfigReason.MISSING_KEY))
        doc3 = json.loads(T.to_json_string(_minimal()))
        del doc3["calibration"]["record"]
        _b3, r3 = T.from_json_string(json.dumps(doc3))
        self.assertIsNotNone(r3.find("calibration.record",
                                    T.ConfigReason.MISSING_KEY))
        doc4 = json.loads(T.to_json_string(_minimal()))
        del doc4["radio"]["peers"]
        _b4, r4 = T.from_json_string(json.dumps(doc4))
        self.assertIsNotNone(r4.find("radio.peers", T.ConfigReason.MISSING_KEY))
        # A missing key inside a TimedField.
        doc5 = json.loads(T.to_json_string(_minimal()))
        del doc5["timing"]["poll_start"]["ns"]
        _b5, r5 = T.from_json_string(json.dumps(doc5))
        self.assertIsNotNone(r5.find("timing.poll_start.ns",
                                    T.ConfigReason.MISSING_KEY))
        # An entirely absent document.
        _b6, r6 = T.from_json_string("{}")
        self.assertTrue(r6.has(T.ConfigReason.MISSING_KEY))
        self.assertTrue(r6.has_field("session"))
        self.assertTrue(r6.has_field("phy"))

    def test_type_mismatch_is_refused_not_coerced(self):
        doc = json.loads(T.to_json_string(_minimal()))
        doc["rx"]["detection_threshold"] = "0.3"
        _b, report = T.from_json_string(json.dumps(doc))
        self.assert_machine_readable(report)
        self.assertIsNotNone(report.find("rx.detection_threshold",
                                        T.ConfigReason.TYPE_MISMATCH))
        # A float where an integer belongs is NEVER truncated silently.
        doc2 = json.loads(T.to_json_string(_minimal()))
        doc2["session"]["session_id"] = 2863311530.7
        _b2, r2 = T.from_json_string(json.dumps(doc2))
        self.assertIsNotNone(r2.find("session.session_id",
                                    T.ConfigReason.TYPE_MISMATCH))
        # An exponent form counts as a float too.
        doc3 = json.loads(T.to_json_string(_minimal()))
        doc3["session"]["session_id"] = 1e5
        _b3, r3 = T.from_json_string(json.dumps(doc3))
        self.assertIsNotNone(r3.find("session.session_id",
                                    T.ConfigReason.TYPE_MISMATCH))
        # A bool where a bool belongs is fine; a number where a bool belongs
        # is not.
        doc4 = json.loads(T.to_json_string(_minimal()))
        doc4["frame"]["ranging_bit"] = 1
        _b4, r4 = T.from_json_string(json.dumps(doc4))
        self.assertIsNotNone(r4.find("frame.ranging_bit",
                                    T.ConfigReason.TYPE_MISMATCH))
        # An int64 field given a non-decimal string.
        doc5 = json.loads(T.to_json_string(_minimal()))
        doc5["session"]["session_id"] = "0x10"
        _b5, r5 = T.from_json_string(json.dumps(doc5))
        self.assertIsNotNone(r5.find("session.session_id",
                                    T.ConfigReason.INTEGER_PRECISION_LOSS))
        # A group given a non-object.
        doc6 = json.loads(T.to_json_string(_minimal()))
        doc6["frame"]["geometry"] = [1, 2]
        _b6, r6 = T.from_json_string(json.dumps(doc6))
        self.assertIsNotNone(r6.find("frame.geometry",
                                    T.ConfigReason.TYPE_MISMATCH))
        # An array given where a timed object belongs.
        doc7 = json.loads(T.to_json_string(_minimal()))
        doc7["timeouts"]["rx_timeout"] = 5
        _b7, r7 = T.from_json_string(json.dumps(doc7))
        self.assertIsNotNone(r7.find("timeouts.rx_timeout",
                                    T.ConfigReason.TYPE_MISMATCH))

    def test_unknown_enum_value_is_refused(self):
        for path, key, value in (("session", "protocol", "sideways"),
                                 ("session", "role", "boss"),
                                 ("phy", "prf_class", "bprf128"),
                                 ("phy", "data_rate", "11m"),
                                 ("frame", "sfd_mode", "4z5"),
                                 ("frame", "phr_mode", "super"),
                                 ("frame", "sts_mode", "sp2048"),
                                 ("frame", "fcs_append", "both"),
                                 ("tx", "power_policy", "auto"),
                                 ("rx", "agc", "auto"),
                                 ("calibration", "link_delay_unit", "ticks"),
                                 ("calibration", "first_path_algorithm", "magic"),
                                 ("calibration", "cfo_compensation", "maybe")):
            doc = json.loads(T.to_json_string(_minimal()))
            doc[path][key] = value
            _back, report = T.from_json_string(json.dumps(doc))
            self.assert_machine_readable(report)
            self.assertIsNotNone(
                report.find("%s.%s" % (path, key),
                            T.ConfigReason.UNKNOWN_ENUM_VALUE),
                "%s.%s = %r was accepted" % (path, key, value))
        # ... and inside a TimedField.
        doc = json.loads(T.to_json_string(_minimal()))
        doc["timing"]["poll_start"]["domain"] = "solar"
        _b, r = T.from_json_string(json.dumps(doc))
        self.assertIsNotNone(r.find("timing.poll_start.domain",
                                    T.ConfigReason.UNKNOWN_ENUM_VALUE))
        doc2 = json.loads(T.to_json_string(_minimal()))
        doc2["timing"]["poll_start"]["marker"] = "vibes"
        _b2, r2 = T.from_json_string(json.dumps(doc2))
        self.assertIsNotNone(r2.find("timing.poll_start.marker",
                                    T.ConfigReason.UNKNOWN_ENUM_VALUE))
        doc3 = json.loads(T.to_json_string(_minimal()))
        doc3["timeouts"]["rx_timeout"]["reference"] = "whenever"
        _b3, r3 = T.from_json_string(json.dumps(doc3))
        self.assertIsNotNone(r3.find("timeouts.rx_timeout.reference",
                                    T.ConfigReason.UNKNOWN_ENUM_VALUE))

    def test_malformed_documents_are_refused(self):
        for text in ("", "{", "[]", "null", "true", '{"a":}', '{"a": 1,}',
                     "{'a': 1}", '{"a": 01}', '{"a": 1.}', '{"a": 1e}',
                     '{"a": "unterminated', '{"a": 1} trailing'):
            _back, report = T.from_json_string(text)
            self.assert_machine_readable(report)
            self.assertTrue(
                report.has(T.ConfigReason.MALFORMED_JSON) or
                report.has(T.ConfigReason.TYPE_MISMATCH),
                "accepted a malformed document: %r" % text)
        # Deep nesting is refused rather than blowing the stack.
        deep = "[" * 200 + "]" * 200
        _back, report = T.from_json_string(deep)
        self.assertFalse(report.ok())

    def test_nan_and_infinity_literals_are_refused_on_import(self):
        for text in ('{"phy": {"center_frequency_hz": NaN}}',
                     '{"phy": {"center_frequency_hz": Infinity}}',
                     '{"phy": {"center_frequency_hz": -Infinity}}'):
            _back, report = T.from_json_string(text)
            self.assert_machine_readable(report)
            self.assertTrue(report.has(T.ConfigReason.MALFORMED_JSON), text)

    def test_effective_config_export_carries_requested_effective_and_reasons(self):
        c = _minimal(T.Protocol.DS, T.Role.INITIATOR)
        c.timing.response_to_final.value = T.Duration(600_002)
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        doc = T.effective_to_json_dict(e)
        self.assertTrue(doc["ok"])
        self.assertEqual(doc["config_hash"], e.config_hash)
        self.assertEqual(doc["profile_version"], "qa-synthetic-frame/0")
        self.assertEqual(doc["calibration_version"], "qa-synthetic-cal/0")
        self.assertTrue(doc["changes"])
        self.assertEqual(doc["quantised"]["response_to_final_ticks"], 442369)
        self.assertEqual(doc["frame_budget"]["final_bytes"], 31)
        self.assertEqual(doc["frame_budget"]["poll_bytes"], 16)
        self.assertEqual(doc["frame_budget"]["response_bytes"], 26)
        self.assertEqual(doc["frame_budget"]["phr_bytes"], 2)
        self.assertEqual(doc["frame_budget"]["phr_coded_bits"], 19)
        self.assertEqual(doc["frame_budget"]["max_psdu_bytes"], 127)
        self.assertEqual(doc["violations"], [])
        # It is a real JSON document too.
        text = json.dumps(doc, allow_nan=False)
        self.assertIn('"fnv1a64:', text)

        # A rejected snapshot lists its reasons.
        bad = _minimal()
        bad.frame.sts_mode = T.StsMode.SP256
        re_ = T.effective_config(bad)
        self.assertFalse(re_.ok)
        rdoc = T.effective_to_json_dict(re_)
        self.assertFalse(rdoc["ok"])
        reasons = {v["reason"] for v in rdoc["violations"]}
        self.assertIn("out_of_scope", reasons)
        self.assertTrue(any(v["status"] == "unsupported" for v in rdoc["violations"]))
        self.assertTrue(any(v["requirement"] == "REQ-SCOPE-04"
                            for v in rdoc["violations"]))
        # A rejected snapshot carries NO effective values.
        self.assertEqual(rdoc["calibration"]["calibration_id"], "")
        self.assertEqual(rdoc["timing"]["response_to_final"]["ns"], 0)


# ==========================================================================
# I. Calibration (REQ-CAL-01)
# ==========================================================================


class TestCalibration(TwrConfigTestBase):

    def test_calibration_is_applicable_exactly_once(self):
        cal = _minimal().calibration
        status, why = T.apply_calibration_once(cal, 0)
        self.assertIs(status, T.ExchangeStatus.OK)
        self.assertEqual(cal.applied_count, 1)
        status, why = T.apply_calibration_once(cal, 0)
        self.assertIs(status, T.ExchangeStatus.CALIBRATION_MISSING)
        self.assertIn("exactly once", why)
        # A config that already carries an applied calibration is not valid
        # input.
        c = _minimal()
        c.calibration = cal
        v = self.assert_has(T.validate(c), "calibration.applied_count",
                            T.ConfigReason.CALIBRATION_ALREADY_APPLIED)
        self.assertIs(v.status, T.ExchangeStatus.CALIBRATION_MISSING)
        # Applying it does not mutate the caller's config either.
        d = _minimal()
        d.calibration.applied_count = 1
        before = T.config_hash(d)
        T.apply_calibration_once(d.calibration, 0)
        self.assertEqual(T.config_hash(d), before)

    def test_missing_and_expired_calibration(self):
        none = _minimal().calibration
        none.calibration_id = ""
        status, why = T.apply_calibration_once(none, 0)
        self.assertIs(status, T.ExchangeStatus.CALIBRATION_MISSING)
        self.assertIn("uncalibrated", why)

        stale = _minimal().calibration
        stale.record.valid_until_monotonic_ns = 1000
        status, why = T.apply_calibration_once(stale, 2000)
        self.assertIs(status, T.ExchangeStatus.CALIBRATION_EXPIRED)
        self.assertIn("expired", why)
        # Still valid before the deadline.
        fresh = _minimal().calibration
        fresh.record.valid_until_monotonic_ns = 1000
        status, _why = T.apply_calibration_once(fresh, 500)
        self.assertIs(status, T.ExchangeStatus.OK)

        # An uncalibrated absolute range may not be claimed.
        a = _minimal()
        a.calibration.calibration_id = ""
        a.calibration.record.calibration_id = ""
        v = self.assert_has(T.validate(a), "calibration.calibration_id",
                            T.ConfigReason.CALIBRATION_MISSING)
        self.assertIs(v.status, T.ExchangeStatus.CALIBRATION_MISSING)
        self.assertEqual(v.requirement, "REQ-CAL-01")

    def test_mismatched_calibration_is_refused(self):
        # The record must describe THIS channel / rate / profile / gain.
        b = _minimal()
        b.calibration.record.channel = 9
        self.assert_has(T.validate(b), "calibration.record.channel",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        d = _minimal()
        d.calibration.record.native_sample_rate_hz = T.NATIVE_RATE_CG400_HZ
        self.assert_has(T.validate(d), "calibration.record.native_sample_rate_hz",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        e = _minimal()
        e.calibration.record.profile_version = "other-profile/9"
        self.assert_has(T.validate(e), "calibration.record.profile_version",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        f = _minimal()
        f.calibration.record.gain_db = 10.0
        self.assert_has(T.validate(f), "calibration.record.gain_db",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        g = _minimal()
        g.meta.calibration_version = "different-cal/0"
        self.assert_has(T.validate(g), "meta.calibration_version",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        h = _minimal()
        h.calibration.record.calibration_id = "other-cal/0"
        self.assert_has(T.validate(h), "calibration.record.calibration_id",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_nanoseconds_seconds_and_native_ticks_stay_separate(self):
        c = _minimal()
        c.calibration.link_delay_unit = T.TimeUnit.SECONDS
        self.assert_has(T.validate(c), "calibration.link_delay_unit",
                        T.ConfigReason.UNIT_MISMATCH)
        d = _minimal()
        d.calibration.link_delay_unit = T.TimeUnit.NATIVE_TICKS
        self.assert_has(T.validate(d), "calibration.link_delay_unit",
                        T.ConfigReason.FIELD_CONFLICT)
        e = _minimal()
        e.calibration.link_delay_unit = T.TimeUnit.NATIVE_TICKS
        e.calibration.tx_link_delay_native_ticks = 88
        e.calibration.rx_link_delay_native_ticks = 133
        self.assertTrue(T.validate(e).ok(), T.validate(e).to_string())
        # The native-tick fields do not change the nanosecond fields.
        self.assertEqual(e.calibration.tx_link_delay.nanos(), 120)
        # A negative delay in either unit is refused.
        f = _minimal()
        f.calibration.rx_link_delay_native_ticks = -1
        self.assert_has(T.validate(f), "calibration.tx_link_delay_native_ticks",
                        T.ConfigReason.NEGATIVE_VALUE)
        g = _minimal()
        g.calibration.antenna_delay = T.Duration(-1)
        self.assert_has(T.validate(g), "calibration.tx_link_delay",
                        T.ConfigReason.NEGATIVE_VALUE)

    def test_tx_gain_amplitude_and_dbm_are_three_distinct_fields(self):
        c = _minimal()
        c.tx.gain_db = None
        c.tx.power_policy = T.TxPowerPolicy.MANUAL_GAIN_DB
        self.assert_has(T.validate(c), "tx.power_policy",
                        T.ConfigReason.FIELD_CONFLICT)
        d = _minimal()
        d.tx.iq_amplitude = 0.5
        d.tx.power_policy = T.TxPowerPolicy.LEAVE_UNTOUCHED
        self.assert_has(T.validate(d), "tx.power_policy",
                        T.ConfigReason.FIELD_CONFLICT)
        e = _minimal()
        e.tx.gain_db = None
        e.tx.calibrated_tx_power_dbm = 10.0
        e.tx.power_policy = T.TxPowerPolicy.CALIBRATED_DBM
        e.calibration.record.gain_db = None
        self.assert_has(T.validate(e), "tx.gain_db", T.ConfigReason.FIELD_CONFLICT)
        ok = _minimal()
        ok.tx.calibrated_tx_power_dbm = 10.0
        ok.tx.power_policy = T.TxPowerPolicy.CALIBRATED_DBM
        self.assertTrue(T.validate(ok).ok(), T.validate(ok).to_string())
        # A vendor power word is a FOURTH field and needs its backend id.
        f = _minimal()
        f.tx.vendor_power_word = 0x1234
        self.assert_has(T.validate(f), "tx.vendor_power_word",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        g = _minimal()
        g.tx.vendor_power_word = 0x1234
        g.tx.vendor_power_word_backend = "cg600-power-word-v1"
        self.assertTrue(T.validate(g).ok(), T.validate(g).to_string())
        h = _minimal()
        h.tx.vendor_power_word_backend = "cg600-power-word-v1"
        self.assert_has(T.validate(h), "tx.vendor_power_word_backend",
                        T.ConfigReason.FIELD_CONFLICT)

    def test_vendor_pac_needs_a_backend_and_a_real_step(self):
        c = _minimal()
        c.rx.vendor_pac_value = 512
        report = T.validate(c)
        self.assert_has(report, "rx.vendor_pac_value",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        self.assert_has(report, "rx.vendor_pac_applied_step",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        d = _minimal()
        d.rx.vendor_pac_value = 512
        d.rx.vendor_pac_backend = "cg600-pac-v1"
        self.assert_has(T.validate(d), "rx.vendor_pac_applied_step",
                        T.ConfigReason.CALIBRATION_MISMATCH)
        e = _minimal()
        e.rx.vendor_pac_value = 512
        e.rx.vendor_pac_backend = "cg600-pac-v1"
        e.rx.vendor_pac_applied_step = 0.0
        self.assert_has(T.validate(e), "rx.vendor_pac_applied_step",
                        T.ConfigReason.OUT_OF_RANGE)
        ok = _minimal()
        ok.rx.vendor_pac_value = 512
        ok.rx.vendor_pac_backend = "cg600-pac-v1"
        ok.rx.vendor_pac_applied_step = 2.0
        self.assertTrue(T.validate(ok).ok(), T.validate(ok).to_string())
        f = _minimal()
        f.rx.vendor_pac_applied_step = 2.0
        self.assert_has(T.validate(f), "rx.vendor_pac_applied_step",
                        T.ConfigReason.FIELD_CONFLICT)


# ==========================================================================
# J. Lifecycle: overrides, immutability, mid-exchange mutation
# ==========================================================================


class TestLifecycle(TwrConfigTestBase):

    def test_overrides_apply_at_an_exchange_boundary(self):
        caps = T.capabilities()
        e = T.effective_config(_minimal(T.Protocol.DS, T.Role.INITIATOR), caps)
        snap = T.TwrConfigSnapshot(e, caps)
        self.assertTrue(snap.ok())
        self.assertEqual(snap.in_flight(), 0)

        o = T.PerMessageOverrides(measurement_count=5,
                                  measurement_interval_ns=80_000_000)
        report, nxt = snap.set_overrides(o)
        self.assertTrue(report.ok(), report.to_string())
        self.assertTrue(nxt.ok())
        self.assertEqual(nxt.get().effective.session.measurement_count, 5)
        self.assertEqual(nxt.get().requested.session.measurement_count, 1)
        # The original is untouched: a snapshot is a value, not a handle.
        self.assertEqual(snap.get().effective.session.measurement_count, 1)
        # The requested side of the new snapshot is still the original request.
        self.assertEqual(nxt.get().effective.timing.response_to_final.value.nanos(),
                         600_000)

    def test_overrides_carry_no_phy_frequency_gain_address_or_calibration(self):
        """Synchronisation-affecting values are NOT overridable at all."""
        forbidden = ("preamble", "code", "channel", "frequency", "hz", "gain",
                     "amplitude", "address", "pan", "calibration", "sts",
                     "sfd", "prf", "data_rate", "bandwidth", "serial", "profile")
        for name in T.PerMessageOverrides.OVERRIDABLE_FIELDS:
            for token in forbidden:
                self.assertNotIn(token, name,
                                 "%s must not be overridable" % name)
        # And there is no attribute to sneak one in through.
        instance = T.PerMessageOverrides()
        for token in forbidden:
            self.assertFalse(
                any(token in attr for attr in vars(instance)),
                "PerMessageOverrides exposes a %s field" % token)
        self.assertEqual(len(vars(instance)), 7)
        self.assertFalse(T.PerMessageOverrides().any())
        self.assertTrue(T.PerMessageOverrides(measurement_count=1).any())

    def test_overrides_are_validated_not_trusted(self):
        caps = T.capabilities()
        snap = T.TwrConfigSnapshot(
            T.effective_config(_minimal(T.Protocol.DS, T.Role.INITIATOR), caps),
            caps)
        # An RX timeout smaller than the 20 us RX window it must contain.
        report, nxt = snap.set_overrides(T.PerMessageOverrides(rx_timeout_ns=10))
        self.assert_machine_readable(report)
        self.assert_has(report, "timeouts.response_rx_window.ns",
                        T.ConfigReason.TIMING_ORDER_VIOLATION)
        self.assertFalse(nxt.ok())

    def test_mid_exchange_mutation_is_refused(self):
        caps = T.capabilities()
        snap = T.TwrConfigSnapshot(
            T.effective_config(_minimal(T.Protocol.DS, T.Role.INITIATOR), caps),
            caps)
        begin = snap.begin_exchange()
        self.assertTrue(begin.ok(), begin.to_string())
        self.assertEqual(snap.in_flight(), 1)

        o = T.PerMessageOverrides(measurement_count=99,
                                  measurement_interval_ns=80_000_000)
        report, nxt = snap.set_overrides(o)
        self.assert_machine_readable(report)
        v = report.violations[0]
        self.assertIs(v.reason, T.ConfigReason.MID_EXCHANGE_MUTATION)
        self.assertIs(v.status, T.ExchangeStatus.CONFIG_REJECTED)
        self.assertEqual(v.requirement, "REQ-API-03")
        self.assertFalse(nxt.ok())
        # The running exchange keeps the snapshot it started with.
        self.assertEqual(snap.get().effective.session.measurement_count, 1)
        self.assertEqual(
            snap.get().effective.timing.response_to_final.value.nanos(), 600_000)

        # Once the exchange has ended, the same override is legal.
        snap.end_exchange()
        report2, nxt2 = snap.set_overrides(o)
        self.assertTrue(report2.ok(), report2.to_string())
        self.assertTrue(nxt2.ok())
        self.assertEqual(nxt2.get().effective.session.measurement_count, 99)

    def test_second_concurrent_exchange_is_queue_full_through_the_snapshot(self):
        caps = T.capabilities()
        snap = T.TwrConfigSnapshot(
            T.effective_config(_minimal(), caps), caps)
        self.assertTrue(snap.begin_exchange().ok())
        second = snap.begin_exchange()
        self.assert_machine_readable(second)
        self.assertIs(second.violations[0].status, T.ExchangeStatus.QUEUE_FULL)
        snap.end_exchange()
        self.assertTrue(snap.begin_exchange().ok())

    def test_overrides_on_an_invalid_base_are_refused(self):
        caps = T.capabilities()
        bad = _minimal()
        bad.frame.sts_mode = T.StsMode.SP128
        snap = T.TwrConfigSnapshot(T.effective_config(bad, caps), caps)
        self.assertFalse(snap.ok())
        report, nxt = snap.set_overrides(T.PerMessageOverrides(
            measurement_count=3))
        self.assert_machine_readable(report)
        self.assertFalse(nxt.ok())


# ==========================================================================
# J2. Value semantics at the config boundary (M0.1, closes R1)
# ==========================================================================
#
# docs/twr/M0_复核报告.md R1 is a three-line reproducer:
#
#     e = T.effective_config(c)
#     c.phy.channel = 9          # bypassing set_overrides() entirely
#
# It used to leave ``e.ok`` True, ``e.config_hash`` unchanged and
# ``e.effective.phy.channel == 9``: a config that no longer validates, behind a
# snapshot that still asserts that it does, with the hash still naming the
# config that was validated.  The defect is not one aliased attribute -- the
# whole boundary handed out references instead of values, so the same bypass
# existed through the readback, through nested lists and through the
# process-wide capability whitelist.
#
# Everything below is the inverse statement, and it is a CONTRACT, not a
# convention: no reference this module hands out may reach a validated config,
# and the only legal way to change one is ``set_overrides()`` at an exchange
# boundary (REQ-API-01, REQ-API-03).


class TestValueIsolation(TwrConfigTestBase):
    """A validated config is a value; the snapshot is a value of a value."""

    # -- helpers ---------------------------------------------------------
    def assertNotSameObject(self, a, b, what):
        """Identity, with a short message.

        ``assertIsNot`` on a TwrConfig or an EffectiveConfig prints a 6 kB repr
        when it fails, which buries the assertion that actually matters.
        """
        self.assertIsNot(a, b, "%s must not be the same object" % what)

    def _snapshot_of(self, protocol=T.Protocol.SS, role=T.Role.INITIATOR):
        """A valid snapshot, plus the EffectiveConfig it was built from."""
        caps = T.capabilities()
        e = T.effective_config(_minimal(protocol, role), caps)
        self.assertTrue(e.ok, e.validation.to_string())
        return T.TwrConfigSnapshot(e, caps), e

    @staticmethod
    def _fields(cfg):
        """The complete canonical text of a config: one string, every field."""
        return T.canonical_text(T.flatten_fields(cfg))

    @staticmethod
    def _quantisation(e):
        """Everything ``effective_config()`` derived, as a comparable tuple."""
        return (e.tick_rate_hz, e.poll_start_ticks, e.poll_to_response_ticks,
                e.response_to_final_ticks, e.post_tx_rx_enable_ticks,
                e.poll_start_effective.nanos(),
                e.poll_to_response_effective.nanos(),
                e.response_to_final_effective.nanos(),
                e.post_tx_rx_enable_effective.nanos(),
                e.poll_bytes, e.response_bytes, e.final_bytes,
                e.max_psdu_bytes, e.max_timestamp_count,
                tuple((ch.path, ch.requested, ch.effective) for ch in e.changes))

    @staticmethod
    def _cap_mutations(caps):
        """Every way a caller could try to widen or narrow a capability table.

        Container mutation, element assignment, in-place reordering and plain
        attribute rebinding are separate holes and are all attempted: blocking
        only ``append`` would still leave ``caps.channels = [...]`` open.  Row
        EDITING is attempted too, because a mutable row reached through a
        read-only table is a hole straight through the freeze.

        Row attempts are skipped on a table that has no rows (the pre-matrix
        default-deny table is empty by construction), since there is nothing to
        edit there and an IndexError would not be the refusal under test.
        """
        row = T.PhyCapabilityRow(
            native_rate_hz=T.NATIVE_RATE_UC200_HZ, code_index=9,
            sync_repetitions=128, sfd_mode=T.SfdMode.R4Z2,
            max_psdu_bytes=T.MAX_PSDU_BYTES, ranging=True,
            status=T.CapabilityStatus.MEASURED, reason="synthetic, not measured")
        attempts = [
            ("append a channel", lambda: caps.channels.append(9)),
            ("assign a channel", lambda: caps.channels.__setitem__(0, 9)),
            ("delete a channel", lambda: caps.channels.__delitem__(0)),
            ("clear the channels", lambda: caps.channels.clear()),
            ("sort the channels", lambda: caps.channels.sort(reverse=True)),
            ("extend the channels", lambda: caps.channels.extend([9, 10])),
            ("append a native rate", lambda: caps.native_rates_hz.append(1.2e8)),
            ("append a data rate",
             lambda: caps.data_rates.append(T.DataRate.R850K)),
            ("append SYNC 128", lambda: caps.sync_repetitions.append(128)),
            ("append a measured row", lambda: caps.phy_matrix.append(row)),
            ("clear the measured rows", lambda: caps.phy_matrix.clear()),
            ("append an STS mode",
             lambda: caps.sts_modes.append(T.StsMode.SP128)),
            ("rebind channels", lambda: setattr(caps, "channels", [9])),
            ("rebind sts_supported", lambda: setattr(caps, "sts_supported", True)),
            ("rebind phy_matrix", lambda: setattr(caps, "phy_matrix", [row])),
            ("delete a table field", lambda: delattr(caps, "channels")),
        ]
        if len(caps.phy_matrix):
            attempts += [
                ("rewrite a row's verdict",
                 lambda: setattr(caps.phy_matrix[0], "verdict", "supported")),
                ("rewrite a row's reason",
                 lambda: setattr(caps.phy_matrix[0], "reason", "rewritten")),
                ("rewrite a row's max_psdu_bytes",
                 lambda: setattr(caps.phy_matrix[0], "max_psdu_bytes", 127)),
            ]
        if len(caps.phy_matrix_rejections):
            attempts.append(
                ("rewrite a rejection row's reason",
                 lambda: setattr(caps.phy_matrix_rejections[0], "reason", "gone")))
        return attempts

    # -- 1. the requested config is not the effective one ----------------
    def test_mutating_the_requested_config_cannot_reach_the_snapshot(self):
        """Every axis of the caller's object, one by one."""
        c = _minimal()
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        before_hash = e.config_hash
        before_validation = e.validation.to_string()
        before_fields = self._fields(e.effective)
        before_quant = self._quantisation(e)

        # (a) a scalar field, (b) a nested dataclass field, (c) a list ELEMENT
        # and the list itself, (d) the readback the snapshot also carries.
        c.phy.channel = 9
        c.timing.post_tx_rx_enable.value = T.Duration(1)
        c.radio.peers[0].tx_channel = 0
        c.radio.peers.append(T.EndpointBinding(id="Z"))
        c.radio.readback.mpm_string = "CG400"

        self.assertEqual(e.requested.phy.channel, 5)
        self.assertEqual(e.effective.phy.channel, 5)
        self.assertEqual(e.requested.timing.post_tx_rx_enable.value.nanos(), 2000)
        self.assertEqual(e.effective.timing.post_tx_rx_enable.value.nanos(), 2000)
        self.assertEqual(e.effective.radio.peers[0].tx_channel, 1)
        self.assertEqual(len(e.effective.radio.peers), 1)
        self.assertEqual(e.requested.radio.peers[0].id, "B")
        self.assertEqual(e.readback.mpm_string, "X410")
        self.assertEqual(e.readback.tx_channel, e.effective.radio.tx_channel)
        self.assertEqual(self._fields(e.effective), before_fields)
        self.assertEqual(self._fields(e.requested), before_fields)
        # The derived facts must not move either: a hash that still names a
        # config that no longer exists is exactly the R1 symptom.
        self.assertEqual(e.config_hash, before_hash)
        self.assertEqual(e.validation.to_string(), before_validation)
        self.assertEqual(self._quantisation(e), before_quant)
        self.assertTrue(e.ok)
        self.assertTrue(T.validate(e.effective).ok())
        self.assertEqual(T.config_hash(e.effective), before_hash)

    def test_nested_lists_handed_out_by_a_snapshot_are_contained(self):
        """``radio.peers`` is a list of dataclasses: the deepest container here."""
        snap, e = self._snapshot_of()
        held = snap.get()
        self.assertEqual(len(held.effective.radio.peers), 1)

        held.effective.radio.peers[0].tx_channel = 0
        held.effective.radio.peers[0].id = "Z"
        held.effective.radio.peers[0].occupies_resources = False
        held.effective.radio.peers.clear()
        held.effective.radio.peers.append(T.EndpointBinding(id="X"))
        held.effective.phy.channel = 9
        held.effective.calibration.record.channel = 9
        held.requested.radio.peers[0].tx_channel = 0

        after = snap.get()
        self.assertEqual(len(after.effective.radio.peers), 1)
        self.assertEqual(after.effective.radio.peers[0].tx_channel, 1)
        self.assertEqual(after.effective.radio.peers[0].id, "B")
        self.assertTrue(after.effective.radio.peers[0].occupies_resources)
        self.assertEqual(after.effective.phy.channel, 5)
        self.assertEqual(after.effective.calibration.record.channel, 5)
        self.assertEqual(after.requested.radio.peers[0].tx_channel, 1)
        self.assertTrue(T.validate(after.effective).ok())
        # The very EffectiveConfig effective_config() handed out is a value too.
        self.assertEqual(len(e.effective.radio.peers), 1)
        self.assertEqual(e.effective.radio.peers[0].tx_channel, 1)
        self.assertEqual(e.effective.phy.channel, 5)

    # -- 2. get() is a copy, not a handle --------------------------------
    def test_get_returns_a_value_that_cannot_affect_the_snapshot(self):
        snap, _ = self._snapshot_of()
        first = snap.get()
        second = snap.get()
        self.assertNotSameObject(first, second, "two get() results")
        self.assertNotSameObject(first.effective, second.effective,
                                 "the effective config of two get() results")
        self.assertNotSameObject(first.effective.radio.peers[0],
                                 second.effective.radio.peers[0],
                                 "a peer of two get() results")
        self.assertNotSameObject(first.readback, second.readback,
                                 "the readback of two get() results")
        before_fields = self._fields(first.effective)
        before_changes = len(first.changes)
        before_violations = len(first.validation.violations)

        # Tamper with EVERY mutable part of the handed-out value: the config
        # itself, a nested dataclass, a list, the readback, the report and the
        # change list.
        first.effective.phy.channel = 9
        first.effective.calibration.tx_link_delay = T.Duration(999)
        first.effective.radio.peers[0].tx_channel = 0
        first.effective.radio.readback.mpm_string = "CG400"
        first.readback.mpm_string = "CG400"
        first.changes.clear()
        first.validation.add(T.ConfigViolation(
            field="phy.channel", reason=T.ConfigReason.UNSUPPORTED,
            status=T.ExchangeStatus.UNSUPPORTED, message="injected by a caller",
            requirement="REQ-API-01"))
        first.config_hash = "fnv1a64:0000000000000000"
        first.ok = False

        third = snap.get()
        self.assertEqual(self._fields(third.effective), before_fields)
        self.assertEqual(len(third.changes), before_changes)
        self.assertEqual(len(third.validation.violations), before_violations)
        self.assertTrue(third.validation.ok())
        self.assertTrue(third.ok)
        self.assertNotEqual(third.config_hash, "fnv1a64:0000000000000000")
        self.assertEqual(third.readback.mpm_string, "X410")
        self.assertEqual(third.effective.radio.readback.mpm_string, "X410")
        self.assertEqual(third.effective.radio.peers[0].tx_channel, 1)
        self.assertTrue(snap.ok())
        self.assertTrue(T.validate(third.effective).ok())

    def test_snapshot_copies_the_effective_config_it_is_given(self):
        """The constructor takes a value, so the caller's object stays theirs."""
        e = T.effective_config(_minimal())
        caps = T.capabilities()
        self.assertTrue(e.ok, e.validation.to_string())
        snap = T.TwrConfigSnapshot(e, caps)

        e.effective.phy.channel = 9
        e.requested.phy.channel = 9
        e.readback.mpm_string = "CG400"
        e.changes.clear()
        e.ok = False

        got = snap.get()
        self.assertTrue(got.ok)
        self.assertEqual(got.effective.phy.channel, 5)
        self.assertEqual(got.requested.phy.channel, 5)
        self.assertEqual(got.readback.mpm_string, "X410")
        self.assertTrue(got.validation.ok())
        self.assertEqual(got.config_hash, e.config_hash)
        self.assertTrue(T.validate(got.effective).ok())
        self.assertTrue(snap.ok())
        # ... and the capability table it was given is its own value as well.
        self.assertNotSameObject(snap.caps, caps, "the snapshot's caps")

    def test_snapshot_exposes_no_mutable_handle_on_its_payload(self):
        """The docstring claims there is no mutable accessor: prove the surface."""
        snap, _ = self._snapshot_of()
        payload_types = (T.EffectiveConfig, T.TwrConfig, T.SessionConfig,
                         T.PhyConfig, T.FrameGeometry, T.RadioReadback,
                         T.CalibrationRecord)
        for name in dir(type(snap)):
            if name.startswith("_"):
                continue
            attr = getattr(snap, name)
            for kind in payload_types:
                self.assertNotIsInstance(
                    attr, kind,
                    "TwrConfigSnapshot.%s hands out a %s" % (name, kind.__name__))
        # ``caps`` is public and is the one object it does expose: it must be a
        # read-only table, and it must not be rebindable.
        self.assertIsInstance(snap.caps, T.Capabilities)
        with self.assertRaises(AttributeError):
            snap.caps = T.capabilities()

    def test_capability_freeze_cannot_be_undone_through_public_api(self):
        """N06: no public method unfreezes a table that is in use.

        The removed bypass needed no private access at all::

            s.caps.thaw()          # public, unfroze IN PLACE
            s.caps.channels.append(9)   # now accepted

        which made the validation basis of a running exchange mutable.  There is
        no public in-place unfreeze any more: the only way to obtain a mutable
        table is ``mutable_copy()``, and it returns a NEW object.
        """
        snap, _ = self._snapshot_of()
        frozen = snap.caps
        self.assertTrue(frozen.read_only)
        before_channels = list(frozen.channels)
        before_rows = len(frozen.phy_matrix)

        # (1) There is no public in-place unfreeze left to call.
        for name in ("thaw", "unfreeze", "melt", "_thaw"):
            self.assertFalse(hasattr(frozen, name),
                             "Capabilities.%s is a public unfreeze" % name)

        # (2) Exercise EVERY public method that takes no arguments.  A refusal
        #     is fine; a mutation is not.
        for name in dir(type(frozen)):
            if name.startswith("_"):
                continue
            attr = getattr(frozen, name)
            if not callable(attr):
                continue
            try:
                attr()
            except TypeError:
                pass        # needs arguments; the mutators do not
            except Exception:
                pass        # a refusal of any kind is acceptable here
        self.assertTrue(
            frozen.read_only,
            "a public method unfroze the snapshot's capability table")
        self.assertEqual(list(frozen.channels), before_channels)
        self.assertEqual(len(frozen.phy_matrix), before_rows)

        # (3) mutable_copy() is INDEPENDENT: widening it must reach neither the
        #     snapshot nor the process-wide shipped table.
        mine = frozen.mutable_copy()
        self.assertFalse(mine.read_only)
        mine.channels.append(9)
        mine.sync_repetitions.append(128)
        mine.phy_matrix.clear()
        self.assertEqual(list(frozen.channels), before_channels)
        self.assertEqual(len(frozen.phy_matrix), before_rows)
        self.assertEqual(snap.caps.channels, before_channels)
        self.assertEqual(T.capabilities().channels, [5])
        self.assertEqual(T.capabilities().sync_repetitions, [16, 64])
        self.assertEqual(len(T.capabilities().phy_matrix), 48)
        # copy() is the same entry point; a builder gets a mutable table.
        self.assertFalse(frozen.copy().read_only)

    # -- 3. the capability table is a value too --------------------------
    def test_shipped_capability_table_refuses_every_mutation_attempt(self):
        good = _minimal()
        bad = _minimal()
        bad.phy.channel = 9               # not on the measured whitelist
        self.assertFalse(T.validate(bad).ok())
        before = T.validate(bad).to_string()

        tables = [
            ("capabilities()", T.capabilities()),
            ("unmeasured_capabilities()", T.unmeasured_capabilities()),
            ("default_deny_capabilities()", T.default_deny_capabilities()),
            ("build_measured_capabilities()", T.build_measured_capabilities()),
            ("build_default_capabilities()", T.build_default_capabilities()),
            ("build_unmeasured_capabilities()",
             T.build_unmeasured_capabilities()),
            ("snapshot.caps", T.TwrConfigSnapshot(
                T.effective_config(_minimal())).caps),
        ]
        for label, caps in tables:
            for what, mutate in self._cap_mutations(caps):
                with self.assertRaises(
                        (AttributeError, TypeError),
                        msg="%s: %s was accepted" % (label, what)):
                    mutate()

        # Nothing moved, and a later validation is bit-for-bit what it was.
        self.assertEqual(T.capabilities().channels, [5])
        self.assertEqual(T.capabilities().sync_repetitions, [16, 64])
        self.assertEqual(T.capabilities().data_rates, [T.DataRate.R6P8M])
        self.assertEqual(len(T.capabilities().phy_matrix), 48)
        self.assertFalse(T.capabilities().sts_supported)
        self.assertEqual(T.validate(bad).to_string(), before)
        self.assertTrue(T.validate(good).ok())
        # A widened table would have turned this rejection into an acceptance;
        # it must still be a rejection, and still say why.
        self.assert_has(T.validate(bad), "phy.channel", T.ConfigReason.UNSUPPORTED)
        longer = _minimal()
        longer.phy.preamble_symbols = 128
        self.assertFalse(T.validate(longer).ok())

    def test_capability_table_cannot_be_widened_through_a_builder_either(self):
        """The builder's escape hatch must not leak back into the shipped table."""
        caps = T.capabilities()
        wide = T.CapabilitiesBuilder(caps) \
            .add_channel(9).add_sync_repetitions(128).add_phy_row(
                T.PhyCapabilityRow(
                    native_rate_hz=T.NATIVE_RATE_UC200_HZ, code_index=9,
                    sync_repetitions=128, sfd_mode=T.SfdMode.R4Z2,
                    max_psdu_bytes=T.MAX_PSDU_BYTES, ranging=True,
                    status=T.CapabilityStatus.MEASURED, reason="synthetic")
            ).build()
        self.assertIn(9, wide.channels)
        self.assertIn(128, wide.sync_repetitions)
        # The shipped table is untouched...
        self.assertEqual(caps.channels, [5])
        self.assertEqual(caps.sync_repetitions, [16, 64])
        self.assertEqual(len(caps.phy_matrix), 48)
        # ... and its rows are values, so the evidence cannot be edited in place.
        # A mutable row reached through a read-only table would turn a measured
        # rejection into an acceptance with one assignment, so the row itself
        # must refuse it.  (FrozenInstanceError is a dataclasses subclass of
        # AttributeError; naming it keeps a non-frozen row from passing here.)
        row = caps.phy_matrix[0]
        with self.assertRaises(dataclasses.FrozenInstanceError):
            row.verdict = "supported"
        self.assertEqual(caps.phy_matrix[0].verdict, row.verdict)
        # A different row may be DERIVED; it just is not the shipped one.
        derived = dataclasses.replace(
            row, sync_repetitions=128, reason="synthetic")
        self.assertNotSameObject(derived, row, "a derived row")
        self.assertEqual(row.sync_repetitions, 16)
        self.assertEqual(derived.sync_repetitions, 128)
        # ... and the derived table is a value too: editing it is not an option.
        for what, mutate in self._cap_mutations(wide):
            with self.assertRaises((AttributeError, TypeError),
                                   msg="derived table: %s was accepted" % what):
                mutate()

    # -- 4. overrides stay the only legal mutation -----------------------
    def test_set_overrides_is_the_only_accepted_mutation_path(self):
        snap, _ = self._snapshot_of(T.Protocol.DS, T.Role.INITIATOR)
        report, nxt = snap.set_overrides(T.PerMessageOverrides(
            measurement_count=5, measurement_interval_ns=80_000_000))
        self.assertTrue(report.ok(), report.to_string())
        self.assertTrue(nxt.ok())

        # The new snapshot's value is still a value.
        held = nxt.get()
        held.effective.session.measurement_count = 99
        held.effective.timing.response_to_final.value = T.Duration(1)
        held.effective.radio.peers[0].tx_channel = 0
        self.assertEqual(nxt.get().effective.session.measurement_count, 5)
        self.assertEqual(
            nxt.get().effective.timing.response_to_final.value.nanos(), 600_000)
        self.assertEqual(nxt.get().effective.radio.peers[0].tx_channel, 1)
        # The requested side still carries the ORIGINAL request.
        self.assertEqual(nxt.get().requested.session.measurement_count, 1)
        # ... and the base snapshot is a different value from the new one.
        self.assertNotSameObject(nxt.get().effective, snap.get().effective,
                                 "the new snapshot's effective config")
        self.assertEqual(snap.get().effective.session.measurement_count, 1)
        self.assertEqual(snap.get().effective.phy.channel, 5)
        # Tampering with the returned value must not break the next override
        # either: it is re-validated from the snapshot's own copy.
        report2, nxt2 = nxt.set_overrides(T.PerMessageOverrides(
            measurement_count=7))
        self.assertTrue(report2.ok(), report2.to_string())
        self.assertEqual(nxt2.get().effective.session.measurement_count, 7)

    def test_mid_exchange_override_is_refused_and_the_running_value_survives(self):
        snap, _ = self._snapshot_of(T.Protocol.DS, T.Role.INITIATOR)
        before = self._fields(snap.get().effective)
        self.assertTrue(snap.begin_exchange().ok())
        report, nxt = snap.set_overrides(T.PerMessageOverrides(
            measurement_count=99, measurement_interval_ns=80_000_000))
        self.assert_machine_readable(report)
        self.assertIs(report.violations[0].reason,
                      T.ConfigReason.MID_EXCHANGE_MUTATION)
        self.assertIs(report.violations[0].status, T.ExchangeStatus.CONFIG_REJECTED)
        self.assertFalse(nxt.ok())
        # Neither the running exchange nor the refused snapshot is reachable.
        held = snap.get()
        held.effective.phy.channel = 9
        held.effective.session.measurement_count = 42
        self.assertEqual(self._fields(snap.get().effective), before)
        self.assertEqual(snap.get().effective.phy.channel, 5)
        self.assertEqual(snap.get().effective.session.measurement_count, 1)
        self.assertTrue(T.validate(snap.get().effective).ok())
        snap.end_exchange()
        self.assertTrue(snap.get().ok)

    # -- 5. hash / ok / quantisation survive an attempted tamper ----------
    def test_hash_ok_and_quantisation_are_stable_under_tampering(self):
        """The reviewer's reproducer, with every derived fact asserted."""
        c = _minimal(T.Protocol.SS, T.Role.RESPONDER)
        e = T.effective_config(c)
        self.assertTrue(e.ok, e.validation.to_string())
        old_hash = e.config_hash
        old_validation = e.validation.to_string()
        old_fields = self._fields(e.effective)
        old_quant = self._quantisation(e)
        # The quantisation really ran, so "unchanged" is not vacuous.
        self.assertEqual(e.tick_rate_hz, T.NATIVE_RATE_UC200_HZ)
        self.assertEqual(e.poll_to_response_ticks, 442368)

        c.phy.channel = 9                       # the R1 one-liner
        self.assertTrue(e.ok)
        self.assertEqual(e.effective.phy.channel, 5)
        self.assertEqual(e.requested.phy.channel, 5)
        self.assertEqual(e.config_hash, old_hash)
        self.assertEqual(e.validation.to_string(), old_validation)
        self.assertEqual(self._fields(e.effective), old_fields)
        self.assertEqual(self._quantisation(e), old_quant)
        self.assertTrue(T.validate(e.effective).ok())
        # The hash identifies BOTH sides of the snapshot, and the config the
        # caller tampered with is a different config with a different hash.
        self.assertEqual(T.config_hash(e.requested), old_hash)
        self.assertEqual(T.config_hash(e.effective), old_hash)
        self.assertNotEqual(T.config_hash(c), old_hash)

        # The same holds through a snapshot, including the readback it carries.
        c2 = _minimal()
        clean = T.effective_config(_minimal())
        snap = T.TwrConfigSnapshot(clean, T.capabilities())
        c2.radio.readback.mpm_string = "CG400"
        c2.radio.readback.sample_rate_hz = 491.52e6
        got = snap.get()
        self.assertEqual(got.readback.mpm_string, "X410")
        self.assertEqual(got.readback.sample_rate_hz, T.NATIVE_RATE_UC200_HZ)
        self.assertEqual(got.effective.radio.readback.mpm_string, "X410")
        self.assertEqual(got.config_hash, clean.config_hash)
        # The tampered caller's config is a DIFFERENT config, with its own hash:
        # the snapshot's hash still names the one that was validated.
        self.assertNotEqual(T.config_hash(c2), got.config_hash)

    # -- 6. identity: the snapshot owns three independent values ---------
    def test_effective_config_owns_three_independent_values(self):
        c = _minimal()
        e = T.effective_config(c)
        self.assertNotSameObject(e.effective, c, "effective vs the caller's config")
        self.assertNotSameObject(e.requested, c, "requested vs the caller's config")
        self.assertNotSameObject(e.requested, e.effective, "requested vs effective")
        self.assertNotSameObject(e.readback, c.radio.readback,
                                 "readback vs the caller's readback")
        self.assertNotSameObject(e.readback, e.effective.radio.readback,
                                 "readback vs effective.radio.readback")
        self.assertNotSameObject(e.readback, e.requested.radio.readback,
                                 "readback vs requested.radio.readback")
        self.assertNotSameObject(e.effective.radio, c.radio, "effective.radio")
        self.assertNotSameObject(e.effective.radio.peers, c.radio.peers,
                                 "the peer LIST")
        self.assertNotSameObject(e.effective.radio.peers[0], c.radio.peers[0],
                                 "a peer ELEMENT")
        self.assertNotSameObject(e.effective.calibration.record,
                                 c.calibration.record, "calibration.record")
        self.assertNotSameObject(e.effective.session.measurement_interval,
                                 c.session.measurement_interval,
                                 "a Duration-valued field")
        # A copy is still the same config: equal text, equal hash.
        self.assertEqual(self._fields(e.requested), self._fields(c))
        self.assertEqual(self._fields(e.effective), self._fields(c))
        self.assertEqual(e.config_hash, T.config_hash(c))
        # The rejected path is a value as well, and invents nothing.
        bad = _minimal()
        bad.phy.channel = 9
        eb = T.effective_config(bad)
        self.assertFalse(eb.ok)
        self.assertNotSameObject(eb.requested, bad, "rejected: requested")
        self.assertNotSameObject(eb.effective, bad, "rejected: effective")
        self.assertEqual(eb.effective, T.TwrConfig())
        self.assertEqual(eb.changes, [])
        bad.phy.channel = 5
        self.assertEqual(eb.effective.phy.channel, 5)   # still value-initialised

    def test_value_copy_helper_is_deep(self):
        """The documented copy a caller may use is the one the module uses."""
        c = _minimal()
        dup = T.copy_config(c)
        self.assertNotSameObject(dup, c, "copy_config result")
        self.assertNotSameObject(dup.radio, c.radio, "copy_config radio")
        self.assertNotSameObject(dup.radio.peers, c.radio.peers,
                                 "copy_config peer list")
        self.assertNotSameObject(dup.radio.peers[0], c.radio.peers[0],
                                 "copy_config peer element")
        self.assertEqual(T.canonical_text(T.flatten_fields(dup)),
                         T.canonical_text(T.flatten_fields(c)))
        dup.radio.peers[0].tx_channel = 0
        dup.phy.channel = 9
        self.assertEqual(c.radio.peers[0].tx_channel, 1)
        self.assertEqual(c.phy.channel, 5)


# ==========================================================================
# K. Report quality
# ==========================================================================


class TestReportQuality(TwrConfigTestBase):

    def test_multiple_violations_are_reported_together(self):
        c = _minimal()
        c.frame.sts_mode = T.StsMode.SP128
        c.frame.sts_length_symbols = 128
        c.radio.native_sample_rate_hz = 998.4e6   # the WORK rate, not native
        c.phy.tx_preamble_code = 7
        c.phy.rx_preamble_code = 7
        c.session.max_in_flight_exchanges = 4
        c.rx.detection_threshold = 5.0
        c.diagnostics.io_on_realtime_thread = True
        c.phy.preamble_symbols = 256

        report = T.validate(c)
        self.assert_machine_readable(report)
        self.assertGreaterEqual(len(report.violations), 7, report.to_string())
        self.assert_has(report, "frame.sts_mode", T.ConfigReason.OUT_OF_SCOPE)
        self.assert_has(report, "frame.sts_length_symbols",
                        T.ConfigReason.FIELD_CONFLICT)
        self.assert_has(report, "radio.native_sample_rate_hz",
                        T.ConfigReason.UNSUPPORTED)
        self.assert_has(report, "phy.tx_preamble_code",
                        T.ConfigReason.UNSUPPORTED)
        self.assert_has(report, "session.max_in_flight_exchanges",
                        T.ConfigReason.IN_FLIGHT_NOT_SUPPORTED)
        self.assert_has(report, "rx.detection_threshold",
                        T.ConfigReason.OUT_OF_RANGE)
        self.assert_has(report, "diagnostics.io_on_realtime_thread",
                        T.ConfigReason.FIELD_CONFLICT)
        # And every message is printable.
        msgs = report.messages()
        self.assertEqual(len(msgs), len(report.violations))
        for m in msgs:
            self.assertIn("[", m)
            self.assertIn("]", m)

    def test_report_lookup_helpers(self):
        c = _minimal()
        c.frame.sts_mode = T.StsMode.SP512
        c.frame.sts_length_symbols = 512
        v = T.validate(c)
        self.assert_machine_readable(v)
        self.assertTrue(v.has(T.ConfigReason.OUT_OF_SCOPE))
        self.assertTrue(v.has(T.ConfigReason.OUT_OF_SCOPE, "frame.sts_mode"))
        self.assertFalse(v.has(T.ConfigReason.OUT_OF_SCOPE, "frame.nothing"))
        self.assertTrue(v.has_field("frame.sts_mode"))
        self.assertFalse(v.has_field("frame.nothing"))
        self.assertIsNone(v.find("frame.sts_mode", T.ConfigReason.FIELD_CONFLICT))
        self.assertIsNotNone(v.first_for_field("frame.sts_mode"))
        self.assertIsNone(v.first_for_field("nope"))
        # The report is ordered by section, so the first status here is the
        # STS one.
        self.assertIs(v.first_status(), T.ExchangeStatus.UNSUPPORTED)
        self.assertNotEqual(v.to_string(), "ok")
        # A clean report prints as "ok".
        self.assertEqual(T.validate(_minimal()).to_string(), "ok")
        self.assertIs(T.validate(_minimal()).first_status(),
                      T.ExchangeStatus.OK)

    def test_every_default_constructed_config_is_refused(self):
        # A value-initialised config must not be silently accepted: the default
        # is a starting point for the operator, never a valid profile.
        c = T.TwrConfig()
        v = T.validate(c)
        self.assert_machine_readable(v)
        self.assertGreaterEqual(len(v.violations), 8, v.to_string())
        self.assert_has(v, "session.local_address",
                        T.ConfigReason.FIELD_CONFLICT)
        self.assert_has(v, "session.pan_id", T.ConfigReason.EMPTY_VALUE)
        self.assert_has(v, "phy.tx_preamble_code", T.ConfigReason.ZERO_VALUE)
        self.assert_has(v, "phy.preamble_symbols", T.ConfigReason.ZERO_VALUE)
        self.assert_has(v, "frame.geometry", T.ConfigReason.EMPTY_VALUE)
        self.assert_has(v, "timing.post_tx_rx_enable.domain",
                        T.ConfigReason.EMPTY_VALUE)
        self.assert_has(v, "calibration.calibration_id",
                        T.ConfigReason.CALIBRATION_MISSING)
        self.assert_has(v, "diagnostics.result_queue_capacity",
                        T.ConfigReason.ZERO_VALUE)
        # ... and it produces no effective config at all.
        e = T.effective_config(c)
        self.assertFalse(e.ok)
        self.assertEqual(e.changes, [])
        self.assertEqual(e.tick_rate_hz, 0.0)


# ==========================================================================
# L. Import-time invariants
# ==========================================================================


class TestModuleContract(TwrConfigTestBase):

    def test_module_has_no_third_party_dependency(self):
        """stdlib only: no numpy, no jsonschema, no pybind11, no GNU Radio."""
        import ast
        path = os.path.join(_PYTHON_DIR, "uwb", "twr_config.py")
        with open(path, "r", encoding="utf-8") as handle:
            tree = ast.parse(handle.read(), path)
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    imported.add(alias.name.split(".")[0])
            elif isinstance(node, ast.ImportFrom):
                if node.level:            # a relative import inside the package
                    continue
                if node.module:
                    imported.add(node.module.split(".")[0])
        allowed = {"json", "math", "dataclasses", "enum", "typing", "copy",
                   "__future__", "os", "unittest"}
        self.assertTrue(imported)
        self.assertEqual(imported - allowed, set(),
                         "twr_config.py imports outside the stdlib allow-list: %s"
                         % sorted(imported - allowed))
        # ... and importing it pulled in nothing beyond the stdlib.
        for name in ("numpy", "jsonschema", "gnuradio", "uhd", "pmt",
                     "pybind11"):
            self.assertNotIn(name, sys.modules,
                             "importing uwb.twr_config pulled in %s" % name)

    def test_orchestrator_owned_files_are_untouched(self):
        """`uwb/__init__.py` and the pybind bindings belong to someone else.

        This module must neither be imported by them nor wire itself into them:
        the pybind exposure is deferred to M1, and taking it here would
        collide with the binding work.
        """
        init_py = os.path.join(_PYTHON_DIR, "uwb", "__init__.py")
        self.assertTrue(os.path.isfile(init_py))
        with open(init_py, "r", encoding="utf-8") as handle:
            init_src = handle.read()
        self.assertNotIn("twr_config", init_src,
                         "uwb/__init__.py must not import twr_config (M1 owns "
                         "the export surface)")
        bindings = os.path.join(_PYTHON_DIR, "uwb", "bindings",
                                "python_bindings.cc")
        if os.path.isfile(bindings):
            with open(bindings, "r", encoding="utf-8") as handle:
                bindings_src = handle.read()
            self.assertNotIn("twr_config", bindings_src,
                             "the pybind bindings must not reference twr_config "
                             "(M1 owns the binding surface)")

    def test_public_api_is_complete_and_resolvable(self):
        """Every name in __all__ must exist, and must be usable by a caller."""
        missing = [n for n in T.__all__ if not hasattr(T, n)]
        self.assertEqual(missing, [], "__all__ lists names the module lacks")
        self.assertEqual(len(T.__all__), len(set(T.__all__)),
                         "__all__ has duplicates")
        # Every public class/function is documented: a bare name with no
        # docstring is a name nobody will be able to use correctly.  A CLASS is
        # documented by its own docstring OR by a section comment above it in
        # the source -- the module is organised that way, and requiring one of
        # the two is the real contract.  Requiring a hand-maintained list of
        # every name in the module docstring instead would just be a list that
        # goes stale the next time a symbol is added.
        import ast as _ast
        import inspect as _inspect
        src_path = os.path.join(_PYTHON_DIR, "uwb", "twr_config.py")
        with open(src_path, "r", encoding="utf-8") as handle:
            tree = _ast.parse(handle.read(), src_path)
        by_lineno = {}
        for node in _ast.walk(tree):
            if isinstance(node, (_ast.FunctionDef, _ast.ClassDef)):
                by_lineno[node.lineno] = node
        lines = open(src_path, "r", encoding="utf-8").read().splitlines()
        leading_comment = {}
        for lineno, node in by_lineno.items():
            i = lineno - 2
            while i >= 0 and (lines[i].lstrip().startswith("#") or not lines[i].strip()):
                if lines[i].strip():
                    break
                i -= 1
            if i >= 0 and lines[i].lstrip().startswith("#"):
                leading_comment[node.name] = lines[i].strip()
        undocumented = []
        for name in T.__all__:
            obj = getattr(T, name)
            if isinstance(obj, type):
                if not (obj.__doc__ or "").strip() and name not in leading_comment:
                    undocumented.append(name)
            elif inspect.isfunction(obj):
                if not (obj.__doc__ or "").strip() and name not in leading_comment:
                    undocumented.append(name)
        self.assertEqual(undocumented, [], "undocumented public names")
        # The whole surface a caller needs to drive one exchange.
        for name in ("capabilities", "validate", "effective_config",
                     "to_json_string", "from_json_string", "apply_calibration_once",
                     "TwrConfigSnapshot", "PerMessageOverrides", "ExchangeGate"):
            self.assertIn(name, T.__all__)

    def test_capabilities_are_immutable_by_convention(self):
        caps = T.capabilities()
        self.assertIs(caps, T.capabilities())      # one process-wide instance
        # A builder copy cannot mutate the shipped instance.
        T.CapabilitiesBuilder(caps).add_channel(9).build()
        self.assertEqual(T.capabilities().channels, [5])
        T.default_deny_capabilities()
        self.assertEqual(T.capabilities().channels, [5])


# ==========================================================================
# M. The M0.1 vocabulary: one geometry authority, two typed axes, evidence
# ==========================================================================


class TestM01Vocabulary(TwrConfigTestBase):
    """The drift the M0 review found, closed on the Python side.

    Each test here has a matching C++ assertion in
    gr-uwb/lib/qa_uwb_twr_config_parity.cc or the frozen header, and every one
    of them also has a case in the shared parity corpus.
    """

    def test_phr_rate_is_its_own_enumeration(self):
        self.assertEqual([m.value for m in T.PhrRate], ["850k", "same_as_data"])
        self.assertEqual(T.ALL_PHR_RATES,
                         (T.PhrRate.STANDARD_850K, T.PhrRate.SAME_AS_DATA))
        for m in T.PhrRate:
            self.assertIs(T.PhrRate.from_string(m.value), m)
        # Strict: one spelling per member, and NO payload-rate name.
        for bad in ("6p8m", "27m", "850K", "same_as_data ", "", "SAME_AS_DATA"):
            self.assertIsNone(T.PhrRate.from_string(bad), bad)
        # The base config's pair is the one M0 rejected.
        c = _minimal()
        self.assertIs(c.phy.phr_rate, T.PhrRate.STANDARD_850K)
        self.assertIs(c.phy.data_rate, T.DataRate.R6P8M)
        self.assertNotEqual(str(c.phy.phr_rate), str(c.phy.data_rate))
        self.assertTrue(T.validate(c).ok(), T.validate(c).to_string())
        # The data rate is still its own whitelist axis.
        self.assertEqual(caps_data_rates(), [T.DataRate.R6P8M])
        self.assertTrue(T.capabilities().phr_rate_supported(
            T.PhrRate.STANDARD_850K))
        self.assertFalse(T.capabilities().phr_rate_supported(
            T.PhrRate.SAME_AS_DATA))

    def test_preamble_length_enumeration_is_what_this_decoder_can_run(self):
        self.assertEqual([m.value for m in T.PreambleLength], ["16", "64"])
        self.assertEqual(T.ALL_PREAMBLE_LENGTHS,
                         (T.PreambleLength.SYM16, T.PreambleLength.SYM64))
        self.assertEqual(T.preamble_length_symbols(T.PreambleLength.SYM16), 16)
        self.assertEqual(T.preamble_length_symbols(T.PreambleLength.SYM64), 64)
        for m in T.PreambleLength:
            self.assertIs(T.preamble_length_from_symbols(
                T.preamble_length_symbols(m)), m)
        # Never "round to the nearest".
        for n in (0, 1, 8, 15, 17, 32, 63, 65, 128, 4096):
            self.assertIsNone(T.preamble_length_from_symbols(n), n)
        for n in (16, 64):
            self.assertTrue(T.capabilities().preamble_length_supported(
                T.preamble_length_from_symbols(n)))
        self.assertFalse(T.capabilities().preamble_length_supported(
            T.preamble_length_from_symbols(128)))

    def test_preamble_rejections_name_this_software_and_cite_the_vendor(self):
        """R7: "our decoder can't" must never harden into "the chip can't"."""
        for n in (1, 2, 4, 8, 32, 100, 128, 256, 512, 1024, 2048, 4096, 65):
            text = T.preamble_length_unsupported_reason(n)
            self.assertIn(str(n), text)
            for forbidden in ("the hardware cannot", "the chip cannot",
                              "structurally impossible", "not supported by the"):
                self.assertNotIn(forbidden, text,
                                 "preamble %d: %r" % (n, forbidden))
        # The vendor-supported lengths carry the citation, so an operator can go
        # and check it against the PDF.
        for n in (128, 256, 512, 1024, 2048):
            self.assertTrue(T.preamble_length_supported_by_vendor(n))
            note = T.preamble_length_reject_note(n)
            self.assertIn(T.reject_scope_current_software(), note)
            self.assertIn("not_a_hardware_claim_", note)
            self.assertIn("dw1000_api_guide_v2.7_sec5.12", note)
            self.assertIn("dw3xxx_api_guide_pdf_pp32-34", note)
        self.assertFalse(T.preamble_length_supported_by_vendor(16))
        self.assertFalse(T.preamble_length_supported_by_vendor(100))
        self.assertEqual(T.reject_scope_current_software(),
                         "current_software_implementation_limit")
        self.assertEqual(T.reject_scope_out_of_scope(), "out_of_scope_phase1")
        self.assertEqual(T.reject_scope_not_measured(), "not_measured")
        self.assertEqual(T.reject_scope_api_contract(),
                         "this_build_api_contract")

    def test_frame_profile_is_an_enumeration_and_geometry_is_derived(self):
        """R3: one geometry authority, and the profile selects it."""
        self.assertEqual([m.value for m in T.FrameProfileId], ["frame_v1"])
        self.assertTrue(T.frame_profile_id_is_supported(
            T.FrameProfileId.TWR_V1))
        # The frozen layout, and the reason it is 14 not 7.
        self.assertEqual(T.FRAME_HEADER_BYTES, 14)
        self.assertEqual(T.FRAME_FCS_BYTES, 2)
        self.assertEqual(T.FRAME_TIMESTAMP_BITS, 40)
        self.assertEqual(T.FRAME_TIMESTAMP_BYTES, 5)
        self.assertEqual(T.FRAME_MAX_TIMESTAMPS, 3)
        self.assertEqual(T.PHR_STANDARD_INFO_BYTES, 2)
        self.assertEqual(T.PHR_STANDARD_CODED_BITS, 19)
        self.assertEqual(T.FRAME_MAX_PSDU_BYTES, 127)
        self.assertIsNone(T.frame_geometry_for("frame_v2"))
        self.assertIsNone(T.frame_geometry_of_profile("frame_v2"))
        g = T.frame_geometry_of_profile(T.FrameProfileId.TWR_V1)
        self.assertEqual((g.mac_header_bytes, g.timestamp_bytes,
                          g.mac_footer_bytes, g.mac_fcs_bytes, g.phr_bytes),
                         (14, 5, 0, 0, 2))
        auth = T.frame_geometry_for(T.FrameProfileId.TWR_V1)
        self.assertTrue(auth.executable())
        # MAC PSDU 14/24/29; on air 16/26/31, invariant under who appends the
        # FCS.  24/29 is what M0 called 19/24.
        for t, mac, air in ((T.FrameType.POLL, 14, 16),
                            (T.FrameType.RESPONSE, 24, 26),
                            (T.FrameType.FINAL, 29, 31)):
            self.assertEqual(auth.mac_payload_bytes(t), mac)
            self.assertEqual(auth.on_air_bytes(t), air)
            self.assertEqual(T.frame_psdu_bytes(t, T.FcsAppender.PHY), mac)
            self.assertEqual(T.frame_psdu_bytes(t, T.FcsAppender.MAC), air)
            self.assertEqual(T.frame_bytes_on_air(t, T.FcsAppender.PHY), air)
            self.assertEqual(T.frame_bytes_on_air(t, T.FcsAppender.MAC), air)
        mac = T.frame_geometry_mac_appends_fcs()
        self.assertFalse(mac.executable())
        self.assertEqual(mac.mac_fcs_bytes(), 2)
        for t in (T.FrameType.POLL, T.FrameType.RESPONSE, T.FrameType.FINAL):
            self.assertEqual(mac.on_air_bytes(t), auth.on_air_bytes(t))
        # The authority's own claim checks back with no mismatch: that
        # self-consistency is what the R3 regression relies on.
        self.assertTrue(T.frame_geometry_check(auth.claim(), auth).ok())
        self.assertEqual(auth.claim(), T.frame_geometry_claim(
            T.frame_geometry_of_profile(T.FrameProfileId.TWR_V1)))

    def test_geometry_claim_is_checked_field_by_field(self):
        """Each disagreement is its OWN rejection naming that exact field."""
        report = T.frame_geometry_check(
            T.FrameGeometryClaim(mac_header_bytes=7, timestamp_bytes=4,
                                 mac_footer_bytes=2, mac_fcs_bytes=2,
                                 phr_bytes=12),
            T.frame_geometry_for(T.FrameProfileId.TWR_V1))
        self.assertFalse(report.ok())
        self.assertEqual([m.field for m in report.mismatches],
                         list(T.GEOMETRY_FIELD_ORDER))
        self.assertEqual([(m.field, m.expected, m.actual)
                          for m in report.mismatches],
                         [(T.GeometryField.MAC_HEADER_BYTES, 14, 7),
                          (T.GeometryField.TIMESTAMP_BYTES, 5, 4),
                          (T.GeometryField.MAC_FOOTER_BYTES, 0, 2),
                          (T.GeometryField.MAC_FCS_BYTES, 0, 2),
                          (T.GeometryField.PHR_BYTES, 2, 12)])
        self.assertIn("mac_header_bytes: expected 14, got 7", report.summary())
        self.assertIs(report.first_mismatch().field,
                      T.GeometryField.MAC_HEADER_BYTES)
        # And through the validator, one violation per field, in that order.
        c = _minimal()
        c.frame.geometry = T.FrameGeometry(mac_header_bytes=7,
                                           timestamp_bytes=4,
                                           phr_bytes=12)
        rep = T.validate(c)
        fields = [v.field for v in rep.violations
                  if v.field.startswith("frame.geometry.")]
        self.assertEqual(fields, ["frame.geometry.mac_header_bytes",
                                  "frame.geometry.timestamp_bytes",
                                  "frame.geometry.phr_bytes"])
        for v in rep.violations:
            if v.field.startswith("frame.geometry."):
                self.assertIs(v.reason, T.ConfigReason.FIELD_CONFLICT)
                self.assertEqual(v.requirement, "REQ-PROTO-06")
                self.assertIn("the frame codec is the geometry authority",
                              v.message)
        # The exact 7/12/2 fixture M0 shipped is refused for the same reason.
        d = _minimal()
        d.frame.geometry = T.FrameGeometry(mac_header_bytes=7,
                                           timestamp_bytes=5, phr_bytes=12)
        self.assert_has(T.validate(d), "frame.geometry.mac_header_bytes",
                        T.ConfigReason.FIELD_CONFLICT)
        self.assert_has(T.validate(d), "frame.geometry.phr_bytes",
                        T.ConfigReason.FIELD_CONFLICT)
        # A not-stated geometry is a missing value, not a silent zero.
        e = _minimal()
        e.frame.geometry = T.FrameGeometry()
        v = self.assert_has(T.validate(e), "frame.geometry",
                            T.ConfigReason.EMPTY_VALUE)
        self.assertIn("frame_geometry_of_profile", v.message)

    def test_frame_profile_is_required_and_a_v1_document_is_refused(self):
        c = _minimal()
        self.assertEqual(c.meta.schema_version, "twr-config/2")
        self.assertEqual(c.frame.frame_profile, T.FrameProfileId.TWR_V1)
        doc = T.to_json_dict(c)
        self.assertEqual(doc["frame"]["frame_profile"], "frame_v1")
        self.assertEqual(doc["phy"]["phr_rate"], "850k")
        self.assertEqual(doc["phy"]["data_rate"], "6p8m")
        self.assertIn("frame_profile", doc["frame"])
        # Round trip.
        back, rep = T.from_json_dict(doc)
        self.assertTrue(rep.ok(), rep.to_string())
        self.assertIs(back.frame.frame_profile, T.FrameProfileId.TWR_V1)
        # An unknown profile is refused by the READER; the resulting config
        # keeps the enumeration's zero member, and the REPORT is the authority.
        bad = T.to_json_dict(_minimal())
        bad["frame"]["frame_profile"] = "frame_v2"
        _cfg, rep = T.from_json_dict(bad)
        self.assertFalse(rep.ok())
        self.assert_has(rep, "frame.frame_profile", T.ConfigReason.UNKNOWN_ENUM_VALUE)
        # Absent: a missing_key, because twr-config/2 made it required.
        missing = T.to_json_dict(_minimal())
        del missing["frame"]["frame_profile"]
        _cfg, rep = T.from_json_dict(missing)
        self.assertFalse(rep.ok())
        self.assert_has(rep, "frame.frame_profile", T.ConfigReason.MISSING_KEY)
        # A twr-config/1 document is refused with the MIGRATION message, and
        # the message names the fields that changed.
        # The READER is a shape reader: it accepts the document, because a v1
        # document is still well-formed JSON with well-formed fields.  The
        # VERSION GATE lives in validate(), which is where a config is actually
        # admitted -- and that split is deliberate, so a document can be
        # inspected and diagnosed before it is refused.
        v1 = T.to_json_dict(_minimal())
        v1["meta"]["schema_version"] = "twr-config/1"
        old, rep = T.from_json_dict(v1)
        self.assertTrue(rep.ok(),
                        "the reader accepts a well-formed v1 document")
        v = self.assert_has(T.validate(old, T.capabilities()),
                            "meta.schema_version", T.ConfigReason.UNSUPPORTED)
        self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
        self.assertIn("twr-config/2 is a BREAKING change", v.message)
        self.assertIn("frame.frame_profile is now REQUIRED", v.message)
        self.assertIn("phy.phr_rate changed value domain", v.message)
        self.assertIn('frame.frame_profile = "frame_v1"', v.message)
        self.assertIn('phy.phr_rate = "850k"', v.message)
        # A version this build has never heard of gets the generic refusal,
        # NOT a migration message it knows nothing about.
        v9 = T.to_json_dict(_minimal())
        v9["meta"]["schema_version"] = "twr-config/9"
        c9, rep9 = T.from_json_dict(v9)
        self.assertTrue(rep9.ok(),
                        "the reader accepts any well-formed document")
        v9v = self.assert_has(T.validate(c9, T.capabilities()),
                              "meta.schema_version",
                              T.ConfigReason.UNSUPPORTED)
        self.assertIn("this build implements twr-config/2", v9v.message)
        self.assertNotIn("BREAKING change", v9v.message)
        # An empty version is BOTH "not stated" and "not this build's".
        ve = T.to_json_dict(_minimal())
        ve["meta"]["schema_version"] = ""
        _c, _impe = T.from_json_dict(ve)
        repe = T.validate(_c)
        self.assert_has(repe, "meta.schema_version", T.ConfigReason.EMPTY_VALUE)
        self.assertEqual(len([x for x in repe.violations
                              if x.field == "meta.schema_version"]), 2)
        # The exported document carries the version, and the snapshot reports
        # the version of the TABLE that validated it.
        e = T.effective_config(_minimal())
        self.assertEqual(e.schema_version, "twr-config/2")
        self.assertEqual(T.SCHEMA_VERSION, "twr-config/2")
        self.assertEqual(T.LEGACY_SCHEMA_VERSIONS, ("twr-config/1",))
        self.assertIn("BREAKING", T.SCHEMA_V1_MIGRATION_REASON)

    def test_phr_mode_extended_is_now_refused_with_its_own_reason(self):
        for mode, reason, tag in ((T.PhrMode.EXTENDED, T.ConfigReason.UNSUPPORTED,
                                   "REQ-PHY-01"),
                                  (T.PhrMode.NONE, T.ConfigReason.OUT_OF_SCOPE,
                                   "REQ-PHY-01")):
            c = _minimal()
            c.frame.phr_mode = mode
            v = self.assert_has(T.validate(c), "frame.phr_mode", reason)
            self.assertEqual(v.requirement, tag)
            self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
            self.assertEqual(v.message[len("PHR mode %s is not implemented: " % mode):],
                             T.phr_mode_unsupported_reason(mode))
        # Each member says why, in its own words.
        self.assertIn("13 information bits",
                      T.phr_mode_unsupported_reason(T.PhrMode.EXTENDED))
        self.assertIn("RANGING bit",
                      T.phr_mode_unsupported_reason(T.PhrMode.NONE))
        self.assertIn("2 octet",
                      T.phr_mode_unsupported_reason(T.PhrMode.STANDARD))
        # standard is the only one that passes.
        c = _minimal()
        self.assertIs(c.frame.phr_mode, T.PhrMode.STANDARD)
        self.assertTrue(T.validate(c).ok())

    def test_evidence_is_recorded_on_every_measured_row(self):
        """R7: the 48 rows establish work_decode_verified and NOTHING above.

        A row that claimed toa_verified would let a decode-verified length be
        used to produce a range, which is precisely the overstatement the M0
        whitelist carried.  Ranging accuracy is a function of first-path ToA,
        and nothing measured it.
        """
        caps = T.capabilities()
        self.assertEqual(len(caps.phy_matrix), 48)
        not_established = ("native_roundtrip_verified;toa_verified;"
                           "hardware_verified;vendor_interop_verified")
        for row in caps.phy_matrix:
            self.assertIs(row.evidence_level, T.EvidenceLevel.WORK_DECODE_VERIFIED)
            self.assertEqual(row.evidence_source,
                             "work_direct_998p4_modulate_loopback_demodulate_fcs")
            self.assertEqual(row.evidence_not_established, not_established)
        self.assertEqual(T.MEASURED_ROW_EVIDENCE_LEVEL,
                         T.EvidenceLevel.WORK_DECODE_VERIFIED)
        self.assertIn("no DW1000 / DW3000", T.MEASURED_ROW_EVIDENCE_STATEMENT)
        self.assertIn("nothing was measured for first-path / ToA accuracy",
                      T.MEASURED_ROW_EVIDENCE_STATEMENT)
        # The ladder itself: monotonic, cumulative, and fail-closed.
        self.assertEqual([str(x) for x in T.EVIDENCE_LEVEL_ORDER],
                         ["none", "work_decode_verified",
                          "native_roundtrip_verified", "toa_verified",
                          "hardware_verified", "vendor_interop_verified"])
        for a in T.EVIDENCE_LEVEL_ORDER:
            for b in T.EVIDENCE_LEVEL_ORDER:
                self.assertEqual(T.evidence_level_implies(a, b),
                                 T.evidence_level_index(a)
                                 >= T.evidence_level_index(b))
        # A decode-verified row may NOT produce a range, or run on a radio, or
        # talk to a vendor module.
        for use in (T.EvidenceUse.RANGING, T.EvidenceUse.HARDWARE,
                    T.EvidenceUse.VENDOR_INTEROP,
                    T.EvidenceUse.NATIVE_ROUNDTRIP):
            self.assertFalse(
                T.evidence_allows(T.EvidenceLevel.WORK_DECODE_VERIFIED, use),
                "work-decode must not satisfy %s" % use)
            self.assertIn("evidence_insufficient", T.evidence_explain(
                T.EvidenceLevel.WORK_DECODE_VERIFIED, use))
            self.assertIn(str(T.evidence_required_level(use)),
                          T.evidence_explain(T.EvidenceLevel.WORK_DECODE_VERIFIED,
                                             use))
        self.assertTrue(T.evidence_allows(T.EvidenceLevel.WORK_DECODE_VERIFIED,
                                          T.EvidenceUse.WORK_DECODE))
        self.assertIn("evidence_ok", T.evidence_explain(
            T.EvidenceLevel.TOA_VERIFIED, T.EvidenceUse.RANGING))
        self.assertEqual(
            T.not_established_list(T.EvidenceLevel.WORK_DECODE_VERIFIED),
            not_established)
        self.assertEqual(T.not_established_list(T.EvidenceLevel.VENDOR_INTEROP_VERIFIED),
                         "")
        # The lookup carries the evidence with the answer.
        look = caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 64, T.SfdMode.R4Z2,
                               32, True)
        self.assertTrue(look.allowed)
        self.assertIs(look.established, T.EvidenceLevel.WORK_DECODE_VERIFIED)
        self.assertEqual(look.evidence_not_established, not_established)
        # And a rejected length says what is missing rather than "no".
        miss = caps.lookup_phy(T.NATIVE_RATE_UC200_HZ, 9, 128, T.SfdMode.R4Z2,
                               32, True)
        self.assertFalse(miss.allowed)
        self.assertTrue(miss.evidence_not_established)

    def test_unsigned_fields_refuse_out_of_range_instead_of_wrapping(self):
        """A 16-bit wire field is 16 bits on BOTH sides of the language line.

        This test used to assert the OPPOSITE: that the reader WRAPS a JSON
        number outside the field's width, "mirroring" the C++
        ``static_cast<uintN_t>(get_i64(...))``.  A cast is not a specification.
        ``channel: 261`` silently becoming channel 5, ``session_id: 2**32 + 1``
        becoming session 1 and ``local_address: -1`` becoming the reserved
        address 0xffff are precisely the silent fallback the contract forbids:
        two different documents must not produce one runtime configuration, and
        the value the document carried must be refused rather than folded.

        The second case in each pair is the one a mask would ACCEPT by landing
        back inside the range, which is why "one past the top" alone is not
        enough coverage.
        """
        cases = (
            ("session.local_address", 0x10000),
            ("session.local_address", 0x10000 + 1),
            ("session.local_address", -1),
            ("session.pan_id", 0x10000),
            ("session.session_id", 0x100000000),
            ("session.session_id", 0x100000000 + 1),
            ("session.session_id", -1),
            ("phy.channel", 256),
            ("phy.channel", 256 + 5),
            ("phy.channel", -1),
            ("phy.preamble_symbols", 0x10000),
            ("frame.mac_psdu_bytes", 0x10000),
        )
        for path, value in cases:
            d = T.to_json_dict(_minimal())
            grp, leaf = path.rsplit(".", 1)
            d[grp][leaf] = value
            got, rep = T.from_json_dict(d)
            p = "%s.%s" % (grp, leaf)
            self.assertFalse(rep.ok(), "%s = %r was ACCEPTED" % (path, value))
            v = rep.find(p, T.ConfigReason.OUT_OF_RANGE)
            self.assertIsNotNone(
                v, "%s = %r must fail with out_of_range: %s"
                % (path, value, rep.to_string()))
            self.assertTrue("not masked" in v.message
                            or "never wrapped" in v.message,
                            "%s = %r: refusal must say it did not fold the "
                            "value: %s" % (path, value, v.message))
        # (The reader stores 0 on refusal, so "did it store the folded value"
        # is checked below against the specific foldings that USED to happen;
        # comparing against an arbitrary mask here would pass trivially
        # whenever the folded value also happens to be 0.)
        # The four inputs that used to walk straight through are now refused,
        # at the reader, before the validator ever sees them.
        for path, value, wrapped in (
                ("phy.channel", 261, 5),
                ("session.session_id", 2 ** 32 + 1, 1),
                ("session.local_address", 65537, 1),
                ("session.local_address", -1, 0xFFFF)):
            d = T.to_json_dict(_minimal())
            grp, leaf = path.rsplit(".", 1)
            d[grp][leaf] = value
            got, rep = T.from_json_dict(d)
            self.assertFalse(rep.ok(), "%s = %r was ACCEPTED" % (path, value))
            self.assertIsNotNone(
                rep.find("%s.%s" % (grp, leaf), T.ConfigReason.OUT_OF_RANGE),
                "%s = %r: %s" % (path, value, rep.to_string()))
            self.assertNotEqual(getattr_path(got, path), wrapped,
                                "%s = %r wrapped to %d"
                                % (path, value, wrapped))
        # And a literal outside int64 is refused outright, not wrapped.
        d = T.to_json_dict(_minimal())
        d["session"]["session_id"] = 2 ** 70
        _c, rep3 = T.from_json_dict(d)
        self.assertFalse(rep3.ok())

    def test_an_unknown_enum_leaves_the_zero_member_and_records_the_error(self):
        """The reader's failure behaviour is part of the contract.

        C++ ``get_enum`` returns ``static_cast<E>(0)`` on an absent key, a
        wrong type or an unknown spelling, and always records a violation.  The
        Python side must do the same or the two implementations would report
        different downstream violations for the same document -- and the parity
        corpus compares exactly those.
        """
        for cls in (T.Protocol, T.Role, T.SfdMode, T.PhrMode, T.PhrRate,
                    T.FrameProfileId, T.StsMode,
                    T.FcsAppender, T.TxPowerPolicy, T.PulseShaping, T.AgcMode,
                    T.FirstPathAlgorithm, T.CompensationFlag,
                    T.TimeReferenceEvent, T.TimeDomain, T.TimeUnit):
            zero = next(iter(cls))
            for value in ("nope", 7, None):
                d = T.to_json_dict(_minimal())
                _poke(d, cls, value)
                cfg, rep = T.from_json_dict(d)
                self.assertFalse(rep.ok(), "%s <- %r" % (cls.__name__, value))
                self.assertTrue(any(v.field.endswith("." + _key_of(cls))
                                    for v in rep.violations),
                                "%s <- %r produced %s"
                                % (cls.__name__, value, rep.to_string()))
                # Read back the field that was actually poked.  For the two
                # timed-field enums the key lives INSIDE a timed object
                # (timeouts.rx_timeout.reference), so the CONFIG path differs
                # from the document path -- reading the object itself would
                # compare an unrelated field.
                self.assertIs(getattr_path(cfg, _read_path_of(cls)), zero,
                              "%s <- %r" % (cls.__name__, value))


    def test_the_gates_a_json_document_cannot_reach(self):
        """The mirror of the C++ test of the same name.

        Two of the new checks are unreachable through the JSON path, and it is
        worth saying so rather than letting the corpus look like it covers
        them:

        * ``frame.frame_profile`` is an enum, so a NAME this build has no
          layout for is refused by the READER (unknown_enum_value) and never
          reaches the validator.  The validator's own gate is only reachable
          for a config built in code with an out-of-enum value -- here, a value
          cast in from an int, which no JSON reader would produce.
        * ``phy.preamble_symbols`` is a raw integer, so its PreambleLength gate
          IS reachable from JSON (the corpus's ``preamble_*`` cases prove it),
          but the PreambleLength ENUM itself only exists in code.
        """
        c = _minimal()
        c.frame.frame_profile = 9          # what a bad cast would leave behind
        rep = T.validate(c)
        self.assertFalse(rep.ok())
        v = self.assert_has(rep, "frame.frame_profile",
                            T.ConfigReason.UNSUPPORTED)
        self.assertIs(v.status, T.ExchangeStatus.UNSUPPORTED)
        self.assertEqual(v.requirement, "REQ-PROTO-06")
        self.assertIn("not implemented", v.message)
        self.assertFalse(T.frame_profile_id_is_supported(9))
        self.assertIsNone(T.frame_geometry_for(9))
        # And the same gate reached the ordinary way is the READER's job.
        doc = T.to_json_dict(_minimal())
        doc["frame"]["frame_profile"] = "frame_v2"
        _cfg, imp = T.from_json_dict(doc)
        self.assertFalse(imp.ok())
        self.assert_has(imp, "frame.frame_profile",
                        T.ConfigReason.UNKNOWN_ENUM_VALUE)

    def test_the_preamble_length_reasons_never_claim_a_hardware_limit(self):
        """R7: "our decoder can't" must never harden into "the chip can't"."""
        for n in (1, 2, 4, 8, 32, 100, 128, 256, 512, 1024, 2048, 4096, 65):
            why = T.preamble_length_unsupported_reason(n)
            self.assertIn(str(n), why)
            self.assertNotIn("structurally impossible", why)
            self.assertNotIn("the hardware cannot", why)
            self.assertNotIn("the chip cannot", why)
        # The lengths Qorvo's own API accepts carry the citation in the NOTE
        # that accompanies a whitelist refusal.  1024 and 2048 say "a software
        # limit, not a hardware one" in the reason itself; 128/256/512 name
        # Qorvo there because their reason discusses the PHR self-description
        # the vendor API also exposes.
        for n in (128, 256, 512):
            self.assertIn("Qorvo", T.preamble_length_unsupported_reason(n))
        for n in (1024, 2048):
            self.assertIn("not a hardware one",
                          T.preamble_length_unsupported_reason(n)
                          + T.preamble_length_unsupported_reason(
                              1024 if n == 2048 else 2048))
        for n in (128, 256, 512, 1024, 2048):
            note = T.preamble_length_reject_note(n)
            self.assertTrue(T.preamble_length_supported_by_vendor(n))
            self.assertIn(T.reject_scope_current_software(), note)
            self.assertIn("not_a_hardware_claim_", note)
            self.assertIn(T.vendor_preamble_length_citation(), note)
        self.assertNotIn("_not_a_hardware_claim_",
                         T.preamble_length_reject_note(16))


def caps_data_rates():
    return list(T.capabilities().data_rates)


def getattr_path(obj, path):
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


# Where each enum lives in the document, so the "unknown enum" test can point
# at a real key instead of maintaining a second table.
_ENUM_SITES = {
    T.Protocol: ("session", "protocol"),
    T.Role: ("session", "role"),
    T.SfdMode: ("frame", "sfd_mode"),
    T.PhrMode: ("frame", "phr_mode"),
    T.PhrRate: ("phy", "phr_rate"),
    T.PreambleLength: (None, "preamble_symbols"),
    T.FrameProfileId: ("frame", "frame_profile"),
    T.StsMode: ("frame", "sts_mode"),
    T.FcsAppender: ("frame", "fcs_append"),
    T.TxPowerPolicy: ("tx", "power_policy"),
    T.PulseShaping: ("tx", "pulse_shaping"),
    T.AgcMode: ("rx", "agc"),
    T.FirstPathAlgorithm: ("calibration", "first_path_algorithm"),
    T.CompensationFlag: ("calibration", "cfo_compensation"),
    # The two timed-field enums live INSIDE a timed object, so the key to poke
    # is "domain" / "reference" under timeouts.rx_timeout, not the object
    # itself.  _group_of is the WRITE site (and may be dotted); _path_of is the
    # READ site.
    T.TimeReferenceEvent: ("timeouts.rx_timeout", "reference"),
    T.TimeDomain: ("timeouts.rx_timeout", "domain"),
    T.TimeUnit: ("calibration", "link_delay_unit"),
}


def _group_of(cls):
    """The document path of the OBJECT the enum key lives in (may be dotted)."""
    return _ENUM_SITES[cls][0]


def _poke(doc, cls, value):
    group, key = _ENUM_SITES[cls]
    node = doc
    for part in group.split("."):
        node = node[part]
    node[key] = value


def _key_of(cls):
    return _ENUM_SITES[cls][1]


def _read_path_of(cls):
    """Where the parsed enum value lands on the CONFIG object.

    Same dotted path as the document for every entry: the two timed-field
    enums live inside a TimedField, whose attributes are named exactly as the
    JSON keys are.  ``PreambleLength`` is the exception and has no read site at
    all -- ``phy.preamble_symbols`` is an int, not an enum.
    """
    group, key = _ENUM_SITES[cls]
    if group is None:
        raise KeyError(cls)
    return "%s.%s" % (group, key)


# ==========================================================================
# N. THE DIFFERENTIAL CORPUS (M0.1, plan 2.A item 4)
# ==========================================================================
#
# C++ is the runtime authority of record.  A retained pure-Python validator is
# only defensible if it is pinned to the SAME profile data and compared ITEM BY
# ITEM against the SAME JSON corpus: accepted/rejected, the reason and the
# effective values.  Comparing enums, or grepping source strings, does not
# count -- and neither does "the Python suite is green".
#
# The corpus is testdata/twr/config_parity_corpus.json, read by this class AND
# by gr-uwb/lib/qa_uwb_twr_config_parity.cc.  The two suites therefore pin both
# implementations to one file: a case cannot be quietly dropped from one side,
# and a change in either implementation has to be made in both or the corpus
# has to be re-derived from the C++ side deliberately.

#: Every case the corpus MUST contain, by name, with the requirement it pins.
#: A dropped or renamed case is a FAILURE, not a smaller corpus: a parity test
#: that silently lost a dimension is worse than no parity test, because it
#: looks like coverage.
REQUIRED_CORPUS_CASES = {
    # valid: SS/DS x both roles
    "valid_ss_initiator": "the phase-1 common profile, accepted",
    "valid_ss_responder": "SS responder owns the reply delay",
    "valid_ds_initiator": "DS initiator owns a Final delay and RX window",
    "valid_ds_responder": "DS responder computes the range",
    # every enum member
    "valid_sfd_4z1": "SfdMode member 4z1",
    "valid_sfd_4z2": "SfdMode member 4z2",
    "valid_sfd_4z3": "SfdMode member 4z3",
    "valid_sfd_4z4": "SfdMode member 4z4",
    "valid_sfd_decawave": "SfdMode member decawave",
    "valid_sfd_ieee": "SfdMode member ieee",
    "valid_sync_16": "PreambleLength member 16, admitted and self-describing",
    "valid_phr_850k_with_payload_6p8m": "PhrRate member 850k, the pair M0 rejected",
    "frame_profile_frame_v2_is_refused_by_the_reader":
        "FrameProfileId: an unknown layout is refused",
    # out-of-enum and mistyped values
    "phr_rate_payload_name_is_refused_by_the_reader":
        "a payload-rate spelling is not a PHR rate",
    "phr_rate_850k_spelling_is_refused_by_the_reader": "strict spelling",
    "phr_rate_wrong_type_is_refused_by_the_reader": "wrong JSON type",
    "frame_profile_spelling_is_refused_by_the_reader": "strict spelling",
    "frame_profile_empty_is_refused_by_the_reader": "an empty name is not a default",
    "frame_profile_missing_key_is_refused_on_import": "twr-config/2 requires the key",
    "frame_profile_wrong_type_is_refused_by_the_reader": "wrong JSON type",
    "phr_mode_unknown_is_refused_by_the_reader": "out-of-enum PHR form",
    "sfd_mode_unknown_is_refused_by_the_reader": "out-of-enum SFD mode",
    "fraction_where_an_integer_is_required_is_refused_by_the_reader":
        "a decimal fraction is never truncated into an integer field",
    "integer_precision_loss_is_refused_on_import": "a quoted non-int64",
    "string_where_an_integer_is_required_is_refused_on_import":
        "a non-numeric decimal string",
    "unknown_key_is_refused_by_the_reader": "an unknown key is not ignored",
    "missing_phy_key_is_refused_on_import": "every key of a present group is required",
    # the schema version
    "schema_v1_document_is_rejected": "a v1 document is refused, not misread",
    "schema_unknown_version_is_rejected": "an unknown version is refused",
    "schema_empty_is_rejected": "an empty version is a missing value",
    # PhrRate
    "phr_rate_same_as_data_is_rejected": "the unimplemented PHR rate is refused",
    # PreambleLength
    "preamble_1_is_rejected": "PreambleLength member rejection, 1 SYNC",
    "preamble_2_is_rejected": "PreambleLength member rejection, 2 SYNC",
    "preamble_4_is_rejected": "PreambleLength member rejection, 4 SYNC",
    "preamble_8_is_rejected": "PreambleLength member rejection, 8 SYNC",
    "preamble_32_is_rejected": "PreambleLength member rejection, 32 SYNC",
    "preamble_128_is_rejected": "PreambleLength member rejection, 128 SYNC",
    "preamble_256_is_rejected": "PreambleLength member rejection, 256 SYNC",
    "preamble_512_is_rejected": "PreambleLength member rejection, 512 SYNC",
    "preamble_1024_is_rejected": "PreambleLength member rejection, 1024 SYNC",
    "preamble_2048_is_rejected": "PreambleLength member rejection, 2048 SYNC",
    "preamble_65_is_rejected": "a length that is not a member",
    "preamble_100_is_rejected": "a length outside the modulator API list",
    "preamble_4096_is_rejected": "a legal 802.15.4a duration this build cannot run",
    "preamble_zero_is_a_missing_value": "0 is not stated, not refused",
    # PhrMode
    "phr_mode_extended_is_rejected": "the extended PHR layout is refused",
    "phr_mode_none_is_rejected": "a frame without a PHR is refused",
    # the geometry claim
    "geometry_pre_m0_7_byte_header_is_rejected": "the M0 7-byte header claim",
    "geometry_16_byte_header_is_rejected": "a larger header claim is refused too",
    "geometry_16_byte_geometry_is_rejected": "the other pre-codec geometry",
    "geometry_every_field_disagrees": "five per-field rejections, one per field",
    "geometry_timestamp_width_disagrees": "the timestamp width is a profile value",
    "geometry_not_stated_is_rejected": "an all-zero claim is a missing value",
    "geometry_missing_object_is_refused_on_import": "the geometry object absent",
    "geometry_mac_fcs_reserved_while_phy_appends_is_rejected":
        "reserving FCS bytes the PHY appends",
    "mac_appends_fcs_is_describable_but_not_encodable":
        "the MAC-appends layout is describable, not encodable",
    "fcs_bytes_four_is_rejected": "the FCS size is the codec's",
    "fcs_append_none_is_rejected": "some layer must append the FCS",
    "mac_psdu_includes_fcs_conflicts_with_fcs_append": "the two FCS fields must agree",
    # session ids
    "valid_session_id_1": "session id 1 is legal",
    "valid_session_id_max": "session id 0xFFFF is legal",
    "session_id_zero_is_the_no_session_marker": "0 is the no-session marker",
    "session_id_0x10000_is_refused_not_truncated": "one past the wire field",
    "session_id_uint32_max_is_refused_not_truncated": "the widest local id",
    # the 127-byte boundary
    "valid_psdu_127_boundary": "127 is the measured maximum",
    "valid_psdu_29_minimum": "29 is exactly the Final's MAC payload",
    "mac_psdu_128_is_one_over_the_maximum": "128 is one over",
    "mac_psdu_28_cannot_hold_the_final": "28 cannot hold the Final",
    "application_payload_larger_than_the_psdu": "the payload is a separate concept",
    # channel / frequency
    "channel_9_is_not_a_phase1_claim": "only channel 5 is a phase-1 claim",
    "channel_frequency_conflict": "channel and frequency must agree",
    "channel_17_is_out_of_range": "the channel plan is 0..16",
    "readback_rate_mismatch_is_a_startup_failure": "a readback mismatch is fatal",
    # timing precision, including the 491.52 MS/s 2.0345 ns tick
    "valid_cg400_two_ns_budget": "the CG400 tick is met with a 2 ns budget",
    "cg400_one_ns_budget_is_unreachable": "and is NOT met with a 1 ns budget",
    "zero_nanosecond_budget_is_insufficient": "a 0 ns budget is unreachable",
    "quantisation_rate_this_build_cannot_express": "2 GHz is not a rate we have",
    "negative_quantisation_budget_is_refused": "a negative budget is refused",
    "valid_work_rate_quantisation": "the 998.4 MS/s work rate is expressible",
    "reply_delay_shorter_than_the_tx_lead_time": "REQ-GR-04 feasibility",
    "rx_window_longer_than_the_rx_timeout": "the window must fit the timeout",
    "negative_delay_is_never_a_delay": "a delay may not be negative",
    # calibration present / expired / mismatched
    "calibration_id_missing": "an uncalibrated range may not be claimed",
    "calibration_already_applied": "a calibration applies exactly once",
    "calibration_channel_mismatch": "the record names another channel",
    "calibration_profile_version_mismatch": "the record names another profile",
    "calibration_gain_mismatch": "the record was taken at another TX gain",
    "calibration_record_rate_mismatch": "the record was taken at another rate",
    "calibration_link_delay_unit_seconds": "ns, s and native ticks stay separate",
    "calibration_meta_version_does_not_name_the_loaded_calibration":
        "meta and calibration must name the same record",
    # STS non-off
    "sts_sp64_is_out_of_scope": "StsMode member sp64 is out of scope",
    "sts_sp128_is_out_of_scope": "StsMode member sp128 is out of scope",
    "sts_sp256_is_out_of_scope": "StsMode member sp256 is out of scope",
    "sts_sp512_is_out_of_scope": "StsMode member sp512 is out of scope",
    "sts_sp1024_is_out_of_scope": "StsMode member sp1024 is out of scope",
    "sts_off_with_a_length_requested": "off plus a length is a conflict",
    # the remaining measured axes
    "data_rate_850k_is_refused": "the payload axis is its own whitelist",
    "data_rate_27m_is_refused": "and so is 27 Mb/s",
    "prf_class_hprf64_is_refused": "the HPRF classes are a different geometry",
    "code_8_is_outside_the_api_range": "codes are 9..12",
    "code_13_is_outside_the_api_range": "and 13 is out",
    "tx_and_rx_codes_differ": "one profile uses one code",
    "peer_claims_the_same_role": "one initiator and one responder",
    "peer_claims_the_same_physical_channel": "one device resource, one owner",
    "peer_shares_a_different_native_rate": "endpoints on one device share a rate",
    "peer_id_is_required": "a peer with no id cannot be told apart",
    "sequence_modulus_is_not_a_power_of_two_choice": "the wrap modulus is 4/16/64/256",
    "sequence_does_not_fit_the_modulus": "a sequence above the modulus is an error",
    "retry_without_a_backoff": "retries need a non-zero backoff",
    "diagnostic_budget_without_the_diagnostic": "budget and enable must agree",
    "diagnostic_buffer_above_the_bound": "diagnostic memory is bounded",
    "diagnostic_io_on_the_realtime_thread": "no diagnostic I/O on the RT thread",
    "device_tick_field_without_a_marker": "a device tick names its RF marker",
    "host_monotonic_field_with_a_marker": "a host monotonic field has no marker",
    "host_monotonic_field_requantised": "a host field is quantised at 1e9",
    "timed_field_reference_is_the_wrong_event": "the reference event is named",
    "frame_tail_measured_at_the_first_rx_iq_sample": "the first sample is not the tail",
    "exchange_timeout_shorter_than_the_chain": "the deadline must cover the chain",
    "measurement_interval_shorter_than_one_exchange": "measurements may not overlap",
    "power_policy_names_a_field_that_is_absent": "the three power concepts",
    "power_policy_untouched_with_a_value_set": "and who authorises the power",
    "calibrated_dbm_without_the_gain_it_was_measured_at": "dBm needs its gain",
    "vendor_pac_value_without_a_backend": "a PAC value needs a backend",
    "iq_amplitude_above_full_scale": "IQ amplitude is (0, 1]",
    "rx_correlation_gate_looser_than_the_detection_gate": "the fine gate must bite",
    "first_path_index_outside_the_search_window": "the index is inside the window",
    "ranging_bit_false_is_not_a_twr_profile": "REQ-PROTO-05",
    "report_window_is_reserved": "Report is reserved",
    "report_delay_is_reserved": "and so is the Report delay",
    "two_in_flight_exchanges_are_refused": "one exchange in flight, per endpoint",
    "local_and_peer_address_identical": "identical addresses match everything",
    "negative_local_address_is_refused_not_wrapped": "a negative uint16 is refused, not wrapped",
    # N01: the reader must range-check before narrowing.  A value one modulus
    # past a legal one is the case a mask accepts by landing back in range.
    "channel_uint8_max_plus_one_is_refused": "uint8 overflow is refused",
    "channel_one_modulus_past_a_legal_value_is_refused":
        "one modulus past a legal channel still folds nowhere",
    "session_id_one_modulus_past_uint32_is_refused": "uint32 overflow is refused",
    "local_address_one_past_uint16_is_refused": "uint16 overflow is refused",
    "pan_id_one_past_uint16_is_refused": "the PAN field has the same rule",
    "sequence_uint8_max_plus_one_is_refused": "and so does the sequence",
    "geometry_mac_header_bytes_out_of_uint16_is_refused":
        "geometry is checked, but only after the value fits its field",
    "negative_tx_port_is_refused_not_wrapped": "unsigned means unsigned",
    "sequence_uint8_max_is_accepted_at_the_boundary":
        "the bound is inclusive, not exclusive",
    # non-finite
    "non_finite_phy_center_frequency_hz": "NaN in the centre frequency",
    "non_finite_radio_native_sample_rate_hz": "NaN in the native sample rate",
    "non_finite_rx_detection_threshold": "NaN in the detection gate",
    "non_finite_calibration_native_sample_rate_hz": "NaN in the calibration rate",
    "non_finite_timed_quantisation": "NaN in a quantisation rate",
    "non_finite_several_fields_at_once": "every field is reported, not the first",
    "non_finite_record_rate_is_a_known_gap": "a known C++ gap, pinned out loud",
}

#: The float fields the exhaustive non-finite sweep covers.  Mirrors the
#: C++ kFloatFields table; the two must stay in step or one side sweeps a field
#: the other does not.
CORPUS_FLOAT_FIELDS = 32


class TestConfigParityCorpus(TwrConfigTestBase):
    """The Python half of the Python/C++ differential test."""

    @classmethod
    def setUpClass(cls):
        cls.corpus = T.load_parity_corpus()
        cls.by_name = {c["name"]: c for c in cls.corpus["cases"]}

    def test_the_corpus_is_present_and_targets_this_schema(self):
        path = T.default_parity_corpus_path()
        self.assertTrue(os.path.isfile(path),
                        "the shared corpus is missing: %s" % path)
        corpus = T.load_parity_corpus()
        self.assertEqual(corpus["corpus_version"], T.CORPUS_VERSION)
        self.assertEqual(corpus["config_schema_version"], "twr-config/2")
        self.assertEqual(corpus["config_schema_version"], T.SCHEMA_VERSION)
        self.assertTrue(corpus["note"].strip())
        self.assertEqual(sorted(corpus["base"]),
                         ["ds_initiator", "ds_responder", "ss_initiator",
                          "ss_responder"])
        self.assertEqual(len(corpus["non_finite_fields"]), CORPUS_FLOAT_FIELDS)
        self.assertTrue(len(corpus["cases"]) >= 100,
                        "the corpus must cover the whole surface, not a sample")

    def test_every_required_case_is_present(self):
        missing = sorted(set(REQUIRED_CORPUS_CASES) - set(self.by_name))
        self.assertEqual(missing, [],
                         "corpus cases the plan REQUIRES are missing; a parity "
                         "test that silently lost a dimension looks like coverage")
        for name, why in sorted(REQUIRED_CORPUS_CASES.items()):
            case = self.by_name[name]
            self.assertTrue(case.get("note", "").strip(),
                            "case %r (%s) states no note" % (name, why))
            self.assertIn("base", case)
            self.assertIn(case["base"], self.corpus["base"])
            self.assertIn("expect", case)
            self.assertIn("accepted", case["expect"])
            self.assertIn("violations", case["expect"])

    def test_the_corpus_covers_every_required_dimension(self):
        """A coverage claim has to be checkable, not asserted in prose."""
        seen = {c["name"] for c in self.corpus["cases"]}
        # Each protocol/role corner is an accepted case.
        accepted = {c["name"] for c in self.corpus["cases"]
                    if c["expect"]["accepted"]}
        for corner in ("ss_initiator", "ss_responder", "ds_initiator",
                       "ds_responder"):
            self.assertIn("valid_%s" % corner, accepted)
        # Every enum member of the five axes the plan names, and every value
        # outside them.
        self.assertTrue({"valid_sfd_4z1", "valid_sfd_4z2", "valid_sfd_4z3",
                         "valid_sfd_4z4", "valid_sfd_decawave",
                         "valid_sfd_ieee"} <= accepted)
        self.assertIn("valid_sync_16", accepted)
        self.assertIn("valid_phr_850k_with_payload_6p8m", accepted)
        for name in ("frame_profile_frame_v2_is_refused_by_the_reader",
                     "sfd_mode_unknown_is_refused_by_the_reader",
                     "phr_mode_unknown_is_refused_by_the_reader",
                     "phr_rate_payload_name_is_refused_by_the_reader",
                     "phr_rate_850k_spelling_is_refused_by_the_reader"):
            self.assertIn(name, seen)
        # Session ids at 0 / 1 / 0xFFFF / 0x10000 / UINT32_MAX.
        for name in ("session_id_zero_is_the_no_session_marker",
                     "valid_session_id_1", "valid_session_id_max",
                     "session_id_0x10000_is_refused_not_truncated",
                     "session_id_uint32_max_is_refused_not_truncated"):
            self.assertIn(name, seen)
        # The 7-byte and 16-byte geometry claims.
        self.assertIn("geometry_pre_m0_7_byte_header_is_rejected", seen)
        self.assertIn("geometry_16_byte_header_is_rejected", seen)
        self.assertIn("geometry_16_byte_geometry_is_rejected", seen)
        # 127-byte boundary, both sides.
        self.assertIn("valid_psdu_127_boundary", accepted)
        self.assertIn("mac_psdu_128_is_one_over_the_maximum", seen)
        # Timing precision, including the 491.52 MS/s 2.0345 ns tick both ways.
        self.assertIn("valid_cg400_two_ns_budget", accepted)
        self.assertIn("cg400_one_ns_budget_is_unreachable", seen)
        # Calibration present / expired / mismatched.
        for name in ("calibration_id_missing", "calibration_already_applied",
                     "calibration_channel_mismatch",
                     "calibration_profile_version_mismatch",
                     "calibration_gain_mismatch",
                     "calibration_record_rate_mismatch",
                     "calibration_link_delay_unit_seconds"):
            self.assertIn(name, seen)
        # STS: every non-off member.
        for name in ("sts_sp64_is_out_of_scope", "sts_sp128_is_out_of_scope",
                     "sts_sp256_is_out_of_scope", "sts_sp512_is_out_of_scope",
                     "sts_sp1024_is_out_of_scope"):
            self.assertIn(name, seen)
        # NaN in every float field: the corpus names the representative ones
        # and the sweep covers all of them (below).
        self.assertGreaterEqual(
            len([n for n in seen if n.startswith("non_finite_")]), 6)

    def test_every_corpus_case_matches_the_expectations(self):
        """THE DIFFERENTIAL TEST, Python side.

        For every case in the shared corpus: import it, validate it, and compare
        the import report, the accept/reject verdict, the ORDERED list of
        (field, ConfigReason, ExchangeStatus) triples, the frozen effective
        values and the config hash against what the C++ authority recorded.

        The config hash is the strongest check in the suite: it is an fnv1a64
        over the canonical listing of EVERY field, so one wrong spelling, one
        missing entry or one field in a different order changes it.  A Python
        and a C++ hash that agree mean the two implementations agree on every
        field name, every value rendering and the order of all of them.
        """
        failures = []
        for case in self.corpus["cases"]:
            name = case["name"]
            base = self.corpus["base"][case["base"]]
            got = T.run_parity_case(case, base)
            want = case["expect"]
            problems = []
            if got["import_ok"] != want["import_ok"]:
                problems.append("import_ok: corpus %r, python %r"
                                % (want["import_ok"], got["import_ok"]))
            if got["import_violations"] != want["import_violations"]:
                problems.append("import_violations:\n      corpus: %s\n"
                                "      python: %s"
                                % (want["import_violations"],
                                   got["import_violations"]))
            if got["accepted"] != want["accepted"]:
                problems.append("accepted: corpus %r, python %r"
                                % (want["accepted"], got["accepted"]))
            if got["violations"] != want["violations"]:
                problems.append("violations:\n      corpus: %s\n"
                                "      python: %s"
                                % (want["violations"], got["violations"]))
            # config_hash and effective are absent for a case that is not
            # accepted: there is no effective snapshot to hash or compare.
            if "config_hash" in want and got["config_hash"] != want["config_hash"]:
                problems.append("config_hash: corpus %r, python %r. Every field "
                                "name, value rendering and the order of all of "
                                "them differ between the two implementations"
                                % (want["config_hash"], got["config_hash"]))
            for key, want_value in want.get("effective", {}).items():
                if key not in got["effective"]:
                    problems.append("effective.%s: in the corpus, not produced"
                                    % key)
                elif str(got["effective"][key]) != str(want_value):
                    problems.append("effective.%s: corpus %r, python %r"
                                    % (key, want_value,
                                       got["effective"][key]))
            if problems:
                failures.append("%s:\n    %s" % (name, "\n    ".join(problems)))
        self.assertEqual(failures, [],
                         "the Python validator disagrees with the C++ "
                         "authority on %d corpus case(s):\n%s"
                         % (len(failures), "\n".join(failures)))

    def test_the_four_base_documents_are_accepted(self):
        """Everything derived from a base is only meaningful if the base works."""
        for name in ("ss_initiator", "ss_responder", "ds_initiator",
                     "ds_responder"):
            doc = self.corpus["base"][name]
            cfg, imp = T.from_json_string(json.dumps(doc, allow_nan=False))
            self.assertTrue(imp.ok(), "%s import: %s" % (name, imp.to_string()))
            e = T.effective_config(cfg)
            self.assertTrue(e.ok, "%s: %s" % (name, e.validation.to_string()))
            # The codec's on-air budget, and the PHR of this profile.
            self.assertEqual((e.poll_bytes, e.response_bytes, e.final_bytes),
                             (16, 26, 31))
            self.assertEqual(e.phr_bytes, 2)
            self.assertEqual(e.phr_coded_bits, 19)
            self.assertEqual(e.max_psdu_bytes, 127)
            self.assertEqual(e.schema_version, "twr-config/2")

    def test_non_finite_is_refused_in_every_float_field(self):
        """The exhaustive sweep, mirroring the C++ one field for field.

        A non-finite double cannot be written in strict JSON and neither reader
        produces one, so the corpus names the fields and BOTH loaders assign the
        three non-finite values to the imported config.  That post-import
        assignment is the only way such a value can reach a validator, and doing
        it identically on both sides is what makes the case mirrored rather than
        two different tests.
        """
        base = self.corpus["base"]["ss_initiator"]
        fields = self.corpus["non_finite_fields"]
        # EMPTY, and that is the point.  It used to hold
        # "calibration.record.native_sample_rate_hz", the one field the
        # validator had no finiteness check on: its applicability rule is
        # guarded by `> 0.0` and a NaN fails that comparison, so a non-finite
        # value skipped validation entirely.  The parity corpus found it; both
        # validators now reject it.  Keep the set as a tripwire -- a new float
        # field with no check fails this sweep, which is how the omission was
        # found in the first place.
        unchecked: set = set()
        for path in fields:
            for value in (float("nan"), float("inf"), float("-inf")):
                doc = json.dumps(base, allow_nan=False)
                cfg, _imp = T.from_json_string(doc)
                T.set_config_double(cfg, path, value)
                report = T.validate(cfg)
                if path in unchecked:
                    if value != value:  # NaN
                        self.assertTrue(
                            report.ok(),
                            "%s has a finiteness check now; move it out of the "
                            "known-gap list in BOTH suites" % path)
                    continue
                self.assertFalse(report.ok(),
                                 "a non-finite %s was ACCEPTED" % path)
                v = report.find(path, T.ConfigReason.NOT_FINITE)
                self.assertIsNotNone(v, "a non-finite %s: %s"
                                     % (path, report.to_string()))
                # And it must never be exportable.
                with self.assertRaises(T.ConfigJsonError):
                    T.to_json_string(cfg)

    def test_a_case_whose_patch_does_not_land_is_an_error(self):
        """A silently-empty patch would test the BASE and report a false pass."""
        base = self.corpus["base"]["ss_initiator"]
        with self.assertRaises(KeyError):
            T.apply_corpus_patch(base, {"no_such_group.channel": 1})
        with self.assertRaises(KeyError):
            T.apply_corpus_patch(base, {"phy.no_such_key.deeper": 1})
        with self.assertRaises(KeyError):
            T.apply_corpus_patch(base, {"phy.no_such_key": None})
        with self.assertRaises(KeyError):
            T.apply_corpus_patch(base, {"radio.peers[3].tx_channel": 1})
        # A new LEAF may be added, because "this key is not in the schema" is
        # itself a case worth stating.
        doc = T.apply_corpus_patch(base, {"phy.turbo_mode": True})
        self.assertIn("turbo_mode", doc["phy"])
        # A deletion is a deletion.
        doc = T.apply_corpus_patch(base, {"frame.frame_profile": None})
        self.assertNotIn("frame_profile", doc["frame"])
        # ... and the base is never mutated.
        self.assertIn("frame_profile", base["frame"])

    def test_the_module_is_still_standalone_after_the_corpus_work(self):
        """No new dependency crept in, and nothing radio-shaped was imported."""
        self.assertNotIn("gnuradio", sys.modules)
        for name in ("numpy", "jsonschema", "uhd", "pmt", "pybind11"):
            self.assertNotIn(name, sys.modules,
                             "importing uwb.twr_config pulled in %s" % name)
        # The corpus loader is stdlib-only too: json + os, nothing else.
        src = _read(os.path.join(_PYTHON_DIR, "uwb", "twr_config.py"))
        self.assertIn("import json", src)
        self.assertIn("import os", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
