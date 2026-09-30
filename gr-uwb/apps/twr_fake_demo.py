#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Driver for the M1-B two-endpoint TWR protocol demo (``twr_fake_demo``).

WHAT THIS IS
------------
A thin, pure-Python (stdlib only) front end that

  1. builds a *request* document in the small, versioned
     ``twr-m1b-demo/1`` envelope;
  2. optionally attaches a COMPLETE requested ``twr_config`` built with the
     EXISTING :mod:`uwb.twr_config` module (R09: the Python side submits the
     requested configuration through the one config module -- one validator,
     one schema -- never a parallel definition);
  3. runs the compiled pure-C++ demo binary on it; and
  4. consumes the structured result, including the
     requested -> effective -> core field mapping the C++ side prints.

It contains NO state machine, NO ToF arithmetic and NO capability logic.  All
of that lives in the C++ core and the existing config module; the Python side
only states a scenario/configuration and reads the answer back.  That is the
M1-B rule (instruction section 6): "Python ... only calls the CLI and consumes
the output".

THE ENVELOPE IS ALWAYS MARKED AS SIMULATION
-------------------------------------------
Every request this module emits carries

    simulation.execution = "offline_simulation"
    simulation.mode      = "protocol_estimate"

and the C++ side REFUSES to run a request that does not.  A result is a
protocol estimate over wire claims, NOT a validated measurement and NOT a
hardware ranging result; ``measurement_valid`` is always false and no distance
is ever an accepted measurement.

CLI
---
    python3 twr_fake_demo.py --scenario ss_unity
    python3 twr_fake_demo.py --scenario ds_nominal --json
    python3 twr_fake_demo.py --config my_twr_config.json --protocol ss

The C++ binary is located via, in order: ``--demo``, ``$UWB_TWR_FAKE_DEMO``,
``<repo>/gr-uwb/build/twr_fake_demo`` and a couple of obvious fallbacks; the
chosen path is printed to stderr.  The default is
``gr-uwb/build/twr_fake_demo``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

SCHEMA = "twr-m1b-demo/1"

_HERE = os.path.dirname(os.path.abspath(__file__))          # gr-uwb/apps
_GR_UWB = os.path.dirname(_HERE)                             # gr-uwb
_REPO = os.path.dirname(_GR_UWB)                             # repository root

# --------------------------------------------------------------------------
# The EXISTING typed config module.  R09 requires the Python side to submit a
# requested config through the one config module rather than a parallel schema;
# it is pure stdlib, so importing it here keeps this driver free of GNU Radio.
# --------------------------------------------------------------------------
_PYTHON_DIR = os.path.join(_GR_UWB, "python")
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)
try:
    import uwb.twr_config as T  # noqa: E402
    TWR_CONFIG_IMPORT_ERROR = None
except Exception as _exc:  # pragma: no cover - setup guard
    T = None
    TWR_CONFIG_IMPORT_ERROR = str(_exc)

# Where the C++ demo is expected.  The first entry IS the documented default.
DEFAULT_DEMO_PATH = os.path.join(_GR_UWB, "build", "twr_fake_demo")
_FALLBACK_DEMO_PATHS = (
    DEFAULT_DEMO_PATH,
    os.path.join(_REPO, "twr_fake_demo"),
    "/tmp/opencode/twr_fake_demo",
    "/tmp/twr_fake_demo",
)


# ==========================================================================
# Binary discovery
# ==========================================================================


def find_demo_binary(explicit=None):
    """Locate the compiled demo.  Returns (path, source) or (None, message).

    ``explicit`` wins; then ``$UWB_TWR_FAKE_DEMO``; then the documented default
    and fallbacks.  The caller prints the result, so a fallback is never
    silent.
    """
    if explicit:
        if os.path.isfile(explicit) and os.access(explicit, os.X_OK):
            return explicit, "--demo"
        return None, "--demo %s does not exist or is not executable" % explicit

    env = os.environ.get("UWB_TWR_FAKE_DEMO")
    if env:
        if os.path.isfile(env) and os.access(env, os.X_OK):
            return env, "$UWB_TWR_FAKE_DEMO"
        return None, "$UWB_TWR_FAKE_DEMO=%s does not exist or is not executable" % env

    for candidate in _FALLBACK_DEMO_PATHS:
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            source = "default" if candidate == DEFAULT_DEMO_PATH else "fallback"
            return candidate, source
    return None, "not found; tried: " + ", ".join(_FALLBACK_DEMO_PATHS)


def build_hint():
    return (
        "build it with:\n"
        "  g++ -std=c++17 -Wall -Wextra -I %s \\\n"
        "      -o %s %s %s %s\n"
        % (
            os.path.join(_GR_UWB, "include"),
            DEFAULT_DEMO_PATH,
            os.path.join(_GR_UWB, "apps", "twr_fake_demo.cc"),
            os.path.join(_GR_UWB, "lib", "uwb_twr_core.cc"),
            os.path.join(_GR_UWB, "lib", "uwb_twr_fake_link.cc"),
        )
    )


# ==========================================================================
# The request envelope
# ==========================================================================

_UNITY = {
    "a": {"name": "x410_sn_dev", "tick_rate_hz": 1.0e9, "epoch_id": 1,
          "timestamp_bits": 0},
    "b": {"name": "x410_sn_dev", "tick_rate_hz": 1.0e9, "epoch_id": 1,
          "timestamp_bits": 0},
}
_NOMINAL = {
    "a": {"name": "x410_sn_dev", "tick_rate_hz": 1.0e9, "epoch_id": 1,
          "timestamp_bits": 0},
    "b": {"name": "dw3000_sn1", "tick_rate_hz": 0.8e9, "epoch_id": 1,
          "timestamp_bits": 0},
}

# distance that produces a ground-truth ToF of exactly 100 ticks at 1 GHz.
_TOF100_DISTANCE_M = 100.0 * 299792458.0 / 1.0e9

SCENARIOS = {
    "ss_unity": {
        "protocol": "ss", "domains": _UNITY,
        "clock_ratio": {"kind": "unity_same_clock"},
    },
    "ds_unity": {
        "protocol": "ds", "domains": _UNITY,
        "clock_ratio": {"kind": "unity_same_clock"},
    },
    "ss_nominal": {
        "protocol": "ss", "domains": _NOMINAL,
        "clock_ratio": {"kind": "nominal_rate_ratio", "fA_hz": 1000000000,
                        "fB_hz": 800000000},
    },
    "ds_nominal": {
        "protocol": "ds", "domains": _NOMINAL,
        "clock_ratio": {"kind": "nominal_rate_ratio", "fA_hz": 1000000000,
                        "fB_hz": 800000000},
    },
}


def scenario_names():
    return sorted(SCENARIOS)


def build_request(scenario="ss_unity", protocol=None, distance_m=None, seed=1,
                  poll_air_ticks=2000, response_reply_ticks=500,
                  final_reply_ticks=300, twr_config=None):
    """Build a ``twr-m1b-demo/1`` request.  Raises KeyError on a bad scenario.

    The envelope is ALWAYS a simulation: the two simulation keys are hard-set
    here so no caller can forget them.
    """
    spec = SCENARIOS[scenario]
    proto = protocol if protocol is not None else spec["protocol"]
    if proto not in ("ss", "ds"):
        raise ValueError("protocol must be 'ss' or 'ds'")

    request = {
        "schema": SCHEMA,
        "simulation": {
            "execution": "offline_simulation",
            "mode": "protocol_estimate",
            "seed": int(seed),
            "scenario_id": scenario,
        },
        "protocol": proto,
        "session": {
            "session_id": 7,
            "session_generation": 1,
            "sequence_modulus": 4,
            "initial_sequence": 0,
            "pan_id": 4660,
        },
        "wire": {
            "timestamp_bits": 40,
            "timestamp_unit_hz": 1000000000,
            "max_interval_ticks": 1000000,
        },
        "endpoints": {
            "a": {"local_address": 257, "peer_address": 514},
            "b": {"local_address": 514, "peer_address": 257},
        },
        "domains": json.loads(json.dumps(spec["domains"])),
        "clock_ratio": json.loads(json.dumps(spec["clock_ratio"])),
        "calibration": {
            "id": "cal1",
            "calibrated_epoch": 1,
            "valid_from_ticks": 0,
            "valid_until_ticks": 1000000000,
        },
        "link": {
            "distance_m": float(_TOF100_DISTANCE_M if distance_m is None
                                else distance_m),
            "tx_air_latency_a_ticks": 0,
            "tx_air_latency_b_ticks": 0,
        },
        "timeline": {
            "poll_air_ticks": int(poll_air_ticks),
            "response_reply_ticks": int(response_reply_ticks),
            "final_reply_ticks": int(final_reply_ticks),
        },
    }
    if twr_config is not None:
        request["twr_config"] = twr_config
    return request


def load_twr_config(path):
    """Read an existing TwrConfig JSON file.  It is passed through VERBATIM to
    the C++ side, which re-validates it with the existing parser/validator."""
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def build_twr_config(*, protocol="ss", local_address=257, peer_address=514,
                     pan_id=4660, session_id=7, exchange_id=1, sequence=0,
                     sequence_modulus=4, result_queue_capacity=256,
                     event_queue_capacity=512, exchange_timeout_ns=50_000_000,
                     rx_timeout_ns=40_000, calibration_id="cal1",
                     profile_version="demo-frame/0", label="twr_fake_demo full config"):
    """Build a COMPLETE, currently-valid TwrConfig document.

    This uses the EXISTING ``uwb.twr_config`` module -- one validator, one
    schema, one hash -- and never a parallel definition.  Every field is set
    explicitly (there is no "fill in the blanks" path): a missing value would be
    a rejection, which is the point.

    The values that the envelope ALSO states (protocol, addresses, pan,
    session_id, sequence_modulus, sequence, calibration id) are aligned with
    ``build_request``'s defaults so the two agree; a disagreement is refused by
    the C++ side rather than silently overridden.

    Raising ``RuntimeError`` when the module is unavailable keeps the failure
    explicit instead of returning a partial document.
    """
    if T is None:  # pragma: no cover - setup guard
        raise RuntimeError("cannot import uwb.twr_config from %s: %s"
                           % (_PYTHON_DIR, TWR_CONFIG_IMPORT_ERROR))

    proto = T.Protocol.SS if protocol == "ss" else T.Protocol.DS
    c = T.TwrConfig()
    c.meta.schema_version = T.capabilities().schema_version
    c.meta.profile_version = profile_version
    c.meta.calibration_version = calibration_id
    c.meta.label = label

    # ---- session ---------------------------------------------------------
    s = c.session
    s.protocol = proto
    s.role = T.Role.INITIATOR
    s.local_address = int(local_address)
    s.peer_address = int(peer_address)
    s.pan_id = int(pan_id)
    s.session_id = int(session_id)
    s.exchange_id = int(exchange_id)
    s.sequence = int(sequence)
    s.sequence_modulus = int(sequence_modulus)
    s.measurement_count = 1
    s.measurement_interval = T.Duration(20_000_000)
    s.max_attempts_per_exchange = 1
    s.retry_backoff = T.Duration(0)
    s.max_in_flight_exchanges = T.MAX_IN_FLIGHT_EXCHANGES
    s.require_pan_match = True
    s.require_address_match = True

    # ---- phy: the MEASURED whitelist row (ch5 / 64 SYNC / 4z2 / code 9) ---
    p = c.phy
    p.channel = 5
    p.center_frequency_hz = 6489.6e6
    p.tx_preamble_code = 9
    p.rx_preamble_code = 9
    p.preamble_symbols = 64
    p.prf_class = T.PrfClass.BPRF64
    p.data_rate = T.DataRate.R6P8M
    p.phr_rate = T.PhrRate.STANDARD_850K

    # ---- frame format (geometry derived from the codec authority) --------
    f = c.frame
    f.frame_profile = T.FrameProfileId.TWR_V1
    f.sfd_mode = T.SfdMode.R4Z2
    f.sfd_symbols = 8
    f.sfd_timeout = T.device_field(
        20_000, T.TimeReferenceEvent.RX_ENABLE,
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

    # ---- transmit / receive (the three power concepts stay separate) -----
    c.tx.port = 0
    c.tx.gain_db = 20.0
    c.tx.power_policy = T.TxPowerPolicy.MANUAL_GAIN_DB
    c.tx.pulse_shaping = T.PulseShaping.EXISTING_HRP
    c.rx.port = 0
    c.rx.gain_db = 30.0
    c.rx.agc = T.AgcMode.MANUAL
    c.rx.bandwidth_hz = 500.0e6
    c.rx.detection_threshold = 0.30
    c.rx.correlation_threshold = 0.35
    c.rx.first_path_threshold = 0.25
    c.rx.first_path_index = 8
    c.rx.first_path_window = 30

    # ---- radio -----------------------------------------------------------
    r = c.radio
    r.device_args = "addr=192.168.10.2"
    r.tx_channel = 0
    r.rx_channel = 0
    r.native_sample_rate_hz = T.NATIVE_RATE_UC200_HZ
    r.clock_source = "internal"
    r.time_source = "internal"
    r.peers.append(T.EndpointBinding(
        id="B", role=T.Role.RESPONDER, tx_channel=1, rx_channel=1,
        native_sample_rate_hz=T.NATIVE_RATE_UC200_HZ, occupies_resources=True))
    r.readback = T.RadioReadback(
        present=True, sample_rate_hz=T.NATIVE_RATE_UC200_HZ,
        center_freq_hz=6489.6e6, tx_channel=0, rx_channel=0, mpm_string="X410",
        fpga_image="", uhd_version="4.x", clock_source="internal",
        time_source="internal")
    r.require_readback = True

    # ---- per-message timing ---------------------------------------------
    t = c.timing
    t.poll_start = T.device_field(
        40_000_000_000, T.TimeReferenceEvent.POLL_TX_RMARKER,
        T.TimestampMarker.RMARKER_TX)
    t.poll_to_response = T.device_field(
        0, T.TimeReferenceEvent.POLL_RX_RMARKER, T.TimestampMarker.RMARKER_RX)
    t.response_to_final = T.device_field(
        600_000 if protocol == "ds" else 0,
        T.TimeReferenceEvent.RESPONSE_RX_RMARKER if protocol == "ds"
        else T.TimeReferenceEvent.FINAL_TX_RMARKER,
        T.TimestampMarker.RMARKER_RX)
    t.final_to_report = T.device_field(
        0, T.TimeReferenceEvent.REPORT_TX_RMARKER, T.TimestampMarker.RMARKER_TX)
    t.post_tx_rx_enable = T.device_field(
        2_000, T.TimeReferenceEvent.FRAME_TAIL, T.TimestampMarker.PREAMBLE_START)
    # 20 us is a QA PLACEHOLDER, not a measurement (see the config module).
    t.min_tx_lead_time = T.device_field(
        20_000, T.TimeReferenceEvent.RESPONSE_TX_RMARKER,
        T.TimestampMarker.RMARKER_TX)

    # ---- timeouts --------------------------------------------------------
    to = c.timeouts
    rx = T.TimestampMarker.UHD_RX_FIRST_IQ_SAMPLE
    to.poll_rx_window = T.device_field(0, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.response_rx_window = T.device_field(20_000, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.final_rx_window = T.device_field(
        20_000 if protocol == "ds" else 0, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.report_rx_window = T.device_field(0, T.TimeReferenceEvent.RX_ENABLE, rx)
    to.rx_timeout = T.device_field(int(rx_timeout_ns), T.TimeReferenceEvent.RX_ENABLE, rx)
    to.exchange_timeout = T.host_field(int(exchange_timeout_ns))
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
    k.calibration_id = calibration_id
    k.record = T.CalibrationRecord(
        calibration_id=calibration_id, device_serial="X410-QA", channel=5,
        native_sample_rate_hz=T.NATIVE_RATE_UC200_HZ,
        profile_version=profile_version, gain_db=20.0, valid_until_monotonic_ns=0)

    # ---- diagnostics -----------------------------------------------------
    d = c.diagnostics
    d.result_queue_capacity = int(result_queue_capacity)
    d.event_queue_capacity = int(event_queue_capacity)
    d.stats_cadence = T.host_field(1_000_000_000)

    report = T.validate(c)
    if not report.ok():
        raise RuntimeError("the built config does not validate: %s" % report.to_string())
    return T.to_json_dict(c)


# ==========================================================================
# Invocation
# ==========================================================================


class DemoError(RuntimeError):
    def __init__(self, message, returncode=None, stderr="", output=None):
        super().__init__(message)
        self.returncode = returncode
        self.stderr = stderr
        self.output = output


def run_demo(demo, request, request_path=None, output_path=None, timeout=60):
    """Run the demo on ``request``; return the parsed output dict.

    Raises DemoError when the process exits non-zero.  The already-parsed
    error output (the C++ side emits a machine-readable error JSON) is attached
    so a caller can inspect it.
    """
    tempdir = None
    if request_path is None or output_path is None:
        tempdir = tempfile.mkdtemp(prefix="twr_fake_demo_")
    if request_path is None:
        request_path = os.path.join(tempdir, "request.json")
    if output_path is None:
        output_path = os.path.join(tempdir, "output.json")

    with open(request_path, "w", encoding="utf-8") as handle:
        json.dump(request, handle, indent=2, sort_keys=False)
        handle.write("\n")

    proc = subprocess.run(
        [demo, "--request", request_path, "--output", output_path],
        capture_output=True, text=True, timeout=timeout)
    output = None
    try:
        with open(output_path, "r", encoding="utf-8") as handle:
            output = json.load(handle)
    except (OSError, ValueError):
        output = None

    if proc.returncode != 0:
        raise DemoError(
            "twr_fake_demo exited %d" % proc.returncode,
            returncode=proc.returncode, stderr=proc.stderr, output=output)
    if output is None:
        raise DemoError("twr_fake_demo produced no parseable JSON output",
                        returncode=proc.returncode, stderr=proc.stderr,
                        output=None)
    return output, request_path, output_path


# ==========================================================================
# Result consumption (presentation only -- no arithmetic)
# ==========================================================================


def _tof_text(endpoint):
    tof = endpoint.get("tof") or {}
    if not tof.get("available"):
        return "(no estimate: this endpoint owns none)"
    return "%s/%s ticks in domain %s" % (tof.get("num"), tof.get("den"),
                                         tof.get("domain"))


def summary_lines(output):
    """A human summary that always states the simulation provenance."""
    lines = []
    sim = output.get("simulation") or {}
    lines.append("SIMULATION / protocol estimate -- NOT hardware ranging.")
    lines.append("  execution=%s mode=%s scenario=%s seed=%s"
                 % (sim.get("execution"), sim.get("mode"),
                    sim.get("scenario_id"), sim.get("seed")))
    lines.append("  protocol=%s  status.ok=%s  measurement_valid=%s"
                 % (output.get("protocol"), (output.get("status") or {}).get("ok"),
                    (output.get("provenance") or {}).get("measurement_valid")))
    for key in ("a", "b"):
        ep = (output.get("endpoints") or {}).get(key)
        if not ep:
            continue
        lines.append("  endpoint %s (%s, %s): completion=%s estimate=%s"
                     % (ep.get("endpoint_id"), ep.get("role"),
                        ep.get("protocol"), ep.get("completion"),
                        ep.get("estimate_available")))
        lines.append("    tof: %s" % _tof_text(ep))
        lines.append("    peer_evidence=%s measurement_valid=%s execution_mode=%s"
                     % (ep.get("peer_evidence"), ep.get("measurement_valid"),
                        ep.get("execution_mode")))
    cfg = output.get("configuration") or {}
    lines.append("  configuration source=%s requested_hash=%s effective_hash=%s"
                 % (cfg.get("source"), cfg.get("requested_config_hash"),
                    cfg.get("effective_config_hash")))
    lines.append("  config_sha256=%s" % output.get("config_sha256"))
    return lines


# ==========================================================================
# main
# ==========================================================================


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the M1-B two-endpoint SS/DS TWR simulation demo "
                    "(offline protocol estimate; NOT hardware ranging).")
    parser.add_argument("--scenario", choices=scenario_names(),
                        default="ss_unity",
                        help="built-in scenario (default: ss_unity)")
    parser.add_argument("--config", metavar="TWR_CONFIG_JSON",
                        help="an existing TwrConfig JSON, passed through "
                             "verbatim for the C++ side to re-validate")
    parser.add_argument("--with-config", action="store_true",
                        help="attach a complete, validated TwrConfig built with "
                             "the existing uwb.twr_config module (R09 full "
                             "config end-to-end run)")
    parser.add_argument("--protocol", choices=("ss", "ds"),
                        help="override the scenario protocol (the C++ side "
                             "cross-checks it against any --config)")
    parser.add_argument("--distance-m", type=float, default=None,
                        help="ground-truth transport distance in metres")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--demo", metavar="PATH",
                        help="path to the twr_fake_demo binary")
    parser.add_argument("--request-out", metavar="PATH",
                        help="also write the generated request JSON here")
    parser.add_argument("--output", metavar="PATH",
                        help="write the demo's output JSON here")
    parser.add_argument("--json", action="store_true",
                        help="print the full output JSON to stdout")
    args = parser.parse_args(argv)

    demo, source = find_demo_binary(args.demo)
    if demo is None:
        sys.stderr.write("error: cannot find the twr_fake_demo binary (%s)\n\n%s\n"
                         % (source, build_hint()))
        return 2
    sys.stderr.write("twr_fake_demo: using %s (%s)\n" % (demo, source))

    try:
        if args.config and args.with_config:
            raise ValueError("--config and --with-config are mutually exclusive")
        twr_config = load_twr_config(args.config) if args.config else None
        if args.with_config:
            spec = SCENARIOS[args.scenario]
            proto = args.protocol if args.protocol is not None else spec["protocol"]
            twr_config = build_twr_config(protocol=proto)
        request = build_request(
            scenario=args.scenario, protocol=args.protocol,
            distance_m=args.distance_m, seed=args.seed, twr_config=twr_config)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        sys.stderr.write("error: %s\n" % exc)
        return 2

    if args.request_out:
        with open(args.request_out, "w", encoding="utf-8") as handle:
            json.dump(request, handle, indent=2)
            handle.write("\n")
        sys.stderr.write("twr_fake_demo: request written to %s\n" % args.request_out)

    try:
        output, _req, _out = run_demo(demo, request, output_path=args.output)
    except DemoError as exc:
        sys.stderr.write("error: %s\n" % exc)
        if exc.stderr:
            sys.stderr.write(exc.stderr)
        if exc.output is not None:
            sys.stderr.write(json.dumps(exc.output, indent=2) + "\n")
        return exc.returncode or 1

    if args.json:
        sys.stdout.write(json.dumps(output, indent=2) + "\n")
    else:
        sys.stdout.write("\n".join(summary_lines(output)) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
