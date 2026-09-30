#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Driver for the M1-B two-endpoint TWR protocol demo (``twr_fake_demo``).

WHAT THIS IS
------------
A thin, pure-Python (stdlib only) front end that

  1. builds a *request* document in the small, versioned
     ``twr-m1b-demo/1`` envelope (this file deliberately knows NOTHING about
     the C++ core/FSM/config schema beyond that envelope);
  2. runs the compiled pure-C++ demo binary on it; and
  3. consumes the structured result.

It contains NO state machine, NO ToF arithmetic and NO capability logic.  All
of that lives in the C++ core; the Python side only states a scenario and reads
the answer back.  That is the M1-B rule (instruction section 6): "Python ... only
calls the CLI and consumes the output".

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
    lines.append("  input_sha256=%s" % output.get("input_sha256"))
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
        twr_config = load_twr_config(args.config) if args.config else None
        request = build_request(
            scenario=args.scenario, protocol=args.protocol,
            distance_m=args.distance_m, seed=args.seed, twr_config=twr_config)
    except (OSError, ValueError, KeyError) as exc:
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
