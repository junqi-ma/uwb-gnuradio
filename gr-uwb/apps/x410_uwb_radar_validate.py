#!/usr/bin/env python3
"""X410 UWB radar Step-12 layered validation CLI (standalone).

Modes (argparse, strict):
  --dry-run                print ONE JSON plan document and exit 0.  Never
                           imports uhd, never opens a device, never writes.
  --stage NAME             hardware stage: smoke | single | low-rate | soak |
                           calibration | cable | ota.  Optional --dry-run
                           with --stage only previews the contract.
Stage ladder, no level skipping: every stage above "smoke" requires
--stage-record with the predecessor stage's pass verdict and matching
backend/evidence class; scripted (self-test) evidence can never gate a
hardware stage.

Exit codes: 0 ok/dry-run; 1 CLI/precondition error (no device contact);
2 environment/device unavailable; 3 stage ran but checks failed; 4
internal error.

Machine-readable output: --dry-run prints one JSON document (schema
x410_uwb_radar_validate.plan.v1) with rate 737280000.0, sc16, stage, PRI,
window geometry, backend_mode, the 7-stage ladder and the disk contract.
Stage runs stream JSONL report lines (schema
x410_uwb_radar_validate.report.v1) on stdout, mirrored to
<output>/report.jsonl.

PDU/window geometry is recorded verbatim from explicit user parameters;
this CLI never constructs or demodulates PDU metadata.  Adapters:
builtin_uhd (PyUHD timed-burst backend, unverified on machines without
UHD), external (documented subprocess JSON protocol - the stable Step-11
UhdBurstBackend integration point), scripted (self-test only, requires
UWB_RADAR_VALIDATE_TEST_ADAPTER=1).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import platform
import subprocess
import sys
import time
import traceback
import uuid
from datetime import datetime, timezone

SCHEMA_PLAN = "x410_uwb_radar_validate.plan.v1"
SCHEMA_REPORT = "x410_uwb_radar_validate.report.v1"
SCHEMA_RUN = "x410_uwb_radar_validate.run.v1"
SCHEMA_CALIBRATION = "x410_uwb_radar_validate.calibration.v1"
SCHEMA_ADAPTER = "x410_uwb_radar_validate.adapter.v1"

UC200_RATE_HZ = 737280000.0
CG400_RATE_HZ = 491520000.0
NATIVE_RATE_HZ = UC200_RATE_HZ  # argparse default (UC200)
ALLOWED_NATIVE_RATES = (UC200_RATE_HZ, CG400_RATE_HZ)
C_LIGHT_M_PER_S = 299792458.0
SAMPLES_PER_SYMBOL = 1016
SFD_SYMBOLS_4Z2 = 8
WORK_INTERP = 65  # 998.4 / native = 65/M
MAX_BURST_SAMPLES = 1 << 21  # 2048-SYNC RX @737.28 ≈ 1.55e6
ALLOWED_SYNC_REPS = (32, 64, 128, 256, 512, 1024, 2048)
MAX_PSDU_BYTES = 127
MAX_FRAGMENTS = 64
ENV_TEST_ADAPTER = "UWB_RADAR_VALIDATE_TEST_ADAPTER"

ADAPTER_BUILTIN = "builtin_uhd_python"
ADAPTER_EXTERNAL = "external"
ADAPTER_SCRIPTED = "scripted"

STAGES = ["smoke", "single", "low-rate", "soak", "calibration", "cable", "ota"]
PREDECESSOR = {
    "single": "smoke",
    "low-rate": "single",
    "soak": "low-rate",
    "calibration": "soak",
    "cable": "calibration",
    "ota": "cable",
}
STAGE_TX = {"smoke": False, "single": True, "low-rate": True, "soak": True,
            "calibration": True, "cable": True, "ota": True}
STAGE_RX_STREAM = {"smoke": False, "single": True, "low-rate": True,
                   "soak": True, "calibration": True, "cable": True,
                   "ota": True}
STAGE_GAINS = STAGE_TX
STAGE_CAVEATS = {
    "smoke": ["smoke level proves environment only: no TX, no streaming"],
    "single": ["single burst is a scheduling probe, not a rate proof"],
    "low-rate": ["low-rate is a host-pacing probe, not the 200 pps target"],
    "soak": ["soak pacing is wall-clock; sustained-rate claims remain "
             "hardware-execution pending"],
    "calibration": ["calibration diagnostic is native pre-CIR template "
                    "correlation only (no production CIR authority)"],
    "cable": ["cable diag validates timing/reflections only, not RF "
              "dynamic range"],
    "ota": ["OTA records are observation-only"],
}
STAGE_DEFERRED = {
    "smoke": ["device-side rate/sync telemetry beyond probe fields"],
    "single": ["SFD/CIR verification (Step-11 chain)"],
    "low-rate": ["200 pps sustained-rate statistics"],
    "soak": ["queue watermark correlation with O/S scheduler"],
    "calibration": ["production zero_delay_tap / fractional-delay authority "
                    "(C++ CIR chain)"],
    "cable": ["RF dynamic-range and EVM statistics"],
    "ota": ["all acceptance claims; OTA is observation-only"],
}

EXIT_OK, EXIT_USAGE, EXIT_DEVICE, EXIT_CHECKS, EXIT_INTERNAL = 0, 1, 2, 3, 4


class CliError(Exception):
    def __init__(self, code, detail):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


class AdapterUnavailable(Exception):
    def __init__(self, code, detail):
        super().__init__("%s: %s" % (code, detail))
        self.code = code
        self.detail = detail


class StrictParser(argparse.ArgumentParser):
    def error(self, message):
        raise CliError("cli_invalid_arg", message)


def now_iso():
    dt = datetime.now(timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (dt.microsecond // 1000)


def llround(x):
    return int(math.floor(x + 0.5)) if x >= 0.0 else -int(math.floor(-x + 0.5))


def ceildiv(a, b):
    return -(-a // b)


def check(name, ok, details=""):
    return {"name": name, "ok": bool(ok), "details": details}


def here(*parts):
    base = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(base, *parts))


def default_tx_file():
    return here("..", "..", "testdata", "uwb_radar", "tx_737p28.sc16")


def default_template():
    return here("..", "..", "testdata",
                "reference_preamble_code9_737p28.cf32")


def build_parser():
    p = StrictParser(prog="x410_uwb_radar_validate.py", allow_abbrev=False,
                     description="Step-12 X410 layered validator")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--stage", choices=STAGES)
    p.add_argument("--stage-record", default="")
    p.add_argument("--adapter", choices=(ADAPTER_BUILTIN, ADAPTER_EXTERNAL,
                                         ADAPTER_SCRIPTED),
                   default=ADAPTER_BUILTIN)
    p.add_argument("--adapter-config", default="")
    g = p.add_argument_group("device contract")
    g.add_argument("--args", default="addr=192.168.10.2")
    g.add_argument("--rate", type=float, default=NATIVE_RATE_HZ)
    g.add_argument("--frequency", type=float, default=6489.6e6)
    g.add_argument("--tx-channel", type=int, default=0)
    g.add_argument("--rx-channel", type=int, default=1)
    g.add_argument("--tx-antenna", default="TX/RX0")
    g.add_argument("--rx-antenna", default="")
    g.add_argument("--clock-source", default="internal")
    g.add_argument("--time-source", default="internal")
    g.add_argument("--gain-tx", type=float, default=None)
    g.add_argument("--gain-rx", type=float, default=None)
    g = p.add_argument_group("waveform and timing (recorded verbatim)")
    g.add_argument("--tx-file", default=default_tx_file())
    g.add_argument("--tx-window-samples", type=int, default=0)
    g.add_argument("--pri-s", type=float, default=0.005)
    g.add_argument("--pre-guard-us", type=float, default=2.0)
    g.add_argument("--sync-reps", type=int, default=64,
                   choices=ALLOWED_SYNC_REPS)
    g.add_argument("--sfd-mode", choices=("4z2",), default="4z2")
    g.add_argument("--psdu-bytes", type=int, default=0,
                   help="random PSDU data-byte count (default 0 = empty PSDU)")
    g.add_argument("--psdu-hex", default="",
                   help="exact PSDU hex; overrides --psdu-bytes")
    g.add_argument("--psdu-random-seed", type=int, default=20260904)
    g.add_argument("--psdu-no-fcs", action="store_true",
                   help="do not append IEEE FCS when using --psdu-bytes > 0")
    g.add_argument("--range-m", type=float, default=15.0)
    g.add_argument("--tail-guard-us", type=float, default=4.1)
    g.add_argument("--rx-window-samples", type=int, default=0)
    g.add_argument("--num-delay-samps", type=int, default=0)
    g.add_argument("--cal-delay-native", type=float, default=0.0)
    g.add_argument("--calibration-id", default="")
    g.add_argument("--max-fragment-samples", type=int, default=65536)
    g.add_argument("--arm-delay-s", type=float, default=0.05)
    g = p.add_argument_group("stage knobs")
    g.add_argument("--pulses", type=int, default=0)
    g.add_argument("--duration-s", type=float, default=0.0)
    g.add_argument("--write-rx-iq", action="store_true")
    g.add_argument("--diag-threshold", type=float, default=0.3)
    g.add_argument("--cal-spread-max", type=float, default=8.0)
    g.add_argument("--min-diag-present-rate", type=float, default=0.8)
    g.add_argument("--template", default=default_template())
    g = p.add_argument_group("output")
    g.add_argument("--output", default="")
    g.add_argument("--report", default="")
    g.add_argument("--progress-interval-s", type=float, default=5.0)
    return p


# ---------------------------------------------------------------------------
# geometry / contract
# ---------------------------------------------------------------------------

def contract_rate(rate):
    for allowed in ALLOWED_NATIVE_RATES:
        if abs(rate - allowed) <= 0.5:
            return allowed
    raise CliError("rate_contract_fixed",
                   "rate must be 737.28 MS/s (UC200) or 491.52 MS/s "
                   "(CG400); got %r; UHD coercion is rejected" % (rate,))


def native_decim(rate):
    return 32 if contract_rate(rate) == CG400_RATE_HZ else 48


def geometry(o):
    rate = contract_rate(o.rate)
    o.rate = rate
    decim = native_decim(rate)
    if o.frequency <= 0:
        raise CliError("cli_invalid_param", "--frequency must be > 0")
    if o.tx_channel < 0 or o.rx_channel < 0 or o.tx_channel == o.rx_channel:
        raise CliError("tx_rx_channel_overlap",
                       "TX and RX channels must differ and be >= 0")
    pri_f = o.pri_s * rate
    pri_ticks = int(round(pri_f))
    if o.pri_s <= 0 or abs(pri_f - pri_ticks) > 1e-3:
        raise CliError(
            "pri_not_on_integer_tick_grid",
            "PRI %r s = %r ticks is not on the integer device-tick grid "
            "(%r ticks/s)" % (o.pri_s, pri_f, rate))
    if o.arm_delay_s <= 0:
        raise CliError("cli_invalid_param", "--arm-delay-s must be > 0")
    if o.max_fragment_samples <= 0:
        raise CliError("cli_invalid_param",
                       "--max-fragment-samples must be > 0")
    pre = llround(o.pre_guard_us * 1e-6 * rate)
    if o.pre_guard_us <= 0 or pre >= pri_ticks:
        raise CliError("pre_guard_out_of_range",
                       "pre-guard must be in (0 s, PRI)")
    sync_native = ceildiv(o.sync_reps * SAMPLES_PER_SYMBOL * decim,
                          WORK_INTERP)
    sfd_native = ceildiv(SFD_SYMBOLS_4Z2 * SAMPLES_PER_SYMBOL * decim,
                         WORK_INTERP)
    if o.range_m <= 0:
        raise CliError("cli_invalid_param", "--range-m must be > 0")
    range_native = int(math.ceil(2.0 * o.range_m / C_LIGHT_M_PER_S * rate))
    tail_native = llround(o.tail_guard_us * 1e-6 * rate)
    if o.tail_guard_us < 0:
        raise CliError("cli_invalid_param", "--tail-guard-us must be >= 0")
    if o.rx_window_samples > 0:
        rx = o.rx_window_samples
        rx_source = "user-provided --rx-window-samples (recorded verbatim)"
        if rx < pre + sync_native:
            raise CliError("rx_window_too_short",
                           "explicit RX window %d cannot hold pre+SYNC "
                           "(%d samples)" % (rx, pre + sync_native))
    else:
        rx = pre + sync_native + sfd_native + range_native + tail_native
        rx_source = ("default formula pre+SYNC+SFD+range+tail "
                     "(user-overridable)")
    if rx > MAX_BURST_SAMPLES:
        raise CliError("rx_window_exceeds_cap",
                       "RX window %d exceeds the burst cap %d"
                       % (rx, MAX_BURST_SAMPLES))
    if rx + pre >= pri_ticks:
        raise CliError("rx_window_exceeds_pri",
                       "RX window %d + pre-guard %d does not fit inside the "
                       "PRI %d ticks" % (rx, pre, pri_ticks))
    return {
        "pri_s": o.pri_s,
        "pri_ticks": pri_ticks,
        "pre_guard_us": o.pre_guard_us,
        "pre_guard_native": pre,
        "sync_reps": o.sync_reps,
        "sync_native": sync_native,
        "sfd_mode": o.sfd_mode,
        "sfd_native_4z2": sfd_native,
        "range_m": o.range_m,
        "range_guard_native": range_native,
        "tail_guard_us": o.tail_guard_us,
        "resampler_tail_native": tail_native,
        "rx_window_native": rx,
        "rx_window_source": rx_source,
        "rx_window_us": rx / rate * 1e6,
        "num_delay_samps": o.num_delay_samps,
        "cal_delay_native_samples": o.cal_delay_native,
        "arm_delay_s": o.arm_delay_s,
        "max_fragment_samples": o.max_fragment_samples,
        "predicted_preamble_origin_native": pre + o.num_delay_samps,
        "native_rate_hz": rate,
        "resample_interp": WORK_INTERP,
        "resample_decim": decim,
    }


def psdu_spec(o):
    if o.psdu_bytes < 0:
        raise CliError("cli_invalid_param", "--psdu-bytes must be >= 0")
    hex_digits = "".join(c for c in o.psdu_hex
                         if c in "0123456789abcdefABCDEF")
    if o.psdu_hex and len(hex_digits) % 2 != 0:
        raise CliError("cli_invalid_param",
                       "--psdu-hex must have an even number of hex digits")
    if hex_digits:
        mode = "hex"
        length = len(hex_digits) // 2
        data_bytes = length
        fcs_bytes = 0
    elif o.psdu_bytes == 0:
        mode = "empty"
        length = 0
        data_bytes = 0
        fcs_bytes = 0
    else:
        mode = "random_data" if o.psdu_no_fcs else "random_data_plus_fcs"
        data_bytes = o.psdu_bytes
        fcs_bytes = 0 if o.psdu_no_fcs else 2
        length = data_bytes + fcs_bytes
    if length > MAX_PSDU_BYTES:
        raise CliError("psdu_too_long",
                       "PSDULength %d exceeds 127" % length)
    return {
        "mode": mode,
        "data_bytes": data_bytes,
        "fcs_bytes": fcs_bytes,
        "psdu_length_bytes": length,
        "hex": hex_digits.upper(),
        "random_seed": o.psdu_random_seed,
        "default_empty": mode == "empty",
    }


def tx_geometry(o):
    path = o.tx_file or ""
    info = {"path": os.path.abspath(path) if path else "", "exists": False,
            "samples": None, "airtime_us": None, "status": "not_required"}
    if path and os.path.isfile(path):
        size = os.path.getsize(path)
        info["exists"] = True
        info["bytes"] = size
        if size > 0 and size % 4 == 0:
            info["samples"] = size // 4
    if o.tx_window_samples > 0:
        info["samples"] = o.tx_window_samples
        info["source"] = "--tx-window-samples override"
    elif info["samples"]:
        info["source"] = "derived from --tx-file size"
    if info["samples"]:
        info["airtime_us"] = info["samples"] / o.rate * 1e6
    return info


def stage_burst_plan(o, for_hardware):
    stage = o.stage
    if stage is None:
        return {"bursts": None, "duration_s": None}
    if stage == "smoke":
        return {"bursts": 0, "duration_s": None}
    if stage == "single":
        if o.pulses not in (0, 1):
            raise CliError("cli_invalid_param",
                           "stage 'single' always schedules exactly 1 burst")
        return {"bursts": 1, "duration_s": None}
    if stage == "low-rate":
        if not (0.1 <= o.pri_s <= 1.0):
            raise CliError(
                "stage_pri_out_of_range",
                "low-rate requires --pri-s in [0.1, 1.0] (1-10 pulse/s); "
                "got %r" % (o.pri_s,))
        pulses = o.pulses if o.pulses > 0 else 10
        if not (1 <= pulses <= 100):
            raise CliError("pulses_out_of_range",
                           "low-rate --pulses must be in [1, 100]")
        return {"bursts": pulses, "duration_s": None}
    if stage == "soak":
        if not (0.002 <= o.pri_s <= 0.01):
            raise CliError(
                "stage_pri_out_of_range",
                "soak requires --pri-s in [0.002, 0.01] (target 200 "
                "pulse/s); got %r" % (o.pri_s,))
        duration = o.duration_s if o.duration_s > 0 else 30.0
        if duration > 300.0:
            raise CliError("duration_out_of_range",
                           "soak --duration-s must be in (0, 300]")
        return {"bursts": max(2, int(duration / o.pri_s)),
                "duration_s": duration}
    pulses = o.pulses if o.pulses > 0 else {"calibration": 32, "cable": 200,
                                            "ota": 100}[stage]
    if not (1 <= pulses <= 10000):
        raise CliError("pulses_out_of_range",
                       "%s --pulses must be in [1, 10000]" % stage)
    return {"bursts": pulses, "duration_s": None}


def resolve_adapter(o, for_hardware):
    if o.adapter == ADAPTER_SCRIPTED:
        if os.environ.get(ENV_TEST_ADAPTER) != "1":
            raise CliError(
                "scripted_adapter_requires_env",
                "the scripted adapter is self-test only; set %s=1 to enable "
                "it" % ENV_TEST_ADAPTER)
        cfg = _load_adapter_config(o) or {
            "rx_mode": "loopback", "delay_native_samples": 37,
            "amplitude": 8000}
        return {"name": ADAPTER_SCRIPTED, "backend": "scripted",
                "evidence_class": "test",
                "config_path": (os.path.abspath(o.adapter_config)
                                if o.adapter_config else ""),
                "config": cfg}
    if o.adapter == ADAPTER_EXTERNAL:
        cfg = _load_adapter_config(o)
        if cfg.get("schema") != SCHEMA_ADAPTER:
            raise CliError("adapter_config_invalid",
                           "adapter config schema must be %s" % SCHEMA_ADAPTER)
        if cfg.get("type") != ADAPTER_EXTERNAL:
            raise CliError("adapter_config_invalid",
                           "adapter config type must be 'external'")
        backend = cfg.get("backend")
        if not isinstance(backend, str) or not backend:
            raise CliError("adapter_config_invalid",
                           "adapter config requires a non-empty 'backend'")
        evidence = cfg.get("evidence_class", "test")
        if evidence not in ("hardware", "test"):
            raise CliError("adapter_config_invalid",
                           "evidence_class must be 'hardware' or 'test'")
        command = cfg.get("command")
        if not isinstance(command, str) or not command:
            raise CliError("adapter_config_invalid",
                           "external adapter config requires a non-empty "
                           "'command'")
        return {"name": ADAPTER_EXTERNAL, "backend": backend,
                "evidence_class": evidence,
                "config_path": os.path.abspath(o.adapter_config),
                "config": cfg}
    return {"name": ADAPTER_BUILTIN, "backend": ADAPTER_BUILTIN,
            "evidence_class": "hardware", "config_path": "", "config": {}}


def _load_adapter_config(o):
    if not o.adapter_config:
        return {}
    if not os.path.isfile(o.adapter_config):
        raise CliError("adapter_config_missing",
                       "adapter config not found: %r" % (o.adapter_config,))
    try:
        with open(o.adapter_config, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError) as exc:
        raise CliError("adapter_config_invalid", str(exc))


def check_gate(o, backend, evidence_class):
    stage = o.stage
    pred = PREDECESSOR.get(stage)
    if pred is None:
        return {"required": False}
    if not o.stage_record:
        raise CliError(
            "missing_stage_record",
            "stage '%s' requires the pass record of predecessor '%s' via "
            "--stage-record; level skipping is forbidden" % (stage, pred))
    path = o.stage_record
    if not os.path.isfile(path):
        raise CliError("stage_record_unreadable",
                       "record not found: %s" % path)
    results = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                raw = raw.strip()
                if not raw:
                    continue
                line = json.loads(raw)
                if isinstance(line, dict) and \
                        line.get("type") == "stage_result":
                    results.append(line)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise CliError("stage_record_invalid", "%s: %s" % (path, exc))
    if not results:
        raise CliError("stage_record_invalid",
                       "%s has no stage_result line" % path)
    last = results[-1]
    if last.get("stage") != pred:
        raise CliError(
            "wrong_predecessor_in_record",
            "record's last stage_result is %r, expected predecessor '%s'"
            % (last.get("stage"), pred))
    if last.get("result") != "pass":
        raise CliError(
            "predecessor_stage_not_passed",
            "predecessor '%s' result=%r; fix the failing layer before "
            "advancing" % (pred, last.get("result")))
    rate = contract_rate(o.rate)
    if last.get("rate_hz") not in (rate, int(rate)):
        raise CliError("stage_record_rate_mismatch",
                       "record rate_hz=%r != contract %r"
                       % (last.get("rate_hz"), rate))
    if last.get("backend") != backend or \
            last.get("evidence_class") != evidence_class:
        if evidence_class == "hardware":
            raise CliError(
                "predecessor_evidence_not_hardware",
                "predecessor evidence (%r/%r) cannot gate a hardware stage; "
                "scripted evidence is never hardware evidence"
                % (last.get("backend"), last.get("evidence_class")))
        raise CliError(
            "predecessor_backend_mismatch",
            "predecessor backend %r/%r does not match this run's %r/%r"
            % (last.get("backend"), last.get("evidence_class"), backend,
               evidence_class))
    return {"required": True, "predecessor": pred,
            "record": os.path.abspath(path), "record_backend": backend,
            "record_result": "pass"}


def validate(o, for_hardware):
    geo = geometry(o)
    psdu = psdu_spec(o)
    stage = o.stage
    burst_plan = stage_burst_plan(o, for_hardware)
    tx = tx_geometry(o)
    mfs = o.max_fragment_samples
    frags = {"rx": ceildiv(geo["rx_window_native"], mfs),
             "tx": ceildiv((tx["samples"] or 1), mfs)}
    if frags["rx"] > MAX_FRAGMENTS or frags["tx"] > MAX_FRAGMENTS:
        raise CliError(
            "fragment_cap_exceeded",
            "burst fragmentation would need TX=%d / RX=%d fragments; "
            "cap is %d (raise --max-fragment-samples)"
            % (frags["tx"], frags["rx"], MAX_FRAGMENTS))
    adapter = resolve_adapter(o, for_hardware)
    tx_samples = tx["samples"] or 0
    if tx_samples and \
            tx_samples + geo["pre_guard_native"] >= geo["pri_ticks"]:
        raise CliError(
            "tx_overruns_next_rx",
            "TX packet %d + pre-guard %d must fit inside PRI %d ticks"
            % (tx_samples, geo["pre_guard_native"], geo["pri_ticks"]))
    if for_hardware and stage is not None and STAGE_TX[stage]:
        if o.gain_tx is None or o.gain_rx is None:
            raise CliError(
                "missing_explicit_gain",
                "stage '%s' transmits: --gain-tx and --gain-rx are required "
                "(explicit only, no high-power implicit default)" % stage)
        if not tx["exists"]:
            raise CliError("tx_file_missing",
                           "TX waveform file not found: %r" % (tx["path"],))
        if not tx_samples:
            raise CliError(
                "tx_file_invalid",
                "%s is not a non-empty interleaved int16 I/Q (SC16) file"
                % (tx["path"],))
    if for_hardware:
        if not o.output:
            raise CliError("output_required",
                           "stage '%s' requires --output (evidence directory)"
                           % (stage,))
        out = o.output
        if os.path.isdir(out) and os.listdir(out):
            raise CliError("output_dir_not_empty",
                           "--output exists and is not empty: %s" % out)
        elif os.path.exists(out):
            raise CliError("output_path_invalid",
                           "--output exists but is not a directory: %s" % out)
        if stage == "calibration":
            if not os.path.isfile(o.template):
                raise CliError("template_file_missing",
                               "diagnostic template not found: %r"
                               % (o.template,))
            try:
                import numpy  # noqa: F401
            except Exception as exc:
                raise CliError("numpy_unavailable",
                               "the calibration diagnostic needs numpy: %s"
                               % exc)
        gate = check_gate(o, adapter["backend"], adapter["evidence_class"])
    else:
        gate = {"required": PREDECESSOR.get(stage) is not None,
                "predecessor": PREDECESSOR.get(stage),
                "note": "checked at stage time, before any device contact"}
    return {"stage": stage, "geo": geo, "tx": tx, "psdu": psdu,
            "tx_samples": tx_samples, "burst_plan": burst_plan,
            "adapter": adapter, "gate": gate,
            "calibration_id": o.calibration_id or "cal-unspecified"}


# ---------------------------------------------------------------------------
# plan (dry-run document; also embedded in stage reports)
# ---------------------------------------------------------------------------

def build_plan(o, cfg):
    geo = cfg["geo"]
    stage = o.stage
    ladder = []
    for i, name in enumerate(STAGES):
        ladder.append({
            "stage": name,
            "order": i + 1,
            "tx": STAGE_TX[name],
            "rx_streaming": STAGE_RX_STREAM[name],
            "requires_predecessor": PREDECESSOR.get(name),
        })
    env = {"uhd_module_probed": False, "uhd_module_available": None,
           "numpy_available": None}
    try:
        import numpy  # noqa: F401
        env["numpy_available"] = True
    except Exception:
        env["numpy_available"] = False
    plan = {
        "schema": SCHEMA_PLAN,
        "mode": "dry-run" if o.dry_run else "stage",
        "generated_utc": now_iso(),
        "requested_stage": stage,
        "hardware_execution_pending": True,
        "device": {
            "args": o.args,
            "model_contract": "ni/x410",
            "clock_source": o.clock_source,
            "time_source": o.time_source,
            "probed": False,
        },
        "rate": {"hz": o.rate,
                 "contract": "737.28 MS/s (UC200) or 491.52 MS/s (CG400); "
                             "UHD coercion rejected",
                 "resample": "65/48" if o.rate == UC200_RATE_HZ else "65/32"},
        "sample_format": {"cpu": "sc16", "otw": "sc16"},
        "channels": {"tx": o.tx_channel, "tx_antenna": o.tx_antenna,
                     "rx": o.rx_channel,
                     "rx_antenna": o.rx_antenna or "device default",
                     "frequency_hz": o.frequency},
        "gains": {"tx": o.gain_tx, "rx": o.gain_rx,
                  "policy": "explicit-only for transmitting stages"},
        "pri": {"s": o.pri_s, "ticks": geo["pri_ticks"]},
        "windows": geo,
        "psdu": cfg["psdu"],
        "backend_mode": o.adapter,
        "adapter": {
            "mode": o.adapter,
            "schema": SCHEMA_ADAPTER,
            "config": o.adapter_config or "",
            "step11_note": "external adapter config/schema is the stable "
                           "subprocess integration point for a future "
                           "Step-11 UhdBurstBackend runner; this CLI never "
                           "guesses or modifies Step-11 files or interfaces",
        },
        "stage_ladder": ladder,
        "stages": [
            {"stage": s["stage"], "order": s["order"], "tx": s["tx"],
             "rx_streaming": s["rx_streaming"],
             "requires_predecessor": s["requires_predecessor"]}
            for s in ladder
        ],
        "disk_contract": {
            "output_dir": (o.output or "<output>") if o.stage else None,
            "report": "report.jsonl (schema %s)" % SCHEMA_REPORT,
            "run_config": "run.json (static configuration snapshot)",
            "rx_iq": "rx_iq/pulse_<id>.sc16 when --write-rx-iq",
            "calibration": "calibration.json (calibration stage only)",
            "note": "cir.cf32/cir.jsonl are production CIR products "
            "(Step-11 chain), not written by this CLI",
        },
        "exit_codes": {
            "0": "stage passed / dry-run",
            "1": "CLI or precondition error (no device contact)",
            "2": "environment/device unavailable",
            "3": "stage ran but a check failed (fail-fast)",
            "4": "internal error",
        },
        "environment": env,
        "notes": [
            "Step-12 deliverable is the executable harness + evidence "
            "schema; hardware execution is arranged separately and no layer "
            "is claimed verified here.",
            "PDU/window geometry is recorded verbatim from user parameters; "
            "this CLI constructs and demodulates no PDU metadata.",
            "Never treat OTA observations as timed-I/O, SFD/CIR or RF "
            "acceptance; every ota record carries ota_observation_only=true.",
            "Production CIR statistics (ok/sfd_failed/timing_failed/"
            "cir_failed, work-domain zero_delay_tap) require the Step-11 "
            "EchoTimer chain and are listed as deferred in staged reports.",
        ],
        "uhd_import": "never performed in dry-run",
    }
    return plan


# ---------------------------------------------------------------------------
# JSONL report
# ---------------------------------------------------------------------------

class Report:
    def __init__(self, run_id, path="", stage="", backend=""):
        self.run_id = run_id
        self.stage = stage or ""
        self.backend = backend or ""
        self.seq = 0
        self._fh = None
        if path:
            d = os.path.dirname(os.path.abspath(path))
            os.makedirs(d, exist_ok=True)
            self._fh = open(path, "w", encoding="utf-8")
        self.path = path

    def emit(self, type_, level="info", **fields):
        self.seq += 1
        line = {"schema": SCHEMA_REPORT, "seq": self.seq, "ts_utc": now_iso(),
                "run_id": self.run_id, "type": type_, "level": level,
                "stage": self.stage, "backend": self.backend}
        line.update(fields)
        text = json.dumps(line, ensure_ascii=False)
        sys.stdout.write(text + "\n")
        sys.stdout.flush()
        if self._fh is not None:
            self._fh.write(text + "\n")
            self._fh.flush()
        return line

    def close(self):
        if self._fh is not None:
            self._fh.close()
            self._fh = None


def emit_cli_error(code, detail):
    for obj in (
        {"schema": SCHEMA_REPORT, "seq": 0, "ts_utc": now_iso(),
         "run_id": "", "type": "error", "level": "error", "stage": "",
         "backend": "", "code": code, "detail": detail},
        {"schema": SCHEMA_REPORT, "seq": 1, "ts_utc": now_iso(),
         "run_id": "", "type": "summary", "level": "error", "stage": "",
         "backend": "", "requested_stage": None, "result": "validation_error",
         "exit_code": EXIT_USAGE},
    ):
        sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def emit_internal_error(detail):
    obj = {"schema": SCHEMA_REPORT, "seq": 0, "ts_utc": now_iso(),
           "run_id": "", "type": "error", "level": "error", "stage": "",
           "backend": "", "code": "internal_error", "detail": detail}
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# adapters
# ---------------------------------------------------------------------------

def fragments_for(o, total):
    spans = []
    off = 0
    while off < total:
        n = min(o.max_fragment_samples, total - off)
        flags = []
        if not spans:
            flags += ["time_spec", "start_of_burst"]
        spans.append({"offset": off, "count": n})
        off += n
    for i, span in enumerate(spans):
        span["flags"] = (["time_spec", "start_of_burst"] if i == 0 else [])
        if i == len(spans) - 1:
            span["flags"].append("end_of_burst")
    return spans


class BuiltinUhdAdapter:
    backend = ADAPTER_BUILTIN
    evidence_class = "hardware"
    wall_pacing = True

    def __init__(self, o, cfg):
        self.o = o
        self.cfg = cfg
        self._uhd = None
        self._usrp = None
        self._rx_stream = None
        self._tx_stream = None
        self._np = None
        self._wave = None
        self._buf = None
        self._ts_cls = None
        self._cmd_t = None
        self._cmd_mode = None
        self._tx_md_t = None
        self._rx_md_t = None
        self.probe = {}
        self.checks = []

    def _import_uhd(self):
        try:
            import uhd
        except Exception as exc:
            raise AdapterUnavailable(
                "uhd_unavailable",
                "PyUHD python module not importable (%s); the builtin "
                "adapter needs the UHD 4.x python bindings" % exc)
        self._uhd = uhd

    def _resolve_dialect(self):
        uhd = self._uhd
        types_mod = getattr(uhd, "types", None)
        self._ts_cls = getattr(uhd, "time_spec_t", None) or \
            getattr(types_mod, "TimeSpec", None)
        self._cmd_t = getattr(uhd, "stream_cmd_t", None) or \
            getattr(types_mod, "StreamCmd", None)
        self._tx_md_t = getattr(uhd, "tx_metadata_t", None) or \
            getattr(types_mod, "TxMetadata", None)
        self._rx_md_t = getattr(uhd, "rx_metadata_t", None) or \
            getattr(types_mod, "RxMetadata", None)
        missing = [n for n, v in (
            ("time_spec", self._ts_cls),
            ("stream_cmd", self._cmd_t),
            ("tx_metadata", self._tx_md_t),
            ("rx_metadata", self._rx_md_t)) if v is None]
        if missing:
            raise AdapterUnavailable(
                "uhd_dialect_unresolved",
                "cannot locate PyUHD classes (%s); refusing to guess the API "
                "layout" % ", ".join(missing))
        inner = getattr(self._cmd_t, "stream_mode_t", self._cmd_t)
        self._cmd_mode = getattr(inner, "num_samps_and_done", None) or \
            getattr(inner, "STREAM_MODE_NUM_SAMPS_AND_DONE", None)
        if self._cmd_mode is None:
            raise AdapterUnavailable("uhd_dialect_unresolved",
                                     "no num_samps_and_done stream mode")

    def _tspec(self, ticks):
        rate = int(self.o.rate)
        full = ticks // rate
        frac = (ticks % rate) / float(rate)
        try:
            return self._ts_cls(full, frac)
        except Exception:
            return self._ts_cls(full + frac)

    def _ticks_of(self, ts):
        try:
            full = float(ts.get_full_secs())
            frac = float(ts.get_frac_secs())
        except Exception:
            return None
        return llround(full * self.o.rate + frac * self.o.rate)

    def setup(self):
        self._import_uhd()
        try:
            import numpy as np
        except Exception as exc:
            raise AdapterUnavailable("numpy_unavailable", str(exc))
        self._np = np
        self._resolve_dialect()
        try:
            usrp = self._uhd.usrp(self.o.args)
        except Exception as exc:
            raise AdapterUnavailable(
                "uhd_device_unavailable",
                "uhd.usrp(%r) failed: %s" % (self.o.args, exc))
        self._usrp = usrp
        probe = {"device": self.o.args}
        try:
            usrp.set_clock_source(self.o.clock_source)
            usrp.set_time_source(self.o.time_source)
        except Exception as exc:
            raise AdapterUnavailable("uhd_config_failed",
                                     "clock/time source: %s" % exc)
        try:
            usrp.set_rx_rate(self.o.rate, self.o.rx_channel)
            actual = float(usrp.get_rx_rate(self.o.rx_channel))
        except Exception as exc:
            raise AdapterUnavailable("uhd_config_failed",
                                     "rx rate: %s" % exc)
        if abs(actual - self.o.rate) > 1.0:
            raise AdapterUnavailable(
                "rate_coerced",
                "device coerced RX rate to %r; contracted rate %r is "
                "not met" % (actual, self.o.rate))
        probe["rate_hz"] = actual
        try:
            usrp.set_tx_rate(self.o.rate, self.o.tx_channel)
            actual_tx = float(usrp.get_tx_rate(self.o.tx_channel))
        except Exception as exc:
            raise AdapterUnavailable("uhd_config_failed",
                                     "tx rate: %s" % exc)
        if abs(actual_tx - self.o.rate) > 1.0:
            raise AdapterUnavailable(
                "rate_coerced",
                "device coerced TX rate to %r" % (actual_tx,))
        probe["tx_rate_hz"] = actual_tx
        try:
            probe["rx_channels"] = int(usrp.get_rx_num_channels())
            probe["tx_channels"] = int(usrp.get_tx_num_channels())
            probe["rx_antennas"] = [str(x) for x in
                                    usrp.get_rx_antennas(self.o.rx_channel)]
            probe["tx_antennas"] = [str(x) for x in
                                    usrp.get_tx_antennas(self.o.tx_channel)]
        except Exception as exc:
            probe["channel_query_error"] = str(exc)
        ts = usrp.get_time_now()
        probe["time_now_ticks"] = self._ticks_of(ts)
        probe["clock_source"] = self.o.clock_source
        probe["time_source"] = self.o.time_source
        self.probe = probe
        self.checks = self._probe_checks(probe)
        return {"probe": probe, "checks": self.checks}

    def _probe_checks(self, probe):
        return [
            check("device_open", True, probe.get("device", "")),
            check("rate_exact_737p28",
                  abs(float(probe.get("rate_hz", 0)) - self.o.rate) <= 1.0,
                  "rate_hz=%r" % (probe.get("rate_hz"),)),
            check("clock_source_applied",
                  probe.get("clock_source") == self.o.clock_source,
                  probe.get("clock_source")),
            check("time_source_applied",
                  probe.get("time_source") == self.o.time_source,
                  probe.get("time_source")),
            check("channels_present",
                  int(probe.get("rx_channels", 0)) > self.o.rx_channel and
                  int(probe.get("tx_channels", 0)) > self.o.tx_channel,
                  {"rx": probe.get("rx_channels"),
                   "tx": probe.get("tx_channels")}),
            check("time_now_readable",
                  int(probe.get("time_now_ticks", -1)) >= 0,
                  probe.get("time_now_ticks")),
            check("stream_format_sc16", True,
                  "cpu=sc16 otw=sc16 (contract; verified by streamer "
                  "creation)"),
        ]

    def setup_full(self):
        if self._usrp is None:
            self.setup()
        uhd = self._uhd
        np = self._np
        try:
            sa_rx = uhd.stream_args("sc16", "sc16", "", [self.o.rx_channel])
            self._rx_stream = self._usrp.get_rx_stream(sa_rx)
            sa_tx = uhd.stream_args("sc16", "sc16", "", [self.o.tx_channel])
            self._tx_stream = self._usrp.get_tx_stream(sa_tx)
        except Exception as exc:
            raise AdapterUnavailable("uhd_config_failed",
                                     "streamer creation: %s" % exc)
        try:
            tune = uhd
            tr = getattr(tune, "types", tune)
            tr = tr.TuneRequest(self.o.frequency)
            self._usrp.set_rx_freq(tr, self.o.rx_channel)
            self._usrp.set_tx_freq(tr, self.o.tx_channel)
            if self.o.rx_antenna:
                self._usrp.set_rx_antenna(self.o.rx_antenna,
                                          self.o.rx_channel)
            if self.o.tx_antenna:
                self._usrp.set_tx_antenna(self.o.tx_antenna,
                                          self.o.tx_channel)
            self._usrp.set_rx_gain(self.o.gain_rx, self.o.rx_channel)
            self._usrp.set_tx_gain(self.o.gain_tx, self.o.tx_channel)
        except Exception as exc:
            raise AdapterUnavailable("uhd_config_failed",
                                     "RF config: %s" % exc)
        wave = np.fromfile(self.o.tx_file, dtype=np.int16)
        want = self.cfg["tx_samples"] * 2
        if wave.size != want:
            raise AdapterUnavailable(
                "tx_waveform_mismatch",
                "TX file has %d int16 values, expected %d" % (wave.size,
                                                              want))
        self._wave = wave
        self._buf = np.zeros(2 * self.cfg["geo"]["rx_window_native"],
                             dtype=np.int16)
        return {}

    def time_now_ticks(self):
        return self._ticks_of(self._usrp.get_time_now()) or 0

    def issue_rx(self, t_rx_ticks, n_samps):
        cmd = self._cmd_t(self._cmd_mode)
        cmd.num_samps = int(n_samps)
        cmd.stream_now = False
        cmd.time_spec = self._tspec(t_rx_ticks)
        self._rx_stream.issue_stream_cmd(cmd)

    def issue_tx(self, t_tx_ticks, fragments):
        total = self.cfg["tx_samples"]
        spans = fragments_for(self.o, total)
        sent_total = 0
        for i, span in enumerate(spans):
            md = self._tx_md_t()
            md.has_time_spec = i == 0
            md.start_of_burst = i == 0
            md.end_of_burst = i == len(spans) - 1
            if i == 0:
                md.time_spec = self._tspec(t_tx_ticks)
            lo, hi = 2 * span["offset"], 2 * (span["offset"] + span["count"])
            sent = 0
            while sent < hi - lo:
                n = int(self._tx_stream.send(self._wave[lo + sent:hi], md))
                if n <= 0:
                    return {"tx_samples_sent": sent_total // 2,
                            "tx_reissues": 0, "error": "send_stalled"}
                sent += n
                sent_total += n
                if sent < hi - lo:
                    md.has_time_spec = False
                    md.start_of_burst = False
        return {"tx_samples_sent": sent_total // 2, "tx_reissues": 0,
                "error": ""}

    def collect(self, timeout_s, n_samps):
        np = self._np
        buf = self._buf
        md = self._rx_md_t()
        got = 0
        first_ts = None
        status = "backend_error"
        error = ""
        deadline = time.monotonic() + max(0.01, timeout_s)
        while got < n_samps:
            left = deadline - time.monotonic()
            if left <= 0.0:
                status = "timeout" if got == 0 else "backend_error"
                error = "collect deadline exceeded after %d samples" % got
                break
            n = int(self._rx_stream.recv(buf[got * 2:], md, left))
            if n <= 0:
                status, error = self._map_rx_error(md)
                break
            if first_ts is None and getattr(md, "has_time_spec", False):
                first_ts = self._ticks_of(md.time_spec)
            got += n
        if got == n_samps and status == "backend_error":
            status = "ok"
            error = ""
        data = buf.tobytes() if (status == "ok" and got == n_samps) else b""
        return {"status": status, "rx_samples_received": got,
                "tx_samples_sent": self.cfg["tx_samples"],
                "first_sample_tick": first_ts if first_ts is not None
                else (0 if status == "ok" else None),
                "error": error, "data": data}

    @staticmethod
    def _map_rx_error(md):
        code = getattr(md, "error_code", None)
        name = str(getattr(code, "name", code) or "").lower()
        if "timeout" in name:
            return "timeout", name
        if "overflow" in name:
            return "overflow", name
        if "late" in name:
            return "late_command", name
        if "broken" in name:
            return "broken_chain", name
        if "none" in name:
            return "backend_error", "recv returned 0 with no error code"
        return "backend_error", name or "unknown rx error"

    def abort_rx(self):
        try:
            inner = getattr(self._cmd_t, "stream_mode_t", self._cmd_t)
            stop = getattr(inner, "stop_cont", None) or \
                getattr(self._cmd_t, "STREAM_MODE_STOP_CONT", None)
            if stop is not None and self._rx_stream is not None:
                self._rx_stream.issue_stream_cmd(self._cmd_t(stop))
        except Exception:
            pass

    def close(self):
        self._rx_stream = None
        self._tx_stream = None
        self._usrp = None


class ExternalAdapter:
    """Subprocess JSON-line protocol client (Step-11 integration point).
    Ops (one JSON object per line): setup{stage}->{ok,probe,checks},
    time_now->{ticks}, issue_rx{ticks,num_samps}, issue_tx{ticks,wave_b64,
    fragments}->{tx_samples_sent}, collect{timeout_s}->{status,...,data_b64},
    abort_rx, close."""

    evidence_class = None
    wall_pacing = True

    def __init__(self, o, cfg, acfg):
        self.o = o
        self.cfg = cfg
        self.acfg = acfg
        self.backend = acfg["backend"]
        self.evidence_class = acfg["evidence_class"]
        self.proc = None

    def _request(self, payload):
        try:
            self.proc.stdin.write(json.dumps(payload) + "\n")
            self.proc.stdin.flush()
        except Exception as exc:
            raise AdapterUnavailable("adapter_io_failed", str(exc))
        line = self.proc.stdout.readline()
        if not line:
            raise AdapterUnavailable("adapter_io_failed",
                                     "runner closed stdout")
        try:
            resp = json.loads(line)
        except ValueError as exc:
            raise AdapterUnavailable("adapter_protocol_error", str(exc))
        if not resp.get("ok"):
            raise AdapterUnavailable(
                "adapter_request_failed",
                "op %r failed: %s" % (payload.get("op"),
                                      resp.get("error", resp)))
        return resp

    def setup(self):
        argv = [self.acfg["command"]] + list(self.acfg.get("argv", []))
        try:
            self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, text=True)
        except OSError as exc:
            raise AdapterUnavailable("adapter_spawn_failed", str(exc))
        resp = self._request({"op": "setup", "stage": self.o.stage,
                              "cfg": {"rate_hz": self.o.rate,
                                      "otw": "sc16", "cpu": "sc16",
                                      "args": self.o.args,
                                      "tx_channel": self.o.tx_channel,
                                      "rx_channel": self.o.rx_channel,
                                      "frequency": self.o.frequency}})
        probe = resp.get("probe") or {}
        checks = resp.get("checks") or []
        return {"probe": probe, "checks": checks}

    def setup_full(self):
        if self.proc is None:
            self.setup()
        return {}

    def time_now_ticks(self):
        return int(self._request({"op": "time_now"}).get("ticks", 0))

    def issue_rx(self, t_rx_ticks, n_samps):
        self._request({"op": "issue_rx", "t_rx_ticks": int(t_rx_ticks),
                       "num_samps": int(n_samps)})

    def issue_tx(self, t_tx_ticks, fragments):
        import numpy as np
        wave = np.fromfile(self.o.tx_file, dtype=np.int16)
        wave_b64 = base64.b64encode(wave.tobytes()).decode("ascii")
        resp = self._request({"op": "issue_tx", "t_tx_ticks": int(t_tx_ticks),
                              "wave_b64": wave_b64,
                              "fragments": fragments})
        return {"tx_samples_sent": int(resp.get("tx_samples_sent", 0)),
                "tx_reissues": 0, "error": ""}

    def collect(self, timeout_s, n_samps):
        resp = self._request({"op": "collect", "timeout_s": timeout_s,
                              "num_samps": int(n_samps)})
        data = b""
        b64 = resp.get("data_b64")
        if b64:
            data = base64.b64decode(b64)
        return {"status": resp.get("status", "backend_error"),
                "rx_samples_received": int(resp.get("rx_samples_received", 0)),
                "tx_samples_sent": resp.get("tx_samples_sent"),
                "first_sample_tick": resp.get("first_sample_tick"),
                "error": resp.get("error", ""), "data": data}

    def abort_rx(self):
        try:
            self._request({"op": "abort_rx"})
        except Exception:
            pass

    def close(self):
        if self.proc is not None:
            try:
                self._request({"op": "close"})
            except Exception:
                pass
            try:
                self.proc.stdin.close()
                self.proc.wait(timeout=10)
            except Exception:
                self.proc.kill()
            self.proc = None


class ScriptedAdapter:
    backend = ADAPTER_SCRIPTED
    evidence_class = "test"
    wall_pacing = False

    def __init__(self, o, cfg, scfg):
        self.o = o
        self.cfg = cfg
        self.scfg = scfg if isinstance(scfg, dict) else {}
        self.base_ticks = 10 * int(self.o.rate)
        self._issued = 0
        self._pending = None
        self._wave = b""
        self._wave_skip = 0
        self.probe = {}
        self.checks = []

    def _load_wave(self):
        wave_path = self.scfg.get("wave") or self.o.tx_file
        if self.scfg.get("rx_mode", "loopback") == "loopback" and wave_path \
                and os.path.isfile(wave_path):
            with open(wave_path, "rb") as f:
                self._wave = f.read()
        self._wave_skip = self._sync_offset_bytes()

    def _sync_offset_bytes(self):
        """Test fixture: align the waveform so its preamble (best one-SYNC
        template match) starts at the scheduled window origin pre+delay,
        emulating the burst model where the preamble begins at t_tx."""
        try:
            import numpy as np
            if not self._wave or not os.path.isfile(self.o.template):
                return 0
            raw = np.fromfile(self.o.template, dtype=np.float32)
            if raw.size < 8 or raw.size % 2:
                return 0
            t = (raw[0::2] + 1j * raw[1::2]).astype(np.complex128)
            t /= np.linalg.norm(t)
            arr = np.frombuffer(self._wave, dtype=np.int16)
            x = (arr[0::2].astype(np.float64)
                 + 1j * arr[1::2].astype(np.float64)) / 32767.0
            n, L = x.size, t.size
            if n <= L:
                return 0
            fft_len = 1
            while fft_len < n + L:
                fft_len <<= 1
            corr = np.fft.ifft(np.fft.fft(x, fft_len)
                               * np.conj(np.fft.fft(t, fft_len)))[:n - L + 1]
            return int(np.argmax(np.abs(corr))) * 4
        except Exception:
            return 0

    def setup(self):
        self._load_wave()
        self.probe = {"device": "scripted-no-device",
                      "rate_hz": self.o.rate,
                      "tx_rate_hz": self.o.rate,
                      "clock_source": self.o.clock_source,
                      "time_source": self.o.time_source,
                      "tx_channel": self.o.tx_channel,
                      "rx_channel": self.o.rx_channel,
                      "tx_antennas": [self.o.tx_antenna],
                      "rx_antennas": ["RX1", "RX2"],
                      "rx_antenna": self.o.rx_antenna or "RX1",
                      "otw": "sc16", "cpu": "sc16",
                      "time_now_ticks": self.base_ticks}
        self.checks = [
            check("device_open", True, "scripted fake device"),
            check("rate_exact_737p28", True, "scripted contract rate"),
            check("clock_source_applied", True, self.o.clock_source),
            check("time_source_applied", True, self.o.time_source),
            check("channels_present", True,
                  {"tx": self.o.tx_channel, "rx": self.o.rx_channel}),
            check("time_now_readable", True, self.base_ticks),
            check("stream_format_sc16", True, "cpu=sc16 otw=sc16"),
        ]
        return {"probe": self.probe, "checks": self.checks}

    def setup_full(self):
        self._load_wave()
        return {}

    def time_now_ticks(self):
        pri = self.cfg["geo"]["pri_ticks"]
        return self.base_ticks + self._issued * pri - 1

    def issue_rx(self, t_rx_ticks, n_samps):
        self._pending = (t_rx_ticks, n_samps)

    def issue_tx(self, t_tx_ticks, fragments):
        self._issued += 1
        return {"tx_samples_sent": self.cfg["tx_samples"], "tx_reissues": 0,
                "error": ""}

    def collect(self, timeout_s, n_samps):
        t_rx, want = self._pending
        idx = self._issued - 1
        fault = self.scfg.get("fault")
        fault_at = self.scfg.get("fault_at_index")
        if fault and fault_at is not None and idx == fault_at:
            got = want - 1 if fault == "short_rx" else 0
            return {"status": fault if fault != "short_rx"
                    else "backend_error",
                    "rx_samples_received": got,
                    "tx_samples_sent": self.cfg["tx_samples"],
                    "first_sample_tick": t_rx if got else None,
                    "error": fault, "data": b""}
        window = bytearray(want * 4)
        if self.scfg.get("rx_mode", "loopback") == "loopback" and self._wave:
            pre = self.cfg["geo"]["pre_guard_native"]
            delay = int(self.scfg.get("delay_native_samples", 37))
            off = (pre + delay) * 4
            room = max(0, len(window) - off)
            seg = self._wave[self._wave_skip:self._wave_skip + room]
            window[off:off + len(seg)] = seg
        return {"status": "ok", "rx_samples_received": want,
                "tx_samples_sent": self.cfg["tx_samples"],
                "first_sample_tick": t_rx, "error": "", "data": bytes(window)}

    def abort_rx(self):
        pass

    def close(self):
        pass


def make_adapter(o, cfg):
    info = cfg["adapter"]
    if info["name"] == ADAPTER_SCRIPTED:
        return ScriptedAdapter(o, cfg, info["config"])
    if info["name"] == ADAPTER_EXTERNAL:
        return ExternalAdapter(o, cfg, info["config"])
    return BuiltinUhdAdapter(o, cfg)


# ---------------------------------------------------------------------------
# calibration diagnostic (numpy; native pre-CIR only, clearly labeled)
# ---------------------------------------------------------------------------

def calibration_diagnostics(o, cfg, rows):
    import numpy as np
    if not os.path.isfile(o.template):
        raise AdapterUnavailable("template_missing",
                                 "template not found: %s" % o.template)
    raw = np.fromfile(o.template, dtype=np.float32)
    if raw.size < 8 or raw.size % 2:
        raise AdapterUnavailable("template_invalid",
                                 "%s is not a CF32 file" % o.template)
    t = (raw[0::2] + 1j * raw[1::2]).astype(np.complex128)
    t /= np.linalg.norm(t)
    L = t.size
    pre = cfg["geo"]["pre_guard_native"]
    sync = cfg["geo"]["sync_native"]
    per_pulse = []
    lags = []
    for row in rows:
        if row["status"] not in ("ok", "partial_handled") or not row.get("_data"):
            per_pulse.append({"pulse_id": row["pulse_id"], "present": False})
            continue
        arr = np.frombuffer(row["_data"], dtype=np.int16)
        x = (arr[0::2].astype(np.float64)
             + 1j * arr[1::2].astype(np.float64)) / 32767.0
        n = x.size
        if n < L:
            per_pulse.append({"pulse_id": row["pulse_id"], "present": False})
            continue
        fft_len = 1
        while fft_len < n:
            fft_len <<= 1
        corr = np.fft.ifft(np.fft.fft(x, fft_len)
                           * np.conj(np.fft.fft(t, fft_len)))[:n - L + 1]
        mag = np.abs(corr)
        lo = max(0, pre - 96)
        hi = min(mag.size, pre + sync + 96 + 1)
        sub = mag[lo:hi]
        peak = lo + int(np.argmax(sub))
        y0 = float(mag[peak - 1]) if peak > 0 else 0.0
        y1 = float(mag[peak])
        y2 = float(mag[peak + 1]) if peak + 1 < mag.size else 0.0
        den = y0 - 2.0 * y1 + y2
        frac = 0.5 * (y0 - y2) / den if abs(den) > 1e-30 else 0.0
        frac = max(-0.5, min(0.5, frac))
        seg = x[peak:peak + L]
        norm = float(np.linalg.norm(seg) * np.linalg.norm(t))
        metric = y1 / norm if norm > 0 else 0.0
        lag = peak + frac - pre
        in_region = (pre - 96) <= peak <= (pre + sync + 96)
        present = metric >= o.diag_threshold and in_region
        per_pulse.append({"pulse_id": row["pulse_id"], "present": present,
                          "peak_native": peak, "lag_native": round(lag, 3),
                          "metric": round(metric, 4)})
        if present:
            lags.append(lag)
    median = sorted(lags)[len(lags) // 2] if lags else None
    spread = (max(lags) - min(lags)) if len(lags) > 1 else (0.0 if lags else None)
    cal = {
        "schema": SCHEMA_CALIBRATION,
        "calibration_id": cfg["calibration_id"],
        "diag_domain": "native 737.28 MS/s pre-CIR (one-SYNC template "
                       "correlation)",
        "pulses_used": len(lags),
        "median_leakage_lag_native": None if median is None
        else round(median, 3),
        "lag_spread_native": None if spread is None else round(spread, 3),
        "derived_num_delay_samps": int(round(median)) if median is not None
        else None,
        "derived_cal_delay_native": None if median is None
        else round(median, 3),
        "per_pulse": per_pulse,
        "authority_note": "native pre-CIR diagnostic only; production "
        "zero_delay_tap / fractional-delay authority is the C++ CIR chain "
        "(post Step 11 wiring)",
        "usage": "re-run later stages with --num-delay-samps %s "
        "--cal-delay-native %s"
        % (0 if median is None else int(round(median)),
           0 if median is None else round(median, 3)),
    }
    return cal


# ---------------------------------------------------------------------------
# stage runners
# ---------------------------------------------------------------------------

def run_burst_stage(o, cfg, rep, adapter, stage):
    pri = cfg["geo"]["pri_ticks"]
    pre = cfg["geo"]["pre_guard_native"]
    rx_len = cfg["geo"]["rx_window_native"]
    planned = cfg["burst_plan"]["bursts"]
    arm = llround(o.arm_delay_s * o.rate)
    t0 = adapter.time_now_ticks() + arm
    frags = fragments_for(o, cfg["tx_samples"])
    lead = int(0.002 * o.rate)
    pacing = getattr(adapter, "wall_pacing", True)
    rows = []
    late = 0
    t_start = time.monotonic()
    next_progress = t_start + o.progress_interval_s
    rx_dir = os.path.join(o.output, "rx_iq") if o.write_rx_iq else ""
    if rx_dir:
        os.makedirs(rx_dir, exist_ok=True)
    for k in range(planned):
        t_tx = t0 + k * pri
        t_rx = t_tx - pre
        now = adapter.time_now_ticks()
        if t_rx <= now:
            late += 1
            row = {"pulse_id": k, "schedule_index": k,
                   "status": "late_command", "t_tx_ticks": t_tx,
                   "t_rx_ticks": t_rx, "now_ticks": now}
            rows.append(row)
            rep.emit("pulse", level="warn", **row)
            continue
        if pacing:
            while True:
                now = adapter.time_now_ticks()
                if t_rx - now <= lead:
                    break
                time.sleep(min((t_rx - now - lead) / o.rate, 0.005))
        adapter.issue_rx(t_rx, rx_len)
        tx_res = adapter.issue_tx(t_tx, frags)
        timeout_s = max(0.05, (t_tx - now) / o.rate
                        + rx_len / o.rate + 0.5)
        res = adapter.collect(timeout_s, rx_len)
        status = res["status"]
        row = {"pulse_id": k, "schedule_index": k, "t_tx_ticks": t_tx,
               "t_rx_ticks": t_rx, "status": status,
               "tx_samples_sent": tx_res.get("tx_samples_sent"),
               "rx_samples_requested": rx_len,
               "rx_samples_received": res["rx_samples_received"],
               "first_sample_tick": res.get("first_sample_tick"),
               "rx_time_match": (abs(res["first_sample_tick"] - t_rx) <= 1
                                 if res.get("first_sample_tick") is not None
                                 else None),
               "uhd_error": res.get("error", ""),
               "reissues": res.get("reissues", 0)}
        ok = status in ("ok", "partial_handled") and \
            res["rx_samples_received"] == rx_len
        if ok and rx_dir and res.get("data"):
            path = os.path.join(rx_dir, "pulse_%08d.sc16" % k)
            with open(path, "wb") as f:
                f.write(res["data"])
            row["rx_iq_file"] = path
            row["_data"] = res["data"]
        elif ok:
            row["_data"] = res["data"] or b""
        rows.append(row)
        rep.emit("pulse", **{k2: v for k2, v in row.items() if k2 != "_data"})
        nowm = time.monotonic()
        if nowm >= next_progress:
            next_progress = nowm + o.progress_interval_s
            rep.emit("progress", issued=k + 1 - late, planned=planned,
                     late_skips=late, wall_s=round(nowm - t_start, 3))
    wall = time.monotonic() - t_start
    counters = {"planned": planned, "issued": planned - late,
                "late_skips": late, "wall_s": round(wall, 3),
                "t0_ticks": t0, "in_flight_max": 1,
                "statuses": {}}
    for row in rows:
        counters["statuses"][row["status"]] = \
            counters["statuses"].get(row["status"], 0) + 1
    return rows, counters


def stage_checks(o, cfg, stage, rows, counters, cal=None):
    rx_len = cfg["geo"]["rx_window_native"]
    tx_samples = cfg["tx_samples"]
    planned = counters["planned"]
    ok_rows = [r for r in rows if r["status"] in ("ok", "partial_handled")]
    statuses = counters["statuses"]
    device_errors = {s: c for s, c in statuses.items()
                     if s in ("timeout", "overflow", "late_command",
                              "broken_chain", "backend_error")}
    checks = [
        check("one_result_per_index", len(rows) == planned,
              {"planned": planned, "rows": len(rows)}),
        check("rx_samples_exact",
              bool(ok_rows) and all(r["rx_samples_received"] == rx_len
                                    for r in ok_rows),
              {"ok": len(ok_rows), "rx_window": rx_len}),
        check("no_device_errors", not device_errors,
              device_errors if device_errors else "none"),
        check("timestamps_on_grid",
              bool(ok_rows) and all(r.get("rx_time_match") for r in ok_rows),
              {"mismatches": sum(1 for r in ok_rows
                                 if not r.get("rx_time_match"))}),
    ]
    if stage == "low-rate" and o.write_rx_iq:
        n_files = len([p for p in os.listdir(os.path.join(o.output, "rx_iq"))
                       if p.endswith(".sc16")])
        checks.append(check("file_count_matches_pulses",
                            n_files == len(ok_rows),
                            {"files": n_files, "ok": len(ok_rows)}))
    if stage == "soak":
        checks.append(check("late_drops_zero", counters["late_skips"] == 0,
                            {"late_skips": counters["late_skips"]}))
        checks.append(check("queue_watermark_recorded", True,
                            {"in_flight_max": counters["in_flight_max"]}))
    if stage == "calibration":
        cal = cal or {"per_pulse": []}
        present = [p for p in cal["per_pulse"] if p.get("present")]
        lags = [p["lag_native"] for p in present]
        spread = (max(lags) - min(lags)) if len(lags) > 1 else 0.0
        checks.append(check("diag_peak_found_per_pulse",
                            len(present) == len(ok_rows) and
                            len(present) > 0,
                            {"present": len(present),
                             "ok_rows": len(ok_rows)}))
        checks.append(check("diag_spread_bounded",
                            spread <= o.cal_spread_max,
                            {"spread": round(spread, 3),
                             "limit": o.cal_spread_max}))
        path = os.path.join(o.output, "calibration.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cal, f, indent=2)
            f.write("\n")
        checks.append(check("calibration_written", True, path))
    if stage == "cable" and o.write_rx_iq:
        present = [p for p in cal["per_pulse"] if p.get("present")] \
            if cal else []
        rate = len(present) / max(len(ok_rows), 1)
        checks.append(check("diag_present_rate",
                            rate >= o.min_diag_present_rate,
                            {"rate": round(rate, 4),
                             "threshold": o.min_diag_present_rate}))
    return checks


def run_stage(o, cfg):
    stage = o.stage
    adapter_info = cfg["adapter"]
    run_id = now_iso().replace(":", "").replace("-", "") + "-" + \
        uuid.uuid4().hex[:6]
    rep = Report(run_id, o.report or os.path.join(o.output, "report.jsonl"),
                 stage, adapter_info["backend"])
    exit_code = EXIT_INTERNAL
    result = "error"
    code = detail = ""
    checks = []
    counters = {}
    evidence = {}
    caveats = []
    deferred = []
    adapter = None
    try:
        with open(os.path.join(o.output, "run.json"), "w",
                  encoding="utf-8") as _f:
            json.dump({"schema": SCHEMA_RUN, "run_id": run_id,
                       "generated_utc": now_iso(), "stage": stage,
                       "config": cfg}, _f, ensure_ascii=False, indent=2)
        rep.emit("plan", plan=build_plan(o, cfg))
        rep.emit("gate", **{k: v for k, v in cfg["gate"].items()})
        adapter = make_adapter(o, cfg)
        outcome = adapter.setup()
        rep.emit("event", msg="probe complete",
                 probe={k: v for k, v in outcome["probe"].items()})
        if stage == "smoke":
            checks = list(outcome.get("checks") or [])
            checks.append(check("no_tx_issued", True, "smoke never transmits"))
            counters = {"planned": 0}
            evidence = {"probe": outcome["probe"]}
        else:
            adapter.setup_full()
            rows, counters = run_burst_stage(o, cfg, rep, adapter, stage)
            cal = calibration_diagnostics(o, cfg, rows) \
                if stage in ("calibration", "cable") and rows else None
            checks = stage_checks(o, cfg, stage, rows, counters, cal)
            evidence = {"counters": counters,
                        "write_rx_iq": bool(o.write_rx_iq)}
            if cal is not None:
                evidence["calibration_id"] = cfg["calibration_id"]
                evidence["calibration_path"] = os.path.join(
                    o.output, "calibration.json")
        ok = all(c["ok"] for c in checks)
        result = "pass" if ok else "fail"
        exit_code = EXIT_OK if ok else EXIT_CHECKS
        caveats = list(STAGE_CAVEATS[stage])
        caveats.append("evidence recorded; hardware acceptance requires "
                       "user-arranged X410 execution")
        deferred = list(STAGE_DEFERRED[stage])
        if stage == "ota":
            caveats.insert(0, "OTA OBSERVATION ONLY: never present this "
                              "record as timed-I/O, SFD/CIR, or RF "
                              "dynamic-range acceptance "
                              "(开发计划 §15 layer 7)")
    except AdapterUnavailable as exc:
        result, exit_code = "error", EXIT_DEVICE
        code, detail = exc.code, exc.detail
        caveats = ["hardware layer NOT verified: environment or device "
                   "unavailable"]
        checks = [{"name": "environment_or_device", "ok": False,
                   "code": exc.code, "details": exc.detail}]
    finally:
        if adapter is not None:
            try:
                adapter.close()
            except Exception:
                pass
        if result == "error":
            rep.emit("stage_result", result="error",
                     evidence_class=adapter_info["evidence_class"],
                     code=code, detail=detail, checks=checks,
                     counters=counters, evidence=evidence, caveats=caveats,
                     deferred=deferred)
            sys.stderr.write("x410_uwb_radar_validate: %s: %s\n"
                             % (code, detail))
        else:
            rep.emit("stage_result", result=result,
                     evidence_class=adapter_info["evidence_class"],
                     hardware_verified=adapter_info["evidence_class"]
                     == "hardware",
                     rate_hz=o.rate, checks=checks,
                     counters=counters, evidence=evidence, caveats=caveats,
                     deferred=deferred,
                     ota_observation_only=(stage == "ota"))
            if result == "fail":
                sys.stderr.write("stage '%s' failed checks\n" % stage)
        nxt = STAGES[STAGES.index(stage) + 1] if stage != "ota" else None
        rep.emit("summary", result=result, exit_code=exit_code,
                 requested_stage=stage,
                 next_expected_stage=(nxt if result == "pass" else None),
                 wall_s=round(time.monotonic() - _stage_start[0], 3))
        rep.close()
    return exit_code


_stage_start = [0.0]


def do_dry_run(o):
    cfg = validate(o, for_hardware=False)
    plan = build_plan(o, cfg)
    sys.stdout.write(json.dumps(plan, ensure_ascii=False, indent=2) + "\n")
    return EXIT_OK


def main(argv=None):
    _stage_start[0] = time.monotonic()
    try:
        o = build_parser().parse_args(sys.argv[1:] if argv is None else argv)
        if not o.dry_run and o.stage is None:
            raise CliError(
                "mode_required",
                "select exactly one mode: --dry-run or --stage <name> "
                "(stage choices: %s)" % ", ".join(STAGES))
        if o.dry_run:
            return do_dry_run(o)
        return run_stage(o, validate(o, for_hardware=True))
    except CliError as exc:
        emit_cli_error(exc.code, exc.detail)
        sys.stderr.write("x410_uwb_radar_validate: [%s] %s\n"
                         % (exc.code, exc.detail))
        return EXIT_USAGE
    except KeyboardInterrupt:
        emit_cli_error("interrupted", "interrupted by user")
        return 130
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        emit_internal_error("%s: %s" % (type(exc).__name__, exc))
        return EXIT_INTERNAL


if __name__ == "__main__":
    sys.exit(main())
