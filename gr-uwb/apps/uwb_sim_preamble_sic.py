#!/usr/bin/env python3
"""S0 preamble-collision / preamble-only SIC simulation CLI (S0-A baseline).

Builds ``y = s + i + w`` windows on the 998.4 MS/s work grid, runs the
existing sensing consumers (radar CIR-only and full-packet demod) on the
clean (``s + w``) and raw (``y``) pair, and writes the S0 artifacts described
in ``docs/sic/S0_artifacts.md``.

Reuses the geometry, HRP source, PCHIP fractional-delay model and AWGN
convention of ``gr-uwb/apps/uwb_sim_cg400_echo_cir.py`` (which is left
untouched) via the ``sic_preamble_sim`` helper module.  The ``uwb`` bindings
are loaded from ``--build-dir`` — never from a hard-coded ``gr-uwb/build``.

The comm signal is **preamble-only** (code 10, 64 SYNC): it has no
PHR/payload/FCS and is never decoded.  Truth objects live in this generator
side only and are never handed to any consumer.

Example (S0-A baseline):

  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
      --build-dir gr-uwb/build-sic --preset baseline --input-grid work \
      --sic off --consumer both --windows 32 \
      --output gr-uwb/build-sic/sic-results/baseline
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import platform
import resource
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np
import pmt
from gnuradio import gr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sic_preamble_sim as S  # noqa: E402

SCHEMA_VERSION = 1
TERMINAL_STATUS_OK = ("success", "ok", "pass")
SFD_MODE = S.SFD_MODE
SPS = S.SPS


# ---------------------------------------------------------------------------
# PMT helpers
# ---------------------------------------------------------------------------
def pmt_to_py(v):
    if pmt.is_symbol(v):
        return pmt.symbol_to_string(v)
    if pmt.is_bool(v):
        return bool(pmt.to_bool(v))
    if pmt.is_uint64(v):
        return int(pmt.to_uint64(v))
    if pmt.is_integer(v):
        return int(pmt.to_long(v))
    if pmt.is_real(v):
        return float(pmt.to_double(v))
    return v


def meta_dict(meta):
    out = {}
    if not pmt.is_dict(meta):
        return out
    items = pmt.dict_items(meta)
    for i in range(pmt.length(items)):
        kv = pmt.nth(i, items)
        out[pmt.symbol_to_string(pmt.car(kv))] = pmt.cdr(kv)
    return out


def dget(d, key, default=None):
    v = d.get(key, pmt.PMT_NIL)
    if v is pmt.PMT_NIL:
        return default
    got = pmt_to_py(v)
    return default if got is pmt.PMT_NIL else got


def dnum(d, key, default=None):
    v = dget(d, key, None)
    return default if v is None else v


def put_str(meta, key, val):
    return pmt.dict_add(meta, pmt.intern(key), pmt.intern(str(val)))


def put_int(meta, key, val):
    return pmt.dict_add(meta, pmt.intern(key), pmt.from_long(int(val)))


def put_float(meta, key, val):
    return pmt.dict_add(meta, pmt.intern(key), pmt.from_double(float(val)))


def put_bool(meta, key, val):
    return pmt.dict_add(meta, pmt.intern(key), pmt.from_bool(bool(val)))


# ---------------------------------------------------------------------------
# Result sink (no built-in Python sink exists for these blocks)
# ---------------------------------------------------------------------------
class ResultSink(gr.basic_block):
    """Bounded consumer PDU sink.

    Keeps the running ``count`` of every ``(meta, payload)`` PDU plus the most
    recent dict (``latest``) and the first ``keep_iq`` dicts (``retained``).
    Older IQ arrays are dropped so a long run does not grow without bound; the
    per-window stats are extracted from ``latest`` before the next PDU arrives.
    """

    def __init__(self, name="result_sink", keep_iq=0):
        gr.basic_block.__init__(self, name=name, in_sig=None, out_sig=None)
        self.message_port_register_in(pmt.intern("in"))
        self.set_msg_handler(pmt.intern("in"), self._handle)
        self.keep_iq = max(0, int(keep_iq))
        self.count = 0
        self.latest = None
        self.retained = []
        self.dropped_iq = 0

    def _handle(self, msg):
        t_ns = time.monotonic_ns()
        if not pmt.is_pair(msg):
            d = {"__error": "non_pair", "__t_ns": t_ns}
        else:
            meta = pmt.car(msg)
            payload = pmt.cdr(msg)
            d = meta_dict(meta)
            if pmt.is_u8vector(payload):
                d["__u8"] = bytes(pmt.u8vector_elements(payload))
            elif pmt.is_c32vector(payload):
                d["__c32"] = np.array(pmt.c32vector_elements(payload),
                                      dtype=np.complex64)
            elif pmt.is_s16vector(payload):
                d["__s16"] = np.array(pmt.s16vector_elements(payload),
                                      dtype=np.int16)
            d["__t_ns"] = t_ns
        self.count += 1
        if len(self.retained) < self.keep_iq:
            self.retained.append(d)
        elif self.keep_iq == 0 or self.count > self.keep_iq:
            self.dropped_iq += 1
        # Replace the previous "latest" so its IQ array is released.
        self.latest = d


DEMOD_STAGES = [
    ("stage_timing_us", "timing", "cxx", "demod_timing"),
    ("stage_timing_coarse_us", "timing_coarse", "cxx", "demod_timing_child"),
    ("stage_timing_fine_track_us", "timing_fine_track", "cxx",
     "demod_timing_child"),
    ("stage_cfo_us", "cfo", "cxx", "demod"),
    ("stage_sfd_us", "sfd", "cxx", "demod"),
    ("sfd_bootstrap_us", "sfd_bootstrap", "cxx", "demod_sfd_child"),
    ("sfd_tail_retrack_us", "sfd_tail_retrack", "cxx", "demod_sfd_child"),
    ("sfd_cfo_refit_us", "sfd_cfo_refit", "cxx", "demod_sfd_child"),
    ("sfd_final_us", "sfd_final", "cxx", "demod_sfd_child"),
    ("sfd_anchor_other_us", "sfd_anchor_other", "cxx", "demod_sfd_child"),
    ("stage_cir_us", "cir", "cxx", "demod"),
    ("cir_estimate_us", "cir_estimate", "cxx", "demod_cir_child"),
    ("cir_softfir_us", "soft_fir", "cxx", "demod_cir_child"),
    ("cir_postprocess_us", "postprocess", "cxx", "demod_cir_child"),
    ("stage_ns_sfd_us", "ns_sfd", "cxx", "demod"),
    ("stage_phr_us", "phr", "cxx", "demod"),
    ("stage_payload_us", "payload", "cxx", "demod"),
    ("stage_total_us", "total", "cxx", "demod"),
]

# SIC core stage timers (scope=sic, tier=cxx).  Children of core_total_us.
SIC_CORE_STAGES = [
    ("sic_detect_coarse_us", "detect_coarse"),
    ("sic_detect_fine_us", "detect_fine"),
    ("sic_cfo_us", "cfo"),
    ("sic_cir_us", "cir"),
    ("sic_path_select_us", "path_select"),
    ("sic_fractional_refine_us", "fractional_refine"),
    ("sic_matrix_build_us", "matrix_build"),
    ("sic_gain_fit_us", "gain_fit"),
    ("sic_reconstruct_us", "reconstruct"),
    ("sic_validate_commit_us", "validate_commit"),
    ("core_total_us", "core_total"),
]
# SIC adapter timers (scope=adapter, tier=adapter).
SIC_ADAPTER_STAGES = [
    ("handler_us", "handler"),
    ("queue_wait_us", "queue_wait"),
    ("output_wrap_publish_us", "output_wrap_publish"),
    ("completion_latency_us", "completion_latency"),
]


def recon_pair(i_true, i_hat, lo, hi):
    """NMSE / suppression over [lo,hi) with the verifier's exact arithmetic.

    Returns (nmse, suppression_db, lo, hi) or None when the interval is empty
    or the reference energy is zero.  Noise is NOT part of i_true (the clean
    injected comm), so the metric excludes AWGN.
    """
    n = int(min(len(i_true), len(i_hat)))
    lo = max(0, int(lo))
    hi = min(n, int(hi))
    if hi <= lo:
        return None
    a = i_true[lo:hi].astype(np.complex128)
    b = i_hat[lo:hi].astype(np.complex128)
    den = float(np.vdot(a, a).real)
    if den <= 0.0:
        return None
    d = b - a
    num = float(np.vdot(d, d).real)
    nmse = num / den
    sup = (-10.0 * math.log10(nmse)) if num > 0.0 else 60.0
    return float(nmse), float(sup), int(lo), int(hi)


def px_median(values):
    vals = [float(v) for v in values if v is not None and
            math.isfinite(float(v))]
    return float(np.median(np.asarray(vals, dtype=float))) if vals else None


def px_p95(values):
    vals = [float(v) for v in values if v is not None and
            math.isfinite(float(v))]
    return float(np.percentile(np.asarray(vals, dtype=float), 95)) \
        if vals else None


# ---------------------------------------------------------------------------
# Preset / config
# ---------------------------------------------------------------------------
def load_preset(repo, preset):
    path = os.path.join(repo, "testdata", "sic_preamble", "cases",
                        "%s.json" % preset)
    if not os.path.isfile(path):
        raise SystemExit("unknown preset %r: missing %s" % (preset, path))
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def materialize_seeds(spec, count):
    start = int(spec["start"])
    step = int(spec.get("step", 1))
    total = int(spec["count"])
    return [start + i * step for i in range(min(count, total))], total


def stdin_hash(obj):
    blob = json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()


def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def ldd_lines(so_path):
    try:
        out = subprocess.run(["ldd", so_path], capture_output=True, text=True,
                             timeout=30)
        return [ln.strip() for ln in out.stdout.splitlines() if ln.strip()]
    except Exception as e:  # pragma: no cover
        return ["ldd failed: %s" % e]


def volk_version():
    try:
        import volk  # type: ignore
        for attr in ("version", "__version__"):
            if hasattr(volk, attr):
                v = getattr(volk, attr)
                return str(v() if callable(v) else v)
    except Exception:
        pass
    for cmd in (["volk-config-info", "--version"],
                ["volk_profile", "--version"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True,
                                 timeout=10)
            if out.returncode == 0 and out.stdout.strip():
                return out.stdout.strip().splitlines()[0]
        except Exception:
            continue
    return None


def cmake_cache_value(build_dir, key):
    path = os.path.join(build_dir, "CMakeCache.txt")
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith(key + ":"):
                return line.split("=", 1)[1].strip()
    return None


def cpu_model():
    try:
        with open("/proc/cpuinfo", "r", encoding="utf-8",
                  errors="replace") as f:
            for line in f:
                if line.lower().startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except Exception:
        pass
    return platform.processor() or platform.machine()


# ---------------------------------------------------------------------------
# SIC adapter construction (feature-detected) + source credit tracking
# ---------------------------------------------------------------------------
_SIC_REQUIRED = ("sync", "preamble", "comm_code", "sync_reps",
                 "samples_per_symbol", "sample_rate", "k_paths", "fit_reps",
                 "validation_reps", "roi_lo", "roi_hi", "coarse_stride",
                 "mode", "detect_threshold", "validate_threshold")
_SIC_OPTIONAL = ("in_flight_max", "iq_budget_bytes", "queue_capacity",
                 "publish_reconstruction", "post_tail", "enable_fractional",
                 "enable_cfo", "path_min_spacing", "max_window_samples")


def pybind_param_names(fn):
    """Best-effort parameter names of a pybind11 callable (from __doc__)."""
    import re
    doc = getattr(fn, "__doc__", "") or ""
    head = doc.split("->", 1)[0]
    return set(re.findall(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*(?::|=|,|\))",
                          head))


def sic_adapter_supported_params(uwb):
    try:
        names = pybind_param_names(uwb.preamble_sic.make_from_templates)
    except Exception:
        names = set()
    return sorted(n for n in _SIC_OPTIONAL if n in names), names


def call_sic_make_from_templates(uwb, vals, extras):
    """Build the adapter, passing new make() params only when supported."""
    fn = uwb.preamble_sic.make_from_templates
    names = pybind_param_names(fn)
    used = {}
    if all(n in names for n in _SIC_REQUIRED):
        kwargs = {n: vals[n] for n in _SIC_REQUIRED}
        for n in _SIC_OPTIONAL:
            if n in names and n in extras:
                kwargs[n] = extras[n]
                used[n] = extras[n]
        return fn(**kwargs), used
    # Legacy positional fallback (pre-feature-detection binding).
    for n in _SIC_OPTIONAL:
        if n in extras:
            used[n] = extras[n]
    return fn(*[vals[n] for n in _SIC_REQUIRED],
              int(extras.get("in_flight_max", 8)),
              int(extras.get("iq_budget_bytes", 64 << 20)),
              int(extras.get("queue_capacity", 8)),
              bool(extras.get("publish_reconstruction", True))), used


class CreditTracker:
    """Explicit SOURCE credit accounting (indication §4).

    The simulator source must never publish more than `in_flight_max` windows
    that are published but not yet terminal, nor exceed the total-IQ budget.
    `acquire` is called immediately before publishing; a failure means the
    source would miss its planned submission at that moment and is recorded as
    a miss (never masked by lowering the input rate).
    """

    def __init__(self, in_flight_max, iq_budget_bytes):
        self.in_flight_max = int(in_flight_max)
        self.iq_budget_bytes = int(iq_budget_bytes)
        self.in_flight = 0
        self.pending_bytes = 0
        self.offered = 0
        self.admitted = 0
        self.missed_admission = 0
        self.inflight_high_watermark = 0
        self.pending_bytes_peak = 0
        self.by_reason = {"in_flight_limit": 0, "iq_budget": 0}

    def would_admit(self, nbytes):
        if self.in_flight >= self.in_flight_max:
            return False, "in_flight_limit"
        if self.pending_bytes + int(nbytes) > self.iq_budget_bytes:
            return False, "iq_budget"
        return True, None

    def acquire(self, nbytes):
        """Reserve a credit; returns (ok, reason)."""
        self.offered += 1
        ok, reason = self.would_admit(nbytes)
        if not ok:
            self.missed_admission += 1
            self.by_reason[reason] += 1
            return False, reason
        self.in_flight += 1
        self.pending_bytes += int(nbytes)
        self.admitted += 1
        self.inflight_high_watermark = max(self.inflight_high_watermark,
                                           self.in_flight)
        self.pending_bytes_peak = max(self.pending_bytes_peak,
                                      self.pending_bytes)
        return True, None

    def release(self, nbytes):
        if self.in_flight > 0:
            self.in_flight -= 1
        self.pending_bytes = max(0, self.pending_bytes - int(nbytes))

    def snapshot(self, planned_ns):
        return {
            "source_offered": self.offered,
            "source_admitted": self.admitted,
            "source_missed_admission": self.missed_admission,
            "source_miss_reasons": dict(self.by_reason),
            "source_inflight_at_publish": self.in_flight,
            "source_pending_bytes_at_publish": self.pending_bytes,
            "source_planned_publish_ns": int(planned_ns),
            "in_flight_max": self.in_flight_max,
            "iq_budget_bytes": self.iq_budget_bytes,
        }


# ---------------------------------------------------------------------------
# Simulator runtime
# ---------------------------------------------------------------------------
class SimRuntime:
    """Owns the bounded top_block + consumers for a whole run."""

    def __init__(self, uwb, *, tpl_path, consumers, windows,
                 demod_softchip="bypass", sic=None, keep_iq=4):
        self.uwb = uwb
        self.consumers = consumers
        self.want_cir = consumers in ("cir", "both")
        self.want_demod = consumers in ("demod", "both")
        self.keep_iq = max(0, int(keep_iq))
        self.sic = None
        self.sic_params_used = {}
        self.sink_sic_out = None
        self.sink_sic_ihat = None
        self.tb = gr.top_block("uwb_sim_preamble_sic")
        self.cir = None
        self.demod = None
        self.sink_cir = None
        self.sink_demod = None
        qcap = max(64, 2 * int(windows) + 8)
        rake = 0
        mode = demod_softchip
        if self.want_cir:
            self.cir = uwb.radar_cir_estimator(
                tpl_path, 64, SFD_MODE, 9, 16, 100, 10, 0, 128, 32, 0.12,
                0.02, True, qcap)
            self.sink_cir = ResultSink("cir_sink", self.keep_iq)
            self.tb.msg_connect((self.cir, "cir"), (self.sink_cir, "in"))
        if self.want_demod:
            self.demod = uwb.realtime_demodulator(
                tpl_path, 1, qcap, SFD_MODE, rake, mode, 9, 64, 14)
            self.sink_demod = ResultSink("demod_sink", self.keep_iq)
            self.tb.msg_connect((self.demod, "result"),
                                (self.sink_demod, "in"))
        if sic is not None:
            vals = {
                "sync": sic["sync"].astype(np.complex64).tolist(),
                "preamble": sic["preamble"].astype(np.complex64).tolist(),
                "comm_code": int(sic["comm_code"]),
                "sync_reps": int(sic["sync_reps"]),
                "samples_per_symbol": int(S.SPS),
                "sample_rate": float(S.WORK_HZ),
                "k_paths": int(sic["k_paths"]),
                "fit_reps": int(sic["fit_reps"]),
                "validation_reps": int(sic["validation_reps"]),
                "roi_lo": int(sic["roi"][0]),
                "roi_hi": int(sic["roi"][1]),
                "coarse_stride": int(sic["coarse_stride"]),
                "mode": str(sic["mode"]),
                "detect_threshold": float(sic["detect_threshold"]),
                "validate_threshold": float(sic["validate_threshold"]),
            }
            extras = {
                "in_flight_max": int(sic.get("in_flight_max", 8)),
                "iq_budget_bytes": int(sic.get("iq_budget_bytes", 64 << 20)),
                "queue_capacity": int(sic.get("queue_capacity", 8)),
                "publish_reconstruction": True,
                "post_tail": int(sic.get("post_tail", 100)),
                "enable_fractional": bool(sic.get("enable_fractional", True)),
                "enable_cfo": bool(sic.get("enable_cfo", True)),
                "path_min_spacing": int(sic.get("path_min_spacing", 0)),
                "max_window_samples": int(sic.get("max_window_samples", 0)),
            }
            self.sic, self.sic_params_used = \
                call_sic_make_from_templates(uwb, vals, extras)
            self.sink_sic_out = ResultSink("sic_out_sink", self.keep_iq)
            self.sink_sic_ihat = ResultSink("sic_ihat_sink", self.keep_iq)
            self.tb.msg_connect((self.sic, "out"), (self.sink_sic_out, "in"))
            self.tb.msg_connect((self.sic, "i_hat"),
                                (self.sink_sic_ihat, "in"))

    def start(self):
        self.tb.start()

    def stop(self):
        self.tb.stop()
        self.tb.wait()

    def posted(self):
        n = 0
        if self.want_cir:
            n = self.sink_cir.count
        if self.want_demod:
            n = self.sink_demod.count
        return n

    def counts(self):
        return (self.sink_cir.count if self.want_cir else -1,
                self.sink_demod.count if self.want_demod else -1)

    def post_cir(self, meta, iq):
        self.cir.to_basic_block()._post(
            pmt.intern("rx"), pmt.cons(meta, pmt.init_c32vector(len(iq),
                                                                iq.tolist())))

    def post_demod(self, meta, iq):
        self.demod.to_basic_block()._post(
            pmt.intern("samples"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def post_sic_input(self, meta, iq):
        self.sic.to_basic_block()._post(
            pmt.intern("in"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def post_sic_truth(self, truth):
        self.sic.to_basic_block()._post(pmt.intern("truth"), truth)

    def sic_counts(self):
        if self.sic is None:
            return (-1, -1)
        return (self.sink_sic_out.count, self.sink_sic_ihat.count)

    def wait_sic(self, n_out, n_ihat, deadline_s=120.0):
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            o, h = self.sic_counts()
            if o >= n_out and h >= n_ihat:
                return True
            time.sleep(0.002)
        return False

    def wait_for(self, n_cir, n_demod, deadline_s=120.0):
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            c, dd = self.counts()
            ok_c = (not self.want_cir) or c >= n_cir
            ok_d = (not self.want_demod) or dd >= n_demod
            if ok_c and ok_d:
                return True
            time.sleep(0.002)
        return False


def run_canonical_cir_reference(uwb, repo, tpl_path, out):
    """G0: feed the canonical radar RX to the CIR block; compare to golden."""
    rx = os.path.join(repo, "testdata", "uwb_radar", "rx_clean_998p4.cf32")
    gold = os.path.join(repo, "testdata", "uwb_radar",
                        "cir_norm_clean_radar.cf32")
    if not (os.path.isfile(rx) and os.path.isfile(gold)):
        return {"ran": False, "reason": "canonical fixture missing"}
    iq = np.fromfile(rx, dtype=np.complex64)
    est = uwb.radar_cir_estimator(tpl_path, 64, SFD_MODE, 9, 16, 100, 10, 0,
                                  128, 32, 0.12, 0.02, True, 8)
    sink = ResultSink("canon_sink")
    tb = gr.top_block("canon_ref")
    tb.msg_connect((est, "cir"), (sink, "in"))
    tb.start()
    meta = pmt.make_dict()
    meta = put_int(meta, "pulse_id", 0)
    meta = put_int(meta, "pre_guard_samples", 1997)
    meta = put_int(meta, "window_start_sample", 0)
    meta = put_float(meta, "sample_rate", S.WORK_HZ)
    meta = put_str(meta, "sfd_mode", SFD_MODE)
    meta = put_int(meta, "sync_repetitions", 64)
    meta = put_int(meta, "code_index", 9)
    meta = put_float(meta, "calibration_delay_native_samples", 0.0)
    est.to_basic_block()._post(
        pmt.intern("rx"), pmt.cons(meta, pmt.init_c32vector(len(iq),
                                                            iq.tolist())))
    t0 = time.time()
    while time.time() - t0 < 30 and sink.count == 0:
        time.sleep(0.01)
    tb.stop()
    tb.wait()
    if sink.count == 0:
        return {"ran": False, "reason": "no CIR result"}
    d = sink.latest
    raw = d.get("__c32", np.zeros(0, dtype=np.complex64))
    gnorm = np.fromfile(gold, dtype=np.complex64)
    norm = d.get("normalized_taps")
    norm_arr = (np.array(pmt.c32vector_elements(norm), dtype=np.complex64)
                if norm is not None else None)
    res = {
        "ran": True,
        "status": dget(d, "status"),
        "peak_tap": dnum(d, "peak_tap"),
        "tap_count": dnum(d, "tap_count"),
        "golden_tap_count": int(gnorm.size),
        "max_abs_diff_norm": (float(np.max(np.abs(norm_arr - gnorm)))
                              if norm_arr is not None and
                              norm_arr.size == gnorm.size else None),
        "golden_peak_tap": int(np.argmax(np.abs(gnorm))),
        "raw_l2_norm": dnum(d, "raw_l2_norm"),
    }
    out["g0"]["cir_canonical_reference"] = res
    return res


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------
def summarize(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return {"n": 0}
    a = np.asarray(vals, dtype=np.float64)
    return {
        "n": int(a.size),
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "max": float(a.max()),
    }


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


def _mag(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, (int, float)):
        return abs(float(v))
    if isinstance(v, (list, tuple)) and len(v) == 2:
        try:
            return math.hypot(float(v[0]), float(v[1]))
        except (TypeError, ValueError):
            return None
    return None


def _isnum(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


def pooled_effect(windows):
    """Effect aggregate over non-truth windows (verifier's exact formula)."""
    sups, nmses, shifts, amps = [], [], [], []
    n_truth = 0
    for w in windows:
        st = w.get("sic_state") or {}
        if st.get("uses_truth") is True:
            n_truth += 1
            continue
        rec = w.get("recon")
        if isinstance(rec, dict):
            if _isnum(rec.get("suppression_db")):
                sups.append(float(rec["suppression_db"]))
            if _isnum(rec.get("nmse")):
                nmses.append(float(rec["nmse"]))
        c, s = w.get("clean"), w.get("sic")
        if isinstance(c, dict) and isinstance(s, dict):
            if isinstance(c.get("cir_peak_tap"), int) and \
                    isinstance(s.get("cir_peak_tap"), int):
                shifts.append(abs(s["cir_peak_tap"] - c["cir_peak_tap"]))
            a, b = _mag(c.get("target_amp")), _mag(s.get("target_amp"))
            if a and b and a > 0 and b > 0:
                amps.append(20.0 * math.log10(b / a))
    out = {
        "n_windows": len(sups),
        "n_truth_excluded": n_truth,
        "median_suppression_db": (float(np.median(sups)) if sups else None),
        "frac_ge_3db": (sum(1.0 for s in sups if s >= 3.0) / len(sups)
                        if sups else None),
        "nmse_median": (float(np.median(nmses)) if nmses else None),
        "peak_shift_max": (max(shifts) if shifts else None),
        "target_amp_err_db_p95": (float(np.percentile(np.asarray(amps), 95))
                                  if amps else None),
        "target_amp_abs_err_db_p95": (
            float(np.percentile(np.abs(np.asarray(amps)), 95))
            if amps else None),
    }
    return out


def wilson_ci(k, n, z=1.96):
    if n <= 0:
        return (None, None)
    p = k / n
    d = 1.0 + z * z / n
    c = p + z * z / (2 * n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (max(0.0, (c - half) / d), min(1.0, (c + half) / d))


def _invoke_main(child_argv):
    try:
        main(child_argv)
        return 0
    except SystemExit as e:
        code = e.code
        if code is None:
            return 0
        return int(code) if isinstance(code, int) else 1


def _run_child(child_argv, log_path):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as f, \
            contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
        return _invoke_main(child_argv)


def _sanitize(v):
    return ("%g" % v).replace("-", "m").replace(".", "p")


def pick_seed_set(preset, wanted, default):
    """Choose the seed list that can supply `wanted` windows."""
    specs = (preset.get("generator", {}) or {}).get("seeds", {})
    avail = {k: int(v.get("count", 0)) for k, v in specs.items()
             if isinstance(v, dict)}
    if avail.get(default, 0) >= wanted:
        return default
    for key in ("acceptance", "calibration"):
        if avail.get(key, 0) >= wanted:
            return key
    raise SystemExit("no seed list can supply %d windows (available=%r)"
                     % (wanted, avail))


def build_child_argv(args, output, isr=None, offset=None, cfo=None,
                     channel=None, case=None, windows=None,
                     seed_set=None):
    argv = ["--build-dir", args.build_dir, "--preset", args.preset,
            "--input-grid", args.input_grid, "--sic", args.sic,
            "--consumer", args.consumer, "--output", output,
            "--seed-set", seed_set or args.seed_set,
            "--interference", args.interference,
            "--fixtures", str(args.fixtures),
            "--sic-detect-threshold", str(args.sic_detect_threshold),
            "--sic-validate-threshold", str(args.sic_validate_threshold),
            "--sic-in-flight-max", str(args.sic_in_flight_max),
            "--sic-iq-budget-mib", str(args.sic_iq_budget_mib),
            "--keep-iq", str(args.keep_iq)]
    if windows is not None:
        argv += ["--windows", str(int(windows))]
    elif args.windows:
        argv += ["--windows", str(args.windows)]
    if case is not None:
        argv += ["--case", case]
    if isr is not None:
        argv += ["--isr-db", str(isr)]
    if offset is not None:
        argv += ["--start-offset", str(offset)]
    if cfo is not None:
        argv += ["--cfo-hz", str(cfo)]
    if channel is not None:
        argv += ["--comm-channel", channel]
    return argv


def _read_json(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _read_windows(path):
    out = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            for ln in f:
                ln = ln.strip()
                if ln:
                    out.append(json.loads(ln))
    except Exception:
        pass
    return out


def run_sweep(args, repo, argv):
    """§8.2 main effect matrix: one per-cell run per axis combination."""
    preset = load_preset(repo, "main")
    gen = preset.get("generator", {})
    isr_axis = [float(x) for x in gen.get("isr_db_axis", [10.0])]
    off_axis = [float(x) for x in gen.get("start_offset_work_axis", [0.0])]
    cfo_axis = [float(x) for x in gen.get("cfo_hz_axis", [0.0])]
    ch_axis = [str(x) for x in gen.get("comm_channel_axis", ["single_int"])]
    if args.isr_db is not None:
        isr_axis = [float(args.isr_db)]
    if args.start_offset is not None:
        off_axis = [float(args.start_offset)]
    if args.cfo_hz is not None:
        cfo_axis = [float(args.cfo_hz)]
    if args.comm_channel is not None:
        ch_axis = ["single_int" if args.comm_channel == "single"
                   else "four_path"]
    if args.seed_set not in ("calibration", "acceptance"):
        raise SystemExit("bad seed set")
    per_cell = args.windows if args.windows else int(
        preset.get("windows_per_cell", 100))
    seed_set = pick_seed_set(preset, per_cell, args.seed_set)
    cells = [(i, o, c, ch) for i in isr_axis for o in off_axis
             for c in cfo_axis for ch in ch_axis]
    os.makedirs(args.output, exist_ok=True)
    print("sweep: %d cells x %d windows = %d windows (preset main)"
          % (len(cells), per_cell, len(cells) * per_cell), flush=True)
    rows = []
    all_windows = []
    nominal = None
    t_all = time.time()
    for idx, (isr, off, cfo, ch) in enumerate(cells):
        cid = "c%02d_isr%s_off%s_cfo%s_%s" % (
            idx, _sanitize(isr), _sanitize(off), _sanitize(cfo),
            "single" if ch == "single_int" else "four")
        cout = os.path.join(args.output, "cells", cid)
        ca = build_child_argv(args, cout, isr, off, cfo,
                              "single" if ch == "single_int" else "four",
                              windows=per_cell, seed_set=seed_set)
        rc = _run_child(ca, os.path.join(cout, "run.log"))
        summ = _read_json(os.path.join(cout, "summary.json")) or {}
        wins = _read_windows(os.path.join(cout, "windows.jsonl"))
        all_windows.extend(wins)
        eff = summ.get("effect", {}) or {}
        row = {
            "cell_id": cid, "cell_index": idx,
            "isr_db": isr, "start_offset_work": off, "cfo_hz": cfo,
            "comm_channel": ch,
            "windows": len(wins), "returncode": rc,
            "median_suppression_db": eff.get("median_suppression_db"),
            "frac_ge_3db": eff.get("frac_ge_3db"),
            "target_amp_abs_err_db_p95":
                eff.get("target_amp_abs_err_db_p95"),
            "peak_shift_max": eff.get("peak_shift_max"),
            "nmse_median": eff.get("nmse_median"),
            "holdout_suppression_db_median":
                eff.get("holdout_suppression_db_median"),
            "holdout_nmse_median": eff.get("holdout_nmse_median"),
            "eff_pass": (summ.get("gates", {}) or {}).get("eff_pass"),
            "sic_ok": (summ.get("demod", {}) or {}).get("sic_ok"),
            "raw_ok": (summ.get("demod", {}) or {}).get("raw_ok"),
        }
        # nominal cell for the primary gate: isr 10, offset 0, cfo 0, single
        if (abs(isr - 10.0) < 1e-9 and abs(off) < 1e-12 and
                abs(cfo) < 1e-12 and ch == "single_int"):
            nominal = row
        rows.append(row)
        print("  cell %2d/%d %-38s n=%3d med=%s frac3=%s amp95=%s "
              "shift=%s nmse=%s eff_pass=%s rc=%d"
              % (idx + 1, len(cells), cid, row["windows"],
                 _fmt(row["median_suppression_db"]),
                 _fmt(row["frac_ge_3db"]),
                 _fmt(row["target_amp_abs_err_db_p95"]),
                 row["peak_shift_max"], _fmt(row["nmse_median"]),
                 row["eff_pass"], rc), flush=True)

    overall = pooled_effect(all_windows)
    matrix = {
        "preset": "main", "mode": "sweep",
        "axis": {"isr_db": isr_axis, "start_offset_work": off_axis,
                 "cfo_hz": cfo_axis, "comm_channel": ch_axis},
        "windows_per_cell": per_cell,
        "n_cells": len(cells),
        "cells": rows,
        "overall": overall,
        "nominal_cell": nominal,
        "wall_s": time.time() - t_all,
        "note": ("effect_aggregates pooled over all non-truth windows; "
                 "per-cell medians are computed by the child runs. eff_pass is "
                 "per-cell and only meaningful for the main preset."),
    }
    write_json(os.path.join(args.output, "main_matrix.json"), matrix)
    with open(os.path.join(args.output, "main_matrix.csv"), "w",
              encoding="utf-8") as f:
        cols = ["cell_id", "isr_db", "start_offset_work", "cfo_hz",
                "comm_channel", "windows", "median_suppression_db",
                "frac_ge_3db", "target_amp_abs_err_db_p95", "peak_shift_max",
                "nmse_median", "holdout_suppression_db_median",
                "holdout_nmse_median", "eff_pass", "sic_ok", "raw_ok",
                "returncode"]
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join("" if r.get(c) is None else str(r.get(c))
                             for c in cols) + "\n")
        f.write(",".join("" if overall.get(c) is None else str(overall.get(c))
                         for c in cols) + "\n")
    n_pass = sum(1 for r in rows if r.get("eff_pass") is True)
    print("sweep done: %d cells, %d windows, eff_pass %d/%d, wall %.1fs"
          % (len(cells), len(all_windows), n_pass, len(cells),
             matrix["wall_s"]), flush=True)
    print("overall:", json.dumps(overall, sort_keys=True), flush=True)
    print("nominal cell:", json.dumps(nominal, sort_keys=True), flush=True)
    return 0


def _fmt(v, nd=3):
    return "n/a" if v is None else ("%%.%df" % nd) % float(v)


def run_negative(args, repo, argv):
    """Negative set: per-case runs + false-detect/false-commit table."""
    preset = load_preset(repo, "negative")
    cases = preset.get("generator", {}).get("cases", [])
    per_case = args.windows if args.windows else 1000
    seed_set = pick_seed_set(preset, per_case, args.seed_set)
    # Cases for which there is NO actionable interferer to cancel; a commit
    # there is a false commit.  Other cases carry a real interferer.
    expected_negative = {"no_interference", "no_overlap", "wrong_code11"}
    os.makedirs(args.output, exist_ok=True)
    print("negative: %d cases x %d windows" % (len(cases), per_case),
          flush=True)
    rows = []
    fp = fn = 0
    bypass_n = bypass_ok = 0
    for idx, case in enumerate(cases):
        cid = str(case.get("id", "case%d" % idx))
        cout = os.path.join(args.output, "cases", cid)
        ca = build_child_argv(args, cout, case=cid, windows=per_case,
                              seed_set=seed_set)
        rc = _run_child(ca, os.path.join(cout, "run.log"))
        summ = _read_json(os.path.join(cout, "summary.json")) or {}
        wins = _read_windows(os.path.join(cout, "windows.jsonl"))
        n = len(wins)
        n_det = sum(1 for w in wins
                    if (w.get("sic_state") or {}).get("detected") is True)
        n_com = sum(1 for w in wins
                    if (w.get("sic_state") or {}).get("committed") is True)
        sic = summ.get("sic") or {}
        bbi = (sic.get("bypass_bit_identical") or {})
        bypass_n += int(bbi.get("n_bypassed") or 0)
        bypass_ok += int(bbi.get("n_bit_identical") or 0)
        is_neg = cid in expected_negative
        if is_neg:
            fp += n_det
            fn += n_com
        row = {
            "case": cid, "expected_negative": is_neg,
            "windows": n, "returncode": rc,
            "detected": n_det, "committed": n_com,
            "false_detect": n_det if is_neg else None,
            "false_commit": n_com if is_neg else None,
            "detect_rate": (n_det / n) if n else None,
            "commit_rate": (n_com / n) if n else None,
            "bypassed": bbi.get("n_bypassed"),
            "bypass_bit_identical": bbi.get("n_bit_identical"),
            "bypass_all_identical": bbi.get("all_identical"),
            "isr_db": (summ.get("config", {}) or {}).get("isr_db"),
            "returncode_ok": rc == 0,
        }
        rows.append(row)
        print("  case %-26s n=%4d det=%4d commit=%4d neg=%s bypass=%s/%s rc=%d"
              % (cid, n, n_det, n_com, is_neg,
                 bbi.get("n_bit_identical"), bbi.get("n_bypassed"), rc),
              flush=True)

    neg_n = sum(r["windows"] for r in rows if r["expected_negative"])
    overall = {
        "negative_windows": neg_n,
        "false_detect": fp,
        "false_commit": fn,
        "false_detect_rate": (fp / neg_n) if neg_n else None,
        "false_commit_rate": (fn / neg_n) if neg_n else None,
        "false_detect_ci95": wilson_ci(fp, neg_n),
        "false_commit_ci95": wilson_ci(fn, neg_n),
        "gate_false_commit_max": 0.01,
        "gate_windows_min": 1000,
        "gate_pass": bool(neg_n >= 1000 and
                          (neg_n > 0 and fn / neg_n <= 0.01)),
        "bypass_windows": bypass_n,
        "bypass_bit_identical": bypass_ok,
        "bypass_all_identical": bool(bypass_n == 0 or bypass_n == bypass_ok),
        "expected_negative_cases": sorted(expected_negative),
        "ci_method": "Wilson score interval, z=1.96 (95%)",
        "note": ("false-detect/commit are counted only on the "
                 "no-actionable-interferer cases (no_interference, no_overlap, "
                 "wrong_code11); never zero-risk claimed"),
    }
    write_json(os.path.join(args.output, "negative_table.json"),
               {"cases": rows, "overall": overall})
    with open(os.path.join(args.output, "negative_table.csv"), "w",
              encoding="utf-8") as f:
        cols = ["case", "expected_negative", "windows", "detected",
                "committed", "false_detect", "false_commit", "detect_rate",
                "commit_rate", "bypassed", "bypass_bit_identical",
                "bypass_all_identical", "returncode"]
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join("" if r.get(c) is None else str(r.get(c))
                             for c in cols) + "\n")
    print("negative done:", json.dumps(overall, sort_keys=True), flush=True)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="S0 preamble-collision / preamble-only SIC simulation")
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--preset", required=True,
                    choices=["baseline", "main", "negative", "stress"])
    ap.add_argument("--input-grid", default="work",
                    choices=["work", "cg400"])
    ap.add_argument("--sic", default="off",
                    choices=["off", "estimate", "oracle"])
    ap.add_argument("--sic-detect-threshold", type=float, default=0.40,
                    help="SIC detection threshold (calibration-set value)")
    ap.add_argument("--sic-validate-threshold", type=float, default=0.25,
                    help="SIC holdout commit threshold")
    ap.add_argument("--consumer", default="both",
                    choices=["cir", "demod", "both"])
    ap.add_argument("--windows", type=int, default=None)
    ap.add_argument("--rate", type=float, default=None,
                    help="paced replay rate (Hz) -- future stage")
    ap.add_argument("--duration", type=float, default=None,
                    help="paced replay duration (s) -- future stage")
    ap.add_argument("--output", required=True)
    ap.add_argument("--seed-set", default="calibration",
                    choices=["calibration", "acceptance"])
    ap.add_argument("--interference", default="auto",
                    choices=["auto", "on", "off"])
    ap.add_argument("--fixtures", type=int, default=4,
                    help="number of windows dumped as npz+truth sidecars")
    ap.add_argument("--case", default=None,
                    help="negative/main sub-case id (default: nominal)")
    ap.add_argument("--sic-fit-reps", type=int, default=None,
                    help="SIC estimation window in SYNC reps (default: preset, "
                         "auto-extended to cover the comm preamble)")
    ap.add_argument("--sic-validation-reps", type=int, default=None,
                    help="SIC holdout reps, disjoint from fit_reps")
    ap.add_argument("--sic-in-flight-max", type=int, default=8,
                    help="source/adapter cap on published-but-not-terminal "
                         "windows")
    ap.add_argument("--sic-iq-budget-mib", type=float, default=64.0,
                    help="total pending-IQ budget for the source/adapter (MiB)")
    ap.add_argument("--sic-post-tail", type=int, default=100,
                    help="SIC reconstruction tail beyond the support (samples)")
    ap.add_argument("--sic-path-min-spacing", type=int, default=0,
                    help="SIC path min spacing (0 => template mainlobe)")
    ap.add_argument("--sic-max-window-samples", type=int, default=0,
                    help="SIC max window size hint (0 => auto)")
    ap.add_argument("--sic-no-fractional", action="store_true",
                    help="disable fractional-delay refinement in the core")
    ap.add_argument("--sic-no-cfo", action="store_true",
                    help="disable CFO estimation in the core")
    # §8.2 main-matrix axes (single-cell overrides) and the sweep driver.
    ap.add_argument("--sweep", action="store_true",
                    help="iterate the main-preset axes (per-cell runs)")
    ap.add_argument("--isr-db", type=float, default=None,
                    help="override ISR (dB) for the main preset cell")
    ap.add_argument("--start-offset", type=float, default=None,
                    help="override comm start offset (work samples)")
    ap.add_argument("--cfo-hz", type=float, default=None,
                    help="override comm CFO (Hz)")
    ap.add_argument("--comm-channel", default=None,
                    choices=["single", "four"],
                    help="override the comm channel (single|four paths)")
    ap.add_argument("--burst", type=int, default=None,
                    help="continuous N-window 100%%-collision burst at --rate")
    ap.add_argument("--max-wall-s", type=float, default=240.0,
                    help="wall-clock cap for paced/burst runs")
    ap.add_argument("--keep-iq", type=int, default=4,
                    help="bounded number of consumer PDUs whose IQ array is "
                         "retained by the sinks (memory bound for long runs)")
    args = ap.parse_args(argv)

    if args.windows is not None and args.windows <= 0:
        raise SystemExit("--windows must be > 0")
    if args.input_grid != "work":
        raise SystemExit("--input-grid %s (CG400 native roundtrip) is S0 "
                         "stage B and is not implemented in S0-A"
                         % args.input_grid)

    repo = S.find_repo_root(os.path.dirname(os.path.abspath(__file__)))

    # ---- driver dispatch (sweep / negative-set / paced / burst) ----
    if args.sweep:
        if args.preset != "main":
            raise SystemExit("--sweep is only defined for --preset main")
        if args.rate is not None or args.duration is not None or \
                args.burst is not None:
            raise SystemExit("--sweep cannot be combined with paced replay")
        return run_sweep(args, repo, argv)
    if args.rate is not None or args.duration is not None or \
            args.burst is not None:
        pass  # handled inside the single-run path via the `paced` flag
    elif args.preset == "negative" and args.case is None:
        return run_negative(args, repo, argv)

    build_dir = os.path.abspath(args.build_dir)
    preset = load_preset(repo, args.preset)
    gen = preset.get("generator", {})
    runtime = preset.get("runtime", {})

    # Load bindings from --build-dir only.
    uwb = S.load_uwb(build_dir)
    bindir = os.path.join(build_dir, "python", "uwb", "bindings")
    so = sorted(__import__("glob").glob(
        os.path.join(bindir, "uwb_python*.so")))[-1]

    # Interference selection.
    preset_interf = str(gen.get("interference", "")).lower()
    neg_case = None
    if args.preset == "negative":
        for case in gen.get("cases", []):
            if args.case is None or case.get("id") == args.case:
                neg_case = case
                break
        if neg_case is None:
            raise SystemExit("negative preset has no case %r" % args.case)
    if args.interference == "on":
        comm_present = True
    elif args.interference == "off":
        comm_present = False
    elif args.preset == "baseline":
        comm_present = False
    elif args.preset in ("main", "stress"):
        comm_present = True
    elif args.preset == "negative":
        comm_present = neg_case.get("id") != "no_interference"
    else:  # pragma: no cover
        comm_present = preset_interf not in ("off", "none", "")

    sensing_code = int(gen.get("sensing_code", 9))
    comm_code = int(gen.get("comm_code", 10))
    sync_reps = int(gen.get("sync_reps", 64))
    snr_db = gen.get("snr_db", 30.0)
    diagnostic = bool(gen.get("diagnostic_preset", False))
    comm_paths = [[0, [1.0, 0.0]]]
    start_offset_work = 0.0
    cfo_hz = 0.0
    isr_db = None
    if args.preset == "main":
        isr_db = float(preset.get("nominal_for_benchmark", {}).get(
            "isr_db", 10.0))
        start_offset_work = float(preset.get("nominal_for_benchmark", {}).get(
            "start_offset_work", 0.0))
        cfo_hz = float(preset.get("nominal_for_benchmark", {}).get(
            "cfo_hz", 0.0))
        comm_paths = preset.get("generator", {}).get(
            "comm_paths", {}).get("single_int", comm_paths)
        # §8.2 single-cell axis overrides.
        if args.isr_db is not None:
            isr_db = float(args.isr_db)
        if args.start_offset is not None:
            start_offset_work = float(args.start_offset)
        if args.cfo_hz is not None:
            cfo_hz = float(args.cfo_hz)
        if args.comm_channel is not None:
            key = ("single_int" if args.comm_channel == "single"
                   else "four_path")
            comm_paths = preset.get("generator", {}).get(
                "comm_paths", {}).get(key, comm_paths)
    elif args.preset == "negative" and neg_case is not None:
        isr_db = neg_case.get("isr_db", 10.0)
        comm_code = int(neg_case.get("comm_code", comm_code))
        start_offset_work = float(neg_case.get("start_offset_work", 0.0))
        snr_db = neg_case.get("snr_db", snr_db)
        diagnostic = bool(neg_case.get("diagnostic_preset", diagnostic))
        if not comm_present:
            isr_db = None
    elif args.preset == "stress":
        isr_db = 10.0
    isr_db = None if isr_db is None else float(isr_db)
    if comm_present and sensing_code == comm_code and not diagnostic:
        raise SystemExit("sensing_code == comm_code (%d) is only allowed via "
                         "an explicit diagnostic preset" % sensing_code)

    pulse_id = gen.get("pulse_id", "pulse_trunc_linear_rc183_240.f32")
    sic_on = args.sic in ("estimate", "oracle")
    generator = S.SicSignalGenerator(
        uwb, sensing_code=sensing_code, comm_code=comm_code,
        sync_reps=sync_reps, sfd_mode=SFD_MODE,
        pulse_shape=S.DEFAULT_PULSE_SHAPE, comm_present=comm_present,
        build_comm=(comm_present or sic_on),
        snr_db=snr_db, isr_db=isr_db,
        verbose=True)

    # SIC runtime config (adapter is block config; it never reads truth in
    # estimate mode).  Only known presets/values are accepted.
    sic_k = int(runtime.get("sic_k", 0)) if sic_on else 0
    if sic_on and sic_k not in (1, 2, 4, 8, 16):
        raise SystemExit("--sic %s requires sic_k in 1/2/4/8/16 (preset had "
                         "%r)" % (args.sic, sic_k))
    fit_reps = int(runtime.get("fit_reps", 8)) if sic_on else None
    validation_reps = int(runtime.get("validation_reps", 8)) if sic_on else None
    fit_source = "preset"
    if sic_on:
        if args.sic_fit_reps is not None or args.sic_validation_reps is not None:
            fit_reps = int(args.sic_fit_reps) if args.sic_fit_reps is not None \
                else fit_reps
            validation_reps = (int(args.sic_validation_reps)
                               if args.sic_validation_reps is not None
                               else validation_reps)
            fit_source = "cli"
        # The frozen core subtracts only the CONFIRMED repetitions
        # (support_hi = detected_start + confirmed*sps + post_tail) and
        # `confirmed <= fit_reps + validation_reps`.  With the preset nominal
        # 8/8 the mask would cancel only 16 of the 64 comm SYNC symbols, so
        # auto-extend the estimation window (within the declared fit_reps
        # axis) to cover the interferer preamble.
        if fit_reps + validation_reps < sync_reps:
            base = max(1, min(32, fit_reps))
            fit_reps = base
            validation_reps = sync_reps - base
            fit_source = "auto_cover_preamble"
        if fit_reps <= 0 or validation_reps < 8:
            raise SystemExit("fit_reps>0 and validation_reps>=8 required for "
                             "SIC (got %r/%r)" % (fit_reps, validation_reps))
        if fit_reps + validation_reps > sync_reps:
            raise SystemExit("fit_reps + validation_reps (%d) exceeds "
                             "sync_reps (%d)" % (fit_reps + validation_reps,
                                                 sync_reps))

    # Public candidate search ROI for the comm preamble.  The online SIC only
    # knows this window (plus the comm code/template/rate) -- never the true
    # injected start.  `nominal` is the sensing SYNC span; `narrow`/`wide` are
    # the declared stress scalings.
    roi_mode = runtime.get("comm_search_roi", "nominal")
    roi_base = (max(0, generator.pre),
                min(generator.n, generator.pre + sync_reps * S.SPS))
    if roi_mode == "narrow":
        span = max(1, (roi_base[1] - roi_base[0]) // 4)
        roi_pair = (roi_base[0], min(generator.n, roi_base[0] + span))
    elif roi_mode == "wide":
        roi_pair = (max(0, roi_base[0] - 2 * S.SPS),
                    min(generator.n, roi_base[1] + 2 * S.SPS))
    else:
        roi_mode = "nominal"
        roi_pair = roi_base
    roi_pair = (int(roi_pair[0]), int(roi_pair[1]))
    if sic_on and not (0 <= roi_pair[0] < roi_pair[1] <= generator.n):
        raise SystemExit("resolved SIC roi %r is outside the window [0,%d)"
                         % (roi_pair, generator.n))

    # Seeds.
    seed_specs = gen.get("seeds", {})
    seeds_all = {}
    for key in ("calibration", "acceptance"):
        if key in seed_specs:
            lst, total = materialize_seeds(
                seed_specs[key], args.windows if args.windows else
                int(seed_specs[key]["count"]))
            seeds_all[key] = {"list": lst, "available": total}
    if args.seed_set not in seeds_all:
        raise SystemExit("preset has no %s seed list" % args.seed_set)

    # Paced replay (§8.4): the schedule fixes the window count, not --windows.
    paced = (args.rate is not None or args.duration is not None or
             args.burst is not None)
    pace_rate = float(args.rate) if args.rate else 200.0
    if args.burst is not None:
        planned = int(args.burst)
    elif args.duration is not None:
        planned = int(round(pace_rate * float(args.duration)))
    else:
        planned = int(args.windows) if args.windows else 0
    if paced and planned <= 0:
        raise SystemExit("paced replay needs a positive window count "
                         "(--burst N or --rate/--duration)")

    spec0 = seed_specs.get(args.seed_set, {"start": 0, "step": 1})
    if paced:
        run_seeds = [int(spec0["start"]) + i * int(spec0.get("step", 1))
                     for i in range(planned)]
        windows = planned
        available = planned
    else:
        run_seeds = seeds_all[args.seed_set]["list"]
        available = seeds_all[args.seed_set]["available"]
        windows = args.windows if args.windows else len(run_seeds)
        if windows > available:
            raise SystemExit("--windows %d exceeds %s seed count %d"
                             % (windows, args.seed_set, available))
        run_seeds = run_seeds[:windows]

    os.makedirs(args.output, exist_ok=True)
    tpl_dir = os.path.join(args.output, "templates")
    os.makedirs(tpl_dir, exist_ok=True)
    fix_dir = os.path.join(args.output, "fixtures")
    os.makedirs(fix_dir, exist_ok=True)
    tpl_path = os.path.join(tpl_dir, "sync_template_code9_998p4.cf32")
    generator.template.tofile(tpl_path)
    comm_sync_path = None
    comm_pre_path = None
    if generator.comm_wave is not None:
        comm_pre_path = os.path.join(
            tpl_dir, "comm_preamble_code%d_998p4.cf32" % comm_code)
        generator.comm_wave.tofile(comm_pre_path)
        if generator.comm_sync is not None:
            comm_sync_path = os.path.join(
                tpl_dir, "comm_sync_code%d_998p4.cf32" % comm_code)
            generator.comm_sync.tofile(comm_sync_path)

    # SIC templates are for the NOMINAL comm code (the preset's), never the
    # injected case code: the wrong_code11 negative injects a code-11 comm and
    # the SIC must keep the code-10 template (no cache reuse).
    sic_comm_code = int(gen.get("comm_code", comm_code))
    sic_sync = generator.comm_sync
    sic_preamble = generator.comm_wave
    sic_tpl_source = "generator"
    sic_sync_path = None
    sic_pre_path = None
    if sic_on and sic_comm_code != comm_code:
        tpl_gen = S.SicSignalGenerator(
            uwb, sensing_code=sensing_code, comm_code=sic_comm_code,
            sync_reps=sync_reps, sfd_mode=SFD_MODE,
            pulse_shape=S.DEFAULT_PULSE_SHAPE, comm_present=True,
            build_comm=True, snr_db=snr_db, isr_db=isr_db, verbose=False)
        sic_sync = tpl_gen.comm_sync
        sic_preamble = tpl_gen.comm_wave
        sic_tpl_source = "sic_template_generator"
        sic_sync_path = os.path.join(
            tpl_dir, "sic_sync_code%d_998p4.cf32" % sic_comm_code)
        sic_sync.tofile(sic_sync_path)
        sic_pre_path = os.path.join(
            tpl_dir, "sic_preamble_code%d_998p4.cf32" % sic_comm_code)
        sic_preamble.tofile(sic_pre_path)

    run_id = "%s-%s-%s" % (args.preset, args.input_grid,
                           datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    epoch_id = 0

    resolved_config = {
        "preset": args.preset, "input_grid": args.input_grid,
        "sic_mode": args.sic, "consumer": args.consumer,
        "sensing_code": sensing_code, "comm_code": comm_code,
        "sync_reps": sync_reps, "sfd_mode": SFD_MODE,
        "pulse_shape": generator.pulse_shape,
        "pulse_taps": generator.pulse_taps,
        "insert_sts": False, "append_fcs": False,
        "sensing_paths": [[p.delay_samples, [p.gain.real, p.gain.imag]]
                          for p in generator.sensing_paths],
        "comm_present": comm_present, "comm_paths": comm_paths,
        "start_offset_work": start_offset_work, "cfo_hz": cfo_hz,
        "snr_db": snr_db, "isr_db": isr_db,
        "frac_model": generator.frac_model,
        "pre_guard": generator.pre, "geometry": generator.geom,
        "windows": windows, "seed_set": args.seed_set,
        "seeds": run_seeds, "demod_softchip_mode": "bypass",
        "sic_k": int(runtime.get("sic_k", 0)),
        "sync_reps_schema": sync_reps,
    }
    config_hash = stdin_hash(resolved_config)

    summary = {
        "run_id": run_id, "preset": args.preset,
        "input_grid": args.input_grid, "sic_mode": args.sic,
        "consumer": args.consumer, "epoch_id": epoch_id,
        "windows_expected": windows, "windows_completed": 0,
        "clean": {}, "raw": {}, "sic": None,
        "effect": {"median_suppression_db": None, "frac_ge_3db": None,
                   "target_amp_err_db_p95": None, "peak_shift_max": None,
                   "nmse_median": None},
        "demod": {}, "negative": {"false_detect": None,
                                  "false_commit": None, "n_windows": windows,
                                  "ci95": None,
                                  "note": "SIC detect/commit not implemented "
                                          "in S0-A"},
        "runtime": {}, "gates": {}, "g0": {}, "not_run": [],
    }
    not_run = summary["not_run"]
    not_run.append("native_cg400_roundtrip (S0 stage B; --input-grid cg400)")
    not_run.append("matlab_cross_check (no MATLAB/Octave run)")
    not_run.append("core_cxx_benchmark_10000x3 (benchmark_preamble_sic; S0-D)")
    if args.sic == "off":
        not_run.append("sic_estimate_core (sic_mode=off)")
        not_run.append("sic_oracle (diagnostic only)")
    else:
        not_run.append("sic_speed_pass (single-worker C++ core P50/P99 gate "
                       "not evaluated here)")
    if args.preset != "main" or not args.sweep:
        not_run.append("main_matrix_full_sweep (ISR/offset/CFO axes)")
    if args.preset != "negative":
        not_run.append("negative_false_commit_1000w (negative preset)")
    if not paced:
        not_run.append("paced_replay_200wps (--rate/--duration)")
    if args.sic == "off":
        not_run.append("summary.sic == null (sic_mode=off)")
        not_run.append("summary.demod.sic_ok == null (sic_mode=off)")
        not_run.append("summary.demod.raw_ok_sic_fail == null (sic_mode=off)")
        not_run.append("summary.demod.raw_fail_sic_ok == null (sic_mode=off)")
        not_run.append("summary.effect.* == null (sic_mode=off; no SIC "
                       "estimate)")
        not_run.append("windows.fit_reps/validation_reps/roi/recon == null "
                       "(sic_mode=off)")

    # ---------------- static G0 checks ----------------
    g0 = summary["g0"]
    g0["psdu_canonical_hex"] = S.DEFAULT_PSDU_HEX
    g0["pulse_shape"] = generator.pulse_shape
    # TX vs canonical golden tx (SYNC region overlap).
    gold_tx_path = os.path.join(repo, "testdata", "uwb_radar",
                                "tx_998p4.cf32")
    if os.path.isfile(gold_tx_path):
        g_tx = np.fromfile(gold_tx_path, dtype=np.complex64)
        m = min(generator.tx.size, g_tx.size)
        a = generator.tx[:m].astype(np.complex128)
        b = g_tx[:m].astype(np.complex128)
        g0["tx_vs_canonical"] = {
            "compared_samples": int(m),
            "max_abs_diff": float(np.max(np.abs(generator.tx[:m] - g_tx[:m]))),
            "sync_region_max_abs_diff": float(np.max(np.abs(
                generator.tx[:65024] - g_tx[:65024]))),
            "sync_region_corr": float(abs(np.vdot(a[:65024], b[:65024])) /
                                      (np.linalg.norm(a[:65024]) *
                                       np.linalg.norm(b[:65024]))),
            "note": "S0 sensing uses insert_sts=False (see run_manifest) so "
                    "the payload/STS tail differs; SYNC is bit-identical.",
        }
    # Wrong-template control (code 9 vs code 10 / 11).
    g0["wrong_template_control"] = {"target_code": comm_code}
    for code in (10, 11):
        try:
            other = S.SicSignalGenerator(
                uwb, comm_present=True, comm_code=code, verbose=False)
        except Exception as e:  # pragma: no cover
            g0["wrong_template_control"]["code%d_error" % code] = str(e)
            continue
        seg = other.comm_wave[:SPS]
        c9 = generator.template
        corr = abs(np.vdot(c9, seg)) / (np.linalg.norm(c9) *
                                        np.linalg.norm(seg))
        # best-lag correlation over the whole comm preamble
        full = other.comm_wave
        cross = np.correlate(c9, full.astype(np.complex128), mode="valid")
        g0["wrong_template_control"]["code%d" % code] = {
            "zero_lag_norm_corr": float(corr),
            "best_lag_abs_corr": float(np.max(np.abs(cross))),
            "comm_len": int(full.size),
            "comm_sha256": S.sha256_array(full),
        }
    g0["wrong_template_control"]["note"] = (
        "code10/code11 comm templates are independently generated; low "
        "zero-lag correlation vs the code9 SYNC template confirms no cache "
        "reuse. best_lag_abs_corr is unnormalized over the full preamble.")

    # CIR canonical reference (tight geometry/template proof).
    out_holder = {"g0": g0}
    run_canonical_cir_reference(uwb, repo, tpl_path, out_holder)

    # ---------------- run ----------------
    sic_cfg = None
    if sic_on:
        if generator.comm_wave is None:
            raise SystemExit("--sic %s needs the comm templates (comm_code=%d)"
                             % (args.sic, comm_code))
        sic_cfg = {
            "sync": sic_sync,
            "preamble": sic_preamble,
            "comm_code": sic_comm_code,
            "sync_reps": sync_reps,
            "k_paths": sic_k,
            "fit_reps": fit_reps,
            "validation_reps": validation_reps,
            "fit_reps_source": fit_source,
            "roi": roi_pair,
            "coarse_stride": 14,
            "mode": args.sic,
            "detect_threshold": args.sic_detect_threshold,
            "validate_threshold": args.sic_validate_threshold,
            "in_flight_max": int(args.sic_in_flight_max),
            "iq_budget_bytes": int(round(args.sic_iq_budget_mib * (1 << 20))),
            "queue_capacity": 8,
            "post_tail": int(args.sic_post_tail),
            "path_min_spacing": int(args.sic_path_min_spacing),
            "max_window_samples": int(args.sic_max_window_samples),
            "enable_fractional": not args.sic_no_fractional,
            "enable_cfo": not args.sic_no_cfo,
        }
        if args.sic_in_flight_max < 1:
            raise SystemExit("--sic-in-flight-max must be >= 1")
    rt = SimRuntime(uwb, tpl_path=tpl_path, consumers=args.consumer,
                    windows=windows, sic=sic_cfg,
                    keep_iq=max(0, int(args.keep_iq)))
    # Explicit SOURCE credit tracker (indication §4): never publish more than
    # in_flight_max published-but-not-terminal windows, nor exceed the IQ
    # budget.  Determined before any window is posted.
    credit = (CreditTracker(sic_cfg["in_flight_max"],
                            sic_cfg["iq_budget_bytes"]) if sic_cfg else None)
    rt.start()

    # Feature detection for the adapter's newer make() params (fall back
    # cleanly when the built binding does not expose them yet).
    sic_optional_supported, _sic_doc_params = sic_adapter_supported_params(uwb)
    if sic_on:
        print("sic_adapter: params_used=%s supported_new=%s"
              % (sorted(rt.sic_params_used), sic_optional_supported),
              flush=True)

    windows_jsonl = open(os.path.join(args.output, "windows.jsonl"), "w",
                         encoding="utf-8")
    truth_jsonl = open(os.path.join(fix_dir, "truth.jsonl"), "w",
                       encoding="utf-8")
    stage_rows = []
    consumer_stage_values = {"clean": {}, "raw": {}}
    if sic_on:
        consumer_stage_values["sic"] = {}
    gen_us_all = []
    published_ns = {}
    completions = {"clean": None, "raw": None}
    target_tap = None
    clean_cir_norm = None
    clean_cir_raw = None
    clean_queue_us = None
    failing_status = {"clean": {}, "raw": {}}
    if sic_on:
        failing_status["sic"] = {}
    clean_ok = raw_ok = sic_ok = 0
    raw_ok_sic_fail = raw_fail_sic_ok = 0
    bypass_n = 0
    bypass_identical = 0
    oracle_windows = 0
    recon_records = []
    eff_nmse = []
    eff_supp = []
    eff_holdout_supp = []
    eff_holdout_nmse = []
    eff_target_err_db = []
    eff_peak_shift = []
    sic_adapter_hwm = []
    psdu_match = 0
    demod_ran = 0
    clean_psdu_match = 0
    clean_demod_ran = 0
    first_windows = []
    det_check = None

    n_posted_cir = 0
    n_posted_demod = 0
    deadline_miss = 0
    completed_windows = 0

    # Paced replay (§8.4): pre-generate a pool of distinct windows (>=64
    # distinct-content rotation guidance) BEFORE timing; the schedule fixes the
    # publish times.  Nothing is generated inside the paced timing path.
    paced_t0_ns = None
    paced_wall0 = None
    paced_not_submitted = 0
    paced_deadline_us = 5000.0        # planned publish -> terminal deadline
    paced_late_ns = []
    paced_submit_late_ns = []
    paced_latency_us = []
    paced_deadline_miss = 0
    paced_pool_us_each = 0.0
    paced_pool = []
    paced_pool_seeds = []
    if paced:
        pool_n = min(planned, 64)
        tp0 = time.perf_counter()
        for i in range(pool_n):
            ps = run_seeds[i]
            paced_pool.append(generator.build(
                seed=ps, comm_paths=comm_paths if comm_present else None,
                start_offset_work=start_offset_work, cfo_hz=cfo_hz,
                isr_db=isr_db, snr_db=snr_db))
            paced_pool_seeds.append(ps)
        paced_pool_us_each = ((time.perf_counter() - tp0) * 1e6 /
                              max(1, pool_n))
        paced_t0_ns = time.monotonic_ns() + int(0.05e9)
        paced_wall0 = time.monotonic()
        print("paced: rate=%.1f planned=%d pool=%d pool_win_us=%.0f "
              "in_flight_max=%s wall_cap=%.0fs"
              % (pace_rate, planned, pool_n, paced_pool_us_each,
                 (credit.in_flight_max if credit else "n/a"),
                 float(args.max_wall_s)), flush=True)

    for wid in range(windows):
        if paced:
            elapsed = time.monotonic() - paced_wall0
            if elapsed > float(args.max_wall_s) + planned / pace_rate:
                paced_not_submitted = planned - wid
                print("paced: wall cap hit at wid=%d/%d; %d windows not "
                      "submitted (reported, not hidden)"
                      % (wid, planned, paced_not_submitted), flush=True)
                break
            win = paced_pool[wid % len(paced_pool)]
            seed = paced_pool_seeds[wid % len(paced_pool)]
            gen_us = paced_pool_us_each
            gen_us_all.append(gen_us)
        else:
            seed = run_seeds[wid]
            t0 = time.perf_counter()
            win = generator.build(
                seed=seed,
                comm_paths=comm_paths if comm_present else None,
                start_offset_work=start_offset_work, cfo_hz=cfo_hz,
                isr_db=isr_db, snr_db=snr_db)
            gen_us = (time.perf_counter() - t0) * 1e6
            gen_us_all.append(gen_us)

        if wid == 0:
            # Determinism: rebuild the same window with the same seed.
            win2 = generator.build(
                seed=seed,
                comm_paths=comm_paths if comm_present else None,
                start_offset_work=start_offset_work, cfo_hz=cfo_hz,
                isr_db=isr_db, snr_db=snr_db)
            g0["determinism"] = {
                "seed": int(seed),
                "s_equal": bool(np.array_equal(win["s"], win2["s"])),
                "i_equal": bool(np.array_equal(win["i"], win2["i"])),
                "w_equal": bool(np.array_equal(win["w"], win2["w"])),
                "y_equal": bool(np.array_equal(win["y"], win2["y"])),
            }

        # Posted IQ: a single exact CF32 array per consumer.  When
        # interference is off, raw reuses the *same* clean array so
        # raw.input_sha256 == clean.input_sha256 bitwise (REV 2 adjudication 4).
        clean_iq = np.ascontiguousarray(win["s"] + win["w"],
                                        dtype=np.complex64)
        raw_iq = (np.ascontiguousarray(win["y"], dtype=np.complex64)
                  if comm_present else clean_iq)

        # ---- SIC (estimate / oracle) on the raw window ----------------
        cleaned_iq = None
        sic_block = None
        sic_state_rec = None
        recon_rec = None
        sic_credit_snap = None
        win_planned_ns = None
        win_submit_ns = None
        if rt.sic is not None:
            # Paced replay: hold the window until its planned publish time.
            # A late submit (pipeline can't keep up) is recorded, never hidden
            # by lowering the rate.
            if paced:
                win_planned_ns = paced_t0_ns + int(
                    round(wid * 1e9 / pace_rate))
                now_ns = time.monotonic_ns()
                if now_ns < win_planned_ns:
                    time.sleep((win_planned_ns - now_ns) * 1e-9)
            # SOURCE credit: reserve before publishing.  With the serial
            # single-worker driver the credit is always free for
            # in_flight_max>=1, but the limit is explicit and recorded.
            planned_ns = (win_planned_ns if win_planned_ns is not None
                          else time.monotonic_ns())
            src_ok, src_reason = credit.acquire(int(raw_iq.size) * 8)
            if not src_ok:
                credit.release(int(raw_iq.size) * 8)
                raise SystemExit(
                    "source credit exhausted at window_id=%d (%s); the source "
                    "must not mask this by lowering the input rate"
                    % (wid, src_reason))
            in_meta = pmt.make_dict()
            in_meta = put_int(in_meta, "window_id", wid)
            in_meta = put_int(in_meta, "sample_count", int(raw_iq.size))
            in_meta = put_int(in_meta, "code_index", sensing_code)
            in_meta = put_int(in_meta, "sync_repetitions", sync_reps)
            in_meta = put_str(in_meta, "sfd_mode", SFD_MODE)
            in_meta = put_str(in_meta, "sensing_profile_id",
                              "code%d-%dSYNC-%s" % (sensing_code, sync_reps,
                                                    SFD_MODE))
            in_meta = put_str(in_meta, "template_id",
                              "sync_template_code%d_998p4" % sensing_code)
            in_meta = pmt.dict_add(in_meta, pmt.intern("planned_publish_ns"),
                                   pmt.from_long(int(planned_ns)))
            if args.sic == "oracle":
                truth_msg = pmt.make_dict()
                truth_msg = put_int(truth_msg, "window_id", wid)
                truth_msg = put_float(
                    truth_msg, "start_sample",
                    float(generator.pre) + float(start_offset_work))
                truth_msg = put_float(truth_msg, "cfo_hz", float(cfo_hz))
                tpaths = (S.PathSpec.from_pairs(comm_paths)
                          if comm_present else [S.PathSpec(0.0, 1.0 + 0j)])
                scale_t = float(win["truth"]["comm"]["scale"])
                tau_v = [float(p.delay_samples) for p in tpaths]
                gr_v = [float(p.gain.real * scale_t) for p in tpaths]
                gi_v = [float(p.gain.imag * scale_t) for p in tpaths]
                truth_msg = put_int(truth_msg, "n_paths", len(tpaths))
                truth_msg = pmt.dict_add(
                    truth_msg, pmt.intern("tau"),
                    pmt.init_f64vector(len(tau_v), tau_v))
                truth_msg = pmt.dict_add(
                    truth_msg, pmt.intern("gain_re"),
                    pmt.init_f64vector(len(gr_v), gr_v))
                truth_msg = pmt.dict_add(
                    truth_msg, pmt.intern("gain_im"),
                    pmt.init_f64vector(len(gi_v), gi_v))
                rt.post_sic_truth(truth_msg)
            n_out = rt.sink_sic_out.count + 1
            n_ih = rt.sink_sic_ihat.count + 1
            rt.post_sic_input(in_meta, raw_iq)
            win_submit_ns = time.monotonic_ns()
            if paced and win_planned_ns is not None:
                paced_submit_late_ns.append(win_submit_ns - win_planned_ns)
            if not rt.wait_sic(n_out, n_ih, deadline_s=180.0):
                raise SystemExit("SIC adapter did not terminate window_id=%d "
                                 "(out=%r i_hat=%r)" % (wid, rt.sic_counts(),
                                                        rt.sic_counts()))
            out_meta = rt.sink_sic_out.latest
            ihat_meta = rt.sink_sic_ihat.latest
            # Terminal reached: capture the credit state, then release.
            sic_credit_snap = credit.snapshot(planned_ns)
            credit.release(int(raw_iq.size) * 8)
            cleaned_iq = np.ascontiguousarray(
                out_meta.get("__c32", raw_iq), dtype=np.complex64)
            ihat = np.ascontiguousarray(
                ihat_meta.get("__c32", np.zeros_like(raw_iq)),
                dtype=np.complex64)
            if not bool(dget(out_meta, "sic_committed", False)):
                bypass_n += 1
                if np.array_equal(cleaned_iq, raw_iq):
                    bypass_identical += 1
            uses_truth = bool(dget(out_meta, "sic_uses_truth", False))
            sic_state_rec = {
                "detected": bool(dget(out_meta, "sic_detected", False)),
                "committed": bool(dget(out_meta, "sic_committed", False)),
                "partial": bool(dget(out_meta, "sic_partial", False)),
                "ambiguous": bool(dget(out_meta, "sic_ambiguous", False)),
                "bypass_reason": dget(out_meta, "sic_bypass_reason", None),
                "uses_truth": uses_truth,
                "state": dget(out_meta, "sic_state", None),
                "detected_start": dnum(out_meta, "sic_detected_start", -1),
                "n_paths": dnum(out_meta, "sic_n_paths", 0),
                "cfo_hz": dnum(out_meta, "sic_cfo_hz", 0.0),
                "support_lo": dnum(out_meta, "sic_support_lo", -1),
                "support_hi": dnum(out_meta, "sic_support_hi", -1),
                "residual_corr": dnum(out_meta, "sic_residual_corr", 0.0),
            }
            if uses_truth:
                oracle_windows += 1
                sic_state_rec["diagnostic_only"] = True
            # Truth evaluation interval = the true injected comm support.
            comm_lo = int(math.floor(
                win["truth"]["comm"]["start_sample_exact"]))
            comm_len = int(win["truth"]["comm"]["length_samples"])
            comm_hi = min(generator.n, comm_lo + comm_len)
            if comm_hi <= comm_lo:
                # no interferer injected: evaluate over the nominal comm span
                comm_lo, comm_hi = int(roi_pair[0]), int(roi_pair[1])
            ev = recon_pair(win["i"], ihat, comm_lo, comm_hi)
            ev_zero = False
            if ev is None:
                # ||i||^2 == 0: the reconstruction is measured against the
                # (zero) injected interferer; 0 residual => NMSE 0, a phantom
                # reconstruction => NMSE 1.  Both are declared conventions.
                num = float(np.sum(np.abs(
                    ihat[comm_lo:comm_hi].astype(np.complex128)) ** 2))
                if num == 0.0:
                    ev = (0.0, 60.0, comm_lo, comm_hi)
                else:
                    ev = (1.0, 0.0, comm_lo, comm_hi)
                ev_zero = True
            det_start = sic_state_rec["detected_start"]
            anchor = det_start if (isinstance(det_start, int) and
                                   det_start >= 0) else comm_lo
            ho_lo = anchor + fit_reps * S.SPS
            ho_hi = ho_lo + validation_reps * S.SPS
            ho = recon_pair(win["i"], ihat, ho_lo, ho_hi)
            if ho is None:
                h_lo = max(0, min(generator.n, ho_lo))
                h_hi = max(h_lo, min(generator.n, ho_hi))
                hnum = float(np.sum(np.abs(
                    ihat[h_lo:h_hi].astype(np.complex128)) ** 2))
                if hnum == 0.0:
                    ho = (0.0, 60.0, h_lo, h_hi)
                else:
                    ho = (1.0, 0.0, h_lo, h_hi)
            recon_rec = {
                "nmse": ev[0] if ev else None,
                "suppression_db": ev[1] if ev else None,
                "eval_lo": ev[2] if ev else comm_lo,
                "eval_hi": ev[3] if ev else comm_hi,
                "holdout_nmse": ho[0] if ho else None,
                "holdout_suppression_db": ho[1] if ho else None,
                "holdout_lo": ho[2] if ho else None,
                "holdout_hi": ho[3] if ho else None,
                "mask_lo": sic_state_rec["support_lo"],
                "mask_hi": sic_state_rec["support_hi"],
                "mask_samples": (max(0, sic_state_rec["support_hi"] -
                                     sic_state_rec["support_lo"])
                                 if sic_state_rec["support_hi"] >
                                 sic_state_rec["support_lo"] else 0),
                "noise_included": False,
                "zero_reference_energy": bool(ev_zero),
                "note": ("nmse/suppression are the i-based metric on the true "
                         "injected comm support; AWGN is excluded (i carries "
                         "no noise). holdout is the disjoint validation-rep "
                         "span [detected_start+fit_reps*sps, "
                         "+validation_reps*sps)."),
            }
            if ev is not None and not uses_truth:
                eff_nmse.append(ev[0])
                eff_supp.append(ev[1])
                if ho is not None:
                    eff_holdout_nmse.append(ho[0])
                    eff_holdout_supp.append(ho[1])
            # consumer=sic core + adapter stage rows
            for key, stage in SIC_CORE_STAGES:
                val = dnum(out_meta, key)
                if val is not None:
                    stage_rows.append((run_id, wid, "sic", "sic", "cxx",
                                       stage, int(val), "preamble_sic_core"))
            for key, stage in SIC_ADAPTER_STAGES:
                val = dnum(out_meta, key)
                if val is not None:
                    stage_rows.append((run_id, wid, "sic", "adapter",
                                       "adapter", stage, int(val),
                                       "preamble_sic_adapter"))
            stamp = time.monotonic_ns()
            stage_rows.append((run_id, wid, "sic", "outer", "outer",
                               "generator", round(gen_us, 3), "sic_window"))

        consumer_log = {"clean": {}, "raw": {}}
        consumers_iter = [("clean", clean_iq, "s+w"),
                          ("raw", raw_iq,
                           "s+i+w" if comm_present else "s+w")]
        if cleaned_iq is not None:
            consumer_log["sic"] = {}
            consumers_iter.append(("sic", cleaned_iq, "sic_cleaned"))
        for cname, sig, gen_detail in consumers_iter:
            input_sha = hashlib.sha256(
                np.ascontiguousarray(sig, dtype=np.complex64).tobytes()
            ).hexdigest()
            meta_cir = pmt.make_dict()
            meta_cir = put_int(meta_cir, "pulse_id", wid + 1)
            meta_cir = put_int(meta_cir, "schedule_index", wid)
            meta_cir = put_int(meta_cir, "pre_guard_samples", generator.pre)
            meta_cir = put_int(meta_cir, "window_start_sample", 0)
            meta_cir = put_float(meta_cir, "sample_rate", S.WORK_HZ)
            meta_cir = put_str(meta_cir, "sfd_mode", SFD_MODE)
            meta_cir = put_int(meta_cir, "sync_repetitions", sync_reps)
            meta_cir = put_int(meta_cir, "code_index", sensing_code)
            meta_cir = put_float(meta_cir, "calibration_delay_native_samples",
                                 0.0)

            meta_dem = pmt.make_dict()
            meta_dem = put_int(meta_dem, "schema_version", SCHEMA_VERSION)
            meta_dem = put_str(meta_dem, "run_id", run_id)
            meta_dem = put_int(meta_dem, "window_id", wid)
            meta_dem = put_int(meta_dem, "epoch_id", epoch_id)
            meta_dem = put_int(meta_dem, "packet_id", wid + 1)
            meta_dem = put_int(meta_dem, "schedule_index", wid)
            meta_dem = put_int(meta_dem, "predicted_start_sample",
                               generator.pre)
            meta_dem = put_int(meta_dem, "window_start_sample", 0)
            meta_dem = put_int(meta_dem, "predicted_sensing_start",
                               generator.pre)
            meta_dem = put_int(meta_dem, "sample_count", int(sig.size))
            meta_dem = put_float(meta_dem, "sample_rate", S.WORK_HZ)
            meta_dem = put_str(meta_dem, "sensing_profile_id",
                               "code%d-%dSYNC-%s" % (sensing_code, sync_reps,
                                                     SFD_MODE))
            meta_dem = put_int(meta_dem, "sensing_code", sensing_code)
            meta_dem = put_int(meta_dem, "sensing_sync_reps", sync_reps)
            meta_dem = put_str(meta_dem, "sensing_sfd_mode", SFD_MODE)
            meta_dem = put_str(meta_dem, "template_id",
                               "sync_template_code%d_998p4" % sensing_code)

            t_pub = time.monotonic_ns()
            published_ns[(wid, cname)] = t_pub
            if rt.want_cir:
                rt.post_cir(meta_cir, sig)
                n_posted_cir += 1
            if rt.want_demod:
                rt.post_demod(meta_dem, sig)
                n_posted_demod += 1
            ok = rt.wait_for(n_posted_cir, n_posted_demod, deadline_s=180.0)
            if not ok:
                deadline_miss += 1
                raise SystemExit(
                    "consumer results did not arrive (window=%d consumer=%s "
                    "cir=%s demod=%s)" % (wid, cname, rt.counts(),
                                          rt.sink_demod.count if
                                          rt.sink_demod else -1))
            c_ns = time.monotonic_ns()
            completions[cname] = c_ns

            cir_d = rt.sink_cir.latest if rt.want_cir else {}
            dem_d = (rt.sink_demod.latest if rt.want_demod else {})

            cir_status = dget(cir_d, "status") if rt.want_cir else None
            cir_peak = dnum(cir_d, "peak_tap") if rt.want_cir else None
            cir_norm = None
            cir_raw = cir_d.get("__c32") if rt.want_cir else None
            if rt.want_cir and "normalized_taps" in cir_d:
                nt = cir_d["normalized_taps"]
                cir_norm = np.array(pmt.c32vector_elements(nt),
                                    dtype=np.complex64)
            cir_valid_reps = (dnum(cir_d, "valid_repetitions")
                              if rt.want_cir else None)
            if cname == "clean" and cir_norm is not None and \
                    cir_peak is not None:
                clean_cir_norm = cir_norm
                clean_cir_raw = cir_raw
                lo = max(0, int(cir_peak) + 25)
                hi = min(cir_norm.size, int(cir_peak) + 45)
                if hi > lo:
                    target_tap = lo + int(np.argmax(np.abs(
                        cir_norm[lo:hi])))

            dem_status = dget(dem_d, "status") if rt.want_demod else None
            fcs = bool(dget(dem_d, "fcs_pass", False)) if rt.want_demod \
                else None
            psdu_bytes = dem_d.get("__u8", b"") if rt.want_demod else b""
            psdu_hex_raw = psdu_bytes.hex().upper() if psdu_bytes else None
            psdu_ok = (psdu_hex_raw == S.DEFAULT_PSDU_HEX) if rt.want_demod \
                else None
            # Only expose the decoded PSDU when the canonical code-9 bytes are
            # confirmed; a wrong/partial decode would otherwise be reported as
            # a code-9 PSDU and rejected by the verifier.
            psdu_hex = psdu_hex_raw if psdu_ok else None
            if rt.want_demod:
                demod_ran += 1
                if psdu_ok:
                    psdu_match += 1
                if cname == "clean":
                    clean_demod_ran += 1
                    if psdu_ok:
                        clean_psdu_match += 1
                okc = (dem_status in TERMINAL_STATUS_OK)
                if cname == "clean":
                    clean_ok += int(okc)
                elif cname == "raw":
                    raw_ok += int(okc)
                else:
                    sic_ok += int(okc)

            block = {
                "status": dem_status if rt.want_demod else cir_status,
                "cir_status": cir_status,
                "cir_peak_tap": cir_peak,
                "cir_valid_repetitions": cir_valid_reps,
                "input_sha256": input_sha,
                "input_samples": int(sig.size),
                "target_amp": (None if (target_tap is None or
                                        cir_norm is None)
                               else [float(cir_norm[target_tap].real),
                                     float(cir_norm[target_tap].imag)]),
                "demod_status": dem_status,
                "fcs_pass": fcs,
                "psdu_ok": psdu_ok,
                "psdu_hex": psdu_hex if rt.want_demod else None,
            }
            for key, stage, tier, detail in DEMOD_STAGES:
                val = dnum(dem_d, key) if rt.want_demod else None
                block[key] = val
                if rt.want_demod and val is not None:
                    stage_rows.append((run_id, wid, cname, "demod", tier,
                                       stage, int(val), detail))
                    consumer_stage_values[cname].setdefault(
                        stage, []).append(int(val))
            consumer_log[cname] = block
            _st = str(block.get("status"))
            failing_status[cname][_st] = \
                failing_status[cname].get(_st, 0) + 1

            # adapter/outer timings
            q_us = dnum(cir_d, "queue_delay_us") if rt.want_cir else None
            if rt.want_demod:
                q_us = dnum(dem_d, "queue_delay_us")
            if cname == "clean":
                clean_queue_us = q_us
            if rt.want_cir:
                est_us = dnum(cir_d, "estimator_us")
                if est_us is not None:
                    stage_rows.append((run_id, wid, cname, "radar", "cxx",
                                       "cir", int(est_us),
                                       "radar_cir_only"))
                    if not rt.want_demod:
                        # CIR-only consumer: the authoritative per-consumer
                        # total is the radar CIR estimator service time.
                        stage_rows.append((run_id, wid, cname, "demod", "cxx",
                                           "total", int(est_us),
                                           "cir_only_total"))
            stage_rows.append((run_id, wid, cname, "outer", "outer",
                               "generator", round(gen_us, 3), gen_detail))
            if q_us is not None:
                stage_rows.append((run_id, wid, cname, "adapter", "adapter",
                                   "queue_wait", int(q_us),
                                   "cir" if (rt.want_cir and not rt.want_demod)
                                   else "demod"))
            comp_us = (c_ns - t_pub) / 1000.0
            stage_rows.append((run_id, wid, cname, "adapter", "adapter",
                               "completion_latency", round(comp_us, 3),
                               "from_publish_to_terminal"))

        # paced replay terminal/deadline accounting (planned -> terminal)
        if paced and win_planned_ns is not None:
            terms = [completions[c] for c in consumer_log
                     if completions.get(c)]
            if terms:
                term_ns = max(terms)
                lat_us = (term_ns - win_planned_ns) / 1000.0
                paced_latency_us.append(lat_us)
                paced_late_ns.append((term_ns - win_planned_ns) / 1000.0)
                if lat_us > paced_deadline_us:
                    paced_deadline_miss += 1

        # effect accumulators (clean vs sic): target amplitude + peak shift.
        if cleaned_iq is not None and not (sic_state_rec or {}).get(
                "uses_truth", False):
            cb, sb = consumer_log.get("clean"), consumer_log.get("sic")
            if isinstance(cb, dict) and isinstance(sb, dict):
                a = cb.get("target_amp")
                b = sb.get("target_amp")
                if (isinstance(a, (list, tuple)) and len(a) == 2 and
                        isinstance(b, (list, tuple)) and len(b) == 2):
                    ma = math.hypot(float(a[0]), float(a[1]))
                    mb = math.hypot(float(b[0]), float(b[1]))
                    if ma > 0.0 and mb > 0.0:
                        eff_target_err_db.append(20.0 * math.log10(mb / ma))
                if isinstance(cb.get("cir_peak_tap"), int) and \
                        isinstance(sb.get("cir_peak_tap"), int):
                    eff_peak_shift.append(abs(sb["cir_peak_tap"] -
                                              cb["cir_peak_tap"]))
            # raw/sic demod recovery bookkeeping (consumer_ok semantics)
            def _ok(blk):
                return (isinstance(blk, dict) and
                        blk.get("status") in TERMINAL_STATUS_OK and
                        (blk.get("fcs_pass") is True or
                         blk.get("psdu_ok") is True))
            if rt.want_demod:
                if _ok(consumer_log.get("raw")) and not _ok(
                        consumer_log.get("sic")):
                    raw_ok_sic_fail += 1
                elif (not _ok(consumer_log.get("raw"))) and _ok(
                        consumer_log.get("sic")):
                    raw_fail_sic_ok += 1

        # admission (single consumer pair per window)
        pub = published_ns.get((wid, "clean"))
        comp = completions.get("clean")
        missed_admission = False
        if sic_credit_snap is not None:
            missed_admission = bool(
                sic_credit_snap["source_missed_admission"] > 0 or
                dget(out_meta, "sic_missed_admission", False))
        admission = {
            "published_ns": int(pub) if pub else None,
            "admitted_ns": (int(pub + clean_queue_us * 1000)
                            if (pub and clean_queue_us is not None) else None),
            "completion_ns": int(comp) if comp else None,
            "missed_admission": missed_admission,
        }
        if sic_credit_snap is not None:
            admission.update({
                "source_offered": sic_credit_snap["source_offered"],
                "source_admitted": sic_credit_snap["source_admitted"],
                "source_missed_admission":
                    sic_credit_snap["source_missed_admission"],
                "source_miss_reasons": sic_credit_snap["source_miss_reasons"],
                "source_inflight_at_publish":
                    sic_credit_snap["source_inflight_at_publish"],
                "source_pending_bytes_at_publish":
                    sic_credit_snap["source_pending_bytes_at_publish"],
                "source_planned_publish_ns":
                    sic_credit_snap["source_planned_publish_ns"],
                "adapter_missed_admission": bool(
                    dget(out_meta, "sic_missed_admission", False)),
                "in_flight_max": sic_credit_snap["in_flight_max"],
                "iq_budget_bytes": sic_credit_snap["iq_budget_bytes"],
            })

        # bounds
        cir_origin = generator.pre
        cir_roi_lo = cir_origin - 16
        cir_roi_hi = cir_origin + 100
        last_rep_hi = cir_origin + (10 + 54 - 1) * SPS + 100
        bounds = {
            "window_lo": 0, "window_hi": int(sig.size),
            "cir_roi_lo": int(cir_roi_lo), "cir_roi_hi": int(cir_roi_hi),
            "cir_last_rep_hi": int(last_rep_hi),
            "predicted_sfd_start": int(generator.pre + 0 +
                                       sync_reps * SPS),
            "in_range": bool(cir_roi_lo >= 0 and last_rep_hi <= sig.size),
            "partial": bool(not (cir_roi_lo >= 0 and
                                 last_rep_hi <= sig.size)),
        }
        if not bounds["in_range"]:
            det_check = det_check or {}
            det_check["partial_windows"] = det_check.get("partial_windows",
                                                         0) + 1

        rec = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run_id,
            "window_id": wid,
            "epoch_id": epoch_id,
            "sample_rate": S.WORK_HZ,
            "sample_count": int(sig.size),
            "window_start_sample": 0,
            "predicted_sensing_start": int(generator.pre),
            "sensing_profile_id": "code%d-%dSYNC-%s" % (sensing_code,
                                                        sync_reps, SFD_MODE),
            "sensing_code": sensing_code,
            "sensing_sync_reps": sync_reps,
            "sensing_sfd_mode": SFD_MODE,
            "template_id": "sync_template_code%d_998p4" % sensing_code,
            "overlap": dict(win["overlap"]),
            "bounds": bounds,
            "isr_db": win["isr_db"],
            "snr_db": snr_db,
            "comm_code": comm_code,
            "comm_present": comm_present,
            "diagnostic_preset": bool(diagnostic),
            "sic_k": (0 if not sic_on else sic_k),
            "fit_reps": (None if not sic_on else fit_reps),
            "validation_reps": (None if not sic_on else validation_reps),
            "fit_reps_source": (None if not sic_on else fit_source),
            "roi": (None if not sic_on else [int(roi_pair[0]),
                                             int(roi_pair[1])]),
            "demod_softchip_mode": "bypass",
            "seed": int(seed),
            "clean": consumer_log["clean"],
            "raw": consumer_log["raw"],
            "sic_state": (sic_state_rec if sic_on else
                          {"detected": False, "committed": False,
                           "partial": False, "ambiguous": False,
                           "bypass_reason": "sic_off",
                           "uses_truth": False}),
            "admission": admission,
            "recon": (recon_rec if sic_on else None),
        }
        if sic_on:
            rec["sic"] = consumer_log.get("sic")
            if args.sic == "oracle":
                rec["diagnostic_only"] = True
                rec["oracle"] = True
                rec["sic_state"]["diagnostic_only"] = True
        windows_jsonl.write(json.dumps(rec, sort_keys=True) + "\n")
        windows_jsonl.flush()
        completed_windows += 1

        t_rec = {
            "run_id": run_id, "window_id": wid, "seed": int(seed),
            "sensing_code": sensing_code, "comm_code": comm_code,
            "comm": {k: v for k, v in win["truth"]["comm"].items()},
            "sensing": win["truth"]["sensing"],
            "hashes": {
                "template_id": "sync_template_code%d_998p4" % sensing_code,
                "template_sha256": generator.sensing_template_info().sha256,
            },
        }
        if wid < args.fixtures:
            truth_jsonl.write(json.dumps(t_rec, sort_keys=True) + "\n")
            truth_jsonl.flush()
            np.savez_compressed(
                os.path.join(fix_dir, "window_%04d.npz" % wid),
                s=win["s"].astype(np.complex64),
                i=win["i"].astype(np.complex64),
                w=win["w"].astype(np.complex64),
                y=win["y"].astype(np.complex64),
                seed=np.int64(seed), isr_db=np.float64(
                    win["isr_db"] if win["isr_db"] is not None else np.nan),
                overlap_lo=np.int64(win["overlap"]["lo"]),
                overlap_hi=np.int64(win["overlap"]["hi"]))
            if sic_on and cleaned_iq is not None:
                recon_dir = os.path.join(args.output, "recon")
                os.makedirs(recon_dir, exist_ok=True)
                np.savez_compressed(
                    os.path.join(recon_dir, "window_%d.npz" % wid),
                    i=win["i"].astype(np.complex64),
                    i_hat=np.ascontiguousarray(ihat, dtype=np.complex64))
            first_windows.append(wid)
        if wid < 2:
            print("window %d/%d seed=%d gen_us=%.1f isr=%s overlap=%s "
                  "clean=%s raw=%s" % (
                      wid + 1, windows, seed, gen_us, win["isr_db"],
                      win["overlap"],
                      consumer_log["clean"].get("demod_status"),
                      consumer_log["raw"].get("demod_status")), flush=True)

    windows_jsonl.close()
    truth_jsonl.close()

    # CIR cleanup: wait for blocks drained, then stop.
    if rt.want_cir:
        try:
            rt.cir.drain()
        except Exception:
            pass
    if rt.want_demod:
        try:
            rt.demod.drain()
        except Exception:
            pass
    rt.stop()

    summary["windows_completed"] = completed_windows
    paced_stats = None
    if paced:
        elapsed_s = max(1e-9, time.monotonic() - paced_wall0)
        lat = sorted(paced_latency_us)
        sub_late = sorted(ns / 1000.0 for ns in paced_submit_late_ns)

        def _pct(vals, q):
            if not vals:
                return None
            return float(np.percentile(np.asarray(vals, dtype=float), q))
        paced_stats = {
            "rate_hz_target": float(pace_rate),
            "planned_windows": int(planned),
            "submitted": int(completed_windows),
            "not_submitted": int(paced_not_submitted),
            "admitted": int(completed_windows),
            "completed": int(completed_windows),
            "deadline_us": float(paced_deadline_us),
            "deadline_misses": int(paced_deadline_miss),
            "sustained_windows_per_s": float(completed_windows / elapsed_s),
            "observed_wall_s": float(elapsed_s),
            "submit_late_us": {
                "n": len(sub_late), "p50": _pct(sub_late, 50),
                "p95": _pct(sub_late, 95), "p99": _pct(sub_late, 99),
                "max": (sub_late[-1] if sub_late else None),
            },
            "completion_latency_us": {
                "n": len(lat), "p50": _pct(lat, 50), "p95": _pct(lat, 95),
                "p99": _pct(lat, 99),
                "max": (lat[-1] if lat else None),
            },
            "in_flight_max": int(credit.in_flight_max) if credit else None,
            "source_missed_admission": int(credit.missed_admission)
            if credit else None,
            "budget_bytes": int(credit.iq_budget_bytes) if credit else None,
            "window_bytes": int(generator.n * 8),
            "note": ("submit latency is measured from the planned publish time; "
                     "a pipeline that cannot keep up is recorded as a late "
                     "submit / deadline miss and never masked by lowering the "
                     "actual rate"),
        }
        summary["paced"] = paced_stats
        if paced_not_submitted > 0:
            not_run.append("windows: %d of %d planned paced windows were not "
                           "submitted before the wall-clock cap (reported, not "
                           "silently truncated)"
                           % (paced_not_submitted, planned))

    # ---------------- G0 clean-consumer CIR comparison ----------------
    if clean_cir_raw is not None and clean_cir_norm is not None:
        raw_g = np.fromfile(os.path.join(repo, "testdata", "uwb_radar",
                                         "cir_raw_clean_radar.cf32"),
                            dtype=np.complex64)
        gnorm = np.fromfile(os.path.join(repo, "testdata", "uwb_radar",
                                         "cir_norm_clean_radar.cf32"),
                            dtype=np.complex64)
        cmp = {"golden_taps": int(gnorm.size),
               "clean_taps": int(clean_cir_norm.size)}
        if gnorm.size == clean_cir_norm.size == raw_g.size == \
                clean_cir_raw.size:
            peak = int(np.argmax(np.abs(gnorm)))
            lo = max(0, peak - 10)
            hi = min(gnorm.size, peak + 10)
            diff_raw = np.abs(clean_cir_raw - raw_g)
            diff_norm = np.abs(clean_cir_norm - gnorm)
            cmp["raw_full_max_abs_diff"] = float(diff_raw.max())
            cmp["raw_main_lobe_max_abs_diff"] = float(diff_raw[lo:hi].max())
            cmp["norm_full_max_abs_diff"] = float(diff_norm.max())
            cmp["norm_main_lobe_max_abs_diff"] = float(diff_norm[lo:hi].max())
            cmp["main_lobe"] = [int(lo), int(hi)]
            cmp["golden_peak_tap"] = peak
            cmp["clean_peak_argmax"] = int(np.argmax(
                np.abs(clean_cir_norm)))
            # The injected weak target only enters the L2 normalization, so
            # the normalized-CIR main lobe must be a uniform down-scale.
            lobe_c = clean_cir_norm[lo:hi].astype(np.complex128)
            lobe_g = gnorm[lo:hi].astype(np.complex128)
            den = float(np.vdot(lobe_g, lobe_g).real)
            sstar = (float(np.vdot(lobe_g, lobe_c).real) / den
                     if den > 0 else 0.0)
            resid = float(np.max(np.abs(lobe_c - sstar * lobe_g)))
            mask = np.abs(lobe_g) > 0.01
            scale_std = (float(np.std(np.abs(
                lobe_c[mask] / lobe_g[mask]))) if np.any(mask) else 0.0)
            cmp["norm_main_lobe_scale_ls"] = sstar
            cmp["norm_main_lobe_scale_std"] = scale_std
            cmp["norm_main_lobe_ls_residual"] = resid
            cmp["predicted_scale_from_l2"] = float(
                np.linalg.norm(raw_g) / np.linalg.norm(clean_cir_raw))
            if target_tap is not None:
                cmp["weak_target_tap"] = int(target_tap)
                cmp["weak_target_raw_clean_amp"] = float(
                    np.abs(clean_cir_raw[target_tap]))
                cmp["weak_target_raw_golden_amp"] = float(
                    np.abs(raw_g[target_tap]))
            cmp["tol_raw_main_lobe"] = 5e-3
            cmp["raw_main_lobe_pass"] = bool(
                cmp["raw_main_lobe_max_abs_diff"] <= 5e-3)
            cmp["peak_tap_pass"] = bool(cmp["clean_peak_argmax"] == peak)
            cmp["norm_scale_explained"] = bool(
                abs(sstar - cmp["predicted_scale_from_l2"]) <= 2e-3 and
                resid <= 5e-3)
            cmp["note"] = (
                "Clean consumer s carries the frozen weak target (delay 37). "
                "Its raw CIR main lobe must match cir_raw_clean_radar.cf32; "
                "the L2-normalized CIR main lobe is a uniform down-scale "
                "(norm_scale_explained) because the weak target only adds "
                "energy to the L2 norm. The extra tap at weak_target_tap is "
                "the deliberate target, reported separately. The tight "
                "geometry/template proof is cir_canonical_reference, which "
                "feeds rx_clean_998p4.cf32 and matches cir_norm_clean_radar.")
        g0["cir_clean_consumer"] = cmp

    canon = g0.get("cir_canonical_reference", {})
    g0["cir_canonical_pass"] = bool(
        canon.get("ran") and canon.get("max_abs_diff_norm") is not None and
        canon["max_abs_diff_norm"] <= 1e-5)
    g0["cir_clean_pass"] = bool(
        g0.get("cir_clean_consumer", {}).get("raw_main_lobe_pass") and
        g0.get("cir_clean_consumer", {}).get("peak_tap_pass") and
        g0.get("cir_clean_consumer", {}).get("norm_scale_explained"))
    g0["psdu_canonical_pass"] = bool(clean_demod_ran > 0 and
                                      clean_psdu_match == clean_demod_ran)
    g0["determinism_pass"] = bool(g0.get("determinism", {}).get("y_equal"))
    g0["window_bounds_pass"] = bool(not det_check)
    g0["wrong_template_pass"] = bool(
        g0["wrong_template_control"].get("code10", {}).get(
            "zero_lag_norm_corr", 1.0) < 0.5)

    # ---------------- summary aggregates ----------------
    for cname in (("clean", "raw", "sic") if sic_on else ("clean", "raw")):
        summary[cname] = {
            "stages": {k: summarize(v) for k, v in
                       consumer_stage_values[cname].items()},
            "windows": windows,
        }
        if consumer_stage_values[cname]:
            summary[cname]["total_mean_us"] = summarize(
                consumer_stage_values[cname].get("total", [])).get("mean")
    if sic_on:
        summary["sic"]["adapter"] = {
            "offered": int(rt.sic.windows_offered()),
            "admitted": int(rt.sic.windows_admitted()),
            "completed": int(rt.sic.windows_completed()),
            "failed": int(rt.sic.windows_failed()),
            "bypassed": int(rt.sic.windows_bypassed()),
            "dropped": int(rt.sic.windows_dropped()),
            "invalid": int(rt.sic.windows_invalid()),
            "missed_admission": int(rt.sic.windows_missed_admission()),
            "inflight_high_watermark": int(rt.sic.inflight_high_watermark()),
            "queue_high_watermark": int(rt.sic.queue_high_watermark()),
            "drained": bool(rt.sic.drained()),
        }
        summary["sic"]["fit_reps"] = int(fit_reps)
        summary["sic"]["validation_reps"] = int(validation_reps)
        summary["sic"]["fit_reps_source"] = fit_source
        summary["sic"]["roi"] = [int(roi_pair[0]), int(roi_pair[1])]
        summary["sic"]["bypass_bit_identical"] = {
            "n_bypassed": int(bypass_n),
            "n_bit_identical": int(bypass_identical),
            "all_identical": bool(bypass_n == 0 or
                                  bypass_n == bypass_identical),
        }
        # F5: SIC front-end cost (scope=sic rows from stage_times.csv).  This is
        # NOT the demod-on-sic timing kept separately in summary.sic.stages.
        sic_core_vals = {}
        for _r in stage_rows:
            if _r[3] == "sic":
                sic_core_vals.setdefault(_r[5], []).append(int(_r[6]))
        summary["sic"]["core"] = {
            "description": ("SIC front-end (scope=sic) stage timings "
                            "aggregated from stage_times.csv; keep distinct "
                            "from summary.sic.stages which is demod-on-sic"),
            "n_windows": int(completed_windows),
            "stages": {_stage: summarize(sic_core_vals.get(_stage, []))
                       for _key, _stage in SIC_CORE_STAGES},
            "core_total": summarize(sic_core_vals.get("core_total", [])),
            "per_window_rows": "stage_times.csv scope=sic (one row per "
                               "window per stage)",
        }
        summary["sic"]["config_params_used"] = sorted(rt.sic_params_used)
        summary["sic"]["config_params_unsupported"] = [
            p for p in ("post_tail", "enable_fractional", "enable_cfo",
                        "path_min_spacing", "max_window_samples")
            if p not in rt.sic_params_used]
        summary["sic"]["source_credits"] = {
            "in_flight_max": int(credit.in_flight_max),
            "iq_budget_bytes": int(credit.iq_budget_bytes),
            "offered": int(credit.offered),
            "admitted": int(credit.admitted),
            "missed_admission": int(credit.missed_admission),
            "miss_reasons": dict(credit.by_reason),
            "inflight_high_watermark": int(credit.inflight_high_watermark),
            "pending_bytes_peak": int(credit.pending_bytes_peak),
            "window_bytes": int(generator.n * 8),
        }
    summary["demod"] = {
        "clean_ok": int(clean_ok), "raw_ok": int(raw_ok),
        "sic_ok": (int(sic_ok) if sic_on else None),
        "completed": int(demod_ran), "windows": int(windows),
        "psdu_ok": int(psdu_match),
        "raw_ok_sic_fail": (int(raw_ok_sic_fail) if sic_on else None),
        "raw_fail_sic_ok": (int(raw_fail_sic_ok) if sic_on else None),
        "failing_stages": sorted(
            c for c, hist in failing_status.items()
            if any(st not in TERMINAL_STATUS_OK for st in hist)),
        "clean_psdu": S.DEFAULT_PSDU_HEX,
    }

    # effect aggregates over the NON-truth windows (exact verifier formula).
    sup3 = [1.0 for s in eff_supp if s >= 3.0]
    eff_abs_err = [abs(v) for v in eff_target_err_db]
    summary["effect"] = {
        "median_suppression_db": px_median(eff_supp),
        "frac_ge_3db": (len(sup3) / len(eff_supp)) if eff_supp else None,
        "target_amp_err_db_p95": px_p95(eff_target_err_db),
        "target_amp_abs_err_db_p95": px_p95(eff_abs_err),
        "peak_shift_max": (max(eff_peak_shift) if eff_peak_shift else None),
        "nmse_median": px_median(eff_nmse),
        "holdout_nmse_median": px_median(eff_holdout_nmse),
        "holdout_suppression_db_median": px_median(eff_holdout_supp),
        "n_windows": len(eff_supp),
        "oracle_windows": int(oracle_windows),
        "oracle_excluded": True,
        "note": ("primary gate: median suppression >=10 dB AND >=95% windows "
                 ">=3 dB, target |amp err| <=1 dB (p95) and |peak shift| <=1 "
                 "work sample. Aggregate excludes uses_truth windows."),
    }
    eff_ok = bool(
        sic_on and args.sic == "estimate" and args.preset == "main" and
        oracle_windows == 0 and
        summary["effect"]["median_suppression_db"] is not None and
        summary["effect"]["median_suppression_db"] >= 10.0 and
        summary["effect"]["frac_ge_3db"] is not None and
        summary["effect"]["frac_ge_3db"] >= 0.95 and
        summary["effect"]["target_amp_abs_err_db_p95"] is not None and
        summary["effect"]["target_amp_abs_err_db_p95"] <= 1.0 and
        summary["effect"]["peak_shift_max"] is not None and
        summary["effect"]["peak_shift_max"] <= 1)

    rss = resource.getrusage(resource.RUSAGE_SELF)
    summary["runtime"] = {
        "offered": completed_windows, "admitted": completed_windows,
        "completed": completed_windows, "failed": 0, "bypassed": 0,
        "dropped": 0,
        "planned": windows,
        "deadline_misses": (paced_deadline_miss if paced else deadline_miss),
        "inflight_hw": (rt.demod.queue_high_watermark() if rt.want_demod
                        else (rt.cir.queue_high_watermark()
                              if rt.want_cir else 0)),
        "queue_hw": (rt.cir.queue_high_watermark() if rt.want_cir else 0),
        "rss_bytes": int(rss.ru_maxrss * 1024),
        "cpu_seconds": float(rss.ru_utime + rss.ru_stime),
        "demod_dropped": (int(rt.demod.jobs_dropped()) if rt.want_demod
                          else None),
        "cir_dropped": (int(rt.cir.pdus_dropped()) if rt.want_cir else None),
        "sic_offered": (int(rt.sic.windows_offered()) if sic_on else None),
        "sic_admitted": (int(rt.sic.windows_admitted()) if sic_on else None),
        "sic_missed_admission": (int(rt.sic.windows_missed_admission())
                                 if sic_on else None),
    }
    if sic_on:
        summary["runtime"].update({
            "source_offered": int(credit.offered),
            "source_admitted": int(credit.admitted),
            "source_missed_admission": int(credit.missed_admission),
            "source_inflight_hw": int(credit.inflight_high_watermark),
            "source_pending_bytes_peak": int(credit.pending_bytes_peak),
            "in_flight_max": int(credit.in_flight_max),
            "iq_budget_bytes": int(credit.iq_budget_bytes),
        })
    # F7: bounded-result-sink evidence (kept IQ arrays vs dropped).
    summary["runtime"]["sinks"] = {
        "keep_iq": int(args.keep_iq),
        "cir": ({"count": rt.sink_cir.count,
                 "retained": len(rt.sink_cir.retained),
                 "dropped_iq": rt.sink_cir.dropped_iq}
                if rt.sink_cir is not None else None),
        "demod": ({"count": rt.sink_demod.count,
                   "retained": len(rt.sink_demod.retained),
                   "dropped_iq": rt.sink_demod.dropped_iq}
                  if rt.sink_demod is not None else None),
        "sic_out": ({"count": rt.sink_sic_out.count,
                     "retained": len(rt.sink_sic_out.retained),
                     "dropped_iq": rt.sink_sic_out.dropped_iq}
                    if rt.sink_sic_out is not None else None),
        "sic_ihat": ({"count": rt.sink_sic_ihat.count,
                      "retained": len(rt.sink_sic_ihat.retained),
                      "dropped_iq": rt.sink_sic_ihat.dropped_iq}
                     if rt.sink_sic_ihat is not None else None),
    }
    g0_parts = [g0.get("cir_canonical_pass"),
                g0.get("wrong_template_pass"),
                g0.get("determinism_pass"),
                g0.get("window_bounds_pass")]
    if rt.want_demod:
        g0_parts.append(g0.get("psdu_canonical_pass"))
    if rt.want_cir:
        g0_parts.append(g0.get("cir_clean_pass"))
    summary["gates"] = {
        "eff_pass": bool(eff_ok),
        "sic_speed_pass": False,
        "demod_speed_pass": False, "native_pass": False,
        "g0_pass": bool(all(g0_parts)),
        "note": ("g0_pass is the S0-A closure gate over the checks applicable "
                 "to the selected --consumer; eff_pass is the S0 primary "
                 "effect gate (main/estimate only); sic_speed/demod_speed/"
                 "native belong to later stages and are not claimed here."),
    }
    g0["g0_pass"] = summary["gates"]["g0_pass"]
    if not eff_ok and sic_on:
        not_run.append("gates.eff_pass (primary effect gate not met or not "
                       "applied: preset=%s sic=%s)" % (args.preset, args.sic))

    # ---------------- manifests / tables ----------------
    manifest = {
        "run_id": run_id,
        "commit": subprocess.run(["git", "rev-parse", "HEAD"],
                                 capture_output=True, text=True).stdout.strip(),
        "dirty": bool(subprocess.run(["git", "status", "--porcelain"],
                                     capture_output=True, text=True
                                     ).stdout.strip()),
        "build_type": cmake_cache_value(build_dir, "CMAKE_BUILD_TYPE"),
        "compiler": subprocess.run(["c++", "--version"], capture_output=True,
                                   text=True).stdout.splitlines()[0]
        if subprocess.run(["which", "c++"], capture_output=True).returncode
        == 0 else None,
        "flags": [cmake_cache_value(build_dir, "CMAKE_CXX_FLAGS") or "",
                  cmake_cache_value(build_dir, "CMAKE_CXX_FLAGS_RELEASE") or
                  ""],
        "lib_paths": {"gnuradio_uwb": so,
                      "loaded": ldd_lines(so)},
        "cpu": cpu_model(),
        "os": platform.platform(),
        "gnuradio_version": gr.version(),
        "volk_version": volk_version(),
        "config_path": os.path.join(repo, "testdata", "sic_preamble", "cases",
                                    "%s.json" % args.preset),
        "config_hash": config_hash,
        "config": resolved_config,
        "fixture_hashes": {},
        "template_hashes": {
            "sensing_sync_code%d" % sensing_code:
                generator.sensing_template_info().sha256,
            "comm_preamble_code%d" % comm_code:
                (generator.comm_template_info().sha256
                 if generator.comm_template_info() else
                 g0.get("wrong_template_control", {}).get(
                     "code%d" % comm_code, {}).get("comm_sha256")),
            "sensing_tx": S.sha256_array(generator.tx),
        },
        "template_meta": {
            "sensing_sync_code%d" % sensing_code: {
                "length": generator.sensing_template_info().length,
                "source": "hrp_packet_source(sync_reps=%d, code=%d, "
                          "sfd=%s)[:1016]" % (sync_reps, sensing_code,
                                              SFD_MODE),
                "sha256": generator.sensing_template_info().sha256,
            },
            "comm_preamble_code%d" % comm_code: {
                "length": (generator.comm_template_info().length
                           if generator.comm_template_info() else
                           g0.get("wrong_template_control", {}).get(
                               "code%d" % comm_code, {}).get("comm_len")),
                "source": "hrp_packet_source(code=%d)[:sync_reps*1016+"
                          "pulse_taps]" % comm_code,
                "sha256": (generator.comm_template_info().sha256
                           if generator.comm_template_info() else
                           g0.get("wrong_template_control", {}).get(
                               "code%d" % comm_code, {}).get("comm_sha256")),
            },
            "sensing_tx": {
                "length": int(generator.tx.size),
                "source": "hrp_packet_source(code=%d, insert_sts=False)"
                          % sensing_code,
                "sha256": S.sha256_array(generator.tx),
            },
        },
        "seed_lists": {            "calibration": seeds_all.get("calibration", {}).get("list"),
            "acceptance": seeds_all.get("acceptance", {}).get("list"),
            "used": run_seeds, "used_set": args.seed_set,
        },
        "sample_rate_work": S.WORK_HZ,
        "sample_rate_native": S.CG400_HZ,
        "input_grid": args.input_grid,
        "preset": args.preset,
        "sic_mode": args.sic,
        "insert_sts": False,
        "insert_sts_note": (
            "The S0 sensing full packet uses insert_sts=False.  The project's "
            "hrp_packet_source places the STS field immediately after the SFD, "
            "so UwbRealtimeDemodulator (which expects PHR right after the SFD) "
            "cannot decode the STS variant.  SYNC/SFD are bit-identical to the "
            "canonical STS packet (verified: sync_region_max_abs_diff=0.0), so "
            "the canonical CIR golden still applies."),
        "generator_geometry": generator.geom,
        "demod_softchip_mode": "bypass",
        "sic_k": int(sic_k),
        "sic_config": ({"comm_code": int(sic_cfg["comm_code"]),
                        "sync_reps": int(sic_cfg["sync_reps"]),
                        "fit_reps_source": fit_source,
                        "k_paths": int(sic_cfg["k_paths"]),
                        "fit_reps": int(sic_cfg["fit_reps"]),
                        "validation_reps": int(sic_cfg["validation_reps"]),
                        "roi": [int(sic_cfg["roi"][0]),
                                int(sic_cfg["roi"][1])],
                        "coarse_stride": int(sic_cfg["coarse_stride"]),
                        "mode": str(sic_cfg["mode"]),
                        "detect_threshold": float(
                            sic_cfg["detect_threshold"]),
                        "validate_threshold": float(
                            sic_cfg["validate_threshold"]),
                        "in_flight_max": int(sic_cfg["in_flight_max"]),
                        "iq_budget_bytes": int(sic_cfg["iq_budget_bytes"]),
                        "queue_capacity": int(sic_cfg["queue_capacity"]),
                        "post_tail": int(sic_cfg.get("post_tail", 100)),
                        "path_min_spacing": int(
                            sic_cfg.get("path_min_spacing", 0)),
                        "max_window_samples": int(
                            sic_cfg.get("max_window_samples", 0)),
                        "enable_fractional": bool(
                            sic_cfg.get("enable_fractional", True)),
                        "enable_cfo": bool(sic_cfg.get("enable_cfo", True)),
                        "make_params_used": sorted(rt.sic_params_used),
                        "make_params_unsupported": [
                            p for p in ("post_tail", "enable_fractional",
                                        "enable_cfo", "path_min_spacing",
                                        "max_window_samples")
                            if p not in rt.sic_params_used],
                        "sync_sha256": (S.sha256_array(sic_sync)
                                        if sic_on and sic_sync is not None
                                        else None),
                        "preamble_sha256": (S.sha256_array(sic_preamble)
                                            if sic_on and
                                            sic_preamble is not None else None),
                        "sic_template_source": sic_tpl_source}
                       if sic_cfg else None),
    }
    if sic_on and sic_sync is not None:
        manifest["template_hashes"]["sic_sync_code%d" % sic_comm_code] = \
            S.sha256_array(sic_sync)
        manifest["template_meta"]["sic_sync_code%d" % sic_comm_code] = {
            "length": int(sic_sync.size),
            "source": "hrp_packet_source(code=%d)[:1016]" % sic_comm_code,
            "sha256": S.sha256_array(sic_sync),
        }
    if sic_on and sic_preamble is not None:
        manifest["template_hashes"]["sic_preamble_code%d" % sic_comm_code] \
            = S.sha256_array(sic_preamble)
        manifest["template_meta"]["sic_preamble_code%d" % sic_comm_code] = {
            "length": int(sic_preamble.size),
            "source": "hrp_packet_source(code=%d)[:sync_reps*1016+pulse_taps] "
                      "for the SIC adapter (nominal comm code)"
                      % sic_comm_code,
            "sha256": S.sha256_array(sic_preamble),
        }
    for wid in first_windows:
        p = os.path.join(fix_dir, "window_%04d.npz" % wid)
        if os.path.isfile(p):
            manifest["fixture_hashes"][os.path.relpath(p, args.output)] = \
                file_sha256(p)
        rp = os.path.join(args.output, "recon", "window_%d.npz" % wid)
        if os.path.isfile(rp):
            manifest["fixture_hashes"][os.path.relpath(rp, args.output)] = \
                file_sha256(rp)
    for tp in (comm_sync_path, comm_pre_path, sic_sync_path, sic_pre_path):
        if tp and os.path.isfile(tp):
            manifest["fixture_hashes"][os.path.relpath(tp, args.output)] = \
                file_sha256(tp)
    for rel in ("testdata/uwb_radar/rx_clean_998p4.cf32",
                "testdata/uwb_radar/cir_norm_clean_radar.cf32",
                "testdata/uwb_radar/tx_998p4.cf32"):
        p = os.path.join(repo, rel)
        if os.path.isfile(p):
            manifest["fixture_hashes"]["repo:" + rel] = file_sha256(p)
    write_json(os.path.join(args.output, "run_manifest.json"), manifest)

    with open(os.path.join(args.output, "stage_times.csv"), "w",
              encoding="utf-8") as f:
        f.write("run_id,window_id,consumer,scope,tier,stage,us,detail\n")
        for r in stage_rows:
            f.write("%s,%d,%s,%s,%s,%s,%s,%s\n" % (
                r[0], r[1], r[2], r[3], r[4], r[5], ("%.3f" % r[6])
                if isinstance(r[6], float) else r[6], r[7]))
    write_json(os.path.join(args.output, "summary.json"), summary)

    # ---------------- print + exit ----------------
    print("\n=== S0-A G0 checks ===", flush=True)
    for key in ("psdu_canonical_pass", "cir_canonical_pass",
                "cir_clean_pass", "wrong_template_pass",
                "determinism_pass", "window_bounds_pass", "g0_pass"):
        print("  %-24s %s" % (key, g0.get(key)), flush=True)
    print("  psdu_ok %d/%d" % (psdu_match, demod_ran), flush=True)
    print("  cir_canonical max_abs_diff_norm", canon.get("max_abs_diff_norm"),
          flush=True)
    print("  cir_clean_consumer",
          json.dumps(g0.get("cir_clean_consumer", {}), sort_keys=True),
          flush=True)
    print("  wrong_template_control",
          json.dumps(g0.get("wrong_template_control", {}), sort_keys=True),
          flush=True)
    print("  determinism", json.dumps(g0.get("determinism", {})), flush=True)
    print("  gates", json.dumps(summary["gates"], sort_keys=True), flush=True)
    print("  demod", json.dumps(summary["demod"], sort_keys=True), flush=True)
    if sic_on:
        print("  effect", json.dumps(summary["effect"], sort_keys=True),
              flush=True)
        print("  sic adapter", json.dumps(summary["sic"].get("adapter"),
                                          sort_keys=True), flush=True)
        print("  sic total mean us", summary["sic"].get("total_mean_us"),
              flush=True)
    print("  clean total mean us",
          summary["clean"].get("total_mean_us"), flush=True)
    print("  raw total mean us", summary["raw"].get("total_mean_us"),
          flush=True)
    print("  output", args.output, flush=True)

    ok = bool(summary["gates"]["g0_pass"])
    raise SystemExit(0 if ok else 4)


if __name__ == "__main__":
    main()
