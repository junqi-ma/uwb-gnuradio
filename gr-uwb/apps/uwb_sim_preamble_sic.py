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


def actual_cancellation(y, y_clean, i_true, lo, hi):
    """R01 actual-output cancellation over [lo, hi).

    ``y`` is the raw SIC input, ``y_clean`` is the IQ the SIC actually output;
    ``delta_actual = y - y_clean`` and ``residual = i - delta_actual``.  The
    metric is evaluated over the generator-side truth overlap (never a mask
    chosen to look best).  ``||i||^2 == 0`` is N/A by the frozen contract.

    Returns a dict (or None for an empty interval).  ``i_energy`` and
    ``output_changed`` are carried for the zero-interference accounting.
    """
    n = int(min(len(y), len(y_clean), len(i_true)))
    lo = max(0, int(lo))
    hi = min(n, int(hi))
    if hi <= lo:
        return None
    yy = y[lo:hi].astype(np.complex128)
    yc = y_clean[lo:hi].astype(np.complex128)
    ii = i_true[lo:hi].astype(np.complex128)
    den = float(np.vdot(ii, ii).real)
    changed = bool(np.any(yy != yc))
    if den <= 0.0:
        return {"actual_nmse": None, "actual_suppression_db": None, "na": True,
                "i_energy": 0.0, "output_changed": changed}
    delta = yy - yc
    resid = ii - delta
    num = float(np.vdot(resid, resid).real)
    nmse = num / den
    sup = (-10.0 * math.log10(nmse)) if num > 0.0 else 60.0
    return {"actual_nmse": float(nmse), "actual_suppression_db": float(sup),
            "na": False, "i_energy": float(den), "output_changed": changed,
            "delta_energy": float(np.vdot(delta, delta).real)}


def changed_support(y, y_clean, lo, hi):
    """The real ``y != y_clean`` support inside [lo, hi) (the applied mask)."""
    n = int(min(len(y), len(y_clean)))
    lo = max(0, int(lo))
    hi = min(n, int(hi))
    if hi <= lo:
        return 0, 0
    idx = np.nonzero(y[lo:hi] != y_clean[lo:hi])[0]
    if idx.size == 0:
        return 0, 0
    return lo + int(idx[0]), lo + int(idx[-1]) + 1


def region_metric(i_true, i_hat, lo, hi):
    """``(nmse, suppression_db, lo, hi)`` over [lo,hi) plus a zero-energy flag.

    Mirrors the frozen convention: ``||i||^2 == 0`` yields NMSE 0 / 60 dB when
    the reconstruction is also zero, else NMSE 1 / 0 dB.  Returns
    ``(None, False)`` for an empty interval."""
    m = recon_pair(i_true, i_hat, lo, hi)
    if m is not None:
        return m, False
    n = int(min(len(i_true), len(i_hat)))
    lo = max(0, int(lo))
    hi = min(n, int(hi))
    if hi <= lo:
        return None, False
    num = float(np.sum(np.abs(i_hat[lo:hi].astype(np.complex128)) ** 2))
    if num == 0.0:
        return (0.0, 60.0, lo, hi), True
    return (1.0, 0.0, lo, hi), True


def sync_rep_scan(y, sync_norm, anchor, reps, sps, threshold):
    """Public-template repetition scan (R02 signal-evidence).

    For each candidate SYNC repetition ``anchor + k*sps`` compute the same
    normalized correlation the core's detector uses and count the CONSECUTIVE
    repetitions whose correlation is >= ``threshold``.  Pure online data (y +
    the public template); the injected start/CFO/paths are never read."""
    n = int(len(y))
    ls = int(len(sync_norm))
    yc = np.asarray(y, dtype=np.complex128)
    sn = np.asarray(sync_norm, dtype=np.complex128)
    norms = []
    for k in range(int(reps)):
        d = int(anchor) + k * int(sps)
        if d < 0 or d + ls > n:
            norms.append(0.0)
            continue
        seg = yc[d:d + ls]
        den = float(np.linalg.norm(seg)) * float(np.linalg.norm(sn))
        norms.append(float(abs(np.vdot(sn, seg)) / den) if den > 0 else 0.0)
    consecutive = 0
    for v in norms:
        if v >= float(threshold):
            consecutive += 1
        else:
            break
    return {"norms": norms, "threshold": float(threshold),
            "consecutive_reps": int(consecutive),
            "max_norm": (max(norms) if norms else None),
            "reps_scanned": int(reps), "sps": int(sps),
            "anchor": int(anchor)}



def raw_target_metrics(cir_clean, cir_sic, target_lo, target_hi,
                       detect_ratio=3.0, false_peak_ratio=0.5,
                       main_lobe_guard=3):
    """R04 raw (non-L2-normalised) target protection metrics.

    The target tap is the frozen ROI's clean argmax (never re-searched on the
    SIC result); the amplitude/phase error is measured at that fixed tap.  The
    local peak shift is the ROI argmax difference.  ``weak_target_detected`` is
    the ROI peak relative to the out-of-ROI background; ``false_peak_count``
    counts strong local maxima outside the ROI (main lobe excluded).
    """
    cl = np.asarray(cir_clean, dtype=np.complex128).ravel()
    cs = np.asarray(cir_sic, dtype=np.complex128).ravel()
    lo, hi = int(target_lo), int(target_hi)
    if len(cl) != len(cs) or not (0 <= lo < hi) or hi > len(cl) or hi > len(cs):
        return None
    seg_c = cl[lo:hi]
    seg_s = cs[lo:hi]
    t = lo + int(np.argmax(np.abs(seg_c)))
    amp_c = abs(complex(cl[t]))
    amp_s = abs(complex(cs[t]))
    if amp_c <= 0.0:
        return None
    amp_err_db = 20.0 * math.log10(amp_s / amp_c) if amp_s > 0 else None
    dphi = math.atan2(cs[t].imag, cs[t].real) \
        - math.atan2(cl[t].imag, cl[t].real)
    dphi = (dphi + math.pi) % (2.0 * math.pi) - math.pi
    phase_err_deg = math.degrees(dphi)
    peak_c = lo + int(np.argmax(np.abs(seg_c)))
    peak_s = lo + int(np.argmax(np.abs(seg_s)))
    # Background outside the ROI (main lobe excluded by its own guard band).
    abs_s = np.abs(cs)
    in_roi = np.zeros(abs_s.size, dtype=bool)
    in_roi[lo:hi] = True
    pk = int(np.argmax(abs_s))
    g_lo = max(0, pk - int(main_lobe_guard))
    g_hi = min(abs_s.size, pk + int(main_lobe_guard) + 1)
    in_guard = np.zeros(abs_s.size, dtype=bool)
    in_guard[g_lo:g_hi] = True
    bg = abs_s[~(in_roi | in_guard)]
    noise_floor = float(np.median(bg)) if bg.size else 0.0
    roi_peak = float(abs_s[lo:hi].max())
    weak_detected = bool(noise_floor > 0 and roi_peak > detect_ratio * noise_floor)
    weak_snr_db = (20.0 * math.log10(roi_peak / noise_floor)
                   if noise_floor > 0 else None)
    false_peaks = 0
    for i in range(1, abs_s.size - 1):
        if in_roi[i] or in_guard[i]:
            continue
        if abs_s[i] > abs_s[i - 1] and abs_s[i] >= abs_s[i + 1] \
                and abs_s[i] > false_peak_ratio * roi_peak:
            false_peaks += 1
    return {
        "target_tap": int(t),
        "target_clean_peak_tap": int(peak_c),
        "target_sic_peak_tap": int(peak_s),
        "target_amp_err_db": (None if amp_err_db is None else float(amp_err_db)),
        "target_phase_err_deg": float(phase_err_deg),
        "target_peak_shift": int(peak_s - peak_c),
        "target_amp_clean": float(amp_c),
        "target_amp_sic": float(amp_s),
        "cir_raw_target_amp_abs_db": (20.0 * math.log10(amp_s)
                                      if amp_s > 0 else None),
        "weak_target_detected": weak_detected,
        "weak_target_snr_db": weak_snr_db,
        "false_peak_count": int(false_peaks),
        "noise_floor": noise_floor,
    }


def load_target_roi(repo, profile_id):
    """Load the frozen R04 target ROI for a sensing profile (never re-searched).

    The ROI/reference are fixed on the calibration set and stored in
    ``testdata/sic_preamble/cases/target_roi.json`` (hashed into the manifest).
    """
    path = os.path.join(repo, "testdata", "sic_preamble", "cases",
                        "target_roi.json")
    if not os.path.isfile(path):
        return None, None
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    profiles = cfg.get("profiles", {})
    want = str(profile_id).lower()
    prof = None
    for key, val in profiles.items():
        if str(key).lower() == want:
            prof = val
            break
    if prof is None:
        return None, path
    roi = prof.get("target_roi")
    if isinstance(roi, (list, tuple)) and len(roi) == 2:
        lo, hi = int(roi[0]), int(roi[1])
    else:
        lo, hi = int(prof["target_lo"]), int(prof["target_hi"])
    prof = dict(prof)
    prof["target_lo"] = lo
    prof["target_hi"] = hi
    prof["target_roi"] = [lo, hi]
    prof["_config_path"] = path
    return prof, path


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


def _git_capture(repo, args):
    try:
        out = subprocess.run(["git", "-C", repo] + list(args),
                             capture_output=True, text=True, timeout=120)
        return out.stdout
    except Exception:  # pragma: no cover - defensive
        return ""


def git_worktree_provenance(repo):
    """R10: the real commit plus a sha256 id of the working-tree diff.

    The dirty-patch identifier hashes ``git diff HEAD`` together with the
    untracked-file listing so a dirty tree always carries a stable, non-empty
    id (never the empty-tree sha256).  A clean tree is reported as
    ``dirty=False``.  Everything is re-derived from the working tree at run
    time; nothing is read back from a previous manifest.
    """
    head = _git_capture(repo, ["rev-parse", "HEAD"]).strip()
    branch = _git_capture(repo, ["rev-parse", "--abbrev-ref", "HEAD"]).strip()
    status = _git_capture(repo, ["status", "--porcelain"])
    diff = _git_capture(repo, ["diff", "HEAD"])
    untracked = _git_capture(repo, ["ls-files", "--others", "--exclude-standard"])
    h = hashlib.sha256()
    h.update(diff.encode("utf-8", "replace"))
    h.update(b"\x00--untracked--\x00")
    h.update(untracked.encode("utf-8", "replace"))
    return {"head": head, "branch": branch,
            "dirty": bool(status.strip()),
            "status_sha256": h.hexdigest()}


def pick_loaded_library(build_dir):
    """The real ``libgnuradio-uwb.so*`` file (not the symlink) under build_dir."""
    import glob
    candidates = sorted(glob.glob(
        os.path.join(build_dir, "lib", "libgnuradio-uwb.so*")))
    for cand in candidates:
        if not os.path.islink(cand) and os.path.isfile(cand):
            return cand
    for cand in candidates:
        real = os.path.realpath(cand)
        if os.path.isfile(real):
            return real
    return None


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


def _proc_peak_rss_bytes():
    """Peak resident set (VmHWM) for THIS process, or None."""
    try:
        with open("/proc/self/status", "r", encoding="utf-8",
                  errors="replace") as f:
            for line in f:
                if line.startswith("VmHWM:"):
                    return int(line.split()[1]) * 1024
    except Exception:
        pass
    return None


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
                 demod_softchip="bypass", sic=None, keep_iq=4,
                 demod_rake_top_k=0, radar_use_predicted=False):
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
        rake = int(demod_rake_top_k) if demod_softchip == "rake4" else 0
        if demod_softchip == "rake4" and rake <= 0:
            rake = 4
        mode = "rake" if demod_softchip == "rake4" else demod_softchip
        if self.want_cir:
            self.cir = uwb.radar_cir_estimator(
                tpl_path, 64, SFD_MODE, 9, 16, 100, 10, 0, 128, 32, 0.12,
                0.02, True, qcap, bool(radar_use_predicted))
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


class NativeFrontEnd:
    """R08 CG400 native front-end: common-scale SC16 + PDU 65/32 resample.

    Owns the production ``pdu_rational_resampler_ccf_65_32`` block (SC16 input,
    UnitRange policy so the work-grid amplitude matches the SC16/32768 scale)
    in its own top block.  Every window is quantized with ONE scale shared by
    its clean/raw mixtures; the resampler maps 491.52 -> 998.4 with the frozen
    group-delay-centered map.  The known SIMD non-determinism is recorded (via
    repeat runs), never fixed here.
    """

    def __init__(self, uwb, *, taps: str, workers: int = 1,
                 headroom: float = 1.0, native_rate: float = S.CG400_HZ):
        self.uwb = uwb
        self.taps_name = taps
        self.workers = max(1, int(workers))
        self.headroom = float(headroom)
        self.native_rate = float(native_rate)
        self.tb = gr.top_block("uwb_native_frontend")
        self.res = uwb.pdu_rational_resampler_ccf_65_32(
            taps, S.WORK_HZ, True, 2097152, self.workers,
            uwb.Sc16ScalePolicy.UnitRange)
        self.sink = ResultSink("native_resample_sink")
        self.tb.msg_connect((self.res, "packet"), (self.sink, "in"))
        self.tap_count = int(self.res.tap_count())
        self.gd_in = int(round(S.native_group_delay_input(self.tap_count)))
        self.kernel = None
        try:
            self.kernel = self.res.resampler_kernel()
        except Exception:
            self.kernel = None
        self.total_windows = 0
        self.total_in_samples = 0

    def start(self):
        self.tb.start()

    def stop(self):
        self.tb.stop()
        self.tb.wait()

    def _meta(self, n_in: int, wid: int):
        gd = self.gd_in
        pre = min(gd, max(0, n_in // 4))
        post = min(gd, max(0, n_in // 4))
        cap = max(0, n_in - pre - post)
        meta = pmt.make_dict()
        meta = put_float(meta, "sample_rate", self.native_rate)
        meta = put_str(meta, "sample_format", "sc16")
        meta = put_int(meta, "window_start_sample", 0)
        meta = put_int(meta, "pre_guard_samples", pre)
        meta = put_int(meta, "capture_samples", cap)
        meta = put_int(meta, "post_guard_samples", post)
        meta = put_int(meta, "sample_count", int(n_in))
        meta = put_int(meta, "packet_id", int(wid))
        meta = put_int(meta, "schedule_index", int(wid))
        return meta

    def resample(self, nat_i16: np.ndarray, wid: int, deadline_s: float = 180.0):
        """Post one SC16 native PDU; return (work_cf32, timings_us)."""
        n_in = int(np.asarray(nat_i16).size // 2)
        r0 = int(self.res.resample_total_us())
        h0 = int(self.res.handler_total_us())
        c0 = int(self.res.input_convert_total_us())
        n_out = self.sink.count + 1
        t_wrap0 = time.perf_counter()
        vec = pmt.init_s16vector(int(nat_i16.size), np.asarray(nat_i16,
                                                               dtype=np.int16
                                                               ).tolist())
        t_wrap = (time.perf_counter() - t_wrap0) * 1e6
        t_post0 = time.perf_counter()
        self.res.to_basic_block()._post(
            pmt.intern("packet"), pmt.cons(self._meta(n_in, wid), vec))
        t_post = (time.perf_counter() - t_post0) * 1e6
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            if self.sink.count >= n_out:
                break
            time.sleep(0.0005)
        if self.sink.count < n_out:
            raise SystemExit("native resampler did not emit window %d "
                             "(in=%d)" % (wid, n_in))
        d = self.sink.latest
        wf = np.ascontiguousarray(d.get("__c32", np.zeros(0, np.complex64)),
                                  dtype=np.complex64)
        stats = {
            "wrap_us": t_wrap, "post_us": t_post,
            "resample_us": int(self.res.resample_total_us()) - r0,
            "handler_us": int(self.res.handler_total_us()) - h0,
            "convert_us": int(self.res.input_convert_total_us()) - c0,
        }
        self.total_windows += 1
        self.total_in_samples += n_in
        return wf, stats

    def stats(self):
        return {
            "taps": self.taps_name,
            "tap_count": int(self.tap_count),
            "workers": int(self.workers),
            "kernel": self.kernel,
            "group_delay_input_native": float(
                S.native_group_delay_input(self.tap_count)),
            "group_delay_output_work": float(
                S.native_group_delay_output(self.tap_count)),
            "pdus_received": int(self.res.pdus_received()),
            "pdus_emitted": int(self.res.pdus_emitted()),
            "pdus_dropped": int(self.res.pdus_dropped()),
            "short_guard_events": int(self.res.short_guard_events()),
            "resets": int(self.res.resets()),
            "total_input_samples": int(self.res.total_input_samples()),
            "total_output_samples": int(self.res.total_output_samples()),
            "resample_total_us": int(self.res.resample_total_us()),
            "resample_max_us": int(self.res.resample_max_us()),
            "handler_total_us": int(self.res.handler_total_us()),
            "input_convert_total_us": int(self.res.input_convert_total_us()),
            "publish_total_us": int(self.res.publish_total_us()),
        }


def _native_repeat_probe(front, i16, wid, repeats):
    """Repeat the SAME native PDU to expose the known SIMD non-determinism."""
    outs = []
    dmax = 0.0
    for r in range(max(1, int(repeats))):
        wf, _ = front.resample(i16, wid * 1000 + r)
        outs.append(wf)
    ref = outs[0]
    for o in outs[1:]:
        if o.size == ref.size:
            dmax = max(dmax, float(np.max(np.abs(o - ref))))
        else:
            dmax = float("inf")
    return {"repeats": len(outs), "identical": bool(dmax == 0.0),
            "max_abs_diff": dmax}


def _native_scalar_reference(front, nat_i16, work_block):
    """Independent scipy upfirdn reference (same taps, same dequantized data).

    The reference is fed the EXACT int16 PDU payload dequantized with the block's
    UnitRange policy (int16/32768), so the only residual is the C++ FIR / kernel
    arithmetic vs scipy -- the scalar/reference error floor.
    """
    taps = np.asarray(front.res.taps(), dtype=np.float64)
    iq = np.asarray(nat_i16, dtype=np.float64)
    x = iq[0::2] / 32768.0 + 1j * (iq[1::2] / 32768.0)
    ref = S.reference_resample_65_32(x, taps)
    a = np.asarray(work_block, dtype=np.complex128)
    b = np.asarray(ref, dtype=np.complex128)
    m = min(a.size, b.size)
    if m == 0:
        return {"ran": False, "reason": "empty"}
    a = a[:m]
    b = b[:m]
    cross = np.correlate(a, b, mode="full")
    lag = int(np.argmax(np.abs(cross)) - (b.size - 1))
    if lag >= 0:
        aa = a[lag:]
        bb = b[:aa.size]
    else:
        bb = b[-lag:]
        aa = a[:bb.size]
    k = min(aa.size, bb.size)
    aa, bb = aa[:k], bb[:k]
    if k == 0:
        return {"ran": False, "reason": "alignment empty", "lag": lag}
    d = aa - bb
    den = float(np.vdot(aa, aa).real)
    return {
        "ran": True, "lag": lag, "compared_samples": int(k),
        "max_abs_diff": float(np.max(np.abs(d))),
        "rms_diff": float(np.sqrt(np.mean(np.abs(d) ** 2))),
        "nmse": (float(np.vdot(d, d).real) / den if den > 0 else None),
        "reference": "scipy.signal.upfirdn (same taps, same int16 payload)",
        "note": ("residual is the C++ FIR vs scipy scalar arithmetic only (the "
                 "known resampler SIMD path); it is the numerical error floor, "
                 "not a filter mismatch"),
    }


def _estimate_lag(a, b):
    """Integer lag so that a[n] ~= b[n - lag] (peak of the cross-correlation)."""
    a = np.asarray(a, dtype=np.complex128).ravel()
    b = np.asarray(b, dtype=np.complex128).ravel()
    m = min(a.size, b.size)
    if m < 8:
        return 0
    cross = np.correlate(a[:m], b[:m], mode="full")
    return int(np.argmax(np.abs(cross)) - (m - 1))


def native_transform_window(front, win, *, comm_present: bool,
                            headroom: float, wid: int,
                            do_scalar: bool = False,
                            repeat_check: int = 0):
    """R08: turn one work-grid window into the three-way native capture.

    Returns ``(clean_work, raw_work, i_metric, ov_lo, ov_hi, rec)`` where
    ``clean_work``/``raw_work`` are the PDU 65/32 outputs (CF32, work grid),
    ``i_metric = raw_work - clean_work`` is the *realized* interference under
    the identical common scaling/filtering/valid-range convention, and
    ``[ov_lo, ov_hi)`` is the truth overlap mapped into that same domain.
    """
    s = np.asarray(win["s"], dtype=np.complex128)
    i = np.asarray(win["i"], dtype=np.complex128)
    w = np.asarray(win["w"], dtype=np.complex128)
    clean_w = s + w
    raw_w = s + i + w

    s_nat = S.work_to_native(s)
    i_nat = S.work_to_native(i)
    w_nat = S.work_to_native(w)
    raw_nat = s_nat + i_nat + w_nat
    clean_nat = s_nat + w_nat

    peaks = [float(np.max(np.abs(raw_nat))) if raw_nat.size else 0.0,
             float(np.max(np.abs(clean_nat))) if clean_nat.size else 0.0]
    scale = S.common_sc16_scale(peaks, headroom=headroom)

    raw_i16, rci, rcq, _ = S.quantize_sc16(raw_nat, scale)
    cln_i16, cci, ccq, _ = S.quantize_sc16(clean_nat, scale)

    # The two mixtures share the SAME scale and the SAME resampler/valid range.
    raw_work, rstats = front.resample(raw_i16, wid * 2)
    cln_work, cstats = front.resample(cln_i16, wid * 2 + 1)

    lag = _estimate_lag(cln_work, clean_w)
    ov = win.get("overlap") or {"lo": 0, "hi": 0}
    ov_lo = max(0, int(ov["lo"]) + int(lag))
    ov_hi = min(min(raw_work.size, cln_work.size), int(ov["hi"]) + int(lag))

    n_i = min(raw_work.size, cln_work.size)
    i_metric = (raw_work[:n_i].astype(np.complex128)
                - cln_work[:n_i].astype(np.complex128)).astype(np.complex64)

    # Error floor: the un-quantized mixture through the SAME taps/upsampler.  The
    # residual raw_work - ideal is the SC16 quantization error + numerical noise.
    # scipy resample_poly is NOT used here (its Kaiser filter would masquerade
    # as a floor); the reference uses the production taps.
    ideal = S.reference_resample_65_32(raw_nat, np.asarray(front.res.taps(),
                                                           dtype=np.float64))
    # The block dequantizes the int16 payload with 1/32768, while this window
    # was quantized with `scale`; the produced work amplitude is therefore
    # (scale/32768) * ideal.  Compensate before measuring the residual.
    dequant_gain = float(scale) / 32768.0
    ideal = ideal * dequant_gain
    a = raw_work.astype(np.complex128)
    lag_ideal = _estimate_lag(a, ideal)
    if lag_ideal >= 0:
        aa = a[lag_ideal:]
        bb = ideal[:aa.size]
    else:
        bb = ideal[-lag_ideal:]
        aa = a[:bb.size]
    kf = min(aa.size, bb.size)
    if kf > 0:
        err = aa[:kf] - bb[:kf]
        den = float(np.vdot(bb[:kf], bb[:kf]).real)
        floor = {
            "compared_samples": int(kf), "lag": int(lag_ideal),
            "effective_noise_rms": float(np.sqrt(np.mean(np.abs(err) ** 2))),
            "effective_noise_nmse": (float(np.vdot(err, err).real) / den
                                     if den > 0 else None),
            "note": ("y_work - ideal_filtered(s+i+w) = SC16 quantization + "
                     "numerical error under the SAME production taps; NOT an "
                     "interference gain and not an SIC implementation error"),
        }
    else:
        floor = {"compared_samples": 0}

    rec = {
        "ran": True,
        "iq_scale": float(scale),
        "headroom": float(headroom),
        "scale_method": "common_sc16_scale=32767*headroom/max_peak(all mixtures)",
        "native_rate": S.CG400_HZ,
        "work_rate": S.WORK_HZ,
        "native_samples": int(raw_nat.size),
        "work_samples_in": int(raw_w.size),
        "work_samples_out": int(raw_work.size),
        "clean_clip_i": int(cci), "clean_clip_q": int(ccq),
        "raw_clip_i": int(rci), "raw_clip_q": int(rcq),
        "peak_native": max(peaks),
        "common_scale_shared": True,
        "per_component_normalization": False,
        "nonlinearity": S.quantize_nonlinearity(s_nat, i_nat, w_nat, scale),
        "roundtrip_lag_work": int(lag),
        "roundtrip_lag_expected_work": float(
            S.native_group_delay_output(front.tap_count)),
        "resampler_group_delay_input_native": float(
            S.native_group_delay_input(front.tap_count)),
        "resampler_group_delay_output_work": float(
            S.native_group_delay_output(front.tap_count)),
        "resampler_taps": int(front.tap_count),
        "resampler_kernel": front.kernel,
        "valid_lo": int(front.gd_in),
        "valid_hi": int(max(0, raw_nat.size - front.gd_in)),
        "filter_delay": float(S.native_group_delay_input(front.tap_count)),
        "work_to_native": [S.NATIVE_DECIM, S.NATIVE_INTERP],
        "native_to_work": [S.NATIVE_INTERP, S.NATIVE_DECIM],
        "error_floor": floor,
        "resample_stats": {"raw": rstats, "clean": cstats},
        "filtering_convention": (
            "clean and raw both quantized with ONE common scale then passed "
            "through the SAME PDU 65/32 resampler; the metric reference "
            "i_metric=raw_work-clean_work is the realized interference under "
            "that identical convention (R08)"),
    }
    if do_scalar:
        rec["scalar_reference"] = _native_scalar_reference(front, raw_i16,
                                                           raw_work)
    if repeat_check and repeat_check > 1:
        rec["repeat_run_difference"] = _native_repeat_probe(
            front, raw_i16, wid, repeat_check)
    return cln_work, raw_work, i_metric, ov_lo, ov_hi, rec


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
    actual = []
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
            if _isnum(rec.get("actual_suppression_db")):
                actual.append(float(rec["actual_suppression_db"]))
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
        "median_actual_suppression_db": (float(np.median(actual)) if actual
                                         else None),
        "frac_actual_ge_3db": (sum(1.0 for s in actual if s >= 3.0)
                               / len(actual) if actual else None),
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
    # R02: propagate the EXPLICIT window config so every sweep child uses the
    # same case/config (never a silent per-child change).
    if getattr(args, "sic_fit_reps", None) is not None:
        argv += ["--sic-fit-reps", str(int(args.sic_fit_reps))]
    if getattr(args, "sic_validation_reps", None) is not None:
        argv += ["--sic-validation-reps", str(int(args.sic_validation_reps))]
    if getattr(args, "sic_cover_preamble", False):
        argv += ["--sic-cover-preamble"]
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


def run_r01_regression(args, repo, argv):
    """Emit the two R01 regression inputs as real, verifier-shaped runs.

    Case A ``case_a_tiny_mask``: a very accurate full reconstruction (small
    ``recon.nmse``) whose ACTUAL applied subtraction mask only covers the
    confirmed reps (``--sic-fit-reps 8 --sic-validation-reps 8
    --sic-no-auto-cover``).  The main actual-output gate must judge it as
    insufficient (low ``actual_suppression_db``, small ``coverage``).
    Case B ``case_b_uncommitted``: the core reconstructs but never commits
    (``validate_threshold`` unreachable) so the output is bit-identical to the
    input; the window must NOT be counted as cancelled
    (``actual_suppression_db == 0``).
    """
    root = args.output
    n = args.windows if args.windows else 8
    cases = [
        ("case_a_tiny_mask",
         ["--sic-fit-reps", "8", "--sic-validation-reps", "8",
          "--sic-no-auto-cover"],
         "accurate i_hat but a tiny applied subtraction mask"),
        ("case_b_uncommitted",
         ["--sic-validate-threshold", "0.000001"],
         "reconstruction available but not committed (bypassed output)"),
    ]
    os.makedirs(root, exist_ok=True)
    rows = []
    for cid, extra, purpose in cases:
        cout = os.path.join(root, cid)
        ca = ["--build-dir", args.build_dir, "--preset", "main",
              "--input-grid", args.input_grid, "--sic", "estimate",
              "--consumer", args.consumer, "--output", cout,
              "--seed-set", args.seed_set, "--windows", str(int(n)),
              "--interference", "on", "--fixtures",
              str(max(4, int(args.fixtures)))] + extra
        rc = _run_child(ca, os.path.join(cout, "run.log"))
        summ = _read_json(os.path.join(cout, "summary.json")) or {}
        wins = _read_windows(os.path.join(cout, "windows.jsonl"))
        eff = summ.get("effect", {}) or {}
        tgt = summ.get("target", {}) or {}
        rec = None
        for w in wins:
            if isinstance(w.get("recon"), dict):
                rec = w["recon"]
                break
        row = {
            "case": cid, "purpose": purpose, "run_dir": cout,
            "returncode": rc, "windows": len(wins),
            "recon_nmse_first": (rec or {}).get("nmse"),
            "recon_suppression_db_first": (rec or {}).get("suppression_db"),
            "actual_nmse_first": (rec or {}).get("actual_nmse"),
            "actual_suppression_db_first": (rec or {}).get(
                "actual_suppression_db"),
            "coverage_first": (rec or {}).get("coverage"),
            "sub_lo_first": (rec or {}).get("sub_lo"),
            "sub_hi_first": (rec or {}).get("sub_hi"),
            "median_actual_suppression_db": eff.get(
                "median_actual_suppression_db"),
            "frac_actual_ge_3db": eff.get("frac_actual_ge_3db"),
            "median_suppression_db": eff.get("median_suppression_db"),
            "target_amp_abs_err_db_p95": eff.get("target_amp_abs_err_db_p95"),
            "target_peak_shift_max": eff.get("target_peak_shift_max"),
            "target_lo": tgt.get("target_lo"), "target_hi": tgt.get("target_hi"),
            "eff_pass": (summ.get("gates", {}) or {}).get("eff_pass"),
            "n_interference_windows": eff.get("n_interference_windows"),
            "n_na_windows": eff.get("n_na_windows"),
        }
        if cid == "case_a_tiny_mask":
            row["judged_insufficient"] = bool(
                row["median_actual_suppression_db"] is not None and
                row["median_actual_suppression_db"] < 10.0 and
                row["recon_suppression_db_first"] is not None and
                row["recon_suppression_db_first"] >= 10.0 and
                row["eff_pass"] is not True)
        else:
            row["judged_not_cancelled"] = bool(
                row["actual_suppression_db_first"] is not None and
                abs(float(row["actual_suppression_db_first"])) <= 1e-9 and
                row["eff_pass"] is not True)
        rows.append(row)
        print("  regression %-20s rc=%d median_actual=%s recon_supp=%s "
              "coverage=%s eff_pass=%s"
              % (cid, rc, _fmt(row["median_actual_suppression_db"]),
                 _fmt(row["median_suppression_db"]),
                 _fmt(row["coverage_first"]), row["eff_pass"]), flush=True)
    out = {"mode": "r01_regression", "root": root, "windows": int(n),
           "cases": rows}
    write_json(os.path.join(root, "regression_r01.json"), out)
    print("regression_r01:", json.dumps(out, sort_keys=True), flush=True)
    return 0


def _sic_once(rt, meta, iq):
    """Post one window and return its terminal out_meta (blocking)."""
    n_out = rt.sink_sic_out.count + 1
    n_ih = rt.sink_sic_ihat.count + 1
    rt.post_sic_input(meta, iq)
    if not rt.wait_sic(n_out, n_ih, deadline_s=180.0):
        raise SystemExit("R06 data-dependency: SIC did not terminate "
                         "(out=%r i_hat=%r)" % (rt.sic_counts(),
                                                rt.sic_counts()))
    return rt.sink_sic_out.latest, rt.sink_sic_ihat.latest


def run_r06_data_dep(args, repo, argv):
    """R06 data-dependency experiment (real cores, two windows).

    Window A is the nominal window.  Window B is byte-identical to A on
    [0, eval_lo) and has a DIFFERENT interferer realization inside the
    independent final-eval region [eval_lo, eval_hi).  The frozen fit / online
    gate results (detected_start, cfo, paths, residual_corr, committed support)
    must not move; only the eval-region metric may change.
    """
    uwb = S.load_uwb(os.path.abspath(args.build_dir))
    preset = load_preset(repo, "main")
    gen = preset.get("generator", {})
    runtime = preset.get("runtime", {})
    sensing_code = int(gen.get("sensing_code", 9))
    comm_code = int(gen.get("comm_code", 10))
    sync_reps = int(gen.get("sync_reps", 64))
    fit = int(runtime.get("fit_reps", 8))
    val = int(runtime.get("validation_reps", 8))
    isr_db = float(preset.get("nominal_for_benchmark", {}).get("isr_db", 10.0))
    generator = S.SicSignalGenerator(
        uwb, sensing_code=sensing_code, comm_code=comm_code,
        sync_reps=sync_reps, sfd_mode=SFD_MODE,
        pulse_shape=S.DEFAULT_PULSE_SHAPE, comm_present=True, build_comm=True,
        snr_db=float(gen.get("snr_db", 30.0)), isr_db=isr_db, verbose=False)
    seed_specs = gen.get("seeds", {})
    seeds, _ = materialize_seeds(seed_specs.get("calibration",
                                                {"start": 0, "step": 1,
                                                 "count": 1}), 1)
    seed = int(seeds[0])
    comm_paths = gen.get("comm_paths", {}).get("single_int", [[0, [1.0, 0.0]]])
    win = generator.build(seed=seed, comm_paths=comm_paths,
                          start_offset_work=0.0, cfo_hz=0.0, isr_db=isr_db,
                          snr_db=float(gen.get("snr_db", 30.0)))
    n = generator.n
    anchor = int(generator.pre)
    eval_lo = anchor + (fit + val) * S.SPS
    eval_hi = anchor + sync_reps * S.SPS
    eval_hi = min(eval_hi, n)
    if eval_hi <= eval_lo:
        raise SystemExit("R06: fit_reps+validation_reps=%d covers sync_reps=%d; "
                         "no independent eval region exists for this config "
                         "(use --sic-validation-reps < sync_reps - fit_reps)"
                         % (fit + val, sync_reps))
    # Variant B: same s/w and same comm prefix, different eval-region content.
    alt = np.zeros(n, dtype=np.complex128)
    S.apply_path(alt, generator.comm_wave, anchor, 13.7, 0.7, "bandlimited")
    i_b = np.asarray(win["i"], dtype=np.complex128).copy()
    i_b[eval_lo:eval_hi] = alt[eval_lo:eval_hi]
    y_a = np.ascontiguousarray(win["y"], dtype=np.complex64)
    # Start from the EXACT array A and rewrite only the eval slice, so the
    # frozen prefix stays bit-identical (any float noise there would be a
    # different test).
    y_b = y_a.copy()
    y_b[eval_lo:eval_hi] = np.ascontiguousarray(
        (win["s"].astype(np.complex128) + i_b
         + win["w"].astype(np.complex128)), dtype=np.complex64)[eval_lo:eval_hi]
    prefix_equal = bool(np.array_equal(y_a[:eval_lo], y_b[:eval_lo]))

    # Build the SIC adapter exactly as the main path does (same template/ROI).
    tpl_dir = os.path.join(args.output, "templates")
    os.makedirs(tpl_dir, exist_ok=True)
    tpl_path = os.path.join(tpl_dir, "sync_template_code9_998p4.cf32")
    generator.template.tofile(tpl_path)
    search_roi = (max(0, generator.pre),
                  min(generator.n, generator.pre + sync_reps * S.SPS))
    subtraction_roi = (search_roi[0],
                       min(generator.n, search_roi[1]
                           + generator.comm_wave.size + int(args.sic_post_tail)))
    core_roi = (search_roi[0], subtraction_roi[1])
    sic_cfg = {
        "sync": generator.comm_sync, "preamble": generator.comm_wave,
        "comm_code": comm_code, "sync_reps": sync_reps,
        "k_paths": int(runtime.get("sic_k", 4)), "fit_reps": fit,
        "validation_reps": val, "fit_reps_source": "preset",
        "roi": core_roi, "coarse_stride": 14, "mode": "estimate",
        "detect_threshold": float(args.sic_detect_threshold),
        "validate_threshold": float(args.sic_validate_threshold),
        "in_flight_max": 8, "iq_budget_bytes": 64 << 20, "queue_capacity": 8,
        "post_tail": int(args.sic_post_tail),
        "path_min_spacing": 0, "max_window_samples": 0,
        "enable_fractional": True, "enable_cfo": True,
    }
    rt = SimRuntime(uwb, tpl_path=tpl_path, consumers="cir", windows=2,
                    sic=sic_cfg, keep_iq=0)
    rt.start()

    def meta_for(wid):
        m = pmt.make_dict()
        m = put_int(m, "window_id", wid)
        m = put_int(m, "sample_count", int(y_a.size))
        m = put_int(m, "code_index", sensing_code)
        m = put_int(m, "sync_repetitions", sync_reps)
        m = put_str(m, "sfd_mode", SFD_MODE)
        m = put_str(m, "sensing_profile_id",
                    "code%d-%dSYNC-%s" % (sensing_code, sync_reps, SFD_MODE))
        return m

    out_a, ihat_a_meta = _sic_once(rt, meta_for(0), y_a)
    out_b, ihat_b_meta = _sic_once(rt, meta_for(1), y_b)
    rt.stop()

    def ihat_of(meta):
        return np.ascontiguousarray(meta.get("__c32", y_a), dtype=np.complex64)

    def frozen(om):
        return {
            "detected": bool(dget(om, "sic_detected", False)),
            "committed": bool(dget(om, "sic_committed", False)),
            "detected_start": dnum(om, "sic_detected_start", -1),
            "cfo_hz": dnum(om, "sic_cfo_hz", 0.0),
            "n_paths": dnum(om, "sic_n_paths", 0),
            "support_lo": dnum(om, "sic_support_lo", -1),
            "support_hi": dnum(om, "sic_support_hi", -1),
            "residual_corr": dnum(om, "sic_residual_corr", 0.0),
            "bypass_reason": dget(om, "sic_bypass_reason", None),
        }

    fa, fb = frozen(out_a), frozen(out_b)
    important = ("detected", "committed", "detected_start", "cfo_hz", "n_paths",
                 "support_lo", "support_hi", "bypass_reason")
    exact = ("detected", "committed", "detected_start", "n_paths",
             "support_lo", "support_hi", "bypass_reason")
    frozen_equal_exact = all(fa[k] == fb[k] for k in exact)
    frozen_equal = bool(
        frozen_equal_exact and
        abs(float(fa["cfo_hz"]) - float(fb["cfo_hz"])) <= 1e-3 and
        abs(float(fa["residual_corr"]) - float(fb["residual_corr"])) <= 1e-6)
    ihat_a = ihat_of(ihat_a_meta)
    ihat_b = ihat_of(ihat_b_meta)

    def eval_metrics(i_true, ihat):
        lo = eval_lo
        hi = min(eval_hi, len(i_true), len(ihat))
        if hi <= lo:
            return None, None
        a = np.asarray(i_true[lo:hi], dtype=np.complex128)
        b = np.asarray(ihat[lo:hi], dtype=np.complex128)
        den = float(np.vdot(a, a).real)
        if den <= 0:
            return None, None
        num = float(np.vdot(b - a, b - a).real)
        nmse = num / den
        return nmse, (-10.0 * math.log10(nmse) if num > 0 else 60.0)

    nmse_a, sup_a = eval_metrics(win["i"], ihat_a)
    nmse_b, sup_b = eval_metrics(np.ascontiguousarray(i_b,
                                                      dtype=np.complex64),
                                 ihat_b)
    eval_changed = bool(
        nmse_a is not None and nmse_b is not None and
        (abs(nmse_a - nmse_b) > 1e-9 or abs((sup_a or 0) - (sup_b or 0)) > 1e-9))
    report = {
        "mode": "r06_data_dependency",
        "seed": seed, "eval_region": [int(eval_lo), int(eval_hi)],
        "fit_reps": fit, "validation_reps": val, "sync_reps": sync_reps,
        "prefix_equal_before_eval": prefix_equal,
        "frozen_A": fa, "frozen_B": fb,
        "frozen_keys_compared": list(important),
        "frozen_equal_exact": bool(frozen_equal_exact),
        "frozen_equal": frozen_equal,
        "eval_nmse_A": nmse_a, "eval_suppression_db_A": sup_a,
        "eval_nmse_B": nmse_b, "eval_suppression_db_B": sup_b,
        "eval_metric_changed": eval_changed,
        "pass": bool(prefix_equal and frozen_equal and eval_changed),
        "note": ("changing ONLY the independent final-eval region content must "
                 "not move the frozen fit/online-gate results; the eval metric "
                 "must change. The coarse search sees the candidate ROI "
                 "(disclosed), but no eval sample feeds a fit or the gate."),
    }
    write_json(os.path.join(args.output, "r06_data_dependency.json"), report)
    print("r06_data_dependency:", json.dumps(report, sort_keys=True), flush=True)
    return 0 if report["pass"] else 1


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
    fp = fn = fo = fa = 0
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
        # R01/R10: a negative is judged by a false COMMIT *and* an actual output
        # change (the real y!=y_clean support), not by the raw detection alone: a
        # real comm detected in the wide search ROI but outside the sensing
        # processing ROI is legitimately detected, yet must be bypassed.
        n_chg = sum(1 for w in wins
                    if bool((w.get("recon") or {}).get("output_changed")))
        # Actionable overlap the core measured against the sensing processing ROI.
        n_act = sum(1 for w in wins
                    if (w.get("sic_state") or {}).get("sub_overlap") not in
                    (None, -1) and
                    int((w.get("sic_state") or {}).get("sub_overlap")) > 0)
        sic = summ.get("sic") or {}
        bbi = (sic.get("bypass_bit_identical") or {})
        bypass_n += int(bbi.get("n_bypassed") or 0)
        bypass_ok += int(bbi.get("n_bit_identical") or 0)
        is_neg = cid in expected_negative
        if is_neg:
            fp += n_det
            fn += n_com
            fo += n_chg
            # ACTIONABLE detection: a detected interferer whose support actually
            # overlaps the sensing processing ROI.  This is the detection metric
            # the "<= 1%" actionable-negative bound refers to; a raw template hit
            # outside the sensing ROI (no_overlap) is real but not actionable.
            fa += n_act
        row = {
            "case": cid, "expected_negative": is_neg,
            "windows": n, "returncode": rc,
            "detected": n_det, "committed": n_com, "output_changed": n_chg,
            "actionable_overlap": n_act,
            "false_detect": n_det if is_neg else None,
            "false_commit": n_com if is_neg else None,
            "output_change": n_chg if is_neg else None,
            "detect_rate": (n_det / n) if n else None,
            "commit_rate": (n_com / n) if n else None,
            "bypassed": bbi.get("n_bypassed"),
            "bypass_bit_identical": bbi.get("n_bit_identical"),
            "bypass_all_identical": bbi.get("all_identical"),
            "isr_db": (summ.get("config", {}) or {}).get("isr_db"),
            "sensing_roi": ((summ.get("config", {}) or {}).get("sensing_roi")),
            "returncode_ok": rc == 0,
        }
        rows.append(row)
        print("  case %-26s n=%4d det=%4d commit=%4d chg=%4d neg=%s "
              "bypass=%s/%s rc=%d"
              % (cid, n, n_det, n_com, n_chg, is_neg,
                 bbi.get("n_bit_identical"), bbi.get("n_bypassed"), rc),
              flush=True)

    neg_n = sum(r["windows"] for r in rows if r["expected_negative"])
    overall = {
        "negative_windows": neg_n,
        "false_detect": fp,
        "false_actionable_detect": fa,
        "false_commit": fn,
        "false_output_change": fo,
        "false_detect_rate": (fp / neg_n) if neg_n else None,
        "false_actionable_detect_rate": (fa / neg_n) if neg_n else None,
        "false_commit_rate": (fn / neg_n) if neg_n else None,
        "false_output_change_rate": (fo / neg_n) if neg_n else None,
        "false_detect_ci95": wilson_ci(fp, neg_n),
        "false_actionable_detect_ci95": wilson_ci(fa, neg_n),
        "false_commit_ci95": wilson_ci(fn, neg_n),
        "false_output_change_ci95": wilson_ci(fo, neg_n),
        "gate_false_commit_max": 0.01,
        "gate_windows_min": 1000,
        "gate_pass": bool(neg_n >= 1000 and
                          (neg_n > 0 and fn / neg_n <= 0.01)),
        "gate_pass_false_commit_or_change": bool(
            neg_n >= 1000 and neg_n > 0 and fo / neg_n <= 0.01),
        "bypass_windows": bypass_n,
        "bypass_bit_identical": bypass_ok,
        "bypass_all_identical": bool(bypass_n == 0 or bypass_n == bypass_ok),
        "expected_negative_cases": sorted(expected_negative),
        "ci_method": "Wilson score interval, z=1.96 (95%)",
        "note": ("false-detect/commit/output-change are counted only on the "
                 "no-actionable-interferer cases (no_interference, no_overlap, "
                 "wrong_code11).  false-commit is the frozen S0 gate (<=1%); a "
                 "raw DETECT of a real comm in the wide search ROI that lands "
                 "outside the sensing processing ROI is not a commit: it is "
                 "bypassed (output bit-identical), so false_output_change stays "
                 "0.  Never zero-risk claimed."),
    }
    write_json(os.path.join(args.output, "negative_table.json"),
               {"cases": rows, "overall": overall})
    with open(os.path.join(args.output, "negative_table.csv"), "w",
              encoding="utf-8") as f:
        cols = ["case", "expected_negative", "windows", "detected",
                "committed", "output_changed", "actionable_overlap",
                "false_detect", "false_commit", "output_change", "detect_rate",
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
    ap.add_argument("--sic-cover-preamble", action="store_true",
                    help="EXPLICITLY extend the observation window "
                         "(fit_reps+validation_reps) to cover the whole nominal "
                         "comm preamble (sync_reps). Recorded as "
                         "fit_reps_source=explicit_cover_preamble and timed; "
                         "never applied silently. With full coverage there is no "
                         "repetition left for an independent final-eval region, "
                         "which is then reported as unavailable (R06).")
    ap.add_argument("--sic-no-auto-cover", action="store_true",
                    help="deprecated no-op: the silent auto-bump has been "
                         "removed (R02). The declared fit/validation reps are "
                         "used verbatim; coverage shortfall is reported, never "
                         "hidden.")
    ap.add_argument("--emit-r06-data-dep", action="store_true",
                    help="run the R06 data-dependency experiment: perturb only "
                         "the independent final-eval region content and assert "
                         "the frozen fit/online-gate results do not move.")
    # §8.2 main-matrix axes (single-cell overrides) and the sweep driver.
    ap.add_argument("--sweep", action="store_true",
                    help="iterate the main-preset axes (per-cell runs)")
    ap.add_argument("--emit-r01-regression", action="store_true",
                    help="write the two R01 regression inputs (tiny applied "
                         "mask / uncommitted) as verifier-shaped runs under "
                         "--output")
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
    # R08: CG400 native front-end (491.52 MS/s RX / SC16 / PDU 65-32).
    ap.add_argument("--native-taps", default="quality_minorder",
                    help="65/32 taps profile/path (testdata/resampler_65_32) or "
                         "an absolute float32 taps file")
    ap.add_argument("--native-resampler-workers", type=int, default=1,
                    help="Persistent 65/32 FIR worker count (default 1)")
    ap.add_argument("--native-headroom", type=float, default=1.0,
                    help="Common SC16 scale headroom (<1 leaves clip margin)")
    ap.add_argument("--native-repeat-check", type=int, default=3,
                    help="Repeat the SAME native PDU N times to quantify the "
                         "known resampler SIMD non-determinism (recorded, not "
                         "fixed)")
    ap.add_argument("--native-scalar-check", action="store_true",
                    help="Also compute the independent scipy upfirdn reference "
                         "and the scalar/error-floor comparison for the first "
                         "window (separate path, not the production chain)")
    # R09: real consumer variants (functions/correctness bound to one case).
    ap.add_argument("--demod-softchip", default="bypass",
                    choices=["bypass", "full", "rake4"],
                    help="Sensing demod CIR soft-chip filter mode")
    ap.add_argument("--demod-rake-top-k", type=int, default=0,
                    help="RAKE top-k (used when --demod-softchip rake4)")
    ap.add_argument("--radar-timing", default="search",
                    choices=["search", "planned"],
                    help="radar CIR timing: full search vs planned coordinate "
                         "(use_predicted_timing)")
    # R09: pacing measurement controls.
    ap.add_argument("--timing-instrumentation", default="on",
                    choices=["on", "off"],
                    help="on: full per-window stage/eval instrumentation; off: "
                         "skip Python-side per-window aggregation to bound its "
                         "cost (paired short comparison)")
    ap.add_argument("--paced-pool", type=int, default=64,
                    help="number of distinct pre-generated windows in the paced "
                         "pool (>=1)")
    ap.add_argument("--paced-target-only", action="store_true",
                    help="R09: in the paced timed region run ONLY the target "
                         "path (SIC->consumer when SIC is on), skipping the "
                         "clean/raw pairing consumers. The effect pairing is "
                         "verified by a separate load; without this the loop "
                         "still runs all three and reports every bucket.")
    args = ap.parse_args(argv)

    if args.windows is not None and args.windows <= 0:
        raise SystemExit("--windows must be > 0")
    if args.windows is not None and (args.rate is not None
                                     or args.duration is not None):
        raise SystemExit("--windows is mutually exclusive with --rate/"
                         "--duration (the paced schedule fixes the count)")
    # R08: the CG400 native front-end is implemented; no grid is refused here.
    # The paced/burst driver (§8.4) is a work-grid measurement and is not
    # combined with the native front-end (kept separate, never silently mixed).
    if args.input_grid == "cg400" and (
            args.rate is not None or args.duration is not None
            or args.burst is not None):
        raise SystemExit("--input-grid cg400 is not combined with paced/burst "
                         "replay; measure the native effect with --windows")

    repo = S.find_repo_root(os.path.dirname(os.path.abspath(__file__)))
    native = (args.input_grid == "cg400")
    instrumentation = (str(args.timing_instrumentation) == "on")

    # ---- driver dispatch (sweep / negative-set / paced / burst) ----
    if args.sweep:
        if args.preset != "main":
            raise SystemExit("--sweep is only defined for --preset main")
        if args.rate is not None or args.duration is not None or \
                args.burst is not None:
            raise SystemExit("--sweep cannot be combined with paced replay")
        return run_sweep(args, repo, argv)
    if args.emit_r01_regression:
        if args.rate is not None or args.duration is not None or \
                args.burst is not None:
            raise SystemExit("--emit-r01-regression cannot be combined with "
                             "paced replay")
        if args.preset != "main":
            raise SystemExit("--emit-r01-regression is defined for "
                             "--preset main")
        return run_r01_regression(args, repo, argv)
    if args.emit_r06_data_dep:
        if args.rate is not None or args.duration is not None or \
                args.burst is not None:
            raise SystemExit("--emit-r06-data-dep cannot be combined with "
                             "paced replay")
        return run_r06_data_dep(args, repo, argv)
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
    cover_preamble = False
    if sic_on:
        if args.sic_fit_reps is not None or args.sic_validation_reps is not None:
            fit_reps = int(args.sic_fit_reps) if args.sic_fit_reps is not None \
                else fit_reps
            validation_reps = (int(args.sic_validation_reps)
                               if args.sic_validation_reps is not None
                               else validation_reps)
            fit_source = "cli"
        # R02: the observation window (fit_reps -> parameter FIT; validation ->
        # online gate) is used VERBATIM.  The core subtracts only the CONFIRMED
        # repetitions (support_hi = detected_start + confirmed*sps + post_tail)
        # and `confirmed <= fit_reps + validation_reps`, so a short window gives
        # a genuinely PARTIAL subtraction mask.  That shortfall is reported
        # (coverage / partial), never silently repaired.  Full coverage is an
        # explicit opt-in that is recorded and timed.
        if args.sic_cover_preamble:
            cover_preamble = True
            if fit_reps + validation_reps != sync_reps:
                base = max(1, min(fit_reps, sync_reps - 8))
                fit_reps = base
                validation_reps = sync_reps - base
            fit_source = "explicit_cover_preamble"
        if fit_reps <= 0 or validation_reps < 8:
            raise SystemExit("fit_reps>0 and validation_reps>=8 required for "
                             "SIC (got %r/%r)" % (fit_reps, validation_reps))
        if fit_reps + validation_reps > sync_reps:
            raise SystemExit(
                "UNSATISFIABLE SIC config: fit_reps+validation_reps (%d) > "
                "sync_reps (%d); refused (never silently rewritten)"
                % (fit_reps + validation_reps, sync_reps))

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
    search_roi = (int(roi_pair[0]), int(roi_pair[1]))
    if sic_on and not (0 <= search_roi[0] < search_roi[1] <= generator.n):
        raise SystemExit("resolved SIC search roi %r is outside the window "
                         "[0,%d)" % (search_roi, generator.n))

    # R02: the SEARCH ROI (the core's `roi_lo/roi_hi`) stays deliberately WIDE
    # so the natural comm preamble tail `support_hi = detected_start +
    # confirmed*sps + post_tail` (confirmed <= fit+validation) is never clipped
    # by the candidate-start range.  It is extended past the candidate range by
    # the maximum possible support tail.
    rtot_reps = int(fit_reps) + int(validation_reps) if sic_on else 0
    subtraction_roi = (
        int(search_roi[0]),
        int(min(generator.n,
                search_roi[1] + rtot_reps * S.SPS + int(args.sic_post_tail))))
    core_roi = (int(search_roi[0]), int(subtraction_roi[1]))
    if sic_on and not (0 <= core_roi[0] < core_roi[1] <= generator.n):
        raise SystemExit("resolved SIC core roi %r is outside the window "
                         "[0,%d)" % (core_roi, generator.n))

    # R10: the SENSING processing ROI = the sensing preamble / CIR window
    # [pre, pre+sync_reps*sps).  It is a SEPARATE region from the wide search
    # ROI above: it is the subtraction/commit region, and the core commits only
    # when the detected comm support actually OVERLAPS it.  A real comm detected
    # entirely outside the sensing processing window is real but not actionable
    # and is bypassed (out == y) -- S0 §8.3 requires that a no-overlap case is
    # never counted as a cancellation success.  Passed per-window in the SIC PDU
    # meta (`sensing_roi_lo`/`sensing_roi_hi`).
    sensing_roi = (int(roi_base[0]), int(roi_base[1]))
    if sic_on and not (0 <= sensing_roi[0] < sensing_roi[1] <= generator.n):
        raise SystemExit("resolved sensing processing roi %r is outside the "
                         "window [0,%d)" % (sensing_roi, generator.n))


    # R04: frozen raw-CIR target-protection ROI (calibration set only).  It is
    # never re-searched on SIC results; the run records the config path + hash.
    sensing_profile_id = "code%d-%dSYNC-%s" % (sensing_code, sync_reps,
                                               SFD_MODE)
    target_cfg, target_cfg_path = load_target_roi(repo, sensing_profile_id)
    target_lo = target_hi = None
    target_ref_amp = None
    target_detect_ratio = 3.0
    target_false_peak_ratio = 0.5
    target_main_lobe_guard = 3
    if target_cfg is not None:
        target_lo = int(target_cfg["target_lo"])
        target_hi = int(target_cfg["target_hi"])
        target_ref_amp = target_cfg.get("target_ref_amp")
        target_detect_ratio = float(target_cfg.get("detect_ratio", 3.0))
        target_false_peak_ratio = float(target_cfg.get("false_peak_ratio", 0.5))
        target_main_lobe_guard = int(target_cfg.get("main_lobe_guard", 3))
    if sic_on and args.consumer in ("cir", "both") and target_lo is None:
        raise SystemExit(
            "R04 target protection needs a frozen ROI for sensing profile %r "
            "in testdata/sic_preamble/cases/target_roi.json (missing or no "
            "matching profile); refusing to re-search a target on SIC results"
            % sensing_profile_id)

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
        # R02 named quantities (nullable when SIC is off).
        "fit_reps": (None if not sic_on else int(fit_reps)),
        "validation_reps": (None if not sic_on else int(validation_reps)),
        "fit_reps_source": (None if not sic_on else fit_source),
        "cover_preamble": (False if not sic_on else bool(cover_preamble)),
        "fit_len": (None if not sic_on else int(fit_reps) * S.SPS),
        "estimation_len": (None if not sic_on else int(fit_reps) * S.SPS),
        "online_validation_len": (None if not sic_on
                                  else int(validation_reps) * S.SPS),
        "independent_eval_len": (None if not sic_on else
                                 max(0, sync_reps - int(fit_reps) -
                                     int(validation_reps)) * S.SPS),
        "search_roi": [int(search_roi[0]), int(search_roi[1])],
        "subtraction_roi": ([int(subtraction_roi[0]),
                             int(subtraction_roi[1])] if sic_on else None),
        "core_roi": ([int(core_roi[0]), int(core_roi[1])] if sic_on else None),
        "sensing_roi": ([int(sensing_roi[0]), int(sensing_roi[1])]
                        if sic_on else None),
        "post_tail": int(args.sic_post_tail),
        "coarse_stride": 14,
        "detect_threshold": float(args.sic_detect_threshold),
        "validate_threshold": float(args.sic_validate_threshold),
        "comm_template_mode": generator.comm_meta.get("mode"),
        "comm_template_sha256": (generator.comm_template_info().sha256
                                 if generator.comm_template_info() else None),
        "comm_template_length": (generator.comm_template_info().length
                                 if generator.comm_template_info() else None),
        "sic_no_auto_cover_deprecated": bool(args.sic_no_auto_cover),
        "sensing_profile_id": sensing_profile_id,
        "target_roi": ([int(target_lo), int(target_hi)]
                       if target_lo is not None else None),
        "target_ref_amp": (float(target_ref_amp)
                           if target_ref_amp is not None else None),
    }
    config_hash = stdin_hash(resolved_config)
    # R02 effective runtime config (the identity functional/core/GR/native must
    # share) + its hash.  Distinct from the broader run config above.
    effective_config = {
        "case_id": (str(args.case) if args.case else "nominal"),
        "preset": args.preset, "input_grid": args.input_grid,
        "input_length": int(generator.n),
        "sample_rate": float(S.WORK_HZ), "samples_per_symbol": int(S.SPS),
        "sensing_code": int(sensing_code), "comm_code": int(sic_comm_code),
        "sync_reps": int(sync_reps),
        "k_paths": int(sic_k),
        "fit_reps": (None if not sic_on else int(fit_reps)),
        "validation_reps": (None if not sic_on else int(validation_reps)),
        "fit_reps_source": (None if not sic_on else fit_source),
        "cover_preamble": (False if not sic_on else bool(cover_preamble)),
        "search_roi": [int(search_roi[0]), int(search_roi[1])],
        "subtraction_roi": ([int(subtraction_roi[0]),
                             int(subtraction_roi[1])] if sic_on else None),
        "core_roi": ([int(core_roi[0]), int(core_roi[1])] if sic_on else None),
        "sensing_roi": ([int(sensing_roi[0]), int(sensing_roi[1])]
                        if sic_on else None),
        "coarse_stride": 14,
        "post_tail": int(args.sic_post_tail),
        "detect_threshold": float(args.sic_detect_threshold),
        "validate_threshold": float(args.sic_validate_threshold),
        "comm_template_id": ("comm_preamble_code%d_%s" % (sic_comm_code,
                                                          generator.comm_meta.get(
                                                              "mode", "-"))),
        "comm_template_sha256": (S.sha256_array(sic_preamble)
                                 if sic_on and sic_preamble is not None
                                 else None),
        "comm_template_length": (int(sic_preamble.size)
                                 if sic_on and sic_preamble is not None
                                 else None),
    }
    effective_config_hash = stdin_hash(effective_config)


    summary = {
        "run_id": run_id, "preset": args.preset,
        "input_grid": args.input_grid, "sic_mode": args.sic,
        "consumer": args.consumer, "epoch_id": epoch_id,
        "windows_expected": windows, "windows_completed": 0,
        "clean": {}, "raw": {}, "sic": None,
        "effect": {"median_suppression_db": None, "frac_ge_3db": None,
                   "target_amp_err_db_p95": None, "peak_shift_max": None,
                   "median_actual_suppression_db": None,
                   "frac_actual_ge_3db": None,
                   "n_interference_windows": None, "n_na_windows": None,
                   "nmse_median": None},
        "target": None,
        "demod": {}, "negative": {"false_detect": None,
                                  "false_commit": None, "n_windows": windows,
                                  "ci95": None,
                                  "note": "SIC detect/commit not implemented "
                                          "in S0-A"},
        "runtime": {}, "gates": {}, "g0": {}, "not_run": [],
    }
    not_run = summary["not_run"]
    if not native:
        not_run.append("native_cg400_roundtrip (S0-B native front-end; this "
                       "run used --input-grid work)")
    not_run.append("matlab_cross_check (no MATLAB/Octave run)")
    not_run.append("core_cxx_benchmark_10000x3 (benchmark_preamble_sic; S0-D)")
    not_run.append("layer:core (SIC math core benchmark; not run in this "
                   "invocation; R03)")
    not_run.append("layer:gr (standalone GNU Radio PDU chain; this run "
                   "exercises the PDU adapter in-process)")
    if not native:
        not_run.append("layer:native (CG400 native roundtrip; this run used "
                       "--input-grid work)")
    # R10: the speed gates are never exercised by a functional run, so they are
    # null (= untested) and must be named here regardless of sic_mode.
    not_run.append("gates.sic_speed_pass (single-worker C++ core P50/P99 "
                   "speed gate NOT TESTED in this functional run; R10: null = "
                   "untested)")
    not_run.append("gates.demod_speed_pass (downstream full-demod cost gate "
                   "NOT TESTED in this functional run; R10: null = untested)")
    if args.sic == "off":
        not_run.append("sic_estimate_core (sic_mode=off)")
        not_run.append("sic_oracle (diagnostic only)")
        not_run.append("gates.eff_pass (untested; sic_mode=off, no SIC "
                       "estimate to gate)")
    else:
        if args.consumer == "demod":
            not_run.append("R04 raw target protection (consumer=demod; no CIR "
                           "consumer was run)")
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

    # R07: the comm template is a FINITE SYNC code-pulse train (not a truncated
    # packet).  Record the independent-reference agreement, the SFD/PHR/payload
    # independence, and the edge/tail checks; the gate consumes `pass`.
    r07 = generator.comm_template_evidence() if sic_on or comm_present else None
    if r07 is not None:
        g0["comm_template_r07"] = r07
        g0["comm_template_r07_pass"] = bool(r07.get("pass"))
        g0["comm_template_mode"] = generator.comm_meta.get("mode")
        g0["comm_template_length"] = generator.comm_template_info().length
        g0["comm_template_sha256"] = generator.comm_template_info().sha256
        if not r07.get("pass"):
            print("WARNING: R07 comm-template evidence did not pass: %s"
                  % json.dumps(r07, sort_keys=True), flush=True)

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
            "roi": core_roi,
            "search_roi": search_roi,
            "subtraction_roi": subtraction_roi,
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
    sic_sync_norm = None
    if sic_on and sic_sync is not None:
        _sn = float(np.linalg.norm(np.asarray(sic_sync, dtype=np.complex128)))
        sic_sync_norm = (np.asarray(sic_sync, dtype=np.complex128) / _sn
                         if _sn > 0 else np.asarray(sic_sync,
                                                   dtype=np.complex128))
    rt = SimRuntime(uwb, tpl_path=tpl_path, consumers=args.consumer,
                    windows=windows, sic=sic_cfg,
                    keep_iq=max(0, int(args.keep_iq)),
                    demod_softchip=args.demod_softchip,
                    demod_rake_top_k=int(args.demod_rake_top_k),
                    radar_use_predicted=(args.radar_timing == "planned"))
    # R08: CG400 native front-end (SC16 common scale + PDU 65/32).  Only built
    # for the native grid; the work-grid path is byte-identical to before.
    front = None
    if native:
        front = NativeFrontEnd(
            uwb, taps=args.native_taps,
            workers=int(args.native_resampler_workers),
            headroom=float(args.native_headroom))
    # Explicit SOURCE credit tracker (indication §4): never publish more than
    # in_flight_max published-but-not-terminal windows, nor exceed the IQ
    # budget.  Determined before any window is posted.
    credit = (CreditTracker(sic_cfg["in_flight_max"],
                            sic_cfg["iq_budget_bytes"]) if sic_cfg else None)
    rt.start()
    if front is not None:
        front.start()

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
    # R02/R06 region accumulators.
    eff_online_nmse = []
    eff_online_supp = []
    eff_eval_nmse = []
    eff_eval_supp = []
    eff_overlap_nmse = []
    eff_overlap_supp = []
    eff_eval_available = []
    eff_regions = []
    preamble_scan_us_all = []
    eff_target_err_db = []
    eff_peak_shift = []
    # R01 actual-output accumulators (non-truth windows only).
    eff_actual_sup = []
    eff_actual_sup_confirmed = []
    eff_coverage = []
    n_interference_windows = 0
    n_na_windows = 0
    n_uncommitted_changed = 0
    # R04 raw-CIR target accumulators (non-truth windows only).
    eff_target_err_raw_db = []
    eff_target_shift_raw = []
    eff_target_phase_raw = []
    eff_target_weak_detected = []
    eff_target_false_peaks = []
    target_rois_used = set()
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

    # R08: native front-end per-window diagnostics + effect/timing evidence.
    native_records = []
    native_i_metric = None
    # R09: separated paced timing buckets (no double counting).
    t_gen_us = []          # window generation (pre-generated => pool average)
    t_pmt_wrap_us = []     # Python -> PMT vector wrapping
    t_post_us = []         # message entry (blocking _post call)
    t_worker_us = []       # SIC adapter worker / core
    t_consumer_us = []     # consumer service (publish -> terminal)
    t_eval_us = []         # offline evaluation overhead inside the loop
    run_utime0 = run_stime0 = 0.0
    proc_hwm0 = None

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
    paced_service_us = []
    paced_deadline_miss = 0
    paced_pool_us_each = 0.0
    paced_pool = []
    paced_pool_seeds = []
    paced_deferred_records = []
    if paced:
        pool_n = max(1, min(planned, int(args.paced_pool)))
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

    # R09: per-run resource delta baseline (never a cross-run cumulative).
    _ru0 = resource.getrusage(resource.RUSAGE_SELF)
    run_utime0, run_stime0 = float(_ru0.ru_utime), float(_ru0.ru_stime)
    proc_hwm0 = _proc_peak_rss_bytes()
    timed_t0 = time.monotonic()

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
            t_gen_us.append(gen_us)
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
            t_gen_us.append(gen_us)

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

        # ---- R08 CG400 native front-end (work -> SC16 native -> PDU 65/32) --
        # The three-way clean/raw/sic all use the SAME common scale and the SAME
        # resampler; the metric reference is the realized interference
        # i_metric = raw_work - clean_work in the identical domain.
        i_metric = win["i"]
        metric_ov = dict(win.get("overlap") or {"lo": 0, "hi": 0,
                                                "samples": 0})
        native_rec = None
        work_shift = 0
        if native:
            _tn = time.perf_counter()
            (clean_iq, raw_iq, i_metric, _ov_lo, _ov_hi, native_rec) = \
                native_transform_window(
                    front, win, comm_present=comm_present,
                    headroom=float(args.native_headroom), wid=wid,
                    do_scalar=(bool(args.native_scalar_check) and wid == 0),
                    repeat_check=(int(args.native_repeat_check)
                                  if wid == 0 else 0))
            work_shift = int(native_rec["roundtrip_lag_work"])
            metric_ov = {"lo": int(_ov_lo), "hi": int(_ov_hi),
                         "samples": int(max(0, _ov_hi - _ov_lo)),
                         "source": "truth overlap mapped through the native "
                                   "65/32 round trip"}
            native_records.append({"window_id": int(wid), **native_rec})
            print("native window %d/%d scale=%.1f lag=%d ov=[%d,%d) "
                  "clip(raw)=%d/%d floor_nmse=%s kernel=%s"
                  % (wid + 1, windows, native_rec["iq_scale"],
                     native_rec["roundtrip_lag_work"], _ov_lo, _ov_hi,
                     native_rec["raw_clip_i"], native_rec["raw_clip_q"],
                     _fmt(native_rec["error_floor"].get(
                         "effective_noise_nmse")),
                     native_rec.get("resampler_kernel")), flush=True)

        # ---- SIC (estimate / oracle) on the raw window ----------------
        cleaned_iq = None
        sic_block = None
        sic_state_rec = None
        recon_rec = None
        regions = None
        sic_credit_snap = None
        win_planned_ns = None
        win_submit_ns = None
        uses_truth = False
        clean_cir_raw_win = None
        sic_cir_raw_win = None
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
            # R10: sensing processing ROI (subtraction/commit region).  The core
            # only commits when the detected comm support overlaps this window.
            in_meta = put_int(in_meta, "sensing_roi_lo", int(sensing_roi[0]))
            in_meta = put_int(in_meta, "sensing_roi_hi", int(sensing_roi[1]))
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
            # R09 buckets: Python -> PMT wrapping vs message entry are timed
            # separately (never attributed to the C++ worker or generator).
            _tw = time.perf_counter()
            _payload = pmt.init_c32vector(int(raw_iq.size), raw_iq.tolist())
            t_pmt_wrap_us.append((time.perf_counter() - _tw) * 1e6)
            _tp = time.perf_counter()
            rt.sic.to_basic_block()._post(
                pmt.intern("in"), pmt.cons(in_meta, _payload))
            t_post_us.append((time.perf_counter() - _tp) * 1e6)
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
            t_worker_us.append(float(dnum(out_meta, "core_total_us", 0.0) or 0.0))
            _teval0 = time.perf_counter()
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
                # R10 actionable overlap of the detected support with the sensing
                # processing ROI (the subtraction/commit gate).
                "sub_overlap": dnum(out_meta, "sic_sub_overlap", -1),
                "residual_corr": dnum(out_meta, "sic_residual_corr", 0.0),
            }
            if uses_truth:
                oracle_windows += 1
                sic_state_rec["diagnostic_only"] = True
            det_start = sic_state_rec["detected_start"]
            spp = int(S.SPS)
            post_tail = int(args.sic_post_tail)
            rtot = int(fit_reps) + int(validation_reps)
            n_win = int(generator.n)
            # R01 truth overlap = the FULL truth overlap (the intersection of
            # the injected comm support and the sensing processing window); it
            # is the reference interval for the actual-output metric and must
            # never be shortened to a mask chosen to look best.  For the native
            # grid it is mapped through the SAME 65/32 round trip (metric_ov).
            ov = metric_ov
            if int(ov.get("samples", 0)) > 0:
                comm_lo, comm_hi = int(ov["lo"]), int(ov["hi"])
            else:
                comm_lo, comm_hi = int(search_roi[0]), int(search_roi[1])
            anchor = det_start if (isinstance(det_start, int) and
                                   det_start >= 0) else comm_lo
            # ---- R06: three explicit, declared regions (window-local) -------
            # estimation == the parameter FIT window (gain LS / path select /
            # fractional refine).  online-validation == the core's commit
            # holdout.  independent eval == the repetitions the core NEVER uses
            # for CFO / gain / commit; it only feeds the offline metric.
            est_lo = int(anchor)
            est_hi = est_lo + int(fit_reps) * spp
            on_lo = est_hi
            on_hi = on_lo + int(validation_reps) * spp
            ev_lo = on_hi
            ev_hi = int(anchor) + int(sync_reps) * spp
            est_lo = max(0, min(n_win, est_lo))
            est_hi = max(est_lo, min(n_win, est_hi))
            on_lo = max(est_hi, min(n_win, on_lo))
            on_hi = max(on_lo, min(n_win, on_hi))
            ev_lo = max(on_hi, min(n_win, ev_lo))
            ev_hi = max(ev_lo, min(n_win, ev_hi))
            eval_available = bool(ev_hi > ev_lo)

            # Explicit public-template repetition scan (R02 signal evidence).
            # `--timing-instrumentation off` skips this Python-loop-heavy scan
            # (and the raw target re-search) for the on/off cost comparison.
            _ts = time.perf_counter()
            if instrumentation:
                scan = (sync_rep_scan(raw_iq, sic_sync_norm, int(anchor),
                                      int(sync_reps), spp,
                                      float(args.sic_detect_threshold))
                        if sic_sync_norm is not None else
                        {"consecutive_reps": 0, "reps_scanned": 0,
                         "threshold": float(args.sic_detect_threshold),
                         "norms": [], "max_norm": None, "anchor": int(anchor),
                         "sps": spp})
            else:
                scan = {"consecutive_reps": 0, "reps_scanned": 0,
                        "threshold": float(args.sic_detect_threshold),
                        "norms": [], "max_norm": None, "anchor": int(anchor),
                        "sps": spp, "not_run": "timing_instrumentation_off"}
            scan_us = (time.perf_counter() - _ts) * 1e6
            preamble_scan_us_all.append(scan_us)
            scan["scan_us"] = scan_us
            # Signal-evidenced start/stop = the CONSECUTIVE reps the public
            # template confirms at/after the detected start, capped at the
            # observed window (rtot).  Never extrapolated to the nominal 64.
            signal_reps = int(max(1, min(int(scan["consecutive_reps"]),
                                         max(1, rtot))))
            signal_lo = int(anchor)
            signal_hi = int(anchor) + signal_reps * spp

            # ---- R01 truth overlap (full) -----------------------------------
            ov_ev, ov_zero = region_metric(i_metric, ihat, comm_lo, comm_hi)
            if ov_ev is None:
                ov_ev = (0.0, 60.0, comm_lo, comm_hi)

            # ---- online-validation metric (gate region) ---------------------
            on_ev, _on_zero = region_metric(i_metric, ihat, on_lo, on_hi)
            # ---- independent final-eval metric (never feeds estimation/gate)
            ev_ev, ev_zero = region_metric(i_metric, ihat, ev_lo, ev_hi)
            # "holdout" = the second half of the independent eval region (kept
            # as a named blind sub-window; disjoint from fit/online).
            if eval_available and (ev_hi - ev_lo) >= 2:
                ho_lo = ev_lo + (ev_hi - ev_lo) // 2
                ho_hi = ev_hi
            else:
                ho_lo = ho_hi = 0
            ho, _ho_zero = region_metric(i_metric, ihat, ho_lo, ho_hi)

            # ---- R01 actual-output cancellation (delta_actual = y - y_clean)
            act = actual_cancellation(raw_iq, cleaned_iq, i_metric,
                                      comm_lo, comm_hi)
            # Full applied subtraction support (window-wide) for the R02
            # SUBTRACTION ROI; the R01 `sub_lo/sub_hi` stays clipped to the
            # truth overlap so `coverage` and the mask check are well-defined.
            applied_lo, applied_hi = changed_support(raw_iq, cleaned_iq,
                                                     0, n_win)
            sub_lo = max(int(comm_lo), int(applied_lo))
            sub_hi = min(int(comm_hi), int(applied_hi))
            if sub_hi <= sub_lo:
                sub_lo = sub_hi = 0
            span = max(0, int(comm_hi) - int(comm_lo))
            coverage = (((sub_hi - sub_lo) / float(span)) if span > 0 else 0.0)
            # Coverage against the signal-evidenced preamble support (R02).
            sig_span = max(1, int(signal_hi) - int(signal_lo))
            sig_cov = (max(0, min(int(applied_hi), int(signal_hi))
                           - max(int(applied_lo), int(signal_lo)))
                       / float(sig_span)
                       if applied_hi > applied_lo else 0.0)
            c_lo = max(int(comm_lo), int(sub_lo))
            c_hi = min(int(comm_hi), int(sub_hi))
            act_c = (actual_cancellation(raw_iq, cleaned_iq, i_metric, c_lo, c_hi)
                     if c_hi > c_lo else None)
            regions = {
                "fit_lo": int(est_lo), "fit_hi": int(est_hi),
                "fit_len": int(fit_reps) * spp,
                "estimation_lo": int(est_lo), "estimation_hi": int(est_hi),
                "estimation_len": int(fit_reps) * spp,
                # R02 (b): the sample range actually used for CFO / repetition
                # confirmation / gain / path / fractional estimation.  The core
                # reads the ONLINE rep range as well, so this is wider than the
                # fit window; recorded explicitly rather than assumed equal.
                "cfo_estimation_lo": int(est_lo),
                "cfo_estimation_hi": int(on_hi),
                "cfo_estimation_len": (int(on_hi) - int(est_lo)),
                "estimation_includes_online": True,
                "online_validation_lo": int(on_lo),
                "online_validation_hi": int(on_hi),
                "online_validation_len": int(validation_reps) * spp,
                "eval_lo": (int(ev_lo) if eval_available else None),
                "eval_hi": (int(ev_hi) if eval_available else None),
                "eval_len": ((int(ev_hi) - int(ev_lo)) if eval_available
                             else None),
                "eval_available": bool(eval_available),
                "eval_limitation": (
                    None if eval_available else
                    "no independent repetitions remain (fit_reps+"
                    "validation_reps == sync_reps); the R06 claim is downgraded "
                    "for this config -- no blind final-eval region exists"),
                "signal_start": int(signal_lo), "signal_stop": int(signal_hi),
                "signal_evidenced_reps": int(signal_reps),
                "signal_evidenced_reps_full_scan": int(
                    scan.get("consecutive_reps", 0)),
                "signal_evidence": ("public-template normalized correlation >= "
                                    "detect_threshold at detected_start + "
                                    "k*sps; signal_stop is capped at the "
                                    "observed window (fit+validation reps) so "
                                    "it never exceeds the applied mask"),
                "preamble_scan": scan,
                "search_roi": [int(search_roi[0]), int(search_roi[1])],
                "subtraction_roi": [int(applied_lo), int(applied_hi)],
                "subtraction_roi_overlap_clipped": [int(sub_lo), int(sub_hi)],
                "core_roi": [int(core_roi[0]), int(core_roi[1])],
                # R10: the sensing processing ROI (subtraction/commit region),
                # and the actionable overlap the core actually measured.
                "sensing_roi": [int(sensing_roi[0]), int(sensing_roi[1])],
                "sensing_roi_overlap": int(
                    sic_state_rec.get("sub_overlap", -1)),
                "gate_lo": int(on_lo), "gate_hi": int(on_hi),
                "gate_keys": ["residual_corr", "online_nmse",
                              "online_suppression_db"],
                "support_lo": int(sic_state_rec["support_lo"]),
                "support_hi": int(sic_state_rec["support_hi"]),
                "coverage_overlap": float(coverage),
                "coverage_signal": float(sig_cov),
                "partial": bool(rtot < sync_reps),
                "observation_reps": int(rtot),
                "sync_reps": int(sync_reps),
                "data_dependency": {
                    "estimation_region": "fit window only (gain LS / path / "
                                         "fractional); the core's CFO and "
                                         "repetition confirmation ALSO read the "
                                         "online-validation repetitions, so the "
                                         "online region is not a blind holdout",
                    "online_is_used_for_cfo": True,
                    "eval_feeds_estimation": False,
                    "eval_feeds_gate": False,
                    "coarse_search_sees_candidate_roi": True,
                    "disclosure": ("coarse detection correlates the public "
                                   "template over the search ROI, so the eval "
                                   "region's candidate window is search-visible; "
                                   "eval content is excluded from every "
                                   "parameter fit and from the commit gate."),
                },
            }
            recon_rec = {
                # recon.nmse/suppression_db = the INDEPENDENT final-eval region
                # (R06).  This is the offline metric only.
                "nmse": ev_ev[0] if ev_ev else None,
                "suppression_db": ev_ev[1] if ev_ev else None,
                "eval_lo": (ev_ev[2] if ev_ev else (int(ev_lo)
                                                    if eval_available else None)),
                "eval_hi": (ev_ev[3] if ev_ev else (int(ev_hi)
                                                    if eval_available else None)),
                "eval_available": bool(eval_available),
                "zero_reference_energy": bool(ev_zero or ov_zero),
                # R06: the core's real commit (online) region, reported
                # separately and never conflated with the eval region.
                "online_nmse": on_ev[0] if on_ev else None,
                "online_suppression_db": on_ev[1] if on_ev else None,
                "online_validation_lo": int(on_lo),
                "online_validation_hi": int(on_hi),
                # R01 full-overlap diagnostic (frozen contract interval).
                "overlap_lo": int(comm_lo), "overlap_hi": int(comm_hi),
                "overlap_nmse": ov_ev[0] if ov_ev else None,
                "overlap_suppression_db": ov_ev[1] if ov_ev else None,
                # blind sub-holdout inside the independent eval region.
                "holdout_nmse": ho[0] if ho else None,
                "holdout_suppression_db": ho[1] if ho else None,
                "holdout_lo": (ho[2] if ho else None),
                "holdout_hi": (ho[3] if ho else None),
                "mask_lo": sic_state_rec["support_lo"],
                "mask_hi": sic_state_rec["support_hi"],
                "mask_samples": (max(0, sic_state_rec["support_hi"] -
                                     sic_state_rec["support_lo"])
                                 if sic_state_rec["support_hi"] >
                                 sic_state_rec["support_lo"] else 0),
                # ---- R01 frozen measurement contract ----------------------
                "actual_nmse": (None if act is None else act["actual_nmse"]),
                "actual_suppression_db": (None if act is None
                                          else act["actual_suppression_db"]),
                "sub_lo": int(sub_lo),
                "sub_hi": int(sub_hi),
                "coverage": float(coverage),
                "coverage_signal": float(sig_cov),
                "uses_truth": bool(uses_truth),
                "actual_na": (True if act is None else bool(act["na"])),
                "output_changed": (False if act is None
                                   else bool(act["output_changed"])),
                "confirmed_lo": int(c_lo),
                "confirmed_hi": int(c_hi),
                "actual_nmse_confirmed": (None if act_c is None
                                          else act_c["actual_nmse"]),
                "actual_suppression_db_confirmed":
                    (None if act_c is None else act_c["actual_suppression_db"]),
                "noise_included": False,
                "note": ("nmse/suppression_db are the i-based reconstruction "
                         "over the INDEPENDENT final-eval region (reps beyond "
                         "fit+validation); overlap_* is the R01 full-overlap "
                         "diagnostic; online_* is the core's commit region; "
                         "actual_suppression_db is the REAL-output metric "
                         "(delta_actual = y - y_clean)."),
            }
            if ov_ev is not None and not uses_truth:
                eff_nmse.append(ov_ev[0])
                eff_supp.append(ov_ev[1])
                eff_overlap_nmse.append(ov_ev[0])
                eff_overlap_supp.append(ov_ev[1])
                if on_ev is not None:
                    eff_online_nmse.append(on_ev[0])
                    eff_online_supp.append(on_ev[1])
                eff_eval_available.append(bool(eval_available))
                if ev_ev is not None:
                    eff_eval_nmse.append(ev_ev[0])
                    eff_eval_supp.append(ev_ev[1])
                if ho is not None:
                    eff_holdout_nmse.append(ho[0])
                    eff_holdout_supp.append(ho[1])
                eff_regions.append(regions)
                if act is not None:
                    if act["na"]:
                        n_na_windows += 1
                    else:
                        n_interference_windows += 1
                        eff_actual_sup.append(act["actual_suppression_db"])
                        eff_coverage.append(coverage)
                        if act_c is not None and not act_c["na"]:
                            eff_actual_sup_confirmed.append(
                                act_c["actual_suppression_db"])
                    if not sic_state_rec["committed"] and act["output_changed"]:
                        n_uncommitted_changed += 1

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
            t_eval_us.append((time.perf_counter() - _teval0) * 1e6)

        consumer_log = {"clean": {}, "raw": {}}
        if paced and args.paced_target_only:
            # R09: timed region runs ONLY the target processing path.
            if cleaned_iq is not None:
                consumers_iter = [("sic", cleaned_iq, "sic_target_only")]
            else:
                consumers_iter = [("raw", raw_iq,
                                   "target_only"
                                   + ("_s+i+w" if comm_present else "_s+w"))]
        else:
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
            meta_cir = put_int(meta_cir, "pre_guard_samples",
                               generator.pre + work_shift)
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
                               generator.pre + work_shift)
            meta_dem = put_int(meta_dem, "window_start_sample", 0)
            meta_dem = put_int(meta_dem, "predicted_sensing_start",
                               generator.pre + work_shift)
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
            if rt.want_cir and cir_raw is not None:
                if cname == "clean":
                    clean_cir_norm = cir_norm
                    clean_cir_raw = cir_raw
                    clean_cir_raw_win = np.asarray(cir_raw, dtype=np.complex64)
                    # R04: the target tap is the clean argmax inside the FROZEN
                    # calibration ROI (never re-searched on the SIC result).
                    if target_lo is not None and clean_cir_raw_win is not None:
                        hi_c = min(clean_cir_raw_win.size, int(target_hi))
                        if hi_c > int(target_lo):
                            seg = np.abs(clean_cir_raw_win[int(target_lo):hi_c])
                            if seg.size:
                                target_tap = int(target_lo) + int(
                                    np.argmax(seg))
                elif cname == "sic":
                    sic_cir_raw_win = np.asarray(cir_raw, dtype=np.complex64)

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
            if cname == ("sic" if cleaned_iq is not None else "raw"):
                t_consumer_us.append(comp_us)

        # R09 target-only: declare the clean/raw pairing consumers as skipped
        # (never faked as a measured 0-us service); the timed region ran only
        # the target path. The pairing was verified on a separate load.
        if paced and args.paced_target_only:
            _src = ("sic" if completions.get("sic") else "raw")
            for _c in ("clean", "raw"):
                if _c in consumer_log and not consumer_log[_c]:
                    consumer_log[_c] = {
                        "status": "skipped",
                        "skip_reason": "paced_target_only",
                        "note": "clean/raw pairing is verified on a separate "
                                "load; not part of the paced timed region (R09)",
                    }
                    failing_status[_c]["skipped"] = 1
                published_ns.setdefault((wid, _c), published_ns.get((wid, _src)))
                completions[_c] = completions.get(_src)
            clean_queue_us = 0.0

        # paced replay terminal/deadline accounting (planned -> terminal).
        # R09: the target path terminal is the SIC output consumer when SIC is
        # on (the accepted 200/s gate is the SIC->CIR terminal), else raw/clean.
        if paced and win_planned_ns is not None:
            target_c = ("sic" if (sic_on and completions.get("sic"))
                        else ("raw" if completions.get("raw") else "clean"))
            term_ns = completions.get(target_c)
            if term_ns:
                lat_us = (term_ns - win_planned_ns) / 1000.0
                paced_latency_us.append(lat_us)
                paced_late_ns.append((term_ns - win_planned_ns) / 1000.0)
                if win_submit_ns is not None:
                    paced_service_us.append(
                        (term_ns - win_submit_ns) / 1000.0)
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

        # ---- R04 raw (non-L2-normalized) target protection ----------------
        target_rec = None
        if instrumentation and sic_on and rt.want_cir and cleaned_iq is not None \
                and clean_cir_raw_win is not None \
                and sic_cir_raw_win is not None \
                and target_lo is not None and target_hi is not None:
            tm = raw_target_metrics(
                clean_cir_raw_win, sic_cir_raw_win, target_lo, target_hi,
                detect_ratio=target_detect_ratio,
                false_peak_ratio=target_false_peak_ratio,
                main_lobe_guard=target_main_lobe_guard)
            if tm is not None:
                ref = (float(target_ref_amp) if target_ref_amp else
                       tm["target_amp_clean"])
                target_rec = {
                    "target_lo": int(target_lo), "target_hi": int(target_hi),
                    "target_roi": [int(target_lo), int(target_hi)],
                    "target_tap": int(tm["target_tap"]),
                    "target_ref_amp": float(ref),
                    "cir_raw_target_amp_db": (
                        float(20.0 * math.log10(tm["target_amp_sic"] / ref))
                        if (ref > 0.0 and tm["target_amp_sic"] > 0.0)
                        else None),
                    "cir_raw_target_amp_db_clean": (
                        float(20.0 * math.log10(tm["target_amp_clean"] / ref))
                        if (ref > 0.0 and tm["target_amp_clean"] > 0.0)
                        else None),
                    "cir_raw_target_amp_abs_db":
                        tm["cir_raw_target_amp_abs_db"],
                    "target_amp_err_db": tm["target_amp_err_db"],
                    "target_phase_err_deg": tm["target_phase_err_deg"],
                    "target_peak_shift": int(tm["target_peak_shift"]),
                    "target_clean_peak_tap": int(tm["target_clean_peak_tap"]),
                    "target_sic_peak_tap": int(tm["target_sic_peak_tap"]),
                    "weak_target_detected": bool(tm["weak_target_detected"]),
                    "weak_target_snr_db": tm["weak_target_snr_db"],
                    "false_peak_count": int(tm["false_peak_count"]),
                    "noise_floor": float(tm["noise_floor"]),
                    "uses_truth": bool(uses_truth),
                    "note": ("raw (non-L2) complex CIR at the frozen "
                             "calibration target ROI; the target tap is the "
                             "clean ROI argmax and is never re-searched on the "
                             "SIC result. The L2-normalized target_amp stays "
                             "as a shape-only diagnostic."),
                }
                target_rois_used.add((int(target_lo), int(target_hi)))
                if not (sic_state_rec or {}).get("uses_truth", False):
                    if tm["target_amp_err_db"] is not None:
                        eff_target_err_raw_db.append(tm["target_amp_err_db"])
                    eff_target_shift_raw.append(int(tm["target_peak_shift"]))
                    eff_target_phase_raw.append(tm["target_phase_err_deg"])
                    eff_target_weak_detected.append(
                        bool(tm["weak_target_detected"]))
                    eff_target_false_peaks.append(int(tm["false_peak_count"]))

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
        cir_origin = generator.pre + work_shift
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
            "predicted_sensing_start": int(generator.pre + work_shift),
            "sensing_profile_id": "code%d-%dSYNC-%s" % (sensing_code,
                                                        sync_reps, SFD_MODE),
            "sensing_code": sensing_code,
            "sensing_sync_reps": sync_reps,
            "sensing_sfd_mode": SFD_MODE,
            "template_id": "sync_template_code%d_998p4" % sensing_code,
            "overlap": (metric_ov if native else dict(win["overlap"])),
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
            "roi": (None if not sic_on else [int(search_roi[0]),
                                             int(search_roi[1])]),
            "search_roi": [int(search_roi[0]), int(search_roi[1])],
            "subtraction_roi": (None if not sic_on else
                                [int(subtraction_roi[0]),
                                 int(subtraction_roi[1])]),
            "core_roi": (None if not sic_on else [int(core_roi[0]),
                                                  int(core_roi[1])]),
            "sensing_roi": (None if not sic_on else [int(sensing_roi[0]),
                                                     int(sensing_roi[1])]),
            "cover_preamble": (False if not sic_on else bool(cover_preamble)),
            "demod_softchip_mode": args.demod_softchip,
            "seed": int(seed),
            "clean": consumer_log["clean"],
            "raw": consumer_log["raw"],
            "sic_state": (sic_state_rec if sic_on else
                          {"detected": False, "committed": False,
                           "partial": False, "ambiguous": False,
                           "bypass_reason": "sic_off",
                           "uses_truth": False}),
            "admission": admission,
            "timing": {
                "generator_us": float(gen_us),
                "pmt_wrap_us": (float(t_pmt_wrap_us[-1])
                                if t_pmt_wrap_us else None),
                "post_us": (float(t_post_us[-1]) if t_post_us else None),
                "worker_us": (float(t_worker_us[-1]) if t_worker_us else None),
                "eval_us": (float(t_eval_us[-1]) if t_eval_us else None),
                "consumer_us": (float(t_consumer_us[-1])
                                if t_consumer_us else None),
                "note": ("R09 separated buckets: generator / PMT wrap / message "
                         "entry / SIC worker(core) / offline eval / consumer "
                         "service. Never summed as one number."),
            },
            "recon": (recon_rec if sic_on else None),
            "regions": (regions if sic_on else None),
            "target": (target_rec if sic_on else None),
        }
        if native and native_rec is not None:
            rec["coord"] = {
                "work_to_native": [S.NATIVE_DECIM, S.NATIVE_INTERP],
                "native_to_work": [S.NATIVE_INTERP, S.NATIVE_DECIM],
                "filter_delay": float(
                    S.native_group_delay_input(front.tap_count)),
                "filter_delay_work": float(
                    S.native_group_delay_output(front.tap_count)),
                "filter_delay_native": float(
                    S.native_group_delay_input(front.tap_count)),
                "valid_lo": int(front.gd_in),
                "valid_hi": int(max(0, native_rec["native_samples"]
                                    - front.gd_in)),
                "native_samples": int(native_rec["native_samples"]),
                "work_samples": int(native_rec["work_samples_out"]),
                "roundtrip_lag_work": int(native_rec["roundtrip_lag_work"]),
                "metric_overlap": [int(metric_ov["lo"]), int(metric_ov["hi"])],
                "note": ("native 491.52 <-> work 998.4 mapping; the metric "
                         "overlap is the truth overlap advanced by the measured "
                         "round-trip lag (R08)"),
            }
            rec["native"] = {k: v for k, v in native_rec.items()
                             if k != "resample_stats"}
            rec["native"]["resample_stats"] = native_rec["resample_stats"]
        if sic_on:
            rec["sic"] = consumer_log.get("sic")
            if args.sic == "oracle":
                rec["diagnostic_only"] = True
                rec["oracle"] = True
                rec["sic_state"]["diagnostic_only"] = True
        # R09: the paced timed region must not do disk I/O; buffer the JSONL
        # records in memory and flush them after the timed loop.
        if paced:
            paced_deferred_records.append(json.dumps(rec, sort_keys=True))
        else:
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
        if native:
            # The evaluation interval is truth-defined and mapped through the
            # SAME 65/32 round trip as the metrics (R08); keep the sidecar
            # consistent with windows.overlap.
            t_rec["comm"]["real_overlap_lo"] = int(metric_ov["lo"])
            t_rec["comm"]["real_overlap_hi"] = int(metric_ov["hi"])
            t_rec["comm"]["overlap_samples"] = int(metric_ov["samples"])
            t_rec["comm"]["native_roundtrip_lag_work"] = int(work_shift)
            t_rec["comm"]["overlap_domain"] = (
                "work grid after the 491.52<->998.4 SC16 round trip")
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
                overlap_lo=np.int64(metric_ov["lo"]),
                overlap_hi=np.int64(metric_ov["hi"]))
            if native:
                np.savez_compressed(
                    os.path.join(fix_dir, "window_%04d_native.npz" % wid),
                    clean_work=np.ascontiguousarray(clean_iq,
                                                    dtype=np.complex64),
                    raw_work=np.ascontiguousarray(raw_iq, dtype=np.complex64),
                    i_metric=np.ascontiguousarray(i_metric,
                                                  dtype=np.complex64),
                    roundtrip_lag_work=np.int64(work_shift),
                    iq_scale=np.float64(native_rec["iq_scale"]))
            if sic_on and cleaned_iq is not None:
                recon_dir = os.path.join(args.output, "recon")
                os.makedirs(recon_dir, exist_ok=True)
                recon_arrays = dict(
                    # Native: the reference is the REALIZED interference under
                    # the identical common-scale/filtering convention (R08), so
                    # the verifier recomputes on exactly the metric reference.
                    i=(i_metric if native else win["i"]).astype(np.complex64),
                    i_hat=np.ascontiguousarray(ihat, dtype=np.complex64),
                    # R01: the raw SIC input `y` and the actual SIC output
                    # `y_clean`, so the verifier can recompute delta_actual and
                    # never synthesize the output from a declared mask.
                    y=np.ascontiguousarray(raw_iq, dtype=np.complex64),
                    y_clean=np.ascontiguousarray(cleaned_iq,
                                                 dtype=np.complex64))
                # R04: raw (non-normalized) complex CIRs of the same window.
                if clean_cir_raw_win is not None:
                    recon_arrays["cir_raw_clean"] = np.ascontiguousarray(
                        clean_cir_raw_win, dtype=np.complex64)
                if sic_cir_raw_win is not None:
                    recon_arrays["cir_raw_sic"] = np.ascontiguousarray(
                        sic_cir_raw_win, dtype=np.complex64)
                np.savez_compressed(
                    os.path.join(recon_dir, "window_%d.npz" % wid),
                    **recon_arrays)
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
    # R09: flush the buffered paced records after the timed region ended.
    if paced and paced_deferred_records:
        with open(os.path.join(args.output, "windows.jsonl"), "w",
                  encoding="utf-8") as f:
            for ln in paced_deferred_records:
                f.write(ln + "\n")

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
    if front is not None:
        front.stop()

    summary["windows_completed"] = completed_windows
    paced_stats = None
    if paced:
        elapsed_s = max(1e-9, time.monotonic() - paced_wall0)
        lat = sorted(paced_latency_us)
        sub_late = sorted(ns / 1000.0 for ns in paced_submit_late_ns)
        missed_submission = sum(1 for v in sub_late if v > 0.0)

        def _pct(vals, q):
            if not vals:
                return None
            return float(np.percentile(np.asarray(vals, dtype=float), q))

        def _summ(vals):
            v = [float(x) for x in vals if x is not None]
            if not v:
                return {"n": 0}
            a = np.asarray(v, dtype=float)
            return {"n": int(a.size), "mean": float(a.mean()),
                    "p50": float(np.percentile(a, 50)),
                    "p95": float(np.percentile(a, 95)),
                    "p99": float(np.percentile(a, 99)),
                    "max": float(a.max())}
        # R09: offered / admitted / missed-deadline / completed never collapsed
        # into one "admitted rate".  A missed submission is a FAILURE.
        paced_stats = {
            "rate_hz_target": float(pace_rate),
            "planned_windows": int(planned),
            "offered": int(credit.offered) if credit else int(completed_windows),
            "admitted": int(credit.admitted) if credit else int(completed_windows),
            "missed_admission": (int(credit.missed_admission) if credit
                                 else None),
            "miss_reasons": (dict(credit.by_reason) if credit else None),
            "attempted_submit": int(len(paced_submit_late_ns)),
            "missed_submission": int(missed_submission),
            "not_submitted": int(paced_not_submitted),
            "completed": int(completed_windows),
            "failed": 0,
            "deadline_us": float(paced_deadline_us),
            "deadline_misses": int(paced_deadline_miss),
            "achieved_window_rate": float(completed_windows / elapsed_s),
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
            "service_latency_us": {
                "description": ("submit -> CIR terminal (the per-window "
                                "processing latency, excluding the accumulated "
                                "planned-submit deficit)"),
                "n": len(paced_service_us),
                "p50": _pct(paced_service_us, 50),
                "p95": _pct(paced_service_us, 95),
                "p99": _pct(paced_service_us, 99),
                "max": (max(paced_service_us) if paced_service_us else None),
            },
            "implied_serial_capacity_windows_per_s": (
                (1e6 / _pct(paced_service_us, 50))
                if paced_service_us and _pct(paced_service_us, 50) else None),
            "in_flight_max": int(credit.in_flight_max) if credit else None,
            "budget_bytes": int(credit.iq_budget_bytes) if credit else None,
            "window_bytes": int(generator.n * 8),
            "timing_buckets_us": {
                "generator": _summ(t_gen_us),
                "pmt_wrap": _summ(t_pmt_wrap_us),
                "message_entry": _summ(t_post_us),
                "sic_worker_core": _summ(t_worker_us),
                "offline_eval": _summ(t_eval_us),
                "consumer_service": _summ(t_consumer_us),
                "note": ("separate buckets; generator is the pre-generated pool "
                         "cost, not the timed loop. No single bucket is claimed "
                         "to explain every deadline miss."),
            },
            "rss_cpu_per_run": {
                "cpu_seconds": float(
                    (resource.getrusage(resource.RUSAGE_SELF).ru_utime
                     + resource.getrusage(resource.RUSAGE_SELF).ru_stime)
                    - run_utime0 - run_stime0),
                "peak_rss_bytes": proc_hwm0,
                "note": "per-run delta from a fresh process, not cross-run",
            },
            "note": ("a planned publish that is not submitted on time counts as "
                     "missed_submission; the admitted rate is never substituted "
                     "for the planned load"),
        }
        summary["paced"] = paced_stats
        if paced_not_submitted > 0:
            not_run.append("windows: %d of %d planned paced windows were not "
                           "submitted before the wall-clock cap (reported, not "
                           "silently truncated)"
                           % (paced_not_submitted, planned))
        if missed_submission > 0:
            not_run.append("paced deadline: %d planned submissions were late "
                           "(missed_submission; a failure, not masked)"
                           % missed_submission)

    # ---------------- G0 clean-consumer CIR comparison ----------------
    if (not native) and clean_cir_raw is not None and clean_cir_norm is not None:
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
    if native:
        # The canonical work-grid clean golden is not the native (band-limited,
        # SC16-quantized) clean, so this G0 check is work-grid only.  The native
        # clean CIR is still produced and compared via the R04 raw target block.
        g0["cir_clean_consumer_skipped"] = (
            "native front-end: the clean consumer CIR is the 491.52->998.4 "
            "round trip of SC16 samples, not the canonical 998.4 work-grid RX; "
            "the canonical comparisons are cir_canonical_pass (work-grid "
            "fixture) and the per-window native target block")
        g0["cir_clean_pass"] = None
    elif not (rt.want_cir and clean_cir_raw is not None):
        g0["cir_clean_consumer_skipped"] = (
            "the clean pairing consumer did not run (e.g. R09 target-only paced "
            "region); the canonical clean-golden check needs the clean CIR")
        g0["cir_clean_pass"] = None
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
        summary["sic"]["cover_preamble"] = bool(cover_preamble)
        summary["sic"]["search_roi"] = [int(search_roi[0]),
                                        int(search_roi[1])]
        summary["sic"]["subtraction_roi"] = [int(subtraction_roi[0]),
                                             int(subtraction_roi[1])]
        summary["sic"]["core_roi"] = [int(core_roi[0]), int(core_roi[1])]
        summary["sic"]["sensing_roi"] = [int(sensing_roi[0]),
                                         int(sensing_roi[1])]
        summary["sic"]["roi"] = [int(search_roi[0]), int(search_roi[1])]
        summary["sic"]["named_quantities"] = {
            "fit_reps": int(fit_reps),
            "validation_reps": int(validation_reps),
            "fit_len": int(fit_reps) * S.SPS,
            "estimation_len": int(fit_reps) * S.SPS,
            "online_validation_len": int(validation_reps) * S.SPS,
            "independent_eval_len": max(0, int(sync_reps) - int(fit_reps) -
                                        int(validation_reps)) * S.SPS,
            "observation_reps": int(fit_reps) + int(validation_reps),
            "sync_reps": int(sync_reps),
            "coverage_nominal": ((int(fit_reps) + int(validation_reps))
                                 / float(sync_reps)),
            "search_roi": [int(search_roi[0]), int(search_roi[1])],
            "subtraction_roi": [int(subtraction_roi[0]),
                                int(subtraction_roi[1])],
            "core_roi": [int(core_roi[0]), int(core_roi[1])],
            "sensing_roi": [int(sensing_roi[0]), int(sensing_roi[1])],
            "fractional_estimate_source": "core (fit window)",
            "preamble_scan_us_median": px_median(preamble_scan_us_all),
            "effective_config_hash": effective_config_hash,
        }
        summary["sic"]["regions_semantics"] = {
            "estimation": "[detected_start, +fit_reps*sps) - gain LS / path "
                          "select / fractional refine",
            "online_validation": "[detected_start+fit_reps*sps, "
                                 "+validation_reps*sps) - the core commit gate",
            "independent_eval": "[detected_start+(fit+validation)*sps, "
                                "+sync_reps*sps) - offline metric only",
            "shares_data_with_estimation": (
                "the core's CFO and repetition confirmation read the "
                "ONLINE region as well, so the online region is NOT an "
                "estimation-independent holdout; only `independent_eval` is "
                "disjoint from estimation and gate (R06 disclosure)"),
            "search_roi": [int(search_roi[0]), int(search_roi[1])],
            "subtraction_roi": [int(subtraction_roi[0]),
                                int(subtraction_roi[1])],
            "core_roi_note": ("the core's roi_lo/roi_hi serve BOTH candidate "
                              "search and the subtraction clip; core_roi is the "
                              "union so the natural preamble tail is not clipped "
                              "by the candidate range (R02)"),
        }
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
    # recon.suppression_db / recon.nmse are now the INDEPENDENT eval region, so
    # the verifier-recomputed median_suppression_db/nmse_median aggregate those.
    sup3 = [1.0 for s in eff_eval_supp if s >= 3.0]
    eff_abs_err = [abs(v) for v in eff_target_err_raw_db]
    eff_norm_abs_err = [abs(v) for v in eff_target_err_db]
    act3 = [1.0 for s in eff_actual_sup if s >= 3.0]
    n_gate = len(eff_nmse)
    raw_shift_abs = [abs(v) for v in eff_target_shift_raw]
    summary["effect"] = {
        # --- R01 actual-output cancellation (PRIMARY gate) ---------------
        "median_actual_suppression_db": px_median(eff_actual_sup),
        "frac_actual_ge_3db": (len(act3) / len(eff_actual_sup))
        if eff_actual_sup else None,
        "median_actual_suppression_db_confirmed":
            px_median(eff_actual_sup_confirmed),
        "coverage_median": px_median(eff_coverage),
        "coverage_min": (min(eff_coverage) if eff_coverage else None),
        "n_interference_windows": int(n_interference_windows),
        "n_na_windows": int(n_na_windows),
        "n_uncommitted_output_changed": int(n_uncommitted_changed),
        # --- R06 independent final-eval reconstruction (diagnostic) -------
        "median_suppression_db": px_median(eff_eval_supp),
        "frac_ge_3db": (len(sup3) / len(eff_eval_supp))
        if eff_eval_supp else None,
        "nmse_median": px_median(eff_eval_nmse),
        "n_eval_windows": len(eff_eval_nmse),
        "eval_available_frac": (sum(1.0 for v in eff_eval_available if v)
                                / len(eff_eval_available))
        if eff_eval_available else None,
        # --- R06 online-validation (gate) region --------------------------
        "online_nmse_median": px_median(eff_online_nmse),
        "online_suppression_db_median": px_median(eff_online_supp),
        # --- R01 full truth-overlap reconstruction (diagnostic) -----------
        "overlap_nmse_median": px_median(eff_overlap_nmse),
        "overlap_suppression_db_median": px_median(eff_overlap_supp),
        "holdout_nmse_median": px_median(eff_holdout_nmse),
        "holdout_suppression_db_median": px_median(eff_holdout_supp),

        # --- R04 raw-CIR target protection (gate) ------------------------
        "target_amp_err_db_p95": px_p95(eff_target_err_raw_db),
        "target_amp_abs_err_db_p95": px_p95(eff_abs_err),
        "target_phase_err_deg_p95": px_p95(eff_target_phase_raw),
        "target_peak_shift_max": (max(raw_shift_abs) if raw_shift_abs
                                  else None),
        "target_weak_detected_frac": (
            sum(1.0 for v in eff_target_weak_detected if v)
            / len(eff_target_weak_detected)) if eff_target_weak_detected
        else None,
        "target_false_peak_max": (max(eff_target_false_peaks)
                                  if eff_target_false_peaks else None),
        # --- normalized-CIR (L2) shape-only diagnostic -------------------
        "normalized_target_amp_err_db_p95": px_p95(eff_target_err_db),
        "normalized_target_amp_abs_err_db_p95": px_p95(eff_norm_abs_err),
        "peak_shift_max": (max(eff_peak_shift) if eff_peak_shift else None),
        "n_windows": n_gate,
        "oracle_windows": int(oracle_windows),
        "oracle_excluded": True,
        "note": ("primary effect gate: median ACTUAL-output suppression >=10 dB "
                 "AND >=95% windows >=3 dB (delta_actual = y - y_clean over the "
                 "full truth overlap), plus raw-CIR target |amp err| <=1 dB "
                 "(p95) and |peak shift| <=1 work sample. median_suppression_db/"
                 "nmse_median are the R06 INDEPENDENT final-eval reconstruction "
                 "diagnostic (not the gate); online_* is the core commit region; "
                 "overlap_* is the R01 full-overlap diagnostic; normalized_* are "
                 "the L2 shape-only diagnostic. Aggregate excludes uses_truth "
                 "windows; n_na_windows are ||i||^2==0."),
    }
    eff_ok = bool(
        sic_on and args.sic == "estimate" and args.preset == "main" and
        oracle_windows == 0 and
        summary["effect"]["median_actual_suppression_db"] is not None and
        summary["effect"]["median_actual_suppression_db"] >= 10.0 and
        summary["effect"]["frac_actual_ge_3db"] is not None and
        summary["effect"]["frac_actual_ge_3db"] >= 0.95 and
        summary["effect"]["target_amp_abs_err_db_p95"] is not None and
        summary["effect"]["target_amp_abs_err_db_p95"] <= 1.0 and
        summary["effect"]["target_peak_shift_max"] is not None and
        summary["effect"]["target_peak_shift_max"] <= 1)

    # R04 raw-CIR target-protection aggregate (frozen calibration ROI).
    _tgt_n = len(eff_target_err_raw_db)
    _tgt_shift_abs = [abs(v) for v in eff_target_shift_raw]
    summary["target"] = {
        "profile": sensing_profile_id,
        "target_lo": (int(target_lo) if target_lo is not None else None),
        "target_hi": (int(target_hi) if target_hi is not None else None),
        "target_ref_amp": (float(target_ref_amp)
                           if target_ref_amp is not None else None),
        "config_path": target_cfg_path,
        "n_windows": int(_tgt_n),
        "target_amp_err_db_median": px_median(eff_target_err_raw_db),
        "target_amp_abs_err_db_p95": px_p95(eff_abs_err),
        "target_phase_err_deg_p95": px_p95(eff_target_phase_raw),
        "target_peak_shift_max": (max(_tgt_shift_abs) if _tgt_shift_abs
                                  else None),
        "frac_amp_le_1db": ((sum(1.0 for a in eff_abs_err if a <= 1.0)
                             / float(_tgt_n)) if _tgt_n else None),
        "frac_shift_le_1": ((sum(1.0 for s in _tgt_shift_abs if s <= 1)
                             / float(len(_tgt_shift_abs)))
                            if _tgt_shift_abs else None),
        "weak_target_detected_frac": (
            (sum(1.0 for v in eff_target_weak_detected if v)
             / float(len(eff_target_weak_detected)))
            if eff_target_weak_detected else None),
        "false_peak_max": (max(eff_target_false_peaks)
                           if eff_target_false_peaks else None),
        "roi_fixed_across_windows": bool(len(target_rois_used) <= 1),
        "note": ("R04 raw (non-L2-normalized) complex CIR target protection at "
                 "the frozen calibration ROI. The L2-normalized target_amp is a "
                 "shape-only diagnostic and does not satisfy this gate."),
    }

    # ---------------- R08 native effect + timing summary ----------------
    native_block = None
    native_pass_value = False
    if native:
        clip_total = sum(int(r.get("raw_clip_i", 0)) + int(r.get("raw_clip_q", 0))
                         + int(r.get("clean_clip_i", 0))
                         + int(r.get("clean_clip_q", 0))
                         for r in native_records)
        scales = [float(r["iq_scale"]) for r in native_records]
        lags = [int(r["roundtrip_lag_work"]) for r in native_records]
        floors = [r.get("error_floor", {}).get("effective_noise_nmse")
                  for r in native_records]
        nl = [r.get("nonlinearity", {}).get("max_abs_diff")
              for r in native_records]
        res_us = [r.get("resample_stats", {}).get("raw", {}).get("resample_us")
                  for r in native_records]
        hdl_us = [r.get("resample_stats", {}).get("raw", {}).get("handler_us")
                  for r in native_records]
        scalar = next((r.get("scalar_reference") for r in native_records
                       if r.get("scalar_reference")), None)
        repeat = next((r.get("repeat_run_difference") for r in native_records
                       if r.get("repeat_run_difference")), None)
        res_stats = front.stats() if front is not None else {}
        native_block = {
            "ran": bool(native_records),
            "input_grid": "cg400",
            "native_rate": S.CG400_HZ,
            "work_rate": S.WORK_HZ,
            "windows": len(native_records),
            "resampler": res_stats,
            "common_sc16_scale": True,
            "per_component_normalization": False,
            "iq_scale": {"min": (min(scales) if scales else None),
                         "max": (max(scales) if scales else None),
                         "n": len(scales)},
            "clip_counts_total": int(clip_total),
            "clip_free": bool(clip_total == 0),
            "roundtrip_lag_work": {
                "min": (min(lags) if lags else None),
                "max": (max(lags) if lags else None),
                "expected": float(S.native_group_delay_output(
                    front.tap_count)) if front is not None else None},
            "quantization_nonlinearity_max_abs_diff": {
                "n": len(nl), "max": (max(nl) if nl else None)},
            "error_floor_effective_noise_nmse": summarize(floors),
            "resample_us": summarize(res_us),
            "resample_handler_us": summarize(hdl_us),
            "scalar_reference": scalar,
            "repeat_run_difference": repeat,
            "coord": {
                "work_to_native": [S.NATIVE_DECIM, S.NATIVE_INTERP],
                "native_to_work": [S.NATIVE_INTERP, S.NATIVE_DECIM],
                "filter_delay": (float(S.native_group_delay_input(
                    front.tap_count)) if front is not None else None),
                "valid_lo": (int(front.gd_in) if front is not None else None),
                "valid_hi": (int(max(0, native_records[0]["native_samples"]
                                     - front.gd_in))
                             if (front is not None and native_records)
                             else None),
            },
            "effect": {
                "median_actual_suppression_db":
                    summary["effect"].get("median_actual_suppression_db"),
                "frac_actual_ge_3db":
                    summary["effect"].get("frac_actual_ge_3db"),
                "coverage_median": summary["effect"].get("coverage_median"),
                "n_interference_windows":
                    summary["effect"].get("n_interference_windows"),
                "note": ("actual-output suppression measured on the REALIZED "
                         "interference i_metric=raw_work-clean_work under the "
                         "identical common scale/filtering (R08)"),
            },
            "note": ("native capture is ONE quantized mixture; clean/raw/sic use "
                     "the same scale, resampler and valid range. The known "
                     "resampler SIMD non-determinism is recorded (repeat/scalar "
                     "comparison + error floor), not fixed."),
        }
        summary["native"] = native_block
        native_pass_value = bool(
            native_block["ran"] and native_block["clip_free"]
            and int(res_stats.get("pdus_dropped", 0)) == 0
            and bool(eff_ok))
    else:
        native_pass_value = False

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
    if g0.get("comm_template_r07") is not None:
        g0_parts.append(g0.get("comm_template_r07_pass"))
    if rt.want_demod:
        g0_parts.append(g0.get("psdu_canonical_pass"))
    if rt.want_cir and not native and clean_cir_raw is not None:
        g0_parts.append(g0.get("cir_clean_pass"))
    # ---- R10 gate discipline -------------------------------------------------
    # A gate that was exercised is a real boolean AND carries explicit tested
    # evidence; a gate that was NOT exercised in this invocation is null and is
    # named untested in not_run[].  A ``false`` gate means tested-and-failed and
    # must NOT also be named untested (that was the pre-R10 contradiction).
    eff_pass = bool(eff_ok) if sic_on else None
    sic_speed_pass = None            # core P50/P99 speed gate: not run here
    demod_speed_pass = None          # downstream demod-cost gate: later stage
    native_gate = native_pass_value if native else None
    gate_evidence = {}
    if sic_on:
        gate_evidence["eff_pass"] = (
            "recomputed over %s non-truth windows on the actual-output "
            "suppression: median_actual_suppression_db=%s (>=10), "
            "frac_actual_ge_3db=%s (>=0.95), target_amp_abs_err_db_p95=%s "
            "(<=1), target_peak_shift_max=%s (<=1)"
            % (summary["effect"].get("n_windows"),
               summary["effect"].get("median_actual_suppression_db"),
               summary["effect"].get("frac_actual_ge_3db"),
               summary["effect"].get("target_amp_abs_err_db_p95"),
               summary["effect"].get("target_peak_shift_max")))
    if native:
        gate_evidence["native_pass"] = (
            "native front-end ran on %d window(s): clip_free=%s "
            "pdus_dropped=%s eff_ok=%s (see summary.native); a false result is "
            "tested-and-failed, not untested"
            % (len(native_records),
               (native_block or {}).get("clip_free"),
               res_stats.get("pdus_dropped"), bool(eff_ok)))
    summary["gates"] = {
        "eff_pass": eff_pass,
        "sic_speed_pass": sic_speed_pass,
        "demod_speed_pass": demod_speed_pass,
        "native_pass": native_gate,
        "g0_pass": bool(all(g0_parts)),
        "note": ("g0_pass is the S0-A closure gate over the checks applicable "
                 "to the selected --consumer; eff_pass is the S0 primary "
                 "effect gate (main/estimate only); native_pass requires the "
                 "native front-end to run clip-free with no dropped PDU AND the "
                 "same effect gate.  R10: sic_speed_pass/demod_speed_pass/ "
                 "native_pass are null when NOT TESTED in this invocation and "
                 "are then named in not_run[]; a false gate is a "
                 "tested-and-failed result carrying evidence."),
    }
    summary["gate_evidence"] = gate_evidence
    g0["g0_pass"] = summary["gates"]["g0_pass"]
    if native_gate is None:
        not_run.append("gates.native_pass (untested: native front-end not run "
                       "in this invocation; use --input-grid cg400)")

    # ---------------- manifests / tables ----------------
    # R10: bind the ACTUAL run to the real commit, a sha256 id of the
    # working-tree diff, the loaded library / benchmark binary file hashes, the
    # template hashes tied to the effective config, and the input/fixture
    # hashes.  Every one of these is re-derived from disk right now.
    gitp = git_worktree_provenance(repo)
    lib_uwb_path = pick_loaded_library(build_dir)
    bench_exe_path = os.path.join(build_dir, "apps", "benchmark_preamble_sic")
    bench_src_path = os.path.join(repo, "gr-uwb", "apps",
                                  "benchmark_preamble_sic.cc")
    provenance_block = {
        "repo_root": repo,
        "build_dir": build_dir,
        "git_head": gitp["head"],
        "git_branch": gitp["branch"],
        "git_dirty": bool(gitp["dirty"]),
        "git_status_sha256": gitp["status_sha256"],
        "dirty_patch_sha256": gitp["status_sha256"],
        "libgnuradio_uwb_path": lib_uwb_path,
        "libgnuradio_uwb_sha256": (file_sha256(lib_uwb_path)
                                   if lib_uwb_path and
                                   os.path.isfile(lib_uwb_path) else None),
        "benchmark_exe_path": (bench_exe_path
                               if os.path.isfile(bench_exe_path) else None),
        "benchmark_exe_sha256": (file_sha256(bench_exe_path)
                                 if os.path.isfile(bench_exe_path) else None),
        "benchmark_source_sha256": (file_sha256(bench_src_path)
                                    if os.path.isfile(bench_src_path)
                                    else None),
        "config_hash": effective_config_hash,
        "input_source": args.input_grid,
        "input_grid": args.input_grid,
        "production_profile": False,
        "stamped_from": "run_time",
    }
    manifest = {
        "run_id": run_id,
        "commit": gitp["head"] or subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True,
            text=True).stdout.strip(),
        "dirty": bool(gitp["dirty"]),
        "provenance": provenance_block,
        "matlab": {
            "status": "未实跑",
            "command": ("matlab -batch \"run('testdata/sic_preamble/matlab/"
                        "verify_sic_preamble.m')\""),
            "log_path": "testdata/sic_preamble/matlab/matlab_verify.log",
            "note": ("No MATLAB/Octave is available in this environment, so "
                     "the independent MATLAB cross-check is NOT RUN (R10); the "
                     "script is kept runnable and this is recorded as "
                     "未实跑, never as parity evidence."),
        },
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
        "effective_config": effective_config,
        "effective_config_hash": effective_config_hash,
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
                "source": ("finite %d-SYNC code-pulse train (spread code -> "
                           "chip/pulse train -> HRP pulse-shaping FIR); "
                           "SYNC-only, no SFD/PHR/payload (R07)"
                           % sync_reps),
                "sha256": (generator.comm_template_info().sha256
                           if generator.comm_template_info() else
                           g0.get("wrong_template_control", {}).get(
                               "code%d" % comm_code, {}).get("comm_sha256")),
                "r07_evidence": (g0.get("comm_template_r07", {}).get("pass")
                                 if g0.get("comm_template_r07") else None),
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
        "target_protection": ({
            "profile": sensing_profile_id,
            "target_lo": int(target_lo),
            "target_hi": int(target_hi),
            "target_ref_amp": (float(target_ref_amp)
                               if target_ref_amp is not None else None),
            "config_path": target_cfg_path,
            "config_hash": (file_sha256(target_cfg_path)
                            if target_cfg_path
                            and os.path.isfile(target_cfg_path) else None),
            "roi_fixed_across_windows": bool(len(target_rois_used) <= 1),
            "detect_ratio": float(target_detect_ratio),
            "false_peak_ratio": float(target_false_peak_ratio),
            "main_lobe_guard": int(target_main_lobe_guard),
            "note": ("R04 frozen calibration-set target ROI in the CIR tap "
                     "domain; never re-searched on SIC results."),
        } if target_lo is not None else None),
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
            "source": ("finite %d-SYNC code-pulse train for the SIC adapter "
                       "(nominal comm code); SYNC-only (R07)" % sync_reps),
            "sha256": S.sha256_array(sic_preamble),
        }
    # R02: declare the effective config identity for every implementation layer.
    # This work-grid run is the functional layer; core/GR/native are separate
    # stages, so they are either declared identically or explicitly not run.
    _same_diffs = [] if not sic_on else [
        "core: benchmark_preamble_sic.cc uses its own synthetic load "
        "(R03 scope) unless run with the same case; not run in this invocation",
        "gr: GNU Radio flowgraph is the same functional chain here; the "
        "standalone PDU adapter path is exercised in-process",
    ]
    _native_entry = None
    if native:
        _native_entry = {
            "case_id": effective_config["case_id"],
            "effective_config_hash": effective_config_hash,
            "template_id": effective_config.get("comm_template_id"),
            "template_sha256": effective_config.get("comm_template_sha256"),
            "input_length": int(generator.n),
            "input_grid": "cg400",
            "native_rate": S.CG400_HZ,
            "native_samples": (int(native_records[0]["native_samples"])
                               if native_records else None),
            "resampler_taps": (int(front.tap_count)
                               if front is not None else None),
            "difference_note": (
                "native front-end: the SAME case/config/template/input length "
                "is pushed through CG400 491.52 MS/s SC16 + PDU 65/32 before "
                "SIC; only the input grid differs and is recorded here"),
        }
        _same_diffs = list(_same_diffs) + [
            "native: same case/config/template/input_length; front-end is the "
            "CG400 491.52 MS/s SC16 + PDU 65/32 round trip (R08)"]
    else:
        _same_diffs = list(_same_diffs) + [
            "native: CG400 native roundtrip not run in this invocation"]
    manifest["configuration"] = {
        "functional": {
            "case_id": effective_config["case_id"],
            "effective_config_hash": effective_config_hash,
            "template_id": effective_config.get("comm_template_id"),
            "template_sha256": effective_config.get("comm_template_sha256"),
            "input_length": int(generator.n),
        },
    }
    manifest["implementation_configs"] = {
        "functional": {
            "case_id": effective_config["case_id"],
            "effective_config_hash": effective_config_hash,
            "template_id": effective_config.get("comm_template_id"),
            "input_length": int(generator.n),
            "differences": _same_diffs,
        },
    }
    if _native_entry is not None:
        manifest["implementation_configs"]["native"] = _native_entry
    manifest["config_differences"] = _same_diffs
    manifest["roi_semantics"] = (
        "effective_config.subtraction_roi/core_roi are the CONFIGURED ranges "
        "passed to the core; each window's regions.subtraction_roi is the "
        "ACTUAL applied mask support in that window (R02).")

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
        print("  target", json.dumps(summary.get("target"), sort_keys=True),
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
