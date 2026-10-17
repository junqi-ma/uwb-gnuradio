#!/usr/bin/env python3
"""Export one archived S2-A window as a self-contained MATLAB-readable fixture.

This is a small, dependency-light (NumPy + stdlib only) bridge between the
frozen S2-A archive and the independent MATLAB cross-check
``testdata/sic_accum/matlab/verify_s2a_cir.m``.  It does NOT recompute any
metric and does NOT touch the SIC core: it copies the already-archived arrays
out as raw CF32 files plus a ``fixture.json`` describing them.

What it writes into ``--out <dir>``
-----------------------------------
* ``clean.cf32`` / ``raw.cf32`` / ``sic.cf32``  complex64, interleaved I/Q,
  little-endian, exactly as ``np.ndarray.tofile`` writes them (1-D, length
  ``sample_count``).
* ``code.cf32``                                the frozen sampled code
  (complex64, length ``sps``), taken from the run's
  ``operator/operator_arrays.npz`` (``sampled_code``).
* ``expected_H_clean.cf32`` / ``expected_H_raw.cf32`` /
  ``expected_H_sic.cf32``                      the archived CIR tensors stored as
  ``complex128 -> complex64``, row-major ``(sync_reps, ntaps)``; row ``r`` is
  ``H[r]`` over the tap grid ``[tap_lo, tap_hi)`` at work-sample offset
  ``origin + r*sps + t + m``.
* ``fixture.json``                             geometry, provenance (source
  archive sha256) and the array shapes.

The source archive is one ``*-<cell>_<wid>.npz`` produced by
``gr-uwb/apps/uwb_sim_sic_accum.py`` (keys ``clean``, ``raw``, ``sic``,
``H_clean``, ``H_raw``, ``H_sic``).  Only the first 8 windows of every cell are
archived, so only those windows can be exported.

Usage
-----
    # first archived window of a run's archive dir (default)
    python3 testdata/sic_accum/export_matlab_fixture.py \
        --run gr-uwb/build-sic-accum/results/s2a/smoke \
        --out /tmp/s2a_fixture

    # a specific window
    python3 testdata/sic_accum/export_matlab_fixture.py \
        --run gr-uwb/build-sic-accum/results/s2a/smoke \
        --window isr10_cfo0_single_00000 \
        --out /tmp/s2a_fixture
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import sys

import numpy as np

#: Arrays copied verbatim from the archive into the MATLAB fixture.
IQ_KEYS = ("clean", "raw", "sic")
H_KEYS = ("H_clean", "H_raw", "H_sic")


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def archive_dir_of(run_dir: str) -> str:
    return os.path.join(os.path.abspath(run_dir), "archive")


def pick_archive(run_dir: str, window: str = None) -> str:
    d = archive_dir_of(run_dir)
    if not os.path.isdir(d):
        raise SystemExit("archive directory not found: %s" % d)
    if window:
        name = window if window.endswith(".npz") else window + ".npz"
        path = os.path.join(d, name)
        if not os.path.exists(path):
            raise SystemExit("requested window not archived: %s" % path)
        return path
    files = sorted(glob.glob(os.path.join(d, "*.npz")))
    if not files:
        raise SystemExit("no archived windows (*.npz) in %s" % d)
    return files[0]


def load_operator(run_dir: str):
    """Read the frozen geometry (run's own copy) + the archived sampled code."""
    op_path = os.path.join(run_dir, "operator.json")
    if not os.path.exists(op_path):
        raise SystemExit("run's operator.json not found: %s" % op_path)
    with open(op_path, "r", encoding="utf-8") as fh:
        op = json.load(fh)
    arrays_path = os.path.join(run_dir, "operator", "operator_arrays.npz")
    if not os.path.exists(arrays_path):
        raise SystemExit("operator arrays not found: %s" % arrays_path)
    arrays = np.load(arrays_path, allow_pickle=False)
    if "sampled_code" not in arrays.files:
        raise SystemExit("operator_arrays.npz has no 'sampled_code'")
    return op, arrays


def cell_and_window(archive_path: str, meta: dict) -> tuple:
    """Prefer the record's own cell/window ids; fall back to the file name."""
    if meta.get("cell_id") is not None and meta.get("window_id") is not None:
        return str(meta["cell_id"]), int(meta["window_id"])
    base = os.path.basename(archive_path)[:-4]        # drop .npz
    cell, _, rest = base.rpartition("_")
    return cell, int(rest)


def to_cf32(path: str, arr: np.ndarray) -> None:
    np.ascontiguousarray(arr, dtype=np.complex64).tofile(path)


def export(run_dir: str, archive_path: str, out_dir: str) -> dict:
    op, arrays = load_operator(run_dir)
    o = op["operator"]
    origin = int(o["origin"])
    sps = int(o["sps"])
    sync_reps = int(o["sync_reps"])
    tap_lo = int(o["tap_lo"])
    tap_hi = int(o["tap_hi"])
    ntaps = int(tap_hi - tap_lo)
    code_energy = float(np.real(np.vdot(
        np.asarray(arrays["sampled_code"], dtype=np.complex128),
        np.asarray(arrays["sampled_code"], dtype=np.complex128))))
    code = np.asarray(arrays["sampled_code"], dtype=np.complex64)
    if code.size != sps:
        raise SystemExit("archived sampled_code length %d != sps %d"
                         % (code.size, sps))

    z = np.load(archive_path, allow_pickle=False)
    missing = [k for k in IQ_KEYS + H_KEYS if k not in z.files]
    if missing:
        raise SystemExit("archive %s is missing arrays: %s"
                         % (archive_path, missing))
    iq = {}
    for k in IQ_KEYS:
        iq[k] = np.asarray(z[k], dtype=np.complex64)
    sample_count = int(iq["clean"].size)
    if not (iq["raw"].size == iq["sic"].size == sample_count):
        raise SystemExit("clean/raw/sic lengths differ in %s" % archive_path)
    if origin < 0 or origin + sync_reps * sps > sample_count:
        raise SystemExit("origin %d + %d*%d exceeds window %d"
                         % (origin, sync_reps, sps, sample_count))

    H = {}
    for k in H_KEYS:
        H[k] = np.asarray(z[k], dtype=np.complex128)
        if H[k].shape != (sync_reps, ntaps):
            raise SystemExit("%s shape %s != (sync_reps, ntaps)=%s"
                             % (k, H[k].shape, (sync_reps, ntaps)))

    meta = {}
    if "meta_json" in z.files:
        try:
            meta = json.loads(bytes(z["meta_json"]).decode("utf-8"))
        except Exception:
            meta = {}
    cell_id, window_id = cell_and_window(archive_path, meta)

    os.makedirs(out_dir, exist_ok=True)
    to_cf32(os.path.join(out_dir, "clean.cf32"), iq["clean"])
    to_cf32(os.path.join(out_dir, "raw.cf32"), iq["raw"])
    to_cf32(os.path.join(out_dir, "sic.cf32"), iq["sic"])
    to_cf32(os.path.join(out_dir, "code.cf32"), code)
    for k in H_KEYS:
        to_cf32(os.path.join(out_dir, "expected_%s.cf32" % k), H[k])

    fixture = {
        "schema_version": 1, "task": "S2-A",
        "origin": origin, "sps": sps, "sync_reps": sync_reps,
        "tap_lo": tap_lo, "tap_hi": tap_hi, "ntaps": ntaps,
        "code_energy": code_energy,
        "sample_count": sample_count,
        "cell_id": cell_id, "window_id": window_id,
        "correlation": "H[r,t] = sum_m y[origin + r*sps + t + m] * "
                       "conj(code[m]) / code_energy  (average convention, "
                       "identity phase)",
        "source_run": os.path.abspath(run_dir),
        "source_archive_file": os.path.basename(archive_path),
        "source_archive_path": os.path.abspath(archive_path),
        "source_archive_sha256": sha256_file(archive_path),
        "array_shapes": {
            "clean": [sample_count], "raw": [sample_count],
            "sic": [sample_count], "code": [int(code.size)],
            "expected_H_clean": [sync_reps, ntaps],
            "expected_H_raw": [sync_reps, ntaps],
            "expected_H_sic": [sync_reps, ntaps],
        },
        "arrays": ["clean", "raw", "sic", "code",
                   "expected_H_clean", "expected_H_raw", "expected_H_sic"],
        "dtype": "complex64 (interleaved I/Q, little-endian, np.tofile order)",
        "H_storage": "row-major (sync_reps, ntaps), complex128 -> complex64",
        "note": "does not recompute any metric; copies the frozen archive",
    }
    with open(os.path.join(out_dir, "fixture.json"), "w",
              encoding="utf-8") as fh:
        json.dump(fixture, fh, indent=2, sort_keys=True)
    return fixture


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--run", help="S2-A run directory (contains archive/)")
    ap.add_argument("--archive", help="explicit archived window .npz; when "
                                      "omitted, --run's first archived window")
    ap.add_argument("--out", required=True, help="fixture output directory")
    ap.add_argument("--window", help="optional <cell>_<wid> selecting a "
                                     "specific archived window of --run")
    args = ap.parse_args(argv)

    if args.archive and args.window:
        ap.error("--archive and --window are mutually exclusive")
    if args.archive:
        archive_path = os.path.abspath(args.archive)
        if not os.path.exists(archive_path):
            raise SystemExit("archive not found: %s" % archive_path)
        run_dir = os.path.dirname(os.path.dirname(archive_path))
    else:
        if not args.run:
            ap.error("either --run or --archive is required")
        run_dir = os.path.abspath(args.run)
        if not os.path.isdir(run_dir):
            raise SystemExit("run directory not found: %s" % run_dir)
        archive_path = pick_archive(run_dir, args.window)
    if not os.path.isdir(run_dir):
        raise SystemExit("run directory not found: %s" % run_dir)

    fixture = export(run_dir, archive_path, os.path.abspath(args.out))
    print(json.dumps({
        "out": os.path.abspath(args.out),
        "source_archive": os.path.relpath(archive_path, run_dir),
        "cell_id": fixture["cell_id"], "window_id": fixture["window_id"],
        "sample_count": fixture["sample_count"],
        "H_shape": fixture["array_shapes"]["expected_H_sic"],
        "code_energy": fixture["code_energy"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
