#!/usr/bin/env python3
"""Re-read UWB radar goldens and check lengths, peak, SFD/SYNC, native period."""

from __future__ import annotations

import json
import os
import sys

import numpy as np
from scipy.signal import correlate, resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))


def read_cf32(path: str) -> np.ndarray:
    return np.fromfile(path, dtype=np.complex64)


def fail(msg: str) -> None:
    raise AssertionError(msg)


def check_coords(meta: dict) -> list[str]:
    lines = []
    sps = int(meta["samples_per_symbol"])
    n_sync = int(meta["sync_repetitions"])
    coords = meta["coordinates_0based"]
    for name, extra in (
        ("tx_998p4", 0),
        ("rx_clean_998p4", 0),
        ("rx_delay_int_998p4", int(meta["delay_int"]["samples"])),
    ):
        origin = int(coords[name]["sync_origin"])
        sfd = int(coords[name]["sfd_start"])
        expect = origin + n_sync * sps
        if sfd != expect:
            fail(f"{name}: sfd_start {sfd} != sync_origin {origin} + {n_sync}*{sps} = {expect}")
        lines.append(f"  {name}: sync_origin={origin} sfd_start={sfd} (= origin+{n_sync}*{sps})")
    clean_o = int(coords["rx_clean_998p4"]["sync_origin"])
    int_o = int(coords["rx_delay_int_998p4"]["sync_origin"])
    d = int(meta["delay_int"]["samples"])
    if int_o != clean_o + d:
        fail(f"delay_int sync_origin {int_o} != clean {clean_o} + {d}")
    clean_s = int(coords["rx_clean_998p4"]["sfd_start"])
    int_s = int(coords["rx_delay_int_998p4"]["sfd_start"])
    if int_s != clean_s + d:
        fail(f"delay_int sfd_start {int_s} != clean {clean_s} + {d}")
    lines.append(f"  delay_int origin/sfd shifted by {d} vs clean")
    return lines


def expected_file_lengths(meta: dict) -> dict[str, int]:
    rxn = int(meta["rx_window"]["length_998p4"])
    radar_taps = int(meta["cir"]["radar"]["tap_count"])
    demod_taps = int(meta["cir"]["demod_8_30"]["tap_count"])
    expected = {
        "tx_998p4.cf32": int(meta["tx_length_998p4"]),
        "tx_737p28.cf32": int(meta["tx_length_737p28"]),
        "rx_clean_998p4.cf32": rxn,
        "rx_delay_int_998p4.cf32": rxn,
        "rx_delay_frac_998p4.cf32": rxn,
        "cir_raw_clean_radar.cf32": radar_taps,
        "cir_norm_clean_radar.cf32": radar_taps,
        "cir_raw_clean_8_30.cf32": demod_taps,
        "cir_norm_clean_8_30.cf32": demod_taps,
        "cir_raw_delay_int_radar.cf32": radar_taps,
        "cir_norm_delay_int_radar.cf32": radar_taps,
        "cir_raw_delay_int_8_30.cf32": demod_taps,
        "cir_norm_delay_int_8_30.cf32": demod_taps,
    }
    files = meta.get("files") or {}
    for name, n in expected.items():
        if name in files:
            expected[name] = int(files[name])
    return expected


def check_files(meta: dict, out_dir: str) -> list[str]:
    lines = []
    files = expected_file_lengths(meta)
    for name, n_expect in files.items():
        path = os.path.join(out_dir, name)
        if not os.path.isfile(path):
            fail(f"missing {path}")
        x = read_cf32(path)
        if int(x.size) != int(n_expect):
            fail(f"{name}: length {x.size} != metadata {n_expect}")
        lines.append(f"  {name}: {x.size} samples")
    tx = read_cf32(os.path.join(out_dir, "tx_998p4.cf32"))
    peak = float(np.max(np.abs(tx)))
    want = float(meta["peak_amplitude"])
    if abs(peak - want) > 1e-5:
        fail(f"tx_998p4 peak {peak} != {want}")
    lines.append(f"  tx_998p4 peak={peak:.9f} (target {want})")
    if int(tx.size) != int(meta["tx_length_998p4"]):
        fail("tx_length_998p4 mismatch")
    native = read_cf32(os.path.join(out_dir, "tx_737p28.cf32"))
    if int(native.size) != int(meta["tx_length_737p28"]):
        fail("tx_length_737p28 mismatch")
    return lines


def check_native_roundtrip(meta: dict, out_dir: str) -> list[str]:
    """Resample native back 65/48; successive SYNC spacing stays within 1 of 1016."""
    tx = read_cf32(os.path.join(out_dir, "tx_998p4.cf32"))
    native = read_cf32(os.path.join(out_dir, "tx_737p28.cf32"))
    rs = meta["resample"]
    rt = resample_poly(
        native.astype(np.complex128),
        int(rs["decim"]),
        int(rs["interp"]),
        window=("kaiser", 5.0),
    )
    sps = int(meta["samples_per_symbol"])
    n_sync = int(meta["sync_repetitions"])
    sync = tx[:sps].astype(np.complex128)
    # Group delay of the cascade, in 998.4 samples: two FIR delays.
    gd_native = float(rs["group_delay_samples"])
    gd_fwd_work = gd_native * float(rs["decim"]) / float(rs["interp"])
    ntaps_back = 2 * int(rs["half_length"]) * max(int(rs["interp"]), int(rs["decim"])) + 1
    gd_back_work = (ntaps_back - 1) / 2.0 / float(rs["interp"])
    gd_total = gd_fwd_work + gd_back_work

    corr = np.abs(correlate(rt, sync, mode="valid"))
    # resample_poly delay-compensates, so the first SYNC is near 0; an
    # uncompensated FIR would land near gd_total. Take the strongest peak
    # in a window that covers both (not a sidelobe at the theoretical gd).
    search_hi = min(corr.size, max(128, int(np.ceil(gd_total)) + 64))
    first = int(np.argmax(corr[:search_hi]))
    peaks = [first]
    for k in range(1, n_sync):
        center = first + k * sps
        lo = max(0, center - 2)
        hi = min(corr.size, center + 3)
        if hi <= lo:
            fail(f"round-trip SYNC {k} search out of range (center={center})")
        local = lo + int(np.argmax(corr[lo:hi]))
        peaks.append(local)
    peaks = np.asarray(peaks, dtype=np.int64)
    spacing = np.diff(peaks)
    err = np.abs(spacing.astype(np.float64) - float(sps))
    max_err = float(np.max(err))
    mean_sp = float(np.mean(spacing))
    accum = peaks - (peaks[0] + np.arange(n_sync, dtype=np.int64) * sps)
    max_accum = int(np.max(np.abs(accum)))
    if max_err > 1.0 + 1e-9 or max_accum > 1:
        fail(
            f"native round-trip SYNC spacing drifted: mean={mean_sp:.4f} "
            f"max|err|={max_err:.4f} max_accum={max_accum} (limit 1 sample vs {sps})"
        )
    sfd_expect = first + n_sync * sps
    sfd_lo = max(0, sfd_expect - 2)
    sfd_hi = min(corr.size, sfd_expect + 3)
    # 4z2 first symbol is -SYNC, so |corr| still peaks at SFD start.
    sfd_peak = sfd_lo + int(np.argmax(corr[sfd_lo:sfd_hi])) if sfd_hi > sfd_lo else -1
    sfd_err = abs(sfd_peak - sfd_expect) if sfd_peak >= 0 else 99
    if sfd_err > 1:
        fail(f"round-trip SFD location {sfd_peak} vs expect {sfd_expect} (err {sfd_err})")
    return [
        f"  round-trip length={rt.size} first_SYNC={first} (gd≈{gd_total:.3f})",
        f"  SYNC spacing mean={mean_sp:.6f} max|err|={max_err:.6f} "
        f"max_accum={max_accum} samples vs {sps}",
        f"  SFD peak={sfd_peak} expect={sfd_expect} |err|={sfd_err}",
    ]


def check_cir_peaks(meta: dict) -> list[str]:
    radar = meta["cir"]["radar"]
    lines = []
    # Pulse shaping can move the sampled_code peak by ~1 tap; allow ±2.
    for label, measured, expected in (
        ("clean_radar", radar["peak_tap_measured_clean"], radar["peak_tap_expected_clean"]),
        (
            "delay_int_radar",
            radar["peak_tap_measured_delay_int"],
            radar["peak_tap_expected_delay_int"],
        ),
    ):
        err = abs(int(measured) - int(expected))
        lines.append(f"  CIR {label}: measured={measured} expected={expected} |err|={err}")
        if err > 2:
            fail(f"CIR {label} peak tap {measured} more than 2 taps from expected {expected}")
    demod = meta["cir"]["demod_8_30"]
    err8 = abs(int(demod["peak_tap_measured_clean"]) - int(demod["peak_tap_expected_clean"]))
    lines.append(
        f"  CIR clean_8_30: measured={demod['peak_tap_measured_clean']} "
        f"expected={demod['peak_tap_expected_clean']} |err|={err8}"
    )
    if err8 > 2:
        fail("CIR clean 8/30 peak tap off by more than 2")
    shift = int(radar["peak_tap_measured_delay_int"]) - int(radar["peak_tap_measured_clean"])
    want_shift = int(meta["delay_int"]["samples"])
    lines.append(f"  CIR delay_int-clean peak shift={shift} (channel delay {want_shift})")
    if shift != want_shift:
        fail(f"CIR peak shift {shift} != integer delay {want_shift}")
    return lines


def main() -> int:
    out_dir = HERE
    if len(sys.argv) > 1:
        out_dir = sys.argv[1]
    meta_path = os.path.join(out_dir, "metadata.json")
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)
    report = ["UWB radar golden verification", f"  metadata={meta_path}"]
    try:
        if meta.get("dtype") != "complex64":
            fail(f"dtype {meta.get('dtype')!r} != complex64")
        if int(meta.get("sample_index_base", -1)) != 0:
            fail("sample_index_base must be 0")
        if int(meta["sync_repetitions"]) != 64:
            fail("sync_repetitions must be 64")
        if meta.get("sfd_mode") != "4z2":
            fail("sfd_mode must be 4z2")
        if list(meta["sfd_sequence"]) != [-1, -1, -1, 1, -1, -1, 1, -1]:
            fail("sfd_sequence is not 4z2")
        report.extend(check_files(meta, out_dir))
        report.extend(check_coords(meta))
        report.extend(check_cir_peaks(meta))
        report.extend(check_native_roundtrip(meta, out_dir))
    except AssertionError as exc:
        print("\n".join(report))
        print("FAIL:", exc)
        return 1
    report.append("PASS")
    print("\n".join(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
