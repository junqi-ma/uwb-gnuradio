#!/usr/bin/env python3
"""Zero-hardware study of a UWB communication interferer's effect on the
monostatic sensing CIR as a function of its carrier offset df.

Method
------
Take an existing jam-off RX capture (native SC16 windows + per-frame metadata)
and add a synthetic HRP preamble/packet of a *different* preamble code
(default 10) on top of each RX frame, then re-run a numpy reference copy of
the production radar CIR estimator:

    native SC16 window
      -> 65/48 polyphase resample to the 998.4 MS/s work grid
      -> origin = map(window_start) + (map(pre) - map(window_start))
                  + llround(cal_delay_work)
      -> coherent average of windows at origin + k*SPS, k = cir_skip .. reps-1
      -> correlate with the sampled reference preamble code (8 samples/chip)
      -> 116 raw complex CIR taps

The reference pipeline is validated against the shipped cir.cf32 of the
capture (self-check in the output / CSV report header).  The CIR operator is
linear in the RX window, so the CIR of (capture + jammer) equals the CIR of
the capture plus the CIR of the jammer term; both are computed and summed,
which is exactly "compute the reference CIR on the composite" but avoids one
resample per (frame, df).

This is a numerical experiment only: no UHD device is opened and no file in
the repo is modified.

See docs/phase1/分析_X410双TX_离线数字注入CFO零点.md for the write-up.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

import numpy as np
from scipy.signal import upfirdn

WORK_HZ = 998400000.0
SPS = 1016          # 998.4 MS/s work samples per HRP SYMBOL
INTERP = 65
DECIM = 48
SFD_MODE = "4z2"
IQ_SCALE = 32768.0
TAP_COUNT = 116
DEFAULT_CAPTURE = "/tmp/opencode/cg600_rx100"
DEFAULT_OUTDIR = "/tmp/opencode/jam_offline"

# Coarse offsets requested by the experiment, plus fine steps around the
# theoretical first null at WORK_HZ / (2*SPS) = 491.34 kHz.
NULL_KHZ = WORK_HZ / (2.0 * SPS) / 1e3
_DEFAULT_DF_KHZ = [
    0.0, 50.0, 100.0, 200.0, 245.67, 400.0, 450.0, 480.0,
    NULL_KHZ, 500.0, 550.0, 700.0, 800.0, 900.0, 982.68, 1100.0,
]
for _d in (-80.0, -40.0, -20.0, -10.0, -5.0, -2.0, -1.0,
           1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0):
    _DEFAULT_DF_KHZ.append(NULL_KHZ + _d)
DEFAULT_DF_HZ = sorted(set(round(v * 1e3, 3) for v in _DEFAULT_DF_KHZ))


# ---------------------------------------------------------------------------
# setup helpers (import the app module lazily so --help needs no bindings)
# ---------------------------------------------------------------------------

def find_repo_root(start=None):
    cur = os.path.abspath(start or os.path.dirname(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(cur, "testdata", "resampler_65_48")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise SystemExit("cannot find repo root from %s" % __file__)


def load_app_module():
    """Import gr-uwb/apps/x410_cg400_hrp_echo_cir.py (bootstraps uwb bindings)."""
    apps = os.path.join(find_repo_root(), "gr-uwb", "apps")
    if apps not in sys.path:
        sys.path.insert(0, apps)
    import x410_cg400_hrp_echo_cir as base  # noqa: E402
    return base


def load_taps(path):
    taps = np.fromfile(path, dtype=np.float32)
    if taps.size == 0:
        raise SystemExit("empty taps file: %s" % path)
    return taps


def parse_preamble_codes(header_path):
    txt = open(header_path, encoding="utf-8").read()
    out = {}
    pat = re.compile(r"kPreambleCode(\d+)\s*=\s*\{\s*\{([^}]*)\}\s*\}")
    for m in pat.finditer(txt):
        vals = [int(v) for v in re.findall(r"-?\d+", m.group(2))]
        out[int(m.group(1))] = np.asarray(vals, dtype=np.float64)
    if not out:
        raise SystemExit("cannot parse preamble codes from %s" % header_path)
    return out


# ---------------------------------------------------------------------------
# REFERENCE implementation of gr::uwb::radar::estimate_radar_cir
# ---------------------------------------------------------------------------

class CirOperator:
    """Reference estimator: native window -> 116 raw complex CIR taps.

    Mirrors gr-uwb/include/gnuradio/uwb/uwb_radar_cir_core.h +
    uwb_radar_cir_estimator.h + the 65/48 PDU resampler mapping
    (uwb_pdu_rational_resampler_ccf_65_48.cc).  This is a REFERENCE, not the
    production block; it is checked against the shipped cir.cf32 below.
    """

    def __init__(self, taps, codes, code_index, pre_guard_native,
                 cal_delay_native, sync_reps, sps=SPS, cir_pre=16,
                 cir_post=100, cir_skip=10, cir_reps=0,
                 rate_native=737280000.0):
        self.taps = np.asarray(taps, dtype=np.float64)
        self.T = self.taps.size
        self.code = np.asarray(codes[code_index], dtype=np.float64)
        self.sync_reps = int(sync_reps)
        self.sps = int(sps)
        self.cir_pre = int(cir_pre)
        self.cir_post = int(cir_post)
        self.cir_skip = int(cir_skip)
        self.cir_reps = int(cir_reps) if int(cir_reps) > 0 else \
            self.sync_reps - self.cir_skip
        self.rate_native = float(rate_native)
        self.pre_guard = int(pre_guard_native)
        self.cal_work = float(cal_delay_native) * WORK_HZ / self.rate_native

        # sampled reference code: spread by 4, place on the 2-sample chip
        # grid -> non-zero every 8 work samples (code_len 127 -> 1016).
        spread = np.zeros(508, dtype=np.float64)
        for c in range(self.code.size):
            pos = c * 4
            if pos < 508:
                spread[pos] = self.code[c]
        self.sampled_code = np.zeros(self.sps, dtype=np.float64)
        self.sampled_code[0::2] = spread
        self.code_energy = float(np.sum(self.sampled_code ** 2))
        self.nz = np.nonzero(self.sampled_code)[0]

        # work-domain origin, exactly as the production adapter computes it.
        self.origin = (self._map(0)
                       + (self._map(self.pre_guard) - self._map(0))
                       + int(math.floor(self.cal_work + 0.5)))
        self.wlen = self.sps + self.cir_pre + self.cir_post - 1

    def _map(self, p):
        return (2 * int(p) * INTERP + (self.T - 1) + DECIM) // (2 * DECIM)

    def resample(self, native):
        return upfirdn(self.taps, np.asarray(native, dtype=np.complex128),
                       INTERP, DECIM).astype(np.complex64)

    def cir_from_work(self, rx_work):
        """CIR of an already-resampled work window (reference estimator)."""
        avg = np.zeros(self.wlen, dtype=np.complex128)
        valid = 0
        last_ok = rx_work.size - self.wlen
        for k in range(self.cir_skip, self.cir_skip + self.cir_reps):
            lo = self.origin + k * self.sps - self.cir_pre
            if lo < 0 or lo > last_ok:
                continue
            avg += rx_work[lo:lo + self.wlen]
            valid += 1
        if valid == 0:
            raise RuntimeError("no valid CIR repetitions (origin=%d n=%d)"
                               % (self.origin, rx_work.size))
        avg /= valid
        raw = np.zeros(TAP_COUNT, dtype=np.complex128)
        for nn in range(TAP_COUNT):
            raw[nn] = np.sum(avg[nn + self.nz] * self.sampled_code[self.nz]) \
                / self.code_energy
        return raw

    def cir_from_native(self, native):
        return self.cir_from_work(self.resample(native))


# ---------------------------------------------------------------------------
# capture loading
# ---------------------------------------------------------------------------

def load_capture(cap_dir):
    meta = json.load(open(os.path.join(cap_dir, "metadata.json"), encoding="utf-8"))
    recs = [json.loads(l) for l in
            open(os.path.join(cap_dir, "capture.jsonl"), encoding="utf-8")
            if l.strip()]
    iq = np.memmap(os.path.join(cap_dir, "capture.iq"), dtype=np.int16, mode="r")
    pub = int(meta.get("echo", {}).get("publish_native", 0))
    if pub <= 0:
        pub = int(recs[0]["sample_count"]) if recs else 0
    return meta, recs, iq, pub


def frame_native(iq, rec, limit=None):
    off = int(rec["file_offset_samples"])
    n = int(rec["sample_count"])
    if limit is not None:
        n = min(n, int(limit))
    seg = np.asarray(iq[2 * off:2 * (off + n)]).reshape(-1, 2)
    x = (seg[:, 0] + 1j * seg[:, 1]).astype(np.complex64) / IQ_SCALE
    return x


# ---------------------------------------------------------------------------
# jammer construction
# ---------------------------------------------------------------------------

def build_jammer(base, rate_native, jam_mode, jam_code_index, sync_reps,
                 insert_sts, psdu_hex, pulse_shape, sigma_ns, bw_mhz,
                 pri_s=0.05):
    profile = base.NativeRateProfile(rate_native)
    psdu = base.hex_to_bytes(psdu_hex)
    src = base.uwb.hrp_packet_source(
        psdu, sync_reps, SFD_MODE, jam_code_index, 0.8, pri_s, False,
        insert_sts, False, pulse_shape, sigma_ns, bw_mhz, "")
    work = np.asarray(src.samples(), dtype=np.complex64)
    if jam_mode == "preamble":
        work = work[:sync_reps * SPS]
    native = profile.tx_native(work).astype(np.complex64)
    return native, float(np.max(np.abs(native))) or 1.0


def build_jam_term(jam_native, pub_len, offset, df_hz, rate_native, scale):
    """jam_scale * jam_native placed at `offset`, x exp(j2*pi*df*n/fs)."""
    term = np.zeros(pub_len, dtype=np.complex128)
    n = jam_native.size
    if offset < 0 or offset >= pub_len:
        raise SystemExit("jam offset %d outside [0,%d)" % (offset, pub_len))
    m = min(n, pub_len - offset)
    idx = np.arange(m, dtype=np.float64)
    cfo = np.exp(1j * 2.0 * np.pi * df_hz * idx / rate_native)
    term[offset:offset + m] = jam_native[:m].astype(np.complex128) * cfo * scale
    return term


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def summarize(comp_raw, base_raw, peak_tap):
    """comp_raw/base_raw: (nframes, 116) complex. Returns metric dict."""
    mag_mean = np.abs(comp_raw).mean(axis=0)
    peak = float(mag_mean[peak_tap])
    mask = np.ones(TAP_COUNT, dtype=bool)
    mask[max(0, peak_tap - 1):min(TAP_COUNT, peak_tap + 2)] = False
    floor = float(mag_mean[mask].mean())
    diff = comp_raw - base_raw
    base_pow = float(np.mean(np.abs(base_raw) ** 2)) or 1e-30
    evm_power = float(np.mean(np.abs(diff) ** 2)) / base_pow
    return {
        "floor": floor,
        "peak": peak,
        "peak_over_floor": peak / floor if floor > 0 else float("inf"),
        "evm_power": evm_power,
    }


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def build_parser():
    repo = find_repo_root()
    p = argparse.ArgumentParser(
        description="Offline digital jammer injection into a jam-off radar "
                    "capture, to map the sensing CIR vs interferer carrier "
                    "offset.  No hardware is used.")
    p.add_argument("--capture", default=DEFAULT_CAPTURE,
                   help="capture directory with capture.iq/capture.jsonl/"
                        "metadata.json (and optionally cir.cf32)")
    p.add_argument("--output", default=DEFAULT_OUTDIR,
                   help="output directory for degradation.csv / png")
    p.add_argument("--taps", default=os.path.join(
        repo, "testdata", "resampler_65_48", "taps_quality_minorder.txt"),
        help="65/48 float32 resampler taps (default: repo quality_minorder)")
    p.add_argument("--header", default=os.path.join(
        repo, "gr-uwb", "include", "gnuradio", "uwb", "uwb_phy_profile.h"),
        help="header holding the HRP preamble codes")
    p.add_argument("--frames", type=int, default=100,
                   help="number of RX frames to use (<=0 = all)")
    p.add_argument("--jam-scale", type=float, default=1.0,
                   help="linear amplitude of the injected jammer")
    p.add_argument("--jam-mode", choices=["preamble", "packet"],
                   default="preamble",
                   help="preamble = pure periodic SYNC (null test); "
                        "packet = full HRP packet incl. SFD/STS")
    p.add_argument("--jam-code-index", type=int, default=10,
                   choices=[9, 10, 11, 12],
                   help="interferer preamble code (capture is code 9)")
    p.add_argument("--jam-delay-native", type=int, default=None,
                   help="native sample where the jammer starts "
                        "(default: pre_guard + cal_delay)")
    p.add_argument("--cir-skip", type=int, default=10)
    p.add_argument("--cir-reps", type=int, default=0,
                   help="CIR averaged reps; 0 = sync_reps - skip (production)")
    p.add_argument("--df-hz", type=float, nargs="+", default=None,
                   help="explicit carrier offsets in Hz")
    p.add_argument("--df-max-hz", type=float, default=0.0,
                   help="if >0, scan 0..df-max with --df-step instead")
    p.add_argument("--df-step-hz", type=float, default=2000.0)
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--no-selfcheck", action="store_true")
    return p


def main():
    args = build_parser().parse_args()

    base = load_app_module()
    repo = find_repo_root()
    os.makedirs(args.output, exist_ok=True)

    meta, recs, iq, pub = load_capture(args.capture)
    if args.frames and args.frames > 0:
        recs = recs[:args.frames]
    nframes = len(recs)
    rate_native = float(meta["rate_native_hz"])
    code_index = int(meta["code_index"])
    sync_reps = int(meta["sync_repetitions"])
    cal_native = float(meta.get("cal_delay_native_samples",
                                meta.get("echo", {}).get("cal_delay_native", 0.0)))
    insert_sts = bool(meta.get("insert_sts", True))
    pre_guard = int(recs[0]["pre_guard_samples"])

    taps = load_taps(args.taps)
    codes = parse_preamble_codes(args.header)
    if code_index not in codes:
        raise SystemExit("capture code %d missing from header" % code_index)

    cir = CirOperator(taps, codes, code_index, pre_guard, cal_native,
                      sync_reps, cir_skip=args.cir_skip,
                      cir_reps=args.cir_reps, rate_native=rate_native)

    print("capture=%s" % args.capture)
    print("frames=%d window_native=%d publish_native=%d rate_native=%.1f MS/s"
          % (nframes, int(recs[0]["sample_count"]), pub, rate_native / 1e6))
    print("code_index=%d (capture) jam_code_index=%d jam_mode=%s jam_scale=%.3g"
          % (code_index, args.jam_code_index, args.jam_mode, args.jam_scale))
    print("taps=%d map(0)=%d pre=%d cal_native=%.1f cal_work=%.4f "
          "origin=%d sync_reps=%d cir_reps=%d"
          % (taps.size, cir._map(0), pre_guard, cal_native, cir.cal_work,
             cir.origin, sync_reps, cir.cir_reps))

    # ---- A. native windows + baseline reference CIR per frame -------------
    pub = min(pub, int(recs[0]["sample_count"]))
    base_raw = np.empty((nframes, TAP_COUNT), dtype=np.complex128)
    for fi, rec in enumerate(recs):
        nat = frame_native(iq, rec, limit=pub)
        base_raw[fi] = cir.cir_from_work(cir.resample(nat))
    print("baseline CIR computed: mean|CIR|=%.6g" % np.abs(base_raw).mean())

    # ---- D. self-check against the shipped production CIR ----------------
    selfcheck = None
    cir_path = os.path.join(args.capture, "cir.cf32")
    if not args.no_selfcheck and os.path.isfile(cir_path):
        prod = np.fromfile(cir_path, dtype=np.complex64).reshape(-1, TAP_COUNT)
        prod = prod[:nframes]
        corr, scale, peak_agree = [], [], []
        for fi in range(nframes):
            a, b = prod[fi], base_raw[fi]
            corr.append(abs(np.vdot(a, b)) /
                        (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30))
            scale.append(np.linalg.norm(b) / (np.linalg.norm(a) + 1e-30))
            peak_agree.append(int(np.argmax(np.abs(a)) == np.argmax(np.abs(b))))
        selfcheck = {
            "corr_mean": float(np.mean(corr)),
            "corr_min": float(np.min(corr)),
            "scale_mean": float(np.mean(scale)),
            "peak_tap_agree": float(np.mean(peak_agree)),
            "max_abs_first": float(np.max(np.abs(prod[0] - base_raw[0]))),
        }
        print("SELF-CHECK vs %s: corr mean=%.6f min=%.6f  |CIR| scale=%.6f  "
              "peak-tap agree=%.3f  max|diff|(frame0)=%.3g"
              % (cir_path, selfcheck["corr_mean"], selfcheck["corr_min"],
                 selfcheck["scale_mean"], selfcheck["peak_tap_agree"],
                 selfcheck["max_abs_first"]))
    else:
        print("SELF-CHECK skipped (no cir.cf32 or --no-selfcheck)")

    # ---- B. jammer waveform ----------------------------------------------
    jam_native, jam_peak = build_jammer(
        base, rate_native, args.jam_mode, args.jam_code_index, sync_reps,
        insert_sts, "47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221",
        "legacy", 2.5, 200.0)
    if args.jam_delay_native is not None:
        jam_off = int(args.jam_delay_native)
    else:
        jam_off = int(round(pre_guard + cal_native))
    print("jammer mode=%s work/native=%d peak=%.4g rms=%.4g placed@native=%d"
          % (args.jam_mode, jam_native.size, jam_peak,
             float(np.sqrt(np.mean(np.abs(jam_native) ** 2))), jam_off))

    # ---- E. sweep df ------------------------------------------------------
    if args.df_hz is not None:
        dfs = list(args.df_hz)
    elif args.df_max_hz > 0:
        dfs = list(np.arange(0.0, args.df_max_hz + 0.5 * args.df_step_hz,
                             args.df_step_hz))
    else:
        dfs = list(DEFAULT_DF_HZ)

    # jammer-only CIR per df (identical for every frame; CIR is linear).
    jam_raw = {}
    for df in dfs:
        term = build_jam_term(jam_native, pub, jam_off, df, rate_native,
                              args.jam_scale)
        jam_raw[df] = cir.cir_from_work(cir.resample(term))

    # The jammer term is deterministic (identical every frame) and its peak
    # tap is set by the injection delay, not by df; use df=0 so the tap is
    # found on the unsuppressed jammer response even for null-centred scans.
    if 0.0 in jam_raw:
        peak_tap = int(np.argmax(np.abs(jam_raw[0.0])))
    else:
        term0 = build_jam_term(jam_native, pub, jam_off, 0.0, rate_native,
                               args.jam_scale)
        peak_tap = int(np.argmax(np.abs(cir.cir_from_work(cir.resample(term0)))))

    rows = []
    for df in dfs:
        comp = base_raw + jam_raw[df]
        m = summarize(comp, base_raw, peak_tap)
        rows.append((float(df), m))

    csv_path = os.path.join(args.output, "degradation.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("df_hz,floor,peak,peak_over_floor,evm_power\n")
        for df, m in rows:
            f.write("%.1f,%.8g,%.8g,%.8g,%.8g\n"
                    % (df, m["floor"], m["peak"], m["peak_over_floor"],
                       m["evm_power"]))
    print("wrote %s (%d rows)" % (csv_path, len(rows)))

    # ---- F. optional plot -------------------------------------------------
    if not args.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            x = np.array([r[0] for r in rows]) / 1e3
            evm = np.array([r[1]["evm_power"] for r in rows])
            pfr = np.array([r[1]["peak_over_floor"] for r in rows])
            order = np.argsort(x)
            fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
            ax[0].plot(x[order], 10 * np.log10(evm[order] + 1e-30))
            ax[0].set_ylabel("10log10 EVM power (rel)")
            ax[1].plot(x[order], pfr[order])
            ax[1].set_ylabel("peak/floor")
            ax[1].set_xlabel("jammer carrier offset (kHz)")
            for a in ax:
                a.axvline(NULL_KHZ, color="r", ls="--", lw=0.8)
                a.grid(True, alpha=0.3)
            fig.tight_layout()
            png = os.path.join(args.output, "degradation.png")
            fig.savefig(png, dpi=120)
            print("wrote %s" % png)
        except Exception as e:  # matplotlib missing / headless quirks
            print("plot skipped: %s" % e)

    # ---- summary table ----------------------------------------------------
    print("\n%-12s %-12s %-12s %-14s %-12s" % (
        "df_kHz", "floor", "peak", "peak/floor", "evm_power"))
    for df, m in rows:
        print("%-12.3f %-12.5g %-12.5g %-14.5g %-12.5g"
              % (df / 1e3, m["floor"], m["peak"], m["peak_over_floor"],
                 m["evm_power"]))

    if rows:
        best = min(rows, key=lambda r: r[1]["evm_power"])
        print("\nobserved EVM minimum: df=%.1f Hz (%.4f kHz) evm=%.5g"
              % (best[0], best[0] / 1e3, best[1]["evm_power"]))
        print("predicted null f_work/(2*SPS) = %.4f kHz" % NULL_KHZ)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
