#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Equiripple (Parks-McClellan / Remez) redesign of the 65/48 anti-imaging FIR.

Task: docs/performance/OpenCode_65_48等波纹FIR设计实验任务书_2026-11-11.md

Frozen spec (never relaxed by this script)
------------------------------------------
    Fv = 65 * 737.28 MHz = 47.9232 GHz
    passband [0, B]          : 20log10(max|H|/min|H|) <= 0.1 dB
    stopband [Fin-B, Fv/2]   : 20log10(max|H_stop|/min|H_pass|) <= -70 dB
    sum(h32) in float64      : |sum - 65| < 1e-3
    odd length, real, symmetric (Type I)

The *design* target defaults to 0.09 dB / 72 dB so that float32 quantisation
and numerical slack are covered.  That is a design margin, not a relaxation:
acceptance is always re-checked at 0.1 dB / 70 dB by the independent verifier.

Why length class matters more than raw tap count
------------------------------------------------
The production kernel pads the per-phase length to a multiple of 8:

    H    = ceil(T / 65)
    Hpad = 8 * ceil(H / 8)

and the AVX2 inner loop always runs `Hpad` complex taps per output sample.
So the *cost* is proportional to Hpad, not to T.  A redesign only buys speed
when it crosses a class boundary:

    realtime  Hpad 24 -> 16   needs T <= 1040
    quality   Hpad 48 -> 40   needs T <= 2600
    quality   Hpad 48 -> 32   needs T <= 2080

Within a class the largest admissible odd T is therefore the best choice: it
costs exactly the same as the smallest one but has the most margin.

Usage
-----
    python3 testdata/design_resampler_equiripple.py --profile realtime \
        --t-min 900 --t-max 1400 --out testdata/resampler_65_48_equiripple
"""
import argparse
import csv
import hashlib
import json
import os
import sys
import time

import numpy as np
from scipy.signal import remez

R_IN = 737.28e6
R_OUT = 998.4e6
L, M = 65, 48
FV = L * R_IN

PASSBAND_MHZ = {"realtime": 290.0, "quality": 330.0}

# acceptance (frozen)
SPEC_RIPPLE_DB = 0.1
SPEC_STOP_DB = -70.0
DC_TARGET = float(L)
DC_TOL = 1e-3

# default design margin
DEF_RIPPLE_DB = 0.09
DEF_STOP_DB = -72.0


# ---------------------------------------------------------------- helpers ---

def hpad_of(T):
    H = -(-int(T) // L)
    return 8 * (-(-H // 8)), H


def delta_p(ripple_db):
    a = 10.0 ** (ripple_db / 20.0)
    return (a - 1.0) / (a + 1.0)


def delta_s(stop_db):
    return 10.0 ** (stop_db / 20.0)


def design(T, pb_hz, sb_hz, ws, grid_density=32, maxiter=100):
    """Remez design, DC-normalised to sum == 65, returned as float32."""
    h = remez(int(T), [0.0, pb_hz / FV, sb_hz / FV, 0.5], [1.0, 0.0],
              weight=[1.0, float(ws)], fs=1.0,
              maxiter=int(maxiter), grid_density=int(grid_density))
    if not np.all(np.isfinite(h)):
        raise FloatingPointError("remez returned non-finite coefficients")
    h = h * (L / h.sum())
    return h.astype("<f4")


def screen(h32, pb_hz, sb_hz, nfft=1 << 20):
    """Cheap design-time screen on the *quantised* coefficients.

    This is a screening tool only.  Acceptance is decided by
    testdata/verify_resampler_equiripple.py, which re-derives everything from
    the binary file and refines every extremum off-grid.
    """
    h = h32.astype(np.float64)
    H = np.abs(np.fft.rfft(h, nfft))
    f = np.arange(H.size) * (FV / nfft)
    pb = H[f <= pb_hz]
    sb = H[f >= sb_hz]
    # exact critical frequencies, so a grid miss cannot flatter the result
    crit = np.abs(np.exp(-2j * np.pi * np.outer(
        np.array([0.0, pb_hz, sb_hz, FV / 2.0]), np.arange(h.size)) / FV) @ h)
    pb_max = max(pb.max(), crit[0], crit[1])
    pb_min = min(pb.min(), crit[0], crit[1])
    sb_max = max(sb.max(), crit[2], crit[3])
    return {
        "ripple_db": float(20 * np.log10(pb_max / pb_min)),
        "rel_stop_db": float(20 * np.log10(sb_max / pb_min)),
        "rel_stop_vs_max_db": float(20 * np.log10(sb_max / pb_max)),
        "dc_err": float(h.sum() - DC_TARGET),
    }


def meets(m, ripple_db, stop_db):
    return (m["ripple_db"] <= ripple_db
            and m["rel_stop_db"] <= stop_db
            and abs(m["dc_err"]) < DC_TOL)


def sha256_arr(h32):
    return hashlib.sha256(h32.tobytes()).hexdigest()


# ----------------------------------------------------------------- search ---

def evaluate(T, pb_hz, sb_hz, ws, grid_density, maxiter, log):
    """Design + screen one (T, ws).  Never raises; failures are logged."""
    rec = {"T": int(T), "ws": float(ws),
           "H": int(-(-T // L)), "Hpad": hpad_of(T)[0]}
    t0 = time.time()
    try:
        h32 = design(T, pb_hz, sb_hz, ws, grid_density, maxiter)
    except ValueError as e:
        rec.update(status="remez_no_converge", detail=str(e)[:120])
        log.append(rec)
        return None, rec
    except FloatingPointError as e:
        rec.update(status="non_finite", detail=str(e)[:120])
        log.append(rec)
        return None, rec
    m = screen(h32, pb_hz, sb_hz)
    rec.update(status="ok", **m, design_s=round(time.time() - t0, 3))
    log.append(rec)
    return h32, rec


def search_profile(profile, t_min, t_max, weights, ripple_db, stop_db,
                   grid_density, maxiter, verbose=True):
    pb_hz = PASSBAND_MHZ[profile] * 1e6
    sb_hz = R_IN - pb_hz
    log = []

    # --- stage 1: coarse length sweep at the theory weight ----------------
    ws_mid = weights[len(weights) // 2]
    coarse = []
    for T in range(t_min | 1, t_max + 1, 32):
        h32, rec = evaluate(T, pb_hz, sb_hz, ws_mid, grid_density, maxiter, log)
        coarse.append(rec)
        if verbose:
            st = rec.get("status")
            extra = ("" if st != "ok" else
                     f" ripple={rec['ripple_db']:7.4f} stop={rec['rel_stop_db']:8.2f}")
            print(f"  coarse T={T:5d} Hpad={rec['Hpad']:3d} {st}{extra}")

    ok_T = [r["T"] for r in coarse if r.get("status") == "ok"
            and meets(r, ripple_db, stop_db)]
    if not ok_T:
        return log, None
    first_ok = min(ok_T)

    # --- stage 2: every odd length at the boundary ------------------------
    lo = max(t_min, first_ok - 96)
    boundary = []
    for T in range(lo | 1, first_ok + 1, 2):
        h32, rec = evaluate(T, pb_hz, sb_hz, ws_mid, grid_density, maxiter, log)
        boundary.append(rec)
    ok_b = [r["T"] for r in boundary if r.get("status") == "ok"
            and meets(r, ripple_db, stop_db)]
    shortest = min(ok_b) if ok_b else first_ok
    if verbose:
        print(f"  shortest passing T at ws={ws_mid:.2f}: {shortest}")

    # --- stage 3: weight refinement at each interesting length ------------
    # For every Hpad class that the shortest candidate could reach, take the
    # LARGEST odd T inside that class (same kernel cost, best margin) and
    # search the weight that maximises the worst-case *relative* margin.
    #
    # The objective must be scale-free: ripple is measured in dB against a
    # 0.09 dB target while the stopband is measured against 72 dB, so a raw
    # min() would always be dominated by whichever unit happens to be larger.
    def rel_margin(rec):
        r1 = (ripple_db - rec["ripple_db"]) / ripple_db
        r2 = (abs(rec["rel_stop_db"]) - abs(stop_db)) / abs(stop_db)
        return min(r1, r2)

    def best_ws_at(T, ws_list):
        cand = None
        for ws in ws_list:
            h32, rec = evaluate(T, pb_hz, sb_hz, ws, grid_density, maxiter, log)
            if rec.get("status") != "ok" or not meets(rec, ripple_db, stop_db):
                continue
            m = rel_margin(rec)
            if cand is None or m > cand[0]:
                cand = (m, ws, h32, rec)
        return cand

    classes = sorted({hpad_of(T)[0] for T in
                      range(shortest, min(t_max, shortest + 65 * 48) + 1, 2)})
    best = {}
    for hp in classes:
        T_cls = min(t_max, L * hp)
        if T_cls % 2 == 0:
            T_cls -= 1
        if T_cls < shortest:
            continue
        cand = best_ws_at(T_cls, weights)
        if cand is None:
            continue
        # local refinement: golden-section on log(ws) around the grid winner
        gr = (np.sqrt(5.0) - 1.0) / 2.0
        lo, hi = np.log(cand[1] / 2.0), np.log(cand[1] * 2.0)
        for _ in range(12):
            a = hi - gr * (hi - lo)
            b = lo + gr * (hi - lo)
            ca = best_ws_at(T_cls, [float(np.exp(a))])
            cb = best_ws_at(T_cls, [float(np.exp(b))])
            if ca is not None and (cb is None or ca[0] >= cb[0]):
                hi, keep = b, ca
            else:
                lo, keep = a, cb
        for extra in (keep, best_ws_at(T_cls, [float(np.exp(0.5 * (lo + hi)))])):
            if extra is not None and extra[0] > cand[0]:
                cand = extra
        best[hp] = {"margin": cand[0], "ws": cand[1],
                    "h32": cand[2], "rec": cand[3], "T": T_cls}
        if verbose:
            print(f"  class Hpad={hp:3d} T={T_cls:5d} ws={cand[1]:.3f} "
                  f"ripple={cand[3]['ripple_db']:.4f} "
                  f"stop={cand[3]['rel_stop_db']:.2f} "
                  f"rel_margin={cand[0]:.3f}")
    return log, {"shortest": shortest, "classes": best, "ws_mid": ws_mid}


# ------------------------------------------------------------------ output ---

def write_candidate(out_dir, profile, tag, h32, meta):
    base = os.path.join(out_dir, f"taps_{profile}_eq_{tag}")
    h32.tofile(base + ".txt")
    with open(base + ".csv", "w") as f:
        for t in h32.astype(np.float64):
            f.write(f"{t:.17g}\n")
    meta = dict(meta)
    meta.update(profile=profile, tag=tag, taps=int(h32.size),
                sha256=sha256_arr(h32), file=os.path.basename(base + ".txt"))
    with open(base + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    return base, meta


def plot_candidate(out_dir, profile, tag, h32, pb_hz, sb_hz, meta):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"  (plot skipped: {e})")
        return
    h = h32.astype(np.float64)
    n = 1 << 21
    H = np.abs(np.fft.rfft(h, n))
    f = np.arange(H.size) * (FV / n)
    db = 20 * np.log10(np.maximum(H, 1e-30))
    fig, ax = plt.subplots(3, 1, figsize=(11, 10))
    ax[0].plot(f / 1e6, db, lw=0.6)
    ax[0].axvline(pb_hz / 1e6, color="g", ls="--", lw=0.8, label="passband edge")
    ax[0].axvline(sb_hz / 1e6, color="r", ls="--", lw=0.8, label="stopband edge")
    ax[0].set_xlim(0, FV / 2e6)
    ax[0].set_xlabel("MHz"); ax[0].set_ylabel("|H| dB")
    ax[0].set_title(f"{profile} {tag}: T={meta['taps']} Hpad={meta['Hpad']} "
                    f"ripple={meta['ripple_db']:.4f} dB "
                    f"stop={meta['rel_stop_db']:.2f} dB")
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    m = f <= pb_hz
    ax[1].plot(f[m] / 1e6, db[m], lw=0.9)
    ax[1].set_xlabel("MHz"); ax[1].set_ylabel("|H| dB")
    ax[1].set_title("passband"); ax[1].grid(alpha=0.3)
    m = f >= sb_hz
    ax[2].plot(f[m] / 1e6, db[m], lw=0.6)
    ax[2].set_xlabel("MHz"); ax[2].set_ylabel("|H| dB")
    ax[2].set_title("stopband"); ax[2].grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(out_dir, f"response_{profile}_{tag}.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)
    print(f"  wrote {p}")

    fig, ax = plt.subplots(figsize=(11, 3.5))
    ax.plot(np.arange(h.size), h, lw=0.8)
    ax.set_xlabel("tap"); ax.set_ylabel("h")
    ax.set_title(f"{profile} {tag} impulse response (T={h.size})")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(out_dir, f"impulse_{profile}_{tag}.png")
    fig.savefig(p, dpi=110)
    plt.close(fig)


# -------------------------------------------------------------------- main ---

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profile", required=True, choices=sorted(PASSBAND_MHZ))
    ap.add_argument("--t-min", type=int, default=None)
    ap.add_argument("--t-max", type=int, default=None)
    ap.add_argument("--weight-points", type=int, default=9)
    ap.add_argument("--weight-min", type=float, default=None)
    ap.add_argument("--weight-max", type=float, default=None)
    ap.add_argument("--target-ripple-db", type=float, default=DEF_RIPPLE_DB)
    ap.add_argument("--target-stop-db", type=float, default=DEF_STOP_DB)
    ap.add_argument("--grid-density", type=int, default=32)
    ap.add_argument("--maxiter", type=int, default=100)
    ap.add_argument("--conservative-ripple-db", type=float, default=0.01)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    defaults = {"realtime": (900, 1600), "quality": (1900, 2800)}
    t_min = args.t_min or defaults[args.profile][0]
    t_max = args.t_max or defaults[args.profile][1]

    # weight grid around the theory value ws0 = delta_p / delta_s
    ws0 = delta_p(args.target_ripple_db) / delta_s(args.target_stop_db)
    w_lo = args.weight_min or ws0 / 8.0
    w_hi = args.weight_max or ws0 * 8.0
    weights = list(np.logspace(np.log10(w_lo), np.log10(w_hi),
                               args.weight_points))

    os.makedirs(args.out, exist_ok=True)
    print(f"[{args.profile}] B={PASSBAND_MHZ[args.profile]} MHz  "
          f"T in [{t_min},{t_max}]  ws0={ws0:.2f}  "
          f"weights={['%.2f' % w for w in weights]}")
    print(f"[{args.profile}] design target {args.target_ripple_db} dB / "
          f"{args.target_stop_db} dB; acceptance {SPEC_RIPPLE_DB} dB / "
          f"{SPEC_STOP_DB} dB")

    t_start = time.time()
    log, res = search_profile(args.profile, t_min, t_max, weights,
                              args.target_ripple_db, args.target_stop_db,
                              args.grid_density, args.maxiter)

    # full search log
    logp = os.path.join(args.out, f"search_{args.profile}.csv")
    fields = ["T", "H", "Hpad", "ws", "status", "ripple_db", "rel_stop_db",
              "rel_stop_vs_max_db", "dc_err", "design_s", "detail"]
    with open(logp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in log:
            w.writerow(r)
    print(f"  wrote {logp} ({len(log)} design attempts, "
          f"{time.time() - t_start:.1f} s)")

    if res is None:
        print(f"[{args.profile}] no candidate met the design target in the "
              f"searched range -> keep Kaiser")
        with open(os.path.join(args.out, f"design_{args.profile}.json"), "w") as f:
            json.dump({"profile": args.profile, "result": "no_candidate",
                       "t_min": t_min, "t_max": t_max,
                       "weights": weights}, f, indent=2)
        return 2

    pb_hz = PASSBAND_MHZ[args.profile] * 1e6
    sb_hz = R_IN - pb_hz
    produced = {}
    for hp in sorted(res["classes"]):
        c = res["classes"][hp]
        tag = f"hpad{hp}"
        base, meta = write_candidate(
            args.out, args.profile, tag, c["h32"],
            {"Hpad": hp, "H": hpad_of(c["T"])[1], "ws": c["ws"],
             "ripple_db": c["rec"]["ripple_db"],
             "rel_stop_db": c["rec"]["rel_stop_db"],
             "rel_stop_vs_max_db": c["rec"]["rel_stop_vs_max_db"],
             "dc_err": c["rec"]["dc_err"],
             "design_target_ripple_db": args.target_ripple_db,
             "design_target_stop_db": args.target_stop_db,
             "group_delay_out_samples": (c["T"] - 1) / (2.0 * M)})
        produced[tag] = meta
        print(f"  candidate {tag}: T={meta['taps']} sha256={meta['sha256'][:16]}")
        plot_candidate(args.out, args.profile, tag, c["h32"], pb_hz, sb_hz, meta)

    # --- conservative candidate ------------------------------------------
    # The margin-optimal designs above deliberately spend passband ripple to
    # buy stopband.  If the aggressive candidate turns out to hurt UWB quality
    # (task book §5), the fallback is a design that keeps ripple at or below
    # the Kaiser level while still meeting the frozen spec.  For each class,
    # scan the weight finely and keep the best stopband among the designs
    # whose ripple stays under the conservative bound.
    cons = None
    cons_scan = []
    for hp in sorted(res["classes"]):
        c = res["classes"][hp]
        T_cls = c["T"]
        best_here = None
        for ws in np.logspace(np.log10(ws0 / 8.0), np.log10(ws0 * 8.0), 25):
            h32, rec = evaluate(T_cls, pb_hz, sb_hz, float(ws),
                                args.grid_density, args.maxiter, log)
            if rec.get("status") != "ok":
                continue
            cons_scan.append(dict(rec))
            if rec["ripple_db"] > args.conservative_ripple_db:
                continue
            if not meets(rec, SPEC_RIPPLE_DB, SPEC_STOP_DB):
                continue
            if best_here is None or rec["rel_stop_db"] < best_here[1]["rel_stop_db"]:
                best_here = (hp, rec, h32, float(ws))
        if best_here is not None and cons is None:
            cons = best_here
    if cons is not None:
        hp, c_rec, c_h32, c_ws = cons
        c = {"rec": c_rec, "ws": c_ws, "h32": c_h32, "T": c_rec["T"]}
        base, meta = write_candidate(
            args.out, args.profile, "conservative", c["h32"],
            {"Hpad": hp, "H": hpad_of(c["T"])[1], "ws": c["ws"],
             "ripple_db": c["rec"]["ripple_db"],
             "rel_stop_db": c["rec"]["rel_stop_db"],
             "rel_stop_vs_max_db": c["rec"]["rel_stop_vs_max_db"],
             "dc_err": c["rec"]["dc_err"],
             "design_target_ripple_db": args.target_ripple_db,
             "design_target_stop_db": args.target_stop_db,
             "group_delay_out_samples": (c["T"] - 1) / (2.0 * M)})
        produced["conservative"] = meta
        print(f"  candidate conservative: T={meta['taps']} "
              f"ripple={meta['ripple_db']:.4f} dB")

    summary = {
        "profile": args.profile,
        "passband_mhz": PASSBAND_MHZ[args.profile],
        "stopband_start_mhz": sb_hz / 1e6,
        "design_rate_hz": FV,
        "spec": {"ripple_db": SPEC_RIPPLE_DB, "rel_stop_db": SPEC_STOP_DB,
                 "dc_target": DC_TARGET, "dc_tol": DC_TOL},
        "design_target": {"ripple_db": args.target_ripple_db,
                          "rel_stop_db": args.target_stop_db},
        "search": {"t_min": t_min, "t_max": t_max,
                   "weights": [float(w) for w in weights],
                   "grid_density": args.grid_density,
                   "maxiter": args.maxiter,
                   "attempts": len(log),
                   "shortest_passing_T": res["shortest"]},
        "conservative_ripple_db_target": args.conservative_ripple_db,
        "conservative_scan": cons_scan,
        "candidates": produced,
        "scipy": __import__("scipy").__version__,
        "numpy": np.__version__,
        "elapsed_s": round(time.time() - t_start, 1),
    }
    with open(os.path.join(args.out, f"design_{args.profile}.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(f"  wrote design_{args.profile}.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
