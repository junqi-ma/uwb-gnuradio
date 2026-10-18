#!/usr/bin/env python3
"""S2-A QA entry point (runs the whole test suite in one command).

Runs, in order:

1. `tools/sic/accum_metrics.py --self-test`   (fixed-operator math, qa1-qa6, 16 checks)
2. `tools/sic/verify_s2a.py  --self-test`     (independent verifier primitives)
3. `gr-uwb/apps/uwb_sim_sic_accum.py --self-test` (app/contract checks)
4. a pure-NumPy end-to-end operator check on a synthetic window with a known
   two-path profile (strong path at offset +2, weak path delay 37 -> +39), which
   exercises the real frozen constants and the ROI/peak arithmetic;
5. optionally, when `--run <dir>` is given, the independent verifier plus its
   reject-QA on that real run directory.

Exit code == number of failed checks.

    python3 testdata/sic_accum/qa_s2a.py
    python3 testdata/sic_accum/qa_s2a.py --run gr-uwb/build-sic-accum/results/s2a/smoke
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
sys.path.insert(0, os.path.join(REPO, "tools", "sic"))
sys.path.insert(0, os.path.join(REPO, "gr-uwb", "apps"))

FAILS = 0

#: The public HRP preamble code 9, recovered (and independently re-verified in
#: every run) from the simulator's own modulator output.  ``+``/``.``/``-`` are
#: the ternary chips +1 / 0 / -1; 64 non-zero chips, energy 64.  Kept here so the
#: pure-NumPy QA below exercises the REAL code's autocorrelation sidelobes.
CODE9 = ("+..+...-.--..--+.+.+..-+-++.+....++-...+..-..--.-+.+.--.-+++.++..."
         "+-.+..-.++-.+++..-+..+.+.-.++-+--+......+.....-+....-.-...--+")


def code9_chips():
    return np.array([1 if c == "+" else (-1 if c == "-" else 0)
                     for c in CODE9], dtype=np.int8)


def check(name, cond, detail=""):
    global FAILS
    print("%-62s %s %s" % (name, "PASS" if cond else "FAIL", detail))
    if not cond:
        FAILS += 1


def run_script(name, argv, expect_zero=True):
    global FAILS
    p = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    okk = (p.returncode == 0) if expect_zero else True
    tail = (p.stdout.strip().splitlines() or [""])[-1]
    check(name, okk, "rc=%d | %s" % (p.returncode, tail[:80]))
    return p


def operator_end_to_end(uwb_free=False):
    """Synthetic two-path window checked against the frozen operator math."""
    import accum_metrics as AM

    sps = AM.SPS
    reps = AM.SYNC_REPS
    origin = AM.PRE_GUARD_REF
    chips = code9_chips()
    assert chips.size == AM.PREAMBLE_CODE_LEN
    assert int(np.count_nonzero(chips)) == 64
    assert int(np.sum(chips.astype(np.int64) ** 2)) == 64
    code = AM.sampled_code_from_chips(chips)
    energy = AM.code_energy(code)
    n = origin + reps * sps + AM.TAP_HI + 64

    # Public-causal pulse whose centre tap is at +2 (like the real modulator's
    # shaping FIR).  A path at delay d therefore peaks at offset d+2: the strong
    # path at +2 and the weak (delay 37, gain 0.25) path at +39.
    pulse = np.array([0.1, 0.3, 0.6, 0.3, 0.1], dtype=np.float64)
    train = np.zeros(reps * sps + 8, dtype=np.float64)
    for j in np.flatnonzero(chips):
        train[int(j) * 8] = float(chips[j])
    shaped = np.convolve(train, pulse)
    iq = np.zeros(n, dtype=np.complex128)
    iq[origin:origin + shaped.size] += shaped
    iq[origin + 37:origin + 37 + shaped.size] += 0.25 * shaped

    op = AM.Operator(origin=origin, code=code, energy=energy)
    H = AM.fixed_operator_cir(np.asarray(iq, dtype=np.complex64), op)
    hc = AM.accumulate(H, AM.rep_indices(True, reps))
    dpeak = int(np.argmax(np.abs(hc[AM.DIRECT_LO - op.tap_lo:
                                    AM.DIRECT_HI - op.tap_lo]))) + AM.DIRECT_LO
    check("operator: strong path peaks on the frozen direct offset",
          dpeak == 2, "peak=%d" % dpeak)
    tlo, thi = AM.TARGET_LO - op.tap_lo, AM.TARGET_HI - op.tap_lo
    peak = int(np.argmax(np.abs(hc[tlo:thi])) + AM.TARGET_LO)
    check("operator: weak-target peak lands on the frozen target tap",
          peak == AM.TARGET_OFFSET, "peak=%d frozen=%d"
          % (peak, AM.TARGET_OFFSET))
    ratio = float(abs(hc[AM.TARGET_OFFSET - op.tap_lo])
                  / abs(hc[2 - op.tap_lo]))
    check("operator: weak/strong amplitude ratio == 0.25",
          abs(ratio - 0.25) < 0.02, "%.4f" % ratio)
    rt = AM.residual_table(H, H, H, op)
    check("operator: clean==raw==sic -> C_CIR N/A, no fabricated value",
          rt["full"]["64"]["c_cir_flag"]["full"] == "na")
    check("operator: perfect subtraction -> zero residual energy",
          rt["full"]["64"]["e_sic"]["full"] == 0.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", action="append", default=None,
                    help="real run directory for the verifier + reject-QA")
    ap.add_argument("--budget", action="append", default=None,
                    help="A-R03 error-budget run for the budget verifier + "
                         "reject-QA")
    ap.add_argument("--skip-gr", action="store_true",
                    help="skip the GNU Radio app self-test (no build tree)")
    args = ap.parse_args(argv)

    run_script("accum_metrics --self-test",
               [sys.executable, "tools/sic/accum_metrics.py", "--self-test"])
    run_script("verify_s2a --self-test",
               [sys.executable, "tools/sic/verify_s2a.py", "--self-test"])
    run_script("residual_budget --self-test",
               [sys.executable, "tools/sic/residual_budget.py", "--self-test"])
    run_script("equal_condition_compare --self-test",
               [sys.executable, "tools/sic/equal_condition_compare.py",
                "--self-test"])
    if not args.skip_gr:
        run_script("uwb_sim_sic_accum --self-test",
                   [sys.executable, "gr-uwb/apps/uwb_sim_sic_accum.py",
                    "--self-test"])
        run_script("uwb_sim_sic_error_budget --self-test",
                   [sys.executable, "gr-uwb/apps/uwb_sim_sic_error_budget.py",
                    "--self-test"])
    operator_end_to_end()
    for run in (args.run or []):
        run_script("verify_s2a --run %s" % os.path.basename(run.rstrip("/")),
                   [sys.executable, "tools/sic/verify_s2a.py", "--run", run])
        run_script("verify_s2a --reject-qa %s" % os.path.basename(run.rstrip("/")),
                   [sys.executable, "tools/sic/verify_s2a.py", "--run", run,
                    "--reject-qa"])
    for bdir in (args.budget or []):
        run_script("equal_condition_compare --verify-baseline %s"
                   % os.path.basename(bdir.rstrip("/")),
                   [sys.executable, "tools/sic/equal_condition_compare.py",
                    "--budget", bdir, "--verify-baseline"])
        run_script("verify_s2a --budget %s" % os.path.basename(
            bdir.rstrip("/")),
            [sys.executable, "tools/sic/verify_s2a.py", "--budget", bdir])
        run_script("verify_s2a --reject-qa-budget %s" % os.path.basename(
            bdir.rstrip("/")),
            [sys.executable, "tools/sic/verify_s2a.py", "--budget", bdir,
             "--reject-qa-budget"])
    print("qa_s2a: %d failure(s)" % FAILS)
    return FAILS


if __name__ == "__main__":
    raise SystemExit(main())
