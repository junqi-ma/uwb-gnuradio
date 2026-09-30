#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate the M1-A ToF oracle vectors from an explicit PHYSICAL model.

WHY THIS FILE EXISTS
--------------------
REQ-QA-01 requires an oracle that is independent of the implementation: the
golden values must come from a physical model, never from reading the C++
output back.  The oracle of record for M1-A is `generate_tof_oracle.m`
(MATLAB).  When M1-A was first delivered there was no runnable MATLAB on the
development machine (the `/usr/local/MATLAB/R2024a` tree had no launcher and no
main binary) and no Octave, so the checked-in vectors were produced by THIS
file instead, using `fractions.Fraction` for exact rational arithmetic.

That substitution was recorded, not hidden, and it has since been RESOLVED:
`generate_tof_oracle.m` was run on MATLAB R2025b and its output agrees with
this reference vector for vector.  The checked-in
`tof_oracle_vectors.json` is now the MATLAB output (see
`README_tof_oracle.md`).  This script is kept as the second, independent
check on the same model -- `tools/twr/verify_m1_a_tof.py` and the C++ QA
consume the MATLAB vectors, and re-running this script must reproduce them
exactly.

  * `tof_oracle_vectors.json` carries a provenance block naming the MATLAB
    oracle and stating that it WAS executed;
  * `README_tof_oracle.md` gives the exact command, the MATLAB version and
    the file hashes;
  * every value here is derived from the physical model below, so the C++ is
    checked against physics and not against itself.

Re-running this script overwrites the MATLAB-produced golden with an
equivalent Python-produced one; do not do it unless you mean to.

THE MODEL (must match `generate_tof_oracle.m` statement for statement)
---------------------------------------------------------------------
Two clocks, `fA` and `fB` (Hz).  All physical quantities below are exact
rationals; the true ToF is `tau`, expressed in A ticks as `tau_A`.

  k = fA / fB                     # A ticks per B tick

  RA = t4A - t1A = 2*tau_A + dB_A         (A ticks)
  DA = t5A - t4A = dA_A                   (A ticks)
  DB = t3B - t2B = dB_A / k               (B ticks)
  RB = t6B - t3B = (2*tau_A + dA_A) / k   (B ticks)

  SS: ToF_A = (RA - k*DB) / 2 = (2*tau_A + dB_A - dB_A) / 2 = tau_A
  DS: ToF_A = (RA*kRB - DA*kDB) / (RA + kRB + DA + kDB) = tau_A

Both are exact identities, so the expected value is `tau_A` by construction
and the model can assert it against a full re-evaluation of the formula.

The four intervals must be representable as a DW timestamp fraction
(`ticks + frac_num/frac_den`, `frac_den <= 32767`), which is a real
constraint: a ratio like 1000001/1000000 cannot be expressed.  The vectors
therefore use ratios whose derived intervals stay inside that denominator
bound, and this script refuses to emit a vector that violates it.

OUTPUT
------
`tof_oracle_vectors.json` next to this file.  Run:

    python3 testdata/twr/gen_tof_oracle.py
"""

import json
import os
from fractions import Fraction as F

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "tof_oracle_vectors.json")

# The denominator bound of a DW sub-tick fraction (see
# `kFractionDenominatorMax` in uwb_twr_timestamp.h).
FRAC_DEN_MAX = 32767
# 40-bit wire counter.
WRAP_BITS = 40
WRAP_PERIOD = 1 << WRAP_BITS

# ---------------------------------------------------------------------------
# Domains
# ---------------------------------------------------------------------------
UUS_RATE = 499.2e6 * 128.0  # 63 897 600 000.0 Hz, the DW UUS grid

DOM_A = {"name": "dw1000_snA_ch5_uus", "tick_rate_hz": UUS_RATE, "epoch": 7,
         "timestamp_bits": WRAP_BITS}
DOM_B = {"name": "dw3000_snB_ch5_uus", "tick_rate_hz": UUS_RATE, "epoch": 7,
         "timestamp_bits": WRAP_BITS}
# For the rate-ratio vectors the two nominal rates really are 625/624 apart.
DOM_A_RR = {"name": "dw1000_snA_ch5_uus", "tick_rate_hz": 1.0e9, "epoch": 7,
            "timestamp_bits": WRAP_BITS}
DOM_B_RR = {"name": "dw3000_snB_ch5_uus", "tick_rate_hz": 998.4e6, "epoch": 7,
            "timestamp_bits": WRAP_BITS}
# Same PHYSICAL clock: one name, one rate, one epoch (the single-X410 case).
DOM_A_SAME = DOM_A


# ---------------------------------------------------------------------------
# Exact rational helpers
# ---------------------------------------------------------------------------
def endpoint(x):
    """A non-negative Fraction -> (ticks, num, den) with den <= FRAC_DEN_MAX.

    `x` is already in lowest terms, so `den` is numerator-independent.
    """
    assert x >= 0, x
    n, d = x.numerator, x.denominator
    assert d <= FRAC_DEN_MAX, "fraction %s has denominator %d > %d" % (x, d,
                                                                       FRAC_DEN_MAX)
    ticks, num = divmod(n, d)
    if num == 0:
        return {"ticks": int(ticks), "num": 0, "den": 0}
    return {"ticks": int(ticks), "num": int(num), "den": int(d)}


def ratio(x):
    """A Fraction -> {"num","den"} in lowest terms."""
    assert x.denominator <= 2 ** 63 - 1, x
    return {"num": int(x.numerator), "den": int(x.denominator)}


def _abs_endpoint(x, base, wrap):
    """Absolute (ticks,num,den) of instant `x` on a timeline whose origin is
    `base`, folded into the counter width when `wrap` is set.

    Without `wrap` the instants are the plain physical values.  With `wrap`
    the whole A timeline is translated by `base` (chosen so one interval
    straddles the counter wrap) and folded modulo the period, which is what a
    40-bit counter would actually have recorded.
    """
    y = x + base
    if wrap:
        y = y % WRAP_PERIOD
    return endpoint(y)


# ---------------------------------------------------------------------------
# Vector construction
# ---------------------------------------------------------------------------
def make_vector(vid, proto, k, dom_a, dom_b, tau_a, db_a, da_a,
                source, wrap_a=False, expect_status="ok", note="",
                expect_tof=None):
    """Build one vector from the physical model and re-derive its expected value.

    `tau_a` is the true ToF in A ticks.  It is NEGATIVE only for the two
    deliberate "the measurement is internally inconsistent" vectors, where the
    intervals themselves stay non-negative and the formula's signed result is
    what the test is about.

    `expect_tof` is an INDEPENDENT hand calculation, used as a cross-check on
    the model's own re-evaluation; for every physical vector it equals `tau_a`.
    """
    if proto == "ss":
        ra = 2 * tau_a + db_a
        db = db_a / k
        da = None
        rb = None
    else:
        ra = 2 * tau_a + db_a
        da = da_a
        db = db_a / k
        rb = (2 * tau_a + da_a) / k

    # --- the model's own independent re-evaluation of the formulas --------
    if proto == "ss":
        # Everything first into A ticks, exactly as the C++ is required to do.
        db_in_a = k * db
        derived = (ra - db_in_a) / 2
    else:
        rb_in_a = k * rb
        db_in_a = k * db
        num = ra * rb_in_a - da * db_in_a
        den = ra + rb_in_a + da + db_in_a
        derived = num / den

    assert derived == tau_a, ("model self-check failed", vid, derived, tau_a)
    tof = derived
    if expect_tof is not None:
        assert derived == expect_tof, ("hand check disagrees", vid, derived,
                                       expect_tof)

    # --- absolute timestamps on each domain's own timeline ---------------
    # A: t1A = 0, t4A = RA, t5A = RA + DA
    # B: t2B = 0, t3B = DB, t6B = DB + RB
    t1a, t4a, t5a = F(0), ra, (ra + da) if da is not None else None
    t2b, t3b, t6b = F(0), db, (db + rb) if rb is not None else None
    # Translate the A timeline so `ra` straddles the wrap when asked.
    base_a = (WRAP_PERIOD - 100) if wrap_a else F(0)

    intervals = {
        "ra": {"later": _abs_endpoint(t4a, base_a, wrap_a),
               "earlier": _abs_endpoint(t1a, base_a, wrap_a),
               "later_marker": "rmarker_rx", "earlier_marker": "rmarker_tx"},
        "db": {"later": _abs_endpoint(t3b, F(0), False),
               "earlier": _abs_endpoint(t2b, F(0), False),
               "later_marker": "rmarker_tx", "earlier_marker": "rmarker_rx"},
    }
    if proto == "ds":
        intervals["da"] = {"later": _abs_endpoint(t5a, base_a, wrap_a),
                           "earlier": _abs_endpoint(t4a, base_a, wrap_a),
                           "later_marker": "rmarker_tx",
                           "earlier_marker": "rmarker_rx"}
        intervals["rb"] = {"later": _abs_endpoint(t6b, F(0), False),
                           "earlier": _abs_endpoint(t3b, F(0), False),
                           "later_marker": "rmarker_rx",
                           "earlier_marker": "rmarker_tx"}

    return {
        "id": vid,
        "protocol": proto,
        "note": note,
        "clock_ratio": {"source": source, "k": ratio(k)},
        "domain_a": dom_a,
        "domain_b": dom_b,
        "intervals": intervals,
        "expected": {
            "status": expect_status,
            "tof_domain": "a",
            "tof": ratio(tof) if expect_status in ("ok", "negative_tof") else None,
        },
    }


def build():
    vectors = []

    # SS-1: one physical clock, symmetric reply.  Hand check: RA=2000,
    # DB=1000, k=1 -> (2000-1000)/2 = 500.
    vectors.append(make_vector(
        "ss1_common_clock", "ss", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(500), db_a=F(1000), da_a=None,
        source="nominal_same_clock",
        note="hand check: RA=2000, DB=1000, k=1 -> ToF=500"))

    # SS-2: k = 625/624 from two NOMINAL rates (1.0 GHz vs 998.4 MHz).  RA is
    # 2000 A ticks and DB becomes 4992/5 B ticks, so the sub-tick path is used.
    vectors.append(make_vector(
        "ss2_nominal_rate_ratio", "ss", F(625, 624), DOM_A_RR, DOM_B_RR,
        tau_a=F(500), db_a=F(1000), da_a=None,
        source="nominal_rate_ratio",
        note="hand check: k*DB = (625/624)*(4992/5) = 1000 A ticks; (2000-1000)/2=500"))

    # SS-3: the finest representable deviation either side of nominal.
    vectors.append(make_vector(
        "ss3a_a_faster_30ppm", "ss", F(32768, 32767), DOM_A_RR, DOM_B_RR,
        tau_a=F(500), db_a=F(32768), da_a=None,
        source="nominal_rate_ratio",
        note="k>1 (A runs fast): DB=32767 B ticks -> back to 32768 A ticks"))
    vectors.append(make_vector(
        "ss3b_a_slower_30ppm", "ss", F(32767, 32768), DOM_A_RR, DOM_B_RR,
        tau_a=F(500), db_a=F(32767), da_a=None,
        source="nominal_rate_ratio",
        note="k<1 (A runs slow): DB=32768 B ticks -> back to 32767 A ticks"))

    # SS-5f: fractional ToF (tau = 5/2 ticks), whole-tick intervals.
    vectors.append(make_vector(
        "ss5f_fractional_tof", "ss", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(5, 2), db_a=F(1000), da_a=None,
        source="nominal_same_clock",
        note="RA=1005, DB=1000, k=1 -> ToF=5/2 (must not truncate)"))

    # SS-8: the A-domain interval crosses the 40-bit wrap.  The gate resolves
    # the relative interval; the FORMULA must not do its own modular maths.
    vectors.append(make_vector(
        "ss8_wrapped_a_interval", "ss", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(105), db_a=F(1000), da_a=None,
        source="nominal_same_clock", wrap_a=True,
        note="t1A=P-100, t4A=1110 -> RA=1210 across wrap; (1210-1000)/2=105"))

    # SS-7: negative ToF is RETAINED signed, not clamped.  A negative `tau` here
    # is the model's way of saying "these two intervals are inconsistent":
    # RA=1000, DB=2000 -> ToF=-500.
    vectors.append(make_vector(
        "ss7_negative_tof", "ss", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(-500), db_a=F(2000), da_a=None,
        source="nominal_same_clock",
        expect_status="negative_tof", expect_tof=F(-500),
        note="RA=1000, DB=2000 -> ToF=-500, kept signed (never clamped to 0)"))

    # DS-1: symmetric replies.  Must agree with SS-1 on the same physical ToF.
    vectors.append(make_vector(
        "ds1_symmetric", "ds", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(500), db_a=F(1000), da_a=F(1000),
        source="nominal_same_clock",
        note="RA=2000,RB=2000,DA=DB=1000 -> (4e6-1e6)/6000=500, same as SS-1"))

    # DS-2: ASYMMETRIC replies (APS013).  DA != DB is legal.
    vectors.append(make_vector(
        "ds2_asymmetric", "ds", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(500), db_a=F(1000), da_a=F(3000),
        source="nominal_same_clock",
        note="RA=2000,RB=4000,DA=3000,DB=1000 -> (8e6-3e6)/10000=500"))

    # DS-3: tiny ToF, long turnaround, fractional everywhere.
    vectors.append(make_vector(
        "ds3_tiny_tof_long_turnaround", "ds", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(1, 7), db_a=F(3000) + F(2, 7), da_a=F(5000) + F(3, 7),
        source="nominal_same_clock",
        note="tau=1/7; long asymmetric turnaround; no ns truncation anywhere"))

    # DS-7: asymmetric replies AND k != 1, so the unit conversion is load
    # bearing.  Both B intervals land on a fraction with denominator 5.
    vectors.append(make_vector(
        "ds7_asymmetric_rate_ratio", "ds", F(625, 624), DOM_A_RR, DOM_B_RR,
        tau_a=F(500), db_a=F(1000), da_a=F(1000),
        source="nominal_rate_ratio",
        note="DB=4992/5, RB=9984/5 B ticks; converted back -> 500 A ticks"))

    # DS-6: negative ToF retained signed.  RA=RB=100, DA=DB=1000.
    v = make_vector(
        "ds6_negative_tof", "ds", F(1, 1), DOM_A_SAME, DOM_A_SAME,
        tau_a=F(-450), db_a=F(1000), da_a=F(1000),
        source="nominal_same_clock",
        expect_status="negative_tof",
        expect_tof=F(-450),
        note="RA=RB=100, DA=DB=1000 -> N=-990000, D=2200 -> -450")
    vectors.append(v)

    return vectors


def main():
    vectors = build()
    doc = {
        "schema": "uwb-twr-tof-oracle/1",
        "provenance": {
            "generator": "testdata/twr/gen_tof_oracle.py",
            "generator_language": "Python 3 fractions.Fraction (exact)",
            "matlab_script": "testdata/twr/generate_tof_oracle.m",
            "matlab_executed": False,
            "matlab_executed_note": (
                "This file is the Python reference of the physical model, not "
                "the oracle of record.  The oracle of record is "
                "generate_tof_oracle.m, which HAS been run (MATLAB R2025b) and "
                "agrees with these vectors vector for vector; the checked-in "
                "tof_oracle_vectors.json is that MATLAB output.  Regenerating "
                "with this script would replace it with an equivalent "
                "Python-produced file and drop the MATLAB provenance."),
            "units": "ticks",
            "tof_domain": "A (clock ratio k = fA/fB converts B ticks to A ticks)",
            "formulas": {
                "ss": "(RA_A - k*DB_B) / 2",
                "ds": "(RA_A*k*RB_B - DA_A*k*DB_B) / (RA_A + k*RB_B + DA_A + k*DB_B)",
            },
        },
        "vectors": vectors,
    }
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, sort_keys=False)
        f.write("\n")
    print("wrote %s (%d vectors)" % (OUT, len(vectors)))
    for v in vectors:
        e = v["expected"]
        print("  %-30s %-3s status=%-13s tof=%s/%s" % (
            v["id"], v["protocol"], e["status"],
            e["tof"]["num"] if e["tof"] else "-",
            e["tof"]["den"] if e["tof"] else "-"))


if __name__ == "__main__":
    main()
