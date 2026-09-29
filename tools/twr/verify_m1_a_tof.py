#!/usr/bin/env python3
"""Independent re-derivation of the M1-A SS/DS ToF mathematics.

This does NOT trust the C++ and it does NOT trust the oracle's own `expected`
field.  It:

  1. reads testdata/twr/tof_oracle_vectors.json;
  2. recomputes every ToF from the RAW ENDPOINT TICKS using
     fractions.Fraction, applying the counter wrap itself -- an independent
     derivation, in a different language and a different formulation;
  3. checks the oracle's `expected` field agrees with (2);
  4. generates a small C++ probe that runs the REAL `compute_ss_tof` /
     `compute_ds_tof` on the same vectors through the real ranging gate,
     compiles and runs it, and checks the C++ answer against (2).

Step 4 compares C++ against the Python re-derivation, never against another
C++ function.

    python3 tools/twr/verify_m1_a_tof.py

Exit status 0 iff every check passes.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from fractions import Fraction as F

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INC = os.path.join(REPO, "gr-uwb", "include")
ORACLE = os.path.join(REPO, "testdata", "twr", "tof_oracle_vectors.json")

results: list[tuple[str, str, str]] = []


def record(item: str, verdict: str, detail: str) -> None:
    results.append((item, verdict, detail))
    print(f"[{verdict:4}] {item}\n         {detail}")


# ---------------------------------------------------------------------------
# 1-2. Recompute from endpoints, independently
# ---------------------------------------------------------------------------
def endpoint_ticks(e: dict) -> F:
    """(ticks, num, den) -> exact rational ticks."""
    n, d = e["num"], e["den"]
    if d == 0:
        return F(e["ticks"])
    return F(e["ticks"]) + F(n, d)


def interval(iv: dict, bits: int) -> F:
    """Later - earlier, resolved modulo the counter width exactly as the
    timestamp layer does.  The result is a non-negative relative interval."""
    later = endpoint_ticks(iv["later"])
    earlier = endpoint_ticks(iv["earlier"])
    d = later - earlier
    if bits:
        d %= 1 << bits
    return d


def recompute(v: dict) -> F:
    bits = int(v["domain_a"]["timestamp_bits"])
    k = F(v["clock_ratio"]["k"]["num"], v["clock_ratio"]["k"]["den"])
    ra = interval(v["intervals"]["ra"], bits)
    db = interval(v["intervals"]["db"], int(v["domain_b"]["timestamp_bits"]))
    if v["protocol"] == "ss":
        return (ra - k * db) / 2
    rb = interval(v["intervals"]["rb"], int(v["domain_b"]["timestamp_bits"]))
    da = interval(v["intervals"]["da"], bits)
    return (ra * (k * rb) - da * (k * db)) / (ra + k * rb + da + k * db)


# ---------------------------------------------------------------------------
# 4. The C++ probe, generated from the same vectors
# ---------------------------------------------------------------------------
MARKER = {
    "rmarker_rx": "TimestampMarker::RmarkerRx",
    "rmarker_tx": "TimestampMarker::RmarkerTx",
    "antenna_plane": "TimestampMarker::AntennaPlane",
}
SOURCE = {
    "nominal_same_clock": "ClockRatioSource::NominalSameClock",
    "nominal_rate_ratio": "ClockRatioSource::NominalRateRatio",
    "calibrated": "ClockRatioSource::Calibrated",
    "estimated": "ClockRatioSource::Estimated",
}

PROBE_HEAD = r"""
#include <gnuradio/uwb/uwb_twr_math.h>
#include <cstdio>
#include <string>
using namespace gr::uwb::twr;

struct EP {
    long long ticks;
    int num;
    unsigned den;
    TimestampMarker marker;
};

static RangingIntervalAdmission admit_pair(const ClockDomain& d, EP later, EP earlier)
{
    const std::string cal_id = std::string("cal-m1a-verify-") + d.name;
    CalibrationStamp cal;
    cal.id = cal_id;
    cal.calibrated_epoch = d.epoch_id;
    cal.valid_from_ticks = 1000;
    cal.valid_until_ticks = 2000000;
    CalibrationApplication app;
    app.calibration_id = cal_id;
    app.result = CalibrationResult::Applied;
    cal.applications.push_back(app);

    Timestamp lt, et;
    Timestamp::from_fractional_ticks(later.ticks, later.num, later.den, d, later.marker,
                                     TimestampSource::HardwareMeasured, kCorrectionNone, lt);
    Timestamp::from_fractional_ticks(earlier.ticks, earlier.num, earlier.den, d,
                                     earlier.marker, TimestampSource::HardwareMeasured,
                                     kCorrectionNone, et);
    apply_calibration_ticks(lt, cal_id, 0, 0, 0u, timestamp_required_corrections(later.marker));
    apply_calibration_ticks(et, cal_id, 0, 0, 0u,
                            timestamp_required_corrections(earlier.marker));

    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);
    return admit_ranging_interval(lt, et, ctx);
}

int main()
{
"""

PROBE_TAIL = r"""
    return 0;
}
"""


def ep(e: dict, marker: str) -> str:
    return ("EP{%d, %d, %du, %s}" % (e["ticks"], e["num"], e["den"],
                                     MARKER[marker]))


def emit_vector(v: dict) -> str:
    vid = v["id"]
    a, b = v["domain_a"], v["domain_b"]
    k = v["clock_ratio"]["k"]
    iv = v["intervals"]
    out = []
    out.append("    { // %s" % vid)
    out.append('        ClockDomain a, b;')
    out.append('        ClockDomain::make("%s", %.17g, %du, %du, a);'
               % (a["name"], float(a["tick_rate_hz"]), a["epoch"], a["timestamp_bits"]))
    out.append('        ClockDomain::make("%s", %.17g, %du, %du, b);'
               % (b["name"], float(b["tick_rate_hz"]), b["epoch"], b["timestamp_bits"]))
    if v["clock_ratio"]["source"] == "nominal_same_clock":
        out.append("        ClockRatio k;")
        out.append("        ClockRatio::unity_same_clock(a, k);")
    else:
        out.append("        ClockRatio k;")
        out.append("        ClockRatio::make(a, b, %d, %d, %s, k);"
                   % (k["num"], k["den"], SOURCE[v["clock_ratio"]["source"]]))
    out.append("        RangingIntervalAdmission ra = admit_pair(a, %s, %s);"
               % (ep(iv["ra"]["later"], iv["ra"]["later_marker"]),
                  ep(iv["ra"]["earlier"], iv["ra"]["earlier_marker"])))
    out.append("        RangingIntervalAdmission db = admit_pair(b, %s, %s);"
               % (ep(iv["db"]["later"], iv["db"]["later_marker"]),
                  ep(iv["db"]["earlier"], iv["db"]["earlier_marker"])))
    if v["protocol"] == "ss":
        out.append("        TofResult r = compute_ss_tof(*ra.value, *db.value, k);")
    else:
        out.append("        RangingIntervalAdmission rb = admit_pair(b, %s, %s);"
                   % (ep(iv["rb"]["later"], iv["rb"]["later_marker"]),
                      ep(iv["rb"]["earlier"], iv["rb"]["earlier_marker"])))
        out.append("        RangingIntervalAdmission da = admit_pair(a, %s, %s);"
                   % (ep(iv["da"]["later"], iv["da"]["later_marker"]),
                      ep(iv["da"]["earlier"], iv["da"]["earlier_marker"])))
        out.append("        TofResult r = compute_ds_tof(*ra.value, *rb.value, *da.value,"
                   " *db.value, k);")
    out.append('        std::printf("' + vid + '|%s|%lld|%lld\\n",'
               " tof_status_to_string(r.status),")
    out.append("                    (long long)r.tof.num, (long long)r.tof.den);")
    out.append("    }")
    return "\n".join(out)


def build_probe(vectors: list[dict]) -> str:
    body = "\n".join(emit_vector(v) for v in vectors)
    return PROBE_HEAD + body + PROBE_TAIL


def main() -> int:
    with open(ORACLE, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    vectors = doc["vectors"]

    # ---- 1-3. independent recomputation vs the oracle's own claim --------
    mismatched = []
    for v in vectors:
        mine = recompute(v)
        exp = F(v["expected"]["tof"]["num"], v["expected"]["tof"]["den"])
        if mine != exp:
            mismatched.append((v["id"], str(mine), str(exp)))
    record("M1-A oracle `expected` agrees with an independent endpoint re-derivation",
           "PASS" if not mismatched else "FAIL",
           f"{len(vectors)} vectors recomputed from raw endpoint ticks with "
           f"fractions.Fraction ({'all agree' if not mismatched else mismatched})")

    # ---- 4. the C++ answer vs the same independent recomputation ---------
    src = build_probe(vectors)
    build_err = ""
    probe: dict[str, tuple[str, F]] = {}
    with tempfile.TemporaryDirectory() as td:
        cpp = os.path.join(td, "probe.cc")
        exe = os.path.join(td, "probe")
        with open(cpp, "w", encoding="utf-8") as fh:
            fh.write(src)
        cxx = os.environ.get("CXX", "/usr/bin/c++")
        comp = subprocess.run([cxx, "-std=c++17", "-O1", f"-I{INC}", cpp, "-o", exe],
                              capture_output=True, text=True)
        if comp.returncode != 0:
            build_err = comp.stderr[-1500:]
        else:
            run = subprocess.run([exe], capture_output=True, text=True)
            for line in run.stdout.splitlines():
                parts = line.split("|")
                if len(parts) == 4:
                    probe[parts[0]] = (parts[1], F(int(parts[2]), int(parts[3])))

    if build_err:
        record("M1-A C++ probe compiles and runs", "FAIL", build_err)
        bad = [("__build__", "", "")]
    else:
        bad = []
        for v in vectors:
            mine = recompute(v)
            if v["id"] not in probe:
                bad.append((v["id"], "<no output>", str(mine)))
                continue
            status, val = probe[v["id"]]
            expect_status = v["expected"]["status"]
            # A negative vector still returns its signed value.
            if status != expect_status:
                bad.append((v["id"], status, expect_status))
            elif val != mine:
                bad.append((v["id"], str(val), str(mine)))
        record("M1-A C++ compute_* agrees with the Python fractions re-derivation",
               "PASS" if not bad else "FAIL",
               f"{len(vectors)} vectors, C++ vs Fraction over the same endpoint "
               f"ticks ({'all agree' if not bad else bad[:4]})")

        # The probe must also prove the common-clock example the install
        # consumer uses, so a regression there is caught here too.
        ok_install = bool(probe) and any(
            v["id"] == "ss1_common_clock" and probe.get(v["id"], ("", F(0)))[1] == 500
            for v in vectors)
        record("M1-A install-consumer example (RA=2000, DB=1000 -> 500 A ticks)",
               "PASS" if ok_install else "FAIL",
               "ss1_common_clock must be exactly 500/1 in A ticks")

        record("M1-A every oracle vector terminated with a known status",
               "PASS" if len(probe) == len(vectors) else "FAIL",
               f"{len(probe)}/{len(vectors)} vectors produced a status")

    print("=" * 72)
    failed = [r for r in results if r[1] == "FAIL"]
    print(f"{len(results) - len(failed)}/{len(results)} checks pass")
    if failed:
        print("FAILED: " + ", ".join(f[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
