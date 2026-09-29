#!/usr/bin/env python3
"""Independent re-derivation of the M0.1 review findings (R1-R8).

Recomputes each claim from source/CSV rather than trusting a summary, and
prints PASS/FAIL per item.  Run from the repo root:

    python3 tools/twr/verify_m0_1_findings.py

This is a verification aid, not a unit test.  It deliberately re-derives
things the C++/Python suites also assert, because the point is to catch the
case where a suite and its implementation share a wrong assumption.
"""
from __future__ import annotations

import io
import csv
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
INC = os.path.join(REPO, "gr-uwb", "include")

results: list[tuple[str, str, str]] = []


def record(item: str, verdict: str, detail: str) -> None:
    results.append((item, verdict, detail))
    print(f"[{verdict:4}] {item}\n         {detail}")


# ---------------------------------------------------------------------------
# C++ findings: compile a scratch program that calls the public API.
# ---------------------------------------------------------------------------
CPP_PROBE = r"""
#include <gnuradio/uwb/uwb_twr_frame.h>
#include <gnuradio/uwb/uwb_twr_timestamp.h>
#include <gnuradio/uwb/uwb_twr_tof_input.h>
#include <cstdio>
using namespace gr::uwb::twr;
int main() {
    ClockDomain d; ClockDomain::make("x410", 737.28e6, 7, 40, d);
    const uint32_t C = timestamp_required_corrections(TimestampMarker::RmarkerTx);
    Timestamp a, b;
    Timestamp::from_ticks(0, d, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C, a);
    Timestamp::from_ticks(1, d, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C, b);
    auto iv = timestamp_interval(b, a);
    printf("r4_ticks=%lld r4_ns=%lld r4_whole=%d r4_lossless=%d\n",
        (long long)iv.ticks, (long long)iv.duration.nanos(),
        (int)iv.is_whole_ticks, (int)iv.duration_is_lossless_ns);

    ClockDomain w; ClockDomain::make("w8", 1.0, 1, 8, w);
    const uint32_t C2 = timestamp_required_corrections(TimestampMarker::RmarkerTx);
    Timestamp x, y;
    Timestamp::from_ticks(0, w, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C2, x);
    Timestamp::from_ticks(128, w, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C2, y);
    bool p1 = false, p2 = false;
    bool ok1 = timestamp_precedes(x, y, p1);
    bool ok2 = timestamp_precedes(y, x, p2);
    printf("r5_fwd=%d r5_rev=%d\n", (int)(ok1 && p1), (int)(ok2 && p2));

    Timestamp f1, f2;
    Timestamp::from_fractional_ticks(5, 1, 4, w, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C2, f1);
    Timestamp::from_fractional_ticks(5, 3, 4, w, TimestampMarker::RmarkerTx,
        TimestampSource::HardwareMeasured, C2, f2);
    TimestampOrder ord;
    timestamp_compare(f1, f2, ord);
    printf("r5_frac=%d\n", (int)ord);

    // R6: the GENERAL interval tool may legitimately say Ok for two raw
    // sample times -- that is a general "time between samples" utility.
    // What must not happen is those raw times becoming a RANGING input.
    Timestamp q1, q2;
    Timestamp::from_ticks(1000, d, TimestampMarker::UhdRxFirstIqSample,
        TimestampSource::HardwareMeasured, 0u, q1);
    Timestamp::from_ticks(2000, d, TimestampMarker::UhdRxFirstIqSample,
        TimestampSource::HardwareMeasured, 0u, q2);
    auto ri = timestamp_interval(q2, q1);
    printf("r6_general_interval=%d\n", (int)ri.status);

    RangeAdmissionContext ctx;
    auto ad = admit_ranging_interval(q2, q1, ctx);
    printf("r6_admitted=%d r6_reason=%s\n", (int)ad.admitted,
        range_admission_reason_to_string(ad.reason));

    // Forging every correction bit must NOT buy admission.
    Timestamp g1, g2;
    const uint32_t all = timestamp_required_corrections(TimestampMarker::AntennaPlane);
    Timestamp::from_ticks(1000, d, TimestampMarker::UhdRxFirstIqSample,
        TimestampSource::HardwareMeasured, all, g1);
    Timestamp::from_ticks(2000, d, TimestampMarker::UhdRxFirstIqSample,
        TimestampSource::HardwareMeasured, all, g2);
    auto af = admit_ranging_interval(g2, g1, ctx);
    printf("r6_forged_admitted=%d r6_forged_reason=%s\n", (int)af.admitted,
        range_admission_reason_to_string(af.reason));

    // An uncorrected default quality context must not default to "fine".
    printf("r6_default_quality=%s\n",
        first_path_decision_to_string(FirstPathQuality{}.decision));

    // R3: the frame authority is 14 B, and a 7 B claim must be named.
    FrameProfile p; FrameProfileGeometry g;
    bool got = frame_geometry_for(FrameProfileId::TwrV1, g);
    FrameGeometryClaim claim;
    claim.mac_header_bytes = 7; claim.timestamp_bytes = 5; claim.phr_bytes = 2;
    auto chk = frame_geometry_check(claim, g, p);
    printf("r3_got=%d r3_header=%zu r3_phr=%zu r3_ok=%d r3_mismatch=%zu\n",
        (int)got, g.mac_header_bytes(), g.phr_bytes(), (int)chk.ok(),
        chk.fields.count);
    for (size_t i = 0; i < chk.fields.count; ++i) {
        const GeometryMismatch* m = chk.fields.at(i);
        printf("r3_field=%s expected=%zu actual=%zu\n",
            geometry_field_to_string(m->field), m->expected, m->actual);
    }
    uint16_t w16 = 0; std::string err;
    bool fits = session_id_to_wire(0x1234, w16, err);
    bool over  = session_id_to_wire(0x10000, w16, err);
    printf("r3_sess_fits=%d r3_sess_over=%d\n", (int)fits, (int)over);
    return 0;
}
"""


def probe_cpp() -> dict[str, str]:
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, "probe.cpp")
        exe = os.path.join(td, "probe")
        with open(src, "w", encoding="utf-8") as fh:
            fh.write(CPP_PROBE)
        cxx = subprocess.run(
            ["/usr/bin/c++", "-std=c++17", "-O1", f"-I{INC}", src, "-o", exe],
            capture_output=True, text=True)
        if cxx.returncode != 0:
            return {"__error__": cxx.stderr[-1500:]}
        run = subprocess.run([exe], capture_output=True, text=True)
        # Every token is a `key=value` pair; a line may carry several.
        out: dict[str, str] = {}
        for tok in run.stdout.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                out[k] = v
        return out


# ---------------------------------------------------------------------------
# Python finding: snapshot aliasing (R1)
# ---------------------------------------------------------------------------
def probe_python() -> dict[str, str]:
    code = r"""
import runpy, sys
m = runpy.run_path("gr-uwb/apps/test_twr_config.py")
T = m["T"]; c = m["_minimal"]()
e = T.effective_config(c); h = e.config_hash
c.phy.channel = 9
print("r1_effective_channel", e.effective.phy.channel)
print("r1_hash_stable", int(h == e.config_hash))
print("r1_revalidate_ok", int(T.validate(e.effective).ok()))
print("r1_distinct", int(e.effective is not c and e.requested is not e.effective))
bad = [x for x in ("numpy", "gnuradio", "uhd", "pmt") if x in sys.modules]
print("r1_foreign_deps", len(bad))
"""
    run = subprocess.run([sys.executable, "-c", code], cwd=REPO,
                         capture_output=True, text=True)
    out: dict[str, str] = {}
    for line in run.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2:
            out[parts[0]] = parts[1]
    if run.returncode != 0:
        out["__error__"] = run.stderr[-800:]
    return out


# ---------------------------------------------------------------------------
# CSV evidence scope (R7)
# ---------------------------------------------------------------------------
def probe_csv() -> dict[str, str]:
    path = os.path.join(REPO, "testdata", "twr", "phy_matrix_737280000.csv")
    if not os.path.exists(path):
        return {"__error__": "csv missing"}
    lines = open(path, encoding="utf-8").read().splitlines()
    start = next(i for i, l in enumerate(lines) if l.startswith("native_rate_hz"))
    rows = list(csv.DictReader(io.StringIO("\n".join(lines[start:]))))
    paths = {r["path"].split("_998p4")[0] for r in rows}
    return {
        "rows": str(len(rows)),
        "path_prefixes": ",".join(sorted(paths)),
        "not_run": str(len([r for r in rows if r["result"].upper() == "NOT_RUN"])),
        "supported": str(len([r for r in rows if r["whitelist"] == "supported"])),
    }


def main() -> int:
    print("=" * 72)
    print("M0.1 review findings — independent re-derivation")
    print("=" * 72)

    c = probe_cpp()
    if "__error__" in c:
        print("C++ probe failed to build:\n", c["__error__"])
        return 2
    p = probe_python()
    v = probe_csv()

    # R4: lossless flag must be false where truncation occurred.
    ok = c.get("r4_whole") == "1" and c.get("r4_lossless") == "0"
    record("R4 nanosecond projection is not silently lossless",
           "PASS" if ok else "FAIL",
           f"1 tick @737.28MHz -> ns={c.get('r4_ns')} (true 1.356336806), "
           f"is_whole_ticks={c.get('r4_whole')}, lossless={c.get('r4_lossless')}")

    # R5: neither direction may order an exactly-half-period pair.
    ok = c.get("r5_fwd") == "0" and c.get("r5_rev") == "0"
    record("R5 exactly-half-period is ambiguous in both directions",
           "PASS" if ok else "FAIL",
           f"precedes(0,128)={c.get('r5_fwd')} precedes(128,0)={c.get('r5_rev')} "
           f"(M0 returned true for both)")

    # R5b: fractional ordering must resolve.
    ok = c.get("r5_frac") not in (None, "", "0")
    record("R5 sub-tick fractions are ordered", "PASS" if ok else "FAIL",
           f"compare(0.25, 0.75) -> {c.get('r5_frac')} (M0 could not order)")

    # R6: the general interval tool MAY say Ok; the ranging gate must refuse,
    # must refuse again when the correction bits are forged, and must not
    # default an unrecorded first-path quality to "passed".
    ok = (c.get("r6_admitted") == "0"
          and c.get("r6_reason") == "not_range_capable_marker"
          and c.get("r6_forged_admitted") == "0"
          and c.get("r6_default_quality") == "not_recorded")
    record("R6 raw UhdRxFirstIqSample cannot become a ranging input",
           "PASS" if ok else "FAIL",
           f"general interval status={c.get('r6_general_interval')} "
           f"(Ok is correct for a general utility); "
           f"ranging gate admitted={c.get('r6_admitted')} "
           f"reason={c.get('r6_reason')}; "
           f"with forged correction bits admitted={c.get('r6_forged_admitted')}; "
           f"default quality={c.get('r6_default_quality')}")

    # R3: the authority must be 14 B and a 7 B claim must be named.
    ok = (c.get("r3_header") == "14" and c.get("r3_ok") == "0"
          and c.get("r3_mismatch", "0") != "0"
          and c.get("r3_field") == "mac_header_bytes")
    record("R3 frame authority is 14 B and names a 7 B claim",
           "PASS" if ok else "FAIL",
           f"header={c.get('r3_header')} phr={c.get('r3_phr')} "
           f"mismatches={c.get('r3_mismatch')} field={c.get('r3_field')}")

    # R3: an out-of-range session id must be refused, not truncated.
    ok = c.get("r3_sess_fits") == "1" and c.get("r3_sess_over") == "0"
    record("R3 session id narrowing refuses out-of-range", "PASS" if ok else "FAIL",
           f"0x1234 accepted={c.get('r3_sess_fits')} "
           f"0x10000 accepted={c.get('r3_sess_over')}")

    # R1: snapshot must not alias the caller's config.
    ok = (p.get("r1_effective_channel") == "5"
          and p.get("r1_revalidate_ok") == "1"
          and p.get("r1_distinct") == "1")
    record("R1 snapshot does not alias the caller's config",
           "PASS" if ok else "FAIL",
           f"after mutating the source, snapshot channel="
           f"{p.get('r1_effective_channel')} (must stay 5), "
           f"revalidate_ok={p.get('r1_revalidate_ok')}, "
           f"distinct={p.get('r1_distinct')}, "
           f"foreign_deps={p.get('r1_foreign_deps')}")

    # R7: the CSV must not claim native evidence it does not have.
    ok = v.get("path_prefixes") == "work_direct"
    record("R7 capability CSV is work-grid evidence only",
           "PASS" if ok else "FAIL",
           f"rows={v.get('rows')} path prefixes={v.get('path_prefixes')} "
           f"not_run={v.get('not_run')} supported={v.get('supported')} "
           f"— a native_rate_hz label is NOT native round-trip evidence")

    print("=" * 72)
    failed = [r for r in results if r[1] == "FAIL"]
    print(f"{len(results) - len(failed)}/{len(results)} checks pass")
    if failed:
        print("FAILED: " + ", ".join(f[0].split()[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
