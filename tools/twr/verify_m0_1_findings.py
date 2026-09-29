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
#include <gnuradio/uwb/uwb_twr_config.h>
#include <gnuradio/uwb/uwb_twr_timestamp.h>
#include <gnuradio/uwb/uwb_twr_tof_input.h>
#include <cstdio>
#include <string>
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
    const bool ord_ok = timestamp_compare(f1, f2, ord);
    printf("r5_frac=%d r5_frac_ok=%d\n", (int)ord, (int)ord_ok);

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

    // -----------------------------------------------------------------------
    // N01: an out-of-range integer must be REFUSED at the reader, not masked.
    // The default config serialises; splice in a value one modulus past a
    // legal one, which a mask would accept by landing back in range.
    // -----------------------------------------------------------------------
    {
        TwrConfig c; std::string js;
        if (to_json_string(c, js, err)) {
            auto splice = [](std::string s, const std::string& key,
                             const std::string& from, const std::string& to) {
                const std::string needle = "\"" + key + "\": " + from;
                const size_t at = s.find(needle);
                if (at == std::string::npos) return std::string();
                return s.substr(0, at) + "\"" + key + "\": " + to +
                       s.substr(at + needle.size());
            };
            struct N1 { const char* key; const char* from; const char* to;
                        const char* field; };
            const N1 rows[] = {
                {"channel", "5", "261", "phy.channel"},
                {"session_id", "0", "4294967297", "session.session_id"},
                {"local_address", "0", "65537", "session.local_address"},
                {"local_address", "0", "-1", "session.local_address"},
            };
            int refused = 0, total = 0;
            for (const N1& r : rows) {
                const std::string doc = splice(js, r.key, r.from, r.to);
                if (doc.empty()) continue;
                ++total;
                TwrConfig back;
                const ValidationReport rep = from_json_string(doc, back);
                if (!rep.ok() &&
                    rep.find(r.field, ConfigReason::OutOfRange) != nullptr)
                    ++refused;
            }
            printf("n01_refused=%d n01_total=%d\n", refused, total);
        } else {
            printf("n01_refused=-1 n01_total=-1\n");
        }
    }

    // -----------------------------------------------------------------------
    // N02: a scheduled TX is admitted WITH send evidence and refused without
    // it; the admitted value is never relabelled as a hardware measurement.
    // -----------------------------------------------------------------------
    {
        ClockDomain d2;
        ClockDomain::make("n02", 63.8976e9, 7, 40, d2);
        Timestamp tx;
        const uint32_t ctx_mask = timestamp_required_corrections(TimestampMarker::RmarkerTx);
        Timestamp::from_ticks(5000, d2, TimestampMarker::RmarkerTx,
                              TimestampSource::ScheduledCalibrated, ctx_mask, tx);
        tx.calibration_id = "cal-x";

        CalibrationStamp cal;
        cal.id = "cal-x";
        cal.calibrated_epoch = d2.epoch_id;
        cal.valid_from_ticks = 0;
        cal.valid_until_ticks = 1000000;
        CalibrationApplication app;
        app.calibration_id = "cal-x";
        app.result = CalibrationResult::Applied;
        cal.applications.push_back(app);

        RangeAdmissionContext c2;
        c2.calibration = &cal;
        c2.reference_ticks = 4000;
        c2.reference_ticks_recorded = true;

        const RangeAdmission without = admit_range_capable_time(tx, c2);
        TxSendEvidence ev;
        ev.command_time_recorded = true;
        ev.quantised_instant_recorded = true;
        ev.marker_offset_recorded = true;
        ev.calibrated_air_time_recorded = true;
        ev.send_accepted = true;
        ev.outcome = TxOutcome::Completed;
        c2.tx_evidence = &ev;
        const RangeAdmission with = admit_range_capable_time(tx, c2);

        // send_accepted alone (outcome Unknown) must not be enough.
        TxSendEvidence only_send = ev;
        only_send.outcome = TxOutcome::Unknown;
        c2.tx_evidence = &only_send;
        const RangeAdmission send_only = admit_range_capable_time(tx, c2);

        printf("n02_no_evidence=%d n02_send_only=%d n02_with_evidence=%d "
               "n02_source_is_scheduled=%d\n",
               (int)without.admitted, (int)send_only.admitted,
               (int)with.admitted,
               (int)(with.value.has_value() &&
                     with.value->source() == TimestampSource::ScheduledCalibrated));
    }

    // -----------------------------------------------------------------------
    // N03: the projected ns and the lossless flag come from ONE exact rational,
    // so an exact value is never reported one nanosecond low.
    // -----------------------------------------------------------------------
    {
        struct N3 { double rate; int64_t ticks; int64_t exact_ns; };
        const N3 rows[] = {
            {737.28e6, 361728, 490625}, {491.52e6, 241152, 490625},
            {998.4e6, 8112, 8125}, {63.8976e9, 519168, 8125}, {1.0e9, 15, 15},
        };
        int exact = 0, lossless = 0, total = 0;
        for (const N3& r : rows) {
            ClockDomain dd;
            ClockDomain::make("n03", r.rate, 1, 0, dd);
            Timestamp later, earlier;
            const uint32_t rx = timestamp_required_corrections(TimestampMarker::RmarkerRx);
            const uint32_t txm = timestamp_required_corrections(TimestampMarker::RmarkerTx);
            Timestamp::from_ticks(r.ticks, dd, TimestampMarker::RmarkerRx,
                                  TimestampSource::HardwareMeasured, rx, later);
            Timestamp::from_ticks(0, dd, TimestampMarker::RmarkerTx,
                                  TimestampSource::HardwareMeasured, txm, earlier);
            const TimeInterval iv = timestamp_interval(later, earlier);
            ++total;
            if (iv.duration.nanos() == r.exact_ns) ++exact;
            if (iv.duration_is_lossless_ns) ++lossless;
        }
        printf("n03_exact_ns=%d n03_lossless=%d n03_total=%d\n", exact, lossless, total);
    }

    // -----------------------------------------------------------------------
    // N04: the half-period bound is re-checked AFTER the sub-tick fraction is
    // merged, at both the interval and the admission layer.
    // -----------------------------------------------------------------------
    {
        ClockDomain dd;
        ClockDomain::make("n04", 737.28e6, 1, 12, dd); // P=4096, P/2=2048
        Timestamp later, earlier;
        Timestamp::from_fractional_ticks(2048, 3, 4, dd, TimestampMarker::RmarkerRx,
                                         TimestampSource::HardwareMeasured,
                                         timestamp_required_corrections(TimestampMarker::RmarkerRx),
                                         later);
        Timestamp::from_fractional_ticks(0, 1, 4, dd, TimestampMarker::RmarkerTx,
                                         TimestampSource::HardwareMeasured,
                                         timestamp_required_corrections(TimestampMarker::RmarkerTx),
                                         earlier);
        const TimeInterval iv = timestamp_interval(later, earlier);
        printf("n04_interval_status=%d\n", (int)iv.status);
    }

    // -----------------------------------------------------------------------
    // N07: an enum value the enum does not have must be REFUSED, never
    // admitted.  Base-validity independent: the check is that the violation
    // NAMES the field as out-of-domain, which holds whatever else is wrong.
    // -----------------------------------------------------------------------
    {
        struct Site { const char* field; void (*set)(TwrConfig&, int); };
        const Site sites[] = {
            {"rx.agc", [](TwrConfig& x, int v) { x.rx.agc = static_cast<AgcMode>(v); }},
            {"calibration.link_delay_unit",
             [](TwrConfig& x, int v) {
                 x.calibration.link_delay_unit = static_cast<TimeUnit>(v); }},
            {"frame.fcs_append",
             [](TwrConfig& x, int v) { x.frame.fcs_append = static_cast<FcsAppender>(v); }},
            {"tx.pulse_shaping",
             [](TwrConfig& x, int v) { x.tx.pulse_shaping = static_cast<PulseShaping>(v); }},
            {"calibration.first_path_algorithm",
             [](TwrConfig& x, int v) {
                 x.calibration.first_path_algorithm = static_cast<FirstPathAlgorithm>(v); }},
            {"calibration.cfo_compensation",
             [](TwrConfig& x, int v) {
                 x.calibration.cfo_compensation = static_cast<CompensationFlag>(v); }},
            {"calibration.sfo_compensation",
             [](TwrConfig& x, int v) {
                 x.calibration.sfo_compensation = static_cast<CompensationFlag>(v); }},
            {"session.protocol",
             [](TwrConfig& x, int v) { x.session.protocol = static_cast<Protocol>(v); }},
            {"session.role",
             [](TwrConfig& x, int v) { x.session.role = static_cast<Role>(v); }},
            {"timeouts.rx_timeout.domain",
             [](TwrConfig& x, int v) {
                 x.timeouts.rx_timeout.domain = static_cast<TimeDomain>(v); }},
            {"timeouts.rx_timeout.marker",
             [](TwrConfig& x, int v) {
                 x.timeouts.rx_timeout.marker = Opt<TimestampMarker>(
                     static_cast<TimestampMarker>(v)); }},
        };
        int refused = 0, total = 0;
        for (const Site& s : sites) {
            for (const int v : { 250 }) {
                TwrConfig x;
                s.set(x, v);
                ++total;
                const ValidationReport rep = validate(x, capabilities());
                if (rep.find(s.field, ConfigReason::UnknownEnumValue) != nullptr)
                    ++refused;
            }
        }
        printf("n07_config_refused=%d n07_config_total=%d\n", refused, total);

        // The ranging gate: only a COMPLETED burst may publish an instant.
        ClockDomain d7;
        ClockDomain::make("n07", 63.8976e9, 7, 40, d7);
        Timestamp tx7;
        Timestamp::from_ticks(5000, d7, TimestampMarker::RmarkerTx,
                              TimestampSource::ScheduledCalibrated,
                              timestamp_required_corrections(TimestampMarker::RmarkerTx),
                              tx7);
        tx7.calibration_id = "cal-n07";
        CalibrationStamp cal7;
        cal7.id = "cal-n07";
        cal7.calibrated_epoch = d7.epoch_id;
        cal7.valid_from_ticks = 0;
        cal7.valid_until_ticks = 1000000;
        CalibrationApplication app7;
        app7.calibration_id = "cal-n07";
        app7.result = CalibrationResult::Applied;
        cal7.applications.push_back(app7);
        RangeAdmissionContext ctx7;
        ctx7.calibration = &cal7;
        ctx7.reference_ticks = 4000;
        ctx7.reference_ticks_recorded = true;

        int bad_admitted = 0, bad_total = 0, ok_admitted = 0;
        for (const int v : { 5, 6, 99, 255 }) {
            TxSendEvidence ev;
            ev.command_time_recorded = true;
            ev.quantised_instant_recorded = true;
            ev.marker_offset_recorded = true;
            ev.calibrated_air_time_recorded = true;
            ev.send_accepted = true;
            ev.outcome = static_cast<TxOutcome>(v);
            ctx7.tx_evidence = &ev;
            ++bad_total;
            if (admit_range_capable_time(tx7, ctx7).admitted)
                ++bad_admitted;
        }
        {
            TxSendEvidence ev;
            ev.command_time_recorded = true;
            ev.quantised_instant_recorded = true;
            ev.marker_offset_recorded = true;
            ev.calibrated_air_time_recorded = true;
            ev.send_accepted = true;
            ev.outcome = TxOutcome::Completed;
            ctx7.tx_evidence = &ev;
            ok_admitted = admit_range_capable_time(tx7, ctx7).admitted ? 1 : 0;
        }
        printf("n07_tx_out_of_domain_admitted=%d n07_tx_out_of_domain_total=%d "
               "n07_tx_completed_admitted=%d\n",
               bad_admitted, bad_total, ok_admitted);
    }

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
import runpy, sys, json, copy
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

# --- N01: out-of-range integers are refused at the reader, not masked. ---
d = json.load(open("testdata/twr/config_parity_corpus.json", encoding="utf-8"))
base = d["base"]["ss_initiator"]
rows = [("phy.channel", 261), ("session.session_id", 4294967297),
        ("session.local_address", 65537), ("session.local_address", -1)]
refused = 0
for path, val in rows:
    doc = copy.deepcopy(base)
    grp, key = path.rsplit(".", 1)
    node = doc
    for part in grp.split("."):
        node = node[part]
    node[key] = val
    _cfg, rep = T.from_json_dict(doc)
    if (not rep.ok()) and rep.find(path, T.ConfigReason.OUT_OF_RANGE) is not None:
        refused += 1
print("n01_refused", refused)
print("n01_total", len(rows))

# --- N07: an enum value the enum does not have is refused, and named. ---
# JSON cannot express an out-of-domain enum (the reader only accepts known
# strings), so this is injected directly -- exactly the path a C++ caller or an
# async-event mapping would take, and the path a dataclass does not guard.
_c = T.from_json_dict(json.loads(json.dumps(base)))[0]
_n07_sites = [
    ("rx.agc", ("rx", "agc")), ("calibration.link_delay_unit",
     ("calibration", "link_delay_unit")),
    ("frame.fcs_append", ("frame", "fcs_append")),
    ("tx.pulse_shaping", ("tx", "pulse_shaping")),
    ("calibration.first_path_algorithm", ("calibration", "first_path_algorithm")),
    ("calibration.cfo_compensation", ("calibration", "cfo_compensation")),
    ("calibration.sfo_compensation", ("calibration", "sfo_compensation")),
    ("session.protocol", ("session", "protocol")),
    ("session.role", ("session", "role")),
    ("timeouts.rx_timeout.domain", ("timeouts", "rx_timeout", "domain")),
    ("timeouts.rx_timeout.marker", ("timeouts", "rx_timeout", "marker")),
]
_n07_refused = 0
for _field, _path_parts in _n07_sites:
    _cc = T.from_json_dict(json.loads(json.dumps(base)))[0]
    _node = _cc
    for _part in _path_parts[:-1]:
        _node = getattr(_node, _part)
    setattr(_node, _path_parts[-1], 250)
    _rep = T.validate(_cc, T.capabilities())
    if _rep.find(_field, T.ConfigReason.UNKNOWN_ENUM_VALUE) is not None:
        _n07_refused += 1
print("n07_py_refused", _n07_refused)
print("n07_py_total", len(_n07_sites))

# --- N06: no public in-place unfreeze of a running capability table. ---
snap = T.TwrConfigSnapshot(T.effective_config(m["_minimal"]()), T.capabilities())
print("n06_thaw_gone", int(not hasattr(snap.caps, "thaw")))
print("n06_still_read_only", int(snap.caps.read_only))
mine = snap.caps.mutable_copy()
mine.channels.append(9)
print("n06_copy_independent",
      int(snap.caps.channels == [5] and T.capabilities().channels == [5]))
"""
    run = subprocess.run([sys.executable, "-c", code], cwd=REPO,
                         capture_output=True, text=True)
    out: dict[str, str] = {}
    for line in run.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2:
            out[parts[0]] = parts[1]
    if run.returncode != 0:
        out["__error__"] = run.stderr[-1500:]
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


def probe_n05() -> dict[str, str]:
    """N05: the installed public API must be complete and current.

    Source-level, so it runs without an install prefix; the end-to-end half is
    `tools/twr/verify_install_consumer.sh`, which the release gate runs.
    """
    hdr_cmake = os.path.join(REPO, "gr-uwb", "include", "gnuradio", "uwb",
                             "CMakeLists.txt")
    app_cmake = os.path.join(REPO, "gr-uwb", "apps", "CMakeLists.txt")
    cc = os.path.join(REPO, "gr-uwb", "apps", "install_consumer",
                      "install_consumer.cc")
    py = os.path.join(REPO, "gr-uwb", "apps", "install_consumer",
                      "install_consumer.py")
    out: dict[str, str] = {}
    try:
        hdr = open(hdr_cmake, encoding="utf-8").read()
        app = open(app_cmake, encoding="utf-8").read()
        ccs = open(cc, encoding="utf-8").read()
        pys = open(py, encoding="utf-8").read()
    except OSError as exc:
        return {"__error__": str(exc)}

    def installed(name: str) -> bool:
        # An actual list entry, not a mention in a comment.
        return any(line.strip() == name for line in hdr.splitlines())

    out["n05_tof_input_installed"] = int(installed("uwb_twr_tof_input.h"))
    out["n05_test_output_installed"] = int(installed("uwb_twr_test_output.h"))
    out["n05_consumer_registered"] = int("uwb_qa_install_consumer" in app)
    out["n05_cc_schema_current"] = int(
        'caps.schema_version == "twr-config/2"' in ccs)
    out["n05_py_schema_current"] = int(
        'caps.schema_version == "twr-config/2"' in pys)
    # The stale ASSERTION, not any mention: a consumer may legitimately name
    # "twr-config/1" while asserting it is a refused legacy version.
    out["n05_cc_stale_schema"] = int(
        'caps.schema_version == "twr-config/1"' in ccs)
    out["n05_py_stale_schema"] = int(
        'caps.schema_version == "twr-config/1"' in pys)
    out["n05_cc_uses_admission"] = int("admit_range" in ccs)
    return out


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
    n5 = probe_n05()

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

    # R5b: fractional ordering must resolve to the EXACT answer, and the call
    # must actually succeed.  Asserting "not zero" would accept Equal (1) and
    # Later (3) -- which is precisely the "two same-source predicates agree"
    # mistake this script exists to catch.
    ok = c.get("r5_frac") == "2" and c.get("r5_frac_ok") == "1"
    record("R5 sub-tick fractions are ordered", "PASS" if ok else "FAIL",
           f"compare(0.25, 0.75) -> {c.get('r5_frac')} "
           f"(must be exactly 2 = Earlier), call_ok={c.get('r5_frac_ok')}")

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

    # -----------------------------------------------------------------------
    # M0.1 review findings N01-N06 (codex, 2026-09-29)
    # -----------------------------------------------------------------------

    # N01: an out-of-range integer must be refused, not masked.
    ok = (c.get("n01_refused") == c.get("n01_total")
          and c.get("n01_total") not in (None, "0", "-1")
          and p.get("n01_refused") == p.get("n01_total")
          and p.get("n01_total") not in (None, "0"))
    record("N01 out-of-range integers are refused, not masked",
           "PASS" if ok else "FAIL",
           f"C++ refused {c.get('n01_refused')}/{c.get('n01_total')}, "
           f"Python refused {p.get('n01_refused')}/{p.get('n01_total')} "
           f"of channel=261, session_id=2^32+1, local_address=65537, "
           f"local_address=-1 (M0 masked each to 5 / 1 / 1 / 0xffff)")

    # N02: a scheduled TX is admitted WITH send evidence, refused without it,
    # and never relabelled as a hardware measurement.
    ok = (c.get("n02_no_evidence") == "0"
          and c.get("n02_send_only") == "0"
          and c.get("n02_with_evidence") == "1"
          and c.get("n02_source_is_scheduled") == "1")
    record("N02 scheduled TX needs send evidence and stays labelled",
           "PASS" if ok else "FAIL",
           f"no evidence admitted={c.get('n02_no_evidence')} (must be 0), "
           f"send_accepted-only admitted={c.get('n02_send_only')} (must be 0), "
           f"complete evidence admitted={c.get('n02_with_evidence')} (must be 1), "
           f"source still scheduled_calibrated="
           f"{c.get('n02_source_is_scheduled')} (must be 1)")

    # N03: the projected ns and the lossless flag share one exact rational.
    ok = (c.get("n03_exact_ns") == c.get("n03_total")
          and c.get("n03_lossless") == c.get("n03_total")
          and c.get("n03_total") not in (None, "0"))
    record("N03 projected ns and lossless flag agree exactly",
           "PASS" if ok else "FAIL",
           f"{c.get('n03_exact_ns')}/{c.get('n03_total')} rows return the exact "
           f"nanosecond value and {c.get('n03_lossless')}/{c.get('n03_total')} "
           f"report it lossless (M0 returned exact-1 for all five)")

    # N04: the half-period bound holds after the fraction is merged.
    ok = c.get("n04_interval_status") not in (None, "0", "1")
    record("N04 half-period bound holds after the fraction is merged",
           "PASS" if ok else "FAIL",
           f"interval(2048+3/4, 1/4) in a 12-bit domain -> status "
           f"{c.get('n04_interval_status')} (0 would be Ok; P/2 = 2048 and the "
           f"merged value is 2048.5)")

    # N06: no public in-place unfreeze of a running capability table.
    ok = (p.get("n06_thaw_gone") == "1"
          and p.get("n06_still_read_only") == "1"
          and p.get("n06_copy_independent") == "1")
    record("N06 capability freeze cannot be undone through public API",
           "PASS" if ok else "FAIL",
           f"thaw() absent={p.get('n06_thaw_gone')} (must be 1), "
           f"still read-only={p.get('n06_still_read_only')} (must be 1), "
           f"mutable_copy independent={p.get('n06_copy_independent')} (must be 1)")

    # N05: the installed public API must be complete and current.
    def n5v(key):
        try:
            return int(n5.get(key, -1))
        except (TypeError, ValueError):
            return -1
    ok = (n5v("n05_tof_input_installed") == 1
          and n5v("n05_consumer_registered") == 1
          and n5v("n05_cc_schema_current") == 1
          and n5v("n05_py_schema_current") == 1
          and n5v("n05_cc_stale_schema") == 0
          and n5v("n05_py_stale_schema") == 0
          and n5v("n05_cc_uses_admission") == 1)
    record("N05 installed public API is complete and current",
           "PASS" if ok else "FAIL",
           f"uwb_twr_tof_input.h installed={n5.get('n05_tof_input_installed')} "
           f"(must be 1); test_output.h installed="
           f"{n5.get('n05_test_output_installed')} (deliberately 0); "
           f"consumer registered in CTest={n5.get('n05_consumer_registered')}; "
           f"consumers on twr-config/2 C++={n5.get('n05_cc_schema_current')} "
           f"Python={n5.get('n05_py_schema_current')}, stale /1 C++="
           f"{n5.get('n05_cc_stale_schema')} Python={n5.get('n05_py_stale_schema')}; "
           f"consumer exercises the admission entry="
           f"{n5.get('n05_cc_uses_admission')} "
           f"(end-to-end: tools/twr/verify_install_consumer.sh)")

    # N07: an enum value the enum does not have must be refused, never admitted.
    ok = (c.get("n07_config_refused") == c.get("n07_config_total")
          and c.get("n07_config_total") not in (None, "0", "-1")
          and c.get("n07_tx_out_of_domain_admitted") == "0"
          and c.get("n07_tx_out_of_domain_total") not in (None, "0")
          and c.get("n07_tx_completed_admitted") == "1"
          and p.get("n07_py_refused") == p.get("n07_py_total")
          and p.get("n07_py_total") not in (None, "0"))
    record("N07 an out-of-domain enum is refused, not admitted",
           "PASS" if ok else "FAIL",
           f"config enum sites named as out-of-domain: C++ "
           f"{c.get('n07_config_refused')}/{c.get('n07_config_total')}, Python "
           f"{p.get('n07_py_refused')}/{p.get('n07_py_total')}; "
           f"ranging gate admitted {c.get('n07_tx_out_of_domain_admitted')}"
           f"/{c.get('n07_tx_out_of_domain_total')} out-of-domain outcomes "
           f"(must be 0) while a Completed one is admitted="
           f"{c.get('n07_tx_completed_admitted')} (must be 1)")

    print("=" * 72)
    failed = [r for r in results if r[1] == "FAIL"]
    print(f"{len(results) - len(failed)}/{len(results)} checks pass")
    if failed:
        print("FAILED: " + ", ".join(f[0].split()[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
