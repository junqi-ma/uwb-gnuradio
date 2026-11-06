#!/usr/bin/env python3
"""Synthetic contract QA for the independent discrete-CIR verifier.

No C++ build, no GNU Radio import and no generated UWB case are needed.  The
fixture is a hand-built finite preamble with one weak echo; every mutation
tampers with a declared artifact or record and must be rejected.
"""
from __future__ import annotations

import copy
import json
import math
import tempfile
from pathlib import Path

import numpy as np

import verify_discrete_cir_compare as V

N = 1024
SPS = 16
ORIGIN = 32
OBSERVED_END = 256
SUPPORT = [96, 512]
WIRE = b"qa"


def _wire(text):
    payload = text.encode()
    return (payload + V.crc16(payload).to_bytes(2, "little")).hex()


def _add(target, wave, origin, gain=1.0):
    high = min(len(target), origin + len(wave))
    if high <= origin:
        return
    target[origin:high] += np.complex64(gain) * wave[:high - origin]


def _times():
    times = {"estimate": 7.0, "comm_demod": 2.0, "validate": 3.0, "tx": 4.0,
             "channel": 6.0, "subtract": 6.0, "copy": 1.0,
             "sic_total": 29.0, "cir": 8.0, "sensing_demod": 9.0,
             "service": 46.0, "thread_cpu": 100.0}
    assert times["sic_total"] == sum(
        times[k] for k in ("estimate", "comm_demod", "validate", "tx",
                           "channel", "subtract", "copy"))
    assert times["service"] == times["sic_total"] + times["cir"] + times["sensing_demod"]
    return times


def _discrete():
    return {
        "model_id": 9876543210987654321,
        "q": ORIGIN,
        "cfo_hz": 0.0,
        "lag_lo": -8,
        "tap_count": 8,
        "active_count": 5,
        "condition": 1234.5,
        "effective_rank": 8,
        "fit_residual": 0.01,
        "waveform_scale": 0.8,
        "support": list(SUPPORT),
        "sub": list(SUPPORT),
        "times_us": {
            "detect": 1.0, "cfo": 1.0, "support": 1.0, "projection": 1.0,
            "solve": 1.0, "fit_check": 1.0, "estimate_total": 7.0,
            "validate": 3.0, "tx": 4.0, "scale": 1.0,
            "convolution": 2.0, "cfo_rot": 1.0, "subtract": 5.5,
        },
    }


def _events():
    return {"required_sample": N, "submit_us": 0.0, "start_us": 1.0,
            "finish_us": 2.0, "result_us": 3.0, "first_clean_us": 2.5,
            "sample_ready_us": 0.5, "scheduled_us": 0.0, "submit_late_us": 0.0}


def _row(side, raw_hash, comm_wire, sensing_wire):
    row = {
        "window_id": 0,
        "source_window": 0,
        "mode": "full",
        "reconstruction": "full" if side == "old" else "discrete_cir",
        "rx_pipeline": "fast",
        "rx_avx2": True,
        "n": N,
        "committed": True,
        "reason": "committed",
        "model_valid": True,
        "model_id": 111 if side == "old" else _discrete()["model_id"],
        "origin": ORIGIN if side == "old" else ORIGIN - 1,
        "observed_end": OBSERVED_END,
        "support": list(SUPPORT),
        "rejected": False,
        "comm_fcs": True,
        "comm_status": 0,
        "comm_psdu_hex": comm_wire,
        "sensing_fcs": True,
        "sensing_psdu_hex": sensing_wire,
        "sensing_status": 0,
        "cir_status": 0,
        "times_us": _times(),
        "events": _events(),
        "input_hash": raw_hash,
        "input_hash_added_offline": True,
    }
    if side == "new":
        row["discrete_cir"] = _discrete()
    return row


def _save_rows(run, rows):
    (run / "windows.jsonl").write_text(
        "".join(json.dumps(row, allow_nan=False) + "\n" for row in rows))


def _summary(reconstruction):
    return {"schema": "fullpacket-compare/1", "reconstruction": reconstruction,
            "submitted": 1, "completed": 1, "rejected": 0, "source_late": 0,
            "paced_rate": 0, "rx_pipeline": "fast", "rx_avx2": True,
            "input_pool": 1}


def build_fixture(tmp):
    case, old, new = tmp / "case", tmp / "old", tmp / "new"
    window = case / "windows" / "w000000"
    for path in (window, old, new):
        path.mkdir(parents=True)
    rng = np.random.default_rng(7)
    sync = (rng.standard_normal(SPS) + 1j * rng.standard_normal(SPS)).astype(np.complex64)
    sync *= np.complex64(0.1)
    train = np.zeros(N, dtype=np.complex64)
    for repeat in range(64):
        _add(train, sync, ORIGIN + repeat * SPS)
    weak = np.zeros(N, dtype=np.complex64)
    weak[37:] = np.complex64(0.25) * train[:N - 37]
    noise = (1e-4 * (rng.standard_normal(N) + 1j * rng.standard_normal(N))).astype(np.complex64)
    clean = (train + weak + noise).astype(np.complex64)
    interference = np.zeros(N, dtype=np.complex64)
    grid = np.arange(SUPPORT[0], SUPPORT[1])
    interference[SUPPORT[0]:SUPPORT[1]] = np.complex64(0.05) * np.exp(
        2j * np.pi * 0.01 * grid)
    raw = (clean + interference).astype(np.complex64)
    old_out = raw.copy()
    new_out = raw.copy()
    old_out[SUPPORT[0]:SUPPORT[1]] = (clean + np.complex64(0.35) * interference)[
        SUPPORT[0]:SUPPORT[1]]
    new_out[SUPPORT[0]:SUPPORT[1]] = (clean + np.complex64(0.15) * interference)[
        SUPPORT[0]:SUPPORT[1]]
    comm_wire, sensing_wire = _wire("comm"), _wire("sensing")
    raw_hash = V.sha256_bytes(raw.tobytes())
    public = {"schema": "sic-fullpacket-case/1", "cell": "qa",
              "sample_rate": 998400000.0, "n": N, "observed_end": OBSERVED_END,
              "target_lo": 0, "target_hi": N, "sensing_seed": ORIGIN,
              "roi_lo": 0, "roi_hi": 64, "template_hashes": {}}
    templates = {"comm_sync.cf32": np.ones(SPS, np.complex64),
                 "comm_preamble.cf32": np.ones(2 * SPS, np.complex64),
                 "sensing_sync.cf32": sync}
    for name, wave in templates.items():
        wave.tofile(case / name)
        public["template_hashes"][name] = V.sha256(case / name)
    raw.tofile(window / "raw.cf32")
    clean.tofile(window / "clean.cf32")
    np.zeros(N, np.complex64).tofile(window / "sensing.cf32")
    interference.tofile(window / "interference.cf32")
    sidecar = {"comm_psdu_hex": comm_wire, "sensing_psdu_hex": sensing_wire,
               "sensing_start": ORIGIN, "target_delay_samples": 37,
               "fields": {"SYNC": list(SUPPORT), "payload": [200, 300]},
               "hashes": {"raw.cf32": V.sha256(window / "raw.cf32"),
                          "clean.cf32": V.sha256(window / "clean.cf32")}}
    (window / "sidecar.json").write_text(json.dumps(sidecar))
    (case / "public.json").write_text(json.dumps(public))
    rows = {"old": _row("old", raw_hash, comm_wire, sensing_wire),
            "new": _row("new", raw_hash, comm_wire, sensing_wire)}
    _save_rows(old, [rows["old"]])
    _save_rows(new, [rows["new"]])
    (old / "runner_summary.json").write_text(json.dumps(_summary("full")))
    (new / "runner_summary.json").write_text(json.dumps(_summary("discrete_cir")))
    old_out.tofile(old / "w000000_full.cf32")
    new_out.tofile(new / "w000000_full.cf32")
    return dict(case=case, old=old, new=new, window=window, public=public,
                sidecar=sidecar, rows=rows, raw=raw, clean=clean,
                old_out=old_out, new_out=new_out, comm_wire=comm_wire,
                sensing_wire=sensing_wire)


def evaluate(fixture, rows=None, require_provenance=False, summary_only=False):
    rows = rows or fixture["rows"]
    _save_rows(fixture["old"], [rows["old"]])
    _save_rows(fixture["new"], [rows["new"]])
    return V.compare(fixture["case"], fixture["old"], fixture["new"],
                     require_provenance=require_provenance,
                     summary_only=summary_only)


def _expect_rejected(fixture, label, rows, needle=None):
    result = evaluate(fixture, rows)
    assert not result["passed"], label + " escaped the gate"
    text = " ".join(result["structural_errors"] + result["failures"])
    if needle:
        assert needle in text, f"{label}: expected {needle!r} in {text!r}"
    return result


def main():
    mutations = 0
    with tempfile.TemporaryDirectory(prefix="discrete-verifier-qa-") as tmpdir:
        tmp = Path(tmpdir)
        fixture = build_fixture(tmp)
        case, old, new = fixture["case"], fixture["old"], fixture["new"]
        rows = fixture["rows"]

        baseline = evaluate(fixture, require_provenance=True)
        assert baseline["passed"], (baseline["failures"],
                                   baseline["structural_errors"])
        assert baseline["iq_effects_verified"]
        assert baseline["provenance_verified"]
        assert baseline["verification_level"] == "dumped_iq"
        cell = baseline["cells"][0]
        assert len(cell["gates"]) == 7
        assert all(gate["evaluated"] for gate in cell["gates"]), cell["gates"]
        assert all(gate["passed"] for gate in cell["gates"]), cell["gates"]
        differences = cell["exact_differences"]
        assert differences["comm_fcs"] == 0 and differences["comm_exact"] == 0
        assert differences["sensing_fcs"] == 0 and differences["sensing_exact"] == 0
        assert differences["target_suppression_db"] > 5.0, differences
        assert cell["counts"]["old"]["n"] == 1
        assert cell["counts"]["old"]["comm_exact"] == 1
        # Independent model ids/origins are explicitly allowed to differ.
        assert rows["old"]["model_id"] != rows["new"]["model_id"]
        assert rows["old"]["origin"] != rows["new"]["origin"]
        V.write_outputs(baseline, tmp / "comparison.json")
        assert (tmp / "comparison.csv").is_file()
        header = (tmp / "comparison.csv").read_text().splitlines()[0]
        assert "metric" in header and "passed" in header
        try:
            V.write_outputs(baseline, tmp / "comparison.json")
        except FileExistsError:
            pass
        else:
            raise AssertionError("comparison output overwrite was accepted")

        # 1. Missing discrete_cir object.
        changed = copy.deepcopy(rows)
        changed["new"].pop("discrete_cir")
        _expect_rejected(fixture, "missing discrete_cir", changed,
                         "discrete_cir object missing")
        mutations += 1

        # 2. Tampered output outside the declared support.
        tampered = fixture["new_out"].copy()
        tampered[10] += np.complex64(0.5)
        tampered.tofile(new / "w000000_full.cf32")
        _expect_rejected(fixture, "outside support", rows, "outside its support")
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        mutations += 1

        # 3. Changed raw IQ hash.
        changed_raw = fixture["raw"].copy()
        changed_raw[5] += np.complex64(1.0)
        changed_raw.tofile(fixture["window"] / "raw.cf32")
        provenance = V.compare(case, old, new, require_provenance=True)
        assert not provenance["passed"]
        assert any("input hash" in error or "sidecar hash" in error
                   for error in provenance["structural_errors"]), \
            provenance["structural_errors"]
        fixture["raw"].tofile(fixture["window"] / "raw.cf32")
        mutations += 1

        # 4. Reduced target suppression (still a legal in-support output).
        worse = fixture["raw"].copy()
        low, high = SUPPORT
        worse[low:high] = (fixture["clean"] +
                           np.complex64(0.9) * (fixture["raw"] - fixture["clean"]))[low:high]
        worse.tofile(new / "w000000_full.cf32")
        reduced = _expect_rejected(fixture, "reduced suppression", rows,
                                   "target_suppression_db")
        assert reduced["cells"][0]["exact_differences"]["target_suppression_db"] < -1.0
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        mutations += 1

        # 5. Old archive mislabelled as the cached backend.
        changed = copy.deepcopy(rows)
        changed["old"]["reconstruction"] = "payload_cached"
        _expect_rejected(fixture, "old reconstruction", changed,
                         "reconstruction is not full")
        mutations += 1

        # 6. Bypass changed IQ.
        changed = copy.deepcopy(rows)
        changed["new"].update(committed=False, reason="model_invalid")
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        _expect_rejected(fixture, "bypass changed", changed, "bypass changed IQ")
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        mutations += 1

        # 6b. An invalid model bypasses with a zero-width hull and zero
        # active taps; that must remain structurally valid (only the decode
        # count can then fail the gate).
        changed = copy.deepcopy(rows)
        changed["new"].update(committed=False, model_valid=False,
                              comm_fcs=False, comm_psdu_hex="",
                              reason="detection_below_threshold", support=[0, 0])
        invalid = changed["new"]["discrete_cir"]
        invalid.update(model_id=0, q=-1, cfo_hz=0.0, lag_lo=0, tap_count=0,
                       active_count=0, condition=0.0, effective_rank=0,
                       fit_residual=0.0, waveform_scale=0.0,
                       support=[0, 0], sub=[0, 0])
        invalid["times_us"] = dict.fromkeys(invalid["times_us"], 0.0)
        fixture["raw"].tofile(new / "w000000_full.cf32")
        invalid_result = evaluate(fixture, changed)
        assert invalid_result["cells"][0]["structural"]["passed"], \
            invalid_result["cells"][0]["structural"]["errors"]
        assert invalid_result["passed"] is False
        assert any("comm_fcs" in failure for failure in invalid_result["failures"])
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        mutations += 1

        # 7. Duplicate source exposure in a functional run.
        changed = copy.deepcopy(rows)
        duplicate = copy.deepcopy(changed["new"])
        duplicate["window_id"] = 1
        _save_rows(new, [changed["new"], duplicate])
        _save_rows(old, [changed["old"]])
        duplicate_result = V.compare(case, old, new)
        assert not duplicate_result["passed"]
        assert any("repeats source windows" in error
                   for error in duplicate_result["structural_errors"])
        _save_rows(new, [rows["new"]])
        _save_rows(old, [rows["old"]])
        mutations += 1

        # 8. Discrete estimate parent exceeds the row estimate parent.
        changed = copy.deepcopy(rows)
        changed["new"]["discrete_cir"]["times_us"]["estimate_total"] = 500.0
        _expect_rejected(fixture, "estimate parent", changed,
                         "estimate_total exceeds")
        mutations += 1

        # 9. Sub interval outside the convolution support.
        changed = copy.deepcopy(rows)
        changed["new"]["discrete_cir"]["sub"] = [0, 200]
        _expect_rejected(fixture, "sub outside support", changed,
                         "sub outside the convolution support")
        mutations += 1

        # 10. Support wider than the finite-support contract.
        changed = copy.deepcopy(rows)
        changed["new"]["discrete_cir"]["tap_count"] = 2000
        _expect_rejected(fixture, "tap count", changed, "tap_count")
        mutations += 1

        # 10b. Active taps must stay inside the continuous hull and a
        # committed model must have at least one active tap.
        changed = copy.deepcopy(rows)
        changed["new"]["discrete_cir"]["active_count"] = 9
        _expect_rejected(fixture, "active count above hull", changed,
                         "active_count")
        changed = copy.deepcopy(rows)
        changed["new"]["discrete_cir"]["active_count"] = 0
        _expect_rejected(fixture, "committed without active taps", changed,
                         "no active tap")
        mutations += 2

        # 11. Missing dumped output.
        (new / "w000000_full.cf32").rename(tmp / "parked.cf32")
        _expect_rejected(fixture, "missing output", rows, "dumped output missing")
        (tmp / "parked.cf32").rename(new / "w000000_full.cf32")
        mutations += 1

        # 12. SIC stage partition broken.
        changed = copy.deepcopy(rows)
        changed["old"]["times_us"]["sic_total"] += 10.0
        _expect_rejected(fixture, "SIC partition", changed, "SIC stage partition")
        mutations += 1

        # 13. Decode regression with a structurally valid bypass.
        changed = copy.deepcopy(rows)
        changed["new"].update(committed=False, comm_fcs=False,
                              comm_psdu_hex="", reason="comm_fcs_failed")
        fixture["raw"].tofile(new / "w000000_full.cf32")
        decode = evaluate(fixture, changed)
        assert not decode["passed"]
        assert any("comm_fcs" in failure for failure in decode["failures"]), \
            decode["failures"]
        fixture["new_out"].tofile(new / "w000000_full.cf32")
        mutations += 1

        # 14. Old row must not carry a discrete object.
        changed = copy.deepcopy(rows)
        changed["old"]["discrete_cir"] = copy.deepcopy(changed["new"]["discrete_cir"])
        _expect_rejected(fixture, "old discrete object", changed,
                         "unexpectedly carries discrete_cir")
        mutations += 1

        # 15. Equal zero sensing counts are equal, not an undefined ratio.
        changed = copy.deepcopy(rows)
        for side in ("old", "new"):
            changed[side].update(sensing_fcs=False, sensing_psdu_hex="")
        equal_zero = evaluate(fixture, changed)
        assert equal_zero["passed"], (equal_zero["failures"],
                                      equal_zero["structural_errors"])
        assert equal_zero["cells"][0]["counts"]["old"]["sensing_fcs"] == 0
        assert equal_zero["cells"][0]["counts"]["new"]["sensing_fcs"] == 0
        mutations += 1

        # 16. A paired count regression must fail; counts are not ratios.
        changed = copy.deepcopy(rows)
        changed["new"].update(sensing_fcs=False, sensing_psdu_hex="")
        lost = evaluate(fixture, changed)
        assert not lost["passed"]
        assert any("sensing_fcs" in failure for failure in lost["failures"])
        mutations += 1

        # 17. Summary-only validates records without certifying IQ effects.
        (new / "w000000_full.cf32").rename(tmp / "parked2.cf32")
        summary = evaluate(fixture, rows, summary_only=True)
        assert summary["passed"], (summary["failures"], summary["structural_errors"])
        assert summary["verification_level"] == "summary_only"
        assert not summary["iq_effects_verified"]
        assert summary["cells"][0]["gates"] == []
        (tmp / "parked2.cf32").rename(new / "w000000_full.cf32")
        mutations += 1

        # 18. Output hash declared by the record must match the dumped file.
        changed = copy.deepcopy(rows)
        changed["new"]["output_hash"] = "0" * 64
        _expect_rejected(fixture, "output hash", changed, "output hash mismatch")
        mutations += 1

        # Restore the baseline and prove it still passes after all mutations.
        _save_rows(old, [rows["old"]])
        _save_rows(new, [rows["new"]])
        final = V.compare(case, old, new, require_provenance=True)
        assert final["passed"], (final["failures"], final["structural_errors"])

    print(f"qa_discrete_cir_compare: baseline + {mutations} mutations PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
