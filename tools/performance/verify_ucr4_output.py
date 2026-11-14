#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent verifier for the pure-C++ pipeline output writer.

It parses the on-disk contract produced by `PipelineOutputWriter`
(`<cir_base>.ucr4` + `<cir_base>.jsonl` + `run.json` + `<comm_base>.jsonl`)
WITHOUT importing anything from this repository: the whole point is to be a
second, independent reader of the bytes so that a bug in the C++ writer cannot
be hidden by a bug in the C++ reader.

Contract implemented here (see
docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent开发方案_2026-11-14.md §3):

  run.json
      tap_format == "ucr4", header_bytes == 52, bytes_per_tap == 4,
      byte_order == "little-endian", range_m_per_tap > 0.

  <cir_base>.ucr4
      Concatenated 52-byte little-endian headers, each followed by
      tap_count * 4 bytes of interleaved int16 I/Q.  A frame whose
      tap_count == 0 writes NO record (jsonl only) and does not advance
      file_offset_taps.

  <cir_base>.jsonl
      One JSON object per window, including failed frames.

  <comm_base>.jsonl
      One JSON object per comm attempt; psdu_hex (when non-empty) must hash to
      psdu_fnv1a with FNV-1a-64.

Usage:
    verify_ucr4_output.py --dir <output_dir> [--cir-base cir]
                          [--comm-base demod_results] [--expect-windows N]
    verify_ucr4_output.py --self-test

Exit codes: 0 = every check passed, 1 = at least one check failed,
2 = usage error.  `--self-test` exits 0 only when the valid fixture is
accepted AND every deliberately corrupted fixture is rejected.
"""

import argparse
import json
import math
import os
import shutil
import struct
import sys
import tempfile

HEADER_BYTES = 52
BYTES_PER_TAP = 4
MAGIC = b"UCR4"
STATUS_MAX = 5

# char[4], u32 pulse_id, u16 status, u16 tap_count, u16 repetition_index,
# u16 repetition_count, f32 sfd_metric, f32 cir_peak_metric, i32 peak_tap,
# u32 estimator_us, f64 freq_hz, f64 freq_offset_hz, f32 cir_scale
HEADER_FMT = "<4sIHHHHffiIddf"
HEADER_STRUCT = struct.Struct(HEADER_FMT)
HEADER_FIELDS = (
    "magic",
    "pulse_id",
    "status",
    "tap_count",
    "repetition_index",
    "repetition_count",
    "sfd_metric",
    "cir_peak_metric",
    "peak_tap",
    "estimator_us",
    "freq_hz",
    "freq_offset_hz",
    "cir_scale",
)
assert HEADER_STRUCT.size == HEADER_BYTES, "UCR4 header must be 52 bytes"

FNV_OFFSET_BASIS = 0xCBF29CE484222325
FNV_PRIME = 0x100000001B3
UINT64_MASK = 0xFFFFFFFFFFFFFFFF


def fnv1a64(data):
    """64-bit FNV-1a, the same function the C++ side uses."""
    h = FNV_OFFSET_BASIS
    for byte in data:
        h ^= byte
        h = (h * FNV_PRIME) & UINT64_MASK
    return h


class Report(object):
    """Ordered PASS/FAIL record; one entry per named check."""

    def __init__(self):
        self.checks = []

    def check(self, name, ok, detail=""):
        self.checks.append((name, bool(ok), detail))
        return bool(ok)

    @property
    def ok(self):
        return all(entry[1] for entry in self.checks)

    def failures(self):
        return [entry for entry in self.checks if not entry[1]]

    def failed_names(self):
        return [entry[0] for entry in self.failures()]

    def print_lines(self, stream=sys.stdout):
        for name, ok, detail in self.checks:
            line = "  %s %s" % ("PASS" if ok else "FAIL", name)
            if detail:
                line += ": " + detail
            stream.write(line + "\n")


def _read_jsonl(path):
    """Return (rows, errors).  Blank lines are ignored."""
    rows = []
    errors = []
    with open(path, "r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                obj = json.loads(text)
            except ValueError as exc:
                errors.append("line %d: invalid JSON (%s)" % (lineno, exc))
                continue
            if not isinstance(obj, dict):
                errors.append("line %d: not a JSON object" % lineno)
                continue
            rows.append(obj)
    return rows, errors


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _as_int(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and float(value).is_integer():
        return int(value)
    return None


def _check_run_json(report, directory):
    path = os.path.join(directory, "run.json")
    if not os.path.isfile(path):
        report.check("run_json_present", False, "%s is missing" % path)
        return
    try:
        with open(path, "r", encoding="utf-8") as handle:
            meta = json.load(handle)
    except ValueError as exc:
        report.check("run_json_present", False, "%s is not valid JSON (%s)" % (path, exc))
        return
    if not isinstance(meta, dict):
        report.check("run_json_present", False, "%s is not a JSON object" % path)
        return
    report.check("run_json_present", True, path)

    problems = []
    if meta.get("tap_format") != "ucr4":
        problems.append("tap_format=%r (want 'ucr4')" % (meta.get("tap_format"),))
    if _as_int(meta.get("header_bytes")) != HEADER_BYTES:
        problems.append("header_bytes=%r (want %d)" % (meta.get("header_bytes"), HEADER_BYTES))
    if _as_int(meta.get("bytes_per_tap")) != BYTES_PER_TAP:
        problems.append("bytes_per_tap=%r (want %d)" % (meta.get("bytes_per_tap"), BYTES_PER_TAP))
    if meta.get("byte_order") != "little-endian":
        problems.append("byte_order=%r (want 'little-endian')" % (meta.get("byte_order"),))
    range_per_tap = meta.get("range_m_per_tap")
    if not _is_number(range_per_tap) or not math.isfinite(float(range_per_tap)) \
            or float(range_per_tap) <= 0.0:
        problems.append("range_m_per_tap=%r (want a positive number)" % (range_per_tap,))
    report.check("run_json_fields", not problems, "; ".join(problems))


def _check_cir_jsonl(report, directory, cir_base):
    path = os.path.join(directory, cir_base + ".jsonl")
    if not os.path.isfile(path):
        report.check("cir_jsonl_present", False, "%s is missing" % path)
        return None
    report.check("cir_jsonl_present", True, path)
    rows, errors = _read_jsonl(path)
    report.check("cir_jsonl_parse", not errors, "; ".join(errors[:5]))
    return rows


def _check_cir_rows(report, rows):
    """file_offset_taps / zero_delay_tap / booleans / peak delay."""
    offset_problems = []
    field_problems = []
    delay_problems = []

    expected_offset = 0
    for index, row in enumerate(rows):
        pulse_id = row.get("pulse_id")
        tap_count = _as_int(row.get("tap_count"))
        if tap_count is None or tap_count < 0:
            field_problems.append("row %d (pulse %r): tap_count=%r is not a non-negative int"
                                  % (index, pulse_id, row.get("tap_count")))
            continue

        file_offset = _as_int(row.get("file_offset_taps"))
        if file_offset is None:
            offset_problems.append("row %d (pulse %r): file_offset_taps=%r is not an int"
                                   % (index, pulse_id, row.get("file_offset_taps")))
        elif file_offset != expected_offset:
            offset_problems.append("row %d (pulse %r): file_offset_taps=%d, expected %d"
                                   % (index, pulse_id, file_offset, expected_offset))
        expected_offset += tap_count

        zero_delay = _as_int(row.get("zero_delay_tap"))
        cir_pre = _as_int(row.get("cir_pre_samples"))
        if zero_delay is None:
            field_problems.append("row %d (pulse %r): zero_delay_tap=%r is not an int"
                                  % (index, pulse_id, row.get("zero_delay_tap")))
        if cir_pre is None:
            field_problems.append("row %d (pulse %r): cir_pre_samples=%r is not an int"
                                  % (index, pulse_id, row.get("cir_pre_samples")))
        if zero_delay is not None and cir_pre is not None and zero_delay != cir_pre:
            field_problems.append("row %d (pulse %r): zero_delay_tap=%d != cir_pre_samples=%d"
                                  % (index, pulse_id, zero_delay, cir_pre))

        for key in ("sfd_ok", "timing_ok"):
            if not isinstance(row.get(key), bool):
                field_problems.append("row %d (pulse %r): %s=%r is not a boolean"
                                      % (index, pulse_id, key, row.get(key)))

        if tap_count > 0:
            peak_tap = _as_int(row.get("peak_tap"))
            sample_rate = row.get("sample_rate")
            reported = row.get("peak_delay_from_calibration_ns")
            if peak_tap is None or zero_delay is None:
                delay_problems.append("row %d (pulse %r): peak_tap/zero_delay_tap missing"
                                      % (index, pulse_id))
            elif not _is_number(sample_rate) or float(sample_rate) <= 0.0:
                delay_problems.append("row %d (pulse %r): sample_rate=%r is not positive"
                                      % (index, pulse_id, sample_rate))
            elif not _is_number(reported):
                delay_problems.append("row %d (pulse %r): peak_delay_from_calibration_ns=%r "
                                      "is not a number" % (index, pulse_id, reported))
            else:
                expected_ns = (float(peak_tap) - float(zero_delay)) \
                    / float(sample_rate) * 1.0e9
                if abs(expected_ns) > 0.0:
                    rel = abs(float(reported) - expected_ns) / abs(expected_ns)
                    if rel > 1.0e-6:
                        delay_problems.append(
                            "row %d (pulse %r): peak_delay_from_calibration_ns=%.12g, "
                            "expected %.12g (relative error %.3g > 1e-6)"
                            % (index, pulse_id, float(reported), expected_ns, rel))
                elif abs(float(reported)) > 1.0e-9:
                    delay_problems.append(
                        "row %d (pulse %r): peak_delay_from_calibration_ns=%.12g, "
                        "expected 0" % (index, pulse_id, float(reported)))

    report.check("cir_jsonl_offsets", not offset_problems, "; ".join(offset_problems[:5]))
    report.check("cir_jsonl_fields", not field_problems, "; ".join(field_problems[:5]))
    report.check("cir_peak_delay", not delay_problems, "; ".join(delay_problems[:5]))


def _check_ucr4(report, directory, cir_base, rows):
    path = os.path.join(directory, cir_base + ".ucr4")
    with_taps = [row for row in rows if (_as_int(row.get("tap_count")) or 0) > 0]
    expected_size = sum((_as_int(row.get("tap_count")) or 0) * BYTES_PER_TAP
                        for row in rows) + HEADER_BYTES * len(with_taps)

    if not os.path.isfile(path):
        if expected_size == 0:
            report.check("ucr4_present", True,
                         "%s absent but every jsonl row has tap_count == 0" % path)
            report.check("ucr4_total_size", True, "expected 0 bytes")
            report.check("ucr4_record_headers", True, "no records")
            return
        report.check("ucr4_present", False, "%s is missing (expected %d bytes)"
                     % (path, expected_size))
        report.check("ucr4_total_size", False, "file missing")
        report.check("ucr4_record_headers", False, "file missing")
        return
    report.check("ucr4_present", True, path)

    actual_size = os.path.getsize(path)
    size_ok = (actual_size == expected_size)
    report.check("ucr4_total_size", size_ok,
                 "actual %d bytes, expected %d (%d records x %d + %d taps x %d)"
                 % (actual_size, expected_size, len(with_taps), HEADER_BYTES,
                    (expected_size - HEADER_BYTES * len(with_taps)) // BYTES_PER_TAP,
                    BYTES_PER_TAP))

    header_problems = []
    with open(path, "rb") as handle:
        previous_pulse_id = None
        for index, row in enumerate(with_taps):
            blob = handle.read(HEADER_BYTES)
            if len(blob) != HEADER_BYTES:
                header_problems.append("record %d: truncated header (%d of %d bytes)"
                                       % (index, len(blob), HEADER_BYTES))
                break
            values = HEADER_STRUCT.unpack(blob)
            header = dict(zip(HEADER_FIELDS, values))
            row_tap_count = _as_int(row.get("tap_count")) or 0
            row_pulse_id = _as_int(row.get("pulse_id"))

            if header["magic"] != MAGIC:
                header_problems.append("record %d: magic=%r (want %r)"
                                       % (index, header["magic"], MAGIC))
            if not 0 <= header["status"] <= STATUS_MAX:
                header_problems.append("record %d: status=%d outside 0..%d"
                                       % (index, header["status"], STATUS_MAX))
            if header["tap_count"] != row_tap_count:
                header_problems.append("record %d: header tap_count=%d != jsonl tap_count=%d"
                                       % (index, header["tap_count"], row_tap_count))
            if not math.isfinite(header["cir_scale"]) or header["cir_scale"] < 0.0:
                header_problems.append("record %d: cir_scale=%r is not finite and >= 0"
                                       % (index, header["cir_scale"]))
            if previous_pulse_id is not None and header["pulse_id"] <= previous_pulse_id:
                header_problems.append("record %d: pulse_id=%d is not strictly ascending "
                                       "(previous %d)"
                                       % (index, header["pulse_id"], previous_pulse_id))
            previous_pulse_id = header["pulse_id"]
            if row_pulse_id is not None and header["pulse_id"] != row_pulse_id:
                header_problems.append("record %d: header pulse_id=%d != jsonl pulse_id=%d"
                                       % (index, header["pulse_id"], row_pulse_id))

            payload = handle.read(header["tap_count"] * BYTES_PER_TAP)
            if len(payload) != header["tap_count"] * BYTES_PER_TAP:
                header_problems.append("record %d: truncated taps (%d of %d bytes)"
                                       % (index, len(payload),
                                          header["tap_count"] * BYTES_PER_TAP))
                break

        trailing = handle.read(1)
        if trailing:
            header_problems.append("trailing bytes after the last record")

    report.check("ucr4_record_headers", not header_problems, "; ".join(header_problems[:5]))


def _check_comm_jsonl(report, directory, comm_base):
    path = os.path.join(directory, comm_base + ".jsonl")
    if not os.path.isfile(path):
        report.check("comm_jsonl_present", False, "%s is missing" % path)
        return
    report.check("comm_jsonl_present", True, path)
    rows, errors = _read_jsonl(path)
    report.check("comm_jsonl_parse", not errors, "; ".join(errors[:5]))

    problems = []
    hash_problems = []
    for index, row in enumerate(rows):
        packet_id = row.get("packet_id")
        if _as_int(packet_id) is None:
            problems.append("line %d: packet_id=%r is not an integer" % (index + 1, packet_id))
        psdu_hex = row.get("psdu_hex")
        if psdu_hex is None or psdu_hex == "":
            continue
        if not isinstance(psdu_hex, str):
            hash_problems.append("line %d (packet %r): psdu_hex is not a string"
                                 % (index + 1, packet_id))
            continue
        try:
            payload = bytes.fromhex(psdu_hex)
        except ValueError as exc:
            hash_problems.append("line %d (packet %r): psdu_hex is not valid hex (%s)"
                                 % (index + 1, packet_id, exc))
            continue
        reported = row.get("psdu_fnv1a")
        if _as_int(reported) is None:
            hash_problems.append("line %d (packet %r): psdu_fnv1a=%r is not an integer"
                                 % (index + 1, packet_id, reported))
            continue
        computed = fnv1a64(payload)
        if computed != _as_int(reported):
            hash_problems.append(
                "line %d (packet %r): psdu_fnv1a=0x%016x but FNV-1a-64(psdu_hex)=0x%016x"
                % (index + 1, packet_id, _as_int(reported), computed))

    report.check("comm_jsonl_fields", not problems, "; ".join(problems[:5]))
    report.check("comm_psdu_fnv1a", not hash_problems, "; ".join(hash_problems[:5]))


def _check_expect_windows(report, rows, expect_windows):
    pulse_ids = [_as_int(row.get("pulse_id")) for row in rows]
    problems = []
    if len(pulse_ids) != expect_windows:
        problems.append("got %d jsonl rows, expected %d" % (len(pulse_ids), expect_windows))
    expected = list(range(expect_windows))
    if pulse_ids != expected:
        problems.append("pulse_id sequence is %r, expected %r"
                        % (pulse_ids[:10], expected[:10]))
    report.check("expect_windows", not problems, "; ".join(problems))


def verify_dir(directory, cir_base="cir", comm_base="demod_results", expect_windows=None):
    """Run every check against an output directory and return a Report."""
    report = Report()
    if not os.path.isdir(directory):
        report.check("output_dir_present", False, "%s is not a directory" % directory)
        return report
    report.check("output_dir_present", True, directory)

    _check_run_json(report, directory)

    rows = _check_cir_jsonl(report, directory, cir_base)
    if rows is None:
        rows = []
    else:
        _check_cir_rows(report, rows)
    _check_ucr4(report, directory, cir_base, rows)
    _check_comm_jsonl(report, directory, comm_base)

    if expect_windows is not None:
        _check_expect_windows(report, rows, expect_windows)

    return report


# ---------------------------------------------------------------------------
# --self-test: build one valid fixture and several corrupted ones
# ---------------------------------------------------------------------------
def _header_bytes(pulse_id, status, tap_count, peak_tap, sample_rate=998.4e6,
                  cir_scale=0.5, magic=MAGIC):
    return HEADER_STRUCT.pack(
        magic,
        pulse_id,
        status,
        tap_count,
        0xFFFF,          # repetition_index: averaged frame
        1,               # repetition_count
        0.75,            # sfd_metric
        1.25,            # cir_peak_metric
        peak_tap,
        42,              # estimator_us
        3993.6e6,        # freq_hz
        0.0,             # freq_offset_hz
        cir_scale,
    )


def _write_fixture(directory, tap_rows, comm_rows):
    """Write a complete, self-consistent output directory."""
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, "run.json"), "w", encoding="utf-8") as handle:
        json.dump({
            "block": "PipelineOutputWriter",
            "directory": directory,
            "tap_format": "ucr4",
            "byte_order": "little-endian",
            "header_bytes": HEADER_BYTES,
            "bytes_per_tap": BYTES_PER_TAP,
            "reconstruction": "fc32 = sc16 * cir_scale",
            "range_m_per_tap": 299792458.0 / (2.0 * 998.4e6),
        }, handle)
        handle.write("\n")

    offset = 0
    cir_jsonl = []
    ucr4 = bytearray()
    for pulse_id, tap_count, peak_tap in tap_rows:
        cir_jsonl.append({
            "pulse_id": pulse_id,
            "schedule_index": pulse_id,
            "status": "ok" if tap_count > 0 else "sfd_failed",
            "sample_rate": 998.4e6,
            "tap_count": tap_count,
            "cir_pre_samples": 2,
            "cir_post_samples": 6,
            "file_offset_taps": offset,
            "zero_delay_tap": 2,
            "peak_tap": peak_tap,
            "cir_peak_metric": 1.25,
            "peak_delay_from_calibration_ns":
                (float(peak_tap) - 2.0) / 998.4e6 * 1.0e9 if tap_count > 0 else 0.0,
            "range_m_per_tap": 299792458.0 / (2.0 * 998.4e6),
            "raw_l2_norm": 3.5,
            "sfd_ok": tap_count > 0,
            "timing_ok": tap_count > 0,
            "estimator_us": 42,
        })
        if tap_count > 0:
            ucr4 += _header_bytes(pulse_id, 0, tap_count, peak_tap)
            ucr4 += bytes(2 * tap_count * 2)  # int16 I/Q pairs
        offset += tap_count

    with open(os.path.join(directory, "cir.jsonl"), "w", encoding="utf-8") as handle:
        for row in cir_jsonl:
            handle.write(json.dumps(row) + "\n")
    with open(os.path.join(directory, "cir.ucr4"), "wb") as handle:
        handle.write(bytes(ucr4))

    with open(os.path.join(directory, "demod_results.jsonl"), "w", encoding="utf-8") as handle:
        for packet_id, psdu_hex in comm_rows:
            payload = bytes.fromhex(psdu_hex)
            handle.write(json.dumps({
                "packet_id": packet_id,
                "status": "ok",
                "fcs_pass": True,
                "psdu_hex": psdu_hex,
                "psdu_fnv1a": fnv1a64(payload),
            }) + "\n")


def _copy_fixture(src, dst):
    shutil.copytree(src, dst)
    return dst


def _patch_jsonl_row(path, index, **updates):
    with open(path, "r", encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle if line.strip()]
    rows[index].update(updates)
    with open(path, "w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _patch_ucr4_header(path, record_index, **updates):
    with open(path, "rb") as handle:
        blob = bytearray(handle.read())
    base = 0
    for _ in range(record_index):
        header = dict(zip(HEADER_FIELDS, HEADER_STRUCT.unpack_from(blob, base)))
        base += HEADER_BYTES + header["tap_count"] * BYTES_PER_TAP
    header = dict(zip(HEADER_FIELDS, HEADER_STRUCT.unpack_from(blob, base)))
    header.update(updates)
    HEADER_STRUCT.pack_into(blob, base, *[header[name] for name in HEADER_FIELDS])
    with open(path, "wb") as handle:
        handle.write(bytes(blob))


def self_test():
    """Return True only if the valid fixture passes and every corruption fails."""
    tmp = tempfile.mkdtemp(prefix="verify-ucr4-selftest.")
    ok = True
    try:
        tap_rows = [(0, 4, 4), (1, 0, 0), (2, 6, 3)]
        comm_rows = [(0, "deadbeef"), (1, ""), (2, "00ff10")]
        valid = os.path.join(tmp, "valid")
        _write_fixture(valid, tap_rows, comm_rows)

        report = verify_dir(valid, expect_windows=3)
        print("[self-test] valid fixture")
        report.print_lines()
        if not report.ok:
            print("  FAIL: the valid fixture was rejected")
            ok = False
        else:
            print("  PASS: valid fixture accepted (%d checks)" % len(report.checks))

        cases = []

        def add_case(name, expect_check, mutate):
            cases.append((name, expect_check, mutate))

        def bad_magic(directory):
            _patch_ucr4_header(os.path.join(directory, "cir.ucr4"), 0, magic=b"XCR4")

        def truncated(directory):
            path = os.path.join(directory, "cir.ucr4")
            with open(path, "rb") as handle:
                blob = handle.read()
            with open(path, "wb") as handle:
                handle.write(blob[:-4])

        def offset_discontinuity(directory):
            _patch_jsonl_row(os.path.join(directory, "cir.jsonl"), 2,
                             file_offset_taps=5)

        def tap_count_mismatch(directory):
            _patch_ucr4_header(os.path.join(directory, "cir.ucr4"), 1, tap_count=5)

        def non_ascending_pulse_id(directory):
            _patch_ucr4_header(os.path.join(directory, "cir.ucr4"), 1, pulse_id=0)

        def bad_psdu_fnv1a(directory):
            _patch_jsonl_row(os.path.join(directory, "demod_results.jsonl"), 0,
                             psdu_fnv1a=1)

        add_case("bad magic", "ucr4_record_headers", bad_magic)
        add_case("truncated record", "ucr4_total_size", truncated)
        add_case("file_offset_taps discontinuity", "cir_jsonl_offsets",
                 offset_discontinuity)
        add_case("tap_count mismatch", "ucr4_record_headers", tap_count_mismatch)
        add_case("non-ascending pulse_id", "ucr4_record_headers",
                 non_ascending_pulse_id)
        add_case("bad psdu_fnv1a", "comm_psdu_fnv1a", bad_psdu_fnv1a)

        for name, expect_check, mutate in cases:
            directory = _copy_fixture(valid, os.path.join(tmp, "case-" +
                                                          name.replace(" ", "-")))
            mutate(directory)
            case_report = verify_dir(directory, expect_windows=3)
            failed = case_report.failed_names()
            if case_report.ok:
                print("  FAIL: corruption '%s' was ACCEPTED" % name)
                ok = False
            elif expect_check not in failed:
                print("  FAIL: corruption '%s' was rejected, but '%s' did not fail "
                      "(failed: %s)" % (name, expect_check, ", ".join(failed)))
                ok = False
            else:
                print("  PASS: corruption '%s' rejected by '%s'" % (name, expect_check))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Independent verifier for PipelineOutputWriter output.")
    parser.add_argument("--dir", help="output directory to verify")
    parser.add_argument("--cir-base", default="cir", help=".ucr4/.jsonl base name")
    parser.add_argument("--comm-base", default="demod_results",
                        help="demod_results base name")
    parser.add_argument("--expect-windows", type=int, default=None,
                        help="require exactly N pulse_ids, numbered 0..N-1")
    parser.add_argument("--self-test", action="store_true",
                        help="build valid + corrupted fixtures and assert the verdicts")
    args = parser.parse_args(argv)

    if args.self_test:
        print("verify_ucr4_output.py --self-test")
        ok = self_test()
        print("PASS: self-test" if ok else "FAIL: self-test")
        return 0 if ok else 1

    if not args.dir:
        parser.error("--dir is required unless --self-test is given")
    if args.expect_windows is not None and args.expect_windows < 0:
        parser.error("--expect-windows must be >= 0")

    print("verify_ucr4_output.py --dir %s (cir=%s, comm=%s)"
          % (args.dir, args.cir_base, args.comm_base))
    report = verify_dir(args.dir, cir_base=args.cir_base, comm_base=args.comm_base,
                        expect_windows=args.expect_windows)
    report.print_lines()
    if report.ok:
        print("PASS: %d checks" % len(report.checks))
        return 0
    print("FAIL: %d of %d checks failed" % (len(report.failures()), len(report.checks)))
    return 1


if __name__ == "__main__":
    sys.exit(main())
