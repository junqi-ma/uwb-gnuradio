#!/usr/bin/env python3
"""Self-test for the Step-12 layered validation CLI.

Runs x410_uwb_radar_validate.py as a subprocess and asserts:
  1. --dry-run prints one JSON plan (737280000.0 / sc16 / stage / PRI /
     windows / backend_mode) and exits 0 without touching a device
  2. parameter / mode / adapter errors exit 1 with a JSONL error line
  3. ladder gating rejects level skipping and bad stage records
  4. without UHD the builtin adapter fails cleanly with exit 2
     (device_unavailable / uhd_unavailable), no traceback on stdout
  5. the scripted self-test adapter runs the full ladder, and injected
     faults fail the stage with exit 3
  6. external-adapter subprocess protocol (Step-11 integration point)
  7. every report line matches the JSONL schema

Run directly (stdlib unittest) or under pytest:
  python3 gr-uwb/apps/x410_uwb_radar_validate_selftest.py
  python3 -m pytest gr-uwb/apps/x410_uwb_radar_validate_selftest.py -q
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, "x410_uwb_radar_validate.py")
ENV_KEY = "UWB_RADAR_VALIDATE_TEST_ADAPTER"
SCHEMA_REPORT = "x410_uwb_radar_validate.report.v1"
SCHEMA_PLAN = "x410_uwb_radar_validate.plan.v1"
SCHEMA_ADAPTER = "x410_uwb_radar_validate.adapter.v1"
STAGES = ["smoke", "single", "low-rate", "soak", "calibration", "cable", "ota"]
GAIN_ARGS = ["--gain-tx", "40", "--gain-rx", "30"]


def run_cli(args, env_extra=None, timeout=120):
    env = dict(os.environ)
    env.pop(ENV_KEY, None)
    if env_extra:
        env.update(env_extra)
    return subprocess.run(
        [sys.executable, APP] + args, capture_output=True, text=True,
        timeout=timeout, env=env)


def run_scripted(args, env_extra=None, **kw):
    env = dict(os.environ)
    env[ENV_KEY] = "1"
    if env_extra:
        env.update(env_extra)
    return run_cli(args, env_extra=env, **kw)


def json_lines(text):
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def find_line(lines, type_):
    for line in lines:
        if line.get("type") == type_:
            return line
    return None


def write_adcfg(tmp, **cfg):
    path = os.path.join(tmp, "adapter_config.json")
    body = {"schema": SCHEMA_ADAPTER, "type": "scripted"}
    body.update(cfg)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(body, f)
    return path


def out_arg(tmp, name="out"):
    return os.path.join(tmp, name)


def report_args(tmp, name="out"):
    return ["--output", out_arg(tmp, name)]


def valid_record(path, stage, backend="builtin_uhd_python",
                 evidence="hardware", result="pass", rate=737280000.0):
    with open(path, "w", encoding="utf-8") as f:
        f.write(json.dumps({"schema": SCHEMA_REPORT, "seq": 0,
                            "type": "run_start", "stage": stage}) + "\n")
        f.write(json.dumps({"schema": SCHEMA_REPORT, "seq": 1,
                            "type": "stage_result", "stage": stage,
                            "result": result, "backend": backend,
                            "evidence_class": evidence, "rate_hz": rate,
                            "checks": [{"name": "x", "ok": True,
                                        "details": "fixture"}]}) + "\n")


class Step12SelfTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        assert os.path.isfile(APP), APP

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="x410r-selftest-")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    # ------------------------------------------------------------------
    # 1. dry-run
    # ------------------------------------------------------------------

    def test_dry_run_default_plan(self):
        r = run_cli(["--dry-run"])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        plan = json.loads(r.stdout)
        self.assertEqual(plan["schema"], SCHEMA_PLAN)
        self.assertEqual(plan["mode"], "dry-run")
        self.assertEqual(plan["rate"]["hz"], 737280000.0)
        self.assertEqual(plan["sample_format"], {"cpu": "sc16", "otw": "sc16"})
        self.assertIsNone(plan["requested_stage"])
        self.assertEqual(plan["backend_mode"], "builtin_uhd_python")
        self.assertEqual(plan["pri"]["s"], 0.005)
        self.assertEqual(plan["pri"]["ticks"], 3686400)
        w = plan["windows"]
        self.assertEqual(w["pre_guard_native"], 1475)
        self.assertEqual(w["sync_native"], 48018)
        self.assertEqual(w["sfd_native_4z2"], 6003)
        self.assertEqual(w["range_guard_native"], 74)
        self.assertEqual(w["resampler_tail_native"], 3023)
        self.assertEqual(w["rx_window_native"], 58593)
        stages = [s["stage"] for s in plan["stages"]]
        self.assertEqual(stages, STAGES)
        self.assertTrue(plan["hardware_execution_pending"])
        self.assertFalse(plan["environment"]["uhd_module_probed"])

    def test_dry_run_requested_stage(self):
        r = run_cli(["--dry-run", "--stage", "soak"])
        self.assertEqual(r.returncode, 0, r.stdout)
        plan = json.loads(r.stdout)
        self.assertEqual(plan["requested_stage"], "soak")

    def test_dry_run_does_not_create_output(self):
        out = os.path.join(self.tmp, "never")
        r = run_cli(["--dry-run", "--output", out])
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertFalse(os.path.exists(out))

    # ------------------------------------------------------------------
    # 2. mode / CLI errors (exit 1, machine-readable)
    # ------------------------------------------------------------------

    def test_missing_mode(self):
        r = run_cli([])
        self.assertEqual(r.returncode, 1)
        lines = json_lines(r.stdout)
        self.assertEqual(lines[-1]["type"], "summary")
        self.assertEqual(lines[-1]["result"], "validation_error")
        self.assertEqual(lines[-1]["exit_code"], 1)

    def test_unknown_stage(self):
        r = run_cli(["--stage", "bogus"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("cli_invalid_arg", r.stdout)

    def test_rate_contract_fixed(self):
        r = run_cli(["--dry-run", "--rate", "750e6"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("rate_contract_fixed", r.stdout)

    def test_pri_off_tick_grid(self):
        r = run_cli(["--dry-run", "--pri-s", "0.00333"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("pri_not_on_integer_tick_grid", r.stdout)

    def test_low_rate_pri_range(self):
        r = run_cli(["--stage", "low-rate", "--pri-s", "0.05"] + GAIN_ARGS +
                    ["--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 1)
        self.assertIn("stage_pri_out_of_range", r.stdout)

    def test_rx_window_cap(self):
        r = run_cli(["--dry-run", "--rx-window-samples", "1200000"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("rx_window_exceeds_cap", r.stdout)

    def test_tx_overruns_pri(self):
        r = run_cli(["--dry-run", "--stage", "single", "--pri-s", "0.0001"])
        self.assertEqual(r.returncode, 1)
        self.assertTrue("tx_overruns_next_rx" in r.stdout or
                        "rx_window_exceeds_pri" in r.stdout, r.stdout)

    def test_fragment_cap(self):
        r = run_cli(["--dry-run", "--stage", "single",
                     "--max-fragment-samples", "1000"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("fragment_cap_exceeded", r.stdout)

    def test_missing_explicit_gain(self):
        r = run_scripted(["--stage", "single",
                          "--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 1)
        self.assertIn("missing_explicit_gain", r.stdout)

    def test_output_required(self):
        r = run_scripted(["--stage", "single"] + GAIN_ARGS)
        self.assertEqual(r.returncode, 1)
        self.assertIn("output_required", r.stdout)

    def test_output_not_empty(self):
        out = out_arg(self.tmp)
        os.makedirs(out)
        with open(os.path.join(out, "x"), "w") as f:
            f.write("x")
        r = run_scripted(["--stage", "smoke", "--output", out])
        self.assertEqual(r.returncode, 1)
        self.assertIn("output_dir_not_empty", r.stdout)

    # ------------------------------------------------------------------
    # 3. ladder gating (stage records)
    # ------------------------------------------------------------------

    def test_missing_stage_record(self):
        r = run_scripted(["--stage", "single"] + GAIN_ARGS +
                         ["--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 1)
        self.assertIn("missing_stage_record", r.stdout)

    def test_predecessor_not_passed(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke", result="fail")
        r = run_scripted(["--stage", "single"] + GAIN_ARGS +
                         ["--output", out_arg(self.tmp),
                          "--stage-record", rec])
        self.assertEqual(r.returncode, 1)
        self.assertIn("predecessor_stage_not_passed", r.stdout)

    def test_wrong_predecessor_in_record(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke")
        r = run_scripted(["--stage", "low-rate"] + GAIN_ARGS +
                         ["--output", out_arg(self.tmp),
                          "--stage-record", rec, "--pri-s", "0.1"])
        self.assertEqual(r.returncode, 1)
        self.assertIn("wrong_predecessor_in_record", r.stdout)

    def test_record_bad_json(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        with open(rec, "w") as f:
            f.write("{not json}\n")
        r = run_scripted(["--stage", "single"] + GAIN_ARGS +
                         ["--output", out_arg(self.tmp),
                          "--stage-record", rec])
        self.assertEqual(r.returncode, 1)
        self.assertIn("stage_record_invalid", r.stdout)

    def test_record_unparseable(self):
        r = run_scripted(["--stage", "single"] + GAIN_ARGS +
                         ["--output", out_arg(self.tmp),
                          "--stage-record", os.path.join(self.tmp, "nope")])
        self.assertEqual(r.returncode, 1)
        self.assertIn("stage_record_unreadable", r.stdout)

    # ------------------------------------------------------------------
    # 4. no-UHD hardware path (builtin adapter on this machine)
    # ------------------------------------------------------------------

    def test_smoke_no_uhd_exit2(self):
        r = run_cli(["--stage", "smoke", "--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        lines = json_lines(r.stdout)
        self.assertNotIn("Traceback", r.stdout)
        sr = find_line(lines, "stage_result")
        self.assertIsNotNone(sr)
        self.assertEqual(sr["result"], "error")
        codes = {c["code"] for c in sr.get("checks", []) if "code" in c}
        self.assertTrue(codes & {"uhd_unavailable", "uhd_device_unavailable",
                                 "device_unavailable"},
                        "unexpected codes %r" % (codes,))
        summary = find_line(lines, "summary")
        self.assertEqual(summary["exit_code"], 2)
        report = os.path.join(out_arg(self.tmp), "report.jsonl")
        self.assertTrue(os.path.isfile(report))
        with open(report, encoding="utf-8") as f:
            file_lines = [json.loads(x) for x in f if x.strip()]
        self.assertEqual(file_lines, lines)

    def test_single_no_uhd_after_valid_gate(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke")
        r = run_cli(["--stage", "single"] + GAIN_ARGS +
                    ["--output", out_arg(self.tmp), "--stage-record", rec])
        self.assertEqual(r.returncode, 2)
        lines = json_lines(r.stdout)
        gate = find_line(lines, "gate")
        self.assertIsNotNone(gate)
        sr = find_line(lines, "stage_result")
        self.assertEqual(sr["result"], "error")

    # ------------------------------------------------------------------
    # 5. scripted adapter (self-test only)
    # ------------------------------------------------------------------

    def test_scripted_requires_env(self):
        r = run_cli(["--stage", "smoke", "--adapter", "scripted",
                     "--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 1)
        self.assertIn("scripted_adapter_requires_env", r.stdout)

    def test_scripted_smoke_pass(self):
        r = run_scripted(["--stage", "smoke", "--adapter", "scripted",
                          "--output", out_arg(self.tmp)])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        lines = json_lines(r.stdout)
        sr = find_line(lines, "stage_result")
        self.assertEqual(sr["result"], "pass")
        self.assertEqual(sr["evidence_class"], "test")
        self.assertFalse(sr["hardware_verified"])
        names = [c["name"] for c in sr["checks"]]
        self.assertIn("rate_exact_737p28", names)
        self.assertIn("no_tx_issued", names)

    def test_scripted_single_pass(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke", backend="scripted", evidence="test")
        r = run_scripted(["--stage", "single", "--adapter", "scripted"] +
                         GAIN_ARGS + ["--output", out_arg(self.tmp),
                                      "--stage-record", rec])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        sr = find_line(json_lines(r.stdout), "stage_result")
        self.assertEqual(sr["result"], "pass")

    def test_scripted_ladder_chain(self):
        rec = ""
        for i, stage in enumerate(STAGES):
            out = out_arg(self.tmp, "stage%d" % i)
            args = ["--stage", stage, "--adapter", "scripted", "--output", out]
            if stage != "smoke":
                args += ["--stage-record", rec]
            if stage in ("single", "low-rate", "soak", "calibration",
                         "cable", "ota"):
                args += GAIN_ARGS
            if stage == "low-rate":
                args += ["--pulses", "4", "--pri-s", "0.1"]
            if stage == "soak":
                args += ["--duration-s", "0.2", "--pri-s", "0.01"]
            if stage in ("calibration", "cable", "ota"):
                args += ["--pulses", "4", "--write-rx-iq"]
            r = run_scripted(args, timeout=120)
            self.assertEqual(r.returncode, 0,
                             "stage %s failed: %s%s" % (stage, r.stdout,
                                                        r.stderr))
            lines = json_lines(r.stdout)
            sr = find_line(lines, "stage_result")
            self.assertEqual(sr["result"], "pass", stage)
            self.assertEqual(sr["evidence_class"], "test")
            rec = os.path.join(out, "report.jsonl")
            self.assertTrue(os.path.isfile(rec))

    def test_ota_observation_only(self):
        rec = ""
        stages_upto = ["smoke", "single", "low-rate", "soak", "calibration",
                       "cable", "ota"]
        for i, stage in enumerate(stages_upto):
            out = out_arg(self.tmp, "l%d" % i)
            args = ["--stage", stage, "--adapter", "scripted", "--output", out]
            if stage != "smoke":
                args += ["--stage-record", rec]
            if stage != "smoke":
                args += GAIN_ARGS
            if stage == "low-rate":
                args += ["--pulses", "3", "--pri-s", "0.1"]
            if stage == "soak":
                args += ["--duration-s", "0.2", "--pri-s", "0.01"]
            if stage in ("calibration", "cable", "ota"):
                args += ["--pulses", "3", "--write-rx-iq"]
            r = run_scripted(args, timeout=120)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            rec = os.path.join(out, "report.jsonl")
        lines = json_lines(open(rec, encoding="utf-8").read())
        sr = find_line(lines, "stage_result")
        self.assertEqual(sr["stage"], "ota")
        self.assertEqual(sr["result"], "pass")
        self.assertTrue(sr["ota_observation_only"])
        text = json.dumps(sr)
        self.assertIn("observation", text.lower())
        self.assertIn("never", text.lower())

    def test_scripted_fault_overflow_fail(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "single", backend="scripted", evidence="test")
        recdir = out_arg(self.tmp, "base")
        r = run_scripted(["--stage", "low-rate", "--adapter", "scripted",
                          "--output", recdir] + GAIN_ARGS +
                         ["--pulses", "2", "--pri-s", "0.1",
                          "--stage-record", rec])
        self.assertEqual(r.returncode, 0, r.stdout)
        cfg = write_adcfg(self.tmp, fault="overflow", fault_at_index=0)
        r = run_scripted(["--stage", "soak", "--adapter", "scripted",
                          "--adapter-config", cfg, "--pri-s", "0.01",
                          "--duration-s", "0.2", "--output",
                          out_arg(self.tmp, "fault"),
                          "--stage-record",
                          os.path.join(recdir, "report.jsonl")] + GAIN_ARGS)
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        sr = find_line(json_lines(r.stdout), "stage_result")
        self.assertEqual(sr["result"], "fail")
        self.assertFalse(all(c["ok"] for c in sr["checks"]))

    def test_scripted_fault_short_rx_fail(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke", backend="scripted", evidence="test")
        cfg = write_adcfg(self.tmp, fault="short_rx", fault_at_index=0)
        r = run_scripted(["--stage", "single", "--adapter", "scripted",
                          "--adapter-config", cfg, "--output",
                          out_arg(self.tmp, "srx"),
                          "--stage-record", rec] + GAIN_ARGS)
        self.assertEqual(r.returncode, 3)
        sr = find_line(json_lines(r.stdout), "stage_result")
        self.assertEqual(sr["result"], "fail")

    def test_scripted_fault_gates_next_stage(self):
        out = out_arg(self.tmp, "f")
        r = run_scripted(["--stage", "smoke", "--adapter", "scripted",
                          "--output", out])
        self.assertEqual(r.returncode, 0)
        rec = os.path.join(out, "report.jsonl")
        r = run_scripted(["--stage", "single", "--adapter", "scripted",
                          "--output", out_arg(self.tmp, "s2"),
                          "--stage-record", rec] + GAIN_ARGS)
        self.assertEqual(r.returncode, 0)
        r = run_scripted(["--stage", "low-rate", "--adapter", "scripted",
                          "--output", out_arg(self.tmp, "s3"),
                          "--stage-record",
                          os.path.join(out_arg(self.tmp, "s2"),
                                       "report.jsonl")] + GAIN_ARGS +
                         ["--pulses", "2", "--pri-s", "0.1"])
        self.assertEqual(r.returncode, 0)

    def test_scripted_evidence_cannot_gate_hardware(self):
        rec = os.path.join(self.tmp, "sc.jsonl")
        valid_record(rec, "smoke", backend="scripted", evidence="test")
        r = run_cli(["--stage", "single"] + GAIN_ARGS +
                    ["--output", out_arg(self.tmp), "--stage-record", rec])
        self.assertEqual(r.returncode, 1)
        self.assertIn("predecessor_evidence_not_hardware", r.stdout)

    # ------------------------------------------------------------------
    # 6. external adapter subprocess protocol
    # ------------------------------------------------------------------

    def test_external_adapter_stub_smoke(self):
        stub = os.path.join(self.tmp, "stub_runner.py")
        with open(stub, "w", encoding="utf-8") as f:
            f.write(STUB_RUNNER)
        cfg = {"schema": SCHEMA_ADAPTER, "type": "external",
               "backend": "stub_external", "evidence_class": "test",
               "command": sys.executable, "argv": [stub]}
        cpath = os.path.join(self.tmp, "ext.json")
        with open(cpath, "w", encoding="utf-8") as f:
            json.dump(cfg, f)
        r = run_cli(["--stage", "smoke", "--adapter", "external",
                     "--adapter-config", cpath,
                     "--output", out_arg(self.tmp, "ext")])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        lines = json_lines(r.stdout)
        sr = find_line(lines, "stage_result")
        self.assertEqual(sr["result"], "pass")
        self.assertEqual(sr["evidence_class"], "test")

    # ------------------------------------------------------------------
    # 7. JSONL schema
    # ------------------------------------------------------------------

    def test_report_schema_fields(self):
        r = run_scripted(["--stage", "smoke", "--adapter", "scripted",
                          "--output", out_arg(self.tmp, "schema")])
        self.assertEqual(r.returncode, 0)
        lines = json_lines(r.stdout)
        self.assertGreater(len(lines), 2)
        seqs = []
        for line in lines:
            self.assertEqual(line["schema"], SCHEMA_REPORT)
            self.assertIn("seq", line)
            self.assertIn("ts_utc", line)
            self.assertIn("run_id", line)
            self.assertIn("type", line)
            self.assertIn("level", line)
            seqs.append(line["seq"])
        self.assertEqual(seqs, sorted(seqs))
        self.assertEqual(len(set(seqs)), len(seqs))
        types = {line["type"] for line in lines}
        self.assertIn("plan", types)
        self.assertIn("gate", types)
        self.assertIn("stage_result", types)
        summary = find_line(lines, "summary")
        self.assertEqual(summary["requested_stage"], "smoke")
        self.assertEqual(summary["next_expected_stage"], "single")

    def test_report_summary_next_stage(self):
        rec = os.path.join(self.tmp, "rec.jsonl")
        valid_record(rec, "smoke", backend="scripted", evidence="test")
        r = run_scripted(["--stage", "single", "--adapter", "scripted",
                          "--output", out_arg(self.tmp, "n"),
                          "--stage-record", rec] + GAIN_ARGS)
        self.assertEqual(r.returncode, 0)
        summary = find_line(json_lines(r.stdout), "summary")
        self.assertEqual(summary["requested_stage"], "single")
        self.assertEqual(summary["next_expected_stage"], "low-rate")


STUB_RUNNER = '''#!/usr/bin/env python3
"""Minimal external-adapter stub (x410_uwb_radar_validate.adapter.v1)."""
import json, sys

SEQ = [0]


def send(obj):
    sys.stdout.write(json.dumps(obj) + "\\n")
    sys.stdout.flush()


def main():
    probe = {"device": "stub-external", "rate_hz": 737280000.0,
             "clock_source": "internal", "time_source": "internal",
             "tx_channel": 0, "rx_channel": 1,
             "tx_antennas": ["TX/RX0"], "rx_antennas": ["RX1"],
             "rx_antenna": "RX1", "otw": "sc16", "cpu": "sc16",
             "tx_streamer_created": False, "rx_streamer_created": True}
    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        req = json.loads(raw)
        SEQ[0] += 1
        op = req.get("op")
        if op == "setup":
            send({"seq": SEQ[0], "ok": True, "probe": probe})
        elif op == "close":
            send({"seq": SEQ[0], "ok": True})
            return 0
        else:
            send({"seq": SEQ[0], "ok": True})
    return 0


if __name__ == "__main__":
    sys.exit(main())
'''


if __name__ == "__main__":
    unittest.main(verbosity=2)
