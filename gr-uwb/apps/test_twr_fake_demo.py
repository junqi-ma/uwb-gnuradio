#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""QA for the M1-B two-endpoint TWR demo (``twr_fake_demo``).

Run:
    python3 gr-uwb/apps/test_twr_fake_demo.py

WHAT THIS PROVES
----------------
  1. the real C++ two-endpoint SS/DS simulation runs end to end from Python;
  2. the ToF it publishes is the EXACT integer this test derives
     independently with ``fractions.Fraction`` from the ACTUAL frame field
     values and local evidence in the output -- the test never calls the C++
     formula;
  3. a deliberately wrong configuration is REJECTED (non-zero exit, status.ok
     false), and a non-simulation envelope cannot be smuggled in;
  4. the protocol status can NEVER be read as a ranging success:
     ``measurement_valid`` is false, ``yields_range`` is false, the provenance
     is ``wire_claim`` / ``simulation``, and no endpoint carries a distance.

The binary is taken from ``$UWB_TWR_FAKE_DEMO`` / ``gr-uwb/build/twr_fake_demo``
/ obvious fallbacks, and otherwise built out of tree with ``g++``.  If neither a
binary nor ``g++`` is available the suite SKIPS with an explicit message.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from fractions import Fraction as F

_HERE = os.path.dirname(os.path.abspath(__file__))          # gr-uwb/apps
_GR_UWB = os.path.dirname(_HERE)                             # gr-uwb
_REPO = os.path.dirname(_GR_UWB)                             # repository root
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import twr_fake_demo as D  # noqa: E402  (path set above)

if "gnuradio" in sys.modules:  # pragma: no cover - setup guard
    raise SystemExit("test_twr_fake_demo must import with NO GNU Radio present")


# ==========================================================================
# Locate (or build) the demo binary
# ==========================================================================


def _locate_or_build():
    demo, source = D.find_demo_binary()
    if demo is not None:
        return demo, "found (%s): %s" % (source, demo)

    gxx = shutil.which("g++")
    if gxx is None:
        return None, "no demo binary and no g++ to build one (%s)" % source

    build_dir = tempfile.mkdtemp(prefix="twr_fake_demo_build_")
    out = os.path.join(build_dir, "twr_fake_demo")
    cmd = [
        gxx, "-std=c++17", "-Wall", "-Wextra",
        "-I", os.path.join(_GR_UWB, "include"),
        "-o", out,
        os.path.join(_HERE, "twr_fake_demo.cc"),
        os.path.join(_GR_UWB, "lib", "uwb_twr_core.cc"),
        os.path.join(_GR_UWB, "lib", "uwb_twr_fake_link.cc"),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        return None, "g++ build failed (%d):\n%s" % (proc.returncode, proc.stderr)
    return out, "built out of tree: %s" % out


_BINARY, _BINARY_NOTE = _locate_or_build()


# ==========================================================================
# Test fixtures
# ==========================================================================


class DemoCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="twr_fake_demo_qa_")

    def run_scenario(self, scenario, **kwargs):
        request = D.build_request(scenario, **kwargs)
        request_path = os.path.join(self.tmp, "%s.request.json" % scenario)
        output_path = os.path.join(self.tmp, "%s.output.json" % scenario)
        try:
            output, req_path, out_path = D.run_demo(
                _BINARY, request, request_path=request_path,
                output_path=output_path)
        except D.DemoError as exc:
            self.fail("demo failed for %s: %s\nstderr=%s\noutput=%s"
                      % (scenario, exc, exc.stderr, exc.output))
        return output, request_path, out_path

    def run_request(self, request, name):
        request_path = os.path.join(self.tmp, "%s.request.json" % name)
        with open(request_path, "w", encoding="utf-8") as handle:
            json.dump(request, handle, indent=2)
        output_path = os.path.join(self.tmp, "%s.output.json" % name)
        proc = subprocess.run(
            [_BINARY, "--request", request_path, "--output", output_path],
            capture_output=True, text=True)
        output = None
        if os.path.isfile(output_path):
            with open(output_path, "r", encoding="utf-8") as handle:
                try:
                    output = json.load(handle)
                except ValueError:
                    output = None
        return proc.returncode, output

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _endpoint(output, name):
        return output["endpoints"][name]

    @staticmethod
    def _local(ep, group, ftype):
        return ep["local_evidence"][group][ftype]

    @staticmethod
    def _frame(ep, ftype, direction):
        for f in ep["frames"]:
            if f["type"] == ftype and f["dir"] == direction:
                return f
        raise AssertionError("no %s %s frame" % (direction, ftype))

    @staticmethod
    def _ts(fr, name):
        return fr["fields"]["timestamps"][name]


# ==========================================================================
# The tests
# ==========================================================================


@unittest.skipUnless(_BINARY, "no twr_fake_demo binary: %s" % _BINARY_NOTE)
class TestDemo(DemoCase):
    def test_ss_tof_is_exact_integer(self):
        """SS: initiator ToF == (RA - k*DB)/2, derived here from the output."""
        output, req_path, out_path = self.run_scenario("ss_unity")
        self.assertTrue(output["status"]["ok"], "run status must be ok")
        self.assertEqual(output["protocol"], "ss")

        a = self._endpoint(output, "a")
        self.assertEqual(a["completion"], "complete")
        self.assertTrue(a["estimate_available"])
        resp = self._frame(a, "response", "rx")
        t1 = int(self._local(a, "tx_planned_ticks", "poll"))
        t4 = int(self._local(a, "rx_marker_ticks", "response"))
        db = F(int(self._ts(resp, "t3B")) - int(self._ts(resp, "t2B")))
        expected = (F(t4 - t1) - F(1) * db) / 2
        self.assertEqual(expected, F(100))
        tof = a["tof"]
        self.assertEqual(F(int(tof["num"]), int(tof["den"])), expected)
        self.assertEqual(int(tof["num"]), 100)
        self.assertEqual(int(tof["den"]), 1)

        # The responder owns NO estimate: the result is never copied across.
        b = self._endpoint(output, "b")
        self.assertEqual(b["completion"], "complete")
        self.assertFalse(b["estimate_available"])
        self.assertFalse(b["tof"]["available"])

    def test_ds_tof_is_exact_integer(self):
        """DS: responder ToF == (RA*k*RB - DA*k*DB)/(RA + k*RB + DA + k*DB)."""
        output, req_path, out_path = self.run_scenario("ds_unity")
        self.assertTrue(output["status"]["ok"])
        self.assertEqual(output["protocol"], "ds")

        b = self._endpoint(output, "b")
        self.assertEqual(b["completion"], "complete")
        self.assertTrue(b["estimate_available"])
        resp = self._frame(b, "response", "tx")
        fin = self._frame(b, "final", "rx")
        t2b = int(self._ts(resp, "t2B"))
        t3b = int(self._ts(resp, "t3B"))
        t6 = int(self._local(b, "rx_marker_ticks", "final"))
        ra = F(int(self._ts(fin, "t4A")) - int(self._ts(fin, "t1A")))
        da = F(int(self._ts(fin, "t5A")) - int(self._ts(fin, "t4A")))
        db = F(t3b - t2b)
        rb = F(t6 - t3b)
        expected = (ra * rb - da * db) / (ra + rb + da + db)
        self.assertEqual(expected, F(100))
        tof = b["tof"]
        self.assertEqual(F(int(tof["num"]), int(tof["den"])), expected)
        self.assertEqual(int(tof["num"]), 100)

        a = self._endpoint(output, "a")
        self.assertFalse(a["estimate_available"])
        self.assertFalse(a["tof"]["available"])

    def test_nominal_rate_ratio_exercises_k(self):
        """SS across two nominal clock rates: k = fA/fB = 5/4, not 1."""
        output, _, _ = self.run_scenario("ss_nominal")
        self.assertTrue(output["status"]["ok"])
        a = self._endpoint(output, "a")
        self.assertTrue(a["estimate_available"])
        resp = self._frame(a, "response", "rx")
        t1 = int(self._local(a, "tx_planned_ticks", "poll"))
        t4 = int(self._local(a, "rx_marker_ticks", "response"))
        db = F(int(self._ts(resp, "t3B")) - int(self._ts(resp, "t2B")))
        k = F(1000000000, 800000000)
        expected = (F(t4 - t1) - k * db) / 2
        self.assertEqual(expected, F(100))
        self.assertEqual(F(int(a["tof"]["num"]), int(a["tof"]["den"])), expected)
        # If k had been dropped, (RA-DB)/2 would not be 100.
        self.assertNotEqual((F(t4 - t1) - db) / 2, expected)

    def test_deliberately_wrong_config_is_rejected(self):
        request = D.build_request("ss_unity")
        # A.local must equal B.peer; break it.
        request["endpoints"]["b"]["local_address"] = 999
        rc, output = self.run_request(request, "bad_addr")
        self.assertNotEqual(rc, 0, "a wrong config must be rejected")
        self.assertIsNotNone(output)
        self.assertFalse(output["status"]["ok"])
        self.assertIn("addressing", output["status"]["error"])

    def test_bad_twr_config_is_rejected(self):
        request = D.build_request("ss_unity")
        request["twr_config"] = {}  # empty: the existing validator rejects it
        rc, output = self.run_request(request, "bad_twr_config")
        self.assertNotEqual(rc, 0)
        self.assertFalse(output["status"]["ok"])
        self.assertIn("twr_config", output["status"]["error"])

    def test_non_simulation_envelope_is_rejected(self):
        request = D.build_request("ss_unity")
        request["simulation"]["execution"] = "hardware"
        rc, output = self.run_request(request, "bad_sim")
        self.assertNotEqual(rc, 0, "a hardware claim must be refused")
        self.assertFalse(output["status"]["ok"])

    def test_protocol_status_is_never_a_ranging_success(self):
        """The central M1-B guard: a completed protocol exchange is NOT a range."""
        for scenario in ("ss_unity", "ds_unity"):
            output, _, _ = self.run_scenario(scenario)
            self.assertFalse(output["provenance"]["measurement_valid"])
            self.assertEqual(output["provenance"]["peer_evidence"], "wire_claim")
            self.assertEqual(output["provenance"]["execution_mode"], "simulation")
            for name in ("a", "b"):
                ep = self._endpoint(output, name)
                self.assertFalse(ep["measurement_valid"],
                                 "%s.%s must not be a valid measurement"
                                 % (scenario, name))
                self.assertEqual(ep["peer_evidence"], "wire_claim")
                self.assertEqual(ep["execution_mode"], "simulation")
                self.assertFalse(ep["yields_range"])
                self.assertFalse(ep["is_hardware_measurement"])
                self.assertFalse(ep["is_validated_measurement"])
                self.assertNotIn("distance", ep)
                self.assertNotIn("distance_m", ep)
            # A "complete" protocol exchange may still own no estimate, and an
            # owned estimate is still not a measurement.
            self.assertEqual(self._endpoint(output, "a")["completion"], "complete")

    def test_same_seed_is_deterministic(self):
        output1, _, _ = self.run_scenario("ds_unity")
        output2, _, _ = self.run_scenario("ds_unity")
        self.assertEqual(output1["output_sha256"], output2["output_sha256"])
        self.assertEqual(json.dumps(output1, sort_keys=True),
                         json.dumps(output2, sort_keys=True))

    # -- R09: the TwrConfig is an execution authority, not a discarded check --

    @staticmethod
    def _canonical(fields):
        """Reproduce the C++ demo's executed-field canonical text.

        The output publishes ``configuration.executed_fields`` as path/value
        pairs and ``configuration.canonical_layout`` says exactly how they are
        joined, so this is an INDEPENDENT recomputation, not a copy of a number
        the C++ side handed out.
        """
        return "".join(f["path"] + "=" + f["value"] + "\n" for f in fields)

    def _config_run(self, name, request):
        rc, output = self.run_request(request, name)
        if rc != 0:
            self.fail("demo refused %s: rc=%s output=%s" % (name, rc, output))
        return output

    def test_full_legal_twr_config_runs_end_to_end(self):
        """A COMPLETE requested TwrConfig is validated and then EXECUTED."""
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable: %s" % D.TWR_CONFIG_IMPORT_ERROR)
        twr_config = D.build_twr_config(protocol="ss")
        request = D.build_request("ss_unity", twr_config=twr_config)
        output = self._config_run("full_config", request)

        self.assertTrue(output["status"]["ok"])
        self.assertEqual(output["protocol"], "ss")
        self.assertEqual(self._endpoint(output, "a")["completion"], "complete")

        cfg = output["configuration"]
        self.assertEqual(cfg["source"], "twr_config")
        self.assertEqual(cfg["requested_config_hash"],
                         cfg["effective_config_hash"])
        self.assertTrue(cfg["requested_config_hash"].startswith("fnv1a64:"))

        # The hashes are over the ACTUAL executed values and are independently
        # reproducible from what the output itself publishes.
        self.assertEqual(
            hashlib.sha256(self._canonical(cfg["executed_fields"]).encode()).hexdigest(),
            output["config_sha256"])
        self.assertEqual(
            hashlib.sha256(self._canonical(cfg["profile_fields"]).encode()).hexdigest(),
            output["profile_sha256"])

        # The mapping must show the config drove the session fields.
        by_field = {m["field"]: m for m in cfg["mapping"]}
        for field in ("session.protocol", "session.local_address",
                      "session.session_id", "session.pan_id",
                      "session.sequence_modulus", "session.sequence",
                      "diagnostics.result_queue_capacity"):
            self.assertIn(field, by_field)
            self.assertEqual(by_field[field]["source"], "twr_config", field)
            self.assertTrue(by_field[field]["consumed"], field)
        # event_queue_capacity is stated but NOT consumed offline, with a limit.
        self.assertFalse(by_field["diagnostics.event_queue_capacity"]["consumed"])
        self.assertIn("queue", by_field["diagnostics.event_queue_capacity"]["note"])

        # Every unconsumed section is named with its use and limit.
        self.assertGreaterEqual(len(cfg["unconsumed"]), 8)
        for row in cfg["unconsumed"]:
            self.assertTrue(row["field"])
            self.assertTrue(row["use"])
            self.assertTrue(row["limit"])

    def test_r08_numeric_plan_is_verifiable_and_matches_the_frame(self):
        """The plan's internal identity and the frame's t3B/t5A agree."""
        twr_config = D.build_twr_config(protocol="ds")
        request = D.build_request("ds_unity", twr_config=twr_config)
        output = self._config_run("plan", request)

        for name in ("a", "b"):
            ep = self._endpoint(output, name)
            tx_frames = [f for f in ep["frames"] if f["dir"] == "tx"]
            self.assertTrue(tx_frames, name)
            for fr in tx_frames:
                plan = fr["plan"]
                self.assertTrue(plan["valid"])
                self.assertTrue(plan["internally_consistent"])
                self.assertTrue(plan["identity_holds"])
                q = int(plan["quantised_instant_ticks"])
                off = int(plan["marker_offset_ticks"])
                air = int(plan["calibrated_air_ticks"])
                self.assertEqual(q + off, air)
                self.assertGreaterEqual(q, 0)
                self.assertGreaterEqual(off, 0)
                self.assertLessEqual(int(plan["command_time_ticks"]), q)
                # The frame's t3B/t5A carry the calibrated air instant.
                ts = fr["fields"]["timestamps"]
                for fld in ("t3B", "t5A"):
                    if fld in ts:
                        self.assertEqual(int(ts[fld]), air)

    def test_config_protocol_change_changes_trace(self):
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        ss = self._config_run(
            "proto_ss",
            D.build_request("ss_unity", twr_config=D.build_twr_config(protocol="ss")))
        ds = self._config_run(
            "proto_ds",
            D.build_request("ds_unity", twr_config=D.build_twr_config(protocol="ds")))
        self.assertEqual(ss["protocol"], "ss")
        self.assertEqual(ds["protocol"], "ds")
        # DS has a Final frame at B; SS does not.
        self.assertTrue(any(f["type"] == "final" for f in
                            self._endpoint(ds, "b")["frames"]))
        self.assertFalse(any(f["type"] == "final" for f in
                             self._endpoint(ss, "b")["frames"]))
        self.assertNotEqual(ss["config_sha256"], ds["config_sha256"])

    def test_config_address_change_changes_frames_and_trace(self):
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        local, peer = 300, 700
        twr_config = D.build_twr_config(local_address=local, peer_address=peer)
        request = D.build_request("ss_unity", twr_config=twr_config)
        request["endpoints"] = {
            "a": {"local_address": local, "peer_address": peer},
            "b": {"local_address": peer, "peer_address": local},
        }
        output = self._config_run("addr", request)

        poll = self._frame(self._endpoint(output, "a"), "poll", "tx")
        self.assertEqual(poll["fields"]["src_addr"], local)
        self.assertEqual(poll["fields"]["dst_addr"], peer)
        by_field = {m["field"]: m for m in output["configuration"]["mapping"]}
        self.assertEqual(by_field["session.local_address"]["requested"], str(local))
        self.assertIn(str(local), by_field["session.local_address"]["core"])

        base = self._config_run(
            "addr_base",
            D.build_request("ss_unity", twr_config=D.build_twr_config()))
        self.assertNotEqual(output["config_sha256"], base["config_sha256"])

    def test_config_initial_sequence_changes_frame_seq(self):
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        for seq in (0, 2):
            twr_config = D.build_twr_config(sequence=seq, sequence_modulus=4)
            request = D.build_request("ss_unity", twr_config=twr_config)
            request["session"]["initial_sequence"] = seq
            request["session"]["sequence_modulus"] = 4
            output = self._config_run("seq%d" % seq, request)
            poll = self._frame(self._endpoint(output, "a"), "poll", "tx")
            self.assertEqual(poll["fields"]["seq"], seq)
            by_field = {m["field"]: m for m in output["configuration"]["mapping"]}
            self.assertEqual(by_field["session.sequence"]["requested"], str(seq))

    def test_config_capacity_and_timeout_changes_are_observable(self):
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        # Capacity: only in the config; the executed snapshot must move.
        cap1 = self._config_run(
            "cap1", D.build_request(
                "ss_unity",
                twr_config=D.build_twr_config(result_queue_capacity=1)))
        cap3 = self._config_run(
            "cap3", D.build_request(
                "ss_unity",
                twr_config=D.build_twr_config(result_queue_capacity=3)))
        map1 = {m["field"]: m for m in cap1["configuration"]["mapping"]}
        map3 = {m["field"]: m for m in cap3["configuration"]["mapping"]}
        self.assertEqual(map1["diagnostics.result_queue_capacity"]["core"], "1")
        self.assertEqual(map3["diagnostics.result_queue_capacity"]["core"], "3")
        self.assertNotEqual(cap1["config_sha256"], cap3["config_sha256"])

        # Timeout: the whole-exchange deadline the core handed the driver moves.
        long_ = self._config_run(
            "to_long", D.build_request(
                "ss_unity",
                twr_config=D.build_twr_config(exchange_timeout_ns=50_000_000)))
        short = self._config_run(
            "to_short", D.build_request(
                "ss_unity",
                twr_config=D.build_twr_config(exchange_timeout_ns=5_000_000)))
        d_long = int(self._endpoint(long_, "a")["arm_rx"][0]["deadline_ticks"])
        d_short = int(self._endpoint(short, "a")["arm_rx"][0]["deadline_ticks"])
        self.assertEqual(d_long, 50_000_000)
        self.assertEqual(d_short, 5_000_000)
        self.assertLess(d_short, d_long)

    def test_malformed_full_config_is_rejected(self):
        """A structurally bad config is refused by the existing validator."""
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        bad = D.build_twr_config()
        bad["session"]["session_id"] = 70000  # does not fit the 16-bit wire field
        rc, output = self.run_request(
            D.build_request("ss_unity", twr_config=bad), "bad_session_id")
        self.assertNotEqual(rc, 0)
        self.assertFalse(output["status"]["ok"])
        self.assertIn("twr_config", output["status"]["error"])

        unknown = D.build_twr_config()
        unknown["session"]["no_such_field"] = 1
        rc, output = self.run_request(
            D.build_request("ss_unity", twr_config=unknown), "unknown_key")
        self.assertNotEqual(rc, 0)
        self.assertFalse(output["status"]["ok"])
        self.assertIn("unknown_key", output["status"]["error"])

    def test_envelope_config_conflict_is_rejected(self):
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        ds = D.build_twr_config(protocol="ds")
        request = D.build_request("ss_unity", twr_config=ds)
        rc, output = self.run_request(request, "conflict")
        self.assertNotEqual(rc, 0, "a config/envelope conflict must be refused")
        self.assertFalse(output["status"]["ok"])
        self.assertIn("conflict", output["status"]["error"])

    def test_output_mutation_is_detectable(self):
        """A verifier recomputing the published canonical text catches a mutation."""
        if D.T is None:
            self.skipTest("uwb.twr_config unavailable")
        output = self._config_run(
            "mutation",
            D.build_request("ss_unity", twr_config=D.build_twr_config()))
        cfg = output["configuration"]
        clean = hashlib.sha256(self._canonical(cfg["executed_fields"]).encode()).hexdigest()
        self.assertEqual(clean, output["config_sha256"])

        mutated = copy.deepcopy(cfg["executed_fields"])
        mutated[len(mutated) // 2]["value"] = "0xDEADBEEF"
        dirty = hashlib.sha256(self._canonical(mutated).encode()).hexdigest()
        self.assertNotEqual(dirty, output["config_sha256"],
                            "a mutated executed field must change the hash")

    def test_binary_does_not_link_gnuradio_or_uhd(self):
        ldd = shutil.which("ldd")
        if ldd is None:
            self.skipTest("ldd not available")
        proc = subprocess.run([ldd, _BINARY], capture_output=True, text=True)
        if proc.returncode != 0:
            self.skipTest("ldd failed on %s" % _BINARY)
        lowered = proc.stdout.lower()
        for forbidden in ("gnuradio", "uhd"):
            self.assertNotIn(forbidden, lowered,
                             "%s links %s:\n%s" % (_BINARY, forbidden, proc.stdout))
        # And the SOURCE must not pull a GNU Radio block/UHD header.
        with open(os.path.join(_HERE, "twr_fake_demo.cc"), "r",
                  encoding="utf-8") as handle:
            source = handle.read()
        for forbidden in ("gnuradio/block", "gnuradio/uhd", "uhd/usrp",
                          "gnuradio/pmt", "gnuradio/runtime"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    if not _BINARY:
        sys.stderr.write("SKIP: %s\n" % _BINARY_NOTE)
    unittest.main(verbosity=2)
