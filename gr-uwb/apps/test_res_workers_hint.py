#!/usr/bin/env python3
"""Unit tests for --res-workers accounting in the echo CIR app.

The app itself imports GNU Radio and UHD, so this test never imports it:
it extracts ``NativeRateProfile.make_pdu_resampler`` and the ``main()``
res-worker lines with ``ast`` (same pattern as ``test_jam_app_args.py``).
The extracted method runs against a stub ``uwb`` module and a capturing
``print`` so the WARN / worker-collapse behaviour is exercised without the
C++ bindings.  It runs on a bare host.

Run:
    python3 gr-uwb/apps/test_res_workers_hint.py
"""
from __future__ import annotations

import ast
import os
import re
import unittest


APP_NAME = "x410_cg400_hrp_echo_cir.py"

WARN_TEXT = ("WARN --res-workers %d is ignored for PDU 65/48 "
             "(no worker pool; 65/32 only); continuing with 1 worker")


def _app_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), APP_NAME)


def _source():
    with open(_app_path(), "r", encoding="utf-8") as f:
        return f.read()


class _StubPolicy:
    FullWindow = "FullWindow"


class _StubUwb:
    """Stand-in for the C++ uwb bindings, recording constructor args."""

    pdu_resampler_emit_policy = _StubPolicy()

    def __init__(self):
        self.calls = []
        self.prints = []

    def pdu_rational_resampler_ccf_65_48(self, *args):
        self.calls.append(("65_48", args))
        return "blk_65_48"

    def pdu_rational_resampler_ccf_65_32(self, *args):
        self.calls.append(("65_32", args))
        return "blk_65_32"


def _load_make_pdu_resampler(stub, prints):
    """Extract make_pdu_resampler; capture its print output into *prints*."""
    path = _app_path()
    tree = ast.parse(_source(), path)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "NativeRateProfile":
            for method in node.body:
                if (isinstance(method, ast.FunctionDef)
                        and method.name == "make_pdu_resampler"):
                    module = ast.Module(body=[method], type_ignores=[])
                    ast.fix_missing_locations(module)
                    ns = {"uwb": stub, "print": _capture_print(prints),
                          "WORK_HZ": 998400000.0}
                    exec(compile(module, path, "exec"), ns)
                    return ns["make_pdu_resampler"]
    raise AssertionError("missing NativeRateProfile.make_pdu_resampler")


def _capture_print(sink):
    def _print(*args, flush=False, **_kw):
        sink.append(" ".join(str(a) for a in args))
    return _print


class MakePduResamplerWorkerHintTest(unittest.TestCase):
    """make_pdu_resampler collapses 65/48 to 1 effective worker and WARNs."""

    @classmethod
    def setUpClass(cls):
        cls.prints, cls.uwb_stub = [], _StubUwb()
        cls.make = staticmethod(  # staticmethod: self is NOT the profile
            _load_make_pdu_resampler(cls.uwb_stub, cls.prints))

    def _profile(self, pdu):
        """Minimal NativeRateProfile stand-in exposing just .pdu."""
        return type("MiniProfile", (), {"pdu": pdu})()

    def test_65_48_warns_and_continues(self):
        if self.uwb_stub is None:
            self.fail("stub missing")
        self.prints.clear()
        n0 = len(self.uwb_stub.calls)
        blk = self.make(self._profile("65_48"), "taps", 4, "scale")
        self.assertEqual(blk, "blk_65_48")
        # Still builds the 65/48 PDU block, not a worker pool.
        self.assertEqual(
            [c[0] for c in self.uwb_stub.calls[n0:]], ["65_48"])
        # Exactly one explicit WARN line naming the requested count.
        warns = [ln for ln in self.prints
                 if ln.startswith("WARN --res-workers")]
        self.assertEqual(warns, [WARN_TEXT % 4])

    def test_65_48_no_warn_at_one_worker(self):
        self.prints.clear()
        n0 = len(self.uwb_stub.calls)
        blk = self.make(self._profile("65_48"), "taps", 1, "scale")
        self.assertEqual(blk, "blk_65_48")
        self.assertEqual([c[0] for c in self.uwb_stub.calls[n0:]], ["65_48"])
        self.assertEqual(self.prints, [])

    def test_65_32_passes_workers_through(self):
        self.prints.clear()
        n0 = len(self.uwb_stub.calls)
        blk = self.make(self._profile("65_32"), "taps", 4, "scale")
        self.assertEqual(blk, "blk_65_32")
        self.assertEqual(
            [c[0] for c in self.uwb_stub.calls[n0:]], ["65_32"])
        # No WARN on the 65/32 path; workers go to the call as-is.
        self.assertEqual(self.prints, [])
        # workers is the 5th positional of the 65/32 call (taps, WORK_HZ,
        # True, max_input_samples, workers, sc16_scale) -> args[4].
        self.assertEqual(self.uwb_stub.calls[n0][1][4], 4)


class ResWorkerReportingTest(unittest.TestCase):
    """The main() worker accounting must reflect the 65/48 collapse."""

    def test_effective_worker_count_from_profile(self):
        src = _source().replace("\n", " ")
        self.assertRegex(
            src, r"res_workers_effective\s*=\s*\(?\s*1 if profile\.pdu == "
            r"\"65_48\" else int\(a\.res_workers\)\s*\)?")

    def test_requested_value_kept_separately(self):
        self.assertIn("res_workers_requested = int(a.res_workers)",
                      _source())

    def test_summary_reports_effective_and_requested(self):
        src = _source()
        self.assertIn('"res_workers": res_workers_effective,', src)
        self.assertIn('"res_workers_requested": res_workers_requested,', src)
        # The old silent reporting of the raw CLI value must be gone.
        self.assertNotIn('"res_workers": int(a.res_workers)', src)
        self.assertNotIn('int(a.res_workers), a.res_sc16_scale',
                         _source())

    def test_log_line_reports_effective_workers(self):
        src = _source().replace("\n", " ")
        m = re.search(r"print\(\"resampler pdu=%s workers=%d sc16_scale=%s\""
                      r" % \(\s*profile\.pdu,\s*([^,]+?),", src)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1).strip(), "res_workers_effective")
        # The ignored-request case gets an explicit follow-up note.
        self.assertIn("resampler res_workers_requested=%d "
                      "(ignored; effective=1)", _source())

    def test_warn_line_shape_in_source(self):
        src = _source().replace("\n", " ")
        # The two literal halves sit on separate lines in the app source;
        # match each half, then the joined runtime line shape.
        self.assertIn('WARN --res-workers %d is ignored for PDU 65/48 "', src)
        self.assertIn('"(no worker pool; 65/32 only); '
                      'continuing with 1 worker"', src)
        # The hint must come from make_pdu_resampler's >1 guard, i.e. it
        # fires on the effective runtime state, not on every log line.
        self.assertIn("if req_workers > 1:", _source())


if __name__ == "__main__":
    unittest.main()
