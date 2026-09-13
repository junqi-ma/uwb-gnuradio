#!/usr/bin/env python3
"""Unit tests for the stream app's buffer-size helper.

Pure Python on purpose: importing ``x410_cg400_hrp_echo_cir_stream`` would
pull in GNU Radio / numpy / UHD (and re-exec the UHD bootstrap), so the test
extracts ``buffer_plan`` (and ``apply_buffer_plan``) from the app source with
the ``ast`` module and executes only those functions.  This keeps the test
runnable on a bare host while still checking the real code in the app.

Run:
    python3 gr-uwb/apps/test_echo_stream_buffers.py
"""
from __future__ import annotations

import ast
import os
import unittest


APP_NAME = "x410_cg400_hrp_echo_cir_stream.py"


def _load_functions(*names):
    """Compile the named module-level functions from the app source."""
    app_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            APP_NAME)
    with open(app_path, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read(), app_path)

    wanted = {n for n in names}
    found = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            found.append(node)
            wanted.discard(node.name)
    if wanted:
        raise AssertionError("missing function(s) %s in %s"
                             % (sorted(wanted), app_path))

    module = ast.Module(body=found, type_ignores=[])
    ast.fix_missing_locations(module)
    ns = {}
    exec(compile(module, app_path, "exec"), ns)
    return app_path, {name: ns[name] for name in names}


class _FakeBlock:
    """Minimal set_min_output_buffer recorder for apply_buffer_plan."""

    def __init__(self):
        self.min_output = {}

    def set_min_output_buffer(self, port, items):
        self.min_output[port] = int(items)


class BufferPlanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, funcs = _load_functions("buffer_plan", "apply_buffer_plan")
        cls.buffer_plan = staticmethod(funcs["buffer_plan"])
        cls.apply_buffer_plan = staticmethod(funcs["apply_buffer_plan"])

    def test_known_numbers(self):
        # Reference geometry from the stream app: tx native length and the
        # 7th return value of base.rx_geometry for a preamble-64 window.
        plan = self.buffer_plan(94492, 109288)
        self.assertEqual(plan["tx_ts_out"], 188984)   # 2 * tx
        self.assertEqual(plan["echo_out"], 437152)    # 4 * rx
        self.assertEqual(plan["res_in"], 218576)      # 2 * rx
        self.assertEqual(plan["bridge_in"], 221992)   # ceil(rx * 65 / 32)
        self.assertEqual(plan["res_out"], 443984)     # 2 * work
        self.assertEqual(plan["res_out"], 2 * plan["bridge_in"])

    def test_work_formula_matches_ceil(self):
        for rx in (1, 2, 32, 33, 1016, 4096, 109288, 141300, 999999):
            plan = self.buffer_plan(1000, rx)
            expected = -(-(rx * 65) // 32)  # integer ceil
            self.assertEqual(plan["bridge_in"], expected,
                             "rx=%d" % rx)
            self.assertEqual(plan["res_out"], 2 * expected, "rx=%d" % rx)

    def test_common_windows(self):
        # preamble 64: ~109288 native; preamble 128: ~141300 native.
        self.assertEqual(self.buffer_plan(94492, 109288)["bridge_in"], 221992)
        self.assertEqual(self.buffer_plan(94492, 141300)["bridge_in"],
                         -(-(141300 * 65) // 32))

    def test_exact_division(self):
        # 32 native samples map to exactly 65 work samples.
        plan = self.buffer_plan(32, 32)
        self.assertEqual(plan["bridge_in"], 65)
        self.assertEqual(plan["res_out"], 130)

    def test_zero(self):
        plan = self.buffer_plan(0, 0)
        self.assertEqual(plan, {"tx_ts_out": 0, "echo_out": 0, "res_in": 0,
                                "res_out": 0, "bridge_in": 0})

    def test_shared_buffer_relations(self):
        plan = self.buffer_plan(94492, 109288)
        # A connection has one buffer: the receiver's minimum must not exceed
        # the sender's, and echo_out/res_out are the values the app applies.
        self.assertGreaterEqual(plan["echo_out"], plan["res_in"])
        self.assertGreaterEqual(plan["res_out"], plan["bridge_in"])

    def test_apply_uses_max_for_shared_buffers(self):
        plan = self.buffer_plan(94492, 109288)
        tx_ts, echo, res = _FakeBlock(), _FakeBlock(), _FakeBlock()
        applied = self.apply_buffer_plan(tx_ts, echo, res, plan)
        # tx_ts output feeds the echo input.
        self.assertEqual(tx_ts.min_output[0], plan["tx_ts_out"])
        # echo output and resampler input share one buffer -> larger wins.
        self.assertEqual(echo.min_output[0],
                         max(plan["echo_out"], plan["res_in"]))
        self.assertEqual(echo.min_output[0], plan["echo_out"])
        # resampler output and bridge input share one buffer.
        self.assertEqual(res.min_output[0],
                         max(plan["res_out"], plan["bridge_in"]))
        self.assertEqual(res.min_output[0], plan["res_out"])
        self.assertEqual(applied["tx_ts_out"], plan["tx_ts_out"])
        self.assertEqual(applied["echo_out"], plan["echo_out"])
        self.assertEqual(applied["res_out"], plan["res_out"])

    def test_applies_no_smaller_than_plan(self):
        plan = self.buffer_plan(94492, 109288)
        for key in ("tx_ts_out", "echo_out", "res_in", "res_out", "bridge_in"):
            self.assertGreaterEqual(plan[key], 0)
        self.assertLess(plan["tx_ts_out"], plan["echo_out"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
