#!/usr/bin/env python3
"""Unit tests for the jammer app's pure parsing / static contract.

The app itself imports GNU Radio and UHD, so this test never imports it:
it extracts ``parse_freq_offsets`` from the source with ``ast`` (same pattern
as ``test_echo_stream_buffers.py``) and checks the CLI surface textually.  It
runs on a bare host.

Run:
    python3 gr-uwb/apps/test_jam_app_args.py
"""
from __future__ import annotations

import ast
import os
import unittest


APP_NAME = "x410_cg400_hrp_echo_cir_jam.py"
PLAN_NAME = "echo_cir_jam_plan.py"

REQUIRED_OPTIONS = (
    "--jam-enable",
    "--jam-mode",
    "--jam-channel",
    "--jam-antenna",
    "--jam-gain-tx",
    "--jam-freq-offset",
    "--jam-freq-offsets",
    "--jam-dwell",
    "--jam-code-index",
    "--jam-preamble-length",
    "--jam-psdu-hex",
    "--jam-waveform",
    "--jam-pulse-shape",
    "--jam-delay-us",
    "--jam-scale",
    "--jam-repeat-pri-us",
    "--jam-freq-settle-s",
    "--dry-run",
)


def _app_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), APP_NAME)


def _source():
    with open(_app_path(), "r", encoding="utf-8") as f:
        return f.read()


def _load_functions(*names):
    """Compile the named module-level functions from the app source."""
    path = _app_path()
    tree = ast.parse(_source(), path)
    wanted = set(names)
    found = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            found.append(node)
            wanted.discard(node.name)
    if wanted:
        raise AssertionError("missing function(s) %s in %s"
                             % (sorted(wanted), path))
    module = ast.Module(body=found, type_ignores=[])
    ast.fix_missing_locations(module)
    ns = {}
    exec(compile(module, path, "exec"), ns)
    return {name: ns[name] for name in names}


class ParseFreqOffsetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # staticmethod avoids the implicit self binding on class attributes.
        cls.fn = staticmethod(
            _load_functions("parse_freq_offsets")["parse_freq_offsets"])

    def test_empty(self):
        self.assertEqual(self.fn(""), [])
        self.assertEqual(self.fn(None), [])

    def test_single(self):
        self.assertEqual(self.fn("491340"), [491340.0])

    def test_scientific_and_units(self):
        self.assertEqual(self.fn("0,245.67e3,491.34e3"),
                         [0.0, 245670.0, 491340.0])

    def test_whitespace_and_trailing_comma(self):
        self.assertEqual(self.fn(" 0 , 100e3 ,"), [0.0, 100000.0])

    def test_negative(self):
        self.assertEqual(self.fn("-491.34e3"), [-491340.0])

    def test_bad_token_raises(self):
        with self.assertRaises(ValueError):
            self.fn("0,abc")


class StaticContractTest(unittest.TestCase):
    def test_all_jam_options_declared(self):
        src = _source()
        for opt in REQUIRED_OPTIONS:
            self.assertIn('"%s"' % opt, src,
                          "missing CLI option %s" % opt)

    def test_rejects_cpp_pdu(self):
        src = _source()
        self.assertIn("cpp-pdu", src)
        self.assertIn("cannot carry", src)

    def test_forces_python_backend(self):
        src = _source()
        self.assertIn('a.echo_backend = "python"', src)

    def test_uses_two_tx_channels(self):
        src = _source()
        self.assertIn("tx_channels=tx_channels", src)
        self.assertIn("[int(tx_ch), int(jam_ch)]", src)

    def test_sends_composite_payload(self):
        src = _source()
        self.assertIn("jp.compose_tx_native(", src)
        self.assertIn("self._tx_payload = jp.compose_tx_native(", src)

    def test_plan_module_is_pure(self):
        plan = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            PLAN_NAME)
        with open(plan, "r", encoding="utf-8") as f:
            plan_src = f.read()
        for banned in ("import gnuradio", "import uhd", "import pmt"):
            self.assertNotIn(banned, plan_src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
