#!/usr/bin/env python3
"""Unit tests for multi-channel TX support in ``TimedUhdEcho``.

Pure Python / dependency-light on purpose: importing
``x410_cg400_hrp_echo_cir`` would pull in GNU Radio, UHD and trigger the UHD
bootstrap re-exec, so most checks read the app source and assert its structure
(optionally via ``ast``).  A small behavioural test extracts the real ``_send``
shape logic and drives it with fake UHD objects, so no radio is touched.

Run:
    python3 gr-uwb/apps/test_echo_tx_channels.py
"""
from __future__ import annotations

import ast
import os
import unittest

try:  # numpy is present on any host that can run the app; keep it optional.
    import numpy as np
except ImportError:  # pragma: no cover - bare host without numpy
    np = None


APP_NAME = "x410_cg400_hrp_echo_cir.py"
CLASS_NAME = "TimedUhdEcho"


def _read_app_source():
    app_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            APP_NAME)
    with open(app_path, "r", encoding="utf-8") as f:
        return app_path, f.read()


def _method_sources():
    """Return {method_name: source} for the app's TimedUhdEcho class."""
    app_path, source = _read_app_source()
    tree = ast.parse(source, app_path)
    cls = None
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == CLASS_NAME:
            cls = node
            break
    if cls is None:
        raise AssertionError("class %s not found in %s" % (CLASS_NAME, app_path))
    out = {}
    for node in cls.body:
        if isinstance(node, ast.FunctionDef):
            seg = ast.get_source_segment(source, node)
            if seg is None:
                raise AssertionError("cannot get source for %s" % node.name)
            out[node.name] = seg
    return app_path, source, out


class TimedUhdEchoStructureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app_path, cls.source, cls.methods = _method_sources()

    def test_init_takes_and_stores_tx_channels(self):
        init = self.methods["__init__"]
        self.assertIn("tx_channels", init,
                      "__init__ must accept a tx_channels parameter")
        self.assertIn("self.tx_channels", init,
                      "__init__ must assign self.tx_channels")
        # The new keyword must default to None so the sweep subclass and all
        # existing positional callers keep working.
        tree = ast.parse(_read_app_source()[1], self.app_path)
        init_node = None
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and node.name == CLASS_NAME:
                for m in node.body:
                    if isinstance(m, ast.FunctionDef) and m.name == "__init__":
                        init_node = m
        args = init_node.args
        names = [a.arg for a in args.args]
        defaults = [None] * (len(names) - len(args.defaults)) + list(args.defaults)
        pairs = dict(zip(names, defaults))
        self.assertIn("tx_channels", pairs)
        self.assertIsNone(ast.literal_eval(pairs["tx_channels"]))
        self.assertLess(names.index("rx_freq_offset"), names.index("tx_channels"))

    def test_send_handles_1d_and_2d_and_slices_columns(self):
        send = self.methods["_send"]
        self.assertIn("buf.ndim == 1", send)
        self.assertIn("buf.ndim != 2", send)
        self.assertIn("buf[:, sent", send)
        # sample count must be per-channel, not the flattened element count
        self.assertIn("buf.shape[1]", send)

    def test_set_tx_native_geometry_and_keep_payload(self):
        stn = self.methods["set_tx_native"]
        self.assertIn("keep_payload", stn)
        self.assertIn("self.tx_geometry_native", stn)
        self.assertIn("geom_native", stn)
        self.assertIn("self._tx_payload = self._native", stn)

    def test_one_burst_sends_tx_payload(self):
        burst = self.methods["_one_burst"]
        self.assertIn("self._send(self._tx_payload", burst)

    def test_tx_payload_defaults_to_native(self):
        # The single-channel path must still transmit self._native.
        self.assertIn("self._tx_payload = self._native", self.source)

    def test_status_reports_tx_channels(self):
        self.assertIn('"tx_channels": self.tx_channels', self.methods["__init__"])


class _FakeMD:
    def __init__(self):
        self.has_time_spec = False
        self.start_of_burst = False
        self.end_of_burst = False
        self.time_spec = None


class _FakeTypes:
    TXMetadata = _FakeMD


class _FakeUhd:
    types = _FakeTypes


class _FakeTxStream:
    def __init__(self, maxp):
        self.maxp = int(maxp)
        self.sent = []

    def get_max_num_samps(self):
        return self.maxp

    def send(self, buf, md, timeout):
        self.sent.append(np.array(buf, copy=True))
        return buf.shape[1]


class _FakeEcho:
    def __init__(self, maxp):
        self._uhd = _FakeUhd
        self._tx_stream = _FakeTxStream(maxp)

    def _tspec(self, seconds):
        return seconds


def _extract_send():
    """Compile the real ``_send`` method as a standalone function."""
    app_path, source = _read_app_source()
    tree = ast.parse(source, app_path)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == CLASS_NAME:
            for m in node.body:
                if isinstance(m, ast.FunctionDef) and m.name == "_send":
                    fndef = m
                    break
    fn = ast.FunctionDef(
        name="_send", args=fndef.args, body=fndef.body,
        decorator_list=[], returns=None, type_comment=None)
    module = ast.Module(body=[fn], type_ignores=[])
    ast.fix_missing_locations(module)
    ns = {"np": np}
    exec(compile(module, app_path, "exec"), ns)
    return ns["_send"]


@unittest.skipIf(np is None, "numpy not available")
class SendSlicingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.send = staticmethod(_extract_send())

    def _run(self, wave, maxp):
        echo = _FakeEcho(maxp)
        sent, problems = self.send(echo, wave, 0.0, 1.0)
        return echo._tx_stream.sent, sent, problems

    def test_1d_single_channel_roundtrip(self):
        wave = np.arange(10, dtype=np.complex64)
        chunks, sent, problems = self._run(wave, 4)
        self.assertEqual(problems, "")
        self.assertEqual(sent, 10)
        for c in chunks:
            self.assertEqual(c.ndim, 2)
            self.assertEqual(c.shape[0], 1)
            self.assertLessEqual(c.shape[1], 4)
        self.assertTrue(np.array_equal(np.concatenate(chunks, axis=1)[0], wave))

    def test_2d_two_channel_roundtrip(self):
        wave = (np.arange(20, dtype=np.complex64).reshape(2, 10))
        chunks, sent, problems = self._run(wave, 3)
        self.assertEqual(problems, "")
        self.assertEqual(sent, 10)  # per-channel sample count, not 20
        for c in chunks:
            self.assertEqual(c.shape[0], 2)
            self.assertLessEqual(c.shape[1], 3)
        merged = np.concatenate(chunks, axis=1)
        self.assertTrue(np.array_equal(merged, wave))

    def test_bad_ndim_raises(self):
        wave = np.zeros((2, 2, 2), dtype=np.complex64)
        with self.assertRaises(ValueError):
            self._run(wave, 4)

    def test_short_send_is_accounted(self):
        class _ShortTxStream(_FakeTxStream):
            def send(self, buf, md, timeout):
                half = max(1, buf.shape[1] // 2)
                self.sent.append(np.array(buf[:, :half], copy=True))
                return half

        echo = _FakeEcho(4)
        echo._tx_stream = _ShortTxStream(4)
        wave = np.ones(9, dtype=np.complex64)
        sent, problems = self.send(echo, wave, 0.0, 1.0)
        self.assertEqual(sent, 9)
        self.assertIn("short_send", problems)


if __name__ == "__main__":
    unittest.main(verbosity=2)
