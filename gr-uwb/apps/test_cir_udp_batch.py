#!/usr/bin/env python3
"""Regression test for packed repetition PDU -> per-record UCR4 fanout."""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np
import pmt

os.environ.setdefault("UWB_UHD_BOOTSTRAPPED", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cir_udp_recv as rx  # noqa: E402
import x410_cg400_hrp_echo_cir as app  # noqa: E402


class _FakeSocket:
    def __init__(self):
        self.datagrams = []

    def sendto(self, data, _destination):
        self.datagrams.append(data)


class CirUdpBatchTest(unittest.TestCase):
    def test_batch_fans_out_in_repetition_order(self):
        # Bypass the GNU Radio block/socket constructor: _on_cir only needs
        # this small state surface and the fake captures exact wire bytes.
        sink = object.__new__(app.CirUdpSink)
        sink.tap_count = 4
        sink.freq_lookup = None
        sink._dst = ("test", 0)
        sink._sock = _FakeSocket()
        sink.sent = sink.sent_ok = sink.sent_fail = 0
        sink.sent_freq = sink.dropped = 0

        meta = pmt.make_dict()
        fields = (
            ("repetition_batch", pmt.PMT_T),
            ("pulse_id", pmt.from_uint64(7)),
            ("tap_count", pmt.from_uint64(4)),
            ("repetition_first", pmt.from_uint64(5)),
            ("repetition_count", pmt.from_uint64(3)),
            ("repetition_status_code", pmt.init_u64vector(3, [0, 3, 0])),
            ("repetition_peak_tap", pmt.init_u64vector(3, [1, 0, 2])),
            ("repetition_peak_metric", pmt.init_f32vector(3, [0.1, 0, 0.2])),
        )
        for key, value in fields:
            meta = pmt.dict_add(meta, pmt.intern(key), value)
        taps = (np.arange(12, dtype=np.float32) + 1j).astype(np.complex64)
        sink._on_cir(pmt.cons(meta, pmt.init_c32vector(12, taps.tolist())))

        records = [rx.parse_datagram(d) for d in sink._sock.datagrams]
        self.assertEqual([r["repetition_index"] for r in records], [5, 6, 7])
        self.assertEqual([r["repetition_count"] for r in records], [3, 3, 3])
        self.assertEqual([r["status"] for r in records],
                         ["ok", "cir_failed", "ok"])
        self.assertEqual((sink.sent, sink.sent_ok, sink.sent_fail, sink.dropped),
                         (3, 2, 1, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
