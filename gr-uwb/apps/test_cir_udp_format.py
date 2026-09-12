#!/usr/bin/env python3
"""Unit tests for the CIR UDP datagram wire format (UCR1/UCR2/raw taps).

Run:
    python3 gr-uwb/apps/test_cir_udp_format.py
"""
from __future__ import annotations

import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cir_udp_recv as rx  # noqa: E402


def _taps(n):
    return np.arange(n, dtype=np.float32) + 1j * (np.arange(n, dtype=np.float32) + 0.5)


class UdpFormatTest(unittest.TestCase):
    def test_ucr2_roundtrip_with_frequency(self):
        taps = _taps(4).astype(np.complex64)
        hdr = rx.HDR_V2.pack(
            rx.MAGIC_V2, 7, 0, 116, 0.31, 0.42, 30, 1234,
            6494.6e6, 5.0e6)
        rec = rx.parse_datagram(hdr + taps.tobytes())
        self.assertTrue(rec["framed"])
        self.assertEqual(rec["version"], 2)
        self.assertEqual(rec["pulse_id"], 7)
        self.assertEqual(rec["status"], "ok")
        self.assertEqual(rec["tap_count"], 116)
        self.assertEqual(rec["peak_tap"], 30)
        self.assertAlmostEqual(rec["freq_hz"], 6494.6e6)
        self.assertAlmostEqual(rec["freq_offset_hz"], 5.0e6)
        np.testing.assert_allclose(rec["taps"], taps)
        self.assertEqual(rx.HDR_V2.size, 44)

    def test_ucr1_has_no_frequency(self):
        taps = _taps(3).astype(np.complex64)
        hdr = rx.HDR.pack(rx.MAGIC, 1, 3, 116, 0.0, 0.01, 22, 99)
        rec = rx.parse_datagram(hdr + taps.tobytes())
        self.assertEqual(rec["version"], 1)
        self.assertIsNone(rec["freq_hz"])
        self.assertIsNone(rec["freq_offset_hz"])
        self.assertEqual(rec["status"], "cir_failed")
        np.testing.assert_allclose(rec["taps"], taps)
        self.assertEqual(rx.HDR.size, 28)

    def test_raw_taps(self):
        taps = _taps(5).astype(np.complex64)
        rec = rx.parse_datagram(taps.tobytes())
        self.assertFalse(rec["framed"])
        self.assertEqual(rec["version"], 0)
        self.assertEqual(rec["taps"].size, 5)
        np.testing.assert_allclose(rec["taps"], taps)

    def test_ucr2_freq_keeps_khz_precision(self):
        # f32 at 6.5 GHz cannot resolve 1 kHz; f64 must.
        hdr = rx.HDR_V2.pack(rx.MAGIC_V2, 0, 0, 0, 0.0, 0.0, 0, 0,
                             6489.6e6 + 1.0e3, 1.0e3)
        rec = rx.parse_datagram(hdr + b"")
        self.assertEqual(rec["freq_hz"] - 6489.6e6, 1.0e3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
