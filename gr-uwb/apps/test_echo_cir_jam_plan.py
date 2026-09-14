#!/usr/bin/env python3
"""Unit tests for echo_cir_jam_plan (pure Python; numpy only).

Run:
    python3 gr-uwb/apps/test_echo_cir_jam_plan.py
"""
from __future__ import annotations

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import echo_cir_jam_plan as jp  # noqa: E402


def good_args(**overrides):
    """A fully valid, jam-enabled config; override individual keys."""
    args = {
        "jam_enabled": True,
        "jam_mode": "align",
        "jam_channel": jp.DEFAULT_JAM_CHANNEL,
        "tx_channel": 0,
        "rx_channel": 2,
        "jam_freq_offset_hz": 0.0,
        "jam_code_index": jp.DEFAULT_JAM_CODE_INDEX,
        "jam_scale": 1.0,
        "jam_repeat_pri_us": 5000.0,
    }
    args.update(overrides)
    return args


class ConstantsTest(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(jp.JAM_MODES, ("off", "align", "continuous"))
        self.assertEqual(jp.DEFAULT_JAM_CODE_INDEX, 10)
        self.assertEqual(jp.DEFAULT_JAM_CHANNEL, 1)


class JamFreqTest(unittest.TestCase):
    def test_sum(self):
        self.assertEqual(jp.jam_freq_hz(6489.6e6, 0.0), 6489.6e6)
        self.assertEqual(jp.jam_freq_hz(6489.6e6, 1e6), 6490.6e6)
        self.assertEqual(jp.jam_freq_hz(6489.6e6, -1e6), 6488.6e6)
        self.assertIsInstance(jp.jam_freq_hz(1, 2), float)

    def test_negative_offset(self):
        self.assertAlmostEqual(jp.jam_freq_hz(100.0, -42.5), 57.5)

    def test_non_finite(self):
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.assertRaises(ValueError):
                jp.jam_freq_hz(bad, 0.0)
            with self.assertRaises(ValueError):
                jp.jam_freq_hz(0.0, bad)

    def test_bad_type(self):
        with self.assertRaises(ValueError):
            jp.jam_freq_hz("nope", 0.0)


class PlacementTest(unittest.TestCase):
    def test_delay_zero(self):
        self.assertEqual(jp.placement_native(0.0, 1e9), 0)

    def test_round_trip(self):
        # 1 us at 1 GHz native = 1000 samples.
        self.assertEqual(jp.placement_native(1.0, 1e9), 1000)
        self.assertEqual(jp.placement_native(0.5, 1e9), 500)

    def test_half_away_from_zero(self):
        # 0.5, 1.5, 2.5 native samples round up.
        self.assertEqual(jp.placement_native(1.0, 500e3), 1)   # 0.5 -> 1
        self.assertEqual(jp.placement_native(3.0, 500e3), 2)   # 1.5 -> 2
        self.assertEqual(jp.placement_native(5.0, 500e3), 3)   # 2.5 -> 3

    def test_negative_delay(self):
        with self.assertRaises(ValueError):
            jp.placement_native(-0.001, 1e9)

    def test_non_positive_native(self):
        with self.assertRaises(ValueError):
            jp.placement_native(1.0, 0.0)
        with self.assertRaises(ValueError):
            jp.placement_native(1.0, -1.0)

    def test_non_finite(self):
        with self.assertRaises(ValueError):
            jp.placement_native(float("nan"), 1e9)
        with self.assertRaises(ValueError):
            jp.placement_native(1.0, float("inf"))


class CombinedLenTest(unittest.TestCase):
    def test_no_jam(self):
        self.assertEqual(jp.combined_tx_len(16, 0, 0), 16)
        self.assertEqual(jp.combined_tx_len(16, 0, 5), 16)
        self.assertEqual(jp.combined_tx_len(16, -3, 5), 16)

    def test_jam_shorter(self):
        # jam (12) + delay (2) = 14 < sense 16.
        self.assertEqual(jp.combined_tx_len(16, 12, 2), 16)

    def test_jam_longer(self):
        self.assertEqual(jp.combined_tx_len(4, 10, 2), 12)

    def test_jam_extends_past_sense(self):
        self.assertEqual(jp.combined_tx_len(10, 4, 8), 12)

    def test_invalid(self):
        with self.assertRaises(ValueError):
            jp.combined_tx_len(0, 1, 0)
        with self.assertRaises(ValueError):
            jp.combined_tx_len(-1, 1, 0)
        with self.assertRaises(ValueError):
            jp.combined_tx_len(4, 1, -1)


class ComposeTest(unittest.TestCase):
    def test_shape_dtype(self):
        out = jp.compose_tx_native([1 + 1j, 2 + 0j], [3 + 0j], 3)
        self.assertEqual(out.shape, (2, 4))
        self.assertEqual(out.dtype, np.complex64)

    def test_exact_placement(self):
        sense = np.array([1 + 0j, 2 + 0j, 3 + 0j], dtype=np.complex64)
        jam = np.array([9 + 9j, 8 + 8j], dtype=np.complex64)
        delay = 2
        out = jp.compose_tx_native(sense, jam, delay)
        L = jp.combined_tx_len(len(sense), len(jam), delay)
        self.assertEqual(out.shape, (2, L))
        self.assertTrue(np.array_equal(out[0, :len(sense)], sense))
        self.assertTrue(np.all(out[0, len(sense):] == 0))
        self.assertTrue(np.array_equal(out[1, delay:delay + len(jam)], jam))
        # Explicit zeros before/after the jammer.
        self.assertTrue(np.all(out[1, :delay] == 0))
        self.assertTrue(np.all(out[1, delay + len(jam):] == 0))

    def test_delay_zero(self):
        sense = np.array([1 + 0j, 2 + 0j], dtype=np.complex64)
        jam = np.array([7 + 0j], dtype=np.complex64)
        out = jp.compose_tx_native(sense, jam, 0)
        self.assertTrue(np.array_equal(out[1, 0:1], jam))
        self.assertTrue(np.array_equal(out[0], sense))

    def test_jam_longer_than_sense(self):
        sense = np.array([1 + 0j, 2 + 0j], dtype=np.complex64)
        jam = np.arange(1, 6).astype(np.complex64)
        out = jp.compose_tx_native(sense, jam, 1)
        self.assertEqual(out.shape, (2, 6))
        self.assertTrue(np.array_equal(out[1, 1:6], jam))
        self.assertTrue(np.all(out[0, 2:] == 0))

    def test_jam_extends_past_sense(self):
        sense = np.arange(1, 11).astype(np.complex64)
        jam = np.array([1 + 0j, 2 + 0j, 3 + 0j, 4 + 0j], dtype=np.complex64)
        out = jp.compose_tx_native(sense, jam, 8)
        self.assertEqual(out.shape, (2, 12))
        self.assertTrue(np.array_equal(out[1, 8:12], jam))
        self.assertTrue(np.all(out[0, 10:12] == 0))

    def test_jam_empty(self):
        sense = np.array([1 + 0j, 2 + 0j], dtype=np.complex64)
        for empty in (None, [], np.array([], dtype=np.complex64)):
            out = jp.compose_tx_native(sense, empty, 0)
            self.assertEqual(out.shape, (2, 2))
            self.assertTrue(np.all(out[1] == 0))
            self.assertTrue(np.array_equal(out[0], sense))

    def test_inputs_not_mutated(self):
        sense = np.array([1 + 1j, 2 + 2j], dtype=np.complex64)
        jam = np.array([5 + 5j], dtype=np.complex64)
        sense_copy = sense.copy()
        jam_copy = jam.copy()
        jp.compose_tx_native(sense, jam, 2)
        self.assertTrue(np.array_equal(sense, sense_copy))
        self.assertTrue(np.array_equal(jam, jam_copy))

    def test_sense_invalid(self):
        with self.assertRaises(ValueError):
            jp.compose_tx_native([], [1 + 0j], 0)
        with self.assertRaises(ValueError):
            jp.compose_tx_native(np.zeros((2, 2), dtype=np.complex64),
                                 [1 + 0j], 0)

    def test_negative_delay(self):
        with self.assertRaises(ValueError):
            jp.compose_tx_native([1 + 0j], [1 + 0j], -1)


class ValidateTest(unittest.TestCase):
    def test_success_align(self):
        self.assertIsNone(jp.validate_jam_args(good_args()))
        self.assertIsNone(jp.validate_jam_args(good_args(jam_mode="off")))
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_mode="continuous")))

    def test_disabled_short_circuits(self):
        # Everything else invalid, but jam_enabled is falsy -> no error.
        self.assertIsNone(jp.validate_jam_args({"jam_enabled": False}))
        self.assertIsNone(jp.validate_jam_args({}))
        self.assertIsNone(jp.validate_jam_args({"jam_enabled": 0}))

    def test_bad_mode(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(good_args(jam_mode="bogus"))
        self.assertIn("bogus", str(cm.exception))

    def test_channel_not_int(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(good_args(jam_channel=1.0))
        self.assertIn("1.0", str(cm.exception))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(tx_channel="0"))

    def test_channel_out_of_range(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(good_args(jam_channel=4))
        self.assertIn("jam_channel", str(cm.exception))
        self.assertIn("4", str(cm.exception))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(tx_channel=-1))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(rx_channel=9))

    def test_channel_collision(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(good_args(jam_channel=0, tx_channel=0))
        self.assertIn("jam_channel", str(cm.exception))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(jam_channel=2, rx_channel=2))

    def test_bad_freq_offset(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(good_args(jam_freq_offset_hz=1e6 + 1))
        self.assertIn("jam_freq_offset_hz", str(cm.exception))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(jam_freq_offset_hz=-2e6))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(jam_freq_offset_hz=float("nan")))
        # Boundary +/-1e6 is allowed.
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_freq_offset_hz=1e6)))
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_freq_offset_hz=-1e6)))

    def test_bad_code_index(self):
        for bad in (8, 13, 0, None):
            with self.assertRaises(ValueError) as cm:
                jp.validate_jam_args(good_args(jam_code_index=bad))
            self.assertIn("jam_code_index", str(cm.exception))
        for ok in (9, 10, 11, 12):
            self.assertIsNone(
                jp.validate_jam_args(good_args(jam_code_index=ok)))

    def test_bad_scale(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                jp.validate_jam_args(good_args(jam_scale=bad))

    def test_continuous_requires_positive_pri(self):
        with self.assertRaises(ValueError) as cm:
            jp.validate_jam_args(
                good_args(jam_mode="continuous", jam_repeat_pri_us=0.0))
        self.assertIn("jam_repeat_pri_us", str(cm.exception))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(
                good_args(jam_mode="continuous", jam_repeat_pri_us=-1.0))
        # A missing/None PRI also fails for continuous.
        with self.assertRaises(ValueError):
            jp.validate_jam_args(
                good_args(jam_mode="continuous", jam_repeat_pri_us=None))
        # PRI is irrelevant for align.
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_repeat_pri_us=0.0)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
