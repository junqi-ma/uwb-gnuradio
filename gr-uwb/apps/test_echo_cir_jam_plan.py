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


class MakeFreqOffsetScanTest(unittest.TestCase):
    def test_descending_includes_stop(self):
        # 491 kHz -> 0, 300 Hz: last grid point is 200 Hz, stop 0 is appended.
        out = jp.make_freq_offset_scan(491e3, 0.0, 300.0)
        self.assertEqual(out[0], 491000.0)
        self.assertEqual(out[1], 490700.0)
        self.assertEqual(out[-2], 200.0)
        self.assertEqual(out[-1], 0.0)
        self.assertEqual(len(out), 1638)

    def test_ascending_on_grid(self):
        out = jp.make_freq_offset_scan(0.0, 900.0, 300.0)
        self.assertEqual(out, [0.0, 300.0, 600.0, 900.0])

    def test_single_point(self):
        self.assertEqual(jp.make_freq_offset_scan(491e3, 491e3, 300.0),
                         [491e3])

    def test_zero_step_rejected(self):
        with self.assertRaises(ValueError):
            jp.make_freq_offset_scan(0.0, 1.0, 0.0)


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
        self.assertEqual(jp.placement_native(-1.0, 1e9), -1000)
        self.assertEqual(jp.placement_native(-0.5, 1e9), -500)

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

    def test_delay_random_ok(self):
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_delay_random_us=1.018)))
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_delay_random_us=0.0)))
        self.assertIsNone(
            jp.validate_jam_args(good_args(jam_delay_random_us=None)))

    def test_delay_random_bad(self):
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(jam_delay_random_us=-0.1))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(
                good_args(jam_delay_random_us=jp._MAX_DELAY_RANDOM_US + 1))
        with self.assertRaises(ValueError):
            jp.validate_jam_args(good_args(jam_delay_random_us=float("nan")))


class ParseDelayRandomTest(unittest.TestCase):
    def test_empty(self):
        self.assertIsNone(jp.parse_delay_random_us(""))
        self.assertIsNone(jp.parse_delay_random_us(None))
        self.assertIsNone(jp.parse_delay_random_us("  "))

    def test_single_is_plus_minus_span(self):
        self.assertEqual(jp.parse_delay_random_us("1.018"), 1.018)
        self.assertEqual(jp.parse_delay_random_us("5e-1"), 0.5)
        self.assertEqual(jp.parse_delay_random_us("0"), 0.0)

    def test_rejects_pair(self):
        with self.assertRaises(ValueError) as cm:
            jp.parse_delay_random_us("0,1.018")
        self.assertIn("single", str(cm.exception))

    def test_rejects_negative(self):
        with self.assertRaises(ValueError):
            jp.parse_delay_random_us("-1")

    def test_rejects_over_cap(self):
        with self.assertRaises(ValueError) as cm:
            jp.parse_delay_random_us("3000")
        self.assertIn("2000", str(cm.exception))

    def test_non_finite(self):
        with self.assertRaises(ValueError):
            jp.parse_delay_random_us("nan")
        with self.assertRaises(ValueError):
            jp.parse_delay_random_us("inf")


class DelayHalfSpanNativeTest(unittest.TestCase):
    def test_1us_at_1ghz(self):
        self.assertEqual(jp.delay_half_span_native(1.0, 1e9), 1000)

    def test_sync_period_737p28(self):
        # 1.018 us @ 737.28 MS/s ≈ 751 native samples (1016 * 48/65).
        self.assertEqual(jp.delay_half_span_native(1.018, 737.28e6), 751)

    def test_zero(self):
        self.assertEqual(jp.delay_half_span_native(0.0, 737.28e6), 0)


class DrawDelayNativeTest(unittest.TestCase):
    def test_degenerate(self):
        rng = np.random.default_rng(0)
        self.assertEqual(jp.draw_delay_native(rng, 7, 7), 7)

    def test_inclusive_and_reproducible(self):
        a = [jp.draw_delay_native(np.random.default_rng(1), -5, 5)
             for _ in range(20)]
        b = [jp.draw_delay_native(np.random.default_rng(1), -5, 5)
             for _ in range(20)]
        self.assertEqual(a, b)
        rng = np.random.default_rng(2)
        draws = [jp.draw_delay_native(rng, -5, 5) for _ in range(8000)]
        self.assertEqual(min(draws), -5)
        self.assertEqual(max(draws), 5)
        self.assertEqual(len(set(draws)), 11)

    def test_bad_span(self):
        rng = np.random.default_rng(0)
        with self.assertRaises(ValueError):
            jp.draw_delay_native(rng, 5, 4)


class BipolarTxLenTest(unittest.TestCase):
    def test_no_jam(self):
        self.assertEqual(jp.bipolar_tx_len(16, 0, 5), 16)

    def test_fits_both_ends(self):
        # sense=10 at D=4 -> ends 14; jam=3 at 2D=8 -> ends 11; L=14.
        self.assertEqual(jp.bipolar_tx_len(10, 3, 4), 14)
        # jam longer: sense=4 at D=4 -> 8; jam=10 at 8 -> 18.
        self.assertEqual(jp.bipolar_tx_len(4, 10, 4), 18)


class ComposeAtTest(unittest.TestCase):
    def test_sense_parked_jam_leads(self):
        sense = np.array([1 + 0j, 2 + 0j], dtype=np.complex64)
        jam = np.array([9 + 9j], dtype=np.complex64)
        out = jp.compose_tx_native_at(sense, jam, 2, 1, 4)
        self.assertEqual(out.shape, (2, 4))
        self.assertTrue(np.all(out[0, :2] == 0))
        self.assertTrue(np.array_equal(out[0, 2:4], sense))
        self.assertEqual(out[1, 1], jam[0])
        self.assertTrue(np.all(out[1, [0, 2, 3]] == 0))


class PlaceJamRowTest(unittest.TestCase):
    def test_places_and_clears_previous(self):
        sense = np.array([1 + 0j, 2 + 0j, 3 + 0j, 4 + 0j], dtype=np.complex64)
        jam = np.array([9 + 9j, 8 + 8j], dtype=np.complex64)
        payload = jp.compose_tx_native(sense, jam, 2)
        jp.place_jam_row(payload, jam, 0)
        self.assertTrue(np.array_equal(payload[0], sense))
        self.assertTrue(np.array_equal(payload[1, 0:2], jam))
        self.assertTrue(np.all(payload[1, 2:] == 0))

    def test_empty_jam_zeros_row(self):
        payload = np.ones((2, 4), dtype=np.complex64)
        jp.place_jam_row(payload, None, 0)
        self.assertTrue(np.all(payload[1] == 0))
        self.assertTrue(np.all(payload[0] == 1))

    def test_overflow_rejected(self):
        payload = np.zeros((2, 4), dtype=np.complex64)
        with self.assertRaises(ValueError):
            jp.place_jam_row(payload, np.ones(3, dtype=np.complex64), 2)

    def test_bad_shape(self):
        with self.assertRaises(ValueError):
            jp.place_jam_row(np.zeros(4, dtype=np.complex64), [1], 0)


class BuildCppScheduleMetaTest(unittest.TestCase):
    def test_keys_exact(self):
        meta = jp.build_cpp_schedule_meta(
            sense_len=100, jam_len=50, native_hz=737.28e6)
        self.assertEqual(tuple(meta.keys()), jp.CPP_SCHEDULE_META_KEYS)
        self.assertEqual(jp.CPP_JAM_DELAY_MODES, ("fixed", "uniform"))

    def test_fixed_zero_delay(self):
        meta = jp.build_cpp_schedule_meta(
            sense_len=1000, jam_len=500, native_hz=737.28e6,
            delay_us=0.0, delay_seed=0, jam_offsets_hz=[],
            jam_dwell=50, freq_settle_s=0.05)
        self.assertEqual(meta["tx_channel_count"], 2)
        # L = max(1000, 500 + 0).
        self.assertEqual(meta["tx_samples"], 1000)
        self.assertEqual(meta["tx_waveform_samples"], [1000, 500])
        self.assertEqual(meta["tx_base_offsets_native"], [0, 0])
        self.assertEqual(meta["jam_logical_channel"], 1)
        self.assertEqual(meta["jam_delay_mode"], "fixed")
        self.assertEqual(meta["jam_delay_lo_native"], 0)
        self.assertEqual(meta["jam_delay_hi_native"], 0)
        self.assertEqual(meta["jam_delay_seed"], 0)
        self.assertEqual(meta["jam_freq_offsets_hz"], [])
        self.assertEqual(meta["jam_dwell"], 50)
        # 0.05 s @ 737.28 MS/s, converted to integer ticks in Python.
        self.assertEqual(meta["jam_freq_settle_ticks"], 36864000)

    def test_fixed_positive_delay_matches_composite(self):
        meta = jp.build_cpp_schedule_meta(
            sense_len=1000, jam_len=500, native_hz=737.28e6,
            delay_us=1.0)
        delay_n = jp.placement_native(1.0, 737.28e6)
        self.assertEqual(delay_n, 737)
        self.assertEqual(meta["tx_samples"],
                         jp.combined_tx_len(1000, 500, delay_n))
        self.assertEqual(meta["tx_base_offsets_native"], [0, delay_n])
        self.assertEqual(
            (meta["jam_delay_lo_native"], meta["jam_delay_hi_native"]),
            (delay_n, delay_n))

    def test_uniform_parks_sense_at_D(self):
        meta = jp.build_cpp_schedule_meta(
            sense_len=1000, jam_len=500, native_hz=737.28e6,
            delay_random_us=1.018, delay_seed=7,
            jam_offsets_hz=[0.0, 245670.0], jam_dwell=20,
            freq_settle_s=0.05)
        d = jp.delay_half_span_native(1.018, 737.28e6)
        self.assertEqual(d, 751)
        self.assertEqual(meta["jam_delay_mode"], "uniform")
        self.assertEqual(meta["tx_samples"],
                         jp.bipolar_tx_len(1000, 500, d))
        self.assertEqual(meta["tx_base_offsets_native"], [d, d])
        self.assertEqual(meta["jam_delay_lo_native"], -d)
        self.assertEqual(meta["jam_delay_hi_native"], d)
        self.assertEqual(meta["jam_delay_seed"], 7)
        self.assertEqual(meta["jam_freq_offsets_hz"], [0.0, 245670.0])
        self.assertEqual(meta["jam_dwell"], 20)

    def test_zero_span_is_uniform(self):
        meta = jp.build_cpp_schedule_meta(
            sense_len=10, jam_len=4, native_hz=1e9, delay_random_us=0.0)
        self.assertEqual(meta["jam_delay_mode"], "uniform")
        self.assertEqual(
            (meta["jam_delay_lo_native"], meta["jam_delay_hi_native"]),
            (0, 0))
        self.assertEqual(meta["tx_base_offsets_native"], [0, 0])

    def test_json_serialisable(self):
        import json
        meta = jp.build_cpp_schedule_meta(
            sense_len=100, jam_len=50, native_hz=737.28e6,
            delay_random_us=1.018, delay_seed=3,
            jam_offsets_hz=[0.0], jam_dwell=1)
        json.dumps(meta)

    def test_bad_lengths(self):
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=0, jam_len=10, native_hz=1e9)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=0, native_hz=1e9)

    def test_bad_rate_and_span(self):
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=0.0)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9,
                delay_random_us=-1.0)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9,
                delay_random_us=3000.0)

    def test_bad_seed_dwell_settle(self):
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9, delay_seed=-1)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9, jam_dwell=-1)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9, freq_settle_s=-0.1)
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9,
                jam_offsets_hz=[float("nan")])

    def test_negative_fixed_delay_rejected_like_composite(self):
        with self.assertRaises(ValueError):
            jp.build_cpp_schedule_meta(
                sense_len=10, jam_len=4, native_hz=1e9, delay_us=-1.0)


class ContiguousWindowLayoutTest(unittest.TestCase):
    def test_uniform_backing_and_windows(self):
        lay = jp.contiguous_window_layout(
            sense_len=1000, jam_len=500, native_hz=737.28e6,
            delay_random_us=1.018)
        d = jp.delay_half_span_native(1.018, 737.28e6)
        L = jp.bipolar_tx_len(1000, 500, d)
        self.assertEqual(lay["layout"], "contiguous-window")
        self.assertEqual(lay["L"], L)
        self.assertEqual(lay["D"], d)
        self.assertEqual(lay["backing_length"], L + 2 * d)
        self.assertEqual(lay["jam_window_begin_min"], 0)
        self.assertEqual(lay["jam_window_begin_max"], 2 * d)
        self.assertEqual(lay["data_fragments"], 1)

    def test_fixed_no_backing_pad(self):
        lay = jp.contiguous_window_layout(
            sense_len=1000, jam_len=500, native_hz=737.28e6, delay_us=1.0)
        delay_n = jp.placement_native(1.0, 737.28e6)
        L = jp.combined_tx_len(1000, 500, delay_n)
        self.assertEqual(lay["layout"], "contiguous-window")
        self.assertEqual(lay["L"], L)
        self.assertEqual(lay["D"], 0)
        self.assertEqual(lay["backing_length"], L)
        self.assertEqual(lay["jam_window_begin_min"], delay_n)
        self.assertEqual(lay["jam_window_begin_max"], delay_n)
        self.assertEqual(lay["data_fragments"], 1)

    def test_fragment_must_cover_L(self):
        jp.require_fragment_covers_L(262144, 189003)
        with self.assertRaises(ValueError) as ctx:
            jp.require_fragment_covers_L(65536, 189003)
        self.assertIn("max_fragment_size", str(ctx.exception))
        self.assertIn("contiguous-window", str(ctx.exception))


class JamScaleSc16ZeroTest(unittest.TestCase):
    def test_jam_scale_1e_8_is_digital_zero(self):
        jam = np.exp(1j * np.linspace(0, 6.0, 256)).astype(np.complex64)
        jam_peak = float(np.max(np.abs(jam))) or 1.0
        sense_peak = 0.8
        scaled = (jam / jam_peak * (1e-8 * sense_peak)).astype(np.complex64)
        sc16 = jp.fc32_to_sc16(scaled)
        self.assertEqual(sc16.dtype, np.int16)
        self.assertTrue(np.all(sc16 == 0))

    def test_unity_scale_is_nonzero(self):
        jam = np.ones(32, dtype=np.complex64)
        sc16 = jp.fc32_to_sc16(jam)
        self.assertTrue(np.any(sc16 != 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
