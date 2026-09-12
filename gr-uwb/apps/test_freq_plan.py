#!/usr/bin/env python3
"""Unit tests for echo_cir_freq_plan (no GNU Radio / UHD / numpy needed).

Run:
    python3 gr-uwb/apps/test_freq_plan.py
"""
from __future__ import annotations

import json
import os
import queue
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import echo_cir_freq_plan as fp  # noqa: E402


NOMINAL = 6489.6e6


class ParseValueTest(unittest.TestCase):
    def test_units(self):
        self.assertAlmostEqual(fp.parse_freq_value("6489.6MHz"), 6.4896e9)
        self.assertAlmostEqual(fp.parse_freq_value("6.4896GHz"), 6.4896e9)
        self.assertAlmostEqual(fp.parse_freq_value("500kHz"), 5.0e5)
        self.assertAlmostEqual(fp.parse_freq_value("123Hz"), 123.0)

    def test_bare_defaults_to_khz(self):
        # Bare numbers (no suffix, no exponent) use the default unit = kHz.
        self.assertAlmostEqual(fp.parse_freq_value("+50"), 50.0e3)
        self.assertAlmostEqual(fp.parse_freq_value("491"), 491.0e3)
        # Scientific notation is always Hz.
        self.assertAlmostEqual(fp.parse_freq_value("10e6"), 10.0e6)
        self.assertAlmostEqual(fp.parse_freq_value("-10e6"), -10.0e6)
        self.assertAlmostEqual(fp.parse_freq_value("500e3"), 5.0e5)

    def test_default_unit_override(self):
        self.assertAlmostEqual(fp.parse_freq_value("5", "mhz"), 5.0e6)
        self.assertAlmostEqual(fp.parse_freq_value("5", "hz"), 5.0)
        self.assertAlmostEqual(fp.parse_freq_value("6489.6", "mhz"), 6.4896e9)
        with self.assertRaises(ValueError):
            fp.parse_freq_value("5", "bogus")

    def test_bad(self):
        for bad in ["", "abc", "MHz", "1.2.3MHz", "10GHzx"]:
            with self.assertRaises(ValueError):
                fp.parse_freq_value(bad)


class ParseCommandTest(unittest.TestCase):
    def test_stop_status(self):
        self.assertEqual(fp.parse_freq_command("q", NOMINAL), ("stop", None))
        self.assertEqual(fp.parse_freq_command("quit", NOMINAL),
                         ("stop", None))
        self.assertEqual(fp.parse_freq_command("", NOMINAL),
                         ("status", None))
        self.assertEqual(fp.parse_freq_command("?", NOMINAL),
                         ("status", None))

    def test_absolute_and_delta(self):
        self.assertEqual(fp.parse_freq_command("6489.6MHz", NOMINAL),
                         ("set", 6.4896e9))
        self.assertEqual(fp.parse_freq_command("f 6.5e9", NOMINAL),
                         ("set", 6.5e9))
        self.assertEqual(fp.parse_freq_command("+5MHz", NOMINAL),
                         ("delta", 5.0e6))
        self.assertEqual(fp.parse_freq_command("-2.5MHz", NOMINAL),
                         ("delta", -2.5e6))
        self.assertEqual(fp.parse_freq_command("off +1MHz", NOMINAL),
                         ("delta", 1.0e6))
        self.assertEqual(fp.parse_freq_command("offset -1MHz", NOMINAL),
                         ("delta", -1.0e6))
        # Bare delta uses the default unit (kHz) unless overridden.
        self.assertEqual(fp.parse_freq_command("+50", NOMINAL),
                         ("delta", 50.0e3))
        self.assertEqual(fp.parse_freq_command("+50", NOMINAL, "mhz"),
                         ("delta", 50.0e6))

    def test_bad(self):
        with self.assertRaises(ValueError):
            fp.parse_freq_command("nonsense", NOMINAL)


class FixedPlanTest(unittest.TestCase):
    def test_fixed(self):
        plan = fp.FreqPlan("fixed", NOMINAL)
        self.assertEqual(plan.n_points, 1)
        self.assertIsNone(plan.total_pulses())
        for pid in (0, 7, 100000):
            self.assertEqual(plan.freq_for(pid), NOMINAL)
        self.assertEqual(plan.dwell_index(5), 0)


class ScanPlanTest(unittest.TestCase):
    def make(self, **kw):
        args = dict(mode="scan", nominal_hz=NOMINAL, start_hz=NOMINAL - 2e6,
                    stop_hz=NOMINAL + 2e6, step_hz=1e6, dwell=3, once=True)
        args.update(kw)
        return fp.FreqPlan(**args)

    def test_points_and_total(self):
        plan = self.make()
        self.assertEqual(plan.n_points, 5)
        self.assertEqual(plan.total_pulses(), 15)
        self.assertEqual(plan.freqs[0], NOMINAL - 2e6)
        self.assertEqual(plan.freqs[-1], NOMINAL + 2e6)

    def test_dwell_boundaries(self):
        plan = self.make()
        for pid in range(0, 3):
            self.assertEqual(plan.freq_for(pid), NOMINAL - 2e6)
        for pid in range(3, 6):
            self.assertEqual(plan.freq_for(pid), NOMINAL - 1e6)
        for pid in range(12, 15):
            self.assertEqual(plan.freq_for(pid), NOMINAL + 2e6)
        self.assertEqual(plan.dwell_index(7), 2)

    def test_once_clamps(self):
        plan = self.make()
        self.assertEqual(plan.freq_for(10**6), plan.freqs[-1])

    def test_cycle_wraps(self):
        plan = self.make(once=False)
        self.assertIsNone(plan.total_pulses())
        self.assertEqual(plan.freq_for(15), plan.freqs[0])
        self.assertEqual(plan.freq_for(18), plan.freqs[1])

    def test_step_not_exact(self):
        plan = fp.FreqPlan("scan", 100.0, start_hz=100.0, stop_hz=110.0,
                           step_hz=3.0, dwell=1)
        self.assertEqual(plan.freqs, [100.0, 103.0, 106.0, 109.0])

    def test_inclusive_stop(self):
        plan = fp.FreqPlan("scan", 100.0, start_hz=100.0, stop_hz=110.0,
                           step_hz=5.0, dwell=1)
        self.assertEqual(plan.freqs, [100.0, 105.0, 110.0])

    def test_missing_stop_or_step(self):
        with self.assertRaises(SystemExit):
            fp.FreqPlan("scan", NOMINAL, start_hz=NOMINAL, stop_hz=None,
                        step_hz=1e6)
        with self.assertRaises(SystemExit):
            fp.FreqPlan("scan", NOMINAL, start_hz=NOMINAL, stop_hz=NOMINAL,
                        step_hz=0.0)
        with self.assertRaises(SystemExit):
            fp.FreqPlan("scan", NOMINAL, start_hz=NOMINAL + 1e6,
                        stop_hz=NOMINAL, step_hz=1e6)


class ManualPlanTest(unittest.TestCase):
    def test_manual_holds_and_updates(self):
        q = queue.Queue()
        plan = fp.FreqPlan("manual", NOMINAL, manual_q=q)
        self.assertEqual(plan.freq_for(0), NOMINAL)
        q.put("100MHz")
        self.assertEqual(plan.freq_for(1), 100.0e6)
        # holds across pulses until a new command arrives
        self.assertEqual(plan.freq_for(2), 100.0e6)
        q.put("+5MHz")
        self.assertEqual(plan.freq_for(3), NOMINAL + 5e6)
        self.assertEqual(plan.freq_for(4), NOMINAL + 5e6)

    def test_manual_bad_and_stop(self):
        q = queue.Queue()
        plan = fp.FreqPlan("manual", NOMINAL, manual_q=q)
        q.put("not-a-freq")
        self.assertEqual(plan.freq_for(0), NOMINAL)  # ignored
        q.put("q")
        plan.freq_for(1)
        self.assertTrue(plan.stop_requested)

    def test_manual_freq_unit(self):
        q = queue.Queue()
        plan = fp.FreqPlan("manual", NOMINAL, manual_q=q, freq_unit="khz")
        q.put("+50")
        self.assertAlmostEqual(plan.freq_for(0), NOMINAL + 50.0e3)

        q2 = queue.Queue()
        plan2 = fp.FreqPlan("manual", NOMINAL, manual_q=q2, freq_unit="mhz")
        q2.put("+50")
        self.assertAlmostEqual(plan2.freq_for(0), NOMINAL + 50.0e6)

    def test_manual_invalid_unit(self):
        with self.assertRaises(SystemExit):
            fp.FreqPlan("manual", NOMINAL, freq_unit="bogus")


class AnalyzeTest(unittest.TestCase):
    def test_group_by_freq(self):
        with tempfile.TemporaryDirectory() as d:
            cir = os.path.join(d, "cir.jsonl")
            recs = [
                {"pulse_id": 0, "status": "ok", "peak_tap": 10,
                 "cir_peak_metric": 0.5},
                {"pulse_id": 1, "status": "ok", "peak_tap": 12,
                 "cir_peak_metric": 0.7},
                {"pulse_id": 2, "status": "sfd_failed"},
                {"pulse_id": 3, "status": "ok", "peak_tap": 11,
                 "cir_peak_metric": 0.6},
            ]
            with open(cir, "w", encoding="utf-8") as f:
                for r in recs:
                    f.write(json.dumps(r) + "\n")
            freq_records = [
                {"pulse_id": 0, "freq_hz": 6489.6e6,
                 "freq_offset_hz": 0.0},
                {"pulse_id": 1, "freq_hz": 6490.6e6,
                 "freq_offset_hz": 1e6},
                {"pulse_id": 2, "freq_hz": 6489.6e6,
                 "freq_offset_hz": 0.0},
                {"pulse_id": 3, "freq_hz": 6490.6e6,
                 "freq_offset_hz": 1e6},
            ]
            stats = fp.analyze_cir_by_freq(cir, freq_records)
            self.assertEqual(len(stats), 2)
            self.assertAlmostEqual(stats[0]["freq_hz"], 6489.6e6)
            self.assertEqual(stats[0]["n"], 2)
            self.assertEqual(stats[0]["ok"], 1)
            self.assertEqual(stats[0]["fail"], 1)
            self.assertAlmostEqual(stats[0]["metric_mean"], 0.5)
            self.assertAlmostEqual(stats[1]["freq_hz"], 6490.6e6)
            self.assertEqual(stats[1]["n"], 2)
            self.assertEqual(stats[1]["ok"], 2)
            self.assertAlmostEqual(stats[1]["peak_tap_mean"], 11.5)
            self.assertAlmostEqual(stats[1]["metric_max"], 0.7)

    def test_missing_file(self):
        self.assertEqual(fp.analyze_cir_by_freq("/no/such.jsonl", []), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
