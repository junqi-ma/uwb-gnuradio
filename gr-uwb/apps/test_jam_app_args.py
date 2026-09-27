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
import contextlib
import io
import os
import time
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
    "--jam-freq-start",
    "--jam-freq-stop",
    "--jam-freq-step",
    "--jam-dwell",
    "--jam-code-index",
    "--jam-preamble-length",
    "--jam-psdu-hex",
    "--jam-waveform",
    "--jam-pulse-shape",
    "--jam-delay-us",
    "--jam-delay-random-us",
    "--jam-overlap-mode",
    "--jam-overlap-min-reps",
    "--jam-overlap-max-reps",
    "--jam-overlap-side",
    "--jam-overlap-seed",
    "--jam-scale",
    "--cir-emit-normalized",
    "--cir-writer-queue-pdus",
    "--cir-avg-writer-queue-pdus",
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


def _load_class_method(class_name, method_name):
    """Run a hardware-free class method with a fake instance."""
    path = _app_path()
    tree = ast.parse(_source(), path)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for method in node.body:
                if isinstance(method, ast.FunctionDef) and method.name == method_name:
                    module = ast.Module(body=[method], type_ignores=[])
                    ast.fix_missing_locations(module)
                    ns = {"time": time}
                    exec(compile(module, path, "exec"), ns)
                    return ns[method_name]
    raise AssertionError("missing %s.%s" % (class_name, method_name))


class CirWriterWorstCaseBytesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fn = staticmethod(
            _load_functions(
                "cir_writer_worst_case_bytes")["cir_writer_worst_case_bytes"])

    def test_scales_linearly_with_capacity(self):
        total1, payload, ucr4, jsonl = self.fn(64, 128, 116, True)
        total2, _, _, _ = self.fn(128, 128, 116, True)
        self.assertEqual(total2, 2 * total1)
        # Payload dominates: raw + normalized FC32 batches.
        self.assertEqual(payload, 128 * 116 * 8 * 2 + 128 * 24)
        self.assertEqual(ucr4, 128 * (52 + 4 * 116))
        self.assertGreater(jsonl, 0)

    def test_average_mode_single_record(self):
        total, payload, ucr4, _ = self.fn(64, 1, 116, True)
        self.assertEqual(payload, 116 * 8 * 2 + 24)
        self.assertEqual(ucr4, 52 + 4 * 116)
        self.assertEqual(total, 64 * (payload + ucr4 + 640 + 120))

    def test_without_normalized_taps(self):
        _, payload, _, _ = self.fn(4, 128, 116, False)
        self.assertEqual(payload, 128 * 116 * 8 + 128 * 24)


class CirWriterAggregateTest(unittest.TestCase):
    """Writer aggregate-bytes pass-through (盘停免疫) static contract.

    Default must be ENABLED at 1 MiB (the soak failure mode — per-record
    writes hitting a multi-second disk trough and filling the queue — is
    exactly what aggregation fixes), 0 must restore the legacy per-record
    path, and both base and jam chains must wire the flag into every
    UwbCirWriter they create and publish the runtime stats in the summary.
    """

    BASE_NAME = "x410_cg400_hrp_echo_cir.py"

    def _base_source(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            self.BASE_NAME)
        with open(path, "r", encoding="utf-8") as f:
            return f.read()

    def test_cli_declared_default_enabled_1mib(self):
        # The jam app INHERITS --cir-writer-aggregate-bytes from the base
        # parser (parents=[base.build_parser()]); redeclaring it here would
        # raise argparse "conflicting option string".  So the base source
        # owns the CLI declaration; the jam source references it in both
        # writer constructions and the summary keys.
        base = self._base_source()
        self.assertIn('"--cir-writer-aggregate-bytes"', base,
                      "%s: missing aggregate-bytes option" % self.BASE_NAME)
        self.assertIn("type=int, default=1048576", base,
                      "%s: aggregate default must be 1 MiB" % self.BASE_NAME)
        for src, name in ((_source(), APP_NAME), (base, self.BASE_NAME)):
            self.assertIn('"cir_writer_aggregate_bytes"', src,
                          "%s: summary key missing" % name)
            self.assertIn('"cir_writer_aggregate_flushes"', src,
                          "%s: summary key missing" % name)
            self.assertIn('"cir_writer_aggregate_max_bytes"', src,
                          "%s: summary key missing" % name)

    def test_writers_receive_aggregate_args(self):
        src = _source()
        # Both jam-chain writers (cir + cir_avg) forward the aggregate
        # threshold (0 stays a supported explicit disable).
        self.assertEqual(
            src.count("a.cir_writer_aggregate_bytes"), 3,
            "jam app must reference the aggregate flag in the cir and "
            "cir_avg writer construction plus the summary key")
        self.assertIn("max(0, int(a.cir_writer_aggregate_bytes)), 50)",
                      src, "writer call must clamp negatives and pin the "
                      "50 ms age window")
        base = self._base_source()
        self.assertEqual(
            base.count("wr_aggregate_bytes"), 7,
            "base app must reference the clamped aggregate value in both "
            "writers + summary keys")
        self.assertIn("wr_aggregate_bytes, wr_aggregate_window_ms)", base)


class CirEmitNormalizedPolicyTest(unittest.TestCase):
    """Stage-3 consumer-driven normalized policy (静态契约).

    Consumers of the estimator's "cir"/"cir_avg" ports on the jam chain:

      * UwbCirWriter — UCR4 stores raw SC16 + cir_scale only;
        write_normalized=true merely REQUIRES the normalized_taps field,
        the normalized data is never written to any file.
      * CirUdpSink — frames the raw cdr c32vector into UCR5 datagrams;
        normalized_taps is never read.
      * message_debug — not attached on this chain.

    So the default must be: estimator emit_normalized=False, writers
    created with write_normalized=emit_norm (they must not require the
    absent field), and an explicit --cir-emit-normalized switch restores
    the legacy message contract.
    """

    def test_default_off_and_switch_restores_legacy(self):
        src = _source()
        self.assertIn('"--cir-emit-normalized"', src)
        self.assertIn("action=\"store_true\"", src)
        # Default OFF: the flag is opt-in (store_true defaults to False).
        self.assertNotIn('a.cir_emit_normalized, default=True', src)
        # Estimator + both writers follow the policy flag, not a literal.
        self.assertLessEqual(
            src.count("emit_norm, est_q, use_pred"), 1,
            "estimator emit_normalized wired to the policy flag")
        self.assertIn('a.sync_refine_threshold, emit_norm, est_q, use_pred',
                      src)
        self.assertIn('base.uwb.cir_writer(a.output, "cir", emit_norm, '
                      'wr_queue_pdus,', src)
        self.assertIn('"cir_avg", emit_norm', src)
        # Worst-case RAM follows the policy too (payload bound halves).
        self.assertIn("cir_taps, emit_norm)", src)

    def test_metrics_are_independent_of_normalized(self):
        # raw_l2_norm is computed from the raw taps and must stay published
        # when the normalized PMT is omitted.
        src = _source()
        self.assertIn("raw_l2_norm", src)


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

    def test_allows_cpp_pdu_align(self):
        # M3: align + cpp-pdu builds a one-shot multi-TX schedule PDU; the
        # old "cannot carry" hard reject is gone, while continuous + cpp-pdu
        # is still explicitly refused (python-only independent streamer).
        src = _source()
        self.assertNotIn("cannot carry", src)
        self.assertIn(
            "--echo-backend cpp-pdu does not support --jam-mode continuous",
            src)
        self.assertIn("JamCppPduEcho", src)
        self.assertIn("tx_channel_count", src)
        self.assertIn("pmt_vector", src)
        self.assertIn("make_vector", src)

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

    def test_randomizes_delay_per_pulse(self):
        src = _source()
        self.assertIn("_randomize_jam_delay", src)
        self.assertIn("jp.place_jam_row(", src)
        self.assertIn("jp.draw_delay_native(", src)
        self.assertIn("jp.parse_delay_random_us(", src)
        self.assertIn("jp.compose_tx_native_at(", src)
        self.assertIn("[-T, +T]", src)

    def test_plan_module_is_pure(self):
        plan = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            PLAN_NAME)
        with open(plan, "r", encoding="utf-8") as f:
            plan_src = f.read()
        for banned in ("import gnuradio", "import uhd", "import pmt"):
            self.assertNotIn(banned, plan_src)

    def test_cpp_contiguous_window_dry_run(self):
        src = _source()
        self.assertIn("contiguous-window", src)
        self.assertIn("data_fragments", src)
        self.assertIn("backing_length", src)
        self.assertIn("require_fragment_covers_L", src)

    def test_cir_window_comes_from_cli(self):
        src = _source()
        self.assertIn("resolve_cir_window", src)
        self.assertIn("cir_pre, cir_post, cir_taps", src)
        self.assertNotIn("a.code_index, 16, 100,", src)

    def test_repetition_roi_does_not_apply_average_skip(self):
        src = _source()
        self.assertIn("cir_skip=base.cir_skip_for_output(a.cir_output)", src)
        base_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "x410_cg400_hrp_echo_cir.py")
        with open(base_path, "r", encoding="utf-8") as f:
            base_src = f.read()
        self.assertIn('return cir_output in ("repetitions", "both")',
                      base_src)
        self.assertIn("LEGACY_CIR_AVERAGE_SKIP = 10", base_src)

    def test_cpp_pdu_carries_absolute_scan_base(self):
        # A jam scan offset is ABSOLUTE base + offset: without the base the
        # C++ worker would tune the jammer to the bare offset (measured on
        # X410: coerced to 1 MHz, out of band).  The PDU must carry
        # "freq_hz", and the caller must pass the real centre frequency.
        src = _source()
        self.assertIn("base_freq_hz", src)
        self.assertIn('"freq_hz"', src)
        self.assertIn("base_freq_hz=self.freq", src)

    def test_overlap_requires_packet_and_cpp(self):
        src = _source()
        # The overlap mode is a real packet collision: it must demand
        # --jam-waveform packet, refuse --jam-delay-random-us / nonzero
        # --jam-delay-us, and be cpp-pdu only (never a python approximation).
        self.assertIn("random-reps", src)
        self.assertIn("random-reps requires ", src)
        self.assertIn("requires --jam-waveform", src)
        self.assertIn("cannot combine with", src)
        self.assertIn("only supported with ", src)
        self.assertIn("echo-backend cpp-pdu", src)
        self.assertIn("jam_overlap_lead_delays_native", src)
        self.assertIn("jam_overlap_lag_delays_native", src)
        self.assertIn("init_s64vector", src)

    def test_payload_jam_covers_sensing_preamble(self):
        # The payload jammer must overlap the CIR estimation window: the
        # PSDU field is tiled to Ns*SPS and started at the sensing TX
        # start; negative delays and random delays are refused.
        src = _source()
        self.assertIn("jp.fill_preamble_span_work(", src)
        self.assertIn("jp.payload_start_work(", src)
        self.assertNotIn("sense_payload_start_native", src)
        self.assertIn("requires a non-negative ", src)


class ScanRetryTest(unittest.TestCase):
    def test_failed_attempt_reuses_pending_random_payload(self):
        run_schedule = _load_class_method("JamTimedUhdEcho", "run_schedule")

        class FakeEcho:
            _native = object()
            jam_offsets = [0.0]
            jam_dwell = 2
            max_pulses = 2
            pri_s = 0.001
            freq_settle_s = 0.0
            jam_freq_offset = 0.0
            jam_enable = False
            _ok = 0
            _fail = 0
            _late = 0
            _sc16_written = 0
            jam_delay_us = 0.0
            jam_retune_count = 0

            def __init__(self):
                self.slots = []
                self.attempts = iter((False, True, True))

            def open_sc16_dump(self):
                pass

            def close_sc16_dump(self):
                pass

            def start_publisher(self):
                pass

            def _randomize_jam_delay(self):
                pass

            def _select_jam_payload(self, index):
                self.slots.append(index)
                if index >= self.max_pulses:
                    raise IndexError("payload bank exhausted")

            def _one_burst(self, pulse_id):
                captured = next(self.attempts)
                self._ok += int(captured)
                self._fail += int(not captured)
                return captured

        fake = FakeEcho()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(run_schedule(fake), (2, 1, 0))
        self.assertEqual(fake.slots, [0, 0, 1])


if __name__ == "__main__":
    unittest.main(verbosity=2)
