#!/usr/bin/env python3
"""RX window length must be a multiple of the FPGA samples-per-cycle.

The X410 radio delivers RX in 8-sample units.  A ``NUM_SAMPS_AND_DONE``
request that is not a multiple of 8 makes each capture consume
``ceil(n/8)*8`` samples of the device stream; the capture position then
drifts vs. the commanded ticks by the leftover samples per burst, and the
CIR window slides across the burst (+5.33 work taps/packet was measured
with rx_len=499300, preamble 512).  The base ``rx_geometry`` must round
the window up to a multiple of 8.

Run:
    python3 gr-uwb/apps/test_rx_window_alignment.py
"""
from __future__ import annotations

import ast
import os
import unittest


APP_NAME = "x410_cg400_hrp_echo_cir.py"


def _load_functions(*names):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), APP_NAME)
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()
    tree = ast.parse(src, path)
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
    ns = {"llround": lambda x: int(round(x)),
          "ceildiv": lambda a, b: -(-a // b),
          "math": __import__("math"),
          "SPS": 1016,
          "SFD_SYMS_4Z2": 8,
          "C_LIGHT": 299792458.0}
    exec(compile(module, path, "exec"), ns)
    return {name: ns[name] for name in names}


class RxWindowAlignmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fns = _load_functions("rx_geometry")
        cls.rx_geometry = staticmethod(fns["rx_geometry"])

    def test_preamble512_window_is_multiple_of_8(self):
        # The measured drift case: preamble 512 + full TX body on CG600.
        # The old geometry produced rx_len=499300 (mod 8 = 4), which made
        # the FPGA capture consume 499304 samples per burst and the CIR
        # window slide +5.33 work taps/packet across the burst.
        pre, sync, sfd, rng, tail, pad, rx = self.rx_geometry(
            737.28e6, 2.0, 512, 15.0, 2.0, tx_native_samples=477106,
            pad_us=8.0, tx_interp=48, tx_decim=65)
        self.assertEqual(rx % 8, 0, "rx_len must be a multiple of 8")
        # Coverage: the window still spans pre + body + range + pad + tail.
        self.assertGreaterEqual(rx, pre + 477106 + rng + pad + tail)
        # The rounded-up window never exceeds the old 4-multiple by more
        # than 4 samples.
        self.assertLessEqual(rx - (pre + 477106 + rng + pad + tail), 7)

    def test_all_supported_preambles_multiple_of_8(self):
        for reps in (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048):
            for native in (737.28e6, 491.52e6):
                interp, decim = ((48, 65) if native > 500e6 else (32, 65))
                *_, rx = self.rx_geometry(
                    native, 2.0, reps, 15.0, 2.0, tx_native_samples=0,
                    pad_us=8.0, tx_interp=interp, tx_decim=decim)
                self.assertEqual(rx % 8, 0, "reps=%d native=%r" % (reps, native))

    def test_rounds_up_not_down(self):
        # A window just above a multiple of 8 must round UP, never lose
        # coverage of the TX burst.
        *_, rx = self.rx_geometry(737.28e6, 2.0, 8, 15.0, 2.0,
                                  tx_native_samples=477105, pad_us=8.0,
                                  tx_interp=48, tx_decim=65)
        self.assertGreaterEqual(rx, 477105 + 1475 + 8)

    def test_source_uses_octave_rounding(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            APP_NAME)
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
        self.assertIn("(rx + 7) // 8 * 8", src)


if __name__ == "__main__":
    unittest.main(verbosity=2)
