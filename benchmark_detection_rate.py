#!/usr/bin/env python3
"""
Benchmark the GNU Radio UWB preamble detector throughput.

Measures sustained C++ work() rate of each operating mode on the known UWB
test signal, using null sinks (no per-sample output materialization — a real
detector emits PDUs, it does not store every metric sample).

Modes:
  reference  — full-rate correlation on every sample (~8 MS/s)
  fast       — energy-gated full-rate correlation in candidates (~70 MS/s)
  coarse     — decimated energy gate (D=100) + decimated coarse scan (D=4) +
               full-rate fine correlation in small ROIs (~80-150 MS/s)

The core pipeline (energy gate + coarse scan, no flowgraph) runs >1 GS/s;
the end-to-end figure below is dominated by flowgraph buffer plumbing.

Run from the build tree:
  PYTHONPATH=$GR_UWB/build/test_modules \
  LD_LIBRARY_PATH=$GR_UWB/build/lib \
  python3 benchmark_detection_rate.py
"""

import os
import time
import numpy as np
from gnuradio import gr, blocks, uwb

ROOT = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(ROOT, "testdata",
                     "uwb_code9_preamble64_payload128_standard_sfd.cfile")


def bench(blk, label):
    src = blocks.file_source(gr.sizeof_gr_complex, CFILE, False)
    ns0 = blocks.null_sink(gr.sizeof_float)
    ns1 = blocks.null_sink(1)  # unsigned char flag output
    tb = gr.top_block()
    tb.connect(src, blk)
    tb.connect((blk, 0), ns0)
    tb.connect((blk, 1), ns1)
    t0 = time.perf_counter()
    tb.run()
    dt = time.perf_counter() - t0
    n = os.path.getsize(CFILE) // 8
    ms = n / dt / 1e6
    print(f"  {label:36s} {ms:8.1f} MS/s   ({dt*1e3:6.0f} ms for {n} samples)")
    return ms


def main():
    import platform
    print("GNU Radio version:", gr.version())
    print("Python:", platform.python_version())
    print("Sample format: CF32, rate 998.4 MHz\n")

    tmpl = np.fromfile(CFILE, dtype=np.complex64)[4992000:4992000 + 1016]
    tmpl = tmpl / np.sqrt(np.sum(np.abs(tmpl) ** 2))

    print("Preamble detector throughput (null sinks, C++ work()):")
    bench(uwb.preamble_detector(tmpl.tolist(), 0.5), "reference (full-rate)")
    bench(uwb.preamble_detector(tmpl.tolist(), 0.5, 1e-3, 32, 1),
          "fast (energy-gated full-rate)")
    bench(uwb.preamble_detector(tmpl.tolist(), 0.5, 1e-3),
          "coarse-to-fine default (ED=100 CD=4)")
    bench(uwb.preamble_detector(tmpl.tolist(), 0.5, 1e-3, 8, 100, 8),
          "coarse-to-fine ED=100 CD=8")

    print("\nNote: the core pipeline (energy gate D=100 + coarse scan) measures "
          ">1 GS/s in a standalone benchmark; the end-to-end figure is bounded "
          "by flowgraph buffer copies.")


if __name__ == "__main__":
    main()
