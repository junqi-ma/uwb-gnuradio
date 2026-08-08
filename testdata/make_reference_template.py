#!/usr/bin/env python3
"""Extract the one-SYNC-symbol reference preamble template from the known UWB
test signal and write it to testdata/reference_preamble.bin.

Template semantics follow UWB_demodulation/+uwbdecoder/buildUwbReference.m:
the preamble waveform is one SYNC symbol (1016 samples) L2-normalized.
"""

import numpy as np

CFILE = "/home/junqima/workspace/uwb-gnuradio/testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile"
OUT = "/home/junqima/workspace/uwb-gnuradio/testdata/reference_preamble.bin"
SAMPLES_PER_SYMBOL = 1016
PACKET_START = 4992000  # 0-based index of first non-zero sample


def main():
    data = np.fromfile(CFILE, dtype=np.complex64)
    template = data[PACKET_START:PACKET_START + SAMPLES_PER_SYMBOL]
    assert template.size == SAMPLES_PER_SYMBOL
    energy = float(np.sum(np.abs(template) ** 2))
    if energy > 0:
        template = template / np.sqrt(energy)
    template.astype(np.complex64).tofile(OUT)
    print(f"wrote {OUT}: {template.size} samples, energy={np.sum(np.abs(template)**2):.6f}")


if __name__ == "__main__":
    main()
