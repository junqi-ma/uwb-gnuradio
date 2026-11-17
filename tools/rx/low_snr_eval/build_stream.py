#!/usr/bin/env python3
"""Build a native-rate (.sc16) stream of complete BPRF frames at a target
in-band SNR, for the low-SNR evaluation matrix.

The 998.4 MS/s work-grid waveform is produced by `gen_full_frame` (which calls
the production `gr::uwb::mod::modulate_one`).  This script resamples it 48/65
to 737.28 MS/s with the same Kaiser(beta=5.0) polyphase filter used by the
earlier AWGN study, scales the PACKET rms to 542 LSB, adds white noise at the
requested in-band (±250 MHz) SNR, and writes interleaved int16.

Usage:
    python3 build_stream.py --snr -12 --packets 200 --out /tmp/low_snr_eval/full_m12.sc16
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import hashlib
from pathlib import Path

import numpy as np

REPO = str(Path(__file__).resolve().parents[3])
sys.path.insert(0, os.path.join(REPO, "tools", "rx"))
import proto_low_snr_signal as sig  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = None  # Supplied explicitly: no binary artifacts in the source tree.


def build(snr_db, packets, out_path, seed=1, psdu_bytes=127, rms_lsb=542.0,
          work_cf32=None, keep_work=False):
    if work_cf32 is None:
        work_cf32 = str(out_path) + ".work.cf32"
    subprocess.run(
        [GEN, "--packets", str(packets), "--psdu-bytes", str(psdu_bytes),
         "--seed", str(seed), "--out", work_cf32],
        check=True,
    )
    x998 = np.fromfile(work_cf32, dtype=np.complex64)
    if not keep_work:
        os.remove(work_cf32)

    # Per-packet normalisation: the packet occupies the head of each 1 ms slot.
    slot_work = 998400
    pkt_work = 444352  # modulate_one length for 127 B / 256 SYNC / 4z2
    if x998.size != slot_work * packets:
        raise RuntimeError(f"unexpected work length {x998.size}")
    # Measure the packet rms over the first packet's written region.
    first = x998[:pkt_work]
    pkt_rms = float(np.sqrt(np.mean(np.abs(first) ** 2)))
    x998 = (x998 * (rms_lsb / pkt_rms)).astype(np.complex64)

    x = sig.resample_to_native(x998)
    slot_native = 737280
    if x.size != slot_native * packets:
        raise RuntimeError(f"unexpected native length {x.size}")

    if snr_db is not None:
        x = sig.add_awgn(x, snr_db, rms_lsb=rms_lsb, seed=seed)
    else:
        p_noise = sig.noise_power_for(-14.0, rms_lsb)
        rng = np.random.default_rng(seed)
        n = (rng.standard_normal(x.size) + 1j * rng.standard_normal(x.size))
        n *= np.sqrt(p_noise / 2.0)
        x = n.astype(np.complex64)

    q = sig.quantize_sc16(x)
    q.tofile(out_path)
    digest = hashlib.sha256()
    with open(out_path, "rb") as fixture:
        for chunk in iter(lambda: fixture.read(1024 * 1024), b""):
            digest.update(chunk)
    meta = dict(sha256=digest.hexdigest(),snr_db=snr_db, packets=packets, seed=seed,
                psdu_bytes=psdu_bytes, rms_lsb=rms_lsb,
                sample_rate=737.28e6, slot_samples=slot_native,
                packet_starts=[k * slot_native for k in range(packets)],
                out=out_path)
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--snr", type=float, default=-12.0)
    ap.add_argument("--noise", action="store_true", help="pure AWGN, no packets")
    ap.add_argument("--packets", type=int, default=200)
    ap.add_argument("--psdu-bytes", type=int, default=127)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", required=True)
    ap.add_argument("--meta", default=None)
    ap.add_argument("--generator", required=True, help="compiled gen_full_frame executable")
    a = ap.parse_args()
    if a.packets < 1 or a.psdu_bytes != 127:
        ap.error("packets must be positive; this calibrated fixture supports only 127 B")
    global GEN
    GEN = str(Path(a.generator).resolve())
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    meta = build(None if a.noise else a.snr, a.packets, a.out, seed=a.seed,
                 psdu_bytes=a.psdu_bytes)
    with open(a.meta or a.out + ".json", "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps({k: v for k, v in meta.items() if k != "packet_starts"}))


if __name__ == "__main__":
    main()
