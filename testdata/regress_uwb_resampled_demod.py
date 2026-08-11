#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
§8.5 UWB algorithm regression through the 65/48 software resampler.

Pipeline:
  998.4 MHz cfile
    -> upfirdn 48/65 (anti-alias, realtime taps)  = what X410 would capture
    -> UwbRationalResamplerCcf65_48 (737.28 -> 998.4 MS/s)
    -> UwbDetector (CF32) -> packet PDU
    -> UwbRealtimeDemodulator -> result (FCS / status / packet start)

Checks:
  1. The detector still finds the packet after the round trip.
  2. The demodulator result has FCS pass and Success status.
  3. detected_start_sample is close to the original-domain packet start
     mapped through the 48/65 and 65/48 round trip.

Run:
  PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib \
  python3 testdata/regress_uwb_resampled_demod.py
"""
import os
import sys

import numpy as np
from scipy.signal import upfirdn

from gnuradio import blocks, gr, uwb

HERE = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(HERE, "uwb_code9_preamble64_payload128_standard_sfd.cfile")
TAPS_Q = os.path.join(HERE, "resampler_65_48", "taps_quality.txt")
TAPS_R = os.path.join(HERE, "resampler_65_48", "taps_realtime.txt")
PREAMBLE = os.path.join(HERE, "reference_preamble.bin")

ORIG_START = 4992000            # packet start in the 998.4 MHz cfile (0-based)
DEMOD_PRE = 9984
DEMOD_CAPTURE = 309184


def load_cf32(path):
    return np.fromfile(path, dtype=np.complex64)


TRAILING_SILENCE = 5000000  # close detector regions AND give the async demod
                            # worker time to publish its result before EOS
                            # (results published at stop() are not delivered)


def run_det_demod(x998, label):
    """Detect + demod a 998.4 MHz stream; return result summary.
    Appends trailing silence so the detector can close the packet region."""
    import pmt as p
    tmpl = load_cf32(PREAMBLE)
    det = uwb.detector(tmpl.tolist(), DEMOD_PRE, DEMOD_CAPTURE)
    demod = uwb.realtime_demodulator.make_from_template(
        tmpl.tolist(), 2, 64, "ieee")
    dbg = blocks.message_debug()
    sig = np.concatenate([x998, np.zeros(TRAILING_SILENCE, np.complex64)])
    src = blocks.vector_source_c(sig.tolist(), False)
    tb = gr.top_block()
    tb.connect((src, 0), (det, 0))
    tb.msg_connect(det, "packet", demod, "samples")
    tb.msg_connect(demod, "result", dbg, "store")
    tb.run()
    demod.drain()               # flush async worker results into the queue
    # Async worker publishes the result message; give the scheduler a moment.
    import time
    n = dbg.num_messages()
    for _ in range(200):
        if n > 0:
            break
        time.sleep(0.005)
        n = dbg.num_messages()
    print(f"[{label}] results={n} packets_received={demod.jobs_received()} "
          f"completed={demod.jobs_completed()} failed={demod.jobs_failed()}")
    if n == 0:
        return None
    msg = dbg.get_message(0)
    m = p.car(msg) if p.is_pair(msg) else msg
    status = pmt_status(m)
    fcs = pmt_fcs(m)
    start = pmt_int(m, "detected_start_sample")
    print(f"[{label}] status={status} fcs_pass={fcs['pass']} "
          f"rx_fcs=0x{fcs['rx']:04x} calc_fcs=0x{fcs['calc']:04x} "
          f"detected_start={start}")
    return {"status": status, "fcs": fcs, "detected_start": start}


def pmt_status(m):
    import pmt as p
    return p.symbol_to_string(p.dict_ref(m, p.intern("status"), p.intern("?")))


def pmt_fcs(m):
    import pmt as p
    return {
        "pass": bool(p.to_bool(p.dict_ref(m, p.intern("fcs_pass"), p.from_bool(False)))),
        "rx": p.to_long(p.dict_ref(m, p.intern("received_fcs"), p.from_long(0))),
        "calc": p.to_long(p.dict_ref(m, p.intern("calculated_fcs"), p.from_long(0))),
    }


def pmt_int(m, key):
    import pmt as p
    return p.to_long(p.dict_ref(m, p.intern(key), p.from_long(-1)))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--upsample_profile", default="quality",
                    choices=["quality", "realtime", "quality_minorder",
                             "realtime_minorder"])
    args, _ = ap.parse_known_args()
    upsample_taps = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "resampler_65_48",
        f"taps_{args.upsample_profile}.txt")

    x998 = load_cf32(CFILE)
    print(f"cfile: {len(x998)} samples @ 998.4 MHz, packet start ~{ORIG_START}")

    # --- round trip: 998.4 -> 737.28 (X410) -> 998.4 (fixed block) ---
    # Anti-alias filter for the 48/65 decimation: same shape as the realtime
    # profile but DC gain must be 48 (the decimation interpolation factor), not
    # 65 — otherwise the round trip gains 65/48 ≈ 1.354 in amplitude.
    dec_taps = np.fromfile(TAPS_R, dtype=np.float32) * (48.0 / 65.0)
    x737 = upfirdn(dec_taps, x998.astype(np.complex64), 48, 65)
    print(f"decimated 737.28: {len(x737)} samples")
    taps = np.fromfile(upsample_taps, dtype=np.float32)
    blk = uwb.rational_resampler_ccf_65_48.make_from_taps(taps.tolist())
    src = blocks.vector_source_c(x737.tolist(), False)
    snk = blocks.vector_sink_c()
    tb = gr.top_block()
    tb.connect((src, 0), (blk, 0))
    tb.connect((blk, 0), (snk, 0))
    tb.run()
    x_rt = np.array(snk.data(), dtype=np.complex64)
    print(f"resampled back to 998.4: {len(x_rt)} samples "
          f"(energy ratio {np.mean(np.abs(x_rt)**2)/np.mean(np.abs(x998)**2):.4f})")

    r_orig = run_det_demod(x998, "ORIGINAL 998.4")
    r_rt = run_det_demod(x_rt, f"ROUND-TRIP ({args.upsample_profile})")

    ok = True
    if r_rt is None:
        print("FAIL: no packet detected after round trip")
        ok = False
    else:
        if not r_rt["fcs"]["pass"]:
            print("FAIL: FCS did not pass after round trip")
            ok = False
        if r_rt["status"].lower() != "success":
            print("FAIL: demod status not success:", r_rt["status"])
            ok = False
        # Round-trip filters add a group delay (~83 output samples for the
        # realtime-decimate + quality-resample pair); allow that plus margin.
        if r_orig and abs(r_rt["detected_start"] - r_orig["detected_start"]) > 200:
            print("FAIL: detected_start drifted too far:",
                  r_orig["detected_start"], "->", r_rt["detected_start"])
            ok = False
    print("REGRESSION:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
