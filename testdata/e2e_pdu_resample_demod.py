#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Independent e2e for the PDU-level 65/48 resampler (main architecture §3.2):

  998.4 MHz cfile
    -> upfirdn 48/65 (anti-alias, DC gain 48)  = X410 native capture @737.28
    -> UwbScheduledExtractor @737.28  (window PDU at native rate)
    -> UwbPduRationalResamplerCcf65_48 (65/48 -> 998.4 PDU, group-delay map)
    -> UwbRealtimeDemodulator -> FCS / status / detected start

With --native-rate 491.52 the same cfile fixture is decimated 32/65 to the
CG400 native rate and runs through the 65/32 PDU resampler instead:

  998.4 MHz cfile
    -> upfirdn 32/65 (anti-alias, DC gain 32)  = X410 native capture @491.52
    -> UwbScheduledExtractor @491.52  (pre/cap/post 5333/160000/2667)
    -> UwbPduRationalResamplerCcf65_32 (65/32 -> 998.4 PDU, group-delay map)
    -> UwbRealtimeDemodulator -> FCS / status / detected start

Checks:
  1. PDU resampler emits the resampled window (length + sample_rate meta).
  2. Demodulator passes FCS with success status.
  3. detected_start (998.4 domain) is consistent with the known packet start.

Run:
  PYTHONPATH=$PWD/gr-uwb/build/test_modules LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/e2e_pdu_resample_demod.py [--native-rate {737.28,491.52}]
"""
import argparse
import os
import sys
import time

import numpy as np
from scipy.signal import upfirdn
from gnuradio import blocks, gr, uwb

HERE = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(HERE, "uwb_code9_preamble64_payload128_standard_sfd.cfile")
PREAMBLE = os.path.join(HERE, "reference_preamble.bin")

ORIG_START = 4992000            # packet start @998.4 (0-based)
FS998 = 998.4e6
FS737 = 737.28e6
FS491 = 491.52e6

# Native capture rates accepted by --native-rate. Output is always 998.4.
NATIVE_RATES = {
    "737.28": FS737,
    "491.52": FS491,
}
# Per-rate synthesis/extraction geometry. The 491.52 guards are the 737.28
# guards scaled by 32/48: pre/cap/post = 5333/160000/2667.
GEOM = {
    "737.28": {"input_rate": FS737, "interp": 65, "decim": 48,
               "taps_dir": "resampler_65_48",
               "pre": 8000, "cap": 240000, "post": 4000},
    "491.52": {"input_rate": FS491, "interp": 65, "decim": 32,
               "taps_dir": "resampler_65_32",
               "pre": 5333, "cap": 160000, "post": 2667},
}


def parse_native_rate(s):
    """Normalize --native-rate to (canonical_key, rate_hz).

    Accepts MHz ("737.28"/"491.52", also "737p28"/"491p52") or Hz
    (737280000.0/491520000.0) to match offline_qm35_auto_lock.py and
    x410_auto_scheduled_capture.py conventions.
    """
    norm = str(s).strip().lower().replace("_", "").replace("p", ".")
    rate = NATIVE_RATES.get(norm)
    if rate is None:
        try:
            v = float(norm)
        except ValueError:
            v = float("nan")
        for known in (FS737, FS491):
            if abs(v * 1e6 - known) <= 1e3 or abs(v - known) <= 1e3:
                rate = known
                break
    if rate is None:
        raise ValueError(
            f"unsupported --native-rate {s!r}: want 737.28 or 491.52")
    canon = "491.52" if abs(rate - FS491) < 1.0 else "737.28"
    return canon, rate


def make_pdu_resampler(native_key, taps_min_path):
    """Build the native -> 998.4 PDU resampler for the native rate."""
    if native_key == "491.52":
        # 65_32 Python binding exposes only the profile/path constructor
        # (no make_from_taps / emit_policy); FullWindow is fixed in C++.
        return uwb.pdu_rational_resampler_ccf_65_32(taps_min_path, FS998)
    return uwb.pdu_rational_resampler_ccf_65_48.make_from_taps(
        np.fromfile(taps_min_path, np.float32).tolist(), FS998)


def parse_args():
    ap = argparse.ArgumentParser(
        description="PDU resample e2e: cfile -> native -> extractor -> "
                    "65/N resampler -> demodulator")
    ap.add_argument("--native-rate", default="737.28",
                    help="native capture rate in MHz: 737.28 (default) "
                         "or 491.52 (65/32 PDU resampler to 998.4)")
    return ap.parse_args()


def main():
    import pmt as p
    args = parse_args()
    try:
        native_key, fs_native = parse_native_rate(args.native_rate)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(2)
    geom = GEOM[native_key]
    decim = geom["decim"]
    taps_r = os.path.join(HERE, geom["taps_dir"], "taps_realtime.txt")
    taps_min = os.path.join(HERE, geom["taps_dir"],
                            "taps_quality_minorder.txt")
    x998 = np.fromfile(CFILE, np.complex64)
    print(f"cfile {len(x998)} @998.4, packet start ~{ORIG_START}")
    print(f"native-rate {native_key} MS/s (decim {decim}/65)")

    # Chunk around the packet (same idiom as the QA e2e), then decimate.
    chunk0 = max(0, ORIG_START - 300000)
    chunk = x998[chunk0:]
    dec_taps = np.fromfile(taps_r, np.float32) * (decim / 65.0)
    x_native = upfirdn(dec_taps, chunk, decim, 65)
    dd = 0.5 * (len(dec_taps) - 1)
    local998 = ORIG_START - chunk0
    pkt_native = int(round((local998 * decim + dd) / 65.0))
    print(f"chunk @998.4={len(chunk)} -> @{native_key}={len(x_native)} "
          f"pkt_native={pkt_native}")

    # Geometry validated in qa_uwb_pdu_rational_resampler.cc test #6 (737);
    # 491 guards are round(737-guard * 32/48).
    pre, cap, post = geom["pre"], geom["cap"], geom["post"]
    need = pkt_native + pre + cap + post + 500000
    stream = np.concatenate([x_native, np.zeros(max(need - len(x_native),
                                                  500000),
                                                np.complex64)])
    print(f"stream @{native_key}: {len(stream)} samples")

    extractor = uwb.scheduled_extractor(fs_native, 0.05, pkt_native,
                                        pre, cap, post, 4,
                                        uwb.scheduled_extractor.EmitPolicy.EverySlot,
                                        False)
    resampler = make_pdu_resampler(native_key, taps_min)
    tmpl = np.fromfile(PREAMBLE, np.complex64)

    # Stage A: extractor @native -> PDU resampler -> capture resampled PDU.
    dbg_pkt = blocks.message_debug()
    src = blocks.vector_source_c(stream.tolist(), False)
    tb = gr.top_block()
    tb.connect((src, 0), (extractor, 0))
    tb.msg_connect(extractor, "packet", resampler, "packet")
    tb.msg_connect(resampler, "packet", dbg_pkt, "store")
    tb.run()
    print(f"extractor scheduled={extractor.scheduled_windows()} "
          f"emitted={extractor.emitted_windows()} | resampler "
          f"rx={resampler.pdus_received()} emitted={resampler.pdus_emitted()} "
          f"dropped={resampler.pdus_dropped()}")

    ok = True
    if dbg_pkt.num_messages() == 0:
        print("FAIL: no resampled PDU")
        sys.exit(1)

    # Stage B: feed the resampled PDU to the demod with the FG kept alive
    # (async worker must publish before stop()).
    demod = uwb.realtime_demodulator.make_from_template(tmpl.tolist(), 2, 16, "ieee")
    dbg = blocks.message_debug()
    tb2 = gr.top_block()
    tb2.msg_connect(demod, "result", dbg, "store")
    tb2.start()
    demod._post(p.intern("samples"), dbg_pkt.get_message(0))
    deadline = time.time() + 20
    while time.time() < deadline:
        if dbg.num_messages() > 0:
            break
        time.sleep(0.01)
    tb2.stop()
    tb2.wait()
    print(f"demod rx={demod.jobs_received()} completed={demod.jobs_completed()} "
          f"failed={demod.jobs_failed()} results={dbg.num_messages()}")

    if dbg.num_messages() == 0:
        print("FAIL: no demod result")
        sys.exit(1)
    msg = dbg.get_message(0)
    m = p.car(msg) if p.is_pair(msg) else msg
    status = p.symbol_to_string(p.dict_ref(m, p.intern("status"), p.intern("?")))
    fcs = p.to_bool(p.dict_ref(m, p.intern("fcs_pass"), p.from_bool(False)))
    det = p.to_long(p.dict_ref(m, p.intern("detected_start_sample"), p.from_long(-1)))
    pred = resampler.map_input_offset_to_output(pkt_native)
    print(f"demod status={status} fcs_pass={fcs} detected_start={det} "
          f"mapped_predicted~{pred} |det-pred|={abs(det - pred)}")
    if status.lower() != "success" or not fcs:
        print("FAIL: demod not success / FCS fail")
        ok = False
    if abs(det - pred) > 5000:
        print("FAIL: detected_start far from mapped predicted")
        ok = False
    print(f"E2E PDU-RESAMPLE [{native_key}]:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
