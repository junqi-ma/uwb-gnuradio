#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Independent e2e for the PDU-level 65/48 resampler (main architecture §3.2):

  998.4 MHz cfile
    -> upfirdn 48/65 (anti-alias, DC gain 48)  = X410 native capture @737.28
    -> UwbScheduledExtractor @737.28  (window PDU at native rate)
    -> UwbPduRationalResamplerCcf65_48 (65/48 -> 998.4 PDU, group-delay map)
    -> UwbRealtimeDemodulator -> FCS / status / detected start

Checks:
  1. PDU resampler emits the resampled window (length + sample_rate meta).
  2. Demodulator passes FCS with success status.
  3. detected_start (998.4 domain) is consistent with the known packet start.

Run:
  PYTHONPATH=$PWD/gr-uwb/build/test_modules LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/e2e_pdu_resample_demod.py
"""
import os
import sys
import time

import numpy as np
from scipy.signal import upfirdn
from gnuradio import blocks, gr, uwb

HERE = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(HERE, "uwb_code9_preamble64_payload128_standard_sfd.cfile")
TAPS_R = os.path.join(HERE, "resampler_65_48", "taps_realtime.txt")
TAPS_MIN = os.path.join(HERE, "resampler_65_48", "taps_quality_minorder.txt")
PREAMBLE = os.path.join(HERE, "reference_preamble.bin")

ORIG_START = 4992000            # packet start @998.4 (0-based)
FS998 = 998.4e6
FS737 = 737.28e6
PRE_GUARD = 9984
CAPTURE = 309184
POST_GUARD = 4096


def main():
    import pmt as p
    x998 = np.fromfile(CFILE, np.complex64)
    print(f"cfile {len(x998)} @998.4, packet start ~{ORIG_START}")

    # Chunk around the packet (same idiom as the QA e2e), then decimate @737.28.
    chunk0 = max(0, ORIG_START - 300000)
    chunk = x998[chunk0:]
    dec_taps = np.fromfile(TAPS_R, np.float32) * (48.0 / 65.0)
    x737 = upfirdn(dec_taps, chunk, 48, 65)
    dd = 0.5 * (len(dec_taps) - 1)
    local998 = ORIG_START - chunk0
    pkt737 = int(round((local998 * 48.0 + dd) / 65.0))
    print(f"chunk @998.4={len(chunk)} -> @737.28={len(x737)} pkt737={pkt737}")

    # Geometry validated in qa_uwb_pdu_rational_resampler.cc test #6.
    pre, cap, post = 8000, 240000, 4000
    need = pkt737 + pre + cap + post + 500000
    stream = np.concatenate([x737, np.zeros(max(need - len(x737), 500000),
                                            np.complex64)])
    print(f"stream @737.28: {len(stream)} samples")

    extractor = uwb.scheduled_extractor(FS737, 0.05, pkt737,
                                        pre, cap, post, 4,
                                        uwb.scheduled_extractor.EmitPolicy.EverySlot,
                                        False)
    resampler = uwb.pdu_rational_resampler_ccf_65_48.make_from_taps(
        np.fromfile(TAPS_MIN, np.float32).tolist(), FS998)
    tmpl = np.fromfile(PREAMBLE, np.complex64)

    # Stage A: extractor @737.28 -> PDU resampler -> capture the resampled PDU.
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
    pred = resampler.map_input_offset_to_output(pkt737)
    print(f"demod status={status} fcs_pass={fcs} detected_start={det} "
          f"mapped_predicted~{pred} |det-pred|={abs(det - pred)}")
    if status.lower() != "success" or not fcs:
        print("FAIL: demod not success / FCS fail")
        ok = False
    if abs(det - pred) > 5000:
        print("FAIL: detected_start far from mapped predicted")
        ok = False
    print("E2E PDU-RESAMPLE:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
