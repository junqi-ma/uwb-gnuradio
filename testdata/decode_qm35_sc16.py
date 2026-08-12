#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagnose C++ QM35 decode from the X410 SC16 capture (737.28 MS/s, 6489.6 MHz).

The QM35 transmits at 998.4 MS/s native; the capture is 737.28 = 998.4*48/65,
so windows are first upsampled 65/48 to 998.4, then fed to the C++ demod
(UwbRealtimeDemodulator, code_index=9, sfd "4z2", preamble 64 SYNC).

Usage:
  PYTHONPATH=$PWD/gr-uwb/build/test_modules LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/decode_qm35_sc16.py <dat_path> [slot_index] [--all]
"""
import os
import sys
import time

import numpy as np
from scipy.signal import upfirdn

FS737 = 737.28e6
FS998 = 998.4e6
TAPS_R = os.path.join(os.path.dirname(__file__), "resampler_65_48", "taps_realtime.txt")
PREAMBLE = os.path.join(os.path.dirname(__file__), "reference_preamble.bin")

# QM35 schedule found in the capture: period 5 ms = 3686400 samples @737.28.
# t0 = first burst start (energy).
SLOT_PERIOD = 3686400
T0 = 3543552

# Window geometry @737.28: generous pre/post around the 160 us packet.
PRE = 30000
POST = 160000


def read_sc16(fn, lo, hi):
    raw = np.memmap(fn, dtype=np.int16, mode="r")
    w = raw[2 * lo:2 * hi].reshape(-1, 2).astype(np.float32)
    return w[:, 0] + 1j * w[:, 1]


def resample_65_48(x737):
    taps = np.fromfile(TAPS_R, np.float32)
    return upfirdn(taps, x737, 65, 48)


def main():
    import pmt as p
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    seed_first = "--seed-first-sync" in sys.argv
    fn = args[0] if args else (
        "/mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat")
    slots = args[1] if len(args) > 1 else "0"
    if slots == "--all":
        idxs = range(0, 100, 1)
    else:
        idxs = [int(x) for x in slots.split(",")]

    tmpl = np.fromfile(PREAMBLE, np.complex64)
    demod = __import__("gnuradio.uwb", fromlist=["uwb"]).realtime_demodulator
    d = demod.make_from_template(tmpl.tolist(), 2, 64, "4z2", 0, "bypass", 9, 64)
    dbg = __import__("gnuradio.blocks", fromlist=["blocks"]).message_debug()
    tb = __import__("gnuradio.gr", fromlist=["gr"]).top_block()
    tb.msg_connect(d, "result", dbg, "store")
    tb.start()

    for k in idxs:
        lo = T0 + k * SLOT_PERIOD - PRE
        hi = lo + PRE + POST
        x737 = read_sc16(fn, lo, hi)
        x998 = resample_65_48(x737)
        n_in = len(x998)
        # window_start_sample in 998.4 domain
        ws = int(round((lo) * 65.0 / 48.0))
        # Seed the timing stage: use the schedule (t0 mapped) by default, or a
        # correlation-found first SYNC when --seed-first-sync is set.
        if seed_first:
            c = np.convolve(x998, tmpl[::-1].conj(), "valid")
            wp = np.convolve(np.abs(x998) ** 2, np.ones(len(tmpl)), "valid")
            m = np.abs(c) ** 2 / (wp * np.sum(np.abs(tmpl) ** 2) + 1e-12)
            first = int(np.argmax(m[:40000]))
            seed = ws + first
        else:
            seed = int(round(lo * 65.0 / 48.0))
        meta = p.make_dict()
        meta = p.dict_add(meta, p.intern("packet_id"), p.from_uint64(k))
        meta = p.dict_add(meta, p.intern("predicted_start_sample"),
                          p.from_long(seed))
        meta = p.dict_add(meta, p.intern("window_start_sample"), p.from_long(ws))
        meta = p.dict_add(meta, p.intern("sample_count"), p.from_long(n_in))
        meta = p.dict_add(meta, p.intern("sample_rate"), p.from_double(FS998))
        vec = p.init_c32vector(n_in, x998.tolist())
        d._post(p.intern("samples"), p.cons(meta, vec))
        if dbg.num_messages() > 0:
            break
    deadline = time.time() + 30
    while time.time() < deadline and dbg.num_messages() < len(idxs):
        time.sleep(0.05)
    tb.stop()
    tb.wait()

    def as_int(v):
        try:
            return p.to_long(v)
        except Exception:
            return p.to_uint64(v)

    def as_num(v):
        for f in (p.to_long, p.to_uint64, p.to_double):
            try:
                return f(v)
            except Exception:
                pass
        return 0.0

    print(f"jobs rx={d.jobs_received()} completed={d.jobs_completed()} "
          f"failed={d.jobs_failed()} results={dbg.num_messages()}")
    for i in range(dbg.num_messages()):
        m = dbg.get_message(i)
        md = p.car(m) if p.is_pair(m) else m
        status = p.symbol_to_string(p.dict_ref(md, p.intern("status"), p.intern("?")))
        fcs = p.to_bool(p.dict_ref(md, p.intern("fcs_pass"), p.from_bool(False)))
        det = as_int(p.dict_ref(md, p.intern("detected_start_sample"), p.from_long(-1)))
        pk = as_int(p.dict_ref(md, p.intern("packet_id"), p.from_uint64(0)))
        ns_metric = p.to_double(p.dict_ref(md, p.intern("ns_sfd_metric"), p.from_double(-1)))
        ns_start = as_int(p.dict_ref(md, p.intern("ns_sfd_start_chip"), p.from_long(-1)))
        phr_len = as_int(p.dict_ref(md, p.intern("phr_psdu_length"), p.from_long(-1)))
        phr_corr = p.to_bool(p.dict_ref(md, p.intern("phr_corrected"), p.from_bool(False)))
        phr_unc = p.to_bool(p.dict_ref(md, p.intern("phr_uncorrectable"), p.from_bool(False)))
        pay_n = as_int(p.dict_ref(md, p.intern("payload_nbytes"), p.from_long(-1)))
        t_total = as_num(p.dict_ref(md, p.intern("stage_total_us"), p.from_double(-1)))
        print(f"slot {pk}: status={status} fcs={fcs} det_start={det} "
              f"ns_sfd_metric={ns_metric:.3f} ns_start_chip={ns_start} "
              f"phr_len={phr_len} phr_corr={phr_corr} phr_unc={phr_unc} "
              f"payload_bytes={pay_n} t_total={t_total:.0f}us")


if __name__ == "__main__":
    main()
