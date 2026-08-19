#!/usr/bin/env python3
"""X410 / file / synthetic entry for blind QM35 acquire + 5 ms scheduled capture.

No first_packet_sample.  The auto extractor finds the first confirmable QM35
packet on a native 737.28 MS/s SC16 stream and then locks EverySlot windows.

Sources:
  --source synthetic   generate a short native SC16 QM35-like stream (no radio)
  --source file        read interleaved int16 I/Q from --input
  --source x410        live UHD usrp_source (cpu=sc16, otw=sc16, 737.28 MS/s)

--dry-run prints the rate contract and exits 0.  --identity loopback posts
demod-style lock_obs for file/synthetic runs so the shipped block can lock
without a live UwbRealtimeDemodulator.

--identity demod is not wired here yet (no 65/48 + realtime demod + writer).
When that closed loop lands, dump vs demod geometry must stay split the
same way as testdata/offline_qm35_auto_lock.py: extractor long window for
native SC16 dump, UwbPduWindowCrop 10/190/4.1 before 65/48.  Do not send
the 590 µs dump PDU into the FIR.
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import os
import sys


def load_in_tree_uwb():
    """Prefer the just-built bindings over a stale site-packages gnuradio.uwb."""
    here = os.path.dirname(os.path.abspath(__file__))
    roots = [
        os.path.join(here, "..", "build", "python", "uwb", "bindings"),
        os.path.join(here, "..", "build", "test_modules", "gnuradio", "uwb"),
    ]
    matches = []
    for root in roots:
        matches.extend(glob.glob(os.path.join(os.path.abspath(root), "uwb_python*.so")))
    if not matches:
        from gnuradio import uwb
        return uwb
    import gnuradio.gr  # noqa: F401  — bindings need the base module
    spec = importlib.util.spec_from_file_location("uwb_python", matches[0])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

RADIO_RATE = 737.28e6
HOST_RATE = 998.4e6
INTERVAL_S = 0.005
PRE = 7373
CAPTURE = 140083
POST = 3023
DUMP_PRE = 221184   # 300 µs @737.28 — DW1000 head for offline SIC
DUMP_POST = 73728   # 100 µs


def parser():
    p = argparse.ArgumentParser()
    p.add_argument("--source", choices=("synthetic", "file", "x410"),
                   default="synthetic")
    p.add_argument("--input", default="", help="SC16 interleaved int16 file")
    p.add_argument("--template", default="",
                   help="native 737.28 CF32 one-SYNC template")
    p.add_argument("--args", default="addr=192.168.10.2")
    p.add_argument("--frequency", type=float, default=6489.6e6)
    p.add_argument("--gain", type=float, default=60.0)
    p.add_argument("--antenna", default="TX/RX0")
    p.add_argument("--packet-interval", type=float, default=INTERVAL_S)
    p.add_argument("--pre-guard", type=int, default=PRE,
                   help="scheduled / demod-crop pre-guard (default 7373)")
    p.add_argument("--capture", type=int, default=CAPTURE)
    p.add_argument("--post-guard", type=int, default=POST,
                   help="scheduled / demod-crop post-guard (default 3023)")
    p.add_argument("--dump-pre", type=int, default=DUMP_PRE,
                   help="native dump pre-guard when writer is attached")
    p.add_argument("--dump-post", type=int, default=DUMP_POST,
                   help="native dump post-guard when writer is attached")
    p.add_argument("--output", default="")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--identity", choices=("loopback", "none", "demod"),
                   default="loopback")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="live capture duration; 0 = run until source ends")
    p.add_argument("--seed", type=int, default=7)
    return p


def default_template():
    here = os.path.dirname(os.path.abspath(__file__))
    cand = [
        os.path.join(here, "..", "..", "testdata",
                     "reference_preamble_code9_737p28.cf32"),
        os.path.join(os.getcwd(), "testdata",
                     "reference_preamble_code9_737p28.cf32"),
    ]
    for c in cand:
        if os.path.isfile(c):
            return os.path.normpath(c)
    return cand[0]


def load_cf32(path):
    import numpy as np
    x = np.fromfile(path, dtype=np.complex64)
    if x.size == 0:
        raise RuntimeError(f"empty template {path}")
    n = float(np.linalg.norm(x))
    return x if n <= 0 else (x / n).astype(np.complex64)


def make_synthetic(tmpl, seed=7):
    import numpy as np
    rng = np.random.RandomState(seed)
    offset = int(rng.randint(2000, 8000))
    t0 = offset + 12000
    # Compact native-rate period for file/synthetic launch (still known T,
    # same block).  Live X410 uses the production 5 ms interval.
    period = 20000
    n_slots = 4
    n = t0 + n_slots * period + 8000
    iq = np.zeros(n, dtype=np.complex64)
    reps = 8
    for k in range(n_slots):
        s = t0 + k * period
        for r in range(reps):
            a = s + r * tmpl.size
            b = a + tmpl.size
            if b <= n:
                iq[a:b] = tmpl
    # communication energy that must not create extra locked windows
    for c in range(30):
        a = t0 + 150000 + c * (period // 16)
        b = min(n, a + 1500)
        if a < n:
            iq[a:b] += (0.4 * (rng.randn(b - a) + 1j * rng.randn(b - a))).astype(
                np.complex64)
    sc16 = np.empty(n * 2, dtype=np.int16)
    sc16[0::2] = np.clip(np.real(iq) * 20000.0, -32767, 32767).astype(np.int16)
    sc16[1::2] = np.clip(np.imag(iq) * 20000.0, -32767, 32767).astype(np.int16)
    return sc16, t0, offset, period


def _dict_get(meta, key, default=None):
    import pmt
    k = pmt.intern(key)
    if not pmt.is_dict(meta) or not pmt.dict_has_key(meta, k):
        return default
    v = pmt.dict_ref(meta, k, pmt.PMT_NIL)
    if pmt.is_uint64(v):
        return pmt.to_uint64(v)
    if pmt.is_integer(v):
        return pmt.to_long(v)
    if pmt.is_real(v):
        return pmt.to_double(v)
    if pmt.is_symbol(v):
        return pmt.symbol_to_string(v)
    return default


def print_pdus(dbg):
    import pmt
    starts = []
    preds = []
    modes = []
    n = dbg.num_messages()
    for i in range(n):
        msg = dbg.get_message(i)
        meta = pmt.car(msg) if pmt.is_pair(msg) else msg
        mode = str(_dict_get(meta, "capture_mode", ""))
        start = int(_dict_get(meta, "start_sample", 0))
        pred = int(_dict_get(meta, "predicted_start_sample", 0))
        modes.append(mode)
        starts.append(start)
        preds.append(pred)
        print(f"pdu[{i}] mode={mode} start_sample={start} "
              f"predicted_start_sample={pred}")
    print(f"window_count={n}")
    return starts, preds, n, modes


class IdentityLoopback:
    """Posts demod-style lock_obs so the shipped extractor can lock."""

    def __init__(self, ext):
        self.ext = ext
        self.confirmed = False

    def __call__(self, msg):
        import pmt
        meta = pmt.to_python(pmt.car(msg)) if pmt.is_pair(msg) else None
        if isinstance(meta, dict):
            def get(key, default=None):
                return meta.get(key, default)
            mode = str(get("capture_mode", ""))
            d = pmt.make_dict()
            d = pmt.dict_add(d, pmt.intern("command"), pmt.intern("observe"))
            d = pmt.dict_add(d, pmt.intern("status"), pmt.intern("success"))
            d = pmt.dict_add(d, pmt.intern("fcs_pass"), pmt.PMT_T)
            d = pmt.dict_add(d, pmt.intern("code_index"), pmt.from_long(9))
            d = pmt.dict_add(d, pmt.intern("preamble_repetitions"),
                             pmt.from_long(64))
            d = pmt.dict_add(d, pmt.intern("sfd_mode"), pmt.intern("4z2"))
            d = pmt.dict_add(d, pmt.intern("timing_ok"), pmt.PMT_T)
            d = pmt.dict_add(d, pmt.intern("sample_rate"),
                             pmt.from_double(RADIO_RATE))
            d = pmt.dict_add(d, pmt.intern("native_sample_rate"),
                             pmt.from_double(RADIO_RATE))
            d = pmt.dict_add(d, pmt.intern("acquisition_epoch"),
                             pmt.from_uint64(int(get("acquisition_epoch", 0))))
            d = pmt.dict_add(d, pmt.intern("schedule_generation"),
                             pmt.from_uint64(int(get("schedule_generation", 0))))
            start = int(get("start_sample", 0))
            pred = int(get("predicted_start_sample", start))
            d = pmt.dict_add(d, pmt.intern("detected_start_sample"),
                             pmt.from_long(pred if mode != "acquisition" else start))
            if get("schedule_index", None) is not None:
                d = pmt.dict_add(d, pmt.intern("schedule_index"),
                                 pmt.from_uint64(int(get("schedule_index", 0))))
            self.ext.post_lock_obs(d)
            self.confirmed = True
            return
        meta = pmt.car(msg) if pmt.is_pair(msg) else msg
        if not pmt.is_dict(meta):
            return
        def get(key, default=None):
            k = pmt.intern(key)
            if not pmt.dict_has_key(meta, k):
                return default
            v = pmt.dict_ref(meta, k, pmt.PMT_NIL)
            if pmt.is_uint64(v):
                return pmt.to_uint64(v)
            if pmt.is_integer(v):
                return pmt.to_long(v)
            if pmt.is_real(v):
                return pmt.to_double(v)
            if pmt.is_symbol(v):
                return pmt.symbol_to_string(v)
            return default
        mode = get("capture_mode", "")
        d = pmt.make_dict()
        d = pmt.dict_add(d, pmt.intern("command"), pmt.intern("observe"))
        d = pmt.dict_add(d, pmt.intern("status"), pmt.intern("success"))
        d = pmt.dict_add(d, pmt.intern("fcs_pass"), pmt.PMT_T)
        d = pmt.dict_add(d, pmt.intern("code_index"), pmt.from_long(9))
        d = pmt.dict_add(d, pmt.intern("preamble_repetitions"), pmt.from_long(64))
        d = pmt.dict_add(d, pmt.intern("sfd_mode"), pmt.intern("4z2"))
        d = pmt.dict_add(d, pmt.intern("timing_ok"), pmt.PMT_T)
        d = pmt.dict_add(d, pmt.intern("sample_rate"), pmt.from_double(RADIO_RATE))
        d = pmt.dict_add(d, pmt.intern("native_sample_rate"),
                         pmt.from_double(RADIO_RATE))
        d = pmt.dict_add(d, pmt.intern("acquisition_epoch"),
                         pmt.from_uint64(int(get("acquisition_epoch", 0))))
        d = pmt.dict_add(d, pmt.intern("schedule_generation"),
                         pmt.from_uint64(int(get("schedule_generation", 0))))
        start = int(get("start_sample", 0))
        pred = int(get("predicted_start_sample", start))
        d = pmt.dict_add(d, pmt.intern("detected_start_sample"),
                         pmt.from_long(pred if mode != "acquisition" else start))
        if get("schedule_index", None) is not None:
            d = pmt.dict_add(d, pmt.intern("schedule_index"),
                             pmt.from_uint64(int(get("schedule_index", 0))))
        self.ext.post_lock_obs(d)
        self.confirmed = True


def run_flow(a, tmpl_path, iq=None):
    import pmt
    from gnuradio import blocks, gr
    uwb = load_in_tree_uwb()

    tmpl = load_cf32(tmpl_path)
    ext_kwargs = dict(
        known_preamble=[complex(x) for x in tmpl],
        sample_rate=RADIO_RATE,
        packet_interval_s=a.packet_interval,
        pre_guard_samples=a.pre_guard,
        capture_samples=a.capture,
        post_guard_samples=a.post_guard,
    )
    if a.source == "synthetic":
        ext_kwargs.update(
            energy_threshold=1e-6,
            energy_gate_decimation=4,
            acquire_pre_trigger=128,
            acquire_capture=2048,
        )
    ext = uwb.auto_scheduled_extractor_sc16(**ext_kwargs)
    dbg = blocks.message_debug()
    tb = gr.top_block("x410_auto_scheduled_capture")
    if a.source in ("synthetic", "file"):
        src = blocks.vector_source_s(iq.tolist(), False, 2)
        tb.connect(src, ext)
    else:
        raise RuntimeError("x410 path is handled in main()")
    tb.msg_connect(ext, "packet", dbg, "store")
    loop = IdentityLoopback(ext) if a.identity == "loopback" else None
    if loop is not None:
        class _Sink(gr.sync_block):
            def __init__(self, cb):
                gr.sync_block.__init__(
                    self, name="identity_loop",
                    in_sig=None, out_sig=None)
                self._cb = cb
                self.message_port_register_in(pmt.intern("packet"))
                self.set_msg_handler(pmt.intern("packet"), self.handle_packet)

            def handle_packet(self, msg):
                self._cb(msg)
        sink = _Sink(loop)
        tb.msg_connect(ext, "packet", sink, "packet")
    tb.run()
    starts, preds, n, modes = print_pdus(dbg)
    print(f"lock_state={ext.lock_state_name()} identity={ext.identity_confirmed()} "
          f"energy_after_lock={ext.energy_regions_after_lock()} "
          f"emitted={ext.emitted_windows()} scheduled={ext.scheduled_windows()}")
    return starts, preds, n, modes, ext.lock_state_name()


def main():
    a = parser().parse_args()
    tmpl = a.template or default_template()
    print(f"rate contract: X410 Radio {RADIO_RATE:.0f} S/s SC16 native "
          f"(no RFNoC 65/48). host demod rate {HOST_RATE:.0f} S/s")
    print(f"template={tmpl}")
    print("no first_packet_sample; acquire then lock T=5 ms")
    if a.dry_run:
        print("dry-run ok")
        return 0

    if a.source == "x410":
        try:
            from gnuradio import gr, uhd
            uwb = load_in_tree_uwb()
        except Exception as exc:
            print(f"x410 unavailable: {exc}")
            return 2
        try:
            src = uhd.usrp_source(
                a.args,
                uhd.stream_args(cpu_format="sc16", otw_format="sc16",
                                channels=[0]))
            src.set_samp_rate(RADIO_RATE)
            src.set_center_freq(a.frequency, 0)
            src.set_gain(a.gain, 0)
            try:
                src.set_antenna(a.antenna, 0)
            except Exception:
                pass
        except Exception as exc:
            print(f"x410 unavailable: {exc}")
            return 2
        tmpl_wf = load_cf32(tmpl)
        ext = uwb.auto_scheduled_extractor_sc16(
            list(tmpl_wf), RADIO_RATE, a.packet_interval,
            a.pre_guard, a.capture, a.post_guard)

        import pmt
        class OverflowToControl(gr.sync_block):
            """UHD 4.6 overflow is async-only; forward it as control disc."""

            def __init__(self, extractor):
                gr.sync_block.__init__(
                    self, name="uhd_overflow_to_control",
                    in_sig=None, out_sig=None)
                self.ext = extractor
                self.message_port_register_in(pmt.intern("async_msgs"))
                self.set_msg_handler(pmt.intern("async_msgs"),
                                     self.handle_async)

            def handle_async(self, msg):
                # UHD 4.6 usrp_source_impl publishes
                #   cons("uhd_async_msg", dict{overflows: count})
                # on port "async_msgs" (no overflow stream tag).
                d = msg
                if pmt.is_pair(msg) and not pmt.is_dict(msg):
                    d = pmt.cdr(msg)
                if pmt.is_pair(d) and pmt.is_dict(pmt.car(d)):
                    d = pmt.car(d)
                if not pmt.is_dict(d):
                    return
                if not pmt.dict_has_key(d, pmt.intern("overflows")):
                    return
                c = pmt.make_dict()
                c = pmt.dict_add(c, pmt.intern("command"),
                                 pmt.intern("discontinuity"))
                c = pmt.dict_add(c, pmt.intern("reason"),
                                 pmt.intern("overflow"))
                self.ext.post_control(c)

        ovf = OverflowToControl(ext)
        tb = gr.top_block("x410_auto_live")
        tb.connect(src, ext)
        tb.msg_connect(src, "async_msgs", ovf, "async_msgs")
        tb.start(1048576)
        import time
        time.sleep(max(a.seconds, 1.0))
        tb.stop()
        tb.wait()
        print(f"lock_state={ext.lock_state_name()} emitted={ext.emitted_windows()} "
              f"disc={ext.discontinuities()}")
        return 0 if ext.identity_confirmed() else 3

    if a.source == "file":
        if not a.input or not os.path.isfile(a.input):
            raise SystemExit(f"missing --input {a.input}")
        import numpy as np
        iq = np.fromfile(a.input, dtype=np.int16)
        if iq.size % 2:
            iq = iq[:-1]
    else:
        tmpl_wf = load_cf32(tmpl)
        iq, t0, offset, period = make_synthetic(tmpl_wf, a.seed)
        a.packet_interval = period / RADIO_RATE
        a.pre_guard = 128
        a.capture = 2048
        a.post_guard = 32
        print(f"synthetic t0={t0} offset={offset} period={period} n={iq.size//2} "
              f"interval_s={a.packet_interval}")

    starts, preds, n, modes, state = run_flow(a, tmpl, iq)
    if n == 0 or "acquisition" not in modes:
        print("FAIL: no acquisition PDU")
        return 1
    if state not in ("provisional", "locked") and "provisional" not in modes \
            and "scheduled" not in modes:
        print("FAIL: identity did not start scheduled capture")
        return 1
    print(f"SUCCESS windows={n} lock={state}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
