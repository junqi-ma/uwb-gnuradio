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

--source x410 --identity demod is the live host path:

  usrp_source SC16 @737.28
    -> UwbAutoScheduledExtractorSc16
         --output: 590 µs native dump window
           -> Writer (capture.iq, no 65/48)
           -> UwbPduWindowCrop 10/190/4.1 µs
                -> PDU 65/48 -> UwbRealtimeDemodulator
         no dump: extractor already emits the short demod window
           -> PDU 65/48 -> UwbRealtimeDemodulator
    -> schedule_feedback -> lock_obs

--output only opens the Writer.  It must not enlarge the FIR window.
After a 0-drop dump, uwb_offline_postprocess_dump notches each window
and upsamples to 998.4 SC16 without overwriting capture.iq.
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import os
import shutil
import subprocess
import sys


def load_in_tree_uwb():
    """Prefer the just-built bindings over a stale site-packages gnuradio.uwb."""
    here = os.path.dirname(os.path.abspath(__file__))
    libdir = os.path.abspath(os.path.join(here, "..", "build", "lib"))
    so_lib = os.path.join(libdir, "libgnuradio-uwb.so")
    if os.path.isfile(so_lib):
        import ctypes
        ctypes.CDLL(so_lib, mode=ctypes.RTLD_GLOBAL)
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
# Native dump: 300 µs head / 190 µs body / 100 µs tail.  FIR must crop
# to PRE/CAPTURE/POST before 65/48; do not send this window into the resampler.
PRE_DW = 221184
POST_DW = 73728


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
                   help="demod-crop pre-guard (default 7373 = 10 µs)")
    p.add_argument("--capture", type=int, default=CAPTURE,
                   help="QM35 body samples (dump and demod, default 140083)")
    p.add_argument("--post-guard", type=int, default=POST,
                   help="demod-crop post-guard (default 3023 = 4.1 µs)")
    p.add_argument("--demod-pre", type=int, default=0,
                   help="override demod crop pre-guard")
    p.add_argument("--demod-capture", type=int, default=0,
                   help="override demod crop body")
    p.add_argument("--demod-post", type=int, default=0,
                   help="override demod crop post-guard")
    p.add_argument("--dump-pre", type=int, default=0,
                   help="dump pre-guard when --output is set (default 221184)")
    p.add_argument("--dump-capture", type=int, default=0,
                   help="dump QM35 body when --output is set")
    p.add_argument("--dump-post", type=int, default=0,
                   help="dump post-guard when --output is set (default 73728)")
    p.add_argument(
        "--output", default="",
        help="write native 737.28 SC16 dump windows to DIR/capture.iq; "
             "does not enlarge the 65/48 FIR window")
    p.add_argument("--skip-postprocess", action="store_true",
                   help="do not run uwb_offline_postprocess_dump after dump")
    p.add_argument("--postprocess-bin", default="",
                   help="path to uwb_offline_postprocess_dump")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--identity", choices=("loopback", "none", "demod"),
                   default="loopback")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="live capture duration; 0 = 15 s on x410, else until EOS")
    p.add_argument("--workers", type=int, default=2,
                   help="UwbRealtimeDemodulator worker count")
    p.add_argument("--energy-threshold", type=float, default=0.02)
    p.add_argument("--cir-filter-mode", default="bypass")
    p.add_argument("--template-998", default="",
                   help="998.4 CF32 one-SYNC template for the demodulator")
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


def default_template_998():
    here = os.path.dirname(os.path.abspath(__file__))
    cand = [
        os.path.join(here, "..", "..", "testdata", "reference_preamble.bin"),
        os.path.join(os.getcwd(), "testdata", "reference_preamble.bin"),
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


def resolve_geometry(a, write_dir):
    demod_pre = a.demod_pre if a.demod_pre > 0 else a.pre_guard
    demod_cap = a.demod_capture if a.demod_capture > 0 else a.capture
    demod_post = a.demod_post if a.demod_post > 0 else a.post_guard
    dump_pre = a.dump_pre if a.dump_pre > 0 else PRE_DW
    dump_cap = a.dump_capture if a.dump_capture > 0 else demod_cap
    dump_post = a.dump_post if a.dump_post > 0 else POST_DW
    return {
        "demod_pre": int(demod_pre),
        "demod_cap": int(demod_cap),
        "demod_post": int(demod_post),
        "dump_pre": int(dump_pre),
        "dump_cap": int(dump_cap),
        "dump_post": int(dump_post),
    }


def find_postprocess_bin(explicit):
    if explicit:
        return explicit
    env = os.environ.get("UWB_OFFLINE_POSTPROCESS", "")
    if env and os.path.isfile(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(here, "..", "build", "apps", "uwb_offline_postprocess_dump"),
        os.path.join(here, "..", "..", "build", "apps",
                     "uwb_offline_postprocess_dump"),
    ]
    for c in cands:
        c = os.path.normpath(c)
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return shutil.which("uwb_offline_postprocess_dump") or ""


def run_offline_postprocess(write_dir, explicit_bin):
    print()
    print("=== Offline postprocess (notch + 65/48) ===")
    bin_path = find_postprocess_bin(explicit_bin)
    if not bin_path:
        print("  FAIL: uwb_offline_postprocess_dump not found "
              "(build gr-uwb/apps or pass --postprocess-bin)")
        return 2
    cmd = [bin_path, write_dir, "--tone-rf-hz", "6256.640e6",
           "--out-format", "sc16"]
    print("  " + " ".join(cmd), flush=True)
    try:
        rc = subprocess.call(cmd)
    except OSError as e:
        print(f"  FAIL: could not exec {bin_path}: {e}")
        return 2
    out = os.path.join(write_dir, "capture_998p4.iq")
    meta = os.path.join(write_dir, "capture_998p4.jsonl")
    if rc != 0:
        print(f"  FAIL: postprocess exit {rc}")
        return rc
    if not os.path.isfile(out) or not os.path.isfile(meta):
        print(f"  FAIL: missing {out} or {meta}")
        return 2
    print(f"  raw unchanged: {os.path.join(write_dir, 'capture.iq')}")
    print(f"  wrote {out}")
    print(f"  wrote {meta}")
    return 0


def write_demod_results(path, packets, rows):
    by_id = {int(x.get("packet_id", -1)): x for x in packets}
    with open(path, "w") as f:
        for r in rows:
            src = by_id.get(int(r.get("id", -1)), {})
            rec = {
                "packet_id": int(r.get("id", -1)),
                "schedule_index": int(src.get("schedule_index", -1)),
                "status": r.get("status", ""),
                "fcs_pass": bool(r.get("fcs", False)),
                "detected_start_sample": int(r.get("det", -1)),
                "predicted_start_sample": int(r.get("pred", -1)),
                "native_predicted_start": int(
                    src.get("predicted_start_sample", -1)),
                "native_window_start": int(
                    src.get("window_start_sample", -1)),
                "resample_us": r.get("resample_us", float("nan")),
                "t_total_us": r.get("t_total_us", float("nan")),
            }
            f.write(json.dumps(rec) + "\n")
    return path


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


def _pmt_dict_get(meta, key, default=None):
    import pmt
    k = pmt.intern(key)
    if not pmt.is_dict(meta) or not pmt.dict_has_key(meta, k):
        return default
    v = pmt.dict_ref(meta, k, pmt.PMT_NIL)
    if pmt.is_bool(v):
        return bool(pmt.to_bool(v))
    if pmt.is_uint64(v):
        return pmt.to_uint64(v)
    if pmt.is_integer(v):
        return pmt.to_long(v)
    if pmt.is_real(v):
        return pmt.to_double(v)
    if pmt.is_symbol(v):
        return pmt.symbol_to_string(v)
    return default


class PacketTap:
    """Keep extractor PDU metadata (native 737.28 coordinates)."""

    def __init__(self):
        from gnuradio import gr
        import pmt

        class _Blk(gr.sync_block):
            def __init__(self):
                gr.sync_block.__init__(
                    self, name="packet_meta_tap",
                    in_sig=None, out_sig=None)
                self.rows = []
                self.message_port_register_in(pmt.intern("packet"))
                self.set_msg_handler(pmt.intern("packet"), self.handle)

            def handle(self, msg):
                meta = pmt.car(msg) if pmt.is_pair(msg) else msg
                self.rows.append({
                    "packet_id": int(_pmt_dict_get(meta, "packet_id", 0) or 0),
                    "capture_mode": str(
                        _pmt_dict_get(meta, "capture_mode", "")),
                    "schedule_index": int(
                        _pmt_dict_get(meta, "schedule_index", -1) or -1),
                    "predicted_start_sample": int(
                        _pmt_dict_get(meta, "predicted_start_sample", -1)
                        or -1),
                    "window_start_sample": int(
                        _pmt_dict_get(meta, "window_start_sample", -1) or -1),
                    "sample_count": int(
                        _pmt_dict_get(meta, "sample_count", -1) or -1),
                    "pre_guard_samples": int(
                        _pmt_dict_get(meta, "pre_guard_samples", -1) or -1),
                    "capture_samples": int(
                        _pmt_dict_get(meta, "capture_samples", -1) or -1),
                    "post_guard_samples": int(
                        _pmt_dict_get(meta, "post_guard_samples", -1) or -1),
                })

        self.blk = _Blk()

    @property
    def rows(self):
        return self.blk.rows


class OverflowToControl:
    """UHD 4.6 overflow is async-only; forward it as control disc."""

    def __init__(self, extractor):
        from gnuradio import gr
        import pmt

        class _Blk(gr.sync_block):
            def __init__(self, ext):
                gr.sync_block.__init__(
                    self, name="uhd_overflow_to_control",
                    in_sig=None, out_sig=None)
                self.ext = ext
                self.overflows = 0
                self.message_port_register_in(pmt.intern("async_msgs"))
                self.set_msg_handler(pmt.intern("async_msgs"),
                                     self.handle_async)

            def handle_async(self, msg):
                d = msg
                if pmt.is_pair(msg) and not pmt.is_dict(msg):
                    d = pmt.cdr(msg)
                if pmt.is_pair(d) and pmt.is_dict(pmt.car(d)):
                    d = pmt.car(d)
                if not pmt.is_dict(d):
                    return
                if not pmt.dict_has_key(d, pmt.intern("overflows")):
                    return
                self.overflows += 1
                c = pmt.make_dict()
                c = pmt.dict_add(c, pmt.intern("command"),
                                 pmt.intern("discontinuity"))
                c = pmt.dict_add(c, pmt.intern("reason"),
                                 pmt.intern("overflow"))
                self.ext.post_control(c)

        self.blk = _Blk(extractor)

    @property
    def overflows(self):
        return self.blk.overflows


class ResultTap:
    """Collect demod result metadata without storing IQ."""

    def __init__(self):
        from gnuradio import gr
        import pmt

        class _Blk(gr.sync_block):
            def __init__(self):
                gr.sync_block.__init__(
                    self, name="demod_result_tap",
                    in_sig=None, out_sig=None)
                self.rows = []
                self.fcs_ok = 0
                self.message_port_register_in(pmt.intern("result"))
                self.set_msg_handler(pmt.intern("result"), self.handle)

            def handle(self, msg):
                meta = pmt.car(msg) if pmt.is_pair(msg) else msg
                status = str(_pmt_dict_get(meta, "status", "?"))
                fcs = bool(_pmt_dict_get(meta, "fcs_pass", False))
                row = {
                    "id": int(_pmt_dict_get(meta, "packet_id", 0) or 0),
                    "status": status,
                    "fcs": fcs,
                    "mode": str(_pmt_dict_get(meta, "capture_mode", "")),
                    "det": int(_pmt_dict_get(meta, "detected_start_sample", -1)
                               or -1),
                    "pred": int(_pmt_dict_get(meta, "predicted_start_sample", -1)
                                or -1),
                    "peaks": int(_pmt_dict_get(meta, "timing_peaks", -1) or -1),
                    "metric": float(_pmt_dict_get(meta, "timing_metric", -1.0)
                                    or -1.0),
                    "cfo": float(_pmt_dict_get(meta, "cfo_hz", 0.0) or 0.0),
                    "sfd": float(_pmt_dict_get(meta, "sfd_metric", -1.0)
                                 or -1.0),
                    "nbytes": int(_pmt_dict_get(meta, "payload_nbytes", -1)
                                  or -1),
                    "resample_us": float(
                        _pmt_dict_get(meta, "resample_us", float("nan"))
                        or float("nan")),
                    "t_total_us": float(
                        _pmt_dict_get(meta, "stage_total_us", float("nan"))
                        or float("nan")),
                }
                self.rows.append(row)
                if fcs:
                    self.fcs_ok += 1

        self.blk = _Blk()

    @property
    def rows(self):
        return self.blk.rows

    @property
    def fcs_ok(self):
        return self.blk.fcs_ok


def _normalize_x410_args(args):
    """Kernel UDP on the live 100GbE NIC.

    Do not inject use_dpdk=0: that key still loads DPDK EAL, and this host's
    /etc/uhd/uhd.conf DPDK section points at the unused ConnectX port.
    """
    text = (args or "addr=192.168.10.2").strip()
    if "recv_buff_size" not in text:
        text = text + ",recv_buff_size=250000000"
    return text


def run_x410_live(a, tmpl_path):
    """Blind-acquire QM35 on live X410 SC16 and demodulate."""
    import platform
    import time
    from gnuradio import gr, uhd

    uwb = load_in_tree_uwb()
    tmpl737 = load_cf32(tmpl_path)
    seconds = a.seconds if a.seconds > 0 else 15.0
    dev_args = _normalize_x410_args(a.args)
    write_dir = (a.output or "").strip()

    print(f"GNU Radio {gr.version()}")
    print(f"compiler={platform.python_compiler()} cpu={platform.processor() or platform.machine()}")
    print("sample_format=sc16 sample_rate=737280000")
    print(f"device_args={dev_args}")
    print(f"rf freq={a.frequency:.1f} gain={a.gain} antenna={a.antenna}")
    print(f"identity={a.identity} duration_s={seconds:.1f} "
          f"{'(native SC16 dump)' if write_dir else '(no disk writer)'}")

    src = uhd.usrp_source(
        dev_args,
        uhd.stream_args(cpu_format="sc16", otw_format="sc16", channels=[0]))
    try:
        src.set_clock_source("internal")
        src.set_time_source("internal")
    except Exception as exc:
        print(f"clock/time source warning: {exc}")
    src.set_samp_rate(RADIO_RATE)
    actual = float(src.get_samp_rate())
    print(f"usrp samp_rate requested={RADIO_RATE:.0f} actual={actual:.0f}")
    if abs(actual - RADIO_RATE) > 1.0:
        raise RuntimeError(
            f"X410 coerced sample rate to {actual}, expected {RADIO_RATE}")
    src.set_center_freq(a.frequency, 0)
    src.set_gain(a.gain, 0)
    try:
        src.set_antenna(a.antenna, 0)
    except Exception as exc:
        print(f"antenna {a.antenna!r} rejected ({exc}); leaving default")
    print(f"usrp tuned freq={src.get_center_freq(0):.1f} "
          f"gain={src.get_gain(0)} antenna={src.get_antenna(0)}")

    geom = resolve_geometry(a, write_dir)
    demod_pre = geom["demod_pre"]
    demod_cap = geom["demod_cap"]
    demod_post = geom["demod_post"]
    demod_n = demod_pre + demod_cap + demod_post
    ext_pre = geom["dump_pre"] if write_dir else demod_pre
    ext_cap = geom["dump_cap"] if write_dir else demod_cap
    ext_post = geom["dump_post"] if write_dir else demod_post
    if write_dir:
        os.makedirs(write_dir, exist_ok=True)
        print(f"write_sc16={write_dir}  "
              f"dump_head={ext_pre/RADIO_RATE*1e6:.1f} us  "
              f"dump_body={ext_cap/RADIO_RATE*1e6:.1f} us  "
              f"dump_tail={ext_post/RADIO_RATE*1e6:.1f} us  "
              f"fir_in={demod_n} samples "
              f"({demod_n/RADIO_RATE*1e6:.1f} us)")
    print(f"extractor=[{ext_pre},{ext_cap},{ext_post}]  "
          f"demod_crop=[{demod_pre},{demod_cap},{demod_post}]")

    ext = uwb.auto_scheduled_extractor_sc16(
        list(tmpl737), RADIO_RATE, a.packet_interval,
        ext_pre, ext_cap, ext_post,
        float(a.energy_threshold))
    ovf = OverflowToControl(ext)
    tb = gr.top_block("x410_auto_live_demod")
    tb.connect(src, ext)
    tb.msg_connect(src, "async_msgs", ovf.blk, "async_msgs")
    writer = None
    pkt_tap = PacketTap()
    tb.msg_connect(ext, "packet", pkt_tap.blk, "packet")
    if write_dir:
        writer = uwb.packet_writer(write_dir, "capture", False)
        tb.msg_connect(ext, "packet", writer, "packet")

    demod = None
    resampler = None
    crop = None
    tap = None
    if a.identity == "demod":
        tmpl998_path = a.template_998 or default_template_998()
        if not os.path.isfile(tmpl998_path):
            raise RuntimeError(f"missing 998.4 template {tmpl998_path}")
        tmpl998 = load_cf32(tmpl998_path)
        print(f"template_998={tmpl998_path} n={tmpl998.size} "
              f"workers={a.workers} cir={a.cir_filter_mode}")
        resampler = uwb.pdu_rational_resampler_ccf_65_48("quality_minorder")
        demod = uwb.realtime_demodulator.make_from_template(
            tmpl998.tolist(),
            max(1, int(a.workers)),
            64,
            "4z2",
            0,
            a.cir_filter_mode,
            9,
            64,
            14,
        )
        tap = ResultTap()
        if write_dir:
            crop = uwb.pdu_window_crop(demod_pre, demod_cap, demod_post)
            tb.msg_connect(ext, "packet", crop, "packet")
            tb.msg_connect(crop, "packet", resampler, "packet")
        else:
            tb.msg_connect(ext, "packet", resampler, "packet")
        tb.msg_connect(resampler, "packet", demod, "samples")
        tb.msg_connect(demod, "schedule_feedback", ext, "lock_obs")
        tb.msg_connect(demod, "result", tap.blk, "result")
    elif a.identity == "loopback":
        loop = IdentityLoopback(ext)

        class _Sink(gr.sync_block):
            def __init__(self, cb):
                import pmt
                gr.sync_block.__init__(
                    self, name="identity_loop", in_sig=None, out_sig=None)
                self._cb = cb
                self.message_port_register_in(pmt.intern("packet"))
                self.set_msg_handler(pmt.intern("packet"), self.handle_packet)

            def handle_packet(self, msg):
                self._cb(msg)

        sink = _Sink(loop)
        tb.msg_connect(ext, "packet", sink, "packet")

    print("starting live flowgraph (no t0"
          f"{', write native SC16' if writer is not None else ', no writer'})...",
          flush=True)
    def _cnt(obj, name):
        fn = getattr(obj, name, None)
        if callable(fn):
            try:
                return fn()
            except Exception:
                return 0
        return 0 if fn is None else fn

    t0 = time.time()
    tb.start(1048576)
    last = 0.0
    t_energy = t_cand = t_ident = t_prov = t_lock = None
    try:
        while time.time() - t0 < seconds:
            now = time.time() - t0
            if t_energy is None and _cnt(ext, "energy_regions") > 0:
                t_energy = now
                print(f"  MARK first_energy_region {t_energy*1e3:.1f} ms",
                      flush=True)
            if t_cand is None and _cnt(ext, "candidates_emitted") > 0:
                t_cand = now
                print(f"  MARK first_candidate {t_cand*1e3:.1f} ms",
                      flush=True)
            if t_ident is None and _cnt(ext, "identity_confirmed"):
                t_ident = now
                print(f"  MARK identity_confirmed {t_ident*1e3:.1f} ms",
                      flush=True)
            st = ext.lock_state_name()
            if t_prov is None and st == "provisional":
                t_prov = now
                print(f"  MARK provisional {t_prov*1e3:.1f} ms", flush=True)
            if t_lock is None and st == "locked":
                t_lock = now
                print(f"  MARK schedule_locked {t_lock*1e3:.1f} ms", flush=True)
            if now - last >= 1.0:
                last = now
                msg = (
                    f"  t={now:5.1f}s state={ext.lock_state_name()} "
                    f"ident={bool(_cnt(ext, 'identity_confirmed'))} "
                    f"energy={_cnt(ext, 'energy_regions')} "
                    f"cand={_cnt(ext, 'candidates_emitted')}/"
                    f"rej={_cnt(ext, 'candidates_rejected')} "
                    f"sched={_cnt(ext, 'scheduled_windows')} "
                    f"emit={_cnt(ext, 'emitted_windows')} "
                    f"drop={_cnt(ext, 'dropped_windows')} "
                    f"pool={_cnt(ext, 'pool_drops')} "
                    f"qfull={_cnt(ext, 'queue_full_drops')} "
                    f"disc={_cnt(ext, 'discontinuities')} "
                    f"ovf={ovf.overflows}"
                )
                if demod is not None:
                    msg += (
                        f" demod_rx={demod.jobs_received()} "
                        f"comp={demod.jobs_completed()} "
                        f"fail={demod.jobs_failed()} "
                        f"ddrop={demod.jobs_dropped()} "
                        f"fcs={tap.fcs_ok}/{len(tap.rows)}"
                    )
                if writer is not None:
                    msg += (
                        f" wr={writer.packets_written()} "
                        f"wrdrop={writer.packets_dropped()}"
                    )
                if crop is not None:
                    msg += (
                        f" crop={crop.pdus_cropped()}/"
                        f"pass={crop.pdus_passthrough()}"
                    )
                print(msg, flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("interrupted", flush=True)
    if demod is not None:
        try:
            demod.drain()
        except Exception:
            pass
        time.sleep(0.2)
    tb.stop()
    tb.wait()
    wall = time.time() - t0

    print()
    print("=== Live QM35 demod outcome"
          f"{' (native SC16 dump)' if writer is not None else ' (no dump)'} ===")
    print(f"wall_s={wall:.2f}")
    print("=== Lock timing (from tb.start) ===")
    def _ms(x):
        return f"  {x*1e3:.1f} ms" if x is not None else "  (none)"
    print(f"first_energy_region{_ms(t_energy)}")
    print(f"first_candidate{_ms(t_cand)}")
    print(f"identity_confirmed{_ms(t_ident)}")
    print(f"provisional{_ms(t_prov)}")
    print(f"schedule_locked{_ms(t_lock)}")
    print(f"lock_state={ext.lock_state_name()}")
    print(f"identity_confirmed={ext.identity_confirmed()}")
    print(f"locked_t0_native={ext.locked_t0():.3f} "
          f"({ext.locked_t0()/RADIO_RATE*1e3:.3f} ms)")
    print(f"locked_period_s={ext.locked_period_s():.9f}")
    print(f"energy_regions={_cnt(ext, 'energy_regions')} "
          f"after_lock={_cnt(ext, 'energy_regions_after_lock')}")
    print(f"candidates_emitted={_cnt(ext, 'candidates_emitted')} "
          f"rejected={_cnt(ext, 'candidates_rejected')}")
    print(f"scheduled_windows={_cnt(ext, 'scheduled_windows')} "
          f"emitted={_cnt(ext, 'emitted_windows')} "
          f"dropped={_cnt(ext, 'dropped_windows')} "
          f"pool_drops={_cnt(ext, 'pool_drops')} "
          f"queue_full={_cnt(ext, 'queue_full_drops')}")
    print(f"stale_feedback={ext.stale_feedback()} "
          f"unmapped={ext.unmapped_feedback()} "
          f"disc={ext.discontinuities()} ovf_async={ovf.overflows}")
    if writer is not None:
        print(f"writer dir={write_dir} "
              f"recv={writer.packets_received()} "
              f"written={writer.packets_written()} "
              f"dropped={writer.packets_dropped()} "
              f"samples={writer.samples_written()} "
              f"qhwm={writer.queue_high_watermark()}")
    if crop is not None:
        print(f"crop received={crop.pdus_received()} "
              f"emitted={crop.pdus_emitted()} "
              f"cropped={crop.pdus_cropped()} "
              f"passthrough={crop.pdus_passthrough()} "
              f"clamped={crop.pdus_clamped()}")
    if resampler is not None:
        print(f"resampler emitted={resampler.pdus_emitted()} "
              f"dropped={resampler.pdus_dropped()}")
        if resampler.pdus_emitted() > 0:
            mean_in = resampler.total_input_samples() / resampler.pdus_emitted()
            print(f"resampler mean_input_samples={mean_in:.1f} "
                  f"(demod_window={demod_n})")
    if demod is not None:
        print(f"demod rx={demod.jobs_received()} "
              f"completed={demod.jobs_completed()} "
              f"failed={demod.jobs_failed()} "
              f"dropped={demod.jobs_dropped()} "
              f"qhwm={demod.queue_high_watermark()}")
        print(f"fcs_pass={tap.fcs_ok}/{len(tap.rows)}")
        from collections import Counter
        rows = tap.rows
        hist = Counter(r["status"] for r in rows)
        print(f"demod_status_hist={dict(hist)}")
        n = len(rows)
        if n >= 40:
            early = Counter(r["status"] for r in rows[: n // 2])
            late = Counter(r["status"] for r in rows[n // 2 :])
            print(f"demod_status_hist_first_half={dict(early)}")
            print(f"demod_status_hist_second_half={dict(late)}")
        print("=== Demod status by ~1s bucket (200 pkts) ===")
        bucket = 200
        for i in range(0, n, bucket):
            chunk = rows[i:i + bucket]
            print(f"  id {i:4d}-{i + len(chunk) - 1:<4d} "
                  f"{dict(Counter(r['status'] for r in chunk))}")

        def dpred(r):
            if r["det"] < 0 or r["pred"] < 0:
                return None
            return r["det"] - r["pred"]

        print("=== Timing / SFD by status ===")
        for st, grp in sorted(
                {s: [r for r in rows if r["status"] == s] for s in hist}.items(),
                key=lambda kv: -len(kv[1])):
            mets = [r["metric"] for r in grp if r["metric"] >= 0]
            sfds = [r["sfd"] for r in grp if r["sfd"] >= 0]
            cfos = [abs(r["cfo"]) for r in grp]
            dps = [dpred(r) for r in grp if dpred(r) is not None]
            peaks = [r["peaks"] for r in grp if r["peaks"] >= 0]
            def med(xs):
                if not xs:
                    return float("nan")
                ys = sorted(xs)
                return ys[len(ys) // 2]
            print(
                f"  {st:<16} n={len(grp):4d} "
                f"tmet_med={med(mets):6.3f} sfd_med={med(sfds):6.3f} "
                f"peaks_med={med(peaks):5.1f} |cfo|_med={med(cfos):8.1f} "
                f"dpred_med={med(dps) if dps else float('nan'):+.0f}")

        fails = [r for r in rows if r["status"] != "success"]
        if fails:
            first_fail = fails[0]
            print(
                f"first_fail id={first_fail['id']} status={first_fail['status']} "
                f"sfd={first_fail['sfd']:.3f} tmet={first_fail['metric']:.3f} "
                f"cfo={first_fail['cfo']:.1f} peaks={first_fail['peaks']} "
                f"dpred={dpred(first_fail)} nbytes={first_fail['nbytes']}")
            print("=== Last 12 non-success demod results ===")
            for row in fails[-12:]:
                print(
                    f"  id={row['id']} status={row['status']} "
                    f"fcs={int(row['fcs'])} det={row['det']} "
                    f"tmet={row['metric']:.3f} sfd={row['sfd']:.3f} "
                    f"cfo={row['cfo']:.1f} peaks={row['peaks']} "
                    f"dpred={dpred(row)} nbytes={row['nbytes']}")
        print("=== First demod results ===")
        if not tap.rows:
            print("  (none)")
        for row in tap.rows[:12]:
            print(
                f"  id={row['id']} mode={row['mode']} status={row['status']} "
                f"fcs={int(row['fcs'])} det={row['det']} "
                f"metric={row['metric']:.3f} sfd={row['sfd']:.3f} "
                f"cfo={row['cfo']:.1f} nbytes={row['nbytes']}")
        if len(tap.rows) > 12:
            print(f"  ... {len(tap.rows) - 12} more")

    post_rc = 0
    dump_ok = True
    if writer is not None:
        dump_ok = writer.packets_dropped() == 0 and writer.packets_written() > 0
        if not dump_ok:
            print("FAIL: writer dropped packets or wrote none")
        if write_dir and tap is not None:
            demod_jsonl = os.path.join(write_dir, "demod_results.jsonl")
            write_demod_results(demod_jsonl, pkt_tap.rows, tap.rows)
            print(f"wrote {demod_jsonl} ({len(tap.rows)} rows)")
        if dump_ok and write_dir and not a.skip_postprocess:
            post_rc = run_offline_postprocess(write_dir, a.postprocess_bin)
        elif dump_ok and a.skip_postprocess:
            print("=== Offline postprocess skipped (--skip-postprocess) ===")
    if not dump_ok:
        return 1
    if post_rc != 0:
        print(f"WARN: dump OK, offline postprocess failed (exit {post_rc})")
        return 4
    return 0 if ext.identity_confirmed() else 3


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
            return run_x410_live(a, tmpl)
        except Exception as exc:
            print(f"x410 live demod failed: {exc}")
            return 2

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
