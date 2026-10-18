#!/usr/bin/env python3
"""S2-B: sensing-preamble interference on a communication payload.

Builds ``y = s + i + w`` windows on the 998.4 MS/s work grid where

* ``s`` is a full code-10 communication packet (64 SYNC + SFD(4z2) + PHR +
  random PSDU with a single FCS), decoded by the code-10 demod profile, and
* ``i`` is a code-9 SENSING preamble-only finite SYNC train placed inside the
  comm payload data region (its first 8 SYNC overlap unknown random payload).

Three paired paths per window: ``clean = s + w``, ``raw = y`` and ``sic`` =
``y`` through the frozen S1 PDU SIC adapter (code-9 templates).  The comm
SYNC/SFD/PHR stay interference-free; the main S2-B question is whether SIC
recovers the payload decode.

Roles, geometry, signal model, truth firewall, metrics and the staged matrix
are specified in ``docs/sic/S2B_设计与测试契约.md``.  This app is a thin driver
over the reusable ``sic_preamble_sim`` helpers and the frozen C++ blocks; it
never re-implements the SIC math or the demodulator.

Examples (from the repo root, forcing the worktree build):

  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py --help

  # functional smoke (clean closure + first 8 windows, no formal claims)
  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py \
      --build-dir gr-uwb/build-sic-payload --phase b0 --windows 8 \
      --output gr-uwb/build-sic-payload/results/s2b_b0_smoke

  # targeted QA (no flowgraph for the pure checks)
  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py --self-test

  # B1 acceptance cell (100 paired packets)
  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py \
      --build-dir gr-uwb/build-sic-payload --phase b1 --windows 100 \
      --seed-set acceptance --output gr-uwb/build-sic-payload/results/s2b_b1

Formal performance numbers must be collected serially under
``flock -n /tmp/uwb-sic-benchmark.lock`` when the other session is active.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np
import pmt
from gnuradio import gr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "tools", "sic"))
import sic_preamble_sim as S            # noqa: E402
import sic_payload_sim as PL            # noqa: E402
import uwb_sim_preamble_sic as S1       # noqa: E402  (reuse plumbing)
import payload_metrics as PM            # noqa: E402

# Official four-path channel profile (testdata/sic_preamble/cases/main.json).
# Delays/gains are the frozen S0/S1 profile; bound by hash in every manifest.
FOUR_PATH_PROFILE = [[0, [1.0, 0.0]], [7.4, [0.0, 0.5]],
                     [23, [0.276, -0.116]], [51.2, [0.2, 0.0]]]
# Old custom integer-spaced diagnostic (kept ONLY as a named non-official mode).
CUSTOM_INTEGER_FOUR = [[0, [1.0, 0.0]], [37, [0.8, 0.0]],
                       [74, [0.64, 0.0]], [111, [0.512, 0.0]]]

SCHEMA_VERSION = 2
S2B_SCHEMA = "sic-s2b-payload/1"
TERMINAL_OK = ("success",)

# Frozen calibration set (see the design contract §7).  Thresholds are chosen
# on calibration; acceptance never re-tunes them.
CALIBRATION_BASE = 21000000
ACCEPTANCE_BASE = 22000000
SUBSTREAM_STRIDE = 100000

# Detection mainlobe of the code-9 sync is ~1 sample wide at full rate, so the
# coarse stride must be small (measured on the calibration set; see the report).
DEFAULT_COARSE_STRIDE = 2
DEFAULT_DETECT_THRESHOLD = 0.40
DEFAULT_VALIDATE_THRESHOLD = 0.25

# Online SIC input meta whitelist (no injection truth).  Declared per window so
# the verifier can check the online interface stayed truth-free.
SIC_INPUT_META_KEYS = ("packet_id", "window_id", "predicted_start_sample",
                       "window_start_sample", "sample_count", "sample_rate",
                       "code_index", "sync_repetitions", "sfd_mode",
                       "observed_end_sample", "sensing_roi_lo",
                       "sensing_roi_hi")


# ---------------------------------------------------------------------------
# PMT / sink plumbing
# ---------------------------------------------------------------------------
class KeyedSink(gr.basic_block):
    """Accumulates one decoded result dict per ``packet_id`` (thread-safe)."""

    def __init__(self, name: str):
        gr.basic_block.__init__(self, name=name, in_sig=None, out_sig=None)
        self.message_port_register_in(pmt.intern("in"))
        self.set_msg_handler(pmt.intern("in"), self._handle)
        self.lock = __import__("threading").Lock()
        self.by_id = {}
        self.count = 0

    def _handle(self, msg):
        try:
            if pmt.is_pair(msg):
                meta = pmt.car(msg)
                payload = pmt.cdr(msg)
            else:
                meta = msg
                payload = pmt.PMT_NIL
            d = pmt.to_python(meta)
            if not isinstance(d, dict):
                return
            if pmt.is_u8vector(payload):
                d["__u8"] = bytes(pmt.u8vector_elements(payload))
            elif pmt.is_c32vector(payload):
                d["__c32"] = np.array(pmt.c32vector_elements(payload),
                                      dtype=np.complex64)
            key = int(d.get("packet_id", d.get("window_id", self.count)))
            with self.lock:
                self.by_id[key] = d
                self.count += 1
        except Exception as exc:  # pragma: no cover
            sys.stderr.write("KeyedSink error: %r\n" % (exc,))

    def get(self, key):
        with self.lock:
            return self.by_id.get(key)


class PayloadRuntime:
    """Owns the bounded top_block: demodulator (+ SIC adapter when enabled)."""

    def __init__(self, uwb, *, tpl_path, demod_code, sync_reps, sfd_mode,
                 demod_stride, sic_cfg=None, keep_iq=0, qcap=64):
        self.uwb = uwb
        self.tb = gr.top_block("uwb_sim_sic_payload")
        self.demod = uwb.realtime_demodulator(
            tpl_path, 1, int(qcap), sfd_mode, 0, "bypass", int(demod_code),
            int(sync_reps), int(demod_stride))
        self.sink_demod = KeyedSink("demod_sink")
        self.tb.msg_connect((self.demod, "result"), (self.sink_demod, "in"))
        self.sic = None
        self.sink_out = None
        self.sink_ihat = None
        self.sink_status = None
        self.sic_params_used = {}
        if sic_cfg is not None:
            vals = {
                "sync": sic_cfg["sync"].astype(np.complex64).tolist(),
                "preamble": sic_cfg["preamble"].astype(np.complex64).tolist(),
                "comm_code": int(sic_cfg["comm_code"]),
                "sync_reps": int(sic_cfg["sync_reps"]),
                "samples_per_symbol": int(S.SPS),
                "sample_rate": float(S.WORK_HZ),
                "k_paths": int(sic_cfg["k_paths"]),
                "fit_reps": int(sic_cfg["fit_reps"]),
                "validation_reps": int(sic_cfg["validation_reps"]),
                "roi_lo": int(sic_cfg["roi_lo"]),
                "roi_hi": int(sic_cfg["roi_hi"]),
                "coarse_stride": int(sic_cfg["coarse_stride"]),
                "mode": str(sic_cfg["mode"]),
                "detect_threshold": float(sic_cfg["detect_threshold"]),
                "validate_threshold": float(sic_cfg["validate_threshold"]),
            }
            extras = {
                "in_flight_max": int(sic_cfg.get("in_flight_max", 8)),
                "iq_budget_bytes": int(sic_cfg.get("iq_budget_bytes", 64 << 20)),
                "queue_capacity": int(sic_cfg.get("queue_capacity", 8)),
                "publish_reconstruction": bool(
                    sic_cfg.get("publish_reconstruction", True)),
                "post_tail": int(sic_cfg.get("post_tail", 100)),
                "enable_fractional": bool(sic_cfg.get("enable_fractional", True)),
                "enable_cfo": bool(sic_cfg.get("enable_cfo", True)),
                "path_min_spacing": int(sic_cfg.get("path_min_spacing", 0)),
                "max_window_samples": int(sic_cfg.get("max_window_samples", 0)),
                "support_mode": str(sic_cfg.get("support_mode",
                                                "configured_nominal")),
                "online_validation_reps": int(
                    sic_cfg.get("online_validation_reps", 0)),
                "fit_residual_max": float(sic_cfg.get("fit_residual_max", 0.8)),
            }
            self.sic, self.sic_params_used = \
                S1.call_sic_make_from_templates(uwb, vals, extras)
            self.sink_out = KeyedSink("sic_out_sink")
            self.sink_ihat = KeyedSink("sic_ihat_sink")
            self.sink_status = KeyedSink("sic_status_sink")
            # cleaned IQ goes C++ -> C++ straight into the demodulator; a
            # parallel sink records the cleaned IQ + frozen-model meta.
            self.tb.msg_connect((self.sic, "out"), (self.demod, "samples"))
            self.tb.msg_connect((self.sic, "out"), (self.sink_out, "in"))
            self.tb.msg_connect((self.sic, "i_hat"), (self.sink_ihat, "in"))
            self.tb.msg_connect((self.sic, "status"), (self.sink_status, "in"))

    def start(self):
        self.tb.start()

    def stop(self):
        self.tb.stop()
        self.tb.wait()

    def post_demod(self, meta, iq):
        self.demod.to_basic_block()._post(
            pmt.intern("samples"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def post_sic(self, meta, iq):
        self.sic.to_basic_block()._post(
            pmt.intern("in"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def wait_keys(self, sink, keys, deadline_s=180.0):
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            with sink.lock:
                if all(k in sink.by_id for k in keys):
                    return True
            time.sleep(0.002)
        return False


# ---------------------------------------------------------------------------
# Per-window execution
# ---------------------------------------------------------------------------
def _demod_meta(packet_id, comm_start, iq_len):
    meta = pmt.make_dict()
    meta = pmt.dict_add(meta, pmt.intern("packet_id"), pmt.from_long(packet_id))
    meta = pmt.dict_add(meta, pmt.intern("window_id"), pmt.from_long(packet_id))
    meta = pmt.dict_add(meta, pmt.intern("predicted_start_sample"),
                        pmt.from_long(int(comm_start)))
    meta = pmt.dict_add(meta, pmt.intern("window_start_sample"),
                        pmt.from_long(0))
    meta = pmt.dict_add(meta, pmt.intern("sample_count"), pmt.from_long(iq_len))
    meta = pmt.dict_add(meta, pmt.intern("sample_rate"), pmt.from_double(S.WORK_HZ))
    return meta


def _sic_meta(packet_id, gen, rois, iq_len, sensing_code, sync_reps, sfd_mode):
    meta = pmt.make_dict()
    meta = pmt.dict_add(meta, pmt.intern("packet_id"), pmt.from_long(packet_id))
    meta = pmt.dict_add(meta, pmt.intern("window_id"), pmt.from_long(packet_id))
    meta = pmt.dict_add(meta, pmt.intern("predicted_start_sample"),
                        pmt.from_long(int(gen.comm_start)))
    meta = pmt.dict_add(meta, pmt.intern("window_start_sample"), pmt.from_long(0))
    meta = pmt.dict_add(meta, pmt.intern("sample_count"), pmt.from_long(iq_len))
    meta = pmt.dict_add(meta, pmt.intern("sample_rate"), pmt.from_double(S.WORK_HZ))
    meta = pmt.dict_add(meta, pmt.intern("code_index"),
                        pmt.from_long(int(sensing_code)))
    meta = pmt.dict_add(meta, pmt.intern("sync_repetitions"),
                        pmt.from_long(int(sync_reps)))
    meta = pmt.dict_add(meta, pmt.intern("sfd_mode"), pmt.intern(str(sfd_mode)))
    meta = pmt.dict_add(meta, pmt.intern("observed_end_sample"),
                        pmt.from_long(int(rois["observed_end"])))
    meta = pmt.dict_add(meta, pmt.intern("sensing_roi_lo"),
                        pmt.from_long(int(rois["sub_roi_lo"])))
    meta = pmt.dict_add(meta, pmt.intern("sensing_roi_hi"),
                        pmt.from_long(int(rois["sub_roi_hi"])))
    return meta


def _status_ok(d):
    return str(d.get("status", "")) in TERMINAL_OK


def run_window(rt, gen, *, wid, seed, payload_seed, start_offset,
               cfo_hz, isr_db, snr_db, interferer_present, wrong_code,
               no_overlap, template_override, i_paths, archive, rois):
    """Generate + run one paired window; return the per-window record."""
    t_gen0 = time.monotonic_ns()
    win = gen.build(seed=seed, payload_seed=payload_seed, cfo_hz=cfo_hz,
                    isr_db=isr_db, snr_db=snr_db, start_offset=start_offset,
                    interferer_present=interferer_present,
                    interferer_template=template_override, i_paths=i_paths)
    truth = win["truth"]
    y = win["y"]
    t_gen_us = (time.monotonic_ns() - t_gen0) // 1000

    pid_clean, pid_raw, pid_sic = 3 * wid, 3 * wid + 1, 3 * wid + 2
    t_post = time.monotonic_ns()
    rt.post_demod(_demod_meta(pid_clean, gen.comm_start, y.size), win["clean"])
    rt.post_demod(_demod_meta(pid_raw, gen.comm_start, y.size), y)
    if rt.sic is not None:
        rt.post_sic(_sic_meta(pid_sic, gen, rois, y.size, gen.sensing_code,
                              gen.sync_reps, gen.sfd_mode), y)
    t_post_us = (time.monotonic_ns() - t_post) // 1000

    keys = [pid_clean, pid_raw]
    sink = rt.sink_demod
    if rt.sic is not None:
        keys.append(pid_sic)
    ok = rt.wait_keys(sink, keys)
    sic_out = None
    if rt.sic is not None:
        rt.wait_keys(rt.sink_out, [pid_sic], deadline_s=30.0)
        sic_out = rt.sink_out.get(pid_sic)
    rec = {"window_id": int(wid), "packet_ids": [pid_clean, pid_raw, pid_sic],
           "seed": int(seed), "payload_seed": int(payload_seed),
           "jitter": int(start_offset), "cfo_hz": float(cfo_hz),
           "snr_db_target": snr_db, "isr_db_target": isr_db,
           "isr_db_realized": win["isr_db"],
           "position": gen.position, "snr_db_ref": snr_db,
           "interferer_present": bool(interferer_present),
           "wrong_code": int(wrong_code), "no_overlap": bool(no_overlap),
           "psdu_wire_hex": truth["psdu_wire_hex"],
           "psdu_wire_sha256": hashlib.sha256(
               bytes.fromhex(truth["psdu_wire_hex"])).hexdigest(),
           "input_sha256": hashlib.sha256(
               np.ascontiguousarray(y, dtype=np.complex64).tobytes()).hexdigest(),
           "psdu_wire_len": truth["psdu_wire_len"],
           "overlap": truth["overlap"],
           "i_start": truth["interferer"]["start_sample"],
           "i_support": truth["interferer"]["support"],
           "i_template_len": truth["interferer"]["template_len"],
           "i_paths": [p["delay_samples"]
                       for p in truth["interferer"]["paths"]],
           "online_meta_keys": list(SIC_INPUT_META_KEYS),
           "training_span": list(gen.training_span(int(start_offset))),
           "rois": {k: rois[k] for k in rois},
           "demod": {}, "sic": {}, "timing": {"gen_us": int(t_gen_us),
                                              "post_us": int(t_post_us)}}
    for name, pid in (("clean", pid_clean), ("raw", pid_raw),
                      ("sic", pid_sic)):
        d = sink.get(pid)
        if d is None:
            rec["demod"][name] = {"status": "timeout"}
            continue
        got = d.get("__u8") or b""
        rec["demod"][name] = {
            "status": str(d.get("status", "")),
            "status_code": d.get("status_code"),
            "fcs_pass": bool(d.get("fcs_pass", False)),
            "phr_psdu_length": d.get("phr_psdu_length"),
            "payload_nbytes": d.get("payload_nbytes"),
            "psdu_hex": got.hex().upper(),
            "psdu_match": bool(got.hex().upper() == truth["psdu_wire_hex"]),
            "cfo_hz": d.get("cfo_hz"),
            "stage_total_us": d.get("stage_total_us"),
            "wall_latency_us": d.get("wall_latency_us"),
        }
    # SIC frozen-model publication + actual output suppression.
    if sic_out is not None:
        o = sic_out
        so = {"state": o.get("sic_state"), "detected": o.get("sic_detected"),
              "committed": o.get("sic_committed"),
              "bypass_reason": o.get("sic_bypass_reason"),
              "reject_reason": o.get("sic_reject_reason"),
              "detected_start": o.get("sic_detected_start"),
              "cfo_hz": o.get("sic_cfo_hz"), "n_paths": o.get("sic_n_paths"),
              "support_lo": o.get("sic_support_lo"),
              "support_hi": o.get("sic_support_hi"),
              "sub_lo": o.get("sic_sub_lo"), "sub_hi": o.get("sic_sub_hi"),
              "support_source": o.get("sic_support_source"),
              "training_lo": o.get("sic_training_lo"),
              "training_hi": o.get("sic_training_hi"),
              "visible_end_sample": o.get("sic_visible_end_sample"),
              "observed_end_sample": o.get("sic_observed_end_sample"),
              "model_mode": o.get("sic_model_mode"),
              "model_fingerprint": o.get("sic_model_fingerprint"),
              "fit_residual": o.get("sic_fit_residual"),
              "cond2": o.get("sic_cond2"),
              "apply_refit_count": o.get("sic_apply_refit_count"),
              "core_total_us": o.get("core_total_us"),
              "completion_latency_us": o.get("completion_latency_us"),
              "sub_overlap": o.get("sic_sub_overlap"),
              "uses_truth": o.get("sic_uses_truth")}
        rec["sic"] = so
        yhat = o.get("__c32")
        if yhat is not None and yhat.size == y.size:
            ov_lo = int(truth["overlap"]["lo"])
            ov_hi = int(truth["overlap"]["hi"])
            res = yhat.astype(np.complex128) - win["clean"].astype(np.complex128)
            itrue = win["i"].astype(np.complex128)
            if ov_hi > ov_lo:
                e_i = float(np.sum(np.abs(itrue[ov_lo:ov_hi]) ** 2))
                e_r = float(np.sum(np.abs(res[ov_lo:ov_hi]) ** 2))
                rec["suppression_db_overlap"] = (
                    10.0 * math.log10(e_i / e_r) if e_r > 0 and e_i > 0 else None)
            # mask exactness: out == y outside [sub_lo, sub_hi)
            mask_lo = int(so.get("sub_lo") or 0)
            mask_hi = int(so.get("sub_hi") or 0)
            raw = y.astype(np.complex128)
            oh = yhat.astype(np.complex128)
            outside = np.ones(y.size, dtype=bool)
            if mask_hi > mask_lo:
                outside[mask_lo:mask_hi] = False
            rec["mask_outside_changed"] = int(np.count_nonzero(
                np.abs(oh[outside] - raw[outside]) > 0.0))
    if archive:
        rec["_archive"] = {"i_true": win["i"], "clean": win["clean"],
                           "raw": y, "out": None}
        if sic_out is not None:
            rec["_archive"]["out"] = sic_out.get("__c32")
    return rec


def channel_paths(spec, seed):
    """Deterministic channel model.

    ``single`` = one unit-gain path.  ``four`` = the OFFICIAL frozen profile from
    ``testdata/sic_preamble/cases/main.json`` (bound by hash in the manifest).
    ``custom_integer_four`` = the old integer-spaced diagnostic, explicitly NOT
    the official four-path channel.
    """
    if spec == "single":
        return [[0, [1.0, 0.0]]]
    if spec == "four":
        return [list(p) for p in FOUR_PATH_PROFILE]
    if spec == "custom_integer_four":
        return [list(p) for p in CUSTOM_INTEGER_FOUR]
    raise SystemExit("unknown channel spec %r" % spec)


def channel_profile_id(spec):
    return {"single": "single_int", "four": "four_path_main_json",
            "custom_integer_four": "custom_integer_four_diagnostic"}.get(
        spec, "unknown")


# ---------------------------------------------------------------------------
# Single-run driver
# ---------------------------------------------------------------------------
def seed_streams(seed_set, windows, step=1):
    base = CALIBRATION_BASE if seed_set == "calibration" else ACCEPTANCE_BASE
    out = []
    for i in range(windows):
        out.append({
            "noise": base + i * step,
            "payload": base + SUBSTREAM_STRIDE + i * step,
            "jitter": base + 2 * SUBSTREAM_STRIDE + i * step,
            "path": base + 3 * SUBSTREAM_STRIDE + i * step,
        })
    return out


def effective_case_config(args):
    """The deterministic signal/SIC/demod configuration of one run (hashed)."""
    return {
        "phase": args.phase, "snr_db": args.snr_db, "isr_db": args.isr_db,
        "position": args.position, "cfo_hz": args.cfo_hz,
        "comm_channel": args.comm_channel,
        "comm_channel_profile": channel_profile_id(args.comm_channel),
        "interferer_paths": args.interferer_paths,
        "interferer_paths_profile": channel_profile_id(args.interferer_paths),
        "interferer": args.interferer, "wrong_code": args.wrong_code,
        "sic": args.sic, "comm_code": args.comm_code,
        "sensing_code": args.sensing_code, "sync_reps": args.sync_reps,
        "sfd_mode": args.sfd_mode, "psdu_bytes": args.psdu_bytes,
        "pulse_shape": args.pulse_shape, "seed_set": args.seed_set,
        "seed_step": args.seed_step,
        "sic_k_paths": args.sic_k_paths, "sic_fit_reps": args.sic_fit_reps,
        "sic_validation_reps": args.sic_validation_reps,
        "sic_post_tail": args.sic_post_tail,
        "sic_coarse_stride": args.sic_coarse_stride,
        "sic_detect_threshold": args.sic_detect_threshold,
        "sic_validate_threshold": args.sic_validate_threshold,
        "start_jitter": args.start_jitter, "guard": args.guard,
        "demod_stride": args.demod_stride,
        "four_path_profile": FOUR_PATH_PROFILE if args.interferer_paths in
        ("four",) or args.comm_channel == "four" else None,
    }


def window_params(args, gen, st):
    """Deterministic per-window injection params (single source of truth)."""
    jrng = np.random.default_rng(st["jitter"])
    jitter = (int(jrng.integers(-args.start_jitter, args.start_jitter + 1))
              if args.start_jitter > 0 else 0)
    no_overlap = (args.position == "no_overlap")
    if no_overlap:
        start_offset = (gen.comm_start + gen.fields["packet_len"] + 500
                        - gen.i_nominal_start)
    else:
        start_offset = jitter
    i_paths = channel_paths(args.interferer_paths, st["path"])
    return start_offset, i_paths, no_overlap


def run_single(args, repo):
    uwb = S.load_uwb(args.build_dir)
    os.makedirs(args.output, exist_ok=True)
    tpl_dir = os.path.join(args.output, "templates")
    os.makedirs(tpl_dir, exist_ok=True)
    recon_dir = os.path.join(args.output, "recon")
    os.makedirs(recon_dir, exist_ok=True)

    comm_paths = channel_paths(args.comm_channel, 0)
    pos_for_gen = "center" if args.position == "no_overlap" else args.position
    gen = PL.SicPayloadSignalGenerator(
        uwb, comm_code=args.comm_code, sensing_code=args.sensing_code,
        sync_reps=args.sync_reps, sfd_mode=args.sfd_mode,
        psdu_bytes=args.psdu_bytes, pulse_shape=args.pulse_shape,
        peak_amplitude=args.peak_amplitude,
        position=pos_for_gen, guard=args.guard,
        snr_db=args.snr_db, isr_db=args.isr_db,
        interferer_present=(args.interferer != "off"),
        comm_paths=comm_paths, verbose=True)
    gen.position = args.position  # label (ROI/placement computed at center)

    # Templates: code-10 SYNC for the demod; code-9 sync+preamble for the SIC.
    comm_sync_path = os.path.join(tpl_dir, "comm_sync_code%d_998p4.cf32"
                                  % args.comm_code)
    gen.comm_sync.tofile(comm_sync_path)
    i_sync_path = os.path.join(tpl_dir, "sensing_sync_code%d_998p4.cf32"
                               % args.sensing_code)
    gen.i_sync.tofile(i_sync_path)
    i_pre_path = os.path.join(tpl_dir, "sensing_preamble_code%d_998p4.cf32"
                              % args.sensing_code)
    gen.i_wave.tofile(i_pre_path)

    wrong_template = None
    if args.interferer == "wrong_code":
        wt = PL.build_finite_sync_template(
            uwb, code_index=int(args.wrong_code), sync_reps=args.sync_reps,
            sfd_mode=args.sfd_mode, pulse_shape=args.pulse_shape,
            peak_amplitude=args.peak_amplitude)
        wrong_template = wt["waveform"]
        wt["waveform"].tofile(os.path.join(
            tpl_dir, "wrong_code%d_preamble_998p4.cf32" % args.wrong_code))

    rois = gen.public_rois(start_jitter=args.start_jitter,
                           post_tail=args.sic_post_tail,
                           fit_reps=args.sic_fit_reps)

    sic_cfg = None
    if args.sic != "off":
        sic_cfg = {
            "sync": gen.i_sync, "preamble": gen.i_wave,
            "comm_code": int(args.sensing_code), "sync_reps": int(args.sync_reps),
            "k_paths": int(args.sic_k_paths), "fit_reps": int(args.sic_fit_reps),
            "validation_reps": int(args.sic_validation_reps),
            "roi_lo": int(rois["detect_roi_lo"]),
            "roi_hi": int(rois["detect_roi_hi"]),
            "coarse_stride": int(args.sic_coarse_stride), "mode": "estimate",
            "detect_threshold": float(args.sic_detect_threshold),
            "validate_threshold": float(args.sic_validate_threshold),
            "post_tail": int(args.sic_post_tail),
            "support_mode": "configured_nominal", "online_validation_reps": 0,
            "publish_reconstruction": True,
        }
    rt = PayloadRuntime(uwb, tpl_path=comm_sync_path, demod_code=args.comm_code,
                        sync_reps=args.sync_reps, sfd_mode=args.sfd_mode,
                        demod_stride=args.demod_stride, sic_cfg=sic_cfg,
                        keep_iq=0)
    rt.start()

    streams = seed_streams(args.seed_set, int(args.windows), int(args.seed_step))
    planned = len(streams)
    plan_dir = args.output
    case_cfg = effective_case_config(args)
    case_cfg_hash = S1.stdin_hash(case_cfg)
    plan = {
        "schema": S2B_SCHEMA, "phase": args.phase,
        "planned_windows": planned,
        "window_ids": list(range(planned)),
        "seeds": streams,
        "case_config": case_cfg, "case_config_hash": case_cfg_hash,
        "rois": {k: rois[k] for k in rois},
        "channel_four_path_profile": [list(p) for p in FOUR_PATH_PROFILE],
        "templates": {"comm_sync_sha256": S1.file_sha256(comm_sync_path),
                      "sensing_sync_sha256": S1.file_sha256(i_sync_path),
                      "sensing_preamble_sha256": S1.file_sha256(i_pre_path)},
        "generator": {"comm_code": gen.comm_code,
                      "sensing_code": gen.sensing_code,
                      "psdu_bytes": gen.psdu_bytes, "sync_reps": gen.sync_reps,
                      "geometry": gen.fields, "n": gen.n, "pre_guard": gen.pre,
                      "payload_abs": [gen.payload_lo, gen.payload_hi],
                      "interferer_template_len": gen.i_template_len,
                      "interferer_nominal_start": gen.i_nominal_start},
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    S1.write_json(os.path.join(plan_dir, "plan.json"), plan)

    archive_idx = set(range(min(int(args.archive_windows), planned)))
    records = []
    arch_fixed = {}
    windows_path = os.path.join(args.output, "windows.jsonl")
    t_run0 = time.monotonic_ns()
    try:
        with open(windows_path, "w") as fh:
            for i, st in enumerate(streams):
                start_offset, i_paths, no_overlap = window_params(args, gen, st)
                arch = (i in archive_idx)
                rec = run_window(
                    rt, gen, wid=i, seed=st["noise"], payload_seed=st["payload"],
                    start_offset=start_offset, cfo_hz=args.cfo_hz,
                    isr_db=args.isr_db, snr_db=args.snr_db,
                    interferer_present=(args.interferer != "off"),
                    wrong_code=(int(args.wrong_code)
                                if args.interferer == "wrong_code" else 0),
                    no_overlap=no_overlap, template_override=wrong_template,
                    i_paths=i_paths, archive=arch, rois=rois)
                arc = rec.pop("_archive", None)
                if arc is not None:
                    arch_fixed[i] = arc
                records.append(rec)
                fh.write(json.dumps(rec, sort_keys=True) + "\n")
                fh.flush()
                if args.progress and (i + 1) % max(1, int(args.progress)) == 0:
                    print("  window %d/%d" % (i + 1, planned), flush=True)
        # B-R04: really archive the rule-selected "worst" windows (not just a
        # marker).  Rule (fixed BEFORE looking at the numbers, cap = 3):
        #   (1) lowest suppression_db_overlap among committed windows,
        #   (2) first window whose SIC PSDU failed,
        #   (3) first raw_ok_sic_fail damage window.
        # The pooled statistics always use ALL rows; these are extra evidence.
        worst_ids = select_worst_windows(records, args.sic != "off", cap=3)
        arch_worst = {}
        for wid in worst_ids:
            if wid in arch_fixed:
                continue
            arc = rerun_archive_window(rt, gen, args, rois, wrong_template,
                                       wid, streams, records[wid])
            if arc is not None:
                arch_worst[wid] = arc
    finally:
        rt.stop()

    run_wall_us = (time.monotonic_ns() - t_run0) // 1000
    archive_files = []
    for wid, arc in list(arch_fixed.items()) + list(arch_worst.items()):
        np.savez_compressed(
            os.path.join(recon_dir, "window_%04d.npz" % wid),
            i_true=np.asarray(arc["i_true"], dtype=np.complex64),
            clean=np.asarray(arc["clean"], dtype=np.complex64),
            raw=np.asarray(arc["raw"], dtype=np.complex64),
            out=(np.asarray(arc["out"], dtype=np.complex64)
                 if arc["out"] is not None
                 else np.zeros(0, dtype=np.complex64)))
        archive_files.append("recon/window_%04d.npz" % wid)
    index = {
        "schema": S2B_SCHEMA, "phase": args.phase, "windows": len(records),
        "planned_windows": planned,
        "seed_set": args.seed_set, "snr_db": args.snr_db, "isr_db": args.isr_db,
        "position": args.position, "cfo_hz": args.cfo_hz,
        "interferer": args.interferer, "sic": args.sic,
        "case_config_hash": case_cfg_hash,
        "archive_fixed_windows": sorted(arch_fixed),
        "archive_worst_windows": sorted(arch_worst),
        "worst_rule": ("lowest suppression_db_overlap among committed; first SIC "
                       "PSDU failure; first raw_ok_sic_fail damage; cap 3"),
        "archived_windows": sorted(list(arch_fixed) + list(arch_worst)),
        "archive_policy": {
            "archived": len(archive_files), "total": planned,
            "fixed": len(arch_fixed), "worst": len(arch_worst),
            "rule": ("first %d planned windows (fixed before the run) plus up to "
                     "3 rule-selected worst windows; the archived subset cannot "
                     "reproduce the pooled statistic and is not the distribution"
                     % min(int(args.archive_windows), planned)),
        },
    }
    with open(os.path.join(args.output, "index.json"), "w") as fh:
        json.dump(index, fh, indent=2, sort_keys=True)

    summary = summarize_run(records, args, gen, rois, run_wall_us,
                            planned=planned, case_cfg_hash=case_cfg_hash)
    S1.write_json(os.path.join(args.output, "summary.json"), summary)
    write_stage_csv(os.path.join(args.output, "stage_times.csv"), records)
    manifest = build_manifest(args, repo, gen, rois, comm_sync_path,
                              i_sync_path, i_pre_path, rt, records,
                              planned=planned, case_cfg_hash=case_cfg_hash)
    S1.write_json(os.path.join(args.output, "run_manifest.json"), manifest)
    try:
        S1.update_results_index(args.output)
    except Exception as exc:  # pragma: no cover
        sys.stderr.write("index update failed: %r\n" % (exc,))
    print("summary:", json.dumps(summary["per"], sort_keys=True), flush=True)
    print("output:", args.output, flush=True)
    return run_exit_code(records, planned, args)


def select_worst_windows(records, sic_on, cap=3):
    """Rule-selected worst windows (fixed rule; used ONLY for extra archiving)."""
    out = []
    if sic_on:
        committed = [r for r in records
                     if r.get("sic", {}).get("committed")
                     and r.get("suppression_db_overlap") is not None]
        if committed:
            out.append(min(committed,
                           key=lambda r: r["suppression_db_overlap"])["window_id"])
        for r in records:
            if not (r.get("sic", {}).get("committed")):
                continue
            if r["demod"].get("sic", {}).get("psdu_match") is not True:
                out.append(r["window_id"])
                break
        for r in records:
            raw_ok = r["demod"]["raw"].get("psdu_match") is True
            sic_ok = r["demod"].get("sic", {}).get("psdu_match") is True
            if raw_ok and not sic_ok:
                out.append(r["window_id"])
                break
    dedup = []
    for w in out:
        if w not in dedup:
            dedup.append(w)
    return dedup[:cap]


def rerun_archive_window(rt, gen, args, rois, wrong_template, wid, streams, rec):
    """Regenerate one window, verify its input hash, run SIC, return arrays.

    Refuses to archive if the regenerated input does not match the recorded
    per-window input hash (no fake archiving).
    """
    st = streams[wid]
    start_offset, i_paths, no_overlap = window_params(args, gen, st)
    win = gen.build(seed=st["noise"], payload_seed=st["payload"],
                    cfo_hz=args.cfo_hz, isr_db=args.isr_db, snr_db=args.snr_db,
                    start_offset=start_offset,
                    interferer_present=(args.interferer != "off"),
                    interferer_template=wrong_template, i_paths=i_paths)
    y = win["y"]
    got = hashlib.sha256(
        np.ascontiguousarray(y, dtype=np.complex64).tobytes()).hexdigest()
    if got != rec.get("input_sha256"):
        sys.stderr.write("refusing to archive window %d: input hash mismatch\n"
                         % wid)
        return None
    pid = 3 * wid + 2
    before = rt.sink_out.count
    rt.post_sic(_sic_meta(pid, gen, rois, y.size, gen.sensing_code,
                          gen.sync_reps, gen.sfd_mode), y)
    rt.wait_keys(rt.sink_out, [pid], deadline_s=60.0)
    o = rt.sink_out.get(pid)
    out = o.get("__c32") if o is not None else None
    return {"i_true": win["i"], "clean": win["clean"], "raw": y, "out": out}


def run_exit_code(records, planned, args):
    """Nonzero on any required-run failure (B-R04)."""
    if len(records) != planned:
        return 3
    for r in records:
        for path in (["clean", "raw"]
                     + (["sic"] if args.sic != "off" else [])):
            d = r["demod"].get(path)
            if d is None or str(d.get("status")) == "timeout":
                return 4
    return 0


# ---------------------------------------------------------------------------
# Metrics / summary
# ---------------------------------------------------------------------------
def _wilson(k, n, z=1.96):
    return S1.wilson_ci(k, n, z)


def per_metric(rows, path):
    """(errors, total, per, per_ci_lo, per_ci_hi, success_lo, success_hi)."""
    err = 0
    ok = 0
    n = 0
    for r in rows:
        d = r["demod"].get(path) or {}
        n += 1
        if d.get("psdu_match") is True:
            ok += 1
        else:
            err += 1
    per = (err / n) if n else None
    plo, phi = PM.per_ci(err, n)
    slo, shi = PM.wilson_ci(ok, n)
    return err, n, per, plo, phi, slo, shi


def summarize_run(records, args, gen, rois, run_wall_us, planned=None,
                  case_cfg_hash=None):
    per = {}
    paths = ["clean", "raw"] + (["sic"] if args.sic != "off" else [])
    for path in paths:
        if not records:
            continue
        err, n, p, plo, phi, slo, shi = per_metric(records, path)
        fcs = PM.per_fcs(records, path)
        per[path] = {"errors": err, "success": n - err, "total": n, "per": p,
                     "per_wilson95": [plo, phi],
                     "success_wilson95": [slo, shi],
                     "per_fcs_errors": fcs["errors"], "per_fcs": fcs["per"],
                     "per_fcs_wilson95": fcs["per_wilson95"],
                     "fcs_pass": fcs.get("fcs_pass"),
                     "status_counts": _count_status(records, path)}
    rescued = damaged = None
    if args.sic != "off":
        rescued = sum(1 for r in records
                      if not (r["demod"]["raw"].get("psdu_match") is True)
                      and r["demod"].get("sic", {}).get("psdu_match") is True)
        damaged = sum(1 for r in records
                      if r["demod"]["raw"].get("psdu_match") is True
                      and not (r["demod"].get("sic", {}).get("psdu_match")
                               is True))
    supp = [r["suppression_db_overlap"] for r in records
            if r.get("suppression_db_overlap") is not None]
    mask_bad = sum(1 for r in records if r.get("mask_outside_changed", 0) > 0)
    commits = sum(1 for r in records if r.get("sic", {}).get("committed"))
    detects = sum(1 for r in records if r.get("sic", {}).get("detected"))
    return {
        "schema": S2B_SCHEMA, "phase": args.phase,
        "run_id": os.path.basename(os.path.normpath(args.output)),
        "windows_expected": int(planned) if planned is not None
        else len(records),
        "windows_ok": len(records),
        "windows_missing": ((int(planned) - len(records))
                            if planned is not None else 0),
        "case_config_hash": case_cfg_hash,
        "config": effective_case_config(args),
        "per": per,
        "paired": {"raw_fail_sic_ok": rescued, "raw_ok_sic_fail": damaged},
        "sic": {"detected": detects, "committed": commits,
                "median_suppression_db": (
                    float(np.median(supp)) if supp else None),
                "mask_outside_changed_windows": mask_bad},
        "runtime": {"run_wall_us": int(run_wall_us),
                    "windows_per_s": (len(records) / (run_wall_us / 1e6)
                                      if run_wall_us > 0 else None)},
        "gates": _gates(records, args, per),
        "not_run": _not_run(args),
    }


def _count_status(records, path):
    out = {}
    for r in records:
        s = (r["demod"].get(path) or {}).get("status", "missing")
        out[s] = out.get(s, 0) + 1
    return out


def _gates(records, args, per):
    n = len(records)
    g = {"windows": n}
    clean = per.get("clean", {})
    g["clean_perfect"] = bool(n > 0 and clean.get("errors") == 0)
    if args.sic != "off":
        g["mask_exact"] = all(r.get("mask_outside_changed", 0) == 0
                              for r in records) if n else False
        g["no_truth_leak"] = all(not r.get("sic", {}).get("uses_truth", False)
                                 for r in records) if n else False
        g["refit_zero"] = all((r.get("sic", {}).get("apply_refit_count") in
                               (0, None)) for r in records) if n else False
    return g


def _not_run(args):
    nr = []
    if args.sic == "off":
        nr.append("summary.paired.* (sic_mode=off)")
    nr.append("MATLAB cross-check (matlab.status=未实跑)")
    nr.append("BER (decoder exposes no aligned bits): N/A / 未实现")
    nr.append("formal benchmark/throughput (serial, separate run)")
    return nr


def write_stage_csv(path, records):
    with open(path, "w") as fh:
        fh.write("window_id,gen_us,post_us,raw_status,raw_ms,sic_status,"
                 "sic_ms,sic_state,fit_residual,suppress_db,core_total_us\n")
        for r in records:
            raw = r["demod"].get("raw", {})
            sic = r["demod"].get("sic", {})
            fh.write("%d,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s\n" % (
                r["window_id"], r["timing"]["gen_us"], r["timing"]["post_us"],
                raw.get("status"), raw.get("stage_total_us"),
                sic.get("status"), sic.get("stage_total_us"),
                r.get("sic", {}).get("state"),
                r.get("sic", {}).get("fit_residual"),
                r.get("suppression_db_overlap"),
                r.get("sic", {}).get("core_total_us")))


def build_manifest(args, repo, gen, rois, comm_sync_path, i_sync_path,
                   i_pre_path, rt, records, planned=None, case_cfg_hash=None):
    try:
        prov = S1.git_worktree_provenance(repo)
    except Exception as exc:
        prov = {"error": "%r" % (exc,)}
    try:
        lib = S1.pick_loaded_library(args.build_dir)
    except Exception:
        lib = None
    libsha = S1.file_sha256(lib) if lib and os.path.exists(lib) else None
    # B-R03 item 2: bind EVERY source that determines the signal/score, not just
    # the driver.
    here = os.path.dirname(os.path.abspath(__file__))
    src_hash = {
        "driver": S1.file_sha256(os.path.abspath(__file__)),
        "generator": S1.file_sha256(os.path.join(here, "sic_payload_sim.py")),
        "metrics": S1.file_sha256(os.path.join(
            repo, "tools", "sic", "payload_metrics.py")),
        "verifier": S1.file_sha256(os.path.join(
            repo, "tools", "sic", "verify_s2b.py")),
        "core_header": S1.file_sha256(os.path.join(
            repo, "gr-uwb", "include", "gnuradio", "uwb",
            "uwb_preamble_sic_core.h")),
        "core_src": S1.file_sha256(os.path.join(
            repo, "gr-uwb", "lib", "uwb_preamble_sic_core.cc")),
        "adapter_src": S1.file_sha256(os.path.join(
            repo, "gr-uwb", "lib", "uwb_preamble_sic.cc")),
    }
    bindings = None
    import glob as _glob
    bglob = sorted(_glob.glob(os.path.join(
        args.build_dir, "python", "uwb", "bindings", "uwb_python*.so")))
    if bglob:
        bindings = {"path": bglob[-1], "sha256": S1.file_sha256(bglob[-1])}
    return {
        "schema": S2B_SCHEMA, "run_id": os.path.basename(
            os.path.normpath(args.output)),
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "planned_windows": planned, "completed_windows": len(records),
        "case_config_hash": case_cfg_hash,
        "provenance": {
            "git": prov, "loaded_library": lib, "loaded_library_sha256": libsha,
            "build_dir": os.path.abspath(args.build_dir),
            "source_sha256": src_hash, "bindings": bindings,
            "script_sha256": src_hash["driver"],
        },
        "generator": {
            "comm_code": gen.comm_code, "sensing_code": gen.sensing_code,
            "psdu_bytes": gen.psdu_bytes, "sync_reps": gen.sync_reps,
            "sfd_mode": gen.sfd_mode, "pulse_shape": gen.pulse_shape,
            "geometry": gen.fields, "n": gen.n, "pre_guard": gen.pre,
            "payload_abs": [gen.payload_lo, gen.payload_hi],
            "interferer_template_len": gen.i_template_len,
            "interferer_nominal_start": gen.i_nominal_start,
            "placement": {"position": gen.placement.position,
                          "guard": gen.placement.guard},
        },
        "templates": {
            "comm_sync": S1.file_sha256(comm_sync_path),
            "sensing_sync": S1.file_sha256(i_sync_path),
            "sensing_preamble": S1.file_sha256(i_pre_path),
            "comm_sync_info": _tpl_info(gen.comm_sync_info()),
            "sensing_preamble_info": _tpl_info(gen.interferer_template_info()),
        },
        "rois": rois,
        "sic_config": ({k: (v if not isinstance(v, np.ndarray) else None)
                        for k, v in rt.sic_params_used.items()}
                       if rt.sic is not None else None),
        "fcs_contract": {
            "appended_once_by": "generator (sic_payload_sim.append_fcs)",
            "modulator_append_fcs": False,
            "crc": "IEEE 802.15.4 CRC-16 (poly 0x8408, init 0), little-endian",
            "wire_psdu_bytes": gen.psdu_bytes,
            "data_bytes": gen.psdu_bytes - PL.DEFAULT_FCS_BYTES,
            "fcs_bytes": PL.DEFAULT_FCS_BYTES,
            "decoded_psdu_contains_fcs": True,
        },
        "channel_profiles": {
            "comm_channel": channel_profile_id(args.comm_channel),
            "interferer_paths": channel_profile_id(args.interferer_paths),
            "official_four_path": [list(p) for p in FOUR_PATH_PROFILE],
        },
        "effective_config": effective_case_config(args),
        "archive": {"windows": len(records)},
        "not_run": _not_run(args),
    }


def _tpl_info(info):
    d = dict(info.__dict__)
    d.pop("waveform", None)
    return d


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(
        description="S2-B sensing-preamble interference on a comm payload")
    ap.add_argument("--build-dir", help="gr-uwb build dir with uwb_python")
    ap.add_argument("--output", help="run output directory")
    ap.add_argument("--phase", default="b1",
                    choices=["b0", "b1", "b2", "b3", "b4"])
    ap.add_argument("--seed-set", default="calibration",
                    choices=["calibration", "acceptance"])
    ap.add_argument("--seed-step", type=int, default=1)
    ap.add_argument("--windows", type=int, default=8)
    ap.add_argument("--archive-windows", type=int, default=8)
    ap.add_argument("--progress", type=int, default=0)
    # signal axes
    ap.add_argument("--snr-db", type=float, default=30.0)
    ap.add_argument("--isr-db", type=float, default=10.0)
    ap.add_argument("--position", default="center",
                    choices=["early", "center", "late", "no_overlap"])
    ap.add_argument("--cfo-hz", type=float, default=0.0)
    ap.add_argument("--comm-channel", default="single",
                    choices=["single", "four", "custom_integer_four"],
                    help="desired COMM channel (official four = main.json)")
    ap.add_argument("--interferer-paths", default="single",
                    choices=["single", "four", "custom_integer_four"],
                    help="INTERFERER channel (official four = main.json)")
    ap.add_argument("--interferer", default="on",
                    choices=["on", "off", "wrong_code"])
    ap.add_argument("--wrong-code", type=int, default=11)
    ap.add_argument("--start-jitter", type=int, default=256)
    ap.add_argument("--guard", type=int, default=PL.DEFAULT_GUARD)
    # PHY
    ap.add_argument("--comm-code", type=int, default=PL.DEFAULT_COMM_CODE)
    ap.add_argument("--sensing-code", type=int, default=PL.DEFAULT_SENSING_CODE)
    ap.add_argument("--sync-reps", type=int, default=64)
    ap.add_argument("--sfd-mode", default="4z2")
    ap.add_argument("--psdu-bytes", type=int, default=PL.DEFAULT_PSDU_BYTES)
    ap.add_argument("--pulse-shape", default="legacy")
    ap.add_argument("--peak-amplitude", type=float, default=PL.DEFAULT_PEAK_AMPLITUDE)
    # SIC
    ap.add_argument("--sic", default="estimate", choices=["off", "estimate"])
    ap.add_argument("--sic-k-paths", type=int, default=4)
    ap.add_argument("--sic-fit-reps", type=int, default=8)
    ap.add_argument("--sic-validation-reps", type=int, default=8)
    ap.add_argument("--sic-post-tail", type=int, default=100)
    ap.add_argument("--sic-coarse-stride", type=int,
                    default=DEFAULT_COARSE_STRIDE)
    ap.add_argument("--sic-detect-threshold", type=float,
                    default=DEFAULT_DETECT_THRESHOLD)
    ap.add_argument("--sic-validate-threshold", type=float,
                    default=DEFAULT_VALIDATE_THRESHOLD)
    # demod
    ap.add_argument("--demod-stride", type=int, default=14)
    # orchestration
    ap.add_argument("--smoke", action="store_true",
                    help="8-window functional smoke, no formal claims")
    ap.add_argument("--self-test", action="store_true",
                    help="run targeted QA (no flowgraph) and exit")
    ap.add_argument("--matrix", action="store_true",
                    help="run the staged B1/B2/B3/B4 matrix (child runs)")
    ap.add_argument("--exploratory", action="store_true",
                    help="reduced B2/B3/B4 counts (100/cell); NOT formal")
    return ap


def _default_output(args, phase):
    return os.path.join("gr-uwb", "build-sic-payload", "results",
                        "s2b_%s" % phase)


def main(argv=None):
    ap = build_parser()
    args = ap.parse_args(argv)
    repo = S.find_repo_root()

    if args.self_test:
        return selftest_main(repo)

    if not args.build_dir:
        ap.error("--build-dir is required (unless --self-test)")
    if not args.output:
        args.output = _default_output(args, args.phase)

    if args.smoke:
        args.windows = 8
        args.phase = "b0"
        args.seed_set = args.seed_set if args.seed_set else "calibration"

    if args.matrix:
        return run_matrix(args, repo)

    return run_single(args, repo)


def run_matrix(args, repo):
    """Staged matrix (rectification): B0 + B1 + B2 + B3(12 cells) + B4.

    Formal counts: B0 100 (no interference), B1 100, B2 1000/cell, B3 500/cell
    (12 cells per the original instruction), B4 1000/cell.  Any child rc != 0
    makes the matrix exit nonzero (B-R04).
    """
    root = args.output
    os.makedirs(root, exist_ok=True)
    exploratory = bool(args.exploratory)
    w_b0 = 100
    w_b1 = 100
    w_b2 = 100 if exploratory else 1000
    w_b3 = 100 if exploratory else 500
    w_b4 = 100 if exploratory else 1000
    # Frozen SNR points (calibration scan; see the report/contract).
    frozen_snr = [-5.0, -3.5, 0.0, 4.0, 8.0]
    # Calibration-frozen transition SNR (clean waterfall between -5 and -3.5).
    transition_snr = -3.5
    cells = []
    seed_set = "acceptance"
    cells.append(("b0_clean_snr30", dict(
        phase="b0", position="center", interferer="off", isr_db=0.0,
        snr_db=30.0, windows=w_b0, seed_set=seed_set)))
    cells.append(("b1_center_isr10_snr30", dict(
        phase="b1", position="center", isr_db=10.0, snr_db=30.0,
        windows=w_b1, seed_set=seed_set, interferer="on")))
    for snr in frozen_snr:
        for isr in (0.0, 10.0, 20.0):
            cells.append(("b2_snr%.1f_isr%.0f" % (snr, isr), dict(
                phase="b2", position="center", isr_db=isr, snr_db=snr,
                windows=w_b2, seed_set=seed_set, interferer="on")))
    # B3 (12 cells): early/late x {transition, 30} x ISR10 x single, and
    # CFO +-50k x ISR {0,10} x {single, four(official)}.
    for pos in ("early", "late"):
        for snr, tag in ((transition_snr, "trans"), (30.0, "snr30")):
            cells.append(("b3_pos_%s_%s" % (pos, tag), dict(
                phase="b3", position=pos, isr_db=10.0, snr_db=snr, cfo_hz=0.0,
                interferer_paths="single", windows=w_b3, seed_set=seed_set,
                interferer="on")))
    for cfo in (-50000.0, 50000.0):
        for isr in (0.0, 10.0):
            for ch in ("single", "four"):
                cells.append(("b3_cfo%d_isr%.0f_%s" % (int(cfo), isr, ch), dict(
                    phase="b3", position="center", cfo_hz=cfo, isr_db=isr,
                    snr_db=30.0, interferer_paths=ch, windows=w_b3,
                    seed_set=seed_set, interferer="on")))
    cells.append(("b4_no_sensing", dict(
        phase="b4", position="center", interferer="off",
        isr_db=10.0, snr_db=30.0, windows=w_b4, seed_set=seed_set)))
    cells.append(("b4_wrong_code11", dict(
        phase="b4", position="center", interferer="wrong_code",
        isr_db=10.0, snr_db=30.0, windows=w_b4, seed_set=seed_set)))
    cells.append(("b4_no_overlap", dict(
        phase="b4", position="no_overlap", interferer="on",
        isr_db=10.0, snr_db=30.0, windows=w_b4, seed_set=seed_set)))
    b3_cells = [c for c, ov in cells if ov["phase"] == "b3"]
    cell_summaries = []
    for cid, ov in cells:
        cout = os.path.join(root, "cells", cid)
        child = [sys.executable, os.path.abspath(__file__),
                 "--build-dir", args.build_dir, "--output", cout,
                 "--seed-set", ov.get("seed_set", "acceptance"),
                 "--windows", str(ov.get("windows", 100)),
                 "--archive-windows", "8",
                 "--phase", ov["phase"], "--position", ov["position"],
                 "--snr-db", str(ov["snr_db"]), "--isr-db", str(ov["isr_db"]),
                 "--cfo-hz", str(ov.get("cfo_hz", 0.0)),
                 "--comm-channel", ov.get("comm_channel", "single"),
                 "--interferer-paths", ov.get("interferer_paths", "single"),
                 "--interferer", ov["interferer"],
                 "--sic", args.sic]
        os.makedirs(cout, exist_ok=True)
        log = os.path.join(cout, "run.log")
        t0 = time.time()
        print("== cell %s (%d windows)" % (cid, ov.get("windows", 100)),
              flush=True)
        with open(log, "w") as lf:
            rc = subprocess.call(child, stdout=lf, stderr=subprocess.STDOUT)
        summ = None
        sfile = os.path.join(cout, "summary.json")
        if os.path.exists(sfile):
            with open(sfile) as fh:
                summ = json.load(fh)
        cell_summaries.append({
            "cell": cid, "rc": rc, "wall_s": round(time.time() - t0, 1),
            "spec": {k: v for k, v in ov.items()},
            "per": (summ or {}).get("per"), "paired": (summ or {}).get("paired"),
            "sic": (summ or {}).get("sic"), "gates": (summ or {}).get("gates")})
    aggregate = {
        "schema": "sic-s2b-matrix/1", "exploratory": exploratory,
        "frozen_snr_points": frozen_snr, "transition_snr": transition_snr,
        "counts": {"b0": w_b0, "b1": w_b1, "b2": w_b2, "b3": w_b3, "b4": w_b4},
        "expected_cells": [c for c, _ in cells],
        "b3_expected_cells": b3_cells,
        "cells": cell_summaries,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
    }
    with open(os.path.join(root, "matrix.json"), "w") as fh:
        json.dump(aggregate, fh, indent=2, sort_keys=True)
    failed = [c["cell"] for c in cell_summaries if c["rc"] != 0]
    print("matrix done:", len(cell_summaries), "cells; failed rc:",
          failed, flush=True)
    return 5 if failed else 0


# ---------------------------------------------------------------------------
# Self-test (targeted QA; no flowgraph needed for the pure checks)
# ---------------------------------------------------------------------------
def selftest_main(repo):
    import test_sic_payload as qa
    return qa.run_all(repo, discover_build=os.environ.get("UWB_BUILD_DIR"))


if __name__ == "__main__":
    raise SystemExit(main())
