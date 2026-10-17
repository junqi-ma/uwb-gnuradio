#!/usr/bin/env python3
"""Targeted QA for the S2-B payload-collision study.

Runs the pure-Python signal/geometry/truth-firewall checks always, and the
build-dependent checks (code-10 demod closure, SIC adapter freeze) when a build
directory with ``uwb_python`` is available (``UWB_BUILD_DIR`` or the default
``gr-uwb/build-sic-payload``).  Returns 0 only if no check FAILED; skipped
build-dependent checks are reported, never silently passed.

Usage:
  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/test_sic_payload.py
  env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py --self-test
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sic_preamble_sim as S            # noqa: E402
import sic_payload_sim as PL            # noqa: E402

RESULTS = []


def _ok(name, detail=""):
    RESULTS.append(("PASS", name, detail))
    print("PASS  %-46s %s" % (name, detail), flush=True)


def _fail(name, detail=""):
    RESULTS.append(("FAIL", name, detail))
    print("FAIL  %-46s %s" % (name, detail), flush=True)


def _skip(name, detail=""):
    RESULTS.append(("SKIP", name, detail))
    print("SKIP  %-46s %s" % (name, detail), flush=True)


def find_build(repo, discover_build=None):
    for cand in (discover_build,
                 os.path.join(repo, "gr-uwb", "build-sic-payload"),
                 os.path.join(repo, "gr-uwb", "build")):
        if cand and os.path.exists(os.path.join(
                cand, "python", "uwb", "bindings")):
            import glob
            if glob.glob(os.path.join(cand, "python", "uwb", "bindings",
                                      "uwb_python*.so")):
                return cand
    return None


# ---------------------------------------------------------------------------
def qa_crc_and_fcs():
    canon = S.hex_to_bytes(S.DEFAULT_PSDU_HEX)
    crc = PL.crc16_802154(canon[:-2])
    recv = canon[-2] | (canon[-1] << 8)
    if crc == recv == 0x2112:
        _ok("crc16_matches_canonical", "crc=0x%04x" % crc)
    else:
        _fail("crc16_matches_canonical", "got 0x%04x recv 0x%04x" % (crc, recv))
    data = PL.make_random_data(94, 12345)
    wire = PL.append_fcs(data)
    if len(wire) == 96 and wire[:94] == data:
        _ok("fcs_appended_once", "wire=96B data=94B fcs=%s" % wire[-2:].hex())
    else:
        _fail("fcs_appended_once", "len=%d" % len(wire))


def qa_random_psdu_markers():
    a = PL.make_random_data(94, 1)
    b = PL.make_random_data(94, 2)
    ok = (0x00 in a and 0xFF in a and a != b and len(set(a)) > 8)
    (_ok if ok else _fail)("random_psdu_has_00_ff_and_varies",
                           "n_unique=%d differs=%s"
                           % (len(set(a)), a != b))


def qa_interferer_preamble_only(gen):
    meta = gen.interferer["meta"]
    wave = gen.i_wave
    conv_ok = meta["convolve_max_abs_diff"] == 0.0
    len_ok = (wave.size == meta["natural_length"]
              == S.natural_sync_length(gen.sync_reps, meta["pulse_taps"]))
    # The finite template is SFD-free: the region at/after the last SYNC chip
    # carries ONLY the shaping tail (tiny).  A packet-truncation diagnostic of
    # the same length DOES carry SFD energy there, which the finite template
    # must not.  tail_rel is the (tiny) natural shaping-tail energy fraction.
    sfd_lo = gen.sync_reps * PL.SPS
    sfd_lo = min(sfd_lo, wave.size)
    tail_rel = float(np.sum(np.abs(wave[sfd_lo:]) ** 2)
                     / max(1e-30, np.sum(np.abs(wave) ** 2)))
    full = PL.comm_packet_samples(
        gen.uwb, PL.append_fcs(bytes(94)), sync_reps=gen.sync_reps,
        sfd_mode=gen.sfd_mode, comm_code=gen.sensing_code,
        pulse_shape=gen.pulse_shape, peak_amplitude=gen.peak_amplitude)
    trunc = full[:wave.size]
    sfd_leak = float(np.max(np.abs(
        trunc[sfd_lo:].astype(np.complex128)
        - wave[sfd_lo:].astype(np.complex128)))) if sfd_lo < wave.size else 0.0
    if conv_ok and len_ok and tail_rel < 5e-3 and sfd_leak > 1e-4:
        _ok("interferer_is_preamble_only",
            "len=%d conv=%.1e tail_rel=%.1e sfd_leak=%.3g"
            % (wave.size, meta["convolve_max_abs_diff"], tail_rel, sfd_leak))
    else:
        _fail("interferer_is_preamble_only",
              "conv=%s len_ok=%s tail_rel=%.3e sfd_leak=%.3g"
              % (conv_ok, len_ok, tail_rel, sfd_leak))


def qa_geometry(repo):
    # A pure-geometry generator without a build: use the module helpers.
    fields = PL.comm_field_offsets(64, 8, 96)
    payload_len = fields["payload_hi"] - fields["payload_lo"]
    D = S.natural_sync_length(64, 48)
    G = PL.DEFAULT_GUARD
    room_ok = payload_len >= D + 2 * G
    offs_ok = (fields["sync_lo"] == 0 and fields["sfd_lo"] == 64 * 1016
               and fields["phr_lo"] == 64 * 1016 + 8 * 1016
               and fields["payload_lo"] == fields["phr_lo"] + 21 * 1024)
    short_rejected = False
    try:
        PL.Placement.compute("center", 0, D + 2 * G - 1, D, G)
    except ValueError:
        short_rejected = True
    if offs_ok and room_ok and short_rejected:
        _ok("payload_geometry_and_short_rejection",
            "payload_len=%d D=%d G=%d" % (payload_len, D, G))
    else:
        _fail("payload_geometry_and_short_rejection",
              "offs=%s room=%s short_rej=%s" % (offs_ok, room_ok, short_rejected))


def qa_positions_and_unknown_training(gen):
    pl_lo, pl_hi = gen.payload_lo, gen.payload_hi
    D = gen.i_template_len
    ok_pos = True
    details = []
    for pos in ("early", "center", "late"):
        pl = PL.Placement.compute(pos, pl_lo, pl_hi, D, gen.guard)
        s = pl.nominal_start
        if not (s >= pl_lo + gen.guard and s + D <= pl_hi - gen.guard):
            ok_pos = False
        details.append("%s=%d" % (pos, s))
    # training span must be entirely inside the payload region (unknown payload)
    t0, t1 = gen.training_span(0)
    train_inside = (t0 >= pl_lo and t1 <= pl_hi)
    if ok_pos and train_inside:
        _ok("positions_inside_payload_and_training_unknown",
            " ".join(details))
    else:
        _fail("positions_inside_payload_and_training_unknown",
              "pos_ok=%s training=[%d,%d) payload=[%d,%d)"
              % (ok_pos, t0, t1, pl_lo, pl_hi))


def qa_no_truth_in_online_meta(gen):
    """The meta handed to the online SIC/demod must not carry injection truth."""
    import pmt
    # Reuse the app's meta builder without importing the whole flowgraph.
    meta = pmt.make_dict()
    for k, v in (("window_id", 1), ("code_index", gen.sensing_code),
                 ("sync_repetitions", gen.sync_reps),
                 ("observed_end_sample", 100),
                 ("sensing_roi_lo", 10), ("sensing_roi_hi", 20)):
        meta = pmt.dict_add(meta, pmt.intern(k), pmt.from_long(int(v)))
    keys = set(pmt.to_python(meta).keys())
    banned = {"start_sample", "cfo_hz", "paths", "gain", "i_truth",
              "payload_hex", "psdu_wire_hex", "injected_start", "isr_db"}
    leaked = keys & banned
    if not leaked:
        _ok("no_truth_in_online_meta", "keys=%d" % len(keys))
    else:
        _fail("no_truth_in_online_meta", "leaked=%s" % sorted(leaked))


def qa_observation_budget(gen):
    """observed_end must cover the training span for every allowed jitter."""
    J = PL.DEFAULT_START_JITTER
    rois = gen.public_rois(start_jitter=J, fit_reps=8)
    ok = True
    for off in (-J, 0, J):
        t0, t1 = gen.training_span(off, 8)
        if rois["observed_end"] < t1:
            ok = False
    det_ok = (rois["detect_roi_lo"] <= gen.i_nominal_start - J
              and rois["detect_roi_hi"] >= gen.i_nominal_start + J)
    sub_ok = (rois["sub_roi_lo"] <= gen.i_nominal_start - J
              and rois["sub_roi_hi"]
              >= gen.i_nominal_start + J + gen.i_template_len)
    if ok and det_ok and sub_ok:
        _ok("observation_budget_covers_jitter",
            "observed_end=%d det=[%d,%d) sub=[%d,%d)"
            % (rois["observed_end"], rois["detect_roi_lo"],
               rois["detect_roi_hi"], rois["sub_roi_lo"], rois["sub_roi_hi"]))
    else:
        _fail("observation_budget_covers_jitter",
              "cover=%s det=%s sub=%s" % (ok, det_ok, sub_ok))


def qa_clean_equals_raw_without_interferer(gen):
    win = gen.build(seed=7, payload_seed=8, interferer_present=False)
    identical = bool(np.array_equal(win["y"], win["clean"]))
    i_zero = bool(np.count_nonzero(win["i"]) == 0)
    if identical and i_zero:
        _ok("clean_bit_identical_when_no_interferer", "n=%d" % win["y"].size)
    else:
        _fail("clean_bit_identical_when_no_interferer",
              "identical=%s i_zero=%s" % (identical, i_zero))


def qa_wrong_code_low_metric(gen, wrong_code=11):
    wt = PL.build_finite_sync_template(
        gen.uwb, code_index=wrong_code, sync_reps=gen.sync_reps,
        sfd_mode=gen.sfd_mode, pulse_shape=gen.pulse_shape,
        peak_amplitude=gen.peak_amplitude)
    win = gen.build(seed=9, payload_seed=10, interferer_present=True,
                    interferer_template=wt["waveform"])
    y = win["y"].astype(np.complex128)
    sync = gen.i_sync.astype(np.complex128)
    Ls = sync.size
    d = gen.i_nominal_start
    seg = y[d:d + Ls]
    m = abs(np.vdot(sync, seg)) / (np.linalg.norm(seg) * np.linalg.norm(sync))
    if m < 0.30:
        _ok("wrong_code_low_correlation", "metric=%.3f" % m)
    else:
        _fail("wrong_code_low_correlation", "metric=%.3f (too high)" % m)


# ---------------------------------------------------------------------------
# Build-dependent checks
# ---------------------------------------------------------------------------
def qa_build(repo, build_dir):
    import pmt
    from gnuradio import gr
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import uwb_sim_sic_payload as APP
    uwb = S.load_uwb(build_dir)
    gen = PL.SicPayloadSignalGenerator(uwb, position="center",
                                       snr_db=30.0, isr_db=10.0)
    import tempfile
    tpl = os.path.join(tempfile.mkdtemp(), "comm_sync_code10.cf32")
    gen.comm_sync.tofile(tpl)

    # (a) clean closure: code-10 packet decodes byte-exactly.
    win = gen.build(seed=21000000, payload_seed=21000001)
    rt = APP.PayloadRuntime(uwb, tpl_path=tpl, demod_code=10, sync_reps=64,
                            sfd_mode="4z2", demod_stride=14, sic_cfg=None)
    rt.start()
    rt.post_demod(APP._demod_meta(1, gen.comm_start, win["y"].size),
                  win["clean"])
    got = rt.wait_keys(rt.sink_demod, [1], deadline_s=60.0)
    d = rt.sink_demod.get(1) or {}
    rt.stop()
    wire = win["truth"]["psdu_wire_hex"]
    if got and d.get("status") == "success" and d.get("fcs_pass") \
            and (d.get("__u8") or b"").hex().upper() == wire:
        _ok("build_clean_code10_closure", "psdu=%dB" % (len(wire) // 2))
    else:
        _fail("build_clean_code10_closure",
              "status=%s fcs=%s match=%s" % (
                  d.get("status"), d.get("fcs_pass"),
                  (d.get("__u8") or b"").hex().upper() == wire))

    # (b) code-9 mismatch: a code-10 signal demodulated as code-9 must fail.
    rt = APP.PayloadRuntime(uwb, tpl_path=tpl, demod_code=9, sync_reps=64,
                            sfd_mode="4z2", demod_stride=14, sic_cfg=None)
    rt.start()
    rt.post_demod(APP._demod_meta(2, gen.comm_start, win["y"].size),
                  win["clean"])
    rt.wait_keys(rt.sink_demod, [2], deadline_s=60.0)
    d2 = rt.sink_demod.get(2) or {}
    rt.stop()
    if not (d2.get("status") == "success"
            and (d2.get("__u8") or b"").hex().upper() == wire):
        _ok("build_code_role_mismatch_fails",
            "code9 status=%s" % d2.get("status"))
    else:
        _fail("build_code_role_mismatch_fails",
              "code-9 profile wrongly decoded a code-10 packet")

    # (c) SIC adapter: freeze + mask exactness + no truth leak.
    rois = gen.public_rois()
    sic_cfg = {"sync": gen.i_sync, "preamble": gen.i_wave, "comm_code": 9,
               "sync_reps": 64, "k_paths": 4, "fit_reps": 8,
               "validation_reps": 8, "roi_lo": rois["detect_roi_lo"],
               "roi_hi": rois["detect_roi_hi"], "coarse_stride": 2,
               "mode": "estimate", "detect_threshold": 0.40,
               "validate_threshold": 0.25, "post_tail": 100,
               "support_mode": "configured_nominal", "online_validation_reps": 0}
    rt = APP.PayloadRuntime(uwb, tpl_path=tpl, demod_code=10, sync_reps=64,
                            sfd_mode="4z2", demod_stride=14, sic_cfg=sic_cfg)
    rt.start()
    rt.post_sic(APP._sic_meta(3, gen, rois, win["y"].size, 9, 64, "4z2"),
                win["y"])
    rt.wait_keys(rt.sink_out, [3], deadline_s=60.0)
    o = rt.sink_out.get(3) or {}
    yhat = o.get("__c32")
    committed = bool(o.get("sic_committed"))
    mask_lo = int(o.get("sic_sub_lo") or 0)
    mask_hi = int(o.get("sic_sub_hi") or 0)
    changed_outside = -1
    if yhat is not None and yhat.size == win["y"].size and mask_hi > mask_lo:
        outside = np.ones(win["y"].size, dtype=bool)
        outside[mask_lo:mask_hi] = False
        changed_outside = int(np.count_nonzero(
            np.abs(yhat[outside].astype(np.complex128)
                   - win["y"][outside].astype(np.complex128)) > 0.0))
    refit = o.get("sic_apply_refit_count")
    uses_truth = o.get("sic_uses_truth")
    # Frozen-model invariance to a mutation AFTER observed_end.
    y2 = win["y"].copy()
    oe = int(rois["observed_end"])
    y2[oe:] = (y2[oe:] * 0.0).astype(np.complex64)  # zero the visible tail
    rt.post_sic(APP._sic_meta(4, gen, rois, y2.size, 9, 64, "4z2"), y2)
    rt.wait_keys(rt.sink_out, [4], deadline_s=60.0)
    o2 = rt.sink_out.get(4) or {}
    rt.stop()
    fp_equal = (o.get("sic_model_fingerprint") == o2.get("sic_model_fingerprint"))
    if (committed and changed_outside == 0 and refit == 0
            and uses_truth is False and fp_equal):
        _ok("build_sic_freeze_and_mask",
            "commit=%s mask_outside_changed=%d refit=%s fp_equal=%s"
            % (committed, changed_outside, refit, fp_equal))
    else:
        _fail("build_sic_freeze_and_mask",
              "commit=%s changed_outside=%s refit=%s uses_truth=%s fp_eq=%s"
              % (committed, changed_outside, refit, uses_truth, fp_equal))


# ---------------------------------------------------------------------------
def run_all(repo, discover_build=None):
    del RESULTS[:]
    print("== S2-B targeted QA ==", flush=True)
    qa_crc_and_fcs()
    qa_random_psdu_markers()
    qa_geometry(repo)
    build = find_build(repo, discover_build)
    if build is None:
        _skip("build_clean_code10_closure", "no build dir with uwb_python")
        _skip("build_code_role_mismatch_fails", "no build dir")
        _skip("build_sic_freeze_and_mask", "no build dir")
        # Geometry-only generator for the pure checks needs a build for the
        # modulator; run them only when a build exists.
        _skip("pure_signal_checks", "need uwb_python to build templates")
    else:
        uwb = S.load_uwb(build)
        gen = PL.SicPayloadSignalGenerator(uwb, position="center",
                                           snr_db=30.0, isr_db=10.0)
        qa_interferer_preamble_only(gen)
        qa_positions_and_unknown_training(gen)
        qa_no_truth_in_online_meta(gen)
        qa_observation_budget(gen)
        qa_clean_equals_raw_without_interferer(gen)
        qa_wrong_code_low_metric(gen)
        try:
            qa_build(repo, build)
        except Exception as exc:  # pragma: no cover
            _fail("build_checks_exception", repr(exc))
    n_fail = sum(1 for r in RESULTS if r[0] == "FAIL")
    n_skip = sum(1 for r in RESULTS if r[0] == "SKIP")
    print("== QA done: %d pass, %d fail, %d skip =="
          % (sum(1 for r in RESULTS if r[0] == "PASS"), n_fail, n_skip),
          flush=True)
    return 1 if n_fail else 0


if __name__ == "__main__":
    root = S.find_repo_root()
    raise SystemExit(run_all(root, discover_build=os.environ.get("UWB_BUILD_DIR")))
