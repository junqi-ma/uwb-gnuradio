#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""INDEPENDENT verifier for the M2-A native-rate PHY demo.

It accepts the versioned `twr-m2a-native/2` schema (G0 appendix A.5) and, for
backward compatibility, the older `twr-m2a-native/1` schema.

WHAT IT CHECKS, AND HOW IT IS INDEPENDENT
-----------------------------------------
It reads ONLY the demo's JSON manifest and recomputes every expected value in
Python.  It never imports, calls or links the C++ helper, the C++ codec, the C++
resampler, the C++ impairment model or the C++ demo: two C++ functions agreeing
is never treated as truth.

For `twr-m2a-native/2` it independently derives:

  * the MAC byte layout from the frame FIELDS and compares it byte-for-byte
    with ``mac_hex``; the CRC-16 (reflected poly 0x8408, init 0, no final xor,
    little-endian append) over the reconstructed MAC and compares it with
    ``fcs`` / ``fcs_hex`` / the PSDU tail;
  * the expected work-grid waveform length and the expected native / RX lengths
    from the CAUSAL FULL-CONVOLUTION contract ``ceil(((N-1)*L + T)/M)``;
  * the decode verdict, the comparator verdict, the provenance hashes and the
    executed-config hash, exactly as for /1;
  * the impairment block: the parameters must be echoed AND self-consistent.
    When AWGN is enabled the SNR must be the one implied by the reported
    valid-region mean power and per-dimension variance
    (``SNR = 10*log10(P_valid / (2*sigma^2))``), the region must be inside the
    buffer, and the region-based power must differ from the padded full-buffer
    mean when padding exists (i.e. the SNR is NOT diluted by leading/trailing
    zeros).  The parameters are also cross-checked against ``config.requested``;
  * the stage coordinates: every stage states its unit; ``origin``/``trim`` are
    the frozen values; ``pad_front`` + ``pad_back`` == ``padding`` and, for the
    resample stages, equal the tap-count split; the demod search guards and ROI
    are recomputed from the reported input length and padding; and the
    diagnostic decode coordinates are rebased into ``[0, work_rx)`` (or -1 on a
    failed decode, never a padded index);
  * the ``kernel`` names are real kernel names for this repository;
  * ``context.prepared`` is true and ``context.capacity_samples`` covers every
    observed buffer length; the context path matches whether the impairment is
    an identity;
  * the evidence ceiling: ``native_roundtrip_verified`` only on a real
    byte-exact success, a failed scenario never claims native evidence,
    ``measurement_valid`` is false and ``hardware_readback`` is null.

``--self-test`` mutates a deep copy of the manifest and asserts each mutation
makes the verifier fail: MAC byte, FCS, lengths, stage counts/delay/out_count,
payload, decode status/fcs_pass/PHR length, comparator, evidence level/kind,
``measurement_valid``, ``hardware_readback``, ``yields_range``, provenance and
executed hashes, an effective config field, ``matlab.executed``, AND the new /2
fields (impairment parameters and region, origin, pad_front/pad_back, unit,
search guards/ROI, kernel names, context capacity/path, status counters,
diagnostic coordinates).

    python3 tools/twr/verify_m2_a_native_phy.py --output out.json
    python3 tools/twr/verify_m2_a_native_phy.py --output out.json --self-test

Exit status 0 iff every check passes; non-zero with a clear diff otherwise.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
import struct
import sys

# ---------------------------------------------------------------------------
# Frozen constants (G0 §1, §3, §6, appendix A.3/A.5).  Repeated here on purpose:
# this file is a second, independent statement of the contract, not a reader of
# the C++.
# ---------------------------------------------------------------------------

WORK_HZ = 998400000.0
NATIVE_HZ = {"737280000": 737280000.0, "491520000": 491520000.0}
TX_LM = {"737280000": (48, 65), "491520000": (32, 65)}
RX_L = 65

SYNC_REPS = 64
SYNC_SAMPLES_PER_SYMBOL = 1016
SFD_SYMBOLS = 8
PHR_SAMPLES = 21 * 512 * 2  # 21 symbols * 512 chips * 2 samples/chip
PAYLOAD_SAMPLES_PER_SYMBOL = 128

FRAME_FUNCTION = {"poll": 0, "response": 1, "final": 2}
FRAME_TIMESTAMPS = {"poll": 0, "response": 2, "final": 3}
FRAME_MAC_BYTES = {"poll": 14, "response": 24, "final": 29}
TIMESTAMP_BITS = 40
TIMESTAMP_BYTES = TIMESTAMP_BITS // 8

MAX_ABS_ERROR = 1e-4
RELATIVE_L2 = 1e-5

# The frozen demod timing-search margin (uwb_phy_profile.h: 40960 samples).
TIMING_SEARCH_MARGIN = 40960
M2A_MAX_SAMPLES = 1 << 20

EVIDENCE_ALLOWED = {"work_decode_verified", "native_roundtrip_verified"}
EVIDENCE_ALLOWED_V2 = {"none", "work_decode_verified", "native_roundtrip_verified"}
EVIDENCE_ABOVE = {
    "toa_verified",
    "hardware_verified",
    "vendor_interop_verified",
    "ranging",
}

# Real kernel names for this repository.
KERNEL_RESAMPLER = {
    "scalar_macroblock",
    "volk_macroblock",
    "avx2_fma_macroblock",
    "scalar_legacy",
    "volk_legacy",
}
KERNEL_DEMOD = {"multi_acc8", "volk", "avx2_fixed38"}

M2A_STATUSES = {
    "ok", "invalid_config", "invalid_frame", "modulate_failed",
    "tx_resample_failed", "quantise_failed", "rx_resample_failed",
    "demod_failed", "fcs_failed", "bytes_mismatch", "length_mismatch",
    "capacity_exceeded", "internal_error",
}
DEMOD_STATUSES = {
    "success", "invalid_input", "timing_failed", "cfo_failed", "sfd_failed",
    "phr_failed", "payload_failed", "fcs_failed", "queue_full",
    "internal_error", "cir_failed",
}
CONTEXT_PATHS = {"prepared_context", "stage_runner_impairment"}

HEX64 = re.compile(r"^[0-9a-f]{64}$")

results = []
_QUIET = False


def record(item, verdict, detail):
    results.append((item, verdict, detail))
    if not _QUIET:
        print("[%s] %s\n         %s" % (verdict, item, detail))


def fail(item, detail):
    record(item, "FAIL", detail)


def ok(item, detail):
    record(item, "PASS", detail)


# ---------------------------------------------------------------------------
# Independent recomputation
# ---------------------------------------------------------------------------


def crc16_802154(data: bytes) -> int:
    """CRC-16/X.25 as used by IEEE 802.15.4: reflected poly 0x8408, init 0."""
    crc = 0x0000
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def reconstruct_mac(frame: dict):
    """(bytes, error) rebuilt from the frame fields, never from mac_hex."""
    ftype = frame.get("type")
    if ftype not in FRAME_FUNCTION:
        return None, "frame.type %r is not poll/response/final" % (ftype,)
    fields = frame.get("fields")
    if not isinstance(fields, dict):
        return None, "frame.fields is missing (cannot rebuild the bytes)"
    try:
        version = int(fields["version"])
        function_code = int(fields["function_code"])
        session_id = int(fields["session_id"])
        seq = int(fields["seq"])
        pan_id = int(fields["pan_id"])
        src_addr = int(fields["src_addr"])
        dst_addr = int(fields["dst_addr"])
        flags = int(fields["flags"])
        timestamps = [int(v) for v in fields["timestamps"]]
    except (KeyError, TypeError, ValueError) as exc:
        return None, "frame.fields is incomplete: %s" % (exc,)

    n_ts = FRAME_TIMESTAMPS[ftype]
    if len(timestamps) != n_ts:
        return None, "frame.fields.timestamps has %d entries, want %d" % (
            len(timestamps), n_ts)
    if version != 1:
        return None, "frame.fields.version is %d, want 1" % (version,)
    if function_code != FRAME_FUNCTION[ftype]:
        return None, "function_code %d does not match type %s (%d)" % (
            function_code, ftype, FRAME_FUNCTION[ftype])
    if flags & 0xFFFC:
        return None, "flags 0x%04x sets reserved bits 2..15" % (flags,)
    if flags & 0x0002:
        return None, "flags 0x%04x sets sts_present (out of scope)" % (flags,)
    for name, value in (("session_id", session_id), ("seq", seq), ("pan_id", pan_id),
                        ("src_addr", src_addr), ("dst_addr", dst_addr),
                        ("flags", flags)):
        if not (0 <= value <= 0xFFFF):
            return None, "frame.fields.%s = %d does not fit u16" % (name, value)
    for i, value in enumerate(timestamps):
        if not (0 <= value <= (1 << TIMESTAMP_BITS) - 1):
            return None, "timestamp[%d] = %d does not fit %d bits" % (
                i, value, TIMESTAMP_BITS)

    out = bytearray()
    out.append(version)
    out.append(function_code)
    for value in (session_id, seq, pan_id, src_addr, dst_addr, flags):
        out += struct.pack("<H", value)
    for value in timestamps:
        out += bytes((value >> (8 * i)) & 0xFF for i in range(TIMESTAMP_BYTES))
    return bytes(out), None


def expected_work_samples(psdu_bytes: int) -> int:
    bits = psdu_bytes * 8
    blocks = (bits + 329) // 330
    nsym = bits + 48 * blocks
    return (SYNC_REPS * SYNC_SAMPLES_PER_SYMBOL
            + SFD_SYMBOLS * SYNC_SAMPLES_PER_SYMBOL
            + PHR_SAMPLES
            + nsym * PAYLOAD_SAMPLES_PER_SYMBOL)


def resampled_length(n: int, taps: int, l: int, m: int) -> int:
    if n == 0 or taps == 0 or l == 0 or m == 0:
        return 0
    num = (n - 1) * l + taps
    if num <= 0:
        return 0
    return (num + m - 1) // m


def effective_canonical(eff: dict) -> str:
    """The integer-only canonical text the C++ writes before hashing."""
    keys = ["frame", "native_rate", "tx_l", "tx_m", "rx_l", "rx_m", "iq_format",
            "code_index", "sync_repetitions", "sfd_mode", "insert_sts", "ranging",
            "peak_amplitude_micro", "sc16_scale_micro", "tx_taps", "rx_taps",
            "timestamp_bits"]
    lines = []
    for key in keys:
        value = eff.get(key)
        if isinstance(value, bool):
            text = "true" if value else "false"
        else:
            text = str(value)
        lines.append("%s=%s" % (key, text))
    return "\n".join(lines) + "\n"


def stage_by_name(out: dict, name: str):
    for stage in out.get("stages") or []:
        if stage.get("name") == name:
            return stage
    return None


def is_hex64(value) -> bool:
    return isinstance(value, str) and HEX64.match(value) is not None


def is_num(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_int(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


# ---------------------------------------------------------------------------
# Frame / FCS / lengths / decode / compare (shared by /1 and /2)
# ---------------------------------------------------------------------------


def check_frame_and_fcs(out):
    frame = out.get("frame")
    if not isinstance(frame, dict):
        fail("frame block is present", "got %r" % (frame,))
        return None, None
    ftype = frame.get("type")
    mac, err = reconstruct_mac(frame)
    if mac is None:
        fail("the MAC byte layout is rebuildable from the frame fields", err)
        return None, None

    mac_hex = frame.get("mac_hex")
    try:
        mac_wire = bytes.fromhex(mac_hex)
    except (TypeError, ValueError):
        fail("frame.mac_hex is valid hex", "got %r" % (mac_hex,))
        return None, None

    if len(mac) == FRAME_MAC_BYTES.get(ftype):
        ok("the reconstructed MAC length matches frame v1 geometry",
           "%s -> %d B" % (ftype, len(mac)))
    else:
        fail("the reconstructed MAC length matches frame v1 geometry",
             "%s -> %d B, want %d" % (ftype, len(mac), FRAME_MAC_BYTES.get(ftype)))
    if frame.get("mac_bytes") == len(mac):
        ok("frame.mac_bytes equals the reconstructed length", str(len(mac)))
    else:
        fail("frame.mac_bytes equals the reconstructed length",
             "mac_bytes=%r reconstructed=%d" % (frame.get("mac_bytes"), len(mac)))
    if mac == mac_wire:
        ok("frame.mac_hex equals the bytes rebuilt from the frame fields",
           "%d B byte-exact" % len(mac))
    else:
        fail("frame.mac_hex equals the bytes rebuilt from the frame fields",
             "reconstructed=%s wire=%s" % (mac.hex(), mac_wire.hex()))

    crc = crc16_802154(mac)
    want_fcs = struct.pack("<H", crc)
    if frame.get("fcs") == "0x%04x" % crc:
        ok("frame.fcs equals the independently computed CRC-16", "0x%04x" % crc)
    else:
        fail("frame.fcs equals the independently computed CRC-16",
             "fcs=%r recomputed=0x%04x" % (frame.get("fcs"), crc))
    if frame.get("fcs_hex") == "%04x" % crc:
        ok("frame.fcs_hex equals the independently computed CRC-16", "%04x" % crc)
    else:
        fail("frame.fcs_hex equals the independently computed CRC-16",
             "fcs_hex=%r recomputed=%04x" % (frame.get("fcs_hex"), crc))

    psdu = mac + want_fcs
    if frame.get("psdu_bytes") == len(psdu):
        ok("frame.psdu_bytes equals MAC + 2 FCS bytes", str(len(psdu)))
    else:
        fail("frame.psdu_bytes equals MAC + 2 FCS bytes",
             "psdu_bytes=%r want %d" % (frame.get("psdu_bytes"), len(psdu)))
    return mac, psdu


def impairment_growth(out) -> int:
    """The extra native samples a full-convolution multipath channel adds.

    CFO / AWGN / integer+fractional delay all preserve the buffer length; only
    multipath (full convolution with M taps) grows it by M-1.  Read from the
    declared taps, never from the C++.
    """
    imp = out.get("impairment")
    if not isinstance(imp, dict):
        return 0
    taps = imp.get("multipath")
    if isinstance(taps, list) and taps:
        return len(taps) - 1
    return 0


def check_lengths(out, mac, psdu):
    rate = out.get("rate") or {}
    samples = out.get("samples") or {}
    native_key = None
    for key, hz in NATIVE_HZ.items():
        if rate.get("native_hz") == hz:
            native_key = key
    if native_key is None:
        fail("rate.native_hz is one of the two frozen rates",
             "got %r" % (rate.get("native_hz"),))
        return None, None, None, None
    tx_l, tx_m = TX_LM[native_key]
    if (rate.get("tx_l"), rate.get("tx_m"), rate.get("rx_l"), rate.get("rx_m")) == \
            (tx_l, tx_m, RX_L, tx_l):
        ok("rate L/M match the frozen direction table", "%d/%d and %d/%d" % (
            tx_l, tx_m, RX_L, tx_l))
    else:
        fail("rate L/M match the frozen direction table",
             "tx=%r/%r rx=%r/%r want %d/%d and %d/%d" % (
                 rate.get("tx_l"), rate.get("tx_m"), rate.get("rx_l"),
                 rate.get("rx_m"), tx_l, tx_m, RX_L, tx_l))
    if rate.get("work_hz") == WORK_HZ:
        ok("rate.work_hz is the frozen 998.4 MS/s work grid", str(WORK_HZ))
    else:
        fail("rate.work_hz is the frozen 998.4 MS/s work grid",
             "got %r" % (rate.get("work_hz"),))

    tx_taps = rate.get("tx_taps")
    rx_taps = rate.get("rx_taps")
    if not (isinstance(tx_taps, int) and tx_taps > 1 and
            isinstance(rx_taps, int) and rx_taps > 1):
        fail("rate.tx_taps/rx_taps are integer tap counts", "tx=%r rx=%r" % (
            tx_taps, rx_taps))
        return None, None, None, None

    want_work = expected_work_samples(len(psdu))
    if samples.get("work_tx") == want_work:
        ok("samples.work_tx equals the frozen waveform-length formula",
           "%d for a %d B PSDU" % (want_work, len(psdu)))
    else:
        fail("samples.work_tx equals the frozen waveform-length formula",
             "work_tx=%r want %d" % (samples.get("work_tx"), want_work))

    work = samples.get("work_tx")
    want_native = resampled_length(work, tx_taps, tx_l, tx_m)
    if samples.get("native") == want_native:
        ok("samples.native equals ceil(((N-1)*L+T)/M)",
           "%d -> %d (L=%d M=%d T=%d)" % (work, want_native, tx_l, tx_m, tx_taps))
    else:
        fail("samples.native equals ceil(((N-1)*L+T)/M)",
             "native=%r want %d (N=%r L=%d M=%d T=%d)" % (
                 samples.get("native"), want_native, work, tx_l, tx_m, tx_taps))

    native = samples.get("native")
    growth = impairment_growth(out)
    impaired = native + growth if isinstance(native, int) else None
    if "native_impaired" in samples or growth:
        if impaired is not None and samples.get("native_impaired") == impaired:
            ok("samples.native_impaired is native + the multipath tail",
               "%d = %d + %d" % (impaired, native, growth))
        else:
            fail("samples.native_impaired is native + the multipath tail",
                 "got %r want %r (native=%r growth=%d)" % (
                     samples.get("native_impaired"), impaired, native, growth))

    want_rx = resampled_length(impaired, rx_taps, RX_L, tx_l)
    if samples.get("work_rx") == want_rx:
        ok("samples.work_rx equals ceil(((N-1)*L+T)/M) on the RX direction",
           "%d -> %d (L=%d M=%d T=%d)" % (impaired, want_rx, RX_L, tx_l, rx_taps))
    else:
        fail("samples.work_rx equals ceil(((N-1)*L+T)/M) on the RX direction",
             "work_rx=%r want %d (N=%r L=%d M=%d T=%d)" % (
                 samples.get("work_rx"), want_rx, impaired, RX_L, tx_l, rx_taps))
    return (tx_l, tx_m, tx_taps, rx_taps), want_work, want_native, impaired


def check_filter(out):
    filt = out.get("filter")
    rate = out.get("rate") or {}
    if not isinstance(filt, dict):
        fail("filter block is present", "got %r" % (filt,))
        return
    if filt.get("tx_taps") == rate.get("tx_taps") and \
            filt.get("rx_taps") == rate.get("rx_taps"):
        ok("filter tap counts agree with the rate block",
           "tx=%r rx=%r" % (filt.get("tx_taps"), filt.get("rx_taps")))
    else:
        fail("filter tap counts agree with the rate block",
             "filter=%r/%r rate=%r/%r" % (
                 filt.get("tx_taps"), filt.get("rx_taps"),
                 rate.get("tx_taps"), rate.get("rx_taps")))
    if filt.get("phase") == 0 and filt.get("crop") == 0:
        ok("filter records a causal, uncropped chain", "phase=0 crop=0")
    else:
        fail("filter records a causal, uncropped chain",
             "phase=%r crop=%r" % (filt.get("phase"), filt.get("crop")))
    tx_taps = rate.get("tx_taps")
    if isinstance(tx_taps, int) and filt.get("group_delay") == 0.5 * (tx_taps - 1):
        ok("filter.group_delay is 0.5*(T-1)", str(filt.get("group_delay")))
    else:
        fail("filter.group_delay is 0.5*(T-1)",
             "got %r want %r" % (filt.get("group_delay"),
                                 0.5 * (tx_taps - 1) if isinstance(tx_taps, int) else None))
    scale = filt.get("scale")
    if isinstance(scale, (int, float)) and scale > 0:
        ok("filter.scale is positive", str(scale))
    else:
        fail("filter.scale is positive", "got %r" % (scale,))


def check_provenance(out, mac, psdu):
    prov = out.get("provenance")
    if not isinstance(prov, dict):
        fail("provenance block is present", "got %r" % (prov,))
        return
    if is_hex64(prov.get("input_sha256")):
        ok("provenance.input_sha256 is well formed", prov["input_sha256"])
    else:
        fail("provenance.input_sha256 is well formed",
             "got %r" % (prov.get("input_sha256"),))
    want_in = hashlib.sha256(mac).hexdigest()
    if prov.get("input_sha256") == want_in:
        ok("provenance.input_sha256 is the SHA-256 of the reconstructed MAC", want_in)
    else:
        fail("provenance.input_sha256 is the SHA-256 of the reconstructed MAC",
             "output=%r recomputed=%s" % (prov.get("input_sha256"), want_in))
    if is_hex64(prov.get("output_sha256")):
        ok("provenance.output_sha256 is well formed", prov["output_sha256"])
    else:
        fail("provenance.output_sha256 is well formed",
             "got %r" % (prov.get("output_sha256"),))
    dec = out.get("decode") or {}
    try:
        decoded = bytes.fromhex(dec.get("payload_hex") or "")
    except (TypeError, ValueError):
        decoded = None
    if decoded is None:
        fail("provenance.output_sha256 is the SHA-256 of the decoded payload",
             "decode.payload_hex is not valid hex")
    else:
        want_out = hashlib.sha256(decoded).hexdigest()
        if prov.get("output_sha256") == want_out:
            ok("provenance.output_sha256 is the SHA-256 of the decoded payload",
               want_out)
        else:
            fail("provenance.output_sha256 is the SHA-256 of the decoded payload",
                 "output=%r recomputed=%s" % (prov.get("output_sha256"), want_out))
    if isinstance(prov.get("dirty"), bool):
        ok("provenance.dirty is a boolean", str(prov.get("dirty")))
    else:
        fail("provenance.dirty is a boolean", "got %r" % (prov.get("dirty"),))
    for key in ("revision", "compiler", "loaded_library"):
        value = prov.get(key)
        if isinstance(value, str) and value:
            ok("provenance.%s is a non-empty string" % key, value[:120])
        else:
            fail("provenance.%s is a non-empty string" % key, "got %r" % (value,))
    matlab = prov.get("matlab")
    if isinstance(matlab, dict) and matlab.get("executed") is False:
        ok("provenance.matlab.executed is false (no MATLAB claim)", "executed=false")
    else:
        fail("provenance.matlab.executed is false (no MATLAB claim)",
             "got %r" % (matlab,))


def check_config(out):
    cfg = out.get("config")
    if not isinstance(cfg, dict):
        fail("config block is present", "got %r" % (cfg,))
        return
    eff = cfg.get("effective")
    if not isinstance(eff, dict):
        fail("config.effective is present", "got %r" % (eff,))
        return
    want = hashlib.sha256(effective_canonical(eff).encode("utf-8")).hexdigest()
    if cfg.get("executed_sha256") == want:
        ok("config.executed_sha256 is the SHA-256 of the canonical effective text",
           want)
    else:
        fail("config.executed_sha256 is the SHA-256 of the canonical effective text",
             "output=%r recomputed=%s" % (cfg.get("executed_sha256"), want))
    req = cfg.get("requested")
    frame = out.get("frame") or {}
    rate = out.get("rate") or {}
    if isinstance(req, dict) and req.get("frame") == frame.get("type") and \
            req.get("native_rate") == rate.get("native_hz"):
        ok("config.requested agrees with the frame and rate actually run",
           "frame=%s native_rate=%s" % (req.get("frame"), req.get("native_rate")))
    else:
        fail("config.requested agrees with the frame and rate actually run",
             "requested=%r frame=%r rate=%r" % (req, frame.get("type"),
                                                rate.get("native_hz")))
    checks = [
        ("frame", frame.get("type")),
        ("native_rate", rate.get("native_hz")),
        ("tx_l", rate.get("tx_l")),
        ("tx_m", rate.get("tx_m")),
        ("rx_l", rate.get("rx_l")),
        ("rx_m", rate.get("rx_m")),
        ("tx_taps", rate.get("tx_taps")),
        ("rx_taps", rate.get("rx_taps")),
        ("code_index", 9),
        ("sync_repetitions", 64),
        ("sfd_mode", "ieee"),
        ("insert_sts", False),
        ("ranging", True),
        ("timestamp_bits", 40),
    ]
    bad = ["%s=%r want %r" % (k, eff.get(k), v) for k, v in checks
           if eff.get(k) != v]
    if bad:
        fail("config.effective restates the frozen minimal profile", "; ".join(bad))
    else:
        ok("config.effective restates the frozen minimal profile",
           "%d fields cross-checked" % len(checks))


# ---------------------------------------------------------------------------
# /2: success verdict, status, evidence, decode, compare
# ---------------------------------------------------------------------------


def success_verdict(out, psdu):
    dec = out.get("decode") or {}
    comp = out.get("compare") or {}
    try:
        payload = bytes.fromhex(dec.get("payload_hex") or "")
    except (TypeError, ValueError):
        payload = None
    ok_ = (dec.get("status") == "success" and dec.get("fcs_pass") is True
           and comp.get("bytes_exact") is True)
    if payload is not None:
        ok_ = ok_ and payload == psdu
    return bool(ok_)


def check_status_v2(out, measured):
    status = out.get("status")
    if not isinstance(status, dict):
        fail("status block is present", "got %r" % (status,))
        return
    attempted = status.get("attempted")
    success = status.get("exact_success")
    failure = status.get("explicit_failure")
    if is_int(attempted) and attempted > 0 and is_int(success) and success >= 0 \
            and is_int(failure) and failure >= 0:
        ok("status counters are non-negative integers",
           "attempted=%d exact_success=%d explicit_failure=%d" % (
               attempted, success, failure))
    else:
        fail("status counters are non-negative integers",
             "attempted=%r exact_success=%r explicit_failure=%r" % (
                 attempted, success, failure))
        return
    if attempted == success + failure:
        ok("status.attempted == exact_success + explicit_failure",
           "%d == %d + %d" % (attempted, success, failure))
    else:
        fail("status.attempted == exact_success + explicit_failure",
             "%d != %d + %d" % (attempted, success, failure))
    ok_flag = status.get("ok")
    if ok_flag == (success == attempted):
        ok("status.ok is true iff every attempt succeeded",
           "ok=%r" % (ok_flag,))
    else:
        fail("status.ok is true iff every attempt succeeded",
             "ok=%r success=%d attempted=%d" % (ok_flag, success, attempted))
    exit_code = status.get("exit_code")
    if exit_code == (0 if ok_flag is True else 1):
        ok("status.exit_code matches status.ok", str(exit_code))
    else:
        fail("status.exit_code matches status.ok",
             "exit_code=%r ok=%r" % (exit_code, ok_flag))
    if status.get("m2a_status") in M2A_STATUSES:
        ok("status.m2a_status is a known terminal state", status.get("m2a_status"))
    else:
        fail("status.m2a_status is a known terminal state",
             "got %r" % (status.get("m2a_status"),))
    if not isinstance(status.get("reason"), str):
        fail("status.reason is a string", "got %r" % (status.get("reason"),))
    else:
        ok("status.reason is a string", status.get("reason")[:120] or "(empty)")
    # The overall success flag must agree with the independently derived verdict.
    if ok_flag is True and not measured:
        fail("status.ok agrees with the independently derived decode verdict",
             "ok=true but decode/compare/payload are not a byte-exact success")
    elif ok_flag is False and measured:
        fail("status.ok agrees with the independently derived decode verdict",
             "ok=false but the decode is byte-exact")
    else:
        ok("status.ok agrees with the independently derived decode verdict",
           "ok=%r measured=%s" % (ok_flag, measured))


def check_evidence_v2(out, measured):
    ev = out.get("evidence")
    if not isinstance(ev, dict):
        fail("evidence block is present", "got %r" % (ev,))
        return
    level = ev.get("level")
    if level in EVIDENCE_ABOVE:
        fail("evidence.level does not exceed native_roundtrip_verified",
             "level=%r is above the M2-A ceiling" % (level,))
    elif level in EVIDENCE_ALLOWED_V2:
        ok("evidence.level is within the M2-A ceiling", str(level))
    else:
        fail("evidence.level is within the M2-A ceiling",
             "level=%r not in %s" % (level, sorted(EVIDENCE_ALLOWED_V2)))
    if ev.get("measurement_valid") is False:
        ok("evidence.measurement_valid is false", "false")
    else:
        fail("evidence.measurement_valid is false",
             "got %r" % (ev.get("measurement_valid"),))
    if ev.get("hardware_readback") is None:
        ok("evidence.hardware_readback is null / not_measured", "null")
    else:
        fail("evidence.hardware_readback is null / not_measured",
             "got %r" % (ev.get("hardware_readback"),))
    if ev.get("scope") == "m2a-native-roundtrip/1":
        ok("evidence.scope is the narrow versioned artifact", ev.get("scope"))
    else:
        fail("evidence.scope is the narrow versioned artifact",
             "got %r" % (ev.get("scope"),))
    if ev.get("yields_range") is True:
        fail("evidence does not claim a range", "yields_range=true")
    else:
        ok("evidence does not claim a range", "yields_range absent/false")

    if level == "native_roundtrip_verified":
        if ev.get("kind") == "measured" and measured:
            ok("native_roundtrip_verified is backed by a real byte-exact run",
               "kind=measured")
        else:
            fail("native_roundtrip_verified is backed by a real byte-exact run",
                 "kind=%r measured=%s" % (ev.get("kind"), measured))
    else:
        if ev.get("kind") == "not_measured" and not measured:
            ok("a non-native level does not claim a measurement", "kind=not_measured")
        else:
            fail("a non-native level does not claim a measurement",
                 "level=%r kind=%r measured=%s" % (level, ev.get("kind"), measured))


def check_decode_v2(out, mac, psdu, measured):
    dec = out.get("decode")
    if not isinstance(dec, dict):
        fail("decode block is present", "got %r" % (dec,))
        return
    if dec.get("status") not in DEMOD_STATUSES:
        fail("decode.status is a known demod status", "got %r" % (dec.get("status"),))
    else:
        ok("decode.status is a known demod status", dec.get("status"))
    # A success status and an FCS pass must agree; a "success" with fcs_pass
    # false (or vice versa) is an inconsistent manifest, not a failure.
    if dec.get("status") == "success" and dec.get("fcs_pass") is not True:
        fail("decode.status == success implies fcs_pass", "status=success fcs_pass=%r"
             % (dec.get("fcs_pass"),))
    elif dec.get("fcs_pass") is True and dec.get("status") != "success":
        fail("decode.fcs_pass implies status == success",
             "fcs_pass=true status=%r" % (dec.get("status"),))
    else:
        ok("decode.status and decode.fcs_pass agree",
           "status=%r fcs_pass=%r" % (dec.get("status"), dec.get("fcs_pass")))
    if dec.get("phr_psdu_length") == len(psdu):
        ok("decode.phr_psdu_length equals the PSDU length", str(len(psdu)))
    else:
        fail("decode.phr_psdu_length equals the PSDU length",
             "phr_psdu_length=%r want %d" % (dec.get("phr_psdu_length"), len(psdu)))
    try:
        payload = bytes.fromhex(dec.get("payload_hex") or "")
    except (TypeError, ValueError):
        payload = None
        fail("decode.payload_hex is valid hex", "got %r" % (dec.get("payload_hex"),))
    if measured:
        if dec.get("status") == "success":
            ok("decode.status is success", "success")
        else:
            fail("decode.status is success", "got %r" % (dec.get("status"),))
        if dec.get("fcs_pass") is True:
            ok("decode.fcs_pass is true", "true")
        else:
            fail("decode.fcs_pass is true", "got %r" % (dec.get("fcs_pass"),))
        if payload == psdu:
            ok("decode.payload_hex equals MAC + independently computed FCS",
               "%d B byte-exact" % len(payload))
        else:
            fail("decode.payload_hex equals MAC + independently computed FCS",
                 "payload=%r want=%r" % (
                     None if payload is None else payload.hex(), psdu.hex()))
    else:
        hidden = (dec.get("status") == "success" and dec.get("fcs_pass") is True
                  and payload == psdu)
        if hidden:
            fail("a failed run does not secretly decode byte-exactly",
                 "status.success + fcs_pass + payload match, but ok=false")
        else:
            ok("a failed run does not secretly decode byte-exactly",
               "status=%r fcs_pass=%r" % (dec.get("status"), dec.get("fcs_pass")))

    diag = dec.get("diagnostic")
    work_rx = (out.get("samples") or {}).get("work_rx")
    if not (isinstance(diag, dict) and is_int(diag.get("sfd_start_sample"))
            and is_int(diag.get("packet_start_sample"))):
        fail("decode.diagnostic carries two integer sample coordinates",
             "got %r" % (diag,))
        return
    sfd = diag["sfd_start_sample"]
    packet = diag["packet_start_sample"]
    if not isinstance(work_rx, int):
        fail("decode.diagnostic is rebased into [0, work_rx)",
             "samples.work_rx is %r" % (work_rx,))
        return
    if measured:
        if 0 <= sfd < work_rx and 0 <= packet < work_rx:
            ok("decode.diagnostic is rebased into [0, work_rx)",
               "sfd=%d packet=%d work_rx=%d" % (sfd, packet, work_rx))
        else:
            fail("decode.diagnostic is rebased into [0, work_rx)",
                 "sfd=%d packet=%d work_rx=%d" % (sfd, packet, work_rx))
    else:
        if sfd == -1 and packet == -1:
            ok("a failed decode clears the diagnostic coordinates",
               "sfd=-1 packet=-1")
        else:
            fail("a failed decode clears the diagnostic coordinates",
                 "sfd=%d packet=%d want -1/-1" % (sfd, packet))


def check_compare_v2(out, measured):
    comp = out.get("compare")
    if not isinstance(comp, dict):
        fail("compare block is present", "got %r" % (comp,))
        return
    if measured:
        if comp.get("bytes_exact") is True:
            ok("compare.bytes_exact is true", "true")
        else:
            fail("compare.bytes_exact is true", "got %r" % (comp.get("bytes_exact"),))
        mae, rl2 = comp.get("max_abs_error"), comp.get("relative_l2")
        if is_num(mae) and 0.0 <= mae <= MAX_ABS_ERROR:
            ok("compare.max_abs_error is within the frozen tolerance", str(mae))
        else:
            fail("compare.max_abs_error is within the frozen tolerance",
                 "got %r want <= %g" % (mae, MAX_ABS_ERROR))
        if is_num(rl2) and 0.0 <= rl2 <= RELATIVE_L2:
            ok("compare.relative_l2 is within the frozen tolerance", str(rl2))
        else:
            fail("compare.relative_l2 is within the frozen tolerance",
                 "got %r want <= %g" % (rl2, RELATIVE_L2))
    else:
        if comp.get("bytes_exact") is False:
            ok("a failed run reports bytes_exact=false", "false")
        else:
            fail("a failed run reports bytes_exact=false",
                 "got %r" % (comp.get("bytes_exact"),))
        for key in ("max_abs_error", "relative_l2"):
            if is_num(comp.get(key)) and comp.get(key) >= 0.0:
                ok("compare.%s is a non-negative number" % key, str(comp.get(key)))
            else:
                fail("compare.%s is a non-negative number" % key,
                     "got %r" % (comp.get(key),))
    samples = out.get("samples") or {}
    dec = out.get("decode") or {}
    try:
        returned = len(bytes.fromhex(dec.get("payload_hex") or ""))
    except (TypeError, ValueError):
        returned = None
    if returned is not None and samples.get("returned") == returned:
        ok("samples.returned equals the decoded payload length", str(returned))
    else:
        fail("samples.returned equals the decoded payload length",
             "returned=%r want %r" % (samples.get("returned"), returned))


# ---------------------------------------------------------------------------
# /2: stage coordinates
# ---------------------------------------------------------------------------


def check_stages_v2(out, dims, want_work, want_native, impaired):
    if dims is None:
        fail("stages[] carries every frozen coordinate", "rate block unusable")
        return
    tx_l, tx_m, tx_taps, rx_taps = dims
    stages = out.get("stages")
    if not isinstance(stages, list) or len(stages) != 4:
        fail("stages[] carries exactly the four frozen stages",
             "got %r" % (len(stages) if isinstance(stages, list) else stages,))
        return
    names = [s.get("name") for s in stages]
    if names == ["hrp_mod", "tx_resample", "rx_resample", "demod"]:
        ok("stages[] names and order are the frozen four", str(names))
    else:
        fail("stages[] names and order are the frozen four", str(names))
        return

    rate = out["rate"]
    samples = out["samples"]
    native_hz = rate.get("native_hz")

    # Every stage states its own INPUT and OUTPUT unit (G0 A.2).  A stage can be
    # MIXED -- the HRP modulator consumes PSDU BYTES and produces SAMPLES, the
    # demod consumes samples and produces PSDU BYTES -- so a single label is not
    # enough (review E finding 1).
    want_units = {"hrp_mod": ("bytes", "samples"),
                  "tx_resample": ("samples", "samples"),
                  "rx_resample": ("samples", "samples"),
                  "demod": ("samples", "bytes")}
    for stage in stages:
        name = stage.get("name")
        iu = stage.get("in_unit")
        ou = stage.get("out_unit")
        exp = want_units.get(name)
        if exp is not None and (iu, ou) == exp:
            ok("stage %s states in/out units %s/%s" % (name, iu, ou),
               "%s/%s" % (iu, ou))
        elif exp is not None:
            fail("stage %s states in/out units %s/%s" % (name, exp[0], exp[1]),
                 "got %r/%r" % (iu, ou))
        if stage.get("unit") == ou:
            ok("stage %s unit aliases out_unit" % name, str(ou))
        else:
            fail("stage %s unit aliases out_unit" % name,
                 "unit=%r out_unit=%r" % (stage.get("unit"), ou))
        if stage.get("origin") == 0:
            ok("stage %s records 0-based origin 0" % name, "0")
        else:
            fail("stage %s records 0-based origin 0" % name,
                 "got %r" % (stage.get("origin"),))
        if stage.get("trim") == 0 and stage.get("phase") == 0:
            ok("stage %s is causal (trim=0, phase=0)" % name, "trim=0 phase=0")
        else:
            fail("stage %s is causal (trim=0, phase=0)" % name,
                 "trim=%r phase=%r" % (stage.get("trim"), stage.get("phase")))
        pf, pb, pad = stage.get("pad_front"), stage.get("pad_back"), stage.get("padding")
        if is_int(pf) and is_int(pb) and is_int(pad) and pf >= 0 and pb >= 0 \
                and pf + pb == pad:
            ok("stage %s splits padding into pad_front + pad_back" % name,
               "%d + %d == %d" % (pf, pb, pad))
        else:
            fail("stage %s splits padding into pad_front + pad_back" % name,
                 "pad_front=%r pad_back=%r padding=%r" % (pf, pb, pad))

    checks = [
        ("hrp_mod", 1, 1, WORK_HZ, out["frame"]["psdu_bytes"], want_work, 0.0),
        ("tx_resample", tx_l, tx_m, native_hz, want_work, want_native,
         0.5 * (tx_taps - 1)),
        ("rx_resample", RX_L, tx_l, WORK_HZ, impaired, samples.get("work_rx"),
         0.5 * (rx_taps - 1)),
        ("demod", 1, 1, WORK_HZ, samples.get("work_rx"), None, 0.0),
    ]
    for name, l, m, hz, in_count, out_count, delay in checks:
        stage = stage_by_name(out, name)
        if stage is None:
            fail("stage %s is present" % name, "missing")
            continue
        if stage.get("l") == l and stage.get("m") == m:
            ok("stage %s records L/M" % name, "%d/%d" % (l, m))
        else:
            fail("stage %s records L/M" % name,
                 "got %r/%r want %d/%d" % (stage.get("l"), stage.get("m"), l, m))
        if stage.get("rate_hz") == hz:
            ok("stage %s records its rate" % name, str(hz))
        else:
            fail("stage %s records its rate" % name,
                 "got %r want %r" % (stage.get("rate_hz"), hz))
        if in_count is not None and stage.get("in_count") != in_count:
            fail("stage %s input count is the previous stage's output" % name,
                 "in_count=%r want %r" % (stage.get("in_count"), in_count))
        elif in_count is not None:
            ok("stage %s input count is the previous stage's output" % name,
               str(in_count))
        if out_count is not None and stage.get("out_count") != out_count:
            fail("stage %s output count matches the independent length" % name,
                 "out_count=%r want %r" % (stage.get("out_count"), out_count))
        elif out_count is not None:
            ok("stage %s output count matches the independent length" % name,
               str(out_count))
        if stage.get("filter_delay") == delay:
            ok("stage %s records the group delay 0.5*(T-1)" % name, str(delay))
        else:
            fail("stage %s records the group delay 0.5*(T-1)" % name,
                 "got %r want %r" % (stage.get("filter_delay"), delay))
        vf, vt = stage.get("valid_from"), stage.get("valid_to")
        if is_int(vf) and is_int(vt) and 0 <= vf < vt:
            ok("stage %s has a non-empty valid range" % name, "[%d,%d)" % (vf, vt))
        else:
            fail("stage %s has a non-empty valid range" % name,
                 "valid_from=%r valid_to=%r" % (vf, vt))

    # The resample stages must split their tap span exactly.
    for name, taps in (("tx_resample", tx_taps), ("rx_resample", rx_taps)):
        stage = stage_by_name(out, name)
        if stage is None:
            continue
        span = taps - 1
        want_pf = span // 2
        want_pb = span - want_pf
        if stage.get("pad_front") == want_pf and stage.get("pad_back") == want_pb:
            ok("stage %s pad split equals the tap span" % name,
               "%d/%d of %d" % (want_pf, want_pb, span))
        else:
            fail("stage %s pad split equals the tap span" % name,
                 "got %r/%r want %d/%d" % (stage.get("pad_front"),
                                           stage.get("pad_back"), want_pf, want_pb))
        if stage.get("search_guard_front") == 0 and \
                stage.get("search_guard_back") == 0 and stage.get("search_roi") == [0, 0]:
            ok("stage %s adds no search guards/ROI" % name, "guard=0/0 roi=[0,0]")
        else:
            fail("stage %s adds no search guards/ROI" % name,
                 "guard=%r/%r roi=%r" % (stage.get("search_guard_front"),
                                         stage.get("search_guard_back"),
                                         stage.get("search_roi")))
        if stage.get("valid_from") == 0 and stage.get("valid_to") == stage.get("out_count"):
            ok("stage %s valid range is [0, out_count)" % name, "ok")
        else:
            fail("stage %s valid range is [0, out_count)" % name,
                 "valid=[%r,%r) out=%r" % (stage.get("valid_from"),
                                           stage.get("valid_to"),
                                           stage.get("out_count")))

    # hrp_mod adds no padding/guards.
    hrp = stage_by_name(out, "hrp_mod")
    if hrp is not None:
        if hrp.get("padding") == 0 and hrp.get("search_guard_front") == 0 and \
                hrp.get("search_guard_back") == 0 and hrp.get("search_roi") == [0, 0]:
            ok("stage hrp_mod adds no padding or search guards", "0")
        else:
            fail("stage hrp_mod adds no padding or search guards",
                 "padding=%r guard=%r/%r roi=%r" % (
                     hrp.get("padding"), hrp.get("search_guard_front"),
                     hrp.get("search_guard_back"), hrp.get("search_roi")))

    # demod: the search guards are the padding and the ROI is recomputable.
    demod = stage_by_name(out, "demod")
    if demod is None:
        return
    pf = demod.get("pad_front")
    pb = demod.get("pad_back")
    in_count = demod.get("in_count")
    if not (is_int(pf) and is_int(pb) and is_int(in_count)):
        fail("stage demod coordinates are integers",
             "pad_front=%r pad_back=%r in_count=%r" % (pf, pb, in_count))
        return
    if demod.get("search_guard_front") == pf and demod.get("search_guard_back") == pb:
        ok("stage demod search guards equal its padding", "%d/%d" % (pf, pb))
    else:
        fail("stage demod search guards equal its padding",
             "guard=%r/%r pad=%r/%r" % (demod.get("search_guard_front"),
                                        demod.get("search_guard_back"), pf, pb))
    buf_n = in_count + pf + pb
    seed = buf_n // 2
    want_lo = seed - TIMING_SEARCH_MARGIN if seed > TIMING_SEARCH_MARGIN else 0
    want_hi = min(buf_n, seed + TIMING_SEARCH_MARGIN)
    roi = demod.get("search_roi")
    if roi == [want_lo, want_hi]:
        ok("stage demod search ROI is the recomputed [seed-margin, seed+margin)",
           "[%d,%d) in a %d-sample buffer" % (want_lo, want_hi, buf_n))
    else:
        fail("stage demod search ROI is the recomputed [seed-margin, seed+margin)",
             "got %r want [%d,%d)" % (roi, want_lo, want_hi))
    if is_int(demod.get("valid_from")) and demod.get("valid_from") == want_lo and \
            demod.get("valid_to") == want_hi:
        ok("stage demod valid range equals the search ROI", "[%d,%d)" % (want_lo, want_hi))
    else:
        fail("stage demod valid range equals the search ROI",
             "valid=[%r,%r) want [%d,%d)" % (demod.get("valid_from"),
                                             demod.get("valid_to"), want_lo, want_hi))
    if in_count == samples.get("work_rx"):
        ok("stage demod input is the RX work buffer", str(in_count))
    else:
        fail("stage demod input is the RX work buffer",
             "in_count=%r work_rx=%r" % (in_count, samples.get("work_rx")))


# ---------------------------------------------------------------------------
# /2: impairment / context / kernel / observation
# ---------------------------------------------------------------------------


def _taps_from_json(mp):
    if not isinstance(mp, list):
        return None
    out = []
    for item in mp:
        if not isinstance(item, dict):
            return None
        re_, im_ = item.get("re"), item.get("im")
        if not (is_num(re_) and is_num(im_)):
            return None
        out.append((re_, im_))
    return out


def impairment_is_identity(imp: dict):
    if imp.get("enabled") is not True:
        return True
    taps = _taps_from_json(imp.get("multipath"))
    return (imp.get("cfo_hz") == 0 and imp.get("awgn_enabled") is not True
            and imp.get("delay_int_samples") == 0
            and imp.get("delay_frac_num") == 0 and (taps == []))


def check_impairment_v2(out):
    imp = out.get("impairment")
    if not isinstance(imp, dict):
        fail("impairment block is present", "got %r" % (imp,))
        return
    cfg = out.get("config") or {}
    req = cfg.get("requested") or {}
    scenario = req.get("scenario")

    if imp.get("enabled") == (scenario != "clean"):
        ok("impairment.enabled matches the requested scenario",
           "enabled=%r scenario=%r" % (imp.get("enabled"), scenario))
    else:
        fail("impairment.enabled matches the requested scenario",
             "enabled=%r scenario=%r" % (imp.get("enabled"), scenario))

    if not is_num(imp.get("cfo_hz")):
        fail("impairment.cfo_hz is a number", "got %r" % (imp.get("cfo_hz"),))
    else:
        ok("impairment.cfo_hz is a number", str(imp.get("cfo_hz")))
    awgn_on = imp.get("awgn_enabled") is True
    if (imp.get("awgn_snr_db") is None) == (not awgn_on):
        ok("impairment.awgn_snr_db is present iff AWGN is enabled",
           "awgn_enabled=%r snr=%r" % (awgn_on, imp.get("awgn_snr_db")))
    else:
        fail("impairment.awgn_snr_db is present iff AWGN is enabled",
             "awgn_enabled=%r snr=%r" % (awgn_on, imp.get("awgn_snr_db")))
    if (imp.get("awgn_seed") is None) == (not awgn_on):
        ok("impairment.awgn_seed is present iff AWGN is enabled",
           "awgn_enabled=%r seed=%r" % (awgn_on, imp.get("awgn_seed")))
    else:
        fail("impairment.awgn_seed is present iff AWGN is enabled",
             "awgn_enabled=%r seed=%r" % (awgn_on, imp.get("awgn_seed")))
    if awgn_on and not is_num(imp.get("awgn_snr_db")):
        fail("impairment.awgn_snr_db is a number when enabled",
             "got %r" % (imp.get("awgn_snr_db"),))
    if awgn_on and not is_int(imp.get("awgn_seed")):
        fail("impairment.awgn_seed is an integer when enabled",
             "got %r" % (imp.get("awgn_seed"),))

    dint = imp.get("delay_int_samples")
    dnum = imp.get("delay_frac_num")
    dden = imp.get("delay_frac_den")
    if is_int(dint) and dint >= 0:
        ok("impairment.delay_int_samples is a non-negative integer", str(dint))
    else:
        fail("impairment.delay_int_samples is a non-negative integer",
             "got %r" % (dint,))
    if is_int(dden) and dden > 0 and is_int(dnum) and 0 <= dnum < dden:
        ok("impairment.delay_frac_num/den is a proper fraction", "%d/%d" % (dnum, dden))
    else:
        fail("impairment.delay_frac_num/den is a proper fraction",
             "num=%r den=%r" % (dnum, dden))

    taps = _taps_from_json(imp.get("multipath"))
    if taps is None:
        fail("impairment.multipath is a list of complex taps",
             "got %r" % (imp.get("multipath"),))
    elif not taps:
        ok("impairment.multipath is empty or well formed", "[]")
    elif taps[0] == (0, 0):
        fail("impairment.multipath[0] (the first path) is non-zero", "0+0j")
    else:
        ok("impairment.multipath[0] (the first path) is non-zero", str(taps[0]))

    if imp.get("snr_definition") == "valid-region mean power":
        ok("impairment.snr_definition is the frozen definition",
           imp.get("snr_definition"))
    else:
        fail("impairment.snr_definition is the frozen definition",
             "got %r" % (imp.get("snr_definition"),))

    # Cross-check the echoed parameters against config.requested.
    req_taps = _taps_from_json(req.get("multipath"))
    pairs = [
        ("cfo_hz", imp.get("cfo_hz"), req.get("cfo_hz")),
        ("awgn_snr_db", imp.get("awgn_snr_db"), req.get("awgn_snr_db")),
        ("awgn_seed", imp.get("awgn_seed"), req.get("awgn_seed")),
        ("delay_int_samples", dint, req.get("delay_int")),
        ("delay_frac_num", dnum, req.get("delay_frac_num")),
        ("delay_frac_den", dden, req.get("delay_frac_den")),
        ("multipath", taps, req_taps),
    ]
    bad = ["%s=%r want %r" % (k, a, b) for k, a, b in pairs if a != b]
    if bad:
        fail("the impairment block echoes config.requested", "; ".join(bad))
    else:
        ok("the impairment block echoes config.requested",
           "%d parameters cross-checked" % len(pairs))

    samples = out.get("samples") or {}
    if awgn_on:
        region = imp.get("valid_region")
        region_ok = (isinstance(region, list) and len(region) == 2
                     and is_int(region[0]) and is_int(region[1]))
        buf = imp.get("buffer_samples")
        if not region_ok:
            fail("impairment.valid_region is a [lo, hi] pair", "got %r" % (region,))
            return
        lo, hi = region
        if is_int(buf) and 0 <= lo < hi <= buf:
            ok("impairment.valid_region is inside the native buffer",
               "[%d,%d) of %d" % (lo, hi, buf))
        else:
            fail("impairment.valid_region is inside the native buffer",
                 "[%r,%r) of %r" % (lo, hi, buf))
        if imp.get("region_samples") == hi - lo:
            ok("impairment.region_samples equals hi-lo", str(hi - lo))
        else:
            fail("impairment.region_samples equals hi-lo",
                 "got %r want %d" % (imp.get("region_samples"), hi - lo))
        if buf == samples.get("native"):
            ok("impairment.buffer_samples is the native buffer length",
               str(samples.get("native")))
        else:
            fail("impairment.buffer_samples is the native buffer length",
                 "buffer=%r native=%r" % (buf, samples.get("native")))
        pv = imp.get("signal_power_valid")
        pf = imp.get("signal_power_full")
        sigma2 = imp.get("noise_sigma2_per_dim")
        snr = imp.get("awgn_snr_db")
        if is_num(pv) and pv > 0 and is_num(pf) and pf >= 0 and is_num(sigma2) \
                and sigma2 > 0 and is_num(snr):
            calc = 10.0 * math.log10(pv / (2.0 * sigma2))
            if abs(calc - snr) <= 0.05:
                ok("the AWGN SNR is the valid-region power / per-dim variance",
                   "recomputed=%.4f dB declared=%.4f dB" % (calc, snr))
            else:
                fail("the AWGN SNR is the valid-region power / per-dim variance",
                     "recomputed=%.4f dB declared=%.4f dB" % (calc, snr))
            if pf <= pv:
                ok("the region power is not diluted by padding", "p_full<=p_valid")
            else:
                fail("the region power is not diluted by padding",
                     "p_full=%r > p_valid=%r" % (pf, pv))
            if hi - lo < buf:
                if pf < pv:
                    ok("zero padding would have diluted the SNR but did not",
                       "p_full < p_valid with %d padding samples" % (buf - (hi - lo)))
                else:
                    fail("zero padding would have diluted the SNR but did not",
                         "region < buffer but p_full == p_valid")
            else:
                ok("the valid region is the whole buffer (no padding to dilute)",
                   "region == buffer == %d" % buf)
        else:
            fail("the AWGN region power/variance fields are positive numbers",
                 "p_valid=%r p_full=%r sigma2=%r snr=%r" % (pv, pf, sigma2, snr))
    else:
        nulls = all(imp.get(k) is None for k in (
            "valid_region", "region_samples", "signal_power_valid",
            "signal_power_full", "noise_sigma2_per_dim"))
        if nulls:
            ok("a non-AWGN impairment reports no region power fields", "null")
        else:
            fail("a non-AWGN impairment reports no region power fields",
                 "got %r" % {k: imp.get(k) for k in (
                     "valid_region", "region_samples", "signal_power_valid",
                     "signal_power_full", "noise_sigma2_per_dim")})


def check_context_v2(out):
    ctx = out.get("context")
    if not isinstance(ctx, dict):
        fail("context block is present", "got %r" % (ctx,))
        return
    if ctx.get("prepared") is True:
        ok("context.prepared is true", "true")
    else:
        fail("context.prepared is true", "got %r" % (ctx.get("prepared"),))
    cap = ctx.get("capacity_samples")
    samples = out.get("samples") or {}
    demod = stage_by_name(out, "demod") or {}
    observed = [samples.get("work_tx"), samples.get("native"), samples.get("work_rx")]
    if is_int(demod.get("in_count")) and is_int(demod.get("padding")):
        observed.append(demod["in_count"] + demod["padding"])
    observed = [v for v in observed if is_int(v)]
    if is_int(cap) and cap > 0 and all(cap >= v for v in observed):
        ok("context.capacity_samples covers every observed buffer length",
           "capacity=%d observed_max=%d" % (cap, max(observed) if observed else 0))
    else:
        fail("context.capacity_samples covers every observed buffer length",
             "capacity=%r observed=%r" % (cap, observed))
    imp = out.get("impairment") or {}
    identity = impairment_is_identity(imp)
    want_path = "prepared_context" if identity else "stage_runner_impairment"
    if ctx.get("path") in CONTEXT_PATHS and ctx.get("path") == want_path:
        ok("context.path matches whether the impairment is an identity",
           ctx.get("path"))
    else:
        fail("context.path matches whether the impairment is an identity",
             "path=%r want %r (identity=%s)" % (ctx.get("path"), want_path, identity))


def check_kernel_v2(out):
    kern = out.get("kernel")
    if not isinstance(kern, dict):
        fail("kernel block is present", "got %r" % (kern,))
        return
    for key, allowed in (("tx", KERNEL_RESAMPLER), ("rx", KERNEL_RESAMPLER),
                         ("demod", KERNEL_DEMOD)):
        value = kern.get(key)
        if value in allowed:
            ok("kernel.%s is a real kernel name" % key, str(value))
        else:
            fail("kernel.%s is a real kernel name" % key,
                 "got %r not in %s" % (value, sorted(allowed)))


def check_observation(out):
    obs = out.get("observation")
    if obs is None:
        return
    if not isinstance(obs, dict):
        fail("observation block is an object when present", "got %r" % (obs,))
        return
    status = out.get("status") or {}
    iters = obs.get("iters")
    if is_int(iters) and iters > 0 and iters == status.get("attempted"):
        ok("observation.iters equals status.attempted", str(iters))
    else:
        fail("observation.iters equals status.attempted",
             "iters=%r attempted=%r" % (iters, status.get("attempted")))
    vals = [obs.get(k) for k in ("p50_us", "p95_us", "p99_us", "max_us", "cold_us")]
    if all(is_num(v) and v >= 0 for v in vals):
        if vals[0] <= vals[1] <= vals[2] <= vals[3]:
            ok("observation percentiles are monotonic",
               "p50<=p95<=p99<=max")
        else:
            fail("observation percentiles are monotonic",
                 "p50=%r p95=%r p99=%r max=%r" % tuple(vals[:4]))
    else:
        fail("observation timing fields are non-negative numbers",
             "got %r" % (vals,))
    if is_int(obs.get("rss_kb")) and obs.get("rss_kb") >= 0:
        ok("observation.rss_kb is a non-negative integer", str(obs.get("rss_kb")))
    else:
        fail("observation.rss_kb is a non-negative integer",
             "got %r" % (obs.get("rss_kb"),))
    alloc = obs.get("alloc")
    if alloc is None:
        ok("observation.alloc is null (not instrumented here)", "null")
    elif isinstance(alloc, dict) and all(is_int(v) and v >= 0 for v in alloc.values()):
        ok("observation.alloc counters are non-negative integers", "ok")
    else:
        fail("observation.alloc counters are non-negative integers",
             "got %r" % (alloc,))


# ---------------------------------------------------------------------------
# /1 (legacy) checks
# ---------------------------------------------------------------------------


def check_status_v1(out):
    status = out.get("status")
    if isinstance(status, dict) and status.get("ok") is True:
        ok("status.ok is true", "true")
    else:
        fail("status.ok is true", "got %r" % (status,))
    if isinstance(status, dict) and status.get("exit_code") == 0:
        ok("status.exit_code is 0", "0")
    else:
        fail("status.exit_code is 0",
             "got %r" % (status.get("exit_code") if isinstance(status, dict) else status))


def check_stages_v1(out, dims, want_work, want_native, impaired):
    if dims is None:
        fail("stages[] carries every frozen coordinate", "rate block unusable")
        return
    tx_l, tx_m, tx_taps, rx_taps = dims
    stages = out.get("stages")
    if not isinstance(stages, list) or len(stages) != 4:
        fail("stages[] carries exactly the four frozen stages",
             "got %r" % (len(stages) if isinstance(stages, list) else stages,))
        return
    names = [s.get("name") for s in stages]
    if names == ["hrp_mod", "tx_resample", "rx_resample", "demod"]:
        ok("stages[] names and order are the frozen four", str(names))
    else:
        fail("stages[] names and order are the frozen four", str(names))
    rate = out["rate"]
    samples = out["samples"]
    native_hz = rate.get("native_hz")
    checks = [
        ("hrp_mod", 1, 1, WORK_HZ, out["frame"]["psdu_bytes"], want_work, 0.0),
        ("tx_resample", tx_l, tx_m, native_hz, want_work, want_native,
         0.5 * (tx_taps - 1)),
        ("rx_resample", RX_L, tx_l, WORK_HZ, impaired, samples.get("work_rx"),
         0.5 * (rx_taps - 1)),
        ("demod", 1, 1, WORK_HZ, None, None, 0.0),
    ]
    for name, l, m, hz, in_count, out_count, delay in checks:
        stage = stage_by_name(out, name)
        if stage is None:
            fail("stage %s is present" % name, "missing")
            continue
        if stage.get("l") == l and stage.get("m") == m:
            ok("stage %s records L/M" % name, "%d/%d" % (l, m))
        else:
            fail("stage %s records L/M" % name,
                 "got %r/%r want %d/%d" % (stage.get("l"), stage.get("m"), l, m))
        if stage.get("rate_hz") == hz:
            ok("stage %s records its rate" % name, str(hz))
        else:
            fail("stage %s records its rate" % name,
                 "got %r want %r" % (stage.get("rate_hz"), hz))
        if in_count is not None and stage.get("in_count") != in_count:
            fail("stage %s input count is the previous stage's output" % name,
                 "in_count=%r want %d" % (stage.get("in_count"), in_count))
        elif in_count is not None:
            ok("stage %s input count is the previous stage's output" % name,
               str(in_count))
        if out_count is not None and stage.get("out_count") != out_count:
            fail("stage %s output count matches the independent length" % name,
                 "out_count=%r want %r" % (stage.get("out_count"), out_count))
        elif out_count is not None:
            ok("stage %s output count matches the independent length" % name,
               str(out_count))
        if stage.get("filter_delay") == delay:
            ok("stage %s records the group delay 0.5*(T-1)" % name, str(delay))
        else:
            fail("stage %s records the group delay 0.5*(T-1)" % name,
                 "got %r want %r" % (stage.get("filter_delay"), delay))
        if stage.get("trim") == 0 and stage.get("phase") == 0:
            ok("stage %s is causal (trim=0, phase=0)" % name, "trim=0 phase=0")
        else:
            fail("stage %s is causal (trim=0, phase=0)" % name,
                 "trim=%r phase=%r" % (stage.get("trim"), stage.get("phase")))
        vf, vt = stage.get("valid_from"), stage.get("valid_to")
        if is_int(vf) and is_int(vt) and 0 <= vf < vt:
            ok("stage %s has a non-empty valid range" % name, "[%d,%d)" % (vf, vt))
        else:
            fail("stage %s has a non-empty valid range" % name,
                 "valid_from=%r valid_to=%r" % (vf, vt))


def check_decode_v1(out, mac, psdu):
    dec = out.get("decode")
    if not isinstance(dec, dict):
        fail("decode block is present", "got %r" % (dec,))
        return
    if dec.get("status") == "success":
        ok("decode.status is success", "success")
    else:
        fail("decode.status is success", "got %r" % (dec.get("status"),))
    if dec.get("fcs_pass") is True:
        ok("decode.fcs_pass is true", "true")
    else:
        fail("decode.fcs_pass is true", "got %r" % (dec.get("fcs_pass"),))
    if dec.get("phr_psdu_length") == len(psdu):
        ok("decode.phr_psdu_length equals the PSDU length", str(len(psdu)))
    else:
        fail("decode.phr_psdu_length equals the PSDU length",
             "phr_psdu_length=%r want %d" % (dec.get("phr_psdu_length"), len(psdu)))
    payload_hex = dec.get("payload_hex")
    try:
        payload = bytes.fromhex(payload_hex)
    except (TypeError, ValueError):
        fail("decode.payload_hex is valid hex", "got %r" % (payload_hex,))
        payload = None
    if payload is not None:
        if payload == psdu:
            ok("decode.payload_hex equals MAC + independently computed FCS",
               "%d B byte-exact" % len(payload))
        else:
            fail("decode.payload_hex equals MAC + independently computed FCS",
                 "payload=%s want=%s" % (payload.hex(), psdu.hex()))
    diag = dec.get("diagnostic")
    if isinstance(diag, dict) and is_int(diag.get("sfd_start_sample")) and \
            is_int(diag.get("packet_start_sample")):
        ok("decode.diagnostic carries the two integer sample coordinates",
           "sfd=%s packet=%s" % (diag["sfd_start_sample"],
                                 diag["packet_start_sample"]))
    else:
        fail("decode.diagnostic carries the two integer sample coordinates",
             "got %r" % (diag,))


def check_compare_v1(out):
    comp = out.get("compare")
    if not isinstance(comp, dict):
        fail("compare block is present", "got %r" % (comp,))
        return
    if comp.get("bytes_exact") is True:
        ok("compare.bytes_exact is true", "true")
    else:
        fail("compare.bytes_exact is true", "got %r" % (comp.get("bytes_exact"),))
    mae, rl2 = comp.get("max_abs_error"), comp.get("relative_l2")
    if is_num(mae) and 0.0 <= mae <= MAX_ABS_ERROR:
        ok("compare.max_abs_error is within the frozen tolerance", str(mae))
    else:
        fail("compare.max_abs_error is within the frozen tolerance",
             "got %r want <= %g" % (mae, MAX_ABS_ERROR))
    if is_num(rl2) and 0.0 <= rl2 <= RELATIVE_L2:
        ok("compare.relative_l2 is within the frozen tolerance", str(rl2))
    else:
        fail("compare.relative_l2 is within the frozen tolerance",
             "got %r want <= %g" % (rl2, RELATIVE_L2))
    samples = out.get("samples") or {}
    dec = out.get("decode") or {}
    try:
        returned = len(bytes.fromhex(dec.get("payload_hex") or ""))
    except ValueError:
        returned = None
    if returned is not None and samples.get("returned") == returned:
        ok("samples.returned equals the decoded PSDU length", str(returned))
    else:
        fail("samples.returned equals the decoded PSDU length",
             "returned=%r want %r" % (samples.get("returned"), returned))


def check_evidence_v1(out):
    ev = out.get("evidence")
    if not isinstance(ev, dict):
        fail("evidence block is present", "got %r" % (ev,))
        return
    level = ev.get("level")
    if level in EVIDENCE_ABOVE:
        fail("evidence.level does not exceed native_roundtrip_verified",
             "level=%r is above the M2-A ceiling" % (level,))
    elif level in EVIDENCE_ALLOWED:
        ok("evidence.level is within the M2-A ceiling", str(level))
    else:
        fail("evidence.level is within the M2-A ceiling",
             "level=%r not in %s" % (level, sorted(EVIDENCE_ALLOWED)))
    if ev.get("measurement_valid") is False:
        ok("evidence.measurement_valid is false", "false")
    else:
        fail("evidence.measurement_valid is false",
             "got %r" % (ev.get("measurement_valid"),))
    if ev.get("hardware_readback") is None:
        ok("evidence.hardware_readback is null / not_measured", "null")
    else:
        fail("evidence.hardware_readback is null / not_measured",
             "got %r" % (ev.get("hardware_readback"),))
    if ev.get("scope") == "m2a-native-roundtrip/1":
        ok("evidence.scope is the narrow versioned artifact", ev.get("scope"))
    else:
        fail("evidence.scope is the narrow versioned artifact",
             "got %r" % (ev.get("scope"),))
    if ev.get("yields_range") is True:
        fail("evidence does not claim a range", "yields_range=true")
    else:
        ok("evidence does not claim a range", "yields_range absent/false")
    comp = out.get("compare") or {}
    dec = out.get("decode") or {}
    status = out.get("status") or {}
    success = (status.get("ok") is True and dec.get("status") == "success"
               and comp.get("bytes_exact") is True and dec.get("fcs_pass") is True)
    if level == "native_roundtrip_verified":
        if ev.get("kind") == "measured" and success:
            ok("native_roundtrip_verified is backed by a real byte-exact run",
               "kind=measured status.ok=true")
        else:
            fail("native_roundtrip_verified is backed by a real byte-exact run",
                 "kind=%r success=%s" % (ev.get("kind"), success))
    elif ev.get("kind") == "not_measured" and not success:
        ok("a not_measured run does not claim native evidence", "kind=not_measured")
    elif ev.get("kind") == "not_measured":
        fail("a not_measured run does not claim native evidence",
             "kind=not_measured but the run succeeded and level=%r" % (level,))


# ---------------------------------------------------------------------------
# Suites
# ---------------------------------------------------------------------------


def run_checks_v1(out):
    check_status_v1(out)
    mac, psdu = check_frame_and_fcs(out)
    if mac is None or psdu is None:
        return [r for r in results if r[1] == "FAIL"]
    dims, want_work, want_native, impaired = check_lengths(out, mac, psdu)
    check_stages_v1(out, dims, want_work, want_native, impaired)
    check_decode_v1(out, mac, psdu)
    check_compare_v1(out)
    check_filter(out)
    check_evidence_v1(out)
    check_provenance(out, mac, psdu)
    check_config(out)
    return [r for r in results if r[1] == "FAIL"]


def run_checks_v2(out):
    mac, psdu = check_frame_and_fcs(out)
    if mac is None or psdu is None:
        return [r for r in results if r[1] == "FAIL"]
    measured = success_verdict(out, psdu)
    check_status_v2(out, measured)
    dims, want_work, want_native, impaired = check_lengths(out, mac, psdu)
    check_stages_v2(out, dims, want_work, want_native, impaired)
    check_decode_v2(out, mac, psdu, measured)
    check_compare_v2(out, measured)
    check_filter(out)
    check_evidence_v2(out, measured)
    check_provenance(out, mac, psdu)
    check_config(out)
    check_impairment_v2(out)
    check_context_v2(out)
    check_kernel_v2(out)
    check_observation(out)
    return [r for r in results if r[1] == "FAIL"]


def run_checks(out):
    """Run the whole suite; return the list of failing checks."""
    del results[:]
    schema = out.get("schema")
    if schema == "twr-m2a-native/2":
        ok("the manifest declares the twr-m2a-native/2 schema", schema)
        return run_checks_v2(out)
    if schema == "twr-m2a-native/1":
        # Explicit backward compatibility: the /1 consumer is accepted and the
        # new /2-only fields are simply absent (never invented).
        ok("the manifest declares the twr-m2a-native/1 schema (backward compatible)",
           schema)
        return run_checks_v1(out)
    fail("the manifest declares a known twr-m2a-native schema",
         "got %r (want twr-m2a-native/2 or /1)" % (schema,))
    return [r for r in results if r[1] == "FAIL"]


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------


def _stage(out, name):
    return stage_by_name(out, name)


def _mutation_specs(out):
    specs = []

    def mac_byte(o):
        frame = o["frame"]
        mac = bytearray.fromhex(frame["mac_hex"])
        mac[6] ^= 0x01
        frame["mac_hex"] = mac.hex()
    specs.append(("MAC byte", mac_byte))

    def fcs(o):
        o["frame"]["fcs"] = "0x0000" if o["frame"]["fcs"] != "0x0000" else "0x1111"
    specs.append(("FCS", fcs))

    def fcs_hex(o):
        o["frame"]["fcs_hex"] = "dead"
    specs.append(("FCS hex", fcs_hex))

    def psdu_len(o):
        o["frame"]["psdu_bytes"] = int(o["frame"]["psdu_bytes"]) + 1
    specs.append(("PSDU length", psdu_len))

    def work_len(o):
        o["samples"]["work_tx"] = int(o["samples"]["work_tx"]) + 1
    specs.append(("work length", work_len))

    def native_len(o):
        o["samples"]["native"] = int(o["samples"]["native"]) + 1
    specs.append(("native length", native_len))

    def rx_len(o):
        o["samples"]["work_rx"] = int(o["samples"]["work_rx"]) + 1
    specs.append(("RX length", rx_len))

    def stage_out(o):
        _stage(o, "tx_resample")["out_count"] = int(
            _stage(o, "tx_resample")["out_count"]) + 1
    specs.append(("stage out_count", stage_out))

    def stage_delay(o):
        _stage(o, "tx_resample")["filter_delay"] = 0.0
    specs.append(("stage filter_delay", stage_delay))

    def payload(o):
        dec = o["decode"]
        payload = bytearray.fromhex(dec["payload_hex"])
        payload[6] ^= 0x01
        dec["payload_hex"] = payload.hex()
    specs.append(("decoded payload", payload))

    def decode_status(o):
        o["decode"]["status"] = "fcs_failed"
    specs.append(("decode status", decode_status))

    def fcs_pass(o):
        o["decode"]["fcs_pass"] = False
    specs.append(("decode fcs_pass", fcs_pass))

    def phr_len(o):
        o["decode"]["phr_psdu_length"] = int(o["decode"]["phr_psdu_length"]) + 1
    specs.append(("PHR length", phr_len))

    def compare(o):
        o["compare"]["bytes_exact"] = False
    specs.append(("compare bytes_exact", compare))

    def evidence_level(o):
        o["evidence"]["level"] = "toa_verified"
    specs.append(("evidence level above ceiling", evidence_level))

    def evidence_kind(o):
        o["evidence"]["kind"] = "not_measured"
    specs.append(("evidence kind inconsistent", evidence_kind))

    def measurement_valid(o):
        o["evidence"]["measurement_valid"] = True
    specs.append(("measurement_valid", measurement_valid))

    def hardware_readback(o):
        o["evidence"]["hardware_readback"] = {"device": "x410"}
    specs.append(("hardware_readback", hardware_readback))

    def yields_range(o):
        o["evidence"]["yields_range"] = True
    specs.append(("yields_range", yields_range))

    def input_hash(o):
        o["provenance"]["input_sha256"] = "0" * 64
    specs.append(("input_sha256", input_hash))

    def output_hash(o):
        o["provenance"]["output_sha256"] = "1" * 64
    specs.append(("output_sha256", output_hash))

    def executed_hash(o):
        o["config"]["executed_sha256"] = "2" * 64
    specs.append(("executed_sha256", executed_hash))

    def effective_field(o):
        o["config"]["effective"]["ranging"] = False
    specs.append(("effective config field", effective_field))

    def matlab(o):
        o["provenance"]["matlab"]["executed"] = True
    specs.append(("matlab.executed", matlab))

    # ---- /2-only fields ---------------------------------------------------
    # A /1 manifest has none of these blocks, so its self-test stops here.
    if out.get("schema") != "twr-m2a-native/2":
        return specs

    def imp_enabled(o):
        o["impairment"]["enabled"] = not o["impairment"]["enabled"]
    specs.append(("impairment.enabled", imp_enabled))

    def _different(cur, candidates):
        """Pick the first candidate that differs from `cur`."""
        for c in candidates:
            if c != cur:
                return c
        return candidates[-1]

    def imp_cfo(o):
        o["impairment"]["cfo_hz"] = _different(o["impairment"]["cfo_hz"],
                                               [123.0, 456.0, 0.5])
    specs.append(("impairment.cfo_hz", imp_cfo))

    def imp_awgn_snr(o):
        o["impairment"]["awgn_snr_db"] = _different(o["impairment"]["awgn_snr_db"],
                                                    [5.0, 7.5, 21.0])
    specs.append(("impairment.awgn_snr_db", imp_awgn_snr))

    def imp_awgn_seed(o):
        o["impairment"]["awgn_seed"] = _different(o["impairment"]["awgn_seed"],
                                                  [5, 6, 7])
    specs.append(("impairment.awgn_seed", imp_awgn_seed))

    def imp_delay_int(o):
        o["impairment"]["delay_int_samples"] = _different(
            o["impairment"]["delay_int_samples"], [3, 4, 5])
    specs.append(("impairment.delay_int_samples", imp_delay_int))

    def imp_delay_num(o):
        # Must stay a PROPER fraction and must DIFFER from the current value:
        # a fixed literal is a no-op when the scenario already uses it.
        imp = o["impairment"]
        den = imp.get("delay_frac_den")
        num = imp.get("delay_frac_num")
        if isinstance(den, int) and den > 1 and isinstance(num, int):
            imp["delay_frac_num"] = (num + 1) % den
        else:
            # den == 1 forces num == 0; move to a different proper fraction.
            imp["delay_frac_den"] = 2
            imp["delay_frac_num"] = 1
    specs.append(("impairment.delay_frac_num", imp_delay_num))

    def imp_delay_den(o):
        o["impairment"]["delay_frac_den"] = 0
    specs.append(("impairment.delay_frac_den", imp_delay_den))

    def imp_multipath(o):
        cur = o["impairment"]["multipath"]
        cand = [{"re": 1.0, "im": 0.0}, {"re": 0.5, "im": 0.0}]
        if cur == cand:
            cand = [{"re": 0.25, "im": 0.0}]
        o["impairment"]["multipath"] = cand
    specs.append(("impairment.multipath", imp_multipath))

    def imp_snr_def(o):
        o["impairment"]["snr_definition"] = "whole-buffer mean power"
    specs.append(("impairment.snr_definition", imp_snr_def))

    def imp_region(o):
        o["impairment"]["valid_region"] = [0, 5]
    specs.append(("impairment.valid_region", imp_region))

    def imp_region_samples(o):
        o["impairment"]["region_samples"] = 5
    specs.append(("impairment.region_samples", imp_region_samples))

    def ctx_prepared(o):
        o["context"]["prepared"] = False
    specs.append(("context.prepared", ctx_prepared))

    def ctx_capacity(o):
        o["context"]["capacity_samples"] = 1
    specs.append(("context.capacity_samples", ctx_capacity))

    def ctx_path(o):
        o["context"]["path"] = "bogus_path"
    specs.append(("context.path", ctx_path))

    def kernel_tx(o):
        o["kernel"]["tx"] = "made_up_kernel"
    specs.append(("kernel.tx", kernel_tx))

    def kernel_rx(o):
        o["kernel"]["rx"] = "made_up_kernel"
    specs.append(("kernel.rx", kernel_rx))

    def kernel_demod(o):
        o["kernel"]["demod"] = "made_up_kernel"
    specs.append(("kernel.demod", kernel_demod))

    def stage_origin(o):
        _stage(o, "tx_resample")["origin"] = 1
    specs.append(("stage origin", stage_origin))

    def stage_pad_front(o):
        _stage(o, "tx_resample")["pad_front"] = int(
            _stage(o, "tx_resample")["pad_front"]) + 1
    specs.append(("stage pad_front", stage_pad_front))

    def stage_pad_back(o):
        _stage(o, "tx_resample")["pad_back"] = int(
            _stage(o, "tx_resample")["pad_back"]) + 1
    specs.append(("stage pad_back", stage_pad_back))

    def stage_unit(o):
        _stage(o, "demod")["unit"] = "widgets"
    specs.append(("stage unit", stage_unit))

    def stage_in_unit(o):
        _stage(o, "hrp_mod")["in_unit"] = "widgets"
    specs.append(("stage in_unit", stage_in_unit))

    def stage_out_unit(o):
        _stage(o, "demod")["out_unit"] = "widgets"
    specs.append(("stage out_unit", stage_out_unit))

    def stage_unit_alias(o):
        # unit must keep aliasing out_unit
        _stage(o, "tx_resample")["unit"] = "widgets"
    specs.append(("stage unit aliases out_unit", stage_unit_alias))

    def stage_trim(o):
        _stage(o, "rx_resample")["trim"] = 5
    specs.append(("stage trim", stage_trim))

    def stage_guard(o):
        _stage(o, "demod")["search_guard_front"] = int(
            _stage(o, "demod")["search_guard_front"]) + 1
    specs.append(("stage search_guard_front", stage_guard))

    def stage_roi(o):
        roi = _stage(o, "demod")["search_roi"]
        _stage(o, "demod")["search_roi"] = [int(roi[0]) + 1, int(roi[1])]
    specs.append(("stage search_roi", stage_roi))

    def status_attempted(o):
        o["status"]["attempted"] = int(o["status"]["attempted"]) + 1
    specs.append(("status.attempted", status_attempted))

    def status_success(o):
        o["status"]["exact_success"] = int(o["status"]["exact_success"]) + 1
    specs.append(("status.exact_success", status_success))

    def status_failure(o):
        o["status"]["explicit_failure"] = int(o["status"]["explicit_failure"]) + 1
    specs.append(("status.explicit_failure", status_failure))

    def status_m2a(o):
        o["status"]["m2a_status"] = "bogus_status"
    specs.append(("status.m2a_status", status_m2a))

    def diag(o):
        o["decode"]["diagnostic"]["sfd_start_sample"] = o["samples"]["work_rx"]
    specs.append(("diagnostic coordinate", diag))

    return specs


def self_test(out):
    global _QUIET
    _QUIET = False
    print("== self-test: the pristine manifest must PASS ==")
    base = run_checks(out)
    if base:
        print("SELF-TEST FAILED: the pristine manifest already fails: %s"
              % [f[0] for f in base])
        return 1
    print("PASS (%d checks)" % len(results))
    missed = []
    for name, mutate in _mutation_specs(out):
        mutant = copy.deepcopy(out)
        try:
            mutate(mutant)
        except Exception as exc:  # fixture problem, not a verifier result
            print("SELF-TEST MUTATION ERROR (%s): %s" % (name, exc))
            missed.append(name)
            continue
        _QUIET = True
        caught = run_checks(mutant)
        _QUIET = False
        if caught:
            print("[PASS] mutation '%s' caught by %d check(s): %s"
                  % (name, len(caught), caught[0][0]))
        else:
            print("[FAIL] mutation '%s' was NOT caught" % name)
            missed.append(name)
    print("=" * 72)
    if missed:
        print("SELF-TEST FAILED: %s" % missed)
        return 1
    print("SELF-TEST PASSED: every mutation was detected")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", required=True,
                    help="the twr-m2a-native/{1,2} JSON the demo wrote")
    ap.add_argument("--self-test", action="store_true",
                    help="also mutate a copy of the manifest and require failure")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.output):
        sys.stderr.write("error: %s does not exist\n" % args.output)
        return 2
    try:
        with open(args.output, "r", encoding="utf-8") as handle:
            out = json.load(handle)
    except (ValueError, OSError) as exc:
        sys.stderr.write("error: cannot read the manifest: %s\n" % exc)
        return 2

    if args.self_test:
        return self_test(out)

    failed = run_checks(out)
    print("=" * 72)
    print("%d/%d checks pass" % (len(results) - len(failed), len(results)))
    if failed:
        print("FAILED: " + ", ".join(f[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
