#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""INDEPENDENT verifier for the M2-A native-rate PHY demo (``twr-m2a-native/1``).

WHAT IT CHECKS, AND HOW IT IS INDEPENDENT
-----------------------------------------
It reads ONLY the demo's JSON manifest and recomputes every expected value in
Python.  It never imports, calls or links the C++ helper, the C++ codec, the C++
resampler or the C++ demo: two C++ functions agreeing is never treated as truth.

For each manifest it independently derives:

  * the MAC byte layout from the frame FIELDS (frame v1: 1 B version, 1 B
    function code, six little-endian u16 header fields, then 0/2/3 40-bit
    little-endian timestamps) and compares it byte-for-byte with ``mac_hex``;
  * the CRC-16 (reflected poly 0x8408, init 0, no final xor, little-endian
    append) over the reconstructed MAC bytes and compares it with ``fcs`` and
    the PSDU tail;
  * the expected work-grid waveform length from the PSDU length
    (``94656 + nsym*128``, ``nsym = 8*psdu + 48*ceil(8*psdu/330)``);
  * the expected native and RX lengths from the CAUSAL FULL-CONVOLUTION
    contract ``ceil(((N-1)*L + T)/M)`` using the JSON's tap counts and L/M;
  * the decode result: PHR PSDU length, decoded PSDU bytes and FCS pass;
  * the comparator verdict (bytes exact, error tolerances).

It REFUSES to accept the run as more than a native round trip:

  * ``evidence.level`` may only be ``native_roundtrip_verified`` or
    ``work_decode_verified``; anything above (ToA / hardware / vendor / ranging)
    fails;
  * a ``native_roundtrip_verified`` claim requires ``kind == "measured"`` AND a
    genuinely successful decode, and the comparator must actually be exact;
  * ``measurement_valid`` must be ``false`` and ``hardware_readback`` ``null``;
  * ``matlab.executed`` must be ``false`` and no ``yields_range`` may be true.

The provenance hashes are re-derived here too: ``input_sha256`` must be the
SHA-256 of the reconstructed MAC bytes, ``output_sha256`` the SHA-256 of the
decoded PSDU, and ``config.executed_sha256`` the SHA-256 of a canonical,
integer-only text built from ``config.effective``.

``--self-test`` mutates a deep copy of the manifest (a MAC byte, the FCS, the
payload, the work/native lengths, a stage coordinate, the evidence level,
``measurement_valid``, ``hardware_readback``, a provenance hash and the
executed-config hash) and asserts each mutation makes the verifier fail.  That
is the negative test for this verifier itself.

    python3 tools/twr/verify_m2_a_native_phy.py --output out.json
    python3 tools/twr/verify_m2_a_native_phy.py --output out.json --self-test

Exit status 0 iff every check passes; non-zero with a clear diff otherwise.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import struct
import sys

# ---------------------------------------------------------------------------
# Frozen constants (G0 §1, §3, §6).  Repeated here on purpose: this file is a
# second, independent statement of the contract, not a reader of the C++.
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

EVIDENCE_ALLOWED = {"work_decode_verified", "native_roundtrip_verified"}
EVIDENCE_ABOVE = {
    "toa_verified",
    "hardware_verified",
    "vendor_interop_verified",
    "ranging",
}

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


# ---------------------------------------------------------------------------
# Checks
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

    # Independent FCS over the RECONSTRUCTED bytes.
    crc = crc16_802154(mac)
    want_fcs = struct.pack("<H", crc)
    if frame.get("fcs") == "0x%04x" % crc:
        ok("frame.fcs equals the independently computed CRC-16",
           "0x%04x" % crc)
    else:
        fail("frame.fcs equals the independently computed CRC-16",
             "fcs=%r recomputed=0x%04x" % (frame.get("fcs"), crc))
    if frame.get("fcs_hex") == "%04x" % crc:
        ok("frame.fcs_hex equals the independently computed CRC-16",
           "%04x" % crc)
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
        return None, None, None
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
        return None, None, None

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
    want_rx = resampled_length(native, rx_taps, RX_L, tx_l)
    if samples.get("work_rx") == want_rx:
        ok("samples.work_rx equals ceil(((N-1)*L+T)/M) on the RX direction",
           "%d -> %d (L=%d M=%d T=%d)" % (native, want_rx, RX_L, tx_l, rx_taps))
    else:
        fail("samples.work_rx equals ceil(((N-1)*L+T)/M) on the RX direction",
             "work_rx=%r want %d (N=%r L=%d M=%d T=%d)" % (
                 samples.get("work_rx"), want_rx, native, RX_L, tx_l, rx_taps))
    return (tx_l, tx_m, tx_taps, rx_taps), want_work, want_native


def check_stages(out, dims, want_work, want_native):
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
        ("rx_resample", RX_L, tx_l, WORK_HZ, want_native, samples.get("work_rx"),
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
        if isinstance(vf, int) and isinstance(vt, int) and 0 <= vf < vt:
            ok("stage %s has a non-empty valid range" % name, "[%d,%d)" % (vf, vt))
        else:
            fail("stage %s has a non-empty valid range" % name,
                 "valid_from=%r valid_to=%r" % (vf, vt))


def check_decode(out, mac, psdu):
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
    if isinstance(diag, dict) and isinstance(diag.get("sfd_start_sample"), int) and \
            isinstance(diag.get("packet_start_sample"), int):
        ok("decode.diagnostic carries the two integer sample coordinates",
           "sfd=%s packet=%s" % (diag["sfd_start_sample"],
                                 diag["packet_start_sample"]))
    else:
        fail("decode.diagnostic carries the two integer sample coordinates",
             "got %r" % (diag,))


def check_compare(out):
    comp = out.get("compare")
    if not isinstance(comp, dict):
        fail("compare block is present", "got %r" % (comp,))
        return
    if comp.get("bytes_exact") is True:
        ok("compare.bytes_exact is true", "true")
    else:
        fail("compare.bytes_exact is true", "got %r" % (comp.get("bytes_exact"),))
    mae, rl2 = comp.get("max_abs_error"), comp.get("relative_l2")
    if isinstance(mae, (int, float)) and 0.0 <= mae <= MAX_ABS_ERROR:
        ok("compare.max_abs_error is within the frozen tolerance", str(mae))
    else:
        fail("compare.max_abs_error is within the frozen tolerance",
             "got %r want <= %g" % (mae, MAX_ABS_ERROR))
    if isinstance(rl2, (int, float)) and 0.0 <= rl2 <= RELATIVE_L2:
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


def check_evidence(out):
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

    # A measured native claim requires a genuinely successful run.
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
        ok("a not_measured run does not claim native evidence",
           "kind=not_measured")
    elif ev.get("kind") == "not_measured":
        fail("a not_measured run does not claim native evidence",
             "kind=not_measured but the run succeeded and level=%r" % (level,))


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
        ok("provenance.input_sha256 is the SHA-256 of the reconstructed MAC",
           want_in)
    else:
        fail("provenance.input_sha256 is the SHA-256 of the reconstructed MAC",
             "output=%r recomputed=%s" % (prov.get("input_sha256"), want_in))
    if is_hex64(prov.get("output_sha256")):
        ok("provenance.output_sha256 is well formed", prov["output_sha256"])
    else:
        fail("provenance.output_sha256 is well formed",
             "got %r" % (prov.get("output_sha256"),))
    want_out = hashlib.sha256(psdu).hexdigest()
    if prov.get("output_sha256") == want_out:
        ok("provenance.output_sha256 is the SHA-256 of the decoded PSDU",
           want_out)
    else:
        fail("provenance.output_sha256 is the SHA-256 of the decoded PSDU",
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
        ok("provenance.matlab.executed is false (no MATLAB claim)",
           "executed=false")
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
    # The effective block must restate the frozen profile.
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


def check_status(out):
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


def run_checks(out):
    """Run the whole suite; return the list of failing checks."""
    del results[:]
    if out.get("schema") != "twr-m2a-native/1":
        fail("the manifest declares the twr-m2a-native/1 schema",
             "got %r" % (out.get("schema"),))
        return [r for r in results if r[1] == "FAIL"]
    ok("the manifest declares the twr-m2a-native/1 schema", "twr-m2a-native/1")

    check_status(out)
    mac, psdu = check_frame_and_fcs(out)
    if mac is None or psdu is None:
        return [r for r in results if r[1] == "FAIL"]
    dims, want_work, want_native = check_lengths(out, mac, psdu)
    check_stages(out, dims, want_work, want_native)
    check_decode(out, mac, psdu)
    check_compare(out)
    check_filter(out)
    check_evidence(out)
    check_provenance(out, mac, psdu)
    check_config(out)
    return [r for r in results if r[1] == "FAIL"]


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------


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
        for s in o["stages"]:
            if s["name"] == "tx_resample":
                s["out_count"] = int(s["out_count"]) + 1
                return
    specs.append(("stage out_count", stage_out))

    def stage_delay(o):
        for s in o["stages"]:
            if s["name"] == "tx_resample":
                s["filter_delay"] = 0.0
                return
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
                    help="the twr-m2a-native/1 JSON the demo wrote")
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
