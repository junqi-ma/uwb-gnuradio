#!/usr/bin/env python3
# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""INDEPENDENT verifier for the M1-B two-endpoint protocol demo.

WHAT IT CHECKS, AND HOW IT IS INDEPENDENT
-----------------------------------------
It reads a ``twr-m1b-demo/1`` request and the demo's output and recomputes the
expected ToF from

  * the ACTUAL FRAME FIELD VALUES the demo emitted (t1A/t2B/t3B/t4A/t5A),
  * the ACTUAL local evidence the demo emitted (each endpoint's planned TX
    ticks and RX marker ticks), and
  * the request's transport constraints (distance, per-direction TX air
    latency, clock domains and the reply tick budgets),

using Python ``fractions.Fraction`` for the exact arithmetic.

It does NOT import, call or link the C++ FSM, the C++ ToF formulas, the C++
demo or any helper derived from them: the expected value is derived here, in a
different language and a different formulation.  Two C++ helpers agreeing is
never treated as truth.

It also refuses to accept the result as a ranging success: ``measurement_valid``
must be false, the provenance must be ``wire_claim`` / ``simulation``, and no
endpoint may publish a validated range.  It additionally re-derives the
transport arrival relation so a wire field cannot be a value the transport
could not have produced.

REMEDIATION CHECKS (R01-R09)
----------------------------
Beyond the ToF re-derivation, the same run re-checks the review findings that
do not depend on the formula, entirely in Python:

  * R08  every TX frame's numeric ``plan`` must satisfy its own identity
         ``calibrated_air_ticks == quantised_instant_ticks + marker_offset_ticks``,
         live on THIS endpoint's declared domain (name/rate/epoch/width), cite
         the calibration in force with the RMARKER_TX correction set, and its
         ``calibrated_air_ticks`` must equal the frame's ``t3B`` (Response) /
         ``t5A`` (Final) and the recorded local TX instant.
  * R04  the per-endpoint token trace must be strictly increasing (a reused or
         non-positive token fails) and the executed session generation must
         match the requested one (a stale-generation binding fails).
  * R05/R14  per endpoint ``accepted == terminal + in_flight`` (with the
         derived in-flight when the demo does not publish it), no more terminals
         than accepted, and ``results_dropped == 0``.
  * R07  every ``arm_rx`` deadline must be positive and must never exceed the
         endpoint's absolute exchange deadline ``accept + exchange_timeout``
         (the evidence wait may tighten, never extend it).
  * R09  ``config_sha256`` / ``profile_sha256`` are recomputed here, with
         ``hashlib``, from ``configuration.executed_fields`` /
         ``configuration.profile_fields`` and compared; the
         requested->effective->core ``mapping`` must be present and the executed
         values must match the request, so a mutated executed value fails.
  * provenance  every endpoint must stay ``wire_claim`` / ``simulation`` /
         ``measurement_valid == false`` / ``yields_range == false``.

``--self-test`` mutates a copy of the output in memory (a ToF numerator, a
frame timestamp, a provenance flag, a plan identity, a token, an executed
config field, a counter and a deadline) and asserts each mutation makes the
verifier fail.  That is the negative test for this verifier itself.

    python3 tools/twr/verify_m1_b_protocol.py --request req.json --output out.json
    python3 tools/twr/verify_m1_b_protocol.py --request req.json --output out.json --self-test

Exit status 0 iff every check passes; non-zero with a clear diff otherwise.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import sys
from fractions import Fraction as F

C_MPS = 299792458.0  # exactly the fake link's kFakeLinkSpeedOfLightMps

# R08: RMARKER_TX requires exactly these correction bits to be set (see
# uwb_twr_timestamp.h::timestamp_required_corrections).  Kept here as a
# documented constant rather than imported from the C++ tree.
REQUIRED_RMARKER_TX_CORRECTIONS = (1 << 4) | (1 << 6) | (1 << 7)  # 208
REQUIRED_TX_CORRECTION_NAMES = (
    "waveform_geometry", "tx_command_to_air", "delayed_tx_quantization",
)
# The frame field a plan-derived TX instant is written into, per frame type.
PLAN_FRAME_FIELD = {"response": "t3B", "final": "t5A"}

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
# Exact helpers -- NO C++ code is imported or consulted
# ---------------------------------------------------------------------------


def round_half_away(value_float):
    """The exact value of a Python/C++ double, rounded half away from zero.

    ``llround()`` rounds half away from zero; replicating it ON THE EXACT
    DOUBLE (Fraction(x) is the exact binary value of the double) keeps the
    verifier independent of, but numerically identical to, the transport.
    """
    f = F(value_float)
    if f < 0:
        raise ValueError("round_half_away expects a non-negative value")
    q, r = divmod(f.numerator, f.denominator)
    if 2 * r >= f.denominator:
        q += 1
    return q


def arrival_ticks(req, tx_air_ticks, source):
    """The exact tick at which the transport would deliver a source->dest frame.

    Mirrors the documented arrival arithmetic with exact arithmetic where the
    C++ uses exact integers and Fraction(x) where the C++ uses a double, so a
    value the transport could not have produced is rejected.
    """
    dom_a = req["domains"]["a"]
    dom_b = req["domains"]["b"]
    src = dom_a if source == "a" else dom_b
    dst = dom_b if source == "a" else dom_a
    latency = (req["link"]["tx_air_latency_a_ticks"] if source == "a"
               else req["link"]["tx_air_latency_b_ticks"])
    src_ticks = int(tx_air_ticks) + int(latency)

    if float(src["tick_rate_hz"]) == float(dst["tick_rate_hz"]):
        dest_ticks = src_ticks
    else:
        seconds = float(src_ticks) / float(src["tick_rate_hz"])
        dest_ticks = round_half_away(seconds * float(dst["tick_rate_hz"]))

    prop = round_half_away(float(req["link"]["distance_m"]) / C_MPS *
                           float(dst["tick_rate_hz"]))
    return dest_ticks + prop


def clock_ratio(req):
    cr = req.get("clock_ratio") or {}
    kind = cr.get("kind")
    if kind == "unity_same_clock":
        return F(1)
    if kind == "nominal_rate_ratio":
        return F(int(cr["fA_hz"]), int(cr["fB_hz"]))
    raise ValueError("unknown clock_ratio.kind %r" % kind)


# ---------------------------------------------------------------------------
# Output accessors
# ---------------------------------------------------------------------------


def endpoint(output, name):
    eps = output.get("endpoints") or {}
    return eps.get(name)


def local_ticks(ep, group, ftype):
    ev = ep.get("local_evidence") or {}
    return ev.get(group, {}).get(ftype)


def frame(ep, ftype, direction):
    for f in ep.get("frames") or []:
        if f.get("type") == ftype and f.get("dir") == direction:
            return f
    return None


def frame_ts(fr, name):
    fields = fr.get("fields") or {}
    return (fields.get("timestamps") or {}).get(name)


def tof_fraction(ep):
    tof = ep.get("tof") or {}
    if not tof.get("available"):
        return None
    return F(int(tof["num"]), int(tof["den"]))


def is_hex64(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value or "") is not None


def counters_of(ep):
    """The endpoint's counter object, whether emitted as an object or a JSON string."""
    c = (ep or {}).get("counters")
    if isinstance(c, str):
        try:
            return json.loads(c)
        except ValueError:
            return None
    return c


def kv_map(entries):
    """``configuration.executed_fields`` / ``profile_fields`` -> {path: value}."""
    out = {}
    for row in entries or []:
        if isinstance(row, dict) and "path" in row:
            out[row["path"]] = row.get("value")
    return out


def canonical_fields(entries):
    """The documented canonical text: one "path=value" per entry, '\\n' joined,
    with a trailing newline (matches configuration.canonical_layout)."""
    return "\n".join("%s=%s" % (r["path"], r["value"]) for r in entries or []) + "\n"


def tx_frames(ep):
    return [f for f in (ep.get("frames") or []) if f.get("dir") == "tx"]


def rx_frames(ep):
    return [f for f in (ep.get("frames") or []) if f.get("dir") == "rx"]


def frame_type(fr):
    return fr.get("type")


def as_int(value):
    """Accept an int, a decimal string, or a JSON number."""
    if value is None:
        raise ValueError("missing integer")
    return int(value)


DOMAIN_RE = re.compile(
    r"^(?P<name>.+)@(?P<rate>-?\d+(?:\.\d+)?)Hz,epoch=(?P<epoch>-?\d+),bits=(?P<bits>-?\d+)$")


def parse_domain_string(text):
    m = DOMAIN_RE.match(text or "")
    if not m:
        return None
    d = m.groupdict()
    return {"name": d["name"], "tick_rate_hz": float(d["rate"]),
            "epoch_id": int(d["epoch"]), "timestamp_bits": int(d["bits"])}


def expected_domain(req, name):
    d = req["domains"][name]
    return {"name": d["name"], "tick_rate_hz": float(d["tick_rate_hz"]),
            "epoch_id": int(d["epoch_id"]), "timestamp_bits": int(d["timestamp_bits"])}


def same_domain(a, b):
    return (a and b and a["name"] == b["name"]
            and float(a["tick_rate_hz"]) == float(b["tick_rate_hz"])
            and int(a["epoch_id"]) == int(b["epoch_id"])
            and int(a["timestamp_bits"]) == int(b["timestamp_bits"]))


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------


def check_common(req, out, request_bytes):
    sim = out.get("simulation") or {}
    if sim.get("execution") == "offline_simulation" and \
            sim.get("mode") == "protocol_estimate":
        ok("envelope is an explicit offline simulation / protocol estimate",
           "execution=%s mode=%s" % (sim.get("execution"), sim.get("mode")))
    else:
        fail("envelope is an explicit offline simulation / protocol estimate",
             "got execution=%r mode=%r" % (sim.get("execution"), sim.get("mode")))

    if out.get("protocol") == req.get("protocol"):
        ok("output protocol matches the request", out.get("protocol"))
    else:
        fail("output protocol matches the request",
             "output=%r request=%r" % (out.get("protocol"), req.get("protocol")))

    digest = hashlib.sha256(request_bytes).hexdigest()
    if out.get("input_sha256") == digest:
        ok("input_sha256 is the SHA-256 of the request bytes", digest)
    else:
        fail("input_sha256 is the SHA-256 of the request bytes",
             "output=%r recomputed=%s" % (out.get("input_sha256"), digest))

    for key in ("config_sha256", "profile_sha256", "output_sha256"):
        if is_hex64(out.get(key)):
            ok("%s is present and well formed" % key, out[key])
        else:
            fail("%s is present and well formed" % key, "got %r" % out.get(key))

    prov = out.get("provenance") or {}
    if (prov.get("peer_evidence") == "wire_claim" and
            prov.get("measurement_valid") is False and
            prov.get("execution_mode") == "simulation"):
        ok("top-level provenance is wire_claim / simulation / not a measurement",
           "peer_evidence=wire_claim measurement_valid=False execution_mode=simulation")
    else:
        fail("top-level provenance is wire_claim / simulation / not a measurement",
             json.dumps(prov))
    if "distance_m" in prov:
        fail("no distance is presented as an accepted measurement",
             "provenance carries a distance_m")
    else:
        ok("no distance is presented as an accepted measurement",
           "distance exists only in transport_model (observer-only)")

    try:
        want_k = clock_ratio(req)
    except (KeyError, ValueError) as exc:
        fail("the request declares a usable clock ratio", str(exc))
    else:
        cr = out.get("clock_ratio") or {}
        try:
            got_k = F(int(cr["k_num"]), int(cr["k_den"]))
        except (KeyError, TypeError, ValueError):
            got_k = None
        if got_k == want_k:
            ok("the output echoes the clock ratio the formulas consumed",
               "k=%s (%s)" % (got_k, cr.get("source")))
        else:
            fail("the output echoes the clock ratio the formulas consumed",
                 "output=%r want=%s" % (cr, want_k))


def check_endpoint_provenance(ep, expect_estimate):
    label = "endpoint %s" % ep.get("endpoint_id")
    good = (
        ep.get("peer_evidence") == "wire_claim" and
        ep.get("measurement_valid") is False and
        ep.get("execution_mode") == "simulation" and
        ep.get("yields_range") is False and
        ep.get("is_hardware_measurement") is False and
        ep.get("is_validated_measurement") is False
    )
    if good:
        ok("%s cannot be read as a ranging success" % label,
           "peer_evidence=wire_claim measurement_valid=False yields_range=False "
           "is_hardware_measurement=False is_validated_measurement=False")
    else:
        fail("%s cannot be read as a ranging success" % label,
             json.dumps({k: ep.get(k) for k in (
                 "peer_evidence", "measurement_valid", "execution_mode",
                 "yields_range", "is_hardware_measurement",
                 "is_validated_measurement")}))
    if ep.get("estimate_available") is bool(expect_estimate):
        ok("%s estimate_available == %s (role assignment)" % (label, expect_estimate),
           ep.get("completion"))
    else:
        fail("%s estimate_available == %s (role assignment)" % (label, expect_estimate),
             "got %r, completion=%r" % (ep.get("estimate_available"), ep.get("completion")))
    # The endpoint that owns no estimate must not fabricate one.
    if not expect_estimate:
        if tof_fraction(ep) is None and not (ep.get("tof") or {}).get("available"):
            ok("%s owns no ToF (never copies the peer's result)" % label,
               (ep.get("tof") or {}).get("note", ""))
        else:
            fail("%s owns no ToF (never copies the peer's result)" % label,
                 json.dumps(ep.get("tof")))
    # No endpoint-level distance field.
    if "distance" not in ep and "distance_m" not in ep:
        ok("%s carries no distance field" % label, str(sorted(ep.keys()))[:200])
    else:
        fail("%s carries no distance field" % label, "found a distance key")


def check_wire_copies(out):
    """The fields the SENDER encoded and the fields the RECEIVER decoded must be
    identical: both are decodes of the same on-air bytes.  This catches a tamper
    in the received copy that the sender-side formula re-derivation would miss."""
    label = "wire copy"
    pairs = [("a", "poll", "b"), ("b", "response", "a")]
    if out.get("protocol") == "ds":
        pairs.append(("a", "final", "b"))
    for src, ftype, dst in pairs:
        tf = frame(endpoint(out, src), ftype, "tx")
        rf = frame(endpoint(out, dst), ftype, "rx")
        if tf is None or rf is None:
            fail("%s: %s->%s %s pair exists" % (label, src, dst, ftype),
                 "tx=%s rx=%s" % (tf is not None, rf is not None))
            continue
        if tf.get("fields") == rf.get("fields"):
            ok("%s: %s->%s %s decoded fields equal the encoded fields" % (
                label, src, dst, ftype), "session/seq/addressing/timestamps agree")
        else:
            fail("%s: %s->%s %s decoded fields equal the encoded fields" % (
                label, src, dst, ftype),
                 "tx=%s rx=%s" % (json.dumps(tf.get("fields")), json.dumps(rf.get("fields"))))


def check_frames_addressed(ep, name):
    local = ep.get("local_address")
    peer = ep.get("peer_address")
    tried = 0
    for fr in ep.get("frames") or []:
        fields = fr.get("fields") or {}
        if fr.get("dir") == "tx":
            if fields.get("src_addr") == local and fields.get("dst_addr") == peer:
                tried += 1
            else:
                fail("endpoint %s %s frame addressing" % (name, fr.get("type")),
                     "src=%r dst=%r want src=%r dst=%r" % (
                         fields.get("src_addr"), fields.get("dst_addr"), local, peer))
                return
        else:
            if fields.get("src_addr") == peer and fields.get("dst_addr") == local:
                tried += 1
            else:
                fail("endpoint %s %s frame addressing" % (name, fr.get("type")),
                     "src=%r dst=%r want src=%r dst=%r" % (
                         fields.get("src_addr"), fields.get("dst_addr"), peer, local))
                return
    ok("endpoint %s frame addressing is self-consistent" % name,
       "%d frames checked" % tried)


def verify_ss(req, out):
    a = endpoint(out, "a")
    b = endpoint(out, "b")
    if not a or not b:
        fail("SS: both endpoints are present", "missing endpoint object")
        return
    check_endpoint_provenance(a, True)
    check_endpoint_provenance(b, False)
    check_frames_addressed(a, "a")
    check_frames_addressed(b, "b")

    resp_rx = frame(a, "response", "rx")
    resp_tx = frame(b, "response", "tx")
    if resp_rx is None or resp_tx is None:
        fail("SS: the Response frame is present at both endpoints",
             "rx=%s tx=%s" % (resp_rx is not None, resp_tx is not None))
        return

    t1 = local_ticks(a, "tx_planned_ticks", "poll")
    t4 = local_ticks(a, "rx_marker_ticks", "response")
    t2b = frame_ts(resp_rx, "t2B")
    t3b = frame_ts(resp_rx, "t3B")
    if None in (t1, t4, t2b, t3b):
        fail("SS: the required frame fields and local evidence are present",
             "t1=%r t4=%r t2B=%r t3B=%r" % (t1, t4, t2b, t3b))
        return

    # The wire fields must equal the responder's own local evidence.
    if frame_ts(resp_tx, "t2B") == t2b and frame_ts(resp_tx, "t3B") == t3b:
        ok("SS: the Response wire fields equal the responder's local evidence",
           "t2B=%s t3B=%s" % (t2b, t3b))
    else:
        fail("SS: the Response wire fields equal the responder's local evidence",
             "rx(t2B=%s,t3B=%s) tx(t2B=%s,t3B=%s)" % (
                 t2b, t3b, frame_ts(resp_tx, "t2B"), frame_ts(resp_tx, "t3B")))

    k = clock_ratio(req)
    ra = F(int(t4) - int(t1))
    db = F(int(t3b) - int(t2b))
    expected = (ra - k * db) / 2
    got = tof_fraction(a)
    if got is None:
        fail("SS: the initiator publishes an exact ToF", "tof.available is false")
        return
    if got == expected:
        ok("SS: initiator ToF matches the independent Fraction re-derivation",
           "RA=%s DB=%s k=%s -> %s ; output %s" % (ra, db, k, expected, got))
    else:
        fail("SS: initiator ToF matches the independent Fraction re-derivation",
             "expected %s, output %s (RA=%s DB=%s k=%s)" % (expected, got, ra, db, k))

    # Transport re-derivation: the arrival of each frame must be exactly what
    # the request's distance/latency/domain model produces.
    b_poll = local_ticks(b, "rx_marker_ticks", "poll")
    t3 = local_ticks(b, "tx_planned_ticks", "response")
    t2b_local = local_ticks(b, "rx_marker_ticks", "poll")
    pred = arrival_ticks(req, t1, "a")
    if b_poll == t2b == t2b_local == pred:
        ok("SS: the Poll arrival is the transport's own propagation model",
           "arrival=%s" % pred)
    else:
        fail("SS: the Poll arrival is the transport's own propagation model",
             "b.rx_poll=%r wire.t2B=%r predicted=%s" % (b_poll, t2b, pred))
    pred2 = arrival_ticks(req, t3b, "b")
    if int(t4) == pred2:
        ok("SS: the Response arrival is the transport's own propagation model",
           "arrival=%s" % pred2)
    else:
        fail("SS: the Response arrival is the transport's own propagation model",
             "a.rx_response=%r predicted=%s (t3B=%s)" % (t4, pred2, t3b))

    reply = req["timeline"]["response_reply_ticks"]
    if int(t3b) == int(b_poll) + int(reply):
        ok("SS: the Response TX instant obeys the reply budget",
           "t3B = rx_poll(%s) + reply(%s)" % (b_poll, reply))
    else:
        fail("SS: the Response TX instant obeys the reply budget",
             "t3B=%s rx_poll=%s reply=%s" % (t3b, b_poll, reply))
    if int(t1) == int(req["timeline"]["poll_air_ticks"]):
        ok("SS: the Poll TX instant equals the requested poll air tick",
           str(req["timeline"]["poll_air_ticks"]))
    else:
        fail("SS: the Poll TX instant equals the requested poll air tick",
             "t1=%s want %s" % (t1, req["timeline"]["poll_air_ticks"]))


def verify_ds(req, out):
    a = endpoint(out, "a")
    b = endpoint(out, "b")
    if not a or not b:
        fail("DS: both endpoints are present", "missing endpoint object")
        return
    check_endpoint_provenance(a, False)
    check_endpoint_provenance(b, True)
    check_frames_addressed(a, "a")
    check_frames_addressed(b, "b")

    resp_tx = frame(b, "response", "tx")
    final_rx = frame(b, "final", "rx")
    final_tx = frame(a, "final", "tx")
    if None in (resp_tx, final_rx, final_tx):
        fail("DS: Response and Final frames are present",
             "resp=%s final_rx=%s final_tx=%s" % (
                 resp_tx is not None, final_rx is not None, final_tx is not None))
        return

    t2b = frame_ts(resp_tx, "t2B")
    t3b = frame_ts(resp_tx, "t3B")
    t1a = frame_ts(final_rx, "t1A")
    t4a = frame_ts(final_rx, "t4A")
    t5a = frame_ts(final_rx, "t5A")
    b_poll = local_ticks(b, "rx_marker_ticks", "poll")
    b_final = local_ticks(b, "rx_marker_ticks", "final")
    t3_local = local_ticks(b, "tx_planned_ticks", "response")
    a_poll = local_ticks(a, "tx_planned_ticks", "poll")
    a_resp = local_ticks(a, "rx_marker_ticks", "response")
    t5_local = local_ticks(a, "tx_planned_ticks", "final")

    needed = (t2b, t3b, t1a, t4a, t5a, b_poll, b_final, t3_local, a_poll,
              a_resp, t5_local)
    if None in needed:
        fail("DS: the required frame fields and local evidence are present",
             "some of %r are missing" % (needed,))
        return

    # Wire values vs local evidence at BOTH ends.
    checks = [
        ("t2B equals the responder Poll RX marker", t2b, b_poll),
        ("t3B equals the responder Response TX plan", t3b, t3_local),
        ("t1A equals the initiator Poll TX plan", t1a, a_poll),
        ("t4A equals the initiator Response RX marker", t4a, a_resp),
        ("t5A equals the initiator Final TX plan", t5a, t5_local),
    ]
    for name, wire, local in checks:
        if int(wire) == int(local):
            ok("DS: %s" % name, "%s" % wire)
        else:
            fail("DS: %s" % name, "wire=%s local=%s" % (wire, local))

    k = clock_ratio(req)
    ra = F(int(t4a) - int(t1a))
    da = F(int(t5a) - int(t4a))
    db = F(int(t3b) - int(t2b))
    rb = F(int(b_final) - int(t3b))
    expected = (ra * k * rb - da * k * db) / (ra + k * rb + da + k * db)
    got = tof_fraction(b)
    if got is None:
        fail("DS: the responder publishes an exact ToF", "tof.available is false")
        return
    if got == expected:
        ok("DS: responder ToF matches the independent Fraction re-derivation",
           "RA=%s RB=%s DA=%s DB=%s k=%s -> %s ; output %s"
           % (ra, rb, da, db, k, expected, got))
    else:
        fail("DS: responder ToF matches the independent Fraction re-derivation",
             "expected %s, output %s (RA=%s RB=%s DA=%s DB=%s k=%s)"
             % (expected, got, ra, rb, da, db, k))

    # Transport re-derivation.
    p_poll = arrival_ticks(req, a_poll, "a")
    p_resp = arrival_ticks(req, t3b, "b")
    p_final = arrival_ticks(req, t5a, "a")
    if int(b_poll) == p_poll and int(t2b) == p_poll:
        ok("DS: the Poll arrival is the transport's own propagation model",
           "arrival=%s" % p_poll)
    else:
        fail("DS: the Poll arrival is the transport's own propagation model",
             "b.rx_poll=%s wire.t2B=%s predicted=%s" % (b_poll, t2b, p_poll))
    if int(a_resp) == p_resp and int(t4a) == p_resp:
        ok("DS: the Response arrival is the transport's own propagation model",
           "arrival=%s" % p_resp)
    else:
        fail("DS: the Response arrival is the transport's own propagation model",
             "a.rx_response=%s wire.t4A=%s predicted=%s" % (a_resp, t4a, p_resp))
    if int(b_final) == p_final:
        ok("DS: the Final arrival is the transport's own propagation model",
           "arrival=%s" % p_final)
    else:
        fail("DS: the Final arrival is the transport's own propagation model",
             "b.rx_final=%s predicted=%s" % (b_final, p_final))

    reply = req["timeline"]["response_reply_ticks"]
    if int(t3b) == int(b_poll) + int(reply):
        ok("DS: the Response TX instant obeys the reply budget",
           "t3B = rx_poll(%s) + reply(%s)" % (b_poll, reply))
    else:
        fail("DS: the Response TX instant obeys the reply budget",
             "t3B=%s rx_poll=%s reply=%s" % (t3b, b_poll, reply))
    freply = req["timeline"]["final_reply_ticks"]
    if int(t5a) == int(a_resp) + int(freply):
        ok("DS: the Final TX instant obeys the reply budget",
           "t5A = rx_response(%s) + reply(%s)" % (a_resp, freply))
    else:
        fail("DS: the Final TX instant obeys the reply budget",
             "t5A=%s rx_response=%s reply=%s" % (t5a, a_resp, freply))


# ---------------------------------------------------------------------------
# Remediation checks (R04/R05/R07/R08/R09/R14)
# ---------------------------------------------------------------------------


def check_config(req, out):
    label = "R09 config"
    cfg = out.get("configuration")
    if not isinstance(cfg, dict) or not cfg:
        fail(label + ": configuration block is present", "got %r" % (cfg,))
        return
    executed = cfg.get("executed_fields")
    profile = cfg.get("profile_fields")
    mapping = cfg.get("mapping")
    if not isinstance(executed, list) or not executed or \
            not isinstance(profile, list) or not profile:
        fail(label + ": executed and profile field lists are present",
             "executed=%s profile=%s" % (len(executed or []), len(profile or [])))
        return

    want_cfg = hashlib.sha256(canonical_fields(executed).encode()).hexdigest()
    if out.get("config_sha256") == want_cfg:
        ok(label + ": config_sha256 == sha256(executed_fields, canonical form)", want_cfg)
    else:
        fail(label + ": config_sha256 == sha256(executed_fields, canonical form)",
             "output=%r recomputed=%s" % (out.get("config_sha256"), want_cfg))
    want_prof = hashlib.sha256(canonical_fields(profile).encode()).hexdigest()
    if out.get("profile_sha256") == want_prof:
        ok(label + ": profile_sha256 == sha256(profile_fields, canonical form)", want_prof)
    else:
        fail(label + ": profile_sha256 == sha256(profile_fields, canonical form)",
             "output=%r recomputed=%s" % (out.get("profile_sha256"), want_prof))

    emap = kv_map(executed)
    if isinstance(mapping, list) and mapping:
        rows = {r.get("field"): r for r in mapping if isinstance(r, dict)}
        need = ["session.protocol", "session.local_address", "session.peer_address",
                "session.session_id", "session.sequence_modulus"]
        missing = [f for f in need if f not in rows]
        if missing:
            fail(label + ": requested->effective->core mapping covers the identity",
                 "missing rows: %s" % missing)
        else:
            ok(label + ": requested->effective->core mapping covers the identity",
               "%d rows" % len(mapping))
        proto_row = rows.get("session.protocol") or {}
        if (proto_row.get("requested") == req.get("protocol")
                and proto_row.get("effective") == req.get("protocol")):
            ok(label + ": mapping.protocol requested==effective==request", req.get("protocol"))
        else:
            fail(label + ": mapping.protocol requested==effective==request",
                 json.dumps(proto_row))
    else:
        fail(label + ": requested->effective->core mapping is present", "got %r" % (mapping,))

    checks = [
        ("core.a.protocol", req.get("protocol")),
        ("core.b.protocol", req.get("protocol")),
        ("core.a.role", "initiator"),
        ("core.b.role", "responder"),
        ("core.a.local_address", str(req["endpoints"]["a"]["local_address"])),
        ("core.a.peer_address", str(req["endpoints"]["a"]["peer_address"])),
        ("core.b.local_address", str(req["endpoints"]["b"]["local_address"])),
        ("core.b.peer_address", str(req["endpoints"]["b"]["peer_address"])),
        ("core.a.session_id", str(req["session"]["session_id"])),
        ("core.b.session_id", str(req["session"]["session_id"])),
        ("core.a.sequence_modulus", str(req["session"]["sequence_modulus"])),
        ("core.a.initial_sequence", str(req["session"]["initial_sequence"])),
        ("core.a.session_generation", str(req["session"]["session_generation"])),
        ("core.b.session_generation", str(req["session"]["session_generation"])),
    ]
    bad = ["%s=%r want %r" % (p, emap.get(p), w) for p, w in checks
           if str(emap.get(p)) != w]
    if bad:
        fail(label + ": executed core fields match the requested values", "; ".join(bad))
    else:
        ok(label + ": executed core fields match the requested values",
           "%d executed fields cross-checked" % len(checks))
    for name in ("a", "b"):
        got = parse_domain_string(emap.get("core.%s.local_domain" % name))
        if same_domain(got, expected_domain(req, name)):
            ok(label + ": executed core.%s.local_domain is the requested domain" % name,
               emap.get("core.%s.local_domain" % name))
        else:
            fail(label + ": executed core.%s.local_domain is the requested domain" % name,
                 "got=%r want=%s" % (emap.get("core.%s.local_domain" % name),
                                     expected_domain(req, name)))
    for path in ("core.a.result_queue_capacity", "core.a.exchange_timeout_ticks"):
        if path in emap:
            ok(label + ": executed field %s is present" % path, emap[path])
        else:
            fail(label + ": executed field %s is present" % path, "missing")


def check_plans(req, out):
    label = "R08 plan"
    for name in ("a", "b"):
        ep = endpoint(out, name)
        if not ep:
            fail(label + ": endpoint %s present" % name, "missing")
            continue
        dom = expected_domain(req, name)
        cal_id = req["calibration"]["id"]
        for fr in tx_frames(ep):
            ftype = frame_type(fr)
            ctx = "%s %s tx" % (name, ftype)
            plan = fr.get("plan")
            if not isinstance(plan, dict):
                fail(label + ": %s carries a numeric plan" % ctx, "plan=%r" % (plan,))
                continue
            try:
                q = as_int(plan.get("quantised_instant_ticks"))
                mo = as_int(plan.get("marker_offset_ticks"))
                ca = as_int(plan.get("calibrated_air_ticks"))
                cmd = as_int(plan.get("command_time_ticks"))
                corr = as_int(plan.get("applied_corrections"))
            except (TypeError, ValueError) as exc:
                fail(label + ": %s plan tick fields parse" % ctx, str(exc))
                continue
            if ca == q + mo:
                ok(label + ": %s calibrated_air == quantised_instant + marker_offset" % ctx,
                   "%d == %d + %d" % (ca, q, mo))
            else:
                fail(label + ": %s calibrated_air == quantised_instant + marker_offset" % ctx,
                     "%d != %d + %d" % (ca, q, mo))
            if 0 <= mo and 0 <= q and 0 <= cmd <= q:
                ok(label + ": %s plan ticks ordered (0<=command<=quantised)" % ctx,
                   "cmd=%d q=%d marker=%d" % (cmd, q, mo))
            else:
                fail(label + ": %s plan ticks ordered (0<=command<=quantised)" % ctx,
                     "cmd=%d q=%d marker=%d" % (cmd, q, mo))
            if plan.get("valid") is True and plan.get("internally_consistent") is True:
                ok(label + ": %s plan valid and internally consistent" % ctx,
                   "identity_holds=%s" % plan.get("identity_holds"))
            else:
                fail(label + ": %s plan valid and internally consistent" % ctx,
                     "valid=%r consistent=%r note=%r" % (
                         plan.get("valid"), plan.get("internally_consistent"),
                         plan.get("consistency_note")))
            got_dom = {"name": plan.get("domain_name"),
                       "tick_rate_hz": plan.get("tick_rate_hz"),
                       "epoch_id": plan.get("epoch_id"),
                       "timestamp_bits": plan.get("timestamp_bits")}
            if same_domain(got_dom, dom):
                ok(label + ": %s plan domain is the endpoint's declared domain" % ctx,
                   plan.get("domain"))
            else:
                fail(label + ": %s plan domain is the endpoint's declared domain" % ctx,
                     "got=%r want=%s" % (got_dom, dom))
            if plan.get("source") == "scheduled_calibrated":
                ok(label + ": %s plan source is scheduled_calibrated" % ctx, plan.get("source"))
            else:
                fail(label + ": %s plan source is scheduled_calibrated" % ctx,
                     "got %r" % plan.get("source"))
            if plan.get("calibration_id") == cal_id:
                ok(label + ": %s plan cites the calibration in force" % ctx, cal_id)
            else:
                fail(label + ": %s plan cites the calibration in force" % ctx,
                     "got %r want %r" % (plan.get("calibration_id"), cal_id))
            if (corr & REQUIRED_RMARKER_TX_CORRECTIONS) == REQUIRED_RMARKER_TX_CORRECTIONS:
                ok(label + ": %s plan records every RMARKER_TX correction" % ctx,
                   "%d (%s)" % (corr, plan.get("corrections_text")))
            else:
                fail(label + ": %s plan records every RMARKER_TX correction" % ctx,
                     "applied=%d text=%r" % (corr, plan.get("corrections_text")))
            text = plan.get("corrections_text") or ""
            if all(n in text for n in REQUIRED_TX_CORRECTION_NAMES):
                ok(label + ": %s correction text names the required stages" % ctx, text)
            else:
                fail(label + ": %s correction text names the required stages" % ctx,
                     "text=%r" % text)
            if as_int(fr.get("ticks")) == ca:
                ok(label + ": %s frame tick == plan calibrated_air" % ctx, str(ca))
            else:
                fail(label + ": %s frame tick == plan calibrated_air" % ctx,
                     "frame=%r plan=%d" % (fr.get("ticks"), ca))
            fld = PLAN_FRAME_FIELD.get(ftype)
            if fld is not None:
                wire = frame_ts(fr, fld)
                if wire is not None and as_int(wire) == ca:
                    ok(label + ": %s frame %s == plan calibrated_air" % (ctx, fld), str(ca))
                else:
                    fail(label + ": %s frame %s == plan calibrated_air" % (ctx, fld),
                         "%s=%r plan=%d" % (fld, wire, ca))


def check_identity(req, out, emap):
    label = "R04 identity"
    for name in ("a", "b"):
        ep = endpoint(out, name)
        if not ep:
            fail(label + ": endpoint %s present" % name, "missing")
            continue
        for kind, frames in (("tx", tx_frames(ep)), ("rx", rx_frames(ep))):
            if any("token" not in fr for fr in frames):
                fail(label + ": %s %s frames carry a token" % (name, kind), "missing token")
                continue
            toks = [as_int(fr["token"]) for fr in frames]
            if all(t > 0 for t in toks) and all(b > a for a, b in zip(toks, toks[1:])):
                ok(label + ": %s %s tokens positive and strictly increasing" % (name, kind),
                   str(toks))
            else:
                fail(label + ": %s %s tokens positive and strictly increasing" % (name, kind),
                     str(toks))
    for name in ("a", "b"):
        want = str(req["session"]["session_generation"])
        got = str(emap.get("core.%s.session_generation" % name))
        if got == want:
            ok(label + ": %s executed session generation matches the request" % name, got)
        else:
            fail(label + ": %s executed session generation matches the request" % name,
                 "executed=%r requested=%r (stale-generation binding)" % (got, want))


def check_conservation(out):
    label = "R05/R14 conservation"
    for name in ("a", "b"):
        ep = endpoint(out, name)
        c = counters_of(ep) if ep else None
        if not isinstance(c, dict):
            fail(label + ": endpoint %s exposes its counters" % name, "got %r" % (c,))
            continue
        try:
            accepted = int(c["accepted_exchanges"])
            terminal = int(c["terminal_results"])
            dropped = int(c["results_dropped"])
        except (KeyError, TypeError, ValueError) as exc:
            fail(label + ": endpoint %s counter fields present" % name, str(exc))
            continue
        if dropped == 0:
            ok(label + ": endpoint %s results_dropped == 0" % name, "0")
        else:
            fail(label + ": endpoint %s results_dropped == 0" % name, "got %d" % dropped)
        if terminal <= accepted:
            ok(label + ": endpoint %s terminal_results <= accepted_exchanges" % name,
               "%d <= %d" % (terminal, accepted))
        else:
            fail(label + ": endpoint %s terminal_results <= accepted_exchanges" % name,
                 "%d > %d" % (terminal, accepted))
        reported = ep.get("in_flight")
        if reported is None:
            reported = c.get("in_flight")
        if reported is not None:
            if accepted == terminal + int(reported):
                ok(label + ": endpoint %s accepted == terminal + in_flight" % name,
                   "%d == %d + %d" % (accepted, terminal, int(reported)))
            else:
                fail(label + ": endpoint %s accepted == terminal + in_flight" % name,
                     "%d != %d + %d" % (accepted, terminal, int(reported)))
        else:
            derived = accepted - terminal
            converged = ep.get("completion") not in (None, "not_complete")
            if derived < 0 or (converged and derived != 0):
                fail(label + ": endpoint %s accepted == terminal + derived in_flight "
                     "(in_flight not published)" % name,
                     "accepted=%d terminal=%d derived=%d completion=%r"
                     % (accepted, terminal, derived, ep.get("completion")))
            else:
                ok(label + ": endpoint %s accepted == terminal + derived in_flight "
                   "(in_flight not published)" % name,
                   "%d == %d + %d" % (accepted, terminal, derived))


def check_deadlines(req, out, emap):
    label = "R07 deadline"
    for name in ("a", "b"):
        ep = endpoint(out, name)
        if not ep:
            fail(label + ": endpoint %s present" % name, "missing")
            continue
        poll_rx = local_ticks(ep, "rx_marker_ticks", "poll")
        accept = 0 if ep.get("role") == "initiator" else (
            as_int(poll_rx) if poll_rx is not None else None)
        try:
            exchange = int(emap.get("core.%s.exchange_timeout_ticks" % name, 0) or 0)
            evidence = int(emap.get("core.%s.evidence_wait_ticks" % name, 0) or 0)
        except (TypeError, ValueError):
            exchange, evidence = 0, 0
        if exchange > 0 or evidence > 0:
            ok(label + ": endpoint %s states a finite termination budget" % name,
               "exchange=%d evidence=%d" % (exchange, evidence))
        else:
            fail(label + ": endpoint %s states a finite termination budget" % name,
                 "both exchange and evidence budgets are 0")
        if accept is not None and exchange > 0:
            ceiling = accept + exchange
            offenders = [a.get("deadline_ticks") for a in (ep.get("arm_rx") or [])
                         if int(a.get("deadline_ticks", 0)) <= 0
                         or int(a.get("deadline_ticks", 0)) > ceiling]
            if not offenders:
                ok(label + ": endpoint %s arm_rx deadlines never exceed the absolute "
                   "exchange deadline" % name,
                   "accept=%d exchange=%d ceiling=%d" % (accept, exchange, ceiling))
            else:
                fail(label + ": endpoint %s arm_rx deadlines never exceed the absolute "
                   "exchange deadline" % name,
                     "ceiling=%d offenders=%s" % (ceiling, offenders))
        reason = ep.get("terminal_failure_reason") or ep.get("failure_reason")
        if reason == "protocol_timeout":
            c = counters_of(ep) or {}
            if int(c.get("timeouts", 0)) >= 1:
                ok(label + ": endpoint %s timeout terminal counted" % name, "timeouts>=1")
            else:
                fail(label + ": endpoint %s timeout terminal counted" % name,
                     "failure_reason=protocol_timeout but timeouts=%r" % c.get("timeouts"))


def run_checks(req, out, request_bytes):
    """Run the whole check suite; return the list of failing checks."""
    del results[:]
    status = out.get("status") or {}
    if status.get("ok") is not True:
        fail("the demo reported a successful run", "status=%s" % json.dumps(status))
        return [r for r in results if r[1] == "FAIL"]

    check_common(req, out, request_bytes)
    cfg = out.get("configuration")
    if isinstance(cfg, dict) and cfg:
        check_config(req, out)
        emap = kv_map(cfg.get("executed_fields"))
    else:
        emap = {}
        fail("R09 config: configuration block is present", "got %r" % (cfg,))
    check_plans(req, out)
    check_identity(req, out, emap)
    check_conservation(out)
    check_deadlines(req, out, emap)
    check_wire_copies(out)
    proto = req.get("protocol")
    if proto == "ss":
        verify_ss(req, out)
    elif proto == "ds":
        verify_ds(req, out)
    else:
        fail("the request names a protocol", "got %r" % proto)
    return [r for r in results if r[1] == "FAIL"]


def _endpoint_with_estimate(out, req):
    expect = "a" if req.get("protocol") == "ss" else "b"
    for name in (expect, "a", "b"):
        ep = endpoint(out, name)
        if ep and ep.get("estimate_available") and (ep.get("tof") or {}).get("available"):
            return name
    return expect


def _mutation_specs(req, out):
    """Each mutation, applied to a deep copy, MUST make the verifier fail."""
    est = _endpoint_with_estimate(out, req)
    specs = []

    def tof_num(o):
        o["endpoints"][est]["tof"]["num"] = str(int(o["endpoints"][est]["tof"]["num"]) + 1)
    specs.append(("ToF numerator", tof_num))

    def frame_field(o):
        for fr in o["endpoints"]["a"]["frames"]:
            if fr.get("type") == "response" and fr.get("dir") == "rx":
                fr["fields"]["timestamps"]["t3B"] += 1
                return
        raise RuntimeError("no response rx frame at A")
    specs.append(("frame field (rx copy) t3B", frame_field))

    def frame_field_tx(o):
        for fr in o["endpoints"]["b"]["frames"]:
            if fr.get("type") == "response" and fr.get("dir") == "tx":
                fr["fields"]["timestamps"]["t3B"] += 1
                return
        raise RuntimeError("no response tx frame at B")
    specs.append(("frame field (wire) t3B", frame_field_tx))

    def provenance(o):
        o["endpoints"]["a"]["measurement_valid"] = True
    specs.append(("provenance measurement_valid", provenance))

    def yields_range(o):
        o["endpoints"]["a"]["yields_range"] = True
    specs.append(("provenance yields_range", yields_range))

    def peer_evidence(o):
        o["endpoints"]["a"]["peer_evidence"] = "peer_hardware_event"
    specs.append(("provenance peer_evidence", peer_evidence))

    def execution_mode(o):
        o["endpoints"]["a"]["execution_mode"] = "hardware"
    specs.append(("provenance execution_mode", execution_mode))

    def plan_identity(o):
        for ep_name in ("a", "b"):
            for fr in o["endpoints"][ep_name]["frames"]:
                if fr.get("plan"):
                    fr["plan"]["calibrated_air_ticks"] = \
                        str(int(fr["plan"]["calibrated_air_ticks"]) + 1)
                    return
        raise RuntimeError("no tx plan")
    specs.append(("plan identity", plan_identity))

    def plan_domain(o):
        for ep_name in ("a", "b"):
            for fr in o["endpoints"][ep_name]["frames"]:
                if fr.get("plan"):
                    fr["plan"]["tick_rate_hz"] = float(fr["plan"]["tick_rate_hz"]) * 2.0
                    return
        raise RuntimeError("no tx plan")
    specs.append(("plan domain", plan_domain))

    def token_reuse(o):
        txs = [f for f in o["endpoints"]["a"]["frames"] if f.get("dir") == "tx"]
        if len(txs) >= 2:
            txs[1]["token"] = txs[0]["token"]
        else:
            txs[0]["token"] = 0 if int(txs[0]["token"]) != 0 else 1
    specs.append(("token reuse", token_reuse))

    def exec_field(o):
        for row in o["configuration"]["executed_fields"]:
            if row["path"] == "core.a.protocol":
                row["value"] = "ss" if row["value"] != "ss" else "ds"
                return
        raise RuntimeError("no core.a.protocol executed field")
    specs.append(("executed config field", exec_field))

    def dropped(o):
        o["endpoints"]["b"]["counters"]["results_dropped"] = 1
    specs.append(("results_dropped", dropped))

    def conservation(o):
        o["endpoints"]["b"]["counters"]["terminal_results"] = \
            int(o["endpoints"]["b"]["counters"]["accepted_exchanges"]) + 1
    specs.append(("conservation terminal>accepted", conservation))

    def deadline(o):
        ar = o["endpoints"]["b"].get("arm_rx") or []
        if not ar:
            raise RuntimeError("no arm_rx")
        ar[0]["deadline_ticks"] = str(int(ar[0]["deadline_ticks"]) + 10 ** 12)
    specs.append(("deadline extension", deadline))

    return specs


def self_test(req, out, request_bytes):
    global _QUIET
    _QUIET = False
    print("== self-test: the pristine output must PASS ==")
    base = run_checks(req, out, request_bytes)
    if base:
        print("SELF-TEST FAILED: the pristine output already fails: %s"
              % [f[0] for f in base])
        return 1
    print("PASS (%d checks)" % len(results))
    missed = []
    for name, mutate in _mutation_specs(req, out):
        mutant = copy.deepcopy(out)
        try:
            mutate(mutant)
        except Exception as exc:  # fixture problem, not a verifier result
            print("SELF-TEST MUTATION ERROR (%s): %s" % (name, exc))
            missed.append(name)
            continue
        _QUIET = True
        caught = run_checks(req, mutant, request_bytes)
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
    ap.add_argument("--request", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--self-test", action="store_true",
                    help="also mutate a copy of the output and require the verifier to fail")
    args = ap.parse_args(argv)

    for path in (args.request, args.output):
        if not os.path.isfile(path):
            sys.stderr.write("error: %s does not exist\n" % path)
            return 2

    with open(args.request, "rb") as handle:
        request_bytes = handle.read()
    try:
        req = json.loads(request_bytes.decode("utf-8"))
        with open(args.output, "r", encoding="utf-8") as handle:
            out = json.load(handle)
    except (ValueError, OSError) as exc:
        sys.stderr.write("error: cannot read the request/output: %s\n" % exc)
        return 2

    if args.self_test:
        return self_test(req, out, request_bytes)

    failed = run_checks(req, out, request_bytes)
    print("=" * 72)
    print("%d/%d checks pass" % (len(results) - len(failed), len(results)))
    if failed:
        print("FAILED: " + ", ".join(f[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
