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

    python3 tools/twr/verify_m1_b_protocol.py --request req.json --output out.json

Exit status 0 iff every check passes; non-zero with a clear diff otherwise.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from fractions import Fraction as F

C_MPS = 299792458.0  # exactly the fake link's kFakeLinkSpeedOfLightMps

results = []


def record(item, verdict, detail):
    results.append((item, verdict, detail))
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--request", required=True)
    ap.add_argument("--output", required=True)
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

    status = out.get("status") or {}
    if status.get("ok") is not True:
        fail("the demo reported a successful run",
             "status=%s" % json.dumps(status))
        print("=" * 72)
        print("0/%d checks pass" % len(results))
        return 1

    check_common(req, out, request_bytes)
    proto = req.get("protocol")
    if proto == "ss":
        verify_ss(req, out)
    elif proto == "ds":
        verify_ds(req, out)
    else:
        fail("the request names a protocol", "got %r" % proto)

    print("=" * 72)
    failed = [r for r in results if r[1] == "FAIL"]
    print("%d/%d checks pass" % (len(results) - len(failed), len(results)))
    if failed:
        print("FAILED: " + ", ".join(f[0] for f in failed))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
