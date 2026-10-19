#!/usr/bin/env python3
"""QA for the paced SIC-only admission terminal.

The pacer already releases in-flight credit on ``sic.status`` when
``target_stage == "sic"``. ``--consumer sic`` is the CLI that selects that
terminal and builds no CIR or demod block. This test does not start a
flowgraph. A CIR-targeted pacer must keep the credit until the consumer
result, so a SIC status alone is not a false completion.
"""
from __future__ import annotations

import os
import sys

import pmt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gr-uwb", "apps"))
import uwb_sim_preamble_sic as sim  # noqa: E402


def _status(wid, state):
    meta = pmt.make_dict()
    meta = pmt.dict_add(meta, pmt.intern("event"), pmt.intern("window"))
    meta = pmt.dict_add(meta, pmt.intern("window_id"), pmt.from_long(wid))
    meta = pmt.dict_add(meta, pmt.intern("sic_state"), pmt.intern(state))
    meta = pmt.dict_add(meta, pmt.intern("sic_committed"), pmt.from_bool(True))
    meta = pmt.dict_add(meta, pmt.intern("core_total_us"), pmt.from_long(100))
    return meta


def _pacer(stage):
    return sim.PipelinedPacer(
        rate_hz=200.0, planned=1, in_flight_max=1,
        iq_budget_bytes=1 << 20, window_bytes=128,
        target_stage=stage)


def main():
    fails = 0

    def check(name, cond):
        nonlocal fails
        print("%-8s %s" % ("PASS" if cond else "FAIL", name), flush=True)
        if not cond:
            fails += 1

    check("stage sic", sim.paced_target_stage("sic") == "sic")
    check("stage cir", sim.paced_target_stage("cir") == "cir")
    check("stage both", sim.paced_target_stage("both") == "cir")
    check("stage demod", sim.paced_target_stage("demod") == "demod")
    try:
        sim.paced_target_stage("nope")
        check("stage unknown rejected", False)
    except ValueError:
        check("stage unknown rejected", True)

    sic = _pacer("sic")
    ok, reason = sic._hold(0, 10)
    check("sic hold", ok and reason is None and sic.in_flight == 1)
    sic.on_sic_status(_status(0, "committed"), 20)
    check("sic status releases credit",
          sic.completed == 1 and sic.in_flight == 0
          and sic.terminal_state[0] == "committed")

    cir = _pacer("cir")
    ok, _reason = cir._hold(0, 10)
    check("cir hold", ok and cir.in_flight == 1)
    cir.on_sic_status(_status(0, "committed"), 20)
    check("cir status keeps credit",
          cir.completed == 0 and cir.in_flight == 1
          and cir.sic_state[0] == "committed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
