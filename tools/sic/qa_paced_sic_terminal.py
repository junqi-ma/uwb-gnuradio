#!/usr/bin/env python3
"""QA for the paced SIC-only admission terminal.

The pacer already releases in-flight credit on ``sic.status`` when
``target_stage == "sic"``. ``--consumer sic`` is the CLI that selects that
terminal and builds no CIR or demod block. This test does not start a
flowgraph. A CIR-targeted pacer must keep the credit until the consumer
result, so a SIC status alone is not a false completion.

Deadline misses are counted on the first terminal only, for both the SIC
status path and the consumer path. A post later than one inter-arrival
period is ``source_late`` and counts as ``missed_submission`` once.
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


def _pacer(stage, planned=1):
    return sim.PipelinedPacer(
        rate_hz=200.0, planned=planned, in_flight_max=1,
        iq_budget_bytes=1 << 20, window_bytes=128,
        target_stage=stage)


def _consumer(wid, status="ok"):
    meta = pmt.make_dict()
    meta = pmt.dict_add(meta, pmt.intern("schedule_index"), pmt.from_long(wid))
    meta = pmt.dict_add(meta, pmt.intern("status"), pmt.intern(status))
    return pmt.cons(meta, pmt.PMT_NIL)


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

    # 200 Hz period is 5_000_000 ns. Deadline is 5000 us.
    late_ns = 5_001_000
    on_time_ns = 5_000_000

    sic_late = _pacer("sic")
    sic_late.planned_ns[0] = 0
    ok, _reason = sic_late._hold(0, 0)
    check("sic late hold", ok)
    sic_late.on_sic_status(_status(0, "committed"), late_ns)
    sic_late.on_sic_status(_status(0, "committed"), late_ns + 1_000_000)
    check("sic deadline counted once",
          sic_late.deadline_misses == 1 and sic_late.completed == 1
          and sic_late.in_flight == 0)

    sic_ontime = _pacer("sic")
    sic_ontime.planned_ns[0] = 0
    sic_ontime._hold(0, 0)
    sic_ontime.on_sic_status(_status(0, "committed"), on_time_ns)
    check("sic on-time is not a deadline miss",
          sic_ontime.deadline_misses == 0 and sic_ontime.completed == 1)

    cir_late = _pacer("cir")
    cir_late.planned_ns[0] = 0
    cir_late._hold(0, 0)
    cir_late.on_sic_status(_status(0, "committed"), late_ns)
    check("cir status does not count the deadline",
          cir_late.deadline_misses == 0 and cir_late.completed == 0
          and cir_late.in_flight == 1)
    cir_late.on_consumer_result(_consumer(0), late_ns)
    cir_late.on_consumer_result(_consumer(0), late_ns + 1_000_000)
    check("consumer deadline counted once",
          cir_late.deadline_misses == 1 and cir_late.completed == 1
          and cir_late.in_flight == 0)

    expired = _pacer("sic")
    expired.planned_ns[0] = 0
    expired._hold(0, 0)
    expire_at = expired.terminal_timeout_ns + late_ns
    expired._expire_stale(expire_at)
    expired._expire_stale(expire_at + 1)
    check("timeout is one deadline miss",
          expired.terminal_timeouts == 1 and expired.deadline_misses == 1
          and expired.completed == 1 and expired.in_flight == 0)

    period = _pacer("sic")._period_ns()
    check("period is 5 ms", period == 5_000_000)
    src = _pacer("sic", planned=2)
    src._t0_ns = 0
    src._start_ns = 0
    src.planned_ns[0] = 0
    src.planned_ns[1] = 0
    check("on-time post",
          src._note_submit(0, 0, period) and src.submit_state[0] == "submitted"
          and src.missed_submission == 0)
    check("repeat post ignored", src._note_submit(0, 0, period + 10) is False
          and src.missed_submission == 0)
    check("source late counts once",
          src._note_submit(1, 0, period + 1)
          and src.submit_state[1] == "source_late"
          and src.missed_submission == 1
          and src.miss_reasons["source_late"] == 1
          and src.submit_ns[1] == period + 1)
    check("repeat source late ignored",
          src._note_submit(1, 0, period + 50) is False
          and src.missed_submission == 1)
    rep = src.report()
    check("report publishes source_late",
          rep["source_late"] == 1 and rep["miss_reasons"]["source_late"] == 1
          and rep["target_load_met"] is False
          and rep["submit_late"]["source_late_n"] == 1)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
