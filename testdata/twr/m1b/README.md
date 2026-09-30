# M1-B golden set and QA coverage (Agent D)

Date: 2026-09-30. Branch: `feature/uwb-ds-twr`.

This directory holds the **independently derived** inputs and expected values for
the M1-B two-endpoint protocol QA. None of these numbers is produced by running
`EndpointCore` or the ToF formula; they come from the documented physical model
plus Python `fractions.Fraction`, and are re-checked by hand below.

Files:

- `event_cases.json` — the input scenarios (clock domains, planning offsets,
  ground-truth distance, seed).
- `expected_results.json` — the exact wire field values, intervals and ToF for
  each case, with the one-line derivation.
- `README.md` — this file: per-case derivation, units, quantisation, causal
  order, and the B01–B18 coverage table.

The C++ QA files that consume this set are
`gr-uwb/lib/qa_uwb_twr_core.cc` (unit) and `gr-uwb/lib/qa_uwb_twr_m1b_e2e.cc`
(two endpoint), driven by the header-only
`gr-uwb/lib/twr_m1b_test_support.h`. Those files hard-code the same numbers and
point back here; no C++ core output is used to build an expectation.

---

## 1. Units, quantisation, and the physical model

- **Tick domains.** Each physical endpoint owns its own `ClockDomain`
  (`name, tick_rate_hz, epoch_id, timestamp_bits`). A wire timestamp is a whole
  number of that endpoint's ticks. The protocol's common unit is **A ticks**
  (initiator), and `k = fA / fB` converts a B-domain interval into A ticks.
- **Quantisation.** The v1 frame carries 40-bit little-endian **integer** ticks
  only. A local instant with a sub-tick fraction cannot be put on the wire; the
  core refuses (or the QA avoids fractions). The fake transport quantises with
  `llround`:
  - rate change: `dst_ticks = llround(src_ticks / f_src * f_dst)`,
  - propagation: `prop = llround(distance_m / c * f_dst)`, with
    `c = 299792458.0 m/s`.
  All cases here are chosen so every conversion is **exact** (no rounding
  remainder), so the expected values are integers and the test does not depend
  on floating-point ties.
- **Ground truth isolation.** `distance_m` lives only in the `FakeTwrLink`
  configuration. The endpoint-visible `FakeRx` / `CoreEvent` carry no distance,
  no ToF, and no peer-private evidence (asserted structurally in
  `b16_*`).
- **Seed.** `seed = 1` for every case. The transport is not random; the seed is
  recorded only for scenario reproducibility (B18).

### 1.1 The deterministic TX-planning model (the "adapter")

The M1-B test adapter plans each transmit from the local activity that caused
it (this is what a real M2 PHY adapter will do):

```
Poll     air = begin_now_ticks      + poll_offset_ticks
Response air = <local Poll RX>      + respond_turnaround_ticks
Final    air = <local Response RX>  + final_turnaround_ticks
```

With one-way propagation `P` each way and a symmetric link, this makes **both**
SS and DS return exactly `P`, *independently of the turnarounds*:

```
SS:  t2 = t1+P, t3 = t2+DB, t4 = t3+P
     RA = t4-t1 = DB + 2P            =>  (RA - k*DB)/2 = P      (k=1)
DS:  t6 = t5+P, and with DA = t5-t4, DB = t3-t2
     RA = DB+2P, RB = DA+2P
     (RA*RB - DA*DB)/(RA+RB+DA+DB) = P                          (k=1)
```

The QA therefore uses `t3` and `t5` as free parameters (unequal reply intervals)
and still expects the same ToF — a check that the timestamps are used correctly,
not a fit to one number.

---

## 2. Per-case derivation

All values are exact rationals; `k = fA/fB`.

| case | req | proto | init | fA / fB (Hz) | k | distance | planning (begin,poll,resp,final) | wire fields | ToF (A ticks) | estimate at |
|---|---|---|---|---|---|---|---|---|---|---|
| `same_clock_ss` | B01,B02 | SS | A | 1e9 / 1e9 | 1/1 | 15 m | 1000,1000,500,300 | Resp t2B=2050,t3B=2550 | **50/1** | A only |
| `same_clock_ds` | B01,B02 | DS | A | 1e9 / 1e9 | 1/1 | 15 m | 1000,1000,500,300 | Resp 2050/2550; Fin 2000/2600/2900 | **50/1** | B only |
| `same_clock_ds_unequal_replies` | B02 | DS | A | 1e9 / 1e9 | 1/1 | 15 m | 1000,1000,**500**,**1300** | Resp 2050/2550; Fin 2000/2600/**3900** | **50/1** | B only |
| `same_clock_role_swap_ss` | B01 | SS | **B** | 1e9 / 1e9 | 1/1 | 15 m | 1000,1000,500,300 | Resp 2050/2550 | **50/1** | B only |
| `same_clock_role_swap_ds` | B01 | DS | **B** | 1e9 / 1e9 | 1/1 | 15 m | 1000,1000,500,300 | Resp 2050/2550; Fin 2000/2600/2900 | **50/1** | A only |
| `diff_rate_k_gt1_ss` | B02 | SS | A | 1e9 / 0.8e9 | 5/4 | 250·c/1e9 m | 0,100000,4000,700 | Resp t2B=80200,t3B=84200 | **250/1** | A only |
| `diff_rate_k_gt1_ds` | B02 | DS | A | 1e9 / 0.8e9 | 5/4 | 250·c/1e9 m | 0,100000,4000,700 | Resp 80200/84200; Fin 100000/105500/106200 | **250/1** | B only |
| `diff_rate_k_lt1_ss` | B02 | SS | A | 0.8e9 / 1e9 | 4/5 | 200·c/0.8e9 m | 0,100000,5000,800 | Resp t2B=125250,t3B=130250 | **200/1** | A only |
| `diff_rate_k_lt1_ds` | B02 | DS | A | 0.8e9 / 1e9 | 4/5 | 200·c/0.8e9 m | 0,100000,5000,800 | Resp 125250/130250; Fin 100000/104400/105200 | **200/1** | B only |

Each case in `expected_results.json` carries the exact one-liner, e.g.:

- `same_clock_ss`: `(RA - k*DB)/2 = (600 - (1/1)*500)/2 = 50/1`
- `same_clock_ds`: `(600*1/1*400 - 300*1/1*500)/(600 + 1/1*400 + 300 + 1/1*500) = 50/1`
- `same_clock_ds_unequal_replies`: `(600*1/1*1400 - 1300*1/1*500)/(600 + 1/1*1400 + 1300 + 1/1*500) = 50/1`
- `diff_rate_k_gt1_ss`: `(5500 - (5/4)*4000)/2 = 250/1`
- `diff_rate_k_gt1_ds`: `(5500*5/4*960 - 700*5/4*4000)/(5500 + 5/4*960 + 700 + 5/4*4000) = 250/1`
- `diff_rate_k_lt1_ss`: `(4400 - (4/5)*5000)/2 = 200/1`
- `diff_rate_k_lt1_ds`: `(4400*4/5*1500 - 800*4/5*5000)/(4400 + 4/5*1500 + 800 + 4/5*5000) = 200/1`

The `expected_results.json` `local_instants` are the fake-link arrival instants
for each endpoint (e.g. `same_clock_ds`: `t1A=2000, t2B=2050, t3B=2550,
t4A=2600, t5A=2900, t6B=2950`); these are what the driver forwards as RX
events, and they are what a physical receiver could observe.

### 2.1 Expected causal order (all success cases)

```
core A: Begin -> ArmRx(Response) + PrepareTx(Poll)
        TxPlanned(Poll) -> SubmitTx(Poll); Poll accepted
        Poll air t1, arrives B at t2
core B: RxFrame(Poll) -> PrepareTx(Response)
        TxPlanned(Response t3) -> SubmitTx(Response)   [DS: also ArmRx(Final)]
        Response air t3, arrives A at t4
core A: RxFrame(Response) -> [SS: estimate (RA-k*DB)/2]
                            [DS: PrepareTx(Final)]
        [DS] TxPlanned(Final t5) -> SubmitTx(Final); Final air t5, arrives B at t6
core B: [DS] RxFrame(Final) -> estimate (RA*k*RB-DA*k*DB)/(RA+k*RB+DA+k*DB)
```

No Response/Final exists without the matching valid RX. A dropped frame removes
the entire downstream branch (asserted in B03).

---

## 3. Build and run (out of tree; the coordinator owns `gr-uwb/build`)

```bash
# unit QA (ONE EndpointCore)
g++ -std=c++17 -Wall -Wextra -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN \
    -I gr-uwb/include -o /tmp/opencode/qa_core \
    gr-uwb/lib/uwb_twr_core.cc gr-uwb/lib/uwb_twr_fake_link.cc \
    gr-uwb/lib/qa_uwb_twr_core.cc -lboost_unit_test_framework
/tmp/opencode/qa_core

# two-endpoint QA
g++ -std=c++17 -Wall -Wextra -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN \
    -I gr-uwb/include -o /tmp/opencode/qa_e2e \
    gr-uwb/lib/uwb_twr_core.cc gr-uwb/lib/uwb_twr_fake_link.cc \
    gr-uwb/lib/qa_uwb_twr_m1b_e2e.cc -lboost_unit_test_framework
/tmp/opencode/qa_e2e

# ASAN+UBSan: add -fsanitize=address,undefined -g -O1
```

Observed (2026-09-30): `qa_core` 21 cases / 295 assertions, **all pass**;
`qa_e2e` 11 cases / 248 assertions, **all pass**. ASAN+UBSan clean (no reports
on either binary).

The single initially-red assertion (B04, §5.1) was a real product defect; the
coordinator fixed it (`uwb_twr_core.cc`) and the assertion was **not** relaxed.

---

## 4. B01–B18 coverage (FULL = asserted; PARTIAL/NOT explained)

| ID | status | where / what is actually asserted |
|---|---|---|
| B01 | **FULL** | `qa_uwb_twr_m1b_e2e.cc::b01_*` — SS and DS × A-init and B-init; both FSMs run (`accepted==1`, `terminal==1`); SS estimate only at initiator, DS only at responder; the other end has `estimate_available==false` and `tof.valid==false` (no copied distance); per-end transmit multiplicity asserted. |
| B02 | **FULL** | `b02_ss_k_greater_than_one`, `b02_ds_k_greater_than_one`, `b02_ss_k_less_than_one`, `b02_ds_unequal_reply_intervals_*` (unequal `DA != DB`, exact frame fields recomputed); `qa_uwb_twr_core.cc::b02_out_of_window_ratio_is_reported_not_used`; missing ratio refused at `configure` (`b15_configure_*`). Independent clock domains are genuinely different rates. |
| B03 | **FULL** | `b03_dropped_poll_*`, `b03_dropped_response_ss_*`, `b03_dropped_response_ds_*`, `b03_dropped_final_ds_*`. Dropped Poll ⇒ responder silent (`tx_prepared==0`, `accepted==0`); dropped Response ⇒ DS Final never produced; active end times out; every end conserves. |
| B04 | **FULL** | `qa_uwb_twr_core.cc::b04_wrong_fields_*` (wrong type/PAN/src/dst/session/seq then a legal frame advances), `b04_bad_fcs_*`, `b04_codec_*` (bad version, reserved flags, STS, truncated/FCS-carrying bytes). No advance/reply on any bad input. The one initial red assertion was a real product defect, now fixed — see §5.1. |
| B05 | **FULL** | `b05_duplicate_plan_and_duplicate_poll_do_not_double_tx` (no second Submit, duplicate counting), `b05_sequence_modulus_is_the_reuse_barrier_and_reset_clears_it` (5th Begin refused once `seq_issued==modulus`; Reset clears). |
| B06 | **FULL** | `b06_frame_fields_come_from_the_same_plan` (Response `t2B/t3B` equal the admitted RX and the same plan), `b06_bad_plan_token_domain_and_deadline_verdict_reject_without_submit` (missing record, wrong token, out-of-domain enum, infeasible deadline). |
| B07 | **FULL** | `b07_accepted_is_not_completion_and_rx_before_outcome_still_completes`, `b07_unresolved_evidence_with_a_deadline_fails_finitely`. |
| B08 | **FULL** | `b08_failure_outcomes_and_faults_fail_without_resend` — Late/Underflow/Cancelled and adapter SeqError/ChainBroken; explicit failure, no re-send, result not a range. |
| B09 | **FULL** | `b09_wire_tick_space_wrap_and_span_policy` — wrapping interval resolved; exactly half accepted; one beyond half refused (`IntervalNotFormable`); over max refused (`IntervalTooLong`); raw out of domain refused; binding max > half refused. The "no lossy integer-ns" half is inherited from the M1-A QA, not re-derived here. |
| B10 | **FULL** | `b10_local_reset_fails_inflight_and_old_events_cannot_revive`. |
| B11 | **FULL** | `b11_local_first_path_and_calibration_are_mandatory` — NotRecorded / Failed first path, missing application record, expired calibration all refuse with the strict-gate status, even with an otherwise-valid peer frame. |
| B12 | **FULL** | `b12_peer_claims_and_protocol_estimate_never_become_a_range` — unnamed convention, over-wide span, unbound claim, different message identity all refused; every range helper false; `static_assert` proves no conversion to `TofResult` / `AdmittedRangingInterval`. |
| B13 | **FULL** | `b13_bounded_results_refuse_a_new_begin_without_losing_a_terminal` — QueueFull refusal, terminal not dropped, control events not starved. |
| B14 | **FULL** | `b14_exactly_one_terminal_per_attempt_and_conservation_holds` — one terminal per attempt, repeated events after terminal ignored, `accepted == terminal + in_flight`, restart. |
| B15 | **FULL** | `b15_negative_tof_kept_signed_and_ds_zero_denominator_reported`, `b15_out_of_domain_event_has_no_action_and_never_throws`, `b15_configure_*` (out-of-domain enums). Kernel overflow asserted at the shared-kernel boundary. |
| B16 | **FULL** | `b16_endpoint_has_no_ground_truth_input_and_a_missing_frame_changes_the_outcome` (member-detection traits + causal deleted-Response), `b16_deleting_the_response_frame_changes_the_outcome` (e2e). |
| B17 | **PARTIAL** | The pure core + fake link compile and run with a plain `g++` (no GNU Radio / UHD include or link), which is the independence part. **NOT covered here:** the install consumer, the standalone consumer program and the Python demo/verifier — these are the coordinator's and Agent E's deliverables. |
| B18 | **PARTIAL** | `b18_same_scenario_is_bit_for_bit_reproducible` asserts same-seed/same-scenario identical results (ToF, statuses, transmit counts, frame fields). The old M0.1 / M1-A QAs are untouched by this work (no shared files, no expectation weakened), but were **not re-run here**; historical throughput was **not** measured here. |

---

## 5. Findings

### 5.1 Product defect (found by this QA, then FIXED by the coordinator): initiator wrong-sequence counted as a session error

- **File:** `gr-uwb/lib/uwb_twr_core.cc`, `Impl::on_rx_frame`, initiator branch
  (the code as delivered to QA):
  ```cpp
  if (ev.frame.session_id != cfg.session_id || !seq_matches(current_seq, ev.frame.seq)) {
      counters.frames_rejected_session++;
      return;
  }
  ```
- **Reproduction:** configure an SS initiator, `Begin`, then deliver a `Response`
  with the correct session/PAN/addresses but `seq = 1` (expected `0`).
  Output: `frames_rejected_seq=0 frames_rejected_session=1 state=poll_sent`,
  required `frames_rejected_seq==1`.
- **Resolution:** the coordinator split the check into two separate tests so a
  wrong sequence increments `frames_rejected_seq` and leaves
  `frames_rejected_session` untouched (commit `19bf2dc`). The QA assertion was
  **not** relaxed; `b04_wrong_fields_do_not_advance_and_a_legal_frame_does` now
  passes.

### 5.2 Interface deviation from the G0 state table (low severity): SS responder emits no `ArmRx`

The G0 table said a responder that accepts a Poll emits `ArmRx(Response 或
Final)`. The implementation emits `ArmRx(Final)` only for DS, and only after
the Response plan; the SS responder has no further air wait. **Resolution:**
the G0 table was corrected to match the implementation (an SS responder simply
emits `PrepareTx(Response)`).

### 5.3 Observation (then FIXED by the coordinator): `CoreConfig::evidence_wait_ticks` was unused

As delivered, `rg evidence_wait gr-uwb/lib/uwb_twr_core.cc` found no use: only
`exchange_timeout_ticks` bounded an unresolved transmit. **Resolution:** the
core now records an evidence deadline (`evidence_deadline_ticks`) whenever a
result is owed but a local transmit outcome has not converged, re-uses the
`ArmRx` action as the driver's deadline channel, and `on_deadline` reports a
`ProtocolTimeout` at that earlier bound. The behaviour remains finite, and a
test that leaves `evidence_wait_ticks == 0` keeps the old exchange-timeout
bound.

---

## 6. Not verified here

- Real PHY / FCS / FEC / waveform timestamp patch, native resampling, two RX
  routing, UHD radio adapter — out of M1-B scope.
- Install/standalone consumer and the Python demo/verifier (B17 remainder).
- Full historical regression and throughput (B18 remainder).
- Any claim of hardware ranging: every result here is a
  `simulation` / `wire_claim` protocol estimate (`measurement_valid == false`,
  `yields_range() == false`).
