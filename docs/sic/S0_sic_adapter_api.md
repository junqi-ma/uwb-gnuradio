# S0 SIC PDU adapter API (frozen)

`gr-uwb/include/gnuradio/uwb/uwb_preamble_sic.h` + `lib/uwb_preamble_sic.cc`,
class `gr::uwb::UwbPreambleSic` (bound as `uwb.preamble_sic`). It wraps the
math core (`docs/sic/S0_sic_core_api.md`); it is a THIN adapter and must not
reimplement the algorithm.

## Scheduler semantics (state explicitly in the header)
- Zero-stream `gr::block`, message handlers only. NO forecast / general_work.
- `in` handler: validate, hold an immutable shared ref to the IQ c32vector,
  enqueue. A SINGLE worker thread calls `sic_one`. Output pool and all scratch
  are allocated at `make()`; the hot path does not grow vectors and does no I/O.
- PMT wrapping / result construction DOES allocate: that cost is attributed to
  the `output_wrap_publish` adapter timer, never to the core.

## Ports
- `in`   : `pmt::cons(meta, c32vector)`. meta carries the sensing description
  (`code_index`/`sync_repetitions`/`sfd_mode`/`pulse_id`, window ids). These
  always describe SENSING; the comm profile is block config and must never
  overwrite them.
- `out`  : `pmt::cons(meta', c32vector)` — exactly ONE per accepted window, in
  order. `meta'` = input meta + the SIC result keys below. The IQ is the cleaned
  window or, on bypass, the ORIGINAL window byte-for-byte.
- `status` : small dict per window (state, reason, counters delta).
- `i_hat` : `pmt::cons(meta', c32vector)` — reconstructed interference, emitted
  ONLY when `publish_reconstruction=true` (diagnostic; for NMSE vs truth).

## make()
```cpp
static sptr make(const std::string& sync_template_path,      // one shaped SYNC .cf32
                 const std::string& preamble_template_path,  // full comm preamble .cf32
                 size_t comm_code, size_t sync_reps,
                 size_t samples_per_symbol, double sample_rate,
                 size_t k_paths, size_t fit_reps, size_t validation_reps,
                 int64_t roi_lo, int64_t roi_hi, size_t coarse_stride,
                 const std::string& mode,        // "off" | "estimate" | "oracle"
                 double detect_threshold, double validate_threshold,
                 size_t in_flight_max /*8*/, size_t iq_budget_bytes /*64<<20*/,
                 size_t queue_capacity, bool publish_reconstruction = false);
```
Plus `make_from_templates(sync, preamble, ...)` for the Python path.

## Admission / backpressure
- `in_flight` = windows published but not yet terminated. Admit only if
  `in_flight < in_flight_max` AND `pending_bytes + window_bytes <= iq_budget_bytes`
  AND the queue has room. Otherwise the window is **bypassed in order with the
  original IQ unchanged** and counted as `missed_admission` (never silently
  dropped, never reordered). Emit a status event with reason.
- Config invalid / unknown mode / template missing → non-zero exit / `make()`
  failure (never silent defaults).

## Counters / lifecycle
`windows_offered/admitted/completed/failed/bypassed/dropped/invalid`,
`inflight_high_watermark()`, `queue_high_watermark()`, `drained()`, `drain()`.
`stop()` drains queued windows before joining the worker; no leftover threads.
Every window has exactly one terminal state.

## Per-window `out` meta keys (in addition to the input meta)
```
sic_state, sic_detected, sic_committed, sic_partial, sic_ambiguous,
sic_bypass_reason, sic_uses_truth,
sic_detected_start, sic_cfo_hz, sic_n_paths, sic_support_lo, sic_support_hi,
sic_residual_corr,
handler_us, queue_wait_us, core_total_us, output_wrap_publish_us,
completion_latency_us,
sic_detect_coarse_us, sic_detect_fine_us, sic_cfo_us, sic_cir_us,
sic_path_select_us, sic_fractional_refine_us, sic_matrix_build_us,
sic_gain_fit_us, sic_reconstruct_us, sic_validate_commit_us
```

## Rev 2 additions (W3-R cleanup, frozen)
1. **Truth port is oracle-only.** In `estimate`/`off` the `truth` port must NOT
   be registered and no truth object may be stored on the block. Only
   `mode == "oracle"` registers/accepts `truth`.
2. **Expose the core knobs** in `make()`/`make_from_templates()` (after
   `publish_reconstruction`): `enable_fractional=true`, `enable_cfo=true`,
   `path_min_spacing=0`, `post_tail=100`, `max_window_samples=0` (0 = size from
   the first window; if a later window is larger, a ONE-TIME grow is allowed and
   counted, never per-window growth).
3. **Honest bounded queue.** Keep ordered bypass, but add `pending_bypass_max`
   (default = `in_flight_max + queue_capacity`) and `bypass_fifo_high_watermark`.
   If the pending-bypass FIFO would exceed the cap, emit a terminal
   `queue_overflow` state for that window (recorded, never a silent drop) and
   stop admitting. The upstream source MUST implement the credit/in-flight limit
   so `offered == admitted + missed_admission` under normal operation.
4. **Adapter timers measure through publish.** `output_wrap_publish_us` includes
   the `message_port_pub` call; `completion_latency_us` is the time from the
   window's planned publish to the terminal publish (also mirrored on `status`).

## QA required (`qa_uwb_preamble_sic.cc`)
- bypass bit-exactness: SIC off / no detection / wrong template ⇒ `out` IQ is
  byte-identical to `in`; the reconstructed `i_hat` (if published) is all zero.
- one `out` per admitted `in`, in order; offered == admitted + missed_admission.
- capacity: exceed `in_flight_max` / byte budget / queue ⇒ missed_admission
  recorded, no drop, no reorder; blocked upstream source reported as a miss, not
  masked by a lower input rate.
- invalid input (bad PMT / empty / wrong length) ⇒ counted invalid, no crash.
- stop/drain: completion counts equal admitted; `drained()` true; no residual
  thread.
- oracle mode sets `sic_uses_truth=true`; estimate mode never does.
