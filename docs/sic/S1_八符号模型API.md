# S1 SIC core API: 8-SYNC estimation, frozen model, full-nominal cancellation

Date: 2026-10-17. Base revision: `feature/sic-research` / `fb77e52` (S0) plus the
uncommitted S1 work in
`gr-uwb/include/gnuradio/uwb/uwb_preamble_sic_core.h`,
`gr-uwb/lib/uwb_preamble_sic_core.cc`,
`gr-uwb/lib/qa_uwb_preamble_sic_core.cc`,
`tools/sic/sic_reference.py`.

This document is the S1 contract for the math core. It supersedes the
`fit/validation/confirmed` coupling of `S0_sic_core_api.md` for the support and
commit semantics, and it keeps the S0 behaviour reachable as an **explicit
legacy mode**. The old NumPy vectors under `testdata/sic_preamble/golden/` were
emitted with S0 semantics; they are still used, but replayed in that explicit
legacy mode (see §7).

Implementation split (unchanged from S0): pure C++ math core (no GNU Radio types,
no scheduler, no I/O, no allocation in the hot path) + the separate PDU adapter
`uwb_preamble_sic.*`. S1 adds **no new block** and reuses the existing PDU
adapter as recommended by the S1 instruction §5.

## 1. The four ranges (S1 §3)

| range | definition | who may read it |
|---|---|---|
| acquisition / search | the public coarse ROI (`roi_lo`,`roi_hi`) inside the **visible prefix** | estimation (detection only) |
| training | `[origin, origin + fit_reps*sps)`, clipped to the visible prefix | estimation (CFO / CIR / paths / fractional / gain / fit_residual) |
| cancellation | predicted nominal support ∩ the permitted ROI | `apply_model` (frozen model only) |
| evaluation | the retained symbols after training | never fed back into training or the default gate |

The injected comm start, CFO, paths, ISR and interferer IQ are **never** inputs to
estimation. `estimate_model` takes only `y`, the template and the config.

## 2. Visibility and strict-prefix rule

`SicConfig::observed_end_sample` is the estimation visibility limit:

* `0` (default) — the whole `sic_one` window is visible. This is the legacy
  caller contract and is what the PDU adapter currently uses.
* `> 0` — only `y[0, min(observed_end, n))` may be read by **any** estimation
  stage. `observed_end` is also clamped to `n`.
* `< 0` — invalid; the call fails closed (`invalid_observed_end`).

This limit applies to estimation only. `apply_model` (and therefore the
cancellation of the remaining preamble) always covers the whole `[0, n)` window.

### 2.1 Strict-prefix enforcement points (estimation)

All in `uwb_preamble_sic_core.cc::estimate_impl`; every item is exercised by the
`test_sic_s1_prefix_isolation` / `test_sic_s1_prefix_guard_page` QA cases:

1. **Detection** — `norm_at(d)` returns `0` unless `0 <= d` and
   `d + sync_len <= visible_end`; the search ROI is additionally clipped to
   `visible_end - sync_len + 1`.
2. **Self-energy prefix** — the `|y|^2` prefix is only filled up to
   `min(prefix_capacity, visible_end)`.
3. **Walk-back / repetition confirmation** — a repetition is counted only when
   its whole correlation row is inside the prefix *and* it is inside the
   training span (`k < fit_reps`). A row that would cross the prefix is dropped
   (counted as unconfirmed), never read.
4. **CFO** — only phase pairs between training repetitions
   (`k, k+1 < fit_reps`) whose norms are confirmed. The S0 estimator averaged
   over `fit_reps + validation_reps`; S1 never reads validation repetitions here.
5. **CIR** — a tap is computed only when `pos >= 0` and
   `pos + sync_len <= min(origin + fit_reps*sps, visible_end)`.
6. **Fractional refine / matrix build / gain fit / fit_residual** — the LS rows
   are the first `Mfit = min(fit_len, visible_end - origin)` samples of the
   training span (`training_window_unavailable` when `Mfit < n_paths`).
7. **Online-validation gate** (only when `online_validation_reps > 0`) — the
   holdout `[origin+fit_len, +online_validation_reps*sps)` must end at or before
   `visible_end`, otherwise `validation_window_unavailable`. It is an *online*
   gate, not a retrospective one.

The detection seed is the public coarse ROI, never the simulated comm start.

## 3. Config additions

```cpp
enum class SicSupportMode : uint8_t { ConfiguredNominal = 0, ConfirmedLegacy = 1 };

struct SicConfig {
    // ... S0 fields unchanged ...
    SicSupportMode support_mode = SicSupportMode::ConfiguredNominal;
    int64_t observed_end_sample = 0;    // 0 => whole window visible
    size_t  online_validation_reps = 0; // 0 => NO holdout wait before cancelling
    double  fit_residual_max = 0.8;     // calibration-fixed sanity ceiling
};
```

Domain rules (fail-closed; `prepare_sic_core` and the runtime both check them):

* `support_mode` must be a defined enumerator (checked **before** any decision
  uses it; the mapping uses no `switch default`).
* `fit_residual_max` must be finite and in `(0, 1]`.
* `online_validation_reps <= validation_reps` (the prepared holdout capacity) and
  `fit_reps + online_validation_reps <= kMaxReps`.
* `observed_end_sample >= 0`.

`fit_residual_max` rationale (calibration-fixed at 0.8, not tuned on acceptance
windows): the metric is the normalized *training* residual
`||y - i_hat||^2 / ||y||^2`. A perfect cancellation of an ISR = 0 dB interferer
still leaves ~0.5 (the sensing signal is inside the residual), while a misaligned
or wrong-template fit leaves ~1.0. 0.8 therefore separates "something was fitted"
from "nothing was fitted" without rejecting the ISR = 0 dB cells.

## 4. SicModel (frozen, hashable)

```cpp
struct SicModel {
    bool valid; SicState state; const char* status; const char* reason;
    uint64_t model_id;              // FNV-1a-64 over the fields below + hashes
    uint64_t template_hash;         // FNV-1a-64 of the prepared template bytes
    uint64_t config_hash;           // FNV-1a-64 of the estimation-relevant config
    int64_t origin; double cfo_hz; size_t n_paths;
    double tau_abs[kSicMaxPaths]; std::complex<float> gain[kSicMaxPaths];
    double gain_condition; double fit_residual;
    int64_t training_lo, training_hi, observed_end_sample, estimation_ready_sample;
    int64_t predicted_support_lo, predicted_support_hi;
    const char* support_source;     // "configured_nominal_prediction"|...|"none"
    int64_t sub_lo, sub_hi;
    size_t training_reps_confirmed, nominal_sync_reps, samples_per_symbol;
    double sample_rate; int64_t phase_reference_sample;
};
```

* `origin` / `tau_abs[k]` are absolute work samples.
* `gain[k]` are complex coefficients in the `phase_reference_sample` frame
  (that anchor is `origin`):
  `i_hat[n] = exp(j*w*(n - phase_reference_sample)) * sum_k gain[k] * preamble(n - tau_abs[k])`,
  `w = 2*pi*cfo_hz/sample_rate`.
* After the model is frozen nothing in `apply_model` updates CFO, delays, gains
  or `n_paths` from the data it sees: **no re-fit, no online PLL, no SFO
  tracking**.
* `model_id` is the FNV-1a-64 of the frozen field set (all `kSicMaxPaths` slots,
  the support/region bounds, the hashes and `support_source`). Inside one
  implementation it is an exact data-dependency check; it is **not** a
  cross-language oracle — compare fields explicitly.
* `estimation_ready_sample == training_hi`: the last sample the estimation
  actually required from the prefix (equals `origin + fit_reps*sps` when the
  prefix is long enough).

## 5. Support semantics (S1-01)

`predicted_support_lo = max(0, origin)` in both modes.

| mode | `predicted_support_hi` | `support_source` |
|---|---|---|
| `ConfiguredNominal` (default) | `min(n, origin + sync_reps*sps + post_tail)` | `configured_nominal_prediction` |
| `ConfirmedLegacy` | `min(n, origin + confirmed*sps + post_tail)` | `confirmed_repetition_legacy` |

The nominal support is a **prediction from the public profile**, not a measured
64-SYNC boundary, and it is **not clamped to the confirmed/validated repetition
count**. That decouples the subtraction length from the training length (the
S1-01 defect: the old core derived the support from the confirmed count, so an
8-SYNC training could only cancel ~8-16 symbols).

Permitted cancellation region:

* with a sensing `sub_roi` configured: `predicted_support ∩ sub_roi` (the gate
  requires `overlap >= max(1, sub_min_overlap)`, else bypass
  `outside_sensing_roi`);
* without `sub_roi`: the legacy mask shape `predicted_support ∩ search ROI`.
  `ConfiguredNominal` uses the **configured public ROI** so the cancellation
  region never inherits the visibility clip; `ConfirmedLegacy` reproduces S0
  exactly, including its `n - sync_len + 1` correlation-row clip.

### 5.1 Commit gate

Common: detected, `training_reps_confirmed >= 2`, finite model, `cond2(A) < 1e6`,
config/capacity checks, and the `sub_roi` overlap gate.

| mode | extra gate |
|---|---|
| `ConfiguredNominal`, `online_validation_reps == 0` (**default**) | `fit_residual <= fit_residual_max`. **No holdout wait**; `residual_corr = NaN` by design. |
| any mode, `online_validation_reps > 0` | the S0 disjoint-holdout residual gate over `[origin+fit_len, +online_validation_reps*sps)`, inside the visible prefix. `ConfiguredNominal` applies **both** gates. EXPLICIT conservative mode; the observation latency is counted. |
| `ConfirmedLegacy` | the holdout gate above (no `fit_residual` gate) — S0 gate set. |

Not reaching the gate never writes a partial output: the result is `Bypassed`
with `out == y` byte-for-byte.

## 6. API

```cpp
UWB_API bool estimate_model(const std::complex<float>* y, size_t n,
                            int64_t observed_end, const SicTemplates&,
                            const SicConfig&, SicScratch&, SicModel&,
                            SicTimings* timings = nullptr,
                            SicResult* diag = nullptr);

UWB_API bool apply_model(const SicModel&, const std::complex<float>* y, size_t n,
                         int64_t offset, const SicTemplates&, const SicConfig&,
                         SicScratch&, std::complex<float>* out,
                         std::complex<float>* i_hat,
                         SicTimings* timings = nullptr);

UWB_API bool sic_one(const std::complex<float>* y, size_t n,
                     const SicTemplates&, const SicConfig&, SicScratch&,
                     SicResult&, const SicTruth* truth = nullptr);
```

`estimate_model` returns `false` only when no defined model could be produced
(null `y`, `n == 0`, unprepared scratch, template/geometry mismatch); otherwise
it returns `true` with a defined `SicModel` (valid or not) whose
`state/status/reason` explain the outcome. `diag`, when given, receives the
`SicResult` diagnostics; its caller-owned `out`/`i_hat` pointers are preserved.

`apply_model` semantics: `y` is the chunk starting at **absolute** sample
`offset`; `out[i] = y[i] - i_hat[offset+i]` only inside
`model.[sub_lo, sub_hi)`, and `out[i] = y[i]` bit-identically everywhere else.
`i_hat` receives the reconstruction over the model's support intersected with the
chunk. Fail-closed: a null `out`, an unprepared scratch, a template/geometry
mismatch, an invalid model or an over-capacity region all leave `out == y`. It
returns `false` only when `out == nullptr`, or when `n > 0 && y == nullptr`.

### 6.1 Chunk invariance (hardware-loop shape)

`i_hat` is computed on a **fixed absolute block grid** (`kReconBlock = 4096`
samples, or `cap_region` when smaller). The block bounds, the block length and
the reconstruction buffers (`scratch.acc[0..len)`, `scratch.ihat[0..len)`,
`scratch.rot_f`) are all model-determined, so every block is rebuilt with the
same VOLK calls, the same lengths and the same absolute CFO phase, independent of
how the caller chunks the window. The CFO phase is anchored absolutely at
`phase_reference_sample` on a block grid aligned to absolute phase multiples
(`fill_rot_f_abs`), so there is **no cross-chunk recurrence state**.

Measured (`test_sic_s1_chunk_invariance`, 70 024-sample window): whole window vs
47 fixed chunks vs 34 random chunks gives **bit-identical** `out` and `i_hat`
(0 differing samples). Documented tolerance: exact; a platform that ever differs
prints the differing-sample counts.

### 6.2 sic_one composition

```
visible_end = observed_end>0 ? min(observed_end,n) : n
estimate_impl(y, n, visible_end, ...)            -> SicModel + SicResult
if model.valid:
    apply_model(model, y[0, visible_end))        # backfill the observed prefix
    apply_model(model, y[visible_end, n))        # forward clean
```

This is the "first 8 symbols buffered, then retro-cleaned, then forward clean"
structure of the S1 instruction §1/§5, expressed with the same `apply_model`
entry point the hardware loop would use.

## 7. Legacy equivalence

`support_mode = ConfirmedLegacy` + `online_validation_reps = validation_reps`
+ `observed_end_sample = 0` reproduces the S0 **support** (confirmed count over
`fit_reps + validation_reps`), the S0 **cancellation mask** (including the
`n - sync_len + 1` clip) and the S0 **holdout commit gate**.

It does **not** reproduce the S0 CFO/CIR estimators: S1 keeps every estimation
read inside the training span in *all* modes (instruction §2/§4.1). The stored
S0 golden vectors (`testdata/sic_preamble/golden/`, emitted by the S0 reference)
therefore differ in `i_hat`/`out` by the measured 8-symbol CFO effect; the QA
replays them in the explicit legacy mode and compares `i_hat`/`out` with the
widened NMSE tolerance `max(meta, 5e-3 / 2e-2)` while keeping the hash, the array
set, the detected mask and the byte-exactness outside the mask strict. Measured
legacy-replay NMSE (`i_hat`, `out`):

| case | NMSE(i_hat) | NMSE(out) |
|---|---:|---:|
| int_single | 1.73e-07 | 1.13e-06 |
| int_four | 6.00e-07 | 3.71e-06 |
| frac | 2.66e-03 | 1.65e-02 |
| cfo | 4.72e-06 | 3.11e-05 |
| k_sweep | 6.00e-07 | 3.71e-06 |
| cross_model | 2.46e-03 | 1.43e-02 |

The C++ and the NumPy reference agree on these values to the printed digits.

## 8. Timings and scratch

New `SicTimings` fields (sequential; **never** summed into `core_total_us` or
into the stage children): `estimate_total_us`, `model_prepare_us`, `apply_us`,
`backfill_us`, `apply_forward_us`. `apply_us ≈ backfill_us + apply_forward_us`
(the two are wall-measured in `sic_one`, the first is the sum of the internal
apply timings); `model_prepare_us` is inside `estimate_total_us`.

Measured on one 64-SYNC / `sps=1016` window (`n = 70024`), `k_paths=4`,
`fit_reps=8`, single path, this host (3 runs):

| stage | µs |
|---|---:|
| core_total | 1681–1948 |
| estimate_total | 1293–1543 |
| detect_coarse / detect_fine | 533–556 / 16 |
| cfo | 2–3 |
| cir | 198–205 |
| path_select | 0–1 |
| fractional_refine | 354–570 |
| matrix_build | 76–90 |
| gain_fit | 39–46 |
| model_prepare (fit_residual) | 51–55 |
| apply | 320–376 |
| backfill | 47–71 |
| apply_forward | 273–306 |
| reconstruct (both passes) | 265–309 |

Peak scratch for that config: **22 157 872 bytes (~21.1 MiB)**, unchanged from
S0 — S1 adds no buffer (only two scalar hashes and the fixed block loop). It is
still dominated by the 32-phase polyphase preamble
(`32 * 65032 * 8 B = 16.6 MB`).

## 9. QA (all in `qa_uwb_preamble_sic_core.cc`)

Existing S0 cases are kept; the ones whose *expectations* changed are marked.
New S1 cases (all measure the **actual output**, not only the reconstruction):

| case | what it checks | result |
|---|---|---|
| `test_sic_s1_prefix_isolation` | 3 independent tail variants (additive mutation, zeroed, unrelated noise) leave **every** model field + `model_id` identical | identical |
| `test_sic_s1_prefix_guard_page` | estimation under `mmap` with `PROT_NONE` 64 samples past `observed_end` (slack for vectorised over-read) | identical model |
| `test_sic_s1_cfo_isolation` | post-prefix +80 kHz phase jump: model identical; independent-region actual suppression drops 72.57 → −3.08 dB | as expected |
| `test_sic_s1_full_support_and_tail` | support = 65 124 samples = `64*sps + post_tail` (not 8/16/60 symbols), coverage 1.000; actual suppression all/indep/last-4-SYNC = 59.31 / 58.74 / 54.63 dB; legacy mode last-4 = 0.00 dB; `out == y - i_hat` recomputed from the raw arrays | as expected |
| ↳ nominal load | ISR 10 dB + SNR 30 dB co-channel signal: last-4 14.57 dB, indep 18.78 dB (8-symbol CFO-limited; reported, not a target) | measured |
| `test_sic_s1_chunk_invariance` | whole vs fixed vs random chunks bit-identical (`out` and `i_hat`) | 0 diffs |
| `test_sic_s1_preamble_boundary` | start ±3 samples, fractional delays, window clipping, short apply chunk; nominal vs measured support (nominal 65 124 vs measured 20 328 samples) | committed, 57.4–58.7 dB |
| `test_sic_s1_extrapolation_mismatch` | matched / short preamble / burst / phase jump / time-varying gain: over-subtraction and target damage **measured** | matched: cancel 41.95 dB, damage −31.6 dB; mismatches: damage 3.7–14.3 dB, cancel −5.6…+6.6 dB |
| `test_sic_s1_bypass_untrusted_model` | invalid model, no interference, chunk outside the mask, disjoint `sub_roi` | all `out == y` |
| `test_sic_core_gain_fit_ill_conditioned` | R05 counterexample + near-collinear / scale / permutation cases (numeric safety, kept) | passing |

An `-fsanitize=address,undefined` build of the core + this QA file (including the
S1 golden set and the guard-page case) reports **no ASan/UBSan diagnostics**
(measured 2026-10-17).

Changed S0 expectations (documented, not silently rewritten):

* `test_sic_core_integer_single_path` — the byte-exactness mask is now
  `predicted_support ∩ search ROI` (nominal), so it extends to 64 symbols instead
  of 16.
* `test_sic_core_gain_condition_semantics`, `test_sic_core_adjacent_path_conditioning`
  — the default path has no holdout, so `residual_corr` is NaN by design and the
  model-quality assertion uses `fit_residual <= fit_residual_max`.
* `test_sic_core_timings_no_double_count` — extended with the S1 decomposition
  invariants.
* `test_sic_core_golden_crosscheck` — S0-era vectors replayed in explicit legacy
  mode with the documented tolerance (§7).

## 10. NumPy reference sync

`tools/sic/sic_reference.py` now mirrors S1: `estimate_model`-equivalent
`_estimate_model` (strict prefix, training-only CFO/CIR, nominal support,
`fit_residual` gate, optional holdout), `_apply_model` (masked subtraction,
chunk-invariant), and `estimate_sic` = estimate + backfill/forward apply. Same
new config keys, same support-source strings.

`--self-test` adds `_s1_self_test`: prefix isolation (all fields), full support,
chunk invariance (bit-identical), untrusted-model bypass, and actual tail
suppression on a 64-SYNC vector. Measured:

```
PASS  s1  support=8192 src=configured_nominal_prediction fit_res=0.0863 |
ISR40/SNR60 tail4=33.02 indep=37.16 dB |
ISR10/SNR30 tail4=3.10 indep=7.18 dB cfo=15206 Hz (8-symbol CFO limited) |
prefix_iso=True chunk=True bypass=True
```

The golden cross-check still runs and passes with the legacy replay described in
§7.  Running the *same* check against freshly emitted **S1** vectors
(`python3 tools/sic/sic_reference.py --emit-golden --golden-dir <tmp>` +
`UWB_SIC_GOLDEN_DIR=<tmp>`) is a direct cross-implementation check of the S1
semantics: the C++ core reproduces the reference `i_hat` at NMSE
1e-15…2e-8 and `out` at 1e-14…2e-7, with byte-exactness outside the mask
(measured 2026-10-17; the temp dir is not committed).

## 11. OPEN / not claimed here

1. **Adapter / CLI plumbing of the S1 modes — DONE (later in this session).**
   `uwb_preamble_sic.*` now exposes `support_mode` / `online_validation_reps` /
   per-window `observed_end_sample` and publishes the frozen model (id,
   fingerprint, ranges, support/sub, phase reference, cond2, fit_residual,
   per-path `tau` + complex gain, `apply.refit_count=0`); the CLI has
   `--sic-model-mode estimate8_frozen|confirmed_legacy`, `--sic-fit-reps 8`,
   `--sic-online-validation-reps 0`, `--sic-evaluation-reps` (offline only) and
   `--sic-cover-preamble` as a legacy alias. See
   `docs/sic/S1_八符号估计与全前导消除开发报告.md`.
2. **Measured effect / negatives / full demod / native / paced — REPORTED in
   `docs/sic/S1_八符号估计与全前导消除开发报告.md`** (acceptance matrix,
   negatives, CG400 native, benchmark profiler, asynchronous paced). This API
   document itself still reports only core-level measurements.
3. **8-symbol CFO accuracy is the depth limit of full-nominal cancellation.**
   With 8 training symbols the CFO phase estimate carries a per-symbol error from
   the co-channel signal and noise; over the remaining 56 symbols it accumulates,
   which caps the far-tail cancellation depth (measured here: 54.6 dB clean,
   14.6 dB at ISR 10 dB / SNR 30 dB on the C++ QA window; 33.0 → 3.1 dB on the
   reference vector). This is visible in the acceptance matrix: every failing
   cell is ISR = 0 dB with CFO = ±50 kHz. The support mechanism itself is
   measured correct; the depth is CFO-limited and would need a longer observation
   or an explicit refinement mode (not implemented, per §4.2 "no implicit PLL /
   no SFO tracking").
4. **MATLAB** was not run for this change (未实跑).
5. Peak scratch is unchanged, but the alternating SIMD resampler issue (S0) is
   untouched by this work.
