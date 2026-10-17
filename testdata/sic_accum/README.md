# S2-A — accumulation-after-SIC sensing CIR evaluation (contract + operations)

This directory is the **frozen evaluation contract and operations README** for
S2-A (`docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`). The contract files
`operator.json` and `cells.json` are frozen on the **calibration** seed set
before the **acceptance** seed set is run; changing any value afterwards
requires a new run directory and an explicit note in the development report.

> 中文速览：本目录冻结 S2-A 的固定算子/ROI/seed/归档契约，并提供可复制的真实
> 命令。`operator.json`、`cells.json` 已冻结；本任务的最终结论以
> `docs/sic/S2A_累加后CIR开发报告.md` 为准。MATLAB 对照脚本已提供但**未实跑**。

## 1. Purpose and scope

S2-A answers one question: does the roughly-28 dB nominal raw-IQ suppression of
the sensing preamble **survive coherent accumulation** over `N` Ipatov SYNC
repetitions, does it protect the weak target, and does the post-SIC residual
decay like an incoherent (`1/N`) term or hit a coherent floor built from frozen
model / CFO error. S2-A **evaluates**; it does not add a detector and does not
optimise the SIC core.

The SIC branch always comes from the real GNU Radio PDU adapter
(`uwb.preamble_sic`, S1 `estimate8_frozen` default); only the offline metrics
after the adapter are NumPy. The S1 shared core/adapter is **not** modified by
S2-A.

## 2. What S2-A does NOT cover

* Native 491.52 MS/s work→native PDU 32/65, UHD, FPGA, hardware, TWR: not
  exercised. Runs declare this in `run_manifest.json` → `not_run.native`.
* Continuous 1 GS/s operation and any real-time/throughput acceptance claim:
  not claimed. Timing recorded in `summary.json` is labelled
  `diagnostic (no benchmark lock held)`.
* Full Pd/Pfa detection scan: not performed. Target quality uses the frozen
  target-protection thresholds only (§3).
* Re-running the S1 72-cell matrix: not required and not done.
* MATLAB acceptance: the script is provided but **not executed** (§10).

## 3. Frozen coordinate and metric contract

Source of truth: `operator.json` (copied verbatim into every run directory by
the app). Summary:

| item | value |
|---|---|
| work grid | 998.4 MS/s (`sample_rate_hz = 998400000.0`) |
| samples per SYNC symbol | `sps = 1016` |
| SYNC repetitions | `sync_reps = 64` (support = 64·1016 = 65024 samples) |
| sensing / comm code | 9 / 10 (different codes; overlap is **time overlap**) |
| SFD mode | `4z2` |
| fixed operator origin | `origin = 1997` (public local sensing plan; never the injected comm start) |
| template | code-9 bare sampled code (127 ternary chips → spread ×4 → 2 samples/pulse), `code_len_chips=127`, `chip_stride=8`, `spread_factor=4`, `samples_per_pulse=2` |
| gain normalisation | fixed `1/code_energy` with `code_energy = 64`; no per-window L2 / peak / per-branch scaling |
| phase rotation | `identity` (same for all three branches; no per-symbol / per-branch derotation) |
| tap grid | `[tap_lo, tap_hi) = [-32, 128)`, `ntaps = 160` |
| convention | **average**: `H_N = (1/N)·sum_r H[r]`; the sum counterpart is recorded as `H_sum = N·H_avg`, never re-declared |

Correlation definition (frozen string, identical to MATLAB `estimateCir` / the
C++ radar CIR estimate direction):

```
H[r,t] = sum_m y[origin + r*sps + t + m] * conj(code[m]) / code_energy
```

`H_x` is a linear functional of the input IQ `y_x`; the three paired branches
share origin, template, tap grid, gain and symbol set.

### ROI (frozen, re-verified on calibration-only seeds at run start)

The weak sensing path is delay 37 work samples, gain 0.25, on top of the
delay-0 unit path. The public causal pulse shape has centre tap +2, so the
direct peak lands on offset 2 and the weak-target peak on offset 39.

| ROI (fixed-operator tap offset domain) | range | reference tap |
|---|---|---|
| target | `[35, 44)` | tap `39` |
| background | `[64, 100)` | — |
| direct | `[-8, 8)` | — |

The ROI is **never re-searched on SIC results**; it is confirmed once at run
start from the calibration-only seeds `11020000..11020004`, which are outside
the per-cell seed grid.

### Receiver tap mapping

The R04 receiver ROI `[51,60)` / tap 55 in `uwb.radar_cir_estimator` is exactly
the fixed-operator ROI/tap shifted by `cir_pre = 16`:

```
receiver_tap = fixed_operator_tap_offset + 16      (16 = RX_TAP_OFFSET)
receiver target ROI   = [51, 60)   tap 55
receiver background   = [80, 116)
```

### Metrics (definitions from `operator.json`)

* `C_CIR(N) = 10log10(||E_raw,N||² / ||E_sic,N||²)`, `E_x,N = H_x,N − H_clean,N`.
* `nmse = ||H_x,N − H_clean,N||² / ||H_clean,N||²`.
* `G(N) = 10log10(P(1)/P(N))` on the **nested same-start** set — this is an
  **accumulation gain, never an SIC gain** (flag `g_is_sic_gain = false`).
* `eta = ||sum_r E_r||² / (N·sum_r ||E_r||²)` on one window: 1 = coherent,
  ~`1/N` = i.i.d. zero mean, `N/A` on zero energy. `eta` alone is **not** proof
  of independence (a phase ramp also lowers `eta`), so lag correlations and the
  dominant-tap phase slope are reported alongside it.
* `iq_suppression_db = 10log10(||raw−clean||² / ||sic−clean||²)` on the actual
  output IQ (the S0 `actual_suppression_db` quantity; not on `i_hat`).
* Target protection thresholds: `amp_err_p95_db ≤ 1.0` and
  `peak_shift ≤ 1` work sample, evaluated per `N`.
* Exact zeros are reported as `+Inf` / `-Inf` / `N/A` with **no** `eps` floor and
  no arbitrary clipping. `*_clipped` fields exist for plots only, with
  `plot_floor_db = -80`, `plot_ceil_db = 140`.
* `target_to_background_proxy = |H[target_tap]|² / mean_bg |H[tap]|²` is an
  explicitly-labelled proxy, **not** a detection SINR.

## 4. Two evaluation layers (reported separately, never mixed)

* **Fixed-operator layer — the main causal result.** One linear de-spread
  operator (above) applied identically to the paired `clean = s + w`,
  `raw = s + i + w` and `sic = raw − i_hat` windows. Failures, `N/A` and
  `±Inf` stay in the denominator of every aggregate.
  Repetition sets (§6) are the fixed-operator `[0,N)` / `[8,8+N)` sets.
* **Receiver layer — the engineering result.** The existing unmodified
  `uwb.radar_cir_estimator` block run on all three branches. Its own sync / SFD
  search and L2 normalisation are allowed; per-branch outcome or failure is
  recorded. Its repetition selection is a block-construction `skip`/`count`, so
  it uses the receiver's **own last-N set**, which is **NOT** the fixed-operator
  `[0,N)` / `[8,8+N)` sets. `operator.json` declares:

  | id | skip | count | repetition range |
  |---|---|---|---|
  | `rx_default` | 10 | 0 (all) | `[10, 64)` = 54 reps |
  | `rx_count8` | 10 | 8 | `[10, 18)` |
  | `rx_count16` | 10 | 16 | `[10, 26)` |
  | `rx_count32` | 10 | 32 | `[10, 42)` |

## 5. Seeds

| set | base | use |
|---|---|---|
| calibration | `11000000` | A1 smoke + A2 calibration pass (requires `--i-know-this-is-calibration`) |
| acceptance | `12000000` | A2 formal acceptance, A3 bypass |

Formula (`operator.json.seeds.rule`):

```
seed = base + cell_index * stride_per_cell + window_index,  stride_per_cell = 1000
```

The only RNG is the per-window AWGN substream `np.random.default_rng(seed)`; the
payload is a fixed deterministic PSDU and the path profile is deterministic (no
RNG). Calibration and acceptance window sets are disjoint by construction and
re-checked at runtime and in the verifier.

Separately, the **ROI-freeze seeds** `11020000..11020004` are calibration-only
and lie **outside** the per-cell grid (`11000000 + cell_index*1000 + window`):
they are used only for the run-start weak-target ROI verification.

## 6. Repetition sets

* Full preamble: `R_N = [0, N)`, `N ∈ {1, 2, 4, 8, 16, 32, 64}`.
* Independent region: `R_N = [8, 8+N)`, `N ∈ {1, 2, 4, 8, 16, 32, 56}`;
  `[8, 72)` is **never** treated as 64 independent symbols.
* `N = 1` also reports the per-repetition residual power and the full
  single-symbol distribution (not one hand-picked "good" symbol).

## 7. A2 acceptance grid and cells

`cells.json` fixes SNR = 30 dB (from `operator.json`) and a 12-cell grid:
9 `ISR{0,10,20} × CFO{0,−50 k,+50 k} / single-path` cells, plus 3 extra cells:
`ISR0/CFO+50k/four`, `ISR10/CFO0/four`, `ISR10/CFO0/offset 0.4/single`. The
four-path profile is reused verbatim from
`testdata/sic_preamble/cases/main.json` (`comm_paths.four`), not redefined.
Smoke cell: `isr10_cfo0_single`. A3 bypass cell: `no_interference`
(`comm_present=false`, 1000 windows, expect byte-identical bypass).

## 8. Commands (copy-pasteable) and run status

All Python commands use `env -u LD_LIBRARY_PATH`, the build tree
`gr-uwb/build-sic-accum`, and write under
`gr-uwb/build-sic-accum/results/s2a/<run>`. Run them from the repo root.

Status legend: **RUN** = executed with the recorded result; **IN PROGRESS** =
in flight while this README was written (no final artifact yet); **NOT RUN** =
not executed by anyone.

```bash
# Whole QA suite in one command (needs the build tree) --------------------- RUN
env -u LD_LIBRARY_PATH python3 testdata/sic_accum/qa_s2a.py \
  --run gr-uwb/build-sic-accum/results/s2a/smoke

# QA / design checks (no build tree, no GNU Radio run)  ---------- RUN (see below)
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py --self-test
env -u LD_LIBRARY_PATH python3 tools/sic/accum_metrics.py --self-test
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py --self-test

# A1 smoke: nominal cell, 8 calibration windows, full arrays archived ------ RUN
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py \
  --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/smoke --smoke

# A2 calibration pass (20 windows/cell) ----------------------------------- RUN
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py \
  --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/calibration \
  --seed-set calibration --i-know-this-is-calibration \
  --windows-per-cell 20 --archive-windows 8

# A2 formal acceptance: 12 cells x 100 acceptance windows, + receiver layer - RUN
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py \
  --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/acceptance \
  --seed-set acceptance --windows-per-cell 100 --archive-windows 8 \
  --receiver --receiver-windows 20 --plots

# A3 no-interference bypass / target-protection: 1000 windows ------------ RUN
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py \
  --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/bypass \
  --bypass --windows-per-cell 1000 --archive-windows 8

# Independent verifier over a finished run -------------------------------- RUN (smoke)
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py \
  --run gr-uwb/build-sic-accum/results/s2a/smoke --verbose
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py \
  --run gr-uwb/build-sic-accum/results/s2a/smoke --reject-qa
```

Observed status (2026-10-17, at the time this README was written):

| command | status | observed |
|---|---|---|
| `uwb_sim_sic_accum.py --self-test` | RUN | `app self-test: 0 failure(s)` |
| `accum_metrics.py --self-test` | RUN | `self-test: 0 case(s) failed` |
| `verify_s2a.py --self-test` | RUN | `self-test: 0 failure(s)` |
| `qa_s2a.py --run .../smoke` | RUN | `qa_s2a: 0 failure(s)` (includes verifier + reject-QA on the smoke run) |
| A1 `smoke` (`--smoke`) | RUN | 1 cell × 8 windows, 8 archived; `summary.not_run` declares MATLAB + native not run |
| A2 `calibration` (`--windows-per-cell 20`) | RUN | 12 cells × 20 = 240 windows |
| A2 `acceptance` (`--windows-per-cell 100 --receiver --receiver-windows 20 --plots`) | RUN | 12 cells × 100 = 1200 windows, 96 archived |
| A3 `bypass` (`--bypass --windows-per-cell 1000`) | RUN | 1000 windows, 1000/1000 bypassed, 0 output changed, 8 archived |
| `verify_s2a.py --run .../smoke --reject-qa` | RUN | `reject-QA: 14/14 mutations caught` |
| `verify_s2a.py --run .../smoke` | RUN | `checks: 45, failures: 0` |
| `verify_s2a.py --run .../calibration` | RUN | `checks: 45, failures: 0` |
| `verify_s2a.py --run .../acceptance` | RUN | `checks: 47, failures: 0` (receiver statuses `ok`/`timing_failed` accounted) |
| `verify_s2a.py --run .../bypass` | RUN | `checks: 48, failures: 0` |
| `verify_s2a.py --run .../acceptance --reject-qa` | RUN | `reject-QA: 14/14 mutations caught` |
| MATLAB `verify_s2a_cir` | NOT RUN | no MATLAB/Octave on this host (§10) |

The smoke/calibration/acceptance/bypass runs are executed by the S2-A task
author; the three `--self-test` checks, the two smoke verifications, the
acceptance verification, and the fixture export (§10) were executed while
writing these fixtures and documents.

## 9. Archive policy and its limitation

`operator.json.archive_policy`: **the first 8 windows of every cell, selected by
window index only** (no metric-based selection). Each archived `.npz`
(`archive/<cell_id>_<wid:05d>.npz`) stores:

```
clean, raw, sic, ihat, i_truth            complex64 full-window IQ
H_clean, H_raw, H_sic                     complex128 (sync_reps, ntaps) = (64, 160)
truth_json, meta_json                     UTF-8 JSON bytes
```

**Explicit limitation:** non-archived windows **cannot be recomputed
independently** from the archive, because the archive keeps the *first 8 by
index* only. The verifier's independent recomputation covers exactly the
archived windows; the aggregate `summary.json` / `cells_summary.csv` cover all
windows and are recomputed from `windows.jsonl` (which keeps the full
denominators, including failures, `N/A` and `±Inf`).

## 10. MATLAB status: script provided, NOT executed (未实跑)

`testdata/sic_accum/matlab/verify_s2a_cir.m` is an independent cross-check that
re-derives the fixed-operator `H` with an explicit double loop and compares it
to `expected_H_*`, then recomputes `C_CIR` at N=1 and N=64. It has **not** been
run: no MATLAB and no Octave exist on this host (`command -v matlab octave` is
empty). No MATLAB-verified result is claimed, and the NumPy reference is **not**
a substitute for a MATLAB-accepted result. See
`testdata/sic_accum/matlab/README.md`.

The fixture it consumes is produced by
`testdata/sic_accum/export_matlab_fixture.py`. It writes, into `--out <dir>`:
`clean.cf32`, `raw.cf32`, `sic.cf32`, `code.cf32`, `expected_H_clean.cf32`,
`expected_H_raw.cf32`, `expected_H_sic.cf32` (complex64, interleaved I/Q, as
`np.ndarray.tofile` writes them; the H tensors are the archived complex128 cast
to complex64 row-major `(sync_reps, ntaps)`) and `fixture.json` with
origin/sps/sync_reps/tap_lo/tap_hi/ntaps/code_energy/sample_count/window_id/
cell_id/source-archive sha256 and the array shapes. It is numpy + stdlib only.

```bash
# Export the first archived smoke window and then run MATLAB (MATLAB NOT RUN here)
python3 testdata/sic_accum/export_matlab_fixture.py \
  --run gr-uwb/build-sic-accum/results/s2a/smoke --out /tmp/opencode/s2a_fixture
matlab -batch "verify_s2a_cir('/tmp/opencode/s2a_fixture')"
```

The export command above was tested on the smoke archive; the MATLAB step was
not.

## 11. File inventory

| file | role |
|---|---|
| `README.md` | this contract + operations document |
| `operator.json` | frozen fixed-operator / ROI / N-set / seed contract (frozen, do not edit) |
| `cells.json` | frozen 12 acceptance cells + `comm_paths` + smoke/bypass cells (frozen, do not edit) |
| `export_matlab_fixture.py` | export one archived window to a MATLAB-readable fixture (numpy + stdlib) |
| `matlab/verify_s2a_cir.m` | independent MATLAB cross-check of `H` and `C_CIR` (base MATLAB only, **未实跑**) |
| `matlab/README.md` | how to run the MATLAB check, what it verifies, scope/limits |
| `qa_s2a.py` | single QA entry point: the three `--self-test`s + a pure-NumPy two-path operator/ROI end-to-end check + optional verifier/reject-QA on a real run |

Run artifacts live under `gr-uwb/build-sic-accum/results/s2a/<run>/` and are not
version-controlled.

## 12. Known limitations / not-run list

* MATLAB cross-check provided but **not executed** (no MATLAB/Octave).
* Native / UHD / FPGA grid, hardware, TWR: not exercised (out of scope).
* Continuous 1 GS/s throughput and any real-time acceptance claim: not made.
* Full Pd/Pfa scan: not performed.
* Receiver layer: the acceptance run shows `timing_failed` on the **raw** branch
  of the three ISR20 single-path cells (240 of 2880 receiver results); clean and
  SIC are `ok` 20/20 everywhere. The verifier now accepts `timing_failed` as a
  legitimate receiver status and accounts for it (`n_ok <= n_total`); the fixed
  operator (main causal) checks are unaffected. This receiver-layer finding is
  reported as-is in `docs/sic/S2A_累加后CIR开发报告.md` §7.
* A3 `bypass`: 1000/1000 bypassed, `out == y` byte-for-byte, no fabricated
  C_CIR; verifier `checks: 48, failures: 0`.
