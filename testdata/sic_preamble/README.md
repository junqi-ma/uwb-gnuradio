# S0 preamble-collision / preamble-only SIC fixtures (truth sidecar)

This directory is the **generator-side** contract for the S0 task
(`docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md`). Nothing here is
consumed by the online SIC API: it contains only *generation* configuration,
the injected ground truth, and generated fixtures. The SIC runtime config lives
separately (`cases/*.json` -> `runtime`), and the SIC block never receives this
directory or any object derived from it.

> "前导重合" means **time overlap**, **not the same code**. Default sensing
> code 9, comm code 10, 64 SYNC, 998.4 MS/s work grid (1016 samples/SYNC).
> `sensing_code == comm_code` is forbidden except in an explicit diagnostic
> preset.

## Layout

```
testdata/sic_preamble/
  README.md                     this file (frozen)
  config_schema.json            field names/types for generator + runtime
  cases/
    baseline.json               no interference / SIC off (G0)
    main.json                   §8.2 main effect matrix (axes + seeds)
    negative.json               negatives: wrong code, no-overlap, low SNR ...
    stress.json                 K/reps/ROI/SFO/model-mismatch/clip stress
  fixtures/                     generated at run time (NOT version-controlled)
    <case_id>/window_<id>.npz   s, i, w, y (+ meta)  [truth sidecar]
    <case_id>/truth.jsonl       per-window injected truth (delay/CFO/paths/ISR)
    <case_id>/manifest.json     hashes, lengths, seeds, commit
```

Generated `.npz`/`.jsonl`/large IQ under `fixtures/` **must not be committed**
(see `AGENTS.md`): they live under a build/tmp or this ignored tree and are only
referenced after explicit archiving.

## Signal model & normalization

`y[n] = s[n] + i[n] + w[n]`, all complex/CF32 linear-amplitude.

- `s` — reused sensing full packet/echo model (existing `SimTimedEcho`
  geometry). Full window is retained so the **complete** SFD/PHR/payload/FCS
  can be demodulated; the CIR ROI is a view/crop of that same window.
- `i` — an **independent code's** finite 64-SYNC preamble through an
  independent multipath/timing/CFO channel, shaped by an independent comm
  template. Only the preamble is generated (no comm PHR/payload/FCS).
- `w` — complex AWGN; `noise_std` is the **per I/Q component** standard
  deviation (total complex noise power = 2·noise_std²).

SNR (30 dB) and ISR (10 dB) are defined on the **specified overlapping SYNC
interval** `[lo, hi)` against the clean sensing energy in that same interval.
The pre/post-scale powers and the realized ISR are recorded. When there is no
overlap or no signal, ISR is `null` (never divide-by-zero, never faked).

### Work vs native

- **A / work direct**: inject the code-10 preamble on the 998.4 MS/s work grid.
  No resampling cost counted.
- **B / CG400 native roundtrip**: comm is generated on work, decimated 32/65 to
  the 491.52 native grid, injected there, then run through the existing PDU
  65/32 back to work. clean/raw/sic go through identical resampling and
  coordinate mapping. SC16 quantization/clipping is counted separately.

SC16 quantization is non-linear: `s/i/w` must **not** be quantized separately
and assumed additive. The native verifier keeps `s_work`, `i_work` in the same
common scale/causal filter and treats `y_work - s_work - i_work` as the
effective noise floor (AWGN + quantization + numeric error), reported
separately. `clip_count` must be 0 for the primary acceptance runs.

The known resampler SIMD uninitialized-read / non-determinism is **recorded,
not fixed** (per `AGENTS.md`); a scalar/reference resample path is kept as a
separate row and the production chain is never claimed as verified.

## Coordinates

- All indices 0-based, intervals `[lo, hi)`.
- Fractional delays state unit and interpolation convention (generator uses
  PCHIP; the core must not assume it — at least one cross-model case uses an
  independent band-limited delay). `tau` in work/native samples unless a field
  name says `_us`/`_hz`.
- Runtime metadata for each window (main chain):

  ```
  schema_version, run_id, window_id, epoch_id, sample_rate, sample_count,
  window_start_sample, predicted_sensing_start, sensing_profile_id,
  sensing_code, sensing_sync_reps, sensing_sfd_mode, template_id
  ```

  native adds: native/work coordinate mapping, filter delay, valid-sample range.

  When feeding existing blocks, the adapter maps `code_index / sync_repetitions
  / sfd_mode / pulse_id` **for the sensing signal only**; the comm profile lives
  in a separate config namespace and must never overwrite those fields.

- The online SIC API may know: comm code, nominal preamble length, template,
  sample rate, and a candidate search ROI. It may **not** read exact comm
  start/CFO/paths/ISR/interferer IQ or any generator-side event. `oracle` mode
  is diagnostic only, emits `uses_truth=true`, and is never valid for
  effect/real-time acceptance.

## Presets (frozen)

All four presets are frozen here and in `cases/*.json`. Unknown parameters are
rejected. Every preset first fixes its **calibration seed list** and
**acceptance seed list**; the two never intersect.

| Preset | Purpose | SIC | Interference |
|---|---|---|---|
| `baseline` | G0 no-interference closure | off | none |
| `main` | §8.2 effect matrix | estimate (+oracle diagnostic) | code9/code10, ISR 0/10/20, offsets, CFO 0/±50 kHz |
| `negative` | false-trigger / bypass evidence | estimate | none / wrong code11 / no-overlap / weak / low-SNR |
| `stress` | boundary/pressure | estimate | K/reps/ROI/SFO/model-mismatch/clip |

Preset fields and types are declared in `config_schema.json`; the concrete
values are in `cases/<preset>.json`.

## Commands (S1 `estimate8_frozen`, actually runnable)

All commands use `env -u LD_LIBRARY_PATH` and the `build-sic` tree; check with
`ldd` that `libgnuradio-uwb` resolves under `gr-uwb/build-sic/lib`.

```bash
# 1. Nominal smoke: strict 8-SYNC estimation + full-preamble cancellation,
#    real CIR + full sensing demod, acceptance seeds.
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
  --build-dir gr-uwb/build-sic --preset main --input-grid work \
  --sic estimate --consumer both --windows 8 --fixtures 8 \
  --seed-set acceptance --sic-model-mode estimate8_frozen \
  --output gr-uwb/build-sic/sic-results/s1_smoke_nominal

# 2. Acceptance main matrix (72 cells x 100 windows), same model mode/config.
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
  --build-dir gr-uwb/build-sic --preset main --sweep --sic estimate \
  --consumer both --windows 100 --seed-set acceptance \
  --sic-model-mode estimate8_frozen \
  --output gr-uwb/build-sic/sic-results/s1_accept_main

# 3. Acceptance negatives (>=1000 windows per actionable category).
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
  --build-dir gr-uwb/build-sic --preset negative --sic estimate \
  --consumer both --windows 1000 --seed-set acceptance \
  --sic-model-mode estimate8_frozen \
  --output gr-uwb/build-sic/sic-results/s1_accept_negative

# 4. CG400 native nominal (same 8-SYNC mode).
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
  --build-dir gr-uwb/build-sic --preset main --input-grid cg400 \
  --sic estimate --consumer both --windows 32 --seed-set acceptance \
  --output gr-uwb/build-sic/sic-results/s1_native

# 5. Pure-target asynchronous paced (100/s then 200/s, 60 s).
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py \
  --build-dir gr-uwb/build-sic --preset main --sic estimate \
  --consumer cir --rate 100 --duration 60 --seed-set acceptance \
  --paced-target-only \
  --output gr-uwb/build-sic/sic-results/paced_s1_100

# 6. Real-input effect-verified benchmark (core / GR / CIR / full demod).
env -u LD_LIBRARY_PATH gr-uwb/build-sic/apps/benchmark_preamble_sic \
  --preset main --mode gr --consumer demod --model-mode estimate8_frozen \
  --windows 10000 --rounds 3 --output gr-uwb/build-sic/sic-results/s1_perf_demod

# 7. Independent verifier (S0 + S1 checks) over declared artifacts.
python3 tools/sic/verify_s0.py --results gr-uwb/build-sic/sic-results/s1_smoke_nominal
python3 tools/sic/verify_s0.py --self-test
```

Legacy control only (NOT the S1 path): `--sic-model-mode confirmed_legacy` or
`--sic-cover-preamble`; those restore the old confirmed-count support and must
not be reported as the 8-SYNC result. `--sic-fit-reps` other than 8 is rejected
in `estimate8_frozen`. `--sic-evaluation-reps` is offline-only (never gates).

MATLAB export/verify scripts (`export_sic_preamble.m`, `verify_sic_preamble.m`)
live in this directory; if no MATLAB/Octave is present they are kept runnable
and reported as **未实跑** — the Python reference is **not** a substitute for a
MATLAB-accepted result.
