# S0 preamble-only SIC golden vectors

Small, deterministic, model-consistent cross-check vectors for the frozen SIC
core contract (`docs/sic/S0_sic_core_api.md`).  The NumPy reference that
produced them is `tools/sic/sic_reference.py`.

Regenerate deterministically:

```bash
python3 tools/sic/sic_reference.py --emit-golden
python3 tools/sic/sic_reference.py --self-test
```

## Size note

These vectors use a **reduced** `samples_per_symbol` (=127) and only
`16` SYNC repetitions so the files stay small and commit-able.  The real work
grid uses `samples_per_symbol=1016`; the algorithm is SPS-agnostic, so the C++
QA **must read `samples_per_symbol`, `sample_rate`, `sync_reps`, ROI and
thresholds from `meta.json`** (never hard-code 1016).

## Layout

```
golden/
  README.md            this file
  index.json           case list + sha256 of every io.npz
  <case>/
    meta.json          cfg / truth / expected / thresholds / PASS criteria
    io.npz             y, sync, preamble, i, i_hat_expected, out_expected
```

Cases: int_single, int_four, frac, cfo, k_sweep, cross_model

## io.npz schema (stable)

All arrays are `complex64`; all indices are 0-based.

| array | shape | meaning |
|---|---|---|
| `y` | `(N,)` | received window `s + i + w`; the only signal `estimate_sic` may see |
| `sync` | `(sps,)` | ONE shaped SYNC template for the comm code (detection) |
| `preamble` | `(Lp,)` | FULL comm preamble template (reconstruction) |
| `i` | `(N,)` | **TRUTH** interferer; never pass to `estimate_sic` |
| `i_hat_expected` | `(N,)` | reference `Estimate` reconstruction of `i` |
| `out_expected` | `(N,)` | reference `Estimate` output (`out == y` outside the mask) |

`i` is included so the oracle / NMSE diagnostics can be evaluated; the online
`estimate_sic` function has no truth argument and never reads it.  The QA
harness must call `estimate_sic(y, sync, preamble, cfg)` only.

## meta.json

* `generator` / `truth` — truth-bearing (generator side only).
* `runtime_cfg` — the exact `SicConfig` the reference used.  `mode` is the
  string `"Estimate"` (map it to `SicMode::Estimate`); `path_min_spacing = 0`
  means "derive from the template autocorrelation mainlobe".
* `expected` — expected `detected_start`, `cfo_hz`, `taus` (absolute work
  samples, `start + delay_k`), `gains`.
* `thresholds` — `detect_threshold`, `validate_threshold`.
* `pass_criteria` — per-case bounds used by `--self-test`.

## Fractional-delay model note

The core owns a **fixed band-limited (windowed-sinc, 16-tap Hamming)**
fractional-delay operator.  `frac` is model-consistent (generator uses the same
operator).  `cross_model` uses an **independent band-limited kernel** (4-tap
Blackman) on the generator side, so generator and estimator are not the same
interpolator; it is a declared stress case.

## Comparison tolerance

The reference uses `complex128` internally.  A faithful C++ port will not match
bit-exactly: compare `detected_start` exactly (or +/-1 sample), `cfo_hz` within
a few kHz, and `i_hat_expected` / `out_expected` by NMSE (a small tolerance,
e.g. re-committed self-consistency < 1e-3), and require `out == y` byte-for-byte
outside the estimated support mask.
