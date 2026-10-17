# S2-A MATLAB independent cross-check (fixture consumer)

**Status: 未实跑 / NOT EXECUTED.** There is no MATLAB and no Octave on this host
(`command -v matlab octave` is empty; exit code 1). `verify_s2a_cir.m` was
reviewed for syntax only and has **not** been run, so **no MATLAB-verified result
is claimed**. The Python/NumPy reference (`tools/sic/accum_metrics.py` +
`tools/sic/verify_s2a.py`) is **not** a substitute for a MATLAB-accepted result.

MATLAB is **not required** for the normal S2-A pipeline. It is an optional extra
witness for the fixed-operator CIR definition; the primary path is the frozen
NumPy operator plus the independent verifier.

## Files

| file | role |
|---|---|
| `verify_s2a_cir.m` | the single-file MATLAB cross-check (base MATLAB only) |
| `README.md` | this file |

The MATLAB input fixture is written by
`testdata/sic_accum/export_matlab_fixture.py`, which copies one archived window
out of an S2-A run. Each fixture directory contains:

```
clean.cf32  raw.cf32  sic.cf32     complex64 interleaved I/Q (np.tofile order)
code.cf32                          the frozen code-9 bare sampled code (1016)
expected_H_clean.cf32              archived CIR tensors, complex64,
expected_H_raw.cf32                row-major (sync_reps=64, ntaps=160), the
expected_H_sic.cf32                complex128 archive cast to complex64
fixture.json                       origin/sps/sync_reps/tap grid/code_energy,
                                   cell_id/window_id, source archive sha256
```

## How to run

```bash
# 1. export one archived window (run this from the repo root)
python3 testdata/sic_accum/export_matlab_fixture.py \
  --run gr-uwb/build-sic-accum/results/s2a/smoke \
  --out /tmp/opencode/s2a_fixture

# 2. run the MATLAB check (any MATLAB R2019a+; base install only)
matlab -batch "verify_s2a_cir('/tmp/opencode/s2a_fixture')"
#   or, from inside testdata/sic_accum/matlab:
#   matlab -batch "verify_s2a_cir"        -> defaults to <thisdir>/out
```

If Octave is available it can be tried instead (`octave --eval ...`), but the
script is written for MATLAB and was not tested under Octave either.

## What it verifies

1. **CIR definition.** It re-derives, with an explicit double loop over
   `(r, t)` and `sum(...)` over the code vector,

   ```
   H[r,t] = sum_m y[origin + r*sps + t + m] * conj(code[m]) / code_energy
   ```

   for `clean`, `raw` and `sic`, and compares against the archived
   `expected_H_*` tensors. Frozen tolerances: `max_abs <= 1e-4` and
   `rel <= 1e-4` (vs `max|expected|`); the archive is complex64, so a
   double-precision recompute agrees to ~1e-6. It prints
   `max|H|`, `max_abs_err`, the relative error and an explicit **PASS/FAIL**
   per branch, plus the number of non-finite cells skipped.
2. **CIR SIC gain.** It recomputes
   `C_CIR(N) = 10log10(||E_raw,N||^2 / ||E_sic,N||^2)` with
   `E_x,N = mean_{r in [0,N)} H_x[r] - mean_{r in [0,N)} H_clean[r]`
   at `N = 1` and `N = 64`, over the full tap grid and over the frozen target
   ROI offsets `[35,44)`, and prints the four values with the same exact-zero
   handling as the Python side (`N/A` / `+Inf` / `-Inf`, no `eps` floor).

It **cannot** read `windows.jsonl`; it only prints its own numbers so they can
be diffed by hand (or by an outer script) against the Python per-window record.

The script starts with a banner that prints the MATLAB `version` and states that
the script was not executed by the author of the change.

## Scope / limits

* One archived window per fixture. Only the first 8 windows of every cell are
  archived by the run, so other windows cannot be exported.
* It re-checks the **fixed-operator** layer only; it does not instantiate or
  re-check the GNU Radio receiver layer (`uwb.radar_cir_estimator`).
* It shares the archived IQ (`clean`/`raw`/`sic`) and the archived expected
  tensors with the C++/Python run; the independent part is the MATLAB
  re-derivation of `H` and `C_CIR`, not a re-generation of the signals.
