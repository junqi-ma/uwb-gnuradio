# S0-D MATLAB independent cross-check scripts

**Status: 未实跑 (NOT RUN).** There is no MATLAB and no Octave on this host
(`which matlab octave` is empty; exit 127 on launch — see `not_run.log`). The
scripts below are the frozen, runnable MATLAB-side cross-check for the S0 task
(`docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md` §9, §3). They were
reviewed for syntax but have **not** been executed, and **no MATLAB-verified
result is claimed**. The Python/NumPy reference (`tools/sic/sic_reference.py`) is
**not** a substitute for a MATLAB-accepted result.

## Why MATLAB is a separate witness

The S0 instruction forbids a circular check where C++ reads its own output back
(`MATLAB 与 C++ 用不同实现生成/定位，不只拿 C++ 输出互相读写形成循环证明`).
These scripts therefore:

| item | MATLAB method | shared input with C++ |
|---|---|---|
| code-10 SYNC (1016) | independent `lrwpanWaveformGenerator` | none |
| code-10 64-SYNC preamble (65072) | independent `lrwpanWaveformGenerator` | none |
| code-9 sensing SYNC (1016) | independent `lrwpanWaveformGenerator` | none |
| `i` (interferer) | MATLAB regenerated from the template + generator truth channel + §3.1 ISR rule | template (independent), truth channel, clean `s` (for ISR scale) |
| `s` (clean sensing) | **loaded** from the C++ fixture (not regenerated) | C++ fixture |
| detection start | independent normalised-correlation, edge-anchored detector on C++ `y` | C++ `y` |
| per-tap CIR | independent matched filter / `estimateCir` port | C++ `y` / `s` |

`s` is deliberately loaded from the C++ fixture and only checked for integrity:
the C++ `hrp_packet_source` packet construction is not replicated here, so the
honest statement is "loaded, not regenerated". `i` and the templates are the
genuine independent witnesses.

`w`/`y` are **not** sample-comparable: MATLAB and NumPy use different RNGs. The
verifier compares the deterministic part `s + i` and reports the C++ AWGN RMS as
a floor.

## Required toolboxes / release

* MATLAB R2019a or newer (uses `matlab -batch`, `jsonencode`, `jsondecode`).
* **Communications Toolbox** with the LR-WPAN HRP UWB support:
  `lrwpanHRPConfig`, `lrwpanWaveformGenerator`, `lrwpanHRPFieldIndices`,
  `lrwpan.internal.HRPCodes`.
  (`export_comm_template.m` fails fast with `export_comm_template:NoToolbox`
  when `lrwpanHRPConfig` is absent.)
* No additional toolbox. The `.npy`/`.npz` reader is a small self-contained
  parser (MATLAB has no native `.npz` reader); it handles the dtypes the S0
  fixtures use (`<c8`, `<f8`, `<f4`, `<i8`, `<i4`).

## Exact invocation

From the repository root:

```bash
# 1) independent templates -> testdata/sic_preamble/matlab/out/
matlab -batch "cd('testdata/sic_preamble/matlab'); export_comm_template"

# 2) one representative S0 case (regenerates i, detects, CIR)
#    default C++ run dir = gr-uwb/build-sic/sic-results/main_est
matlab -batch "cd('testdata/sic_preamble/matlab'); \
  export_sic_preamble('out', '<repo>/gr-uwb/build-sic/sic-results/main_est')"

# 3) field-by-field hard-gate verify vs the C++ artifacts
matlab -batch "cd('testdata/sic_preamble/matlab'); \
  verify_sic_preamble('out', '<repo>/gr-uwb/build-sic/sic-results/main_est')"
```

The C++ run directory is a **required path argument** — never hard-coded, except
for the documented `main_est` default in `export_sic_preamble.m`.

Exit semantics: `verify_sic_preamble.m` calls `error(...)` on any mismatch or
missing required artifact, which makes `matlab -batch` exit non-zero. It writes
`out/verify_report.json` either way.

## Expected outputs (`testdata/sic_preamble/matlab/out/`, all generated)

| file | samples | producer | compared against |
|---|---|---|---|
| `comm_sync_code10_998p4.cf32` | 1016 | independent MATLAB | `templates/comm_sync_code10_998p4.cf32` |
| `comm_preamble_code10_998p4.cf32` | 65072 | independent MATLAB | `templates/comm_preamble_code10_998p4.cf32` |
| `sensing_sync_code9_998p4.cf32` | 1016 | independent MATLAB | `templates/sync_template_code9_998p4.cf32` (optional) |
| `s_998p4.cf32` | 157376 | loaded from C++ fixture | `fixtures/window_0000.npz:s` (integrity) |
| `i_998p4.cf32` | 157376 | MATLAB regenerated | `fixtures/window_0000.npz:i` |
| `y_998p4.cf32` | 157376 | MATLAB `s+i+w` | not compared (RNG) |
| `detected_start.json` | — | MATLAB detector | `windows.jsonl:sic_state.detected_start` |
| `sic_cir_raw_998p4.cf32` | 116 | MATLAB matched filter | C++ `cir/sic_cir_norm_998p4.cf32` (optional) |
| `sic_cir_norm_998p4.cf32` | 116 | MATLAB matched filter | same |
| `radar_cir_raw_998p4.cf32` | 116 | MATLAB `estimateCir` port | C++ `cir/radar_cir_norm_998p4.cf32` (optional) |
| `radar_cir_norm_998p4.cf32` | 116 | MATLAB `estimateCir` port | same |
| `comm_template_meta.json` | — | meta/hashes | — |
| `export_meta.json` | — | case meta | — |
| `verify_report.json` | — | verifier | — |

The window length (157376) is read from the fixture, not assumed. Template
lengths 1016 and 65072 are asserted.

Everything under `out/` is **generated** and must not be committed (the
`.cf32` outputs are already covered by `testdata/sic_preamble/.gitignore`);
only these `.m`/`.md`/`.log` files are version-controlled.

## Frozen tolerances

Frozen in `verify_sic_preamble.m` for this S0-D revision. They may only be
**tightened** by an explicit documented revision and a re-run; they must never
be widened to make a failure pass.

| field | metric | tolerance |
|---|---|---|
| templates (`comm_sync`, `comm_preamble`, `sensing_sync`) | `max_abs`, `rel_l2` | `<= 1e-4` |
| `i` (interferer) | `max_abs`, `rel_l2` | `<= 2e-3` |
| `s` (loaded integrity) | `max_abs`, `rel_l2` | `<= 1e-6` |
| `s+i` (noise-free part) | `max_abs`, `rel_l2` | `<= 2e-3` |
| `detected_start` | absolute sample difference | `<= 1` |
| per-tap CIR | normalised-magnitude NCC, `max|mag|` diff | `NCC >= 0.98`, diff `<= 0.10` |

Rationale: the C++ `hrp_packet_source(pulse_shape="legacy")` reproduces MATLAB
`lrwpanWaveformGenerator` to `< 2e-7` (S0-A G0 note), so the `1e-4` template
bound is conservative. The `i` bound absorbs the float32/float64 difference in
the overlap-energy ISR scale; a real template/channel defect is far larger. The
CIR bound is a shape test (the two implementations may differ in absolute
normalisation).

## Per-tap CIR convention the C++ producer must export

`verify_sic_preamble.m` searches the run directory for optional per-tap CIR
artifacts and marks them `NOT_RUN` when absent (they are **not** a hard gate in
the current C++ run, which does not export CIR to disk):

* `cir/radar_cir_norm_998p4.cf32` — sensing radar CIR from the C++ CIR consumer
  (`radar_cir_estimator`, pre 16 / post 100 / skip 10 / 64 reps), CF32, 116 taps.
* `cir/sic_cir_norm_998p4.cf32` — SIC matched-filter CIR, CF32, 116 taps
  (pre 16 / post 100 around the estimated comm start).
* or pass an explicit file via `verify_sic_preamble(..., 'CirPath', '<file>')`.

If the C++ producer adopts different tap conventions, the comparison must be
reconciled (documented revision), not silently skipped.

## 未实跑 evidence

`not_run.log` records the actual probe on 2026-10-16:

```
$ which matlab octave
exit=1
$ matlab -batch "disp(version)"
/bin/bash: matlab: command not found   (exit 127)
$ octave --eval "disp(version)"
/bin/bash: octave: command not found   (exit 127)
```

Conclusion: no cross-check has been executed. The final S0 report must list this
as `not_run`; the Python reference is not an accepted substitute.
