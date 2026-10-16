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
| code-10 SYNC (1016) | first symbol of the finite-SYNC preamble (see R07 below) | none |
| code-10 64-SYNC preamble (65072) | independent `lrwpan.internal.HRPCodes(10)` code-pulse train + LSQ-fitted 48-tap FIR (finite SYNC, **no SFD/PHR/payload**) | none |
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

## R07 finite-SYNC comm preamble convention

**Problem fixed (R07).** The previous `comm_preamble_code10_998p4.cf32` was a
slice `full_packet(SYNC(1) : SYNC(end)+47)` of a full 64-SYNC packet. Because
the SFD follows the SYNC in the packet, the first SFD pulses leaked into the
last `n_taps` = 48 samples of the "preamble" tail. R07 requires the preamble to
be the natural response of the shaping filter to a **finite-length SYNC
code-pulse train only**.

**Generation (independent MATLAB implementation).** `export_comm_template.m`
now builds the waveform itself; it does **not** slice a packet:

| quantity | value | note |
|---|---|---|
| sample rate | 998.4 MS/s | work grid |
| samples per chip | 2 | `kQm35SamplesPerChip` |
| spreading factor | 4 | `kQm35SpreadingFactor` |
| Ipatov code length | 127, ternary {-1,0,+1} | `lrwpan.internal.HRPCodes(10)`, 64 non-zeros |
| chips per symbol | 508 = 127·4 | `kQm35ChipsPerSymbol` |
| samples per symbol | 1016 = 508·2 | `kQm35SamplesPerSymbol` |
| SYNC repetitions | 64 | |
| shaping FIR taps | 48 real causal, centre tap 2 | recovered by MATLAB LSQ, see below |
| pulse placement | `sample(r,c) = r*1016 + c*8`, amplitude `code(c+1)` | same as C++ `add_shaped_chip` |
| tail convention | `y(n) = Σ_k taps(k)·train(n-k+1)` | causal, `taps(1)` at the pulse sample |
| length convention | `64*1016 + 48 = 65072` | `sync_reps*samples_per_symbol + n_taps` |
| first pulse sample | 0 | leading edge = `taps(1)·code(1)` |
| last pulse sample | 65016 = 63·1016 + 126·8 | |
| last non-zero sample | 65063 (zero-based) | natural filter tail = 65016+47 |
| trailing zeros | 8 | close the 65072 convention |
| contains SFD / PHR / payload | **no / no / no** | by construction |

**Pulse shaping.** The 48 taps are recovered *inside MATLAB* by a
least-squares fit of the toolbox waveform's **SYNC region only**
(`packet(idx.SYNC(1):idx.SYNC(end))`, 65024 samples — upstream of the SFD)
against the known code-pulse train. Nothing is read from the C++ output. The
fitted taps are additionally cross-checked against the frozen profile file
`testdata/uwb_hrp_tx/pulse_48.f32` (diagnostic only; the fit is used unless its
relative residual exceeds `1e-3`, in which case the frozen file is a labelled
fallback). The taps, their `sha256`, the LSQ residual and the frozen-file delta
are all written to `comm_template_meta.json` and stored in
`comm_pulse_taps_998p4.f32`.

**Normalisation.** The preamble is peak-normalised to `peak_amplitude = 0.8`
using its **own peak** (not the full-packet peak), so the preamble-only waveform
is independent of the downstream SFD/PHR/payload. `comm_sync_code10_998p4.cf32`
is the first 1016 samples scaled with the same factor. The scale and the
diagnostic whole-packet peak are recorded in the meta.

**SFD-independence check.** `export_comm_template.m` also regenerates a packet
with an alternative `SFDNumber` (best effort) and records that the 65024-sample
SYNC region is unchanged (`meta.sfd_independence`). This is what makes
"editing the downstream packet cannot change the preamble-only waveform" a
checked statement.

**Edge/tail self-checks** (hard invariants, run inside the exporter): the
leading edge is sample 0, the last significant sample is `65063` (the natural
tail end), the 8 samples past it are exactly zero (no leaked SFD/PHR/payload),
the length is `65072`, and the `comm_sync` first symbol equals
`comm_preamble(1:1016)`.

The old full-packet-slice template and its hashes are **not** overwritten
anywhere under version control; the new template has new length/energy/hash
recorded in `comm_template_meta.json`.

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
| `comm_sync_code10_998p4.cf32` | 1016 | first symbol of the finite-SYNC preamble | `templates/comm_sync_code10_998p4.cf32` |
| `comm_preamble_code10_998p4.cf32` | 65072 | independent MATLAB finite 64-SYNC code-pulse conv (no SFD) | `templates/comm_preamble_code10_998p4.cf32` |
| `comm_pulse_taps_998p4.f32` | 48 | MATLAB LSQ-fitted shaping FIR | `testdata/uwb_hrp_tx/pulse_48.f32` (diagnostic) |
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

`not_run.log` records the actual probes on 2026-10-16 (08:56:52Z and the R07
re-check at 14:16:00Z):

```
$ which matlab octave
exit=1
$ matlab -batch "disp(version)"
/bin/bash: matlab: command not found   (exit 127)
$ octave --eval "disp(version)"
/bin/bash: octave: command not found   (exit 127)
```

Conclusion: no cross-check has been executed. The final S0 report must list this
as `not_run`; the Python reference is not an accepted substitute. In particular
the R07 finite-SYNC preamble (`export_comm_template.m`) and every output hash in
`comm_template_meta.json` remain **未实跑 (NOT RUN)** until a host with the
Communications Toolbox executes the exporter; the convention above is the frozen
contract the C++ producer must match, not a MATLAB-verified result.
