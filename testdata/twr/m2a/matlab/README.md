# M2-A MATLAB oracle (agent C, `m2a_matlab_oracle`)

**Status: M2-A 未完成，MATLAB 待实跑.**

MATLAB is **not** available on the machine that authored these scripts
(`/usr/local/MATLAB/R2024a` has no `bin/matlab` launcher and no main binary;
no `matlab`/`octave`/MCR on `PATH`; not WSL). Therefore:

- the committed `manifest.json` records `matlab_executed: false`;
- **no MATLAB result is claimed anywhere in this directory**;
- **a `.m` file existing is not a pass** — A11 (the §6 three-way MATLAB
  cross-check) stays **未满足** until these scripts are actually run on a
  MATLAB machine and their outputs are reviewed;
- Python/SciPy is **not** used as a substitute for a MATLAB claim.  The one
  Python file here, `gen_golden_inputs.py`, generates *input* vectors only
  (never an oracle output).

This directory is the handoff: frozen inputs, three independent runnable
scripts, and the exact commands and expected outputs.

---

## 1. What the three scripts are

G0 §11 / task-list §6 require three independent cross-checks for A11:

| Script | Cross-check | Consumes | Produces |
|---|---|---|---|
| `matlab_resample_reference.m` | #1: MATLAB `upfirdn` + explicit polyphase reference of the G0 §3.1 causal full-convolution contract vs the C++ resampler, sample by sample | frozen taps, `inputs/*_in.cf32`, C++ `*_cpp.cf32` | `out/matlab_resample_reference_result.json`, `out/<dir>_<case>_matlab.cf32` |
| `matlab_decode_cpp_iq.m` | #2: resample the C++ native IQ with the same contract, decode with `UWB_demodulation/decode_uwb.m`, compare PHR/PSDU/FCS and diagnostic coordinates | a `twr-m2a-native/1` JSON + its native CF32 IQ, frozen RX taps | `out/matlab_decode_cpp_iq_result.json` |
| `matlab_generate_twr_frame.m` | #3: independently serialise Poll/Response/Final, append the FCS via `+uwbdecoder/ieee802154CRC16.m`, generate the standard minimal-profile work-grid waveform, self-decode | repository `+uwbdecoder/buildUwbReference.m`, `frame_golden_v1.json` | `generated/<frame>_work.cf32`, `generated/<frame>_expected.json` |

All three fail closed: a decoder/self-check exception is a non-zero exit, never
a caught-and-reported PASS.  Missing external artifacts are reported as
`blocked`, never as PASS.

## 2. Requirements on the MATLAB machine

- MATLAB with:
  - **Communications Toolbox** — `lrwpanHRPConfig`, `lrwpanWaveformGenerator`
    (required by script #3, and by `+uwbdecoder/buildUwbReference.m` used by
    scripts #2/#3);
  - **Signal Processing Toolbox** — `upfirdn`, `readmatrix` (script #1).  If
    `upfirdn` is absent, script #1 falls back to its own explicit polyphase
    implementation and records `have_upfirdn=false`; the explicit polyphase
    form is still an independent reference of the same contract.
- No Python/NumPy is needed to run the scripts.  `gen_golden_inputs.py` was
  already run to freeze the inputs (see §5).

## 3. How to run

From the repository root, on the MATLAB machine:

```bash
# 1) independent resampler reference vs C++ outputs
matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_resample_reference('cpp_dir','testdata/twr/m2a/cpp')"

# 2) decode a C++ native-chain IQ with the repository decoder
matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_decode_cpp_iq('result_json','testdata/twr/m2a/native_roundtrip_uc200_737280000.json')"

# 3) independently generate the TWR frames (writes generated/*)
matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_generate_twr_frame"
```

Paths may also be given through the environment:
`TWR_M2A_TAPS_DIR`, `TWR_M2A_CPP_DIR`.  `out_dir` / `iq_file` / `result_json`
are name/value options (see each script's header).

The C++ side must have produced, for cross-check #1, one file per direction and
case:

```
testdata/twr/m2a/cpp/<tx_48_65|tx_32_65|rx_65_48|rx_65_32>_<impulse|zeros|random|twr_waveform>_cpp.cf32
```

consuming exactly `inputs/<case>_in.cf32` with the matching frozen taps.  For
cross-check #2 the native IQ is located from the `twr-m2a-native/1` JSON: the
JSON field `artifacts.native_cf32` (preferred) or the JSON path with `.cf32`
appended, in the same directory.

## 4. What each script checks (and the frozen tolerances)

- Length is computed from the frozen formula `ceil(((N-1)*L + T)/M)` — never
  `N*L/M`.  Script #1 asserts the MATLAB output length equals it.
- Same-chain CF32 comparison (fixed normalised inputs):
  `max_abs_error <= 1e-4` and `relative_L2 <= 1e-5` (G0 §6).
  The all-zero input is defined separately and must be **exactly** zero
  sample-by-sample.
- Script #2 byte checks: `phr.psdu_length_bytes == psdu_bytes`,
  `payload.bytes` equals the independently computed `mac + FCS`, and
  `fcs_pass`.  The expected bytes come from the JSON's `frame.mac_hex` plus an
  independent CRC — **never** from the C++ `decode.payload_hex`.
- Diagnostic coordinates: packet-start and SFD-start are compared after a
  **single unified 0-based sample definition** (G0 §6: clean default
  `<= 1 work sample`, no fractional-ToA claim).  MATLAB reports 1-based
  samples/chips; the scripts subtract 1 explicitly.  The MATLAB SFD sample is
  derived from `soft_chip_timing.first_chip_sample_uncropped` and
  `sfd.start_chip` with the measured `samples_per_chip`.  If the unified
  definition turns out to differ on the MATLAB machine, **do not silently widen
  the tolerance** — update G0 and get reviewer agreement first.

## 5. Frozen inputs

`inputs/` was generated once by `gen_golden_inputs.py` (inputs only) and is
committed so the C++ and MATLAB sides consume identical bytes:

| File | n | Definition |
|---|---|---|
| `impulse_in.cf32` | 64 | `x[0] = 1`, rest exactly 0 |
| `zeros_in.cf32` | 256 | exact zeros (all-zero contract) |
| `random_in.cf32` | 4096 | complex Gaussian, seed 20261003, unit average power |
| `twr_waveform_in.cf32` | 4096 | fixed 0-based window 4992000 of the frozen repo work cfile `testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile`, peak-normalised to 1 |

`twr_waveform_in.cf32` is a real work-grid UWB waveform for resampler testing.
It is **not** a TWR frame golden; the TWR frame golden is produced
independently by script #3.

## 6. Frozen taps contract

Scripts look in `testdata/twr/m2a/taps/` (owner: agent A) for each direction
`<L>_<M>` ∈ {`48_65`, `32_65`, `65_48`, `65_32`}, in this order:

1. `design.json` (schema `twr-m2a-tx-taps/1`) with an array `directions`
   (or `taps`), each entry carrying `interp`/`decim` (or `l`/`m`, or `L`/`M`)
   and `frozen_file` (the canonical independent design; `file`/`taps_file` are
   also accepted), relative to the taps directory; or
2. a file named `taps_<L>_<M>.<ext>`, `tx_taps_<L>_<M>.<ext>`,
   `rx_taps_<L>_<M>.<ext>`, `m2a_taps_<L>_<M>.<ext>`, `tx_<L>_<M>.<ext>`, or
   `rx_<L>_<M>.<ext>` for `ext` ∈ `.f32`, `.bin` (float32 LE), `.txt`, `.csv`.

For the RX directions (`65_48`, `65_32`) agent A's `design.json` contains no
entry (it only designs the two TX directions), so the scripts fall back to the
repository's M2-A canonical RX prototypes:

- `testdata/resampler_65_48/taps_quality_minorder.txt`
- `testdata/resampler_65_32/taps_quality_minorder.txt`

The resolved tap path is recorded in every result record.  If a C++ comparison
used a different RX tap file, the comparison will fail loudly rather than
silently pass; the reviewer must reconcile the recorded path.  Missing taps are
reported as `blocked`, never as PASS.

## 7. Blocked items and their acceptance mapping

| Item | Status here | Why |
|---|---|---|
| A11 cross-check #1 (resampler) | **blocked / 未满足** | needs MATLAB + C++ `*_cpp.cf32` outputs |
| A11 cross-check #2 (decode) | **blocked / 未满足** | needs MATLAB + a `twr-m2a-native/1` JSON + native IQ |
| A11 cross-check #3 (generate) | **blocked / 未满足** | needs MATLAB + Communications Toolbox; if the toolbox cannot set the PHR/MAC ranging bit, the script records `generated/BLOCKED.json` and writes **no** waveform golden |
| A11 overall | **未满足** | MATLAB 未实跑 |

Nothing in this directory upgrades a capability: the scripts produce
`matlab_executed: true` **only** in their own runtime output, which is not the
committed manifest.  No evidence level is raised.

## 8. Provenance

`manifest.json` records `schema`, `matlab_executed: false`, the exact commands,
the expected outputs, the taps/C++ contracts, the frozen source hashes, and the
sha256 of every file in this directory (except `manifest.json` itself).
