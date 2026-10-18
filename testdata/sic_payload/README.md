# S2-B: sensing preamble interference on a communication payload

This directory holds the configuration, short fixtures and MATLAB reference
scripts for the S2-B study defined in
`docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md` and the frozen
contract `docs/sic/S2B_设计与测试契约.md`.

Generator / driver / verifier live in the repo source tree (not committed here):

| file | role |
|---|---|
| `gr-uwb/apps/sic_payload_sim.py` | truth-bearing signal generator (reversed roles) |
| `gr-uwb/apps/uwb_sim_sic_payload.py` | GNU Radio flowgraph / experiment driver / CLI |
| `gr-uwb/apps/test_sic_payload.py` | targeted QA (pure + build-dependent) |
| `tools/sic/payload_metrics.py` | PER / Wilson / suppression metrics |
| `tools/sic/verify_s2b.py` | independent verifier + negative self-test |

## Roles and signal model

* desired `s`  = full communication packet, **code 10**, 64 SYNC + SFD(4z2) +
  PHR + random PSDU (single FCS).  Demodulated with the code-10 profile.
* interferer `i` = **code 9** sensing preamble-only finite SYNC train (64 SYNC),
  placed entirely inside the comm payload data region; its first 8 SYNC overlap
  unknown random payload.
* `y = s + i + w`; `clean = s + w`; `sic` = `y` after the frozen S1 SIC adapter
  (code-9 templates).  Raw CF32, no per-path renormalisation; the noise
  realisation is shared across the three paired paths.

## Geometry (work grid 998.4 MS/s, `sps=1016`, 2 samples/chip)

| segment | samples | notes |
|---|---:|---|
| SYNC (64) | 65024 | code 10 |
| SFD (4z2, 8) | 8128 | code 10 |
| PHR (21) | 21504 | 0.85 Mb/s |
| payload (`nsym*128`) | 116736 | 6.81 Mb/s, `nsym=912` for 96 B PSDU |
| **packet** | **211392** | legacy pulse, tail 0 |

`psdu_bytes = 96` is the wire PSDU (94 random data bytes + 2-byte IEEE 802.15.4
CRC-16 FCS appended **once** by the generator; the modulator `append_fcs` stays
False).  The demodulator returns the whole 96-byte PSDU including the FCS and
validates it (`fcs_pass`); nothing strips it.

## Public ROI / truth firewall

Online code sees only the mixed IQ, the public PHY profiles, the public coarse
ROIs and the visible prefix.  Detection ROI `[nominal-J-64, nominal+J+64)`,
subtraction ROI `[nominal-J-64, nominal+J+D+post_tail+128)`, visible prefix
`observed_end = (nominal+J) + fit_reps*sps + 64`, with public jitter budget `J`
(calibration-frozen).  Injected start/CFO/paths/interferer IQ/payload truth live
only in the generator and the offline verifier.

## ISR / SNR

* Reference interval = comm payload region `[payload_lo, payload_hi)`.
* `SNR = 10log10(E_s/E_w)` over that region (sample SNR, not Eb/N0).
* `ISR = 10log10(E_i/E_s)` over the true overlap `i-support ∩ reference`.

## Reproduce

```bash
# targeted QA (pure + build-dependent; incl. B-R01 PER-CI numeric reference)
env -u LD_LIBRARY_PATH UWB_BUILD_DIR=gr-uwb/build-sic-payload \
    python3 gr-uwb/apps/test_sic_payload.py

# functional smoke
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py \
    --build-dir gr-uwb/build-sic-payload --smoke \
    --output gr-uwb/build-sic-payload/results/s2b_b0_smoke
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py \
    --results gr-uwb/build-sic-payload/results/s2b_b0_smoke

# staged matrix (B0 100 + B1 100 + B2 1000/cell + B3 12 cells x 500 +
# B4 1000/cell), serial
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py \
    --build-dir gr-uwb/build-sic-payload --matrix \
    --output gr-uwb/build-sic-payload/results/s2b_matrix_rectification
env -u LD_LIBRARY_PATH python3 tools/sic/s2b_aggregate.py \
    --matrix gr-uwb/build-sic-payload/results/s2b_matrix_rectification
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --self-test
```

## Rectification (2026-10-18)

* **PER CI** is the Wilson interval of the **error** proportion
  (`payload_metrics.per_ci`); the success interval, if kept, is reported
  separately as `success_wilson95`.  The original 21100 windows were re-derived
  (CI/figures only) under `results/s2b_matrix_ci_fix/` (`--legacy`), with error
  and rescue/damage counts unchanged.
* **B3** is 12 cells (early/late × {transition −3.5 dB, 30 dB}; CFO ±50 kHz ×
  ISR {0,10} × {single, four}).  The four-path channel is the FROZEN
  `main.json` profile `[[0,(1,0)],[7.4,(0,0.5)],[23,(0.276,−0.116)],[51.2,(0.2,0)]]`
  (bound by hash); the old integer-spaced profile is only `custom_integer_four`.
* The verifier is self-contained and fail-closed (strict library/archive/packet/
  plan/matrix checks, 17 negative mutations).  Every run writes `plan.json`
  before the first window; `run_matrix`/`s2b_aggregate` exit non-zero on failure.

## FCS / geometry contract

wire PSDU `psdu_bytes = 96` = 94 random data bytes + 2-byte IEEE 802.15.4 CRC-16
(appended exactly once by the generator; the modulator `append_fcs` stays False).
The demodulator returns the whole 96-byte PSDU including the FCS and validates it
(`fcs_pass`); nothing strips it.  (An earlier draft paragraph mixed this up; the
authoritative statement is here.)

## MATLAB

`matlab/` holds reference exports/checks mirroring `testdata/sic_preamble/`.
**Status: 未实跑 (NOT RUN).**  MATLAB was not executed for this study; the
scripts are provided for independent cross-checking and their outputs are not
claimed as validated.
