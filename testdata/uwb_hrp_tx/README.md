# HRP TX generator goldens

P1 frozen profile (IEEE 802.15.4a, no STS):

- 998.4 MS/s, code 9, 64 SYNC, IEEE SFD, 6.81 Mb/s, PSDU 127 B
- packet length 249280
- FCS 0x584b
- IQ golden: `../realtime_demod_golden/window.cfile` samples `[9984 : 9984+249280]`
- payload bytes: `../realtime_demod_golden/stage_payload_bytes.bin`

`pulse_48.f32` is the 48-tap real causal FIR that maps the sparse
`sampled_code` (even samples) onto `reference_preamble.bin`.  Fitted by
least squares; relative residual on one SYNC is ~3e-8.

`sts_drbg_bprf.bin` is the first 32×16 bytes of MATLAB
`allDRBG_STS.mat` (toolbox default key/nonce).  BPRF STS is
gap(512) + 32×(128 bits spread-by-8) + gap(512) chips.

Do not overwrite `../uwb_radar/tx_998p4.cf32`.
