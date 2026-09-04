# Canonical UWB monostatic-radar TX / RX golden

**Generator of record:** `export_uwb_radar_golden.m`
(`lrwpanWaveformGenerator`, Communications Toolbox).

Default PHY:

- IEEE 802.15.4z BPRF / HRP **code index 9**
- **64** SYNC
- **SFDNumber=2** (4z2)
- **20 data bytes + IEEE 802.15.4 FCS** (rng seed 20260904)
- Work rate **998.4 MS/s**; native TX **737.28 MS/s** = one-shot `resample(x,48,65)` of the **entire** packet

Do **not** build native TX by repeating the 751-sample one-SYNC detector
template. Do **not** overwrite these files with the Python splice fixture;
that lives in `testdata/uwb_radar_synthetic/`.

Complete 32/128-SYNC packets (same PHR/PSDU/FCS construction) live in
`packets/sync32/` and `packets/sync128/`. BPRF `lrwpanHRPConfig` only
allows preamble durations 16/64/1024/4096, so those two are **custom
SYNC-length** profiles: MATLAB generates a 64-SYNC complete packet, then
keeps the first 32 pulse-shaped SYNC symbols or concatenates the SYNC
field twice, keeping the original SFD/PHR/PSDU/FCS. Native TX is still
one-shot `resample(x,48,65)` of the **entire** reconstructed packet.
`metadata.profile_kind` is `custom_sync_length`. Regenerate with:

```matlab
cd testdata/uwb_radar
run_export_radar_packets
```

## How to regenerate

From the repo, with MATLAB + Communications Toolbox:

```matlab
cd testdata/uwb_radar
export_uwb_radar_golden
verify_uwb_radar_golden
```

`verify_uwb_radar_golden` is a hard gate: CIR mismatch or exception prints
`FAIL` and errors. Missing toolbox prints `SKIP/UNAVAILABLE` (not PASS).

## Coordinate conventions

- Sample indices are **0-based**.
- `dtype` is **complex64** interleaved little-endian.
- CIR is estimated at 998.4 against the **TX-time / predicted** SYNC origin,
  so an extra channel delay `D` samples shifts the peak by `D` taps relative
  to the clean CIR. The clean peak is typically `pre` plus a pulse-shape
  offset; that offset is an algorithm coordinate, not physical zero range.
- Fractional delay uses MATLAB `interp1(..., 'pchip')`. That is the
  canonical interpolator; Python `ndimage.shift` is not.

## Required files

TX/RX IQ, plus raw and L2-normalized CIR for clean, integer-delay, and
fractional-delay radar windows (`pre=16`, `post=100`). See `metadata.json`
(script-generated; do not hand-edit). `metadata.generator` must remain
`export_uwb_radar_golden.m`.
