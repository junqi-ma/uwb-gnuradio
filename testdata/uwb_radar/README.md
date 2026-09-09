# Canonical UWB monostatic-radar TX / RX golden

**Generator of record:** `export_uwb_radar_golden.m`
(`lrwpanWaveformGenerator`, Communications Toolbox).

Default PHY:

- IEEE 802.15.4z BPRF / HRP **code index 9**
- **64** SYNC
- **SFDNumber=2** (4z2)
- **20 data bytes + IEEE 802.15.4 FCS** (rng seed 20260904)
- Work rate **998.4 MS/s**; native TX **737.28 MS/s** = one-shot `resample(x,48,65)` of the **entire** packet; CG400 native **491.52 MS/s** = `scipy.signal.resample_poly(x,32,65)` (`export_tx_491p52.py`)

Do **not** build native TX by repeating the 751-sample one-SYNC detector
template. Do **not** overwrite these files with the Python splice fixture;
that lives in `testdata/uwb_radar_synthetic/`.

Selectable SYNC lengths are **32, 64, 128, 256, 512, 1024, 2048**.
The **packet generator default PSDU is 0 bytes**. Canonical CIR golden
files in this directory stay **20 data bytes + FCS** and must not be
overwritten by 0-byte exports.

BPRF `lrwpanWaveformGenerator` native `PreambleDuration` is 16/64/1024/4096.
Other SYNC lengths crop/tile the pulse-shaped SYNC field of a 64- or
1024-SYNC complete packet. Native TX is one-shot `resample` of the entire
packet (48/65 and 32/65). Generate with:

```bash
python3 testdata/uwb_radar/generate_uwb_radar_tx.py \
  --sync-reps 256 --psdu-bytes 0 --out-dir /tmp/radar_tx256
python3 testdata/uwb_radar/generate_uwb_radar_tx.py \
  --sync-reps 64 --psdu-hex AABBCCDD --out-dir /tmp/radar_tx_hex
```

```matlab
cd testdata/uwb_radar
export_uwb_radar_packet(fullfile(pwd,'packets','sync256_psdu0'), 256, 'PSDUBytes', 0)
run_export_radar_packets   % all SYNC lengths, default PSDU=0, plus 32/128 20-byte e2e goldens
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
- CF32 descriptor: `sample_format=fc32`, `dtype=complex64`, 8 bytes per
  complex sample, interleaved little-endian IQ.
- SC16 descriptor: `sample_format=sc16`, `dtype=int16`, 4 bytes per complex
  sample, interleaved little-endian IQ.
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

`sync_template_998p4.cf32` is the first 1016 samples of `tx_998p4.cf32`
(one SYNC symbol at 998.4 MS/s). It is the canonical `UwbRadarCirEstimator`
template and matches the template written on the fly by the block QAs.
Regenerate from the golden with:

```python
open("sync_template_998p4.cf32", "wb").write(
    open("tx_998p4.cf32", "rb").read(1016 * 8))
```

SC16 storage of the same TX packets (`tx_998p4.sc16`, `tx_737p28.sc16`) is
little-endian interleaved int16 IQ, `round(clip(x * 32767, -32768, 32767))`.
`tx_998p4_sc16_head16.i16` is the first 16 samples of `tx_998p4.sc16` for
byte-order checks. Sample counts match the CF32 files / sidecar lengths.
`metadata.json` describes the CF32 representation; `metadata.sc16.json`
describes the SC16 representation. PacketSource automatically prefers the
format-specific sidecar, or callers may pass it as `descriptor_path`.
