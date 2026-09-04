# UWB monostatic-radar TX / RX golden

Frozen PHY profile for Step 1 of the spontaneous-TX radar path:

- IEEE 802.15.4z BPRF / HRP **code index 9**
- **64** SYNC repetitions
- **4z2** SFD `[-1,-1,-1,1,-1,-1,1,-1]`
- Full legal PHR / PSDU / FCS (MATLAB) or original PHR+PSDU kept (Python splice)
- Work rate **998.4 MS/s**, native TX **737.28 MS/s** = one-shot 48/65 of the **entire** packet

Do **not** build native TX by repeating the 751-sample one-SYNC detector template.
The true native SYNC period is `1016 * 48 / 65 = 750.276923...` samples; only a
continuous resample of the full 998.4 packet preserves fractional chip phase.

## How to regenerate

From the repo root.

Python (this machine; required for C++ QA):

```bash
python3 testdata/uwb_radar/generate_uwb_radar_golden.py
python3 testdata/uwb_radar/verify_uwb_radar_golden.py
```

MATLAB (generator of record when Communications Toolbox is present):

```matlab
cd testdata/uwb_radar
export_uwb_radar_golden
verify_uwb_radar_golden
```

`export_uwb_radar_golden.m` uses `lrwpanHRPConfig` (`Mode='BPRF'` if available,
else `802.15.4a` with `SFDNumber=2`), `CodeIndex=9`, `PreambleDuration=64`,
`SamplesPerPulse=2`, `MeanPRF=62.4`, `DataRate=6.81`, rng seed **20260904**,
20 data bytes + IEEE 802.15.4 FCS via `uwbdecoder.ieee802154CRC16`, peak
normalize to 0.8, and `resample(x,48,65)` for native TX.

The Python fallback does **not** call the toolbox. It loads
`testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile` (packet start
**4992000**, 0-based), keeps the original 64 SYNC symbols, replaces the 8-symbol
IEEE SFD with `kron(sfd4z2, first_SYNC)` at natural amplitude, keeps the
original PHR+PSDU, then peak-normalizes the assembled packet to 0.8.

## Coordinate conventions

- Sample indices are **0-based**.
- `dtype` is **complex64** (IEEE float32 I/Q interleaved, little-endian).
  Written with `ndarray.tofile` / MATLAB `fwrite(...,'float32')`.
- Work-rate files are at **998400000** Hz; native TX at **737280000** Hz.
- In `tx_998p4` coordinates: SYNC origin = 0, SFD start = `64 * 1016`.
- RX windows: `pre_guard = round(2e-6 * 998.4e6)` leading zeros, then the
  packet (optionally delayed), then `4096` trailing zeros. No CFO.
- Integer-delay RX: delay **37** samples, gain `0.4 * exp(1j*0.7)`. SYNC
  origin and SFD start in that file are the clean coordinates **plus 37**.
- Fractional-delay RX: delay **12.4** samples, gain `0.55 * exp(-1j*1.1)`,
  cubic spline (`scipy.ndimage.shift`, order 3, constant 0). MATLAB uses
  `interp1(..., 'pchip')` as the closest toolbox equivalent.
- CIR is estimated at 998.4 against the **predicted / TX-time** SYNC origin
  (clean origin, not the delayed one), so a channel delay of 37 samples
  shifts the radar CIR peak by 37 taps. Zero extra delay is recorded as
  expected peak tap `pre`; the unshaped `sampled_code` correlator typically
  lands ~2 taps later (HRP pulse-shape peak).
- Radar CIR window: `pre=16`, `post=ceil(2*15/c*998.4e6)=100`, `c=299792458`.
  Demod-comparable window `pre=8, post=30` is also exported.
- CIR **raw** = `conv(average, flipud(conj(sampled_code)), 'valid') / code_energy`
  (no L2). CIR **norm** = L2-normalized raw. Skip first 10 SYNC, average 54.

Native 48/65 uses `scipy.signal.resample_poly(..., window=("kaiser", 5.0))`
with FIR `ntaps = 2*10*max(48,65)+1`. The prototype group delay
`(ntaps-1)/2/65` native samples is recorded in metadata (same contract as
`generate_qm35_reference_737p28.py`). scipy and MATLAB `resample` both
trim/compensate that delay in the returned vector; C++ FIRs that do not
trim should apply the recorded delay. The native **full packet is not
L2-normalized**.

## File list

| File | Contents |
|---|---|
| `tx_998p4.cf32` | Full TX packet, no silence, 998.4 MS/s |
| `tx_737p28.cf32` | Same packet after one-shot 48/65 |
| `rx_clean_998p4.cf32` | Guard + TX + tail, delay 0, gain 1 |
| `rx_delay_int_998p4.cf32` | Same window, integer delay 37 |
| `rx_delay_frac_998p4.cf32` | Same window, fractional delay 12.4 |
| `cir_raw_clean_radar.cf32` / `cir_norm_clean_radar.cf32` | Radar CIR, clean RX |
| `cir_raw_clean_8_30.cf32` / `cir_norm_clean_8_30.cf32` | Demod 8/30 CIR, clean RX |
| `cir_raw_delay_int_radar.cf32` / `cir_norm_delay_int_radar.cf32` | Radar CIR, integer-delay RX |
| `cir_raw_delay_int_8_30.cf32` / `cir_norm_delay_int_8_30.cf32` | Demod 8/30 CIR, integer-delay RX |
| `metadata.json` | Script-generated; do not hand-edit |

`metadata.json` records dtype, rates, code/SFD/SYNC, peak, lengths, 0-based
SYNC origin and SFD start in TX / clean RX / integer-delay RX, resample
interp/decim/half_length/group delay, CIR pre/post/skip and expected peak
taps (clean = `pre`, integer delay = `pre+37`).
