# 65/32 software upsample (491.52 → 998.4 MS/s)

X410 **CG400** native rate. Contract:

```text
491.52e6 × 65 / 32 = 998.4e6
Lout = ceil(((N-1)·65 + T) / 32)
map(p) = round((p·65 + (T-1)/2) / 32)
```

Nyquist is ±245.76 MHz. UWB occupancy is ~±249.6 MHz; band edges are a
hardware limit and are not recovered by this resampler.

Regenerate:

```bash
python3 testdata/design_resampler_491p52_to_998p4.py
python3 testdata/export_resampler_65_32_golden.py --profile quality_minorder
```

Default production taps: `taps_quality_minorder.txt` (relative −70 dB,
B=220 MHz).
