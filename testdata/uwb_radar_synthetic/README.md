# Synthetic (non-canonical) UWB radar fixture

This directory is **not** the Step 1 TX golden.

Canonical packet: `testdata/uwb_radar/` from `export_uwb_radar_golden.m`
(`lrwpanWaveformGenerator`, 20 data bytes + FCS, SFDNumber=2).

This fixture splices `kron(4z2, first SYNC)` onto
`testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile`. Use it only
for algorithm smoke tests that must not depend on Communications Toolbox.

```bash
python3 testdata/uwb_radar_synthetic/generate_uwb_radar_golden.py
python3 testdata/uwb_radar_synthetic/verify_uwb_radar_golden.py
```
