# MATLAB reference — S2-B (STATUS: 未实跑 / NOT RUN)

MATLAB was **not executed** for this study.  These scripts are provided so an
analyst with the shared, read-only `UWB_demodulation/` toolbox can independently
cross-check the C++/Python geometry and payload bit mapping.  Their outputs are
**not** claimed as validated results.

The scripts are read-only in intent: they read the shared toolbox and any
exported C++ fixtures, and write only into this directory's `out/` (gitignored).

## `export_payload_geometry.m`
Uses `lrwpanWaveformGenerator` / `lrwpanHRPConfig` (same toolbox the C++ core
mirrors) to emit, for the frozen profile (code 10, 64 SYNC, 4z2, 6.81 Mb/s,
PSDU 96 B):

* `out/comm_sync_code10_998p4.cf32`  — one shaped SYNC symbol (1016 samples)
* `out/comm_packet_code10_psdu96.cf32` — a full packet for one known PSDU
* `out/payload_field_offsets.json` — SYNC/SFD/PHR/payload/FCS sample intervals
* the CRC-16 of the PSDU data so the single-FCS contract can be checked.

## `verify_s2b_payload.m`
Given a C++ run directory `<run>` (with `templates/` and `recon/*.npz`
exported), it:

1. reproduces the finite code-9 interferer train from the shared toolbox and
   compares it to `<run>/templates/sensing_preamble_code9_998p4.cf32`;
2. compares the code-10 SYNC template to the MATLAB-generated symbol;
3. checks the payload field offsets used by the driver against the toolbox
   `PreambleDuration`/`PSDULength` geometry;
4. cross-checks the archived `i_true`/`clean`/`out` suppression.

Run (once a MATLAB with the toolbox is available):

```matlab
cd testdata/sic_payload/matlab
export_payload_geometry
verify_s2b_payload('../..');   % or an explicit run dir
```

Until then every claim in the report that depends on MATLAB is marked 未实跑.
