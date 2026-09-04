# Complete 32/128-SYNC radar packet goldens

Generator: `../export_uwb_radar_packet.m` via `../run_export_radar_packets.m`.

These are full pulse-shaped packets (SYNC + 4z2 SFD + PHR + PSDU/FCS),
not `N × reference_preamble.bin + kron(SFD, SYNC)` fragments.

IEEE 802.15.4z **BPRF** `PreambleDuration` in MATLAB Communications Toolbox
is 16/64/1024/4096. 32 and 128 are HPRF-only in `lrwpanHRPConfig`, so this
path starts from a MATLAB **BPRF 64-SYNC** complete packet:

- `sync32`: first 32 pulse-shaped SYNC symbols + original SFD/PHR/PSDU/FCS
- `sync128`: SYNC field concatenated twice + original SFD/PHR/PSDU/FCS

`metadata.profile_kind` = `custom_sync_length`. Native files are one-shot
`resample(x,48,65)` of the entire reconstructed packet.
