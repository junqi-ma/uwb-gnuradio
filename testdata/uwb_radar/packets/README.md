# Radar packet goldens (SYNC length × PSDU)

Generator: `../export_uwb_radar_packet.m` or
`python3 testdata/uwb_radar/generate_uwb_radar_tx.py`.

Allowed SYNC repetitions: **32, 64, 128, 256, 512, 1024, 2048**.

IEEE 802.15.4z **BPRF** `PreambleDuration` in MATLAB Communications Toolbox
is 16/64/1024/4096 at `lrwpanWaveformGenerator` validation. Other lengths
start from a MATLAB **BPRF 64-SYNC** (N≤256) or **1024-SYNC** (N>256)
complete packet and crop/tile the pulse-shaped SYNC field. Native files are
one-shot `resample` of the entire reconstructed packet (48/65 and 32/65).

## PSDU

| 方式 | 参数 | 结果 |
|---|---|---|
| 默认 | `--psdu-bytes 0` | `PSDULength=0`，无 MAC payload |
| 随机内容 + 字节数 | `--psdu-bytes N` | N 随机数据字节 + IEEE 802.15.4 FCS（可用 `--no-fcs` 关掉） |
| 指定内容 | `--psdu-hex AABB…` | hex 即为完整 PSDU，不再追加 FCS |

PSDU 总长 ≤ 127。默认 **0 Byte** 只作用于生成器；仓库里的 canonical
`testdata/uwb_radar/tx_998p4.cf32` 仍是 20 数据字节 + 2 FCS，CIR golden 不改。

## Layout

- `sync32/` / `sync128/`: 20-byte data + FCS goldens used by C++ e2e
  `e2e_sync_repetitions`.
- `syncN_psdu0/`: optional 0-byte PSDU packets from
  `run_export_radar_packets.m` (large N are generated on demand, not
  necessarily committed).
