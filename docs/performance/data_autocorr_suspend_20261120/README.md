# 锁定后暂停自相关：回放验证数据（2026-11-20）

对应报告：
[../锁定后暂停自相关_实现与回放验证_2026-11-20.md](../锁定后暂停自相关_实现与回放验证_2026-11-20.md)

本目录只保存**小体积**解析结果与原始控制台日志。原始 SC16 回放夹具为
2.75 GiB，未入库；路径、大小与 SHA-256 记录如下。所有结果均为
**文件回放证据，未上板**。

## 1. 文件

| 文件 | 内容 |
| --- | --- |
| `abba.json` | `tools/rx/autocorr_suspend_ab.py` 的完整 ABBA 结果（8 次运行、配对统计、环境、构建、逐次解析） |
| `abba_logs/run_0[1-8]_*.log` | 上述 8 次运行的原始 stdout/stderr（`--print summary`） |
| `scan_consistency_20261120.json` | `scan` 与改动前基线的逐包判决字段对照，含基线自对照噪声底 |
| `resume_gap50ms_20261120.json` | 合成“目标停发 50 ms”夹具上的恢复/重新锁定结果 |
| `resume_logs/*.log.gz` | 上述恢复专项 7 次运行的 `--print packets` 日志（gzip） |
| `entry_boundary_20261120.json` | ABBA 中 `fcs_ok` ±1 的逐包定位（4 组 scan/suspend，993 个目标槽）；原始 11 MB 日志保留在 `/tmp/opencode/entryjitter/`，JSON 内记录路径与 SHA-256 |

`abba.json` 内已包含：提交号、二进制与夹具 SHA-256、CPU/ISA/亲和性、内核、
CMake 缓存中的构建参数、worker/queue、每轮 `LOCK_STATS`/`COST_TOTAL`/`FINAL`
解析、ABBA 配对差值中位数/均值。

## 2. 回归夹具

主夹具（1 ms 周期、连续发送、1.000 s）：

```text
/tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16
size   : 2,949,120,000 B (737,280,000 complex int16)
sha256 : 66b2a0d51669305d828548c8e3fb32d9e475c5a22fa7c2f2fe8220f519b4bfb4
来源   : 2026-11-20 DW3000 1 ms 实板采集，采集命令与哈希见
         ../DW3000_1ms实板采集_Payload失败分析_2026-11-20.md
```

`testdata/dw3000_6489p6MHz_737p28Msps_1p0s_sc16_20260825_0{1,2}.dat`
在默认 5 ms 与 1 ms 下均未进入稳定锁定（0.3 s 试跑 `lock_entries=0`），
因此未作为本轮夹具。

合成停发夹具（仅用于失锁恢复专项）：

```text
/tmp/opencode/uwb-live-1ms/dw3000_1ms_gap50ms.sc16
size   : 2,359,295,996 B
sha256 : 242c459a63c6cf5d9b9403dbed8e624efea3fc192f0d9524036b016f1e3e2646
构造   : 0.40 s 原采集 + 0.05 s 数字静默 + 从 0.45 s 起 0.35 s；
         1 ms 包周期相位保持不变。使用 tools/rx/make_lock_loss_fixture.py。
```

它是连续样本时间线上的“目标停发”，**不产生 `RX_GAP`**；设备真实
gap/overflow 路径未在本轮验证。

## 3. 命令

ABBA（串行，构建与 ctest 完成后单独运行；子进程环境移除
`LD_LIBRARY_PATH`）：

```sh
python3 tools/rx/autocorr_suspend_ab.py \
  --replay /tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16 \
  --period-ms 1 --workers 2 --queue 24 --rounds 4 \
  --out docs/performance/data_autocorr_suspend_20261120/abba.json
```

`scan` 一致性（改动前基线二进制从 `5ce858a` + 当时工作树改动构建；
逐包按 `time_ms` 对齐，只比较判决字段与非派生数值）：

```sh
uwb_dw3000_live --replay .../dw3000_1ms_1s.sc16 --period-ms 1 \
  --seconds 0 --tone off --workers 1 --print packets \
  --sync locked --interference off --fused on --cir full --taps eq16 \
  --autocorr-when-locked scan
```

失锁恢复：

```sh
uwb_dw3000_live --replay .../dw3000_1ms_gap50ms.sc16 --period-ms 1 \
  --seconds 0 --tone off --workers 2 --print packets \
  --sync locked --interference off --fused on --cir full --taps eq16 \
  --autocorr-when-locked suspend [--autocorr-resume-lookback 0|200000]
```

## 4. 关键结果

- ABBA 4 配对：`frontend_scan_us` 扫描均值 **125,473 µs → 1,877 µs**
  （配对中位数 −123,640 µs，−98.5%）；`acq_seeds` 1002 → 6.5。
- `fcs_failed=2`、`identity_rejects=2`、`lock_entries=1` 每轮一致；
  8 次运行 `gaps=0`、`raw_drop_samples=0`、`source_errors=0`。
- ABBA 第 1 配对 `fcs_ok` 为 997 vs 996；4 组逐包配对显示 993 个目标槽
  判决完全相同，差异只有进锁前 5.577 ms 的 `path=acquire` 包（见
  `entry_boundary_20261120.json`）。
- `scan` 一致性：4 组对照的 status/FCS/PSDU 判决字段 0 差异；包数
  998 vs 999 的差异在**同一基线二进制自对照**中也出现，属于启动边界。
- 停发恢复：7 次运行（scan×2、suspend 默认×3、lookback 0/200000）
  均为 `expected_slots=749`、`fcs_ok=747`、首个 target 4.577 ms，
  相对 `scan` 目标判决 0 差异；默认 backlog=642,000，`0` 为 0，
  重新锁定时刻相同（449.742/450.577/451.577/453.577 ms）。

## 5. 源码/二进制溯源

- HEAD：`5ce858aac9ebf736dd59bba738dcb62e6530a874`（分支
  `feature/sic-research`）。
- 工作树包含本任务改动与既有未提交的 `--sfo-from-cfo` 等改动；
  `abba.json.git.status_porcelain` 记录了完整状态。二进制 SHA-256
  `ea3bd39b7e030e0257ca488a57b9fc75762212b43b2a837e30390d14d01908f8`。
- 本任务源码 SHA-256：

| 文件 | SHA-256 |
| --- | --- |
| `uwb_autocorr_gate.h` | `be9ff0a9da22a8b8727c2b54adc401eb27646e3572f348878a6d5170c689e453` |
| `qa_uwb_autocorr_gate.cc` | `18b545e3366c105534f7b06ae4c8402d4c201be71d356fe0da24993fbeb5f33f` |
| `uwb_dw3000_live.cc` | `bcf87ded0dd8c1453cc93aa118af58bc932b5f5c70200b3e062aa9dbf1b0c96b` |
| `lib/CMakeLists.txt` | `b28e3f6a45fed9a985faabd27680f27ec5361e1eaf3e60071ba0e4e1584659f6` |
| `autocorr_suspend_ab.py` | `92d4a513867b2a32e37e10d4ebda0c446c6b147eb141237ce8f590d65ac2ff78` |
| `make_lock_loss_fixture.py` | `f809850a0b3ad55e131d89ef44d6d8aaa7a00648c51c81b8f3409070c03f2811` |
| `qa_live_dw3000_console.py` | `ce9cc9733398f719df937ad77b4c5a46931a90bc5b1754ee88e387dea6d8dc39` |

完整哈希以上述文件当前内容为准；构建参数记录在
`abba.json.build.cmake_entries`。
