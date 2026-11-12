# Ipatov 前导重构 `template_scale_mismatch` 定位与修复：原始数据（2026-11-12）

代码基线 `64c2aee` + 本目录对应的工作区修改（未提交）。构建目录
`gr-uwb/build-sc16-r1r2`，`env -u LD_LIBRARY_PATH` 运行。

| 产物 | 内容 |
|---|---|
| `raw/before_fix_ipatov_30win.json` | 修复前，`--sic-backend ipatov`，30 窗 |
| `raw/after_fix_ipatov_30win.json` | 修复后，同配置，30 窗 |
| `raw/after_fix_discrete_30win.json` | 修复后，`--sic-backend discrete` 对照，30 窗 |
| `raw/after_fix_ipatov_auto_3win.json` | 修复后，`--sic-policy auto`，3 窗 |
| `raw/after_fix_ipatov_fullpacket_3win.json` | 修复后，`--sic-policy fullpacket`，3 窗 |
| `costsplit/` | `analyze_sc16_cost_split.py --windows 2`，4 profile × 3 consumer |
| `g3/g3_decisions.csv`、`g3/g3_waveform.csv` | `run_equiripple_g3.py --windows 2` |

公共参数：`testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin`，
737.28M → 998.4M，`--period 140982 --t0 140982 --capture 56000`，
pre/post guard 400，`--consumer both --dsp-workers 1 --resampler-workers 1`，
`--pool-size 32`，`--sic-policy preamble --sic-comm-code 9`，
`--resampler-profile quality_minorder`。每次 runner 独占
`/tmp/uwb-sic-benchmark.lock`。

二进制/库标识（修复后）：

```
sha256 apps/uwb_uhd_sc16_pipeline_runner  5cc22ec9e70f8be780cad2273daeed06437635b02dbd04a5acd7e55dc13b3085
sha256 lib/libgnuradio-uwb.so.1.0.0.0     6da2ca6537967586ff9e96ea2d0d0bac82555becad7c846e5b9a24e0f9e183fa
```

单位 µs；`p50/mean/p95` 由逐窗 `e2e_us` 等字段直接统计，**不是**各阶段 P50
相加。`queue_us` 是全速文件回放的排队，不代表 paced 实时延迟。

| 运行 | SIC 状态 | 抑制 | sensing SFD | sic_us p50 | e2e_us p50 | p95 |
|---|---|---:|---:|---:|---:|---:|
| 修复前 ipatov | 30× `template_scale_mismatch` | 0.00 dB | 30/30 | 307.2 | 1541.0 | 1669.6 |
| 修复后 ipatov | 30× `committed_preamble_sparse_pulse` | 44.11 dB | 0/30 | 413.2 | 1643.4 | 2333.4 |
| 修复后 discrete | 30× `committed_preamble_discrete` | 32.54 dB | 0/30 | 809.5 | 1787.7 | 2009.4 |

三次运行账本均 `offered=30 / admitted=30 / completed=30`，0 丢弃、0 处理失败、
0 短读。

`sensing SFD 0/30` 不是 sensing 检测退化：该 fixture 是**同一个 code-9 包**的重复，
SIC 消除后 `--consumer both` 中的 sensing 消费者面对的是已被消除的信号。对照
`costsplit/*__sensing.json`（不跑 SIC）为 `radar_sfd_found=true`。

`queue_us` p50 约 16.7–26.3 ms，由全速回放与队列深度决定，不作为实时性结论。
本批数据只用于功能定位与冒烟，不构成 P99、实时目标或正式性能矩阵。
