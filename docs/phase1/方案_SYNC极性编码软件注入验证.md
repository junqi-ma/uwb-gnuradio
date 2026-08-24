# 方案：SYNC 极性编码软件注入验证（交替码，notched，64 SYNC）

> 分支：`experiment/sync-polarity-validation`  
> 日期：2026-08-24  
> 输入契约：`737.28 MS/s SC16`，与 `qm35_high_power_dw1000_mix_notched_20260817` 一致  
> 关联分析：`UWB模拟域SYNC极性编码干扰抑制分析.md` §1--§12

## 1. 目标与边界

在 QM35-only / DW1000-only 两份已去单音的 0.5 s 捕获上，用理想软件注入验证交替极性码 `c_k = (-1)^k` 的机制。只回答文档结论里的前两项：

* 理想编码是否破坏 QM35 之外的 DW1000 SYNC 相干积累
* 目标回波在零时延锚点下是否保持（FCS 不劣化）

明确不验证：分数时延/距离相关保持、真实模拟开关瞬态、SIC 联合。

## 2. 信号模型（与分析文档 §2 一致）

```
q(t)          : QM35 clean 接收波形（tone-removed）
d(t)          : DW1000 clean 接收波形（tone-removed）
c(t)          : 逐 QM35 SYNC 的交替码，64 符号/槽，槽外 c=+1
q_c(t)=c(t)q(t)
m(t)=q_c(t)+g d(t)          # ADC 域，存为 *_raw.dat
r(t)=c(t)m(t)=q(t)+g c(t)d(t)  # RX 恢复后，供 MATLAB 解调，存为 *_decoded.dat
```

`g ∈ {1.0, 0.501187, 0.251189}` 对应 0 / -6 / -12 dB。落盘与 `qm35_high_power_dw1000_mix_notched_20260817` 的 `gain_power_db`、`sample_format=sc16`、`complex_sample_count=368640000`、`file_bytes=1474560000` 契约一致。

## 3. 输入资产

| 项 | 路径 | 说明 |
|---|---|---|
| QM35 clean (notched) | `/mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/tone_removed_inputs/qm35_clean_tone_removed.dat` | 1.4 GiB，368640000 复样点 |
| DW1000 clean (notched) | `.../dw1000_clean_tone_removed.dat` | 同上 |
| 窗几何 | `/mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/capture.jsonl` | 99 窗，`pre 221184 / body 140083 / post 73728`，与 §3 一致 |
| 逐槽锚点 | `.../scheduled_dump_cpp.csv` 的 `qm35_det_minus_pred` | 用于 `anchor = predicted + round(det_minus*48/65)` |
| 737.28 模板 | `testdata/reference_preamble_code9_737p28.cf32` (751 点) | 仅作文档/可选相关锚定 |

QM35 配置：`code 9 / 64 SYNC / 4z2`，`FS737=737.28e6`。

## 4. 编码细节

* 一个 HRP SYNC = 1016 @998.4 → `Ts = 1016*48/65 = 750.276923` @737.28。
* 每个槽的 64 个 SYNC 区间按  
  `b_k = round(anchor + k*Ts)`，`b_{k+1}=round(anchor+(k+1)Ts)`  
  独立计算，禁止按 750/751 累加。64 符号总跨 `~48017` native 样点（~65.1 us）。
* 交替码 `c_k = (+1 iff k even)`，区间内 I/Q 同乘 `c_k`。
* 99 个槽（含 acquisition）共 6336 区间，总计 `4753782` 复样点（`6447.7 us`）。

锚点优先用 CSV 的 `det_minus_pred`，缺失时回退到 `predicted_start`。已在 `testdata/generate_sync_polarity_mix.py:load_csv_anchors` / `build_intervals` 实现，并对 `schedule_index=-1` 的 acquisition 保留 `packet_id` 映射。

## 5. 生成器实现

`testdata/generate_sync_polarity_mix.py`

* 分块 `chunk_complex=4M`，`memmap` + `float32` 运算，饱和裁剪统计与 `generate_high_power_dw1000_mix.py` 同口径。
* 每 chunk 对 `coded_q` 与 `mixed_raw` 分别在 `c=-1` 区间翻转，保持 `r = c*m` 的恒等式在量化前成立（冒烟实测 `mean|m_raw - (-q+d)| = 0`）。
* 产物（每 gain）：

```
qm35_sync_polarity_gainN_raw.dat              # ADC 域
qm35_sync_polarity_gainN_raw_metadata.json
qm35_sync_polarity_gainN_decoded.dat          # RX 恢复后，供 MATLAB
qm35_sync_polarity_gainN_decoded_metadata.json
manifest.json / manifest_raw.json / manifest_full.json  # 与之前 gain1 结构一致
dump_gainN/capture.iq + capture.jsonl         # 若 --write-dumps，由 decoded 文件按同一 window_start/predicted_start 切窗
```

`--write-dumps` 生成的 dump 与 `docs/phase1/下一步_锁定后SC16截取与DW1000头尾预留.md` §4 契约一致：`sample_rate=737280000`、`sample_format=sc16`、`file_offset_samples` 连续，可直接喂 `testdata/decode_scheduled_sc16_dump.py`。

复现（全量 0.5 s）：

```bash
python3 testdata/generate_sync_polarity_mix.py \
  --clean /mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/tone_removed_inputs/qm35_clean_tone_removed.dat \
  --interference /mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/tone_removed_inputs/dw1000_clean_tone_removed.dat \
  --geometry-json /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/capture.jsonl \
  --csv /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/scheduled_dump_cpp.csv \
  --output-dir /mnt/f/UWB基带数据/qm35_sync_polarity_notched_20260824 \
  --gains 1.0 0.5011872336272722 0.251188643150958 \
  --write-dumps
```

冒烟（0.08 s，60M 复样点）：追加 `--max-complex 60000000 --output-dir /tmp/qm35_polarity_smoke --write-dumps`。

## 6. 分析链路

* Python（本机可跑）：`testdata/decode_scheduled_sc16_dump.py` 已在 65/48 `quality_minorder` + `UwbRealtimeDemodulator (code9/64/4z2/bypass)` 验证通过；`testdata/analyze_sync_polarity.py` 聚合 `dump_gain*/scheduled_dump.csv`。
* MATLAB：`UWB_demodulation/run_analyze_sync_polarity.m` 循环调用 `decode_scheduled_sc16_dump`，输出 `sync_polarity_summary.csv`（`n_windows / fcs_pass / timing_median / sfd_median`）。CIR pre-path SIR 可在该脚本内追加 `decode_uwb` 的 `result.cir` 统计。

解码命令：

```bash
for g in 1 2 3; do
  PYTHONPATH=$PWD/gr-uwb/build/test_modules LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
    python3 testdata/decode_scheduled_sc16_dump.py /mnt/f/UWB基带数据/qm35_sync_polarity_notched_20260824/dump_gain$g
done
python3 testdata/analyze_sync_polarity.py \
  --root /mnt/f/UWB基带数据/qm35_sync_polarity_notched_20260824 \
  --clean-dump /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump
```

或 MATLAB：`matlab -batch "run_analyze_sync_polarity"`。

## 7. 首轮实测（2026-08-24，全量 0.5 s，notched，交替码）

| 数据集 | n_windows | FCS | 备注 |
|---|---:|---:|---|
| clean baseline (`qm35_clean_scheduled_sc16_dump`) | 99 | 98/99 | 1× payload_failed，固有 |
| uncoded notched gain1 (`qm35_clean_plus_dw1000_gain1`) | 99 | 98/99 | dump 由同一几何切窗，baseline |
| uncoded notched gain2 | 99 | 99/99 |  |
| uncoded notched gain3 | 99 | 99/99 |  |
| **polarity decoded gain1** (`dump_gain1`) | 99 | **99/99** | +1 相对 uncoded gain1 |
| polarity decoded gain2 | 99 | 98/99 |  |
| polarity decoded gain3 | 99 | 99/99 |  |
| polarity raw gain1 (无 RX 恢复) | 99 | 6/99 | 大面积 `sfd_failed`，证明必须 RX 恢复 |

* 细节：`gain1_decoded` 在 99 窗上全部 FCS pass，而 uncoded gain1 为 98/99；表明交替码在 0 dB 干扰下至少不劣化，并有 1 包的边际增益。`gain2/3` 保持 98/99--99/99，与 baseline 同等。
* 对照：同一 `gain1_raw`（仅 TX 编码，不做 RX 恢复）仅 6/99，`timing` 亦失锁，直接验证“只翻转 QM35 相邻 SYNC 不做 RX 恢复 = 破坏 QM35”的分析结论。
* 冒烟（16 窗，`--max-complex 60M`）同样得到 `decoded 16/16` vs `raw 0/16` vs `uncoded 16/16` 的分明对照。

未做：分数时延、距离相关保持、DW SYNC 逐符号 `|Σc_k y_k|²` 直测（已在分析文档 §6 列为实验 C），留下一轮。

## 8. 风险与下一轮

* 分数时延未扫：当前边界按整数 `round` 对齐，真实模拟开关的 1--10 ns 过渡需在对应脚本中加入线性/升余弦过渡模型。
* 距离相关保持仅在零时延锚点验证：下一轮应注入 `τ = d*T_s + δ` 的延迟线，测量 `G_s(d)` 与 CIR 主峰损失。
* 异步包偏移未扫：当前 DW 与 QM35 相对时延由两份独立 0.5 s 文件的采样 0 对齐决定（任意异步实例）；下一轮应对 `q ∈ [0, T_s)` 做连续偏移扫描，并报告 `B_max(L)`。

## 9. 交付物

* `testdata/generate_sync_polarity_mix.py`（本方案生成器）
* `testdata/analyze_sync_polarity.py`（Python 汇总）
* `UWB_demodulation/run_analyze_sync_polarity.m`（MATLAB 汇总）
* 落盘：`/mnt/f/UWB基带数据/qm35_sync_polarity_notched_20260824/`（`*_raw.dat` / `*_decoded.dat` / `dump_gain*/` / `manifest*.json`），契约与 `qm35_high_power_dw1000_mix_notched_20260817` 一致
* 本文档

后续：按分析文档 §6 的实验 B--D 补齐 `ρ_SYNC`、二维偏移热图与目标距离扫描后，再评估硬件开关选型。
