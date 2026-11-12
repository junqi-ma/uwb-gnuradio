# Ipatov 前导重构 `template_scale_mismatch` 定位与修复报告（2026-11-12）

任务来源：`docs/performance/OpenCode_单worker测试配置修正_2026-11-12.md`。该任务书
要求先定位「配置修正后仍报 `template_scale_mismatch`」这一重构错误，并且明确
**不能把失败退出的耗时当作有效 SIC 性能**。

结论：错误已定位并修复。同一单配置功能检查现在 30/30 窗提交
（`committed_preamble_sparse_pulse`），实测抑制 **44.11 dB**，全量 CTest
**80 项 79 通过 1 跳过**。本文所有数字绑定构建目录 `gr-uwb/build-sc16-r1r2`、
基线 `64c2aee` + 本文所述工作区修改，原始 JSON 见
`docs/performance/data_sic_ipatov_prefix_fix_20261112/`。

---

## 1. 复现

按任务书给出的单配置功能检查（30 窗）复现，SIC 0/30 提交，全部
`template_scale_mismatch`，sensing SFD 30/30：

```
--sic-policy preamble --sic-backend ipatov --sic-comm-code 9
--period 140982 --t0 140982 --capture 56000 --pre-guard 400 --post-guard 400
--consumer both --dsp-workers 1 --resampler-workers 1 --pool-size 32 --max-windows 30
--resampler-profile quality_minorder
```

即任务书描述的「配置修正已生效（检测通过）、但重构仍拒绝」的状态。

## 2. 定位过程

在 `uwb_sic_unified.cc` 的 L1 分支与 `uwb_discrete_cir_reconstruct.cc` 的
`apply_discrete_cir_impl` 三处 `template_scale_mismatch` 出口临时加打印，重建库后
单窗复现，得到：

```
[DBG] L1 branch: impulse_template=1 sparse=1 cached_events=0 cached_samples=0
[DBG] prepare pulse_list ok=0 count=0 pkt=0 cap=77429 prof_code=9 shape=3 taps=2 n_taps=2 back=2
[DBG] ppl: prefix_samples=94656 cap=77429
[DBG] scale_mismatch at template_error=62.8584 limit=6.4e-09 (scale=1.06845)
```

因果链（三段，缺一不可）：

1. **`modulate_prefix_pulse_list()` 声明的长度含 PHR。** 它用
   `packet_samples_998p4(sync_reps, n_sfd, 0, ...)` 计算 `packet_samples`，该函数
   在 `psdu_bytes=0` 时仍加上 `kPhrSymbols*kPhrChipsPerSymbol*2 = 21504` 样点。
   于是「前缀」长度被报成 `73152 + 21504 = 94656`，而事件表实际只覆盖
   SYNC+SFD 的 73152 样点。同一文件里的 `modulate_prefix_waveform()` 报的是
   73152(+n_taps)，两者本来就不一致。
2. **窗口容量 77429 < 94656。** `derive_sic_geometry()` 把
   `max_packet_samples` 设成 work 窗容量（`ceil(56800·998.4/737.28)+512 = 77429`），
   于是 `94656 > sample_cap`，`modulate_prefix_pulse_list()` 返回 `false`，
   `prepare_sic_unified()` 把 `cached_preamble_events` 清空。
3. **稀疏分支被跳过，落到非稀疏分支。** L1 分支要求
   `dm.impulse_template && sparse_pulse_reconstruct` 且 `pulses.count > 0`；事件表为空
   所以走 `apply_discrete_cir()`，传入的是**成形**前缀波形
   （`cached_preamble_waveform`），而 Ipatov 模型的 `reference_sync` 已被替换为
   **单位冲激**参考（`scratch.impulse_sync`）。尺度拟合得到 `scale=1.068`，
   模板误差 `62.86` 对上限 `6.4e-9`，于是 `template_scale_mismatch`。

第 3 段本身是第二个独立缺陷：Ipatov 后端下非稀疏 L1 路径**永远不可能提交**，
因为它的输入波形语义与模型语义相反。`sparse_pulse_reconstruct` 默认 `true` 掩盖了
它；一旦显式关掉稀疏表示，该路径就是死路。

## 3. 修复

### 3.1 `modulate_prefix_pulse_list()` 报告真实前缀跨度

`gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h`：改为与
`ensure_prefix_events()` / `modulate_prefix_waveform()` 同源的
`(sync_chips + sfd_chips + sts_chips) * kQm35SamplesPerChip`，并新增两条硬约束：

- `prefix_samples == 0` 拒绝；
- 最后一个事件的 `sample_offset` 必须严格小于声明跨度，否则拒绝（事件不能落在
  声明支持之外）。

前缀消除只覆盖 SYNC+SFD，PHR 不属于已知字段，声明它只会让支持长度超过被消除的
窗口。

### 3.2 Ipatov 非稀疏 L1 使用单位冲激前缀

- `SicUnifiedScratch` 新增 `cached_preamble_unit_waveform`；
- `prepare_sic_unified()` 在 Ipatov 后端下由 `cached_preamble_events` 一次性展开为
  `±1` 单位冲激前缀（冷路径，热路径不分配）；事件越界则清空缓存；
- `reconstruct_and_subtract_unified()` 的 L1 离散分支在 `dm.impulse_template` 时
  改用该波形，提交原因记为 **`committed_preamble_ipatov`**（与
  `committed_preamble_discrete` / `committed_preamble_sparse_pulse` 并列，便于区分
  实际走的内核）；缓存缺失时 fail-closed 为 `impulse_prefix_unavailable`，
  不再静默退回成形波形。
- `invalidate_cache()` 同步清空新缓存。

单位冲激前缀与单位参考 SYNC 逐点相等，因此 `waveform_scale` 必须精确为 1.0，
这本身就是一个强一致性检查。

新增缓存只在 Ipatov 后端、`prepare_sic_unified()` 冷路径分配一次：
`73152 × 8 B = 585 KB`/worker（scratch 共享时每 scratch 一份），热路径不分配、
不 resize。

### 3.3 新增 QA

| 测试 | 覆盖 |
|---|---|
| `qa_uwb_hrp_mod_core.cc::prefix_pulse_list_span_is_sync_plus_sfd` | 跨度 = 73152；事件与完整包事件表的前缀逐项相等；事件展开与 `modulate_prefix_waveform()` 逐点相等；`cap = span-1` 拒绝、`cap = span` 接受 |
| `qa_uwb_sic_unified.cc::test_unified_ipatov_preamble_uses_unit_impulse_prefix` | 缓存跨度与单位冲激内容；非稀疏路径提交为 `committed_preamble_ipatov`、`waveform_scale==1`、抑制 >20 dB、`out == y - i_hat` 仅在 ROI 内；同配置稀疏路径提交为 `committed_preamble_sparse_pulse` |

修改前 `modulate_prefix_pulse_list()` **没有任何 QA**，这正是该缺陷能进入主干的
原因。

## 4. 验证

### 4.1 全量回归

`gr-uwb/build-sc16-r1r2`，`env -u LD_LIBRARY_PATH ctest -j1`：
**80 项，79 通过，1 跳过**（`uwb_qa_install_consumer` 按设计跳过），122.85 s，
0 失败。

### 4.2 任务书单配置功能检查（30 窗）

| 运行 | SIC 状态 | 抑制 | sensing SFD | sic_us p50 | e2e_us p50 |
|---|---|---:|---:|---:|---:|
| 修复前 ipatov | 30× `template_scale_mismatch` | 0.00 dB | 30/30 | 307.2 | 1541.0 |
| 修复后 ipatov | 30× `committed_preamble_sparse_pulse` | **44.11 dB** | 0/30 | 413.2 | 1643.4 |
| 修复后 discrete（对照） | 30× `committed_preamble_discrete` | 32.54 dB | 0/30 | 809.5 | 1787.7 |

三次运行账本均 `offered=30 / admitted=30 / completed=30`，0 丢弃、0 处理失败。
抑制为在线 `actual_suppression_db`（`sub_roi` 内 `10log10(‖y‖²/‖out‖²)`），
不是重构 NMSE。

`sensing SFD 0/30` 需要解释：该 fixture 是**同一个 code-9 包**的重复，`--consumer both`
里 sensing 消费者跑在已消除的信号上，包被消掉了自然检不到。对照
`costsplit/*__sensing.json`（不跑 SIC）为 `radar_sfd_found=true`，说明 sensing
检测链本身没有退化。真正的混合码场景（sensing code 9 + comm code 10）不在本
fixture 内，仍未验证。

### 4.3 两个入口冒烟

- `tools/performance/analyze_sc16_cost_split.py --windows 2`：12 次运行全部完成，
  `sic`/`both` 两列均为 `committed_preamble_sparse_pulse`，抑制
  41.02–44.11 dB（realtime_eq16 41.02 / realtime_minorder 41.40 /
  quality_eq32 42.05 / quality_minorder 44.11）；`sensing` 列为
  `radar_sfd_found=true`。
- `tools/performance/run_equiripple_g3.py --windows 2`：4/4 窗功能字段一致，
  两侧均 `committed_preamble_sparse_pulse`；波形对照 EVM 0.730%/0.425%，
  对齐后 lag 0 样点、幅度 +0.0475/+0.0421 dB，与既有等波纹报告同量级。
  抑制在两侧分别为 41.40→41.02 dB、44.11→42.05 dB（`actual_suppression_db`
  不属于 `DECISION_FIELDS`，因此不影响「功能字段一致」判定）。

### 4.4 策略对照（3 窗）

- `--sic-policy auto`（默认）：`committed_preamble_sparse_pulse`，44.11 dB
  —— 整包解调失败后按设计回退到 L1 前导。
- `--sic-policy fullpacket`：`comm_phr_failed`，0 dB —— 该 fixture 是
  preamble-only，PHR 本就无法解出，与本次修复无关。

## 5. 边界与未验证

- 本批数字只用于功能定位与冒烟，**不构成**正式性能矩阵、P99、实时目标或
  `mean≤0.5 ms / 200 窗/s` 的结论。`queue_us`（16.7–26.3 ms p50）是全速文件
  回放的排队，不是 paced 实时延迟。
- 修复后 `sic_us` p50 从 307 µs 升到 413 µs，属于**从「失败/旁路」变成「真实
  消除」**的负载变化，不是回归；修复前的 307 µs 是失败路径耗时，按任务书要求
  不能作为有效 SIC 性能。
- 未上板；未做 MATLAB 交叉验证；未做 paced 到达、8 Worker、AVX-512。
- 混合码（sensing 9 / comm 10）场景、SIC 后 sensing CIR/弱目标保护、
  PSDU/FCS/PER 均未在本轮验证。
- L2 整包离散事务 `fullpacket_cancel_discrete()` 仍硬编码
  `code_index==10` + Legacy 脉冲，`--sic-backend ipatov --sic-comm-code 9` 的整包
  路径仍会被 `unsupported_profile` 拒绝。本 fixture 走不到该分支（PHR 解不出），
  该限制保留为已知缺口，本轮未改。
- 群时延变化对检测位置的影响本轮未涉及；`g3` 的波形对照按脚本自带的时间原点
  校正给出 lag 0。

## 6. 改动文件

```
gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h   modulate_prefix_pulse_list 跨度语义 + 事件边界
gr-uwb/include/gnuradio/uwb/uwb_sic_unified.h    SicUnifiedScratch::cached_preamble_unit_waveform
gr-uwb/lib/uwb_sic_unified.cc                    单位冲激前缀缓存 + L1 Ipatov 分支
gr-uwb/lib/qa_uwb_hrp_mod_core.cc                新增 prefix_pulse_list_span_is_sync_plus_sfd
gr-uwb/lib/qa_uwb_sic_unified.cc                 新增 test_unified_ipatov_preamble_uses_unit_impulse_prefix
```

原始 JSON、CSV、二进制/库 sha256 见
`docs/performance/data_sic_ipatov_prefix_fix_20261112/README.md`。
