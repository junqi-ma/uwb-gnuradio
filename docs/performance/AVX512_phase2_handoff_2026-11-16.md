# AVX-512 第二阶段交接状态（2026-11-16）

用途：磁盘快照。恢复时先读本文件，再读两份规划与两份报告。
工作区：`/tmp/uwb-gnuradio-avx512`，分支 `perf/avx512-kernels`，Git 起点 `5d5ba26`
（含继承的未提交主路径快照 + 第一阶段 + 第二阶段未提交开发）。

## 1. 第一阶段：已完成并冻结

- 报告：`docs/performance/AVX512内核与编译期CPU选择开发报告_2026-11-16.md`
- 测量合同：`docs/performance/AVX512_测量合同_2026-11-16.md`
- 冻结清单：`docs/performance/data_avx512_20261116/phase1_freeze_20261116.json`
- 最终 ABBA：`docs/performance/data_avx512_20261116/abba_final/summary.json`
- 结论：`UWB_SIMD=AUTO|AVX2|AVX512|SCALAR`；SC16/FC32 AVX-512 内核与 AVX2
  **逐位一致**；默认只对 H_pad 24/48 启用 AVX-512（56k 生产窗 +9.4%/+20.1%），
  H_pad 16/32 保持 AVX2。

## 2. 第二阶段：已完成（开发结论，非独立验收）

- 报告：`docs/performance/解调与SIC_AVX512内核扩展开发报告_2026-11-16.md`
- 冻结清单：`docs/performance/data_demod_sic_avx512_20261116/phase2_freeze_20261116.json`
- ABBA：`docs/performance/data_demod_sic_avx512_20261116/abba/summary.json`
- 算子级：`docs/performance/data_demod_sic_avx512_20261116/kernel_bench_avx512_run{1,2,3}.txt`
- 反汇编证据：`docs/performance/data_demod_sic_avx512_20261116/disassembly_hits.txt`

### 结论

- **只有 `sync` 组默认开 AVX-512**（`kAvx512Default`），其余四组保持 AVX2。
- 端到端同二进制 ABBA（5 轮 × 2 次会话）：B（sync=AVX-512）full-packet
  **−5.6% / −5.8%**、preamble-only **−7.3% / −8.1%**；A 参考噪声底 |Δ| ≤ 0.5%。
  `timing` 阶段 79.2 → 59.2 µs（−25.2%）是主要来源。
- C（cir/sic_*=AVX-512）不过门槛且方向不稳 → SIC 专有组保持 AVX2。
- 逐位一致：算子级 16 个 QA 用例；SIC 输出两份 dump 各 1,084,544 B 逐字节相同。
- 回归：三个构建（AVX512/AVX2/SCALAR）各 **54/54 通过**（含安装消费者）。

### 采纳 / 拒绝的内核

采纳（逐位一致）：8-lag 实模板点积（−37%/lag）、批处理 `fold_sfd_window`
（−24…−29%）、逐元素 `accumulate`/`sub`/`sub3`/`scale`（小 n −12…−57%）。
拒绝（实测更慢或无收益，保持 AVX2）：fused CMF 52 点复相关（+29%）、
固定 38-tap CIR FIR（+94%）、20-tap SIC 脉冲累加（+2%）。

## 3. 代码入口

- 选择器：`gr-uwb/include/gnuradio/uwb/uwb_kernel_backend.h` +
  `gr-uwb/lib/uwb_kernel_backend.cc`（编进 baseline-ISA 的 `uwb_simd_probe`）。
  分组 `sync|cfo|cir|sic_reconstruct|sic_energy`；覆盖 `default|scalar|avx2|avx512`。
- 宽内核：`gr-uwb/include/gnuradio/uwb/uwb_demod_sic_avx512.h` +
  `gr-uwb/lib/uwb_demod_sic_avx512.cc`（独立 TU，函数 target 属性）。
- 调用点：`uwb_rx_sync_opt.h`（`real_template_dot_8lag`、`vector_sub3_complex`、
  逐元素/折叠分派）、`uwb_demod_core.h`（两处 4-lag → 8-lag 循环）、
  `uwb_discrete_cir_reconstruct.cc`（S3 相减走共享内核）。
- 基准：`gr-uwb/apps/bench_demod_sic_kernels.cc`、
  `tools/perf/run_demod_sic_abba.py`；
  `bench_unified_timing` 读环境变量 `UWB_KERNEL_BACKEND="sync=avx2,cir=avx512"`。

## 4. 构建目录

| 目录 | UWB_SIMD | CTest |
| --- | --- | --- |
| `gr-uwb/build-avx512-demod-sic` | AVX512 | 54/54 |
| `gr-uwb/build-avx2-demod-sic` | AVX2 | 54/54 |
| `gr-uwb/build-scalar-demod-sic` | SCALAR | 54/54 |

均 `UWB_BUILD_GNURADIO=OFF`、`UWB_BUILD_EXPERIMENTS=ON`、`ENABLE_UHD_BACKEND=OFF`。
安装消费者前缀：`/tmp/opencode/uwb-phase2-install{,-avx2,-scalar}`。

## 5. 下一步（若继续）

1. 独立审查：重建三个构建、复跑 ABBA、确认 `kAvx512Default` 表。
2. 可选：`timing` 的 `pw` 与 `sfd` 的 8128 点功率循环是剩余最大热点
   （约 35 µs，占 full-packet 9.4%），但都是严格串行求和链，**改动需要单独提出
   误差合同 + 独立 double 参考 + 门限边界测试**，不能按本轮逐位一致标准直接做。
3. 可选：`sic_energy`（`compute_roi_energies`）尚未有实测候选。
4. 未验证项（上板/UHD/paced/8 worker/负载 P99/MATLAB/多径/AWGN/ISR/CFO 扫频/SFO）
   见报告 §9，不得当作已通过。

## 6. 约束提醒

- 不改协议、捕获几何、AcquireLegacy/LockedBounded、整数网格准入、
  Full→Bypass 降级、默认 taps、群时延、CFO 估计方法、SIC 算法/阈值/已知字段/
  提交事务；不顺带改 FEC/裁剪/跳过验证。
- 不引入 `-ffast-math`，不取消 `-ffp-contract=off`，不放宽既有 QA 容差。
- 默认准入：目标生产格中位耗时下降 ≥5%、跨轮同向；其他生产格无可重复退化。
- 未上板/MATLAB/paced/8 worker/负载 P99 均未验证，报告如实列未验证。
