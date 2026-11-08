# 移除 SYNC 逐 chip FIR

已在 `rx_fused_phr_cmf` 快路径移除整段 SYNC soft-chip FIR。旧路径及完整失败回退保持原行为；不改变默认开关。此次不处理先前 PHR 复核报告中的 R1–R3，它们仍待关闭。

## 方法

原相位统计为最后 32 个 SYNC 的 `Σr,c spread[c] Σq h[q] IQ[r,c,q]`。现在先累加这些重复符号的对应 IQ 样点，再对一个符号的累加结果做稀疏 spread/CMF 加权，得到相同相位统计（浮点加法顺序不同）。不生成 SYNC 的逐 chip FIR 输出，相位信息仍然保留。

FIR 从 chip 32504 开始，仅计算 SFD 搜索区：最后 8 个名义 SYNC chip 是 NS-SFD 的负向搜索 halo，不能随整段 SYNC 删除。总 FIR 输出从 36,585 降到 **4,081 chip**（减少 88.85%），其中包含 SFD 及两端余量。

为保留现有坐标和包长准入，soft-chip 向量逻辑长度仍为 36,585，跳过的前缀置零。新字段 `soft_fir_computed_chips` 表示实际 FIR 输出数量，`soft_fir_first_chip` 表示起点，不能再仅凭 soft_chip_count 判断计算量。相位直接 IQ 统计计入 `sync_phase_us`，它是 postprocess_us 的子项。

NS-SFD 使用归一化相关；剩余区间使用统一正尺度归一化。PHR/payload 融合只需要冻结相位及 NS-SFD 位置/极性，不读取被跳过的 SYNC soft chips。整数 spc=2 完整 CMF 才使用本路径；其他模式保留旧流。

## 验证

- 新增相位统计对独立 double 双重求和参考的 QA，包括负向左边界、恰好足够及少一个 sample 的右边界，误差门槛 2e-5。
- 合成 32/127 B 集成 QA 增加 SFD 位置/极性及 4,081 个 FIR 输出断言。
- demod_core、fullpacket_fast_rx、rx_fused_cmf、rx_fused_phr 四项 CTest 全部通过。
- 既有六格×16 源窗，新旧 PHR/通信/感知字节、FCS、status、提交结果一致，全部正常融合，无回退。
- 确定性损坏包 16 窗结果一致，完整旧流各回退一次，零提交。

## 配对性能

旧 binary 为已验收的 PHR 融合版；新 binary 使用相同 Release/native 构建设置。两个 build 的库均经 ldd 确认；固定旧 binary 避免覆盖历史。独占 `/tmp/uwb-sic-benchmark.lock`，每变体 16 预热、128×2 观测，ABBA。三格全部四轮逐源窗核对功能字段，失败与拒绝为零。

| 配置 | 前缀 FIR 旧→新 | CIR+前缀总阶段 旧→新 | 通信解码 旧→新 | 总 SIC 旧→新 |
|---|---:|---:|---:|---:|
| 32 B／3 有效重构 tap | 252.8→27.2 µs | 356.5→84.1 µs | 1.565→1.230 ms | 3.657→3.464 ms |
| 32 B／12 有效重构 tap | 264.2→26.8 µs | 384.7→102.2 µs | 1.581→1.337 ms | 5.323→5.117 ms |
| 127 B／3 有效重构 tap | 252.4→40.3 µs | 352.2→124.2 µs | 1.749→1.472 ms | 4.862→4.458 ms |

新 SYNC 相位统计平均 8.3/8.4/9.4 µs，已含在父阶段内。mean 下降明确，但尾延迟有运行波动：12 tap 总 SIC P95 7.580→7.784 ms，127 B 通信 P95 2.655→2.931 ms；不宣称所有分位数改善。127 B 新 FIR mean 40.3 µs 高于 P95 33 µs，存在调度离群值，未删除样本。

## 复现与版本

基础 HEAD `06ef10a` 加本轮未提交实现；代码 dirty 状态、精确补丁 SHA256、两个 binary/lib SHA256、ldd 和全部命令见新 run 的 provenance.json，补丁保存为 code.patch。功能脚本首次因旧损坏包目录没有 command.json 中断；改用已有损坏 fixture 后复用已完成六格继续，未覆盖历史数据。

- 新 build：`gr-uwb/build-sic-fullpacket-no-sync-fir`
- 原始数据：`gr-uwb/build-sic-fullpacket-no-sync-fir/results/sync_fir_removed_20261107_221110/`
- 摘要：该目录 `summary.json`；验证脚本 `verify.py`。
- 未运行 MATLAB/硬件/实时吞吐测试。本轮只验证离线 work-grid 的等价性与性能。

下一步：如需继续验收整个 PHR 交付，先关闭前次复核 R1–R3；本改动不自动开展其他同步/CFO 优化。
