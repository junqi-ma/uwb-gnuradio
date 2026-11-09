# SIC Ipatov 冲激序列相关路径

已实现 `--reconstruction ipatov_cir`：SIC CIR 采用与通信/感知相同的插零 Ipatov 序列前向相关。旧 `discrete_cir` 保留为对照。**新路径尚不替换默认后端：SYNC 碰撞抑制明显退化。**

## 原理与实现

令冲激序列为 s，发射脉冲为 p，传播信道为 h。接收 y=s*p*h；新 CIR 通过 Ipatov 相关近似得到 g=p*h，重构使用 s*g。保留绝对复增益，不采用解码 CMF 的单位范数归一化。

- 获取起点和 CFO 仍使用原来的公开成形模板；CIR 本身只使用冲激形式的 Ipatov。
- 训练限于前 8 个 SYNC。跳过第一个有限前导瞬态，只累加完全位于训练前缀内的对齐窗口；本轮 128 点响应使用 6 个完整窗口。先按绝对样点相位去 CFO，再相关并除以“有效窗口数 × 序列能量”。
- 不再进行数据投影、Gram 分解和最小二乘求解。保留用实际 float 系数重建训练数据的残差准入检查。
- 默认响应区间 [-8,120)，共 128 个采样点；这是含 pulse shaping 的响应长度，不是 128 条物理路径。可通过 --lag-lo/--tap-count 显式配置。不会沿用原来 12 个峰邻域 tap 的含义。
- 整包调制复用原编码/扰码/BPM 流程，但使用单位脉冲 [1,0]，幅度为 1。等效 CIR 只施加一次。卷积按每个非零脉冲累加连续 CIR，避免稀疏整包信号的零乘法。
- 新模型标记 impulse_template；与旧模型/不匹配 scratch 混用时拒绝。完整 FCS、PHR、上下文和数值检查保留。

## 针对性验证

两项 CTest 通过：估计器（原有 7 例 + 新例）和重构器。新例包括独立 double 前向相关、整包单位脉冲重构、绝对幅度、训练后缀隔离、工作缓冲不足和残差拒绝。

七格各 16 源窗：32 B 单径、分数多径、±50 kHz CFO、127 B、低 SNR、SYNC 碰撞。通信/感知字节、FCS、状态和提交结果均与旧路径一致，112/112 提交且两类 FCS 通过。损坏包 16 窗两条路径均零提交，输出与输入逐位相同。

解码一致不能替代消除效果检查。以下从 raw/clean/实际输出重新计算能量，跨 16 窗累加能量后取比值；target 是公开感知评价区。

| 场景 | 整窗抑制旧→新 (dB) | target 区旧→新 (dB) |
|---|---:|---:|
| 32 B 单径 | 51.87→50.67 | 48.23→47.68 |
| 32 B 分数多径 | 25.78→28.24 | 25.89→25.07 |
| CFO −50 kHz | 47.92→47.43 | 44.30→44.10 |
| CFO +50 kHz | 48.19→47.66 | 44.56→44.33 |
| 127 B | 42.48→42.35 | 43.36→43.21 |
| SNR 8 dB | 29.87→28.67 | 26.23→25.68 |
| SYNC 碰撞 | 35.63→19.63 | 35.74→19.66 |

SYNC 碰撞退化约 16 dB。混合训练 IQ 的 Ipatov 相关会包含另一码的交叉相关响应；原来的稀疏路径约束对这类污染有抑制作用，新完整响应没有相同约束。具体误差贡献尚未进一步分离。因此仅将此版本作为可复现的算法实验，不宣布完成基线替换。

## 同环境配对耗时

独占 /tmp/uwb-sic-benchmark.lock，Release，ldd 确认加载各自 build/lib；ABBA，每轮 16 次预热、128 次观测，每变体 256 次。每次观测均与同源功能记录核对。以下为平均值。

| 场景 | 参数估计旧→新 (ms) | CIR/支持阶段旧→新 (µs) | 残差检查旧→新 (µs) | 总 SIC 旧→新 (ms) |
|---|---:|---:|---:|---:|
| 32 B，旧 3 tap / 新 128 点响应 | 0.476→0.266 | 206.5→39.6 | 85.2→86.9 | 2.978→3.112 |
| 32 B，旧 12 tap / 新 128 点响应 | 1.482→0.277 | 204.8→37.5 | 958.6→85.7 | 4.415→3.148 |

新路径 projection/solve 均为 0。分数多径总 SIC 平均下降约 28.7%；单径平均上升约 4.5%，因为完整 128 点响应的重构成本高于原来 3 个有效 tap。不能把估计端加速直接等同于所有场景的端到端加速。

总 SIC P95：单径 3.414→3.553 ms；分数多径 4.875→3.576 ms。其他性能分布见 summary.json。

## 入口与记录

```bash
env -u LD_LIBRARY_PATH gr-uwb/build-sic-fullpacket-ipatov/apps/benchmark_fullpacket_compare_fast_rx \
  --case gr-uwb/build-sic-fullpacket/results/inputs16/payload_four_frac \
  --out /tmp/uwb-ipatov-example-new-run --mode full --reconstruction ipatov_cir \
  --rx-fast-sfd --rx-fast-cfo --rx-seed-refine-tail \
  --rx-fused-phr-cmf --repeat 16
```

源码：uwb_discrete_cir_sic.h、uwb_discrete_cir_estimator.cc、uwb_discrete_cir_reconstruct.cc、benchmark_fullpacket_compare.cc。

本轮最终 run：`gr-uwb/build-sic-fullpacket-ipatov/results/ipatov_20261107_230427/`。含原始 IQ 输出、所有计时行、summary.json、provenance.json、code.patch、verify.py、CTest 和构建日志。基础 HEAD 06ef10a 加未提交补丁；精确补丁与 runner/lib SHA256 见 provenance。

前一探索 run ipatov_20261107_230235 保留；其损坏 fixture 没有 clean.cf32，初次脚本中断，后续改为检查逐位旁路。最终 run 从头完成，未覆盖旧记录。

未运行 MATLAB、native、硬件和实时吞吐验证。先前 PHR R1–R3 仍未处理。本轮不修改冻结的共享 preamble SIC core/adapter/reference。

下一步：单独分析 SYNC 重叠时交叉码污染在 CIR 中的分布，设计适用于直接相关响应的支持约束，再决定是否替换旧后端；不通过增大残差门限掩盖退化。
