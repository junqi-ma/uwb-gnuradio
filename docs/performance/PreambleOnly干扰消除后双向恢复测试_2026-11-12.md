# Preamble-only 干扰消除后的 sensing / comm 恢复测试

## 结论

当前 Ipatov 稀疏前导 SIC 在本轮单径场景中恢复了两个方向完整目标包的
检测、PSDU 和 FCS：160 个有干扰案例，消除前 32/160 解码通过，消除后
160/160 通过，解码字节与生成时的完整 PSDU（含 FCS）逐字节一致。
无干扰对照 64 个案例，0 次误提交消除，解码与 CIR 均保持正常。

但是 CIR 幅度保护尚未通过此前的 0.1 dB 参考门槛。零 CFO 同步重叠场景，
sensing-9 CIR 峰值偏差中位约 -0.275 dB、相对 L2 约 6.04%；
comm-10 对应为 -0.413 dB / 8.42%。峰位置差均为 0 tap。
因此本轮支持“完整包可恢复解码”，不支持“CIR 无损或弱目标保护已通过”。

## 输入和方法

- 基线提交 `bf5a5be`，加本轮独立实验入口；生产 SIC/解调/雷达核心没有修改。
- 998.4 MS/s work grid，直接调用当前生产核心；未经过 SC16、65/48 重采样或
  UHD pipeline。本轮不评价 Kaiser 与等波纹差异。
- 两个方向：完整 code-9 sensing 包 + code-10 前导干扰；完整 code-10 comm 包
  + code-9 前导干扰。每次观测只有一个完整目标包；不是同一窗口内同时解码
  两个完整包。
- 目标包：64 SYNC、4z2、Legacy 脉冲、32 B PSDU（包括 2 B FCS）。
  干扰严格由 `modulate_prefix_waveform` 生成，仅 SYNC+SFD，没有 PHR/payload。
- 目标名义起点 512。同步重叠时干扰起点相同；payload 场景干扰起点晚
  `104*1016=105664` 点，位于目标实际 payload 区域，目标前导不受重叠。
- SNR=20 dB，相对于目标整包平均功率；ISR 相对于两信号前 64 个 SYNC 的功率比。
  干扰相位固定 0.4 rad；目标 CFO=0；指定 CFO 仅加在干扰上。
- 每格 16 个种子，改变目标 PSDU 和配对噪声。共 2 方向 × 7 格 × 16 = 224 个
  案例，每例 clean/raw/sic 三路配对，672 条观测。时延、相位和信道在格内固定，
  不能当作独立信道随机抽样或正式 PER 测试。
- clean 是目标+噪声，raw=clean+干扰；SIC 与 clean 使用完全相同的噪声。
  SIC 在线只接收混合 IQ、公开模板和配置的粗 ROI（名义起点 ±64），不接收
  clean 或干扰真值。两方向均显式 `PreambleOnly`、Ipatov、稀疏重构。
- 完整包解调和雷达 CIR 检测都在三路上独立执行，使用相同公开名义时间及模板，
  不强制 oracle 检测位置，不跳过 SFD 门控。
- 抑制指标采用离线 `10log10(||raw-clean||²/||sic-clean||²)`，包含残余干扰和
  对目标的损伤；不是可能把目标删掉也显得改善的总功率下降。

## 检测与解码

下表结果在 sensing-9 和 comm-10 两个方向相同；每格每方向 16 例。
“通过”同时满足 FCS 通过与 PSDU 逐字节正确。

| 场景 | raw 解码 | SIC 后解码 | raw 雷达 CIR 有效 | SIC 后雷达 CIR 有效 |
|---|---:|---:|---:|---:|
| 无噪声、无干扰 | 16/16 | 16/16 | 16/16 | 16/16 |
| SNR 20 dB、无干扰 | 16/16 | 16/16 | 16/16 | 16/16 |
| 同步重叠，ISR 0 dB | 16/16 | 16/16 | 16/16 | 16/16 |
| 同步重叠，ISR 10 dB | 0/16 | 16/16 | 0/16 | 16/16 |
| 同步重叠，ISR 20 dB | 0/16 | 16/16 | 0/16 | 16/16 |
| 同步重叠，ISR 20 dB，干扰 CFO +50 kHz | 0/16 | 16/16 | 0/16 | 16/16 |
| 干扰落入 payload，ISR 20 dB | 0/16 | 16/16 | 16/16 | 16/16 |

160 个有干扰案例均提交 `committed_preamble_sparse_pulse`。
64 个无干扰对照均返回 `below_detect_threshold`，未误消除。

## CIR 保真和残余

与同一案例 clean CIR 的复数 raw taps 直接比较，未做额外幅度或相位拟合。
未检出的 CIR 不填零冒充正常；逐例 CSV 中有失败状态和无效标记。

| 方向 / 场景 | CIR 峰幅偏差中位 | CIR 相对 L2 中位 | 残余改善中位 |
|---|---:|---:|---:|
| sensing-9，同步 ISR 0 | -0.274 dB | 6.03% | 15.06 dB |
| sensing-9，同步 ISR 10 | -0.275 dB | 6.04% | 25.05 dB |
| sensing-9，同步 ISR 20 | -0.275 dB | 6.04% | 35.04 dB |
| sensing-9，同步 ISR 20 / CFO +50k | +0.0237 dB | 1.38% | 30.11 dB |
| sensing-9，payload ISR 20 | 0 dB | 0% | 38.64 dB |
| comm-10，同步 ISR 0 | -0.413 dB | 8.40% | 13.49 dB |
| comm-10，同步 ISR 10 | -0.413 dB | 8.41% | 23.49 dB |
| comm-10，同步 ISR 20 | -0.413 dB | 8.42% | 33.49 dB |
| comm-10，同步 ISR 20 / CFO +50k | +0.0267 dB | 1.02% | 30.16 dB |
| comm-10，payload ISR 20 | 0 dB | 0% | 39.13 dB |

所有有效 CIR 的峰 tap 差为 0，两个方向零 CFO 的最大峰幅绝对偏差分别约
0.291 / 0.432 dB。payload 干扰场景的 CIR 为零差，是因为目标的整个前导位于
干扰之前，不能用它证明同步重叠时没有目标损伤。

零 CFO 的 CIR 偏差几乎不随 ISR 变化，是相干的估计偏差/残余的线索；目前只
记录现象，尚未完成归因，不把它直接等同于已证实的目标误消除。CFO +50 kHz
这一格的 CIR 偏差更小，也不能推广成 CFO 总能改善 SIC。

## 复现与边界

```bash
cmake -S gr-uwb -B gr-uwb/build-sc16-r1r2
cmake --build gr-uwb/build-sc16-r1r2 --target experiment_preamble_recovery -j2
python3 tools/sic/run_preamble_recovery.py \
  --binary gr-uwb/build-sc16-r1r2/apps/experiment_preamble_recovery \
  --out /tmp/uwb-preamble-recovery-new-run --seeds 16
```

输出目录必须不存在。驱动持有 `/tmp/uwb-sic-benchmark.lock` 串行运行，清除
`LD_LIBRARY_PATH`；保存源码、二进制、实际加载库 hash、命令、逐例 CSV 和汇总。
clean 参考若解码或 CIR 失败，驱动非零退出，避免把失效参考解释为恢复结果。

本轮数据：`docs/performance/data_preamble_recovery_20261112/`。
入口：`tools/sic/experiment_preamble_recovery.cc`、
`tools/sic/run_preamble_recovery.py`。源码通过构建及完整 224 案例执行，
`git diff --check` 通过；没有重新运行与本次实验入口无关的全量回归。

未验证：分数时延/多径、弱目标动态范围、PSLR/ISLR、负 CFO、多 SNR、码/相位/时延
全面扫描、native 重采样后的恢复、真实 pipeline 调度、上板、MATLAB 交叉验证。
本轮没有性能计时，不宣称实时吞吐或 P99 达标。
