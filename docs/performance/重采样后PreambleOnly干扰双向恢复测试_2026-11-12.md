# 737.28→998.4 MS/s 重采样后的前导干扰双向恢复测试

## 结论

六套 FIR 均通过本轮解码恢复测试：每套 160 个有干扰案例，消除后均正确恢复
完整目标包 PSDU 和 FCS；64 个无干扰对照均未误提交消除。每套 clean 参考
224/224 通过，说明实验没有再次使用不完整包窗口或错误码配置。

但是不能据此判定全部 FIR 的消除质量等效：quality 激进等波纹在个别 payload
重叠案例中，残余改善比 Kaiser 低约 5 dB，最差配对差 -5.258 dB，虽然解码仍成功。
低纹波 quality 等波纹的差异较小。本轮仍看到前一轮已有的同步重叠 CIR 偏差，
因此 0.1 dB 幅度保护和弱目标保护尚不能宣布通过。

## 链路和实验合同

链路为：生成双码 work-grid 波形 → 独立 48/65 下采样 → SC16 量化 →
生产 `RationalResampler65_48Core::process_sc16 + flush` → 生产前导 SIC →
生产完整包解调及雷达 CIR 检测。核心设为单 worker、`fixed_phase`。

这验证了实际 SC16 重采样核心及其下游信号处理链路。没有调用 UHD RX、环形队列
或结果提交器，不是对完整线程 pipeline 的功能/调度验收，也没有性能计时。

沿用前一轮的双向信号定义：

- 完整 code-9 sensing 包 + code-10 前导干扰；完整 code-10 comm 包 + code-9
  前导干扰。每例只有一个完整目标包。
- 64 SYNC、4z2、Legacy 脉冲，目标 PSDU=32 B（含 FCS），干扰只有 SYNC+SFD。
- 两格无干扰对照；同步重叠 ISR 0/10/20 dB；同步重叠 ISR 20 dB + 干扰 CFO
  +50 kHz；干扰前导落在目标 payload、ISR 20 dB。一共 7 格，每方向每格 16 例。
- SNR=20 dB，按目标整包平均功率定义；ISR 按前 64 个 SYNC 的功率比定义。
  干扰相位固定 0.4 rad；单径、目标 CFO=0。
- 本轮将名义目标起点设为 `q=512+seed%16`，覆盖 16 个不同的原生采样网格分数
  相位；种子同时改变目标 PSDU 和噪声。没有把这些结果当作独立随机信道 PER。

输入和坐标约束：

1. 原生信号由 SciPy `resample_poly(48,65)` 合成，统一使用 8193 tap、beta=10
   Kaiser；补偿该生成滤波器的群时延。此生成滤波器与被测 FIR 独立，六套被测
   FIR 共享相同原生 SC16 字节。生成器密集频响扫描的通带纹波：290 MHz 内
   0.0000462 dB，330 MHz 内 0.0000828 dB。这只是生成器属性，不代表端到端滤波
   误差已全部消除。
2. 每例 clean/raw 使用同一量化尺度，I/Q 分量峰值留 10% 余量；逐例检查零削顶。
   输入包含完整目标包、干扰支持和尾部上下文，原生长度约 99,770–133,230 点，
   没有沿用只能覆盖前导的 56k 窗口。量化后按同一公开单位转换尺度送入核心。
3. 每个 FIR 按 `(T-1)/96` 输出样点群时延修正接收机和 SIC 的**名义搜索坐标**，
   仅对搜索 seed 四舍五入，由真实检测细化位置。不插值或平移被测输出波形，
   不用真值优化相关位置。该坐标处理只在实验入口中，生产 pipeline 的群时延
   集成状态没有因此改变。
4. SIC 在线只接收混合 IQ、公开模板和配置的 ±64 粗 ROI；clean 真值仅用于
   离线评分。用同一 FIR、同一输入尺度下的 resampled clean 作 raw/SIC 参考，
   保留该链路对噪声和波形的共同影响。

## 检测 / PSDU / FCS

每套 FIR、每个方向的结果相同，下表每格 16 例：

| 场景 | raw 正确 PSDU + FCS | SIC 后正确 PSDU + FCS | raw CIR 有效 | SIC 后 CIR 有效 |
|---|---:|---:|---:|---:|
| 无噪声、无干扰 | 16/16 | 16/16 | 16/16 | 16/16 |
| SNR 20 dB、无干扰 | 16/16 | 16/16 | 16/16 | 16/16 |
| 同步 ISR 0 dB | 16/16 | 16/16 | 16/16 | 16/16 |
| 同步 ISR 10 dB | 0/16 | 16/16 | 0/16 | 16/16 |
| 同步 ISR 20 dB | 0/16 | 16/16 | 0/16 | 16/16 |
| 同步 ISR 20 dB、CFO +50k | 0/16 | 16/16 | 0/16 | 16/16 |
| payload ISR 20 dB | 0/16 | 16/16 | 16/16 | 16/16 |

每套 160 个有干扰案例全部提交 `committed_preamble_sparse_pulse`；64 个对照
全部 `below_detect_threshold`。六套合计 960 个有干扰恢复、384 个无干扰对照；
每套 224 案例各保存 ideal/clean/raw/sic 四路观测，共 5376 条观测。
ideal 是直接 work-grid 参考，重复保存不等于新增独立样本。

## CIR 增量偏差

以下为同步重叠、ISR 20 dB、零 CFO，SIC 后与**同一 FIR 的 resampled clean**
比较；峰幅及相对 L2 均为 16 例中位数。这里评价的是 SIC 在该重采样链路上增加
的误差，不是把链路 clean 本身的滤波失真也归给 SIC。

| FIR | sensing 峰幅偏差 | sensing CIR L2 | comm 峰幅偏差 | comm CIR L2 |
|---|---:|---:|---:|---:|
| realtime Kaiser H24 | -0.291 dB | 6.28% | -0.381 dB | 8.64% |
| realtime 等波纹 H16 | -0.289 dB | 6.58% | -0.387 dB | 8.70% |
| quality Kaiser H48 | -0.274 dB | 6.34% | -0.402 dB | 8.48% |
| quality 等波纹 H32 | -0.263 dB | 6.48% | -0.432 dB | 8.76% |
| realtime 低纹波等波纹 H24 | -0.301 dB | 6.70% | -0.385 dB | 8.58% |
| quality 低纹波等波纹 H40 | -0.282 dB | 6.37% | -0.389 dB | 8.55% |

SIC 相对同链路 clean 的 CIR 峰 tap 差在所有案例均为 0。CFO +50k 格的峰幅
偏差约 +0.021…+0.030 dB，相对 L2 约 1.0…1.4%；payload 重叠格的 CIR 零差，
原因仍是干扰发生在目标整个前导之后，不能外推为同步重叠也无目标损伤。

CSV 的 `clean` 行还保存与 ideal work CIR 的直接差异。该差异同时含带限效应、
脉冲形状变化和检测后分数采样坐标差，没有再做分数时延校准，不能把其幅度或 L2
直接解释成通带纹波，也不能用它作为跨滤波器绝对 CIR 保真的最终判据。

## 残余改善：逐案例配对差异

使用 `10log10(||raw-clean||² / ||sic-clean||²)`，包括残余干扰和目标损伤，
不使用可能奖励误消除目标的总功率下降。每例匹配同一个方向、场景和种子后求
候选减基线；下表覆盖各对比的 160 个有干扰案例。

| 对比 | 配对差最小值 | 中位数 | 最大值 |
|---|---:|---:|---:|
| realtime H16 等波纹 − realtime Kaiser | -0.507 dB | +0.030 dB | +0.973 dB |
| quality H32 等波纹 − quality Kaiser | **-5.258 dB** | -0.117 dB | +4.153 dB |
| quality H40 低纹波等波纹 − quality Kaiser | -1.060 dB | -0.020 dB | +0.869 dB |

quality H32 最差例：comm-10、payload ISR20、seed=14（名义起点 526），
Kaiser 残余改善 40.0196 dB，等波纹 34.7618 dB。二者均正确解码。
seed=0 的同类例为 41.6565→36.6381 dB，独立的 1-seed 冒烟与 16-seed 矩阵
得到相同数值，说明不是只看中位数即可忽略的偶发输出差异。

变化可能同时来自频响、群时延造成的分数采样位置变化与有限 tap CIR 建模；
本轮没有把它单独归因于通带纹波。现有证据支持继续保留 quality 低纹波候选，
不支持仅因所有 FCS 通过就宣布激进 H32 在消除质量上与基线等效。

## 复现与交付

```bash
cmake --build gr-uwb/build-sc16-r1r2 --target experiment_preamble_recovery -j2
python3 tools/sic/run_native_preamble_recovery.py \
  --binary gr-uwb/build-sc16-r1r2/apps/experiment_preamble_recovery \
  --out /tmp/uwb-native-recovery-new-run --seeds 16
```

输出目录必须不存在。生成和全部执行串行持有 `/tmp/uwb-sic-benchmark.lock`，
不并发运行构建或基准；清除 `LD_LIBRARY_PATH`。原始大 IQ 存临时目录并在完成后
删除，保留每例源波形/原生输入 hash、输入长度、量化尺度和完整重生成路径。

数据：`docs/performance/data_native_preamble_recovery_20261112/`，含六套逐例 CSV、
summary、输入 manifest、生成滤波器、命令上下文、实际二进制/库及源码 hash。
驱动：`tools/sic/run_native_preamble_recovery.py`；C++ 入口沿用
`tools/sic/experiment_preamble_recovery.cc`，新增 dump/native 模式，work 模式保留。

验证：构建通过；1-seed 六套功能冒烟；16-seed 六套矩阵通过；独立检查全部
clean/SIC 的 PSDU、FCS、CIR 状态、无干扰误提交、SIC 增量峰位置及完整观测数量。
本轮没有修改生产算法/默认 profile，没有全量回归或硬件验证。

边界：单径、固定相位、单个 SNR、16 个起点、一个正 CFO 测点；未覆盖多径、
弱目标动态范围、PSLR/ISLR、负 CFO、所有多相状态、真实时钟漂移、UHD、paced
到达或正式 PER/吞吐验收。低纹波候选也尚未证明 0.1 dB 的同步重叠 CIR 保护。
