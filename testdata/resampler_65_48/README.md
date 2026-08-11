# 65/48 软件升采样测试数据（737.28 → 998.4 MS/s）

生成脚本：

- `testdata/design_resampler_737p28_to_998p4.py`（canonical，scipy）+ `.m`（MATLAB 交叉验证）
- `testdata/export_resampler_65_48_golden.py` + `.m`
- `testdata/verify_resampler_block.py`（block 建好后对照 golden）

## 滤波器 taps

| 文件 | 档位 | 通带 B | taps | 群时延(输出) | 说明 |
|---|---|---|---|---|---|
| `taps_quality.txt` | quality | 330 MHz | 5363 | 55.85 | 99.9% 信号能量，阻带 −97 dB |
| `taps_realtime.txt` | realtime | 290 MHz | 2635 | 27.44 | 99% 能量，阻带 −96 dB |

- float32 二进制（`numpy.tofile` / `fwrite single`），DC sum = 65（已含 ×65 缩放）。
- `taps_*.csv` 为人类可读 17 位十进制；`taps_*_matlab.txt` 为 MATLAB 独立设计输出（交叉核对用）。
- `design.json`：设计参数与验证指标。

## Golden（契约 = scipy/MATLAB upfirdn）

`golden/<name>_in.cf32`（737.28M 输入）与 `<name>_out.cf32`（998.4M upfirdn 输出）成对：

| name | 内容 | N_in | L_out |
|---|---|---|---|
| impulse | 单位冲激 @0 | 4096 | 5658 |
| dc | 常数 1 | 4096 | 5658 |
| tone_low | 10 MHz 复单音 | 4096 | 5658 |
| tone_pb | 300 MHz（通带边缘） | 4096 | 5658 |
| tone_sb | 420 MHz（阻带，输出≈0） | 4096 | 5658 |
| random | 种子随机复数 | 4096 | 5658 |
| uwb | 998.4M cfile preamble 窗口 → 48/65 抗混叠抽取 | 6090 | 8358 |

长度/相位约定：`L_out = ceil(((N_in−1)·65 + 5363)/48)`；相位律
`y[m] = Σ_k h[(m·48 mod 65) + 65k]·x[floor(m·48/65) − k]`（两侧零填充全卷积）。
已用独立手写多相实现与 scipy 双重验证（逐样本 ≤2e-7）。

## 契约验证记录

- Level A：内置 `rational_resampler_ccf` 稳态样本与 upfirdn 一致，但边界（长度/瞬态）
  不同（前向非因果、无暖机瞬态），有限长输出差约 2×群时延。
- 固定 core 契约定为**与 upfirdn 完全一致**（含暖机与 EOS 尾、精确长度）。
