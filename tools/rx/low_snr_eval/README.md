# 低 SNR 完整帧回放与 ABBA 对照

从原 2026-11-17 临时实验目录归档。此工具生成 **127 B 完整帧**，使用生产
`modulate_one`：code10 / 256 SYNC / 4z2 / 6.81 Mb/s，1 ms 周期，0 ppm。
不含 CFO/SFO、多径或干扰。`--snr` 为 ±250 MHz 带内信噪比。

在仓库根目录运行（Python 依赖 numpy、scipy；先按主文档构建 native）：

```sh
mkdir -p /tmp/uwb-low-snr
c++ -std=c++17 -O2 -Igr-uwb/include -Igr-uwb/build-native/include \
  tools/rx/low_snr_eval/gen_full_frame.cc -lvolk -o /tmp/uwb-low-snr/gen_full_frame
python3 tools/rx/low_snr_eval/build_stream.py \
  --generator /tmp/uwb-low-snr/gen_full_frame --snr -12 --packets 200 \
  --seed 1 --out /tmp/uwb-low-snr/fm12_200.sc16
```

脚本默认写出相邻 `.json`，包含 SHA-256、种子、速率和包起点；生成过程需要数 GB
内存和磁盘空间。标定固定 127 B，不开放其他长度以免沿用错误 RMS 区间。
以下表格列出完整矩阵应生成的文件；每项使用同一命令，仅更换参数：

| 文件 | 参数 |
| --- | --- |
| `f4.sc16` | `--snr 4 --packets 200 --seed 1` |
| `fm4p0.sc16` | `--snr -4 --packets 200 --seed 1` |
| `fm10.sc16` | `--snr -10 --packets 200 --seed 1` |
| `fm12_200.sc16` | `--snr -12 --packets 200 --seed 1` |
| `fm14.sc16` | `--snr -14 --packets 200 --seed 1` |
| `noise_0p5s.sc16` | `--noise --packets 500 --seed 1`（500 ms，无信号） |

`--noise` 的 packets 仅表示毫秒槽数。为保留原实验生成顺序，仍先生成并丢弃信号。
历史报告未保存全部夹具哈希，重新生成应以新元数据为准，不宣称跨 numpy/scipy
版本逐字节一致。

```sh
python3 tools/rx/low_snr_eval/ab.py \
  /path/to/baseline/uwb_dw3000_live gr-uwb/build-native/apps/uwb_dw3000_live \
  /tmp/uwb-low-snr/ab.json --fixtures /tmp/uwb-low-snr --only=-12/locked/w1
```

省略 `--only` 运行全矩阵。基线为 `base-low-snr-speed`（`f5eadbb`），优化版
`ea291ca`（合入 `63c057f`）。使用独立 worktree 编译基线，不覆盖当前源码。
固定 `--cir full --fused on --taps eq16 --tone off`，噪声档单独比较 tone on/off。
每档依次 A、B、B、A；每次测量分别运行 summary 和 packets 两种输出模式。

结果 `cases` 保存原始运行记录及每侧两次值的中位数；`metadata` 保存 CPU、平台、
二进制和夹具哈希。失败进程立即报错，不计入成功结果。噪声 tone 两档有独立键。
SUMMARY（含结尾 SUMMARY_PARTIAL）的 compute/demod 均值是控制台批次均值的无权平均；p95 是批次 p95 的平均，
**不能当作全流 P95**。阶段耗时来自另一次 packets 运行，不与 summary 拼成单包总和。
需评估负载下尾延迟时，应另存逐包时间并计算完整分布。

输出不自动宣告性能验收通过。比较前记录 Git 提交、CMakeCache 中 ISA 与构建选项、
机器负载，并区分 FCS、SFD-only 进锁、种子数和队列丢弃。运行时清除 LD_LIBRARY_PATH。

历史结果和数据缺口见
[归档说明](../../../docs/performance/data_low_snr_speed_20261117/README.md)。

重构等价性可用 `compare_packets.py BASE_DECODER CURRENT_DECODER --fixtures DIR --out DIR`
比较固定窗口 JSONL，忽略耗时，逐项比较状态、度量、CFO、FCS 和 PSDU hex。
工具默认每个 +4/-12/-14 dB 文件 200 包；解码器编译目标为 uwb_dw3000_decode。
这是一致性检查，不替代动态锁定调度的 ABBA 验收。
