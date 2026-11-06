# 离散 CIR：FIR 乘加融合计时报告（2026-11-07）

结论：融合 FIR 乘加值得保留。同一轮 old→new→new→old 配对中，12 tap 纯 FIR
mean 从 1.072 ms 降到 0.548 ms（比值 0.511，降低 48.9%），超过“至少降低 20%”
目标；3 tap 纯 FIR 从 0.296 ms 降到 0.161 ms（比值 0.546），整包 wrapper
从 1.199 ms 降到 1.085 ms（比值 0.905），没有退化。所有测试 tap 数的
FIR/channel/wrapper mean 与 P95 均下降。未改估计器、tap 选择、CFO、FCS 门槛、
数值检查、相减 ROI 或默认后端，因此没有重跑解码率矩阵。

## 改动与实现

- `gr-uwb/lib/uwb_discrete_cir_reconstruct.cc`：FIR 内循环由
  “`tmp = h*src`；`dst += tmp`”改为融合 `dst[i] += h*src[i]`，消除每个 tap
  对临时数组的一次整段写入和一次整段读取。边界、预分配、每个输出样点的
  tap 累加顺序不变。
- `fir_tmp` 仍保留并继续供 CFO 阶段使用，没有误删。
- `gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h` 只更新该临时缓冲的注释。
- 不新增 GNU Radio block；`discrete_cir` 仍不是默认后端。

`std::complex<float>` 的 `operator*` 在本机 GCC 11.4 下会下沉到带 NaN 语义的
`__mulsc3` 调用，不能向量化，因此 helper 用实部/虚部分量写。helper 保持
out-of-line：内联进大型 apply 事务后 GCC 没有向量化该循环；独立函数形式在
`-O3 -DNDEBUG`、基线 x86-64（SSE2）下由 `-fopt-info-vec-optimized` 报告
16 B 与 8 B 向量循环，汇编为 `movups` 非对齐读写、`mulps/addps/subps` 和
标量尾部。每个 active tap 一次调用的开销相对 13 万样点循环可忽略。
没有加 `-march=native`，没有用 fast-math 绕过 NaN/Inf 检查，也没有新增
AVX2/FMA 后端——本轮收益已达标，不需要第二个 SIMD 实现。

## 正确性与验证

- 新 build `gr-uwb/build-sic-fullpacket-fused` 从本提交源码独立配置；
  旧 build `gr-uwb/build-sic-fullpacket-discrete` 未覆盖，旧库 hash
  `43e12c18…5ed29` 与离散 CIR 开发报告一致。
- `uwb_qa_uwb_discrete_cir_reconstruct.cc` 在 new build 下 1/1 通过；
  `ldd` 确认测试实际加载 new build 的 `libgnuradio-uwb.so`。
- `benchmark_discrete_cir_taps` 自带的独立 double 密集卷积 + CFO 检查在
  四次调用中均通过：最大相对 L2 误差 `1.35240526763e-7`，门槛 `2e-5`
  （约 148 倍余量）；有限性和 `out == y - ihat` 相减检查均通过。
  未要求逐位相等；本机基线编译不产生 FMA，旧/新报告误差相同。
- 现有生产包长是 4 的倍数，原 QA 没有单独打到向量尾部；本轮补了
  `discrete_apply_tail_and_unaligned_offsets` 一例：1019 样点
  （254×4 + 2 + 1），lag -1/0/1 使源/目标相对 16 B 边界非对齐，
  与独立 double 卷积比较通过。该用例在正式计时后补测，不改变生产库
  hash `88343beb…c334d`，不影响计时绑定。

## 计时结果

固定 32 B wire PSDU（含 FCS）、code 10、64 SYNC、4z2、998.4 MS/s、
CFO +50 kHz；波形 133568 样点，输入窗 137664 样点，声明支持 64 样点。
每组每轮预热 16 次、观测 128 次；每次调用内部正序/倒序两轮。正式顺序
old→new→new→old，合并后每个 tap 共 512 条观测（2 次调用 × 2 轮 × 128）。
下表为 µs；P95 来自这 512 条热态重复调用，重复曝光不是独立信道样本，
也不代表实时负载尾延迟。

| taps | old FIR mean | new FIR mean | FIR ratio | old FIR P95 | new FIR P95 |
|---:|---:|---:|---:|---:|---:|
| 1 | 127.465 | 69.977 | 0.549 | 184.402 | 93.631 |
| 3 | 295.587 | 161.308 | 0.546 | 356.233 | 201.062 |
| 6 | 553.671 | 287.041 | 0.518 | 648.417 | 334.272 |
| 12 | 1072.152 | 548.388 | 0.511 | 1221.423 | 655.661 |
| 24 | 2122.964 | 1071.605 | 0.505 | 2394.278 | 1233.916 |
| 48 | 4182.295 | 2102.366 | 0.503 | 4619.895 | 2336.212 |
| 64 | 5617.269 | 2810.704 | 0.500 | 6142.044 | 3094.260 |

| taps | old channel mean | new channel mean | old wrapper mean | new wrapper mean | wrapper ratio |
|---:|---:|---:|---:|---:|---:|
| 1 | 451.844 | 386.273 | 1056.439 | 969.470 | 0.918 |
| 3 | 606.747 | 487.111 | 1198.686 | 1084.528 | 0.905 |
| 6 | 864.090 | 603.060 | 1451.165 | 1186.213 | 0.817 |
| 12 | 1384.217 | 866.461 | 1971.874 | 1453.936 | 0.737 |
| 24 | 2448.736 | 1402.470 | 3052.959 | 2000.209 | 0.655 |
| 48 | 4516.047 | 2447.628 | 5149.973 | 3053.308 | 0.593 |
| 64 | 5956.537 | 3172.023 | 6590.567 | 3779.860 | 0.574 |

FIR 比值在所有 tap 数上约 0.50–0.55，说明收益主要来自少一遍临时数组
写读，而不是某一种特殊负载。3 tap 时 FIR 节约 0.134 ms，wrapper 下降
0.114 ms，与 FIR 节约量同量级；没有退化。12 tap 时 wrapper 下降
0.518 ms，P95 从 2.200 ms 降到 1.660 ms。channel 中 FIR 之外的固定成本
（尺度、CFO、数值检查）约 0.32–0.36 ms，未受本轮改动影响。

原始数据：`gr-uwb/build-sic-fullpacket-fused/results/tap_sweep_fused_20261107_003057`。
摘要：`DiscreteCIR_FIR乘加融合摘要_2026-11-07.json`；
表格：`DiscreteCIR_FIR乘加融合_2026-11-07.csv`。

## 绑定与边界

- code-tested commit `ddf335d476620a82e523c4930c8254096edc096a`（计时时的代码
  HEAD；随后只提交 docs/test，不改变生产库）。
- 旧 binary `f8085e03…c6f5d` / library `43e12c18…5ed29`；
  新 binary `bc20ea95…1a784` / library `88343beb…c334d`。
  `ldd` 分别指向各自 build 目录；两边编译参数相同：
  `-O3 -DNDEBUG -fPIC`，GCC 11.4.0，i7-12700。
- 正式计时逐次独占 `/tmp/uwb-sic-benchmark.lock`，`env -u LD_LIBRARY_PATH`；
  运行前后 load average 0.02/0.30（1 分钟）。
- FIR 是生产 `convolution_us`；channel 还包含尺度、CFO、数值检查；
  wrapper 是外部 wall time，还包含 FCS/PHR 校验、重调制、相减与拷贝。
  父子项不重复累加。**wrapper 仍不是总 SIC**，不含 CIR 估计、通信解调、
  样本等待与队列。
- 本轮不涉及 MATLAB、native 重采样、FPGA/UHD/硬件、TWR 或 paced 实时到达；
  不宣称 mean≤0.5 ms、P99≤1 ms 或 200 窗/s 已实现。按上一份开发报告，
  当前端到端主瓶颈仍是通信解调（约 2.0–2.6 ms），本轮未复测。

下一步：本任务按目标停止，不再扩展 FFT、tap 裁剪、估计器或通信解调优化。
