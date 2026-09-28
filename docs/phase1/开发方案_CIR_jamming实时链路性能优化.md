# CIR jamming 实时链路性能优化开发方案

状态：待实施。范围为 `x410_cg400_hrp_echo_cir_jam.py` 的 C++ PDU 实时路径；Python echo 后端和独立 stream flowgraph 仅做回归检查。本文不改变 998.4 MS/s CIR 数值定义、UCR4 文件格式、逐 repetition 顺序或 TX/RX 时序。

## 1. 现状与约束

运行链：`UwbRealtimeEchoTimer` → `UwbPduRationalResamplerCcf65_48` → `UwbRadarCirEstimator` → `UwbCirWriter`，可选 `CirUdpSink`。这些是零流端口消息块，GNU Radio 的 `min_output_buffer` / `max_output_buffer` 不作用于该链。容量约束应按 PDU 样点数、各级 scratch 大小和**队列条目单位**检查。消息处理函数由 GNU Radio 消息调度触发；estimator/writer 各自用有界队列和单 worker。不能让上游复用的 RX scratch 指针逃逸至异步消费者。

现有 50 Hz 硬件记录：128 repetition、103535 native ROI 时 estimator mean/max 为 3.360/5.397 ms；256 repetition、199570 native ROI 时为 4.825/6.130 ms，两个记录的 res/est drop 均为 0。另有 200 Hz、128 repetition、UDP 开启的 60 s 记录：estimator mean/max 3.452/6.133 ms，队列不丢，但 `echo_late=300`，整体硬门槛未通过。硬件 late 与 CPU 阶段耗时分别报告，不能把 CPU 改善当成 late 已解决。

开始改动前，在固定 commit、Release 构建、相同 CPU/系统负载、相同 TX 波形及 ROI 下保存基线：128/256 repetition，`repetitions`/`both`，UDP 开/关。记录 resample/convert、estimator service/queue、writer/UDP 队列高水位与 drop、RSS 峰值、UCR4 条数及逐 tap 数据。离线基准先用 `benchmark_radar_pipeline` 和 `benchmark_resampler_65_48`，硬件验证放在各阶段 QA 通过之后。已有吞吐阈值 QA 受主机负载影响，单独报告，不以一次波动代替回归结论。

## 2. 阶段一：去除重复平均 CIR 计算

**代码边界**：`include/gnuradio/uwb/uwb_radar_cir_core.h`、`lib/uwb_radar_cir_estimator_block.cc`；测试为 `lib/qa_uwb_radar_cir_estimator*.cc`，必要时扩展 pipeline benchmark。

1. 从 `radar_cir_one()` 中抽出可复用的 SFD、SYNC refine、预测 CIR origin 与边界检查；保持公开入口及 `average` 模式结果不变。先写 `repetitions`/`both` 的成功、失败、边界窗 QA。
2. `repetitions`：定位一次后逐 repetition 估计；不先执行相干平均。现有平均结果作为有效性门槛的行为需要显式保留或用等价的有效 repetition 检查替代，避免把原来失败的 pulse 意外标为成功。
3. `both`：定位一次、逐 repetition 一次、仅按 `repetition_average_skip` 求一次平均。注意 scratch 在每次估计后被覆盖，发布前必须持有对应结果，不能复用过期指针。保留 `cir`/`cir_avg` 消息、状态计数及发布顺序。
4. 平均相关先复用已准备的非零码片索引，减少完整 sampled code 扫描；此项单独提交和测试，以便区分“少算一次平均”与“每次平均更快”的收益。

**验收**：原有逐 tap MATLAB golden、UCR4、pulse ID、repetition index/count、peak、`raw_l2_norm`、失败状态与消息数量一致；`both` 的平均使用原指定 skip。先要求标量优化输出与旧版误差落在现有 golden 容差内；如改变浮点加法顺序，单独列出最大绝对/相对误差。相同输入下 estimator 平均及 P99 耗时不得升高；未达成则回退该子改动。

## 3. 阶段二：按 pulse PDU 计量 writer 队列

**代码边界**：`apps/x410_cg400_hrp_echo_cir_jam.py`、writer 统计接口及相关 QA。

当前 estimator 把全部 repetition 打成一条消息，writer 入队也只占一个条目，但 app 把 `4 × repetition 数` 作为队列容量。把容量参数和日志明确命名为 pulse PDU 条目；对 `cir` 与 `cir_avg` 分别配置。起始候选为 64 个 pulse PDU，最终值以生产负载下的高水位、写盘延迟和允许的瞬时阻塞时间确定，不直接把 64 当作验收值。按 `容量 × 每 pulse 最大 PDU 字节数` 计算最坏 RAM 占用并写入日志。

加可控慢 writer 测试：确认高水位、满队列丢弃计数、停止时 drain、文件条数与顺序。硬件对照中 `wr_drop=0`，高水位有余量，RSS 峰值下降；不能为了降低 RSS 使既有可承受的短暂写盘抖动变成丢包。保持 estimator、resampler 输入上限与 ROI 契约不变。

## 4. 阶段三：按消费者选择 normalized CIR

**代码边界**：jam app、`uwb_radar_cir_estimator.h`、estimator block、writer/UDP QA。

先列出 `cir` 与 `cir_avg` 的所有消息消费者。当前 UCR4 writer 只写 raw SC16 加 scale，`write_normalized=true` 仅检查 `normalized_taps` 存在。为 jam app 增加明确的 normalized 输出策略：无消费者时不生成批量 normalized PDU，也不要求 writer 检查；需要兼容旧消息契约的场景可开启。公开 estimator 默认行为及其他 app 不随之改变。若需要省掉核心计算，让估计函数按参数选择是否写归一化 tap，但仍计算用于指标的 L2 norm，并确保旧调用保持原结果。

验证 raw UCR4、UDP 数据、JSONL 指标与阶段二逐字段一致；normalized 开启时与原实现逐 tap 对照。记录 PMT 创建耗时、estimator service 与 RSS；没有可测收益时保留兼容路径，不强改默认消息契约。

## 5. 阶段四：SIMD 与 65/48 resampler 扩展

**先做 CIR 内核**：对稀疏相关实现跨相邻 tap 的 SIMD 原型，保持预计算稀疏码片和 double 累加语义的可比性；标量路径作为回退。分别量测 116 taps、128/256 repetition 的内核时间、整块 service、精度。不要改成对全部 1016 码片做密集 SIMD，除非实测更快。编译期隔离 AVX2/FMA 实现并做运行时 CPU 检测，非 AVX2 CPU 走现有路径。

**再做 resampler**：当前 65/48 core 已有 VOLK 与条件编译的 AVX2/FMA 内核，但本地构建未开启 AVX2/FMA，CG600 PDU 实例也未使用 `--res-workers`。先在相同 tap 文件、103535/199570 ROI 上比较现有 VOLK、隔离编译的 AVX2 内核及单 worker 基准。只有 resampler 的 service/queue 数据证明需要时，才为 PDU 65/48 引入有界 worker 并行；跨 pulse 并行必须保证消息顺序、元数据和停止 drain。不要直接给整个模块设置 `-march=native`。

验收除速度外，必须覆盖不同 ROI、SC16 scale policy、FIR 尾部 flush、输出长度、sample rate 与坐标元数据，以及 MATLAB 对照数据。`--res-workers` 在 CG600 路径要么确实生效且有明确统计，要么明确报错/提示，不能静默忽略。

## 6. 阶段五：减少复制与格式转换

先用现有阶段计时和 profiler 确认复制占比，再逐项处理：

1. 融合 SC16→CF32 转换与 resampler 输入 work 缓冲装配，目标是少一次完整 CF32 窗口复制；预分配容量与历史样本处理必须覆盖最大 PDU。不要让异步消息引用 EchoTimer 可复写的 `d_rx_buf_`。
2. writer 的 SC16 scratch 保持容量、按记录大小调整 `size()`；编码器成功路径会覆盖全部元素，避免每条记录先清零。失败/空 tap 路径仍须写规定的零数据。评估批量写入或减少小 `write()` 调用，仅在写盘 profile 证明瓶颈时采用。
3. Python echo/UDP 路径单独 profile。Python RX 的临时数组及 `tolist()` 转 PMT 复制不应混入 C++ 后端收益统计；如果 UDP 成为瓶颈，再考虑 C++ 化或批处理，保持现有 UCR4 datagram 及顺序。

每项改动单独提交。要求接收窗口内容、resampler 输出样点、CIR tap 和文件/UDP 字节在约定精度或格式下与基线一致；运行中不新增随 pulse 增长的分配或无界队列。不得以牺牲吞吐换取较小 RSS。

## 7. 构建、测试与完成条件

每个有意义改动之后运行 Release build 和受影响 QA，再运行完整 CTest；至少覆盖 `qa_uwb_radar_cir_estimator`、`qa_uwb_radar_cir_estimator_block`、`qa_uwb_cir_writer`、`qa_uwb_pdu_sc16_scale_65_48` 以及相关 resampler QA。测试要有成功、SFD/定时失败、ROI 边界、队列满、stop/restart 和 128/256 repetition。MATLAB 使用 `testdata/uwb_radar/` 已知信号和 `estimateCir.m`/`verify_read_uwb_cir.m`，逐 packet、逐 tap 对比；必要时补充自动化对照脚本。

性能验收分三层：内核微基准、无硬件完整 PDU 链、同配置 X410 运行。每阶段记录均值/P95/P99/最大值、吞吐、队列高水位与 drop、RSS、CPU 使用率、`echo_late`、UHD underflow。与基线比较时使用多轮重复测量；性能差异小于测试波动时不得宣称提速。128 repetition 200 Hz 和 256 repetition 50 Hz 是必须覆盖的生产配置；300 Hz 与 10 min soak 可作为后续目标，未实际通过前不标为验收完成。最终交付每阶段代码提交、测试记录、差异表和剩余瓶颈清单。
