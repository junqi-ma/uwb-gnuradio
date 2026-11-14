# OpenCode 执行任务书：SC16 重采样与 Burst 流水线第二轮优化

日期：2026-11-11。执行方式：主 agent + 三个 subagent，按文件所有权协作。

## 1. 任务与证据边界

目标：消除不必要的初始化和数据搬运，建立可信的稳态性能测量，再通过受控实验决定是否优化 FIR 内核。交付可复现代码、测试、原始数据和报告；不预先承诺加速比。

编写时源码 HEAD：`7ddea4c8faae82d995029918681d26d0d58b553b`，实际分支为 `feature/sic-research`。执行时重新核对 HEAD、工作区和实际加载库；不要根据历史文档推断当前分支。若有用户未提交修改，保留并记录，不 reset、不覆盖。

必读：根目录 `AGENTS.md`、`OPENCODE_START.md`，以及：

- `docs/sic/UHD_SC16_Burst级多核并行开发报告_2026-11-11.md`
- `docs/performance/开发方案_UHD_SC16_Burst级多核并行_2026-11-11.md`
- `gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h`
- `gr-uwb/lib/uwb_uhd_sc16_pipeline.cc`
- `gr-uwb/apps/uwb_uhd_sc16_pipeline_runner.cc`

本任务允许修改上述重采样及流水线实现；不修改 SIC/雷达数学、滤波器系数、输入窗口几何和公开坐标契约。不新增 GNU Radio block，不上板、不安装到 `/usr/local`、不推送、不移动历史 tag、不覆盖历史 build/run。历史指示中的“当前任务”不自动扩大本轮范围。

现有 review 发现，尚非性能归因结论：

1. 重采样器构造时无条件初始化两张旧多线程调度表，共 5 MiB/实例；Burst Worker 强制单核，不使用这些表。
2. runner 的 `start→wait` 总时间包含 Worker 构造、模板加载、scratch 准备和线程退出。100 窗结果不是纯稳态测量。
3. 数据依次经过 source、stream_buf、slot、work_sc16；RX 有 resize 初始化和 erase 搬移。
4. FIR 只复用起点完全相同的输入窗口；相邻滑窗仍重复 SC16 扩展/转换。
5. commit_result 在锁内分配、重排、回调，丢弃窗口也由 RX 调用，存在等待提交锁的可能。

历史报告的 52.5 µs/窗无重采样结果不能直接解释 129 µs/窗平台期；50 µs×100=5 ms，不是 13 ms。必须通过新实验重新归因，不修改旧数据，不把假设写成既成事实。

## 2. 本轮范围与分阶段出口

必做 A：测量合同、旧调度表惰性分配、Worker 就绪协调、生命周期 QA。

必做 B：在 A 通过后优化 RX 预分配缓冲及游标，保持同一输入和窗口输出。

受控实验 C：在 B 归因后，对固定相位调度、块内 SC16 转换复用分别做独立 A/B。主 agent 按第 8 节门槛决定保留，不要求两个实验都进入默认路径。

本轮暂不实现：slot 直接作为 FIR 输入的新接口、结果提交线程/有界重排重构、多个 RX 线程、AVX-512/FPGA 路径。对这些仅记录测量证据和后续建议。尤其不要简单给同一 UHD streamer 增加并发 recv。

## 3. 多 agent 所有权与协作

| 角色 | 独占写入文件/范围 | 交付 |
|---|---|---|
| 主 agent | `gr-uwb/apps/uwb_uhd_sc16_pipeline_runner.cc`、相关 CMakeLists、新基准驱动、总报告 | 冻结测量/API、整合、唯一正式 benchmark 执行者、独立复核 |
| Subagent A：重采样核心 | `uwb_rational_resampler_core.h`、新 `qa_uwb_resampler_burst_opt.cc`、新纯核心基准源码 | 惰性分配、回归、C 阶段内核实验 |
| Subagent B：流水线 | `uwb_uhd_sc16_pipeline.h`、`uwb_uhd_sc16_pipeline.cc` | ready/失败/停止协调、RX 缓冲改造、必要诊断计数 |
| Subagent C：验证 | `qa_uwb_uhd_sc16_pipeline.cc`、新离线比较脚本、验证说明 | 生命周期/窗口边界/账本 QA、逐样点比对、测量有效性审查 |

完整文件路径按现有 include/lib 目录定位。新基准源码建议 `gr-uwb/apps/benchmark_sc16_burst_core.cc`；新验证脚本置于 `tools/performance/`。创建前检查是否已有可复用实现。

所有 agent 可读全仓库，不跨所有权编辑。CMake 修改请求集中交主 agent。共享文件系统下不得多个 agent 同时编辑同一文件；独立 worktree 也不得假设互不影响机器负载。A 的惰性分配与后续内核实验顺序执行，不混成一个无法消融的补丁。

主 agent 先冻结新增计时字段、ready 状态和失败传播接口，随后下发 A/B/C。C 可以先准备 fake source 和边界数据，待 B 接口可用后接入。正式性能测试时暂停所有 agent 的构建、测试和其他重负载。

## 4. Phase 0：基线、构建与测量合同（主 agent）

1. 保存 HEAD、diff、编译器、CMake 参数、CPU/虚拟化拓扑及测试输入/taps 校验和。
2. 保留旧 build；从准确源码建立独立 baseline 和 candidate build，不复制 CMakeCache、不软链旧 build。建议候选目录 `gr-uwb/build-sc16-burst-opt`，基线在独立只读源码 checkout 下构建。
3. 基线和候选使用相同编译选项。记录 ldd 实际库、可执行文件及源码身份；不要仅凭 build 目录名归因。
4. 先构建测量工具并验证计时合同，再采集配对基线。若基线需要加诊断，建立仅含相同诊断的 control 版本，不拿不同计时实现直接对比。

必须分别报告：

- cold_total：保持原 `start→wait` 口径，含初始化和 drain。
- startup_ready：启动到所有 Worker 成功 ready；失败情况另报。
- warm_service：预热后逐窗 resample/SIC/其他 DSP 服务时间。
- steady_throughput：持久 Worker 中固定测量区间完成窗口数除以单调时钟区间；明确区间边界，另报排空尾部。
- RX：实际读入样点/字节、窗口有效样点、source read、拼装/搬移、slot copy、派发耗时。
- 提交：锁等待、重排/回调耗时及最大乱序跨度；诊断开启时才采集细粒度计时。

旧 e2e_us 不包括完整队列等待和 commit，不改名偷换语义；新计时字段另加。父子计时不重复累加；并发线程耗时不能相加当墙钟。诊断计时和正式计时分别跑一次，量化插桩扰动。

稳态基准使用有限池和持续输入，不能把池扩到整个测量 run。预热窗口、测量窗口和尾部窗口各自明确；不能从结果中只挑完成的快窗而遗漏丢弃。fullspeed 吞吐须报告 offered/admitted/completed/dropped 和队列状态。零丢弃基准可在独立基准 feeder 使用显式节拍或批次流控，但不得把它写进生产 RX，也不得隐瞒输入受限。

比较 RX 时保持 native_rate、period、t0、guard、实际读入字节、source 实现相同。不要通过改 native_rate=work_rate 构造“唯一变量为去掉重采样”的实验。可以新增诊断 consumer bypass，但 source/窗口几何必须保持一致。

## 5. Phase A：启动开销与就绪协调

### A1：Subagent A

- 单核构造不 resize 两张百万项调度表。
- 在旧多线程首次需要时保证调度表容量；不能因 size=0 导致 run_mt 无限循环或零进展。
- 复用容量，避免逐调用重新初始化；CF32/SC16、65/48 和 65/32 共用模板路径均需检查。
- 覆盖单核→多核→单核、重复 reset/process/flush、短输入、max_out 限制和多次调用；不删除旧路径。
- 用分配统计或明确容量诊断证明单核不再初始化 5 MiB 表，不能只用 RSS 推断。

### A2：Subagent B

- Worker 绑核、构造及必要准备完成后发布 ready；RX 在全部 ready 成功前不读流。
- 使用明确的 Pending/Ready/Failed 状态或等价机制。任何 Worker 初始化异常必须可传播并唤醒等待者，禁止 std::terminate 或永久等 ready。
- 检查源的 open/start 行为：若 UHD 在 Worker 准备前已启动采集，应相应调整 stream command 时机，不能让设备在屏障后面持续积压。
- stop 可打断 ready 等待；覆盖 stop-before-ready、部分 ready 后失败、重复 start/stop/wait。不能仅依赖收齐 ready 计数退出。
- 保持 RX 完成标志与 Worker 排空语义；不能重新引入 stop_requested 提前退出造成已准入窗口丢失。
- 绑核失败必须可见，不能报告为绑核成功；虚拟环境无法确认物理核时如实说明。

### A3：Subagent C

补确定性测试：慢初始化、初始化失败、准备期间 stop、零窗口/EOF、正常 drain、小池过载。验证输出保序、唯一终态、账本守恒、停止时限。

初始化失败没有正常采集时不得伪造 completed；本轮不强制过载用例保持历史 admitted=8 等调度相关计数，仅验证守恒、无阻塞死锁和输出语义。

## 6. Phase B：RX 缓冲优化（B 实现，C 验证）

基于 source 的真实 short-read/EOF/timeout/gap 行为，采用预分配存储和有效数据游标。避免每次读入前清零即将覆盖的区域；避免每窗 erase 移动残留样点。必要时低频 compact 或环形缓冲，必须测量实际移动字节。

必须满足：

- 区分 allocated capacity、有效样点范围、绝对样点起点；绝不读取 capacity 内未构造或未初始化的无效区域。
- 支持窗口重叠、相邻、间隔大于窗长、跨 chunk、short-read、多次 read 后成窗、EOF 不足窗、首窗负 guard 裁剪。
- 内存规模由窗口/读块及必要保留区决定，不随已运行窗口数增长；远距离 schedule 空隙可分块读过，不把整个空隙留在 stream_buf。
- 保持 source 元数据和绝对坐标语义；若发现既有 gap 处理缺陷，先提交最小复现并隔离修复，不悄悄掩盖。
- 保留每 Worker SPSC、池满准入丢弃、每 Burst 独立 reset、单 Worker 内不嵌套线程池。

测试采用带可预测样点编号的 fake source，逐个比较 slot 样点、native_start_abs/count 及完整结果；不能只验证结果数量或解调成功。

## 7. Phase C：内核优化候选（A 实现，主 agent 裁决）

先交付纯核心基准：预分配输入输出、预热、reset/process/flush 生产语义、确定性数据、输出校验防止编译器消除。将准备和核心执行时间分开，报告吞吐与 P50/P95/P99。

C1 固定调度：预计算四输出分组的相位、相对输入偏移和复用关系；考虑 24/48 padded tap 专用实现。覆盖所有合法起始相位和 remainder，不仅测试 phase=0 的整宏块。

C2 转换复用：缓存小块未缩放的 int16→float 转换，供相邻相位复用；保留现有预缩放 taps、FMA 及归约顺序。转换块需要明确 halo、尾零、对齐和边界。试验有限的块大小，不无限搜索。

分别运行 control、仅 C1、仅 C2；只有独立有效才评估组合。保留显式实验选择，未通过门槛不替代默认内核。`convert_us=0` 仅代表融合计时，不作为转换指令已消失的证据。

硬性正确性要求：对当前 SC16 默认路径逐样点位级一致，含正负满量程、零、随机、脉冲、尾部、短窗和非宏块整数倍长度；覆盖两种现有 scale 模式、两档 profile。原 golden、分块一致性和长度公式全部保留。若归约顺序变化导致无法位级一致，本轮不得擅自放宽门槛。

## 8. 性能矩阵、保留门槛与停止条件

所有正式数字独占 `/tmp/uwb-sic-benchmark.lock`，基线/候选串行 ABBA，至少 5 组配对；记录机器负载和可获得的实际频率。性能测试期间不并行编译或跑 QA。硬件计数器若不可用，记录“未测”，不可假造 IPC/端口饱和结论。

基础矩阵：4000/56000 输入点 × realtime_minorder/quality_minorder × 1/2/4 DSP Worker。8 Worker 只作诊断，不设交付目标。先小规模筛选，再对最终候选跑完整矩阵。

分别保留历史 100/1000 窗冷启动口径，以及足够长的稳态区间；稳态至少采集 10000 个测量窗，必要时延长，所有统计报告样本数。source 不足时使用记录明确的确定性内存回放/循环策略，不悄悄重复跨尾部窗口。报告有效窗口采样吞吐与真实源读取吞吐两者。

本轮工程保留门槛（不是预先承诺）：

- A1：证明无用表初始化消除，全部正确性通过；独立报告 cold 改善，warm 不应显著退化。
- B/C：目标指标配对中位数改善至少 5%，且改善超过实测噪声；其他代表格服务时间/吞吐无超过 5% 的稳定退化。P95/P99 不出现可复现的显著恶化。
- 若数据抖动掩盖门槛，结论为“不确定”，只允许一次有针对性的延长复测，不能重复试到得到好数字。
- 无收益候选不进入默认路径；保存实验结果即可。B 若复杂度显著增加而收益不足，可保留更简单的容量复用改动并撤去无效部分。
- 不用更小滤波器、不同窗口/period、更高丢弃率或不同 consumer 获取“加速”。

流水线功能字段逐窗对照，但不同调度下 worker_id、计时和过载准入集合可能不同，不能要求它们逐位相等。零丢弃同输入实验比较 DSP 输出与功能字段；重采样 IQ 位级检查必须直接进行，11 个 WindowResult 字段相等不能替代 IQ 检查。

## 9. 集成验收与交付

顺序：Phase 0 → A1/A2 并行开发、C 并行准备 QA → 集成 A → 串行测量 A → B → 测量 B → C 候选 → 最终集成。

每次实质修改构建并跑针对性 QA；最终串行全量 CTest，明确通过/失败/跳过数量。不能将 skipped 算作执行通过。内存边界/生命周期修改用小规模 sanitizer 或等价检测补验，不拿 sanitizer build 测性能。

主 agent 最终独立检查：惰性表是否在所有 MT 入口准备、ready 失败和 stop 是否可退出、RX 是否保留重叠数据、尾部 SIMD 是否越界、统计是否遗漏 drop/初始化/排空、是否仍存在提交锁导致 RX 等待。提交路径未重构时明确保留该限制，不宣称整体 RX 已无阻塞。

交付：

1. 按测量、A、B、C 分离的可审查补丁/本地提交；不推送。若不提交，完整 diff/hash 绑定测试身份。
2. 可一条命令复现的基准驱动及参数清单，原始 CSV/JSON、日志、输入和 taps hash；大数据与 build 不入 Git。
3. `docs/performance/UHD_SC16重采样与流水线第二轮优化报告_2026-11-11.md`：代码身份、各变体、冷/热数据、正确性、噪声、收益/退化、未保留候选和未验证项。
4. 更新项目入口的本轮摘要，只写实测通过项。历史报告保留，在新报告中指出旧归因的适用边界。

最终结论不得宣称硬件实时、连续 1 GS/s、UHD overflow 行为或 MATLAB 已验证，除非另有本轮明确授权与实际证据。达到上述交付后结束，不自动进入 deferred 项。
