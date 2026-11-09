# Full-packet 与 preamble-only SIC：历史审查与开发计划

日期：2026-11-06。性质：供 OpenCode 执行的开发任务书；编写时未实现、未运行新实验。
执行原则：主 agent 协调、多个 subagent 按文件边界并行开发，构建和性能测量串行。
先交付可复验的最小比较，再做有证据支持的扩展；不以堆叠测试量代替开发进展。

## 1. 目标与基线

在现有感知侧 preamble-only SIC 上增加 decode-and-reconstruct full-packet SIC，
比较真实处理耗时、样本等待与结果延迟、干扰包解码率、最终感知效果。
full-packet 在线重构只能使用混合 IQ 解出的 PHR/PSDU，不能读取生成器 payload。

当前根目录 `feature/sic-research` HEAD 为 `794056c`，即
`sic-s2-ab-integrated-20261018`。后续实现建议从该 tag 创建
`feature/sic-fullpacket-compare` / `.worktrees/sic-fullpacket-compare`。
本文编写阶段仅新增文档。OpenCode 执行时先检查目标分支/worktree 是否存在：不存在则
从上述 tag 创建，存在则核对来源与修改后复用；不重置已有工作、不修改冻结代码、
不搬移未提交笔记。只复制本任务书到新 worktree，并在该 worktree 的入口标明本轮任务。

`feature/sic-perf` 已存在，HEAD 为 `239954e`，有性能开发与计时整改记录。
本轮不评审其算法或重新认可其数字。主比较先绑定集成基线；后续若评价优化版，
应把经审查的同一优化用于两种方案，并重新配对测量，不能用不同版本拼接比较。

## 2. 历史实现审查

只读审查 `feature/sic-validation`，HEAD `d265730`，对应历史 worktree
`/home/junqima/workspace/uwb-gnuradio-sic`；未在该目录修改或运行构建。

| 对象 | 已有内容 | 复用判断 |
|---|---|---|
| `uwb_tx_reconstructor.h::reconstruct_dw1000` | 解码 PHR/PSDU→RS/卷积编码→BPRF 脉冲→复数 CIR 卷积；独立复查 CRC | 可借鉴字段、编码和 FCS 契约；硬编码 code11、128 SYNC、Decawave SFD，不直接移植为当前主路径 |
| `uwb_sic_core.h::trial_cancel` | FCS 门控、整数对齐、CFO 回归、全局复增益、试减后提交；失败原样旁路 | 可借鉴事务提交；使用第24符号以后的估计、无分数时延，不兼容当前严格8符号模型 |
| `qa_uwb_sic_core.cc` | 合成对齐/CFO/增益及失败旁路 QA；真实 DW1000 fixture 上解码→重构→试减与 MATLAB 产物对照 | 有用的回归来源，但单例 golden 不能证明混合信号通用性或新方案实时性 |
| `开发方案_GNURadio实时SIC.md` | direct-first、必要时先消除期望包再重试、多包候选、有界 PDU worker 设计 | 属历史方案；文中“尚未实现”早于后来核心提交。所查引用未发现完整 SIC block 的接线实现，不能把方案当已完成系统 |

当前更合适的复用对象：

- `uwb_preamble_sic_core.h/.cc`：检测、前8 SYNC 冻结 `SicModel`、多径、分数延迟、绝对 CFO 相位、scratch 和失败旁路。
- `uwb_demod_core.h::demodulate_one`：干扰通信包解码；保留 PHR bits、PSDU、FCS 和失败阶段。绝对/局部坐标转换必须明确。
- `uwb_hrp_mod_core.h::modulate_one`：当前可配置 HRP 整包再调制，优先于另写编码器。首版只接纳可被它精确再生的 PHY/PHR；未知字段拒绝。
- `sic_preamble_sim.py`、`sic_payload_sim.py`：混合信号、公开几何、独立种子、sidecar 与整包生成经验。A 的现有干扰是 SYNC-only，必须新增场景而非改写历史输入。
- `benchmark_preamble_sic.cc` 与 A/B verifier：计时、效果配对、库绑定与可信根约束。

另只读查阅 `UWB_demodulation/sic_pipeline/README.md` 和编排文件。
MATLAB 全文件路径含先解/消期望包、再解干扰以及 PLL、慢相位、CFO2、SFO；
与本轮单干扰、direct-decode 软件实验并非同一算法，不拿其全文件时间直接比较。
实施时继续阅读其引用的 decode、TX 再生、CIR 卷积原语并补针对性对照。
本轮未运行 MATLAB，也未重跑历史 QA。

## 3. 主比较路径与公平性

同一 `y=s+i+w`，s 为完整 sensing 包（code9），i 为完整通信包
（默认 code10、64 SYNC、4z2），998.4 MS/s work-grid CF32，无 STS。
首版只处理一个通信干扰包，不引入期望包辅助消除、迭代或多干扰排序。

1. `raw` / `clean`：原混合与无干扰参考，只用于配对评价。
2. `preamble_only`：保持原8符号估计、完整 SYNC 消除；不解通信包，不受通信 FCS 门控。
3. `full_packet_decode8`：同一检测与8符号模型→在原始混合 IQ 上解通信包→
   校验 PHR/长度/FCS→再调制整包→用冻结模型施加多径/分数延迟/CFO→事务相减。

full-packet 解码不能先从输入中消掉其自身前导。解码器内部为解调所做的同步/CFO
估计正常计时，但主比较的重构模型仍是冻结8符号模型，避免同时改变估计器与相减范围。
如解码器长窗口 CFO/CIR 能改善重构，后续可另设 `full_packet_refined`，单列精度、等待与拟合成本；
本轮不要求实现该增强组或 preamble fallback，避免扩大交付范围。

整包重构直接从原始 y 相减一次，不在已经消除 SYNC 的结果上再减整包。
full-packet 失败默认原样旁路；可另设显式 preamble fallback，但不能混入主组解码成功率
或 full-packet 提交率。FCS 是在线门控，PSDU 真值匹配仅为离线正确率判据。

使用同一 IQ、公开粗 ROI、目标输出 ROI、脉冲、信道、ISR 定义、种子、机器与构建。
生成器按统一通信 SYNC 参考区间确定整包幅度，不因 payload 长度重新归一化 ISR。
计算量会随实际处理长度变化，必须记录上下文样本数、读取上限、重构长度与相减长度。

必须同时研究两种时序：通信 SYNC 重叠 sensing CIR 区；通信 payload 重叠 sensing CIR 区。
后者需提供前置上下文以真实接收通信前导，不可把注入起点作为检测 seed。
对齐场景中两种方案 CIR 接近也有意义：full-packet 可能付出了额外成本而无 CIR 收益。

## 4. 必须先解决的重构契约

- 解出的 wire PSDU 含2字节 FCS；再调制不得重复追加 FCS。PHR 长度/速率/标志要一致，
  不支持的 PHR 明确拒绝，不能重生一个默认 PHR 冒充原包。
- `modulate_one` 末端有整包峰值归一化。整包与独立 SYNC 模板的比例必须显式标定并验证；
  不能假设当前 `SicModel.gain` 可直接乘任意归一化的波形。
- 当前 gain 已相对于成形后的前导模板估计：整包先生成相同成形波形，再施加模型。
  不再额外卷一次含发射脉冲的 CIR，避免重复成形。
- 保持绝对 `tau_abs`、`phase_reference_sample`、同一分数延迟算子和 CFO 符号，
  将支持扩到整包及真实滤波/路径尾部。不得直接绕过现有冻结模型支持边界。
- 新增独立 full-packet apply API/模型封装，不改原 preamble apply 的有限支持契约；
  分开记录 packet support、consumer ROI 与各字段边界，区间外逐位不变。
- 抑制验收以离线真实残余和目标保护为准；总能量下降不能单独构成算法有效证明。

## 5. 延迟、时间与解码率合同

各阶段用单调时钟测 service wall time，必要时另记线程 CPU 时间，不能混称。
阶段至少包括 detection/estimate8、comm demod、CRC/合法性校验、TX 再生、
channel apply、subtract/commit、复制/PMT、下游 CIR、下游 sensing 完整解调。
父子计时不重复相加；初始化/模板缓存/生成器/I/O 单列。

三种延迟分别报告：

- 样本可用等待：从共同公开窗口时间原点到模型就绪/通信帧尾可用的采样时间。
  前8 SYNC 本身约8.14 us，但不是系统延迟实测值；还需包含搜索与必要上下文。
- 数据就绪后的处理与排队延迟：从实际可提交输入到输出完成，拆开 queue/service/consumer。
- 从共同起点到目标 sensing 结果的端到端延迟：按真实依赖时间线记录，不能将并行或
  已覆盖的采样等待与计算简单相加。分别记首个可用净化输出与完整目标结果。

全窗口离线一次性喂入只证明计算时间，不能证明早输出。
第二阶段优先用轻量逐块回放器、前缀可见性与有界缓存测时间线；preamble 路径不得等待通信帧尾。
离线虚拟到达时间与真实 paced 时钟结果分列，不能把虚拟回放称为实测端到端延迟。
早期结果若依赖8符号之后 IQ，必须显式记录实际观察终点，不能只写 `fit_reps=8`。

解码率至少报告：通信 FCS pass / 全部注入包、离线 PSDU exact / 全部注入包、
FCS pass 条件下 PSDU exact、full-packet committed / 全部输入窗；
另报 detection、PHR、payload/FCS、截断、数值、队列失败计数。
原始/净化 sensing 包成功率与通信包成功率分开。总体与成功条件下的耗时都给出，
附样本数与比例置信区间；不从主分母删掉解码失败、拒绝或超时。

## 6. 分阶段交付与验收

| 阶段 | 开发内容 | 完成证据 |
|---|---|---|
| P0 契约与夹具 | 新 worktree、运行 manifest、完整通信干扰生成、公开几何与 sidecar 隔离、计时 schema | 接口冻结；SYNC/payload/无重叠可复现，固定短 golden 与生成器自检 |
| P1 数学闭环 | 先补 QA，再实现 decoded-frame→TX 与 frozen-model→full-waveform apply；C++ 同步调用最小 runner | 无混叠包可解且正确再生；随机长度/PHR、字段与尾部、幅度比例、相位/坐标、分数延迟 QA 通过 |
| P2 配对功能实验 | raw/clean/preamble/full 主组，随机未知 payload、FCS 失败旁路、完整输出 verifier | 同窗输入 hash 一致；真实输出残余、CIR、目标和两类解码率独立复算；困难失败如实保留 |
| P3 到达与延迟闭环 | 轻量分块/真实 paced 回放、观察前缀、缓存/超时、单 worker；优先复用现有调度 | 帧尾前不可能 full commit；preamble 可早输出；失败唯一终态、区间外不变、无遗漏窗 |
| P4 正式计时交付 | 独占 benchmark 锁、同机串行交错配对、热身与重复、全部终态统计 | mean/P50/P95/P99、吞吐/超时/丢弃、IQ 内存、分阶段比例、效果配对和代码/库 hash 可追溯 |

首个实现里程碑是 P0–P2 的单径/零 CFO 正确性闭环；之后继续完成 P3–P4，
不能停在方案或 smoke。不以达到实时目标为完成前提，也不为追数字反复修改算法。
默认不新增生产 GNU Radio block，以已有同步 core 加回放 runner 完成比较，
不开发通用调度框架。若现有接口确实无法提供所需时间线，再记录证据并做最小 adapter。
P3 若需新增 GNU Radio block，实施前必须查本地类似 block，记录 scheduler 语义并先补 QA。
建议零流端口 message/PDU block，handler 只校验与有界入队，worker 独占预分配 scratch；
PMT 入口与 worker 队列均限流。同步 core 与 adapter 分离，不能在 handler 里整包解调。

### 6.1 验证预算与停止规则

以下是默认预算，主 agent 可因明确失败或新增风险定向调整并记录原因。
不运行上述各维度的全笛卡尔积，不复跑历史24200包或完整旧性能矩阵。

| 层次 | 默认工作量 | 何时运行 |
|---|---|---|
| 局部开发 QA | 每个非平凡契约1个有辨识力的用例；分数延迟/字段边界选少量代表值 | 有意义修改后构建受影响 target，只跑相关 QA |
| 接线 smoke | code10/64、32 B、单径/CFO0、SNR30、ISR10/20、SYNC/payload重叠，各4窗 | 接线后一次；修复影响该路径时定向重跑 |
| 新功能代表集 | 约10格，每格16窗，同窗跑全部主组 | 集成正确后一次，不随每个文档/日志改动重跑 |
| 既有回归 | 原core/adapter及受影响demod/mod QA；A12格、旁路、B11格代表入口 | 最终代码集成后统一一次；后续只因相关代码变化或失败重跑 |
| 正式计算计时 | SYNC和payload重叠两个代表格，各1024窗、2轮串行交错配对；每组每轮约32窗热身 | 功能稳定后一次，热身不计入统计；结果按轮及汇总报告 |
| 真实paced延迟 | 同两个代表格，各1次约10秒，单worker；200窗/s为尝试负载 | 一次收集排队/完成/拒绝；失败如实报告，不反复尝试直到“通过” |

10格代表集建议：基准32 B/SNR30/ISR10/单径/CFO0的两种重叠；
payload重叠下16 B和127 B长度边界、SNR8、ISR0和20、CFO±50 kHz、
official四径加0.4 sample时延，共10格。B长度均含FCS，先校验capability。
无重叠与无干扰放入小型负样本QA，不增加大量统计窗。
在 P0 固定格表、seed和计时窗口数量；不根据结果挑选更好看的格。

每格16窗的解码率用于代表性观察，附置信区间并说明不够精确；
正式计时两格的较大样本也用于解码率统计。P99附样本数和轮间差异，不能用少量
smoke样本宣布稳定尾延迟。零失败报告有限样本置信上界。
只有轮间差异足以改变主要比较结论、发现计时污染或修复实质错误时才补一轮；
不得常规追加长soak、多worker、多机器、全性能矩阵或全量sanitizer。
200窗/s过载即可交付实测失败；可补一个较低负载帮助解释，但不是强制项。

负样本覆盖：无干扰、错误 code/profile、坏 FCS、非法 PHR/长度、上下文截断、
NaN/Inf、尾部越界、队列满/取消、前缀之后 IQ mutation、sidecar 真值污染。
这些是少量定向QA，不是各自再建统计矩阵；已有等价QA直接复用。
队列满/取消只测试本轮实际使用的调度层，不为测试而新增通用队列功能。
生成器和重构器即使复用同一 TX 核心，也需独立字段/编码/golden 对照，避免共同错误。
禁止为getter、简单转发、重复实现细节或纯文档修改补测试。局部验证通过且代码未变，
不再运行同一检查；最终集成回归除外。遇到失败先定位并修复，不批量重跑碰运气。

正式计时使用 `/tmp/uwb-sic-benchmark.lock`，无其他吞吐任务；绑定 HEAD、profile、
实际加载库、CPU 环境、窗口长度与输入 hash。`env -u LD_LIBRARY_PATH`，ldd/maps 核对
新 worktree 库。200窗/s 只作待验证负载；延迟提交与入口拒绝均记失败，不降低实际输入率。

## 7. OpenCode 多 subagent 分工与调度

本任务明确要求并行开发。主 agent 负责接口、共享文件、集成、测量和最终报告；
将下列有独立文件边界的任务交给多个 subagent。按可用槽位调度，建议主agent加3个
subagent；槽位不足时排队，不为增加并发再嵌套派生审查agent。

### 7.1 先冻结最小接口，再并行

主 agent 先完成 worktree/入口设置与约一页接口契约，至少固定：
输入 IQ/公开 profile/粗 ROI/observed_end、decoded-frame字段、模型与waveform归一化、
输出IQ/状态/时序事件、JSON schema、命令行和文件格式。禁止在线传入真值。
主 agent 创建公共头文件骨架与runner骨架，确保各任务可以基于stub独立工作；
接口变更由主 agent 统一通知，不能让各agent各写一套schema。

以下名称为建议；P0可统一调整一次。所有路径相对 `gr-uwb`，tools/testdata除外。

| 任务 | 独占文件范围 | 实现与交付 | 依赖 |
|---|---|---|---|
| A：整包再生与相减 | `lib/uwb_fullpacket_sic_core.cc`、`lib/qa_uwb_fullpacket_sic_core.cc` | 解码字段校验、复用HRP再调制、幅度比例、冻结模型施加、有限支持与事务相减；针对性QA及调用示例 | 公共头文件骨架；不依赖B的生成器，先用短fixture |
| B：混合信号与配对输入 | `apps/sic_fullpacket_sim.py`、`testdata/sic_fullpacket/`（排除A/C文件） | 完整通信干扰、两种重叠、独立seed、在线输入/sidecar隔离、固定格表和短fixture；小型生成器自检 | 输入/文件schema；不依赖A实现 |
| C：独立verifier与统计 | `tools/sic/verify_fullpacket_compare.py`、`tools/sic/qa_fullpacket_compare.py` | 从IQ/sidecar独立复算字段残余、CIR/目标、PSDU/FCS与率、终态计数/时间线；少量篡改反例 | 输出schema；先用合成记录stub，之后接B/A产物 |
| 主agent：集成与调度计时 | `include/gnuradio/uwb/uwb_fullpacket_sic_core.h`、`apps/benchmark_fullpacket_compare.cc`、CMake、现有文件必要适配、本文和最终报告 | 接入现有estimate8/demod，串行两组runner、分块/paced到达、缓存与时间戳、最终构建和测量 | A/B/C分别就绪后集成；可在stub阶段先搭框架 |

公共头文件由主 agent 维护，A有接口需求通过消息提出，不同时编辑。
若复用分数延迟算子需要改旧core，先给主agent具体最小改动建议，由主agent实施；
保持旧API/默认行为。禁止复制旧文件后大改成另一套检测/解码器。
C不得调用A的重构函数作为唯一正确性oracle，避免共同错误自证。

### 7.2 调度顺序与交接

1. 主agent完成P0最小契约/骨架后，同时派A、B、C；每份任务附接口、文件所有权和验证预算。
2. 主agent并行搭建runner与解码接线；B交付短fixture后立即接A的闭环，不等待完整统计器。
3. A/B完成后，主agent集成并跑一次smoke；C用真实产物完成验证。
4. 功能代表集与verifier通过后，主agent运行一次既有回归及正式测量，汇总开发报告。
5. 发现问题交回对应owner定向修复；只重跑受影响检查。不要再启动一轮无范围的全面审查。

每个subagent完成时提供：改动文件、接口假设、最短复现命令、已跑QA及结果、
尚未验证项/已知限制。不得只提交计划，也不得私自跑正式benchmark。
可共享同一新worktree并按上述所有权编辑，不允许互相reset/revert/覆盖文件。
若平台使用独立worktree，均从主agent的契约提交分叉，由主agent统一合并。

### 7.3 共享资源约束

仅主agent修改CMake和公共头文件。共享build只由主agent串行构建；subagent通过
主agent请求一次目标构建/QA，或使用独立小型build目录，不能同时写同一build。
任何正式耗时/吞吐测量都等所有编译、QA和其他benchmark停下后由主agent独占锁运行；
并行开发节约的是实现时间，不能并行测量制造噪声。
若某个任务提前完成，可安排其补自己的缺陷或提供结果解释，不追加无关功能与测试。

## 8. 范围与交付物

本轮后续开发不整体合并历史 SIC 分支，不自动合入 perf 分支，不上板、不扩到 TWR/native。
保留 A/B 原回归及结论边界；大 IQ/build 写新结果目录，不覆盖历史 run。
MATLAB 软链接只读，新验证脚本放 testdata；未实跑项明确写未验证。

最终交付：新接口契约、QA、完整包混合仿真 runner、独立 verifier、计时/到达 runner、
配对结果表（效果/解码率/延迟/CPU/失败）和开发报告。先给可复验结论，再讨论优化。
