# OpenCode 开发指示：M1-B 协议核心与双端仿真

日期：2026-09-30。状态：**执行规划已编写，尚未实现**。
规划基线：`feature/uwb-ds-twr`，HEAD `2cf881b`。
本任务单供用户交给 OpenCode 执行；编写任务单不表示任何子任务已经完成。

## 1. 目标、依据与当前事实

交付纯 C++ 的 SS/DS 两端协议核心、确定性 fake link、故障 QA，以及
Python 配置 → C++ 仿真程序 → 结构化结果的最小示例。
端点的应答由有效 RX 事件触发；真值仅供测试观察者验收。
本阶段完成的是**离线协议闭环与可追溯的协议 ToF 估计**，不是硬件测距验收。

执行前阅读：

1. [需求](需求_UWB_SS_DS_TWR.md)、[总路线](开发路线与验收矩阵.md)、
   [参考资料与 API 映射](参考资料与API映射.md)、根目录 `AGENTS.md` / `开发状态.md`。
2. [M1-A 报告](M1-A_ToF数学开发报告.md)，特别是 §7、§9、§10。
3. [N01–N06 评审](评审意见_M0.1_2026-09-29.md)、
   [整改报告](M0.1第二轮整改报告_N01-N06.md)、
   [N07 报告](M0.1_N07修复报告_2026-09-29.md)。
4. [调度语义](M0_调度语义.md) §3、§8，尤其 C-9/C-10/C-13/C-32。
5. `uwb_twr_{types,frame,timestamp,tof_input,math,config}.h` 和相应 QA。

已完成：M0/M0.1 契约整改；M1-A 精确 ToF 数学和 MATLAB oracle。
MATLAB 实跑记录来自 R2025b/WSL 主机，见提交 `10426cf`；当前 Linux 复核
读取检入向量，不意味着在这台机器再次执行 MATLAB。

2026-09-30、本任务单编写前的现场基线：

| 检查 | 实测 |
|---|---|
| 当前 build 全量串行 CTest | 52/53，127.98 s；安装消费者实际执行通过 |
| 唯一失败 | `uwb_qa_uwb_pdu_rational_resampler.cc`，约 214.735 PDU/s，门槛 >400；另有 headroom/input-rate 断言失败 |
| TWR CTest | 8/8（config/frame/PHY matrix/timing/timestamp/admission/parity/math） |
| Python 配置 QA | 138 项通过 |
| M0.1 独立验证 | 15/15 |
| M1-A 独立验证 | 4/4；12 条向量从原始端点 ticks 独立复算，与 C++ 一致 |
| oracle SHA256 | `9d4dd0036240d8abaaeae1d7655eb2045c0da3970613645063909d65d14249d4` |

以上是原有 build 和独立 C++ 探针的验证记录，不是新 M1-B 构建结果。
历史吞吐失败独立追踪，不降门槛，不以一次通过宣称已修复。

## 2. 范围和完成边界

本任务包括：

- 两个独立端点的 SS/DS FSM；每端至多一个在途 exchange。
- 显式事件/动作、帧匹配、定时发送计划、发送证据收敛、超时/取消/reset。
- 两端设备时钟、传播与故障的确定性仿真；固定 seed、可重复事件顺序。
- wire 时间声明的独立输入适配，以及不冒充正式测距的协议估计结果（§3）。
- 有界事件、动作、结果存储及守恒；小包 bytes/trace golden；安装消费验证。
- Python 只负责现有配置构造和结果消费，逐包运行只在 C++。

本任务不实施 GNU Radio block/controller、PMT、UHD radio adapter、真实收发、
两 RX 路由、native 重采样、首径估计、实际波形 timestamp patch、CFO/SFO
估计器、pybind 大迁移、Report、STS 或商用 SDK adapter。
fake 阶段的量化/帧字段填写不等于实际 PHY/FCS/FEC/波形 patch 已实现。
48 行 PHY 白名单保持 work-decode 证据，测距用途仍不开放。

允许的既有代码改动仅限本任务必要的构建/安装/消费者集成，以及 §3.3
规定的最小数学内核提取。其他契约缺陷须先复现、补 QA、独立记录和评审；
不能靠放宽时间准入或重写 golden 使 M1-B 通过。
M1-B 验收后提交报告，不自动进入 M2。

## 3. 前置接口任务：wire 声明与本端测量分开

### 3.1 已知缺口，不能由 fake link 掩盖

v1 codec 是 14 B 头；Poll/Response/Final 的 MAC 长度为 14/24/29 B，
附 FCS 后为 16/26/31 B。Response 携带 `t2B/t3B`，Final 携带
`t1A/t4A/t5A`，默认每字段 40-bit LE **整数** ticks。

这些帧不携带远端 epoch、tick rate、质量、校准记录、分数或 `TxSendEvidence`。
而 `compute_ss_tof()` / `compute_ds_tof()` 只接受 `AdmittedRangingInterval`。
接收帧中的整数不能凭空升级成 `HardwareMeasured`、已校准或 TX Completed。
特别是 B 接收 Final 时，不能从同一 Final 获得 A 随后才收敛的 TX async 结论。
本地 C-32 发送证据规则不等于远端帧可以证明相同事实。

另有必须沿用而非猜测的接口事实：

- `Frame.seq` / `PeerExpectation.seq` 是 **uint16**，`TwrConfig.session.sequence`
  当前是 **uint8 初值**。本阶段初值有界提升到 uint16，运行序号按 codec 管理；
  同时遵守配置的 `sequence_modulus`（现支持 4/16/64/256），不因 wire 能装
  uint16 就静默扩大运行模数。不把 v1 写成 8-bit sequence，也不为此迁移配置 schema。
- `frame_match()` 检查目的地址和 self-addressed，但不保证 `src_addr == configured_peer`；
  FSM 必须额外显式校验配置中的对端源地址。
- 现有 `admit_ranging_interval()` 一个 context 同时检查两个端点。不同本端 TX
  事件的证据先逐时间点核验；若 pair context 无法保留逐点证据，设计明确适配，
  不把某个 TX 的 Completed 位套到所有历史 TX。

### 3.2 本阶段采用的结果语义

保留三帧 v1，不新增回执。新增**协议声明路径**，与现有严格测量路径分开：

| 信息 | 来源与用途 |
|---|---|
| 本端 RX 时间/质量/校准 | 本端 RX 事件；完整进入现有准入门；模拟值标注外层 simulation provenance |
| 本端 TX 时间 | 本端量化计划；Completed 前只能保留计划，不进入严格准入 |
| 远端 wire ticks | 已匹配帧中的协议声明；绑定会话约定，不是对端私有事件或硬件回读 |
| 对端时间单位/marker/校准约定 | 显式版本化的会话 profile 声明；结果说明其为约定而非运行时独立证明 |
| 传播延迟/真实距离 | 仅测试传输模型与验收观察者持有；禁止进入端点接口 |

结果至少分别表达 `protocol_completed`、`estimate_available`、
`local_evidence_complete`、`peer_evidence = wire_claim`、
`measurement_valid = false`、`execution_mode = simulation`。
协议完成使用独立强类型 `ProtocolCompletionStatus`，不使用
`ExchangeStatus::Ok` 作为结果顶层成功状态：现有
`exchange_status_yields_range(ExchangeStatus::Ok)` 会返回 true。
失败可附带既有 `ExchangeStatus` 原因；成功不伪造一个可传给 range helper 的状态。
protocol result 的任何 `yields_range`/measurement-valid 入口都返回 false；
类型检查证明不能隐式传给现有 range helper。负 ToF/数学失败仍保留符号和值并显式失败。
本地证据未收敛时只允许内部候选估计，不发布成功终态。
公开 `estimate_available=true` 要求计算端所需本端证据全部收敛；证据等待
过期或失败时发布失败原因，不泄露候选 ToF 为可消费成功结果。
无 ToF 归属的一端也有本端协议终态：SS responder / DS initiator 的
`estimate_available=false`，不能从另一端结果复制距离。

这不是降低现有测距准入门，而是新增显式的诊断估计类型。M2/M3 正式结果如何
使用可信对端协议声明、如何证明端到端有效性，须另行设计和验收；本任务不声称已解决。

### 3.3 类型与数学复用的实施约束

Agent A 先提出并测试以下最小接口，名称可微调，语义不变：

- `WireTimestampBinding`：peer domain/rate/bits、会话 generation、marker/单位与
  校准约定 ID、允许的最大 interval 和序号复用规则。
- `PeerTimestampClaim`：已匹配帧的字段/raw ticks/消息身份，provenance 为 wire claim。
- `ProtocolInterval`：明确区分本端已准入 interval 与远端声明 interval；
  构造入口默认拒绝；不能隐式转换为 `AdmittedRangingInterval`。
- `ProtocolTofEstimate`：精确 num/den、A domain、计算端、ratio、输入来源和
  声明/量化限制；不能隐式转换为现有严格 `TofResult` 或有效硬件测距。

若为复用数学需改 `uwb_twr_math.h`，只把 SS/DS 精确有理数运算提取为一个
共享内部内核，再由**原来的严格入口**与新的具名协议估计入口各自做输入检查。
不复制公式，不给原 `compute_*` 增加裸 Timestamp/裸 interval 重载，
不增加 `skip_validation`/测试后门，不修改 `uwb_twr_tof_input.h` 的准入规则。
原 API、状态映射、负值、溢出检查、域/ratio/window 检查及 14 例 QA 保持语义。
协议估计入口须做相同必要的数值、域、有效期检查；两条路径对照独立 oracle，
不能只检查两者输出相等。
远端声明路径只检查 binding/消息身份/domain/rate/window/数值与 wire 可表示性，
不能要求或伪造远端 first-path、校准应用记录、TX Completed；相关配置是约定，
不是运行时证据。本端输入仍完整走严格 admission。

在创建数值间隔前先校验 profile、generation、字段来源及 enum 域。
wire wrap 使用已有 tick-space 模差工具；完整间隔遵守其明确上界，包括已知
later/earlier 时允许恰好半周期的既有政策，不把排序的半周期歧义政策误套到间隔。
额外受配置的 exchange-span 上界限制，不允许超过半周期。
wire 无 fraction：超宽 ticks、分数 wire 值、无法表示的映射明确拒绝；
最小基线以 whole-tick wire 场景闭环。分数本端事件不得被静默截断后声称无损，
只能走已记录的量化规则或明确失败，并验证实际 wire 数值决定的估计。

frame 不含 peer epoch，不能宣称能检测一个数值上完全相同的远端隐蔽 reset；
本端观测到 reset/session-generation 变更立即取消在途，未知远端变化是协议声明限制。
任何会话绑定缺席均拒绝；fake transport 不补发隐藏 epoch/校准/完成证据。

**G0 门槛**：上述边界形成设计记录；QA 先证明 wire 不能伪造成严格测量输入；
协调者与只读 reviewer 对照需求和旧 QA 审查通过，才实现依赖此接口的 FSM 数学路径。
普通类型/命名/文件细化由协调者决策，不反复询问用户。
若只有改变 wire 协议、启用真实 radio 或放宽正式测距准入才能解决问题，记录
具体反例与备选方案，完成不依赖它的任务，停止该依赖路径；不要静默扩大范围。

## 4. 协议核心契约

### 4.1 事件、动作和身份

core 是纯 C++ 对象，不是 block。不 include GNU Radio/PMT/UHD，不读取系统时钟，
不 sleep、不 I/O；时间推进由输入事件驱动，格式化留在冷路径。

输入至少包括 `Begin`、`RxFrame`、`TxPlanned`、`TxAccepted`、
`TxOutcomeResolved`、`Deadline`、`Cancel/Stop`、`Overflow`、`Reset`。
输出至少包括 `ArmRx`、`PrepareTx`、`SubmitTx`、`AbortPending`、`TerminalResult`。
新增 enum 显式 `is_known()`，先域检查再无 default switch；非法输入报告失败不抛异常。

每事件关联本端 endpoint、session/generation、exchange/attempt、TX token 或窗口 ID。
不要把对端 private event、真值 ToF、共享 schedule、对端 TX evidence 放进 RxFrame。
RxFrame 包含有界 MAC bytes、FCS/解码状态和本端 RX 时间/质量；不能直接投递 ToF。

一个 exchange 的 Poll/Response/Final 使用一致的 wire seq；运行时
exchange/attempt 身份与短 wire 字段分开。重试必须先保留原 attempt 失败，
再分配新身份/序号，不覆盖 timestamps。序号耗尽或碰到可能存活的旧帧身份时，
拒绝/要求新 session 或等待有明确定义的复用屏障；测试回绕不等于允许立即重用。
仅更换本端 generation 不能使同样 wire bytes 自动证明来自新 session。

### 4.2 定时计划与证据收敛

顺序：有效 RX → PrepareTx → 本端 adapter 的量化计划 → 填写 t3B/t5A
和 codec 编码 → deadline 检查 → SubmitTx → accepted → outcome 收敛。
因果 token 状态表在 G0 固定：core 输出 `PrepareTx(token,intent)`，adapter 回
`TxPlanned(token,plan)`，core 校验后只输出一次 `SubmitTx(token)`，随后才接受
`TxAccepted`/`TxOutcomeResolved`。乱序、重复、未知或过期 token 不能改写计划、
产生第二次 Submit 或推进状态；分别统计或按明确原因终止。
未发送的帧可携带计划的 TX 时刻，这是 delayed TX 构帧必需；
**不能等 TX Completed 才构造将要发送的同一个帧**。
Completed 约束的是测量输入准入与最终结果，不是事先填写计划字段。

计划记录保存 command time、量化时刻、marker offset、校准空口时刻的实际数值
及相互映射，不能只把四个 bool 全置 true。core 核对返回的 plan/token/域/epoch
属于当前动作，frame 中值必须来自同一计划。fake adapter 模拟映射；实际 PHY 留给 M2。
host deadline 时钟与 device ticks 分开，禁止跨 domain 相减。
adapter 在计划事件中给出同一 host deadline 的可提交 verdict/剩余预算；
core 只消费该 verdict 和已绑定 token，不读 host 时钟。verdict 不可行即
`DeadlineMissed`，不输出 Submit。adapter 实际提交前还须重验剩余预算，
编码/队列等待消耗预算后不可行则拒绝提交，不能依赖陈旧的计划 verdict。

`TxAccepted` / BURST_ACK 不能变成 Completed。仿真中的发送完成必须由模拟
adapter 的明确 completion/error-window-closed 事件产生，并说明其模型保证；
不能用“队列目前没错误”作为闭合。收到 RX 后本地 TX outcome 仍可 Unknown：
可以继续合法协议步骤，但成功终态等待相关本地证据收敛。
late/underflow/seq error、取消和无法精确归属的错误使相关未收敛交换失败/降级。
`TxOutcome` 现无 SeqError/ChainBroken 成员；这些错误作为独立 adapter fault
映射到 `ExchangeStatus::TxSeqError/TxChainBroken`，不能 cast 成域外 TxOutcome。
设有限的证据等待 deadline；没有结论也必须终止为明确失败。
闭合后又出现矛盾错误时使 adapter/session 失效并记录契约违例，不能保留已知
被污染的成功测量；正式硬件闭合机制待 M3 绑定版本验证。

### 4.3 帧匹配、生命周期和存储

FCS、parse/version/flags、PAN、src/dst、session、seq、消息类型和状态共同合法
才推进。wrong-peer/重复/旧会话/异序/迟到分别统计；无效帧不能触发应答。
无关噪声/错 peer 不应无条件取消正确在途请求；记录后继续等待合法帧或超时。
可归属当前 exchange 的损坏帧和相关 radio 错误如何终止在事件表中明确写出。

initiator 的 Begin 被接受时计一次 exchange；responder 被动监听本身不创建
虚假 exchange，合法 Poll 被接纳时才计一次。另有显式监听请求时，其生命周期
与 exchange 分开。SS responder 的本端发送成功不证明 initiator 收到 Response；
DS initiator 的本端 Final 成功不证明 responder 收到 Final。

每端、每 attempt：`accepted_exchanges = terminal_results + in_flight`。
请求进入前预留终态容量；满时明确拒绝。已经接受的请求不能因结果队列满而
静默丢失终态。控制/取消/错误路径不得被普通数据队列占满后饿死。
stop/cancel/reset 作废 RX/TX token、排空在途为取消/失败；旧事件不能复活交换。

frame、动作、队列使用有界存储，容量在 setup 校验。core 消费事件不申请
新的动态队列/帧存储；复用已有 string/domain/calibration 类型时要实测分配次数。
既有数学/准入路径如仍有分配，报告位置和上界，不因“纯 C++”就宣称零分配。
不在本阶段重构全部既有契约来追求该宣称。

## 5. OpenCode 多 subagent 分工

协调者负责接口和集成，不让多个 agent 同时写同一个 core/CMake 文件。
首次派发给每个 agent 的任务必须包含：

> 你不是独自在代码库工作。只修改分配给你的文件，不回退其他人的修改；
> 发现依赖接口需要变化时向协调者提出，依据已交付变更适配。
> 阅读本任务单、AGENTS.md 和本任务对应需求/参考；先写失败 QA，再实现。
> 回报改动文件、命令、退出码、验证边界和未决项，不以“测试绿”替代需求论证。

| Agent | 文件所有权 | 任务与交付门槛 |
|---|---|---|
| 协调者 `m1b_integrator` | `uwb_twr_core.h` 的公共声明；全部 CMake；安装脚本/既有消费者；AGENTS/状态/最终开发报告 | 完成 G0 接口与状态表，独占共享文件；负责构建、集成、发布门禁和最终审阅，不写第二份 FSM |
| A `m1b_protocol_time` | 新 `uwb_twr_protocol_time.h`、`qa_uwb_twr_protocol_time.cc`；必要的 `uwb_twr_math.h` 最小内核提取；`docs/twr/M1-B_协议时间输入设计.md` | 解决 §3 的 wire claim/严格输入边界；补拒绝/数值 QA；M1-A API/oracle 继续通过，不能放宽 admission |
| B `m1b_endpoint_core` | 新 `uwb_twr_core.cc` | 唯一 FSM 实现者；先 SS 双角色，再 DS 双角色，再所有失败与生命周期。公共声明变化请求协调者修改，SS/DS 不由两个 agent 同时改此文件 |
| C `m1b_fake_link` | 新 `uwb_twr_fake_link.h/.cc`、`qa_uwb_twr_fake_link.cc` | 独立 fake transport 与每端 adapter，有限容量、确定性时钟/传播/量化/错误；真值接口与端点 API 隔离 |
| D `m1b_protocol_qa` | 新 `qa_uwb_twr_core.cc`、`qa_uwb_twr_m1b_e2e.cc`、`twr_m1b_test_support.h`；`testdata/twr/m1b/` | 先写需求导出的事件/byte golden、失败 QA 和预期；独立测试两个 core + link，检查 §7 每个 ID；不能替 B 修 FSM 或用 core 输出生成期望 |
| E `m1b_demo_verifier` | 新 `apps/twr_fake_demo.cc`、`apps/twr_fake_demo.py`、`apps/test_twr_fake_demo.py`、`tools/twr/verify_m1_b_protocol.py` | Python→JSON→C++ demo；独立 verifier，精确整数输出及错配置测试；不新增 Python FSM、schema 镜像或 pybind |
| F `m1b_reviewer` | 生产/QA/CMake 只读；仅可写 `docs/twr/M1-B_评审报告.md` | 独立核查 provenance、端点隔离、source address、TX causal order、唯一终态、容量/序号复用、数值与安装；提出可复现反例，不能改测试放宽门槛 |

上述路径除文档、工具和 testdata 外均相对 `gr-uwb/`；头在
`include/gnuradio/uwb/`，源与 C++ QA 在 `lib/`。
fake link 是测试/示例支持，不默认为公共安装 ABI。
核心与协议时间公开头是否安装由 G0 明确；公开的头/库必须进入消费者验证。

### 执行批次与依赖

1. **P0：协调者 + A + D + F**。完成接口声明、事件/动作/结果 schema、
   wire binding 设计、状态转移表和第一批失败 QA。B/C 只读调研并提出反馈。
   D 先固定输入与期望，A 补 protocol-time 拒绝用例；记录缺实现导致的红灯。
2. **P1：A、C、D 可并行**。A 完成时间适配及唯一数学内核，C 完成 fake link
   及其 QA，D 按已声明接口补核心/e2e 测试。协调者注册目标并串行运行受影响 QA。
   C 可用 mock endpoint 验证 transport，不依赖 B 的 FSM。
3. **P2：B 实现 SS→DS**。A 的输入门槛通过后接入；SS 双角色 QA 先绿，
   再做 DS Final、不等回复时间与生命周期。C/D 根据集成反例修改各自模块，
   F 只读跟审；不要拆成 SS/DS 两人抢写 core。
4. **P3：E + D + F**。接口稳定后实现 demo/verifier、补全独立 e2e 矩阵，
   F 审查最终代码与试验产物；A/B/C 只修各自收到的明确问题。
5. **P4：协调者**。锁定待验收 revision/工作区 hash，串行构建/全量回归/
   隔离安装，完成报告与状态更新。未通过的 matrix ID 不能打勾。

并发实现建议最多 3 个 agent；耗时性能 QA/benchmark 始终串行，不能在后台
编译/多组测试并行时采集吞吐。agent 数量不是必须同时运行的数量。
共享构建目录只由协调者运行 configure/build/ctest；agent 的小探针使用各自
`mktemp -d` 目录。若使用独立 worktree，先确认本仓库权限/dirty 状态，不能覆盖用户文件。
每个功能提交和对应 QA 成对交付；不自动 push/合并，不提交 build/log/大 IQ。

## 6. fake link、示例和独立真值

fake link 不能直接扩展 radar 的 `uwb_fake_burst_backend.h` 调度语义。
后者围绕 `schedule_index` 与先 arm RX 的 echo 事务；TWR 需独立的接收驱动 transport。

两端各有 adapter、counter、epoch、plan/outcome 与 RX window。transport 只投递
bytes 与接收端可观测的 RX 事件。传播模型可计算到达时间，不能将 ground truth
距离/interval/对端 evidence 放到端点对象。相同模拟物理时钟也不意味着共享 FSM。
物理端点 A/B 与协议 initiator/responder A/B 的命名要分开，角色互换时重新绑定
公式 domain，不能沿用“物理 A 一定是 initiator”的假设。

fake 的 FCS/解码边界要明确：core 接收已验 FCS 的 MAC bytes 和显式检查结果；
模拟坏 FCS/畸形 bytes，不把 codec 等同 CRC/PHY。若 fake 计算 CRC，只放测试适配层，
沿用独立 byte golden 验证，不能改 core 使 FCS 成为两层追加。

Python demo 使用现有 `twr_config.py`，将原 requested 配置及仿真参数提交给 C++。
C++ 用现有 parser/validator 重验并创建不可变执行快照；Python effective 仅供对照，
不作为 C++ 执行权威。scenario 的传播真值只能流向 fake transport，不能流向 core plan。
仿真模式须显式选择，不能绕过正式 capability gate 并宣称硬件测距 profile 已获许可。
不在两个配置模块里继续增加平行业务规则；仿真包装 envelope 独立版本化。
G0 固定 `SimulationEnvelope`：原 TwrConfig 仍经现有 parser/validator 校验，
加上仅用于 simulation 的双端绑定/容量/时钟/场景约束；这些额外检查只在 C++
冷路径集中实现。现有 config 校验不等于 `allows(...,Ranging)` 的用途许可；
demo 显式走仿真用途，不能通过修改 capability row/evidence、删除拒绝项或自动
填充正式校准取得硬件测距许可。未来正式 radio 入口仍须独立检查 Ranging gate。

CLI 可采用 `--request <json> --output <json>`；Python 只调用该 CLI 与消费输出。
stdout/stderr 与机器 JSON 分开；整数超过 2^53 用十进制字符串，保留 signed ToF
num/den、单位/domain、ratio、状态、每端结果/trace、计数、配置/profile/输入 hash。
输出始终保留 simulation/wire-claim 标注；不以 double/ns 显示投影作为数值验收。

golden 和 oracle：

- 保存 `testdata/twr/m1b/{README.md,event_cases.json,expected_results.json}`。
  README 列每例的需求 ID、手算/物理模型依据、seed、单位、量化和预期因果顺序。
- whole-tick 成功例尽量复用 M1-A MATLAB oracle 的物理参数并真正走帧；
  新链路量化例由 Python Fraction/手算独立推导实际 wire 值及 ToF。
- 既有 12 条 oracle 中不适合 whole-tick wire 的向量仍保留为数学回归，不能
  声称 12 条都已通过 v1 三帧 transport。另测这些场景在协议适配层的显式拒绝/量化。
- 新增 MATLAB 模型如要求 MATLAB 验证，必须实跑再声称通过；本任务不重新生成
  或覆盖现有 MATLAB golden，也不把 Python 参考叫作新 MATLAB 验证。
- `verify_m1_b_protocol.py` 从输入场景、实际帧字段和独立事件约束检查结果；
  不能调用被测 FSM/公式来生成 expected，不将两个同源 helper 的相等当成真值。

## 7. 必须逐项验收的矩阵

每个 ID 在 QA 和报告中可定位；一个 QA 可覆盖多个 ID，但不能只列测试总数。

| ID | 场景 | 必须断言 |
|---|---|---|
| B01 | SS/DS × 物理 A/B 两种角色方向 | 四个单元都实际走两个 FSM；SS 估计在 initiator，DS 在 responder；另一端无复制距离 |
| B02 | SS k>1/k<1、不同 nominal rate；DS 不等回复时间 | 实际帧值独立复算相符，k=fA/fB，DS 公共单位 A ticks；缺/过期 ratio 失败 |
| B03 | 逐个丢 Poll/Response/Final | 缺有效 RX 后相应 Response/Final 不出现；活跃端收敛超时；被动端不凭空生成请求 |
| B04 | 坏 FCS/畸形长度/version/flags/type，错 PAN/src/dst/session/seq | 不推进、不应答；分别记录诊断；随后合法帧按约定仍可推进 |
| B05 | 重复/异序/迟到、旧 attempt、seq 回绕与旧帧身份碰撞 | 不重复 TX/终态，不复用旧 timestamps；有明确定义的身份复用屏障 |
| B06 | 计划缺一项、错 token/域/epoch、量化边界 | deadline/帧字段依据同一量化计划；缺记录拒绝；不可表示的 wire fraction 显式处理 |
| B07 | 仅 accepted/ACK、延迟 Completed、永远 Unknown | 不提前成功；RX 先于本地 outcome 仍正确；Unknown 的证据 deadline 能终止 |
| B08 | TX late/underflow/seq error/取消、归属不明错误 | 显式失败，无即时补发；相应待确认结果污染处理；本端 source 不改标 |
| B09 | 40-bit wrap、半周期±边界、过长 interval | 依现有模差政策及额外 span 上界断言；禁止有损整数 ns 和跨域绝对相减 |
| B10 | 本端 reset、可观测 session generation 变化、旧事件晚到 | 在途取消/失败；新会话不混用旧时间；无隐藏 peer epoch 接口 |
| B11 | 首径质量未记录/不合格、缺失/过期校准、corrections 未应用 | 本端严格门拒绝；wire 声明不能补齐这些本端事实 |
| B12 | 远端 claim 缺绑定/错单位/超宽/伪造来源 | 默认拒绝；不能转换为严格 admitted input 或 HardwareMeasured；protocol complete + estimate 的所有 range-valid helper 为 false |
| B13 | 队列/动作/结果容量边界、慢消费者 | 已接受请求终态不丢；新请求可拒绝；控制事件不饿死；内存/积压有上界 |
| B14 | stop/cancel/restart，terminal 后重复事件 | 无旧 RX/TX token 残留；每端每 attempt 终态恰好一次，守恒成立 |
| B15 | 负 ToF/零分母/溢出、enum 域外与默认缺席字段 | 精确数值和明确原因，signed 原值保留；先域检查，入口不抛异常 |
| B16 | 两端隔离审计 | endpoint public API 无 ground truth/private peer evidence；删除帧的因果测试有效 |
| B17 | 单独编译/运行、安装消费、Python demo 与 verifier | 新纯核心不需 GR/UHD；安装后实际调用一个 SS 和 DS 协议例；demo 拒错配置，protocol status 不可隐式转成测距成功状态 |
| B18 | 历史回归、稳定复现、资源观测 | 旧 M0.1/M1-A 通过；同 seed 结果稳定；报告分配/容量和历史吞吐，未新增失败 |

不匹配帧的诊断计数、请求拒绝、exchange 终态是不同对象，不为每个噪声帧制造
一条 measurement terminal。observer 可报告两个端点的总体成功/失败，但不能
把 observer 看到的远端完成事件倒灌到端点；端点本地成功不等于链路整体成功。

## 8. 构建、验证和安装门禁

协调者设置独立的纯 C++ core 目标和仿真支持目标（或自包含头实现），
core/demo 新目标不链接 `gnuradio-uwb` 来取得 GR/UHD 依赖。
新纯 QA 独立注册，只链接所需 core/测试框架；不要盲目加入带全局 GR 链接依赖的
旧 `test_uwb_sources` 列表后声称 UHD-free。既有工程仍可依赖 GR，
必须另用没有 GR/UHD include/lib/package 的小消费者编译运行证明核心独立。
安装消费程序只 include `uwb_twr_core.h` / `uwb_twr_protocol_time.h` 及其公开
依赖，用 `c++ -I<prefix>/include` 和所需 standalone core 库编译；如采用
自包含头则不需要该库。检查 ldd/readelf 没有 gnuradio/uhd 依赖。
fake link 不安装，只作为本工程 QA/demo 支持；公开安装消费者可用手写事件驱动例子。

CMake/安装修改由协调者独占，延续测试库路径固定、testdata 宏及临时输出策略。
安装验证消费最终 revision 的新前缀，不能指向旧前缀后说新交付通过。
`UWB_TWR_INSTALL_PREFIX` 未配置时 SKIP 不算安装验收。

运行示例（新 test/target 名称在 G0 明确后记录实际名称，不照抄不存在的命令）：

```bash
# 在仓库根目录，由协调者串行执行
cmake -S gr-uwb -B gr-uwb/build
cmake --build gr-uwb/build -j 8
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -R 'twr' -j 1 --output-on-failure
python3 gr-uwb/apps/test_twr_config.py
python3 tools/twr/verify_m0_1_findings.py
python3 tools/twr/verify_m1_a_tof.py
python3 tools/twr/verify_m1_b_protocol.py
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j 1 --output-on-failure
```

demo Python QA、standalone consumer 和隔离安装消费者必须注册/实跑。
独立验证脚本如涉及构建二进制，记录其实际绝对路径和 revision，不导入旧安装模块。
手工二进制用 `env -u LD_LIBRARY_PATH`，需要库的目标用 ldd 证实 build/lib；
纯核心目标还要检查没有 GR/UHD 动态依赖。
使用 `mktemp -d` 为安装验证创建专用前缀，沿用
`tools/twr/verify_install_consumer.sh` 并补新 API/独立目标的实际消费。
不要 sudo install，不修改系统安装，不触碰用户未跟踪 IQ/日志/实验文件。

全量测试中的任何新失败均独立复现并解决，不能归为历史吞吐。
已记录的并发池/计时测试可能波动；保留首次失败、单独复跑与判断依据，
不能重复跑到全绿后删除失败记录。每个有意义的代码改动构建并跑受影响 QA；
最终完整回归只在明确新改动/失败后需要重跑，避免无依据重复 benchmark。

## 9. 最终交付与完成判定

协调者提交 `docs/twr/M1-B_协议核心开发报告.md`，写明：

- 最终 revision/工作区 hash、构建环境、所有 agent 文件与任务完成状态。
- 状态转移表、接口与 §3 协议声明边界、参数/单位/量化和支持/拒绝集合。
- B01–B18 的 QA/输入/期望依据、命令/退出码/结果，不以测试总数代替矩阵。
- 每端 trace、所有成功/失败/拒绝计数、守恒、seed、oracle/输入/结果 hash。
- 本地 TX 证据收敛、远端证据限制、不可观测 reset 限制、序号复用策略。
- standalone 编译与新前缀安装消费者、Python demo、独立 verifier 的实际输出。
- 分配/复制/队列容量观测，历史失败和所有仍未验证项目。

全部完成才能写“M1-B 离线协议核心与双端仿真完成”：

- [ ] G0 输入/结果边界明确，reviewer 未决的阻塞反例已关闭。
- [ ] 纯 C++ 两端 SS/DS 四个方向单元通过，实际接收驱动。
- [ ] wire claim 与本端严格输入在类型和输出语义上分开；无伪造证据/后门。
- [ ] 唯一数学内核，现有 M1-A API/14 例 QA/12 oracle 和独立验证继续通过。
- [ ] B01–B18 全部有实际证据；每端 accepted/terminal/in-flight 守恒。
- [ ] deadline/Unknown/late/underflow/取消/overflow/容量均能有限时间收敛。
- [ ] Python 配置→C++ 双端→结果/verifier 可复跑；没有 Python FSM/大迁移。
- [ ] 核心独立消费、最终安装交付通过；无新增回归，历史门槛未改。
- [ ] 文档写清 simulation/protocol estimate 不等于正式测距；PHY/ToA/硬件未升级。
- [ ] 同步 AGENTS/开发状态/总路线，并附 F 的只读审查结论。

未满足时只报告已完成子项和明确未决；不能将数学可算、fake 成功或输出距离
写为 X410 SS/DS 可用。M1-B 完成后下一步是 M2-A native PHY 闭环，再是
M2-B RMARKER/首径/时映射/实际 timestamp patch，随后 M3/M4 双端硬件。
本任务单没有授权 OpenCode 自动进入这些阶段。

## 10. 给 OpenCode 协调者的启动指令

用户将此文件交给你执行后，先复核基线与 dirty worktree，再按 §5 派发任务。
先做 P0/G0，给每个 agent 独占文件与依赖；在接口成立和 QA 已给出失败预期后
才启动依赖的实现。不要让多个 subagent 各自重建时间模型、公式或 FSM。
完成本任务单的授权范围内工作、构建、验收和报告后结束，不自动 push 或进入 M2。
