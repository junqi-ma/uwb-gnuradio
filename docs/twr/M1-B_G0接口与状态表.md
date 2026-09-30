# M1-B G0 接口与状态表（设计记录）

日期：2026-09-30。状态：**G0 接口已冻结，依赖实现进行中**。
基线：`feature/uwb-ds-twr`，HEAD `2cf881b` 之上。

本文件是 M1-B 的 G0 门槛记录：输入/结果边界、事件/动作/结果 schema、状态转移表、
支持与拒绝集合。代码契约以这三个头为准：

- `gr-uwb/include/gnuradio/uwb/uwb_twr_protocol_time.h`（wire claim 边界）
- `gr-uwb/include/gnuradio/uwb/uwb_twr_core.h`（事件/动作/结果/计数器）
- `gr-uwb/include/gnuradio/uwb/uwb_twr_math.h`（唯一的 SS/DS 精确有理数内核）

## 1. 输入边界（指令 §3）

| 信息 | 来源 | 进入哪条路径 |
|---|---|---|
| 本端 RX 时间/质量/校准 | `CoreEvent::rx_time`（RmarkerRx + HardwareMeasured + 修正链 + calibration_id）、`rx_first_path` | 严格 `admit_ranging_interval()` |
| 本端 TX 计划 | `CoreEvent::tx_plan`（`TxPlan`：command time/量化时刻/marker offset/校准空口时刻的**数值**映射，域/单位/校准显式） | 计划先保留（不含完成状态）；`Completed` 后才进严格准入 |
| 远端 wire ticks | `PeerTimestampClaim` + `WireTimestampBinding` | `ProtocolInterval`（`peer_wire_claim`），**不能**变 `AdmittedRangingInterval` |
| 对端单位/marker/校准约定 | `WireTimestampBinding` 的会话声明 | 仅作为约定记录进结果，不是运行时证据 |
| 事件代次 | `CoreEvent::generation` / `CoreAction::generation` | 代次不符的事件只计数、不推进；`Reset` 可声明新 **wire** session |
| 传播延迟/真实距离 | 仅 fake transport 与验收观察者持有 | **禁止**进入 `EndpointCore` 接口 |

`ProtocolTofEstimate` 的 `yields_range()` / `is_hardware_measurement()` /
`is_validated_measurement()` 恒为 `false`，且没有到 `TofResult` 的转换；
`measurement_valid = false`，`execution_mode_is_simulation = true`。
协议完成用 `ProtocolCompletionStatus`，不用 `ExchangeStatus::Ok` 作顶层成功态。

## 2. 事件 → 动作 → 结果

事件：`Begin / RxFrame / TxPlanned / TxAccepted / TxOutcomeResolved / Deadline /
Cancel / Stop / Overflow / Reset`，每个 enum 都有无 `default` 的 `is_known()`，
域检查先于消费。

动作：`ArmRx / PrepareTx / SubmitTx / AbortPending / TerminalResult`，
每次 `post()` 返回定长 `CoreActionBatch`（≤4），核心不为事件申请动态队列。

## 3. 因果 TX token 状态表（本轮复核整改后，R01/R04/R08）

```
PrepareTx(token,intent)                    core -> adapter
TxPlanned(token, intent, TxPlan)           adapter -> core   数值计划（无完成状态）
SubmitTx(token, bytes)   恰好一次          core -> adapter
TxAccepted(token)                         adapter -> core   (send() 返回；不是完成)
TxOutcomeResolved(token, outcome/fault)    adapter -> core   唯一可闭合发送的来源
```

规则（R01/R04/R06）：
- `TxPlanned` **不得**携带 `sent/completed`；它只给数值计划。core 由计划构造
  `TxSendEvidence`（记录计划），`outcome` 初值只能是 `Unknown`，
  **只有** `TxOutcomeResolved` 能把它推进。`TxAccepted`/BURST_ACK 不等于完成。
- token 在会话内单调递增，**`configure`/`reset`/Reset 事件都不清零 token 计数器**，
  旧事件不可能与新 token 值巧合。
- 每个事件带 `generation`；与该端当前会话代次不符的事件只计数、不推进、不生成终态。
- 乱序/重复/未知/过期 token：不重写计划、不二次 Submit、不推进状态。
- 终态出口有防重复/无主守卫（R06）：只有能归属当前已接受在途请求的错误才终止它。

## 3.1 终态容量预留（R05）

initiator `Begin` 与 responder 接纳 Poll **统一**先检查
`results.size() + reserved < result_queue_capacity`；满则在接纳前拒绝并计数，
不增加 `accepted`、不发 `PrepareTx`。接纳即 `reserved++`，终态时 `reserved--` 并写入
结果存储，保证已接受请求不因队列满丢结果（正常输入 `results_dropped == 0`）。

## 3.2 期限（R07）

两个上限**分别保存**：
- `exchange_deadline_ticks`：接纳时确定，**绝不延长**；
- `evidence_deadline_ticks`：待结果但本端 TX 未闭合时由 `evidence_wait_ticks` 确定。

`ArmRx.deadline_ticks` 取二者中**有效且最早**者；`on_deadline` 按最早者判定一次终止。
时间加法用溢出安全的加法，不使用可能溢出的有符号加法。配置须保证已接纳请求有有限
终止条件（某项为 0 关闭时，另一项必须有限）。

证据上限的起算点（实现固定，供 QA/verifier 对齐）：取「本端首次开始欠一个结果」的
事件 tick —— SS initiator 为 Response 的 RX tick；responder 为 Response 的 plan tick；
DS initiator 为 Final 的 plan tick。绝对 exchange 上限始终自接纳时刻起算，二者独立。


## 4. 状态转移表（每端一个 exchange；至多一个在途）

| 角色/协议 | 事件 | 状态 | 动作 |
|---|---|---|---|
| A / SS+DS | `Begin` | Idle→PollSent | ArmRx(Response); PrepareTx(Poll) |
| A | `TxPlanned(Poll)` | PollSent | SubmitTx(Poll) |
| A | `RxFrame(Response, 合法)` | PollSent→ResponseReceived | SS：算 RA−k·DB（本地证据收敛后）；DS：PrepareTx(Final, t1A,t4A,t5A) |
| A / DS | `TxPlanned(Final)` | ResponseReceived→FinalSent | SubmitTx(Final) |
| B / SS+DS | `RxFrame(Poll, 合法)` | Idle→PollReceived | PrepareTx(Response, t2B,t3B)；SS 无后续空口等待，DS 在该计划提交后 ArmRx(Final) |
| B / SS | `TxPlanned(Response)` | PollReceived | SubmitTx(Response)，随后本地证据收敛即完成（无估计） |
| B / DS | `RxFrame(Final, 合法)` | PollReceived→FinalReceived | 算 DS（DB/RB 本地，RA/DA 为 A 声明） |
| 任一端 | `Deadline` 超出证据等待预算 | 待结果→终态 | TerminalResult(ProtocolTimeout)；证据等待预算 `evidence_wait_ticks` 严格于整交换超时 |
| 任一端 | `Deadline` 超出整交换预算 | 在途→终态 | TerminalResult(ProtocolTimeout) |
| 任一端 | `Cancel/Stop/Reset/Overflow` | 在途→终态 | AbortPending + TerminalResult |

守恒：`accepted_exchanges == terminal_results + in_flight`（每端每 attempt）。
`Begin` 在结果队列满时被拒（`QueueFull`），已接受请求的终态不丢。

序号复用屏障（F 评审后补强）：responder 侧用有界 `seq_used[≤256]` 拒绝**已消费**
wire seq 的重放（计 `stale_events`）；initiator 侧用 `seq_issued == sequence_modulus`
拒绝新 Begin。`Reset` **事件**会清 `seq_issued` 但**不清** `seq_used` —— 同样的 wire
bytes 不能因为本端 generation 变了就自动被当作新 session（M1-B §4.1），
这个方向是 fail-closed，属于有意的不对称。

`reset()` **方法**：在途时拒绝（须先 post `Reset`/`Stop` 事件以保留终态）；
要求传入新 generation 的 peer binding；**不清计数器**；**保留**未取走的终态结果。

结果与终态的 fail-closed 类型（F 评审 B-1/N-10）：`CoreAction::status` 为
`ProtocolTerminalStatus`，`ProtocolTofEstimate` 的失败原因只经「仅失败时成功」的
`failure_reason(out)` 访问器暴露；两者都**没有** `ExchangeStatus` 数据成员，
也没有到 `ExchangeStatus` 的转换，`yields_range()` 恒 false。

## 5. 支持 / 拒绝集合（M1-B）

支持：SS 双角色；DS 双角色；whole-tick wire；同物理时钟（`unity_same_clock`）与
独立时钟的 nominal rate ratio；序号模数 4/16/64/256；wrap 模差（含恰好半周期）。

拒绝（显式，不静默）：wire 分数；超宽 ticks；`local_domain.tick_rate_hz` 与
`frame_profile.timestamp_unit_hz` 不等的有损本地→wire 单位换算；缺绑定/会话 generation
不匹配；`src_addr != configured peer`；未知 enum 域；本地首径/校准/发送证据未收敛；
未定义身份复用屏障的旧帧身份。

## 6. 明确不在 M1-B

GNU Radio block/controller、PMT、UHD radio adapter、真实收发、两 RX 路由、
native 重采样、首径估计、实际波形 timestamp patch、CFO/SFO 估计器、pybind
大迁移、Report、STS、商用 SDK adapter。M1-B 结果是带 simulation/protocol-estimate
标注的协议估计，**不是**有效硬件测距；不进入 M2。
