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
| 本端 TX 计划 | `CoreEvent::planned_tx_time`（RmarkerTx + ScheduledCalibrated）、`tx_evidence` | 计划先保留；`Completed` 后才进严格准入 |
| 远端 wire ticks | `PeerTimestampClaim` + `WireTimestampBinding` | `ProtocolInterval`（`peer_wire_claim`），**不能**变 `AdmittedRangingInterval` |
| 对端单位/marker/校准约定 | `WireTimestampBinding` 的会话声明 | 仅作为约定记录进结果，不是运行时证据 |
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

## 3. 因果 TX token 状态表

```
PrepareTx(token,intent)                    core -> adapter
TxPlanned(token, plan, verdict)            adapter -> core   (量化时刻/命令时刻/
                                                             marker offset/校准空口时刻)
SubmitTx(token, bytes)   恰好一次          core -> adapter
TxAccepted(token)                         adapter -> core   (send() 返回；不是完成)
TxOutcomeResolved(token,o)                adapter -> core
```

规则：乱序/重复/未知/过期 token 不改写计划、不产生第二次 Submit、不推进状态；
分别计入 `tx_token_mismatch` / `contract_violations`。`TxAccepted` 不等于完成；
仿真中完成必须来自显式 completion 事件。未收敛前不发布成功终态；
`evidence_wait_ticks` 到期必须有限时间失败。

## 4. 状态转移表（每端一个 exchange；至多一个在途）

| 角色/协议 | 事件 | 状态 | 动作 |
|---|---|---|---|
| A / SS+DS | `Begin` | Idle→PollSent | ArmRx(Response); PrepareTx(Poll) |
| A | `TxPlanned(Poll)` | PollSent | SubmitTx(Poll) |
| A | `RxFrame(Response, 合法)` | PollSent→ResponseReceived | SS：算 RA−k·DB（本地证据收敛后）；DS：PrepareTx(Final, t1A,t4A,t5A) |
| A / DS | `TxPlanned(Final)` | ResponseReceived→FinalSent | SubmitTx(Final) |
| B / SS+DS | `RxFrame(Poll, 合法)` | Idle→PollReceived | ArmRx(Response 或 Final); PrepareTx(Response, t2B,t3B) |
| B / SS | `TxPlanned(Response)` | PollReceived | SubmitTx(Response)，随后本地证据收敛即完成（无估计） |
| B / DS | `RxFrame(Final, 合法)` | PollReceived→FinalReceived | 算 DS（DB/RB 本地，RA/DA 为 A 声明） |
| 任一端 | `Deadline` 超出预算 | 在途→终态 | TerminalResult(失败原因) |
| 任一端 | `Cancel/Stop/Reset/Overflow` | 在途→终态 | AbortPending + TerminalResult |

守恒：`accepted_exchanges == terminal_results + in_flight`（每端每 attempt）。
`Begin` 在结果队列满时被拒（`QueueFull`），已接受请求的终态不丢。

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
