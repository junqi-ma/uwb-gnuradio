# M1-B 协议时间输入设计：wire claim 与严格测量输入的分界

日期：2026-09-30。作者：subagent A `m1b_protocol_time`。
基线：`feature/uwb-ds-twr`，HEAD `2e35c38`。
关联需求与任务单：`docs/twr/需求_UWB_SS_DS_TWR.md`、
`docs/twr/OpenCode开发指示_M1-B_协议核心与双端仿真.md` §3/§5/§7、
`docs/twr/M1-B_G0接口与状态表.md`。
运行时代码：`gr-uwb/include/gnuradio/uwb/uwb_twr_protocol_time.h`；
QA：`gr-uwb/lib/qa_uwb_twr_protocol_time.cc`。

本文件是 G0 门槛要求的“wire 声明路径”设计记录。**它不新增协议、不改严格
准入规则、不声称任何硬件测距能力**，只说明 v1 三帧携带的 40-bit 整数 tick
如何被安全地、显式标注地转换成一个**协议估计**，以及为什么它**不能**变成
M1-A 的严格测量输入。

## 1. 问题与边界（为什么需要第二条路径）

v1 codec 的帧只携带 40-bit 小端**整数** ticks（Poll/Response/Final 的 MAC
长度 14/24/29 B，FCS 由 HRP 层追加为 16/26/31 B）：

- Response 携带 `t2B/t3B`；Final 携带 `t1A/t4A/t5A`；`t6B` **不上空口**。
- 帧中**没有**远端 epoch、tick rate、字段单位、首径质量、校准应用记录、
  发送完成证据或子 tick 分数。

而 `compute_ss_tof()` / `compute_ds_tof()`（M1-A）只接受
`AdmittedRangingInterval`，后者只能由 M0.1 的 `admit_ranging_interval()` 在
两个满足“marker + HardwareMeasured/ScheduledCalibrated(with evidence) +
修正链 + 当前校准 + 首径判决”的 `Timestamp` 上产生。

因此一个从空口读到的整数**不能**被悄悄升级为 `HardwareMeasured`、已校准或
TX Completed。M1-B 新增的是一条**显式更弱**的路径，其输入是会话声明加匹
配帧字段，其输出带 `wire_claim` provenance，且其“是否是有效测距”的入口全
部为 false。

## 2. 两条路径的对照

| | 严格路径（M0.1/M1-A） | 协议声明路径（M1-B） |
|---|---|---|
| 输入契约 | `AdmittedRangingInterval`（只由门产生） | `ProtocolInterval`（`from_local` 或 `make_peer`） |
| 远端时刻 | 不允许（只有本端 `Timestamp`） | `PeerTimestampClaim`（原始 wire ticks + 消息身份） |
| 判定依据 | marker / source / 修正 / 校准 / 首径**运行时证据** | `WireTimestampBinding` 的**会话约定** |
| 消费者 | `compute_ss_tof` / `compute_ds_tof` | `compute_protocol_ss_tof` / `compute_protocol_ds_tof` |
| 结果类型 | `TofResult` | `ProtocolTofEstimate` |
| 成功语义 | `ok == true`，可派生距离 | `completion == Complete` + `estimate_available`，`measurement_valid == false` |
| 能否隐式互转 | — | **不能**（类型级，见 §5） |

关键点：两条路径**共用同一个数值内核**（§6），但各自做**自己的输入检查**。
协议路径比严格路径弱的地方仅在于“对端事实只能声明、不能证明”；数值、域、
epoch、有效期检查一条不少。

## 3. 类型目录

### 3.1 `WireTimestampBinding`（会话级声明）

一个**约定**，不是任何运行时证据：

- `peer_domain`（name/rate/bits/epoch）：对端 tick 域，与严格门比较的域字段
  同一套定义，便于 `ProtocolInterval` 与 `ClockRatio` 做身份/epoch 比对。
- `session_generation` / `binding_generation`：本端会话世代与声明版本；任一
  为 0 即“未设置”，拒绝。重新声明使旧声明下的 claim 失效。
- `peer_marker`：wire 字段表示的 marker。三个 TWR 帧为 `RmarkerRx`/`RmarkerTx`；
  `AntennaPlane` 只能作为声明约定，不能当作测量瞬间。
- `unit_convention` / `calibration_convention_id`：可审计的约定 ID，空则拒绝
  （未命名的约定不能写进结果引用）。
- `max_interval_ticks`：本会话接受的最大 interval（peer ticks），必须为正且不
  超过回绕域半周期 `P/2`。
- `sequence_modulus`：wire 序号复用模数，只允许 4/16/64/256。

`wire_timestamp_binding_is_well_formed()` 是唯一的判定入口，返回具体 `why`。

### 3.2 `PeerTimestampClaim`（一帧的一个字段）

- `field`：`TimestampField`（T1A..T5A）；`Count` 与域外值都是未知字段。
- `raw_ticks` + 消息身份（`frame_type/session/seq/src` + `message_identity`）。
- `bound`：**只由 `make_peer_claim()` 置真**。手搓的 claim 默认 false，于是
  “忘了说明这个数从哪来”不可能被读成合法 claim。
- `message_identity` 是 `frame_message_identity()` 对帧声明内容的 FNV-1a，
  不经过编码器，因此不能靠改 codec 改变它；它只用于证明两个 claim 来自同一
  条消息。

`make_peer_claim()` 只在帧确实携带该字段时才产生 claim，绝不为不存在的字段
合成 0。

### 3.3 `ProtocolInterval`（闭环变体）

只有两种合法来源：

- `ProtocolInterval::from_local(AdmittedRangingInterval)`：包装一个**已经**
  过了严格门的本端 interval，`origin() == LocalAdmitted`。
- `ProtocolInterval::make_peer(later, earlier, binding, out)`：两个已绑定
  claim + 良构 binding，做模差与上界检查，`origin() == PeerWireClaim`。

它**没有** `Timestamp`，**没有**到 `AdmittedRangingInterval` 的转换算符，也
**没有**绝对 tick 坐标，因此严格门无法从它到达。

`make_peer()` 的判定顺序：binding 良构 → claim 已绑定 → 字段已知 → 同消息 →
raw 在域内 → 模差可形成 → 不超过 `max_interval_ticks`。任一失败**不动 `out`**。

### 3.4 `ProtocolTofEstimate`（协议结果）

- `completion`：`NotComplete / Complete / Failed`（**独立**于 `ExchangeStatus`；
  顶层成功态不使用 `ExchangeStatus::Ok`，因为 `exchange_status_yields_range(Ok)`
  为真）。
- `estimate_available`：只有计算端本端证据收敛且数值内核成功才为真；负 ToF
  时为 false。
- `local_evidence_complete`、`peer_evidence_is_wire_claim`：输入来源标注。
- `measurement_valid` **恒 false**；`execution_mode_is_simulation` **恒 true**。
- `tof`：精确有符号 `num/den`（A ticks），负值保留符号，绝不 clamp。
- `yields_range()` / `is_hardware_measurement()` / `is_validated_measurement()`
  恒 false；无到 `TofResult` 的转换。
- `detail` 记录可读原因；机器读 `math_status` + `failure_reason`。

`make_protocol_complete_without_estimate()` 供无 ToF 归属的一端（SS responder /
DS initiator）使用：`Complete` 但 `estimate_available == false`，不从另一端复制
距离。

## 4. 拒绝词表（append-only schema）

`PeerClaimError`：`Ok`、`BindingNotWellFormed`、`ClaimNotBound`、`UnknownField`、
`NotSameMessage`、`RawOutOfDomain`、`IntervalNotFormable`、`IntervalTooLong`、
`SessionGenerationMismatch`。

触发条件（QA 逐条覆盖）：

| 错误 | 触发 |
|---|---|
| `BindingNotWellFormed` | 空单位/校准约定、`max_interval_ticks == 0`、`max_interval_ticks > P/2`、不支持的 `sequence_modulus`、`session_generation == 0`、`binding_generation == 0`、未知 marker、非法域 |
| `ClaimNotBound` | 任一 claim 的 `bound == false` |
| `UnknownField` | 字段为 `Count` 或域外值 |
| `NotSameMessage` | 两 claim 的 `message_identity`/帧身份不一致 |
| `RawOutOfDomain` | raw ticks 超出声明域的计数器范围（如 40-bit 下 ≥ 2^40） |
| `IntervalNotFormable` | 非回绕域上 later < earlier；或模差超过 `P/2`（歧义） |
| `IntervalTooLong` | 可形成但超过 `max_interval_ticks` |

`TofStatus`（复用 M1-A 词表）在协议路径上的拒绝：域/epoch 不匹配 →
`ClockRatioDomainMismatch` / `ClockRatioEpochMismatch`；ratio 不可用 →
`ClockRatioMissing` / `ClockRatioInvalid`；有效期窗口不覆盖 → `ClockRatioNotValidAtTime`；
无效 interval → `InvalidInput`；负值 → `NegativeTof`。**入口只报告，不抛异常。**

## 5. 类型级隔离（wire 不能伪造成严格测量输入）

QA 用编译期断言证明：

```cpp
static_assert(!std::is_convertible_v<ProtocolInterval, AdmittedRangingInterval>);
static_assert(!std::is_constructible_v<AdmittedRangingInterval, ProtocolInterval>);
static_assert(!std::is_convertible_v<ProtocolTofEstimate, TofResult>);
static_assert(!std::is_constructible_v<TofResult, ProtocolTofEstimate>);
static_assert(!std::is_invocable_v<decltype(&compute_ss_tof),
              const ProtocolInterval&, const ProtocolInterval&, const ClockRatio&>);
```

并断言 `ProtocolTofEstimate::yields_range() == false`、
`measurement_valid == false`、`is_hardware_measurement() == false`、
`is_validated_measurement() == false` 在成功与失败结果上都成立。

## 6. 唯一数值内核的复用

SS/DS 公式只写一次，位于 `uwb_twr_math.h` 的 `detail::ss_tof_kernel()` /
`detail::ds_tof_kernel()`（`Rat128` 精确有符号有理数，乘法/加法溢出检查）。
严格入口与协议入口**各自**做输入检查后调用同一内核；内核只接收已校验的
精确有理数，无法导入宽松规则。协议入口不复制公式，也不给原 `compute_*` 增加
裸 Timestamp/interval 重载或 `skip_validation` 后门。

## 7. 手工验证向量（Python `fractions.Fraction` 独立推导）

物理模型：`RA = 2*tau_A + k*DB`，`k*RB = 2*tau_A + DA`，
`ToF_A = (RA - k*DB)/2 = (RA*k*RB - DA*k*DB)/(RA+k*RB+DA+k*DB) = tau_A`。

取 A = 1.0 GHz、B = 998.4 MHz，`k = 1000000000/998400000 = 625/624`，
`tau_A = 1001/2`；选 whole-tick wire 值：

| 量 | 值 | 单位 |
|---|---|---|
| `k` | 625/624 | A ticks / B tick |
| `RA` | 1626 | A ticks（`= 2*(1001/2) + 625`） |
| `DB` | 624 | B ticks（`k*DB = 625`） |
| `k*RB` | 1250 | A ticks（`RB = 1248` B ticks） |
| `DA` | 249 | A ticks（`= 1250 - 1001`） |
| **SS ToF** | **(1626 - 625)/2 = 1001/2** | A ticks |
| **DS ToF** | **(1626*1250 - 249*625)/(1626+1250+249+625) = 1876875/3750 = 1001/2** | A ticks |

四个 wire interval 全是整数 tick，而 ToF 是精确分数 `1001/2`：协议路径不需要
“分数上得了 wire”就能返回精确分数结果。负值向量：`RA = 1000`、`DB = 1248`
（`k*DB = 1250`）→ `(1000-1250)/2 = -125/1`，保留符号并以 `NegativeTof` 失败。

本地路径与 peer 路径对同一物理场景给出**相同** `num/den`（`1001/2`），QA 用
`tof_rational_equals()` 交叉相乘比较。

## 8. 边界与已知限制（不夸大）

- **恰好半周期**：`raw_tick_delta()` 在 `later/earlier` 已知时接受 `d == P/2`；
  QA 证明 40-bit 域下 `later = 0, earlier = 2^39`（回绕、恰好 `P/2`）被接受，
  而 `d > P/2`（如 `later = 2^39+1, earlier = 0`）以 `IntervalNotFormable` 拒绝。
  排序的半周期歧义策略**不会**被误套到 interval 上。
- **`t6B` 不在空口**：DS 的 `RB = t6B - t3B` 只能是本端严格准入 interval，不能
  伪造成 peer claim。协议 DS 的自然组合是 A 侧 `RA/DA` 为 wire claim、B 侧
  `RB/DB` 为本地 interval。
- **单个 `peer_marker`**：`WireTimestampBinding` 只有一个 marker 字段，而
  `RA = t4A - t1A` 跨 RX/TX 两个 marker；协议 interval 的 marker 只是声明，
  严格路径的 marker-pair 规则**不**在协议路径重演。
- **无对端 epoch**：帧不含 peer epoch，因此数值上完全相同的远端隐蔽 reset 无
  法被检测；任何会话绑定缺席都拒绝，fake transport 不补发隐藏 epoch/校准/完成
  证据。
- **有效期窗口语义**：`protocol_window_ok()` 用 `ProtocolInterval` 的 interval
  **时长**对 A ticks 窗口做检查，因为该类型**不携带绝对 tick 坐标**。这是必要
  非充分条件（绝对位置不可知），属于接口的显式限制；严格路径用绝对 ticks，
  不受影响。
- **`PeerClaimError::SessionGenerationMismatch` 已定义但当前无代码路径产生**
  （claim 上没有可比较的 generation 字段）；同时 `PeerClaimError` 没有
  `is_known()` 域测试，与文件头 N07 的“每个 enum 都有 is_known()”表述不完全
  一致。这是契约/文档缺口，未在本次修改头文件（见 QA 报告“发现的 bug/缺口”）。
- **不是硬件测距**：本路径结果是带 simulation / wire-claim 标注的协议估计，
  不升级任何 capability，不进入 M2。

## 9. QA 覆盖与运行

`qa_uwb_twr_protocol_time.cc`（Boost.Test，10 个用例）覆盖：

1. SS peer 路径给出 `1001/2`，`Complete` + `estimate_available`，range 入口全 false；
2. DS A 侧 wire + B 侧本地路径给出 `1001/2`，含全本地对照；
3. `from_local` 与 peer 路径精确 `num/den` 相等；
4. 类型级不可转换 + `yields_range()/measurement_valid` 恒 false + origin 域测试；
5. binding 拒绝矩阵（响应 `BindingNotWellFormed`，含恰好半周期可接受）；
6. claim 拒绝矩阵（`ClaimNotBound`/`UnknownField`/`NotSameMessage`/
   `RawOutOfDomain`/`IntervalNotFormable`/`IntervalTooLong`）；
7. 恰好半周期接受、超半周期拒绝（回绕与非回绕）；
8. 空/无效输入 → `Failed` 且不抛异常，缺 ratio → `ClockRatioMissing`；
9. 负 ToF 保留 `-125/1`，`NegativeTof`；
10. 协议路径仍执行域/epoch/有效期窗口检查。

本机独立构建与运行（协调者拥有 `gr-uwb/build`，本 QA 不注册进该目录）：

```bash
g++ -std=c++17 -Wall -Wextra -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN \
    -I gr-uwb/include \
    -o /tmp/opencode/qa_pt \
    gr-uwb/lib/qa_uwb_twr_protocol_time.cc -lboost_unit_test_framework
# exit 0
/tmp/opencode/qa_pt
# Running 10 test cases... *** No errors detected   (exit 0)
```

（`BOOST_TEST_DYN_LINK;BOOST_TEST_MAIN` 与工程 `GR_ADD_CPP_TEST` 的编译定义
一致，见 `/usr/local/lib/cmake/gnuradio/GrTest.cmake`。协调者把该 QA 注册进
CTest 时可沿用同样的目标名 `uwb_qa_uwb_twr_protocol_time.cc`。）

## 10. 未决项

- 本 QA 只验证“wire 声明路径与严格路径在类型和数值上分开”；它不验证 FSM 如何
  消费 `PeerTimestampClaim`，也不验证帧匹配、会话/序号复用或发送证据收敛——那
  属于 core / fake-link / e2e 的验收（B01–B14）。
- `protocol_window_ok` 的时长语义（§8）需在 M2 设计正式结果时重新评估；若需要
  绝对 instant 的窗口判定，要在 G0 接口层面显式增加坐标，而不是弱化严格门。
- 真实帧上的 epoch/单位/校准只作为声明记录，未与任何硬件回读对照；不声称互通。
