# M1-B 独立评审报告（subagent F · m1b_reviewer）

日期：2026-09-30。角色：**只读评审**（生产/QA/CMake 只读，本文件是唯一写出的文件）。
基线：`feature/uwb-ds-twr`，`HEAD 12df6f8`（`docs(twr): record M1-B completion ...`）。
被审 M1-B 代码在 `309f7bf` 已全部提交，工作区对下列文件无未提交改动
（`git status --short -- gr-uwb/include/.../uwb_twr_{core,protocol_time,fake_link}.h gr-uwb/lib gr-uwb/apps tools/twr` 为空）。

评审依据：`docs/twr/OpenCode开发指示_M1-B_协议核心与双端仿真.md`（尤其 §3、§4、§7 B01–B18、§9）、
`docs/twr/M1-B_G0接口与状态表.md`、根 `AGENTS.md`（N07 / fail-closed 教训）。

方法：通读全部被审文件；在 `/tmp/opencode/m1b_review/` 编写并运行独立探针
（`g++ -std=c++17 -I gr-uwb/include ... gr-uwb/lib/uwb_twr_core.cc`），只用 `gr-uwb/build` 读产物。
未改任何仓库文件。

## 结论速览

| # | 主题 | 判定 |
|---|---|---|
| 1 | provenance / wire claim 不得升级为严格测量输入 | 类型层 **CONFIRMED**；`CoreAction::status` 层 **COUNTEREXAMPLE（阻塞）** |
| 2 | 端点隔离（无真值/传播/对端私有 TX 证据） | **CONFIRMED** |
| 3 | source address（`src_addr == configured peer`） | **CONFIRMED** |
| 4 | TX 因果顺序（每 token 恰一次 Submit；Accepted≠完成） | **CONFIRMED** |
| 5 | 每 exchange 恰一终态 + 守恒 | 事件路径 **CONFIRMED**；`EndpointCore::reset()` 方法 **COUNTEREXAMPLE（阻塞）** |
| 6 | 容量与序号复用屏障 | 容量/Begin 拒绝/initiator 模数 **CONFIRMED**；responder 侧复用 **COUNTEREXAMPLE** |
| 7 | 数值正确性（共享内核、严格语义、无损、无跨域相减） | **CONFIRMED** |
| 8 | enum 域（N07）逐点 | **CONFIRMED（1 处 deviation + 1 处未域检查）** |
| 9 | 安装/独立消费 | **CONFIRMED** |

---

## 1. provenance：远端 wire claim 不能成为严格/测量输入

**声明**：`PeerTimestampClaim` 永远不能升级为 `AdmittedRangingInterval`/`TofResult`/硬件测量；
`ProtocolTofEstimate` 的任何 range/measurement 入口都不得为真，包括 `exchange_status_yields_range` 误用、转换与重载。

**检查**：
- 逐路径阅读 `uwb_twr_protocol_time.h`：`ProtocolInterval` 只有 `from_local(AdmittedRangingInterval)` 与
  `make_peer(...)` 两个入口；`AdmittedRangingInterval` 的构造函数私有、仅 `admit_ranging_interval` 为 friend
  （`uwb_twr_tof_input.h:736-802`），`from_local` 不可能收到伪造值。文件内无 `operator TofResult/AdmittedRangingInterval`。
- `grep -rn "reinterpret_cast|static_cast<TofResult|static_cast<AdmittedRangingInterval|static_cast<ProtocolTofEstimate" gr-uwb/lib`：M1-B 路径无命中（其余命中是别的块的 `f.read`）。
- `qa_uwb_twr_protocol_time.cc` 的 `static_assert(!is_convertible_v<...>)`、`!is_invocable_v<compute_ss_tof, ProtocolInterval...>` 覆盖类型层。
- core 里远端 ticks 只进 `PeerTimestampClaim`，本地 `rx_time/planned_tx_time` 只来自 `CoreEvent`（适配器/本端），
  wire 值从不生成 `Timestamp`/`HardwareMeasured`。

**观察到的问题（COUNTEREXAMPLE）**：成功协议结果所伴随的 `CoreAction::status` 是 `ExchangeStatus::Ok`，
而现有 `exchange_status_yields_range(ExchangeStatus::Ok) == true`。指示 §3.2 明确禁止
“成功不伪造一个可传给 range helper 的状态”。`ProtocolTofEstimate` 本身安全，但动作级 status 泄漏。

复现（`/tmp/opencode/m1b_review/probe1.cc`，驱动 SS initiator 到成功估计后取终态动作）：

```
D protocol terminal: action.status=ok exchange_status_yields_range(status)=1 \
  result.yields_range()=0 completion=complete estimate=1 tof=50/1
```

最小复现：

```cpp
const CoreAction* tr = find(batch, CoreActionKind::TerminalResult); // SS 成功
assert(!tr->result.yields_range());                    // 通过（结果类型安全）
assert(exchange_status_yields_range(tr->status));      // 也通过 -> 泄漏
```

`gr-uwb/apps/twr_fake_demo.cc:1327` 还把该值输出为 `terminal_status:"ok"`；`twr_fake_demo.cc:1352-1354`
自己打印的 `yields_range=false` 是**硬编码**，与 `status` 是两套数字。**判定：阻塞**（见“阻塞结论”B-1）。

## 2. 端点隔离

**声明**：`EndpointCore` 公共 API 不携带真值距离、传播、对端私有 TX 证据或真 ToF；`CoreEvent`/`CoreConfig` 穷举检查。

**检查**：逐字段读完 `CoreEvent`（core.h:256-299）、`CoreConfig`（core.h:443-486）、`FakeRx`（fake_link.h:212-224）：
- `CoreEvent`：`kind,event_id,exchange,fcs_passed,decode_ok,frame,rx_time,rx_first_path,token,tx_intent,planned_tx_time,tx_evidence,deadline_verdict_feasible,deadline_slack_ticks,tx_outcome,adapter_fault,now_ticks,new_generation,cancel_reason`，无 distance/propagation/truth/peer-private。
- `CoreConfig`：`endpoint_id,protocol,role,pan_id,local_address,peer_address,session_id,session_generation,sequence_modulus,initial_sequence,local_domain,peer_binding,ratio,frame_profile,local_calibration,reply_deadline_ticks,exchange_timeout_ticks,evidence_wait_ticks,result_queue_capacity,max_in_flight`，无真值。
- `FakeRx`：`source,destination,token,bytes,nbytes,rx_marker_ticks,fcs_passed,decode_ok,first_path,injected_drop,note`。`rx_marker_ticks` 是接收端可观测的到达时刻（真接收机也看得到），不是 ground truth 距离；`distance_m` 只在 `FakeLinkConfig`，仅被 `compute_arrival()` 读取。

**判定：CONFIRMED。** 备注（非阻塞）：QA 的结构检测只按名字查 `distance_m/tof/truth`，覆盖不完整；本次人工穷举未发现泄漏。

## 3. source address

**声明**：FSM 除 `frame_match`（不校验 src）外，确实显式校验 `src_addr == configured peer`。

**检查**：`frame_match`（`uwb_twr_frame.h:1667-1685`）只查 type/PAN/dst==local/src==local(self)/session/seq，**不查 src==peer**；
`EndpointCore::Impl::validate_frame`（core.cc:289-298）显式 `if (f.src_addr != cfg.peer_address)` 计 `frames_rejected_wrong_peer`，
且在 `on_rx_frame` 的 FCS/decode 之后、角色分支之前对两端统一执行。FSM 未调用 `frame_match`（其诊断由自身实现）。

**观察**：`probe3.cc` 对 responder 投递 src=0xDEAD 的 Poll：

```
H wrong-src frame: actions=0 wrong_peer=1 accepted=0
H source-address check: CONFIRMED
```

**判定：CONFIRMED。**

## 4. TX 因果顺序

**声明**：每 token 只一次 Submit；乱序/重复/未知/过期 token 不改写计划、不产生第二次 Submit、不推进状态；`TxAccepted`/ACK 永不视为完成。

**检查**：`on_tx_planned` 先校验 `pending_prepare && token==pending_token && tx_intent==pending_intent`，成功后置
`pending_prepare=false`；`on_tx_accepted` 只置 `accepted=true`；`on_tx_outcome` 只更新 outcome 并在 `Completed` 时试终态。
`probe2.cc`：

```
F submit#1=1
F submit#2 (duplicate plan) =0 tx_token_mismatch=1
F after TxAccepted: terminals=0 in_flight=1
F causal-order claim: CONFIRMED
F stale plan after terminal: actions=0 submit=0
F stale-token: CONFIRMED
```

另：错 intent 的正确 token 计划 → Abort+终态、不 Submit（`probe4.cc`）：

```
J wrong-intent plan: submit=0 abort=1 terminal=1 status=internal_error
```

4000×40 随机事件 fuzz 中“每 token 的 Submit 次数 >1”=0。**判定：CONFIRMED。**

## 5. 每 exchange 恰一终态 + 守恒 `accepted == terminal + in_flight`

**检查**：`terminal()` 是唯一 `terminal_results++` 与 `clear_exchange()` 处；`fail()` 经它收敛。
`probe2.cc` 的随机 fuzz（4 值模数 × SS/DS × 两角色 × Begin/RxFrame/TxPlanned/TxAccepted/TxOutcome/Deadline/Cancel/Stop/Overflow/Reset）：

```
F fuzz: submit-per-token violations=0 terminal-count mismatches=0
CONSERVATION FAIL 从未打印
```

覆盖 timeout / cancel / stop / overflow / adapter fault / queue-full / **Reset 事件** 均守恒。

**COUNTEREXAMPLE（`EndpointCore::reset()` 公共方法）**：`reset(cfg,gen)` 内部直接调用 `configure()`，
后者执行 `clear_exchange()`、`counters=CoreCounters{}`、`results.clear()`，**不产生终态**。`probe1.cc`：

```
A after begin: accepted=1 in_flight=1 terminals=0
A reset() while in-flight: ret=1 in_flight=0 accepted=0 terminals=0 results_dropped=0
A conservation: accepted(0) == terminals(0)+in_flight(0) ? yes
A reset() dropped a terminal? pop_result=0
```

即：在途 exchange 的终态被静默丢弃，守恒只因计数器被清零而“成立”。头注释
（core.h:506-510）说明“emitting terminal failures ... is the caller's job via stop() first”，
但指示 §4.3 要求 `reset` 作废在途并排空为取消/失败。注意 **`CoreEventKind::Reset` 事件本身是正确的**
（emit_abort + `fail(StaleSession)`，见 `b10_local_reset_*`）。

**判定：事件路径 CONFIRMED；`reset()` 方法 COUNTEREXAMPLE（阻塞，B-2）。**

## 6. 容量与序号复用屏障

**声明**：结果存储有界，满时拒绝新 Begin；序号模数复用屏障（不静默复用可能存活的旧身份）。

**检查**：
- 结果存储：`terminal()` 超过 `result_queue_capacity` 计 `results_dropped` 且不丢已接受终态；`on_begin` 在
  `results.size() >= capacity` 时 `requests_rejected_queue_full++` 并拒绝。`probe2.cc`：
  `queue-full: new_begin_actions=0 rejected_full=1 pop1=1 pop2=0 → CONFIRMED`。
- initiator 模数屏障：`on_begin` 在 `seq_issued >= sequence_modulus` 时拒绝（计 `stale_events`）；Reset 清零
  `seq_issued`。QA `b05_*` 覆盖 4/16/64/256。

**COUNTEREXAMPLE（responder 侧无屏障）**：responder 在无在途时无条件把任何合法 Poll（seq < modulus）接纳为新
exchange，不记录“已服务过的 (session,seq)”。`probe1.cc` 把同一 `seq=0` 的 Poll 投递 3 次：

```
B replay #0: accepted=1 terminals=1
B replay #1: accepted=2 terminals=2
B replay #2: accepted=3 terminals=3
```

B05 要求“有明确定义的身份复用屏障”。initiator 有，responder 没有；一个被重放的旧 Poll 会被当成新 exchange
（`accepted_exchanges` 递增），没有 `duplicate_frames`/拒绝。考虑到 v1 wire 无 generation、responder 无法区分
“重传”与“新交换”，这是**协议层限制**，但至少应记录/计一个重复诊断或明确写入契约。**判定：非阻塞观察（N-1）**，
但属于对声明强形式的反例。

**另（关联阻塞项 B-3）**：`WireTimestampBinding::session_generation` 的文档说“A claim formed under a different
generation is refused”，但运行期无任何代码比较它（只在 `configure()` 比较一次，
core.cc:1058-1060；`PeerClaimError::SessionGenerationMismatch` 注释自认 M1-B 永不返回）。
`probe4.cc` 在 `Reset` 事件后：

```
I after Reset: cfg.session_generation=2 peer_binding.session_generation=1 (mismatch=1) session_id=7
I second exchange after Reset completes=1 (accepted=2 terminals=2)
```

即 Reset 后 binding 已与 session 不一致，但 wire claim 仍被接受并成交。这正对应 AGENTS 的 N01 教训
（语料 note 描述了代码里不存在的机制）。

## 7. 数值正确性

**声明**：SS/DS 协议路径复用 M1-A 同一共享内核、不复制公式；严格路径语义保持；无有损整数 ns、无跨域绝对相减。

**检查**：
- 公式只在 `detail::ss_tof_kernel` / `detail::ds_tof_kernel` 出现一次（math.h:809/827）；
  严格入口（math.h:1016、1109）与协议入口（protocol_time.h:785、857）都调用它。
- `git diff 2cf881b..HEAD -- uwb_twr_math.h`：严格入口的改动只是改调内核；除 `ZeroDenominator` 的 detail 文案
  由“含分母值”改为通用文案外无语义变化（status/数值不变）。
- 独立探针 `probe3.cc` 对同一组时间值做严格 vs 协议逐例对比（含非整 2900/17）：

```
G case t=[1000,1100,1300,1500,1800,2000]: SS strict=150/1 proto=150/1 same=1 ; DS strict=2900/17 proto=2900/17 same=1
G strict==protocol kernel: CONFIRMED
```

- 历史回归实跑：`gr-uwb/build/lib/uwb_qa_uwb_twr_math.cc` → `14 test cases ... No errors detected`；
  `python3 tools/twr/verify_m1_a_tof.py` → `4/4 checks pass`。
- 分数本地 TX 被 `local_ts_to_wire` 显式拒绝（不静默截断）：

```
G fractional planned TX: submit=0 terminal=1 status=invalid_time_domain
  detail=... local timestamp carries a sub-tick fraction; the v1 wire holds whole ticks only
```

- 跨域：core 里所有 `admit_local` 与 `build_frame` 的减法是同 domain 值；wire 用 `raw_tick_delta` 的 tick-space 模差，
  无 ns 投影、无跨域绝对相减。

**判定：CONFIRMED。**

## 8. enum 域（N07）逐点

新 enum 与 `is_known()`：

| enum | `is_known()` | 消费者是否先域检查 |
|---|---|---|
| `CoreEventKind`（core.h:116） | 有（core.h:156） | `post()` 先 `core_event_kind_is_known` 再 no-`default` switch（core.cc:1150）✓ |
| `AdapterFault`（core.h:177） | 有（core.h:211） | `adapter_fault_to_exchange_status` 先查；`on_tx_outcome`、`on_overflow` 先查 ✓ |
| `CoreActionKind`（core.h:305） | 有（core.h:330） | 仅 core 产出；demo/test 的 switch 无 `default`、覆盖全成员 ✓ |
| `ProtocolIntervalOrigin`（protocol_time.h:282） | 有（:298） | `protocol_domain_ok` 先查（:663）✓ |
| `PeerClaimError`（:309） | 有（:327） | `make_peer` 返回、调用方比较，不 switch ✓ |
| `ProtocolCompletionStatus`（:520） | 有（:539） | QA 显式查域；demo/verifier 只做等值比较/格式化，无 switch ✓ |
| `FakeLinkError`（fake_link.h:146） | 有（:179） | QA 先查 ✓ |
| `KernelStatus`（math.h:801） | **无** | 唯一消费者 `protocol_kernel_to_tof_status` 无 `default`、兜底 `InvalidResult`（fail-closed） |

**判定：CONFIRMED，含两点 minor：**
- `KernelStatus` 是 M1-B 新 enum 但没有 `is_known()`（内部枚举、不可外部设置、兜底 fail-closed）→ N-3。
- `on_cancel` 直接用外部 `ev.cancel_reason`（`ExchangeStatus`，**非新 enum，且整个 `ExchangeStatus` 无 `is_known()`**）
  作为终态，无域检查。`probe1.cc`：`C out-of-domain cancel: status=invalid failure_reason=invalid` ——
  报告为失败、不抛异常，但把域外值原样写入结果 → N-2。
- 其余：无 `default` 的 switch 均为内部/已域检查；`switch(intent)`、`switch(a.kind)` 覆盖全成员并显式兜底。

## 9. 安装/独立消费

**声明**：新纯核心无 GNU Radio/UHD 依赖；安装消费者实际运行。

**检查与命令**：
- 编译单元：`compile_commands.json` 中 `uwb_twr_core.cc / uwb_twr_fake_link.cc / twr_fake_demo.cc` 均无
  `gnuradio-runtime|gnuradio/block|uhd|pmt|volk`。
- 静态库未解析符号仅 libc/libstdc++（`nm -u libuwb_twr_core.a`：`frexp,ldexp,memcmp,strlen,...`）。
- `ldd gr-uwb/build/apps/twr_fake_demo` 无 gnuradio/uhd。
- `tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_consumer` → 末行 `VERIFY INSTALL CONSUMER: ALL OK`，
  其中包含：9 个公开 TWR 头（含 `uwb_twr_core.h/uwb_twr_protocol_time.h`）就位、`uwb_twr_fake_link.h` 未安装、
  安装 `libuwb_twr_core.a`、用 `-I$prefix/include` 编译并运行 `twr_core_consumer.cc`、`ldd/readelf` 无 gnuradio/uhd。
- 手动复跑安装 prefix 上的核心消费者：

```
ss: estimate at A = 100/1 (expected 100/1), other end none=yes
ds: estimate at B = 100/1 (expected 100/1), other end none=yes
ALL OK ;  ldd: no gnuradio/uhd deps
```

- demo/verifier 实跑：`python3 gr-uwb/apps/test_twr_fake_demo.py` → `Ran 9 tests ... OK`；
  `verify_m1_b_protocol.py --request/--output` → `29/29 checks pass`。

**判定：CONFIRMED。** 备注（非阻塞 N-7）：CTest 里注册的 `uwb_qa_install_consumer` 调用的
`apps/install_consumer/run_install_consumer.sh` 只编译/运行 M0.1 的 `install_consumer.cc`，
**不**覆盖 M1-B 的 `twr_core_consumer.cc`；M1-B 核心消费者只在未注册 CTest 的
`tools/twr/verify_install_consumer.sh` 里被实跑。即 `ctest` 绿并不等于 M1-B 安装消费已验。

---

## 阻塞结论（Blocking）

**B-1（阻塞，provenance/status）**：成功协议估计的动作携带 `ExchangeStatus::Ok`，而
`exchange_status_yields_range(Ok)==true`。违反指示 §3.2“成功不伪造一个可传给 range helper 的状态”。
`ProtocolTofEstimate` 类型本身是安全的；缺口在**动作级 `status`** 以及 demo 输出的 `terminal_status:"ok"`。
复现：`/tmp/opencode/m1b_review/probe1.cc` 段 D。
建议：终态动作不要用 `ExchangeStatus::Ok`（改用 `ProtocolCompletionStatus`，或让动作的 range 相关 helper 与
`ProtocolTofEstimate` 一致恒 false），并相应调整 demo 输出。

**B-2（阻塞，生命周期/守恒）**：`EndpointCore::reset(cfg,gen)`（core.cc:1121-1129）在在途时调用 `configure()`，
静默丢弃该 exchange 的终态并清零计数器，违反指示 §4.3“stop/cancel/reset 作废在途并排空为取消/失败”与
“已接受请求的终态不丢”。`Reset` 事件本身正确。复现：`probe1.cc` 段 A。
建议：让 `reset()` 像 Reset 事件一样对在途发 `TerminalResult`（例如先内部 stop 或直接 fail），或把它降为非公共 API。

**B-3（阻塞，契约/文档 N01 类）**：`WireTimestampBinding::session_generation` 与
`PeerClaimError::SessionGenerationMismatch` 声称“different generation 的 claim 被拒绝”，但运行期无此检查
（仅 `configure()` 一次；`make_peer` 不接收本端 generation）。`Reset` 后 binding 变陈旧仍被使用并成交。
复现：`probe4.cc`。按 AGENTS 的 N01 教训，这种“note 描述了代码里不存在的机制”必须关闭：要么实现运行期
generation 检查（并在 Reset 时更新/失效 binding），要么删除/改写该声明与其保留枚举值。

## 非阻塞观察（Non-blocking）

- **N-1**：responder 对重放的旧 Poll 无重复/复用屏障（`probe1.cc` 段 B）。v1 wire 无 generation 是协议限制，
  但无任何诊断计数；建议至少计 `duplicate_frames` 或写入契约限制。
- **N-2**：`on_cancel` 未做 `cancel_reason` 域检查，域外值原样进入结果（`probe1.cc` 段 C）。
- **N-3**：新内部 enum `KernelStatus` 缺 `is_known()`（fail-closed 兜底，风险低）。
- **N-4**：`gr-uwb/apps/install_consumer/twr_core_consumer.cc:250` 的
  `const char* tof = ... ea.tof.to_string().c_str() : ...` 对临时 `std::string` 取 `c_str()`，是悬垂指针（UB）；
  本次 ASAN 未报是因为 SSO 栈内存仍在，属潜在缺陷，建议直接 `std::string`。
- **N-5**：`count_match_failure()` 从未被调用（死代码），`frames_rejected_self` 永远为 0；
  self-addressed 帧实际以 `frames_rejected_wrong_peer` 记账。FSM 的字段检查顺序（type→session→seq→pan/dst）
  与 `frame_match` 文档化的固定顺序（type→PAN→addr→session→seq）不同，多字段错误时归因桶会不同（非 fail-open）。
- **N-6**：B16 结构检测只按 `distance_m/tof/truth` 三个名字；人工穷举未见泄漏，但检测覆盖弱于声明“穷举”。
- **N-7**：CTest 的安装消费者不含 M1-B 核心消费者（仅 9 的 tools 脚本覆盖）。

## 未能验证（Unverified）

- 未重跑全量 59 项 CTest、未重测历史吞吐（`uwb_qa_uwb_pdu_rational_resampler.cc` 门槛未动，属历史项）。
- 未在本机重跑 MATLAB oracle；仅读取检入向量（`verify_m1_a_tof.py` 4/4）。
- 未验证 native PHY、真实收发、两 RX 路由、首径、timestamp patch、UHD——按 M1-B 范围本就不在。
- B01–B18 的“全矩阵”我按 §7 抽验了其中与本报告 9 项相关的部分；未逐条重放每一个 ID（例如 B17 中 demo 的
  全部拒绝用例只抽查了 wrong-addr / empty twr_config / hardware envelope；`test_twr_fake_demo.py` 全绿）。

## 复现命令

```bash
cd /home/oi/Desktop/uwb-gnuradio
g++ -std=c++17 -I gr-uwb/include -o /tmp/opencode/m1b_review/probe1 \
    /tmp/opencode/m1b_review/probe1.cc gr-uwb/lib/uwb_twr_core.cc && /tmp/opencode/m1b_review/probe1
g++ -std=c++17 -I gr-uwb/include -o /tmp/opencode/m1b_review/probe2 \
    /tmp/opencode/m1b_review/probe2.cc gr-uwb/lib/uwb_twr_core.cc && /tmp/opencode/m1b_review/probe2
g++ -std=c++17 -I gr-uwb/include -o /tmp/opencode/m1b_review/probe3 \
    /tmp/opencode/m1b_review/probe3.cc gr-uwb/lib/uwb_twr_core.cc && /tmp/opencode/m1b_review/probe3
g++ -std=c++17 -I gr-uwb/include -o /tmp/opencode/m1b_review/probe4 \
    /tmp/opencode/m1b_review/probe4.cc gr-uwb/lib/uwb_twr_core.cc && /tmp/opencode/m1b_review/probe4
tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_consumer
env -u LD_LIBRARY_PATH gr-uwb/build/lib/uwb_qa_uwb_twr_math.cc
python3 tools/twr/verify_m1_a_tof.py
```

（探针源码位于 `/tmp/opencode/m1b_review/`，非仓库文件；本报告未修改任何被测代码。）

---

# 整改复审（re-verification）

日期：2026-09-30。角色：仍为只读评审（本文件是唯一写出的文件）。

**复核 revision**：协调者交办时写的是 `c6db642`，但当前 `HEAD` 是 **`0dfd72d`**
（`harden(twr): expose the protocol failure reason only through a failed-only accessor`，
在 `c6db642` 之上再加一道 B-1 加固）。被审代码在 `0dfd72d` 无未提交改动
（`git status --short -- gr-uwb/... tools/twr` 为空）。**本复审针对 `0dfd72d`**，
下表兼列两级：`c6db642` 的原始整改 + `0dfd72d` 的追加加固。

方法：重读全部改动（`git show c6db642` / `git show 0dfd72d`）；重写并重跑独立探针
`/tmp/opencode/m1b_review/reverify.cc`、`n9.cc`、`residual.cc`、`b1_fail*.cc`；
出树编译并运行 4 个 QA；出树编译新 demo 并跑 Python demo QA + verifier；
用当前源码头 + 新编 `libuwb_twr_core.a` 构造独立 prefix，跑 N-7 的安装消费者包装脚本。
未改任何仓库文件。

## 逐项判定

| 编号 | 判定 | 证据（命令/观察） |
|---|---|---|
| **B-1** action 级 | **CLOSED**（余一处下钻残留 N-10） | `CoreAction::status` 已是 `ProtocolTerminalStatus`；`yields_range()==false`、`measurement_valid()==false`、`completed()==true`；三种误用均**编译失败** |
| **B-2** | **PARTIAL**（在途已闭环；未取走的已完成结果仍被 reset 丢弃） | 在途 `reset()` 被拒、计数器保留；但 `reset()` 仍 `results.clear()` |
| **B-3** | **CLOSED** | Reset 事件后旧 binding 的 peer claim 被拒（`invalid_time_domain`/`session_generation_mismatch`）；`reset()` 要求新 binding 后成功 |
| **N-1** | **CLOSED** | 重放 Poll 被拒：`accepted` 不变、`frames_rejected_seq`+1、`stale_events`+1 |
| **N-2** | **PARTIAL** | `222`/`0` → `cancelled`；但 `15`（域内空洞值）仍被原样暴露 |
| **N-3** | **CLOSED** | `kernel_status_is_known()` 存在且对 `static_cast<KernelStatus>(9)` 返回 false |
| **N-4** | **CLOSED** | 改为持有 `std::string`；ASAN+UBSan 干净 |
| **N-5** | **CLOSED** | 自地址帧 → `frames_rejected_self==1`；死函数已删 |
| **N-6** | **STILL-OPEN（已声明保留）** | B16 结构检测仍只查 3 个成员名；协调者明确记录为覆盖限制，非产品缺陷 |
| **N-7** | **CLOSED** | 新 prefix 下 wrapper 检查 9 头 + fake link 缺席 + 存档 + SS/DS 消费者 + 无 GR/UHD，退出 0 |

### B-1（CLOSED）——强类型终态

```
$ g++ -std=c++17 -I gr-uwb/include -c b1_fail.cc   # exchange_status_yields_range(a.status)
error: cannot convert 'const ProtocolTerminalStatus' to 'ExchangeStatus'      (exit 2)
$ ... b1_fail2.cc                                  # ExchangeStatus s = a.status;
error: cannot convert ... in initialization                                    (exit 1)
$ ... b1_fail3.cc                                  # exchange_status_yields_range(a.status.failure_reason)
error: invalid use of non-static member function 'bool failure_reason(ExchangeStatus&) const'  (exit 1)

$ /tmp/opencode/m1b_review/reverify
RV-B1 terminal: status.yields_range=0 status.measurement_valid=0 completed=1 completion=complete
```

`0dfd72d` 把失败原因改为仅 `bool failure_reason(ExchangeStatus& out) const`（成功时返回 false），
堵住了“读字段绕回 range helper”的写法。demo 输出改为
`terminal_completion=complete`，成功时**不输出** `terminal_failure_reason`（实测 absent）。

### B-2（PARTIAL）——reset 生命周期

```
RV-B2 inflight: accepted=1 in_flight=1
RV-B2 reset(inflight) refused=1 why=cannot reset while an exchange is in flight; \
     post a Reset/Stop event first ...  in_flight=1 accepted=1
RV-B2 reset(after terminal) ok=1 accepted=1 terminals=1
RV-B2 undrained: terminal=1 terminal_results=1
RV-B2 reset(clean) ok=1 then pop_result=0 (undrained result lost if 0) terminal_results=1
```

在途终态丢失与计数器清零（原阻塞项）已闭环：`reset()` 在途时拒绝，计数器保留，守恒恒等式成立。
**残留**：`reset()` 仍执行 `d.results.clear()`（core.cc），一个**已完成但调用方尚未
`pop_result` 取走**的终态结果会被静默丢弃（`pop_result()` 返回 false，而
`terminal_results` 仍为 1）。最小复现：

```cpp
EndpointCore c; c.configure(cfg, why);
const CoreAction* tr = ss_exchange(c, cfg, 1000);   // 完整 SS 交换，产生终态
assert(c.counters().terminal_results == 1);
CoreConfig fresh = cfg; fresh.session_generation = 3; fresh.peer_binding.session_generation = 3;
assert(c.reset(fresh, 3, why));                     // 在途已空，reset 成功
ProtocolTofEstimate out;
assert(!c.pop_result(out));                         // 结果已被 reset 丢弃
```

`core.h` 对 `reset()` 的注释也未声明会清结果队列。这是“已接受请求的终态结果被静默丢失”的
一种形式（计数保留、payload 丢失）。判定 **PARTIAL**：原阻塞缺陷已修，但同一接口的
未取走终态仍会消失。建议 `reset()` 要么保留未取走的已完成结果，要么在契约里明确写出并在
QA 中断言。

### B-3（CLOSED）——会话 binding generation 运行期生效

```
$ /tmp/opencode/m1b_review/reverify
RV-B3 after Reset event: gen=2 binding_gen=1
RV-B3 exchange after Reset: terminal=1 status=invalid_time_domain \
      detail=session_generation_mismatch
RV-B3 after reset(binding gen2)=1 exchange terminal=1 status=complete
```

`make_peer_interval()`（core.cc:482-485）在运行期比较 `peer_binding.session_generation`
与 `cfg.session_generation`，不一致即返回 `PeerClaimError::SessionGenerationMismatch`；
`reset()` 要求 fresh binding。Reset 事件后端点进入 fail-closed 状态（后续 peer-interval
交换失败），需经 `reset()`/`configure()` 换新 binding 才能恢复——方向正确。

### N-1（CLOSED）——responder 重放屏障

```
RV-N1 replay #0: accepted=1 terminal=1 frames_rejected_seq=0 stale=0
RV-N1 replay #1: accepted=1 terminal=1 frames_rejected_seq=1 stale=1
RV-N1 replay #2: accepted=1 terminal=1 frames_rejected_seq=2 stale=2
```

同一 `seq=0` 的 Poll 重放不再产生第二个 exchange。`seq_used[256]` 有界，索引前有
`seq >= sequence_modulus` 检查，安全。

### N-2（PARTIAL）——cancel_reason 域检查

```
RV-N2 cancel_reason=222 -> terminal reason=cancelled (raw=3 exposed=1)
RV-N2 cancel_reason=15  -> terminal reason=invalid   (raw=15 exposed=1)
RV-N2 cancel_reason=0   -> terminal reason=cancelled (raw=3 exposed=1)
```

`on_cancel` 现用 `st > NegativeTof(62) || st == Ok` 归一，`222` 与 `0` 被正确映射；
但 `ExchangeStatus` 的成员**不连续**（空洞：6–9、15–19、23–29、34–39、43–49、52–59），
`static_cast<ExchangeStatus>(15)` 落在域内区间却不属于任何成员，仍被原样存入终态并可由
`failure_reason()` 读出（`exchange_status_to_string` 打印 "invalid"）。
这是按数值范围近似域检查，不是真正的域测试。最小复现：`cancel(static_cast<ExchangeStatus>(15))`。
建议新增 `exchange_status_is_known()`（无 `default`）并用它，而非范围比较。

### N-3/N-4/N-5/N-7（CLOSED）

- N-3：`kernel_status_is_known(static_cast<KernelStatus>(9)) == false`，`Ok/Overflow/ZeroDenominator == true`。
- N-4：`twr_core_consumer.cc` 改持 `std::string tof`；`-fsanitize=address,undefined` 运行 `ALL OK`、无报告。
- N-5：`poll.src_addr = local_address` → `frames_rejected_self=1 wrong_peer=0 accepted=0`；`count_match_failure` 已删除。
- N-7：用当前源码头 + 新编 `libuwb_twr_core.a` 构造独立 prefix 后跑
  `gr-uwb/apps/install_consumer/run_install_consumer.sh`：
  `== M1-B standalone core consumer (SS + DS) == … ALL OK … no GNU Radio / UHD dynamic dependency`，
  末尾 `install consumer: ALL OK`，退出 0。脚本确实检查 `uwb_twr_protocol_time.h`/`uwb_twr_core.h`
  存在、`uwb_twr_fake_link.h` **未**安装、`libuwb_twr_core.a` 存在并实跑 SS/DS 消费者。

## QA 出树实跑（协调者要求，不碰 build 树）

```bash
g++ -std=c++17 -Wall -Wextra -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN \
    -I gr-uwb/include -o /tmp/.../out_<t> \
    gr-uwb/lib/uwb_twr_core.cc gr-uwb/lib/uwb_twr_fake_link.cc gr-uwb/lib/<t>.cc \
    -lboost_unit_test_framework
```

| QA | 结果 |
|---|---|
| `qa_uwb_twr_core.cc` | **25 test cases, rc=0**，`No errors detected`（含 f_b1/f_b2/f_b3/f_n1） |
| `qa_uwb_twr_m1b_e2e.cc` | **11 test cases, rc=0** |
| `qa_uwb_twr_protocol_time.cc` | **10 test cases, rc=0** |
| `qa_uwb_twr_fake_link.cc` | **14 test cases, rc=0** |

新增 4 例没有放宽任何断言：`f_b1` 用 `static_assert` 断言 `CoreAction::status` 的强类型、
不可转 `ExchangeStatus`/`bool`，并断言成功时 `failure_reason()` 不暴露；`f_b2`/`f_b3`/`f_n1`
断言拒绝行为本身，把 QA 内部的 `action_status()` 仅作测试侧映射（产品 API 不再暴露该状态）。

新 demo（出树编译当前源码）实跑：`test_twr_fake_demo.py` **9/9 OK**；
`verify_m1_b_protocol.py` ss_unity **24/24**、ds_unity **29/29**、ss_nominal **24/24**。

## 复审中新发现的残留（均非阻塞，但未关闭）

- **N-8（B-2 残留）**：`reset()` 清除未取走的已完成终态结果（见上，含复现）。
- **N-9**：`on_reset`（Reset 事件）把 `seq_issued=0`，但**不**清 `seq_used`。复现
  （`n9.cc`）：responder 服务过 `seq=0` 后收到 Reset 事件，再收同一 `seq=0` 的 Poll
  仍被拒（`prepare=0 frames_rejected_seq=1 stale=2`）。同一端点的 initiator 侧会重用 seq、
  responder 侧拒绝，行为不对称；方向是 fail-closed，故非阻塞，但新会话语义不完整。
- **N-10（B-1 残留）**：`ProtocolTofEstimate::failure_reason` 仍是公开 `ExchangeStatus`，
  成功时为 `Ok`，`exchange_status_yields_range(e.failure_reason)` 返回 **true**：

  ```
  $ /tmp/opencode/m1b_review/residual
  residual: result.yields_range=0 but exchange_status_yields_range(e.failure_reason)=1
  ```

  新 demo 的成功端点 JSON 仍输出 `"failure_reason":"ok"`（与 `terminal_completion=complete`
  并存）。这是 `0dfd72d` 在 `ProtocolTerminalStatus` 上堵掉、但在 `ProtocolTofEstimate` 上
  同样存在的一处“成功携带可传给 range helper 的状态”。因该字段属 G0 已批准的结果词汇、
  且结果类型自身的 `yields_range()` 为 false，本次仍按**非阻塞残留**记录；若要与 B-1 同等收紧，
  建议对结果字段也用 failed-only 访问器或在成功时置非 `Ok` 哨兵/不输出。
- **N-11（文档）**：两处注释与代码不符——
  1. `uwb_twr_protocol_time.h:318-322` 仍写 `SessionGenerationMismatch`“no M1-B path returns
     this value”，但 core.cc:483 现在会返回它（N01 类“note 描述不存在的机制”的反向版：存在却写没有）；
  2. `uwb_twr_core.h:508-511` 的 `reset()` 注释仍写“Clears all in-flight state（emitting
     terminal failures ... is the caller's job via stop() first）”，与现在的“在途拒绝、需先发
     Reset/Stop、且需 fresh binding”不符，也未提及会清结果队列。

## 复审结论

- 3 项阻塞：**B-1 CLOSED、B-3 CLOSED、B-2 PARTIAL**（在途缺陷已闭环，未取走终态仍丢）。
- 非阻塞：N-1/N-3/N-4/N-5/N-7 **CLOSED**；N-2 **PARTIAL**；N-6 按协调者声明保留；
  新增残留 N-8/N-9/N-10/N-11。
- 未复核：全量 59 项 CTest 与历史吞吐（协调者所有）；`uwb_qa_install_consumer` 我未用
  项目 build 树跑（避免 ABI 陈旧），而是用当前源码头 + 新编存档构造的独立 prefix 实跑，
  逻辑等价但不等同“协调者重新构建后的安装门禁”。

---

# 整改复审（第二轮）

日期：2026-09-30。只读评审，唯一写出本文件。复核 revision：`d71b7b4`
（`fix(twr): close the reviewer's second round`）；当前 `HEAD 94c4364` 仅在其上加一个
**纯文档**提交，`gr-uwb/...` 代码在 `94c4364` 与 `d71b7b4` 相同且无未提交改动
（`git status --short -- gr-uwb tools/twr` 为空）。探针：`/tmp/opencode/m1b_review/reverify2.cc`。

| 项 | 判定 | 命令 / 观察 |
|---|---|---|
| **N-8** reset 保留未取走终态 | **CLOSED** | `reverify2`：`RV2-N8 reset=1 then pop_result=1 retained_estimate=1 terminal_results=1` |
| **N-10** 结果不再有 range 兼容状态 | **CLOSED** | (a)(b) 三种读法均编译失败；(c) 成功 `failure_reason()==false`、失败 `tx_late`；(d) demo 成功端点无 `failure_reason` 键 |
| **N-2** 域内空洞值 | **CLOSED** | `reverify2`：`6/9/15/55/222/0 → cancelled`，合法 `62 → negative_tof` |
| **N-9** responder 屏障 Reset 后不清 | **CLOSED**（fail-closed，已文档化） | `reverify2`：Reset 后重放仍 `prepare=0 frames_rejected_seq=1`；已写入 core.cc 与 G0 |
| **N-11** 两处过期注释 | **CLOSED** | 注释已改写为与实现一致（见下引） |

### N-8（CLOSED）
`EndpointCore::reset()` 不再 `results.clear()`（core.cc 注释：“Undrained terminal results are
RETAINED ... `results` is bounded ... new Begins refused rather than losing anything”）。
实测完整交换后不 drain、`reset(fresh,3)` 成功、`pop_result()` 仍返回该估计（`pop_result=1`）。
QA 新增 `f_n8_reset_retains_undrained_terminal_results`。

### N-10（CLOSED）
```bash
g++ -std=c++17 -I gr-uwb/include -c n10a.cc   # exchange_status_yields_range(e.failure_reason)
# error: invalid use of non-static member function 'bool ...failure_reason(ExchangeStatus&) const'
g++ ... -c n10b.cc                             # return e.failure_reason;
# error: cannot convert '...failure_reason' ... to 'ExchangeStatus'
```
`ProtocolTofEstimate` 现无公开 `ExchangeStatus` 数据成员，失败原因仅经
`bool failure_reason(ExchangeStatus&) const`（仅 `completion==Failed` 时返回 true）暴露；
`(c)` 成功返回 false、失败返回 true 且值正确。demo 的成功端点 JSON **完全不含**
`failure_reason` 键（实测 `'<absent>'`，全 JSON 中 `"failure_reason":"ok"` 计数为 0），
`terminal_failure_reason` 也仅在失败时输出。QA 新增 `f_n10_estimate_failure_reason_is_failed_only`。

### N-2（CLOSED）
core.cc 新增本地 `exchange_status_is_known()`（无 `default` 的完整成员 switch），
`on_cancel` 改用它。实测数字空洞 `6/9/15/55` 与 `222/0` 均归一为 `Cancelled`，
合法成员 `62`（`NegativeTof`）原样保留。QA 新增 `f_n2_out_of_domain_cancel_reason_is_not_stored`
（用空洞值 `6`）。不再有“按数值范围近似域检查”的问题。

### N-9（CLOSED）
`on_reset` 仍不清 `seq_used`，实测 Reset 事件后同一 `seq=0` 的 Poll 重放被拒
（`prepare=0 accepted=1 frames_rejected_seq=1 stale=2`）。这是**有意 fail-closed**，且已文档化：
- `gr-uwb/lib/uwb_twr_core.cc` `on_reset` 内注释明确“deliberately NOT cleared here ...
  asymmetric with `seq_issued`, but it fails closed”；
- `docs/twr/M1-B_G0接口与状态表.md` §4 新增段落说明 responder `seq_used` / initiator
  `seq_issued` 的不对称与 `reset()` 语义。

### N-11（CLOSED）
- `uwb_twr_protocol_time.h` 的 `SessionGenerationMismatch` 注释现写“Produced at
  claim-formation time by `EndpointCore` ...”，与 core.cc 的生产者一致；
- `uwb_twr_core.h` 的 `reset()` 注释现写“REFUSES while an exchange is in flight ...
  requires ... a peer binding for the new generation, does NOT clear the counters, and
  RETAINS undrained terminal results”，与实现一致。

### QA 出树实跑（未碰 build 树）

```bash
g++ -std=c++17 -Wall -Wextra -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN -I gr-uwb/include \
    -o out_<t> gr-uwb/lib/uwb_twr_core.cc gr-uwb/lib/uwb_twr_fake_link.cc gr-uwb/lib/<t>.cc \
    -lboost_unit_test_framework
```

| 目标 | 退出码 | 用例数 | 结果 |
|---|---|---|---|
| `qa_uwb_twr_core.cc` | **0** | **28** | `No errors detected`（含 f_n2/f_n8/f_n10） |
| `qa_uwb_twr_m1b_e2e.cc` | **0** | 11 | `No errors detected` |
| `qa_uwb_twr_protocol_time.cc` | **0** | 10 | `No errors detected` |
| `qa_uwb_twr_fake_link.cc` | **0** | 14 | `No errors detected` |
| `test_twr_fake_demo.py`（新 demo） | **0** | 9 | `OK` |

第二轮 5 项全部 **CLOSED**，无新反例。唯一保留项仍是先前声明的 **N-6**（B16 结构检测只查
3 个成员名，测试强度问题，非产品缺陷）。**未复核**：全量 59 项 CTest 与历史吞吐（协调者所有）。


