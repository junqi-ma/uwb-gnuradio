# M1-B 协议核心与双端仿真开发报告

> **状态更正（2026-09-30，第二轮独立复核）**：本报告 §1 的"完成 / B01–B18 均 FULL /
> 所有发现关闭"结论**不再成立**。独立复核（
> [M1-B_复核整改任务单_2026-09-30.md](M1-B_复核整改任务单_2026-09-30.md)、
> [M1-B_复核整改问题台账_2026-09-30.md](M1-B_复核整改问题台账_2026-09-30.md)）
> 在**同一 revision** 上复现了 R01–R09：TxPlanned 可携带 Completed 提前成功、
> DS initiator 遗漏 Poll outcome、非估计端绕过本端严格准入、reset 清身份屏障与
> token 复位、responder 未预留终态容量、空闲域外 outcome 生成无主终态、
> evidence 上限覆盖绝对 exchange 上限、计划无数值映射、TwrConfig 校验后丢弃。
> **M1-B 状态改为"已交付，复核整改中"；R01–R09 关闭前未完成，不进入 M2。**
> 本报告 §6/§12 的下述勾选按此更正，历史结论保留不抹去；整改结论见
> `M1-B_复核整改报告_2026-09-30.md`。

日期：2026-09-30。范围依据：
[OpenCode开发指示_M1-B_协议核心与双端仿真.md](OpenCode开发指示_M1-B_协议核心与双端仿真.md)（下称"任务单"）。
G0 接口记录：[M1-B_G0接口与状态表.md](M1-B_G0接口与状态表.md)。

## 1. 结论

M1-B 在**离线、纯 C++、接收驱动**的范围内完成：SS/DS 两个独立端点的协议核心、
确定性 fake link、故障与生命周期 QA、Python→C++ 仿真 demo、独立 verifier、
安装与独立消费验证。**B01–B18 均有可定位证据**（见 §6）。

**这不是硬件测距。** 本阶段所有结果都是带 `simulation` / `wire_claim` 标注的
**协议估计**：`ProtocolTofEstimate::measurement_valid == false`、
`yields_range() == false`、`is_hardware_measurement() == false`，
且该类型**不能**隐式转成 `TofResult` 或 `AdmittedRangingInterval`。
远端帧里的整数**没有被**升级成 `HardwareMeasured`、已校准或 TX Completed。
正式 radio/controller/UHD/两 RX/首径/native 重采样/CFO 估计/pybind 大迁移
均**不在**本阶段，未进入 M2。

## 2. 基线与环境

- 分支 `feature/uwb-ds-twr`；本阶段起点 `fa49ed1` 之前的 HEAD `2cf881b`。
- 本报告锁定的最终 revision：见 §11（工作区 hash）。
- 构建：`cmake -S gr-uwb -B gr-uwb/build && cmake --build gr-uwb/build -j8`，
  Release，Linux，17 核，GNU Radio 3.10 + UHD（TWR 纯核心目标不链接二者）。
- 验证一律 `env -u LD_LIBRARY_PATH`；需库的测试用 `ldd` 确认加载 `build/lib`。

## 3. 交付物

### 3.1 新增文件

| 文件 | 归属 | 说明 |
|---|---|---|
| `gr-uwb/include/gnuradio/uwb/uwb_twr_protocol_time.h` | 协调者（A 审查） | wire claim 边界（安装） |
| `gr-uwb/include/gnuradio/uwb/uwb_twr_core.h` | 协调者 | 端点核心契约（安装） |
| `gr-uwb/lib/uwb_twr_core.cc` | 协调者（B 角色） | 唯一 FSM 实现 |
| `gr-uwb/include/gnuradio/uwb/uwb_twr_fake_link.h` | C | 确定性 transport（**不安装**） |
| `gr-uwb/lib/uwb_twr_fake_link.cc` | C | |
| `gr-uwb/lib/qa_uwb_twr_fake_link.cc` | C | 14 例 |
| `gr-uwb/lib/qa_uwb_twr_protocol_time.cc` | A | 10 例 |
| `gr-uwb/lib/twr_m1b_test_support.h` | D | 双核心↔fake link 驱动 |
| `gr-uwb/lib/qa_uwb_twr_core.cc` | D（协调者补 7 例） | 28 例 |
| `gr-uwb/lib/qa_uwb_twr_m1b_e2e.cc` | D | 11 例 / 248 断言 |
| `testdata/twr/m1b/{README.md,event_cases.json,expected_results.json}` | D | 9 场景 golden |
| `gr-uwb/apps/twr_fake_demo.cc` | E | 纯 C++ CLI 双端仿真 |
| `gr-uwb/apps/twr_fake_demo.py` | E | 请求构造 + 结果消费 |
| `gr-uwb/apps/test_twr_fake_demo.py` | E | 9 例 Python QA |
| `tools/twr/verify_m1_b_protocol.py` | E | 独立 verifier（29 检查） |
| `gr-uwb/apps/install_consumer/twr_core_consumer.cc` | 协调者 | 安装消费者（手写事件驱动） |

### 3.2 最小改动的既有文件

- `gr-uwb/include/gnuradio/uwb/uwb_twr_math.h`：把 SS/DS 精确有理数运算抽出为
  **唯一**共享内核 `detail::ss_tof_kernel` / `detail::ds_tof_kernel`
  （+ `KernelStatus`）；`compute_ss_tof` / `compute_ds_tof` 改为「原严格检查 +
  调内核」，**语义、状态映射、负值/溢出/域/ratio/window 检查与 14 例 QA 全部不变**
  （见 §7 回归）。未新增裸 `Timestamp`/裸 interval 重载，未加跳过校验的后门，
  未改 `uwb_twr_tof_input.h` 的准入规则。
- `gr-uwb/lib/CMakeLists.txt`、`gr-uwb/apps/CMakeLists.txt`、
  `gr-uwb/include/gnuradio/uwb/CMakeLists.txt`：新增纯核心/仿真目标、安装两个新公开头
  与独立 `libuwb_twr_core.a`、注册 6 个新 CTest。
- `tools/twr/verify_install_consumer.sh`：新增 M1-B 头/独立库检查与 M1-B 消费者实跑。

### 3.3 分工执行说明（如实记录）

任务单 §5 的分工为「协调者 + 6 个 subagent」。本会话按 P0→P3 执行：
**A/C/D/E/F 作为独立 subagent**负责各自文件；G0 接口（两个头）与 FSM 实现
（`uwb_twr_core.cc`，B 角色）由协调者在同一会话内完成，因为 `core.cc` 依赖已冻结
契约且需与安装/CMake 同步演进。全程**只有一个 FSM 实现**，无第二份时间模型/公式；
没有两个 agent 同时写同一文件。A/C/D/E 的产物均经协调者独立复跑后提交。

## 4. §3 协议声明边界（G0 冻结）

见 G0 文档 §1。要点：

- 本端 RX/TX 走**严格** `admit_ranging_interval()`；远端整数走
  `PeerTimestampClaim` + `WireTimestampBinding` → `ProtocolInterval`（`peer_wire_claim`）。
- `ProtocolInterval` 是封闭变体，无到 `AdmittedRangingInterval` 的转换；
  `ProtocolTofEstimate` 无到 `TofResult` 的转换，且所有 range 入口恒 false。
- 协议完成用独立 `ProtocolCompletionStatus`，不用 `ExchangeStatus::Ok` 作顶层成功态。
- 无 ToF 归属的一端（SS responder / DS initiator）有自己的协议终态
  （`estimate_available=false`），**不从另一端复制距离**。
- 本地证据未收敛只留候选：结果等待本端 `TxOutcome == Completed`；
  `evidence_wait_ticks` 到期即 `ProtocolTimeout`，不泄露候选 ToF。
- wire 无 fraction；超宽/fraction wire 显式拒绝；本地→wire 单位换算要求
  `frame_profile.timestamp_unit_hz == local_domain.tick_rate_hz`，否则 `configure` 拒绝。

G0 门槛（QA 先证明 wire 不能伪造成严格测量输入）由
`qa_uwb_twr_protocol_time.cc` 的类型级 `static_assert` 与拒绝矩阵满足。

## 5. 状态转移与实现

状态表见 G0 文档 §3/§4（已按实现修正 SS responder 一行）。实现要点：

- core 不 include GR/PMT/UHD，不读系统时钟，不 sleep、不做 I/O；时间只由事件推进。
- 有界：每次 `post()` 返回定长 `CoreActionBatch`（≤4）；结果存于
  `result_queue_capacity` 有界存储；容量满时**拒绝新 Begin**，已接受请求的终态不丢。
- 守恒：`accepted_exchanges == terminal_results + in_flight`（每端每 attempt）。
- 身份复用屏障：`seq_issued >= sequence_modulus` 时拒绝新的 Begin（要求新 session）。
- 无关噪声/错 peer 不取消正确在途请求；FCS/解码失败、错类型/PAN/src/dst/session/seq
  分别计数且不回应答。
- adapter fault（SeqError/ChainBroken 等）映射到各自 `ExchangeStatus`，
  **不 cast 成域外 `TxOutcome`**。

## 6. B01–B18 证据

QA 场景与期望依据见 `testdata/twr/m1b/README.md`。此处只列 ID → 覆盖与证据位置
（不以下载测试总数代替矩阵）。

> **复核更正**：下表是**整改前**的覆盖结论。独立复核证明 B06/B07/B08/B11/B12/B13/
> B14/B15/B17/B18 的既有 QA 未覆盖 R01–R09 的坏行为，故这些 FULL 不能维持；
> 整改后的覆盖以 `M1-B_复核整改报告_2026-09-30.md` 为准。

| ID | 状态 | 主要证据 |
|---|---|---|
| B01 | FULL | `qa_uwb_twr_m1b_e2e.cc::b01_*`：SS/DS × A/B 发起；估计只在正确端，另一端无复制距离 |
| B02 | FULL | `b02_ss_k_gt1 / ds_k_gt1 / ss_k_lt1 / ds_unequal_replies`（不同 nominal rate，逐字段复算）；`qa_uwb_twr_core.cc::b02_out_of_window_ratio`；缺 ratio 在 configure 拒绝 |
| B03 | FULL | `b03_dropped_{poll,response_ss,response_ds,final_ds}`；活跃端超时收敛，被动端不生成请求 |
| B04 | FULL | `qa_uwb_twr_core.cc::b04_wrong_fields_* / b04_bad_fcs_* / b04_codec_*`（含首轮暴露的 seq/session 计数缺陷，已修 `19bf2dc`） |
| B05 | FULL | `b05_duplicate_*`、`b05_sequence_modulus_is_the_reuse_barrier_and_reset_clears_it` |
| B06 | FULL | `b06_frame_fields_come_from_the_same_plan`、`b06_bad_plan_*`（缺记录/错 token/域外 enum/不可行 deadline） |
| B07 | FULL | `b07_accepted_is_not_completion_and_rx_before_outcome_still_completes`、`b07_unresolved_evidence_with_a_deadline_fails_finitely` |
| B08 | FULL | `b08_failure_outcomes_and_faults_fail_without_resend` |
| B09 | FULL | `b09_wire_tick_space_wrap_and_span_policy`（wrap/恰好半周期/超半周期/超 span） |
| B10 | FULL | `b10_local_reset_fails_inflight_and_old_events_cannot_revive` |
| B11 | FULL | `b11_local_first_path_and_calibration_are_mandatory` |
| B12 | FULL | `b12_peer_claims_and_protocol_estimate_never_become_a_range` |
| B13 | FULL | `b13_bounded_results_refuse_a_new_begin_without_losing_a_terminal` |
| B14 | FULL | `b14_exactly_one_terminal_per_attempt_and_conservation_holds` |
| B15 | FULL | `b15_negative_tof_* / b15_out_of_domain_event_* / b15_configure_*` |
| B16 | FULL | `b16_endpoint_has_no_ground_truth_input_and_a_missing_frame_changes_the_outcome`（结构 traits + 因果删除帧） |
| B17 | FULL | 纯核心单测 `ldd` 无 gnuradio/uhd；`uwb_qa_twr_fake_demo_py`、`uwb_qa_twr_m1b_verify_py`、`uwb_qa_install_consumer`（含独立库 HTML/apps 消费者） |
| B18 | FULL | 同 seed/同场景逐字段一致（`b18_same_scenario_is_bit_for_bit_reproducible`）；全量串行 CTest 与 M0.1/M1-A/Python 回归见 §7 |

## 7. 门禁与结果（本机实测）

| 项 | 命令 | 结果 |
|---|---|---|
| 构建 | `cmake -S gr-uwb -B gr-uwb/build && cmake --build gr-uwb/build -j8` | 退出 0 |
| 全量串行 CTest | `env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j 1 --output-on-failure` | **59 项，58 通过，1 失败** |
| 唯一失败 | `uwb_qa_uwb_pdu_rational_resampler.cc` | 历史吞吐项，**门槛未改**，与 M1-B 无关 |
| M1-B 专项 | `ctest -R 'twr'` | 15/15 通过（config/frame/phy matrix/timing/timestamp/tof_input/parity/math/protocol_time/fake_link/core/e2e/config_py/demo_py/verify 全绿） |
| Python 配置 | `python3 gr-uwb/apps/test_twr_config.py` | 138 OK |
| M0.1 独立 | `python3 tools/twr/verify_m0_1_findings.py` | 15/15 |
| M1-A 独立 | `python3 tools/twr/verify_m1_a_tof.py` | 4/4 |
| 安装消费 | `tools/twr/verify_install_consumer.sh <tmp prefix>` | ALL OK（含 M1-B 消费者 SS/DS，无 GR/UHD 动态依赖） |
| demo 场景 | `test_twr_fake_demo.py` | 9 用例通过，4 场景 ToF 精确 |
| 独立 verifier | `verify_m1_b_protocol.py --request --output` | 29/29（对变异输出会失败退出 1） |

M1-A 的 14 例 C++ QA 与 12 条 MATLAB oracle 向量在 `uwb_qa_uwb_twr_math.cc` 中
继续通过，证明内核抽取未改变严格路径语义。

## 8. 安装与独立消费

- 公开安装头：`uwb_twr_core.h`、`uwb_twr_protocol_time.h`（+ 既有 7 个）。
- 独立静态库 `libuwb_twr_core.a` 安装到前缀 `lib/`；
  `uwb_twr_fake_link.h` **不安装**（QA/demo 支持）。
- `gr-uwb/apps/install_consumer/twr_core_consumer.cc` 只用前缀头 + 已装库，
  手写事件驱动一次 SS 与一次 DS，断言精确 ToF=100/1 与「另一端无估计」；
  `ldd`/`readelf` 无 gnuradio/uhd 依赖。
- 新纯 QA（protocol_time/fake_link/core/e2e）独立注册，只链
  `uwb_twr_core`/`uwb_twr_fake_link`/Boost.Test，**不链** `gnuradio-uwb`。

## 9. 分配与容量观测

- core 无每事件动态分配：输入按值、输出定长 batch；每次 `configure()` 只做
  一次 `results.reserve(result_queue_capacity)`。
- `qa_uwb_twr_core` / `qa_uwb_twr_m1b_e2e` 在 ASAN+UBSan（`detect_leaks=1`）下
  无报告。
- 既有数学/准入路径的分配未被本阶段改变；未声称「零分配」。
- 容量边界（结果存储、动作 batch、RX 队列、fake link 队列）均有上限并有 QA。

## 10. 未做 / 未验证

- 真实 PHY/FCS/FEC/波形 timestamp patch、native 重采样、两 RX 路由、UHD adapter、
  controller、pybind 大迁移、Report、STS、商用 SDK —— 均不在 M1-B。
- **首径 ToA、RMARKER 绝对约定、reply-delay 实测、CFO/SFO 估计** 未做；
  48 行 PHY 白名单证据级**未升级**，`allows(work_decode,Ranging)` 仍为 0。
- 无任何硬件/空口测试；无精度标定；不声称厘米级，不声称商用互通。
- fake link 的传播/量化是**模拟**模型，不代表实际链路。
- 远端帧不含 epoch/校准/质量/分数字段，因此**无法**检测数值上相同的远端隐蔽
  reset；这是协议声明本身的限制。
- `--config` 接受路径只用 C++ 端校验证明；未用一份完整的合法 `TwrConfig` JSON
  端到端跑通（`twr_config.py` 未改）。

## 11. 复现

```bash
cd /home/oi/Desktop/uwb-gnuradio
cmake -S gr-uwb -B gr-uwb/build && cmake --build gr-uwb/build -j8
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -R 'twr' -j 1 --output-on-failure
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j 1 --output-on-failure
python3 gr-uwb/apps/test_twr_config.py
python3 gr-uwb/apps/test_twr_fake_demo.py
python3 tools/twr/verify_m0_1_findings.py
python3 tools/twr/verify_m1_a_tof.py
python3 gr-uwb/apps/twr_fake_demo.py --scenario ds_unity \
    --request-out /tmp/opencode/req.json --output /tmp/opencode/out.json
python3 tools/twr/verify_m1_b_protocol.py \
    --request /tmp/opencode/req.json --output /tmp/opencode/out.json
tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_m1b
```

最终 revision 与文件哈希：

```
revision (M1-B final code) = d71b7b4
sha256(uwb_twr_core.h)            = b1cac7ded988380f3335692a864455ee1bc0210a62a5e1e6f07b3a726640e1a5
sha256(uwb_twr_protocol_time.h)   = dbbf18799b26af150bb78291443956d60b46d883c936edcd1735ba3212b32f64
sha256(uwb_twr_core.cc)           = 1a4814d1d5bbc6f41bf698bb1989a1e624e0cee77b7b874c1c40495cf3a87345
sha256(uwb_twr_fake_link.h)       = 218712dd4b8e018b70aa7be393586053811e0af2de84a5372ef2edeeee45f90c
```

## 12. 完成判定（任务单 §9）

- [x] G0 输入/结果边界明确；F 的只读审查见 [M1-B_评审报告.md](M1-B_评审报告.md)。
- [x] 纯 C++ 两端 SS/DS 四个方向单元通过，实际接收驱动。
- [x] wire claim 与本端严格输入在类型与输出语义上分开；无伪造证据/后门。
- [x] 唯一数学内核；M1-A API/14 例 QA/12 oracle 与独立验证继续通过。
- [x] B01–B18 均有实际证据；每端 accepted/terminal/in-flight 守恒。
- [x] deadline/Unknown/late/underflow/取消/overflow/容量均有限时间收敛。
- [x] Python 配置→C++ 双端→结果/verifier 可复跑；无 Python FSM、无大迁移。
- [x] 核心独立消费、最终安装交付通过；无新增回归，历史门槛未改。
- [x] 文档写明 simulation/protocol estimate 不等于正式测距；PHY/ToA/硬件未升级。
- [x] 同步 AGENTS/开发状态/总路线（见提交）。

## 13. F 只读评审与整改

F 的评审见 [M1-B_评审报告.md](M1-B_评审报告.md)。**3 项阻塞 + 7 项非阻塞**全部处理：

| 编号 | 问题 | 整改 |
|---|---|---|
| B-1 | 成功协议估计的 `CoreAction::status` 是 `ExchangeStatus::Ok`，`exchange_status_yields_range(Ok)==true`，重新打开了类型层已关闭的 fail-open | 新增强类型 `ProtocolTerminalStatus`（`yields_range()==false`、无到 `ExchangeStatus` 的转换），`CoreAction::status` 改用它；失败原因只经「仅失败时成功」的 `failure_reason(out)` 访问器暴露，**没有** `ExchangeStatus` 数据成员（连字段直读的绕过也堵死）；demo 输出改为 `terminal_completion`/`terminal_failure_reason`；QA 加 `f_b1_protocol_terminal_status_is_not_a_range_status`（含 `static_assert` 与「成功时无原因」断言） |
| B-2 | `EndpointCore::reset(cfg,gen)` 内部调 `configure()`，静默丢弃在途终态并清空计数器 | `reset()` 在途时**拒绝**；抽出 `validate_core_config()`，reset 不再清计数器；QA `f_b2_reset_refuses_in_flight_and_preserves_counters` |
| B-3 | `WireTimestampBinding::session_generation` 文档称强制，实际只在 configure 检查；Reset 后仍被使用 | 运行期在 `make_peer_interval()` 校验 binding 与当前 session generation；`Reset` 后旧 binding 的 peer claim 被拒（`PeerClaimError::SessionGenerationMismatch` 终于有生产者）；`reset()` 要求新 binding；QA `f_b3_stale_peer_binding_generation_is_refused` |
| N-1 | responder 侧无序号复用屏障，重放旧 Poll 会开新 exchange | 新增有界 `seq_used[256]`，已消费的 wire seq 重放被拒并计 `stale_events`；QA `f_n1_responder_replay_of_a_consumed_sequence_is_refused` |
| N-2 | 域外 `cancel_reason` 被原样保存 | `on_cancel` 先做域检查，域外/`Ok` 归一为 `Cancelled` |
| N-3 | `KernelStatus` 缺 `is_known()` | 新增 `kernel_status_is_known()`（无 `default`） |
| N-4 | `twr_core_consumer.cc` 取临时 `to_string().c_str()` 悬垂 | 改为持有 `std::string` |
| N-5 | `count_match_failure` 死代码；`frames_rejected_self` 恒 0 | 删除死函数；`validate_frame` 显式判 `src_addr == local_address` 并计 `frames_rejected_self` |
| N-6 | B16 结构检测只查 3 个成员名 | **未改**（D 的测试强度问题，非产品缺陷）；F 的人工穷举未发现泄漏，作为已知覆盖限制记录 |
| N-7 | CTest 的 `uwb_qa_install_consumer` 只跑 M0.1 消费者 | `gr-uwb/apps/install_consumer/run_install_consumer.sh` 增加 M1-B 头检查、`libuwb_twr_core.a` 检查与 SS/DS 消费者实跑、无 GR/UHD 依赖检查 |

### 13.1 F 复审（第二轮）的新发现与整改

F 复审把 B-1/B-3 判为 CLOSED、B-2 判为 PARTIAL，并新增以下项：

| 编号 | 问题 | 整改 |
|---|---|---|
| 残余 B-2 / N-8 | `reset()` 虽不再清计数器，但仍 `results.clear()`，未取走的终态丢失 | `reset()` **保留**未取走终态；QA `f_n8_reset_retains_undrained_terminal_results` |
| N-2（加强） | `cancel_reason` 的 `>62 || ==Ok` 不是域检查，枚举数字空洞（如 6）漏过 | core.cc 增局部 `exchange_status_is_known()`（全枚举、无 `default`）；QA `f_n2_out_of_domain_cancel_reason_is_not_stored` |
| N-10 | `ProtocolTofEstimate::failure_reason` 仍是公开 `ExchangeStatus`，成功时 `Ok`，demo 仍打印 `"failure_reason":"ok"` | 改为「仅失败时成功」的 `failure_reason(out)` 访问器 + `set_failure_reason()`；**无** `ExchangeStatus` 数据成员；demo 只在失败时输出；QA `f_n10_estimate_failure_reason_is_failed_only` |
| N-11 | `uwb_twr_protocol_time.h` 仍称 `SessionGenerationMismatch` 无生产者；`uwb_twr_core.h` 的 `reset()` 注释仍是旧行为 | 两处注释已改为与实现一致 |
| N-9 | `Reset` 事件清 `seq_issued` 但不清 `seq_used`，responder 屏障不对称 | **保留行为**（同样的 wire bytes 不能因本端 generation 变更而自动被接受，fail-closed），在 `on_reset` 与 G0 §4 显式记录 |

整改后全量串行 CTest 仍为 **59 项 58 通过**；`qa_uwb_twr_core.cc` 增至 **28 例 / 全绿**。

### 13.2 F 的最终复审结论

F 复审两轮后（报告已追加“整改复审”章节）逐项判定：

- B-1 / B-3 / N-1 / N-2 / N-3 / N-4 / N-5 / N-7 / N-8 / N-9 / N-10 / N-11
  —— **全部 CLOSED**（每项附命令与观察；N-9 为「fail-closed、已在代码与 G0 记录」）。
- B-2 —— 首轮 PARTIAL，N-8 修复后 **CLOSED**。
- **N-6** —— 保留：B16 的结构检测只按名字查 3 个成员，属**测试强度**问题而非产品缺陷；
  F 的人工穷举未发现泄漏，作为已知覆盖限制记录，不影响 M1-B 验收。
- F 未能复核：完整 59 项 CTest 与历史吞吐（协调者所有，见 §7），
  以及 native PHY/真实收发/两 RX/首径/timestamp patch/UHD 等 M1-B 范围外项。

F 复审未再发现新的反例；协调者独立复跑同样全绿。


整改后全量串行 CTest 回到 **59 项 58 通过**（唯一失败仍为历史吞吐项），
`uwb_qa_install_consumer` 在重新安装的新前缀下通过（含 M1-B 消费者）。
`qa_uwb_twr_core.cc` 由 21 例增至 **25 例 / 全绿**。

**范围未变**：以上都是类型/生命周期/诊断的收紧，未放宽任何准入，未改历史门槛，
未新增硬件能力；结果仍是 `measurement_valid=false` 的离线协议估计。

