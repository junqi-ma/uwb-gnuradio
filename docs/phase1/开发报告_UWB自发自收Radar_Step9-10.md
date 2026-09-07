# 开发报告：UWB 自发自收 Radar Step 9–10

> 日期：2026-09-05
> 分支：`feature/uwb-monostatic-radar`
> 依据：[开发计划_UWB自发自收Radar.md](开发计划_UWB自发自收Radar.md) Step 9（§12）、
> Step 10（§13）、[开发需求_UWB自发自收Radar.md](开发需求_UWB自发自收Radar.md)
> 状态：**Step 9（Phase-A 离线端到端）与 Step 10（EchoTimer 调度 core +
> fake backend）完成；含 Step 10 正式验收反馈整改（第 8 节：有界输入/
> 固定 scratch、移除不可用的 GRC/Python 暴露）与 Step 9 验收整改
> （R3–R5：loopback native 速率 + capture/window_start meta、坐标约定
> 统一、O(N) MATLAB 校验器、10/100/1000 帧矩阵）。全量 CTest 27/27
> 通过；MATLAB R2025b 14 个输出目录全部 ALL CHECKS PASSED，含 soak
> 目录全量 6000 帧 O(N) 读回与 missfd/gain 跨目录线性（第 4 节）。**
> 未改 [`../../开发状态.md`](../../开发状态.md)。未提交。
> 未开始 Step 11（UHD backend）/ Step 12（X410）。Phase A 只代表算法、PDU
> 与文件契约完成，不代表 UHD/X410/OTA 完成。

---

## 1. 复验结论

| 项目 | 结论 |
|---|---|
| 新增 QA | `uwb_qa_uwb_radar_e2e.cc`（9 case，含 R5 的 10/100/1000 帧矩阵）、`uwb_qa_uwb_echo_timer.cc`（9 case，含整改新增）|
| 新 QA 结果 | 2/2 Passed（e2e 含真实 30 s wall-clock soak 与帧数矩阵）|
| 全量 CTest | **27/27 Passed**（含 Step 1–8 全部回归；整改后复跑 2 轮均通过。注：`uwb_qa_uwb_auto_scheduled_extractor_sc16.cc` 为既有时序敏感 QA，曾出现一次单次 flake，standalone 与复跑均通过，与 Step 10 无关）|
| `git diff --check` | 干净 |
| MATLAB 读回验证 | R2025b（`/mnt/f/MATLAB`，WSL interop headless `-batch`）；**14 个输出目录全部 ALL CHECKS PASSED（exit 0）**，含 soak 全量 6000 帧（非抽样）O(N) 读回 PASS 1/2/3/4/6/7（验收人独立实跑 + 本会话复跑一致，第 4 节）、missfd 失败帧目录与 gain05↔gain20 跨目录原始 CIR 幅度线性 4.000000/0.250000 |
| 200 pulse/s 30 s soak（Phase-A 退出条件） | **6000 帧、0 drop**、estimator 队列水位 2（容量 64）、post 墙钟 29995 ms、RSS 漂移 748 KB（断言 < 200 MB）|
| 未宣称 | UHD backend、X410、OTA、EchoTimer 真实射频均未完成 |

---

## 2. Step 9 — Phase-A 离线端到端

### 2.1 路径（全部真实块，无测试侧 metadata 覆写）

```text
直通：UwbRadarPacketSource(998.4) → UwbLoopbackEcho(998.4 信道)
      → UwbRadarCirEstimator → UwbCirWriter
native：UwbRadarPacketSource(737.28) → UwbLoopbackEcho(native 信道)
      → UwbPduRationalResamplerCcf65_48(validate_input_rate=true)
      → UwbRadarCirEstimator(998.4) → UwbCirWriter
```

块间全部走 message PDU（`tx→rx→packet→rx→cir`）。**本次 Step 9 整改
（R3-1/R3-2）为 `UwbLoopbackEcho` 新增 native 737.28 MS/s 支持**：
`validate_loopback_profile` 由"仅接受 work 998.4"改为同时接受 native
737.28（native SYNC/SFD span 以 `ceil(work×48/65)` 校验，对应 32/64/128
reps 的 24009/48018/96036 约定），延时/增益/AWGN 仍按输入栅格的采样序号
域作用，校准延时以 `calibration_delay_native_samples`（native 栅格）报告；
输出 meta 新增 `window_start_sample=0 / capture_samples=tx 长度 /
post_guard_samples=tail`，满足 resampler 的
`pre_guard + capture ≤ sample_count` 契约（此前非零 pre_guard 会被
`invalid_metadata` 拒绝）。resampler 侧 `calibration_delay_work_samples =
native × 65/48`（37 → 50.104）与 `window_start/pre_guard/map()` 索引映射由
`uwb_radar_pdu_meta.h` 提供；estimator 侧按统一坐标约定消费（见 §5.1，
本次同步统一：预测纳入 `window_start_sample`、`zero_delay_tap = cir_pre`）。

### 2.2 验证矩阵覆盖（`qa_uwb_radar_e2e.cc` 9 case）

| case | 覆盖 |
|---|---|
| `e2e_998p4_direct` | 直通链 ok；SFD/SYNC origin/peak 坐标全部落在契约容差（`sfd−predicted≤2`、`preamble−cir_origin≤2`、`zero_delay_tap=16`、`peak−zero_delay=+2`）；taps 与直接 `radar_cir_one` **逐 bit 一致**；clean 帧与 MATLAB golden CIR `rel_l2<1e-5`；JSONL/raw/norm bytes 一致 |
| `e2e_native_65_48` | **真实顺序 native 链 ok**（无测试侧覆写、`validate_input_rate=true`）；resampler 输出 meta 逐键断言（sample_rate 998.4、`window_start=map(0)=28`、`pre_guard=map(1475)−map(0)`、`capture=map(pre+140982)−map(pre)`、`cal_work=50.104`、profile 透传）；真值 `map(1475+37+48018)=67100` 与 predicted **±2**（坐标统一后无系统性 FIR 头偏置）；`preamble−cir_origin ≤2`；`peak−zero_delay=+2`；taps 与直接 `radar_cir_one` 逐 bit 一致 |
| `e2e_conditions_matrix` | 分数延时(PCHIP)+多径复增益+AWGN(固定种子)+复相位 一次组合 → ok；raw CIR 增益线性 0.5↔2.0（比值 4，rel<1e-4），normalized CIR 不变（rel_l2<1e-6） |
| `e2e_sync_repetitions` | 32/64/128 SYNC per-profile golden（`packets/sync{32,128}/`）→ 全部 ok；valid_repetitions 22/54/118（128 为 block auto=reps−skip，区别于 MATLAB golden 的显式 54，已在 QA 注释中说明） |
| `e2e_missing_sfd` | 正常 SFD vs 静默 RX → ok 与 sfd_failed；失败帧无 taps 但 JSONL 不缺帧（顺序化投递，跨块次序确定）。（清零 SFD 区域不作负例——幅度相关的假峰，Step 7-8 报告 §6）|
| `e2e_10_frames` | 计划 §12 帧数矩阵 10 帧：全链行数/顺序/offset 单调/全块 0 drop/水位/RSS/逐帧坐标一致性 |
| `e2e_100_frames` | 计划 §12 帧数矩阵 100 帧（1 warm-up + 100）：同上断言；水位=1、RSS 漂移 < 上界 |
| `e2e_1000_frames` | 计划 §12 帧数矩阵 1000 帧：同上断言（实测 wall ≈3.5 s，水位 2，RSS 漂移 0） |
| `e2e_soak_200pps_30s` | 真实墙钟 30 s @200 PDU/s（6000 帧）：**0 drop**、written=6000/failed=0、水位 2（容量 64）、post 墙钟 29995 ms（≥ 30 s−100 ms 诚实节拍）、RSS 漂移 0；`UWB_E2E_SOAK_SECONDS` 环境变量可调（默认 30） |

计划 §12 的 10/100/1000 帧矩阵由 `e2e_10_frames` / `e2e_100_frames` /
`e2e_1000_frames` 独立用例显式覆盖（同一 `run_frame_matrix` 断言集：
条数、offset、顺序、drop、RSS、队列水位），6000 帧 soak 为其上的
Phase-A 退出条件增强，不替代矩阵。

### 2.3 Phase-A 退出条件逐项

- [x] SFD/SYNC origin/peak 坐标在契约容差内（±2 work samples）；
- [x] normalized CIR 对齐 MATLAB（L2≈1 逐帧复核），raw CIR 幅度线性正确（增益比值 4.000 精确成立）；
- [x] SFD 失败帧无 CIR taps，JSONL 不缺帧；
- [x] 200 pulse/s × 30 s 软件回放 6000 帧 0 drop，队列水位 2 不增长；
- [x] 全部 CTest 通过（27/27）。

---

## 3. Step 10 — EchoTimer 调度 core + fake backend

### 3.1 实现前调查（本机源码确认）

- **gr-uhd 语义**（`usrp_sink_impl` / `usrp_source_impl`）：TX 由
  `tx_time`/`tx_sob`/`tx_eob` stream tags 转为 `tx_metadata`（SOB 与
  time-spec 必须在首包、EOB 终结）；RX timed burst =
  `issue_stream_cmd(NUM_SAMPS_AND_DONE)` + 非 NOW 的 time spec，完成点是
  阻塞 `recv()`，错误经 `rx_metadata_t::error_code` 上浮。GR 自身还在
  start/stop 时发空 SOB/EOB 包规避 late。
- **gr-radar `usrp_echotimer_cc`**：每次 work() 内 spawn 发送/接收并行线程
  并 join——**只参考语义，不复制**（单次 send/recv、work 内建线程的模式
  与本仓库 handler 只入队/worker 做 I/O 的约定冲突）。
- message-only `gr::block` + 专用 worker 的启停语义复用 Step 7/8 的
  estimator/writer 生命周期（幂等 start、stop 排空、析构安全 join）。

### 3.2 分层

```text
UwbRealtimeEchoTimer (gr::block, 0 流端口)
  └─ 单 worker：EchoGrid 设备时刻调度 core → IRadioBurstBackend
       ├─ FakeBurstBackend（CI，注入故障）
       └─ UhdBurstBackend（Step 11，未实现）
```

### 3.3 `uwb_echo_scheduler_core.h`（header-only）

- **整数 tick 栅格**：PRI 用精确有理数 `pri_num/pri_den`（默认
  0.005 s × 737.28e6 = 3686400 tick），`t_tx = t0 + k*PRI` 全整数
  商/余数状态机，无 double 累积（QA 以 k=100000 验证零漂移）；
- `next_schedule(now)`：已过期槽位跳到第一个未来槽（不追赶，
  `max_catchup_slots` 封顶，超限 → `grid_error` 并 disarmed）；
- `t_rx = t_tx − pre_guard_ticks`，**RX 命令先于 TX**；
- `plan_fragments()`：按 `max_fragment_size` 规划 TX/RX fragment 列表，
  首 fragment 携带 time-spec+SOB、末 fragment 携带 EOB；checked math
  复用 `uwb_radar_checked_math.h`（本步顺带修复其缺失
  `#include <cstddef>` 的隐患）；
- `prepare_echo_scheduler()` 一次冻结配置（对齐 Step 3 scratch-freeze 模式）。

### 3.4 `IRadioBurstBackend` / `FakeBurstBackend`

- 接口：`prepare / issue_rx(先) / issue_tx / collect_result(cv 等待) /
  abort_rx / request_stop`；`BurstStatus` ∈
  `ok/partial_handled/late_command/timeout/overflow/broken_chain/
  stop_during_io/backend_error`。
- Fake backend：`max_io_chunk` 强制 partial TX+RX（re-issue 循环，QA 记录
  每 burst 的 re-issue 次数）；逐 burst 一次性注入 late / timeout /
  overflow / broken-chain / stop-during-I/O；记录每 burst 的 flag 序列与
  **拼接后的 TX/RX 样点序列**（QA 逐点比对）。

### 3.5 `UwbRealtimeEchoTimer` 块

- `gr::block` 0 流端口；`schedule` 入、`burst`/`status` 出；backend 依赖
  注入（CI 注入 fake，Step 11 注入 UHD）；
- handler 只校验+入有界队列（溢出 → drop + status），worker 单线程按
  schedule index 顺序；**每个 schedule index 恰好一条 burst 结果**
  （ok 携带拼接 SC16 RX 数据，失败携带 `uhd_error` 空数据）；
- **有界输入 / 固定 scratch（验收整改后契约）**：`make()` 以
  `max_tx_samples`/`max_rx_samples` 为固定上限（>0 且 `2*cap` 不溢出
  size_t，违反即 `invalid_argument`）；RX scratch 在**构造时一次分配**
  `max_rx_samples*2` 个 int16，此后容量与地址永不变化（重启亦然），
  worker 仅对使用前缀做 `std::fill` 清零、**不得扩容**；handler 严格
  拒绝：0/超上限/负值/超出 int64 的 `t0_ticks`/`2*tx_samples` 不可
  表示/payload 长度不匹配，全部计入 `schedules_invalid` 不入队；
  worker 侧另有一道防御性 cap 复查；
- 输入 dict 键：`t0_ticks/tx_samples/rx_samples`（必需）、
  `schedule_index/pulse_id/burst_count/max_fragment_size/sample_rate`
  （可选，见头文件完整契约）；
- 生命周期：幂等 start（计数清零）、stop 先 `request_stop()` 后排空 join、
  析构安全；stop-during-I/O 以 `stop_during_io` 结果帧呈现，worker 不静默
  退出。

### 3.6 `qa_uwb_echo_timer.cc`（9 case）

| case | 覆盖 |
|---|---|
| checked math 自包含 | `uwb_radar_checked_math.h` 作为 TU 首 include 的回归闸门 + `radar_i64_from_size/mul/add` 正常/溢出语义 |
| tick grid | k=100000 分数 PRI 零累积漂移；边界等值计为过期；skip 上限；prepare 校验；fragment 规划 |
| ok bursts | 每 index 恰一条结果、顺序、tick 值、RX 先于 TX、单 fragment SOB\|TIME\|EOB flag 序列、拼接 RX 逐点 |
| partial stitch | 小 `max_io_chunk`：精确的逐次调用 flag 序列（首 SOB\|TIME，末 EOB）、tx 5 次 / rx 6 次 re-issue、TX+RX 逐点 |
| fault matrix | late/timeout/overflow/broken-chain/stop-during-io 注入后 worker 存活、每 index 一条失败结果、后续 ok 正常 |
| late slot skip | 块级过期槽跳到未来槽（不追赶）|
| stop lifecycle | collect 阻塞中 stop：无死锁、`stop_during_io` 结果、restart 计数复位 |
| destructor | 有 pending I/O 时析构安全 |
| bounded scratch（整改新增） | 6 类非法 PDU 拒绝（tx/rx 超上限、uint64 上界、负值、`t0` 超出 int64）；上限边界值通过；不同长度连续调度下 RX scratch 容量/地址不变；restart 后不变；`make()` 对 0/2 倍溢出 cap 抛 `invalid_argument` |

---

## 4. MATLAB 实际运行验证（Step 9 收尾）

先在 WSL 内跑 `qa_uwb_radar_e2e`（CTest #27 / 独立二进制）产出真实 writer
目录，再以 `/mnt/f/MATLAB/bin/matlab.exe -batch`（UNC 路径）调用
`testdata/uwb_radar/verify_e2e_cir.m`。**该脚本为 O(N) 实现**：每个目录
仅做一次 JSONL 解析（`read_uwb_cir(dir, [])` 批量行），`cir.cf32` /
`cir_norm.cf32` 各打开一次、按每帧 `(file_offset_taps, tap_count)`
`fseek/fread` 读回——6000 帧目录数秒内完成，修复了此前"每 pulse 调
`read_uwb_cir` 重复解析全 JSONL"的 O(N²) 实现（R4）。

### 4.1 全量 6000 帧实跑（非抽样）

```matlab
addpath('\\wsl.localhost\Ubuntu-22.04\home\junqima\workspace\uwb-gnuradio\testdata\uwb_radar');
verify_e2e_cir('\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_radar_e2e_soak', 6000);
```

实际输出（验收人独立实跑与本会话复跑一致，逐项 PASS）：

```text
PASS 1: 6000 JSONL lines, pulse_id 0..5999 in order
PASS 6: 0 failed frames (), others ok
PASS 2: tap_count contract and running offsets for all 6000 frames
PASS 3: cir.cf32 size and per-frame offsets read back (696000 taps)
PASS 7: |sfd-predicted|<=2, |preamble-cir_origin|<=2, zero_delay==cir_pre (all ok frames)
PASS 4: normalized CIR L2 norm ~= 1 for all 6000 ok frames
verify_e2e_cir: ALL CHECKS PASSED for ...uwb_qa_radar_e2e_soak (6000 frames)
```

落盘核对：`cir.jsonl` 6000 行、`cir.cf32`/`cir_norm.cf32` 各
5,568,000 B（= 6000 × 116 × 8，complex64 LE），与 C++ QA 的
written=6000/failed=0 计数器一致。**全量 6000 帧、非抽样。**

### 4.2 其余输出目录

同一 O(N) 校验器批量跑 14 个目录（`verify_all_e2e` 驱动），**14/14 全部
ALL CHECKS PASSED（exit 0）**：

| 目录 | 帧数 | MATLAB 结果 |
|---|---|---|
| soak | 6000 | ALL CHECKS PASSED（§4.1，PASS 1/2/3/7/4）|
| f10 / f100 / f1000 | 11/101/1001 | ALL CHECKS PASSED（PASS 1/2/3/7；无 norm 文件按约定跳过 PASS 4）|
| direct / native / direct_d37 / cond | 各 1 | ALL CHECKS PASSED（含 PASS 4 norm L2≈1）|
| sync32 / sync64 / sync128 | 各 1 | ALL CHECKS PASSED（含 PASS 4）|
| missfd | 2 | ALL CHECKS PASSED（PASS 6 = `1 failed frames (sfd_failed), others ok`）|
| gain20 ↔ gain05 | 各 1 | ALL CHECKS PASSED，**跨目录原始 CIR 幅度线性** PASS 5 双向：`|gain20 A|/|gain05 B| = 4.000000 ~= 4`、`|gain05 A|/|gain20 B| = 0.250000 ~= 0.25` |

校验脚本 PASS 7 的行列形状缺陷已修复：所有经 `[metaAll.x]` 抽取的
metadata 向量（`pulse_id/tap_count/sfd/predicted/preamble/cir_origin`）
统一以 `(:)` 列化，消除与列向量 `okMask/tapCount` 的隐式扩展；并新增
可选第 7 参 `refDir` 的跨目录线性模式（GROUPA 取 writerOutDir、GROUPB
取 refDir，两侧各打开一次 `cir.cf32` 按 offset 读回，O(#ids)）。修复后
missfd（期望 pulse 1 为 sfd_failed）与 gain05/gain20 跨目录线性均实跑
PASS，soak 6000 帧复跑无回归。

---

## 5. 观察与遗留（未改行为，仅记录）

> 原报告 §5.1（"native 路径 +29 work-sample FIR 头偏置、坐标约定待统一
> 评审"）与 §5.2（"loopback 无 capture_samples"）**已失效并删除**：
> 本 Step 9 整改将坐标/校准约定统一（estimator 预测纳入
> `window_start_sample`、`zero_delay_tap = cir_pre`，见 §2.1），全量
> MATLAB 6000 帧读回 PASS 7 证明 |sfd−predicted|≤2 且
> |preamble−cir_origin|≤2；`UwbLoopbackEcho` 现输出
> `window_start_sample/capture_samples/post_guard_samples`，native 链
> 以真实 pre_guard=1475（2 µs）运行并被 resampler 接受。

1. **EOB 语义（Step 11 注意）**：fragment 级契约把 EOB 放在"最后一个
   fragment 的每次调用"上；真实 UHD `send()` 需要在**实际完成突发的那次
   调用**上发 EOB——UhdBurstBackend 映射时必须处理 partial 尾包。
2. **TSan 未跑**：本机对未插桩的系统 GR 库 TSan 启动即崩；echo timer
   QA 为竞态敏感代码，普通模式连续 6 轮通过。
3. `qa_uwb_radar_e2e` 的 soak 默认 30 s（`UWB_E2E_SOAK_SECONDS` 可调），
   CTest 全量约 34 s。

---

## 6. 文件清单

| 动作 | 文件 |
|---|---|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_echo_scheduler_core.h`（设备时刻 tick 栅格 core）|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_echo_burst_backend.h`（IRadioBurstBackend 抽象）|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_fake_burst_backend.h`（CI fake backend）|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`、`gr-uwb/lib/uwb_realtime_echo_timer.cc`（EchoTimer 块）|
| 新增 | `gr-uwb/lib/qa_uwb_echo_timer.cc`（9 case）|
| 新增 | `gr-uwb/lib/qa_uwb_radar_e2e.cc`（9 case）|
| 新增 | `testdata/uwb_radar/verify_e2e_cir.m`（O(N) MATLAB 读回归核：单次 JSONL 解析 + 二进制按 offset 读回；跨目录增益线性模式）|
| 修改 | `gr-uwb/lib/CMakeLists.txt`、`gr-uwb/include/gnuradio/uwb/CMakeLists.txt`（EchoTimer 源/头/CTest 接入）|
| 修改 | `gr-uwb/include/gnuradio/uwb/uwb_radar_checked_math.h`（补 `#include <cstddef>`）|
| **移除** | ~~`gr-uwb/grc/uwb_realtime_echo_timer.block.yml`~~、~~`python_bindings.cc` 中 `realtime_echo_timer`/`echo_scheduler_config` 绑定~~、`grc/CMakeLists.txt` 对应安装行（验收 R2：不可保留不可用接口，详见 §8）|

---

## 7. 未做 / 未验证

- Step 11 `UhdBurstBackend`（partial send/recv、async metadata、rate 严格
  校验、dry-run）未开始；
- Step 12 X410 分层验证、soak/overflow 恢复未开始；
- EchoTimer 仅经 fake backend 验证，未在真实 UHD/USRP 上运行；
- **EchoTimer 的 GRC YAML 与 Python 绑定未提供（Step 10 无此要求，
  见 §8 R2）**：计划中 GRC/Python 属 Phase B（PR5/PR6）与 Step 11/12；
  待 Step 11 `UhdBurstBackend` 工厂定型后一并补齐，避免现在暴露依赖
  注入 backend 的不可实例化接口；
- TSan/ASan 未在本环境完成（系统库未插桩）；
- `uwb_qa_uwb_auto_scheduled_extractor_sc16.cc` 存在偶发单次 flake
  （standalone/复跑均通过，与本步无关，待后续排查）；
- 未提交、未 push；未改 `开发状态.md`。

---

## 8. Step 10 正式验收整改记录（R1/R2）

### R1 有界内存 / 溢出（已整改）

问题：handler 将 uint64 `tx/rx_samples` 转 `size_t` 后乘 2 无范围与
乘法检查；worker 每个 schedule `d_rx_buf_.assign(rx_samples*2)` 会按
输入扩容，可被负/超大 PMT 值回绕或 OOM。

整改：

1. `make()` 新增固定上限 `max_tx_samples`/`max_rx_samples`（默认
   `1<<20`；>0 且 `2*cap <= SIZE_MAX`，违反抛 `invalid_argument`）；
2. RX scratch 在**构造时一次分配**（`max_rx_samples*2` int16），构造/
   start/重启全程容量与地址不变（QA 断言）；worker 仅对使用前缀
   `std::fill` 清零，`apply_schedule` 不再有任何 `assign/resize`；
3. handler 严格拒绝：0、超上限、超出 `size_t/2`（2 倍溢出）、负值
   PMT、`t0_ticks` 超出 int64、payload 元素数与 `2*tx_samples` 不符，
   全部计 `schedules_invalid` 不入队；worker 侧另有防御性 cap 复查；
4. QA 新增 `test_echo_timer_bounded_scratch`：6 类非法 PDU 逐一拒绝且
   无 burst；上限边界值（tx=cap、rx=cap）正常出 ok 结果；不同长度
   连续调度 + restart 后 scratch 容量/地址不变；`make()` 非法 cap
   抛异常。阴性验证：用 HEAD 版（无 `<cstddef>`）的
   `uwb_radar_checked_math.h` 单独编译确认会失败，闸门有效。

### R2 GRC / Python 暴露失效（已按"移除"整改）

问题：`uwb_realtime_echo_timer.block.yml` 引用未定义的 `backend`，
`echo_scheduler_config` 无 kwargs 构造，Python 侧无
`IRadioBurstBackend`/`FakeBurstBackend` 类型，复现
`uwb.echo_scheduler_config(pri_num=1) -> TypeError`。

整改（选择"移除本 Step 不需要的 broken 条目"）：

- 删除 `gr-uwb/grc/uwb_realtime_echo_timer.block.yml` 及
  `grc/CMakeLists.txt` 安装行；
- `python_bindings.cc` 删除 `bind_realtime_echo_timer`（含
  `echo_scheduler_config` 类）与对应 include/注册；**Step 9 的
  `radar_cir_estimator`/`cir_writer` 绑定保持不动**；
- 验证：全量重编后
  `PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib
  python3 -c "from gnuradio import uwb; ..."`——
  `hasattr(uwb,'realtime_echo_timer')/hasattr(uwb,'echo_scheduler_config')`
  均为 False，Step 9 绑定可实例化（`uwb.cir_writer(...)` 正常）；
- 依据：开发计划 §13 Step 10（C10）交付为 core+fake backend+QA；
  GRC/Python 属 Phase B（PR5/PR6）与 Step 11/12。EchoTimer 工厂依赖
  注入 `IRadioBurstBackend`，待 Step 11 `UhdBurstBackend` 定型后随
  fake/UHD backend 类型一并补齐绑定与 GRC，不提前暴露不可用接口。

### 整改后验证（gr-uwb/build 实际运行）

```text
$ cmake --build gr-uwb/build -j$(nproc)                       # 全量，0 error
$ LD_LIBRARY_PATH=gr-uwb/build/lib gr-uwb/build/lib/uwb_qa_uwb_echo_timer.cc
  → Running 9 test cases... *** No errors detected            # 9/9，重复 3 次稳定
$ ctest --test-dir gr-uwb/build --output-on-failure
  → 100% tests passed, 0 tests failed out of 27               # 复跑第 2 轮亦 27/27
$ git diff --check                                            # exit=0
$ python3（broken-binding 复现命令）                           # hasattr 均为 False → 接口已移除
```
