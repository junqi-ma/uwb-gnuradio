# 开发计划：UWB 自发自收 Radar

> 日期：2026-09-04
>
> 依据：[开发需求_UWB自发自收Radar.md](开发需求_UWB自发自收Radar.md)
>
> 原则：逐层开发、每层先定契约再实现、每步都有可独立复现的验证门。

---

## 1. 总体路线

```text
基线与 golden
  → SFD 纯算法
  → SYNC 回推 + CIR 纯算法
  → 失败状态与 radar core 整合
  → PDU 65/48 metadata/坐标契约
  → PacketSource + LoopbackEcho
  → CirEstimator + CirWriter
  → Phase-A 离线端到端
  → EchoTimer 调度 core + fake UHD
  → 真 UHD backend
  → X410 dry-run / 电缆 / OTA
```

每步只新增一类主要不确定性：

| 阶段 | 新增的主要变量 | 暂不引入 |
|---|---|---|
| 纯算法 | SFD/SYNC/CIR 数值 | PMT、线程、UHD |
| PDU 块 | metadata、队列、文件 | 真实射频 |
| EchoTimer fake backend | device-time 调度、partial I/O | X410 |
| UHD backend | UHD API 和传输 | OTA 场景 |
| X410 | 硬件时序与真实 CIR | CFO、RF 动态范围优化 |

---

## 2. 每个开发步骤的固定流程

每一步都按以下顺序执行：

1. 检查当前 branch/HEAD/工作区，不覆盖用户现有改动。
2. 若要新增 GNU Radio block，先搜索本机 GNU Radio 源码和本 OOT 的同类块，
   记录为何选 `gr::block`/message PDU，以及 scheduler/message handler 语义。
3. 先加最小失败 QA 或 golden comparison，再实现使其通过。
4. 只构建和运行当前相关 target，快速收敛。
5. 当前 QA 通过后运行相关回归；每个阶段末运行全部 CTest。
6. 运行 `git diff --check`，记录输入、参数、结果和未验证范围。
7. 未达到当前步骤的退出条件时，不开始下一步。

纯算法 core 的热路径不分配内存。PMT PDU adapter 无法避免创建输出
PDU，但 scratch 容量和有界队列必须在启动前固定，热路径不得扩容。

---

## 3. Step 0：冻结基线与运行环境

### 工作

- 记录 branch、HEAD、GNU Radio/UHD 版本和现有 dirty files。
- 确认 `gr-uwb/build` 可增量构建，现有 CTest 通过。
- 保存现有 65/48、demod CIR、PacketWriter 相关 QA 结果作为回归基线。
- 不修改现有 demod `CirResult` 和 golden。

### 验证门

- 全部现有 CTest 结果有可归档的 pass/fail 清单。
- 若存在与 Radar 无关的旧失败，在计划记录中单独列出，不把它误认为新回归。

---

## 4. Step 1：固定 TX packet 与 MATLAB golden

### 工作

- 选定默认 profile：code 9、64 SYNC、4z2 SFD、固定合法 PHR/PSDU/FCS。
- 在 998.4 MS/s 生成完整 normal UWB packet，对整包一次性 48/65 得到
  737.28 MS/s native TX；禁止 751 点单 SYNC repeat。
- 导出至少两类 golden：
  - 无信道、无 CFO 的整包 TX；
  - 已知整数/分数时延和复数增益的 RX，包含 SFD 位置、SYNC origin、
    raw CIR 和 normalized CIR。
- metadata 必须记录 dtype、sample rate、index base、SYNC 数、SFD 模式、
  SFD/SYNC 坐标、重采样滤波器和群时延。
- 该步先支持 64 SYNC；32/128 在 Step 4 参数化时加入，避免同时调试
  多个 profile。

### 验证门

- MATLAB 读回 998.4/native 文件后，长度、幅度、SFD 位置与 metadata 一致。
- native 整包重采样回 998.4 后，SFD 和 SYNC 间隔无累计整样点漂移。
- golden 可由一条命令重新生成，脚本不依赖人工编辑 metadata。

### 产物

- `testdata/uwb_radar/` 下的 TX/RX/golden metadata。
- 可重复运行的 MATLAB 导出和核对脚本。

---

## 5. Step 2：SFD 窄窗搜索 core

Step 2 开始前先完成一个不阻塞 Phase-1 的小型重采样评估：

- 用同一 native golden 比较“整窗 65/48 + 现有 CIR”和 MATLAB native
  phase-aware CIR 原型。
- 在统一 delay axis 上记录峰位、复增益和主瓣差异。
- 测量实际约 75 µs PDU 的 65/48 mean/P95/P99。
- 评估默认不改变 Phase-1 路径；只有 native 原型数值通过且显示
  显著端到端收益时，才单独变更需求。详见
  [评估_UWB_Radar_CIR重采样必要性.md](评估_UWB_Radar_CIR重采样必要性.md)。

### 调度语义

该步只做纯 C++ core，不是 GNU Radio block；输入是完整 998.4 CF32 窗、
预测 SFD 位置和有界 margin，输出是 SFD start/metric/status。

### 工作

- 从现有 demod SFD 实现抽取最小可复用部分，不复制整个 demod pipeline。
- 只在 `[predicted-margin, predicted+margin]` 内搜索，对所有边界做显式检查。
- 使用复相关幅度/归一化 metric；不做 CFO 估计或补偿。
- scratch 由调用者预分配。

### QA

- 无噪声：SFD start 与 MATLAB/golden 一致。
- 搜索窗左/右边界、SFD 恰在边界、窗口截断。
- 已知时延、复数相位、幅度缩放、AWGN 梯度。
- 删除/替换/破坏 SFD：必须返回 `sfd_failed`，不返回预测位置冒充成功。
- 错误 `sfd_mode`：必须失败或低于门限。

### 退出条件

- 无噪 golden 与 MATLAB 相差不超过 1 个 998.4 MHz 样点。
- 失败用例 100% 返回 `sfd_failed`，不调用 CIR 算法。
- 热路径无动态扩容。

---

## 6. Step 3：SYNC 回推与 CIR core

### 工作

- 由 `sfd_start`、`sync_repetitions`、`samples_per_symbol` 和 SFD 模式回推
  nominal preamble origin。
- 只在小 margin 内校正 SYNC 对齐；输出 `timing_failed` 而不做全窗重搜。
- 移植 `estimateCir.m`：跳过起始 repetitions、对齐相干平均、
  `sampled_code` forward correlation / `code_energy`。
- 同时返回 raw complex taps、L2-normalized taps、peak tap、raw norm和有效
  repetition 数。
- 不做 CFO stage、soft-chip FIR、PHR/payload/FCS。

### QA

- 单路径：整数时延 0/1/窗边界，peak tap 精确一致。
- 分数时延：主峰误差 ≤ 1 tap，normalized CIR 对 MATLAB。
- 多路径：2–4 条复数路径，时延和相对复增益对齐 MATLAB。
- 幅度线性：输入乘 0.1/0.5/2.0，raw CIR 同比例变化，normalized CIR 不变。
- `cir_skip_initial_repetitions`、pre/post tap 边界、截断 repetition、空/过短输入。

### 退出条件

- normalized CIR 复数相对 L2 误差 `<1e-5` 或达到现有 demod golden 同级容差。
- raw CIR 幅度线性测试通过。
- demod CIR 现有 QA 不受影响。

---

## 7. Step 4：整合 `uwb_radar_cir_core`

### 工作

- 实现单次调用契约：

```text
RX window + metadata
  → SFD search
  → SFD success ? SYNC backtrack : sfd_failed
  → timing success ? CIR : timing_failed
  → CIR success ? raw+normalized taps : cir_failed
```

- 定义 `RadarCirResult` 和严格枚举状态，不用自由文本驱动分支。
- 失败时 `tap_count=0`，不返回 seed-only CIR。
- 添加 32/64/128 SYNC 参数化，每种都使用对应完整 packet golden。

### QA / 退出条件

- 通过表驱动 QA 覆盖 `ok/sfd_failed/timing_failed/cir_failed`。
- 32/64/128 SYNC 的 SFD 回推、skip/count 和 CIR 窗不越界。
- 同一输入重复运行结果 bit-stable（容许统计时间字段不同）。

---

## 8. Step 5：补齐 PDU 65/48 metadata 与坐标契约

### 原因

现有 `UwbPduRationalResamplerCcf65_48` 会新建 metadata 字典并选择性保留字段；
Radar 的 pulse/device-time/SFD/校准字段不能默认会透传。

### 工作

- 添加明确白名单：`pulse_id`、`schedule_index`、TX/RX time、
  SYNC/SFD profile、pre/sync/sfd/range guard、`calibration_id`。
- 添加 native→work 坐标映射，记录 FIR delay 和 input/output rate。
- 预分配 SC16→CF32 和 resampler output scratch，不在 handler 内扩容。
- 不改变旧 scheduled-capture metadata 语义。

### QA / 退出条件

- 每个 Radar metadata 字段在重采样后存在且值/坐标正确。
- SC16 极值、短 PDU、guard 不足、错误 rate 和空 PDU 覆盖。
- 现有 `qa_uwb_pdu_rational_resampler` 全绿，真实 scheduled dump 的回归不变。
- 基于当前约 75 µs Radar RX 窗做单包耗时和 P95 记录。

---

## 9. Step 6：`UwbRadarPacketSource` 与 `UwbLoopbackEcho`

### block type / scheduler

- 两者均为 `gr::block`，0 流端口，使用 message PDU。
- PacketSource 只加载/生成完整波形，不用 host timer 产生 PRI。
- LoopbackEcho 是纯软件信道，该步不包含 UHD。

### 工作

- PacketSource 校验 sample rate、dtype、SYNC/SFD profile、packet 长度和 PRI 容量。
- LoopbackEcho 支持整数/分数延时、多路径复增益、AWGN 和固定 random seed。
- LoopbackEcho 输出与 EchoTimer 相同 metadata schema，但注明 `source=loopback`。
- 最大 TX/RX 窗和通道数在 `make()` 固定；超限输入失败而不扩容。

### QA / 退出条件

- 文件 TX 读回 bit-exact；metadata 与 golden 一致。
- 整数时延输出逐样点相等；分数/多径输出与 MATLAB 信道 golden 对齐。
- 固定 seed 噪声输出可重复。
- invalid profile/window 产生明确 status，不发半包 PDU。

---

## 10. Step 7：`UwbRadarCirEstimator` message block

### block type / scheduler

- `gr::block`，0 流端口，`rx` PDU 输入，`cir`/`status` 消息输出。
- handler 只校验和入有界队列；worker 调用 `uwb_radar_cir_core`。
- 第一版只用一个 worker，保证 pulse 顺序；多 worker 优化必须后续带重排序 QA。

### QA

- valid PDU 输出 raw+normalized CIR 和完整 lineage metadata。
- `sfd_failed/timing_failed/cir_failed` 只发 status/空 CIR metadata，不发 taps。
- 错误 dtype/rate/profile、缺 metadata、过短 PDU。
- 队列满的 drop policy、计数器、high-watermark、stop/restart/析构无死锁。
- 连续 pulse IDs 的输出顺序与输入一致。

### 退出条件

- core 直接调用与 message block 输出的 taps/status 一致。
- 200 pulse/s 目标下单 worker 平均服务时间明显低于 5 ms，并记录 P95/P99。

---

## 11. Step 8：`UwbCirWriter` 与 MATLAB reader

### block type / scheduler

- `gr::block`，0 流端口；handler 入有界队列，单 writer worker 顺序写盘。
- 不在 handler 执行文件 I/O。

### 工作

- 实现 `cir.cf32`、可选 `cir_norm.cf32`、`cir.jsonl`、`run.json`。
- 成功帧写 taps 并推进 `file_offset_taps`。
- 失败帧只写 JSONL，`tap_count=0`，offset 不推进。
- MATLAB reader 按 pulse ID/offset/tap count 读回 raw/normalized CIR。

### QA / 退出条件

- ok 与三种 failed status 混合 10 帧；JSONL 恰好 10 行。
- raw/normalized 文件的 byte size、offset、little-endian complex64 逐项检查。
- MATLAB 读回后 bit-exact 或 float32 exact；损坏/截断文件明确报错。
- stop 后队列排空，JSONL 与 binary 不出现半帧偏移。

---

## 12. Step 9：Phase-A 离线端到端

### 路径

```text
native normal packet
  → LoopbackEcho(native channel)
  → PDU 65/48
  → RadarCirEstimator
  → CirWriter
  → MATLAB reader/compare
```

另保留一条 998.4 直通路径，用于区分 CIR 算法错误和重采样坐标错误。

### 验证矩阵

- 998.4 direct：验证 SFD/CIR 算法本身。
- native 737.28 → 998.4：验证群时延和 metadata 映射。
- 整数时延、分数时延、多径、AWGN、复相位。
- 32/64/128 SYNC。
- 正常 SFD 与缺失/错误 SFD。
- 10/100/1000 帧：条数、offset、顺序、drop、RSS 和队列水位。

### Phase-A 退出条件

- SFD/SYNC origin/peak 坐标均在契约容差内。
- normalized CIR 对齐 MATLAB，raw CIR 幅度线性正确。
- SFD 失败帧无 CIR taps，但 JSONL 不缺帧。
- 200 pulse/s 节拍的 30 s 软件回放 0 drop，队列不持续增长。
- 全部 CTest 通过。

Phase A 只代表算法、PDU 和文件契约完成，不代表 UHD/X410 完成。

---

## 13. Step 10：EchoTimer 调度 core + fake backend

### 实现前调查

- 重新检查本机 GNU Radio UHD sink/source 的 timed command/SOB/EOB 实现。
- 检查 `gr-radar/usrp_echotimer_cc`，只参考语义，不照搬其单次 send/recv
  和 work 内建/join 线程。
- 确认 message-only `gr::block` + 专用 worker 的启停语义。

### 先抽象 backend

```text
EchoScheduler
  → IRadioBurstBackend
       ├─ FakeBurstBackend    # CI
       └─ UhdBurstBackend     # Step 11
```

### 工作

- device-time 栅格：`t_tx=t0+k*PRI`、`t_rx=t_tx-pre_guard`。
- 已过期时跳到下一个未来栅格，不追赶。
- Fake backend 可配 max fragment size，强制 partial TX/RX、timeout、overflow、late、
  broken-chain 和 stop-during-I/O。
- 调度器输出每个 schedule index 恰好一个 RX 或失败 status。

### QA / 退出条件

- 时间栅格无累计 double 漂移，优先用整数 tick 计算。
- RX command 早于 TX；SOB/time-spec 只在首 fragment，EOB 在末 fragment。
- partial send/recv 拼接后样点逐点正确。
- 所有 UHD-like error 转为每帧 status，worker 不静默退出。
- stop/restart/析构可在有 pending I/O 时完成，不死锁。

---

## 14. Step 11：真实 UHD backend（无目标物验收）

### 工作

- `find_package(UHD)` 可选编译；无 UHD 环境仍能构建 Phase A 和 fake backend QA。
- 同一 `multi_usrp` 创建 TX/RX streamers，显式配置 SC16/rate/channel/time source。
- 实现 partial send/recv 循环、TX async metadata 和 RX error metadata。
- 启动时校验实际 rate 严格为 737.28 MS/s，不接受静默 coercion。

### 无设备验证

- CMake UHD ON/OFF 两种构建路径。
- `--dry-run` 输出完整 TX/RX/rate/window/PRI 契约，不打开射频。
- UHD backend 的参数校验、错误映射继续用 fake adapter QA。

### 退出条件

- 无 UHD 机器 CTest 不减少 Phase-A/fake 覆盖。
- 有 UHD 但无 USRP 时应明确报“device unavailable”，不崩溃/卡死。

---

## 15. Step 12：X410 分层验证

RF 动态范围优化不在当前需求内，但仍按可观测层级逐步前进：

1. **device/time smoke**：查询 rate/time/channel，不发射。
2. **单帧 timed burst**：只检查 late/timeout/overflow 和 RX 样点数。
3. **低速多帧**：1–10 pulse/s，检查 schedule index、timestamp 和文件数量。
4. **目标 200 pulse/s**：30 s，检查 late/drop/overflow/队列水位。
5. **零距离校准**：记录 `zero_delay_tap`、分数 delay 和 `calibration_id`。
6. **SFD/CIR 电缆观测**：统计 `ok/sfd_failed/timing_failed/cir_failed`。
7. **OTA 试验**：只记录观测，不在本阶段宣称 RF 动态范围验收。

每一层失败都停留在当层，不用后续 OTA 现象掩盖 timed-I/O 或
SFD/CIR 契约错误。X410 soak/overflow 恢复仍需单独测试报告。

---

## 16. 建议的提交/评审边界

| 提交 | 内容 | 必须附带的证据 |
|---|---|---|
| C1 | TX packet + MATLAB golden | 生成命令、metadata 核对 |
| C2 | SFD core | SFD success/failure QA |
| C3 | SYNC/CIR core | MATLAB complex CIR comparison |
| C4 | Radar core + 32/64/128 | 四状态矩阵 |
| C5 | PDU 65/48 metadata | 坐标/字段回归 |
| C6 | PacketSource + Loopback | 逐样点/多径 QA |
| C7 | CirEstimator | 队列/顺序/性能 QA |
| C8 | CirWriter + MATLAB reader | 混合成败帧读写 QA |
| C9 | Phase-A e2e | 30 s / 200 pulse/s / MATLAB report |
| C10 | EchoScheduler + fake backend | partial/error/lifecycle QA |
| C11 | UHD backend + dry-run | UHD ON/OFF build + dry-run |
| C12 | X410 app/report | 分层硬件记录 |

不建议把 C2–C9 合并成一个大提交；那会让 SFD 坐标、CIR 数值、
PMT metadata 和 writer offset 问题无法独立定位。

---

## 17. 开发完成定义

### 算法完成

- SFD 硬门槛、SYNC 回推、raw/normalized CIR 全部对齐 golden。
- CFO 完全不在 pipeline 中。

### Phase A 完成

- native loopback 端到端、writer/MATLAB reader、200 pulse/s 软件回放和全部
  CTest 通过。

### EchoTimer 软件完成

- fake backend 覆盖全部调度、partial I/O、error 和 lifecycle。
- UHD ON/OFF 均能构建，dry-run 通过。

### 硬件状态

- 只按实际完成层级声明 smoke/单帧/200 pulse/s/校准/电缆/OTA。
- 未完成 X410 soak、overflow 恢复或 RF 动态范围验收时，必须明确写为未验证。

---

## 18. QA 命名与建议执行方式

### 计划新增的 QA

| 层 | QA |
|---|---|
| SFD core | `qa_uwb_radar_sfd_core` |
| CIR/core 整合 | `qa_uwb_radar_cir_core` |
| PDU metadata | 扩展 `qa_uwb_pdu_rational_resampler` |
| PacketSource | `qa_uwb_radar_packet_source` |
| Loopback | `qa_uwb_loopback_echo` |
| Estimator | `qa_uwb_radar_cir_estimator` |
| Writer | `qa_uwb_cir_writer` |
| Scheduler/fake backend | `qa_uwb_echo_scheduler` |
| EchoTimer adapter | `qa_uwb_echo_timer` |

### 每次有意义修改后

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build -R '<当前QA|相关回归>' --output-on-failure
git diff --check
```

### 每个 Step 退出前

```bash
ctest --test-dir gr-uwb/build --output-on-failure
```

MATLAB 对照不作为每次 C++ 单元测试的运行时依赖；由 MATLAB 生成并
审核 golden，C++ CTest 读取已入库 golden。任何 golden 变更都必须同时给出
生成命令、差异原因和 MATLAB 复核结果。
