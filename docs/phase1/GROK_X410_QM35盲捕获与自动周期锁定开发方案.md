# Grok 开发方案：X410 从 SC16 盲流捕获 QM35 并自动锁定后续周期包

更新时间：2026-08-14

离线状态（2026-08-14）：mixed 737.28 文件上盲 t0 + 已知 5 ms 的最小闭环已通，
见 [`测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md`](测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md)。
§18 硬件完成定义尚未满足。锁定后的下一产品步骤是原生 SC16 落盘并加宽头尾
以容纳重叠 DW1000，见
[`下一步_锁定后SC16截取与DW1000头尾预留.md`](下一步_锁定后SC16截取与DW1000头尾预留.md)。
这不是本方案的 SIC，也不替代 §18。

## 1. 目标与边界

目标是在不使用 RFNoC 上采样的前提下，实现以下生产链路：

```text
X410 / UHD SC16 @ 737.28 MS/s
  → 盲捕获第一个可确认的 QM35 包
  → 自动建立 t0/T 周期计划
  → 仅按后续 QM35 slot 截取 SC16 窗口
  → PDU 级 65/48 重采样
  → CF32 PDU @ 998.4 MS/s
  → UwbRealtimeDemodulator
  → timing/FCS 反馈维持锁定或触发重捕获
```

完成后的用户行为应是：只配置 X410、QM35 PHY profile 和周期范围，启动后无需手工扫描
`first_packet_sample`。系统必须从任意流起点自动找到 QM35，保留首个确认包，并自动捕获后续包。

本方案的主线始终是 scheduled extraction。能量门只允许在 `ACQUIRE/REACQUIRE` 状态短时
运行；进入锁定后必须关闭全局能量 Region 生成，不能让 500～2000 packet/s 的通信流量持续
决定计算量。

明确不做：

- 不对所有通信包做持续完整解调；
- 不做连续 host 737.28→998.4 MS/s 重采样；
- 不依赖 RFNoC Upsampler；
- 不把“能量超过门限”直接视为 QM35；
- 不在一次 overflow 后继续沿用旧 sample-domain schedule；
- 本阶段不实现 SIC。

## 2. 当前代码基线与缺口

已有能力：

1. `UwbDetectorSc16`：SC16 整数能量门、候选 Region、code-9 粗/细相关、原生 SC16 PDU；
2. `UwbScheduledExtractorSc16`：已知 `t0/T` 后的 737.28 MS/s 固定周期 SC16 截窗；
3. `UwbPduRationalResamplerCcf65_48`：SC16/CF32 PDU 737.28→998.4 MS/s；
4. `UwbRealtimeDemodulator`：QM35 code 9、64 SYNC、4z2、FCS 与 timing feedback；
5. CF32 `UwbScheduledExtractor`：已有运行期 schedule control 和 learn-then-freeze lock tracker。

关键缺口：

- SC16 scheduled block 的 `t0/T` 只能构造时提供；
- SC16 scheduled block没有 `schedule`、`lock_obs`、`status` 端口；
- detector 的首包结果不能在同一连续流坐标系内切换 scheduled 模式；
- 没有“候选身份确认 → provisional schedule → locked”的状态机；
- 没有把 UHD `rx_time`/overflow 不连续纳入 SC16 自动失锁逻辑；
- 737.28 MS/s code-9 检测模板尚未作为受 QA 保护的正式资产固化。

## 3. 核心架构决策

### 3.1 新增统一块，不并联两个全速率分支

建议新增：

```text
gr::uwb::UwbAutoScheduledExtractorSc16
Python: uwb.auto_scheduled_extractor_sc16
GRC: uwb_auto_scheduled_extractor_sc16
```

它是一个 `gr::sync_block`：

```text
stream input : 1 × packed complex<int16_t>
stream output: 0
message out  : packet, status
message in   : lock_obs, control
```

不要采用以下拓扑：

```text
SC16 stream ─┬→ DetectorSc16
             └→ ScheduledExtractorSc16
```

这种拓扑会让两个 block 永久扫描同一条 737.28 MS/s 流，也无法保证异步 detector 反馈与
scheduled block 的当前 absolute sample 在同一状态切换点一致。

统一块在任意时刻只启用一种热路径：

```text
未锁定：acquisition energy/core
已锁定：scheduled bulk-skip/bulk-copy core
```

### 3.2 复用 core，不把 GNU Radio block 嵌套进另一个 block

建议拆分/复用以下纯 C++ core：

- `UwbDetectorStateMachineSc16`：能量 Region 状态机；
- 新增 `UwbPreambleVerifierSc16Core`：从 `UwbDetectorSc16::publish_packet()` 抽出 Q15
  coarse、full-rate fine、弱首 SYNC backtrack；
- `ScheduledWindowCore`：周期窗口坐标与池管理；
- `ScheduleLockTracker`：CF32 scheduled 路径已有的 learn-then-freeze tracker；
- 新增 `Qm35AcquisitionTracker`：身份确认、周期一致性和状态转移。

禁止在新 block 内实例化并手工调用另一个 GNU Radio block 的 `work()`。

## 4. 端到端 flowgraph

```text
uhd.usrp_source
  cpu=sc16, otw=sc16, fs=737.28e6
  freq=6489.6e6, gain=60, antenna=TX/RX0
       │
       ▼
UwbAutoScheduledExtractorSc16
       │ packet: native SC16 PDU @737.28M
       ▼
UwbPduRationalResamplerCcf65_48
  profile=quality_minorder
       │ packet: CF32 PDU @998.4M
       ▼
UwbRealtimeDemodulator
  code=9, preamble=64, sfd=4z2, cir=bypass
       │ result/status
       ├──────────────────────────────→ application sink
       │ schedule_feedback
       └──────────────────────────────→ auto extractor lock_obs
```

首包 acquisition PDU 和 locked scheduled PDU 使用同一个 `packet` 输出端口，通过 metadata
中的 `capture_mode=acquisition|scheduled` 区分。这样首个确认包不会丢失，也无需在 flowgraph
中切换消息连接。

## 5. 状态机

```text
UNLOCKED_ACQUIRE
  │ code-9 fine confirmed, emit candidate PDU
  ▼
CANDIDATE_VERIFY
  │ demod success + FCS pass + QM35 profile match
  ▼
PROVISIONAL_TRACK
  │ immediately schedule future slots with wider guard
  │ 2～3 consistent timing observations
  ▼
LOCKED
  │ sustained misses / timing residual outliers / discontinuity
  ▼
HOLDOVER
  │ short miss: continue every-slot capture with widened guard
  ├─ recovered ───────────────────────────────→ LOCKED
  └─ timeout/discontinuity ───────────────────→ REACQUIRE

REACQUIRE
  │ predicted-neighborhood search first
  │ then global acquisition if needed
  └───────────────────────────────────────────→ PROVISIONAL_TRACK
```

### 5.1 `UNLOCKED_ACQUIRE`

运行 SC16 整数能量门：

- `energy_gate_decimation=100`；
- 每 100 样点取前 16 点的 `I²+Q²` 和；
- 32 个 decimated block 滑动门；
- 默认 normalized threshold `0.001`；
- 低能量连续 8 个 block 后关闭 Region；
- 固定池，不在 `work()` 分配。

Region worker 只做 code-9 前导验证：

1. Q15、4 倍降采样 coarse；
2. coarse 峰附近 full-rate normalized fine；
3. strong threshold 0.5；
4. weak-start threshold 0.2、±8 样点、最多回溯 3 SYNC；
5. 通过后输出 acquisition PDU，不在 detector worker 内执行完整 PHY 解调。

### 5.2 `CANDIDATE_VERIFY`

候选 PDU 经 65/48 和 realtime demod 后，必须同时满足：

```text
status == success
fcs_pass == true
code_index == 9
preamble_repetitions == 64
sfd_mode == 4z2
```

只有能量或 code-9 fine peak 不足以声明 QM35 锁定。通信设备可能使用相同 code，因此最终身份
至少需要 PHY profile + FCS；若后续能获得 QM35 MAC 特征，应作为可配置 identity predicate，
但不能硬编码到基础 core。

### 5.3 `PROVISIONAL_TRACK`

收到首个 FCS-pass 候选后，不等待第二次全局能量检测才开始截窗。用已知 nominal period 立即
生成第一个仍在未来的 slot：

```text
k_next = ceil((current_sample + pre_guard - t0) / T_nominal)
predicted_next = round(t0 + k_next*T_nominal)
```

其中 `t0`、`current_sample`、`T` 全部在 737.28 MHz native sample domain。首个 acquisition
包的 detector `start_sample` 已是 native 坐标，不要先映射到 998.4 再反算。

Provisional 使用较宽 guard，例如 ±20～30 us，并通过 demod `schedule_feedback` 收集 timing。
满足以下条件后进入 `LOCKED`：

- 至少 2 个、建议 3 个 timing-ok observation；
- observation 对 nominal 5 ms 周期残差在 guard 内；
- 稳健拟合后的 period ppm 在配置范围内；
- 至少首个候选 FCS pass。后续 FCS 可失败，timing 仍可用于快反馈。

### 5.4 `LOCKED`

锁定后完全停止全局能量门，只运行：

```text
predicted(k) = llround(t0_exact + k * period_exact)
window = [predicted-pre, predicted+capture+post)
```

每个 k 独立从 double/long double 计算，禁止累计整数 `period_samples`。

默认 `EverySlot`：即使碰撞、FCS 失败或前导不可见，也输出 scheduled 窗口。计算量只与
QM35 slot rate 有关。

### 5.5 `HOLDOVER/REACQUIRE`

建议默认：

- 单个 timing miss：不失锁；
- 连续 3 个 miss：进入 HOLDOVER，guard 扩大；
- 连续 8～16 个 miss或 residual 连续越界：REACQUIRE；
- 明确 `rx_time` discontinuity/overflow：立即使 sample-domain lock 失效；
- REACQUIRE 先搜索旧预测点附近，失败后才恢复全局能量门。

阈值必须参数化，并通过硬件 soak 调整，不能写死为不可配置魔数。

## 6. 已知周期与未知周期

第一版只要求“盲 t0、已知 T”：

```text
period_mode = known
nominal_packet_interval_s = 0.005  # 当前 QM35 200 slot/s 配置
```

这是本项目最小可靠闭环。首个 FCS-pass QM35 即可确定相位，后续 timing observations 微调 T。

第二版可选“盲 t0 + T”：

- 收集至少 3 个 FCS-pass 或高置信 code-9 起点；
- 在配置范围（例如 4～11 ms）内做周期假设；
- 使用 RANSAC/稳健线性拟合 `start_i ≈ t0 + k_i*T`；
- 防止把密集通信包间隔误认为雷达周期；
- 未通过周期一致性前不得进入 LOCKED。

不要把未知 T 纳入第一版验收，以免扩大实现范围。

## 7. 坐标域与 65/48 映射

必须始终记录四个坐标：

1. X410 native stream：737.28 MHz；
2. acquisition/scheduled PDU：737.28 MHz；
3. demod PDU：998.4 MHz；
4. UHD hardware time：秒。

主 schedule 永远存储在 native 域。PDU resampler 使用已有群时延映射：

```text
map_737_to_998(p) = round((p*65 + (taps-1)/2) / 48)
```

demod feedback 必须携带：

```text
detected_start_sample      # observation domain
sample_rate                # normally 998.4e6
native_sample_rate         # 737.28e6
resample_filter_delay
schedule_index
timing_ok
fcs_pass
```

auto extractor 复用 CF32 scheduled block 已有的反向映射逻辑，将 observation 转回 native
坐标后再更新 tracker。禁止直接把 998.4 MHz sample index 写入 737.28 MHz schedule。

## 8. 737.28 MHz code-9 模板

不得在生产脚本启动时临时用任意 scipy 参数生成 detector 模板。应新增可复现 generator：

```text
testdata/generate_qm35_reference_737p28.m
testdata/reference_preamble_code9_737p28.cf32
testdata/reference_preamble_code9_737p28_metadata.json
```

生成必须参考：

- `UWB_demodulation/buildUwbReference.m`；
- 当前 998.4 MHz `testdata/reference_preamble.bin`；
- 与 MATLAB 相同的抗混叠/重采样约定。

验收：

- MATLAB 和 C++ 使用同一 native 模板；
- synthetic 737.28 MHz QM35 的 detector start 与 MATLAB逐样点比较；
- 模板长度、能量归一化、群时延、sample-index base 写入 metadata；
- 生成产物由脚本可重复得到，二进制是否入库按仓库 testdata 策略决定。

## 9. GNU Radio scheduler 与线程语义

实现前 Grok 必须先搜索本机 GNU Radio 源码中的：

- zero-output `sync_block` / sink；
- `block_executor.cc` 对零输出块返回值的处理；
- message handler 与 `message_port_pub`；
- `rx_time` tag 的 UHD source 示例；
- EOS 时 worker drain 的现有 block。

新 block 语义：

- `sync_block`，1 stream in、0 stream out；
- `work()` 返回本次消费的输入 item 数；
- ACQUIRE 中只做整数能量状态机和 bulk Region copy；
- LOCKED 中只做 bulk skip、窗口交集计算和 bulk copy；
- worker 只接收固定池 handle；
- PMT、SC16 vector 构造、相关、完整解调均不在 `work()`；
- message handler 只写 pending observation/control；
- 所有模式切换只由 `work()` 在 chunk 边界应用；
- `work()` 中禁止 vector 扩容、锁等待、文件 I/O 和模板构造；
- `set_max_noutput_items(1048576)`，硬件测试记录实际 work chunk 分布。

异步反馈到达时，流可能已前进多个周期。切换 schedule 时必须从当前
`d_current_sample` 计算未来第一个可完整截取窗口，不能尝试创建已经过去的窗口。

## 10. 内存与队列

建议分开两个固定池：

- acquisition region pool：8；
- scheduled window pool：8～16；
- acquisition verify queue：有界；
- scheduled publish queue：有界。

模式切换时：

- 已发布 acquisition PDU 的 payload 生命周期由 PMT 管理；
- 未完成 acquisition Region 显式 release；
- 不清除仍被 worker 持有的槽；
- generation 递增，旧 feedback 因 generation 不匹配而丢弃；
- 所有 drop 都有独立计数，不能只记录总 drop。

## 11. PMT 接口

### 11.1 `packet` metadata

至少包括：

```text
packet_id
capture_mode                 # acquisition | provisional | scheduled
acquisition_epoch
schedule_generation
schedule_index               # acquisition 时可为 UINT64_MAX/明确 absent
start_sample                 # detector backtracked native start
predicted_start_sample
timing_seed_sample           # first strong SYNC
window_start_sample
trigger_sample
sample_rate                  # 737.28e6
sample_format                # sc16
sample_count
detection_metric
start_metric
start_backtracked_symbols
packet_interval_s
lock_state
rx_time_s                    # 可映射时
```

### 11.2 `lock_obs`

接受 demod `schedule_feedback`，并补充：

```text
packet_id
capture_mode
acquisition_epoch
schedule_generation
schedule_index
detected_start_sample
sample_rate
native_sample_rate
resample_filter_delay
timing_ok
fcs_pass
status
code_index
preamble_repetitions
sfd_mode
```

旧 epoch/generation 的反馈必须忽略并计数。

### 11.3 `status`

至少发布：

```text
acquisition_started
candidate_emitted
candidate_rejected
qm35_identity_confirmed
provisional_schedule_started
schedule_locked
schedule_holdover
schedule_lost
reacquisition_started
rx_discontinuity
window_dropped
queue_full
```

## 12. UHD `rx_time` 与 overflow

近期 X410 测试已经证明：网络 buffer 正确后可达到 200/200 scheduled windows；此前 overflow
会使 sample-domain `t0` 失效。因此硬件不连续不是可忽略诊断项。

block 应读取输入范围内的 `rx_time`/`rx_rate` tags，维护：

```text
absolute GNU Radio item index ↔ hardware time
```

若新 tag 与样本数外推误差超过容差：

1. 当前窗口标记 partial/drop；
2. generation++；
3. 发布 `rx_discontinuity`；
4. 进入 REACQUIRE；
5. 禁止继续按旧 absolute sample schedule 截窗。

若 UHD 只在日志打印 overflow、未给 stream tag，应在 X410 app 层接入 async message/统计并向
block `control` 端口发送 discontinuity。Grok 必须在实现前确认本机 UHD 4.6 的实际 tag/async
行为，不能凭假设实现。

## 13. 参数建议

```text
sample_rate                  = 737.28e6
nominal_packet_interval_s    = 0.005
period_mode                  = known
energy_threshold             = 0.001
energy_gate_decimation       = 100
energy_gate_window           = 32
energy_holdoff_blocks        = 8
detector_coarse_decimation   = 4
detector_fine_threshold      = 0.5
detector_backtrack_threshold = 0.2
detector_backtrack_radius    = 8
detector_max_backtrack       = 3
pre_guard_samples            = round(10 us * 737.28e6)
capture_samples              = round(190 us * 737.28e6)
post_guard_samples           = round(4.1 us * 737.28e6)
provisional_guard_us         = 20～30
lock_observations            = 3
holdover_miss_count          = 3
reacquire_miss_count         = 8～16
```

注意：现有 `9984/189696/4096` 是 998.4 MHz 几何。原生 737.28 MHz 新 block 必须按时间换算，
不能无说明地直接复用这些 sample counts。

## 14. QA 与 MATLAB 对照

### 14.1 纯 core QA

1. 流从 QM35 包前任意 offset 启动，能发现首包；
2. 首个候选 code-9 但 FCS fail，不得直接 LOCKED；
3. 首个 FCS-pass 后从当前 sample 选择未来 slot；
4. 异步反馈延迟 0～10 个周期仍正确建立 schedule；
5. 已知 T 下 3 observation 进入 LOCKED；
6. `t0+kT` 非整数 sample 无累计漂移；
7. 单 miss 不失锁；连续 miss 进入 HOLDOVER/REACQUIRE；
8. stale epoch/generation feedback 被拒绝；
9. pool/queue 满计数准确且不阻塞 stream；
10. chunk size 1、4k、64k、512k、1M 结果一致；
11. EOS 时 acquisition/scheduled 完整 PDU 排空；
12. discontinuity 后旧 schedule 不再输出。

### 14.2 混合流 QA

构造 737.28 MHz SC16 混合流：

- QM35 code 9 / 64 SYNC / 4z2，100/200 slot/s；
- 通信 code 10/11，0、500、1000、2000 packet/s；
- SIR -20～+20 dB；
- 随机启动 offset；
- 首包碰撞、连续漏包、时钟 ppm、jitter；
- 复数 IQ 相加后量化 SC16。

必须证明：

- acquisition 能拒绝非 QM35 候选；
- 锁定后窗口数只跟 QM35 schedule 有关；
- 通信速率不再增加 locked 模式 Region；
- collision slot 仍按 `EverySlot` 输出；
- 首个确认包和后续 scheduled 包均可追溯。

### 14.3 MATLAB 对照

必须参考：

- `UWB_demodulation/buildUwbReference.m`；
- `decode_uwb.m` / `decode_uwb_all.m`；
- fixed-interval candidate 逻辑；
- `testdata/` 已知 QM35 信号。

逐项比较：

```text
energy trigger range
coarse candidate
fine/backtracked start
first FCS-pass identity
t0/T fit
每个 predicted_start
每个 window start/end
737→998 metadata map
demod detected_start
```

packet start 必须逐样本或在明确记录的滤波群时延容差内一致。

## 15. 硬件验收

配置：X410 `192.168.10.2`、SC16、737.28 MS/s、6489.6 MHz、60 dB、TX/RX0，
PDU 65/48，QM35 code 9/64/4z2。

### 15.1 Acquisition

- 随机启动 20 次；
- 每次均自动找到至少一个 FCS-pass QM35；
- 已知 5 ms 周期时，P95 lock time 目标 ≤50 ms；
- 首个确认包保留并可解调；
- 不手工填写 `first_packet_sample`。

### 15.2 Locked capture

- 1 秒：期望 200 slot，0 extractor/resampler/demod queue drop；
- 60 秒：窗口数与硬件时间一致，无 silent slip；
- 10 分钟 soak：记录 overflow、lock loss、reacquire、queue HWM、FCS/timing 分布；
- 通信干扰下仍每 slot 输出；
- 人为短时遮挡后进入 HOLDOVER并恢复；
- 人为制造 source restart/overflow 后必须重捕获，不沿用旧 t0。

网络/输入前提：UHD 必须无持续 overflow。buffer 配置不能替代 discontinuity 检测。

## 16. 性能指标

分别报告：

```text
ACQUIRE input MS/s
LOCKED input MS/s
energy regions/s
code-9 verified candidates/s
candidate demod queue HWM
scheduled windows/s
PDU resampler mean/P95
demod mean/P95
window drops
reacquisition count/time
```

最低要求：

- LOCKED 热路径持续 ≥737.28 MS/s，0 window drop；
- 200 slot/s 下 PDU resampler + demod 无持续积压；
- ACQUIRE 在当前实际通信负载下可在 50 ms 目标内完成；
- 锁定后通信包率不显著改变 stream 吞吐。

## 17. 建议提交序列

### Commit 1：native template 与 golden

```text
Add MATLAB-verified QM35 reference at 737.28 MS/s
```

### Commit 2：抽取 SC16 verifier core

```text
Refactor SC16 preamble verification into reusable core
```

要求现有 `UwbDetectorSc16` QA bit-for-bit/coordinate regression 不变。

### Commit 3：auto block acquisition + QA

```text
Add SC16 QM35 acquisition state to auto scheduled extractor
```

只完成 UNKNOWN→candidate PDU，不先加入 lock。

### Commit 4：provisional/locked schedule

```text
Switch from verified QM35 acquisition to native scheduled capture
```

### Commit 5：demod feedback 与 tracker

```text
Track native QM35 schedule from resampled demod feedback
```

### Commit 6：discontinuity 与 reacquisition

```text
Reacquire QM35 after timing loss or UHD discontinuity
```

### Commit 7：X410 app/GRC 与硬件报告

```text
Add X410 SC16 auto-scheduled QM35 flowgraph and hardware validation
```

每个提交后：构建、定向 QA、`ctest --output-on-failure`、`git diff --check`。不要将 `.mat`、
`.csv`、抓取 IQ 或临时 `/tmp` 脚本提交进代码 commit。

## 18. 完成定义

只有以下全部满足，才能声明“X410 能从盲流锁定 QM35 并自动捕获后续包”：

1. 无需用户提供 `first_packet_sample`；
2. 首个锁定依据包含 FCS-pass QM35 identity；
3. 首个确认包被输出；
4. 后续自动切换到 5 ms scheduled capture；
5. 锁定后关闭全局能量 Region；
6. timing feedback 可微调 native `t0/T`；
7. collision/FCS fail slot 仍按 EverySlot 保留；
8. overflow/rx discontinuity 会显式失锁并重捕获；
9. 737/998 坐标与 MATLAB 对照通过；
10. 20 次随机启动、60 秒连续和10分钟 soak 验收通过；
11. 全部 QA/CTest 通过，无持续队列积压或 silent drop。
