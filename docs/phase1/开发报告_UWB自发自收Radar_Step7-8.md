# 开发报告：UWB 自发自收 Radar Step 7–8（含 Review R1–R4 整改）

> 日期：2026-09-05
> 分支：`feature/uwb-monostatic-radar`
> 依据：[开发计划_UWB自发自收Radar.md](开发计划_UWB自发自收Radar.md) Step 7–8、
> [开发需求_UWB自发自收Radar.md](开发需求_UWB自发自收Radar.md)
> 整改依据：[Review_UWB自发自收Radar_Step7-8.md](Review_UWB自发自收Radar_Step7-8.md)
> 状态：**Step 7–8 完成，Review R1–R4 全部整改完成并复验。全量 CTest 25/25
> 通过，MATLAB R2025b 实际运行读回验证全部通过（见第 9 节）。**
> 未改 [`../../开发状态.md`](../../开发状态.md)。未提交。未开始 Step 9。

---

## 0. Review R1–R4 整改记录

| 项 | Review 结论 | 整改 |
|---|---|---|
| R1 | `readAllJsonl` 用 `meta(k) = jsondecode(line)` 扩展空 struct → MATLAB 报错，reader 完全不可用 | reader 重写：行先解码进 cell array，字段取并集、缺字段回填 `[]`，最后才拼 struct array；mixed 成败帧日志可读。实际 MATLAB 运行验证见第 9 节 |
| R2 | `UWB_demodulation/` 整体被 `.gitignore` 忽略，reader 无法交付 | reader 移至 Git 可追踪路径 **`testdata/uwb_radar/read_uwb_cir.m`**（`git status --short --untracked-files=all` 可见）；`UWB_demodulation/` 中的未受控副本已删除 |
| R3 | `write_frame` 成功条件用 `>=`，payload 比 `tap_count` 长会被静默截写 | 两个条件改为严格 `==`（并检查 `c32vector_elements` 指针非空）；raw/norm 过长、过短、缺失一律按 failed 帧处理：JSONL `tap_count=0`、两个 offset 均不推进、binary 零写入。新增 QA：raw 过长 / norm 过长 / 恰好匹配正向对照 |
| R4 | `llround` 前未检查超大 `calibration_delay_work_samples`，报告宣称的 checked conversion 不成立 | 新增 `cal_round_checked()`（isfinite / ≥0 / ≤9e18 < INT64_MAX 才允许 llround），uint64 > INT64_MAX 与负整数在 `cal_delay_value()` 转换前拒绝；新增 QA 5 例（DBL_MAX、≈2^63、uint64 > INT64_MAX、负整数、负实数）全部稳定 `invalid_metadata`、零入队 |
| — | 性能 P99/max 一次运行值被写成硬上界 | 性能改为多轮运行的代表区间（见第 1 节），不再写单一硬上界 |

## 1. 复验结论（整改后）

| 项目 | 结论 |
|---|---|
| 新增 QA | `uwb_qa_uwb_radar_cir_estimator_block.cc`（8 case，含极端校准延时）、`uwb_qa_uwb_cir_writer.cc`（5 case，normalized 含过长 payload 三例）|
| 新 QA 结果 | 2/2 Passed |
| 全量 CTest | **25/25 Passed**（含既有 radar core / PDU / PacketSource / Loopback / demod 全部回归）|
| `git diff --check` | 干净 |
| MATLAB 读回验证 | **R2025b（`/mnt/f/MATLAB`，经 WSL interop headless `-batch`）实际运行 `verify_read_uwb_cir` 13 项检查全部 PASS**，命令与输出见第 9 节 |
| 200 pulse/s 服务时间（单 worker，warmup 后，多轮代表值） | mean ≈ 1.26–1.39 ms，P95 ≈ 1.5–1.9 ms，P99 ≈ 2.0–4.0 ms；不同运行/机器的 P99 波动到 ≈4 ms 属正常（评审机一例实测 P99=4096 µs），不代表稳定上界；断言门限 mean<5 ms、P99<20 ms |
| 未宣称 | Step 9 e2e、EchoTimer、UHD、X410、Phase A 均未完成 |

---

## 2. 块类型与调度语义（实现前契约）

对照本机 GNU Radio：`UwbRealtimeDemodulator`（bounded queue + worker + message
port）与 `UwbPacketWriter`（handler 入队、worker 写盘）是同类先例；
`blocks/message_strobe` 是 host timer 源，不用于 PRI。

| 块 | 类型 | 端口 | 热路径 |
|---|---|---|---|
| `UwbRadarCirEstimator` | `gr::block`，0 流端口 | `rx` → `cir`/`status` | handler 只校验+入队；单 worker 调 `radar_cir_one`，scratch 在 `make()` 由 `prepare_radar_cir_core` 一次冻结 |
| `UwbCirWriter` | `gr::block`，0 流端口 | `cir` → `status` | handler 只校验+入有界环队；单 writer 顺序写盘；`stop()` 先排空再关文件 |

选 `gr::block` + message PDU（而非 tagged_stream/sync）的原因与 Step 5–6 相同：
输入是突发 PDU、PRI 静默不进流、与既有 demod/writer PDU 契约一致；
handler 内不做 I/O 与重活，避免卡住 scheduler。单 worker 保证 pulse 顺序
（计划 Step 7：多 worker 优化必须另带重排序 QA，本期不做）。

---

## 3. Step 7 — `UwbRadarCirEstimator`

文件：`include/gnuradio/uwb/uwb_radar_cir_estimator_block.h`、
`lib/uwb_radar_cir_estimator_block.cc`（核心头 `uwb_radar_cir_estimator.h`
是 Step 3 的 header-only CIR core，块实现加 `_block` 后缀避免撞名）。

### 3.1 make() 契约

```cpp
make(template_path,              // 1016 点 CF32 SYNC 模板（998.4 MS/s）
     sync_repetitions=64, sfd_mode="4z2", code_index=9,
     cir_pre=16, cir_post=100, cir_skip_initial=10, cir_repetitions=0, // 0=auto
     sfd_search_margin=64, sync_refine_margin=8,
     sfd_threshold=0.3f, sync_refine_threshold=0.3f,
     emit_normalized=true, queue_capacity=64)
```

`make()` 内用 `prepare_radar_cir_core` 冻结 TX profile identity（code/SFD/
SYNC/sps/tx_profile_id）并预分配全部 scratch；模板长度 ≠ 1016、profile 不
支持、scratch 制备失败 → `make()` 抛异常。`cir_repetitions=0` 时按
`reps>skip ? reps-skip : (skip=0, reps)` 自动推导（对齐 demod 约定）。

### 3.2 SFD 预测（handler 内，checked math）

```text
predicted_sfd = pre_guard_samples + round(calibration_delay_work)
                + sync_repetitions × 1016
zero_delay_tap = cir_pre + round(calibration_delay_work)
```

- 校准延时取键优先级：有 `calibration_delay_work_samples`（65/48 映射后）
  用之；否则用 `calibration_delay_native_samples` 原值（直通 998.4 PDU，
  无重采样映射时该键即 work 域）。两者都缺 → `invalid_metadata`。
- 任一坐标运算溢出（checked `radar_i64_*`）→ `invalid_metadata`，不入队。

### 3.3 输入校验（拒绝即发 status，不入队）

| 条件 | status 事件 |
|---|---|
| 非 PDU / 非 dict+c32vector / 空 payload | `invalid_input` |
| 缺 `sample_rate` 或 ≠ 998.4 MHz | `bad_input_rate` |
| `sync_repetitions/sfd_mode/code_index` 缺失或与 prepared profile 不符 | `invalid_profile` |
| 缺 `pulse_id`/`pre_guard_samples`、pre_guard<0、缺校准延时、坐标溢出 | `invalid_metadata` |

### 3.4 输出契约

每个入队 pulse 恰好发布一条 `cir` PDU：

- `status` ∈ `ok/sfd_failed/timing_failed/cir_failed/invalid_input`
  （worker 异常 → `invalid_input` + `worker_exception` 事件，计数器独立）；
- 成功帧：payload = raw CIR taps（c32vector），meta 含
  `tap_count/pre/cir_pre_samples/cir_post_samples/peak_tap/cir_peak_metric/
  raw_l2_norm/valid_repetitions/sfd_start_sample/preamble_start_sample/
  cir_origin_sample/predicted_sfd_start_sample/zero_delay_tap/
  calibration_delay_work_samples/range_m_per_tap(0.150136447)/
  sfd_ok/timing_ok/queue_delay_us/estimator_us`，
  并在 `emit_normalized` 时把 L2-normalized taps 放进 meta 键
  `normalized_taps`（c32vector）；
- 失败帧：payload 恒为空、`tap_count=0`、无 `normalized_taps`，
  失败级坐标保持 -1（不冒充成功，与 core 契约一致）；
- lineage 透传（`radar_meta::kPassthrough` + 窗几何字段）：
  TX/RX device time、`num_delay_samps`、`calibration_id`、`source`、
  `uhd_error`、`sync/sfd/range_guard/tx_packet/rx_capture_samples` 等。

### 3.5 队列与生命周期

- 有界队列（默认 64），队满 → 丢弃 + `queue_full` status（带 pulse_id 与
  queue_depth），`pdus_dropped` 计数；high-watermark 记录。
- `drain()/drained()` 供 QA 与上层同步；`stop()` 先置 stop、notify、join，
  worker 把队列中剩余 pulse 处理完再退出（不丢已入队帧）。
- `start()` 幂等：先 join 旧 worker 再重启（stop→start 复用同一块对象）。
- 析构兜底 join，挂起任务先排空，无死锁（QA 覆盖）。
- 服务时间直方图（64×64 µs bin + overflow）：`service_mean/p95/p99/max_us`。

---

## 4. Step 8 — `UwbCirWriter` 与 MATLAB reader

### 4.1 块

文件：`include/gnuradio/uwb/uwb_cir_writer.h`、`lib/uwb_cir_writer.cc`。

- `make(directory, base_name="cir", write_normalized=false, queue_capacity=64)`。
- `start()`：建目录、打开 `<base>.cf32`/`<base>_norm.cf32`（可选）/
  `<base>.jsonl`（全部 trunc）、写 `run.json`（格式、字节序、
  `bytes_per_tap=8`、`range_m_per_tap`、write_normalized）。
- handler：仅校验 PDU 形状并入有界环队（构造时按容量一次分配）；
  队满 drop、malformed 计 `frames_invalid`（不写任何文件）。
- writer worker（单线程、有序）：
  - `status=="ok"` 且 `tap_count` 与 payload（及开启时的
    `normalized_taps`）长度一致 → 写 taps（complex64 LE）并推进
    `file_offset_taps`/`file_offset_norm_taps`；
  - 其余（三种 failed + 不一致/缺 norm taps）→ 只写 JSONL，
    `tap_count=0`，offset 不推进；**失败帧永不写 tap**；
  - JSONL 每帧恰一行：§10.1 契约字段全量
    （`pulse_id/schedule_index/status/sample_rate/tap_count/pre/post/
    file_offset_taps/zero_delay_tap/peak_tap/cir_peak_metric/
    peak_delay_from_calibration_ns/range_m_per_tap/raw_l2_norm/
    preamble/sfd_start_sample/tx_time/num_delay_samps/calibration_id/
    sfd_ok/timing_ok`，开启时另有 `file_offset_norm_taps`）。
- `stop()`：**先排空队列**再 flush/close —— JSONL 与 binary 永不出现半帧
  偏移。`start()` 幂等：先 join 旧 writer、关闭并重开文件（trunc）、
  计数器清零。

实现过程中修复的两个真实缺陷（均有 QA/复现）：

1. `start()` 二次调用（flowgraph 生命周期 + 手动 start）对已 joinable 的
   `std::thread` 赋值 → `std::terminate`；已改为先 join 再重启。
2. 重启/二次 start 对已打开 `ofstream` 直接 `open()` 是 no-op 且置 failbit
   → 写入静默丢字节；已改为先 close/clear 再 reopen。

### 4.2 MATLAB reader（Git 可追踪路径：`testdata/uwb_radar/`）

受控副本 **`testdata/uwb_radar/read_uwb_cir.m`**（整改 R2；`UWB_demodulation/`
整体被 `.gitignore` 忽略，原未受控副本已删除）：

```matlab
[cir, meta] = read_uwb_cir(outDir, pulseId);        % raw CIR（除以 code_energy）
[nrm, meta] = read_uwb_cir(outDir, pulseId, true);  % cir_norm.cf32
```

- **批量读取**：`[~, metaAll] = read_uwb_cir(outDir, [])` 返回全部行
  （struct array）。注意 JSONL 元数据固定在**第二个输出**，单输出调用
  拿到的是 CIR；
- 按 `cir.jsonl` 的 `pulse_id` 定位行，用 `file_offset_taps`/
  `tap_count` 读 `cir.cf32`（`'float32=>single'`，R2025b 的 `fread` 在
  未显式指明输出类时返回 double）→ `tap_count×1 complex single`；
- **JSONL 解析**：行先解码进 cell array，字段取并集、缺字段回填 `[]`
  再拼 struct array（整改 R1 —— 直接 `meta(k)=jsondecode(line)` 扩展空
  struct 在 MATLAB 非法）；
- failed 帧（`tap_count==0`，含 R3 的长度不一致帧）返回空 `cir` 且保留
  meta 行（失败可观测）；
- 缺行（`read_uwb_cir:noPulse`）、缺 `tap_count`、文件截断
  （`read_uwb_cir:short`）、`cir_norm` 未写而请求 norm
  （`read_uwb_cir:noNorm`）：明确 `error`。
- 与计划 §18 一致：MATLAB 对照不进 CTest 运行时依赖，由离线核对完成
  （本次已完成实际运行，见第 9 节）。

---

## 5. QA 与结果

新 QA（均在 `gr-uwb/lib/`）：

### `qa_uwb_radar_cir_estimator_block.cc`（8 case）

| case | 覆盖 |
|---|---|
| `ok_matches_core_direct` | golden `rx_clean_998p4` PDU → ok；输出 taps/norm_taps 与直接调 `radar_cir_one` **逐 bit 一致**；`sfd_start=67021`、`cir_origin=1997`、`zero_delay_tap=16`、`valid_repetitions=54`、lineage 透传逐字段断言 |
| `failed_frames_no_taps` | 静默 RX → `sfd_failed`；SYNC 清零 → `timing_failed`；第 10..63 个平均 repetition 窗（含 k=63 触到 SFD 前沿的部分）精确清零 → 平均恒零 → `cir_failed`；三者均空 payload、`tap_count=0`、无 `normalized_taps`、失败级坐标 -1 |
| `invalid_inputs` | 非 PDU / s16 dtype / 空 payload / 缺 rate / native rate / sync=32 / sfd=ieee / code=10 / 缺 pre_guard / 缺校准 / 负 pre_guard / 坐标溢出 → 全部不入队（`received=9, invalid=12, enqueued=0`），四类 status 事件齐备 |
| `extreme_calibration_delay` | `calibration_delay_work_samples` = DBL_MAX / ≈2^63 / `uint64 > INT64_MAX` / 负整数 / 负实数 → 5 例全部稳定 `invalid_metadata`（事件恰 5 次）、`pdus_enqueued==0`、零 CIR 输出（整改 R4）|
| `queue_full` | capacity=2 连发 12 帧 → `enqueued+dropped==12`、watermark≤2、`queue_full` 事件、输出子集顺序与输入一致 |
| `output_order` | 分批（每批 < 队列容量）连发 10 帧 → `cir` 输出 pulse_id 严格递增 |
| `stop_restart_destructor` | 挂起任务下 `stop()` 排空且帧全部发布；restart 后可继续；析构在有挂起任务时排空+join 无死锁 |
| `service_time_200pps` | warmup 后 64 帧实测（多轮代表值）：mean=1278–1386 µs，P95=1536–1920 µs，P99=2048–3968 µs（断言 mean<5 ms、P99<20 ms）|

### `qa_uwb_cir_writer.cc`（5 case）

| case | 覆盖 |
|---|---|
| `mixed_frames` | 10 帧（6 ok + sfd/timing/cir/invalid 各 1）：JSONL 恰 10 行；raw 大小 = 8×Σok taps；ok 帧 offset 单调且逐字节 memcmp 相等；失败帧 `tap_count=0` 且 offset 不推进；`run.json` 存在 |
| `normalized` | `cir_norm.cf32` 与 raw 逐字节一致、带 `file_offset_norm_taps`；**raw payload 过长 / norm payload 过长 / 恰好匹配正向对照** 三例：过长者一律 JSONL `tap_count=0`、两个 offset 均不推进、raw/norm 文件大小不变（整改 R3）；norm taps 缺失/过短 → 该帧按失败处理不写任何 tap；失败帧也不写 norm |
| `invalid_pdus` | 非 pair / 非 c32 payload → `frames_invalid=3`，零 JSONL 行、零字节 |
| `stop_drains` | 入队后（等 handler 全部入队）立即 stop → 5 帧全部落盘，binary/JSONL 一致 |
| `restart` | restart 后 trunc、计数清零、失败帧不写 tap |

### 回归

```bash
ctest --test-dir gr-uwb/build --output-on-failure   # 25/25 Passed
git diff --check                                    # 干净
```

---

## 6. 实现中发现并记录的核心层观察（未改 Step 2–4 core）

用“真实幅度” golden（`rx_clean_998p4`）把 SFD 区域清零做负例时，
`search_sfd` 的 `|corr|²/pwr` 度量会在搜索窗内靠近 preamble 尾部的 j 处
产生 ≈0.5 的假峰（>0.3 门限）——因为滑动 pwr 只覆盖 SYNC 尾部少量非零样点，
而 acc 与之相干。合成包（`reference_preamble.bin` 归一化幅度）下该假峰
< 门限，故既有 `qa_uwb_radar_cir_core` 的 `sfd_failed` 用例不受影响。
结论：**“删除/破坏 SFD” 的检测鲁棒性与信号幅度有关**，属 Step 2 core
度量设计问题；本步不改 core（避免破坏 MATLAB 对齐 golden），仅在此记录，
留待后续 core 评审。块级 QA 的 `sfd_failed` 负例改用全零（静默）RX，
确定性地触发 `sfd_failed`。

---

## 7. 文件清单

| 动作 | 文件 |
|---|---|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_radar_cir_estimator_block.h` |
| 新增 | `gr-uwb/lib/uwb_radar_cir_estimator_block.cc` |
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_cir_writer.h` |
| 新增 | `gr-uwb/lib/uwb_cir_writer.cc` |
| 新增 | `gr-uwb/lib/qa_uwb_radar_cir_estimator_block.cc` |
| 新增 | `gr-uwb/lib/qa_uwb_cir_writer.cc` |
| 新增 | `gr-uwb/grc/uwb_radar_cir_estimator.block.yml`、`uwb_cir_writer.block.yml` |
| 新增（Git 可追踪，整改 R2） | `testdata/uwb_radar/read_uwb_cir.m`、`testdata/uwb_radar/verify_read_uwb_cir.m` |
| 修改 | `gr-uwb/lib/CMakeLists.txt`、`gr-uwb/include/gnuradio/uwb/CMakeLists.txt`、`gr-uwb/grc/CMakeLists.txt`、`gr-uwb/python/uwb/bindings/python_bindings.cc`（Python 绑定 `radar_cir_estimator`/`cir_writer`）|

测试数据未新增 binary（复用 `testdata/uwb_radar/` MATLAB golden）。注：
`UWB_demodulation/` 整体被根 `.gitignore` 忽略，reader 不得放在该目录交付。

---

## 9. MATLAB 实际运行验证（整改 R1/R2）

### 环境

Windows MATLAB R2025b（`/mnt/f/MATLAB`，版本 25.2.0），经 WSL2 interop
headless 调用。QA 先在 WSL 内重新生成真实 writer 输出目录：

```bash
cd gr-uwb/build/lib && ./uwb_qa_uwb_cir_writer.cc   # → /tmp/uwb_qa_cir_writer_{mixed,norm}
wslpath -w /tmp/uwb_qa_cir_writer_mixed
# \\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_cir_writer_mixed
```

### 命令

```matlab
% matlab.exe -batch（完整转义 UNC 路径；startup.m 报一条用户机器上的
% 编码噪声错误，与本验证无关）
addpath('\\wsl.localhost\Ubuntu-22.04\home\junqima\workspace\uwb-gnuradio\testdata\uwb_radar');
verify_read_uwb_cir('\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_cir_writer_mixed', ...
                    '\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_cir_writer_norm');
```

### 实际输出（13 项全部 PASS）

```text
PASS 1: pulse 0 raw CIR 116x1 complex single, per-sample exact
PASS 1: pulse 1 raw CIR 116x1 complex single, per-sample exact
PASS 2: pulse 2 (sfd_failed) -> empty cir, tap_count==0
PASS 2: pulse 4 (timing_failed) -> empty cir, tap_count==0
PASS 2: pulse 6 (cir_failed) -> empty cir, tap_count==0
PASS 2: pulse 8 (invalid_input) -> empty cir, tap_count==0
PASS 3: pulse 0 normalized CIR, per-sample exact
PASS 3: pulse 6 normalized CIR, per-sample exact
PASS 3b: normalized-run failed pulse 2 -> empty cir
PASS 4: truncated cir.cf32 -> read_uwb_cir:short
PASS 5: no file_offset_norm_taps -> read_uwb_cir:noNorm
PASS 6: unknown pulse id -> read_uwb_cir:noPulse
PASS 7: bulk read returns 10-line struct array
verify_read_uwb_cir: ALL CHECKS PASSED
```

覆盖 Review 要求的 5 项：① mixed 目录 ok pulse raw CIR（逐样点
`complex single` 相等，float32 算术模式与 C++ QA `make_taps()` 逐位一致）
② failed pulse 空 CIR + `tap_count==0` + 状态保留（四种失败状态各一）
③ normalized CIR 读回并逐样点一致 ④ 截断 `.cf32` 与无
`file_offset_norm_taps` 的明确 error ⑤ 与 C++ QA 写出的 tap 值逐样点一致。
MATLAB 验证不进 CTest 运行时依赖（`testdata/uwb_radar/verify_read_uwb_cir.m`
为受控脚本，可随时重放）。

---

## 10. Review 重新验收门槛复核

- [x] MATLAB reader 位于 Git 可追踪路径（`testdata/uwb_radar/read_uwb_cir.m`），
      且在真实 CirWriter mixed 输出上完成 raw/failed/norm/truncated 的
      MATLAB R2025b 实际运行读回验证（第 9 节）；
- [x] Writer 对 raw/norm tap 数量严格相等验证（`==`），过长与过短均不写
      binary、只写 `tap_count=0` 的 JSONL 行（QA 三例覆盖）；
- [x] Estimator 在 rounding 前安全拒绝超大 calibration delay 和整数转换
      溢出（`cal_round_checked` + `cal_delay_value`；QA 5 例）；
- [x] 报告更新实际性能统计：多轮代表区间 mean≈1.26–1.39 ms、
      P99≈2.0–4.0 ms，并注明 P99 波动不代表稳定上界；
- [x] 目标 QA 2/2、全量 CTest 25/25、`git diff --check` 重新执行通过；
- [x] 未宣称 Step 9、EchoTimer、UHD、X410 或 Phase A 已完成。

---

## 8. 未做 / 未验证

- Step 9 Phase-A 离线端到端（source → loopback → 65/48 → estimator →
  writer → MATLAB 读回）未跑通；
- EchoTimer（Step 10）、UHD backend（Step 11）、X410（Step 12）未开始；
- `sfd_search_margin=64` 仍是软件默认，非硬件冻结值；
- Step 2 core 的 SFD 度量幅度相关性（第 6 节观察）未修；
- 未提交、未 push；未改 `开发状态.md`。
