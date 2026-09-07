# Review：UWB 自发自收 Radar Step 7–8

> Review 日期：2026-09-05
> 被审查报告：[开发报告_UWB自发自收Radar_Step7-8.md](开发报告_UWB自发自收Radar_Step7-8.md)
> 分支：`feature/uwb-monostatic-radar`
> 结论：**Step 7 主体可保留；Step 8 的 MATLAB 交付与 CIR 帧长度契约未闭环，Step 7–8 整体暂不通过。完成 R1–R4 并复验前，不进入 Step 9。**

---

## 1. 本次范围与独立复验

验收范围仅包括：

1. Step 7：`UwbRadarCirEstimator` 的 PDU 契约、单 worker 有界队列、CIR
   core 调用、输出顺序、失败帧和生命周期；
2. Step 8：`UwbCirWriter` 的有界异步写盘、raw/normalized CIR 二进制与 JSONL
   offset 契约，以及 MATLAB `read_uwb_cir` 读回；
3. GRC/C++ 绑定、QA 和非 Radar 回归。

不验收 Step 9 Phase-A 端到端、EchoTimer、UHD、X410 或 OTA。Step 7–8 的纯软件
QA 通过不能据此宣称 timed I/O、硬件收发或完整雷达闭环完成。

本次独立执行：

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build \
  -R 'qa_uwb_radar_cir_estimator_block|qa_uwb_cir_writer' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
git diff --check
```

结果：新增 QA **2/2 Passed**，全量 CTest **25/25 Passed**，`git diff --check`
通过。

另直接运行 `test_estimator_service_time_200pps`，64 帧实测：

```text
mean=1260 us, P95=1472 us, P99=4096 us, max=4057 us
```

平均服务时间低于 5 ms，满足 Step 7 退出条件；但与开发报告所写的
P99/max `<= 3.3 ms` 不一致，报告必须改为实际测量值，或明确为不同机器/运行的
代表性数据，不能将该上界写成稳定结论。

---

## 2. 可保留的实现

- `UwbRadarCirEstimator` 和 `UwbCirWriter` 选用 0 流端口的 `gr::block` +
  message PDU 合理；CIR 计算和文件 I/O 均不在 GNU Radio handler 中执行。
- Estimator 使用单 worker、有界队列和预制 CIR scratch；core 直接调用与块级
  golden QA、失败状态、队列、顺序、stop/restart QA 均已具备。
- Writer 使用固定容量环队列、单 writer 顺序落盘；混合成功/失败帧、stop 排空、
  restart truncation、raw/normalized byte-level QA 已具备。
- 新 CMake、Python bindings 和 GRC block 描述已接入，且现有 25 项 CTest 回归通过。

这些代码与 QA 可以保留；下述修复不得退化为 handler 内同步 I/O 或取消失败帧的
JSONL 可观测性。

---

## 3. 阻断 R1：MATLAB `read_uwb_cir` 不能读取 writer 的实际输出

### 复现

以 `qa_uwb_cir_writer` 生成的真实 mixed（6 ok + 4 failed）目录运行：

```matlab
addpath('UWB_demodulation');
[cir, meta] = read_uwb_cir('/tmp/uwb_qa_cir_writer_mixed', 0);
```

MATLAB 在 `readAllJsonl` 的第一条 JSONL 即报“不同结构之间的下标赋值”。原因是
reader 初始化 `meta = struct()` 后直接执行 `meta(k) = jsondecode(line)`；空结构体
不能这样扩展为具有字段的结构体。

这意味着报告的 raw、failed、normalized MATLAB 读回契约当前均没有成立，Step 8
不能验收。

### 必须整改

- 首条 JSONL 直接初始化结构体，例如首行 `meta = one;`，后续再
  `meta(k) = one;`；或使用 cell array 收集后再统一处理。
- reader 必须能读取 writer 的 mixed 成败帧日志：成功帧返回 `tap_count×1` complex
  single，失败帧返回空 CIR 但保留该行 metadata。
- 必须保留 raw 与 normalized 路径、pulse ID 查找、缺行、无 norm、截断二进制的
  明确 error 行为。

### 必须新增验证

MATLAB 实际运行（不是仅 C++ QA）至少覆盖：

1. mixed JSONL 中一个 ok pulse 的 raw CIR；
2. mixed JSONL 中一个 failed pulse，断言空 CIR 与 `tap_count==0`；
3. normalized CIR；
4. truncated `.cf32` 与无 `file_offset_norm_taps` 的 error；
5. 与 C++ QA 生成的 tap bytes/float32 值逐样点一致。

MATLAB 不必接入 CTest，但命令、输出和真实结果必须写入报告。

---

## 4. 阻断 R2：MATLAB reader 位于被 Git 忽略的目录，不能交付

根目录 `.gitignore` 包含：

```gitignore
UWB_demodulation/
```

因此目前的 `UWB_demodulation/read_uwb_cir.m` 未被版本控制，执行提交时不会进入
Step 8 交付。开发报告将其列为“新增”文件与实际 Git 状态不符。

### 必须整改

- 将 reader 的受控副本置于非忽略、可提交的路径，建议
  `testdata/uwb_radar/read_uwb_cir.m`；如需在 `UWB_demodulation/` 使用，可由
  明确同步脚本或文档说明，但不能只依赖 ignored working-tree 文件。
- 报告中的文件清单必须写出可追踪路径；`git status --short --untracked-files=all`
  不得遗漏 reader。

---

## 5. 阻断 R3：CirWriter 允许 payload 比 `tap_count` 更长

`UwbCirWriter::write_frame()` 的成功条件当前是：

```cpp
raw_len >= tap_count_in
(!write_normalized || norm_len >= tap_count_in)
```

但 Step 8 契约要求 `tap_count` 与 raw payload（及开启 normalized 时的 norm
payload）**长度一致**。当前构造一个 `tap_count=116`、payload=117 taps 的坏 PDU
会被当作 ok，静默只写前 116 taps；这掩盖上游 metadata/payload 不一致。

### 必须整改

- 将两个 `>=` 改为严格 `==`；必要时同时检查 `c32vector_elements()` 返回指针。
- raw 或 norm 任一缺失、过短、过长时：只写一条 JSONL failed frame，
  `tap_count=0`，两个 offset 均不推进，binary 不写任何 tap。

### 必须新增 QA

在现有 normalized QA 中增加：

- raw payload 过长；
- normalized payload 过长；
- raw/norm 恰好匹配的正向对照。

逐项断言 JSONL `tap_count=0`、offset 不动、raw/norm 文件大小不变。

---

## 6. R4：Estimator 的超大 calibration delay 未在 `llround` 前检查

报告宣称所有坐标溢出均由 checked math 拒绝，但实现先对任意有限、非负的
`calibration_delay_work_samples` 执行：

```cpp
static_cast<size_t>(std::llround(cal_work))
```

再调用 checked helper。`cal_work` 接近或超过 `int64_t` 可表示范围时，`llround`
已经越界，不能视为 checked conversion。

### 必须整改

- 在 rounding 前验证 `cal_work <= INT64_MAX - 0.5`，并使用明确的
  double→int64 checked helper；拒绝 `uint64`/integer metadata 的符号溢出。
- 用 checked `int64` 值参与 `pre_guard + calibration + sync_span` 和
  `zero_delay_tap` 运算；任一失败只发 `invalid_metadata`，不入队。

### 必须新增 QA

覆盖 `calibration_delay_work_samples=DBL_MAX`、`INT64_MAX` 边界附近、
`uint64 > INT64_MAX`、负值；每例断言无 CIR PDU、未入队、稳定
`invalid_metadata` status。

---

## 7. 重新验收门槛

在以下全部满足前，不进入 Step 9：

- [ ] MATLAB reader 位于 Git 可追踪路径，且在实际 CirWriter mixed 输出上完成
      raw/failed/norm/truncated 的 MATLAB 读回验证；
- [ ] Writer 对 raw/norm tap 数量严格相等验证，过长与过短均不写 binary；
- [ ] Estimator 在 rounding 前安全拒绝超大 calibration delay 和整数转换溢出；
- [ ] 报告更新实际性能统计，不把一次运行的 P99/max 写成未复现的硬上界；
- [ ] 目标 QA、全量 CTest、`git diff --check` 重新执行并写入实际结果；
- [ ] 不宣称 Step 9、EchoTimer、UHD、X410 或 Phase A 已完成。

