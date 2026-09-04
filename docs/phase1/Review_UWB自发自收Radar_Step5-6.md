# Review：UWB 自发自收 Radar Step 5–6

> Review 日期：2026-09-05
> 被审查报告：[开发报告_UWB自发自收Radar_Step5-6.md](开发报告_UWB自发自收Radar_Step5-6.md)
> 分支：`feature/uwb-monostatic-radar`
> 结论：**主路径数值与本机 QA 可复现，但 Step 5–6 整体暂不通过。完成 R1–R4 并复验前，不进入 Step 7。**

---

## 1. 本次范围与独立复验

验收范围仅包括：

1. Step 5：`UwbPduRationalResamplerCcf65_48` 的 Radar metadata 白名单、
   737.28 → 998.4 坐标映射、SC16 输入及有界 scratch；
2. Step 6：`UwbRadarPacketSource` 与 `UwbLoopbackEcho` 的 PDU 契约、
   完整 packet 读取、软件整数/分数延时、多径、噪声及失败行为；
3. 相应 GNU Radio message-block 调度语义、QA 和构建回归。

不验收 Step 7 `UwbRadarCirEstimator`、CirWriter、EchoTimer、UHD、X410、
Phase A 或 OTA。Loopback 是纯软件信道，不能据此宣称 timed-I/O 或硬件验收。

本次独立执行：

```bash
cmake -S gr-uwb -B gr-uwb/build
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build -R 'qa_uwb_pdu_rational_resampler|qa_uwb_radar_packet_source|qa_uwb_loopback_echo' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
```

结果：目标 QA **3/3 Passed**，全量 CTest **23/23 Passed**。这些结果证明
当前覆盖到的主路径可运行，但不足以满足下述退出条件。

---

## 2. 已确认可保留的部分

- 三个块均使用 `gr::block`、0 流端口和 message PDU；没有把 PRI 放入
  `general_work` 或 host timer。该 scheduler 选择正确。
- `UwbRadarPacketSource` 能 bit-exact 读出 canonical 998.4 / 737.28 FC32
  文件；PacketSource 不自行打 5 ms PRI，语义正确。
- Loopback 的整数 37 样点、复增益与 MATLAB integer golden 对齐；分数
  12.4 样点采用 PCHIP，与 MATLAB `interp1(...,'pchip')` 对照；多径线性和
  固定 seed 噪声测试通过。
- Radar metadata 白名单、native alias（`*_native`）、`map(n)-map(0)` 长度
  映射与 `calibration_delay_work_samples = native * 65/48` 已有实现和
  部分 QA。
- 超出 `max_tx_samples` / `max_rx_samples` 的 Loopback PDU 会发布
  `invalid_window`，不产生 RX PDU。

这些实现和 QA 可以保留并扩展，不要为修复下述问题退化为默认 PMT metadata
透传。

---

## 3. 阻断 R1：Loopback 未验证输入 profile

### 问题

`UwbLoopbackEcho::handle_tx` 当前只验证 PDU 形状、payload 类型和窗口上限。
它读取 `sample_rate`、`sync_repetitions`、`sfd_mode`、`code_index` 后直接
写入输出 metadata；`is_work_rate()` 只决定是否补 `sync_samples` / `sfd_samples`，
不是拒绝条件。

因此下列合法 PDU 形状但非法的 Radar profile 会静默得到一个 RX PDU：

```text
sample_rate=1e6
sync_repetitions=0
sfd_mode="nope"
code_index=99
```

这违反 Step 6 的退出条件：invalid profile/window 必须有明确 status，且不得
发半包 PDU。也会让 Step 7 接收到看似完整、实际不能由 Radar core 解释的元数据。

### 必须整改

在 `handle_tx` 的任何 IQ/scratch 处理之前，新增一个明确的 profile validator。
第一版 Loopback 是 998.4 MHz CIR 工作域，应至少要求：

- `sample_rate` 存在且等于 998.4 MHz（允许现有相对容差）；
- PDU payload 类型与 `sample_format` 一致：CF32→`fc32`，交织 S16→`sc16`；
- `sync_repetitions > 0`，且限制为本期已支持的 32/64/128 或明确的
  profile 支持集合；
- `demod::GetSfdSequence(sfd_mode)` 非空；
- `code_index` 为 9–12；
- `tx_packet_samples` / `sample_count` 如存在，必须与实际 payload 长度一致；
- `sync_samples`、`sfd_samples` 如存在，必须非负并与 profile 一致；
  缺字段时可按已验证 profile 补全。

任一失败：`pdus_dropped++`，只发布 `status(event="invalid_profile")`（或拆分成
`bad_input_rate` / `invalid_profile`，但必须稳定且文档化），不发布 `rx`。

不得通过“自动替换成默认 64/4z2/code9”掩盖输入错误。

### 必须新增 QA

表驱动输入：错误/缺失 rate、payload-format 不一致、0/不支持 SYNC、未知 SFD、
code 8/13、长度字段与 payload 不一致、负 guard。每个用例断言：

```text
pdus_emitted == 0
pdus_dropped == 1
恰有一个稳定 status
无 RX PDU
```

并保留现有有效 64/4z2/code9、整数/分数/多径/噪声 QA。

---

## 4. 阻断 R2：PDU resampler 的 handler 仍可动态扩容

### 问题

Step 5 明确要求 SC16→FC32 和 resampler output scratch 在非热路径预分配，
handler 内不得扩容。当前实现仍含：

- `ensure_scratch()` 的 `d_scratch_.resize(n_out)`；
- `flush` 中的 `d_scratch_.resize(produced + 256)`；
- SC16 分支的 `d_input_scratch_.resize(n_in)`。

“常见 220k 样点窗口通常不扩容”不是有界实时契约；首个大窗口或异常窗口会在
GNU Radio message-handler 线程上分配内存。

### 必须整改

为 `UwbPduRationalResamplerCcf65_48::make()` / `make_from_taps()` 增加固定的
`max_input_samples`（或固定窗口 profile）；构造时据此计算最大输出并完成：

```text
d_input_scratch_.resize(max_input_samples)
d_scratch_.resize(max_output_samples)
```

其中 `max_output_samples` 必须包含一次 `process + flush` 的严格上界，而不是
经验余量。handler 中：

- `n_in > max_input_samples` 时发布 `invalid_window`（或稳定的
  `input_too_large`）并不输出 packet；
- 输出容量不足视为内部契约错误，发布 status 并拒绝该 PDU；
- 删除所有 handler 路径上的 `resize` / `reserve` / vector growth。

既有默认容量可保持当前 scheduled/Radar 窗兼容，但必须公开为可审计的配置。
不能仅把 `reserve` 当作“禁止扩容”的证明。

### 必须新增 QA

- 用 SC16 和 FC32 各跑多次最大允许窗口，记录所有 scratch 的
  `data()` / `capacity()` 不变；
- 发送 `max_input_samples + 1` 的 CF32 和 SC16 PDU，断言稳定 status、0 输出、
  没有 capacity 变化；
- 保持短 PDU、空 PDU、SC16 极值、错误 rate、short guard 和现有实际 dump 回归；
- 在约 75 µs（55650 native 样点）窗口的性能统计中，先做一次 warm-up，
  单独记录 handler mean/P95/P99，不能将首次建容成本混入稳态结论。

---

## 5. 阻断 R3：metadata 完整性与坐标极值 QA 不足

### 问题

`test_pdu_radar_metadata_whitelist` 向输入写入了很多字段，但只断言了其中一部分。
尚未逐项断言 `schedule_index`、`rx_time_full/frac`、`calibration_id`、
`code_index`、`uhd_error`、`sfd_samples`、`range_guard_samples`、
`tx_packet_samples`、`rx_capture_samples` 及相应 `*_native`。

此外重采样器仍直接计算：

```cpp
ws + pre_in
ws + pre_in + cap_in
sc - pre_in - cap_in
```

不可信 metadata 接近 `INT64_MAX` / `INT64_MIN` 时可发生有符号溢出。这与 Step 3–4
中已采用的 checked coordinate arithmetic 不一致。

### 必须整改

- 将所有 Radar whitelist 字段改为表驱动 QA：输入存在时逐一验证输出存在、
  类型正确、值正确；index 验证 `map(p)` 与 `*_native`，length 验证
  `map(n)-map(0)` 与 `*_native`；time/identity 字段原样保持。
- 明确 `calibration_delay_native_samples` 可为小数，work 值为
  `native * 65/48`，误差门限固定。
- 重采样 metadata 坐标计算复用 `uwb_radar_checked_math.h`，转换、加减和
  `map_input_offset_to_output()` 的参数都必须先检查；异常统一拒绝 PDU 并发布
  `invalid_metadata`，不得产出映射后坐标。
- 旧 scheduled-capture 字段的现有语义与测试必须保持。

### 必须新增 QA

- `INT64_MAX`、`INT64_MIN`、负 guard、`sample_count < pre+capture`、
  `uint64 > INT64_MAX` 等 metadata 负向用例；
- 每个用例不得输出 packet，必须有稳定 status；
- 保留并扩大 32/64/128 profile 的 Radar metadata 映射测试。

---

## 6. R4：PacketSource native / SC16 契约和 QA 缺口

这项在本次作为重要整改项；与 R1–R3 一并完成后才允许进入 Step 7。

### 问题

PacketSource 对 998.4 MHz 文件会检查最小 `SYNC+SFD` 长度；但 native 737.28 MHz
路径只检查文件非空和 packet duration `< PRI`。任意极短 native FC32 文件都可能被
当作正常 64/4z2 packet 发出。报告也已注明 SC16 虽有实现但没有独立 golden QA。

### 必须整改

- 对 native 路径建立完整包长度/geometry 契约：可由 metadata sidecar 或已审核
  packet descriptor 提供，不能把“非空”视为完整 packet；
- SC16 的交织 IQ 字节序、样点数、输出 PDU 类型与 metadata 增加独立 golden；
- 错误 dtype、错误字节数、过短 native/SC16 packet、profile 与 descriptor 不匹配
  必须在 `make()` 失败；
- 不要求本步在 PacketSource 内重采样；native TX 仍由 Step 5 PDU 65/48 处理。

---

## 7. 重新验收门槛

在以下全部满足前，禁止进入 Step 7：

- [ ] Loopback 对 rate/dtype/profile/长度字段执行严格验证；所有非法 profile
      仅 status、无 RX PDU；
- [ ] PDU resampler 的最大输入/输出 scratch 在 `make()` 固定，handler 无 vector
      扩容，超限稳定拒绝；
- [ ] Radar metadata 白名单的每个字段和 native/work 映射均有表驱动断言；
- [ ] PDU metadata 坐标极值不会溢出，错误 metadata 不输出 packet；
- [ ] PacketSource native 与 SC16 有独立完整包/格式 QA，过短或错配文件无法构造；
- [ ] 现有 MATLAB integer/fractional golden、完整 32/64/128 packet、scheduled dump
      和所有既有非 Radar 测试继续通过；
- [ ] 重新执行目标 CTest、全量 CTest、`git diff --check`，并在报告写入实际结果；
- [ ] 不宣称 Step 7、EchoTimer、UHD、X410 或 Phase A 完成。

建议拆分提交：C5a（resampler capacity + metadata safety）、C5b（完整 metadata
QA）、C6a（PacketSource descriptor/SC16）、C6b（Loopback profile validator/QA）。
