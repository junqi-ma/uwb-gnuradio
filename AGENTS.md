# UWB GNU Radio Project

## 当前分支：UWB SS-TWR / DS-TWR

`feature/uwb-ds-twr` 从 radar 分支 `b897677` 创建（首个提交 `38fbe2c` 为纯文档），
同时开发 SS-TWR 与 DS-TWR。
主要需求是 **docs/twr/需求_UWB_SS_DS_TWR.md**，开发顺序和验收见
**docs/twr/开发路线与验收矩阵.md**，芯片资料和 API 映射见
**docs/twr/参考资料与API映射.md**。先读这三份文档和 **开发状态.md**。

**M0.1 第一轮已关闭 R1–R8，第二轮已关闭 N01–N06，第三轮已关闭 N07（枚举域
fail-open）；M0.1 契约层完成，TWR 协议能力仍未实现**（2026-09-29）。先读
**docs/twr/评审意见_M0.1_2026-09-29.md**（N01–N06）、
**docs/twr/M0.1第二轮整改报告_N01-N06.md**、以及
**docs/twr/M0.1_N07修复报告_2026-09-29.md**（N07：审核报 1 处，实际 20 处）。
当前顺序为 **M1 独立协议核心 → M2 PHY/时间映射 → M3/M4 上板**。
TWR 协议能力仍未实现（无 FSM、无 ToF 公式、无双端点、无 TWR 硬件收发）。

第二/三轮的关键教训（不要再犯）：

- **把错误行为写进测试约定**：N02 的 QA 曾显式断言"`scheduled_calibrated` TX 必须被拒"
  并因此把 REQ-TIME-03 明确允许的路径堵死；N01 的语料 note 描述了代码里**不存在**的机制。
  语料一致只能证明指定输入上的一致性，**不能证明这些共同约定正确**。
- **枚举的"成员"不是它的"域"**：一个枚举变量可以持有枚举没有的值。所有判定必须
  **显式做域检查并 fail-closed**，且**先于**依赖该值的检查（否则报的是无关字段）。
  `switch` 用**无 `default`** 形式以保留 `-Wswitch` 保护，域测试放在 switch 之前。
- **验证要用穷举而不是抽样**：N07 的 12 处 `TimedField.marker` 是抽样会漏、按类型注解
  穷举才抓到的；可选字段（默认 `None`）尤其容易漏。`validate()` **必须报告，不能抛异常**。
- 逐包时序由 C++/硬件定时驱动；scheduled TX 必须带 `TxSendEvidence`
  （command time / 量化时刻 / marker offset / 校准空口时间 + `outcome == Completed`），
  **`send()` 成功或 BURST_ACK 单独不构成证据**，且入门的值**绝不改标**
  `hardware_measured`。
- **16 SYNC 和 64 SYNC 一样没有 ToA 证据**，"窗更长"不是证据；
  `allows(work_decode, Ranging) = 0` 对全部 48 行成立。

M0 初版设计、回归和调度说明分别见 **docs/twr/M0_设计契约.md**、
**docs/twr/M0_回归基线.md**、**docs/twr/M0_调度语义.md**。
其中与复核报告冲突的结论不再作为冻结约束；不能用“已冻结”阻止修复已复现缺陷。

第一阶段：一台 X410 两个 channel 各作为独立逻辑端点，实现真实空口/有线的
SS/DS 测距及角色互换。第二阶段：X410 与指定 DW1000、DW3000 模组，分别完成
两种协议和两端角色互通。现有 radar、TX/RX、解调和 MATLAB 代码是复用与回归基线。

### 当前事实与必须关闭的缺口

- frame v1 为 14 B 头 + 40-bit LE timestamps；Poll/Response/Final 空口长度为
  16/26/31 B，FCS 由 HRP 层追加。它是阶段一内部协议，商用适配需另验具体固件。
  Report 保留且未实现；STS 本阶段不开发。
- 配置必须对齐 codec：修复现有 7 B 几何、session 位宽不一致以及错误的
  `phr_rate == data_rate` 约束；现有调制器是 0.85 Mb/s PHR + 6.81 Mb/s payload。
- Python effective/snapshot 共享可变对象，尚未做到不可变；必须修复并补修改隔离测试。
- 时间数学需修复 exact 标志、恰好半周期和分数排序；ToF 不使用有损整数 ns 投影，
  且显式校验 range-capable marker、校准和质量，不能只依赖 interval 返回 Ok。
- 48 行白名单证明的是 998.4 MS/s work-grid 解码自洽，未证明 native/ToA/硬件互通。
  16 SYNC 在 ToA 验证前不得用于测距。128 等暂因当前链路限制拒绝，不能写成芯片
  不支持；能力按 work/native/ToA/hardware/vendor 证据分别升级。
- 仓库已有 Python work→native resample_poly；缺口是反应式 TWR 所需的 C++ 集成、
  buffer 复用与时间映射。两路 RX、分数首径和 delayed-frame timestamp patch 仍待实现。
- controller 可采用零流端口 gr::block + 短 handler + 有界队列 + 单 owner；
  还必须限制 PMT 入口积压，其他 adapter 按实际 scheduler 语义选型。
- X4xx ctrlport `EXEC_LATE_CMDS` 与 timed TX 数据是不同路径；TX 需处理设备
  `ERR_TX_LATE_DATA` 和主机 deadline。具体 FPGA/UHD 行为需绑定版本上板验证。

### 验证环境

检查测试二进制实际加载的库。本机默认 LD_LIBRARY_PATH 可能命中旧 /usr/local 库，
CTest 脚本也会继承它；本地验证使用 `env -u LD_LIBRARY_PATH ctest ...` 并用 ldd 确认
build/lib。手工测试在环境正确时有效；不把 sudo install 作为普通测试前置条件。
吞吐 benchmark 串行、同环境对照，失败单列；CSV 输出到临时/build 目录后显式归档。

### TWR 开发必须遵守

1. 将协议 FSM、frame codec、时间戳/ToF 数学核心与 GNU Radio/UHD 适配分开。
   Python 做配置和结果消费，逐包时序由 C++ 及硬件定时驱动。
2. 复用现有 HRP TX、detector/extractor、resampler、demod 和 burst backend。
   现有双 TX/单 RX radar 路径不等于双端 TWR；需显式设计两个 RX 路由和共享设备资源。
3. 端点只使用本端硬件事件及对端空口帧携带的信息。不得用共享 schedule、私有时间戳
   或已知距离计算待测 ToF；真实有效 RX 帧才能触发 Response/Final。
4. 每个时间字段明确 clock domain、epoch、tick rate、marker、量化和校准。
   区分 RX 第一 IQ 样点、RMARKER、首径、预期时间和 TX command time。
   禁止用 radar `predicted_sfd`、最大 CIR tap 或主机收包时间直接当作实际 ToA。
5. delayed TX deadline 不满足时显式失败，不即时补发沿用旧 timestamp。
   回复延迟预算覆盖帧尾、解码、构帧、传输和设备提前量；不得承诺未经验证的短时序。
6. SS clock ratio 的方向/符号/换算及 DS 不等回复时间公式均须独立测试。
   单台 X410 共钟不替代独立时钟、回绕、CFO/SFO 和延迟校准验证。
7. 参数必须经统一 capability/profile 校验并记录 readback。未知或不支持的模式拒绝，
   不静默采用 QM35/radar 默认值；芯片 power word、TX gain、IQ amplitude、dBm 分开。
8. 商用互通绑定具体模组、SDK/固件 hash、帧格式、时间戳单位和 PHY profile。
   优先共同 channel 5 无 STS 配置；每种新增能力单独验证。原版与改时序固件分开报告。
9. 每个 exchange 有唯一 terminal result，保留 timeout、FCS 错、late、underflow、
   overflow、队列丢弃和取消。不得通过跳号补采或重试掩盖失败测量。
10. `testdata/twr/` 保存短信号/帧/time golden 和独立 MATLAB 验证脚本。
    实现前新增针对性 QA，改动后构建/测试。未完成的模组测试、精度标定与 soak
    明确写为待验证；禁止宣称厘米级精度或“完整互通”而没有相应证据。

## 开发规范与参考

### 主要开发需求参考
本分支优先参考上述 TWR 需求；同目录 **开发需求参考.md** 保留早期检测截取需求。
其中“只截取、不做实时解调”“暂不连接 USRP”等历史范围不限制当前 TWR 开发。

### 当前开发状态
请参考同目录下的 **开发状态.md**（已完成的模块、算法参数、测试结果、构建运行方法与下一步计划）。

### 周期旁路雷达截取（Phase-1 已完成）
请参考 **docs/phase1/开发总结_QM35825周期旁路雷达截取.md** 与
**docs/phase1/GROK_X410_QM35825周期旁路开发方案.md**。
已知 `t0/T` 时使用 `UwbScheduledExtractor`，不要用能量门对每个通信包建 Region。

### QM35 盲捕获与自动周期锁定（离线最小闭环已通）
未知 `t0`、已知 T=5 ms 时使用 `UwbAutoScheduledExtractorSc16`，不要并联
Detector + ScheduledExtractor。方案见
**docs/phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md**；
mixed 737.28 离线实测见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md**；
无干扰 QM35 同一流程见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_clean737p28.md**。
硬件 soak / overflow 重捕获未完成，不得写成 X410 盲捕获验收通过。

### UWB 算法与验证
- UWB 检测算法（能量门限、粗检测、细相关、preamble 匹配等）**必须**参考 `UWB_demodulation/` 目录下的 MATLAB 实现（包括 `buildUwbReference.m`、`decode_uwb.m`、`analyze_*` 等文件）。
- 算法正确性验证优先：使用 `testdata/` 下的已知 UWB 测试信号（`.cfile`、`*.dat`、`*_metadata.mat`、`UWB_test_signal_description.md`）进行 MATLAB 对照。
- 检测结果（packet start、IQ 截取范围）必须与 MATLAB 实现逐样本/逐 packet 对比。

### 开发规则（必须严格遵守）

Before implementing any GNU Radio block:

1. Search the local GNU Radio source tree for similar blocks.
2. Determine the correct block type:
   - sync_block
   - block
   - tagged_stream_block
   - message/PDU
3. Explain scheduler semantics before implementation.
4. Avoid allocating memory inside work/general_work.
5. Add QA tests before declaring implementation complete.
6. Run build and tests after each meaningful modification.

### Performance goal

Input data may eventually approach 1 GS/s.

Do NOT assume a naive full-rate correlation implementation is acceptable.

Preferred detection pipeline:

energy / coarse detection
→ candidate region
→ preamble verification
→ packet extraction

### 验证要求

- Synthetic UWB signals should be generated for unit testing (use `testdata/`).
- Detection results **must** eventually be cross-checked against MATLAB implementation in `UWB_demodulation/`.
- Add MATLAB scripts in `testdata/` or root for automated verification.

### 其他

- All test data is located in `testdata/`.
- Full MATLAB demodulation pipeline is in `UWB_demodulation/`.

## Goal

Build configurable SS-TWR and DS-TWR on the existing GNU Radio C++ UWB TX/RX
chain, with Python APIs, timestamp calibration, explicit error handling and
MATLAB cross-checks. First validate two logical endpoints on one X410, then
validate both roles against specified DW1000 and DW3000 modules. Preserve
the existing packet capture, demodulation and radar regression baselines.
