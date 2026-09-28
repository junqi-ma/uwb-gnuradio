# UWB GNU Radio Project

## 当前分支：UWB SS-TWR / DS-TWR

`feature/uwb-ds-twr` 从 radar 分支 `b897677` 创建，同时开发 SS-TWR 与 DS-TWR。
主要需求是 **docs/twr/需求_UWB_SS_DS_TWR.md**，开发顺序和验收见
**docs/twr/开发路线与验收矩阵.md**，芯片资料和 API 映射见
**docs/twr/参考资料与API映射.md**。先读这三份文档和 **开发状态.md**。
当前仅完成需求文档，不能将设计中的 Python API 或 TWR 能力描述为已实现。

第一阶段：一台 X410 两个 channel 各作为独立逻辑端点，实现真实空口/有线的
SS/DS 测距及角色互换。第二阶段：X410 与指定 DW1000、DW3000 模组，分别完成
两种协议和两端角色互通。现有 radar、TX/RX、解调和 MATLAB 代码是复用与回归基线。

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
