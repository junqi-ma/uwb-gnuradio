# UWB GNU Radio Project

## 开发规范与参考

### 主要开发需求参考
请参考同目录下的 **开发需求参考.md**（详细的 GNU Radio UWB Packet 检测与截取模块开发需求）。

### 当前开发状态
请参考同目录下的 **开发状态.md**（已完成的模块、算法参数、测试结果、构建运行方法与下一步计划）。

### 周期旁路雷达截取（Phase-1 已完成）
请参考 **开发总结_QM35825周期旁路雷达截取.md** 与 **GROK_X410_QM35825周期旁路开发方案.md**。
已知 `t0/T` 时使用 `UwbScheduledExtractor`，不要用能量门对每个通信包建 Region。

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

Build a high-throughput GNU Radio C++ module that detects UWB packets
from continuous high-rate IQ data and extracts packet IQ for offline
MATLAB processing.

Initial goal is packet detection and extraction, NOT full UWB demodulation.
