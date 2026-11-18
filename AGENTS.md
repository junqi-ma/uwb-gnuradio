# 项目开发约定

当前分支：`feature/sic-research`。当前能力、入口和验证边界见
[开发状态](开发状态.md)；构建和参数见
[原生 UHD/C++ 使用说明](docs/原生UHD与C++构建使用说明.md)。
最新低 SNR 优化已通过 `63c057f` 合入；相关结果为软件/文件回放，未上板。

## 工作范围与边界

- 按用户本次任务确定范围，不自动推进硬件、TWR 或 SIC 里程碑。
- 后续开发统一使用原生 UHD/C++。
- `uwb_core` 为纯 DSP，`uwb_native` 为原生运行时。历史 GNU Radio 实现在
  `archive/gnuradio-adapter`（`d5dfd30`）。
- 保留分数网格、融合失败回退；timed 无 replay 必须 fail-closed。
- C++ live 默认 `--fused on --cir full --taps eq16`。Python wrapper 为兼容入口，
  默认 `--cir auto`；需要其他 C++ 参数使用 `--` 透传。
- 不把 SFD-only 进锁、前端种子数写成完整帧 FCS 成功率；区分仿真、回放和上板证据。

## 开发与验证

- 完整包消费者使用 `uwb_demod_pipeline.h` 并链接 uwb_core；单阶段 QA 可用
  `uwb_demod_core.h` 聚合入口。TX/RX 共用编码原语放 uwb_hrp_codec.h。
- 修改前检查相关实现、测试和报告。保持逐位一致性约定，不放宽既有 QA 容差。
- UWB 检测/解调算法参考 `UWB_demodulation/` MATLAB 实现；检测位置最终须逐样点/
  逐包交叉验证。未做的 MATLAB 验证明确列出。
- 修改代码后构建并运行相关测试。验证使用 `env -u LD_LIBRARY_PATH`，防止命中旧安装库；
  不把全局 install 作为普通测试前置条件。
- 性能测试串行、同环境对照；有调度反馈的 locked 回放采用 ABBA 配对。
  记录提交、CPU、ISA、构建参数、worker/queue、夹具种子/哈希和统计口径。
- 测试信号通常在 `testdata/`；实验摘要也在 `docs/performance/data_*`。
  新复现工具放入 `tools/`，生成数据放 build 或临时目录并记录来源。
- TWR 相关修改还须遵守历史快照中“验证环境”和“TWR 开发必须遵守”两节的约束；
  未获得 ToA/硬件证据不得声明测距有效。

## 历史记录

原有阶段报告、完整历史规则与 TWR 契约保存在
[2026-11-17 历史快照](docs/archive/AGENTS_2026-11-17历史快照.md)。
其中分支目标、默认参数、测试数量与“本轮”仅属于对应阶段，不作为当前任务目标。
