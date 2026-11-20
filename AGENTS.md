# 项目开发约定

当前分支：`refactor/openuwb-arch`（Phase 0–4 架构重构已完成）。当前能力、入口和
验证边界见[开发状态](开发状态.md)；构建和参数见
[原生 UHD/C++ 使用说明](docs/原生UHD与C++构建使用说明.md)；验收数据见
[重构验收报告](docs/performance/openUWB架构重构验收报告_2026-11-20.md)。
历史低 SNR 优化在 `feature/sic-research`；相关结果为软件/文件回放，未上板。

## 工作范围与边界

- 按用户本次任务确定范围，不自动推进硬件、TWR 或 SIC 里程碑。
- 后续开发统一使用原生 UHD/C++，从仓库根目录构建。
- 目标与目录：`openuwb_support`、`openuwb_dsp`、`openuwb_phy_hrp`、
  `openuwb_sic`、`openuwb_sensing`、`openuwb_twr_core/phy`、`openuwb_radio`、
  `openuwb_runtime`；头文件 `include/openuwb/<domain>/`，源码 `lib/<domain>/`。
  依赖严格单向：`support → dsp → phy_hrp → {sic,sensing,twr_core}`，
  `runtime → phy_hrp + sic + sensing + radio`，只有 `openuwb_radio` 可链接 UHD。
- 历史 GNU Radio 实现在 `archive/gnuradio-adapter`（`d5dfd30`）。过渡兼容层
  `include/gnuradio/uwb/`、`gr::uwb` 别名、`uwb::` 包与 `uwb_*` 应用软链全部
  标注 `TODO(openuwb-compat-removal)`，清除顺序见验收报告 §4.5。
- 保留分数网格、融合失败回退；timed 无 replay 必须 fail-closed。
- `openuwb_rx` 默认 `--fused on --cir full --taps eq16`。Python wrapper 为兼容入口，
  默认 `--cir auto`；需要其他 C++ 参数使用 `--` 透传。
- 不把 SFD-only 进锁、前端种子数写成完整帧 FCS 成功率；区分仿真、回放和上板证据。

## 开发与验证

- 完整包消费者使用 `include/openuwb/phy/hrp/rx/demod_pipeline.h` 并链接
  `openuwb::phy_hrp`（或 `openuwb::runtime`）；单阶段 QA 可用 `demod_core.h`
  聚合入口。TX/RX 共用编码原语放 `include/openuwb/phy/hrp/common/codec.h`。
- 修改前检查相关实现、测试和报告。保持逐位一致性约定，不放宽既有 QA 容差。
- UWB 检测/解调算法参考 `UWB_demodulation/` MATLAB 实现；检测位置最终须逐样点/
  逐包交叉验证。未做的 MATLAB 验证明确列出。
- 修改代码后构建并运行相关测试。默认验证：

  ```sh
  env -u LD_LIBRARY_PATH cmake --preset native-avx2
  env -u LD_LIBRARY_PATH cmake --build --preset native-avx2 -j
  env -u LD_LIBRARY_PATH ctest --preset native-avx2 --output-on-failure
  ```

  另外两个必跑预设是 `native-avx512` 与 `no-uhd`（`scalar-debug` 建议跑）。
  始终使用 `env -u LD_LIBRARY_PATH`，防止命中旧安装库；不把全局 install 作为
  普通测试前置条件。
- 性能测试串行、同环境对照；有调度反馈的 locked 回放采用 ABBA 配对。
  记录提交、CPU、ISA、构建参数、worker/queue、夹具种子/哈希和统计口径。
- 测试信号通常在 `testdata/`；实验摘要也在 `docs/performance/data_*`。
  新复现工具放入 `tools/`，生成数据放 build 或临时目录并记录来源。
- TWR 相关修改还须遵守历史快照中“验证环境”和“TWR 开发必须遵守”两节的约束；
  未获得 ToA/硬件证据不得声明测距有效。

## 历史记录

原有阶段报告、完整历史规则与 TWR 契约保存在 `feature/sic-research` 分支的
`docs/archive/AGENTS_2026-11-17历史快照.md`；本分支不再保留该快照。
其中分支目标、默认参数、测试数量与“本轮”仅属于对应阶段，不作为当前任务目标。
