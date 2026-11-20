# 开发文档索引

仓库根目录只保留当前主线入口。这里存放已经完成的阶段总结、性能实验和历史
交接材料；归档不表示内容无效，而是避免历史状态与当前主线混杂。

## 当前主线：OpenUWB（纯原生 UHD/C++）

更新：2026-11-20。当前分支为 `refactor/openuwb-arch`，Phase 0–4 重构已完成：
仓库根即 `OpenUWB` 工程，公共头文件在 `include/openuwb/<domain>/`，命名空间为
`openuwb`，目标前缀 `openuwb_*`。GNU Radio 源码与运行入口见
`archive/gnuradio-adapter`（`d5dfd30`）。以下历史报告中的 GR 命令使用该分支。

- [OpenUWB 全仓库架构重构验收报告](performance/openUWB架构重构验收报告_2026-11-20.md)
  · [ABBA 原始数据](performance/data_openuwb_refactor_20261120/abba.json)

- [前端 SIMD 与低 SNR 捕获](performance/前端SIMD与-12dB自相关参数升级开发报告_2026-11-17.md)
- [低 SNR 解调与进锁](performance/低SNR解调与进锁优化开发报告_2026-11-17.md)
- [低 SNR 计算耗时优化](performance/低SNR解调计算耗时优化开发报告_2026-11-17.md)
- [项目结构与文档整改](performance/项目结构与文档整改报告_2026-11-17.md)
- [复现工具与统计口径](../tools/rx/low_snr_eval/README.md)


- [现行构建（预设）、架构、安装和接收参数](原生UHD与C++构建使用说明.md)
- [解调与调度分层重构报告](performance/解调与调度分层重构开发报告_2026-11-17.md)
- [解调与窗口执行代码边界](架构_解调与窗口执行.md)
- [OpenUWB 全仓库架构重构执行方案（OpenCode）](OpenCode执行方案_openUWB全仓库架构重构_2026-11-20.md)
- [统一日志与调试接口开发建议](开发建议_统一日志与调试接口_2026-11-20.md)
- [当前开发状态](../开发状态.md)
- [接收工具与 Python/C++ 入口区别](../tools/rx/README.md)
- [解耦清理回合记录](performance/UHD原生解耦清理_回合主路径记录_2026-11-16.md)
- [解耦独立验收及限制](performance/UHD原生链路_GNURadio解耦独立验收_2026-11-15.md)
- [X410 周期锁定、融合门限与实采报告](performance/X410_目标周期锁定与随机干扰双路径开发报告_2026-11-15.md)
- [X410 2 ms 周期收包解调报告](performance/X410_2ms周期收包解调报告_2026-11-15.md)
- [X410 接收信噪比与链路余量（10 dB 衰减 / 1.5 m）](performance/X410_接收信噪比与链路余量_2026-11-16.md)
- [AVX-512 内核与编译期 CPU 选择（65/48、65/32 重采样）](performance/AVX512内核与编译期CPU选择开发报告_2026-11-16.md)
- [解调与 SIC 的 AVX-512 内核扩展（sync 组默认启用）](performance/解调与SIC_AVX512内核扩展开发报告_2026-11-16.md)

## TWR 独立阶段记录

TWR 已包含离线协议核心与 native PHY 阶段工作；硬件测距不在已验收范围。
这些记录不代表当前分支仍处于需求阶段。

- [需求_UWB_SS_DS_TWR.md](twr/需求_UWB_SS_DS_TWR.md)
- [开发路线与验收矩阵.md](twr/开发路线与验收矩阵.md)
- [M2-A 最终实现评审报告](twr/M2-A_最终实现评审报告.md)
- [参考资料与API映射.md](twr/参考资料与API映射.md)

## 硬件环境与算法原理

- [`本机UHD_DPDK_AVX512问题与绕过.md`](本机UHD_DPDK_AVX512问题与绕过.md)
- [`DPDK_X410_CG600启用.md`](DPDK_X410_CG600启用.md)
- [`decoder_CFO_SFD码栅格与成形群时延.md`](decoder_CFO_SFD码栅格与成形群时延.md)

## 性能分析与实验记录

- [`performance/性能分析_解调分阶段耗时报告.md`](performance/性能分析_解调分阶段耗时报告.md)
- [`performance/下一步性能优化指导_CIR_CFO_SFD.md`](performance/下一步性能优化指导_CIR_CFO_SFD.md)
- [`performance/性能分析测试报告_buffer与分层吞吐.md`](performance/性能分析测试报告_buffer与分层吞吐.md)
- [`performance/性能瓶颈分析_门限与Detector合路.md`](performance/性能瓶颈分析_门限与Detector合路.md)
- [`performance/测试报告_发包率扫描.md`](performance/测试报告_发包率扫描.md)
- [`performance/GROK_性能分析测试改进方案.md`](performance/GROK_性能分析测试改进方案.md)
- [`performance/性能分析_buffer扫描结果.csv`](performance/性能分析_buffer扫描结果.csv)
