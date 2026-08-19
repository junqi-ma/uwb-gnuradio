# 开发文档索引

仓库根目录只保留当前主线入口。这里存放已经完成的阶段总结、性能实验和历史
交接材料；归档不表示内容无效，而是避免历史状态与当前主线混杂。

## Phase-1 与硬件入口

- [`当前代码路径现状与后续开发基线.md`](当前代码路径现状与后续开发基线.md)
- [`phase1/开发总结_QM35825周期旁路雷达截取.md`](phase1/开发总结_QM35825周期旁路雷达截取.md)
- [`phase1/GROK_X410_QM35825周期旁路开发方案.md`](phase1/GROK_X410_QM35825周期旁路开发方案.md)
- [`phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md`](phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md)
- [`phase1/测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md`](phase1/测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md)
- [`phase1/测试报告_QM35盲捕获与自动周期锁定_clean737p28.md`](phase1/测试报告_QM35盲捕获与自动周期锁定_clean737p28.md)
- [`phase1/下一步_锁定后SC16截取与DW1000头尾预留.md`](phase1/下一步_锁定后SC16截取与DW1000头尾预留.md)
- [`phase1/开发需求_原生率长窗落盘与离线65_48去单音.md`](phase1/开发需求_原生率长窗落盘与离线65_48去单音.md)（已实现：热路径短窗 FIR + 737.28 长窗落盘，采集后 C++ 去单音/65/48）
- [`phase1/下一步开发计划_QM35825周期旁路.md`](phase1/下一步开发计划_QM35825周期旁路.md)
- [`phase1/SC16检测器开发与性能.md`](phase1/SC16检测器开发与性能.md)
- [`phase1/X410_RFNoC入口与写盘测试.md`](phase1/X410_RFNoC入口与写盘测试.md)
- [`phase1/当前信号检测逻辑.md`](phase1/当前信号检测逻辑.md)
- [`phase1/对照报告_QM35_MATLAB与GNURadio离线解调.md`](phase1/对照报告_QM35_MATLAB与GNURadio离线解调.md)

## 性能分析与实验记录

- [`performance/性能分析_解调分阶段耗时报告.md`](performance/性能分析_解调分阶段耗时报告.md)
- [`performance/下一步性能优化指导_CIR_CFO_SFD.md`](performance/下一步性能优化指导_CIR_CFO_SFD.md)
- [`performance/性能分析测试报告_buffer与分层吞吐.md`](performance/性能分析测试报告_buffer与分层吞吐.md)
- [`performance/性能瓶颈分析_门限与Detector合路.md`](performance/性能瓶颈分析_门限与Detector合路.md)
- [`performance/测试报告_发包率扫描.md`](performance/测试报告_发包率扫描.md)
- [`performance/GROK_性能分析测试改进方案.md`](performance/GROK_性能分析测试改进方案.md)
- [`performance/性能分析_buffer扫描结果.csv`](performance/性能分析_buffer扫描结果.csv)

## 历史快照与交接材料

- [`archive/开发状态_2026-08-09历史快照.md`](archive/开发状态_2026-08-09历史快照.md)
- [`archive/Claude开发规划经验.md`](archive/Claude开发规划经验.md)
- [`archive/GROK_接手说明.md`](archive/GROK_接手说明.md)
