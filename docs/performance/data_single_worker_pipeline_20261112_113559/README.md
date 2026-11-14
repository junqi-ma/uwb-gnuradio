# 单 worker pipeline 耗时检查（2026-11-12）

当前源码 64c2aee 重建 runner 及其动态库。文件回放，737.28M→998.4M，capture=56000，pre/post guard 各 400，period=100000；consumer=both；DSP/resampler worker 均为 1，RX CPU=2、DSP CPU=6，pool=64。每配置 3 次，每次 50 窗，前 10 窗预热、40 窗统计。每次 runner 用 --benchmark 独占 /tmp/uwb-sic-benchmark.lock。12 次均 50/50 完成、0 丢弃、0 处理失败。

单位 µs；表内为每次稳态 P50 的三次中位数。各阶段 P50 不可直接相加作为整体 P50。

| Profile | 重采样 | SIC | sensing | worker 整体 | 提交间隔 |
|---|---:|---:|---:|---:|---:|
| quality_minorder | 269.58 | 54.18 | 657.29 | 998.98 | 1009.70 |
| realtime_minorder | 162.76 | 53.55 | 654.71 | 878.76 | 901.14 |
| quality_eq32 | 212.54 | 53.14 | 657.21 | 946.70 | 963.06 |
| realtime_eq16 | 128.99 | 51.82 | 644.20 | 838.01 | 866.55 |

worker 整体来自代码 e2e_us：从 worker 获得就绪 slot 后开始，到 SIC/sensing 返回结束，包含转换/重采样及少量内部开销；不包含 RX 接收与组窗、队列等待、结果提交。提交间隔是 runner 的稳态提交时间跨度 / 间隔数，包含 worker 周转、提交等影响。文件全速回放的队列等待约 25–29 ms，由输入速度和队列深度决定，不代表 paced 实时延迟。

关键限制：所有稳态窗口 SIC 均未提交，radar SFD 检出为 0。SIC 状态为 below_detect_threshold / fit_residual_exceeded。本次测得完整调用链上的检测失败/旁路负载，不能认定为成功 SIC 消除、成功 CIR 提取或 full-packet 解码的耗时。consumer=both 没有单独的通信用完整包解码阶段；SIC 默认 policy=auto、backend=ipatov。这批窗口仅支持耗时检查，数量不足以支持 P99 或正式实时验收。

逐窗口原始 JSON、stdout/stderr、三次结果 summary.json、二进制及实际加载库 hash 均在本目录。
