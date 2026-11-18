# 2026-11-17 低 SNR 计算耗时结果归档

以下为原实验 `/tmp/opencode/low_snr_eval/` 中基线 `f5eadbb` 与优化版 `ea291ca`
的 ABBA 摘要。文件原样归档。环境和验收边界见
[开发报告](../低SNR解调计算耗时优化开发报告_2026-11-17.md)。

| 文件 | 在报告中的对应关系 |
| --- | --- |
| [`ab_v3.json`](ab_v3.json) | §2 的主批次。`−12 dB locked` 单 worker 2839→1383 µs，四 worker FCS 179→191/200。 |
| [`ab_p4.json`](ab_p4.json) | `+4 dB locked` 四 worker 单项复测。 |
| [`ab_m10w4.json`](ab_m10w4.json) | `−10 dB locked` 四 worker 单项复测。主批次该项有丢弃，保留在 `ab_v3.json`。 |
| [`ab_final.json`](ab_final.json) | 较早的完整批次。`−12 dB locked` 单 worker 3289→1489 µs，四 worker FCS 182→191/200。 |

历史 runner 的 noise 两档使用同名键，文件只保留最后的 tone-off 档；
未保存全套逐次日志、夹具哈希与运行环境快照，无法补造缺失证据。
整改后的[工具](../../../tools/rx/low_snr_eval/README.md)保留独立键、原始运行值、
二进制/夹具哈希和环境信息，提供可移植生成命令。
