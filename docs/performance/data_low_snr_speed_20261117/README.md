# 2026-11-17 低 SNR 计算耗时结果归档

`ab_final.json` 原样取自原实验 `/tmp/opencode/low_snr_eval/ab_final.json`，
为基线 f5eadbb 与优化版 ea291ca 的 ABBA 摘要，不是本次整改重新测量。
环境和验收边界见[开发报告](../低SNR解调计算耗时优化开发报告_2026-11-17.md)。

历史 runner 的 noise 两档使用同名键，文件只保留最后的 tone-off 档；
未保存全套逐次日志、夹具哈希与运行环境快照，无法补造缺失证据。
整改后的[工具](../../../tools/rx/low_snr_eval/README.md)保留独立键、原始运行值、
二进制/夹具哈希和环境信息，提供可移植生成命令。
