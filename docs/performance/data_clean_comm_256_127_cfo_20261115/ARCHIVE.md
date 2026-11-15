# 结果归档范围

本目录是 clean comm（无干扰、无 SIC、无 sensing）256 SYNC / 127 B / 有 CFO 的
1·2·4 worker 耗时与阶段分解测量，对应报告
`docs/performance/clean_comm_256_127_CFO_多worker耗时与分解_2026-11-15.md`。

提交内容：

- `provenance.json`：commit、二进制/库/源码 hash、CPU 拓扑、完整命令行参数
- `fixture.json`：夹具几何、ADC 尺度、CFO 集合、16 组 PSDU 与 mixture hash
- `summary.json`：18 次运行的逐任务统计（每次 512 个任务）
- `aggregate.json`：按 (CFO, worker) 跨三轮的中位数汇总
- `clean_comm_*.csv`：逐任务原始观测（512 行 × 全部阶段/子阶段计时）

`*.log` 被仓库 `.gitignore` 排除，未提交；日志可由同一脚本重生成（本目录的
`clean_comm_*.log` 均为空，`generator.log` 仅含夹具生成输出）。

大 IQ（`mixture_*.sc16`）在临时目录生成，完成后删除，不进入 Git。
可用 `tools/sic/run_clean_comm_workers.py` 按报告中的命令重生成。
