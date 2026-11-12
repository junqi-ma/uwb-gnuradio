# 结果归档范围

本目录提交配置/输入指纹、分轮统计、跨轮汇总及功能对照（存在的项目）：

- `provenance.json`
- `input_manifest.json`
- `summary.json`

逐窗原始 CSV/JSON、运行日志和生成滤波器仍保留在本地工作区，未随本次代码提交。大 IQ 使用临时文件，不进入 Git。完整运行可按对应报告及 tools/sic 下的驱动重生成；报告中的 code-tested SHA 是运行时基线加工作区改动，provenance 中的源码/二进制/库 hash 用于识别当时实际内容。
