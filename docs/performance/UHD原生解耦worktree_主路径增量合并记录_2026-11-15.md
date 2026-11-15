# 主路径增量合并与复测

- 来源：`/home/oi/Desktop/uwb-gnuradio`，`feature/sic-research` 的当前未提交修改；提交仍为 `a8f0016`。
- 目标：`/tmp/uwb-gnuradio-native-cleanup`，`refactor/native-gr-decouple`。
- 以此前冻结的 `/tmp/uwb-native-cleanup-baseline` 为共同基线做三方合并，保留目标 worktree 的解耦开发。主工作区未修改；未提交、未推送、未上板。
- 来源文件 SHA256 见 `native_cleanup_main_sync_20261115.json`；原始 `native_cleanup_baseline_20261115.json` 保持不变。

## 合入内容

整数 chip 网格漂移预算/可配置 `fused_gate_ppm`、单包统一融合准入、拒绝时 Full→Bypass 降级、`grid_reject` 计数、live 默认 `--fused on --cir full`、对应单元测试。同步 AGENTS 与 X410 开发报告，新增采集/时钟诊断工具、9700X 报告及相关性能证据目录。大 IQ、临时输出和无关未跟踪文件未复制。

唯一文本冲突在 `uwb_phy_profile.h`：保留新门限字段；诊断开关仍使用解耦版 `DemodScratch::diagnose_fir_segments`，不恢复生产 profile 中已迁出的字段。合并后与来源比对，核心/QA 差异仅为既有解耦改动。

## 独立验证

| 配置 | 构建 | CTest |
| --- | --- | --- |
| GNU Radio OFF / UHD ON / experiments ON | 通过 | 53/53，22.47 s |
| GNU Radio ON / UHD ON / experiments ON | 通过 | 87/87，109.73 s |

原生 CTest 通过 `without_gnuradio.py` 隐藏系统 GNU Radio/PMT 后执行。两个临时安装前缀均刷新，安装消费者也实际执行。此次未重新跑 UHD OFF 两个配置。

短回放采用本轮默认融合/CIR 配置，4 worker、locked，时间限制 0.1 s：

| 输入 | 包数 | 网格拒绝 | 融合尝试 | FCS | 整流重试 |
| --- | --- | --- | --- | --- | --- |
| 原始 100 ms、未 trim | 20 | 19 | 1 | 19/20 | 0 |
| trim6b | 20 | 0 | 20 | 20/20 | 0 |

两组均 `raw_drop_samples=0 source_errors=0 gaps=0`。这是分支接通的冒烟测试，不是基线逐包一致性或性能验收；此前动态 locked 对照的限制仍以原独立验收报告为准。新增 Python 工具语法检查及 `git diff --check` 通过。`check_clock_offset.py` 的末尾仍沿用来源版本的旧 `1e-9` 门限说明，本轮按来源原样合入，不能用其末尾 PASS/FAIL 判断新融合门限。

构建、安装、CTest、回放日志与合并前备份在 `.opencode-task/main-sync-20261115/`（已由 Git 忽略）。
