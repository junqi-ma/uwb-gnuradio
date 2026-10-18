# S2 双方向 worktree 交接

> 2026-10-18 更新：A/B 开发与已列整改已验收，A 冻结候选 `2c383d5`，B 为 `ac3849e`。
> 下一步本地冻结/集成执行依据为 **[S2_AB冻结与合并执行指南_2026-10-18.md](S2_AB冻结与合并执行指南_2026-10-18.md)**。
> 指南已编写。本地 annotated tag 与证据清单已完成，见
> **[S2_AB冻结清单_2026-10-18.md](S2_AB冻结清单_2026-10-18.md)**。合并尚未执行。
> 下文是 2026-10-17 初始交接记录，保留历史语境。

创建日期：2026-10-17；共同代码基线：93c80b3。S2 尚未实施。
本文件只作集成入口；两个 session 的完整需求、计划和 AGENTS 在各自 worktree。

| 方向 | 分支 | 目录（相对仓库根） | 交接入口 |
|---|---|---|---|
| A：累加后 CIR 与残余相干性 | feature/sic-sensing-accum | .worktrees/sic-sensing-accum | OPENCODE_START.md |
| B：感知前导干扰通信 payload | feature/sic-payload-collision | .worktrees/sic-payload-collision | OPENCODE_START.md |

在两个目录分别启动 OpenCode。各自根目录的 OPENCODE_START.md 有可粘贴任务与构建命令。
每个目录包含 docs/sic/S2_并行开发共同约定.md 及本方向 OpenCode开发指示_S2A/S2B_*.md。
文档改动留在工作树待各 session 纳入自己的提交；此时没有新增算法代码。

约束：共享 SIC 核心先冻结；独立 build 和 results；效果开发可并行，正式性能测试串行且
使用 /tmp/uwb-sic-benchmark.lock 并协调另一 session 暂停重负载工作。
两边 UWB_demodulation 链接同一未跟踪 MATLAB 资料，仅供只读参考，禁止写入。
现有 feature/sic-validation worktree 完全保留，不用于本次并行实现。
需求文档已明确暂缓 native 和 FPGA，不因旧 S1 范围限制而阻止新的 B 方向任务。

A 交付 docs/sic/S2A_累加后CIR开发报告.md；
B 交付 docs/sic/S2B_感知前导干扰通信Payload开发报告.md。
共享核心发现需先形成问题报告，不在两个分支各自独立优化；集成与跨分支修复由用户确认。
不自动合并两个方向或推送远程。
