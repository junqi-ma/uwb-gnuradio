# 当前任务：接入成熟快速 RX 并复测 Full-packet SIC

快速RX接入与复测已交付：`docs/sic/FullPacket_快速RX复测报告_2026-11-06.md`，正式代码 `28c7734`；不自动进入重构优化。历史报告 `docs/sic/FullPacket_开发报告_2026-11-06.md` 与 `5dd2cfb` 结果保留；新run独立归档。仅调整实验runner的RX路径，不改冻结SIC数学，不上板。

执行 docs/sic/FullPacket_vs_PreambleOnly_开发计划_2026-11-06.md。仅本实验分支允许新增整包路径；原 preamble 行为与历史基线保持。主 agent 与三个 subagent 按文件所有权协作。

# S2 A/B 集成基线入口

当前源码基线同时包含已验收的两个方向：

- A：累加后 sensing CIR。历史报告 `docs/sic/S2A_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2a-accepted-20261018`（`2c383d5`）。
- B：sensing 前导干扰通信 payload。历史报告 `docs/sic/S2B_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2b-accepted-20261018`（`ac3849e`）。

执行与验收记录：

- `docs/sic/S2_AB冻结与合并执行指南_2026-10-18.md`
- `docs/sic/S2_AB冻结清单_2026-10-18.md`
- `docs/sic/S2_AB集成验收报告_2026-10-18.md`
- `docs/sic/S2_AB集成回归入口_2026-10-18.md`

先读根目录 `AGENTS.md`。共享 SIC 数学核心保持冻结。不要只做 A 或只做 B。
单方向历史操作说明留在：

- `docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`
- `docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md`

原 A/B worktree 继续作为历史锚点，不在这里改它们的结果目录。

```bash
pwd
git status --short --branch
git rev-parse HEAD
git worktree list
```

两个 build 都从当前源码根串行构建。核对 `ldd` 指向本 worktree 的对应 `build-sic-*/lib`，不要指向父目录 `build-sic` 或 `/usr/local`。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-accum -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-accum -j2
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-accum/lib/libgnuradio-uwb.so.1.0.0.0
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-payload/lib/libgnuradio-uwb.so.1.0.0.0
```

`UWB_demodulation` 是指向主仓库资料的只读软链接，禁止写入。
大产物写各自 `gr-uwb/build-sic-*/results`，不覆盖历史 run，不推送远程。
