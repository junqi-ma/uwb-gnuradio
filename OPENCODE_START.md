# OpenCode Session S2-A 入口

只在当前 worktree 工作。S2 初版已交付并完成独立复核，**整改已实施并交付**
（A-R01～A-R04），见 `docs/sic/S2A_整改开发报告_2026-10-18.md`。
整改需求原文保留在 `docs/sic/S2A_整改意见_2026-10-18.md`（历史依据）。
新一轮任务的最高优先级需求以本文件顶部的 AGENTS 段落为准；当前无未启动整改项。

启动提示（可直接粘贴给本 session）：

> （历史启动提示，整改已完成）执行本 worktree 的 S2-A 整改任务。先完整阅读 AGENTS.md、
> docs/sic/S2A_整改意见_2026-10-18.md，再读
> docs/sic/S2_并行开发共同约定.md 和 docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md。
> 按整改编号先补反例 QA，再修改、构建、复算或补跑、独立校验并提交整改开发报告。
> 冻结共享 SIC 数学核心；不操作其他 worktree。不要因为效果不达标降低门槛。
> 本任务是开发与测试，不是只输出计划；完成后报告真实结果、失败和未实跑项。

先核对目录和分支：

```bash
pwd
git status --short --branch
git rev-parse HEAD
git worktree list
```

独立构建命令（本次仅更新整改文档，未在本次重跑构建；若依赖缺失如实处理）：

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-accum -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-accum -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-accum/lib/uwb_qa_uwb_preamble_sic_core.cc
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-accum --output-on-failure -R preamble_sic
```

核对 ldd 指向本 worktree 的 build-sic-accum/lib；不要指向父目录 build-sic 或 /usr/local。
已有 CLI 与历史结果保留；整改结果为 `_rectification` 后缀的新 run 目录 +
`docs/sic/S2A_整改开发报告_2026-10-18.md`，原报告已加 superseded 提示。
大产物写 gr-uwb/build-sic-accum/results，不自动写到父仓库、不自动 git merge。
UWB_demodulation 是共享只读软链接；请勿覆盖。

