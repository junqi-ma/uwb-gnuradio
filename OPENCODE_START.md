# OpenCode Session S2-B 入口

只在当前 worktree 工作。S2 初版已交付并完成独立复核，目前进入整改阶段。
最高优先级需求为 `docs/sic/S2B_整改意见_2026-10-18.md`；本轮整改尚未实施。

启动提示（可直接粘贴给本 session）：

> 执行本 worktree 的 S2-B 整改任务。先完整阅读 AGENTS.md、
> docs/sic/S2B_整改意见_2026-10-18.md，再读
> docs/sic/S2_并行开发共同约定.md 和 docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md。
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
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-payload/lib/uwb_qa_uwb_preamble_sic_core.cc
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-payload --output-on-failure -R preamble_sic
```

核对 ldd 指向本 worktree 的 build-sic-payload/lib；不要指向父目录 build-sic 或 /usr/local。
已有 CLI 与历史结果保留；整改后补真实 README、测试命令、新产物路径和 superseded 说明。
大产物写 gr-uwb/build-sic-payload/results，不自动写到父仓库、不自动 git merge。
UWB_demodulation 是共享只读软链接；请勿覆盖。

## 整改状态（2026-10-18，已回填）

S2-B 整改 B-R01～B-R04 已实施并实测：PER 置信区间改为误差比例 Wilson；B3 补齐为 12 cell
并使用 `testdata/sic_preamble/cases/main.json` 冻结 four-path；verifier fail-closed
（17 个负样本）；`plan.json`/退出码/最差窗归档均已落地。详见
**docs/sic/S2B_整改开发报告_2026-10-18.md**。原报告 `S2B_感知前导干扰通信Payload开发报告.md`
的 CI 口径已被该整改报告取代（派生重算在 `results/s2b_matrix_ci_fix/`）。MATLAB 仍未实跑。

