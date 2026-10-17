# OpenCode Session S2-B 入口

只在当前 worktree 工作。本目录代码基线 93c80b3；本轮需求/AGENTS 是待实现交接文件，
尚未实施 S2 功能，也未运行 S2 测试。不要因文档存在就宣布开发完成。

启动提示（可直接粘贴给本 session）：

> 执行本 worktree 的 S2-B 任务。先完整阅读 AGENTS.md、
> docs/sic/S2_并行开发共同约定.md 和 docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md。
> 先写设计和针对性 QA，再实现、构建、smoke、正式效果矩阵、独立 verifier 和开发报告。
> 冻结共享 SIC 数学核心；不操作其他 worktree。不要因为效果不达标降低门槛。
> 本任务是开发与测试，不是只输出计划；完成后报告真实结果、失败和未实跑项。

先核对目录和分支：

```bash
pwd
git status --short --branch
git rev-parse HEAD
git worktree list
```

独立构建起步命令（标准构建入口，未在本轮执行；若依赖缺失如实处理）：

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-payload/lib/uwb_qa_uwb_preamble_sic_core.cc
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-payload --output-on-failure -R preamble_sic
```

核对 ldd 指向本 worktree 的 build-sic-payload/lib；不要指向父目录 build-sic 或 /usr/local。
新功能 CLI 尚未实现，由本 session 开发后补真实 README、测试命令和产物路径。
大产物写 gr-uwb/build-sic-payload/results，不自动写到父仓库、不自动 git merge。
UWB_demodulation 是共享只读软链接；请勿覆盖。

