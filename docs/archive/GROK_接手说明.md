# Grok 接手说明：UWB detector 性能优化工作区

更新时间：2026-08-08

## 1. 当前暂停点

用户要求在“方向 2”完成后暂停，由 Grok 接手后续工作。由于该指令到达时方向 3、4 的代码已经写入工作区，因此当前工作区实际包含方向 1～4 的未提交修改；这些优化尚未形成 commit，请先审查和拆分，不要直接执行 `git reset --hard` 或覆盖整个工作区。

当前已提交基线：

```text
67609b0 Document packet-rate benchmark and detector baseline
9f99b75 Initial commit: gr-uwb UWB packet detect/extract OOT
```

## 2. 工作区状态

相对于 `67609b0`，当前有 7 个已跟踪文件被修改，约 467 行新增、88 行删除：

```text
gr-uwb/apps/benchmark_detector.cc
gr-uwb/include/gnuradio/uwb/uwb_detector.h
gr-uwb/include/gnuradio/uwb/uwb_detector_core.h
gr-uwb/lib/qa_uwb_detector.cc
gr-uwb/lib/uwb_detector.cc
../../开发状态.md
../performance/测试报告_发包率扫描.md
```

另有补丁工具产生的临时文件，提交前应在确认其内容不再需要后删除：

```text
gr-uwb/include/gnuradio/uwb/uwb_detector.h.orig
gr-uwb/include/gnuradio/uwb/uwb_detector.h.rej
gr-uwb/lib/uwb_detector.cc.orig
../../开发状态.md.orig
../performance/测试报告_发包率扫描.md.orig
```

## 3. 已实现修改

### 方向 1：减少静默期逐样本热路径开销

- 将 detector core 的 `SEARCH` 和 `IN_REGION` 路径拆开。
- 静默期以 D=100 的 gate block 为单位循环，仅计算每块前 16 个样本的能量。
- pre-trigger ring 增加批量 `push()`，使用至多两次 `memcpy` 处理环形跨界。
- ring 增加批量 `copy_to()`，输入超过容量时只保留最新样本。
- 正常 `work()/process()` 路径避免临时容器分配。

### 方向 2：只在触发后物化候选 IQ

- 构造期预分配 8 个固定 `Region` 槽位。
- 每个槽位预留 `max(pre_trigger + 300000, 524288)` 个 complex 样本。
- 静默期只维护较小的 pre-trigger ring；能量门触发后才把 IQ 复制到 region。
- 使用固定 free/ready handle 数组管理槽位，提供 `take_region()`、`region()`、`release_region()` 和 `dropped_regions()`。
- region 超过固定容量时丢弃并累计计数，避免在实时热路径扩容。

### 方向 3：后台相关与 PDU 发布（已写入，但应由 Grok重点审查）

- `UwbDetector` 在 `start()` 创建一个 worker，在 `stop()` 排空并 join。
- GNU Radio scheduler 线程只运行状态机并把完成的 region handle 放入有界 FIFO。
- worker 按 FIFO 顺序执行粗相关、精相关、PDU 构造和消息发布，然后释放 region 槽位。
- worker 捕获异常，确保槽位能够归还。
- 实现参考了本地 GNU Radio `gr-zeromq/lib/req_msg_source_impl.cc` 的 start/stop/background-thread 生命周期。

### 方向 4：有限流结束与 benchmark 修正（已写入，但应由 Grok重点审查）

- 对有限输入保留一个 sentinel 样本：上游结束时先排空后台任务，再消费最后一个样本，避免最后 PDU 在 message sink 已停止后才发布。
- `PatternSource` 修复最终一次 `work()` 已产生数据却直接返回 `WORK_DONE`、导致最后一批样本丢失的问题。
- benchmark 增加 `dropped regions` 输出。
- 这一部分依赖 GNU Radio scheduler 的 EOS/message 生命周期，建议重点核对 `gnuradio-runtime/lib/block_executor.cc`、`tpb_thread_body.cc` 和 `sync_block.cc`。

## 4. 已增加 QA

`gr-uwb/lib/qa_uwb_detector.cc` 新增或扩展了以下检查：

- ring 批量写入跨界，并只保留最新容量的数据。
- 连续输入两个 packet，验证输出 PDU 数量和顺序。
- PDU payload IQ 与输入捕获区逐样本完全一致。
- 有限流结束前异步任务正确排空。
- `dropped_regions() == 0`。

优化代码完成后曾执行：

```bash
cd gr-uwb/build
make -j8
ctest --output-on-failure
```

当时结果为 **4/4 通过**：energy detector、preamble detector、detector、packet writer。此后又修改了注释和 Markdown，因此接手后仍需重新执行完整构建与测试。

## 5. 性能复测结果

| 模式 | 优化前 | 当前工作区 | 变化 | 结果 |
|---|---:|---:|---:|---:|
| 仅能量门，10 包/s，200M samples | 856.0 MS/s | 2241.4 MS/s | +162% | 3 regions |
| 完整 detector，200 包/s，500M samples | 151.6 MS/s | 178.3 MS/s | +17.6% | 101 detections，0 dropped |
| 完整 detector，1 包/s，1G samples | 189.0 MS/s | 231.5 MS/s | +22.5% | 2 detections，0 dropped |

参考命令：

```bash
gr-uwb/build/apps/benchmark_detector testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile detector-gate --target 200000000 --gap 99583552
gr-uwb/build/apps/benchmark_detector testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile detector-sparse --target 500000000 --gap 4735552
gr-uwb/build/apps/benchmark_detector testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile detector-sparse --target 1000000000 --gap 998143552
```

当前稀包吞吐约 231.5 MS/s，已经接近同一 benchmark 中 `PatternSource → null_sink` 约 264 MS/s 的 plumbing 上限，但仍远未达到 1 GS/s。不要把 2.24 GS/s 的 gate-only 数字误认为完整 detector 吞吐。

## 6. 已知未收尾事项

1. 方向 1/2 和方向 3/4 目前混在同一个未提交 diff 中；若要严格保留“方向 2 完成点”，请用交互式暂存或重新构造 patch 拆分，禁止粗暴重置。
2. 检查 `uwb_detector.h`、`../../开发状态.md` 中新加行是否混入 CRLF；执行 `git diff --check` 并清除尾随 `CR`/空白。
3. 删除上述 `.orig`、`.rej` 前先确认其中没有主文件缺失的修改。
4. 核对 `../performance/测试报告_发包率扫描.md` 的章节顺序应为第 6 节、第 7 节、第 8 节。
5. 审查 8 槽 region 池满时的策略。目前是丢弃并计数；需要确认生产环境希望丢包、阻塞还是扩大池。
6. 审查最大 region 容量 524288 是否覆盖实际最大 burst；超过后当前会丢弃。
7. 审查 sentinel 方案对持续 UHD source、有限文件源、异常 stop 和无尾部静默四种情况的行为。
8. Python binding 尚未暴露 `dropped_regions()`；如果运行时需要从 Python 监控，应补 binding 和测试。

## 7. 建议 Grok 的执行顺序

1. 阅读 `../../AGENTS.md`、`../../开发需求参考.md`、`../../开发状态.md` 和本文件。
2. 用 `git diff 67609b0` 审查当前修改；重点看 core 的所有权、容量边界和线程生命周期。
3. 先修复临时文件、CRLF、尾随空白和文档章节，再运行 `git diff --check`。
4. 重新构建并执行全部 CTest。
5. 增加 region 池耗尽、超长 region、stop/EOS、worker 异常等压力 QA。
6. 运行三组 benchmark，并同时记录 detections、dropped regions、captured IQ、wall time 和 CPU%。
7. 如果保留方向 3/4，实现和测试均确认无误后再提交；建议将方向 1/2 与方向 3/4拆成两个 commit，便于回退和性能二分。
8. 下一轮性能拆分使用相同拓扑依次测 `真实 UHD/source-only → no-op sink → gate-only → full detector`，避免把 PatternSource 或 GNU Radio buffer 上限误判为算法成本。

## 8. 提交前最低验收标准

- `git diff --check` 无输出。
- 不包含 `.orig`、`.rej`。
- CTest 4/4 通过。
- 双包 PDU 顺序和 IQ 逐样本对比通过。
- 三组 benchmark 的 `dropped regions` 均为 0。
- 文档中的性能数字与本次实际命令输出一致。
- `git status` 只包含计划提交的文件。
