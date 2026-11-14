# Pipeline 落盘与实时日志：多 Agent 执行计划

对应任务书 `docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent开发方案_2026-11-14.md`
（下称"任务书"）。本文件只描述**怎么拆、谁改哪个文件、按什么顺序合并、用什么门槛验收**，
不重复任务书里的格式规范。

## 0. 基线

- worktree `.worktrees/sensing-shared-cir`，分支 `feature/sensing-shared-cir`，
  `HEAD = a5d2a9e`（上一轮"单窗端到端低垂果实优化"的收尾提交）。
- build `gr-uwb/build-sensing-shared-cir-r3`，`CMAKE_BUILD_TYPE=Release`、
  `ENABLE_TESTING=ON`、`ENABLE_UHD_BACKEND=AUTO`（本机无 UHD，等价 `UWB_HAVE_UHD=0`）。
- 基线回归：**88 项，87 通过，1 跳过（`uwb_qa_install_consumer` 按设计跳过），0 失败**，
  123.44 s。

## 1. 为什么不能直接按任务书 §5 的三条 Track 并行

任务书 §5 的三个 Track 存在**硬依赖**：

```
Track 2 (pipeline 集成)  →  需要 Track 1 的 OutputPayloadSlot / PipelineOutputWriter 声明
Track 3 (runner CLI)     →  需要 Track 2 引入的 PipelineConfig::output 字段
```

如果三个 Agent 各开 worktree 直接开工，Track 2 和 Track 3 **编译都过不了**
（看不到对方的头文件），只能靠"写完再合并"交付，等于没有验收。

因此本轮把**接口冻结**提前到 Phase 0，由协调者一次性完成，之后三条 Track 才真正互不阻塞：

```
Phase 0（协调者，1 次提交）
  冻结 3 个头文件 + CMake 注册 + 可编译桩
        │
        ├── Track 1（并行）writer 内核：UCR4 / JSONL / run.json / demod_results.jsonl
        │                            / 聚合写盘 / LiveConsoleLogger + 单元 QA
        ├── Track 2（并行）pipeline 集成：槽位池 / commit_result 顺序入队 / stop-wait drain
        │                            + 集成 QA（多 worker 顺序性）
        └── Track 3（并行）runner CLI + 端到端格式兼容 QA（独立解析器）
        │
Phase 2（协调者）
  按 T1 → T2 → T3 合并，全量 CTest，真实 E2E（4 种 consumer × clean/jam），报告
```

## 2. 文件所有权（严格互斥，任何 Track 越界即视为缺陷）

### Phase 0 冻结，任何 Track 不得修改

| 文件 | 内容 |
|---|---|
| `gr-uwb/include/gnuradio/uwb/uwb_pipeline_output_types.h` | `LiveLogMode`、`OutputPersistenceConfig`、`OutputWriterStats`、`OutputRunMeta` |
| `gr-uwb/include/gnuradio/uwb/uwb_pipeline_output_writer.h` | `OutputWindowRecord`、`OutputPayloadSlot`、`PipelineOutputWriter` |
| `gr-uwb/include/gnuradio/uwb/uwb_uhd_sc16_pipeline.h` | `PipelineConfig::output`、`output_writer_stats()`、私有 `output_writer_` |
| `gr-uwb/lib/CMakeLists.txt` | 注册 writer `.cc` 与 `qa_uwb_pipeline_output_writer.cc` |
| `gr-uwb/apps/CMakeLists.txt` | 注册 `qa_uwb_pipeline_output_runner.sh` |

### Track 1（worktree `.worktrees/pl-1`，分支 `perf/pl-track-1`）

- `gr-uwb/include/gnuradio/uwb/uwb_ucr4_format.h`（新增）
- `gr-uwb/lib/uwb_pipeline_output_writer.cc`（把 Phase 0 的桩实现成真）
- `gr-uwb/lib/qa_uwb_pipeline_output_writer.cc`（把 Phase 0 的桩换成真 QA）

### Track 2（worktree `.worktrees/pl-2`，分支 `perf/pl-track-2`）

- `gr-uwb/lib/uwb_uhd_sc16_pipeline.cc`
- `gr-uwb/lib/qa_uwb_uhd_sc16_pipeline.cc`（追加用例，不改既有用例语义）

### Track 3（worktree `.worktrees/pl-3`，分支 `perf/pl-track-3`）

- `gr-uwb/apps/uwb_uhd_sc16_pipeline_runner.cc`
- `gr-uwb/apps/qa_uwb_pipeline_output_runner.sh`（把 Phase 0 的桩换成真）
- `tools/performance/verify_ucr4_output.py`（新增，独立解析器）

**禁止**：任何 Track 修改 `uwb_radar_cir_core.h` / `uwb_demod_core.h` /
`uwb_sic_unified.*` / `uwb_discrete_cir_estimator.cc` / 其它 QA 文件 / 根 `CMakeLists.txt`。
禁止新建需要额外 CMake 注册的源文件（CMake 已在 Phase 0 冻结）。

## 3. 冻结接口的不变式

头文件是唯一事实来源，这里只列**必须由 QA 钉死的语义**：

1. **关闭即零成本**：`cfg.output.output_dir.empty() && live_log_mode == Off` 时，
   `initialize()` 不创建 writer；DSP worker 热路径只多一次 `if (output_writer_)`。
   0 分配、0 锁、0 时钟读取、0 行为变化。
2. **槽位池非阻塞**：`try_acquire_slot()` 池空时返回 `nullptr` 并累加
   `offered/dropped`；`try_submit()` 队满/未运行时把槽位**自己**归还池并累加
   `dropped` 后返回 `false`。调用方绝不重试、绝不阻塞。
3. **账本守恒**：`stop_and_drain()` 返回后必须满足
   `offered == written + dropped`；`cir_written + cir_failed == 成功提交的 sensing 帧`
   （失败帧只写 jsonl，不写 ucr4、不推进 `file_offset_taps`）。
4. **严格顺序**：入队只发生在 `commit_result()` 的按 `window_id` 递增 emit 段内，
   因此 `.ucr4` / `.jsonl` / `demod_results.jsonl` / 终端日志都严格按 `window_id` 升序。
5. **drain 有界**：`UhdSc16Pipeline::stop()` / `wait()` 返回前必须
   `output_writer_->stop_and_drain()`，落盘完所有在途帧；不得丢帧退出。
6. **`WindowResult` 不内嵌变长数组**：`results_` 只保留既有 POD 字段，
   taps/PSDU 只存在于 `OutputPayloadSlot`，写完立即归池。
7. **聚合写盘字节级等价**：`aggregate_bytes = 0`（逐帧写）与 `1 MiB / 50 ms`
   （聚合写）产生的 `.ucr4` 与 `.jsonl` 必须**逐字节相同**。
8. **UCR4 头 52 字节**且与 `uwb_cir_writer.h` 的 `Ucr4Header` 逐字段一致
   （`static_assert(sizeof(Ucr4Header) == 52)` 在纯 C++ 头里再断言一次）。
9. **实时日志不阻塞**：`LiveConsoleLogger` 只在后台线程输出；`Off` 时零输出。
   `summary` 模式的周期由 `live_log_interval_ms` 控制，`events` 只在状态变化时打印。

## 4. 各 Track 的交付物与验收门槛

### Track 1 — writer 内核

交付：
- `uwb_ucr4_format.h`：纯 C++ `Ucr4Header`（52 B）、`encode_cir_sc16()`、
  `decode_cir_sc16()`、`cir_status_name()`、`ucr4_record_bytes()`。
- `uwb_pipeline_output_writer.cc`：`free_pool_`、有界 `write_queue_`、单后台
  `writer_loop()`、`run.json`、聚合缓冲、`LiveConsoleLogger`、`stats()`。
- `qa_uwb_pipeline_output_writer.cc`：见下。

验收（全部必须在自己的 worktree 里跑通）：
1. `sizeof(Ucr4Header) == 52`；块浮点 `encode → decode` 往返误差 ≤ 1 LSB，
   且 `scale == 0` 的零帧不产生 NaN。
2. 逐帧写 vs 聚合写：`.ucr4` 与 `.jsonl` **字节级相同**（`memcmp`）。
3. 失败帧（`status != "ok"` 或 `tap_count == 0`）只写 jsonl、不写 ucr4、
   `file_offset_taps` 不推进；成功帧推进量 == `tap_count`。
4. `demod_results.jsonl` 每行能被 `gr::uwb::core::parse_demod_jsonl_line()`
   解析回 `packet_id` / `status` / `fcs_pass` / `detected_start_sample`；
   `psdu_hex` 与 `comm_psdu_fnv1a` 自洽（同一字节序列）。
5. 慢写注入（`queue_capacity = 4` + 每帧 sleep）：提交线程不阻塞、不死锁，
   `offered == written + dropped`，`dropped > 0` 时账本仍守恒。
6. `Off` 模式 `start()` 返回 `false`/`enabled() == false`，不创建任何文件。

### Track 2 — pipeline 集成

交付：`uwb_uhd_sc16_pipeline.cc` 的
- `initialize()`：按 `cfg_.output` 创建并启动 writer，预计算 `max_cir_taps`
  （`effective_radar_cfg.cir_pre + cir_post`）与 `max_psdu`；
- DSP worker 热路径：`radar_cir_one()` 之后立即把 `radar_scratch.cir.raw_taps`
  前 `tap_count` 个 + `demod_res.payload.bytes` + 元信息拷进槽位；
- `commit_result()`：emit 段内 O(1) 入队；
- `stop()` / `wait()`：`stop_and_drain()`；
- `output_writer_stats()`。

验收：
1. **关闭零回归**：`output` 全默认时，`qa_uwb_uhd_sc16_pipeline.cc` 既有用例
   逐字段不变（`WindowResult` 全字段与基线一致）。
2. **多 worker 顺序性**：`dsp_workers ∈ {1,4,8}`，同一 fixture，
   `cir.jsonl` / `cir.ucr4` / `demod_results.jsonl` 的 `pulse_id` / `packet_id`
   严格升序、无缺号、无重复。**该用例在 Track 2 自己的 worktree 里因 writer 仍是
   Phase 0 桩而必须 skip 并打印原因**，由协调者在合并后重跑。
3. **drain 完整**：`stop()` 返回后 `stats().offered == stats().written + stats().dropped`
   且 `write_queue_` 为空。
4. 槽位池耗尽（`writer_queue_capacity = 1`、`dsp_workers = 8`）时不阻塞、不死锁。

### Track 3 — runner CLI + 端到端格式兼容

交付：runner 新增
`--output-dir` / `--cir-base` / `--no-cir-write` / `--no-comm-write` /
`--comm-write-only-ok` / `--writer-queue` / `--writer-agg-bytes` /
`--writer-agg-ms` / `--live-log <off|events|all|summary>` /
`--live-log-interval-ms` / `--live-log-psdu`，并在 Run Summary 与 `--json-out`
meta 里汇报 `output_writer_stats`。

验收：
1. `--output-dir` 缺省时行为与基线完全一致（不产生任何文件、退出码不变）。
2. `tools/performance/verify_ucr4_output.py` 独立解析 `cir.ucr4` + `cir.jsonl` +
   `run.json`：头 52 B、`magic == "UCR4"`、`tap_count` 与文件剩余长度一致、
   `file_offset_taps` 单调递增且等于累计 tap 数、`fc32 = sc16 * cir_scale` 可还原。
3. `demod_results.jsonl` 的 `psdu_hex` 与 runner 自己报的 `comm_psdu_fnv1a` 一致。
4. `qa_uwb_pipeline_output_runner.sh` 在 `testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin`
   上跑 `--consumer sensing` 与 `--consumer comm`，断言文件存在、可解析、
   `--live-log summary` 有输出且不污染 `--json-out`。
5. 全量 CTest 无回归。

## 5. 合并顺序与冲突预案

1. **T1 → 主干**：`git cherry-pick -x`。T1 只覆盖 Phase 0 桩文件 + 新增格式头，
   无冲突。
2. **T2 → 主干**：只改 `uwb_uhd_sc16_pipeline.cc` 与 `qa_uwb_uhd_sc16_pipeline.cc`，
   与 T1 无交集。
3. **T3 → 主干**：只改 runner + 新脚本/新工具，与前两者无交集。
4. 合并后**重跑 Track 2 的 skip 用例**与 Track 3 的 E2E 脚本，再跑全量 CTest。

若任何 Track 需要改冻结头文件，**必须停下来报告**，不得自行修改（否则三个
worktree 会各自演化出不同的接口）。

## 6. 本轮明确不做

- 不改 `UwbCirWriter`（GNU Radio block）本身；新组件与它**格式兼容但代码独立**。
- 不做 paced 到达、不上板、不做 UHD `recv` 溢出交互验证。
- 不做 MATLAB `read_uwb_cir.m` 实跑交叉验证（只保证格式契约与独立 Python 解析器一致）。
- 不把 `--live-log` 做成 TUI/仪表盘；只做单行/周期行文本。
- 不改变任何 DSP 数值路径；本轮所有改动都在"结果已经算完之后"。
