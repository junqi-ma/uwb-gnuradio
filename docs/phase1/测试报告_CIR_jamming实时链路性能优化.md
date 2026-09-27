# CIR jamming 实时链路性能优化 — 验收报告

- 日期：2026-09-27
- 分支：`perf/cir-jamming-opt`（基点 `b897677` = `feature/uwb-monostatic-radar` 同点）
- 最终合并点：`31e7f11`
- 主机：i7-12700（isolcpus=2,3,4，用户态 17 核），governor=performance，GR 3.10.12.0，Release + LTO，无 `-march=native`
- 基线与本报告所有对比口径：`benchmark_radar_pipeline 16 64` p50，安静窗口多轮取中位；原始输出 `/tmp/opencode/baseline/`（Wave 0 存档）

## 1. 各阶段交付清单（9 个功能 commit + 4 个 merge）

| 阶段 | Commit | 内容 |
|---|---|---|
| 一_1 | `0d4e771` | 从 `radar_cir_one()` 抽出 `radar_cir_locate` 公共定位（front-gate + SFD/SYNC-refine/predicted-timing + origin 预测 + skip/count 边界），公开结果逐字段不变 |
| 一_2 | `a68bc2f` | `both`/`repetitions` 定位一次、逐 rep 一次；新增 `estimate_radar_cir_from_repetitions`（double 累加、active-code 稀疏）替代第二次全窗重建；cir/cir_avg 消息、计数、发布顺序不变 |
| 一_3 | `9a04b4a` | `estimate_radar_cir` 内层 1016 dense 扫描 → 64 非零码片稀疏相关（**逐位一致**，max_abs=0） |
| 二_1 | `8e9eda8` | writer 队列按 pulse-PDU 计量：`--cir-writer-queue-pdus`/`--cir-avg-writer-queue-pdus`（默认 64）、最坏 RAM 估算日志、summary 新键 |
| 二_2 | `c7a0ce8` | 慢盘可控 QA：满队整 pulse 丢弃对账、FIFO 顺序、stop drain、RSS 趋势（256 容量 ≈39.6 MiB vs 64 ≈11.1 MiB） |
| 三_1 | `e1951e6` | 消费者清单确认 UCR4 writer/UDP 均不消费 normalized → jam app `--cir-emit-normalized` 默认 off，不再创建批量 normalized PMT；策略日志 + summary 字段 |
| 三_2 | `65b60ed` | 块级 QA：off 时无 `normalized_taps` 键、raw 逐 tap bit-exact、UCR4 字节级/JSONL 逐行一致；on 时与原实现逐 tap 相等 |
| 三_3 | `c6f357f` | 删除阶段一遗留死声明 `publish_alongside_average` |
| 四a_1/2/3 | `58bd554`/`1893956`/`1410bd0` | CIR 内核 AVX2/FMA（corr4 + 平均窗累加 + rep-sum + per-rep/from_repetitions），`__attribute__((target("avx2,fma")))` 无条件发射 + `__builtin_cpu_supports` 运行时门控，标量路径逐字节保留；新增 `test_radar_cir_avx2_matches_scalar` + 冻结标量参考 `qa_uwb_radar_cir_scalar_ref.h`。**注意**：a_1/a_2 提交消息中的精度/计时是修正前内核所测，一律以 `1410bd0` 后数字为准 |
| res-workers | `1cef27c` | `--res-workers` 对 PDU 65/48 静默无效 → 显式 WARN + status/summary 上报真实生效值（effective=1）+ `res_workers_requested` 新键 + 8 例 python QA |

## 2. 性能差异表（合并后安静窗口 vs 基线 b897677）

| 指标 | 基线 | 最终 | 变化 |
|---|---|---|---|
| estimateCir 54-rep p50 | 406.6 µs | **12.4 µs** | **32.8×**（标量稀疏 10.6× → AVX2 32.8×） |
| estimateCir per-rep | （阶段一才有的口径）14 µs | ~2.0 µs | ~7× |
| from_repetitions（avg 版） | — | ~21–23 µs | — |
| radar_cir_one p50（含通信式 SFD 搜索口径） | 1674 µs | 1333 µs | −20.4% |
| SFD search ±64（通信式口径） | 1448 µs | 1296 µs | −10%（波动带边缘，无单独优化项） |
| CirEstimator serial（block，normalized / no-normalized） | 2000 µs | 2000 / 2000 µs | 持平（阶段三 on/off 无差异，符合预期：116-tap PMT 占比 ~0.1%） |
| CirWriter serial | 252 µs | 250–254 µs | 持平 |
| 饱和 e2e UC200 65/48 SC16 | 497–508 p/s | 498.1 p/s | 持平（resampler handler ~1950 µs 为上限，未动） |
| 饱和 e2e CG400 65/32 SC16 | 497–508 p/s | 499.8 p/s | 持平 |
| writer 队列 RSS（满队） | 混用口径 ~39.6 MiB | 64 PDU 容量 11.1 MiB | **−72%** |
| 生产配置 estimator service（predicted timing + batch + 128 reps） | 3.45 ms（硬件记录）/ 3.54 ms（QA 实测） | AVX2 后未单独复测，预期 ~1.7–1.8 ms（per-rep 128×2 µs ≈ 0.26 ms） | 待硬件复测确认 |

精度：AVX2 vs 冻结标量 raw max_abs=2.98e-8、norm max_abs=5.96e-8（阈值 1e-6 余量 ~30×）；peak_tap 与 raw_l2_norm 完全一致；MATLAB golden 逐 tap（rel L2<1e-5）全过。`both` 模式 cir_avg 与 average 模式不再位等（rel L2 ≤2e-6，阶段一顺序变化的显式化，容差内）。

## 3. QA 汇总

- 全量 `ctest -j17`：43 例中 40 例通过；3 例失败均为环境/负载类：
  1. `uwb_qa_uwb_pdu_rational_resampler` 吞吐阈值（**改动前即失败**，本机 216 vs 阈值 400，历史 575 来自另一宿主，见 `docs/phase1/测试报告_UWB_Radar_PDU速率优化.md` §8）；
  2. `uwb_benchmark_hrp_mod` 吞吐门槛（>1000 µs），串行复跑通过；
  3. `uwb_qa_uwb_pdu_roi_cir_65_32`（j17 并发偶发，串行 3/3 通过）。
- 新增 QA：阶段一 4 例（locate×3 + from_repetitions）、阶段二 4 例（慢盘）、阶段三 2 段（块级 on/off + writer 字节一致）、四a 1 例（avx2_matches_scalar，默认与 AVX2 双构建通过）、res-workers 8 例（python）。
- Python QA：`test_jam_app_args.py` 25 例、`test_echo_cir_jam_plan.py` 103 例、`test_res_workers_hint.py` 8 例、cir_udp_format/echo_stream_buffers 等全过。

## 4. 剩余瓶颈清单（按优先级）

1. **PDU 65/48 resampler handler ≈1950 µs**：软件吞吐硬上限 ≈500 p/s。200 p/s 目标下裕量 2.3–2.5×，暂不动；若推 500+，顺序 = publish/串行段解耦 → AVX2 FIR → worker 池。
2. **SFD 搜索 ≈1300 µs**：仅通信式口径（`--require-sfd` 或 benchmark 默认配置）支付；生产 jam 走 predicted timing 不付。若未来盲捕获链路需要提速，可做窗口收窄/粗到细分层。
3. **X410 侧 `echo_late`**（历史 300 次/60 s @200 Hz）：radio worker 时序/overflow 问题，**任何主机优化都不可达**，需硬件 soak 定位。
4. CirWriter/publish ≈250–290 µs：次要，阶段五可选项。

## 5. 未验证项（诚实清单）

- **硬件验证未做**：X410 128/256-rep 生产配置（50 Hz/200 Hz）、`echo_late`、soak/overflow 重捕获、UHD underflow——按方案 §7 属三层验收的第三层，本报告只覆盖前两层（内核微基准 + 无硬件完整 PDU 链）。不得据此宣称硬件验收通过。
- 非 AVX2 CPU 实机回退：运行时检测代码在位、非 x86 宏保护在位，但未在真非-AVX2/ARM 机器实测。
- MATLAB `verify_read_uwb_cir` 对照：阶段二/三 writer 输出已做字节级/逐行 QA，MATLAB 脚本对照留待硬件数据到位后执行。
- 256-rep 生产配置未实测（QA 以 64/128 reps 覆盖）。
- `/usr/local/lib` 陈旧安装库（8 月 b897677 版）仍存在；QA 均经 build/lib RUNPATH 正确解析，但建议重新 install 以绝后患。

---

# 附录：X410 200 Hz 实机加测（2026-09-27 硬件累积验证）

## 方法
同参数 60 s × 200 Hz × 128 rep × 64 taps × `--cir-output both` × jam+UDP，仅逐步调整 writer 队列/聚合参数共 5 组。软件版本：分支 `perf/writer-aggregate`（合并上述主线 + 3 commit 聚合写）。

## 结果表

| run | 队列 | 聚合 | wr_drop(pulse批) | wr_avg_drop | drain_complete | echo_late | 说明 |
|---|---|---|---|---|---|---|---|
| q64 | 64 | — | 341 | 81 | false | 180 | 首次暴露长 gap |
| q128 | 128 | — | 135 | 0 | false | 60 | 未根治 |
| agg 1M | 128 | 1 MiB | 109 | 0 | false | 300 | 机制生效，稳态 flushes 1085 |
| agg 8M | 128 | 8 MiB | 53 | 0 | false | 260 | 单调收敛 |
| **agg 8M + q256** | 256 | 8 MiB | **0** | **0** | **true** | 180 | **全硬门槛通过** |

- 最优 run：`wr_ok=1536000=12000×128` 精确满额、writer hwm **151/256**（60% 余量，非触顶）、estimator 1707/2438 µs（PRI 的 34%）、UDP 1.536M/1.536M ok、对账 `12000+180=12180=id_max+1` 完全闭合、late 全部为 9 段恰好 20 slot 的 radio 跳发。
- **每小时损失对账**：本 run 每间隔 5-10 s 出现 1 次 20-slot（100 ms）radio 跳发，共 180/12000 = 1.5% ≤ 5% 的 app 宽容判据，交付记录 gap-free。
- 定稿生产参数：`--cir-writer-queue-pdus 256 --cir-avg-writer-queue-pdus 128 --cir-writer-aggregate-bytes 8388608`（RAM 代价 ≈31.75 MB 最坏 + 聚合 buffer 实测 ≤3.2 MB）。

## echo_late（未解决，独立线）
- 三组不同 run：180/60/300/260/180——非启动瞬态、非周期常数，5-10 s 一跳、每段恰好 20 slot（100 ms 收发窗）。
- 最优 run `cpp_tx_async` 出现 **`time_error=2385`**（此前各组为 0）：TX 时基偏移计数首次与 late 段时间相关，方向指向 radio worker 内部时隙/时基分配，**主机代码不可修**；待 X410 侧独立实验（关 jam 对照、不同 PRI、PPS 校核对时）定位。
- 注意按方案 §7 之要求：硬件 late 与 CPU 阶段耗时分别报告；CPU/链路层 wr_drop 已清零≠late 已解决。

## 聚合写机制验证（硬件侧）
- `cir_writer_aggregate_flushes ≈ 1080–1092`（60 s ÷ 55 ms，与 50 ms 窗口一致）；
- `cir_writer_aggregate_max_bytes：1.0 MB（1M 阈值）/ 3.1–3.2 MB（8M 阈值下）`——阈值不是瓶颈，窗口（50 ms）主导稳态输出；
- 8 MiB 阈值把 writer 对盘停容忍度从 640 ms（q128）推到 ~2.1 s（q256+8M），覆盖全部实测 1-2 s 阵发 stall；
- CirWriter 主路径性能不变（QA 微基准 254.0 µs 与基线持平，字节级对账 bit-identical）。

## 后续（超出本轮范围）
- `echo_late` 的 X410 侧独立定位（本轮新线索：time_error=2385 与 late 段时间相关）；
- writeback attribution（如有必要再深挖 long-stall 本体，`/proc/meminfo Dirty/Writeback` 关联）；
- 300 Hz 与 10 min soak 仍是后续目标。
