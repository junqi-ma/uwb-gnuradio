# Grok 执行方案：能量门与 Detector 合路性能分析、测试和改进

更新时间：2026-08-08

## 1. 目标

当前现象：

- 裸 `UwbDetectorStateMachine` 能量门约 **1.7～2.2 GS/s**。
- 独立 Region worker 约 **200.8 MS/s Region IQ**，约 760 region/s。
- GNU Radio 完整稀包流水线只有 **204～223 MS/s 原始输入**。
- 当前 `PatternSource → null_sink` 基线约 **264 MS/s**。

本方案的目标不是立即重写架构，而是先用同口径实验回答以下问题：

1. 264 MS/s 上限中，测试源生成、GNU Radio scheduler/buffer 和 stream copy 分别占多少？
2. 完整 Detector 相对 source-only 多出的约 15%～23% 成本来自 `pre_ring`、状态机还是消息/worker？
3. 正确扩大 buffer/work chunk 后能否达到 400、600 或 1000 MS/s？
4. 若 GNU Radio stream 架构达不到 1 GS/s，应在哪一层融合或旁路？

最终验收目标：在不削弱现有检测正确性的前提下，给出可复现的瓶颈归因、优化前后数据，以及 1 GS/s 的可行/不可行边界。

---

## 2. 当前判断

### 2.1 三个“快”不是同一种吞吐

| 测试 | 输入单位 | 是否经过 GR | 省略的工作 |
|---|---|---|---|
| `detector-gate` | 连续原始 IQ | 否 | source、GR buffer、scheduler、worker、PDU |
| `detector-region` | 已截好的 Region IQ | 否 | 静默输入、source、scheduler、message port |
| `detector-sparse` | 连续原始 IQ | 是 | 无，属于端到端测试 |

因此不能把 2 GS/s gate 和 200 MS/s Region worker 直接串联为 1 GS/s 端到端结论。

### 2.2 Worker 在稀包场景不是主要瓶颈

200 包/s 时 Region 占原始输入约 5.29%，所以 200.8 MS/s Region worker 的等效原始输入能力约为：

```text
200.8 / 0.0529 ≈ 3.8 GS/s
```

1 包/s 时 worker 负载更低。当前稀包端到端时间主要消耗在每个原始样本都必须经过的 source、GR buffer、scheduler 和 `pre_ring`，而不是粗/精相关。

### 2.3 当前 buffer 测试使用了错误 API

现有 benchmark 使用：

```cpp
src->set_max_output_buffer(0, 1 << 22);
```

`set_max_output_buffer()` 只设置最大允许值，不要求扩大 buffer。本地 GNU Radio `block::allocate_buffer()` 从默认值开始，在 max 为正时执行 `min(default, max)`；所以默认 8192 不会被 4M 放大。

正确测试方法：

```cpp
src->set_max_output_buffer(0, -1);
src->set_min_output_buffer(0, buffer_items);
tb->set_max_noutput_items(max_noutput_items);
```

注意：只相信实际 `work(noutput_items)` 统计，不要仅凭配置值判断 chunk 已经变大。

### 2.4 当前证据尚不能区分的部分

`PatternSource → null_sink≈264 MS/s` 只能证明当前 harness 上限，不能单独证明：

- GNU Radio 3.10 本身只能到 264 MS/s；
- DRAM 带宽已经饱和；
- `pre_ring` 是主要瓶颈。

必须使用同一 source、同一 target、同一 buffer 配置做逐层 A/B。

---

## 3. 执行前保护措施

1. 阅读 `../../AGENTS.md`、`GROK_接手说明.md`、`性能瓶颈分析_门限与Detector合路.md`。
2. 记录 `git status --short`、`git log -3 --oneline` 和 `git diff --stat`。
3. 当前工作区可能已有未提交优化，不得使用 `git reset --hard` 或覆盖整个文件。
4. 清理 `.orig/.rej` 前先确认其中没有遗漏内容。
5. benchmark、生产代码和算法改进分 commit，避免无法性能二分。
6. 每次 meaningful modification 后执行对应 QA；阶段结束执行全部 CTest。

---

## 4. Phase A：建立同口径 benchmark

### A1. 给 `benchmark_detector` 增加统一模式

所有 GR 模式必须使用相同的 `PatternSource`、target、gap、buffer 参数和计时范围。

新增或确认以下模式：

| 模式 | 拓扑 | 用途 |
|---|---|---|
| `source-null` | `PatternSource → null_sink` | source + GR plumbing 基线 |
| `source-search` | `PatternSource → UwbDetector`，阈值设为永不触发 | SEARCH + pre_ring 增量 |
| `detector-sparse` | `PatternSource → 正常 UwbDetector → PduCounter` | 完整稀包增量 |
| `detector-gate` | 裸 `sm.process()` | 算法上限，仅作参考 |
| `detector-region` | 裸 Region worker | worker 上限，仅作参考 |

不要在部分测试中使用 `vector_source`、另一部分使用 `PatternSource`。300M/500M/1G 的 source 类型变化会破坏可比性。

### A2. 增加 CLI 参数

建议参数：

```text
--buffer-items N
--max-noutput-items N
--source pattern|vector
--threshold VALUE
--repeat N
```

默认 benchmark 使用 `pattern`。`vector` 只用于确认 PatternSource 生成成本，且必须在相同 target 下比较。

### A3. 增加低扰动统计

在 `PatternSource` 和 `UwbDetector` 各记录：

- `work_calls`；
- `items_total`；
- `min_noutput_items`；
- `max_noutput_items`；
- `mean_noutput_items`；
- 建议直方图区间：`≤8k`、`8k～32k`、`32k～128k`、`128k～512k`、`>512k`。

计数器由各自 work 线程写，在 `top_block::run()` 返回后读取，不需要每次使用 atomic。先测量启用统计前后的差异，确认 instrumentation 本身影响小于 2%。

### A4. 修复 PatternSource 精确计数

当前 `work()` 循环中 `d_total` 在循环结束后才更新，最后一次调用可能输出超过 target、但 benchmark 分母仍使用 target。修改每次产生数量的上限：

```cpp
remaining_target = d_target - d_total - produced;
n = min(n, remaining_target);
```

这是正确性修正，虽然对 1G 测试的误差通常只有一个 chunk。

### A5. Phase A 验收

- 同一 target 下三个 GR 模式使用完全相同 source。
- 实际处理样本严格等于 target。
- 输出实际 work chunk 统计。
- `git diff --check` 无输出。
- CTest 4/4 通过。

---

## 5. Phase B：buffer/chunk 扫描

### B1. 正确设置 buffer

在连接 flowgraph 前：

```cpp
if (buffer_items > 0) {
    src->set_max_output_buffer(0, -1);
    src->set_min_output_buffer(0, buffer_items);
}
tb->set_max_noutput_items(max_noutput_items);
```

不要同时保留正数 `max_output_buffer`，因为 GNU Radio 3.10 的 allocator 对 max/min 使用 `if ... else if`，正数 max 会使 min 分支不执行。

### B2. 扫描矩阵

固定：

- target：先 500M，候选配置再用 1G 复测；
- gap：`998143552`（1 包/s）作为 plumbing 主测试；
- repeat：至少 5 次，报告中位数、最小值、最大值；
- 每轮前确认无其他高负载任务。

扫描：

| buffer items | 约 CF32 容量 | max noutput items |
|---:|---:|---:|
| 默认 | 默认 | 默认 |
| 32,768 | 256 KiB | 32,768 |
| 131,072 | 1 MiB | 131,072 |
| 524,288 | 4 MiB | 524,288 |
| 1,048,576 | 8 MiB | 1,048,576 |
| 4,194,304 | 32 MiB | 1,048,576 和 4,194,304 各测一次 |

每个配置运行 `source-null`、`source-search`、`detector-sparse`。

### B3. 必须记录的结果

```text
mode / target / gap
configured buffer / actual buffer
work calls / min-mean-max noutput_items
wall time / CPU time / CPU utilization
input MS/s / GB/s
detections / dropped regions
captured IQ / output IQ rate
```

### B4. 结果判定

| 结果 | 判断 | 下一步 |
|---|---|---|
| 三个 GR 模式同步大幅提升 | 主要是 buffer/scheduler 粒度 | 选拐点配置，进入 Phase C |
| `source-null` 提升，`source-search` 不提升 | SEARCH/pre_ring 成为瓶颈 | 进入 Phase D |
| `source-null` 始终约 264 MS/s | PatternSource/TPB/WSL plumbing 受限 | 先做 Phase C，再考虑架构融合 |
| `source-search≈source-null`，normal 明显更低 | region/worker/message 成本 | 做包率扫描与 worker profile |

成功标准不是必须选最大 buffer，而是找到吞吐收益开始小于 5%的拐点，避免用几十 MB buffer 换取极小收益和高延迟。

---

## 6. Phase C：拆分 source 与 scheduler 成本

### C1. PatternSource 对比 vector_source

在 target 可放入内存的情况下，例如 100M 或 200M：

```text
PatternSource → null_sink
vector_source → null_sink
PatternSource → source-search
vector_source → source-search
```

两类 source 使用相同 buffer/chunk 配置，stream 构造时间不计入 `top_block::run()`，但必须在报告中明确。

解释：

- vector 明显更快：`PatternSource::work` 的 `memset/memcpy`/分支是主要成本。
- 两者相近：GR buffer、scheduler 和线程交接更主要。

### C2. perf 采样

对 1 包/s、1G 的 `source-null` 和 `source-search` 分别采样：

```bash
perf stat -e cycles,instructions,cache-references,cache-misses,context-switches,cpu-migrations,page-faults <command>
perf record -g -- <command>
perf report
```

重点函数：

- `PatternSource::work`；
- `memset` / `memcpy`；
- GNU Radio `block_executor` / buffer reader/writer；
- `UwbDetector::work`；
- `RingBuffer::push`；
- 调度等待和 futex。

若 WSL 禁止 perf 事件，至少记录 wall/CPU%、work calls 和 chunk 分布，不要用猜测替代测量。

### C3. CPU affinity 实验

只作为诊断，不作为默认生产方案：

- 默认调度；
- 将 benchmark 固定在一个 P-core；
- 将 source 和 detector 固定在不同 P-core（若可控）。

如果跨核性能明显下降，说明 buffer cache-line 迁移和线程唤醒是重要成本。

---

## 7. Phase D：优化 SEARCH 全速率路径

仅在 Phase B/C 证明 `source-search` 明显慢于 `source-null` 后执行。

### D1. 先测成本，不改变算法

增加 benchmark-only 变体：

1. 只读输入，不写 `pre_ring`；
2. 保留 D=100 能量计算，不写 `pre_ring`；
3. 保留 `pre_ring`，跳过能量计算；
4. 完整 SEARCH。

这些变体只用于归因，不进入生产路径，也不能用来声明正确性通过。

### D2. 低风险改进

按优先级测试：

1. 让 `process_search()` 每次处理更大的连续区间，减少 100-sample 内层控制开销。
2. 将能量计算与 ring copy 分离成较大的批次，验证编译器向量化和 `memcpy` 合并。
3. 检查 `RingBuffer::push()` 对小于 ring 容量和远大于容量的输入是否走最少分支。
4. 固定 `pre_trigger` 时避免运行期重建；setter 只允许停流状态调用。
5. 使用 `-march=native` 的 release build，并确认 benchmark 未误用 debug/未优化库。

每项必须保留现有 ring 跨界、双包、PDU IQ 逐样本 QA。

### D3. 中风险改进：SC16 早期格式

建立单独原型，不直接替换 CF32 生产路径：

- source/ingest 保持 SC16，每样本由 8 B 降到 4 B；
- 能量计算使用整数或转换小批量 float；
- 仅候选 Region 转 CF32，或直接以 SC16 PDU 输出给 MATLAB；
- 对比 packet start、门触发点、PDU IQ 和 MATLAB 检测结果。

验收：性能至少提升 30%，且检测结果无回退，才考虑合入。

---

## 8. Phase E：worker 与高包率复测

虽然 worker 不是稀包瓶颈，达到 1 GS/s 后 200 包/s 会产生约：

```text
200 × 264016 ≈ 52.8 MS/s Region IQ
200 × 202032 × 8 ≈ 323 MB/s PDU payload copy
```

因此仍需执行：

- 1、10、100、200 包/s；
- dense/小 gap 压力；
- region 池耗尽；
- job 队列耗尽；
- worker 异常释放；
- stop/EOS/尾部无静默；
- message sink 慢消费。

记录 worker：regions/s、ms/region、队列最高水位、in-flight、drop。不要只记录输入 MS/s。

验收：在等效 1 GS/s、200 包/s 的 region 负载下，长期 0 drop，并保留至少 2× worker 容量余量。

---

## 9. Phase F：何时改变架构

若正确 buffer/chunk 配置、PatternSource 优化和 SEARCH 优化后，`source-search` 仍低于 **600 MS/s**，不建议继续在小函数上微调，应做架构原型。

### F1. 推荐架构

```text
UHD/native receive buffer（优先 SC16）
        │
        ├── 同一 ingest 线程做 D=100 能量门
        ├── source-owned circular history 保存 pre-trigger
        └── 触发后复制候选 Region 到固定池
                         │
                         ▼
                 worker 粗/精相关
                         │
                         ▼
                    PDU / Writer
```

关键点：能量扫描尽量直接发生在接收 buffer 上，避免先写 GR stream buffer、再由 Detector 读回。GNU Radio 只接收事件/PDU，或由融合的 UHD source+detector block 直接发布 PDU。

### F2. 不推荐的伪优化

- 简单拆成 `energy_gate → detector` 两个普通 stream block：如果仍传递全速率 IQ，只会增加一次 buffer 和调度。
- 只降低包事件率：不能消除静默 IQ 的 ingest 成本。
- 删除 pre-trigger ring：会破坏触发前 IQ 和 packet start 正确性。
- 粗相关 stride>1：已有窄峰漏检证据。
- 继续优化 worker 来解决 1 包/s 吞吐：方向错误。

### F3. 架构原型验收

- source/SEARCH-only ≥1 GS/s 或明确被硬件输入上限限制；
- 与 MATLAB packet start 逐样本一致；
- PDU 捕获区逐样本一致或 SC16 误差小于 1 LSB；
- 200 包/s 长时间 0 drop；
- 有明确 backpressure/drop 策略和统计。

---

## 10. 推荐执行顺序与 commit

| 顺序 | 工作 | 建议 commit |
|---:|---|---|
| 1 | 统一 benchmark 模式、计数和精确 target | `Add layered detector performance benchmarks` |
| 2 | 改用正确 min buffer API，完成扫描 | `Benchmark detector with controlled GNU Radio buffers` |
| 3 | perf/source/scheduler 归因 | 数据和文档 commit，不混生产优化 |
| 4 | SEARCH 低风险优化及 QA | 每项独立 commit，方便性能二分 |
| 5 | worker 高包率压力测试 | 独立测试 commit |
| 6 | SC16 或 fused ingest 原型 | 独立分支/commit，不直接替换稳定路径 |

每个性能 commit 必须同时包含：

- 修改前后命令；
- 至少 5 次结果的中位数；
- CPU/工作块统计；
- CTest 结果；
- 检测数和 drop；
- 正确性是否变化。

---

## 11. 最终报告模板

Grok 完成后应在报告中回答：

1. 默认 8192 实际 work chunk 是多少？正确设置 min buffer 后是多少？
2. `source-null`、`source-search`、正常 detector 各自吞吐和增量成本是多少？
3. PatternSource 与 vector_source 差异是多少？
4. 最佳 buffer 拐点是多少，为什么？
5. perf 的前三个热点是什么？
6. 稀包和 200 包/s 的瓶颈是否相同？
7. 当前代码在 i7-12700/WSL2/CF32 下可稳定达到多少？
8. 达到 1 GS/s 还差多少，下一步应继续优化 GR 还是转向 fused ingest/SC16？

最终表格至少包含：

| 版本 | source-null | search-only | 1 pkt/s | 200 pkt/s | work mean | CPU% | drop |
|---|---:|---:|---:|---:|---:|---:|---:|
| 原始基线 | | | | | | | |
| 正确 min buffer | | | | | | | |
| 最佳 chunk | | | | | | | |
| SEARCH 优化 | | | | | | | |
| 架构原型（如执行） | | | | | | | |

---

## 12. 最低完成标准

- 不再使用 `set_max_output_buffer(4M)` 表示“请求大 buffer”。
- 三个 GR 层级基准使用同一 source 和 target。
- 实际 work chunk 有统计证据。
- `git diff --check` 无输出，不提交 `.orig/.rej`。
- 全部 CTest 通过。
- packet start、PDU IQ、双包顺序无回退。
- 1/10/100/200 包/s 均报告 detections 和 dropped regions。
- 瓶颈结论由 A/B 和 profile 支持，而不是由不同拓扑单测相减得到。
