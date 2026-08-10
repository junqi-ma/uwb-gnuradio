# 性能瓶颈分析：能量门与 Detector 合路后为何只有 ~200 MS/s

> 日期：2026-08-08  
> 目的：把当前实现、测量口径、关键数字与「合路 gap」分析整理成**独立、可移交**文档，供外部模型（GPT 等）做进一步架构/优化分析。  
> 相关文档：`../../开发状态.md` §5.3、`测试报告_发包率扫描.md`（§4 / §8）、`../../AGENTS.md`、`../../开发需求参考.md`  
> 相关代码：`gr-uwb/lib/uwb_detector.cc`、`gr-uwb/include/gnuradio/uwb/uwb_detector_core.h`、`gr-uwb/apps/benchmark_detector.cc`  
> 关键提交（截至本文）：`583a965`（发包率 e2e 文档）、`c39a9f4`（P0/P1 worker 优化）

---

## 0. 请外部模型重点回答的问题

1. 在**不削弱检测正确性**的前提下，如何把稀包端到端输入吞吐从 ~200–260 MS/s 推到 **1 GS/s**？
2. 当前「门限压缩包事件率、但静默仍全速率过 GR」的架构是否合理？有无更优的流图拓扑（零拷贝、旁路、双速率、SC16 早期量化等）？
3. `pre_ring` 全速率维护是否可在大 gap 下稀疏化？正确性边界是什么？
4. GNU Radio 3.10 下 buffer 被钳到 8192 时，有哪些可靠手段扩大有效 chunk / 减少调度次数？
5. 算法侧（worker）与数据平面（source/plumbing）应如何分期优化？优先级排序。

---

## 1. 项目目标与约束

### 1.1 产品目标

- 从连续高速率 IQ（目标 **~1 GS/s**，CF32 或后续 SC16）中检测 UWB 包并截取 packet IQ。
- **不是**完整解调；截取结果交给离线 MATLAB（`UWB_demodulation/`）。
- 算法正确性以 MATLAB 与 `testdata/` 对照为准。

### 1.2 性能目标

| 指标 | 目标 |
|---|---|
| 输入处理速率 | 逼近 **1 GS/s real-time** |
| 发包率场景 | 至少覆盖 **1 / 10 / 100 / 200 包/s**（稀包为主） |
| 检测 | 低漏检、可与 MATLAB 对照 |
| Region 丢弃 | 正常负载下 **0 drop** |

### 1.3 测试环境（近期测量）

| 项目 | 值 |
|---|---|
| 平台 | Windows 11 + WSL2 Ubuntu 22.04 |
| CPU | 12th Gen Intel Core i7-12700（10 核 20 线程） |
| 内存 | 32 GB |
| GNU Radio | 3.10.1.1 |
| 编译 | g++ 11.4，`-march=native` |
| 测试信号 | `testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile` |
| 名义采样率 | **998.4 MHz** |

---

## 2. 当前架构

### 2.1 生产流水线（`UwbDetector`）

```
连续 IQ 输入 (gr_complex / CF32)
        │
        ▼
┌───────────────────────────────────────┐
│  UwbDetectorStateMachine（GR work 线程）│
│  SEARCH:  D=100 能量门 + pre_ring 全速率 │
│  IN_REGION: 全速率 IQ 追加到 region 池  │
│  区域结束 → 有界 job 队列入队           │
└───────────────────────────────────────┘
        │  RegionHandle（~264k samples/区）
        ▼
┌───────────────────────────────────────┐
│  后台 worker 线程（异步）               │
│  1) 粗相关 CD=4，仅扫 preamble-horizon │
│  2) first-peak 精相关（metric≥0.5 即停）│
│  3) 截包 + PMT PDU → message_port_pub  │
└───────────────────────────────────────┘
        │
        ▼
  packet 消息口 → 下游（writer / 计数器）
```

要点：

- `UwbDetector` 是 **0 流输出** 的 `sync_block`（只消费输入，出 PDU）。
- 粗/精/PDU **不在** `work()` 内同步完成，避免阻塞 scheduler（异步 worker + 8 槽 region 池）。
- `work()` 末尾对最后一个样本有 sentinel 语义（`noutput_items-1` 消费），流结束前 `wait_for_worker_idle()` 排空。

### 2.2 关键几何尺寸（参考包）

| 量 | 典型值 | 说明 |
|---|---:|---|
| 包有效长度 | 256,448 | 从 cfile 截取的 packet body |
| Region 长度 | **264,016** | pre + packet + 门保持尾（四档发包率一致） |
| PDU 截包长度（配置） | 202,032 | pre_trigger 2032 + capture 200000（短 region 会截断） |
| 能量门 D | 100 | 每 100 样本一块；块内前 16 样本参与 \|x\|² 累加 |
| 粗相关 D | 4（默认） | D=8 冒烟通过，**默认未改** |
| Preamble template L | 1016 | 1 个 SYNC 符号 |
| Coarse horizon | ~77,216 | `L*(64+8+4)` 与 candidate_offset 取 max 后 clamp |
| Region 池 | 8 | 满则 drop |

发包率与 gap（fs = 998.4e6）：

| 发包率 R | 周期 samples | gap samples | 占空比 |
|---:|---:|---:|---:|
| 200/s | 4,992,000 | 4,735,552 | 5.14% |
| 100/s | 9,984,000 | 9,727,552 | 2.57% |
| 10/s | 99,840,000 | 99,583,552 | 0.257% |
| 1/s | 998,400,000 | 998,143,552 | 0.0257% |

### 2.3 状态机 SEARCH 热路径（关键实现细节）

文件：`uwb_detector_core.h` → `UwbDetectorStateMachine::process_search`

对 **每一个输入样本**（静默也一样）：

1. 按 D=100 分块；
2. 块内前 `neigh_len_=16` 个样本累加 `std::norm`；
3. **`pre_ring_.push(...)` 全速率 bulk memcpy**（保证能量穿越时 pre-trigger 完整）；
4. 块满时 `update_gate`：滑动能量与阈值比较。

IN_REGION 时额外：`region.samples.insert(...)` 全速率物化候选区。

→ **门限只压缩「是否开启 region / 是否跑相关」；不压缩「输入样本是否被搬运」。**

### 2.4 Worker 优化（P0/P1，已落地）

| 项 | 优化前 | 优化后 |
|---|---|---|
| 粗扫范围 | 整 Region ~264k | preamble-horizon ~77k（约 29%） |
| 精相关 | 所有 coarse peak | 第一个 fine metric ≥ 0.5 即 break |
| ms/region | **5.442** | **1.315**（**4.14×**） |
| Region IQ 吞吐 | ~48.5 MS/s | **~200.8 MS/s** |

Profile（优化后，约）：coarse ~1.20 ms，fine ~0.006 ms，PDU ~0.22 ms。

---

## 3. 测量口径（极易混淆，必须分开）

### 3.1 三种 benchmark 不是同一拓扑

| 模式 | 代码入口 | 是否经 GNU Radio | 测什么 |
|---|---|---|---|
| `detector-gate` | `run_gate_bench` | **否** | 裸循环 `sm.process` + 立即 release region |
| `detector-region` | `run_region_core_bench` | **否** | 预构造 Region 块，循环 `process_region_like_detector` |
| `detector-sparse` | `run_detector_bench` | **是** | `vector_source`/`PatternSource` → `UwbDetector` → `PduCounter` |
| PatternSource→null_sink | 文档基线 | **是** | 源 + GR plumbing 上限 |

### 3.2 速率定义

| 名称 | 定义 | 单位 |
|---|---|---|
| **输入吞吐** | `processed_input_samples / wall_time` | MS/s |
| **Region IQ 速率** | `Σ region.samples.size() / wall_time` | MS/s |
| **输出 PDU IQ 流** | `Σ PDU payload length / wall_time` | MS/s |
| **实时内容 Region 率** | `packet_rate × 264016`（假设 1 GS/s 输入） | MS/s |

### 3.3 `detector-gate` vs `detector-sparse` 实现差异（gap 分析核心）

**gate（无 GR）** 摘要：

- 预拼 `[packet + gap]` 大数组（可到 1e9 samples ≈ 8 GB）；
- `for` 每 8192 样本调用 `sm.process`；
- region ready 后 `take` 计数并 `release`，**不做粗/精/PDU**。

**sparse（全 GR）** 摘要：

- `target ≤ 3e8`：预拼 stream + `vector_source_c`，`set_max_output_buffer(0, 1<<22)`（请求 4M，**实测常被 GR 钳到 8192**）；
- `target > 3e8`：`PatternSource` 在 `work()` 内循环 memcpy 包 / memset gap；
- `top_block::run()` 驱动；message 连接 `packet` → 计数器；
- Detector 内部异步 worker 做相关与 `pmt::init_c32vector` 拷贝。

**结论：用 gate 的 2 GS/s 直接对比 sparse 的 200 MS/s，会夸大「合路变慢」，二者不可直接归因于「算法合在一起就慢」。**

---

## 4. 关键关键测量数据

### 4.1 仅能量门（`detector-gate`，P0/P1 后，§8.5）

| 发包率 | 样本 | 输入吞吐 | Region 数 | Region IQ 壁钟速率 | 占输入 |
|---:|---:|---:|---:|---:|---:|
| 200/s | 200 M | **2095 MS/s** | 41 | 113.4 MS/s | 5.41% |
| 100/s | 200 M | **2137 MS/s** | 21 | 59.2 MS/s | 2.77% |
| 10/s | 200 M | **2247 MS/s** | 3 | 8.90 MS/s | 0.40% |
| 1/s | 1 G | **1749 MS/s** | 2 | 0.923 MS/s | 0.05% |

→ 裸状态机 **multi-GS/s**；与包率弱相关。

### 4.2 全流水线（`detector-sparse`，P0/P1 后，§8.5）

| 发包率 | 样本 | 输入吞吐 | 输出 PDU IQ | 检出 / drop | 相对 1 GS/s |
|---:|---:|---:|---:|---|---:|
| 200/s | 500 M | **204.4 MS/s** | **8.34 MS/s** | 101 / 0 | ~20% |
| 100/s | 500 M | **212.7 MS/s** | **4.38 MS/s** | 51 / 0 | ~21% |
| 10/s | 500 M | **203.7 MS/s** | **0.49 MS/s** | 6 / 0 | ~20% |
| 1/s | 1 G | **222.6 MS/s** | **0.090 MS/s** | 2 / 0 | ~22% |

历史对比（输入吞吐）：

| 发包率 | 早期基线 §4.2 | 异步后 | **P0/P1 后 §8.5** |
|---:|---:|---:|---:|
| 200/s | 151.6 | 162.4 | **204.4** |
| 1/s | 189.0 | 207.7 | **222.6** |

### 4.3 Worker 与密集流分层

| 层级 | 速率 | 说明 |
|---|---:|---|
| 能量门物化 Region（密集，仅缓冲） | ~880 MS/s | 无粗/精 |
| Worker 粗/精（优化前） | ~45–48 MS/s Region IQ | ~5.5 ms/区，~180 pkt/s |
| Worker 粗/精（**P0/P1 后**） | **~201 MS/s** Region IQ | **1.31 ms/区**，理论 **~760 pkt/s** @ 264k |
| 完整块密集流（优化前） | ~35 MS/s 输入 | 待用新 worker 复测 |
| PatternSource→null_sink | **~264 MS/s** | GR plumbing 基线 |
| 1 pkt/s 门输出 Region IQ（壁钟） | ~0.7–1.2 MS/s | ≪ worker 容量 |

### 4.4 发包率扫描下的输出流规律

输出 PDU IQ 流 ≈ 输入吞吐 × 占空比 × (202032/256448)，与发包率近似线性：

- 200 → 100 → 10 → 1 包/s：约 **8.34 / 4.38 / 0.49 / 0.09 MS/s**。

---

## 5. 表面矛盾（用户问题的形式化）

**观察 A**：能量门能把高速率输入压缩成很低的 Region/包事件率  
（1 包/s 时 Region IQ 仅 ~1 MS/s 量级）。

**观察 B**：Detector worker 在持续 Region 压力下仍可达 ~200 MS/s Region IQ  
（数百 packet/s 量级）。

**观察 C**：A 与 B **合在同一 GR 块里**跑稀疏端到端时，输入吞吐只有 **~200–220 MS/s**  
（≈ 1 GS/s 的 20–22%），且 **几乎不随发包率下降而逼近 1 GS/s**（1 包/s 仅略高于 200 包/s）。

**朴素期望**：  
`端到端 ≈ min(门限容量, 被 region 负载占用后的剩余)`  
稀包时应接近门限/GS 级。

**现实**：  
稀包端到端贴在 **GR source plumbing ~264 MS/s** 附近，而非门限 2 GS/s。

---

## 6. 分析结论（当前团队共识）

### 6.1 核心论点

> **门限压缩的是「包事件 / 相关计算」；  
> GR 路径仍以全速率搬运每个 IQ 样本。  
> 稀包下 bottleneck 是「静默全速率数据平面 + GR scheduler/buffer」，不是 worker。**

### 6.2 壁钟时间预算（证明 worker 不是稀包瓶颈）

**1 包/s，1 G 样本，输入 ~223 MS/s，壁钟 ~4.5 s：**

| 项 | 估计 |
|---|---|
| Worker（2 region × 1.3 ms） | ~2.6 ms |
| 占壁钟 | **~0.06%** |

**200 包/s，500 M 样本，输入 ~204 MS/s，壁钟 ~2.45 s：**

| 项 | 估计 |
|---|---|
| Worker（101 × 1.3 ms） | ~131 ms |
| 占壁钟 | **~5%** |

→ 稀包 **95–99%** 时间不在粗/精相关。

### 6.3 成本拆分模型

```
输入 1 sample (CF32 = 8 B)
    │
    ▼
[1] Source 生成/拷贝进 GR buffer
    │  （PatternSource: 包 memcpy + gap memset）
    │  （buffer 常 8192 items → 调度频繁）
    ▼
[2] UwbDetector::work → sm.process
    │  SEARCH: pre_ring 全速率 memcpy + D=100 能量
    │  IN_REGION: region 全速率 append（仅包附近）
    │  enqueue 有界 job
    ▼
[3] 后台 worker
       horizon 粗相关 + first-peak 精 + PMT 拷贝 PDU

稀包：时间 ≈ [1]+[2]  → 总吞吐 ~200–260 MS/s
密包：[3] 上升        → 总吞吐可掉到数十 MS/s（旧密测 ~35）
```

### 6.4 为何不能从「门 2 GS/s + worker 200 MS/s Region」推出「合路应接近 1 GS/s」

| 因素 | 说明 |
|---|---|
| 测量拓扑不同 | gate/region 无 GR；sparse 有完整 scheduler |
| Plumbing 天花板 | Source→null_sink 仅 ~264 MS/s |
| 全速率 pre_ring | 静默也必须写 ring（正确性：pre-trigger） |
| 多级拷贝 | Source 缓冲 → work 输入 → ring →（触发时）region →（检出时）PMT |
| Buffer 钳位 | 请求 4M 仍见 8192 → 小 chunk 放大 per-call 开销 |
| CPU 利用率 | sparse 约 90–92%：并非单纯等 I/O，仍有算力/同步成本 |

### 6.5 带宽直觉（CF32 @ 1 GS/s）

| 操作 | 量级 |
|---|---|
| 读输入一次 | 8 GB/s |
| pre_ring 再写一次 | +8 GB/s |
| GR 中间缓冲再拷 | 再 +8 GB/s 量级 |

裸 gate 已证明状态机本身可 multi-GS/s；**叠加 GR 多层缓冲后，当前 harness 掉到 ~0.25 GS/s**。

### 6.6 已尝试且记录的优化

| 尝试 | 结果 |
|---|---|
| 粗相关 VOLK | 有效 |
| 状态机 block loop + bulk ring | gate → multi-GS/s |
| Region 池预分配 + 异步 worker | 减少 work 阻塞与分配 |
| preamble-horizon + first-peak fine | worker **4.14×**；高包率 e2e +35% |
| 粗相关 stride>1 | **不可行**（降采样峰极窄，漏峰） |
| 单独调大 source buffer 到 4M | **仍被 GR 钳 8192**，无明显提升 |
| 能量门全量 VOLK | 负结果；16% 稀疏 norm 已优 |

---

## 7. 架构图：正确 vs 错误的心智模型

### 7.1 错误模型

```
1 GS/s ──► 门限过滤 ──► 只处理稀疏 region ──► 应接近 1 GS/s
```

### 7.2 正确模型

```
1 GS/s 全速率数据平面 ──────────────────────────────► 永远存在
         │
         ├── 门限：稀疏「是否开 region」
         └── worker：只在 region 上做 O(horizon) 相关
```

端到端输入吞吐：

```
T_e2e ≈ min( T_plumbing_silence , T_when_regions_busy )
```

- 稀包：`T_plumbing_silence` ≈ 200–260 MS/s（当前）  
- 密包：`T_when_regions_busy` 由 worker + 物化主导  

---

## 8. 已知产品缺口（与吞吐正交或相关）

| 优先级 | 项 | 说明 |
|---|---|---|
| P0 正确性 | 截包长度不保证固定 | 缺独立 CAPTURE/HOLD_OFF；短包实测可仅 ~14k vs 配置 202k |
| P0 验证 | MATLAB 逐样本对照 | 检测 start / 截取范围需与 `UWB_demodulation/` 对齐 |
| P1 性能 | 稀包 1 GS/s | **本文主题：数据平面 / GR plumbing** |
| P1 性能 | 密包 e2e 复测 | P0/P1 worker 后 dense 尚未重测 |
| P2 | SC16 早期量化 | 减带宽；writer 路径已有 SC16 经验 |
| P2 | D=8 默认化 | 冒烟过，需完整 QA + MATLAB |

---

## 9. 建议的下一步实验（供外部模型排序/增补）

按信息量优先（团队建议）：

1. **三档同 buffer 对比（钉死归因）**  
   - A: `vector_source → null_sink`  
   - B: `vector_source → UwbDetector`，阈值极高 / 强制不进 IN_REGION（纯 SEARCH）  
   - C: 正常 sparse  
   - 预期：若 A≈B≈C≈200–260，则几乎全是 plumbing；若 B≪A，则 pre_ring/work 是主因。

2. **chunk / buffer 扫描**  
   - `max_noutput_items`、`set_max_output_buffer`、GR 内部 min/max  
   - 目标：能否从 ~220 推到 400–600+ MS/s。

3. **pre_ring 稀疏化原型**  
   - 大 gap 时仅维护「可能触发邻域」或降采样 ring，触发前再精确回填（若有历史）  
   - 必须量化：漏 pre-trigger / 门延迟对 start 误差的影响。

4. **零拷贝 / 旁路 GR 的静默路径**  
   - 自定义 source 与 detector 共享 ring；或 non-GR 高速 ingest + 仅事件进 GR。

5. **CF32→SC16 在门前量化**  
   - 带宽减半；与 MATLAB/writer 格式对齐。

6. **密集流用新 worker 复测**  
   - 确认高占空比下 e2e 是否从 ~35 提升到接近 worker 上限。

7. **perf/flamegraph 在 sparse 1 包/s**  
   - 标注：`memcpy`、`memset`、`gr::block::work`、scheduler、`RingBuffer::push` 占比。

---

## 10. 给 GPT 的约束与期望输出格式

### 10.1 约束

- 不得假设「naive 全速率 1016-tap 相关」可接受（见 `../../AGENTS.md`）。
- 检测算法须可对照 MATLAB / 现有 QA；改 horizon、D、门限需说明正确性风险。
- 优先 **能量门 → 候选区 → preamble 验证 → 截取** 流水线，不要退回全流相关。
- 注意 WSL2 测量噪声；以数量级与相对排序为主。

### 10.2 期望输出

请给出：

1. **Bottleneck 判断**（同意/修正本文 §6，并指出最不确定处）。  
2. **优化路线图**（P0/P1/P2，每项：改动面、预期加速、风险、验证方法）。  
3. **是否建议改变架构**（例如：静默旁路、双块拆分 energy_gate|preamble、GR 外 ingest）。  
4. **具体可落地的实验设计**（命令级或伪代码级，可对接现有 `benchmark_detector`）。  
5. **1 GS/s 可行性评估**（在 i7-12700 + GR 3.10 + CF32 假设下：乐观/中性/悲观吞吐与条件）。

---

## 11. 关键路径与复现命令

```bash
cd ~/workspace/uwb-gnuradio
B=gr-uwb/build/apps/benchmark_detector
CF=testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile

# 仅能量门（无 GR）
"$B" "$CF" detector-gate --target 200000000 --gap 4735552    # 200/s
"$B" "$CF" detector-gate --target 1000000000 --gap 998143552 # 1/s

# 全流水线（GR）
"$B" "$CF" detector-sparse --target 500000000 --gap 4735552
"$B" "$CF" detector-sparse --target 1000000000 --gap 998143552

# 纯 worker Region 压力
"$B" "$CF" detector-region --regions 200
"$B" "$CF" detector-profile --regions 200

# 密集流
"$B" "$CF" detector-region-stream --target 50000000 --gap 6000
```

关键源码锚点：

| 内容 | 位置 |
|---|---|
| 生产 `work` / worker / `publish_packet` | `gr-uwb/lib/uwb_detector.cc` |
| 状态机 SEARCH/IN_REGION / pre_ring | `gr-uwb/include/gnuradio/uwb/uwb_detector_core.h` |
| gate/sparse/region/profile 基准 | `gr-uwb/apps/benchmark_detector.cc` |
| 发包率与 profile 完整表格 | `测试报告_发包率扫描.md` |
| 状态汇总 | `../../开发状态.md` §5.3 |

---

## 12. 一句话摘要（可贴进 prompt 开头）

> 能量门裸跑 multi-GS/s、worker 可处理数百 pkt/s 的 Region，但 GNU Radio 全链路稀疏场景输入吞吐只有 ~200 MS/s，贴近 Source plumbing（~264 MS/s），因静默期仍全速率经过 source/buffer/`pre_ring`；worker 在 1–200 包/s 下仅占壁钟 ~0–5%。要达 1 GS/s 应优先优化全速率数据平面与 GR 缓冲/拷贝，而非继续压粗相关。

---

## 附录 A：术语表

| 术语 | 含义 |
|---|---|
| Region | 能量门打开期间缓冲的一段 IQ（含 pre-trigger），固定约 264k @ 参考包 |
| SEARCH / IN_REGION | 状态机两态；尚无独立 CAPTURE/HOLD_OFF |
| PDU | PMT pair(meta, c32vector)，经 message port 发出 |
| horizon | 粗相关扫描上界，覆盖 SYNC+SFD+margin，避免扫 payload 尾 |
| plumbing | 与算法无关的源生成、GR buffer、scheduler 开销 |
| drop | region 池或 job 队列满时丢弃的候选区 |

## 附录 B：数据可信度备注

- 短跑（200 M / <0.3 s）吞吐有波动；1 G 样本更稳。
- WSL2 与主机负载影响 GR 数值 ±10–20% 量级曾出现。
- PatternSource buffer 钳位 warning 多次出现；单独加大 buffer 请求未解决。
- 密集 ~35 MS/s 为 **worker 优化前** 数据；优化后未重测 dense e2e。
- 输出流与「实时 1 GS/s 下的包率 × 截包长」不同：表中输出流是 **壁钟速率**（受输入吞吐限制）。

---

*文档结束。可直接将全文或 §0–§10 粘贴给 GPT 做架构分析。*
