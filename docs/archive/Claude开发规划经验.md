# Claude 软件开发规划经验（从最大 UWB Session 提炼）

> 来源：Claude Code session `60304e00-c2d5-4cd1-ad81-dffc3696f60d`  
> 日志：`~/.claude/projects/-home-junqima-workspace-uwb-gnuradio/60304e00-c2d5-4cd1-ad81-dffc3696f60d.jsonl`（约 16.5 MB，全库最大）  
> 工作区：`/home/junqima/workspace/uwb-gnuradio`  
> 主题：GNU Radio `gr-uwb` —— 检测 → 截包 → 写盘 → 周期旁路 → 实时解调 R0–R6  
> 规模：~7308 行 jsonl，~68 轮真实用户指令，64 次 TaskCreate，693 次 Bash  
> 整理目的：学习 Claude 如何规划与推进大型软件开发，而非复述聊天原文

---

## 1. Session 定位

| 项 | 内容 |
|---|---|
| Session ID | `60304e00-c2d5-4cd1-ad81-dffc3696f60d` |
| 相对规模 | 同机 Claude 日志中最大；次大 USRP 相关约 1.5 MB |
| 同项目其它资产 | `../../AGENTS.md`、`../../开发需求参考.md`、`../../开发状态.md`、`../../开发方案_UWB实时解调.md`、`~/.claude/.../memory/uwb-*.md` |

同目录还有更小的 session；**内容最大、规划最完整的是这一条长会话**。

---

## 2. 总体方法：先消歧义，再拆任务，再写代码

用户入口类似：

> 参考 `../../AGENTS.md`，完成 gnuradio UWB 模块……算法参考 `UWB_demodulation`

Claude **第一动作不是写 `.cc`**，而是：

1. **读项目规范**（`../../AGENTS.md` / `../../开发需求参考.md`）
2. **并行 Explore** 三路：
   - MATLAB 算法（能量门、前导相关、常数）
   - 现有 `gr-uwb` OOT 模块状态
   - 测试数据 + 构建环境
3. **用真实信号实证关键常数**（例如 SYNC 周期 = **1016**，而不是只信文档）
4. 再 `TaskCreate` 拆任务并开工

**可复用原则**：规划 = 先消歧义（算法 / 环境 / 现有代码 / 真值数据），再定实现清单。

---

## 3. 任务拆解：依赖序 + 可验收，不是功能堆砌

### 3.1 首轮检测模块（7 步）

```text
读 MATLAB 常数
  → 独立 detector_core（无 GR 依赖）
    → energy block / preamble block
      → CMake / pybind / GRC 接线
        → QA + 真实信号测试
          → build / install / 跑通
```

### 3.2 后续各大阶段的任务形态

| 阶段 | 任务形态 |
|---|---|
| 性能 | 先 profile → 再改算法 → 再独立 benchmark 验证 |
| 合并管线 | 状态机 → 粗+精+PDU → QA+真信号 |
| Writer / 多包 | 写盘块 → MATLAB reader → e2e → 多包/相邻包 |
| 解调 R0–R4 | **先 golden / schema / 空接口**，再按 stage 实现 |
| R5 异步块 | 接口设计 → worker 池 → GRC → QA(stop/drain/queue) → soak |
| R6 | profile 瓶颈 → 优化 or 加 worker → e2e → soak |

### 3.3 Task 描述习惯

每个 `TaskCreate` 尽量写清：

- **做什么**
- **对照什么**（MATLAB / golden / 真值 start）
- **完成标准**（ctest 全绿、metric 阈值、吞吐数字）

示例风格：

- 「与 golden（period=1016, 64 peaks）对照」
- 「真实信号 start_sample≈4992000」
- 「benchmark ≥200 MS/s；检测与全速参考一致」

---

## 4. 文档分层：方案 / 状态 / 记忆

规划被刻意拆成多类工件（本 session 中 Claude 持续维护）：

| 类型 | 例子 | 作用 |
|---|---|---|
| **长期规范** | `../../AGENTS.md`、`../../开发需求参考.md` | 约束：block 类型、禁止 work 内分配、必须 QA、对照 MATLAB |
| **阶段方案** | `../../开发方案_UWB实时解调.md`、`GROK_X410_…方案.md` | 架构图、阶段划分、实时定义、非目标 |
| **进度快照** | `../../开发状态.md` | 已完成 / 缺口 / 优先级 P0–P2 / 生产推荐路径 |
| **经验记忆** | `~/.claude/.../memory/uwb-*.md` | 坑：FIR 抽头反序、stop() 消息、D=100 门窗=32 等 |

用户中途明确要求「更新开发状态到新文档」时，Claude 会把 **完成证据 + 下一步** 写回仓库，让下一轮（或另一个 agent）能接着干。

**分工口诀**：

- **方案** → 要做成什么样  
- **状态** → 现在到哪、差什么  
- **memory** → 已经踩过的坑别再踩  

---

## 5. 规划必须可度量

性能相关规划尤其明显：先立**基准表**，再谈方向。

示例结构：

| 环节 | 吞吐 | 瓶颈判断 |
|---|---|---|
| energy detector | ~90 MS/s | flowgraph/sink 物化，不是算子本身 |
| preamble fast | ~61 MS/s | 能量 O(n) + 候选区全速 FIR |

每个优化方向写清：**目标 / 设计 / 收益 / 风险**。

解调方案里把「实时」也量化：

1. **持续实时**：100/200 packet/s，队列不增长、0 job drop  
2. **低延迟实时（进阶）**：200 packet/s 时 p99 latency < 5 ms  
3. **架构约束**：完整 1 GS/s 流只做 detection/extraction；解调只处理稀疏 PDU  

**原则**：没有数字的「优化一下」不算规划；先测基线，再选杠杆。

---

## 6. 架构原则（贯穿全 session）

1. **算法 core 与 GR 解耦**  
   `uwb_detector_core.h` / `uwb_demod_core.h` 可单测、可独立 benchmark。
2. **先稀疏、后全流**  
   能量门 → 候选区 → 粗相关 → 精相关 → PDU 截包；禁止 naive 全速率相关。
3. **正确性优先于吞吐**  
   每阶段：ctest + 真实 cfile +（解调时）MATLAB golden 逐字段对照。
4. **实现前先定 block 语义**  
   sync_block vs message/PDU、scheduler 语义、stop/drain、队列满只 drop demod 不堵前端。
5. **分阶段裁剪 scope**  
   如 R0 固定一种 PHY profile；ScheduledExtractor Phase-1 不做 `rx_time` 失锁——验收时诚实写「与方案完成标准的差距」。

生产路径（状态文档中的推荐）示意：

```text
通信截包：  IQ → uwb.detector → PDU → packet_writer → MATLAB
周期雷达：  IQ → scheduled_extractor → PDU → 离线 MATLAB / SIC
实时解调：  PDU → UwbRealtimeDemodulator（异步 worker，不反压前端）
```

---

## 7. 大型功能的阶段化模板：实时解调 R0–R6

这是本 session 最教科书的一段规划执行：

```text
R0  冻结 PHY profile + MATLAB golden 导出 + result schema + 空 core/benchmark
R1  Timing + CFO + SFD（先「站稳帧」）
R2  CIR + Soft chips
R3  NS-SFD + PHR
R4  Payload + RS + FCS
R5  GR 异步 block（worker 池、队列、stats）——大块可委托 Grok
R6  100–200 pkt/s soak / 瓶颈（sfd 优化或加 workers）
```

**特点**：

- **R0 不写算法实现**，先把真值和接口钉死（契约先行）  
- 每 Ri 结束有：交付文件表 + golden 对照结果 +（常有）按阶段 commit  
- 难且量大的块（RS 译码、R5 async block）**并行委托 Grok**；Claude 做接口、QA、集成与修坑  

这就是典型的 **垂直切片 + 契约先行 + 阶段门禁**。

---

## 8. 执行循环（规划是反复的，不是一次性的）

从工具使用分布看出的稳定节奏：

```text
探索 / 读参考
  → TaskCreate 拆任务
    → 实现（Edit / Write）
      → 构建 + ctest + e2e / benchmark（Bash 占大头）
        → 总结完成情况（表格）
          → 写 memory / 更新 开发状态
            → 用户下一条需求 → 再规划
```

中途用户打断、改方向、让 Grok 先改 SC16、问瓶颈时，Claude 会 **重新 profile 再改计划**，而不是死磕旧 todo。

Context 满时多次 `/compact`；能撑住超长会话的关键是：

- 磁盘上的 **方案 / 状态 / memory**
- compact 后的 **session summary**
- 可重跑的 **ctest / e2e / benchmark**

---

## 9. 人机协作节奏（本 session 观察到的模式）

### 9.1 用户侧

- 给 **大目标 + 参考路径**（AGENTS、MATLAB 目录、开发方案）
- 中途给 **方向性约束**（降采样倍数、合并管线、SC16、R0–R6 继续）
- 要求 **写回文档**（开发状态、测试报告）
- 对大块能力让 Claude **调用 Grok Build** 并行

### 9.2 Claude 侧

- 探索 → 任务板 → 实现 → 证据型总结  
- 大任务拆小、小任务可验收  
- 验收对照方案写 **差距**（不把 Phase-2 伪装成完成）  
- 修 Grok 交付物时先定位接口/命名空间/PMT API 等集成问题  

### 9.3 适合委托出去的工作

| 更适合 Claude 主导 | 更适合委托（如 Grok） |
|---|---|
| 架构、阶段划分、验收标准 | 大块独立实现（RS 译码、async block 初版） |
| golden 对齐、排坑、集成 | 重复性脚手架、体量大但边界清晰的代码 |
| 对照方案写结论 | 在明确 spec 下的「按图施工」 |

---

## 10. 本 session 踩过、值得记住的工程坑（摘要）

更完整条目见 `memory/uwb-*.md`；这里列规划相关启示：

| 坑 | 启示 |
|---|---|
| FIR 匹配滤波 taps 需逆序 | 参考实现细节必须对照 GR API，不能只移植公式 |
| dual-output + sink 物化锁吞吐 | 测性能要避开错误瓶颈；用独立 benchmark |
| 降采样能量门窗过小会漏峰 | 先 numpy/原型验证再写进 core |
| `stop()` 发的消息可能收不到 | 测试与生产语义要覆盖 EOS / 尾静默 |
| golden 索引 0/1-based | 契约文档要写清索引约定 |
| RS 本原多项式非常见默认值 | 与 MATLAB 不一致时枚举/对照，而不是猜标准库默认 |
| 1 GS/s 只在大 buffer 下成立 | 方案验收要区分 core / GR 默认 chunk / 生产缓冲 |

---

## 11. 可直接套用的 Checklist

规划类似项目时，可按此骨架：

1. **入口文档**：目标、非目标、性能数字、参考实现路径  
2. **并行摸底**：参考算法 / 现有代码 / 环境与数据  
3. **钉死常数与真值**（真实信号或 golden 实证）  
4. **分层架构**：core（纯算法）→ block（I/O 语义）→ GRC/绑定 → 测试/benchmark  
5. **任务 DAG**：每项有验收标准（测试名、指标、对照对象）  
6. **先通主链路，再优化**（检测 → PDU → 落盘 → 解调）  
7. **每完成一大块**：更新状态文档 + 记下坑（memory）  
8. **验收时对照方案写差距**，不把后续 Phase 伪装成完成  
9. **Context 紧张前**：把结论写进仓库文档，而不是只留在对话里  
10. **大块并行**：边界清晰、可独立测试的模块再委托；主 agent 保留集成与门禁  

---

## 12. 推荐阅读顺序（本仓库）

1. 本文：`Claude开发规划经验.md`（方法总览）  
2. `../../AGENTS.md`（硬约束）  
3. `../../开发状态.md`（进度与缺口）  
4. `../../开发方案_UWB实时解调.md`（阶段方案范例）  
5. `~/.claude/projects/-home-junqima-workspace-uwb-gnuradio/memory/MEMORY.md` 及其链接的 `uwb-*.md`（阶段坑与决策）  

原始对话日志仅作考古；**可执行的规划资产是上述 Markdown**。

---

## 13. 一句话总结

> Claude 在本 UWB 长 session 中的规划能力，核心不是「一次写完大计划」，而是：  
> **用文档锚定约束 → 用探索消歧义 → 用可验收任务推进 → 用测试与数字闭环 → 用状态/记忆续航**，并在阶段边界诚实裁剪 scope。
