# GNU Radio Phase-2 实时 SIC 开发方案

更新时间：2026-08-10

## 1. 状态与目标

**状态：S0/S1 基础开发中，SIC 尚未实现。** 本文冻结 Phase-2 的接口、调度
语义、首版算法边界、验证方法和性能验收。当前已落地 production profile
常量 golden 和复数 CIR 结果接口，但还没有 SIC C++ block、相减 core 或硬件
前端预处理闭环。

Phase-2 面向已知 `t0/T` 的 QM35825 雷达时隙。系统在每个 scheduled slot 的
扩展上下文内解调并重构 DW1000 干扰，只提交经过安全门验证的相减，最终输出：

1. 去除 DW1000、保留 QM35 的 `cleaned_slot` CF32 PDU；
2. 净化结果上的 `post_sic_cir` 复数 CIR PDU；
3. 可审计的 `result/status`。

首版目标是 100～200 slot/s 持续实时，容量不少于 250 slot/s；不承诺零延迟
连续 residual stream。

## 2. MATLAB 基线为何不能直接搬入实时链路

`UWB_demodulation/sic_pipeline/` 是算法真源，但其全文件调度不是生产模型：

- 两次全文件检测/解调、两次整文件复制和逐包回写；
- 验证阶段还会第三次完整解调 QM35；
- 单包消除包含 Communications Toolbox 波形生成、整数/分数对齐、两次 CFO
  拟合、CIR 慢相位 SVD、密集平滑求解、全包 SFO 多窗口插值搜索及多套 A/B
  模型。

这些成本适合作为离线 golden 和增强算法实验，不应成为每 5～10 ms 到达一个
slot 时的调度语义。实时版采用“稀疏 PDU、direct-first、最多 4 个干扰包、
有界队列和预分配 worker scratch”。

## 3. 前置差异与阻塞项

### 3.1 输入预处理尚未闭环

MATLAB SIC 输入已完成：

```text
重采样到 998.4 MHz → 同步单音去除 → 数字频移 10 MHz
```

当前 X410 路径只完成 737.28 MS/s 经 RFNoC 65/48 重采样到 998.4 MS/s。
在单音去除、数字频移和 MATLAB 逐样本对照闭环前，硬件入口不能宣称满足
SIC 输入契约。首版 block 本身不负责连续流前端预处理。

### 3.2 DW1000 profile 不一致（已修正）

生产 MATLAB DW1000 使用 **code 11、128 SYNC、Decawave SFD**。C++
`Dw1000Profile` 已按该身份修正；原 code 10、64 SYNC、4z2 profile 已更名为
`SyntheticCode10Profile`，只允许用于合成通信碰撞 fixture。完整 code-11、
sampled code 和 SFD 由 MATLAB R2025b 导出到 `testdata/sic_profile_golden/`，
并由 C++ QA 逐元素复验。

### 3.3 复数 CIR 接口缺失（已修正基础接口）

`CirResult::cir_complex_values` 现保留 L2 归一化复数 taps；旧
`CirResult::cir_values` 继续表示其实部，以兼容 Phase-1 诊断和 golden。
真实 998.4 MHz DW1000 窗口及 MATLAB 复数 CIR 已版本化到
`testdata/dw1000_realtime_golden/`；C++ 已逐 tap 验证，归一化相关度
0.999998，并完整解出 12-byte PSDU 与 FCS `0x4440`。这闭环了解调侧输入，
但 TX pulse impulse、重构波形、相减结果和 mixed-signal post-SIC CIR 仍不能由
该 golden 替代。

## 4. 数据流与输入/输出契约

```text
预处理后的 998.4 MHz CF32 IQ
        │
        └─ UwbScheduledExtractor（扩展上下文、every_slot）
              ├─ raw PDU ─────────────────────→ Async PacketWriter
              └─ context PDU → UwbRealtimeSic worker pool
                                  ├─ cleaned_slot PDU
                                  ├─ post_sic_cir PDU
                                  └─ result/status
```

### 4.1 上下文窗口

目标 QM35 输出区间的左右两侧各覆盖一个最大 DW1000 帧长，再各留 128 samples
整数对齐保护；总上下文目标约 660 us，并强制小于雷达周期。输出不携带整段
上下文，而是裁剪回约 204 us 的目标 QM35 slot。

若配置无法同时满足覆盖范围和“小于周期”，启动时失败，不静默缩短上下文。
slot 靠近采集边界导致上下文不完整时，原样输出可用目标区并报告
`context_truncated`，不得执行跨边界相减。

### 4.2 输入 metadata

至少要求：

```text
schedule_index
window_start_sample       # context PDU 的绝对 0-based 起点
predicted_start_sample    # 目标 QM35 的绝对 0-based seed
sample_rate               # 必须为 998.4e6
sample_count
target_start_offset
target_sample_count
preprocessing_profile/version
```

所有索引均使用 `uint64_t` 绝对样本坐标；局部 offset 必须显式转换并检查上下界。

### 4.3 输出接口

`cleaned_slot` 使用 `meta + c32vector`。metadata 继承 schedule 和绝对坐标，并
增加 `sic_applied`、`dw_candidates`、`dw_cancelled`、`failure_reason`、总耗时和
安全门摘要。

`post_sic_cir` 使用 `meta + c32vector`，携带归一化复数 taps、first-path 绝对
坐标、pre/post samples、timing/CFO metric 和归一化约定。

`result/status` 每个输入 job 恰好发布一次，至少记录：

- `schedule_index` 和绝对窗口/目标坐标；
- 候选、FCS-valid 和已提交相减的数量；
- 每包 packet start、payload/FCS、相减 `[begin,end)`、对齐相关度和抑制度；
- `sic_applied`、失败/拒绝枚举、worker 时间、排队时间；
- received/completed/dropped/exception、queue high-watermark。

raw PDU 是独立不可变分支；SIC 不拥有修改它的权限。

## 5. GNU Radio block 与调度语义

新增 `gr::uwb::UwbRealtimeSic`，类型为零 stream 输入/输出的 `gr::block`：

```text
message input : slot
message output: cleaned_slot, post_sic_cir, result/status
```

选择 message/PDU block 的原因是 SIC 必须等待完整上下文、完成 DW1000 payload
和 FCS 解调后才能重构。它是按 slot 的异步有界延迟任务，不符合 `sync_block`
逐样本生产/消费关系，也不是 `tagged_stream_block` 的同步计算。

消息处理器只完成 schema/长度/坐标校验和有界入队。队列满时立即旁路原 IQ、
发布 `queue_full` 结果，不等待 worker。worker 数和队列容量在构造期固定；每个
worker 独占并复用模板、解调器状态和预分配 scratch，热路径不得扩容。

多个 worker 按完成顺序发布，不设置全局 head-of-line reorder。消费者依靠
`schedule_index` 和绝对样本坐标关联。如果以后需要有序输出，应在独立有界
reorder block 中实现。

`stop()` 默认停止接收新 job、排空已接收队列并发布所有结果；异常只能使当前
job 原样旁路，worker 随后继续服务。QA 必须覆盖 drain、重复 stop 和异常恢复。

## 6. 首版 direct-first 算法

每个 slot 执行：

1. 在原始混合上下文中使用生产 code-11 profile 搜索 DW1000。
2. 若没有可信 DW 候选，跳过完整 QM35 解调，直接对目标 seed 做 QM35
   timing/CFO/CIR-only，输出未修改 slot 和 post-SIC CIR。
3. 若 DW1000 直接解调且 FCS 成功，在原始工作副本中重构并试减。
4. 仅当存在强 DW 候选但直接解调失败时，完整解调 QM35；在临时搜索副本中
   消除 QM35，再重试 DW1000。临时副本的 QM35 相减永不进入最终输出。
5. 候选数不超过上限时，从原始工作副本按确定顺序消除最多 4 个 FCS-valid
   DW1000 包；每次提交后，后续候选都基于更新后的 residual 校验。
6. 使用已知 QM35 seed 在最终净化结果上执行 timing/CFO/CIR-only；不做第三次
   QM35 payload 全解调。

候选超过 4 个时不扩大工作量，也不选择性提交部分结果：报告
`candidate_limit`，原样输出目标 slot，设置 `sic_applied=false`。这样候选
超限与 queue-full/异常具有相同的 fail-safe IQ 语义。

### 6.1 首版重构模型

只实现：

```text
decoded frame → TX pulse impulse → normalized complex CIR
              → integer alignment → CFO → global complex gain → subtraction
```

构造期缓存 profile、扩频码、SFD、脉冲 impulse 和固定工作区。首版不实现
fractional delay、PLL transient、CIR-slow、二级 CFO（CFO2）、全包 SFO 或
字段独立增益；这些只能作为通过独立 golden/A-B 测试后启用的可选增强。

### 6.2 相减安全门

每个 DW1000 包只有同时满足下列条件才提交：

```text
payload FCS valid
alignment correlation >= 0.70
trial-subtraction suppression >= 0.20 dB
```

抑制度必须在实际相减 `[begin,end)` 上，以相同度量比较 trial 前后 residual。
任一门失败时丢弃 trial、保持工作副本逐样本不变并记录原因。禁止通过放宽门限
掩盖 profile、索引、符号或相位错误。

## 7. 接口和 profile 演进

S0 的兼容性改动（profile/schema 部分已落地）：

- `CirResult::cir_complex_values` 保存归一化复数 taps；现有 `cir_values` 的实部
  诊断语义保持，避免破坏已有 QA 和诊断；
- `Dw1000Profile` 使用 production code-11/128/Decawave；
- 历史 code-10/64/4z2 fixture 使用 `SyntheticCode10Profile`，禁止自动映射成
  production；
- profile 和 preprocessing version 写入每个输出，golden 也携带同一标识。

后续 block 修改仍须按 `AGENTS.md` 搜索本地 GNU Radio 相似 block、确定 block
语义、先加 QA，并在每个 meaningful modification 后 build/test。

## 8. 分阶段路线 S0～S5

| 阶段 | 内容 | 完成门槛 |
|---|---|---|
| S0 契约/golden | 冻结预处理、code-11 profile、复数 CIR schema、上下文坐标；从 MATLAB 导出 golden | manifest 可在无 MATLAB 环境复验，profile/索引/相位无歧义 |
| S1 复数 CIR/profile | C++ 复数 CIR、生产 DW profile、TX impulse；不实现相减 block | 既有 QA 兼容，新增逐 tap/profile golden 通过 |
| S2 单包 SIC core | 整数对齐+CFO+CIR+全局增益、trial/commit 安全门 | 单 DW 重构、区间和 residual 对 MATLAB 逐样本通过 |
| S3 自适应 slot core | direct-first、无 DW 快路、QM35 fallback、最多 4 DW、post-SIC CIR | 功能矩阵和 mixed golden 全通过 |
| S4 异步 block | PDU schema、有界队列/worker pool、旁路、统计、stop/drain | queue-full/异常不阻塞且 raw/cleaned/result 语义完整 |
| S5 实时验收 | 端到端 benchmark、10 分钟 soak、硬件预处理闭环 | 250 slot/s 容量；200 slot/s p99 <20 ms；规定场景 0 drop |

当前进度：S0 的 production profile、复数 CIR schema、真实 DW1000 解调窗口及
manifest 已完成；S1 的 complex CIR、payload/FCS、178112-sample TX pulse
impulse、字段边界和 complex-CIR replica 均已闭环。S2 的 `uwb_sic_core.h` 已用
真实 trial received/model/residual 验证整数对齐、逐 SYNC CFO、全局复增益及
0.70/0.20 dB 安全门：C++/MATLAB model 相对 L2 误差 `1.55e-7`，相减区间外
误差为 0，真实 FCS/相关度/抑制度拒绝路径均逐样本旁路。新增
`uwb_tx_reconstructor.h` 后，C++ TX impulse 与 MATLAB 全向量一致，同 CIR
replica 相对 L2 `5.19e-8`；完整 C++ decode/reconstruct/cancel 抑制度 16.2894 dB。
S2 已完成，下一阶段为 S3 自适应 slot core。

不得以 synthetic code-10 或仅合成 trial/commit 结果宣称真实 SIC 正确。

## 9. MATLAB Golden 与验证矩阵

`UWB_demodulation/` 的实际 `.m` 入口是算法真源，但该目录当前不作为本轮文档
改动目标。必要结果导出到已跟踪的 `testdata/`，包含：

- QM35 与真实 DW1000 profile（扩频码、SYNC 数、SFD、字段参数）；
- 归一化复数 CIR、first path 和相位/尺度约定；
- TX pulse impulse 与字段边界；
- 单包重构波形、对齐坐标、全局复增益和相减 `[begin,end)`；
- 原始 mixed、DW trial/committed residual 和 post-SIC QM35 CIR；
- 生成脚本版本、输入 hash、采样率、预处理版本和 0-based 坐标 manifest。

C++ 与 MATLAB 必须比较 packet start、payload/FCS、复数 CIR、重构波形、实际
相减区间和 post-SIC CIR。容差在 S0 由 golden 的数值精度和算法等价性确定，
不能在失败后随意放宽。

### 9.1 C++ QA

至少覆盖：

1. 无 DW：不运行完整 QM35 fallback，IQ 不变，仍输出复数 CIR；
2. 单 DW、多个 DW，以及 direct-first 连续消除；
3. 强候选直接失败后 QM35 临时消除并重试；
4. FCS 失败、相关度低和抑制度低分别拒绝且逐样本不修改；
5. 上下文头尾截断、相减区间越界和绝对/局部坐标转换；
6. 候选超过 4 个时工作量有界；
7. queue-full 立即旁路，result 与 raw 分支不丢；
8. stop/drain、worker 异常、后续 job 恢复；
9. 多 worker 完成乱序时 `schedule_index` 可正确关联。

## 10. 性能验收与观测

分别以 100 和 200 slot/s 持续运行 10 分钟：

- SIC 队列无持续增长，SIC job drop = 0；
- raw Writer 分支 drop = 0；
- 输入数 = cleaned/result 数，异常和拒绝均有明确状态；
- 200 slot/s 端到端 p99 latency < 20 ms；
- 独立容量 benchmark 至少 250 slot/s。

报告每阶段 CPU 时间、wall latency、worker 利用率、队列 high-watermark、候选
分布、fallback 比例、各安全门拒绝数和已提交抑制度。性能数据必须区分纯 core、
PDU/调度和 Writer I/O，不能用离线批处理均值替代在线 p99。

## 11. 首版非目标

- 连续 998.4 MHz cleaned/residual stream 或零延迟相消；
- 在 SIC block 内完成实时单音去除、数字频移或重采样；
- 全部约 2000 communication packet/s 的盲检/完整解调；
- GPU/FPGA；
- MATLAB Runtime 生产依赖或 MATLAB 四次全文件流程的原样移植；
- 未经独立验证的 fractional delay、PLL/CIR-slow/CFO2/SFO/字段增益增强；
- 以 SIC 成功与否决定是否保存 raw slot。

## 12. 文档关系

- [`开发需求参考.md`](开发需求参考.md)：Phase-1 历史基线与 Phase-2 验收需求；
- [`开发状态.md`](开发状态.md)：当前真实完成度与 S0～S5 执行状态；
- [`开发方案_UWB实时解调.md`](开发方案_UWB实时解调.md)：Phase-1 固定 profile
  实时解调基础；其“不在线执行完整 SIC”只描述 Phase-1 范围；
- [`开发总结_QM35825周期旁路雷达截取.md`](开发总结_QM35825周期旁路雷达截取.md)：
  `UwbScheduledExtractor` Phase-1 基线和 Phase-2 上游接口；
- `UWB_demodulation/sic_pipeline/`：离线算法与 golden 真源，不是实时调度模板。
