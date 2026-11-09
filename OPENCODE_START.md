# 最新优化：AGY SIC 非同步深度优化与复核整改闭环（2026-11-08）

执行 `docs/sic/AGY_SIC优化执行任务书_2026-11-08.md` 及复核整改 `docs/sic/AGY_SIC优化验收复核_2026-11-08.md`。完整开发报告见 `docs/sic/AGY_SIC优化开发报告_2026-11-08.md`，复核整改证明详见 `docs/sic/AGY_SIC优化验收整改证明_2026-11-08.md`。
涵盖：
1. **Task A (RX 数据量与零乘精简)**：CFO 旋转解耦支持区间补偿（样点 0 严格为相位原点），覆盖 Bootstrap SFD 候选及 Final SFD/CIR/PHR/Payload，消除冗余样点旋转；`stage_cir_softchips` 改为三元码非零位置（64 点）稀疏互相关，消除 1016 点稠密零乘。
2. **Task B (重构与输出写入)**：延迟旁路拷贝至失败分支，成功路径仅复制相减 ROI 两侧且直接计算 `out = y - recon` 消除双写；内部可信脉冲列表（`transaction_prepared == true`）免去 8,384 事件二次线性扫描；实现 20-tap 专用 AVX2/FMA 脉冲加减内核。
3. **Task C (参数估计与 TX 调制)**：合并前缀有限性扫描与能量累加至单遍循环；三元训练序列单周期（1016 点）预测残差；`HrpModScratch` 缓存扩频 PN 序列；修复 `prepare_discrete_cir` 中死字段引发的无效二次 reserve。
4. **Task D (复核遗留收口)**：彻底关闭 `PHR解扩CMF融合_验收复核_2026-11-07.md` 登记的 R1–R3 缺陷（R1 最大 offset 尾界及防下溢、R2 诊断重放右端范围越界防护、R3 逐轮逐观测完整功能字段校验）。
5. **复核整改 R01–R04 闭环**：
   - R01：纠正原报告历史参考口径，使用同输入、同选项 fast-runner ABBA 持锁实测配对数据。
   - R02：`uwb_rx_cfo_opt.h` 恢复 4 路 ILP 浮点递推核 `derotate_range_recurrence`，彻底消除默认配置感知解码 57.7% 的严重回退，感知解码耗时由 2276.1 µs 压至 2148.5 µs（比基线快 5.6%），端到端服务链耗时由 4564.2 µs 降至 3994.0 µs（-12.5%）。
   - R03：修复 B3 内核动态切换，增加平台/编译器宏保护，单测实证 20-tap 零 CFO 严格逐位一致，隔离对照证实 B3 AVX2 带来 2.31x 卷积加速（256.3 µs → 110.8 µs）。
   - R04：回退全窗旋转耗时前移纳入 `stage_softchip_fallback_us` 与 `stage_total_us` 父计时，满足父子守恒。
   - 范围证据：增加非整数网格显式全窗回退，并在 `DemodResult` 中新增样点数与旋转区间统计。

验证与配对性能：
- 单测与反例：`uwb_qa_uwb_demod_core`（52项）、`discrete_cir_reconstruct`（10项）、`discrete_cir_estimator`、`hrp_mod_core`、`rx_fused_cmf`（12项）、`rx_fused_phr` 针对性测试 100% 通过（2.47 s）；R3 扰动反例准确拒绝；损坏包旁路输出与输入保持逐位一致（Bit-exact）。
- 数值等价：紧凑四径主场景下输出 IQ 最大相对 L2 误差为 `1.944e-07`（门槛 ≤ 2e-05），全窗消减抑制比 `28.183925 dB`（差异 `1.89e-07 dB`，门槛 ≤ 0.01 dB），所有 FCS 与 committed 100% 保持一致。
- 计时结果（持锁 ABBA 配对，20-tap `lag_lo=-1`，old: `build-sic-fullpacket-no-input-scan` vs new: `build-sic-fullpacket-agy-opt`）：SIC 均值由 `1828.2 µs` 降至 **`1402.7 µs`**（P95 **`1680.7 µs`**），降低 **23.3%**；通信解码由 `1158.4 µs` 降至 **`967.2 µs`**（-16.5%）；重构卷积由 `200.3 µs` 降至 **`110.8 µs`**（-44.7%）；下游感知解码由 `2276.1 µs` 降至 **`2148.5 µs`**（-5.6%）；完整端到端 service 耗时由 `4564.2 µs` 降至 **`3994.0 µs`**（-12.5%）。产物位于 `gr-uwb/build-sic-fullpacket-agy-opt/results/rectification_20261108_123827`。

---

# 当前提交快照：整包 SIC 稀疏重构与验证精简（2026-11-08）

本次提交汇总当前实验代码与报告。紧凑四径、32 B、20 tap、lag_lo=-1：总 SIC mean 1.822 ms，调制+重构 227.7 µs。相较最初整包版本历史 21.83 ms 约快 12 倍；不同信道/运行的阶段对比，详见 `docs/sic/SIC_首版至当前性能对比_2026-11-08.md`。下方“未提交”为各轮测量时的历史状态。PHR R1–R3 与 SYNC 碰撞退化仍未关闭；实验选项不自动替代默认路径。

---

# 最新优化：移除稀疏事务剩余输入扫描（2026-11-08）

稀疏事务不再扫描整窗 IQ NaN/Inf；保留一次旁路准备以及 CRC/PHR/模型/边界检查。当前紧凑四径 20 tap、lag_lo=-1：validate 293.9→127.0 µs，总 SIC mean 2.032→1.822 ms（同轮配对）。两项 QA、8 窗实际输出逐位一致。报告 `docs/sic/SIC_移除剩余输入扫描耗时_2026-11-08.md`，未提交。

---

# 最新优化：事务验证与旁路准备去重（2026-11-08）

稀疏事务仅一次 IQ 扫描和旁路准备，内部重构复用结果；公开独立 apply 保留原检查。两项 QA 通过，实际输出逐位一致。两次配对全量合并：包装余项 288.4→10.7 µs，总 SIC mean 2.482→2.383 ms；运行有慢窗，P95 未改善。报告 `docs/sic/SIC_验证与旁路准备去重优化_2026-11-08.md`，代码未提交。

---

# 最新实验：稀疏路径跳过最终数值检查（2026-11-08）

稀疏脉冲路径不再执行整包重构/相减结果有限值扫描，参数与输入检查保留，旧波形路径保留原检查。当前紧凑四径 20 tap、lag_lo=-1：调制 35.3 µs、重构 203.9 µs、总 SIC 2.156 ms（本轮有检查对照 2.335 ms）。8 窗输出逐位一致，两项 QA 通过。报告 `docs/sic/SIC_跳过最终数值检查耗时_2026-11-08.md`，未提交。

---

# 当前紧凑四径 20 tap 配置（2026-11-08）

使用 `--tap-count 20 --lag-lo -1`，保留实际延迟 [-1,18]，不改变 q；重构按绝对 tap 延迟对齐。8 窗全带消减 28.184 dB，128 tap 为 28.244 dB；起点 0 也已测，为 28.153 dB。详情及命令见 `docs/sic/SIC_紧凑四径配置_2026-11-08.md`。尚未重测此起点的性能，通用默认参数未修改。

---

# 最新模拟信道：紧凑四径（2026-11-08）

`payload_four_frac` 新生成延迟为 0/2.4/5/9.2 samples，复增益保留。旧宽四径另存为 `payload_four_frac_wide`；历史 inputs16 未覆盖。新 8 窗及 20/128 tap 对照见 `docs/sic/SIC_紧凑四径配置_2026-11-08.md`。请后续使用新文档记录的输入路径，避免误读旧 inputs16。

---

# 当前任务：稀疏脉冲列表与 CFO 融合重构已交付（2026-11-08）

执行 `docs/sic/OpenCode_稀疏脉冲列表与CFO融合重构任务书_2026-11-08.md`。结果见
`docs/sic/SIC_稀疏脉冲列表重构可行性报告_2026-11-08.md`，摘要 JSON/CSV 同目录。
新增显式 `--sparse-pulse-reconstruct`（默认关闭，仅 `--reconstruction ipatov_cir`）：
调制器直接输出 SYNC/SFD/PHR/payload 的非零位置与 ±1 符号并缓存前导事件；重构在每个
脉冲位置计算 CFO 相位后叠加冻结 128 点 CIR，不再生成整包实数波形，也不再做整包 CFO
旋转。零 CFO 分支用专用加/减，非零 CFO 用逐事件复数 AXPY；输出仍清零、ROI 内相减、
有限值/容量/截断失败保持逐位旁路。

验证：2/32/127 B 事件列表展开后与旧单位实数序列逐样点一致，容量不足拒绝且预分配
不扩容；零 CFO 与旧路径逐位一致，非零 CFO 对独立标量近似公式通过；5 个正常格各 8 窗
加 127 B/损坏包各 2 窗，comm/sensing PSDU、FCS、状态、PHR bits/长度/纠错、提交结果
全部一致，损坏包 2/2 零提交且逐位旁路。聚合抑制：零 CFO/分数多径/SYNC 碰撞约不变
（±0.0002 dB），−50 kHz 0.128 dB、+50 kHz 0.741 dB 下降（旧−新）；最差单窗
−1.446/−1.665 dB。正式计时取 `sparse_pulse_timing_repeat_20261108_5`：32 B 分数
多径调制 56.8→35.1 µs、信道重构 712.3→600.6 µs、总 SIC 2713.7→2559.7 µs
（0.943）；+50 kHz 60.7→33.9 µs、723.0→603.6 µs、2759.4→2568.4 µs（0.931）；
信道与总 SIC 的 mean/P95 均下降。首次最终二进制计时 `_4` 的 +50 kHz 调制子项出现一次
慢跑（90.9 µs），按任务书只补一次同规模 `_5`，以 `_5` 为正式计时并把 `_4` 作重复性
对照。

边界：实验开关不默认；只测两格计时，不扩大矩阵；CIR 仍为原 128 点相关响应，未改
估计器/tap/3 参数约束；SYNC 碰撞本轮实测约 19.32 dB 未修复（历史 19.63 dB 口径/窗口不同）；
PHR R1–R3 未处理；MATLAB/native/FPGA/硬件/paced/P99、mean≤0.5 ms、P99≤1 ms、
200 窗/s 均未验证。
源码基于 HEAD `06ef10a` 加未提交修改，code-tested tracked patch
`29182d75da371331758a7a16027aa0fcb37e8a2e17ee1ba392683ef26a826a2c`；运行后只更新
报告与本文件，未再改源码或重建，**未提交**。

---

# 最新变更：单位实数脉冲重构（2026-11-07）

Ipatov SIC 新路径已移除峰值扫描、归一化及整包实数转复数，并直接用实数脉冲累加复数 CIR。三项 QA 通过，七格与损坏包合计 128 个新旧实际输出逐位一致。32 B 分数多径：调制 246.8→61.3 µs、信道/CFO 891.7→747.2 µs、总 SIC 3.116→2.782 ms。报告：`docs/sic/SIC_单位实数脉冲重构优化报告_2026-11-07.md`。仍为 128 点相关 CIR；SYNC 碰撞退化未改变，3 参数约束尚未集成，前次 PHR R1–R3 未处理。修改未提交。

---

# 最新实验：SIC Ipatov 相关 CIR（2026-11-07）

新增显式 `--reconstruction ipatov_cir`：用 Ipatov 冲激序列相关估计包含 shaping 的响应，整包使用单位脉冲重构，省去投影/LS。七格×16 窗解码一致，损坏包逐位旁路；分数多径参数估计 1.482→0.277 ms、总 SIC 4.415→3.148 ms。但 SYNC 碰撞抑制 35.63→19.63 dB，单径总耗时略增，**尚不替换默认后端、不宣布基线替换验收通过**。报告：`docs/sic/SIC_Ipatov相关CIR开发报告_2026-11-07.md`。保留旧路径与历史数据；本轮仍未提交，前次 PHR R1–R3 未处理。

---

# 最新变更：SYNC 逐 chip FIR 已移除（2026-11-07）

用户要求已落实到 PHR 融合快路径：仅保留 SFD 搜索 FIR，实际输出 36,585→4,081 chip；SYNC 相位改为先累加对应 IQ 再计算等价统计。四项 QA、六格功能及损坏包对照通过。详见 `docs/sic/SYNC逐chip_FIR移除报告_2026-11-07.md`。本轮未提交；前次 PHR 复核 R1–R3 仍待关闭，不把下方历史“已交付”解读为这些问题已修复。

---

# 当前任务：PHR 解扩与 CMF 融合已交付

结果见 [报告](docs/sic/PHR解扩CMF融合开发报告_2026-11-07.md)，code-tested SHA
`609dfca`。新增显式 `--rx-fused-phr-cmf`（默认关闭）：在 payload 融合基础上把
PHR 改为 164 点融合模板 + 四候选相关，缓存 21 对 g0/g1 供联合 Viterbi，并把
soft-chip 前缀从 59,740 缩到 36,585 chip（只保留 NS-SFD +8 halo 的 9 个名义
PHR chip）。6 格×16 源窗逐窗 PHR bits/长度/纠错、comm/sensing PSDU/FCS/status、
committed/reason 一致，96/96 PHR 融合、0 回退；损坏包 16/16 融合尝试、16 次
完整回退、0 误提交。正式三格 `phr_path_with_diag` 87.1/84.3/89.5→18.0/17.5/
17.5 µs（0.206/0.208/0.196），通信 mean 0.8575/0.9065/0.8735，总 SIC mean
0.9375/0.9842/0.9487，prefix FIR 0.5741/0.6555/0.5848。PHR 内核单独（不含旧
FIR）约为旧解扩/译码的 2.0–2.2 倍，收益来自移除 PHR soft-chip FIR 和冗余前缀。
MATLAB、native、FPGA/硬件、paced、P99 与实时目标未验证。

---

# 历史任务：CMF 与解扩融合可行性已交付

结果见 [报告](docs/sic/CMF解扩融合可行性报告_2026-11-07.md)，code-tested SHA
`57f97d5`。新增 `--rx-fused-payload-cmf`（默认关闭）：前缀保持，payload 用
52 点融合模板 + 四候选相关；6 格×16 源窗功能一致，损坏包回退不误提交；
正式三格 `payload_path` mean 比值 0.573/0.581/0.317，comm mean 0.945/0.955/
0.801，总 SIC mean 0.968/0.992/0.921，P95 同向下降。`stage_payload` 单独
约 1.90× 旧值；收益边界见报告。独立复核发现的尾部差一 chip 已修复。
MATLAB/native/FPGA/硬件/paced/实时目标未验证。

---

# 历史任务：通信解码加速已交付

结果见 `docs/sic/通信解码加速开发报告_2026-11-07.md`，code-tested SHA
`f753f17`。三项显式实验路径：A 同步/SFD（VOLK 点积、模板缓存、尾部 ±8 seed
与回退）、B CFO（1024 点绝对相位块 + VOLK 旋转）、C 按需 soft-chip（PHR 前缀
+ 长度驱动扩展 + 完整旧路径回退）。正式三格配对 comm demod mean：
`sync_base` 2025.9→1656.6 µs（0.818）、`payload_base` 2764.8→1925.5 µs
（0.696）、`payload_four_frac` 2762.3→1757.4 µs（0.636）；P95 均未增加。
8 格×16 源窗逐窗 comm/sensing PSDU、FCS、status 一致，0 次 soft-chip fallback；
补充 16/127 B 两格 comm mean 为 0.576/0.809。默认选项全关，旧 build 未覆盖；
MATLAB/native/FPGA/硬件/paced 与 mean≤0.5 ms/P99≤1 ms/200 窗/s 未验证、未宣称。
以下为历史记录。

# 当前任务：FIR 乘加融合优化已交付

结果见 `docs/sic/DiscreteCIR_FIR乘加融合报告_2026-11-07.md`，code-tested SHA
`ddf335d`。融合 `dst += h*src` 后，12 tap 纯 FIR mean 1.072→0.548 ms
（比值 0.511），3 tap FIR 0.296→0.161 ms，3 tap wrapper 1.199→1.085 ms；
所有 tap 数 FIR/channel/wrapper mean 与 P95 下降。独立 double 卷积最大相对
L2 1.3524e-7（门槛 2e-5），针对性 QA 通过。新 build
`gr-uwb/build-sic-fullpacket-fused`，旧 build 未覆盖；未改估计器、tap 选择、
CFO、FCS/ROI 或默认后端，未上板，不宣称实时目标通过。下方为历史开发记录。

# 当前任务：离散等效 CIR full-packet 后端已交付

执行 `docs/sic/FullPacket_离散CIR估计开发任务书_2026-11-06.md`。本任务优先于下方历史入口。
code-tested SHA `10b660b`，新 build `gr-uwb/build-sic-fullpacket-discrete`。新
`discrete_cir` 用前 8 SYNC 直接拟合整数 lag 离散 FIR，冻结 preamble 路径与原
`path_fractional` 保持；不新增 GNU Radio block，不上板。正式结果见
`docs/sic/DiscreteCIR_开发报告_2026-11-06.md`：10 格×16 源窗功能 PASS，
负样本 PASS；两格串行配对总 SIC mean 为旧路径的 0.308/0.343，P95/P99 同时下降。
当前主瓶颈已转为 comm demod（约 2.0–2.6 ms）。`discrete_cir` 仍是显式实验选项，
不自动改默认；MATLAB/native/FPGA/硬件与 paced 到达未验证。

# 当前任务：SYNC+SFD / PHR / payload 分段实验已交付

结果见 `docs/sic/FullPacket_字段重构计时报告_2026-11-06.md`，code-tested SHA `32473f2`；payload_base热缓存分段为0.874/0.321/1.836ms，公共0.249ms，分段总计比原库低4.5%，不得冒充生产精确分摊。用户授权离线分段实验；生产重构库保持 `1adf401` 的VOLK实现。新增独立诊断exe，按波形字段划分滤波，按最早估计路径锚定输出时间区间划分路径叠加与CFO；保留完整波形halo。公共开销单列，冷/热cache分别观察，原库逐次配对验证输出与计时扰动。只测两格16源窗，每格每变体128次×2轮，独占基准锁；针对性自测通过后绑定提交运行，不扩大历史矩阵。

# 当前任务：Full-packet CFO 的 VOLK 加速已交付

结果见 `docs/sic/FullPacket_VOLK加速报告_2026-11-06.md`，code-tested SHA `1adf401`；CFO阶段约7.6–7.8倍加速，双检查保留，10格功能通过。用户已授权在本实验分支加速乘法。保留1024点绝对相位锚定、双精度累加、转换/试减有限性检查与事务提交；采用本机VOLK分派，不新增GNU Radio block，不修改冻结前导核心。新build `gr-uwb/build-sic-fullpacket-volk`；旧build与结果保留。仅做针对性QA、10格功能对照与两格串行配对计时，不自动上板或扩大历史矩阵。

# 当前任务：CFO/转换/数值检查的隔离诊断已交付

结果见 `docs/sic/FullPacket_CFO拆分诊断报告_2026-11-06.md`，code-tested SHA `4b5da96`。已新增离线scratch重放工具，两处通用复乘为主要开销；生产融合循环与在线检查保持不变，原数据保留。不自动开始下一轮优化。

# 当前任务：payload 前缀缓存对照已交付

本轮结果见 `docs/sic/FullPacket_payload滤波对照报告_2026-11-06.md`，正式代码 `cea0fc3`；不自动开始下一轮优化。用户已授权测试仅对payload及边界区逐包滤波，固定前缀相位行按实际波形缓存；完整包仍逐路径处理和消除。新build `gr-uwb/build-sic-fullpacket-payload-cache` 保留旧build与结果，不上板。快速RX接入与复测已交付：`docs/sic/FullPacket_快速RX复测报告_2026-11-06.md`，正式代码 `28c7734`；不自动进入重构优化。历史报告 `docs/sic/FullPacket_开发报告_2026-11-06.md` 与 `5dd2cfb` 结果保留；新run独立归档。仅调整实验runner的RX路径，不改冻结SIC数学，不上板。

执行 docs/sic/FullPacket_vs_PreambleOnly_开发计划_2026-11-06.md。仅本实验分支允许新增整包路径；原 preamble 行为与历史基线保持。主 agent 与三个 subagent 按文件所有权协作。

# S2 A/B 集成基线入口

当前源码基线同时包含已验收的两个方向：

- A：累加后 sensing CIR。历史报告 `docs/sic/S2A_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2a-accepted-20261018`（`2c383d5`）。
- B：sensing 前导干扰通信 payload。历史报告 `docs/sic/S2B_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2b-accepted-20261018`（`ac3849e`）。

执行与验收记录：

- `docs/sic/S2_AB冻结与合并执行指南_2026-10-18.md`
- `docs/sic/S2_AB冻结清单_2026-10-18.md`
- `docs/sic/S2_AB集成验收报告_2026-10-18.md`
- `docs/sic/S2_AB集成回归入口_2026-10-18.md`

先读根目录 `AGENTS.md`。共享 SIC 数学核心保持冻结。不要只做 A 或只做 B。
单方向历史操作说明留在：

- `docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`
- `docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md`

原 A/B worktree 继续作为历史锚点，不在这里改它们的结果目录。

```bash
pwd
git status --short --branch
git rev-parse HEAD
git worktree list
```

两个 build 都从当前源码根串行构建。核对 `ldd` 指向本 worktree 的对应 `build-sic-*/lib`，不要指向父目录 `build-sic` 或 `/usr/local`。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-accum -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-accum -j2
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-accum/lib/libgnuradio-uwb.so.1.0.0.0
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-payload/lib/libgnuradio-uwb.so.1.0.0.0
```

`UWB_demodulation` 是指向主仓库资料的只读软链接，禁止写入。
大产物写各自 `gr-uwb/build-sic-*/results`，不覆盖历史 run，不推送远程。
