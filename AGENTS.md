# 当前提交快照：SIC 第二阶段全链路算力优化（2026-11-10）

已交付 SIC 第二阶段全链路算力优化（信道估计实数展开/局部能量/CFO 双精度向量化、SFD 细搜 4-lag 批处理与相干累加 SIMD、前导持有型持久缓存与 64 位哈希、ROI 抑制能量 SIMD），开发报告详见 `docs/sic/SIC_第二阶段全链路算力优化开发报告_2026-11-10.md`。
- **Preamble-Only 耗时大幅压降至 630 µs 级别**：核心耗时均值从 797.98 µs 降至 **634.84 µs**（加速比 1.26×，净减 ~163 µs，P50 622.65 µs，P95 744.55 µs），抑制比保持 **63.29 dB**（0.00 dB 差异）。
- **Full-Packet SIC P50 进入亚毫秒**：核心耗时均值从 1,162.30 µs 降至 **1,025.23 µs**（P50 **986.18 µs** 进入亚毫秒，P95 从 1,434.22 µs 降至 **1,151.56 µs**，净降 ~283 µs），抑制比保持 **63.33 dB**（0.00 dB 差异）。
- **信道估计提速 2 倍**：`estimate_channel` 从 213.30 µs 降至 **106.03 µs**（2.01×），P95 从 258.62 µs 降至 132.84 µs。
- **验证与数值安全**：新增 11 项针对性单元测试，12/12 项 CTest 全部 100% 通过（耗时 13.46 s）。

---

# 历史提交快照：前导码与 SFD 互相关向量化优化（2026-11-10）

已交付前导码与 SFD 专用向量化优化（AVX2 4 滞后实数展开点积与 SFD 符号级 Kronecker 分解），绑定提交 `4697980`，测试报告详见 `docs/sic/前导码与SFD互相关优化开发报告_2026-11-10.md`。
- **Preamble-Only SIC 突破 1 ms**：核心耗时均值从 1.15–1.25 ms 降低至 **828.12 µs**（P50 812.35 µs，P95 999.00 µs），抑制比保持 **63.29 dB**。
- **Full-Packet SIC 进一步优化**：核心耗时均值从 1.58 ms 降至 **1,142.42 µs**（P50 1,128.12 µs，P95 1,276.16 µs），抑制比保持 **63.33 dB**。
- **通信前导解调提速**：`stage_timing` 从 ~380 µs 降至 97.7 µs（3.9×），`stage_sfd` 从 ~340 µs 降至 212 µs（1.6×）。
- **验证与数值安全**：实数展开 AVX2/FMA 点积与 VOLK 误差 $< 4.8 \times 10^{-7}$，9/9 项 CTest 全部通过。

---

# SIC 全包与前导优化统一集成基线（2026-11-09）

本分支为 `integration/sic-unified-20261109`，由 `feature/sic-fullpacket-compare`（全包 SIC 与离散 CIR）与 `feature/sic-perf`（Preamble-only 核心优化与 PMT/dcons 优化）合并而成。

- **全包与离散 CIR 基线**：包含 Full-packet SIC、离散等效 CIR 估计（`discrete_cir`）、快速通信接收机（Fast SFD/CFO/Softchip、Fused CMF/PHR）以及 AVX2 脉冲重构内核。
- **前导核心与系统开销优化**：包含 Preamble SIC 核心的列/旋转/self-energy 预计算缓存加速，以及 GNU Radio PDU 块的 PMT 静态符号表缓存和 O(1) dcons 前插优化。
- **统一模型愿景**：Preamble-only 被视为已知字段覆盖仅为 SYNC 的全包 SIC 特例。

---

# 当前提交快照：整包 SIC 稀疏重构与验证精简（2026-11-08）

本次提交汇总当前实验代码与报告。紧凑四径、32 B、20 tap、lag_lo=-1：总 SIC mean 1.822 ms，调制+重构 227.7 µs。相较最初整包版本历史 21.83 ms 约快 12 倍；不同信道/运行的阶段对比，详见 `docs/sic/SIC_首版至当前性能对比_2026-11-08.md`。下方“未提交”为各轮测量时的历史状态。PHR R1–R3 与 SYNC 碰撞退化仍未关闭；实验选项不自动替代默认路径。

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

结果见 `docs/sic/CMF解扩融合可行性报告_2026-11-07.md`，code-tested SHA
`57f97d5`。新增显式 `--rx-fused-payload-cmf`（默认关闭）：前缀 PHR/CIR/相位
不变，payload 用 52 点融合模板 + 四候选相关直接产生 BPM 指标；分数 spc、
RAKE、bypass 和失败路径保留旧流一次。6 格×16 源窗逐窗 comm/sensing
PSDU/FCS/status 一致，0 次正常格回退；损坏包 16/16 融合尝试、16 次回退、
0 误提交。正式三格 `payload_path` mean 98.4→56.4（0.573）、99.1→57.6
（0.581）、653.6→207.5 µs（0.317）；comm mean 0.945/0.955/0.801，总 SIC
mean 0.968/0.992/0.921，P95 同向下降。`stage_payload` 单独约 1.90× 旧值，
收益来自不再扩展 payload soft-chip；独立复核发现的尾部差一 chip 已修复。
MATLAB、native、FPGA/硬件、paced 与实时目标未验证。

# 历史任务：通信解码加速已交付

结果见 `docs/sic/通信解码加速开发报告_2026-11-07.md`，code-tested SHA
`f753f17`。A/B/C 三个显式选项：SFD VOLK 点积 + 模板缓存 + 尾部 ±8 seed、
CFO 1024 点块旋转、按 PHR 长度两阶段 soft-chip。正式三格配对 comm demod mean：
`sync_base` 2025.9→1656.6 µs（0.818）、`payload_base` 2764.8→1925.5 µs
（0.696）、`payload_four_frac` 2762.3→1757.4 µs（0.636）；P95 均未增加。
8 格×16 源窗逐窗 comm/sensing PSDU、FCS、status 一致，0 次 soft-chip fallback；
补充 16/127 B 两格 comm mean 为 0.576/0.809。默认选项全关，旧 build
`gr-uwb/build-sic-fullpacket-fused` 未覆盖；MATLAB、native、FPGA/硬件、
paced 到达与 mean≤0.5 ms/P99≤1 ms/200 窗/s 未验证、未宣称。以下为历史记录。

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

---

# SIC 性能优化（feature/sic-perf，2026-10-19）

本 worktree 从 annotated tag `sic-s2-ab-integrated-20261018`（`794056c`）创建，分支 `feature/sic-perf`。任务文件是 `docs/sic/Grok开发指示_SIC性能优化_2026-10-19.md`。

冻结基线的算法、阈值、种子、指标和信道仍然约束历史对照。只有本性能分支可以按该任务改共享 SIC 核心。不要把本分支自动合并回 `feature/sic-research`，不要移动 tag，不要推送。

只读对照 worktree 是 `.worktrees/sic-perf-baseline`（detached `794056c`）。两边各自构建 `gr-uwb/build-sic-accum` 与 `gr-uwb/build-sic-payload`，不覆盖旧集成 build，不复制 CMakeCache。mean≤0.5 ms、P99≤1 ms、200 窗/s 仍是未达成目标，不是本轮承诺。

---

# 已验收 A/B 集成基线（2026-10-18）

本文件是 S2-A 与 S2-B 合并后的共同研究基线。下方 `---` 之后的 S0/S1/TWR
条文是继承规范，不表示当前任务仍是单方向整改或 S1 实现。新的算法实验从集成
tag 另开 worktree，同时保留 A 与 B 的评价；本基线本身不自动开始下一轮实验，
也不上板。

| 方向 | 历史锚点 | 最终报告 | 评价 |
|---|---|---|---|
| A：累加后 sensing CIR | tag `sic-s2a-accepted-20261018`（`2c383d5`），worktree `.worktrees/sic-sensing-accum` | `docs/sic/S2A_整改开发报告_2026-10-18.md` | 12 格代表回归、旁路、A-N02 基线 |
| B：sensing 前导干扰通信 payload | tag `sic-s2b-accepted-20261018`（`ac3849e`），worktree `.worktrees/sic-payload-collision` | `docs/sic/S2B_整改开发报告_2026-10-18.md` | 11 格 integration smoke，不是正式 32-cell 矩阵 |

统一入口是根目录 `OPENCODE_START.md`。冻结与合并过程见
`docs/sic/S2_AB冻结与合并执行指南_2026-10-18.md`、
`docs/sic/S2_AB冻结清单_2026-10-18.md` 和
`docs/sic/S2_AB集成验收报告_2026-10-18.md`。
回归命令见 `docs/sic/S2_AB集成回归入口_2026-10-18.md`。
各方向原来的 `OPENCODE_START` 内容已并入统一入口；单方向开发指示保留为历史文件。
2026-10-18 的集成回归绑定 code-tested SHA `2cea67b`。验收记录在上面的报告里。
annotated tag `sic-s2-ab-integrated-20261018` 指向包含该报告的提交。

必须同时保留的结论边界：

- 共享 SIC 核心、adapter、`tools/sic/sic_reference.py` 和原 core/adapter QA 门槛保持冻结。在线路径只接收混合 IQ、公开模板和公开粗 ROI。注入时延、CFO、路径和 IQ 真值只进 sidecar 与离线 verifier。
- A 的名义 CIR 抑制约 28.4 dB，残余高度相干。困难 CFO 的约 12.61 dB 改善是离线有界诊断，不是新的在线结果，也不把 CFO=0 说成已经局部最优。
- B 的 ISR=20 dB 恢复只适用于已测单径、零 CFO 条件。B-N01 之后，库信任根来自 verifier 所在源码根或显式 `--repo`/`--trusted-build`，不能由 manifest 自报的 `build_dir` 给自己背书。
- MATLAB、native、FPGA/硬件和正式实时吞吐都未作为本基线的通过项。mean≤0.5 ms、P99≤1 ms、200 窗/s 仍是目标，不是已实现事实。功能回归通过不等于性能通过。
- 正式性能数字必须独占 `/tmp/uwb-sic-benchmark.lock`。大 IQ、build 和冻结包不进入 Git。历史 run 不被新 run 覆盖。

两个 build 都从本基线源码串行配置，不复制旧 CMakeCache，不软链旧 build，不安装到 `/usr/local`：

- A：`gr-uwb/build-sic-accum`
- B：`gr-uwb/build-sic-payload`

本地 `UWB_demodulation` 仍是指向主仓库 MATLAB 资料的只读软链接，禁止写入。

---

# UWB GNU Radio Project

## 当前分支：SIC 论文研究（优先执行，2026-10-16）

当前分支 `feature/sic-research`，从 `feature/uwb-ds-twr` 的 `d0c8138` 创建。
当前用户任务是 **S1 八符号估计与全前导消除**，不是继续 TWR 里程碑。
先读根目录 **UWB_Preamble_SIC_GNU_Radio_Design.md**，再读
**docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md** 与
**docs/sic/S1_八符号模型API.md**；前者为 OpenCode 可执行任务与验收依据。
S0 的指示/合同/整改报告保留为历史基线，S1 语义优先于旧 fit/validation 耦合。

### S0 任务与约束

- 复用现有 sensing RX 仿真，在时间上重叠的 SYNC 区注入不同 preamble code 的
  comm 前导：默认 sensing code 9、comm code 10、64 SYNC、998.4 MS/s。
  “重合”是时间重叠，不是同码；同码只在显式困难诊断用例中启用。
- 先做 work-grid CF32，再接现有 CG400 native 491.52 → work 998.4 PDU 65/32。
  保留完整 sensing RX 给解调器，不能把只覆盖 preamble 的 CIR 窗拿去完整包解调。
- comm 是 **preamble-only**；SIC 不要求解码 comm PHR/payload/FCS，不复用旧
  full-packet SIC 的 FCS 门槛。旧 `feature/sic-validation` 保留，不整体合并。
- clean/raw/sic 按同一窗口配对，报告 sensing CIR/弱目标保护及完整 sensing 包
  解调状态。在线 SIC 只接收公开模板/粗 ROI，不得读取注入时延/CFO/路径/IQ 真值；
  oracle 只作单列诊断。相减总功率下降不等于消除了干扰。
- 新增 SIC 数学核心与 GNU Radio adapter 分离；先 Python 数学参考与 C++ QA，
  再薄 PDU block、benchmark。执行范围和顺序按 S0 指示，不自动上板。
- 完整解调、radar CIR-only、SIC 前端分开计时。当前 `stage_sfd_us` 还包含
  tail timing/CFO refit，`stage_cir_us` 含 soft-chip；补诊断子计时，保留旧字段语义，
  父子计时不二次累加。失败路径、PMT/队列与生成器开销必须单列。
- 默认单 worker，scratch 预分配；同时限制 worker 队列与 PMT 入口 in-flight。
  回放源延迟提交也算 missed admission，不能靠降低实际输入率掩盖过载。
- 原有 MATLAB canonical fixture 不覆盖；新信号/脚本存 `testdata/sic_preamble/`。
  MATLAB 未运行明确写未验证。既有 resampler SIMD 非确定性只记录不修，保留
  scalar/reference 对照，不把其风险混同为 SIC 算法失败。
- 所有性能数字重新绑定 commit/profile/实际加载库/窗口长度。SIC mean≤0.5 ms、
  P99≤1 ms 和 200 窗/s 是验证目标，不能预先写为实现事实；不宣称连续 1 GS/s 已通过。
- 新 block 前仍须查本地 GNU Radio 类似实现、说明 scheduler 语义、先补针对性 QA，
  每次有意义修改后构建/测试。吞吐 benchmark 串行、同环境配对。

S0 已实现并完成一轮独立整改（R01–R10）。开发结果见
**docs/sic/S0_前导碰撞仿真与耗时开发报告.md**（含 superseded 说明）与
复核依据 **docs/sic/S0_整改意见报告_2026-10-16.md**、整改结果
**docs/sic/S0_整改开发报告_R01-R10.md**、测量合同
**docs/sic/S0_整改_测量合同.md**。效果以“真实输出”指标（`actual_suppression_db`）
为准，不是重构 NMSE；工作/采样网格与效果/算力/CIR-only/full demod 分列，通过项、
失败项、未实跑项（MATLAB 未实跑等）均列明。关键结论：主矩阵 68/72 cell 通过、
负样本误提交 0/3000（重叠门控修复后）；SIC 核心实测 ~5 ms/窗（真实负载），
未达 mean≤0.5 ms/200 窗/s 相关门槛（core/GR 达标旧口径已 superseded）。

### S1 八符号估计与全前导消除（当前默认路径）

依据 **docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md**，
契约 **docs/sic/S1_八符号模型API.md**。默认路径 `estimate8_frozen`：

- 严格只读前 8 个 SYNC 的 `observed_end_sample` 前缀完成检测/CFO/CIR/分数时延/
  增益估计，冻结为 `SicModel`（`model_id`/fingerprint）；`apply_model` 不再重拟合。
- 支持由 **configured nominal prediction**（`origin + sync_reps*sps + post_tail`）
  预测，不再被 `confirmed*sps` 截断；相减覆盖完整前导，含独立评估区与末 4 SYNC。
- 默认 `online_validation_reps=0`（不等 holdout 才相减）；可选显式保守对照。
  旧 `--sic-cover-preamble` 仅为 `confirmed_legacy` 别名，不得当作 S1 结果。
- 四区分离：acquisition/search、training `[0,8)`、cancellation（全支持）、
  evaluation（离线 `[8,64)`，只评估不反馈）。效果以真实输出残余为准。
- 验证：core QA 28 例（前缀隔离含 guard page、全支持、末 4 SYNC 实减、
  chunk 逐位一致、外推失配、旁路、R05 数值安全）；verifier 增加 S1 检查。
- 关键风险（如实保留）：8 符号 CFO 精度限制远端深度；CFO 非零 cell 的
  tail 抑制明显低于名义 CFO=0 的 20–25 dB 目标。性能门槛仍未达。

S1 阶段结论以 **docs/sic/S1_八符号估计与全前导消除开发报告.md** 为准；
不能只引用本轮阶段性数字。

## 继承基线：UWB SS-TWR / DS-TWR

`feature/uwb-ds-twr` 从 radar 分支 `b897677` 创建（首个提交 `38fbe2c` 为纯文档），
同时开发 SS-TWR 与 DS-TWR。
主要需求是 **docs/twr/需求_UWB_SS_DS_TWR.md**，开发顺序和验收见
**docs/twr/开发路线与验收矩阵.md**，芯片资料和 API 映射见
**docs/twr/参考资料与API映射.md**。先读这三份文档和 **开发状态.md**。

**M0.1 第一轮已关闭 R1–R8，第二轮已关闭 N01–N06，第三轮已关闭 N07（枚举域
fail-open）；M0.1 契约层完成，TWR 协议能力仍未实现**（2026-09-29）。先读
**docs/twr/评审意见_M0.1_2026-09-29.md**（N01–N06）、
**docs/twr/M0.1第二轮整改报告_N01-N06.md**、以及
**docs/twr/M0.1_N07修复报告_2026-09-29.md**（N07：审核报 1 处，实际 20 处）。
当前进行 **M2-A native PHY 闭环**（未完成：A10 损伤压力、A13 分配/延迟、A11 MATLAB 按用户
指示忽略）。见 **docs/twr/M2-A_G0接口与数字坐标.md**、**docs/twr/M2-A_评审报告.md**、
**docs/twr/M2-A_native_PHY开发报告.md**。M2-A 只证 native 往返与数字坐标，**不证**首径
ToA/硬件测距；`allows(NativeRoundtripVerified, Ranging)` 仍为 false。**不自动进入 M2-B**。
另：重采样核 SIMD 路径存在已复现的未初始化读取（输出非逐位可复现），按任务单只记录不修。

此前已完成 **M1-A 独立 ToF 数学 + MATLAB oracle**，以及
**M1-B 离线协议核心与双端仿真**（2026-09-30 交付，经第二轮独立复核发现 R01–R09，
已全部整改关闭，见
**docs/twr/M1-B_复核整改报告_2026-09-30.md** 与
**docs/twr/M1-B_复核整改审核意见_2026-09-30.md**）。报告见
**docs/twr/M1-B_协议核心开发报告.md**（其"完成"结论仅在整改后成立）。
下一阶段是 M2-A native PHY 闭环，再是 M2-B RMARKER/首径/时映射/timestamp patch，
随后 M3/M4 上板；**不自动进入 M2**。
**M1-B（复核整改后）**：纯 C++ SS/DS 两端协议核心、wire claim 边界、确定性 fake link、
B01–B18 QA、离线 demo/独立 verifier、独立安装消费者。复核整改关闭了：计划可携带
Completed 提前成功、DS initiator 遗漏 Poll outcome、非估计端绕过本端准入、
reset 清身份屏障、responder 未预留终态容量、空域外 outcome 生成无主终态、
evidence 覆盖绝对 deadline、计划无数值映射、TwrConfig 校验后丢弃，以及复审的
计划 source 未约束与 `exchange_timeout==0` 无绝对上限。
**它不是硬件测距**：结果是带 simulation/wire-claim 标注的协议估计，
`measurement_valid=false`，ToF 公式未对硬件/native/首径验证。
M2-A native PHY / M2-B RMARKER·首径·时映射·timestamp patch / 两 RX 路由 /
controller / UHD / pybind 大迁移均**不在** M1-B 范围；不自动进入 M2。

第二/三轮的关键教训（不要再犯）：

- **把错误行为写进测试约定**：N02 的 QA 曾显式断言"`scheduled_calibrated` TX 必须被拒"
  并因此把 REQ-TIME-03 明确允许的路径堵死；N01 的语料 note 描述了代码里**不存在**的机制。
  语料一致只能证明指定输入上的一致性，**不能证明这些共同约定正确**。
- **枚举的"成员"不是它的"域"**：一个枚举变量可以持有枚举没有的值。所有判定必须
  **显式做域检查并 fail-closed**，且**先于**依赖该值的检查（否则报的是无关字段）。
  `switch` 用**无 `default`** 形式以保留 `-Wswitch` 保护，域测试放在 switch 之前。
- **验证要用穷举而不是抽样**：N07 的 12 处 `TimedField.marker` 是抽样会漏、按类型注解
  穷举才抓到的；可选字段（默认 `None`）尤其容易漏。`validate()` **必须报告，不能抛异常**。
- 逐包时序由 C++/硬件定时驱动；scheduled TX 必须带 `TxSendEvidence`
  （command time / 量化时刻 / marker offset / 校准空口时间 + `outcome == Completed`），
  **`send()` 成功或 BURST_ACK 单独不构成证据**，且入门的值**绝不改标**
  `hardware_measured`。
- **16 SYNC 和 64 SYNC 一样没有 ToA 证据**，"窗更长"不是证据；
  `allows(work_decode, Ranging) = 0` 对全部 48 行成立。

M0 初版设计、回归和调度说明分别见 **docs/twr/M0_设计契约.md**、
**docs/twr/M0_回归基线.md**、**docs/twr/M0_调度语义.md**。
其中与复核报告冲突的结论不再作为冻结约束；不能用“已冻结”阻止修复已复现缺陷。

第一阶段：一台 X410 两个 channel 各作为独立逻辑端点，实现真实空口/有线的
SS/DS 测距及角色互换。第二阶段：X410 与指定 DW1000、DW3000 模组，分别完成
两种协议和两端角色互通。现有 radar、TX/RX、解调和 MATLAB 代码是复用与回归基线。

### 当前事实与必须关闭的缺口

- frame v1 为 14 B 头 + 40-bit LE timestamps；Poll/Response/Final 空口长度为
  16/26/31 B，FCS 由 HRP 层追加。它是阶段一内部协议，商用适配需另验具体固件。
  Report 保留且未实现；STS 本阶段不开发。
- 配置/codec 的 14 B 几何、session 收窄、独立 PHR 速率约束已整改；现有调制器
  是 0.85 Mb/s PHR + 6.81 Mb/s payload。frame 的运行 seq 是 uint16，配置初值
  `session.sequence` 为 uint8；M1-B 必须显式管理运行身份和旧帧复用屏障。
- Python effective/snapshot 的修改隔离和 capability 公开解冻漏洞已整改，保留回归。
- 时间数学的精确投影、半周期和分数边界已整改；ToF 不使用有损整数 ns 投影，
  本端输入显式校验 range-capable marker、校准和质量，不能只依赖 interval 返回 Ok。
- 48 行白名单证明的是 998.4 MS/s work-grid 解码自洽，未证明 native/ToA/硬件互通。
  16 SYNC 在 ToA 验证前不得用于测距。128 等暂因当前链路限制拒绝，不能写成芯片
  不支持；能力按 work/native/ToA/hardware/vendor 证据分别升级。
- 仓库已有 Python work→native resample_poly；缺口是反应式 TWR 所需的 C++ 集成、
  buffer 复用与时间映射。两路 RX、分数首径和 delayed-frame timestamp patch 仍待实现。
- controller 可采用零流端口 gr::block + 短 handler + 有界队列 + 单 owner；
  还必须限制 PMT 入口积压，其他 adapter 按实际 scheduler 语义选型。
- X4xx ctrlport `EXEC_LATE_CMDS` 与 timed TX 数据是不同路径；TX 需处理设备
  `ERR_TX_LATE_DATA` 和主机 deadline。具体 FPGA/UHD 行为需绑定版本上板验证。

### 验证环境

检查测试二进制实际加载的库。本机默认 LD_LIBRARY_PATH 可能命中旧 /usr/local 库，
CTest 脚本也会继承它；本地验证使用 `env -u LD_LIBRARY_PATH ctest ...` 并用 ldd 确认
build/lib。手工测试在环境正确时有效；不把 sudo install 作为普通测试前置条件。
吞吐 benchmark 串行、同环境对照，失败单列；CSV 输出到临时/build 目录后显式归档。

### TWR 开发必须遵守

1. 将协议 FSM、frame codec、时间戳/ToF 数学核心与 GNU Radio/UHD 适配分开。
   Python 做配置和结果消费，逐包时序由 C++ 及硬件定时驱动。
2. 复用现有 HRP TX、detector/extractor、resampler、demod 和 burst backend。
   现有双 TX/单 RX radar 路径不等于双端 TWR；需显式设计两个 RX 路由和共享设备资源。
3. 端点只使用本端硬件事件及对端空口帧携带的信息。不得用共享 schedule、私有时间戳
   或已知距离计算待测 ToF；真实有效 RX 帧才能触发 Response/Final。
4. 每个时间字段明确 clock domain、epoch、tick rate、marker、量化和校准。
   区分 RX 第一 IQ 样点、RMARKER、首径、预期时间和 TX command time。
   禁止用 radar `predicted_sfd`、最大 CIR tap 或主机收包时间直接当作实际 ToA。
5. delayed TX deadline 不满足时显式失败，不即时补发沿用旧 timestamp。
   回复延迟预算覆盖帧尾、解码、构帧、传输和设备提前量；不得承诺未经验证的短时序。
6. SS clock ratio 的方向/符号/换算及 DS 不等回复时间公式均须独立测试。
   单台 X410 共钟不替代独立时钟、回绕、CFO/SFO 和延迟校准验证。
7. 参数必须经统一 capability/profile 校验并记录 readback。未知或不支持的模式拒绝，
   不静默采用 QM35/radar 默认值；芯片 power word、TX gain、IQ amplitude、dBm 分开。
8. 商用互通绑定具体模组、SDK/固件 hash、帧格式、时间戳单位和 PHY profile。
   优先共同 channel 5 无 STS 配置；每种新增能力单独验证。原版与改时序固件分开报告。
9. 每个 exchange 有唯一 terminal result，保留 timeout、FCS 错、late、underflow、
   overflow、队列丢弃和取消。不得通过跳号补采或重试掩盖失败测量。
10. `testdata/twr/` 保存短信号/帧/time golden 和独立 MATLAB 验证脚本。
    实现前新增针对性 QA，改动后构建/测试。未完成的模组测试、精度标定与 soak
    明确写为待验证；禁止宣称厘米级精度或“完整互通”而没有相应证据。

## 开发规范与参考

### 主要开发需求参考
本分支优先参考上述 TWR 需求；同目录 **开发需求参考.md** 保留早期检测截取需求。
其中“只截取、不做实时解调”“暂不连接 USRP”等历史范围不限制当前 TWR 开发。

### 当前开发状态
请参考同目录下的 **开发状态.md**（已完成的模块、算法参数、测试结果、构建运行方法与下一步计划）。

### 周期旁路雷达截取（Phase-1 已完成）
请参考 **docs/phase1/开发总结_QM35825周期旁路雷达截取.md** 与
**docs/phase1/GROK_X410_QM35825周期旁路开发方案.md**。
已知 `t0/T` 时使用 `UwbScheduledExtractor`，不要用能量门对每个通信包建 Region。

### QM35 盲捕获与自动周期锁定（离线最小闭环已通）
未知 `t0`、已知 T=5 ms 时使用 `UwbAutoScheduledExtractorSc16`，不要并联
Detector + ScheduledExtractor。方案见
**docs/phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md**；
mixed 737.28 离线实测见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md**；
无干扰 QM35 同一流程见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_clean737p28.md**。
硬件 soak / overflow 重捕获未完成，不得写成 X410 盲捕获验收通过。

### UWB 算法与验证
- UWB 检测算法（能量门限、粗检测、细相关、preamble 匹配等）**必须**参考 `UWB_demodulation/` 目录下的 MATLAB 实现（包括 `buildUwbReference.m`、`decode_uwb.m`、`analyze_*` 等文件）。
- 算法正确性验证优先：使用 `testdata/` 下的已知 UWB 测试信号（`.cfile`、`*.dat`、`*_metadata.mat`、`UWB_test_signal_description.md`）进行 MATLAB 对照。
- 检测结果（packet start、IQ 截取范围）必须与 MATLAB 实现逐样本/逐 packet 对比。

### 开发规则（必须严格遵守）

Before implementing any GNU Radio block:

1. Search the local GNU Radio source tree for similar blocks.
2. Determine the correct block type:
   - sync_block
   - block
   - tagged_stream_block
   - message/PDU
3. Explain scheduler semantics before implementation.
4. Avoid allocating memory inside work/general_work.
5. Add QA tests before declaring implementation complete.
6. Run build and tests after each meaningful modification.

### Performance goal

Input data may eventually approach 1 GS/s.

Do NOT assume a naive full-rate correlation implementation is acceptable.

Preferred detection pipeline:

energy / coarse detection
→ candidate region
→ preamble verification
→ packet extraction

### 验证要求

- Synthetic UWB signals should be generated for unit testing (use `testdata/`).
- Detection results **must** eventually be cross-checked against MATLAB implementation in `UWB_demodulation/`.
- Add MATLAB scripts in `testdata/` or root for automated verification.

### 其他

- All test data is located in `testdata/`.
- Full MATLAB demodulation pipeline is in `UWB_demodulation/`.

## Goal

Current branch: validate detection-triggered preamble-only SIC in GNU Radio
simulation, with different sensing/communication codes overlapping in time.
Measure sensing CIR recovery, target preservation, full sensing demodulation
and SIC costs separately. Follow the S0 instruction above; do not advance
to hardware or TWR milestones automatically.

### Inherited TWR goal

Build configurable SS-TWR and DS-TWR on the existing GNU Radio C++ UWB TX/RX
chain, with Python APIs, timestamp calibration, explicit error handling and
MATLAB cross-checks. First validate two logical endpoints on one X410, then
validate both roles against specified DW1000 and DW3000 modules. Preserve
the existing packet capture, demodulation and radar regression baselines.
