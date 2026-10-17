# S2-A：accumulation 后 CIR 恢复与残余相干性

日期：2026-10-17；状态：待实现。基础提交 93c80b3。
分支 feature/sic-sensing-accum；本任务由一个独立 OpenCode session 执行。
先读 S2_并行开发共同约定.md，再读本文；本文优先于 AGENTS 中旧 S1 的当前任务描述。

## 1. 研究问题

原始 IQ 上约 28 dB 的名义抑制，不直接等于累加后的 sensing 收益。
验证：相同 N 下 SIC 是否降低 CIR 干扰、保护弱目标；随着 N 增加，残余是否不相干衰减，
还是由固定模型误差/CFO 误差形成相干地板。不能先假定能额外获得 10log10(N) dB。
这是评价阶段，不进行新检测器、大规模 Pd/Pfa 扫描或核心优化。

## 2. 复用与新增边界

复用 apps/sic_preamble_sim.py、uwb_sim_preamble_sic.py 的信号生成/现有 GNU Radio
SIC adapter、现有 sensing CIR 算法；先定位当前 CIR 实际用了哪些 repetition、
相关方向/归一化/相位参考/CFO refit，并参考 MATLAB 对照，写入设计。
建议新增（文件名可微调，但最终给出真实入口）：
- gr-uwb/apps/uwb_sim_sic_accum.py：新仿真/评价驱动。
- tools/sic/accum_metrics.py、verify_s2a.py：数学参考、独立 verifier。
- testdata/sic_accum/：配置、短解析 fixture、QA、MATLAB 对照脚本与 README。
- docs/sic/S2A_累加后CIR开发报告.md：最终结果。
不在旧 S1 主程序里继续堆大段分支，不改写 S1 指标/历史报告。
若现有模块入口不足，用只增加接口、不改算法的薄 wrapper；公共修改须单独说明。

## 3. 两层评价，避免同步器掩盖问题

### 3.1 固定算子层（主因果评价）

构造与 sensing code9 匹配的逐符号复 CIR H_x[r,tap]。
同一窗的 clean/raw/SIC 必须使用相同的参考模板、时间原点、tap 网格、固定增益归一化、
相位旋转和符号集合，使 L_r(x) 对输入是线性的。
时间/相位计划取公开本端配置或在线可获得的共同参考，不使用注入真值对齐；
若只能用 clean/真值参考，则单独命名 oracle_aligned_diagnostic，不作为在线结果。
不可对三路分别挑峰对齐、逐符号去相位或最小二乘再缩放以人为清除残余。

使用 average 约定：H_x,N=(1/N) sum_{r in R_N} H_x[r]。
同时记 sum/average convention，原始复 CIR 必须保留；N=1 到 N=64 的幅度参考一致。
三路使用同一 realization：clean=s+w，raw=s+i+w，sic=raw-实际相减量。

在同一固定 tap ROI 计算：
- E_raw,N=H_raw,N-H_clean,N，E_sic,N=H_sic,N-H_clean,N。
- CIR SIC 收益 C_CIR(N)=10log10(||E_raw,N||² / ||E_sic,N||²)。
- 相对 clean 的 CIR NMSE；目标 ROI 和背景 ROI 分开，不能只选最有利 tap。
- 累加收益 G_sic(N)=10log10(P_residual(1)/P_residual(N))，在相同 start 的嵌套窗内比较；
  必须说明这不是 SIC 增益。raw 也画同样曲线。
零分子/分母明确标 N/A、+Inf/-Inf 及测量地板，不通过 eps 或随意 60 dB 截顶造好结果。
原始 IQ 上实际抑制继续保留，不能用 i_hat 的重构 NMSE替代实际输出。

### 3.2 接收机层（工程结果）

现有 sensing CIR pipeline 正常运行，允许其现有同步/CFO行为，但逐路记录结果/失败。
与固定算子层分列；不能把经过不同非线性同步/归一化后的三路之差称为纯干扰残余。
若当前接收机不支持可选 repetition，先由薄离线工具实现固定算子层，
将 receiver_N_sweep 标未实现，不以新 Python 工具冒充 GNU Radio 接收机已经支持。

## 4. repetition 选择与保护指标

全前导 R_N=[0,N)，N=1,2,4,8,16,32,64。
训练后独立区 R_N=[8,8+N)，N=1,2,4,8,16,32,56；禁止把 [8,72) 当成 64 个独立符号。
训练以干扰模型 origin 为准；sensing repetition 网格以本端 sensing 原点为准。
offset 非零时，相关窗若碰到干扰 training/guard 边界，必须报告交集并保守剔除整窗；
记录独立区实际样点/符号数，不靠模型 origin 偷换 sensing 相干累加网格。
N=1 不只挑一个“好符号”：另报告逐 repetition 残余功率及全部有效单符号分布。

弱目标沿用 S1 sensing 路径 delay=37、gain=0.25；ROI [51,60) 仅在同坐标/profile复核后沿用，
由 calibration 冻结，所有 N/三路相同，不能每窗追最大峰。
报告目标复幅度误差、相位误差、局部峰移、背景假峰。
目标 SINR 如不能可靠分离目标/直达泄漏/旁瓣，则称 target-to-background proxy，
写清计算分子/分母，不把全 CIR 总能量当作弱目标功率。
全 Pd/Pfa 曲线为后续，不得用少量固定目标检出来宣称完整检测性能。

## 5. 残余相干性：需有正反例

对逐符号 E_sic[r,tap] 计算：相关系数 vs repetition lag、相位轨迹、均值分量和随机分量；
有限样本去均值会移除相干项，须同时报告 uncentered correlation 与 centered covariance。
多窗 ensemble 与单窗符号内统计分开，不能将固定参数误差“跨窗随机”误认为“窗内独立”。
单窗相干度 eta=||sum_r E_r||²/(N sum_r ||E_r||²)，0能量时 N/A；
eta 对完全同相残余为1、独立零均值 ensemble 期望约1/N，但相位斜坡也会抵消，
故不能只靠 eta 判定随机。需结合 CFO相位斜坡/lag 相关和 ensemble 置信区间。

QA 至少包含：
1. 完美相减：输出复 CIR 等于 clean，零残余不产生除零伪数值。
2. 无 SIC：raw 与 sic 一致，C_CIR=0 dB（非零能量时）。
3. 人工逐符号独立零均值残余：多 seed ensemble 的平均功率随1/N变化，
   用统计容差而非单条曲线精确3dB断言。
4. 每符号相同残余：average 后功率不因N衰减。
5. 固定相位斜坡：非单调积累也能出现，不冒称随机噪声。
6. 配对同噪声抵消/固定线性算子的可加性、sum vs average换算、全区与独立区边界。
7. 后续 IQ 改动不改变冻结模型；错误ROI/错N/错缩放/summary篡改被 verifier拒绝。

## 6. 分阶段实验矩阵

A0：设计与 QA；先追踪实际 CIR 实现/MATLAB，冻结坐标和评价公式。
A1：nominal ISR10/SNR30、offset0/CFO0/single，8窗 smoke，完整数组归档。
A2：正式 acceptance 12 cells，每cell100窗，所有N从同一窗派生：
  ISR={0,10,20} × CFO={0,-50000,+50000}，offset0/single，共9cells；
  另3cells：ISR0/CFO+50000/four、ISR10/CFO0/four、ISR10/CFO0/offset0.4/single。
  沿用已有 four-path profile，不悄悄重定义。SNR固定30。
A3：1000窗无干扰旁路/目标保护检查；必要时加固定错码、无重叠短QA。
A4：独立复算、receiver层对照、图表和报告。若成本过高，先交A1/A2及显式剩余清单，
  不以预期收益补缺测数据。不强制重跑S1全72cell矩阵。

图：IQ/CIR抑制 vs N、raw/SIC残余功率 vs N及1/N参考线、逐符号残余、lag相关、
名义与失败条件CIR叠图、弱目标幅相误差。每cell分别列出，池化不能掩盖困难条件。
报告95%区间；跨N bootstrap按window/seed成组抽样，不将同窗不同N当独立样本。

## 7. 验收与交付

数学/QA、同窗同算子、独立区隔离、无真值泄漏、manifest/verifier通过为实现验收。
实际增益不设“必须额外18dB”的门槛：收益为0或变差也要保留并解释。
沿用目标保护参考门：名义幅度绝对误差P95≤1dB、局部峰移≤1 work sample，
分别给每N通过/失败；低信噪条件不降低门槛掩盖失败。
性能仅本方向新增提取/分析的独立开销；正式耗时须遵守共同锁和互斥要求。
报告区分离线评价成本、SIC在线成本、接收机CIR成本，不宣称新实时吞吐能力。

最终提供 S2A_累加后CIR开发报告.md、真实可复制 README 命令、QA/测试日志、
case manifest、逐窗与汇总、独立 verifier、图表及未实跑项。
至少回答：同N的SIC收益多少；N增加的累加收益多少；残余是否相干；弱目标是否受损；
CFO困难场景累加是否有效；哪些结论只属于固定算子诊断，哪些已经经过实际接收机验证。
