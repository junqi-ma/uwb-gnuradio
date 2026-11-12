# Sensing 与 Comm 统一接收前端和 CIR：开发方案

## 1. 目标和基线

代码基线：**`bd311cd`**。本任务是把 sensing 的 CIR 处理改成与 comm 共用的
“最终 CFO 补偿 → SYNC 相干平均 → 稀疏相关”实现，并复用 comm 已优化的前导
检测/CFO/SFD 前端。两者只通过窗口、坐标和输出用途区分，不再维护两套数学核心。

本文件是下一轮实施任务书；当前提交仅冻结既有实现和实验结果，尚未替换 sensing。

已确认的问题和数字（256 SYNC、127 B PSDU，单 worker，quality Kaiser）：

- 现 sensing 入口直接平均 SIC 输出，没有估计/补偿目标 CFO。SIC 重构只消除
  干扰，不会自动消除目标 CFO。此前目标 CFO=0 的矩阵不能验证非零 CFO 感知。
- sensing 使用标量平均/稠密相关；comm 已有向量平均/稀疏码相关。
- sensing 接收阶段约1.237 ms，分段为SFD搜索696 µs、起点细化13 µs、CIR估计
  519 µs。comm CIR估计约87 µs，但仅38 tap；不能将它直接当成116 tap的预测。
- 两任务公共重采样约1.7 ms、前导SIC约1.73 ms。此次优先改sensing与公共接收
  组件，SIC前8符号模型、重构算法及重采样系数不在本任务的优化范围。

开始时从基线创建独立实验worktree/分支，候选用新构建目录
`gr-uwb/build-sensing-shared-cir`；保留现有基线build/结果，不复制CMakeCache。
本轮不新增GNU Radio block，不安装、不上板、不移动历史tag、不推送。

## 2. 必须保留的输出语义

| 项目 | sensing | comm |
|---|---|---|
| 目标CFO | 估计目标码的CFO并补偿 | 使用现有目标CFO链路 |
| CIR窗口 | 默认pre16/post100，116 tap | 默认pre8/post30，38 tap |
| CIR时间原点 | 发送时序/公开预测原点，保留传播时延 | 接收机检测原点 |
| 原始输出 | 复数相关结果 / code_energy，保留幅度和相位 | 共享原始结果可用，现有解调结果不退化 |
| 归一化 | 另给L2-normalized CIR，不覆盖raw | 保持原归一化和CMF系数约定 |
| 后续计算 | 到CIR结束 | NS-SFD、PHR、payload、FCS照常 |

检测原点、CIR距离原点、相位原点是三件事，API中分别命名、分别保存：

1. `detected_sync_origin`：接收前端找到的SYNC原点，用于检测和频偏估计。
2. `cir_axis_origin`：CIR tap零延迟轴所依据的原点。sensing使用公开发送时序，
   不能用检测峰重新居中，否则真实时延会被抹掉。
3. `phase_reference_sample`：去CFO的明确参考样点，连同window absolute origin
   返回，避免无声改变跨窗复数相位。目标CFO与干扰CFO必须分别处理。

重复周期使用统一的显式配置/估计值P；sensing可在保留轴原点的同时采用测得的
周期来补偿SFO。默认整数网格1016与comm的round规则一致，禁止两路各自取整。

## 3. 共享 CIR 内核

建议新增轻量头文件 `gr-uwb/include/gnuradio/uwb/uwb_sync_cir_core.h`。
该内核不反向依赖完整demod、HRP调制器或SIC，避免include环。

接口应包含以下概念，名称可按仓库风格调整：

```cpp
struct SyncCirRequest {
    int64_t cir_axis_origin;
    double samples_per_symbol;
    size_t total_repetitions, skip_initial, max_repetitions;
    size_t pre, post;
};
// PreparedSyncCode: ternary sparse code, code_energy, immutable fingerprint.
// SyncCirScratch: averaged SYNC, correlation sums, raw/norm CIR, all preallocated.
// CompensatedIqView: data pointer, n, final CFO phase model, valid coverage/epoch.
bool estimate_sync_cir(const CompensatedIqView&, const SyncCirRequest&,
                      const PreparedSyncCode&, SyncCirScratch&, SyncCirResult&);
```

数学合同：对所有合法k，取
`lo_k = cir_axis_origin + round(k*P) - pre`，窗口长
`code_len + pre + post - 1`。对已按同一最终CFO补偿的窗口相干平均，再与采样码
相关。raw CIR按公开code_energy标度，不做逐包峰值/增益拟合。

- 直接复用 `rxopt::vector_accumulate_complex`、`vector_scale_complex`、
  `correlate_cir_sparse` 等现有实现；支持38/116 tap，代码9–12，64/256 SYNC。
- 同时保留相关和、raw复数CIR及独立norm输出。comm沿用原浮点运算和归一化
  顺序，避免提取公共函数时顺手改变数值行为；sensing不得复用已经丢失幅度的
  `cir_filter_taps`或只有实部的`cir_values`作为raw CIR。
- clipped窗口按统一规则丢弃，以实际valid_repetitions为分母；0个合法窗口
  明确失败并清空输出有效长度。所有缓冲在prepare时按最大窗口预留。
- 内核不估计干扰、不读真值，不重复旋转已去CFO的IQ，不隐式修改输入。
- 第一阶段先复用现有去旋转缓存；不同时引入新的旋转/平均融合算法，以便隔离
  公共化的数值和性能影响。

## 4. 共享前导接收前端与 CFO 覆盖范围

从 `uwb_demod_core.h::demodulate_one()` 中抽出前导接收helper，复用现有
SYNC检测、CFO初估、fast SFD、tail重跟踪、CFO复估及final SFD验证。
先在原头文件内部提取，理清依赖后再决定是否移到新头文件，不能复制一份长期分叉。

返回一个不拥有IQ的上下文，包含Timing/CFO/SFD结果、最终频偏与相位参考、
补偿缓存view和**已按最终CFO补偿的区间/代次**。支持三个目的：

- `SyncSfdOnly`：供preamble-only SIC，保留当前快速停止行为。
- `CirOnly`：供sensing，SFD通过后补齐全部CIR所需去旋转窗口，到CIR结束。
- `FullPacket`：供comm，保留当前按需软chip/PHR/payload旋转与融合解码。

**关键缺口：不能直接调用现有 `rx_stop_after_sfd=true` 后就读取整个derotated。**
当前代码在该模式下，最终CFO的第二次旋转只覆盖final SFD范围；较早的SYNC缓存
可能还是bootstrap CFO结果，或没有被更新。必须做到：

1. 由CIR的真实轴原点、pre/post、skip/count、周期P推导全部合法输入窗口的并集。
2. 用最终CFO和统一相位参考补齐该并集，保留halo；116 tap所需范围不能按comm
   的38 tap硬编码。发现覆盖不足时先补偿或失败，不能读取旧epoch缓存。
3. CFO从bootstrap更新为final时使旧coverage失效；旋转始终从raw输入计算，
   不能在旧旋转结果上再乘一次而造成双重补偿。
4. 仅复用频偏/时间参考一致的缓存。同码、同frame、同final CFO可以共用，
   不能把SIC中code10干扰的CFO拿给code9目标。

Sensing调用公共前端后，把公开预测轴原点和补偿后的IQ交给共享CIR。Comm调用
同一前端与共享CIR，继续现有解码。SIC可共用前端，但其8-SYNC模型估计与训练
前缀契约保持；不得为SIC-only调用增加不需要的整段目标CIR补偿和累加。

## 5. 检测与门控

统一使用comm优化的核心，不保留新的标量sensing生产分支。公开粗ROI、搜索
seed、margin、阈值及门控结果均须显式传入/返回。

- 当前radar与comm入口的搜索区间和归一化定义不完全相同，迁移时逐项列清。
  不通过无限扩大搜索范围或降低阈值来让测试通过。
- 对sensing发现的候选检查其是否满足配置的公开ROI和完整前导/SFD支持。
- 有效性不足、非有限输入、CFO失败、补偿覆盖不足、无有效重复均失败关闭，
  不返回seed-only或上次窗口残留CIR。
- 现有`use_predicted_timing`调用要显式适配：配置已有目标CFO时可直接补偿；
  需要估计时调用共享前端，不能把“跳过SFD”默认为“目标CFO为零”。

## 6. 实施顺序与文件

### A：提取CIR公共核，先让comm原行为通过

- 新增共享SYNC-CIR核/prepare/scratch。
- `stage_cir_softchips`改调用公共核，后续软chip/归一化/CMF保持原合同。
- 对38/116 tap的独立float64参考、边界、原始幅度、正常/失败输出补QA。
- comm黄金向量、PSDU/FCS和回退路径通过后再进入B。

### B：提取公共前导前端

- 拆出前导状态与补偿coverage，comm和SIC复用，同一调用不重复检测。
- 默认comm/SIC结果、失败原因和早停行为保持；特别检查SIC-only不执行
  CIR/PHR/payload。
- 为final CFO切换、区间补齐、缺halo及过期缓存增加针对性QA。

### C：接入sensing

- `uwb_radar_cir_core.h`接公共前端和共享CIR；适配RadarCirResult与raw/norm taps。
- `uwb_radar_cir_estimator.h`可保留必要薄适配，旧标量数学仅作测试参考，不再成为
  第二套在线算法。
- `uwb_uhd_sc16_pipeline.cc`及独立benchmark贯通新scratch/元数据；维持burst级
  多worker、每worker私有状态，不新增调度块。
- 更新诊断入口，避免 `benchmark_chain_workers.cc` 的旧手工radar成功路径继续
  绕开新的生产实现。新计时通过公共阶段边界获得，并验证与实际生产输出一致。

### D：验证、配对测量、切换

候选在新build完成全部要求后，用共享实现作为sensing默认入口；原基线commit/
build留作对照。无需额外设计一套永久在线legacy维护路径。若未通过，报告失败项，
保留候选和证据，不把未完成的新路径冒称默认已切换。

主要涉及：共享核新头文件、`uwb_demod_core.h`、radar core/estimator、SIC统一
前端调用点、pipeline scratch、相关QA和现有tools/sic实验/计时驱动。

## 7. 正确性验收：数学一致性与估计误差分开

1. **公共核数值QA**：codes9–12、38/116 tap、64/256 SYNC、skip10、整数/分数
   起点、非整数P、clipped边界、0有效重复、容量不足、非有限输入。以相同输入
   CFO和坐标对照独立float64补偿/累加/相关，非零能量例相对L2≤2e-5，峰tap一致。
2. **CFO覆盖回归**：目标CFO=0/±50 kHz，较大公开起点/不同相位参考；专门让
   bootstrap和final CFO不同，验证首/中/末SYNC都使用final补偿，无旧epoch、
   无double-rotation。变更不依赖注入CFO真值作在线输入。
3. **距离轴QA**：已知整数/分数多径相对于clean基准的延迟必须保留。检测起点
   改变时不能把sensing物理峰重新归零；raw/norm坐标一致。
4. **幅度/相位QA**：单径已知复增益、输入放大/缩小，raw CIR按比例变化；norm
   单位化但不覆盖raw。相位参考变化的输出遵守显式旋转关系。
5. **comm/SIC回归**：原黄金、PSDU/FCS逐例一致，错误/截断/损坏包失败原因和
   回退行为不退化；SIC训练仍限制前8符号，最后4SYNC实际消除保持。
6. **负样本**：噪声、错误码、错误SFD、缺前缀/halo不误报；输出有效长度清零。
7. **端到端矩阵**：64/32与256/127；work/native两网格；单径和分数多径；
   两个目标码方向（9受10干扰、10受9干扰）；目标和干扰CFO独立取0/±50k，
   ISR0/10/20。先每格16例功能回归，不做功效不足的“1000包PER证明等效”。

数学核对照使用**相同估计CFO**可隔离浮点误差；接收端估计CFO对真值的误差另列。
不能把后者也强压到2e-5数值门槛，不能把oracle补偿放进在线路径。

旧sensing在非零CFO下本就缺补偿：它作为历史行为对照，不作为非零CFO正确性
金标准；新结果对独立数学参考、clean参考和物理延迟验证。

感知质量至少报告：raw CIR峰位置/幅度、复数相对L2、PSLR/ISLR、主瓣宽度和
强弱双目标保护。固定距离坐标、固定主瓣排除区，不逐变体重定义窗口掩盖旁瓣。
对于本次CIR改造引入的增量，以参考峰位差≤0.1样点、幅度差≤0.1 dB作为目标；
浮点核采用上面的严格数值QA，接收估计误差和SNR另列。
既有SIC在同步重叠下约0.26–0.43 dB偏差单列，不能因新CIR较快便宣布它已关闭。

## 8. 性能验收

先数学/功能，再性能；所有基准串行持有`/tmp/uwb-sic-benchmark.lock`。
control=bd311cd旧build，candidate=新build。绑定实际库、二进制、源码、CPU
亲和性、输入hash、有效窗数及配置，不用改变负载的失败/旁路耗时作基线。

- 单worker：quality Kaiser、realtime等波纹；64/32、256/127，独立sensing任务
  和comm任务；预热≥32次、每臂512有效观测、5组ABBA，三session。
- 用旧实现与候选均正确的零CFO格衡量提速；非零CFO的旧实现不是质量等效基线。
- 直接报告公共前导接收、CFO旋转覆盖长度、平均/相关、SFD/PHR/payload、整体
  服务均值/P50/P95及回退次数。孩子计时不再加给父亲；原stage字段含义保留，
  新路径细分另加命名清楚的字段。
- 性能目标：116-tap公共CIR估计比旧标量核快≥20%，sensing接收阶段整体快≥10%；
  comm和SIC-only整体不得稳定退化>5%。这是待验证门槛，不是预先承诺。
- 若CFO补偿新增的必要工作使整段目标未达，必须拆开解释内核收益与正确性成本，
  不能静默跳过补偿或门控。输出真实矩阵，不靠单session挑最快数值。
- N1通过后补N4/N8生产pipeline及包含comm的完整DSP回归，逐窗功能一致性、
  状态账本、零丢弃；不直接沿用旧手工诊断路径的计时。

## 9. 交付物与完成标准

- 公共前导/CFO上下文和公共CIR核心，sensing/comm真实共用代码。
- 适配后的pipeline与独立实验入口，现有64/256配置和PSDU/FCS契约通过。
- 数学、CFO覆盖、物理距离轴、幅度/相位、失败路径QA与全量串行CTest记录。
- 新目录归档功能矩阵、质量指标、ABBA原始记录与性能报告；大IQ/build不入Git。
- 报告明确CFO补偿缺项已由哪些非零CFO数据关闭，哪些继承质量问题仍未关闭。
- 为评审者解释最终默认行为、原始复数CIR的标度/相位/距离原点、实际收益和限制。

按A→B→C→D依次推进即可。验收基于公共实现的输出契约和实测数据，不以两套
算法“本质不同”为由保留慢路径，也不把“数学一致”误当作可以忽略缓存覆盖、
距离轴或原始幅度的理由。
