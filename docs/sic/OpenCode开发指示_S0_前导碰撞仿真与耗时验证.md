# S0：GNU Radio 前导碰撞仿真、最小 SIC 与耗时验证

日期：2026-10-16。执行分支：`feature/sic-research`，基础提交 `d0c8138`。
状态：**开发指示，尚未实现、尚无本任务性能实测结果**。执行对象：OpenCode。

## 1. 用户目标与交付边界

利用现有 sensing RX 仿真，叠加与 sensing 前导在时间上重叠、但使用不同
preamble code 的 comm preamble；实现 GNU Radio 离线仿真和最小 preamble-only
SIC，分析现有解调器与新增 SIC 各阶段耗时，判断 host 实现的可行范围。

这里的“前导重合”指时间重叠，**不是使用相同 code**。默认 sensing code 9、
comm code 10；主配置均为 64 SYNC、998.4 MS/s，1016 点为一个 SYNC 符号。

本轮包含：信号/真值生成、三路配对比较、纯 C++ SIC 核心、薄 PDU adapter、
解调计时、MATLAB 对照、定速回放、自动核验和开发结果报告。
不包含：UHD 上板、连续 1 GS/s SIC、完整 comm 包重构、多干扰源消除、
通信接收端 payload 碰撞的另一套拓扑、TWR 后续里程碑、旧 SIC 分支整体合并。
SFO、前端非线性和脉冲失配先做有限压力诊断，不要求本轮实现补偿器。

首要判据是 sensing CIR 恢复及目标保护；完整 sensing 包解调/FCS 是辅助结果和
算力对照。**comm preamble-only 没有 PHR/payload/FCS，不对它运行完整包解码，
不以其解码失败拒绝消除。**

## 2. 已核对的仓库基础

先读根目录 `UWB_Preamble_SIC_GNU_Radio_Design.md`、`AGENTS.md` 和本文。
本轮直接参考如下实现，不复制旧 SIC worktree 的整份文件：

| 文件 | 可复用内容 / 注意事项 |
|---|---|
| `gr-uwb/apps/uwb_sim_cg400_echo_cir.py` | Python `SimTimedEcho`、多径/PCHIP/AWGN、完整 RX 与 CIR 截窗、原生率链；其 publisher 的 `.tolist()`/PMT 构造须单列，不能作为 C++ 核心耗时 |
| `gr-uwb/include/gnuradio/uwb/uwb_loopback_echo.h`、`lib/uwb_loopback_echo.cc` | C++ PDU 软件信道、预分配 scratch、整数/分数延迟；支持 native/work grid |
| `testdata/uwb_radar/metadata.json` 与 `export_uwb_radar_golden.m` | canonical MATLAB code-9/64-SYNC/4z2、20 数据字节+FCS、完整包/延迟 RX/CIR；保持文件不变 |
| `testdata/uwb_radar_synthetic/README.md` | splice fixture 仅作 smoke，不能冒称 canonical |
| `UWB_demodulation/+uwbdecoder/buildUwbReference.m`、`estimateCir.m`、`decode_uwb.m` | 成形模板与 sampled code 的区别；CIR /code_energy 后仍有 L2 归一化 |
| `gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h`、`uwb_phy_profile.h` | code 9–12、SYNC/完整包生成和脉冲成形；两路 code/template 必须独立配置 |
| `uwb_radar_cir_core.h`、`uwb_radar_cir_estimator.h` | radar CIR、raw/norm taps、纯 C++ 与无堆增长的 scratch 模式 |
| `uwb_demod_core.h`、`uwb_demod_result.h`、`lib/uwb_realtime_demodulator.cc` | `demodulate_one`、已有七阶段及 queue/worker 计时 |
| `gr-uwb/apps/benchmark_radar_pipeline.cc`、`benchmark_detector.cc` | 现有 core/PDU 分层测量方法；不能假定其内置 fixture 等于新输入 |

重要事实：

1. 现有 CG400 sim 的 CIR 输入是截短 echo window；完整解调必须保留 fullwin，
   覆盖 sensing 的 SFD/PHR/payload/FCS 及多径/滤波拖尾。
2. canonical sensing 包为 4z2；`realtime_demod_golden` 与 benchmark 中另有 IEEE
   SFD 及适配代码。G0 必须按实际文件匹配 profile，不能混用。
3. `benchmark_detector ... demod-stage-profile` 读取固定 golden，不能给它一个新
   文件名就声称测到了该文件；该模式还可能在缺 fixture 时输出 skeleton 并返回 0。
   新 verifier 必须检查实际输入 hash、样点数、完成数和 FCS，不仅检查 exit code。
4. `stage_sfd_us` 当前还含 SFD bootstrap 后的尾部 timing 重跟踪、CFO 重估计和
   final SFD；`stage_cir_us` 含 soft-chip 生成，不等于 sensing CIR-only。
5. 历史 17.2 ms 解调、约 0.38 ms radar estimator、575 PDU/s 等数字属于不同
   commit/profile/窗口，不能拼接为新链路预算；本任务必须重新测量。

## 3. 信号与数据契约

### 3.1 生成顺序与归一化

定义 `y[n] = s[n] + i[n] + w[n]`。`s` 复用原 sensing 完整包/回波模型；`i`
是独立 code 的有限长 SYNC 前导经过独立多径、时移及 CFO 的贡献。

主流程：生成成形波形 → 独立信道/时移/CFO → 按指定 ISR 缩放 i → 相加及 AWGN。
保留 `s`、`i`、`w` 及其和，仅 verifier 能读取分量真值。

- comm 仅生成 64 个 SYNC 的码脉冲并经过成形滤波，保留起止瞬态和滤波尾部；
  不随意截取完整 comm 包来引入未知 PHR/payload，也不简单平铺已被截尾的首个
  1016 点波形而忽略前导边缘。模板能量、pulse taps、长度和 hash 必须记录。
- 主配置 SNR=30 dB、ISR=10 dB；SNR/ISR 均以接收端指定重叠 SYNC 区间上的
  clean sensing 能量为基准。记录区间 `[lo, hi)`、缩放前后功率及实际 ISR。
  无重叠/无信号时 ISR 为不可用，不能除零或伪造数值。
- complex AWGN 的总功率为 I/Q 两分量功率之和，明确 noise_std 是每分量标准差。
- work CF32 保留线性幅度。native SC16 路径采用所有分量共享的一次缩放，记录
  `iq_scale`、clip 数和峰值；主验收要求 clip=0。禁止分别归一化 s、i 或每条分支。
- 仿真端可复用 PCHIP；SIC 分数时延估计不得读取注入 delay，并至少增加一种
  独立 bandlimited delay 生成的交叉模型用例，避免同一插值误差自洽。

### 3.2 双采样率路径

先实现 A，再实现 B，输出分别判断：

**A / work direct**：从现有 code-9 canonical RX 或同 profile 仿真 RX 出发，
在 998.4 MS/s 注入 code-10 前导。用于算法和核心算力结论，不计重采样。

**B / CG400 native roundtrip**：复用现有 491.52 MS/s sim 的 sensing native RX；
comm 从 work grid 下采样 32/65 后在同一 native grid 注入，再经现有 PDU 65/32。
对照分支经过同样重采样与坐标映射。滤波群时延/边界、SC16 量化需单列。
本轮不必再扩展 737.28 MS/s；不得把 A 的结论直接外推 B。

SC16 量化不是线性算子，不能分别量化 s/i/w 后再假定它们之和等于量化 y。
native 验证器保留同一公共缩放/线性滤波下的 s_work、i_work，并将
`y_work-s_work-i_work` 作为含 AWGN、量化/数值误差的有效噪声，单列误差地板。
计算重构 NMSE 时比较 i_work 与实际 i_hat；另给 y_clean 对 clean 对照的误差，
不能把量化噪声算作干扰增益或把其不相消当作 SIC 实现错误。

已有 resampler SIMD 未初始化读取/非确定性按原任务只记录不修。
B 要对重复运行进行一致性检查，并保留 scalar/reference 路径对照（优先使用现有
选择接口；若不可选，离线参考重采样作为单独路径，不冒称生产链已核验）。
一致性无法解释时，B 标记为受已知问题影响；仍完成 A、解调计时及其他独立工作。

### 3.3 运行时信息与真值隔离

PDU 为 `(metadata, c32vector)`，主链只带：

`schema_version, run_id, window_id, epoch_id, sample_rate, sample_count,
window_start_sample, predicted_sensing_start, sensing_profile_id,
sensing_code, sensing_sync_reps, sensing_sfd_mode, template_id`。

native 路径另外保留 native/work 坐标映射、滤波时延和样点有效区间。
接入现有块时显式适配其 `code_index/sync_repetitions/sfd_mode/pulse_id` 等字段，
这些字段始终描述 sensing；comm profile 放独立配置命名空间，不能覆盖它们。

SIC 可知 comm code、名义前导长度、模板、采样率和候选搜索 ROI；
不能读取精确 comm start/CFO/路径/ISR/干扰 IQ 或仿真注入事件。
真值写在独立 fixture/sidecar，由生成器和 verifier 访问，SIC API 不接收该对象。
`oracle` 模式只作上限诊断，输出标注 `uses_truth=true`，不能用于效果或实时验收。

所有索引为 0-based，区间为 `[lo, hi)`；分数时延明确单位与插值约定。
允许 sensing 使用本端计划给出的粗起点；这不是 comm 实际到达时间证据。

## 4. GNU Radio 流图与调度

```text
现有 sensing 完整 RX 仿真 ──> s ───┐
独立 code-10 SYNC → 独立信道 ──> i ├→ y=s+i+w（冻结/回放）
固定 seed 噪声 ───────────────> w ┘

配对实验（同一 window_id；性能测试依次运行，不同时争抢 CPU）：
  clean   s+w             → sensing CIR / sensing 完整包 demod
  raw     y               → sensing CIR / sensing 完整包 demod
  sic     y → PreambleSic  → sensing CIR / sensing 完整包 demod
```

native 模式先产生每条分支对应的 native 窗，统一通过相同 resampler；SIC 放在 work
grid。clean/raw/sic 输入完整长度一致，CIR 所需 ROI 是完整窗口的视图或显式裁剪，
解调仍保留完整帧。输出相减 mask 之外逐样本不变，包括未落入 mask 的 sensing
payload；若滤波/多径拖尾实际延伸入其开头，仅消除可确认的已知前导贡献。

新增块前重新确认本机 GNU Radio 源码，记录实际版本、参考文件和调度选择：

- `/home/junqima/workspace/gnuradio/gr-pdu/lib/tagged_stream_to_pdu_impl.cc`；
- `gnuradio-runtime/lib/tpb_thread_body.cc`、`basic_block.cc`；
- 项目 `uwb_radar_cir_estimator_block.cc` 和 `uwb_realtime_demodulator.cc`。

本轮选择零流端口 `gr::block` / message-PDU，不是 `sync_block` 或
`tagged_stream_block`：完整有界窗口已由上游提供，不需要 forecast/general_work。
消息 handler 只校验、持有不可变 IQ 引用、入队，单 worker 调用纯 C++ 核心。
初始化时分配 scratch/候选路径/矩阵/输出池，核心不增长 vector、不做文件 I/O。
PMT 包装或结果生成若分配，明确计入 adapter 成本，不能笼统宣称全链零分配。

拟定端口：输入 `in`，输出 `out`（每个合法窗口恰好一份 cleaned 或 bypass IQ）、
`status`（检测/消除状态和计时）。没有可信干扰、拟合失败、deadline 不足时旁路。
队列满时也须记唯一终态；优先让既有原始 IQ 按序旁路，不静默丢窗。
无效输入报错且不崩溃。stop/drain 有明确完成计数，无残留线程。

GNU Radio PMT 入口可能在 worker 队列之前积压。测试源需采用 credit/in-flight
上限，覆盖已发布但尚未收到终态的所有窗口；队列同时限制条数和总字节。
建议初始 in-flight=8、总 IQ 预算 64 MiB，实际最大窗口启动时校验。
容量不足导致源无法按期提交时，记录 missed admission，不推迟成较低输入率后宣称通过。
单 worker 保序；不为本轮引入多 worker/reorder 的额外实现。

## 5. 最小 SIC 核心

拟定 `uwb_preamble_sic_core.h/.cc` 与 `uwb_preamble_sic.h/.cc`；实际命名遵循 OOT
现有习惯。禁止复用旧 full-packet SIC 的 FCS 前置门槛。

1. **检测**：在公开候选 ROI 内用 comm template 做粗到细搜索，再作重复性确认。
   不把 sensing CIR 成功作为唯一入口；强干扰下 sensing gate 失败也必须能尝试 SIC。
   记录粗/细候选数、相关次数、ROI 大小和拒绝原因。
2. **起止与周期歧义**：固定名义 64 SYNC 不等于已知真实起点。相关峰需通过可见
   前沿/重复区间确定符号编号。边界证据不足时标记 partial/ambiguous，只在已确认
   区间相减或旁路，不能把周期内峰当成全前导起点。
3. **CFO/时延/CIR**：重复符号的复相关相位用于 CFO；校正干扰参考/估计窗口后
   才相干平均。对整段 y 做干扰 CFO 去旋转后，最终输出必须恢复原坐标/相位，
   推荐保持 y 不变而在模板中加入 CFO。感知目标有自己的相位演化。
4. **路径**：局部峰、最小间距、至多 K 个 distinct delays；K 默认 4，允许
   1/2/4/8/16。间距依据模板自相关主瓣，不把同一路径多次计入 K。
5. **复增益拟合**：选定时延构造实际成形波形列，用复数 QR/稳定 LS 求增益。
   相关 tap 不是天然物理路径增益，L2-normalized CIR 不能用于绝对幅度相减。
   检查秩/条件性、有限数值及容量；不显式求逆。可用现有 Eigen/LAPACK，先查依赖。
6. **重构/验证**：整数延迟先闭环，再加入分数时延；模板包含真实重复、前导边缘、
   脉冲和多径尾部。CFO 用递推旋转并定期绝对相位重锚，避免逐点 sin/cos。
   相减区间是估计的 comm 支持区与公开 sensing 处理 ROI 的交集，加上有定义的拖尾。
   上限来自配置与信号证据，不能从真值 sidecar 取得精确 mask。
7. **trial/commit**：拟合和检查使用不同重复符号；检查 comm 残余相关与参数可信度。
   仅总能量下降不够；失败时输出与输入一致。在线门槛不能读取 clean/s/i 或
   根据评估结果决定是否 commit。固定门槛在校准集确定后，在验收集保持不变。

最短估计窗先用 8 SYNC（8128 点），验证窗使用另外至少 8 SYNC；不足则明确失败。
扫 8/16/32 SYNC 分析精度与观察时间。S0 是 buffered processing，不能把使用
后续符号验证的结果描述为“前 8 SYNC 到达时即可实时输出”。

## 6. 解调模块耗时：必须实测的对象与归因

### 6.1 对象分开

测量三种消费者，不能以其中一种替代其余：

- `radar_cir_only`：sensing 定位/SFD/平均 CIR；另测 `use_predicted_timing=true`
  的计划坐标路径，标明省去的搜索，不把它冒称实际 RX 检测。
- `demod_full`：完整 sensing 包 `demodulate_one`，匹配实际 code/SFD/长度。
- `sic_frontend`：只识别 comm 前导并估计/重构；无 PHR/payload 解码。

Full/RAKE Top-4/Bypass 是 **demod soft-chip** 模式；`sic_k` 是干扰路径数，必须
用不同配置字段。主报告先固定 demod bypass，再测 full 与 RAKE Top-4；每种都
报告 clean/raw/sic 的完整包正确性，不能仅比较成功子集。

### 6.2 现有与新增计时

复用 `DemodResult` 的七阶段字段及 coarse/fine、`cir_estimate_us`、`soft_fir_us`、
`postprocess_us`。保留旧字段语义，在诊断结构中补充不重叠子计时：

`sfd_bootstrap, tail_retrack, cfo_refit, sfd_final, sfd_anchor_other`。
这些是 `stage_sfd_us` 的子项，不能再与父项一起累加；同理 CIR 子项不二次计总。
新计时只包围现有调用，不在本任务顺带修改解调算法。外层 steady_clock 总时间为
权威，微秒截断/未归因差额单列；失败阶段也计时，未执行阶段标为 not_run。

SIC 计时至少分为：

`detect_coarse, detect_fine, cfo, cir, path_select, fractional_refine,
matrix_build, gain_fit, reconstruct, validate_commit, core_total`。

adapter/链路另计 `handler, queue_wait, output_wrap_publish, consumer_service,
resample, completion_latency, thread_cpu_time`。
核心/生成器/读盘/画图/等待回放时钟分别计量，不以 Python sim `.tolist()` 成本
归因 C++ demod，不以生成速度限制后的窗口率宣称处理器容量。
检查插桩开/关的短对照，报告计时本身开销；保留现有数值和解码结果。

`completion_latency` 起点设为该窗口计划发布时刻，包含 admission 迟滞、消息入口
等待、worker 排队、处理直至终态。另报告从实际发布到终态的延迟。
本轮文件回放延迟不含射频接收/硬件传输；理论采样等待另列，不能冒称空口延迟。

## 7. 时间、算力与内存预算

以下均为运算量分析，不是 µs 性能结论：

| 部分 | 规模 | 主配置数量级 |
|---|---|---|
| ±64 点局部相关 | 129×1016 | 131064 complex MAC |
| 8 个重复各自局部相关 | 8×129×1016 | 1048512 complex MAC |
| LS | O(MK²)，M=8128、K=4 | 规模项约 13 万，不等于同量 MAC |
| 重构 | N×K，N=65024、K=4 | 260096 complex MAC |
| 初始粗搜 | 取决于真实 ROI/stride | 必须记录，不包含在上述局部相关内 |

全前导逐点直接搜约 65024×1016=6606 万 MAC/窗，200 窗/s 约 132 亿 MAC/s。
不能只计 ±64 点精搜而漏掉获取精搜中心的成本。核验短 ROI 与宽 ROI 的耗时差别。
LS 的 K² 项使 K=16 不一定划算，不强制所有 K 达成主配置速度。

work CF32 完整包约 19 万点，四份 IQ、矩阵和各消费者 scratch 总量必须实测；
仅 64-SYNC 原始/cleaned 两份约 1.04 MB。1 GS/s SC16 输入约 4 GB/s，CF32
约 8 GB/s；本轮结论仅适用于有界窗口率/占空比，不自动证明连续入口实时。

主优化目标：SIC 核心 mean≤0.5 ms、P99≤1 ms；这是目标，不是硬性数学保证。
200 窗/s、单 worker 每阶段 mean≤2.5 ms 可给约 50% 平均利用率余量。
真实链路需按每个串行阶段 `rate×E[T]` 判断；CIR-only 和 full demod 分别给出
容量结论。流水吞吐由瓶颈阶段限制，单窗延迟仍计所有串行依赖和排队。

即使碰撞比例 q 很低，也须计 `每窗检测成本 + q×消除成本`，另测连续碰撞突发。

## 8. 测试矩阵与验收

### 8.1 G0：先闭合无干扰基线

1. canonical code9 RX 的完整 sensing 包 FCS/PSDU 与已知数据一致，CIR 与已有
   MATLAB golden 对齐；若 profile 不匹配，先修适配，禁止靠重调门槛伪造通过。
2. code10 comm SYNC 模板必须独立生成，对 MATLAB 逐样点核验。code 改为 11
   的错模板对照应表现不同，不能意外复用 sensing code9 的缓存。
3. 禁用干扰/禁用 SIC 时，新 sim 与旧 fixture/链路相同；signal shape 与噪声种子固定。
4. 记录完整窗口和 CIR 子窗的起止、完整帧是否落在窗口内；越界必须拒绝或显式标 partial。

### 8.2 主效果集与诊断集

主效果集：code9/code10，64 SYNC，SNR=30 dB，ISR=0/10/20 dB；comm 相对
sensing SYNC 起点偏移 0、0.4、8、-8 work samples；CFO=0、±50 kHz。
整数单径先实现；增加四径的 comm 信道，超出 K 的压力用例单列。
拟合实际延迟随机相位，不从生成器传入；主矩阵每格 100 个固定 seed。

模型示例（work samples）：sensing 延迟 `[0,37]`、复增益
`[1, 0.25*exp(j*0.7)]`，用于保留弱目标；comm 单径 `[0]`，四径
`[0,7.4,23,51.2]`、复增益 `[1,0.5j,0.3*exp(-j*0.4),0.2]`。
所有 comm 路径还叠加独立的前导起点偏移，并统一按实际 ISR 缩放。
默认局部 CIR post=100 足以覆盖这些延迟，但必须加成形响应和保护边界检查。

诊断集不作全笛卡尔积：

- 无干扰、无重叠、错误 code11、同码近时延、低 SNR、极弱干扰；
- 前导整体平移约 ±半个 SYNC，只有中间片段可见，周期边界及窗口头尾裁切；
- K=1/2/4/8/16，估计长度=8/16/32，宽/窄粗搜 ROI；
- SFO=±20 ppm、生成/拟合使用不同 pulse 或分数延迟模型、SC16 clipping。

必须包含 full/partial/no overlap；记录有效重叠样点数，不能把无重叠当作消除成功。
主配置全部通过再运行扩展压力集；已经出现效果或算力失败时先定位，不盲跑大矩阵。

### 8.3 指标与结论规则

- 重构 NMSE：`||i-i_hat||²/||i||²`，在真值评估区间计算，注明噪声是否包含。
  抑制度为 `10log10(||i||²/||i-i_hat||²)`，不以 `||y||²/||y_clean||²` 替代。
- 除全区指标外，在未用于拟合/门控的 holdout 区间报告同一指标。
- sensing CIR：clean/raw/sic 逐 tap、目标复幅度、峰位相对 clean 的偏差、弱目标
  检测率和假峰；不能只看最大 tap。不得把 radar 峰位当作硬件 ToA。
- 用固定 target ROI 的相干投影/匹配幅度评价目标保护；总能量不是目标幅度。
- 完整 sensing demod：逐窗状态、完成率、PSDU/FCS、失败阶段、成功与失败各自耗时。
  SIC 后仍失败必须计入，不跳号、不只在成功子集统计延迟/抑制度。
- 主条件初始门槛：抑制度中位数≥10 dB，且至少 95% 窗口抑制度≥3 dB；
  ≥95% 窗口的预定义目标幅度误差绝对值≤1 dB、相对 clean 峰位偏差≤1 work sample。
  未过时报告适用条件和失败率，不事后删条件或改验收集门槛。
- 无干扰/错模板：单列误检测、误 commit；旁路逐样本不变。独立负样本至少
  1000 窗，误 commit≤1%；报告计数/样本量及二项置信区间，不声称零风险。
- 对 raw 已可解码的同窗，单列 SIC 导致的成功→失败回退；对不可解码窗单列恢复。
  不强制整个 sensing full demod 达到与 CIR-only 相同的速度或效果。

### 8.4 性能实验

1. Release 单 worker，先纯 C++ core，再 GR/PDU，再 native roundtrip；三个层级分表。
2. 200 窗 warmup 后计时；每配置至少 10000 窗、三轮，配置顺序轮换，benchmark 串行。
   若单轮超 60 s，按批输出进度与累计统计；性能失败不能让脚本无限等待。
3. 热缓存单窗与至少 64 个不同内容窗口轮换分别测；scratch 复用，避免只测同一常量
   或依靠编译器删去计算。负载生成在计时前，终态必须被消费并核验。
4. 优先做 200/s，100% collision、单 worker、60 s 定速回放；再测无碰撞、10%
   碰撞及连续 20 窗碰撞突发。200/s 通过后进行 10 min 确认，再探索 1000/2000/s。
   高速失败有价值，报告实际 admission/completion/deadline，不强制升级多线程。
5. 200/s 首轮端到端工程目标：从计划提交到 CIR 终态 P99≤5 ms，0 丢弃/重复/缺号，
   排队无持续上升；负载源错过计划同样记失败。full demod 单独报告是否达标。
   WSL 结果只证该环境的软件窗口处理，不承诺硬实时。
6. 输出 mean/P50/P95/P99/max、CPU seconds、RSS、队列/入口 in-flight 高水位、
   offered/admitted/completed/failed/bypassed/dropped、deadline misses。
   饱和吞吐实验与定速延迟实验分开；文件落盘/绘图不进主要计时区间。

## 9. 实现顺序与文件交付

按下列顺序推进已授权的 S0 工作；阶段失败先定位并报告，不自动扩展本轮范围。

| 阶段 | 实现及验收产物 |
|---|---|
| S0-A | 复用现有 RX 的 fixture/注入器；独立 truth；clean/raw 无 SIC 的 CIR/完整解调 G0 与计时；先交基线 |
| S0-B | Python/NumPy 数学参考和纯 C++ 核心；oracle 重构仅诊断；实际估计完成单径→四径→分数延迟/CFO；针对性 QA 在对应实现前新增 |
| S0-C | 薄 PDU adapter、Python binding、GNU Radio 仿真 CLI；配对结果、旁路/容量/stop/drain QA |
| S0-D | MATLAB 对照、core/GR/native 分层 benchmark、定速压力、verifier 和结果报告；明确 go/no-go |

建议新增文件（均为待实现路径）：

- `gr-uwb/apps/uwb_sim_preamble_sic.py`：复用已有 sim 的信号/几何，不改旧入口默认行为；
- `gr-uwb/apps/benchmark_preamble_sic.cc`：直接 core 与实际输入 full demod profiler；
- `gr-uwb/include/gnuradio/uwb/uwb_preamble_sic_core.h`、`gr-uwb/lib/uwb_preamble_sic_core.cc`；
- `uwb_preamble_sic.h`/`uwb_preamble_sic.cc`、binding，参照现有命名布局；
- `gr-uwb/lib/qa_uwb_preamble_sic_core.cc`、`qa_uwb_preamble_sic.cc`，CMake 注册；
- `testdata/sic_preamble/README.md`、生成器、`cases/*.json`、少量 fixture 与 MATLAB 导出/核验脚本；
- `tools/sic/verify_s0.py`：核验输入 hash、三路配对、指标/终态/计时完整性；
- `docs/sic/S0_前导碰撞仿真与耗时开发报告.md`：最终填入命令、真实结果、局限和建议。

配置至少包括 `sensing_code, comm_code, sync_reps, pulse/template_id, paths,
start_offset_work, cfo_hz, snr_db, isr_db, seed, sic_k, fit_reps,
comm_search_roi, validation_reps, demod_softchip_mode, input_grid`。
生成器配置与 SIC 运行配置分开保存，只有前者包含注入真值。
默认禁止 sensing_code==comm_code；仅显式 diagnostic preset 允许。

CLI 要实现下面的调用契约；命令目前是**开发后的验收命令，不表示现已存在**。
`baseline/main/negative/stress` preset 在 README 和 JSON 中冻结；参数未知时拒绝。
仿真 CLI 的 main 展开 §8.2 主效果矩阵，`--windows` 是每个 case 的数量；
每个 case 单独摘要。benchmark 的 main 仅取名义 case（ISR=10 dB、CFO=0、
offset=0、K=4），其余轴由单独 case 配置扫描，避免把大矩阵混成一个延迟分布。
所有 preset 必须先生成固定的校准/验收 seed 列表，两者不交叉。

```bash
# 构建前检查现有 cache；可使用独立 build-sic，不能无声改坏用户现有 build 配置。
cmake -S gr-uwb -B gr-uwb/build-sic -DCMAKE_BUILD_TYPE=Release
cmake --build gr-uwb/build-sic -j4
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic/apps/benchmark_preamble_sic
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic --output-on-failure -R 'preamble_sic|demod_core|radar_cir|loopback_echo'

# CLI 必须按 --build-dir 加载同目录 binding/lib，不能硬编码旧 gr-uwb/build。
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py --build-dir gr-uwb/build-sic --preset baseline --input-grid work --sic off --consumer both --windows 100 --output gr-uwb/build-sic/sic-results/baseline
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py --build-dir gr-uwb/build-sic --preset main --input-grid work --sic estimate --consumer both --windows 100 --output gr-uwb/build-sic/sic-results/main
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py --build-dir gr-uwb/build-sic --preset main --input-grid cg400 --sic estimate --consumer both --windows 100 --output gr-uwb/build-sic/sic-results/native

env -u LD_LIBRARY_PATH gr-uwb/build-sic/apps/benchmark_preamble_sic --preset main --mode core --consumer demod --windows 10000 --rounds 3 --output gr-uwb/build-sic/sic-results/core
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_preamble_sic.py --build-dir gr-uwb/build-sic --preset main --input-grid work --sic estimate --consumer cir --rate 200 --duration 60 --output gr-uwb/build-sic/sic-results/paced200
python3 tools/sic/verify_s0.py --results gr-uwb/build-sic/sic-results
git diff --check
```

`--consumer cir|demod|both`，both 在功能配对中运行两类消费者，正式性能测试分开。
`--windows` 与 `--rate/--duration` 互斥；rate 模式预生成多样本并记录计划/实际提交时间。
`--sic off|estimate|oracle`，oracle 只能诊断；estimate 模式始终产出 clean/raw/sic
配对效果，性能时依次跑各配置。参数/fixture 错误返回非零。
verifier 只检查已声明的 required artifacts；所有未执行项须在报告中明确列出，
不能用 summary 缺失默认为成功。

MATLAB 执行前检测环境；脚本使用新目录，不覆盖 canonical。导出 code10 模板、
已知 s/i/y、检测起点和逐 tap CIR，与 C++ 逐样点/逐 packet 对照并冻结容差。
MATLAB 与 C++ 用不同实现生成/定位，不只拿 C++ 输出互相读写形成循环证明。
若 MATLAB 不可用，保留可执行脚本、错误日志并写“未实跑”；继续其他工作，
不得把 Python 参考替代为 MATLAB 已验收。

## 10. 结果归档与最终判断

生成 `run_manifest.json`（commit、dirty 状态、编译 flags、实际加载库路径、
CPU/OS/GR/VOLK、配置/fixture/template hash）、`windows.jsonl`、
`stage_times.csv`、`summary.json` 和简明图表。run_id/窗口 ID 必须跨表可关联。
大文件/原始 benchmark 输出留在 build/tmp，选定结果显式归档后再引用；
不把大量仿真 IQ 或自动生成物混入版本控制。

最终报告分别给出四个判断：

1. **效果**：不同 code 的重叠前导能否被估计并消除，目标是否保留；
2. **SIC 新增开销**：宽/窄搜、K、fit_reps、fractional/CFO 的真实成本；
3. **完整解调成本**：clean/raw/sic 七阶段及细分热点，成功/失败路径分开；
4. **运行边界**：work/native、CIR-only/full demod 在何窗口率和延迟目标下通过。

效果通过但速度未达时，以 profiler 最大热点选择一次针对性优化并重测；
估计不可靠或目标被消除时先修算法，不能靠增加线程解决。
不能因连续 1 GS/s 目标未证就否定窗口方案，也不能以 200 窗/s 通过宣称连续入口实时。
本轮结束提交开发报告和待解决清单，不自动进入硬件或扩大到其他协议/干扰源。
