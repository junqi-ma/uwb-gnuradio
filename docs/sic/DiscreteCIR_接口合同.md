# Discrete CIR full-packet SIC：S0 接口与数学合同（2026-11-06）

任务书：`FullPacket_离散CIR估计开发任务书_2026-11-06.md`。本文件冻结新实验后端
`discrete_cir` 的接口、数学约定、支持选择、归一化和验收边界。旧
`path_fractional`、preamble-only、冻结 core/adapter、`sic_reference.py` 均不改。

## 1. 公共接口

头文件 `gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h`。

- `DiscreteCirConfig`：公开 profile、训练/搜索 ROI、`lag_lo`、最大 hull
  `tap_count`、粗 CIR 峰值门限/最小间距/active 半径、相对 ridge `lambda`、检测/
  拟合门控和 scratch 容量。`observed_end` 作为 `estimate_discrete_cir` 的运行时
  参数传入。
- `DiscreteCirTemplates`：`sync` 为一个成形 SYNC；`train` 为公开 preamble 的前
  `fit_reps` 个连续 SYNC。**不能由 `sync` 循环拼接**，因为真实有限前导包含起始
  瞬态和符号间脉冲拖尾。
- `estimate_discrete_cir`：严格只读 `y[0, min(observed_end,n))`，冻结
  `DiscreteCirModel`。不接收 sidecar/truth。
- `apply_discrete_cir`：输入已重调制完整波形、公开参考 SYNC、冻结模型和 ROI；
  失败时 `out==y` 逐位、`ihat==0`。
- `fullpacket_cancel_discrete`：在线事务包装，校验 FCS/PHR/profile，重调制一次，
  应用冻结模型并只在 ROI 内相减。

## 2. 数学

绝对 work-grid 坐标，包起点 `q`，CFO `w=2*pi*f/fs`，相位锚点 `r=q`：

    y_hat[n] = exp(j*w*(n-r)) * sum_l h[l] * x[n-q-l]

`x` 是重调制器输出的完整、已脉冲成形波形，不是 chip 序列，不是相关器输出。
`h` 是相对于公开成形训练模板的有限整数 lag FIR，吸收分数传播时延和复增益。

训练（只使用前 `fit_reps` 个 SYNC）：

    z[m] = y[q+m] * exp(-j*w*m)
    X[m,i] = train[m - (lag_lo+i)]      # 包起点前为零，右侧越界行丢弃
    h = argmin ||X h - z||^2 + lambda ||h||^2

`training_rows M = fit_reps*sps + min(0, lag_lo)`；这保证负 lag 列不读取训练模板
右侧之外的数据，正 lag 列在包起点左侧按真实零处理，不做循环延拓。

## 3. 支持选择与缓存

1. 用同一 8-SYNC 前缀估计 `q`、`omega`（独立 acquisition helper，参考旧检测/CFO
   语义，不调用旧分数路径搜索或增益拟合）。
2. 去 CFO 后按 `sps` 折叠 8 个 SYNC，计算连续 lag `[lag_lo, lag_lo+tap_count)` 的
   粗匹配滤波 CIR。
3. 找局部极大值，保留 `|cir[l]| >= peak_floor*peak` 的峰；按幅度从大到小贪心选择，
   与已选峰距离小于 `peak_min_spacing` 的丢弃。每个峰激活
   `[peak-active_radius, peak+active_radius]` 的 lag。有效 lag 集合是这若干个
   3-tap（默认 radius=1）组的并集；连续 hull 为 `[min(active),max(active)]`。
   hull 宽度必须 `<= tap_count` 且 `< samples_per_symbol`。hull 内未激活的 tap
   严格为零，模型仍以连续 hull + 零 tap 表示。
4. 热调用对 active lag 集合计算 `X^H z` 和 `X^H X + lambda I` 的小矩阵，用固定
   容量的 factor cache 做 Cholesky 分解；cache key 为
   `(template_hash, config_hash, active lag set, training rows, lambda)`。缓存命中
   直接三角求解，未命中在预分配槽位内计算，不显式求逆、不构造 M x L 大矩阵、
   不在热路径分配。`cond2(X)` 和有效秩由该 K x K 因子计算并随模型记录。

`lambda` 为相对 Gram 平均对角线的固定值，默认 `1e-6`；用独立 seed 校准，禁止在
10 格验收窗口上选参数。S0 原型（离线、仅用于确定设计，不作为验收）显示：
单径格 active 集合约 3 taps，四径 `payload_four_frac` 约 12 taps、hull 约 54；
设计矩阵条件数约 `1e3..1e4`，需在报告中逐包记录实际值。连续宽支持会把 sensing
前导拟合进 h，active 峰值组可避免该退化。

## 4. 归一化

`h` 相对公开 `train`（peak amplitude 0.8 的公开成形前导）拟合。重调制器输出
`w` 有整包峰值归一化，因此 apply 先用公开参考 SYNC 计算一次实标量

    s = <w, ref> / <w, w>

并检查 `s>0`、虚部足够小、`||s*w-ref||^2 <= 1e-10*||ref||^2`，然后用
`x = s*w` 做 FIR。`h` 已包含实际包幅度，**不得再乘一次 s**。这与旧 full-packet
路径的 `waveform_scale` 语义一致。

## 5. 失败与旁路

Estimate 失败（低于检测门限、训练前缀不足、非有限、支持不可用、条件数/残差门控）
=> `model.valid=false`，不产生可相减模型。

Apply 失败（缓冲区、容量、FCS、PHR/profile、模型无效、ROI 无重叠、上下文截断、
模板比例失配、非有限输入/重构）=> `out == y` 逐位、`ihat == 0`，`committed=false`。
唯一允许修改 `out` 的路径是 `committed=true`，且只在 `[sub_lo,sub_hi)` 内。

## 6. 计时

- 估计：`detect_us`、`cfo_us`、`support_us`、`projection_us`、`solve_us`、
  `fit_check_us`；父项 `estimate_total_us`，父项不重复累加子项。
- 重构：`validate_us`、`tx_us`、`scale_us`、`convolution_us`、`cfo_us`、
  `subtract_us`；父项 `channel_us`/`apply_total_us`。
- 下游 comm 解调、CIR、sensing 解调仍由既有 benchmark 单列。正式计时绑定
  code-tested SHA、build、库 hash、profile、样本长度，独占 benchmark 锁。

## 7. 验收

功能：既有 10 格 × 16 源窗，新 `discrete_cir` 与旧 `path_fractional` 同源窗配对；
配对 comm/sensing 正确解码数不低于旧路径；负样本无新增误提交；每格目标区抑制和
弱目标保护相对旧路径劣化不超过 1 dB。困难 CFO 原有失败如实保留。

性能：质量通过后，`sync_base`/`payload_base` 两格各 16 源窗，变体 16 次预热 +
128 次观测 × 2 轮，AB/BA 顺序；配对总 SIC mean 降低至少 10%，P95 不增加。
未达到即报告瓶颈，不无限扫参，不宣称 0.5 ms/200 窗/s 已达标。

MATLAB/native/FPGA/硬件未运行，不得写成通过。
