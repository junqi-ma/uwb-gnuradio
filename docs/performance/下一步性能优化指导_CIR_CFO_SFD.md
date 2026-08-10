# UWB 实时解调下一步性能优化指导

> 执行对象：Claude Code  
> 优化范围：`UwbRealtimeDemodulator` 的 CFO、CIR/soft chips、SFD  
> 基准：`性能分析_解调分阶段耗时报告.md`  
> 原则：MATLAB `UWB_demodulation/` 是算法正确性的唯一基准；每次只改一个热点并完成 QA、构建和 benchmark。

## 目标

将 golden 窗口单包耗时从约 **17.2 ms** 降到 **4–5 ms**：

| 阶段 | 当前 | 目标 |
|---|---:|---:|
| CFO | 3.5 ms | 0.3–0.6 ms |
| CIR | 10.3 ms | 2–3 ms |
| SFD | 2.1 ms | 0.15–0.3 ms |

## 执行顺序

### P0：先补正确性与细分计时

1. CFO 估计必须与 MATLAB 对齐：
   - 使用 **SYNC 匹配滤波峰值的复相位**，不要使用 `arg(rx[peak])`。
   - 对稳定峰相位执行 `unwrap`。
   - 使用相对时间做线性拟合。
2. 增加 CFO sweep QA：`0、±1、±5、±10、±25、±50 kHz`，覆盖 golden 与加噪信号。
3. 将 CIR 计时临时细分为：
   - CIR estimate；
   - soft-chip FIR；
   - phase/real/max/normalize。

未通过 CFO sweep 前，不进入 SIMD 优化。

### P1：优化 CFO 去旋转

将逐样本 `sin/cos` 改为递推旋转：

```text
step = exp(-j*w)
每 1024 samples：
    rot = exp(-j*w*block_start)   // 按绝对相位重新锚定
    out[i] = in[i] * rot
    rot *= step
```

要求：

- 每 1024 点重新计算绝对相位；不要只做幅度归一化。
- `stage_sfd` 改用 CFO 补偿后的 `scratch.derotated`。
- 不改变 CFO 输出字段和坐标语义。

验收：

- 所有 CFO sweep 解调结果、PHR、payload、FCS 与基准一致。
- CIR L2、soft-chip 最大误差保持现有 QA 容差。
- CFO 阶段均值不高于 0.6 ms。

### P2：优化固定 38-tap CIR soft-chip FIR

先将点积改写成连续正向访问：

```text
原式：sum_k conj(values[37-k]) * rx[base-k]
等价：sum_q conj(values[q]) * rx[base-37+q]
```

按以下候选分别 benchmark：

1. 4/8 路 scalar accumulator；
2. VOLK conjugate dot product；
3. 固定 38 taps 的 AVX2/FMA 内核，带 portable scalar fallback。

选取实测最快且通过 golden 的实现。优先固定长度 SIMD；VOLK 的 154k 次短函数调用可能有明显 dispatch 开销。

要求：

- 不启用整个编译单元的 `-ffast-math`。
- 允许浮点结合顺序变化，但必须按 golden 容差验证；不要宣称逐 bit 相同。
- 把 reversed/conjugated taps 在 chip 循环外准备好。
- SIMD 内核必须处理 38 taps 的尾部，不能越界读取。

随后合并后处理：一次循环完成 complex phase rotation、取实部、写入 `soft_chips` 和计算 `max(abs(real))`，再做一次归一化循环。

验收：

- CIR first path、CIR L2、soft chips、PHR、payload、FCS 全部通过现有 golden QA。
- golden 窗口 CIR 阶段不高于 3 ms。
- 同时报告 319168 与 203776 两种窗口的耗时。

### P3：压缩 SFD ROI

当前 sample-domain SFD 实际搜索 `expected ±1016 samples`，即 `±508 chips`；profile 中的 `sfd_search_half_width = 8 chips` 尚未使用。

完整跟踪到 64 个 SYNC 时，搜索中心改为：

```text
expected = last_sync_peak_end
           - (template_length - 1)
           + round(measured_period)
half_width = 8 chips * 2 samples/chip = 16 samples
```

回退策略：

- 64 peaks 完整：`±16 samples`；
- 峰数足够但不完整：`±64 samples`；
- timing 不可信：退回 `±1 symbol` 或 timing failed。

要求：

- 搜索中心使用 `measured_period` 或最后一个峰，不再使用固定 `64×1016` 外推。
- SFD 输入使用 CFO 补偿后的 IQ。
- 修正细化窗口边界，必须 clamp 在配置 ROI 内。
- 可在构造期预计算并缓存固定 4z2 SFD waveform。

验收：

- SFD start/end/polarity 与 MATLAB、golden 逐包一致。
- 增加 SFD 偏移、SFO、AWGN、multipath 测试。
- SFD 阶段均值不高于 0.3 ms。

## 每个修改的强制流程

1. 先搜索本地 GNU Radio/VOLK 中相似 FIR、dot-product 实现，并说明所选实现与调度语义；本模块为纯 message worker job，不改变 GNU Radio scheduler 行为。
2. 先增加或更新 QA，再修改热点。
3. hot path 不新增动态内存分配；复用每 worker 的 `DemodScratch`。
4. 每个 P 阶段完成后运行：

```bash
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build --output-on-failure
./gr-uwb/build/apps/benchmark_detector \
  testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile \
  demod-stage-profile
```

5. 最后运行 100/200 pkt/s soak，报告：mean/p95 latency、worker utilization、queue high-water、drop、各阶段均值。

## 完成标准

- 所有 CTest、golden、CFO/SFD 鲁棒性测试通过。
- golden 单包总耗时达到 4–5 ms，或给出未达标阶段的细分证据。
- 重新评估 worker 数：重点验证 `200 pkt/s + 2 workers` 是否可稳定零丢包。
- 更新 `性能分析_解调分阶段耗时报告.md` 和 `../../开发状态.md`，记录修改前后数据与数值误差。

