# Codex UWB 解调器性能优化记录

> 日期：2026-08-10
> 对象：`UwbRealtimeDemodulator` / `uwb_demod_core`
> 平台：Intel Core i7-12700，GCC `-O3 -march=native`，GNU Radio 3.10，VOLK 2.5
> 基准：319168 个 CF32 samples、code-9、64 SYNC、IEEE SFD、127-byte PSDU
> Golden：`testdata/realtime_demod_golden/` 与 `testdata/reference_preamble.bin`

## 1. 目标与约束

本轮工作针对实时 UWB 解调器中优化后仍然占比较高的 Timing 和 CIR 阶段，目标是：

1. 使用成熟 SIMD 原语优化长相关；
2. 增加可选 Top-K sparse RAKE 多径合并；
3. 解决 sparse RAKE 未充分利用 AVX2、可能慢于 Full-38 FIR 的问题；
4. 保持 MATLAB/golden 的检测坐标、PHR、payload 和 FCS 语义；
5. 默认配置保持原有完整 38-tap CIR matched filter；
6. hot loop 不新增动态内存分配。

`UwbRealtimeDemodulator` 是 message-only block：消息处理线程只校验并入队，实际解调由有界队列后的 worker pool 完成。本轮修改全部位于单个 worker job 内，不改变 GNU Radio stream scheduler 的 `work/general_work`、consume 或 produce 语义。

## 2. 优化前基线

在此前 CFO、SFD 和 CIR 固定 38-tap AVX2 优化完成后，单包典型耗时约为：

| 阶段 | 耗时 |
|---|---:|
| Timing | 约 1534 µs |
| CFO | 约 416 µs |
| SFD | 约 126 µs |
| CIR | 约 1662 µs |
| NS-SFD | 约 45 µs |
| PHR | 约 5 µs |
| Payload | 约 62 µs |
| Total | 约 3849 µs |

CIR 内部约为：

| 子阶段 | 耗时 |
|---|---:|
| CIR estimate | 49 µs |
| 38-tap soft-chip FIR | 1364 µs |
| phase/postprocess | 246 µs |

## 3. Timing：VOLK 长复相关与滑动功率

### 3.1 原始瓶颈

`stage_timing()` 的每个候选位置都会重复执行：

```cpp
for (size_t k = 0; k < L; ++k) {
    acc += rx[j + k] * std::conj(tmpl[k]);
    pw += std::norm(rx[j + k]);
}
```

其中模板长度 `L=1016`。相邻候选窗口重叠 1015 个 samples，但相关和窗口功率都被完整重算。

### 3.2 修改

- 使用 `volk_32fc_x2_conjugate_dot_prod_32fc` 执行 1016 点共轭复点积；
- 第一个候选窗口完整计算功率；
- 后续候选通过下式 O(1) 更新：

```text
pw_next = pw - |old|² + |new|²
```

VOLK 适合这里的原因是单次点积较长，runtime dispatch 和函数调用成本可以被 1016 点计算摊薄。它不适合原有 CIR 的 154578 次 38 点短调用。

### 3.3 结果

| 指标 | 优化前 | 优化后 |
|---|---:|---:|
| Timing | 约 1534 µs | 约 289–317 µs |
| Timing 提速 | — | 约 5.1× |
| 单包总耗时 | 约 3849 µs | 约 2715–2784 µs |

新增 QA 将 VOLK 输出与随机复数标量参考比较，并覆盖非对齐输入地址。

## 4. 可选 Top-K sparse RAKE

### 4.1 参数语义

在 `Qm35825Profile`、`Dw1000Profile` 和 realtime block 中增加：

```cpp
size_t cir_rake_top_k = 0;
```

语义：

- `0`：保留完整 CIR matched filter；
- `1..tap_count-1`：选择 CIR 中能量最大的 K 个 complex taps；
- `K >= tap_count`：等价于完整 FIR；
- realtime block 构造参数最大允许值为 64。

选择过程在每包 CIR 估计后执行：

1. 按 `std::norm(values[q])` 选出 K 个最强 taps；
2. 将索引按时延升序排列，改善访问规律；
3. 保存 `conj(values[q])` 作为复数 RAKE 权重；
4. soft-chip 阶段只累加选中的路径。

权重保留 CIR 的复相位，因此是相干合并，而不是只做幅度求和。选择和权重使用固定栈数组，FIR hot loop 不分配内存。

### 4.2 接口

C++：

```cpp
auto demod = gr::uwb::UwbRealtimeDemodulator::make(
    template_path,
    2,       // workers
    64,      // queue capacity
    "4z2",
    4        // cir_rake_top_k
);
```

Python：

```python
demod = uwb.realtime_demodulator(
    template_path,
    2,
    64,
    "4z2",
    4,
)
```

GRC 参数：

```text
CIR RAKE Top-K (0 = full)
```

Benchmark：

```bash
./gr-uwb/build/apps/benchmark_detector \
  testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile \
  demod-stage-profile --rake-top-k 4
```

## 5. Sparse RAKE 第一版的问题

第一版 sparse RAKE 使用通用标量间接索引内核：

```cpp
acc += weights[i] * rx_win[indices[i]];
```

虽然减少了 complex MAC 数量，但失去了 Full-38 固定 AVX2 内核的优势：

- 输入地址离散；
- `std::complex<float>` 乘法主要为标量路径；
- 每个 soft chip 单独计算和归约；
- Top-8 的两轮依赖链使性能接近或差于 Full-38。

第一版典型结果：

| 策略 | soft FIR |
|---|---:|
| Full-38 AVX2 | 约 1.3–1.5 ms |
| Top-8 scalar | 约 1.4–1.7 ms |
| Top-4 scalar | 约 0.7–0.8 ms |

结论：只减少 K 不足以保证提速，必须改变 SIMD 维度。

## 6. Top4X4 / Top8X4 跨输出 AVX2

### 6.1 核心思路

不在“一个输出的多个离散 tap”维度向量化，而是在“同一 tap 的四个相邻输出”维度向量化。

soft-chip decimation 为 2。对于 tap `q`，四个相邻输出使用：

```text
out[i+0] <- rx[base + q + 0]
out[i+1] <- rx[base + q + 2]
out[i+2] <- rx[base + q + 4]
out[i+3] <- rx[base + q + 6]
```

四个 complex samples 正好组成一个 AVX2 `__m256`。

### 6.2 输入装载

没有采用 `_mm256_i32gather_ps`。每个 tap 使用两次连续 unaligned AVX2 load：

```text
load A = complex q ... q+3
load B = complex q+4 ... q+7
```

随后使用 `permutevar8x32` 和 `permute2f128` 选出：

```text
{q, q+2, q+4, q+6}
```

这样保留连续访问，同时避免硬件 gather 的高延迟。

### 6.3 复数乘加

每个复权重扩展为：

```text
{hr, hi, hr, hi, hr, hi, hr, hi}
```

然后通过 `moveldup`、`movehdup`、复数分量交换及 `fmaddsub` 同时完成四个 complex multiply。Top-4/Top-8 模板参数在编译期固定：

```cpp
dot_topk_x4_avx2<4>()
dot_topk_x4_avx2<8>()
```

每轮直接保存四个 complex outputs，不需要逐输出水平归约。

### 6.4 安全回退

- 仅在 AVX2/FMA 可用、`K=4/8`、`samples_per_chip=2` 时启用；
- helper 连续读取到 `max_selected_q+7`，主循环显式检查输入边界；
- 最后不足四个输出或靠近输入尾部时使用标量 sparse RAKE；
- 非 AVX2 平台完整保留 portable scalar fallback。

## 7. 最终性能

每种策略运行 7 次，以下为中位数：

| 策略 | FIR 中位数 | CIR 总计中位数 | 单包总计中位数 |
|---|---:|---:|---:|
| Full-38 AVX2 | 1335 µs | 1634 µs | 2658 µs |
| Top-8 X4 AVX2 | 335 µs | 693 µs | 1620 µs |
| Top-4 X4 AVX2 | 182 µs | 471 µs | 1426 µs |

相对 Full-38：

| 策略 | FIR 提速 | CIR 提速 | 单包提速 |
|---|---:|---:|---:|
| Top-8 X4 | 3.99× | 2.36× | 1.64× |
| Top-4 X4 | 7.34× | 3.47× | 1.86× |

Top-4 的 FIR 从第一版标量约 0.77–0.82 ms 下降到约 0.18 ms。

当前建议：

```text
高吞吐优先：cir_rake_top_k = 4
保留更多多径能量：cir_rake_top_k = 8
最高兼容性/参考路径：cir_rake_top_k = 0
```

## 8. 正确性与鲁棒性验证

新增/更新 QA 覆盖：

- Timing VOLK 与标量参考一致性；
- sparse Top-K 标量内核与参考一致性；
- Top4X4 与标量 Top-4 四输出逐点对照；
- Top8X4 与标量 Top-8 四输出逐点对照；
- 随机 complex 输入和 complex CIR 权重；
- 非对齐输入地址；
- Top-4、Top-8 clean golden 完整 PHR/payload/FCS；
- 边界尾部标量 fallback。

最终 CTest：

```text
8/8 passed
```

Top-4 鲁棒性小样本结果：

| 项目 | 结果 |
|---|---:|
| AWGN 10–30 dB | 全部 100% FCS pass |
| CFO ±1/±5/±10/±25/±50 kHz | 全部 100% FCS pass |
| Multipath gain×delay | 20/20 pass |
| Collision | 3/3 pass |

Python 五参数构造接口通过；GRC YAML 参数解析通过。完整 `grcc` 在当前环境中受只读 `~/.cache/grc_gnuradio` 限制，未作为本轮功能失败处理。

## 9. 修改文件

- `gr-uwb/include/gnuradio/uwb/uwb_demod_core.h`
- `gr-uwb/include/gnuradio/uwb/uwb_cir_fir_simd.h`
- `gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h`
- `gr-uwb/include/gnuradio/uwb/uwb_realtime_demodulator.h`
- `gr-uwb/lib/uwb_realtime_demodulator.cc`
- `gr-uwb/lib/qa_uwb_demod_core.cc`
- `gr-uwb/python/uwb/bindings/python_bindings.cc`
- `gr-uwb/grc/uwb_realtime_demodulator.block.yml`
- `gr-uwb/apps/benchmark_detector.cc`

## 10. 后续优化方向

Top-4 下 soft FIR 已降至约 182 µs，不再是 CIR 最大子热点。CIR 后处理约 240–260 µs，成为下一目标：

1. AVX2 向量化 complex rotation、real extraction 和 max-abs；
2. 避免旋转后的 `scratch.corr` 全量回写；
3. 在得到 phase gain 后，让后续 FIR 直接输出 real soft chips；
4. 按 PHR 解出的实际 PSDU 长度分段生成 soft chips，避免短包按 127-byte 上限计算；
5. 在真实 LOS/NLOS 捕获上比较 Full-38、Top-8、Top-4 的 PER/FCS，而不仅是 clean golden 与合成多径。

所有进一步算法修改仍需以 `UWB_demodulation/` MATLAB 实现和 `testdata/` 已知信号为正确性基准。
