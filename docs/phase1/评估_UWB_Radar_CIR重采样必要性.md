# 评估：UWB Radar CIR 是否必须从 737.28 重采样到 998.4 MS/s

> 日期：2026-09-04
>
> 结论：**物理上不必须，现有 Phase-1 工程上建议保留。**

---

## 1. 结论

737.28 MS/s 的复数 IQ 可以直接估计 CIR。将其插值到 998.4 MS/s 不会
创造新的射频信息，也不会把物理距离分辨率提高为 0.15 m。

但当前 MATLAB `estimateCir.m` 和 C++ `stage_cir_softchips` 依赖 998.4 MS/s
的整数码片网格：

```text
SamplesPerPulse = 2
samples_per_symbol = 1016
sampled_code = 508 chips × 2 samples/chip
```

因此：

- **不重采样仍能算 CIR**；
- **不能删掉 65/48 后原样调用现有 `estimateCir`**；
- native CIR 需要一套新的分数码片/相位感知相关或已知 TX 波形
  信道估计实现，以及新的 MATLAB golden。

对当前 200 pulse/s 目标，已有 PDU 65/48 在 152k native 样点/PDU、
`realtime_minorder` 下测得约 575 PDU/s，已有约 2.9 倍余量。
Radar RX 窗约 75 µs，即约 55.3k native 样点，明显短于该吞吐测试窗。
所以在尚无证据表明 65/48 是瓶颈时，Phase-1 保留它风险最低。

---

## 2. 信息带宽与距离树格

### 2.1 737.28 MS/s 是否足够

737.28 MS/s 复数采样可表示约 737.28 MHz 总带宽（基带
`-368.64…+368.64 MHz`），能覆盖约 499.2 MHz UWB 占用带宽。
因此在前端滤波和采样契约正确的前提下，native IQ 已保留 CIR
所需的带内信息。

### 2.2 tap 树格不等于物理分辨率

```text
737.28 MS/s: c/(2fs) = 0.203310 m/tap
998.4  MS/s: c/(2fs) = 0.150136 m/tap
```

998.4 域的 tap 更密，但它们是由 737.28 IQ 插值得到的相关样点。
物理距离分辨率主要由有效信号带宽决定；若按 499.2 MHz 名义带宽，
`c/(2B)` 约为 0.30 m。因此 `range_m_per_tap` 只能称为距离树格间隔。

---

## 3. native 域的主要困难

### 3.1 码片和 SYNC 周期都不是整数样点

```text
samples/chip   = 737.28 / 499.2 = 96/65 = 1.476923...
samples/SYNC   = 1016 × 48/65 = 750.276923...
```

每个 SYNC 起点在 native 采样格上的分数相位持续变化。其中
`0.276923... = 18/65`，且 18 与 65 互质，因此连续 65 个 SYNC
会遍历所有 65 个分数相位。64-SYNC packet 已几乎遍历完整相位集。

这意味着：

- 不能把每个 repetition 都当作同一个 751 点模板对齐平均；
- 不能在 native 域直接构造“每 2 样点一个 chip”的 `sampled_code`；
- 必须保留 rational phase，或将各 repetition 映射到一个统一的
  分数时间网格。

### 3.2 MATLAB 逐 tap 契约会变更

native CIR 的 tap 间隔、tap 数、群时延和相关参考都与现有
998.4 `estimateCir.m` 不同。可以做物理等价对照，但不再能声明
两者原生数组“逐 tap 相等”。必须先统一到同一 delay axis 后再比较。

---

## 4. 三条实现路线

### A. 整个 RX PDU 做 65/48（Phase-1 推荐）

```text
SC16 @737.28
  → PDU 65/48
  → SFD + existing-grid CIR @998.4
```

优点：

- 直接复用 MATLAB `estimateCir` 和现有 C++ code geometry；
- SFD/SYNC/tap 都在整数样点网格附近；
- 现有 65/48 core 已有 golden、群时延和 PDU QA；
- 当前吞吐已高于 200 pulse/s 需求。

代价：多一次 SC16→CF32、FIR、PDU 分配/拷贝和坐标映射。

### B. native 分数相位 CIR（可行，但是新算法）

可选实现：

1. 为 65 个 rational phases 预生成对应的 SYNC/code 参考，每个
   repetition 使用自己的 phase。
2. 直接用已知 native TX 波形建立 `y=Xh+n` 局部信道估计，
   解 74 tap 左右的 regularized least-squares/频域估计。

优点：不产生 998.4 PDU，样点和 PMT 数据量更小。

代价：新的数值契约、相位表/求解器、MATLAB native golden、更复杂的边界与
群时延验证，且输出不再与现有 `estimateCir` 逐 tap 直接一致。

### C. hybrid：native SFD + 局部重采样

```text
native SFD template 窄窗搜索
  → 只对 CIR 需要的 SYNC 局部做分数对齐/重采样
  → existing CIR @998.4
```

该方案能减少部分 SFD 处理数据，但当前 RX 窗主要就是 SYNC+SFD，
可节省的数据比例有限。分数对齐仍然需要 polyphase 逻辑，实现复杂度
介于 A/B 之间。

---

## 5. Phase-1 决策

Phase-1 继续使用路线 A，理由是算法可验证性而不是物理必要性。

但在实现 GNU Radio Radar blocks 前增加一个小型对照：

1. 使用同一 native loopback packet。
2. 路线 A 生成 998.4 CIR，作为 Phase-1 基准。
3. MATLAB 实现最小 native phase-aware 原型，生成 737.28 CIR。
4. 把两条 CIR 映射到同一 delay axis，比较峰位、复增益、主瓣和多径。
5. 记录 C++ PDU 65/48 在实际 75 µs 窗上的 mean/P95/P99。

只有同时满足以下条件，才将 native CIR 提升为生产候选：

- 峰位和复数多径结果在统一 delay axis 上达到约定容差；
- native 算法具有独立 MATLAB golden 和全部 65-phase/边界 QA；
- 不重采样带来显著端到端收益，而不只是微基准中减少一个 FIR。

在达到这些条件之前，禁止为了“省一次重采样”破坏 MATLAB
逐 tap 对照和已有 65/48 回归基线。
