# M2-A G0：接口与数字坐标（冻结稿）

日期：2026-10-03。依据：[M2-A 任务单](OpenCode开发指示_M2-A_native_PHY闭环.md)（下称"任务单"）。
性质：**动手前冻结的接口/坐标/容差契约**，由 reviewer 先审；本文件不是实现或验收报告。

状态：**待 reviewer 审核**（任务单 §5：reviewer 同意后才修改主链）。

---

## 0. 结论与范围

本阶段做 **M2-A：完整 TWR 帧在两种 native rate 上的离线 PHY 闭环**。

```text
Frame v1（真实 Poll/Response/Final）
  -> codec 产生 MAC bytes（无 FCS）
  -> HRP 层 append_ieee_fcs 追加唯一 2 B FCS
  -> modulate_one -> work IQ @ 998.4 MS/s
  -> C++ TX resampler（work->native：48/65 或 32/65）
  -> native CF32，可选 CF32->SC16->CF32 量化往返
  -> C++ RX resampler（native->work：65/48 或 65/32）
  -> demodulate_one -> PHR/PSDU/FCS
  -> codec decode -> 与输入逐字节比较
```

**本阶段不能据此证明**：首径 ToA、硬件时间戳、测距有效性、商用互通。

### 0.1 P0 环境事实（本轮实测）

| 项 | 实测 |
|---|---|
| 仓库 HEAD | `4edf6c2`，与远端同步，已跟踪工作区干净（仅 `开发状态.md` 有用户自己的未提交注记，保留不动） |
| 基线 | `uwb_twr_core.cc` SHA256 `f378f441…f4a9`，与 M1-B 整改报告 §9 一致 |
| **MATLAB** | **本机不可用**：`/usr/local/MATLAB/R2024a` 无 `bin/matlab` 启动器与主二进制，无 `matlab`/`matlab.exe` 在 PATH，无 `/mnt/c`（非 WSL），无 Octave/MCR |
| 纯核心 QA `ldd` | 无 gnuradio / uhd / volk 依赖 |
| 复用入口 | `uwb_hrp_mod_core.h`、`uwb_rational_resampler_core.h`、`uwb_demod_core.h`、`UWB_demodulation/decode_uwb.m`、`+uwbdecoder/buildUwbReference.m`、`+uwbdecoder/ieee802154CRC16.m` 均存在 |

**MATLAB 后果（任务单 §6）**：本机只能完成 C++/QA/脚本部分。完成判定中
「MATLAB 核逐样点及双向完整帧交叉实跑」**保持未满足**，全程标注
**「M2-A 未完成，MATLAB 待实跑」**，且**不得**用 Python/SciPy 冒充 MATLAB 验证。
`.m` 文件存在不算通过。

---

## 1. 冻结的 profile 与容量

| 参数 | 冻结值 | 依据 |
|---|---|---|
| work 采样率 | `998.4e6` | `kQm35SampleRate` / `kTwrWorkSampleRateHz` |
| work 每符号样点 | `1016` | `kTwrWorkSamplesPerSymbol` |
| native rate | `737.28e6`（65/48）与 `491.52e6`（65/32），**两者都是必选验收路径** | `kTwrNativeRateUc200Hz` / `kTwrNativeRateCg400Hz` |
| code_index | `9` | 任务单 §3 |
| sync_repetitions | `64` | 任务单 §3（16 SYNC 不阻塞最小交付） |
| SFD | `"ieee"`（标准短 SFD，8 symbols） | 任务单 §3；`HrpModConfig::sfd_mode` 默认值 |
| PHY | BPRF 62.4 MHz 平均 PRF，channel 5 元数据，无 STS | 任务单 §3 |
| PHR | 标准 0.85 Mb/s（21 symbols，512 chips/sym） | `encode_phr19` 固定，无 config 字段 |
| payload | 6.81 Mb/s（64 chips/sym，scrambler offset 1344） | `stage_payload_fcs` 固定 |
| 帧 | 本项目 frame v1；**不是** DW SDK 帧 | `uwb_twr_frame.h` |
| IQ 格式 | native **CF32** 为主；另测 **CF32->SC16->CF32** | 任务单 §3 |
| pulse shaping | `PulseShape::Legacy`（静态 `kPulse48`，`pulse_tail_extra = 0`） | 冻结：最小 profile 不引入 Gaussian/Blackman/External 的额外 tail |
| `peak_amplitude` | `0.8f`（库默认） | `kDefaultPeakAmplitude` |
| `ranging` | `true`（Poll/Response/Final 都置 PHR ranging bit） | REQ-PROTO-02/03。**会改变波形**（reviewer B1）：PHR bit 9 经 `encode_phr19`→SECDED→卷积→BPM 改变 PHR 符号与整段 IQ，且 MAC flags 的 ranging 位（`uwb_twr_frame.h`）也改变帧字节。**ranging 计入冻结配置与 `input_sha256`/`output_sha256`** |

### 1.1 帧字节与 PSDU 容量

| 帧 | MAC bytes | +2 B FCS = PSDU | 波形样点（64 SYNC, ieee, Legacy） |
|---|---|---|---|
| Poll | 14 | **16** | 64·1016 + 8·1016 + 21504 + nsym·128 |
| Response | 24 | **26** | 同上，nsym 视 PSDU 而定 |
| Final | 29 | **31** | 同上 |

`nsym = 8·psdu_bytes + 48·ceil(8·psdu_bytes/330)`；波形长度公式来自
`packet_samples_998p4()`。**PSDU 字节数不是整段空口波形长度**；波形长度须由该函数给出，
含 pulse tail（Legacy 为 0）。

另以 **127 B PSDU** 验证 PHY 容量边界；它**不是**新增 TWR 帧类型。

### 1.2 冻结的既有资产与 hash

**已存在的冻结资产现在就记 hash**（任务单 §3 要求"冻结系数、增益、长度和 hash"）：

| 资产 | 冻结内容 | sha256（本 revision） |
|---|---|---|
| SFD 序列 `kSfdIeee` | `"ieee"`，8 symbols | `sha256(uwb_phy_profile.h) = 709a5296aae2be6c564f4378407010e0d38d899b28521417fc9b3a517f2f04d4` |
| 脉冲系数 `kPulse48` | 48 taps，Legacy，无归一化，center=2 | `sha256(uwb_hrp_mod_core.h) = 636aec1326d9d5fdebf619dd07a6e269ec246c19f5b6cdf144945924e0568912` |
| 解调核 | 全 PHR/payload/FCS 路径 | `sha256(uwb_demod_core.h) = f1dbf344af9e09cff7561617f32893b4c04c30007986b217ee7e6945654f48fc` |
| 重采样核 | 因果 upfirdn 契约 | `sha256(uwb_rational_resampler_core.h) = d0b9af704164a5c6e1f8d3451dfcda065e7af4f173605bf66803cfcd9045e1d4`（**2026-10-06 更正**：本文件在 `985bbc9` 因 M2-A 的 `<48,65>`/`<32,65>` 别名与 `Decim>Interp` 修复而改变；评审时的 `006901ef…` 已过时） |

**新产物**（TX taps、短 golden、manifest）的 hash 由实现报告锁定（G0 不能预先冻结尚未生成
的二进制）；所有产物必须记录 `sha256` 与生成命令。

### 1.3 波形样点数（64 SYNC + ieee + Legacy，`pulse_tail_extra=0`）

`nsym = 8·psdu + 48·ceil(8·psdu/330)`；`n_work = 64·1016 + 8·1016 + 21504 + nsym·128`
（= `94656 + nsym·128`）。**下表是数值表，A08/A13 直接引用**：

| 帧 | MAC B | PSDU B | nsym | n_work | n_native（≈，实际含 T） |
|---|---|---|---|---|---|
| Poll | 14 | 16 | 176 | **117184** | `ceil(((N-1)·48 + T)/65)` |
| Response | 24 | 26 | 256 | **127424** | 同上 |
| Final | 29 | 31 | 296 | **132544** | 同上 |
| 容量边界 | 125 | **127** | 1208 | **249280** | 同上（与既有 QA golden 一致） |

native 长度必须由 §3.1 的长度公式算出（含 taps 长度 `T`），**不得**用 `N·L/M` 近似。
PHR 自身符号率是 0.85 Mb/s；`encode_phr19` 写入 PHR 的**速率字段是 6.81 Mb/s（index 2）**，
二者不要混用。

---

## 2. 形态与所有权

- 新增 **无 scheduler 的 C++ helper**（header + `.cc`），复用现有 core；允许 VOLK 依赖
  （任务单 §5.1）。**不新增 GNU Radio block**。不得依赖 radio/PMT 执行核测试。
- 分发形态（协调者所有）：
  - 公开头 `gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h`：`TxPlan`/帧/PHY 参数与
    `uwb_twr_phy.cc` 的入口；加入安装列表。
  - 实现 `gr-uwb/lib/uwb_twr_phy.cc`。
  - 静态库目标 `uwb_twr_phy`（镜像 `uwb_twr_core` 的先例），`POSITION_INDEPENDENT_CODE ON`；
    若它需要 `gnuradio-uwb`（demod/resampler 头），则**不作为独立安装静态库**，
    改由 `GR_ADD_CPP_TEST` 覆盖并由安装消费者直接 `#include` 头验证。
    **G0 决定**：`uwb_twr_phy.cc` 依赖 `gr-uwb/include` 下的 header-only core
    （hrp_mod / resampler / demod 都是 header-only），因此**可以**做成独立静态库
    `uwb_twr_phy`，链接 `VOLK` 与 `Boost` 之外无 GNU Radio 运行时依赖——但**必须由
    `ldd` 实测**确认；若实测带出 `libgnuradio-*`，则退回"仅安装头 + 消费者测试"。

**B2 关闭（reviewer 阻断项）——安装依赖头必须先补齐。** 事实：
`uwb_hrp_mod_core.h`（**已在安装列表**）传递 include `uwb_demod_core.h`，后者又 include
`uwb_demod_result.h`、`uwb_detector_core.h`、`uwb_phy_profile.h`、`uwb_cir_fir_simd.h`。
当前安装列表**缺** `uwb_demod_core.h`、`uwb_demod_result.h`、`uwb_phy_profile.h`、
`uwb_cir_fir_simd.h`（`uwb_detector_core.h`/`uwb_defaults.h`/`uwb_radar_pdu_meta.h`
已安装）。因此**现有已安装的 `uwb_hrp_mod_core.h` 对隔离消费者已经是坏的**——这是本阶段
暴露的既有缺口，不是 M2-A 新引入的。

**G0 决定（选 (a)）**：协调者把上述 4 个头加入
`gr-uwb/include/gnuradio/uwb/CMakeLists.txt` 的安装列表，并让安装消费者**真正编译**
`#include <gnuradio/uwb/uwb_twr_phy.h>` 并调用一个 M2-A 入口（不是只查文件是否存在）。
若 `ldd` 实测带出 `libgnuradio-*`，退回方案 **(b)**：`uwb_twr_phy.h` 不暴露内部头
（opaque/PIMPL），消费者只依赖已安装头——**退回方案同样必须满足本条的编译验证**。
- 逐帧路径：显式输入长度与输出容量，**有界失败**；失败时输出长度置 0 且不标有效。
- 复用隔离：每个 worker/每端持有自己的 scratch；一个 scratch 不得并发使用
  （`modulate_one` 会改 scratch；`RationalResamplerLmCore` 有可变状态）。
- 不可变执行快照：帧字节、FCS、调制配置、RX profile、taps 来自**同一份** M2-A 配置结构，
  不散落魔法常量。

---

## 3. 重采样契约（任务单 §5.2）

### 3.1 策略：因果全卷积（upfirdn），不是零相位裁剪

**冻结为现有 core 的因果 upfirdn 长度契约**，与
`gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h:14-16` 一致：

```text
Lout = ceil(((N-1)*L + T) / M)
y[m] = sum_k h[(m*M mod L) + L*k] * x[floor(m*M/L) - k],  x[j] = 0 outside [0,N)
```

- `L = Interp`，`M = Decim`，`T` = 原型滤波器长度。
- **不使用** `resample_poly` 的中心裁剪契约，也不做零相位对齐。两种策略不得混用。
- 长度与索引算术用**无符号/带溢出检查**（任务单 §5.2）；核心已用 `int64_t` 计算并
  在 `num<=0` 时返回 0。

### 3.2 N=0 定义（冻结，覆盖核心的默认行为）

任务单要求 `N=0` 单独定义。**M2-A helper 层面**：

- `process(N=0)` 定义输出为 **0**，**不改变**任何滤波器状态（不推进相位、不写历史）。
- helper **不得**把 N=0 传给核心（核心对 `T>L` 的 N=0 会给出非零瞬态样点）。
- `flush()` 在无输入时同样产出 0。

这是 **fail-closed**：空输入不得凭空产出瞬态样点。

### 3.3 方向、增益与 taps

| 方向 | 模板 | Interp/Decim | taps DC 和（=Interp；有效增益 1） | 输入率 | 输出率 |
|---|---|---|---|---|---|
| TX（work→native, UC200） | `RationalResamplerLmCore<48,65>`（**新别名**） | 48/65 | 48 | 998.4e6 | 737.28e6 |
| TX（work→native, CG400） | `RationalResamplerLmCore<32,65>`（**新别名**） | 32/65 | 32 | 998.4e6 | 491.52e6 |
| RX（native→work, UC200） | `RationalResamplerLmCore<65,48>`（既有别名） | 65/48 | 65 | 737.28e6 | 998.4e6 |
| RX（native→work, CG400） | `RationalResamplerLmCore<65,32>`（既有别名） | 65/32 | 65 | 491.52e6 | 998.4e6 |

- 只有 `<65,48>`/`<65,32>` 已实例化；`<48,65>`/`<32,65>` **模板合法但尚无别名**，
  只加别名与验证，**不另写一套重采样**（任务单 §4）。
- **增益**：taps 的 DC 和必须等于 `Interp`（core 不做任何缩放）。
- **TX taps 的取得方式（冻结原则）**：对 `<L,M>`，其归一化截止为 `1/(2·max(L,M))`，
  因而不带裁剪的 `<48,65>` 与 `<65,48>` **截止相同**、只差通带增益。因此 TX taps
  **可以**取 RX 侧同一原型并按 `48/65`（resp. `32/65`）重标定到 DC=48（resp. 32）。
  但任务单 §4 明令"反向使用须重新证明增益、带宽和长度"：实现者必须
  1) 用**独立设计脚本**（`testdata/twr/m2a/design_m2a_tx_taps.py`，仿
     `testdata/design_resampler_minorder.py`）按 Interp/Decim 与通带/阻带重算一份，
     2) 证明其 DC 和 = Interp，截止与阻带满足冻结指标，3) 与"重标定 RX 原型"逐样点
     比较并在报告记录差异。**不得盲目反转比例或用旧 CSV 冒充。**
- taps 为 **float32 LE 二进制**，放在 `testdata/twr/m2a/`，附 `design.json`
  （Interp/Decim/通带/阻带/阶数/DC 和）与 sha256。

### 3.4 相位、padding、flush、reset、有效区间

- **初始相位**：0（构造后与 `reset()` 后，`ctr_=0`）。
- **padding**：两侧隐式零填充；无显式 crop/trim。
- **flush**：所有输入消费完后调用 `flush(out, cap)`，可重复调用直到 `flush_complete()`；
  重复调用续写、不重发。
- **reset**：独立 burst 之间**必须** `reset()`，清除 `hist_`、相位、held sample、计数器与
  `flush_mode_`；否则会串帧泄漏滤波历史，且 `flush()` 后未 `reset()` 再 `process()` 会
  抛 `std::logic_error`。
- **有效区间**：`Lout` 全段有效（含暖机瞬态与 EOS 尾）；数字群时延按 §5 记录，**不作为**
  硬件校准常量。

### 3.5 容量与分配（B5 关闭）

- helper 必须显式给出 `out_cap` 并校验 `Lout <= out_cap`，否则有界失败。
- **冻结的单帧上界**：`N_work <= 2^20`（1048576）且 `N_native <= 2^20`。
  依据：64 SYNC + 127 B 的 work 波形是 **249280** 样点（§1.3），native 侧更小，
  均远低于 `2^20`；**不使用** `kMaxHrpTxSamples = 2^22` 作为容量上界。
- 核心 ctor 预留 `H_ + 2^20` 复杂样点与两块 `2^20` scratch，`process` 仅在
  `(H_-1)+N > H_+2^20` 即 `N > 2^20+1` 时 reserve。因此冻结 `N <= 2^20` 即可保证
  **逐帧 `process()`/`flush()` 不再 reserve**；核心**没有公开 reserve**，所以
  "准备阶段预热"= 在 prepare 时对 `2^20` 上限做一次 dummy `process`+`reset`，
  使首帧不承担冷分配。若实测仍见 `new`，**如实报告**并撤下"零分配"表述。
- **判定以实测为准**：用重载全局 `new/delete` 计数（同 M1-B §4 的做法），
  冷（prepare 后首帧）/热（后续帧）分开报告；不得用"上界声明"推导零分配。

---

## 4. 帧 / FCS / PSDU 契约

- **FCS 唯一归属**：由 HRP 层 `mod::append_ieee_fcs()` 追加一次（CRC-16，反射多项式
  `0x8408`，初值 0，小端追加）。codec **不写** FCS（`frame_profile_validate()` 拒绝
  `fcs_appended_by_modulation_layer=false`）。
- `modulate_one` 的输入是**完整 PSDU（含 FCS）**；它**不**追回 FCS。
- **PSDU = MAC + 2**，因此 Poll/Response/Final 为 16/26/31 B。
- `decode()` **只接受 MAC payload（无 FCS）**；带 FCS 的缓冲按 `LengthMismatch` 拒绝。
  闭环的最后一步必须先用 `mac_payload_from_psdu()` 剥掉 2 B，再 `decode()`。
- **A02 判据**：唯一追加；重复 FCS / 缺失 FCS / 畸形长度必须被拒，绝不能被误报为合法原始帧。

---

## 5. 数字坐标契约（任务单 §5.3）

每个阶段记录：`rate、0-based origin、input/output count、L/M、phase、trim、padding、
filter delay、有效区间`。

- 索引映射用**整数/有理数**，禁止整数 ns 截断。
- 冻结映射（与既有 core 代码一致，非 spec 文档里那条带负号的写法）：

  ```text
  map_input_offset_to_output(p) = llround((p*L + 0.5*(T-1)) / M)   , clamped to >= 0
  ```

  （**E3 修正**：必须是精确的 `0.5*(T-1)`，不是整数除 `(T-1)/2`；对偶数 `T` 二者
  在 `.5` 处相差 1 个输出样点，而 `llround` 对 `.5` 敏感。）

  **依据代码与 QA**（`uwb_rational_resampler_core.h` 与其 QA），不采用
  `docs/performance/规格_固定65_48重采样core契约.md` 中 `ceil((p·L − d)/M)` 的写法——
  该文档与代码/QA 矛盾，本轮以实现为准并在报告记录该矛盾。
- 群时延 `0.5*(T-1)` 出现在 `p*L` 所在的**内插（上采样）域**，换算到输入端为
  `(T-1)/(2L)` 输入样点。**单位必须无歧义**（A05 的"可逆/可解释"依赖它），并与
  pulse shaping 延迟**分别记录**；两者都**不是**硬件校准常量。
- 这些字段是 **sample-grid bookkeeping，不是协议 RMARKER/天线参考面**：
  - 不产生 `HardwareMeasured` RX 时间；
  - 不填虚假 `first-path passed`；
  - 不给 M1-B admission 加 bypass；
  - 解码得到的 SFD/packet-start 只作**诊断**。
- **不要求** native 往返后波形等于原 work 波形（限带必然改变波形）；要求**字节正确**
  且与"同一冻结数字链的独立 oracle"一致。

---

## 6. 比较器与容差（任务单 §6，冻结）

| 对象 | 判据 |
|---|---|
| bytes / FCS / 长度 / 整数坐标 | **精确一致** |
| 同链 CF32（固定归一化输入） | `max_abs_error <= 1e-4`，`relative_L2 <= 1e-5`；全零输入单独定义（要求逐样点 `==0`） |
| packet-start / SFD 对照（统一定义后） | 干净条件下默认 **<= 1 work sample**；不构成分数 ToA 精度承诺 |
| 跨 pulse 实现（工具箱 waveform vs 仓库 pulse） | **不要求**逐样点相等，但**解码仍须 bytes 精确** |

**B3 关闭（reviewer 阻断项）——回退候选不得当成功。** `stage_payload_fcs` 在生产路径
FCS 不过时会继续跑 **±2 长度邻域 × 正反 SFD 极性 × 软 Viterbi** 的候选回退，且可能
`ok=true` 而 `fcs_pass=false`。因此 M2-A 闭环的**唯一成功判据**冻结为：

```text
success  <=>  status == DemodStatus::Success
          AND payload.fcs_pass == true
          AND payload.bytes.size() == 期望 PSDU 长度
          AND payload.bytes 与输入 PSDU 逐字节相等
```

**任何**回退候选命中、`FcsFailed`、或长度不等于期望值，一律判**失败**并计入统计。
A10 的"错误 FCS 但编码合法"负例必须在发端构造（FEC 可能纠错，不能假设任意 IQ bit flip
都导致 FCS 失败）。

容差调整规则：**必须先用独立参考说明原因、更新 G0、经复核同意**；禁止看到失败后
自动放大阈值或逐例重新对齐。

---

## 7. SC16 量化契约（A06）

冻结 CF32->SC16->CF32：

```text
S      = 32767.0 / peak_amplitude          # peak_amplitude = 0.8 -> S = 40958.75
q      = clamp(llround(x * S), -32768, 32767)
x'     = q / S
```

- 舍入为 **round-half-away-from-zero**（`llround`）。
- 饱和**计数**并回报；不把严重削顶默认为高质量。
- **NaN/Inf**：输入即拒绝（`fail-closed`，返回带原因的失败），不静默当选 0。
- 0 / 正负极值 / 半格 / 边界单独测。

---

## 8. 结果与 manifest JSON 契约（`twr-m2a-native/1`）

逐帧记录（协调者冻结 schema，D/E 消费；E 与 C 先冻结输出后再实施）：

```json
{
  "schema": "twr-m2a-native/1",
  "rate": {"work_hz": 998400000.0, "native_hz": 737280000.0, "tx_l": 48, "tx_m": 65,
           "rx_l": 65, "rx_m": 48, "taps_sha256": "..."},
  "frame": {"type": "poll|response|final", "mac_bytes": 14, "psdu_bytes": 16,
            "mac_hex": "...", "fcs": "0xdd53", "fcs_hex": "..."},
  "stages": [
    {"name": "hrp_mod", "rate_hz": 998400000.0, "l": 1, "m": 1, "origin": 0,
     "in_count": 16, "out_count": 117184, "phase": 0, "trim": 0, "padding": 0,
     "filter_delay": 0.0, "valid_from": 0, "valid_to": 117184},
    {"name": "tx_resample", "rate_hz": 737280000.0, "l": 48, "m": 65, "origin": 0,
     "in_count": 117184, "out_count": 0, "phase": 0, "trim": 0, "padding": 0,
     "filter_delay": 0.0, "valid_from": 0, "valid_to": 0},
    {"name": "rx_resample", "rate_hz": 998400000.0, "l": 65, "m": 48, "origin": 0,
     "in_count": 0, "out_count": 0, "phase": 0, "trim": 0, "padding": 0,
     "filter_delay": 0.0, "valid_from": 0, "valid_to": 0},
    {"name": "demod", "rate_hz": 998400000.0, "l": 1, "m": 1, "origin": 0,
     "in_count": 0, "out_count": 0, "phase": 0, "trim": 0, "padding": 0,
     "filter_delay": 0.0, "valid_from": 0, "valid_to": 0}
  ],
  "samples": {"work_tx": 0, "native": 0, "work_rx": 0, "returned": 0},
  "filter": {"tx_taps": 0, "rx_taps": 0, "scale": 0.0, "phase": 0, "crop": 0,
             "group_delay": 0.0, "valid_from": 0, "valid_to": 0},
  "decode": {"status": "success|fcs_failed|timing_failed|...", "phr_psdu_length": 0,
             "payload_hex": "...", "fcs_pass": true,
             "diagnostic": {"sfd_start_sample": -1, "packet_start_sample": -1}},
  "compare": {"bytes_exact": true, "max_abs_error": 0.0, "relative_l2": 0.0},
  "seed": 0,
  "provenance": {"revision": "...", "dirty": false, "compiler": "...", "volk": "...",
                 "loaded_library": "...", "input_sha256": "...", "output_sha256": "...",
                 "matlab": {"executed": false, "version": null, "command": null,
                            "exit_code": null}},
  "evidence": {"level": "native_roundtrip_verified", "kind": "measured",
               "scope": "m2a-native-roundtrip/1",
               "hardware_readback": null, "measurement_valid": false},
  "config": {"requested": {}, "effective": {}, "executed_sha256": "..."}
}
```

- **B4 关闭（reviewer 阻断项）**：§5 要求的每阶段
  `rate、0-based origin、input/output count、L/M、phase、trim、padding、filter delay、
  有效区间` 现在由 `stages[]` 逐阶段承载（`hrp_mod` / `tx_resample` / `rx_resample` /
  `demod`），独立 verifier 据此核验坐标，而不是只看总数。
- `measurement_valid` 恒为 `false`；`yields_range` 不得出现为真。
- `hardware_readback` 恒为 `null / not_measured`，不伪装设备读回。
- `matlab.executed=false` 时不得声称 MATLAB 验证。
- `evidence.level` 的**示例值**表示"已实际实测的 M2-A artifact"；**未实测**时该字段取
  `{"level": "work_decode_verified", "kind": "not_measured", "scope": "m2a-native-roundtrip/1"}`
  且 `compare` 全部为失败。示例不等于默认。

---

## 9. capability evidence 边界（任务单 §8）

- 默认输出只是 `decode-only / simulation`，`measurement_valid=false`，不输出可消费距离。
- 最多对**本阶段实际实测**的 `rate × profile × shaping × taps × 格式` 升级到
  `native_roundtrip_verified`（`Level=2`）。
- **`allows(NativeRoundtripVerified, Ranging) == false`**（Ranging 需 `ToaVerified`）——
  这是本阶段的硬边界。ToA / hardware / vendor 保持 false。
- **不得**改动 `build_default_capabilities()` 的 48 行（它们恒为 `WorkDecodeVerified`）；
  Python R7 测试钉死这 48 行。
- **不得**走 `derive_row_evidence()`（它会把任何 `native_*` 路径降级为 `None`，
  见 `uwb_twr_capability_evidence.h`）。M2-A 用**独立、版本化**的窄作用域 artifact：
  - 文件名形如 `testdata/twr/m2a/native_roundtrip_<profile>_<rate>.json`；
  - 行用显式 `RowEvidence{established=NativeRoundtripVerified, kind=Measured,
    source=<artifact>, provenance_id=<run>}` 构造；
  - 带 `scope="m2a-native-roundtrip/1"` 与 `reject_scope` 说明；
  - 旧 48 行白名单**保持不动**。
- 升级**逐项**判定，不接受"路径名含 native / 配置里有 rate 字段 / CLI 退出 0 / 复用旧
  work CSV"作为证据。

---

## 10. CLI 契约（最小离线）

`apps/twr_native_phy_demo`（C++，无 GNU Radio 运行时）：

```text
twr_native_phy_demo --frame poll|response|final
                    --native-rate 737280000|491520000
                    --iq cf32|sc16
                    [--seed N] [--scenario clean|cfo|awgn|delay|multipath]
                    [--config PATH]          # 复用既有 TwrConfig 解析/校验（若可用）
                    --output PATH.json
```

- 退出码：0 = 闭环且字节精确；非 0 = 失败并给出原因。
- `--config` 若引入，必须复用既有 parser/validator；本阶段**不新增** Python schema/FSM。
- 若实现中发现 `--config` 闭环成本过高，**G0 允许**先以冻结的 profile 常量实现，
  但必须在报告写明"未消费 TwrConfig"，且不得勾选"配置链路"完成项（A12 部分留待）。

---

## 11. MATLAB oracle 契约与阻塞

任务单 §6 的三类对照（**完成硬门槛**）：

1. 相同输入/系数的 MATLAB `upfirdn`（独立多相参考）与 C++ 重采样逐样点比较。
2. C++ 生成并过 native 链的 IQ，由 `decode_uwb` 实际解码，比较 PHR/PSDU/FCS 与诊断坐标，
   **不捕获异常后写 PASS**。
3. MATLAB 独立生成的标准最小 profile TWR 帧，经 native 链后用 C++ 解码。

**本机阻塞**：无可用 MATLAB（§0.1）。因此：

- 交付 `.m` 脚本与 `testdata/twr/m2a/` 的 manifest/README，但**不实跑**；
- 所有相关完成项标 **未满足**，报告写 **「M2-A 未完成，MATLAB 待实跑」**；
- 绝不用 Python/SciPy 替代声明；`.m` 文件存在不算通过；
- 保留"在另一台有 MATLAB 的机器上的 handoff"步骤（同 M1-A README 的做法）。

---

## 12. A01–A14 与 G0 的映射

| ID | 归属 | G0 冻结点 |
|---|---|---|
| A01 | D | 2 rate × 3 帧 × 2 格式 = 12 单元；输入来自真实 codec，至少两组不同 timestamp/地址/seq |
| A02 | D | §4 的 FCS 唯一归属与畸形拒绝 |
| A03 | A/D | §3.3 的 taps 来源、增益、相位、tail |
| A04 | A/D | §3.2/§3.4 的分块/flush/reset |
| A05 | A/D | §5 的坐标映射与多相余数 |
| A06 | D | §7 SC16 |
| A07 | D | fail-closed 参数校验 |
| A08 | D | 127 B / 最小长度 / 空输入 / 坏 PHR / 坏 FCS |
| A09 | D | 随机前置空白、窗口内无包 |
| A10 | D | native 域 CFO/AWGN/分数延迟/多径（G0 固定用例与门槛） |
| A11 | C/F | §11 MATLAB 三类对照 |
| A12 | E | §8 manifest / §9 evidence；变异必须被检出 |
| A13 | A/E | §3.5 分配与延迟 |
| A14 | 协调者 | 回归 + 安装消费（不用 SKIP 代替） |

A10 冻结用例（G0）：`0/±20 kHz` CFO、固定 SNR 的 AWGN（值由 C 与 D 在实现时选定并记录）、
整数与分数延迟、首径弱于后径。干净基线全部通过；压力单元按实际通过范围标注
支持/失败，不静默删测。

---

## 13. 分工与文件所有权（任务单 §9）

| 角色 | 唯一写入 |
|---|---|
| 协调者（本 agent） | 本 G0、`uwb_twr_phy.h`、CMake/安装、capability/config 最小集成、状态文档 |
| A `m2a_native_resampler` | TX resampler helper（`<48,65>`/`<32,65>` 别名）+ 专属 QA；`uwb_rational_resampler_core.h` 必要最小修改 |
| B `m2a_frame_phy` | `lib/uwb_twr_phy.cc`（复用 HRP/demod 的全帧流水线）；不得改 FSM |
| C `m2a_matlab_oracle` | `testdata/twr/m2a/` 的 `.m`、短 golden、manifest/README |
| D `m2a_qa` | `qa_uwb_twr_native_phy.cc` 全帧/故障/分配测试；先补失败 QA |
| E `m2a_demo_verifier` | `apps/twr_native_phy_demo.cc`、轻量 Python 包装、`tools/twr/verify_m2_a_native_phy.py` |
| F `m2a_reviewer` | 只读生产/QA，仅写 `docs/twr/M2-A_评审报告.md`；先审本 G0 |

---

## 14. reviewer 裁决与 B1–B5 关闭

reviewer F 的独立评审：[M2-A_评审报告.md](M2-A_评审报告.md)（对 revision `4edf6c2`）。
总裁决 **APPROVED-WITH-CONDITION**，列出 1 处事实错误与 5 条阻断条件。**逐条关闭如下**
（本版 G0 已写入；B1–B5 均为 G0 文本/契约修正，不改生产代码）：

| ID | reviewer 要求 | 本版落点 | 状态 |
|---|---|---|---|
| B1 | 改正 §1 `ranging` "不影响波形" | §1 表格改为"**会改变波形**"，计入配置与 hash | 已关闭 |
| B2 | 解决传递头安装 | §2 新增"B2 关闭"段：把 4 个缺头加入安装列表并让消费者真编译 | 已关闭 |
| B3 | 钉死 payload 回退路径 | §6 新增唯一成功判据（`Success && fcs_pass && 长度/bytes 精确`） | 已关闭 |
| B4 | §5 坐标字段进 schema | §8 新增 `stages[]` 逐阶段坐标 | 已关闭 |
| B5 | 冻结单帧上限 / 撤下零分配承诺 | §3.5 冻结 `N <= 2^20`，以实测 `new` 计数为准 | 已关闭 |

非阻断项也已处理：E2（"DC 增益"→"taps DC 和（=Interp，有效增益 1）"）、
E3（`0.5*(T-1)` 与群时延单位）、M5（冻结既有资产 hash）、M6（波形样点数值表、PHR 速率澄清）、
§4（`evidence.level` 示例不等于默认）。

reviewer 对 §14 五个未决项的独立意见（**采纳**）：

1. **独立静态库**：有条件同意；先按 B2 解决传递头，再 `ldd`/`readelf` 实测；带出
   `libgnuradio-*` 即退回"仅安装头 + 消费者测试"，退回方案同样要满足 B2。
2. **`--config` 闭环**：同意退化方案；必须写明"未消费 `TwrConfig`"、不勾选 A12 的配置
   链路完成项、未实现时显式拒绝。
3. **TX taps**：同意"两条都算并记录差异"；独立设计脚本、DC 和 = Interp、截止/阻带指标、
   与重标定 RX 原型的逐样点差异、`design.json` + `sha256` 缺一不可；**不得**用旧 CSV
   或盲目反转比例。
4. **§5 文档矛盾**：同意以代码/QA 为准。但 reviewer 保留：spec 文档的
   `m_tag = ceil((p·L − d)/M)` 可能不是同一物理量。**本版要求**：实现者在报告里(a)证明
   二者是否指向同一映射，若同一量则明确"文档已过时并修正"，若不同量则**分别命名**，
   不再笼统称"矛盾"；(b) 按 E3 修正 `0.5*(T-1)` 与群时延单位。
5. **A10 AWGN SNR**：同意推迟，但**必须在 P3 前以 G0 附录冻结**，且 `0/±20 kHz` CFO、
   整数/分数延迟、首径弱于后径、固定 SNR 全部要有可复跑命令与门槛。

**未决（需在实现中关闭，不阻断 P2 开工）**：§14.1 的 `ldd` 实测结论、§14.5 的 SNR 值。
B1–B5 已关闭，可按任务单 §5/§10 进入 P2 并行实现。

---

# 附录 A（2026-10-06 收尾冻结）

依据：[M2-A 收尾指示](OpenCode开发指示_M2-A收尾_2026-10-06.md)（下称"收尾单"）。
本附录在**改动主链前**冻结收尾所需的接口/坐标/损伤/观测契约；reviewer 确认后实施。

## A.1 prepared context 生命周期（收尾单 §4）

新增 **`M2aContext`**（公开头 `uwb_twr_phy.h`，协调者所有），冻结 profile/taps/容量/格式，
持有 TX/RX core、调制 scratch、解调 scratch、解调模板、可复用搜索 workspace 与 IQ 缓冲。

```cpp
class M2aContext {
public:
    // 原子准备：成功后才可用；失败时对象保持"未准备"状态，不半更新。
    bool prepare(const M2aConfig& cfg, std::string& why);
    bool prepared() const;

    // 逐帧：不重新分配 core/scratch；帧失败后可继续下一帧。
    bool run(const Frame& frame, const FrameProfile& profile, M2aResult& out,
             std::string& why);

    // 独立 burst 之间调用：清滤波历史/相位/上一帧 bytes，不重建 core。
    void reset();

    // 冷包装（保持向后兼容）：prepare + run，一次性。
};
bool m2a_native_roundtrip(const Frame&, const FrameProfile&, const M2aConfig&,
                          M2aResult&, std::string&);   // 保留，标记为 cold wrapper
```

规则（冻结）：

1. **不是并发共享对象**：每 worker/端点各自一个实例。
2. `prepare` 失败**原子**：不留下半更新配置；随后 `run` 必须返回失败而不是跑半套。
3. **扩容/配置变更只在 `prepare`**；逐帧路径不得 reserve/resize（除既有 demod 内部临时）。
4. **容量**：所有 guard、tail、损伤滤波长度计入 `kM2aMaxSamples = 2^20`；不得只让原始
   输入满足上界而 padding/搜索缓冲越界。超限**显式拒绝**（`CapacityExceeded`）。
5. 帧失败后**可恢复**；`reset()` 清滤波历史与上一帧 bytes，不得带入。
6. 新 demo/benchmark **必须实际走 prepared 路径**；cold wrapper 只作兼容。
7. **不修改** shared resampler/HRP/demod 的算法；若复用使已知 SIMD 异常更明显，单独报告。

## A.2 stage 坐标契约（收尾单 §6）

- **单位分开**：`codec`/`fcs` 阶段以 **bytes** 计，`modulate`/`tx_resample`/`quantise`/
  `rx_resample`/`demod` 阶段以 **samples** 计。每个 `M2aStageTrace` 必须标明单位。
- **0-based origin**：每阶段记录其输入在**该阶段网格**上的 0-based 起点。
- **padding 分量分开**：`pad_front` 与 `pad_back` 分别记录，**不得**用总和推测前端偏移。
- **搜索窗口**：`m2a_demod_work` 的 guard 以 `search_guard_front` / `search_guard_back` /
  `search_roi = [lo, hi)` 记录；解调诊断坐标必须换算回**未加 guard 的 work_rx 网格**。
- **trim**：因果全卷积，`trim = 0`；如实现另有裁剪必须显式记录区间。
- **filter delay**：`0.5*(T-1)`，位于**内插（上采样）域**；输入端等效 `(T-1)/(2L)`。
  精确有理数与显示用 rounded index **分开**；逆映射只断言定义内可逆/误差界。
- **失败后**：解调失败时诊断字段必须清空/置无效，**不得**保留未换算的 padded 索引冒充输入坐标。
- 这些仍是 **sample-grid bookkeeping**，不是 RMARKER/首径/硬件时间戳。

## A.3 损伤模型与 SNR 定义（收尾单 §5）

损伤只在**测试/离线 harness**，不进入生产 helper 的解码输入真值。

- **冻结顺序**：TX native CF32 → **信道模型** → （可选）RX SC16 → RX resampler → demod。
  若另做 TX DAC 量化，必须作为**独立选项**，不与现有 native SC16 往返语义混用。
- `M2aImpairment` 字段（版本化进 JSON）：`cfo_hz`、`awgn_snr_db`、`awgn_seed`、
  `delay_int_samples`、`delay_frac_num`/`delay_frac_den`、`multipath`（复数抽头数组）、
  `enabled`。
- **CFO**：相位按 **native rate 与样点坐标**推进，跨 chunk **不重置**。
- **SNR 定义**：以**信号有效区间**的平均功率计算，`SNR_dB = 10*log10(P_signal_valid /
  P_noise)`；噪声为复高斯，每维方差 `sigma^2 = P_signal_valid / (2*10^(SNR/10))`。
  **不得**因前置零填充变长而改变同一信号的噪声水平；记录区间、功率与每维方差。
- **固定场景（最小集）**：

  | 场景 | 必测点 |
  |---|---|
  | clean | 现有全部基本单元，输出兼容 |
  | CFO | `0`、`+20 kHz`、`-20 kHz` |
  | AWGN | `30 dB`（常规）、`10 dB`（压力） |
  | 延迟 | 整数 `0/1/17` native samples；分数 `1/4, 1/2, 3/4` sample |
  | 多径 | 至少一组弱首径强后径：复振幅 `0.35` 与 `1.0`，后径 `+8` native samples |
  | 组合 | `CFO+AWGN`、分数延迟+多径 各至少一组 |

- **seed**：含随机项的场景每基本单元固定 **≥10 个 seed**，seed 列表冻结并记录；
  **不得**按跑出的成功种子挑子集。`seed 0` 若特殊必须说明。
- **判据**：clean 与退化路径（零 CFO/零噪声/单位信道）必须与 clean 一致；损伤模型先用
  **独立单位测试**验证（CFO 相位/频偏、AWGN 方差、impulse 延迟、多径系数），不能只靠
  "解调似乎成功"。压力场景允许真实失败，但必须记录终态，
  `attempted = exact_success + explicit_failure`；非精确帧不得计入成功。失败场景**不取得**
  native evidence；clean 成功也不为压力失败背书。
- CLI 原 `--scenario` 的拒绝行为**仅在有真实实现后**替换。

## A.4 A13 观测窗口与准入（收尾单 §4.2）

- **分配计数**：重载全局 `new` / `new[]` / aligned `new` 与 `malloc`；计数器自身不得递归分配。
  分开统计：`prepare`、prepare 后**首帧**、**后续帧**；并分阶段：调制、TX resample、量化、
  RX resample、搜索/解调、结果/JSON 序列化。**未拦截到的分配不宣称为零。**
- **目标**：新 TX/重采样调度与 IQ workspace 热路径零分配；既有 demod 内部临时 vector/
  结果字符串分配**独立统计**，不为"整链零分配"重构 demod core。
- **耗时**：与分配计数**分次运行**；日志/文件 I/O 不计入核心计算时间；端到端 wall time 另列。
- **矩阵**：两 rate × 三帧 × 两格式；相同帧重复与 timestamp/seq/地址变化各一组；
  clean 与长搜索/失败路径**分别**报告，不只测缓存命中的最快路径。
- **样本量**：默认每组 ≥ **1000** 次热调用；报告样本数、P50/P95/P99/max、cold 延迟、
  RSS/峰值内存。样本不足**不报告 P99.9**。重复循环 RSS/容量不得持续增长。
- **对照**：同环境串行比较一次性 wrapper 与 prepared 路径，保留原始 CSV。
  **不**从软件数据推出硬件 reply-delay 下限、1 GS/s 实时率或双 RX 可运行。

## A.5 JSON/schema 扩展（版本 `twr-m2a-native/2`）

在 `twr-m2a-native/1` 基础上**新增**（旧字段保持兼容）：

```json
{
  "schema": "twr-m2a-native/2",
  "impairment": {"enabled": false, "cfo_hz": 0.0, "awgn_snr_db": null,
                 "awgn_seed": null, "delay_int_samples": 0,
                 "delay_frac_num": 0, "delay_frac_den": 1, "multipath": [],
                 "snr_definition": "valid-region mean power"},
  "context": {"prepared": true, "capacity_samples": 1048576},
  "kernel": {"tx": "volk_macroblock", "rx": "volk_macroblock", "demod": "scalar"},
  "stages": [ {"name": "…", "unit": "samples|bytes", "origin": 0,
               "pad_front": 0, "pad_back": 0, "trim": 0,
               "search_guard_front": 0, "search_guard_back": 0,
               "search_roi": [0, 0], "…": "…"} ],
  "observation": {"iters": 1000, "p50_us": 0.0, "p95_us": 0.0, "p99_us": 0.0,
                  "max_us": 0.0, "cold_us": 0.0, "rss_kb": 0,
                  "alloc": {"prepare": 0, "first_frame": 0, "hot_frame": 0,
                            "demod_internal": 0},
                  "counter_scope": "operator new/new[]/aligned/malloc"}
}
```

verifier 必须**变异检查**：`impairment` 参数、`origin`、`pad_front`/`pad_back`、
区间上下界、`unit`、实际 `kernel`、`context.capacity_samples`、`config.executed_*`。
schema 改动版本化；`twr-m2a-native/1` 的消费者按显式向后兼容处理。

## A.6 收尾后允许的结论

最多声明：**"M2-A 本轮非 MATLAB 子项完成；独立 MATLAB 对照未做，SIMD 已知风险保留。"**
A11 按完整口径**仍未满足**；SIMD 缺陷**只记录不修**；**不进入 M2-B**。
