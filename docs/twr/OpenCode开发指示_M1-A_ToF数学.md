# OpenCode 开发指示：M1-A 独立 ToF 数学与 oracle

日期：2026-09-29。对象：OpenCode（本工作区下一阶段的唯一执行说明）。
分支：`feature/uwb-ds-twr`。基线 HEAD 以仓库当时提交为准（N07 修复报告关闭后）。

本文件是 **M1-A 的任务单**，不是总路线。总路线仍是
[开发路线与验收矩阵.md](开发路线与验收矩阵.md) 的 M1→M2→M3/M4。
M1-A 的公式与验收来自 [需求_UWB_SS_DS_TWR.md](需求_UWB_SS_DS_TWR.md)
REQ-PROTO-02 / 03 / 05、REQ-TIME-01 / 04、REQ-QA-01、REQ-ERR-01。
入口类型已经存在于 `uwb_twr_tof_input.h` 的 `admit_ranging_interval()`。

读完本文再写代码。先补会失败的测试，再实现。不要边写边扩范围。

---

## 0. 一句话任务

实现 **纯 C++、UHD-free、GNU-Radio-free** 的 SS/DS ToF 公式，输入只能是
已经 `admit` 过的 `AdmittedRangingInterval` 和显式的 `ClockRatio`；
用 **独立 MATLAB oracle 实跑** 产生 golden，C++ QA 对照同一组向量。
本阶段结束后：公式有、oracle 有、FSM 仍然没有。

---

## 1. 当前状态（不要重新发明）

M0.1 契约层已按 N07 关闭条件标为「契约层完成、协议能力未实现」：

| 事实 | 含义 |
|---|---|
| R1–R8、N01–N06、N07 已关闭 | 配置 / 帧 / 时间戳 / 测距准入 / 枚举域 fail-closed 可用 |
| CTest 52 项 51 通过 | 唯一失败仍是历史吞吐 `qa_uwb_pdu_rational_resampler.cc`，门槛不改 |
| 48 行 PHY 白名单 | 只有 `work_decode_verified`；`allows(work_decode, Ranging) = 0` |
| `admit_ranging_interval()` | M1-A 的唯一 interval 入口；产出 `RelativeTickInterval`（无 ns 字段） |
| TWR 协议能力 | **仍为零**：无 FSM、无 ToF 公式、无双端点、无 TWR 硬件收发 |

「契约层完成」不是「契约被证明正确」，也不是「可以发布距离」。
16 SYNC 和 64 SYNC 都没有 ToA 证据。本阶段 **不算测距验收**。

开工前必读（按这个顺序）：

1. 本文
2. `AGENTS.md`（尤其「第二/三轮的关键教训」和 TWR 十条）
3. [M0.1_N07修复报告_2026-09-29.md](M0.1_N07修复报告_2026-09-29.md) §11
4. [下一步开发方案_M0复核后.md](下一步开发方案_M0复核后.md) §3 M1-A
5. `gr-uwb/include/gnuradio/uwb/uwb_twr_tof_input.h`（`AdmittedRangingInterval`）
6. `gr-uwb/include/gnuradio/uwb/uwb_twr_timestamp.h`（`RelativeTickInterval`、`ClockDomain`）
7. `gr-uwb/include/gnuradio/uwb/uwb_twr_types.h`（`ExchangeStatus::ClockEstimateInvalid` 等）

不要把 [开发状态报告_寻求评审.md](开发状态报告_寻求评审.md) 的「四层契约已冻结」
当成当前结论；该文件顶部已作废。

---

## 2. 范围：只做 M1-A

### 2.1 必须交付

1. 新头 `gr-uwb/include/gnuradio/uwb/uwb_twr_math.h`（安装；header-only；
   只依赖已有 TWR 契约头和 C++ 标准库）。
2. `ClockRatio` 值类型：方向、符号、有理数比值、有效期、来源、不确定度。
3. `compute_ss_tof()`：`ToF_A = (RA - kAB * DB) / 2`，结果单位是 **A 的 ticks**。
4. `compute_ds_tof()`：`ToF = (RA*RB - DA*DB) / (RA + RB + DA + DB)`，
   四个 interval 先进入同一单位，结果单位是该公共域的 ticks。
5. 独立 MATLAB oracle：**必须在本机实跑**（本机 `matlab` 可用，R2024a）。
6. `testdata/twr/` 短向量 golden + 生成脚本 + Python 高精度对照。
7. `gr-uwb/lib/qa_uwb_twr_math.cc` 注册进 CTest。
8. 独立验证脚本（新建 `tools/twr/verify_m1_a_tof.py`，不要把 M0.1 的 15/15
   脚本改成 M1 入口）。
9. 安装消费者能 `#include <gnuradio/uwb/uwb_twr_math.h>` 并调用一个公式入口。
10. 开发报告：`docs/twr/M1-A_ToF数学开发报告.md`（只写实际做了什么、跑了什么、
    仍未做的是什么）。

### 2.2 明确禁止（做了就是超出任务）

- 实现 `uwb_twr_core` / FSM / fake radio / `twr_controller`（那是 **M1-B**）。
- 实现 GNU Radio block、UHD 适配、timed TX、两路 RX、timestamp patch。
- 实现 CFO/SFO **估计器**。本阶段只 **消费** 已构造的 `ClockRatio`。
- 实现 native 往返、首径 ToA、RMARKER 绝对约定、reply-delay 测量。
- 继续扩充 `uwb_twr_config.h` / `twr_config.py` 的业务规则；禁止再开平行 ToF 实现。
- 给 48 行白名单升级 `native_roundtrip` / `toa` / `ranging` 证据。
- 改历史吞吐门槛，或把新失败归进 `qa_uwb_pdu_rational_resampler.cc`。
- 宣称厘米级精度、完整互通、性能无回退。
- 用 C++ 输出生成 MATLAB 真值（REQ-QA-01）。
- 把未运行的 `.m` 写成「MATLAB 验证完成」。
- 夹带 pybind 大迁移。数学入口若要给 Python 测，用 QA/工具进程调 C++，
  或一个 **不超过该公式 API** 的最小探针；不要绑定整个 config。
- 修改 `admit_ranging_interval()` 的准入规则。若发现准入门新缺陷：**停下来写进
  报告，不要在 M1-A 里顺手改契约。**
- 给 `switch` 加 `default` 来“兜住”未知枚举（N07：域测试前置，switch 保持无 default）。

### 2.3 做完 M1-A 不要自动开工 M1-B

报告写清门槛，等下一份指示。M1-B 需要公式已经对照过独立 oracle。

---

## 3. 必须复用的类型（不要另起一套时间）

| 用途 | 用这个 | 不要用 |
|---|---|---|
| 公式输入 interval | `AdmittedRangingInterval` → `interval()` 得到 `RelativeTickInterval` | 裸 `Timestamp`、`timestamp_interval().status==Ok`、整数 ns、`Duration`、`double` 秒 |
| 精确 ticks | `RelativeTickInterval::exact_ratio(num, den)` | `tick_fraction()`、`seconds()`、`ns_projection_is_lossless()` 作为公式输入 |
| 时钟 | `ClockDomain`（name / tick_rate_hz / epoch_id / timestamp_bits） | 两个不同 domain 的绝对 ticks 直接相减 |
| 测距时间点 | 已经过门的 `RangeCapableTime`（只作端点检查，公式吃 interval） | `UhdRxFirstIqSample`、`predicted_sfd`、最大 CIR tap、主机收包时间 |
| 失败终态 | 数学层自己的 `TofStatus`，再映射到已有 `ExchangeStatus` | 静默返回 0、把负 ToF 裁成 0 再标 `Ok` |

`RelativeTickInterval` 没有 ns 字段，这是故意的（R4）。公式路径上出现
`duration.nanos()` / `relative_interval_to_duration()` 即视为实现错误。

显示 / 日志 / 与 oracle 比秒值：可以在 **结果对象的 display 字段** 里投影一次，
并单独标注是否无损。距离计算若需要秒，从 exact rational ticks 和
`tick_rate_hz` 做一次有理数除法，不要先截成整数 ns。

现有 128-bit 整数路径在 `uwb_twr_timestamp.h`（`unsigned __int128`）。
DS 乘加必须走至少 128-bit（必要时 256-bit 分解），溢出 **拒绝**，不要绕回 double。

---

## 4. API 契约（实现必须长这样）

命名空间：`gr::uwb::twr`。头文件自包含，不 include GNU Radio / UHD。

### 4.1 `ClockRatio`

`kAB = fA / fB`。语义（钉死，测试按这个写）：

- **方向**：把 **B 域的时间间隔** 换到 **A 域**。
- **对 SS**：`DB_in_A_ticks = kAB * DB_in_B_ticks`，然后
  `ToF_A_ticks = (RA_A_ticks - DB_in_A_ticks) / 2`。
- **共同时钟**（单台 X410 两 channel 的 nominal）：`kAB = 1` 必须是
  **显式构造**（例如 `ClockRatio::unity_same_clock(domain_a)`），
  独立时钟测试 **不得** 默认填 1。
- **来源**用枚举，例如：`NominalSameClock`、`NominalRateRatio`、
  `Calibrated`、`Estimated`。每个来源配 `xxx_is_known()`；域外值拒绝。
- 携带：A/B 的 `ClockDomain`（至少 name+rate+epoch）、有理数 `k`（num/den）、
  `valid_from_ticks` / `valid_until_ticks`（A 域）、可选不确定度（有理数 ppm 或
  明确“未声明”）。
- 过期、域不匹配、epoch 不一致、`k` 非有限 / 非正 / 分母 0 →
  `TofStatus` 对应失败，映射 `ExchangeStatus::ClockEstimateInvalid`。
- `k` 的符号：频率比为正。负 `k` 拒绝，不要靠它“翻转公式”。

SFO/CFO 怎么估是 M2 的事。这里只检查调用者给的 `ClockRatio` 能不能用。

### 4.2 SS

```text
RA = t4A - t1A     # A 域 admitted interval
DB = t3B - t2B     # B 域 admitted interval
ToF_A = (RA - kAB * DB) / 2     # A ticks，有符号
```

函数形态（名称可微调，语义不能调）：

```cpp
TofResult compute_ss_tof(const AdmittedRangingInterval& ra,  // t4A-t1A
                         const AdmittedRangingInterval& db,  // t3B-t2B
                         const ClockRatio& k_ab);
```

检查（失败要 **点名字段**，先做枚举/域检查）：

1. `ra`/`db`/`k_ab` 自身有效。
2. `ra.domain` 是 A，`db.domain` 是 B；与 `k_ab` 声明的 A/B 一致。
3. 允许 A、B 为不同设备时钟（这正是 `kAB` 存在的理由）。
4. `k_ab` 在 `ra` 的时间范围内有效。
5. 乘法 / 减法 / 除以 2 在有理数 ticks 上精确；除以 2 允许分母变 2，
   **不要** 先取整再除。
6. 负 ToF：**保留有符号原值**，`status != Ok`，理由独立（例如 `TofNegative`），
   映射到一个 **新的或已有的** `ExchangeStatus`（若新增：按 types.h 规则 **append only**，
   并加 `xxx_is_known()` / `to_string` / `from_string`）。禁止裁成 0 再标成功。

REQ-PROTO-05：SS 基线结果在 **A** 可用。结果里写 `computed_at = initiator_a`。

### 4.3 DS

```text
RA = t4A - t1A     DB = t3B - t2B
DA = t5A - t4A     RB = t6B - t3B
ToF = (RA*RB - DA*DB) / (RA + RB + DA + DB)
```

```cpp
TofResult compute_ds_tof(const AdmittedRangingInterval& ra,
                         const AdmittedRangingInterval& rb,
                         const AdmittedRangingInterval& da,
                         const AdmittedRangingInterval& db,
                         const ClockRatio& k_ab);  // 仍要：把两端 interval 换到同一单位
```

规则：

1. **禁止** 在不同设备的原始 tick 上直接套公式。先把四个间隔换到同一时间单位
   （建议：全部换到 A ticks，或全部换到秒的有理数；选一个，写进头文件注释，
   测试按该选择断言）。
2. 不等回复时间是合法输入（`DA != DB` 的向量必须通过）。
3. 分母 0 或非正（在物理模型下分母应为四个正间隔之和）→ 显式失败。
4. 乘法溢出 → 显式失败，不回退 double。
5. 负 ToF 处理与 SS 相同。
6. REQ-PROTO-05：DS 基线结果在 **B** 可用。结果里写 `computed_at = responder_b`。
   公式本身与“谁读结果”分开；不要因为结果在 B 就把单位偷偷换成 B ticks
   而不记录。

### 4.4 `TofResult`

至少包含：

- `ok` / `TofStatus status` / 映射后的 `ExchangeStatus`
- 有符号 ToF：exact rational ticks（num/den）+ 所在 `ClockDomain`
- display seconds（标明投影，不参与后续公式）
- 选用的 interval 有理数（RA/RB/DA/DB 及换算后的值）
- 使用的 `ClockRatio` 副本（来源、k、有效期）
- `detail` 字符串（不作为机器判据）

距离 = `propagation_speed * ToF_seconds` 可以是可选派生字段，
**默认真空光速必须写成命名常量**，并允许测试注入电缆速度。
本阶段不把距离当验收门槛，不写厘米级。

`TofStatus` 是新枚举：域检查 + `xxx_is_known()` + 无 `default` 的 switch。
未知状态 fail-closed。`validate()` 风格的入口 **报告，不抛异常**。

### 4.5 公式入口不自己 admit

`compute_*` **不**接收裸 `Timestamp`，不调用 `admit_ranging_interval()`。
调用方（将来的 FSM）先 admit 再计算。QA 里构造输入时走准入门；
不要为了图省事给 `RangeCapableTime` 开测试后门，除非现有 friend 机制已经提供。

若 QA 需要合法 `AdmittedRangingInterval`，照 `qa_uwb_twr_tof_input.cc` 的夹具：
完整 calibration、first-path、TX evidence（若 TX 侧是 `ScheduledCalibrated`）。
N02 规则继续有效：TX 允许 `scheduled_calibrated` + 完整 `TxSendEvidence` +
`outcome == Completed`；**入门后不得改标 `hardware_measured`**。

---

## 5. 数值与失败矩阵（先写成测试）

每一条都要有 **独立于实现的期望值**（来自 MATLAB oracle 或手算有理数）。
不要断言两个同源 C++ predicate 相等（N03）。

| ID | 场景 | 期望 |
|---|---|---|
| SS-1 | 共同时钟 `kAB=1`，对称回复，已知 ToF | 精确命中 golden |
| SS-2 | `kAB` 由两个不同 nominal rate 构造 | 先换到 A ticks 再算 |
| SS-3 | 正 ppm / 负 ppm 的 `kAB`（相对 nominal） | 符号正确：B 钟快则 `kAB<1` 或按你钉死的定义，**定义和测试必须同一句话** |
| SS-4 | 独立时钟，禁止默认 `kAB=1` | 缺 ratio → `ClockEstimateInvalid` |
| SS-5 | ratio 过期 | `ClockEstimateInvalid` |
| SS-6 | A/B epoch 不一致 | 域错误 |
| SS-7 | 负 ToF | 保留负值，`ok=false` |
| SS-8 | 跨 wrap 但 interval 已由准入门形成 | 公式使用 admitted 的相对间隔，不再自己做模运算 |
| DS-1 | 对称回复 `DA=DB` | 与 SS（`kAB=1`）在同一物理 ToF 上一致（容差由 oracle 规定） |
| DS-2 | `DA ≠ DB` 非对称 | 公式命中 APS013 形式的 golden |
| DS-3 | 极小 ToF + 长 turnaround | 数值稳定，不丢精度到 ns 截断 |
| DS-4 | 分母异常 | 显式失败 |
| DS-5 | 乘法溢出（构造大 interval） | 显式失败 |
| DS-6 | 负 ToF | 同 SS-7 |
| COM-1 | 域外 `ClockRatio` 来源枚举 | 拒绝，点名字段 |
| COM-2 | 把 `RelativeTickInterval::seconds()` 的 double 当输入 | 类型上就做不到；QA 用 `has_nanos_member` 一类静态断言防止回归 |
| COM-3 | 未 admit 的 interval | 编译期无法传入裸 Timestamp |

手算小例子（必须出现在 QA 注释里，数字写死）：

共同时钟、无分数：`RA=2000, DB=1000` ticks → `ToF=500` ticks。
非对称 DS：选一组能整除的整数，使分子分母都是整数，避免“差不多”。

---

## 6. MATLAB oracle（本阶段的正确性权威）

### 6.1 物理模型（脚本必须写明）

独立生成，不读 C++ 输出：

- 真值 ToF `τ`（秒）
- 两端钟：`fA`、`fB`（Hz），可选恒定 ppm 或线性漂移（M1-A 至少做恒定 ppm）
- 回复延迟 `δB`、`δA`（秒，DS 才有 `δA`）
- 由物理时间戳推 interval：
  - `RA = 2τ + δB`（A 钟尺度上；独立钟时按各自钟读数再相减）
  - `DB = δB`（B 钟尺度）
  - DS 再加 `DA`、`RB`
- wrap：在指定 bit 宽的计数器上取模后再交给“测量端”，但 **oracle 真值用未模的物理时间**。
  C++ 侧只吃已经 admit 的相对间隔（wrap 已在 timestamp 层解决）。

脚本输出 JSON 或 CSV 到 `testdata/twr/`。字段用十进制字符串保存大整数，
遵守 REQ-OUT-01。

建议文件：

```
testdata/twr/generate_tof_oracle.m
testdata/twr/tof_oracle_vectors.json
testdata/twr/README_tof_oracle.md   # 如何复跑、matlab 版本、向量 hash
```

Python 高精度（`fractions.Fraction`）可以再算一遍同一组输入，作为第二对照；
**不能替代 MATLAB 实跑**。

### 6.2 必须实跑

```bash
matlab -batch "generate_tof_oracle"
```

工作目录、路径写进 README。退出码 0，向量文件写入仓库。
报告里贴：命令、MATLAB 版本、输出 hash、向量条数。

若某条向量 MATLAB 与手算不一致：先修模型，再写 C++。

### 6.3 C++ 怎么对照

`qa_uwb_twr_math.cc` 读 `tof_oracle_vectors.json`。
对每条：用夹具构造 admitted interval（ticks/frac 来自向量），调用 `compute_*`，
断言 exact rational **相等**（能整除的向量）或与 oracle 的有理数差低于
脚本写明的上限（该上限必须来自模型量化，不能是“看起来差不多的 ns”）。

`UWB_TESTDATA_DIR` 的用法照 `qa_uwb_twr_frame.cc` / `qa_uwb_twr_config_parity.cc`。
默认不要往源码树写新 CSV；oracle 向量是输入 golden，检入 `testdata/twr/`。

---

## 7. 实现顺序（按这个做）

1. **先写 MATLAB 生成脚本和一组手算向量**，实跑，检入 JSON。
2. **再写 `qa_uwb_twr_math.cc`**，此时头文件只有声明或空壳，测试应失败。
3. **实现 `uwb_twr_math.h`**，直到 QA 与 oracle 一致。
4. 注册 CMake：
   - `include/gnuradio/uwb/CMakeLists.txt` 安装 `uwb_twr_math.h`
   - `lib/CMakeLists.txt` 把 `qa_uwb_twr_math.cc` 加到与 `qa_uwb_twr_tof_input.cc`
     同一组；给 testdata 宏。
5. 扩展 `tools/twr/verify_install_consumer.sh` / 消费者源：include 新头并调用
   `compute_ss_tof` 的共同时钟小例子（`RA=2000, DB=1000 → 500`）。
6. 写 `tools/twr/verify_m1_a_tof.py`：从 JSON 自己用 `fractions` 重算 SS/DS，
   并编译一个最小 C++ 探针对照（风格学 `verify_m0_1_findings.py`）。
   断言返回值本身，不要断言“两个 C++ 函数意见一致”。
7. 跑门禁。写 `docs/twr/M1-A_ToF数学开发报告.md`。
8. 更新 `开发状态.md`、`AGENTS.md` 的当前阶段一句：M1-A 进行中/完成、
   **协议 FSM 仍未实现**。

每一步有意义的代码改动后：`cmake --build` + 受影响 QA。不要攒到最后再编。

头文件保持可独立编译。需要 `<new>` / `<cstddef>` 就自己 include（N05 教训）。
不要把数学头 include `uwb_twr_config.h`。

---

## 8. 从三轮审核带进本阶段的硬规则

1. **测试约定必须等于需求。** 合法输入不得写成“必须拒绝”。
2. **语料一致 ≠ 正确。** MATLAB 真值来自物理模型，不来自 C++。
3. **枚举成员 ≠ 枚举域。** 新枚举一律 `xxx_is_known()`，域检查先于 switch，
   switch 无 `default`。
4. **穷举可选字段。** `ClockRatio` 里默认缺席的可选有效期/不确定度也要注入域外值。
5. **`validate`/计算入口报告失败，不抛异常。**
6. **scheduled TX 证据规则不动。** 公式层碰不到 TX outcome；QA 夹具不得把
   `ScheduledCalibrated` 改标成 `HardwareMeasured`。

`M0_调度语义.md` 本阶段相关条目：C-1（core 不 include GR/UHD——数学头同样）、
C-8（时间戳形状）、C-14（ToF/clock-ratio oracle）。C-2～C-7、C-9～C-13 是 M1-B /
controller 的，**不要在本阶段勾掉或假实现。**

---

## 9. 门禁（本阶段结束时必须达到）

在仓库根目录：

```bash
cd gr-uwb/build && cmake . && cmake --build . -j"$(nproc)"

env -u LD_LIBRARY_PATH ctest --output-on-failure
# 期望：原 51/52 仍在；新增 math QA 通过。
# 唯一允许的失败：uwb_qa_uwb_pdu_rational_resampler.cc 历史吞吐。
# 新增失败不得并入该项。

env -u LD_LIBRARY_PATH ctest -R uwb_qa_uwb_twr_math --output-on-failure

python3 gr-uwb/apps/test_twr_config.py          # 期望仍全绿，条数不因 ToF 而被迫大涨
python3 tools/twr/verify_m0_1_findings.py       # 仍 15/15，M0.1 回归不能坏
python3 tools/twr/verify_m1_a_tof.py            # 新脚本，全部 PASS

matlab -batch "cd('testdata/twr'); generate_tof_oracle"   # 按 README 的实际路径
tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_m1a
```

手工跑测试二进制必须 `env -u LD_LIBRARY_PATH`，并用 `ldd` 确认加载 `build/lib`。
不要 `sudo cmake --install` 到系统路径。安装验证只用临时前缀。

CTest 的 `uwb_qa_install_consumer` 未配 `UWB_TWR_INSTALL_PREFIX` 时 SKIP 是设计行为；
**SKIP 不能当成安装交付已验收。** 交付看 shell 脚本的 `ALL OK`。

---

## 10. 完成判定（全部为真才算 M1-A 完成）

- [ ] `uwb_twr_math.h` 已安装，外部消费者能调用 SS 小例子得到 500 ticks。
- [ ] SS：`kAB` 方向/符号/有效期有独立测试；共同时钟与独立时钟分开。
- [ ] DS：非对称回复有独立测试；四个间隔先统一单位。
- [ ] 负 ToF、分母异常、溢出、过期 ratio、域错误均显式失败且保留有符号原值。
- [ ] MATLAB 已实跑；`tof_oracle_vectors.json` 检入；C++ QA 读该文件。
- [ ] 公式路径无整数 ns、无 `double` 秒作为中间真值。
- [ ] 未实现 FSM / fake endpoint / block / pybind 大迁移 / config 膨胀。
- [ ] 历史吞吐失败仍是唯一允许的 CTest 红灯。
- [ ] `开发状态.md` / `AGENTS.md` 写的是「M1-A 公式+oracle 完成，协议能力仍无 FSM」。
- [ ] 报告不声称测距可用、不升级 PHY 证据级、不承诺厘米级。

有一条为假就不能把 M1-A 标完成，更不能开 M1-B。

---

## 11. 提交与文档

- 提交信息用英文祈使句，例如 `feat(twr): add SS/DS ToF math with independent oracle (M1-A)`。
- 不要把 MATLAB 运行日志、`/tmp` 安装树、build 产物推进 git。
- 大 IQ 不要放进 `testdata/twr/`；只要短向量。
- 若中途发现准入门 / 时间戳层的新缺陷：单独开报告，**不要塞进 ToF 提交冒充公式问题。**

---

## 12. 给执行者的工作方式

- 改生产头之前，先让 QA 因缺实现而失败。
- 每个公式数字都能指出是手算、MATLAB 还是二者一致。
- 发现“测试已经绿了但约定和需求相反”时，改测试和实现去贴需求，
  不要把错误行为写进 golden（N02）。
- 不要重构 `twr_config.py`。
- 不要为了“完整”把 M1-B 的事件类型先铺一遍。
- 不确定就停在报告的「未决」里，不要用 radar 默认值填上。
