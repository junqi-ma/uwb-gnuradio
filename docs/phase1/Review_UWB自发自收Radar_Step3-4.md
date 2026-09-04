# Review：UWB 自发自收 Radar Step 3–4

> Review 日期：2026-09-04
> 被审查报告：[开发报告_UWB自发自收Radar_Step3-4.md](开发报告_UWB自发自收Radar_Step3-4.md)
> 审查基线：`c9aea80`
> 结论：**Step 3 数值主路径基本正确，但 Step 3–4 整体暂不通过；完成整改并复验前，不进入 Step 5。**

## 1. Review 范围

本次验收范围：

1. Step 3：SFD 回推 SYNC origin、局部 timing refine、复数 raw/normalized CIR；
2. Step 4：`radar_cir_one` 的 SFD→timing→CIR 状态整合；
3. 32/64/128 SYNC 参数化；
4. MATLAB canonical golden 数值对照；
5. 失败状态、bit stability、hot-path 分配及构建回归。

不验收 Step 5 PDU 65/48 metadata、GNU Radio message block、PacketSource、LoopbackEcho、CirWriter、EchoTimer、UHD、X410 或 Phase A。

## 2. 验收结论

| 项目 | 结论 | 说明 |
|---|---|---|
| Step 3 CIR 数值主路径 | 基本通过 | clean/integer/fractional raw+normalized CIR 与 MATLAB golden 对齐 |
| Step 3 完整 QA | **不通过** | 缺少独立的多路径 MATLAB 对照，以及 delay=1、tap 窗边界等计划用例 |
| Step 4 integrator | **不通过** | prepared scratch 与运行时 profile 未绑定，合法但错配的配置会静默返回 `Ok` |
| 32/64/128 参数化 | **不通过** | 只测试 `N×SYNC+SFD` 合成片段，没有对应完整 packet golden |
| Radar 专项 CTest | 通过 | 独立复跑 4/4 Passed |
| 全量 CTest | 通过 | 本次独立复跑 21/21 Passed；报告中的 20/21 是性能 sanity 波动 |
| MATLAB canonical verifier | 通过 | clean/integer/fractional CIR hard gate 再次通过 |
| 提交状态 | 未完成 | 审查时 Step 3–4 实现和报告仍在 working tree，HEAD 为 `c9aea80` |

当前不能正式进入 Step 5。先完成本 Review 的 R1–R4，再申请 Step 3–4 复验。

## 3. 已确认正确的部分

以下设计和实现可以保留：

- `estimate_radar_cir` 实现了 skip、相干平均、`sampled_code` forward correlation 和 `/code_energy`；
- 同时保留 raw complex taps 和 L2-normalized complex taps；
- clean、整数时延 37、分数时延 12.4 的 C++ CIR 与 MATLAB canonical golden 对齐；
- raw CIR 对输入幅度 0.1/0.5/2.0 呈线性，normalized CIR 基本不变；
- `cir_origin_sample` 使用 TX-time/predicted origin，`preamble_start_sample` 表示 RX refined origin，两者分离是正确的；
- 整数信道时延 37 表现为 CIR peak 从 18 移到 55；
- SFD 失败不会调用 timing/CIR，失败结果 `tap_count=0`；
- `SfdFailed`、`TimingFailed`、`CirFailed`、`InvalidInput` 使用严格枚举；
- 同一输入重复运行 bit-stable；
- hot path 中未发现显式 vector resize/growth；
- 不执行 CFO 估计或补偿，符合当前需求。

## 4. 阻断问题 P0：prepared profile 与运行配置没有绑定

### 4.1 问题

`RadarCirConfig` 声明了：

- `code_index`；
- `sync_repetitions`；
- `samples_per_symbol`；
- `sfd_mode`。

但 `prepare_radar_cir_core` 只接收原始 SFD sequence、SYNC template 和 HRP code，并没有把这些数据的 profile 身份保存到 scratch。

`radar_cir_one` 运行时：

- 只检查 `sfd_mode` 是否属于已知字符串；
- 只检查 `sync_template.size()==samples_per_symbol`；
- 不比较实际 prepared SFD sequence 与 `cfg.sfd_mode`；
- 不比较实际 prepared HRP code 与 `cfg.code_index`；
- 不知道 TX packet 实际使用了多少个 SYNC repetitions。

因此 config 可以合法但与 scratch/TX packet 不一致，算法仍然返回成功。

### 4.2 独立复现一：SFD mode/code index 错配

复现条件：

```text
prepared scratch : 4z2 / code 9
runtime config   : ieee / code 10
input            : canonical 4z2 / code 9 / 64-SYNC packet
```

实际结果：

```text
ok=1 status=Ok sfd=67021 peak=18
```

`cfg.sfd_mode` 和 `cfg.code_index` 实际没有约束算法使用的模板，却可能被后续 metadata 当作结果身份写出。

### 4.3 独立复现二：SYNC repetitions 错配

复现条件：

```text
input packet     : canonical 64-SYNC
runtime config   : sync_repetitions=32
predicted SFD    : 67021
true SYNC origin : 1997
```

实际结果：

```text
ok=1
status=Ok
sfd_start=67021
preamble_start=34509
cir_origin=34509
valid_repetitions=22
peak=18
```

`34509 = 67021 - 32×1016`，它是64-SYNC packet内部的一个重复SYNC，不是真实preamble origin。由于每个SYNC相同，timing refine无法仅从该位置判断“这是第一个SYNC还是第33个SYNC”，因此结果会看起来完全正常。

这违反需求中“TX生成器、SFD回推和CIR必须使用同一SYNC repetition”的约束，并会让距离轴原点错误约32.6 µs。

### 4.4 必须整改

需要建立单一可信的prepared profile。推荐方案之一：

```text
PreparedRadarProfile
  code_index
  sfd_mode / sfd_sequence identity
  n_sfd_symbols
  sync_repetitions
  samples_per_symbol
  TX packet/profile ID
  supported CIR pre/post
```

实现选择：

1. 让 `prepare_radar_cir_core` 接收完整 `RadarCirConfig` 或不可变 profile，并把身份保存到 scratch；
2. `radar_cir_one` 比较 runtime metadata/profile 与 prepared profile，任一不一致返回 `InvalidInput`；或
3. 删除 runtime config 中的重复身份字段，hot path 统一从 prepared profile 读取。

SYNC长度可配置不代表允许同一prepared TX packet随意解释成另一个长度。切换32/64/128 packet时，可以在非hot path重新prepare。

必须新增负向QA：

- prepare 4z2，运行传 `ieee`；
- prepare code9，运行传 code10；
- prepare/TX为64-SYNC，运行传32或128；
- `samples_per_symbol` 不匹配；
- profile/TX ID 不匹配；
- 所有错配必须返回 `InvalidInput`、`tap_count=0`，不得输出成功origin/CIR。

## 5. 阻断问题 P0：32/64/128没有对应完整packet golden

开发计划明确要求：

> 添加32/64/128 SYNC参数化，每种都使用对应完整packet golden。

当前测试数据仅为：

```text
N × reference_preamble.bin + kron(4z2, SYNC)
```

它只包含SYNC和SFD，没有正常packet所需的PHR、PSDU/FCS，也没有验证整包一次性48/65。报告已经明确说明这些合成片段不等于 `lrwpanWaveformGenerator`。

现有QA可以保留为快速结构单测，但不能作为Step 4的完整packet验收。

### 必须整改

1. 为32/64/128分别生成完整pulse-shaped packet；
2. 每个packet包含固定、可复现的PHR和PSDU/FCS；
3. 对完整998.4 packet一次性48/65生成native TX，禁止重复751-sample native SYNC；
4. metadata分别记录SYNC数、SFD mode、code、TX长度、SYNC/SFD坐标、payload/FCS和生成器身份；
5. 为每个完整packet构造RX loopback/golden；
6. C++表驱动QA必须分别读取三种完整packet，验证：
   - SFD detection；
   - preamble origin回推；
   - `cir_origin_sample`；
   - skip/count；
   - CIR窗口不越界；
   - raw/normalized CIR输出；
7. 如果MATLAB toolbox不直接支持某个SYNC长度，需使用经过MATLAB验证的完整packet生成路径，并在metadata中诚实标明标准/自定义profile，不能退化成只有SYNC+SFD的片段。

## 6. 阻断问题 P1：多路径QA没有独立MATLAB对照

开发计划要求2–4条复数路径的时延和相对复增益对齐MATLAB。

当前 `test_radar_cir_multipath_linear` 的方法是：

1. 用C++ estimator分别计算每条路径；
2. 用同一个C++ estimator计算组合信号；
3. 检查组合结果等于各个C++结果之和。

这个测试可以证明实现保持线性，但expected同样来自被测C++实现，因此不能证明多路径时延和复增益与MATLAB一致。

### 必须整改

选择以下任一独立expected来源：

- MATLAB生成2–4 path RX和raw/normalized CIR golden；或
- 从已审核MATLAB clean raw CIR出发，按已知整数/分数时延和复增益构造expected complex taps。

建议至少覆盖：

```text
path 0: delay=0,   gain=0.30∠0
path 1: delay=25,  gain=1.00∠φ1
path 2: delay=50,  gain=0.25+j0.40
```

断言不能只检查最强峰，应同时检查：

- 完整raw complex taps相对L2；
- normalized complex taps相对L2；
- 每条可分辨路径的delay；
- 相对复增益，包括相位；
- 窗口边缘路径的截断约定。

现有C++ superposition线性测试可以继续保留，但不能代替MATLAB/golden测试。

## 7. QA缺口 P1：单路径delay=1和CIR边界未覆盖

开发计划要求单路径覆盖整数时延0/1及窗口边界。当前canonical覆盖clean=0和delay=37；多路径用例包含0/25/50，但没有明确的delay=1和tap窗口边界断言。

需要补充：

- delay=0；
- delay=1；
- 首径落在最前合法tap；
- 路径落在最后合法tap；
- 再向前/向后1 tap时的失败或截断契约；
- `pre=0`、最小`post=1`以及prepare最大tap边界。

每个合法位置应检查peak tap和完整complex taps，而不只是检查函数返回成功。

## 8. 建议整改 P2：整数坐标运算应检查溢出

`nominal_preamble_start` 当前直接执行：

```cpp
static_cast<int64_t>(sync_repetitions) *
static_cast<int64_t>(samples_per_symbol)
```

极端输入可能在转换或有符号乘法时溢出。`estimate_radar_cir`中的repetition sample坐标也存在类似风险。

建议增加checked arithmetic：

- `sync_repetitions <= INT64_MAX / samples_per_symbol`；
- `preamble_start + k*samples_per_symbol`检查乘法和加法；
- `n`、tap/window长度转换到`int64_t`前检查上界；
- 溢出统一返回`InvalidInput`。

此项可以与profile输入验证一起完成。

## 9. 独立复验结果

实际执行：

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build -R 'uwb_qa_uwb_radar_' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
```

结果：

```text
Radar专项：4/4 Passed
全量CTest：21/21 Passed
```

报告记录的 `qa_uwb_pdu_rational_resampler` 吞吐sanity失败本次没有复现。该测试受机器负载影响，本次结果为通过。不要为了Step 3–4修改无关性能门限；如后续持续波动，应单独处理benchmark稳定性。

MATLAB canonical verifier再次实际运行并PASS：

| Case | normalized complex L2 vs `estimateCir` |
|---|---:|
| clean | `1.914e-8` |
| delay_int | `2.969e-8` |
| delay_frac | `2.183e-8` |

其他结果：

- clean peak=18；
- integer-delay peak=55，relative shift=37；
- fractional-delay peak=30；
- integer raw amplitude ratio=1.000；
- fractional raw amplitude ratio=0.938225。

这些结果证明当前Step 3 CIR数值主路径是可信的，但不能消除profile错配和QA覆盖缺口。

## 10. 给Grok的整改顺序

### R1 — 绑定prepared profile与runtime metadata

- 设计不可变prepared profile；
- 消除重复配置的双重事实来源；
- 增加SFD/code/SYNC长度错配负向QA；
- 增加checked coordinate arithmetic。

### R2 — 补32/64/128完整packet goldens

- 生成三种完整packet及native 48/65；
- 补metadata和RX/CIR fixture；
- 保留现有短合成片段作为快速单元测试；
- 新增完整packet表驱动QA。

### R3 — 补独立多路径及边界QA

- 增加MATLAB/独立golden多路径；
- 检查完整complex CIR和相对复增益；
- 增加delay=1、tap左右边界及越界契约。

### R4 — 构建、提交并申请复验

- 运行Radar专项；
- 运行demod core/realtime demod回归；
- 运行全量CTest；
- 运行MATLAB verifier；
- 更新开发报告中的实际测试结果；
- 将CMake、三个header、三个QA和报告完整提交，避免只在working tree通过。

## 11. 下次复验硬门槛

- [ ] prepared profile与runtime的SFD mode、code index、SYNC repetitions完全绑定；
- [ ] 合法但错配的profile全部返回`InvalidInput`和`tap_count=0`；
- [ ] canonical 64-SYNC传入32-SYNC配置不得返回`Ok`；
- [ ] 32/64/128均有对应完整packet golden和metadata；
- [ ] 三种packet均验证整包一次性48/65；
- [ ] 三种packet的SFD回推、skip/count和CIR窗口均通过；
- [ ] clean/integer/fractional raw+normalized CIR继续保持MATLAB L2门限；
- [ ] 2–4 path完整complex CIR与独立MATLAB/golden对齐；
- [ ] 单路径delay=0/1、tap左右边界和越界行为覆盖；
- [ ] `ok/sfd_failed/timing_failed/cir_failed/invalid_input`表驱动状态覆盖；
- [ ] 所有失败状态`tap_count=0`，不暴露上次调用残留结果；
- [ ] hot path无动态扩容；
- [ ] 同一输入bit-stable；
- [ ] 坐标乘法/加法溢出返回`InvalidInput`；
- [ ] Radar专项、demod回归和全量CTest通过；
- [ ] MATLAB canonical verifier hard gate通过；
- [ ] 实现和报告已提交；
- [ ] 不宣称Step 5、Phase A、EchoTimer、UHD或X410完成。

## 12. 是否允许进入Step 5

**暂不允许。**

Step 5会把PDU metadata和重采样坐标接入当前core。如果profile身份尚未绑定，PDU中的合法但错误metadata会被静默接受，并产生看似正常但origin错误的CIR，后续更难定位。

应先完成R1–R3并通过Step 3–4复验，再进入PDU 65/48 metadata开发。
