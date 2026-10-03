# OpenCode 开发指示：M2-A native PHY 闭环

日期：2026-10-03。性质：下一阶段开发任务单，**不是实现或验收报告**。
本轮仅规划；由用户将本任务交给 OpenCode 后执行。完成后不自动进入 M2-B/M3。

## 1. 当前状态与下一步选择

依据：

- [TWR 需求](需求_UWB_SS_DS_TWR.md)、[总路线与验收矩阵](开发路线与验收矩阵.md)。
- [参考资料与 API 映射](参考资料与API映射.md)、根目录 [开发状态](../../开发状态.md)。
- [M1-B 整改报告](M1-B_复核整改报告_2026-09-30.md)、
  [独立审核意见](M1-B_复核整改审核意见_2026-09-30.md)。

M1-A 数学/MATLAB oracle 与 M1-B 离线协议核心作为本阶段基线。
整改报告及独立复审确认 R01–R09 关闭，修复代码 revision 为 `6a36521`；
当前仓库 HEAD `4edf6c2`，包含 2026-10-01 最终复核记录。
本轮检查 `uwb_twr_core.cc` SHA256 与整改报告一致：
`f378f4419af0f9bfdadfc3c4a24e283163c7c0a2bbf12e8dfadb25938614f4a9`。

2026-10-03 规划时，在现有 build 实跑 core/protocol_time/e2e/demo/verifier
五项 CTest，**5/5 通过**；纯核心 QA 的 ldd 无 GNU Radio/UHD 依赖。
这不是再次完成 R01–R09 全面独立审计，也不是本轮全量构建、全量回归或 MATLAB 实跑。
此前全量 59 项 58 通过、唯一历史 PDU 重采样吞吐失败，属于既有报告证据。

下一步选择 **M2-A：完整 TWR 帧的 native-rate 离线 PHY 闭环**。
先解决 work→native C++ 集成、完整帧/FCS、缓冲复用、数字采样坐标和独立 MATLAB
交叉验证；再做 M2-B 的协议 RMARKER/首径/物理时间映射。

| 阶段 | 本阶段要证明什么 | 不能据此证明什么 |
|---|---|---|
| M1-B（已完成） | 隔离端点的协议及故障状态机、数学和证据收敛 | 真实波形能够收发 |
| M2-A（本任务） | 实际 TWR bytes 经两种 native rate 往返仍正确解码，数字坐标有依据 | 首径 ToA、硬件时间戳、测距有效性 |
| M2-B（后续） | RMARKER、分数首径及质量、时间/校准映射、delayed-frame timestamp patch | 自动等于上板或商用互通 |
| M3/M4（后续） | X410 双 RX/双 TX、真实接收驱动 SS/DS、硬件错误与校准 | 独立商用设备互通 |

## 2. 范围与最小交付

必须交付：

1. 复用现有内核的 C++ work→native 发射波形转换，覆盖 32/65、48/65。
2. 完整 frame codec→唯一 FCS 追加→HRP 调制→native→work→解调→codec 验证。
3. 有界容量、输出长度、flush/reset/分块语义和数字坐标契约。
4. 独立 MATLAB 参考与短 golden、原始结果、逐字段比较及可复跑命令。
5. 最小离线 CLI、QA、能力证据与独立 verifier、资源和延迟观测。

本阶段不做：GNU Radio controller、新硬件 adapter、两 RX 路由、UHD 收发、
真实 deadline 提交、首径算法/测距校准、CFO→clock-ratio 估计、商用协议、Report、STS、
128+ preamble 的支持扩张、pybind 大迁移。
不重写 FSM/ToF，不要求在波形链上跑出 SS/DS 距离。
本阶段不把 decode-only 事件伪装成满足 M1-B 严格准入的 RxFrame。

完整成功链路：

```text
Frame v1（真实 Poll / Response / Final 字段）
  → codec MAC bytes（无 FCS）
  → HRP adapter 追加一次 FCS → HRP work IQ @ 998.4 MS/s
  → C++ TX resampler → native CF32 / 可选 SC16 往返量化
  → C++ RX resampler → work IQ
  → 现有 demod（实际 PHR / payload / FCS）→ codec → 与输入逐字节比较
```

传播/噪声模型只在测试 harness；不得把输入 payload、精确包起点、传播真值传给解码器
充当检测结果。允许完整有界搜索窗，但必须记录窗口范围并覆盖随机前置空白。
单位核测试可以用已知锚点；独立全帧验收不得只靠精确 seeded start。

## 3. 默认冻结的最小 profile

本任务先做最小 profile，不要求一次升级现有 48 行白名单。

| 参数 | M2-A 基线 |
|---|---|
| 工作采样率 | 998.4 MS/s |
| native 采样率 | 491.52 MS/s 与 737.28 MS/s，均为必须验收的离线路径 |
| TX / RX 比率 | TX 32/65、RX 65/32；TX 48/65、RX 65/48 |
| PHY | channel 5 元数据、BPRF mean PRF 62.4 MHz、code 9、64 SYNC |
| SFD | 标准短 SFD，对应现有 `ieee`；G0 核实具体序列与长度 |
| PHR / payload | 标准 PHR，0.85 Mb/s / 6.81 Mb/s；无 STS |
| 帧 | 本项目 frame v1 Poll / Response / Final；不是 DW SDK 帧 |
| 格式 | native CF32；另测 CF32→SC16→CF32 的尺度、舍入与饱和 |
| shaping / taps | G0 从已有受支持资产选定，冻结系数、增益、长度和 hash，不继承隐式 radar 默认 |

channel 5 在离线测试里不证明中心频率、RF 带宽或硬件能力。
任一必选 rate 无法通过，报告该路径未完成，不能静默删去它再宣布 M2-A 完成。
16 SYNC 可做后续 decode 扩展但不阻塞最小交付；64 也仍无 ToA 证据。
128 的既有拒绝是软件限制，不能写成芯片不支持。

PSDU 长度必须从 codec 几何推导：Poll/Response/Final 的 MAC bytes 为
14/24/29 B，加一次 2 B FCS 后为 16/26/31 B。不要把 PSDU 字节数称为含 SHR/PHR
的整段空口波形长度；波形长度须另算并含 pulse/filter tail。
另以 127 B PSDU 验证 PHY 容量边界，明确它不是新增 TWR frame 类型。

## 4. 已定位的复用入口与坑点

| 入口 | 复用方向 / 必查事项 |
|---|---|
| `include/gnuradio/uwb/uwb_hrp_mod_core.h` | `modulate_one`、`HrpModScratch`、prefix cache、`append_ieee_fcs`；modulate_one 的输入已经是 PSDU，FCS 所有权不能猜 |
| `include/gnuradio/uwb/uwb_rational_resampler_core.h` | 已有通用 `RationalResamplerLmCore<Interp,Decim>`，当前实例主要 65/48、65/32；优先验证 32/65、48/65，不能说“仓库没有重采样实现”而另抄一套 |
| `include/gnuradio/uwb/uwb_demod_core.h` | 重用完整 PHR/payload/FCS 路径；结果字段有 decode 坐标不代表 RMARKER/首径 |
| `apps/x410_cg400_hrp_echo_cir.py::NativeRateProfile` | 已有 Python `resample_poly` 的 work→native；作为行为参考与回归，不作为反应式 C++ 热路径 |
| `lib/qa_uwb_twr_phy_matrix.cc` | 已有 work-grid 矩阵及 impairment QA；其旧注释“没有 work→native”须按实际 Python 资产与本阶段 C++ 交付区分 |
| `testdata/resampler_65_32/`、`resampler_65_48/` | 可用 taps / golden；反向使用须重新证明增益、带宽和长度，不盲目反转比例 |
| `UWB_demodulation/+uwbdecoder/buildUwbReference.m` | 独立 HRP reference，MATLAB waveform generator / field indices |
| `UWB_demodulation/decode_uwb.m` | 独立解码入口，preprocessedRx 是数值向量；MATLAB 索引转 C++ 0-based 必须显式 |
| `UWB_demodulation/+uwbdecoder/ieee802154CRC16.m` | 独立 FCS 对照 |
| `testdata/regress_resampler_uwb_matlab.m` | 历史脚本只作参考：存在宽松偏移界限及捕获异常后跳过 decode；不能原样作为新验收门禁 |

现有模板 core 的 `process()` 可能按输入需求 reserve；已声明预分配不等于热路径
实际无分配。TX/RX 容量与 scratch 生命周期必须实测。
`resample_poly` 的中心裁剪和原始 `upfirdn` 全卷积不是同一输出契约；
不可只对齐峰值后声称逐样点一致。

## 5. G0：动手前冻结的接口与数字坐标

先交付 `docs/twr/M2-A_G0接口与数字坐标.md`，reviewer 同意后才修改主链。

### 5.1 形态与所有权

- 首选无 scheduler 的 C++ library/helper + 离线 CLI，复用现有 core；允许现有 VOLK
  依赖，不把 M1-B 无 VOLK 链接要求机械搬到 PHY。不得依赖 radio/PMT 来执行核测试。
- 新 block 非本任务默认项；确有必要时先读本地 GNU Radio 相似源码，说明 block 类型、
  消费/产出、history、output multiple、缓冲和背压，再请求确认范围。
- 准备阶段校验 profile、分配 scratch/系数/缓存；逐帧路径显式输入长度与输出容量，
  有界失败。C++17 使用项目现有容器/指针长度约定，不因新接口升级语言标准。
- 定义工作/原生 IQ 所有权、复用时点、每端/worker 隔离；输出状态不能在失败时仍标有效。
- frame bytes、FCS、调制配置和 RX profile 来自同一不可变执行快照。
  无需把 PHY helper 塞入 `EndpointCore`。

### 5.2 重采样契约

- 冻结零相位裁剪或因果全卷积策略，记录 taps、插值增益、初始相位、前后补零、
  尾部 flush、burst reset 和输出有效范围；不得隐式混用两种策略。
- 对全卷积且 N>0 的参考长度可用 `ceil(((N-1)*L+T)/M)`，N=0 单独定义；
  若另有裁剪，则明确扣除区间。尺寸算术使用溢出检查，不能依赖有符号溢出。
- 区分一个 burst 被拆成多 chunk 与两个独立 burst；测试 partial consume、短输出
  buffer、重复 flush、flush 后 process、reset，不能串帧泄漏滤波历史。
- 先证 scalar 的数学正确性，再对现有 SIMD/VOLK 路径对照；本阶段不追求新全率优化。

### 5.3 数字坐标与 M2-B 边界

每个阶段记录 `rate、0-based origin、input/output count、L/M、phase、trim、padding、
filter delay、有效区间`。索引映射使用整数/有理数，不用整数 ns 截断。
用 impulse/ramp/不同起始相位推导并验证映射，不靠对每条波形自由拟合常数“对齐”。
数字滤波群时延与 pulse shaping 延迟分别记录，不当成硬件校准常量。

这些字段是 **sample-grid bookkeeping**，不是协议 RMARKER/天线参考面。
解码出的 SFD/packet-start 可作诊断；不产生 `HardwareMeasured` RX 时间，
不填虚假的 first-path passed，不给 M1-B admission 加 bypass。
M2-A 不要求“native 往返后波形等于原 work 波形”：限带会改变波形；
应比较 C++ 与相同已冻结数字链的独立 oracle，并验证字节正确。

## 6. MATLAB 与独立验证（完成硬门槛）

必须同时有以下三类对照，不能只做 C++ TX→C++ RX 自洽：

1. 相同明确输入/系数的 MATLAB `upfirdn` 或独立多相参考与 C++ 重采样逐样点比较，
   包含冲激、随机复数、TWR 完整波形、长度和 trim/phase 元数据。
2. C++ 生成并经过 native 链的 IQ，由 `decode_uwb` 实际解码，比较 PHR、PSDU、FCS
   与诊断坐标；不捕获异常后写 PASS。
3. MATLAB 独立生成的标准最小 profile TWR 帧，经 native 链后用 C++ 解码；
   CRC/长度/字段由参考脚本独立核算。若标准工具箱有配置限制，明确记录并阻断对应
   验收，不用 C++ 输出复制为 MATLAB golden。

G0 先冻结比较器：bytes/FCS/长度/整数坐标契约精确一致；固定归一化输入的同链
CF32 对照建议 `max_abs_error <= 1e-4`、`relative_L2 <= 1e-5`（全零输入单独处理）。
工具箱 waveform pulse 与仓库 pulse 不同则逐项解释，不强行要求跨 pulse 波形逐样点相等；
跨实现解码仍须 bytes 精确。干净条件下 packet-start/SFD 对照须在统一定义后设置
明确样点界限，默认不超过 1 work sample；它不构成分数 ToA 精度承诺。
数值容差若需调整，必须先用独立参考说明原因并更新 G0，经复核同意，禁止看到失败
后自动扩大阈值或逐例重新对齐。

历史报告通过 Windows `F:\MATLAB` / WSL 调用 R2025b；**本轮未验证当前可用性**。
P0 先实际确认 launcher、toolbox 和短 smoke test。没有 MATLAB 时可完成 C++/QA/
脚本部分，但保持“M2-A 未完成，MATLAB 待实跑”，不能用 Python/SciPy 替代声明。
不自动安装软件或申请许可证。保存版本、命令、退出码、日志、输入/输出 hash、
脚本 revision 与 `matlab_executed` 来源；仅存在 `.m` 文件不算通过。

## 7. QA 与完成矩阵（A01–A14）

| ID | 必测内容 | 正确性判定 |
|---|---|---|
| A01 | 两 native rate × Poll/Response/Final × CF32/SC16，共 12 个干净基本单元 | 同一链实经 native 网格；PHR 长度/速率正确，bytes 精确、FCS 通过 |
| A02 | codec/PSDU/FCS 所有权、独立 CRC、重复/缺失 FCS | 唯一追加；畸形输入不能被误报为合法原始帧 |
| A03 | 32/65、48/65 及 RX 反向核，impulse/随机/golden | 长度、增益、相位、tail 与独立参考一致 |
| A04 | 分块、不足输出空间、reset/flush、独立连续 burst | 与一次处理一致；无越界、无跨帧状态污染 |
| A05 | 非零 origin、全部多相余数、前后 guard、裁剪边界 | 数字坐标可逆/可解释，有理数不丢分数；无自由拟合偏移 |
| A06 | SC16 0/正负极值/舍入/scale/饱和、NaN/Inf 输入策略 | 转换明确且无 UB；饱和计数，不把严重削顶默认为高质量 |
| A07 | 错 rate/profile/enum、STS/不支持长度、容量越界/尺寸溢出 | 启动或入口 fail-closed，原因点名；保留有效边界正例 |
| A08 | 127 B PSDU、最小长度、短帧/截尾、空输入、坏 PHR/坏 FCS | 不崩溃；合法边界正确，错误状态区分；不静默输出上次结果 |
| A09 | 随机前置空白、包不在窗中、连续不同帧 | 不用精确真值 seed；逐包守恒，缺包不能得到复制的 payload |
| A10 | native 域注入 CFO、AWGN、分数延迟、多径 | 参数/seed 固定；成功帧 bytes 正确，失败显式统计，不把后径峰当 ToA |
| A11 | MATLAB 双向交叉 + 核逐样点对照 | §6 全部实跑，无 skip 冒充 PASS |
| A12 | 配置快照、CLI 输出、manifest/verifier 变异测试 | 更改 rate/frame/taps 确实改变执行；伪造长度/hash/evidence 能被检出 |
| A13 | 分配、峰值内存、warm/cold 延迟、buffer 重用 | 新 TX/resampler 准备后热路径零分配；既有解调分配独立计数，不宣称整链零分配 |
| A14 | M0/M1、既有 TX/RX/radar、安装消费 | 受影响 QA 与全量回归；新增公开接口实际消费，无安装 SKIP 代替验证 |

A10 在 G0 固定用例及门槛，至少包含 0/±20 kHz CFO、一个固定 SNR 的 AWGN、
整数及分数延迟、首径弱于后径。固定干净基线全部通过；压力单元按实际通过范围
标注支持/失败，不要求任意坏信道均可解码，也不因压力失败偷偷删除测试。
FEC 可能纠错：坏 FCS 负例应在发端生成“错误 FCS 但编码合法”的 PSDU，
不能假设任意 IQ bit flip 都必然导致 FCS failure。

A01 的输入必须来自实际 frame codec，不复用随机的“同长度 payload”替代。
至少两组不同 timestamp/地址/seq 字节，证明缓存更新没有复用旧载荷。

## 8. capability 证据与结果格式

- 默认输出只为 `decode-only / simulation`，`measurement_valid=false`；不输出可消费距离。
- 最多对**实际实测**的 rate/profile/shaping/taps/格式提升
  `native_roundtrip_verified`；ToA/hardware/vendor 保持 false，Ranging 仍拒绝。
- 不能按路径字符串含 native、配置中 rate 字段、CLI 成功退出或复用旧 work CSV
  批量升级 48 行。独立核对 manifest、实际执行阶段和全部必需检查。
- 如果现有 capability 表的 key 不足以区分 taps/format，先使用窄作用域、版本化的
  M2-A evidence artifact，不扩大通用生产配置白名单；G0 定义必要的最小集成。
- QA 试验许可与已验证能力分开：试验工具可以验证候选 native 路径，但不能提前
  伪造 Measured evidence 来绕过 capability gate。
- CSV/JSON 包含 requested/effective、profile、frame hex/FCS、work/native/returned
  样点数、filter/scale/phase/crop、解码结果/错误、诊断坐标、比较误差、seed、
  revision/dirty hash、编译器/VOLK/加载库、输入/taps/输出 hash、MATLAB provenance。
  未上板明确写 `hardware_readback=null / not_measured`，不伪装设备读回。
- 独立 verifier 不调用待测 C++ 计算 expected，也不只比较另一份相同 JSON；
  验证长度/比例/CRC/坐标及 evidence 权限，主动篡改字段必须失败退出。

## 9. OpenCode 分工（协调者 + 6 个 subagent）

以下是交给 OpenCode 的执行安排，本规划回合没有启动实现 agent。
**所有 agent 不独占仓库，不得覆盖/撤销他人改动；接口变化必须与 owner 协调。**
P0 列出已有未跟踪文档/数据，禁止顺手清理。建议文件名如下，G0 可以统一改名。

| 角色 | 唯一写入所有权 / 职责 | 依赖 |
|---|---|---|
| 协调者 | G0、公开 `uwb_twr_phy.h`（若需要）、CMake/安装、capability/config 最小集成、状态文档 | 审核后冻结 API/manifest；合并门禁 |
| A `m2a_native_resampler` | 新 TX resampler helper 及专属 QA；既有 `uwb_rational_resampler_core.h` 必要最小修改 | G0；独占共享 resampler 文件 |
| B `m2a_frame_phy` | 新 `lib/uwb_twr_phy.cc`，复用 HRP/demod 的全帧流水线 | A 的接口；不得修改 FSM |
| C `m2a_matlab_oracle` | `testdata/twr/m2a/` 的 `.m`、短 golden、manifest/README | G0 独立生成；不从被测结果抄期望 |
| D `m2a_qa` | 新 `qa_uwb_twr_native_phy.cc`、全帧/故障/分配测试 | 先补 QA，依赖 A/B/C；不改生产实现 |
| E `m2a_demo_verifier` | `apps/twr_native_phy_demo.cc`、可选轻量 Python 包装/测试、`tools/twr/verify_m2_a_native_phy.py` | G0 JSON 契约，B/C 输出 |
| F `m2a_reviewer` | 生产/QA 只读，仅写 `docs/twr/M2-A_评审报告.md` | 先审 G0，最后审实际 revision/独立反例 |

B 若需修改共享 `uwb_hrp_mod_core.h` / `uwb_demod_core.h`，先说明缺陷与必要性，
协调者明确授予唯一 owner 并扩展对应历史回归。不能多个 agent 同改公共头。
不是六份相互独立的实现：A/B 只保留一条生产链，C/F 保持参考和审核独立。

## 10. 执行阶段、门禁与性能报告

### P0：确认基线与 oracle 环境

记录 HEAD/dirty、构建配置、共享库加载路径，跑受影响基线与 MATLAB smoke。
读取本文引用的真实执行入口；不要把历史 radar 文档中另一套“M2”当成 TWR M2。
现有 48 行 PHY 白名单、M1-B R01–R09 QA 和有效配置均保持回归。

### P1：G0 + 独立参考 + 失败 QA

冻结 profile/taps/容量/坐标、oracle、容差与 manifest；F 先审。
A 写核 QA，D 写全帧负例，C 建 MATLAB 参考。缺少必需 MATLAB 工具箱时显式阻塞
最终验收，但不阻止完成可做的 C++ 子项。

### P2：两种 rate 的最小全帧闭环

先一个 rate 的 CF32 干净链，再第二个 rate，再 SC16、长度/边界/分块。
每次有意义修改后构建并跑受影响 QA。顺序用于定位，不降低最终两 rate 门槛。

### P3：独立对照、压力与资源观测

完成 MATLAB 双向验证、A01–A14、verifier 变异和 reviewer 反例。
性能分段报告 cold prepare、FCS/调制、TX resample、RX resample、解码；
记录样本量、worker 数、CPU/VOLK、wall time、P50/P95/P99/max、峰值内存及分配。
循环缓存波形和每帧变化载荷分开测试，避免只测 prefix cache 命中。
不从离线耗时承诺设备 reply delay 下限；本阶段不给 1 GS/s 或双 RX 实时通过结论。

### P4：回归、安装与归档

```bash
cmake -S gr-uwb -B gr-uwb/build
cmake --build gr-uwb/build -j8
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -R 'twr|hrp_mod_core|demod_core|rational_resampler_core' -j1 --output-on-failure
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j1 --output-on-failure
python3 gr-uwb/apps/test_twr_config.py
python3 tools/twr/verify_m0_1_findings.py
python3 tools/twr/verify_m1_a_tof.py
python3 gr-uwb/apps/test_twr_fake_demo.py
install_prefix=$(mktemp -d /tmp/twr-m2a-install-XXXXXX)
env -u LD_LIBRARY_PATH bash tools/twr/verify_install_consumer.sh "$install_prefix"
```

新增 CTest 使用包含 `twr` 的名称；新公开接口补实际安装消费者，不要求安装 QA-only
helper。独立 verifier/MATLAB/新 CLI 的最终命令在实现报告列出，不杜撰尚不存在参数。
新路径 ASAN+UBSan，短输出 buffer/极值/flush/reset 必测。
不依赖 sudo install；共享库路径要 ldd 确认，静态核心记录实际 linked target。
benchmark 串行，编译和性能测量分开；历史吞吐失败单列，不降低旧门槛，涉及重采样
公共内核时尤其要同环境前后对照，不能只引用旧“与本次无关”的判断。

普通测试产物写 `build/test-output/twr/m2a/` 或临时目录；显式归档选定短 IQ、
golden 和 provenance 到 `testdata/twr/m2a/`。不让 CTest 覆盖已审核黄金数据。

## 11. 完成判定与后续交接

新增 `docs/twr/M2-A_native_PHY开发报告.md`，逐条列 A01–A14、失败/未测、
MATLAB 实跑证据、最终 revision/hash、独立审核裁决及已知分配边界。
更新状态与路线，保留历史证据及被替代结论；不要把旧 48 行 work 结果改写成 native。

- [ ] 两种 rate、三个真实 TWR 帧、两种 IQ 格式的干净基本单元全部通过。
- [ ] 全帧 PHR/PSDU/FCS 正确；分块/容量/reset/flush 与数字坐标契约通过。
- [ ] MATLAB 核逐样点及双向完整帧交叉实跑；golden 非 C++ 自举。
- [ ] 窄作用域 native evidence 可追溯；Ranging/ToA/hardware/vendor 未越权。
- [ ] buffer 复用和分配实测；既有解调分配、性能限制如实单列。
- [ ] M0/M1、历史 PHY/radar 回归、安装及 sanitizer 通过或有明确独立历史失败证据。
- [ ] reviewer 无未关闭阻断问题；报告锁定实际代码与产物。

满足后仅可写“**M2-A 指定 profile 的离线 native PHY 闭环完成**”。
交给 M2-B 的资产：逐阶段 sample-grid 映射、实际 filter/pulse 延迟、完整帧边界、
解码诊断、数值计划接口缺口清单和阶段耗时。M2-B 再单独冻结 RMARKER 定义、
首径/质量算法、校准模型、分数时间与 timestamp patch 的因果约束。
**M2-A 完成不等于 M2 完成，不自动进入硬件开发。**
