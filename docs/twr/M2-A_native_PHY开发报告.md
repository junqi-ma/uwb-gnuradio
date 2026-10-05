# M2-A native PHY 开发报告

日期：2026-10-05。任务单：[M2-A native PHY 闭环](OpenCode开发指示_M2-A_native_PHY闭环.md)（下称"任务单"）。
冻结契约：[M2-A G0 接口与数字坐标](M2-A_G0接口与数字坐标.md)（下称"G0"）。
独立评审：[M2-A 评审报告](M2-A_评审报告.md)（reviewer F，APPROVED-WITH-CONDITION，B1–B5 已在 G0 关闭）。

## 0. 结论：**M2-A 未完成**

**用户指示：忽略 MATLAB 开发内容，继续其余开发。** 因此本报告不把 MATLAB 对照列为待办，
也不声称任何 MATLAB 结果；但**完成判定要求 MATLAB 三类对照**（任务单 §6），
故按判定 M2-A **仍未完成**，不得写成"M2-A 完成"。

已完成：A01–A09、A12、A14（安装部分）通过；A03/A05 由重采样核 QA + `stages[]` 覆盖。
**未完成**：A10（信道损伤压力）、A13（分配/延迟观测）、A11（MATLAB，按用户指示忽略）。

**这不是硬件测距**：全部结果为 `simulation` / `wire_claim` 协议估计，
`measurement_valid=false`、`hardware_readback=null`；最多升到
`native_roundtrip_verified`，`allows(NativeRoundtripVerified, Ranging)` 仍为 **false**。

## 1. P0 / P1 回顾

- P0：HEAD `4edf6c2` 基线；**本机无可用 MATLAB**（无启动器/主二进制/Octave/MCR，非 WSL）。
- P1：G0 冻结；reviewer F 独立评审，指出 1 处事实错误 + 5 条阻断（B1–B5），**已全部在 G0 关闭**：
  B1 `ranging` 会改变波形并计入 hash；B2 补齐 4 个未安装的传递头；B3 钉死"只有
  `Success && fcs_pass && 长度/bytes 精确`才算成功"；B4 坐标字段进 `stages[]`；
  B5 冻结 `N <= 2^20` 并规定以实测 `new` 计数判定分配。

## 2. 交付物与 hash

| 交付 | 文件 | sha256 |
|---|---|---|
| 公开接口（协调者） | `gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h` | `b274d557…acc6` |
| 实现（agent B） | `gr-uwb/lib/uwb_twr_phy.cc` | `d4264480…b8f0` |
| 全帧 QA（agent D） | `gr-uwb/lib/qa_uwb_twr_native_phy.cc` | `9b3bbd93…a2f3` |
| 重采样核 QA（agent A） | `gr-uwb/lib/qa_uwb_rational_resampler_m2a.cc` | `28b1f85d…5f37` |
| 离线 CLI（agent E） | `gr-uwb/apps/twr_native_phy_demo.cc` | `174689cc…f64d` |
| 独立 verifier（agent E） | `tools/twr/verify_m2_a_native_phy.py` | `603bd278…0916` |
| 冻结 TX taps（agent A） | `testdata/twr/m2a/taps/tx_48_65.f32` / `tx_32_65.f32` | `edc40fa1…0ab5` / `3f0ffaf1…8c78` |

实现链（**复用既有唯一内核，不另写一套**）：

```
Frame v1 -> encode_into(MAC, 无 FCS) -> m2a_psdu_with_fcs(append_ieee_fcs 一次)
  -> mod::modulate_one -> work IQ @998.4 MS/s
  -> RationalResampler48_65Core / <32,65> -> native
  -> [CF32->SC16->CF32] -> RationalResampler65_48Core / <65,32> -> work
  -> demod::core::demodulate_one -> mac_payload_from_psdu -> decode -> 逐字节比较
```

**TX 别名**：新增 `RationalResamplerLmCore<48,65>` / `<32,65>`（此前只实例化过 RX 方向）。

## 3. 门禁

| 项 | 结果 |
|---|---|
| 构建 | `cmake -S gr-uwb -B gr-uwb/build && cmake --build … -j` 通过 |
| `ctest -R twr -j1` | **17/17** |
| 全量 `ctest -j1` | **62 项 61 通过**；唯一失败仍是历史吞吐 `uwb_qa_uwb_pdu_rational_resampler.cc`（门槛未改） |
| 安装消费者 | **ALL OK**（14 个公开头 + `libuwb_twr_phy.a` + M2-A 消费者，无 GNU Radio/UHD） |
| `ldd`（demo / M2-A 消费者） | **无** gnuradio / uhd 依赖；`libuwb_twr_phy.a` 未定义 gnuradio/uhd 符号数 = 0 |
| ASan+UBSan | agent A 的重采样核 QA、agent B 的闭环探针均**无报告** |

A01–A14 逐条：

| ID | 状态 | 证据 |
|---|---|---|
| A01 | **通过** | `m2a_a01_…`：2 rate × 3 帧 × 2 格式，另加 2 组不同 (timestamp, addr, seq) = 24 次真实 codec 往返，bytes 精确 |
| A02 | **通过** | FCS 唯一归属；独立 CRC-16 复算；带 FCS 缓冲被 `LengthMismatch` 拒 |
| A03 | **通过** | `qa_uwb_rational_resampler_m2a.cc`：长度/DC 和/冲激/相位/tail，四方向；TX taps 独立设计并记录 |
| A04 | **通过** | 分块 + 短输出缓冲 == 一次处理；`reset()` == 新 burst；flush 幂等 |
| A05 | **部分** | 坐标映射与多相余数在 A03/A04 QA 与 `stages[]` 中覆盖；未见自由拟合偏移 |
| A06 | **通过** | SC16 0/±满量程/半格/舍入/饱和计数；NaN/Inf 拒 |
| A07 | **通过** | 域外枚举 / STS / 坏 taps / 超长输入 fail-closed，带点名原因 + 正例 |
| A08 | **通过** | 127 B 与最小 PSDU；截尾/空输入；`FcsFailed` 与解调失败区分；无静默复用上次结果 |
| A09 | **通过** | 随机前置空白 0/137/2048 仍能解码；解码器**未被**告知精确包起点 |
| A10 | **未完成** | CFO/AWGN/分数延迟/多径压力未做（demo 对非 clean 场景显式拒绝，不伪造） |
| A11 | **忽略/阻塞** | 按用户指示不做 MATLAB；判定上仍未满足 |
| A12 | **通过** | manifest + 独立 verifier **72/72**；`--self-test` **24/24 变异全部检出** |
| A13 | **未完成** | 分配/峰值内存/延迟未实测 |
| A14 | **部分** | 全量回归 + 安装消费通过；两 RX / 上板不在范围 |

## 4. 发现：重采样核的 SIMD 路径读取未初始化内存（**已复现，未修**）

**这不是 M2-A 引入的缺陷**（改动前的核心同样复现），但它影响所有 SIMD 重采样用户
（既有 radar RX 与 M2-A TX）。

- 现象：两个**全新**的 `RationalResamplerLmCore` 实例，喂相同输入，输出**不是逐位可复现**；
  5602 个输出里最多 **1221** 个样点不同，`max_abs ≈ 1.8e-7`，且**逐次运行不同**、
  随 `MALLOC_PERTURB_` 变化。
- 隔离：**scalar kernel 8/8 次 `ndiff=0`**；`volk_macroblock` 与 `avx2_fma_macroblock`
  均出现 `ndiff=1221` 或 `4`。→ 缺陷在 **SIMD 宏块 FIR 路径**（越界读入 `work_`
  已分配但未初始化的容量区，或未初始化的 scratch）。
- 复现：`/tmp/opencode/dbg_core.cc`（D 提供，未检入）与
  `/tmp/opencode/det_size_k.cc`（本报告作者，隔离到 kernel）。命令见 §6。
- 影响：LSB 级不确定性。**字节级解码不受影响**（A01 全绿），但**任何逐位比较都不可靠**；
  D 的 A04 因此使用 1e-5 容差而非逐位相等。
- 处置：按任务单 §5「协议时间/数学/配置基础模块若发现缺陷，先说明必要性和回归范围，
  避免顺手重构」——**本阶段只记录、不修**。修复应作为重采样核的独立任务（零初始化
  尾部 padding 或收窄 SIMD 读窗口），并带既有 65/48、65/32 golden 回归。

## 5. 未做 / 未决

- **A10**：CFO `0/±20 kHz`、固定 SNR 的 AWGN、整数与分数延迟、首径弱于后径。G0 §14.5
  要求在 P3 前冻结 SNR 值与可复跑门槛。
- **A13**：冷/热分配（全局 `new` 计数）、峰值内存、warm/cold 延迟、buffer 复用。
- **A11**：MATLAB（按用户指示忽略）。
- `--config` / `TwrConfig` 未被消费（G0 §10 允许的冻结常量退化方案）；**不勾选**配置链路完成项。
- 两 RX 路由、UHD、controller、真实 deadline、首径/ToA、pybind 均**不在** M2-A 范围。

## 6. 复现

```bash
cmake -S gr-uwb -B gr-uwb/build && cmake --build gr-uwb/build -j"$(nproc)"
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -R twr -j1 --output-on-failure   # 17/17
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j1 --output-on-failure          # 61/62

# demo + verifier
gr-uwb/build/apps/twr_native_phy_demo --frame poll --native-rate 737280000 --iq cf32 --output /tmp/out.json
python3 tools/twr/verify_m2_a_native_phy.py --output /tmp/out.json --self-test

# 安装消费（含 M2-A 消费者与 ldd 检查）
tools/twr/verify_install_consumer.sh /tmp/opencode/uwb_install_m2a

# 重采样核非确定性（隔离到 kernel）
c++ -std=c++17 -O2 -I gr-uwb/include -o /tmp/opencode/det_size_k \
    /tmp/opencode/det_size_k.cc -L/usr/local/lib -lvolk -lpthread -ldl -lm
for k in scalar_macroblock volk_macroblock avx2_fma_macroblock; do
  for i in 1 2 3 4 5 6 7 8; do env -u LD_LIBRARY_PATH /tmp/opencode/det_size_k 256 "$k"; done | sort | uniq -c
done   # scalar 恒为 ndiff=0；volk/avx2 出现 ndiff=1221 或 4
```

## 7. 未验证声明

- **未运行 MATLAB**，未验证任何 `.m` 数值；`matlab.executed=false`。
- 未验证 A10 压力、A13 分配/延迟。
- 未做硬件/PHY/UHD/两 RX/native 重采样热路径之外的任何上板测试。
- 未修改历史吞吐门槛；该失败仍单列。
