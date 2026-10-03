# M2-A G0 独立评审意见（reviewer F）

日期：2026-10-03。性质：**独立评审（只读）**，不是 G0 的复述，也不是实现/验收报告。
依据：[M2-A 任务单](OpenCode开发指示_M2-A_native_PHY闭环.md)（下称"任务单"）§5（reviewer 先审 G0）、
§9（reviewer 独立审核，不转述协调者文档）。
审核对象：[M2-A G0：接口与数字坐标（冻结稿）](M2-A_G0接口与数字坐标.md)（下称"G0"）。

**本次审核的代码 revision：`4edf6c2`**（`feature/uwb-ds-twr`；已跟踪工作区除 `开发状态.md`
外无未提交改动，与 G0 §0.1 一致）。本机实测 hash：

```
sha256(docs/twr/M2-A_G0接口与数字坐标.md)                     = eb2b459072fcb012f42f80b5d03642ffa988d23cedf507cafadca27c15c6c0e8
sha256(gr-uwb/lib/uwb_twr_core.cc)                            = f378f4419af0f9bfdadfc3c4a24e283163c7c0a2bbf12e8dfadb25938614f4a9
sha256(gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h) = 006901ef11ca5578b51b46ae51894806aeeb58f41f0cbb93c2b54980895a554a
sha256(gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h)        = 636aec1326d9d5fdebf619dd07a6e269ec246c19f5b6cdf144945924e0568912
sha256(gr-uwb/include/gnuradio/uwb/uwb_demod_core.h)          = f1dbf344af9e09cff7561617f32893b4c04c30007986b217ee7e6945654f48fc
sha256(gr-uwb/include/gnuradio/uwb/uwb_twr_frame.h)           = 69ccb0f53ad116f15faca9112f6ecd7ff6771aebbe8fcbb26c5ce0a78087d0a2
sha256(gr-uwb/include/gnuradio/uwb/uwb_twr_capability_evidence.h) = ee8f80bcddb748cf44c6abec6f8548287794d9d96110d70850a4b6688d209fbd
sha256(gr-uwb/include/gnuradio/uwb/uwb_twr_config.h)          = c1f8e5e92178740eabd4ca24c97056390b4426bd2acc8f932499dbf60ac8144c
```

> `uwb_twr_core.cc` 的 hash 与 G0 §0.1 声称的 `f378f441…f4a9` **一致**（独立复算）。
> 本意见只对上述 revision/hash 负责；G0 若被修改，需重新确认受影响条目。

---

## 0. 结论速览

**总裁决：APPROVED-WITH-CONDITION（有条件通过）。**

G0 的方向、边界与绝大多数事实性陈述经源码独立核对**成立**：它把 M2-A 牢牢限制在
"离线 native PHY 闭环 + 数字坐标"，没有越权到 ToA/硬件/测距，且明确标注 MATLAB 缺失、
M2-A 未完成。但 G0 存在 **1 处事实错误**、**若干未冻结项**，以及 **5 条必须先写入 G0 的
阻断条件（B1–B5）**。在 B1–B5 落到 G0 文本之前，**不应开始 P2 主链实现**（任务单 §5）。

| G0 节 | 结论 | 一句话 |
|---|---|---|
| §0 结论与范围 | APPROVED-WITH-CONDITION | 范围/边界正确；MATLAB 不可用属实；"与远端同步"未验证 |
| §1 profile 与容量 | **BLOCKING（B1）** | `ranging` "不影响波形" 是**错的**；SFD/脉冲 hash 未冻结 |
| §2 形态与所有权 | **BLOCKING（B2）** | 安装消费者拿不到 `uwb_demod_core.h`/`uwb_phy_profile.h`（未安装） |
| §3 重采样契约 | APPROVED-WITH-CONDITION（B5） | 长度/因果 upfirdn/N=0 正确；"热路径零分配"与 2^22 上界矛盾 |
| §4 帧/FCS/PSDU | APPROVED-WITH-CONDITION（B3） | FCS/长度/解码拒绝全部核实；payload 回退路径未冻结 |
| §5 数字坐标 | APPROVED-WITH-CONDITION（B4） | 映射公式与代码一致、spec 文档确与其不符；字段未进 JSON |
| §6 比较器与容差 | APPROVED-WITH-CONDITION（B3） | 容差合理；未点名回退候选不得当成功 |
| §7 SC16 | APPROVED | 公式/舍入/NaN 策略明确、可测 |
| §8 manifest JSON | APPROVED-WITH-CONDITION（B4） | schema 缺 §5 要求的 origin/trim/padding/逐阶段计数 |
| §9 capability 边界 | APPROVED | 硬边界与源码一致（Ranging 仍拒） |
| §10 CLI | APPROVED-WITH-CONDITION | `--config` 可退化为冻结常量，但须标注未消费 |
| §11 MATLAB oracle/阻塞 | APPROVED | 正确保持"MATLAB 待实跑 / M2-A 未完成" |
| §12 A01–A14 映射 | APPROVED-WITH-CONDITION | 映射齐全；A10 SNR 值推迟到实现期 |
| §13 分工与所有权 | APPROVED-WITH-CONDITION | 与任务单 §9 一致；安装依赖同 B2 |
| §14 未决项 | APPROVED-WITH-CONDITION | 5 项均给出 reviewer 意见（见 §6） |

---

## 1. 独立核对的事实（不是复述 G0）

下表逐条对照"G0 的说法 → 实际源码证据"。凡我核对成立的，标明 file:line。

### 1.1 重采样

- **长度契约** `Lout = ceil(((N-1)*L + T)/M)`：G0 §3.1 与
  `uwb_rational_resampler_core.h:165-174` 一致（`num<=0` 返回 0）。
- **因果 upfirdn，不是中心裁剪**：G0 §3.1 与 `uwb_rational_resampler_core.h:12-16`
  的 `y[m] = sum_k h[(m*M mod L)+L*k] * x[floor(m*M/L)-k]`（`x` 在 `[0,N)` 外为 0）
  一致；实现见 `:249-328`、`:782-880`。核心注释本身也把 `resample_poly` 的中心裁剪
  与 `upfirdn` 全卷积区分为不同契约。**成立。**
- **只实例化 `<65,48>`/`<65,32>`**：别名只有
  `uwb_rational_resampler_core.h:990-991`；全仓库（排除 build/）无
  `RationalResamplerLmCore<48,65>`/`<32,65>` 实例。**成立**（G0 §3.3）。
- **`map_input_offset_to_output` 用 `+d`/`llround`/`clamp0`**：
  `uwb_rational_resampler_core.h:410-418` 用
  `d = 0.5*(T-1)`、`m=(p*L+d)/M`、`std::llround(m)`、`r<0?0:r`。块级实现同律：
  `uwb_pdu_rational_resampler_ccf_65_32.cc:286-313` 用整数式
  `(2*p*L + (T-1) + M)/(2*M)`。QA 的本地复刻亦同
  （`qa_uwb_twr_timing_budget.cc:299-308` 注释 `round((p*interp+(T-1)/2)/decim), clamped at 0`）。
  **G0 的 pin 正确。**
- **spec 文档与代码矛盾**：`docs/performance/规格_固定65_48重采样core契约.md:96`
  确为 `m_tag = ceil((p·L − d)/M), d=(T−1)/2`，与代码的 `round((p·L + d)/M)` 不同。
  **G0 声称"矛盾"在字面上成立**（但见 §5 对"是否同一量"的保留意见）。

### 1.2 HRP 调制 / FCS

- **`modulate_one` 不追加 FCS**：`uwb_hrp_mod_core.h:897-934` 直接把入参当 PSDU
  编码（`:939` `bytes_to_lsb_bits(psdu, psdu_len, …)`），无 FCS 追加。**成立。**
- **`append_ieee_fcs` 追加一次、小端**：`uwb_hrp_mod_core.h:444-450` 先算
  `crc16_802154`，push 低字节再高字节。**成立。**
- **CRC-16 反射多项式 0x8408、初值 0**：`uwb_demod_core.h:1406-1422`。**成立。**
- **`kMaxHrpTxSamples = 2^22`**：`uwb_hrp_mod_core.h:51`（`4194304`）。**成立。**
- **`pulse_tail_extra(Legacy)=0` / `kPulse48`**：`:111-124`、`:403-408`。**成立。**

### 1.3 帧 codec

- **MAC/PSDU 长度 14/24/29 与 16/26/31**：
  `uwb_twr_frame.h:35`、`:631-640`（`frame_length_for` / `psdu_length_for`）。**成立。**
- **`decode()` 拒绝带 FCS 的缓冲（`LengthMismatch`）**：
  `uwb_twr_frame.h:1477-1487`（`n>need` → `LengthMismatch`，注释直指 FCS 未剥）。
  `mac_payload_from_psdu` 剥 2 B：`:1540-1570`。**成立。**
- **codec 不写 FCS；`frame_profile_validate` 拒绝 `fcs_appended_by_modulation_layer=false`**：
  `:616-621`。**成立。**
- **PHR 13 信息位 / 19 编码位 / 21 符号**：`uwb_twr_frame.h:792-801`、
  `uwb_hrp_mod_core.h:44-46,579-602`。**成立。**

### 1.4 解调状态

- **`FcsFailed` 与 `TimingFailed` 是不同状态**：`uwb_demod_result.h:31-43`
  （`TimingFailed=2`、`FcsFailed=7`）；`uwb_demod_core.h:2162`（timing 失败）、
  `:2360-2362`（payload ok 但 `fcs_pass` 假 → `FcsFailed`）。**成立。**
- **`payload.bytes` 含 FCS**：`uwb_demod_core.h:1962-1983` 把 `out.bytes` 设为整段
  PSDU，`calculated_fcs` 只覆盖 `size()-2`，`received_fcs` 取末 2 B；
  `uwb_demod_result.h:157-164` 注释 "decoded PSDU bytes"。**成立。**

### 1.5 capability / 白名单

- **`allows(NativeRoundtripVerified, Ranging) == false`**：
  `uwb_twr_capability_evidence.h:203-219`（Ranging 需 `ToaVerified`）、
  `:246-249`；`level_implies(2,3)=false`。**成立。**
- **`derive_row_evidence()` 把 native 路径降级为 `None`**：
  `:513-517`（`path_claims_native_roundtrip` → `established=None`）。**成立。**
- **48 行白名单恒为 `WorkDecodeVerified`**：`uwb_twr_config.h:2029-2083`，
  `:2068` 显式 `evidence::Level::WorkDecodeVerified`，`:2071-2078`
  把 native/toa/hardware/vendor 列为 `not_established`。Python 侧确有 48 行钉死：
  `apps/test_twr_config.py:584,819,870,1463,1535`。**成立。**

### 1.6 MATLAB 可用性

G0 §0.1 称本机 MATLAB 不可用。**独立核实属实**：`/usr/local/MATLAB/R2024a`
存在（2.3 G，`toolbox/` 仅 13 个目录），但
`/usr/local/MATLAB/R2024a/bin/` 下**只有 `glnxa64/`、`icutzdata/`，没有 `matlab`
启动器**；`find` 全树无 `matlab`/`MATLAB`/`matlab.exe`；`command -v matlab` 为空；
`/mnt/` 下只有 `data`、`Data`，无 `/mnt/c`（非 WSL）；无 Octave。
**因此 G0 §11 保持"M2-A 未完成，MATLAB 待实跑"是正确的。**

---

## 2. G0 中的事实错误

### E1（BLOCKING，B1）—— §1 称 `ranging` "不影响波形"，与代码相反

G0 §1 表格末行写：`ranging | true（…都置 PHR ranging bit）| …仅影响 PHR bit 9，不影响波形`。

实际：`cfg.ranging` 进入 `encode_phr19` 的 `sys13[9]`
（`uwb_hrp_mod_core.h:590`），再经 `hrp_secded` → `scratch.conv_in`
（`:945-946`）→ `convenc_cl3` → `g0/g1` → `bpm_add_pulses`
（`:1006-1021`），**直接改变 PHR 的 BPM 符号与整段 work IQ**。此外 MAC flags 的
ranging 位（`uwb_twr_frame.h:174`、`:1358-1360`）属于帧字节，也会改变波形。
"不影响波形" **错误**。

影响：若实现者据此认为 ranging 不进波形/hash，冻结 config 与
`input_sha256`/`output_sha256` 会漏掉一个真实变量；A01 的两组帧若只差 ranging 位，
波形仍会变。**必须在 G0 中改正**，并明确 ranging 计入冻结配置与产物 hash。

### E2（非阻断，措辞）—— §3.3 "DC 增益 = Interp"

表列把 `<48,65>`/`<32,65>`/`<65,48>` 的 "DC 增益" 写成 48/32/65。源码核心
**不做任何缩放**（`uwb_rational_resampler_core.h` 无 scale），taps 和等于 Interp 时
系统的**有效直流增益为 1**；48/32/65 是"taps 之和"这一设计约束，不是输出幅度倍数。
文字部分（"taps 的 DC 和必须等于 Interp"）正确，但表头 "DC 增益" 易被读成波形被放大 48 倍。
建议改为"taps DC 和（=Interp，有效增益 1）"。

### E3（非阻断，单位）—— §5 `(T-1)/2` 的维度和奇偶

- 代码用 `0.5*(T-1)`（`uwb_rational_resampler_core.h:412`），G0 写 `(T-1)/2`。
  对**偶数 T** 两者不同（整数截断 vs 保留 .5），而 `llround` 对 .5 敏感，可差 1 个输出样点。
  必须写成 `0.5*(T-1)` 的精确形式。
- G0 把该量标为"输入样点"。在 `map` 中 `p*L` 是**内插域**下标，`d` 加在其上，
  故 `d` 是**内插（上采样）域样点**，换算到输入端为 `(T-1)/(2L)`。§5 要求
  "数字滤波群时延…记录"，因此单位必须无歧义，否则 A05 的"可逆/可解释"无从判定。

---

## 3. G0 的遗漏 / 未冻结项

### M1（BLOCKING，B2）—— 安装依赖头未覆盖

G0 §2 决定把 `uwb_twr_phy` 做成独立静态库并安装公开头 `uwb_twr_phy.h`，
§13 由协调者所有安装，§14.1 以 `ldd` 实测为准。但：
- `uwb_hrp_mod_core.h` include `uwb_demod_core.h` 与 `uwb_phy_profile.h`；
- 安装列表 `gr-uwb/include/gnuradio/uwb/CMakeLists.txt:11-100` **只安装了
  `uwb_rational_resampler_core.h`（:26）与 `uwb_hrp_mod_core.h`（:50）**，
  **没有安装 `uwb_demod_core.h`、`uwb_demod_result.h`、`uwb_phy_profile.h`**
  （grep 计数为 0）。

后果：安装消费者 `#include <gnuradio/uwb/uwb_twr_phy.h>` 会因传递依赖缺头而编译失败；
A14 "安装消费不用 SKIP 代替" 将无法成立。`uwb_twr_core` 的先例之所以成立，是因为它的
全部依赖头都已被安装——**该先例不能直接搬到 PHY helper**。

G0 必须二选一并写入：**(a)** 把 `uwb_demod_core.h`/`uwb_demod_result.h`/
`uwb_phy_profile.h`（及被它们拉入的其它头）加入安装列表；或 **(b)** 让
`uwb_twr_phy.h` 不暴露这些内部头（opaque/PIMPL），消费者只依赖已安装头。

### M2（BLOCKING，B3）—— payload 回退路径未冻结

任务单要求 M2-A 复用完整 PHR/payload/FCS 路径；G0 §4 只说"复用"，§5 只提
SFD/packet-start 诊断，§6 只说"bytes/FCS 精确一致"。但 `stage_payload_fcs`
（`uwb_demod_core.h:1909-2089`）在"生产路径"FCS 不过时会继续跑
**±2 长度邻域 × 正反 SFD 极性 × 软 Viterbi** 的候选回退，并可能
`cand.ok=true` 而 `fcs_pass=false` 后返回 `true`（`:2080-2089`），
由 `demodulate_one` 置 `FcsFailed`（`:2360-2362`）。

G0 必须显式钉死：闭环"成功"判据是
`status==Success && payload.fcs_pass==true && payload.bytes.size()==期望 PSDU &&
bytes 精确`；**任何回退候选、任何 `FcsFailed`、任何长度不等于期望的结果都判失败**，
且失败样本计入统计、不得因 FEC 纠错而把"错误 FCS 但编码合法"的负例当作成功。

### M3（BLOCKING，B4）—— §5 字段未进入 §8 schema

G0 §5 要求每阶段记录 `rate、0-based origin、input/output count、L/M、phase、trim、
padding、filter delay、有效区间`；但 §8 的 `twr-m2a-native/1` schema 只有
`samples.{work_tx,native,work_rx,returned}`、`filter.{scale,phase,crop,group_delay,
valid_from,valid_to}`，**缺 `origin`、`trim`、`padding`、逐阶段 input/output count、
逐阶段 rate/L/M**。A05/A12/A13 要验证的正是这些坐标，schema 不含就无法由独立 verifier
核验。必须扩充 schema，或明确这些字段落在哪个已冻结文件。

### M4（BLOCKING，B5）—— "热路径零分配"与 2^22 上界自相矛盾

G0 §3.5：核心 ctor 预留 `H_+2^20`、两块 `2^20`（
`uwb_rational_resampler_core.h:126,147-149`），`process` 仅在 `ninput` 超预留时
`reserve`（`:279-282`）；而 G0 同时把单帧 work 波形上界取 `kMaxHrpTxSamples=2^22`
（`uwb_hrp_mod_core.h:51`），并声称"使热路径零分配"。核心**没有公开 reserve**，
若真的允许 2^22 输入，首帧即触发 `reserve`。实际 64 SYNC + 127 B 帧约
`64*1016 + 8*1016 + 21*512*2 + nsym*128 ≈ 2.5e5` work 样点，远小于 2^20。
G0 必须冻结**具体的单帧上限数值**（并据此在准备阶段预热容量），或撤下"零分配"承诺。
A13 的判定以实测全局 `new` 计数为准，不能靠"上界声明"。

### M5（非阻断）—— 既有常量 hash 未冻结

G0 §1.2 把"具体产物 hash"全部推迟到实现报告，但 SFD 序列（`kSfdIeee`，
`uwb_phy_profile.h:116`，8 符号已核实）与脉冲系数 `kPulse48`
（`uwb_hrp_mod_core.h:111-124`）是**已存在的冻结资产**，本可在 G0 现在就记
`sha256`。任务单 §3 明确要求"冻结系数、增益、长度和 hash"。

### M6（非阻断）—— 其它

- §1 未给出 64 SYNC/127 B 的**具体波形样点数**（只有公式）。建议在 G0 附一个数值表，
  供 A08/A13 直接引用。
- §1 的 PHR 行"0.85 Mb/s"指 PHR 自身符号率；`encode_phr19` 写入的速率字段是
  **6.81 Mb/s（index 2）**（`uwb_hrp_mod_core.h:585-589`）。建议明写以免混淆。
- §12 A10 的 AWGN SNR 值推迟到 C/D 实现期（§14.5）。任务单允许，但应在 **P3 前**
  以 G0 附录冻结，否则 A10 无可复跑门槛。

---

## 4. 可能误导 reviewer 之处

- **§8 schema 的 `evidence.level` 示例硬编码为 `native_roundtrip_verified`/`kind=measured`**，
  而 §9 首条说"默认输出只是 decode-only/simulation"。二者可自洽（`measurement_valid=false`
  已表达"不产出距离"），但读者可能把示例当默认。建议注明该示例是"实际实测的 M2-A
  artifact"，并给出未实测时的取值。
- **§9 对 capability 的约束很强（正确）**：`allows(NativeRoundtripVerified,Ranging)==false`、
  不改 48 行、不走 `derive_row_evidence`。这三条我已逐条核实，G0 没有放松边界。
- G0 全文没有把 sample-grid 坐标、SFD/packet-start 说成 RMARKER/ToA；
  §0/§5/§8/§9 都反复声明 `measurement_valid=false`、`hardware_readback=null`、
  不给 M1-B admission 加 bypass。**这一点做得对，是本次评审最认可的部分。**
- §2 的独立静态库是**条件性**表述（`ldd` 实测 + 退回方案），不是已成立的结论；
  但配合 M1（B2）的缺头问题，退回方案本身也必须解决传递头安装。

---

## 5. 对 G0 §14 五个未决项的 reviewer 意见

1. **`uwb_twr_phy` 是否独立静态库**：**有条件同意**。先按 B2 解决
   `uwb_demod_core.h`/`uwb_phy_profile.h`/`uwb_demod_result.h` 的安装或隔离；
   再以 `ldd`/`readelf` 实测。带出 `libgnuradio-*` 即退回"仅安装头 + 消费者测试"，
   且退回方案同样要满足 B2。
2. **`--config` 闭环**：**同意** §10 的退化方案，但必须写明"未消费 `TwrConfig`"、
   不勾选 A12 的配置链路完成项，并保留 `--config` 未实现时的显式拒绝。
3. **TX taps 来源**：**同意** §3.3 的"两条都算并记录差异"。强调：独立设计脚本、
   DC 和 = Interp、截止/阻带指标、与重标定 RX 原型的逐样点差异、`design.json` +
   `sha256`，缺一不可；**不得**用旧 CSV 或盲目反转比例。
4. **§5 文档矛盾**：**同意以代码/QA 为准**。但请注意：spec 文档的
   `m_tag = ceil((p·L − d)/M)` 是"tag offset 映射律"，与核心的
   `map_input_offset_to_output` 未必是同一物理量。G0 应(a)在报告里证明二者是否
   指向同一映射，若确为同一量则明确"文档已过时并修正"，若为不同量则分别命名、
   不要笼统称"矛盾"；(b)按 E3 修正 `(T-1)/2` 与群时延单位。
5. **A10 AWGN SNR**：**同意**推迟，但须在 P3 前以 G0 附录冻结，且
   `0/±20 kHz` CFO、整数/分数延迟、首径弱于后径、固定 SNR 全部要有可复跑命令与门槛。

---

## 6. 必须在实现前关闭的阻断条件（汇总）

| ID | 阻断内容 | 落点 |
|---|---|---|
| B1 | 改正 §1 `ranging` "不影响波形"；ranging 计入冻结配置与 hash | G0 §1 |
| B2 | 解决 `uwb_demod_core.h`/`uwb_phy_profile.h`/`uwb_demod_result.h` 的安装或隔离 | G0 §2/§13/§14.1 + `include/.../CMakeLists.txt` |
| B3 | 钉死 payload 回退路径：只有 `Success && fcs_pass && 长度/bytes 精确` 才算成功 | G0 §4/§5/§6/§11 |
| B4 | 把 §5 要求的坐标字段补进 `twr-m2a-native/1` schema（或指明落点） | G0 §5/§8 |
| B5 | 冻结具体单帧上限并据此预热容量，或撤下"热路径零分配"承诺 | G0 §3.5/§13 |

> B1–B5 均为 **G0 文本/契约修正**，不要求改生产代码。写入 G0 后即可进入 P2；
> 建议把修正版 G0 交回 F 复看（或至少由协调者逐条引用 B1–B5 记录关闭）。
> 在此之前，任务单 §5 的"reviewer 同意"**尚未无条件给出**。

---

## 7. 我**没有**验证的内容（明确声明）

- 未重新运行任何构建、CTest、demo、verifier 或 MATLAB；本报告不重复任务单 §1
  的"5/5 通过"等既有结论。
- 未运行 MATLAB（本机不可用，见 §1.6），也未验证 `UWB_demodulation/*.m` 的数值正确性。
- 未验证"仓库与远端同步"（G0 §0.1），仅确认本地 HEAD 与工作区状态。
- 未验证 channel 5 中心频率、BPRF mean PRF 62.4 MHz、RF 带宽等**硬件/标准**事实；
  G0 已声明这些离线不证，我同意。
- 未验证 TX/RX taps 的实际 DC 和、截止、阻带（这是 G0 §3.3 要求实现者独立证明的设计项）。
- 未编译/链接尚不存在的 `uwb_twr_phy`，故 B2 的 `ldd` 结论是"预判"而非实测。
- 未逐条复核 M1-B R01–R09 的关闭（超出本任务范围；仅独立复算了 `uwb_twr_core.cc` hash）。
- 未验证 §1.1 各帧波形样点数的**运行值**（仅按 `packet_samples_998p4` 公式推导，
  结论为 Poll/Response/Final 的 PSDU=16/26/31 B，64 SYNC+ieee+Legacy 下约
  1.17e5/1.17e5/1.18e5 work 样点量级）。

---

## 8. 复核方法（可复跑）

只读：阅读任务单、G0、以及下列源码；未修改任何生产/QA 文件，唯一写入为本文件。

```text
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h
gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h
gr-uwb/include/gnuradio/uwb/uwb_demod_core.h
gr-uwb/include/gnuradio/uwb/uwb_demod_result.h
gr-uwb/include/gnuradio/uwb/uwb_twr_frame.h
gr-uwb/include/gnuradio/uwb/uwb_twr_capability_evidence.h
gr-uwb/include/gnuradio/uwb/uwb_twr_config.h
gr-uwb/include/gnuradio/uwb/CMakeLists.txt
gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_32.cc
gr-uwb/lib/qa_uwb_twr_timing_budget.cc
docs/performance/规格_固定65_48重采样core契约.md
```

关键命令（均已实跑）：

```bash
git -C . rev-parse HEAD
sha256sum gr-uwb/lib/uwb_twr_core.cc docs/twr/M2-A_G0接口与数字坐标.md
grep -rn "RationalResamplerLmCore<" gr-uwb --include=*.h --include=*.cc | grep -v build/
grep -rn "map_input_offset_to_output" gr-uwb/include gr-uwb/lib | grep -v build/
grep -c "uwb_demod_core.h\|uwb_phy_profile.h\|uwb_demod_result.h" \
    gr-uwb/include/gnuradio/uwb/CMakeLists.txt     # -> 0
command -v matlab; find /usr/local/MATLAB -name matlab -o -name MATLAB
```

---

**裁决：APPROVED-WITH-CONDITION。** 关闭 B1–B5 后，M2-A 可按任务单 §5/§10 进入 P2；
MATLAB 缺失导致的"未完成"状态在 G0 中已被正确保留。
