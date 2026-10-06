# M2-A 最终实现独立评审报告（reviewer E）

日期：2026-10-06。性质：**独立复核（只读）**，不是收尾开发报告，也不代签协调者的完成声明。
依据：[M2-A 收尾指示](OpenCode开发指示_M2-A收尾_2026-10-06.md)（下称"收尾单"）§7/§8、
[M2-A G0 接口与数字坐标](M2-A_G0接口与数字坐标.md)（下称"G0"，含附录 A.1–A.6）、
[M2-A native PHY 闭环任务单](OpenCode开发指示_M2-A_native_PHY闭环.md)、仓库 AGENTS.md。

**本次审核的代码 revision：`1002d633e9a87e004e0201ddc96d3cf421cd5f86`**（`feature/uwb-ds-twr`，
工作区对**已跟踪文件无未提交改动**；`git status --porcelain` 仅有未跟踪实验数据/文档）。
本报告审核的是**最终实现**，不是对 `4edf6c2` 的 G0 评审（`M2-A_评审报告.md`）；两者结论不互相代替。

本机实测关键文件 hash（`sha256sum`）：

```
gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h                = cc7c2301fb8e684b907f162e281caa2c503c85c3d3fce719061cf83c81f3025e
gr-uwb/lib/uwb_twr_phy.cc                                = 3c4cd7dd681958faaff4f01fef463707cc1772d9cd2738e01ae9e8e9720e43d6
gr-uwb/lib/uwb_twr_phy_impairment.cc                     = 8d0782ab952009e19c41853521a26a38b3c751bf8b3fa813c362f910d3f43bbd
gr-uwb/lib/qa_uwb_twr_native_phy.cc                      = 1ed1053a8b3565575cd6d7e2f2bac726f03e530320442ece7b700e78a5072a4f
gr-uwb/lib/qa_uwb_twr_phy_impairment.cc                  = 25df1b747c34f0c1f17a8371e907c26cd8cb2511a24f4e353a47cfaeaff4964e
gr-uwb/apps/twr_native_phy_demo.cc                       = 78d8f95d8417b5962c55be6407716f3e91c11901fb4264cd55febc2d38683e86
gr-uwb/apps/twr_m2a_bench.cc                             = bdd1f158b6bc06365b438b53e8b6d8ea0de203d170d2fddb3a87031786ac4839
tools/twr/verify_m2_a_native_phy.py                      = 6fe703ea6656e80841e4d09b0da3d550bdd303269273f8d182e6904f2a9813f3
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h = d0b9af704164a5c6e1f8d3451dfcda065e7af4f173605bf66803cfcd9045e1d4
```

> 我只创建了 `docs/twr/M2-A_最终实现评审报告.md`；未修改任何生产代码、QA、demo、verifier、
> 测试数据或文档，也未"放宽"任何验收。所有反例探针写在 `/tmp/opencode/` 下。

---

## 0. 结论速览

| 项 | 裁决 | 一句话 |
|---|---|---|
| **A05 数字坐标** | **部分（非阻断）** | 四个 stage 的坐标字段已随 `stages[]` 输出，guard 已把**两个**对外诊断坐标 rebase 回 work_rx 网格；但存在**单位标注反例**与**未 rebase 的其它 DemodResult 坐标**（见 §3.1、§3.2）。 |
| **A10 native 损伤** | **通过（模型+QA），证据归档不全（非阻断）** | 顺序/CFO 相位律/有效区 SNR/确定性 seed/延迟/多径与 G0 A.3 一致；独立 QA 直接测量模型量（13 例）。但**≥10 seed/基本单元的冻结 seed 清单未随仓库归档**（CLI 支持逐 seed 运行）。 |
| **A13 分配与延迟** | **部分（非阻断）** | prepared context 复用 core/scratch、`prepare` 原子、容量计入 guard/tail、`reset` 可恢复（均独立复现）；bench 实测 prepared 92 分配/帧 vs cold 153、p50 10.87 ms vs 19.26 ms、RSS 平。但 bench **未按阶段拆分分配**（G0 A.4 要求），"owned workspace 零分配"是设计推断而非分项实测。 |
| **A12 manifest/verifier** | **通过** | verifier 纯 Python 独立复算（120–125 checks），成功清单 **53/53 变异全检出**；独立构造的"失败→成功""证据越界"均被拒。 |
| **A14 回归/安装** | **部分（非阻断）** | 全量 CTest **62/63**（唯一失败是历史 PDU 吞吐，门槛未改）；安装消费者 ALL OK、Python 全绿。但安装消费者**未实际消费 `M2aContext`**；无 ASAN/UBSan 证据。 |
| **SIMD 非确定性** | **仅记录、未修（符合收尾单 §2）** | 独立复现：scalar 恒 ndiff=0，volk/avx2 交替 ndiff=1221/4；closeout 两提交**未触碰**该核（含算法）。G0 §1.2 的核 hash 已过期（见 §3.5）。 |
| **A11 MATLAB** | **未满足（按用户指示未做）** | 全仓库 `matlab.executed=false`，`.m` 脚本保留未跑。按完整口径 A11 仍未满足。 |

**总裁决：本轮非 MATLAB 子项在代码/证据层面基本成立，无新的**代码级**阻断；但存在 1 个
**closeout 交付完整性阻断**（收尾开发报告缺失、`开发状态.md` 未更新）与若干非阻断契约缺口。**
最大可支持结论见 §6。

---

## 1. 方法与实测基线

只读审核：阅读收尾单、G0（A.1–A.5）、公开头、实现、损伤模型、QA、demo、bench、verifier；
在 `/tmp/opencode/` 下**新写并编译独立探针**（`probe_ctx.cc` 生命周期、`probe_diag.cc` 坐标）
与复现程序，不修改仓库任何文件。所有命令均以 `env -u LD_LIBRARY_PATH` 清除旧 `/usr/local` 干扰。

### 1.1 构建与回归（本机实跑）

```bash
cmake -S gr-uwb -B gr-uwb/build          # rc=0
cmake --build gr-uwb/build -j8           # rc=0
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build \
    -R 'twr|rational_resampler|hrp_mod_core|demod_core' -j1 --output-on-failure
# 27/28 通过；唯一失败 = uwb_qa_uwb_pdu_rational_resampler.cc（历史吞吐门槛 >400，实测 213.45 PDU/s）
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j1 --output-on-failure
# 62/63 通过；唯一失败同上；总耗时 137 s
```

安装消费（收尾单 §8 原命令）：

```bash
install_prefix=$(mktemp -d /tmp/twr-m2a-closeout-XXXXXX)
env -u LD_LIBRARY_PATH bash tools/twr/verify_install_consumer.sh "$install_prefix"
# rc=0，VERIFY INSTALL CONSUMER: ALL OK
```

Python 门禁：

```bash
python3 gr-uwb/apps/test_twr_config.py        # OK
python3 tools/twr/verify_m0_1_findings.py     # 15/15
python3 tools/twr/verify_m1_a_tof.py          # 4/4
python3 gr-uwb/apps/test_twr_fake_demo.py     # OK
```

### 1.2 demo 与 verifier（clean + 多损伤场景）

```bash
D=gr-uwb/build/apps/twr_native_phy_demo
$D --frame final --native-rate 491520000 --iq sc16 --output clean_final_sc16.json
#   ok: work_tx=132544 native=65294 work_rx=132711 sc16_saturated=31   （与收尾单 §1.1 一致）
$D --frame poll --native-rate 737280000 --iq cf32 --scenario delay --delay-int 17 \
   --delay-frac-num 1 --delay-frac-den 4 --output delay.json
$D --frame response --native-rate 737280000 --iq sc16 --scenario combo \
   --cfo-hz 20000 --awgn-snr-db 30 --awgn-seed 0 --output combo.json
```

verifier `--self-test`（成功清单）：

| 场景 | 变异检出 | 结果 |
|---|---|---|
| clean/final/491.52/sc16 | **53/53** | SELF-TEST PASSED |
| delay（17+1/4） | **53/53** | SELF-TEST PASSED |
| combo（CFO+AWGN，sc16） | **53/53** | SELF-TEST PASSED |
| cfo（−20 kHz） | **53/53** | SELF-TEST PASSED |
| awgn（10 dB, seed 3） | **53/53** | SELF-TEST PASSED |
| multipath（0.35@0,1.0@8） | **53/53** | SELF-TEST PASSED |
| cfo 491.52 cf32 | **53/53** | SELF-TEST PASSED |

失败场景（`--scenario awgn --awgn-snr-db -20`，demo rc=1）：

```text
evidence = {level:"none", kind:"not_measured", measurement_valid:false, hardware_readback:null}
decode   = {status:"timing_failed", payload_hex:"", fcs_pass:false, diagnostic:{sfd:-1,packet:-1}}
status   = {ok:false, exit_code:1, attempted:1, exact_success:0, explicit_failure:1, m2a_status:"demod_failed"}
verifier: 123/123 checks pass（诚实记录失败，无 native 证据）
```

**失败清单的 self-test：48/53**。5 个"未检出"经逐一核对为**变异在失败清单上是 no-op / fixture 错误**，
不是 verifier 盲点（见 §4.2）。任务要求"报告任何未检出变异的场景"——此处如实报告。

### 1.3 SIMD 非确定性独立复现

```bash
c++ -std=c++17 -O2 -I gr-uwb/include testdata/twr/m2a/diagnostics/repro_det_size_k.cc \
    -o /tmp/opencode/repro_size -L/usr/local/lib -lvolk -lpthread -ldl -lm
for i in $(seq 1 6);  do /tmp/opencode/repro_size testdata/resampler_65_48/taps_quality_minorder.txt 256 scalar_macroblock; done | sort | uniq -c
#   6  ndiff=0            （scalar 恒确定）
for i in $(seq 1 10); do /tmp/opencode/repro_size ... 256 volk_macroblock;     done | sort | uniq -c
#   3  ndiff=1221 max_abs=1.794e-07 ; 7  ndiff=4 max_abs=3.072e-08
for i in $(seq 1 10); do /tmp/opencode/repro_size ... 256 avx2_fma_macroblock; done | sort | uniq -c
#   4  ndiff=1221 max_abs=1.794e-07 ; 6  ndiff=4 max_abs=3.072e-08
```

第三个探针（`resampler_nondeterminism_probe.cc`）**12/12 未复现**，与 diagnostics README §4 的
"allocation-pattern sensitive"记录一致。

### 1.4 A13 bench（默认 1000 热迭代）

```bash
env -u LD_LIBRARY_PATH gr-uwb/build/apps/twr_m2a_bench --iters 1000 --stage-iters 1000 \
    --csv /tmp/opencode/m2a_e/bench_full.csv
```

关键行（CSV 原值）：

| section | group | alloc_prepare | alloc_first | alloc_hot/iter | cold_us | p50_us | p95 | p99 | rss_start→end (KB) |
|---|---|---|---|---|---|---|---|---|---|
| prepared | final/cf32 | 60 | 92 | **92** | 20256 | 10870 | 11426 | 11742 | 50760→50768 |
| cold_wrapper | final/cf32 | — | — | **153** | — | 19264 | 20245 | 20630 | 51960→51960 |
| long_search | blank8192 | — | — | 101 | — | 8153 | 8392 | 8473 | 57656→57656 |
| failure | no-packet | — | — | 23 | — | 63429 | 64034 | 64751 | 58316→58316 |

所有 24 个 matrix 行：`alloc_prepare=60`、`alloc_first_frame=92`、`alloc_hot_per_iter=92`，
且 `alloc_new==alloc_hot_total`、`malloc/calloc/realloc=0`；RSS 在 1000 热迭代内增长 ≤ 8 KB
（多数行 0）。与协调者报告"prepared 92 vs cold 153、p50 10.88 vs 19.40 ms、RSS 平"一致。

---

## 2. A01–A14 / SIMD / A11 逐项裁决

### A01–A04、A06–A09 —— 保持干净基线（本轮定向复核）

`uwb_qa_uwb_twr_native_phy.cc` 9 例、`uwb_qa_uwb_twr_phy_impairment.cc` 13 例、
`uwb_qa_uwb_rational_resampler_m2a.cc`、`uwb_qa_twr_m2a_native_phy_verify_py` 全部通过。
A01 12 单元（2 rate × 3 帧 × 2 格式 × 2 变体）字节精确；A02 FCS 唯一归属与畸形拒绝；
A06 SC16 半格/饱和/NaN 拒绝；A07 域外枚举 fail-closed；A08 127 B 边界与坏 FCS/无包区分；
A09 随机前置空白不泄漏真值。**本项未见回归。**

### A05 数字坐标 —— **部分（非阻断）**

- **已实现**：`M2aStageTrace` 携带 `unit/rate_hz/interp/decim/origin/in_count/out_count/phase/
  trim/pad_front/pad_back/padding/search_guard_front/search_guard_back/search_roi/filter_delay/
  valid_from/valid_to`，由 demo 写入 `stages[]`（`gr-uwb/apps/twr_native_phy_demo.cc:1042-1060`）。
- **rebase 正确**：`m2a_demod_work` 把 core 在 `[gf zeros][work][gb zeros]` 缓冲里报出的坐标
  减去 `best_gf`，落在 guard 或越界一律置 −1（`gr-uwb/lib/uwb_twr_phy.cc:548-571`）。实测
  clean/delay/combo 的 `sfd_start_sample`、`packet_start_sample` 均落在 `[0, work_rx)`。
- **QA 覆盖**：多相余数 N=1..65（`qa_uwb_twr_native_phy.cc:516-570`）、`map_input_offset_to_output`
  p=0..199 与负值钳位（`:574-582`）、pad 拆分与 `filter_delay`、chunk 长度不变性、guard rebase、
  失败清诊断（`:1460-1584`）。
- **反例见 §3.1/§3.2**：`unit` 字段与实际计数单位不符；`m2a_demod_work` 返回的其它坐标未 rebase。
- **未覆盖**：非零 `origin`（本链无 crop，全部 0；verifier 钉死 0 并检出变异，故该项无正例）；
  逆映射（core 无逆映射 API，G0 A.2 的"逆映射"子项不适用）。

### A10 native 域损伤 —— **通过（模型+QA），种子证据归档不全（非阻断）**

损伤模型 `gr-uwb/lib/uwb_twr_phy_impairment.cc` 与 G0 A.3 逐条对照：

| G0 A.3 冻结点 | 代码 | 判定 |
|---|---|---|
| 顺序 CFO→AWGN→delay→multipath | `:220-317` 四段顺序执行 | 一致 |
| CFO 相位按 native rate/样点坐标推进 | `y[n]=x[n]·exp(j2π·cfo·n/rate)`（`:221-228`） | 一致 |
| 有效区 SNR、每维方差 = P/(2·10^(SNR/10)) | `:242-256`（区域=非零支撑 `[lo,hi)`） | 一致 |
| 确定性 seed、固定 Box–Muller、seed 0 普通 | `std::mt19937_64`+固定变换（`:160-185`） | 一致 |
| 整数延迟保长右移；分数因果 16-tap sinc、群延迟=f | `:269-299` | 一致 |
| 多径全卷积、保尾、tap[0] 首径 | `:302-317` | 一致 |
| 退化路径逐位 no-op；非法输入具名拒绝且不改缓冲 | `:203-216,319-325` | 一致 |

**QA 验证的是模型而非"解调好像成功"**：`qa_uwb_twr_phy_impairment.cc` 直接测量输出——CFO
闭环相位与实测频偏（`:276-325`）、AWGN 逐维方差对公式、64 seed 复现、**加前导零不改变噪声**
（`:331-453`）、impulse 延迟落点（`:459-481`）、分数延迟相位参考（`:483-512`）、多径抽头落点与
单位 tap no-op（`:553-597`）、顺序组合（`:603-626`）、非法输入具名拒绝+正对照（`:632-727`）。
13 例全过（本机 `*** No errors detected`）。

**缺口（非阻断）**：收尾单 §5.1/G0 A.3 要求"每基本单元固定 ≥10 个 seed，seed 列表冻结并记录"。
仓库内**没有**归档任何 A10 场景的逐 seed JSON/CSV，也没有冻结 seed 清单；CLI 一次只接受
`--awgn-seed N`，可用重复调用覆盖，但"≥10 seed 已跑"的证据只能来自尚未提交的收尾开发报告。
本报告不对"≥10 seed"背书。

### A13 分配与延迟 —— **部分（非阻断）**

- **prepared context 复用 core/scratch**：`M2aContext::Impl` 持有 TX/RX core、`HrpModScratch`、
  `DemodScratch`、模板、`work_tx/native/quant/work_rx/search`（`uwb_twr_phy.cc:927-962`）；
  `run()` 内**不构造任何 core**，逐帧走 `resample_with_core`/`modulate_with_scratch`/
  `demod_with_resources`（`:1079-1362`）。**独立探针确认**：连续 run 结果一致。
- **`prepare` 原子**：失败时 `prepared=false`，随后 `run` 返回 `InvalidConfig`。**独立探针
  `probe_ctx.cc` 实测**：
  ```text
  prepare(good)=1 prepared=1 ; run1 ok=1
  prepare(bad)=0 why=insert_sts is out of scope for M2-A (no STS) prepared=0
  run-after-bad ok=0 status=invalid_config detail=M2aContext is not prepared
  ```
  （注：QA 未显式覆盖"prepare 失败"分支；我以独立探针补验，非仓库 QA。）
- **容量计入 guard/tail**：`build()` 逐项算 `work_tx_max/native_max/work_rx_max`，并令
  `search_max = 2*work_rx_max`（p=0 放置的 `[gf=n][n][gb=0]`），全部 ≤ `kM2aMaxSamples`
  （`uwb_twr_phy.cc:987-1055`）；`demod_with_resources` 对每个放置再核 `buf_n ≤ kM2aMaxSamples`
  并 `search.reserve(max_buf)`（`:480-493`）。多径尾长在模型与 RX 阶段分别核（`impairment.cc:304`、
  `uwb_twr_phy.cc:776`）。
- **`reset` 语义**：清 core 历史/相位、`frame_scratch`、五个缓冲（`:1059-1077`）；独立探针
  `probe_ctx.cc` 实测 runA/runB `decoded_bytes` 相等、`native` 相等、`measurement_valid=0`。
- **实测数值**：见 §1.4。prepared 92 分配/帧 vs cold 153，p50 10.87 vs 19.26 ms，RSS 平。
- **缺口（非阻断，G0 A.4）**：bench 只分 **阶段**（prepare/first/hot）与**分配器类型**，**没有**
  按 `调制 / TX resample / 量化 / RX resample / 搜索·解调 / 结果·JSON` 分阶段计数（G0 A.4
  第 599–603 行明确要求）。因此"新 TX/重采样与 IQ workspace 热路径零分配"是**代码可读推断**
  （prepare 预留 + `assign/resize` 在容量内、scratch 预置），**不是**分项实测。92 次/帧全部经
  `operator new`，无法从 CSV 区分哪一部分来自 owned workspace。

### A12 manifest / verifier —— **通过**

- **独立性**：`tools/twr/verify_m2_a_native_phy.py` 只读 JSON、仅用标准库，不 import/call C++。
  成功清单 120–125 checks（随损伤字段多少而变）。
- **变异检出**：成功清单 **53/53 全检出**（clean/delay/combo/cfo/awgn/multipath/491.52 均如此）。
  变异覆盖 MAC/FCS/长度、stage 计数/延迟/out_count、payload、decode 状态/fcs_pass/PHR 长度、
  comparator、evidence level/kind、`measurement_valid`、`hardware_readback`、`yields_range`、
  provenance/executed hash、effective 字段、`matlab.executed`，以及 /2 的 impairment 参数/区间、
  origin、pad_front/back、unit、guard/ROI、kernel、context capacity/path、status 计数、诊断坐标
  （`verify_m2_a_native_phy.py:1584-1840`）。
- **不可被欺骗**（我独立构造，非仓库 self-test）：
  ```text
  evidence.level→native_roundtrip_verified/kind=measured 于失败清单 : CAUGHT
  status.ok=true/exit_code=0 于失败清单                        : CAUGHT
  measurement_valid=true                                       : CAUGHT
  yields_range=true                                            : CAUGHT
  evidence.level=toa_verified                                  : CAUGHT
  失败清单上"隐藏的字节精确成功"（status.ok=false）             : CAUGHT
  完全伪造的成功（连 status/evidence 一起改）                   : CAUGHT（诊断坐标仍为 -1）
  ```
- **观察**：失败清单的 self-test 有 5 个 no-op（§4.2）；verifier 不拒绝 schema 外多出的
  `distance_m` 字段（未在代码中出现，非当前风险，但独立 verifier 本可拒绝未知的距离/测距字段）。

### A14 回归 / 安装 —— **部分（非阻断）**

- 全量 CTest **62/63**；唯一失败 `uwb_qa_uwb_pdu_rational_resampler.cc`（`pdus_s=213.45 <= 400`、
  `headroom_200=1.07 <= 2`、`in_msps=32.44 <= 50`），与历史记录一致，门槛未改。
- 安装消费者 `verify_install_consumer.sh` rc=0，14 个公开头 + `libuwb_twr_phy.a` 安装；
  新加 4 个传递头（`uwb_demod_core.h`/`uwb_demod_result.h`/`uwb_phy_profile.h`/
  `uwb_cir_fir_simd.h`）已在安装列表（`gr-uwb/include/gnuradio/uwb/CMakeLists.txt:108-111`）。
- `ldd` 实测：demo 仅 `libvolk.so.3.3`；`uwb_qa_uwb_twr_native_phy.cc` 仅 `libvolk.so.3.3`；
  均**无** `libgnuradio-*`/`libuhd`。与收尾单 §1.1 一致。
- **缺口（非阻断）**：安装消费者 `tiny_phy_consumer.cc` 只调用 `m2a_expected_work_samples`/
  `m2a_rate_tx_lm`/`m2a_resampled_length`/`M2aConfig::is_valid`，**未实例化 `M2aContext`、
  未调用 `m2a_apply_impairment`**（`tools/twr/verify_install_consumer.sh:337-396`）。收尾单 §8
  要求"补新 context 实际安装消费"，此项未落实。另：收尾单 §8 要求 ASAN+UBSan 覆盖容量/寿命/
  失败恢复，仓库与本机均**无**该证据。

### SIMD 非确定性 —— **仅记录、未修（符合收尾单 §2）**

- 独立复现见 §1.3；`scalar` 确定，`volk`/`avx2` 非逐位可复现（≤1.8e-7）。
- `git diff 6193ba1..HEAD -- .../uwb_rational_resampler_core.h` **为空**：收尾两提交未碰该核。
- `git diff 4edf6c2..HEAD` 仅 3 个 hunk：`process()`/`flush()` 的 **Decim>Interp** 流式相位簿记
  与两个 TX 别名（`:298/:334/:990`），`grep` 宏块/VOLK/AVX/FIR 命中为 0——**SIMD/FIR 算法本身
  未被改动**。该改动发生在收尾之前的 `985bbc9`（M2-A P2），且被 `if constexpr (kDecim > kInterp)`
  隔离，既有 RX 方向不受影响。
- **G0 §1.2 核 hash 过期**（见 §3.5）：G0 仍写 `006901ef…`，实际为 `d0b9af70…`。

### A11 MATLAB —— **未满足（按用户指示未做）**

demo 的 `provenance.matlab.executed=false`，verifier 钉死 false；`testdata/twr/m2a/matlab/` 的
`.m` 脚本与 manifest 保留但未实跑。按收尾单 §1/§5 与 G0 §11，A11 **保持未满足**，不得用
Python/SciPy 冒充。本轮未安装、未运行 MATLAB。

---

## 3. 反例 / 不受支持的主张（含 file:line）

### 3.1 【单位标注反例】`unit` 字段与部分计数单位不符

G0 A.2（`M2-A_G0接口与数字坐标.md:552-553`）要求 `codec`/`fcs` 以 **bytes** 计、`modulate`/
`tx_resample`/`quantise`/`rx_resample`/`demod` 以 **samples** 计，且"每个 `M2aStageTrace`
必须标明单位"。实现中：

- `hrp_mod`：`trace.unit="samples"`（`gr-uwb/lib/uwb_twr_phy.cc:357`），但其 `in_count` 是
  **PSDU 字节数**（`:418` `trace.in_count = psdu.size();`）。
- `demod`：`trace.unit="samples"`（`:438`），但其 `out_count` 是**解码载荷字节数**
  （`:581` `trace.out_count = out.payload.bytes.size();`）。
- 头里 `unit` 声明允许 `"samples" | "bytes"`（`gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h:403`），
  但**没有任何 stage 用 `"bytes"`**；G0 A.2 点名的 `codec`/`fcs`/`quantise` stage **根本不输出**。
- verifier 反过来把"所有 stage `unit=="samples"`"写死（`tools/twr/verify_m2_a_native_phy.py:866`），
  并单独核对 `hrp_mod.in_count == psdu_bytes`（`:891`），即 verifier 与实现**共享同一套含糊约定**，
  不构成对单位语义的独立验证。

**影响**：`stages[]` 的 `unit` 字段不能唯一决定其计数单位；读者可能把 `hrp_mod.in_count=16`、
`demod.out_count=16` 当作样点。当前无消费者把这两者当测距坐标，**不构成测距/ToA 过度主张**，
但 G0 A.2 的"单位分开"并未真正落地。**非阻断**。

### 3.2 【未 rebase 的诊断坐标反例】`m2a_demod_work` 只 rebase 两个字段

`m2a_demod_work`（公开 API，`gr-uwb/include/gnuradio/uwb/uwb_twr_phy.h:605-606`）返回完整
`demod::DemodResult`；`demod_with_resources` 只把 `timing.preamble_start_sample` 与
`sfd.sfd_start_sample` 换回 work_rx 网格（`gr-uwb/lib/uwb_twr_phy.cc:564-571`）。其余坐标字段
**仍留在 guard 缓冲网格**，例如 `DemodResult` 的 `sfd.sfd_end_sample`、`sfd.expected_start_sample`
（`gr-uwb/include/gnuradio/uwb/uwb_demod_result.h:93,96`）、`timing.preamble_start_uncropped`
（`:51`）、`timing.peak_samples[]`、`cfo.fit_*`、`predicted/detected/window_start_sample`（`:176-178`）。

**最小反例**（`/tmp/opencode/probe_diag.cc`，用公开 stage 链构造 work_rx 后调用
`m2a_demod_work`）：

```text
work_rx n=117295 pad_front=117295
rebased:      timing.preamble_start_sample=56    sfd.sfd_start_sample=65080
NOT rebased:  sfd.sfd_end_sample=190502          sfd.expected_start_sample=182375
NOT rebased:  timing.preamble_start_uncropped=117351
NOT rebased:  timing.peak_samples[0]=118366 last=182374
```

`sfd.sfd_end_sample=190502 > work_rx=117295`，即一个"padding 索引冒充输入坐标"的实例。
G0 A.2（`:561`）要求"解调诊断坐标必须换算回未加 guard 的 work_rx 网格"。当前 demo/QA 只读
那两个已 rebase 的字段（`twr_native_phy_demo.cc:1397`、`qa_uwb_twr_native_phy.cc:1503-1528`），
故**当前无过度主张**；但这是公开 helper 上的**潜在契约违例**。**非阻断**，建议：要么把
`DemodResult` 的全部绝对坐标统一 rebase，要么在返回前把未 rebase 的字段显式置 −1/无效并在
头注释声明。

### 3.3 【A13 分项证据不足】bench 未按阶段拆分分配

G0 A.4（`:599-603`）要求分阶段统计"调制、TX resample、量化、RX resample、搜索/解调、
结果/JSON 序列化"。`twr_m2a_bench.cc` 的 CSV 列（`:472-478`）只有
`alloc_prepare/alloc_first_frame/alloc_hot_total/alloc_hot_per_iter` 与按分配器类型，**无阶段维度**。
因此"新 TX/重采样与 IQ workspace 热路径零分配"缺少分项实测支撑，只能作为设计推断。**非阻断**。

### 3.4 【A14 安装消费缺口】消费者未用 `M2aContext`

见 §2 A14。`tools/twr/verify_install_consumer.sh:337-396` 未消费新增的 prepared context /
impairment API。收尾单 §8 明确要求补此消费。**非阻断**。

### 3.5 【G0 文档不一致】重采样核 hash 已过期

G0 §1.2（`M2-A_G0接口与数字坐标.md:87`）仍写
`sha256(uwb_rational_resampler_core.h)=006901ef…`，但该文件在 `985bbc9` 已被改动，当前实测
`d0b9af70…`（`testdata/twr/m2a/diagnostics/README.md:37` 用的是正确的新值）。其余三个 G0 冻结
hash（`uwb_phy_profile.h`/`uwb_hrp_mod_core.h`/`uwb_demod_core.h`）与本机实测一致。**非阻断**，
但会让按 hash 复核者困惑，应在 G0 或收尾报告中更正并说明改动在 closeout 之前、且被
`if constexpr (kDecim > kInterp)` 隔离。

### 3.6 【closeout 交付完整性】收尾开发报告缺失、状态文档过期

- 收尾单 §8 要求新增 `docs/twr/M2-A_收尾开发报告_2026-10-06.md`——**仓库中不存在**。
- `开发状态.md:99` 仍写"M2-A native PHY 闭环（**进行中，未完成** 2026-10-05）"，`:110` 仍列
  A10/A13/A11 未完成，`:107` 仍写 verifier "72/72，24/24 变异"（实际 120–125 checks、53 变异）。
  `:25` 仍写收尾规划"尚未实施"。收尾单 §8 明确要求消除这些互相矛盾的最新状态。
- 这属于**交付/文档阻断**：代码评审本身可通过，但在补齐收尾开发报告并更新状态文档前，
  不能把"收尾完成"作为仓库最新事实。

---

## 4. 其它核查

### 4.1 成功判据未变、证据边界未被突破

- `M2aContext::run` 与冷包装 `m2a_native_roundtrip` 的成功判据一致：`Success && fcs_pass &&
  长度==期望 && bytes 逐字节相等`（`uwb_twr_phy.cc:1316-1335`、`:1353-1361`；头 `:59-63`）。
  任何回退候选/FcsFailed/长度不符 → 失败并计状态。
- demo 的 `is_measured`（`:854-858`）与 impairment 路径的判据（`:996-1012`）同构；
  `evidence.level` 只在实测成功时为 `native_roundtrip_verified`，否则 `none`/`not_measured`；
  `measurement_valid` 恒 false、`hardware_readback` 恒 null（`:1414-1418`）。
- demo **不调用** `allows`/`derive_row_evidence`/`build_default_capabilities`；48 行白名单未动。
- 全部 JSON 中无距离、无 RMARKER、无首径、无 `yields_range=true`、无 `measurement_valid=true`。

### 4.2 失败清单 self-test 的 5 个 no-op（如实报告）

失败清单上未检出的变异：`decoded payload`（空 `payload_hex` → 变异抛
`bytearray index out of range`，属 fixture 错误）、`decode status`（把 `timing_failed` 改成
另一个**合法失败态** `fcs_failed`，verifier 有意接受任意已知失败态）、`decode fcs_pass`（本就
false）、`compare bytes_exact`（本就 false）、`evidence kind inconsistent`（本就 `not_measured`）。
它们**都不构成"把失败当成功"**：§4.1 已独立证明该方向被拒。任务要求报告此类场景——此处报告。

### 4.3 N07 枚举域

`m2a_native_rate_is_known`/`m2a_iq_format_is_known`/`m2a_status_is_known` 均为**无 `default`**
的 `switch`，且在消费前先做域检查（`uwb_twr_phy.h:111-119,167-175,227-246`；`is_valid` 先域后依赖，
`:285-350`）。`m2a_native_rate_to_string` 等无 `default`，未知返回占位串。**通过。**

### 4.4 参数/坐标的其它观察

- `filter_delay` 两方向均记 `0.5*(T-1)=1353.0`（T=2707），pad 拆分 `1353/1353`；verifier 与 QA
  独立复算一致。
- demod 的 `pad_front` 可达整段输入长（p=0 放置：`gf=n, gb=0`），ROI 由 `seed±margin` 独立复算；
  这是放置逻辑的结果，不是拆分错误。
- `--config` 未实现：CLI 以 `unknown argument "--config"` 拒绝（`twr_native_phy_demo.cc:320-454`
  无该分支），符合 G0 §10 的降级方案（"未消费 TwrConfig"）；但拒绝文案是通用未知参数，
  不是具名"未支持 TwrConfig"。

---

## 5. 阻断条件

**代码级：无新的阻断。** A05/A13/A14 的缺口（§3.1–§3.4）均为非阻断：不产生测距/ToA/硬件
证据，不改变成功判据，且当前消费者未踩中。

**closeout 交付级：1 条阻断（§3.6）。**

1. **收尾开发报告 `docs/twr/M2-A_收尾开发报告_2026-10-06.md` 缺失，且 `开发状态.md` 未更新**
   至收尾后的最新状态。在补齐并消除 `开发状态.md` 的矛盾（A10/A13"未完成"、verifier"72/72"、
   收尾"尚未实施"）之前，不能把"M2-A 收尾完成"作为仓库最新事实。

另建议（不阻断）在收尾报告中显式记录：A10 的冻结 seed 清单/≥10 seed 证据、A13 分项分配 CSV、
安装消费者对 `M2aContext` 的实际消费、以及 G0 §1.2 核 hash 的更正。

---

## 6. 证据支持的最大结论

依据 `1002d63` 的代码与本机实跑，证据支持收尾单 §5 所设的上限，且**不得超过**：

> **"M2-A 本轮非 MATLAB 子项完成；独立 MATLAB 对照未做，SIMD 已知风险保留。"**

需同时声明以下限定，避免读者过度解读"完成"：

- A11 按完整口径**仍未满足**（MATLAB 未跑，`matlab.executed=false`）。
- SIMD 未初始化读取**只记录、未修**，根因未定位；不得由 ASan/UBSan 通过推断"已排除"。
- A05/A13/A14 为**部分**：单位标注含糊（§3.1）、未 rebase 的其它诊断坐标（§3.2）、
  分配未分阶段实测（§3.3）、安装消费者未消费 context（§3.4）。
- 本轮结论**不证**首径 ToA、RMARKER、硬件测距或商用互通；`measurement_valid` 恒 false，
  `allows(NativeRoundtripVerified, Ranging)` 仍为 false。
- 不进入 M2-B；不恢复"M2-A 整体无条件完成"。

---

## 7. 我未验证 / 未做的

- **未跑 MATLAB**（本机不可用，且按用户指示不安装、不运行）；A11 的 `.m` 脚本内容未执行。
- **未跑 ASAN/UBSan**（收尾单 §8 要求），也未独立做内存/寿命/失败恢复的 sanitizer 覆盖。
- **未独立复算"每随机场景 ≥10 seed"**：仓库无归档证据，只按 CLI 逐 seed 抽测了 7 个场景。
- **未验证 SIMD 未初始化读取的根因/读地址**：只复现可观测症状；`resampler_nondeterminism_probe`
  未复现属已知的 allocation-pattern 敏感性，不等于排除。
- **未穷举/模糊 verifier**：用定向变异与手工构造覆盖了关键方向，非形式化证明。
- **未核 `TwrConfig` 完整链路**：`--config` 被拒，M2-A 走冻结 profile 常量；A12 的配置链路
  完成项未验证（符合降级方案，不作为通过）。
- **未验证硬件/UHD/两 RX/controller/首径/RMARKER/pybind**：均不在 M2-A。
- **未独立复算**收尾单 §1.1 中"开发报告 61/62、历史吞吐"以外的既有数字；本报告只对上述
  本机实跑命令负责。
- 我未修改任何生产/QA/demo/verifier/测试数据；唯一写入是本文件。

---

审核人：subagent E（`m2a_final_review`，只读）。
本意见仅写入 `docs/twr/M2-A_最终实现评审报告.md`。
