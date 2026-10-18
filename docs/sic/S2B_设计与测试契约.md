# S2-B 设计与测试契约：感知前导干扰通信 payload

日期：2026-10-17。分支：`feature/sic-payload-collision`。基线提交：`93c80b3`。
执行依据：`docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md`（下称"指示"）
与 `docs/sic/S2_并行开发共同约定.md`（下称"共同约定"）。
历史基线：S0/S1 报告与 `docs/sic/S1_八符号模型API.md`（下称"S1 API"）。

本文是**先于代码**的设计与测试契约。凡本文中带"计划/待实现"字样的命令与
选项，在实现完成前不得当作现有能力。实测数字只出现在开发报告，不出现在本文承诺里。

> 状态（2026-10-17，实现完成后回填）：本文契约**已实现**并跑通 smoke/QA/verifier/
> 正式矩阵；实测结果、失败与未实跑项见
> `docs/sic/S2B_感知前导干扰通信Payload开发报告.md`。本文保留为设计意图基线。

---

## 1. 目标与角色反转

与 S1/S0 主场景相比，本任务把 desired/interferer 角色对调：

| 角色 | 信号 | 前导码 | 结构 | 接收端 |
|---|---|---|---|---|
| 期望信号 `s` | 通信完整包 | code 10 | 64 SYNC + SFD(4z2) + PHR + 随机 PSDU(+FCS) | comm 解调器（code10 profile） |
| 干扰 `i` | 感知前导-only | code 9 | 64 SYNC 有限模板，无 SFD/PHR/payload | SIC（code9 模板） |
| 噪声 `w` | AWGN | — | 三路配对共用 | — |

`y = s + i + w`；`clean = s + w`；`sic` = `y` 经 SIC 相减后的整窗 IQ。
SIC 的模板必须是 code9，解调 profile 必须是 code10；不允许只改标签。

主场景：干扰前导的**全部 64 SYNC**（含前 8 个训练符号）位于通信 payload 数据区内，
其前 8 个训练符号与**随机未知 payload** 完全重叠，没有干净训练区。
通信 SYNC/SFD/PHR 保持无干扰。跨 PHR/前导碰撞不在本轮（列未来）。

## 2. 包几何（真实 PHY，不由"总长减前导"猜测）

工作网格 `Fs = 998.4e6`，`sps = 1016`，`samples_per_chip = 2`。
由 `uwb_hrp_mod_core.h::packet_samples_998p4` 与 `encode_phr19`：

| 段 | 样点数 | 说明 |
|---|---|---|
| SYNC | `sync_reps * 1016` = 65024（64） | code10 |
| SFD | `n_sfd * 1016` = 8128（4z2 = 8 符号） | code10 |
| PHR | `21 * 512 * 2` = 21504 | 0.85 Mb/s 物理 |
| payload | `nsym * 64 * 2` = `nsym*128` | 6.81 Mb/s BPM-BPSK |
| FCS | 0（不占独立区间） | 是 PSDU 末 2 字节 |

`nsym(psdu_bytes) = 8*B + 48*ceil(8*B/330)`。
默认 `psdu_bytes = 96`（含 2 B FCS；即 94 B 随机数据 + 2 B FCS）：
`nsym = 768 + 48*3 = 912`，payload = 116736 样点，整包 = **211392** 样点（legacy 脉冲，
尾部额外 0）。PSDU 上限 127 B（`radar_meta::kMaxPsduBytes`），96 无需放宽白名单。

通信字段起点（相对 `comm_start = pre_guard`）：
`SYNC [0,65024)`、`SFD [65024,73152)`、`PHR [73152,94656)`、
`payload [94656,211392)`；FCS 在 payload 末尾。

干扰有限模板长度 `D = natural_sync_length(64, taps_len)`（legacy 48 tap → 65064）；
含路径拖尾 `D_eff = D + ceil(max_path_delay)`。
payload 可用区 `[a,b)`，公开 guard `G`：

```
early  = a + G
center = a + floor((b - a - D)/2)
late   = b - G - D
要求 b - a >= D_eff + 2G，否则拒绝该配置（不得裁掉干扰继续宣称 64 符号重叠）
```

默认 `G = 2048`；`b-a = 116736 >= 65064+4096 = 69160`。三个位置都容纳完整 D + 拖尾。

## 3. 信号生成与真值合同（复用 `sic_preamble_sim.py` 公共件）

薄 wrapper：`gr-uwb/apps/sic_payload_sim.py` 复用
`sic_preamble_sim.py` 的 `work_fullwin_geometry / apply_path / PathSpec /
recover_preamble_code / shape_finite_sync / pulse_taps / sha256_array`，
不复制其实现。反转角色：

* `s = comm full packet`：`uwb.hrp_packet_source(psdu_wire, 64, "4z2", 10, 0.8,
  0.01, False, False, append_fcs=False, pulse_shape, ...)`。
  FCS 由生成器**显式追加一次**到 94 B 随机数据（`append_fcs` 开关保持 False，
  避免"追加两次"歧义），记录 FCS 契约与 CRC。
* `i = code9 finite SYNC 模板`：由 `recover_preamble_code` 取回 code9 chip，
  `shape_finite_sync` 直出 64 SYNC 有限列，峰值归一到 0.8；只用 SYNC 滤波自然拖尾。
* 路径：`apply_path`（整数/分数延迟，PCHIP）。CFO 用 `exp(j2π f_cfo n/Fs)`。
* 噪声：同 seed 的相同 realization，`y/clean/sic` 共用；`y=s64+i64+w64`、
  `clean=s64+w64` 用同一表达式，保证 raw 与 clean 在 `i=0` 时逐位一致。
* 原始 CF32，**不逐路峰值/L2 归一化**（ISR 缩放只作用于 `i`）。

**ISR/SNR 定义（事先固定）**

* 参考区 = 通信 payload 区 `[94656, 211392)`（公开 PHY 几何，与注入真值无关）。
* `SNR = 10log10(E_s / E_w)`，`E_s = Σ|s|²`、`E_w = Σ|w|²` 都在参考区内。记为**样点 SNR**。
* `ISR = 10log10(E_i / E_s)`，在真正重叠区 `overlap = i 支撑 ∩ 参考区` 内定义；
  无干扰时 N/A。缩放使 `E_i(overlap) = E_s(overlap) * 10^(ISR/10)`。
* 三路共用同一 `w`；ISR 只改 `i` 的缩放，不改 `s`/`w`。

**随机性与种子**（共同约定 §4）

* calibration `21000000+`，acceptance `22000000+`。
* 每 case 的确定性子流：payload / noise / path / offset 分别由
  `seed_root + {0,1,2,3}*K` 派生并逐窗记录实际值。
* 同一窗的 clean/raw/sic 配对重用 `w`/`s`/payload（有意配对，非独立样本）。
* 不同验收窗不得重用 payload/noise。

## 4. 真值防火墙（在线只拿公开信息）

在线 SIC + 解调只接收：混合 IQ `y`、公开 comm PHY profile、公开 sensing code/
名义长度、公开粗 ROI、公开 guard 与前缀可见性。**不接收**注入起点/位置真值、
CFO、路径、干扰 IQ、payload 真值。解调器 `predicted_start_sample` 是公开的
**调度起点**（期望信号是接收机调度的包），与干扰注入无关，明确记录。

公开 ROI：
* 检测 ROI（core `roi_lo/roi_hi`）= `[nominal - J - 64, nominal + J + 64]`，
  `nominal` = 方案预设位置（early/center/late），`J` = 公开起点抖动预算（校准冻结）。
* 相减 ROI（PDU `sensing_roi_lo/hi`）= `[nominal - J - 64, nominal + J + D + post_tail + 128]`。
* 可见前缀 `observed_end = min(n, (nominal + J) + fit_reps*sps + 64)`（公共量，
  不按注入起点反推）。逐窗报告实际训练区 `[origin, origin+fit_reps*sps)`、
  可见 end、是否 partial。

前缀可见性契约沿用 S1 API §2：estimation 只读 `y[0, observed_end)`；apply 覆盖全窗。
在线不借用 clean 通道；不用已知 payload 造期望波形辅助拟合。
可加的干净训练区/oracle 诊断只作单列，不进主曲线。

## 5. SIC 配置（冻结共享核心，沿用 S1 默认）

`estimate8_frozen`：`fit_reps=8`、`online_validation_reps=0`、`k_paths=4`、
`support_mode=configured_nominal`、`sub_roi` 由 PDU meta 给、`post_tail=100`。
不修改 `uwb_preamble_sic_core.{h,cc}`/adapter/`sic_reference.py`。
若必须修共享核心：保存最小复现、写 `docs/sic/S2_共享核心问题_B.md`、标 BLOCKED，
继续不受影响实验。阈值（detect/validate/fit_residual_max）只在 calibration 冻结，
acceptance 不再调。

**风险（如实记录）**：8 符号 CFO 精度限制远端深度（S1 结论）。S2-B 主点 CFO=0，
CFO=±50 kHz 只作困难条件。若残余在 payload 内不均匀（训练区在 payload 中部），
报告真实位置窗口，不简化。

## 6. 逐包指标与统计

每包唯一 ID，记录：三路状态 stage、decoded PSDU（仅确认时暴露）、PSDU 匹配、
FCS 结果、SIC `detected/committed/bypass/model fingerprint/training 区/support/sub/
各段抑制/时序`、真实重叠比例。

两套错误口径：

* **PER_FCS**：未检测/未输出/超时/头失败/长度错误/FCS 失败，全部计错；分母 = 计划发送包数。
* **PER_PSDU**：未恢复期望 PSDU 或内容不一致计错（含 FCS 误通过但内容错）；分母同上。

PSDU 真值评分严格相等，不从原始 payload 填补失败输出。FCS 契约：往返 PSDU 含末 2 B
FCS，解调返回含 FCS 的全 PSDU，逐包记录 `decode_contains_fcs`、`fcs_pass`。

BER：仅对成功且可对齐的输出计算；失败包无 bits ⇒ `BER_all=N/A`，同时给
`conditional_BER` 与覆盖率/擦除率。优先 PER；BER 缺失不阻塞闭环。

每 cell 给 Wilson 95% CI；0/n 不写 0 风险。只有当曲线实际跨越且单调/区间足够时才
报告固定 PER 所需 SNR 差，否则 `N/A`，不外推 dB 增益。

## 7. 分阶段矩阵（先校准，再冻结验收）

| 阶段 | 内容 | 计划样本 |
|---|---|---|
| B0 | 设计+无干扰 closure，随机 PSDU，先 8 窗后 100 窗高 SNR baseline | 8 / 100 |
| B1 | SNR30/ISR10/center/jitter/CFO0/single，三路逐包配对；工程目标 clean=100/100、sic=100/100 | 100 |
| B2 | 校准 SNR 粗扫定位瀑布；冻结 5 SNR；acceptance 5 SNR × ISR{0,10,20} × center/CFO0/single | 正式 1000/cell；若成本高先 100/cell 探索并标"探索/不足正式样本"，正式矩阵保留 OPEN |
| B3 | 过渡 SNR+SNR30 × ISR10 × {early,late}（4 cells）；SNR30 × ISR{0,10} × CFO{±50k} × {single,four}（8 cells） | 正式 500/cell |
| B4 | no_sensing/wrong_code11/no_overlap 各 1000；误 commit ≤1%；旁路逐位不变 | 1000 each |

无干扰 clean 可按 SNR 配对复用但记录共享 sample id。样本不足时如实标探索。

## 8. 针对性 QA（实现前先写）

代码角色与模板：
1. 干扰只有有限 SYNC，不含 sensing SFD/PHR/payload；模板 == 独立 `shape_finite_sync`
   参考且长度 == natural。
2. 角色：demod=code10 模板 + `code_index=10`；SIC=code9。code10 clean 包可解出
   PSDU；把 demod 配成 code9 时必须失败（证明不是只改标签）。
3. FCS 只追加一次：wire PSDU = 94 B 数据 + 计算 CRC；解调返回含 FCS 且 `fcs_pass`；
   故意不追加 FCS 时 `status=fcs_failed`（记录语义）。

几何与防火墙：

4. 训练 8 符号确实在未知 payload 内；无真值泄漏（配置/ meta 无真值键）。
5. early/center/late 均含完整模板+拖尾且不越 header；短 payload 配置被拒。
6. mask 外逐位不变；样点数/包头不变。
7. 训练后变更：改动 `observed_end` 之后的数据 ⇒ 冻结模型逐字段不变；改动训练区内
   payload ⇒ 允许模型变化。

边界：

8. no_sensing/wrong_code11/no_overlap ⇒ 旁路（`out==y`），误 commit 计数。
9. 失败计数穷举：检测/PHR/FCS/PSDU 长度/timeout/队列异常，唯一终态，总数守恒
   （offered == admitted + missed_admission，completed+failed+bypassed 守恒）。

verifier 负样本：

10. 人工构造"FCS 正确但 PSDU 错"、错分母、漏失败、错 ROI、改数组、错库、WALS 缺失的
    artifacts，verifier 必须抓到。verifier 从归档数组独立复算聚合。

## 9. 交付物

* `gr-uwb/apps/sic_payload_sim.py`（生成器，真值侧）。
* `gr-uwb/apps/uwb_sim_sic_payload.py`（GR flowgraph/实验驱动，`--help/--smoke/--self-test/
  --calibration/--acceptance`）。
* `gr-uwb/apps/test_sic_payload.py`（针对性 QA unittest）。
* `tools/sic/payload_metrics.py`（PER/Wilson/BER 指标）。
* `tools/sic/verify_s2b.py`（独立 verifier + 负样本 self-test）。
* `testdata/sic_payload/`（README、config_schema、cases、MATLAB 对照脚本）。
* `docs/sic/S2B_感知前导干扰通信Payload开发报告.md`（真实结果/失败/未实跑项）。

## 10. 不变量与不做的事

* 只改本 worktree；独立 build `gr-uwb/build-sic-payload`、产物在其 `results/`。
* 不修改共享 SIC 核心/adapter/`sic_reference.py`/旧 QA 门槛；不放宽旧测试。
* 不用真值动态收紧 ROI/起点/gain/mask/门控；oracle 只单列。
* 不复用旧 `feature/sic-validation` 的"干扰 FCS 成功才相减"门槛。
* 不自动上板/native/UHD/TWR；不自动合并。
* 性能数字绑 commit/profile/实际库/窗口长度；正式 benchmark 串行并与另一 session 协调
  （`flock -n /tmp/uwb-sic-benchmark.lock`）。MATLAB 未跑则明确写未验证。
