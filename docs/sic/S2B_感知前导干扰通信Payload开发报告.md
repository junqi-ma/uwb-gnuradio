# S2-B 开发报告：感知前导干扰通信 payload

日期：2026-10-17。分支：`feature/sic-payload-collision`。基线提交：`93c80b3`。
回交对象：`docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md`（下称"指示"）。
契约：`docs/sic/S2B_设计与测试契约.md`（本轮设计与测试契约，先于代码）。
共同约定：`docs/sic/S2_并行开发共同约定.md`。
历史基线：`docs/sic/S1_八符号估计与全前导消除开发报告.md`、
`docs/sic/S1_八符号模型API.md`（S0/S1 口径，本文不重复其结论）。

本文只报告**实测**数字，全部绑定到本 worktree（`/home/junqima/workspace/uwb-gnuradio/.worktrees/sic-payload-collision`）
的 commit 与独立构建 `gr-uwb/build-sic-payload`。共享 SIC 数学核心与 adapter
**未修改**（见 §2）。S0/S1 的数字与本文不同源，不得互相背书。

> **一句话结论**：S2-B 的角色反转（desired=code10 完整包、干扰=code9 感知前导-only）、
> 真实 payload 内碰撞、单次 FCS、真值防火墙、逐包配对与独立复算**均已实现并实测通过**：
> 无干扰 clean 逐包 100/100；名义点（SNR30/ISR10/center/CFO0/single）SIC 逐包 **100/100**
> 对 raw 实测 **48/100**（目标达成）；SIC 实际抑制度中位数 ~40+ dB；mask 外逐位不变；
> 负样本（no_sensing / wrong_code11 / no_overlap）**全部旁路、误提交 0**。
> **但**：B2 的 raw PER 在 ISR10 由**干扰主导**、几乎不随 SNR 改善（~50% 地板），
> clean/SIC 的 SNR 瀑布位于 **SNR≈-4 dB**（-5 全错、-3.5 仅 3.8%）且极陡；CFO=±50 kHz 与四径困难条件的
> SIC 深度受限（如实列出）；**MATLAB 未实跑**；native/UHD/硬件/timestamp 不在本轮。
> acceptance 共 **25 cells / 21100 个配对窗**，全部经独立 verifier 复核通过。

---

## 0. 状态总表

| 编号 | 事项 | 状态 | 代码位置 | 一句话证据 |
|---|---|---|---|---|
| S2B-01 | 角色反转（desired code10 / 干扰 code9） | **CLOSED** | `gr-uwb/apps/sic_payload_sim.py`；`test_sic_payload.py::build_code_role_mismatch_fails` | code10 clean 逐包解出 PSDU；把 demod 配成 code9 时 `sfd_failed`（不是只改标签） |
| S2B-02 | 完整 comm 包几何与单次 FCS | **CLOSED** | `sic_payload_sim.py::comm_field_offsets/append_fcs`；`build_clean_code10_closure` | `psdu_wire=96B`（94 数据 + 2 FCS CRC-16=0x2112），解调返回含 FCS 且 `fcs_pass` |
| S2B-03 | 干扰落在 payload 数据区内、前 8 符号未知 payload | **CLOSED** | `Placement.compute`；QA `positions_inside_payload_and_training_unknown` | early/center/late 均含完整 65064 样点模板；训练区 `[origin,origin+8·1016)` ⊂ payload |
| S2B-04 | 真值防火墙（在线无真值） | **CLOSED** | `_sic_meta`；QA/verifier | online meta 无注入真值键；`uses_truth=false`、`apply_refit_count=0`；`observed_end` 之后改动模型 fingerprint 不变 |
| S2B-05 | 三路配对与逐包分母 | **CLOSED** | `uwb_sim_sic_payload.py`；`tools/sic/payload_metrics.py` | 同窗 clean/raw/sic 共用 payload/噪声/坐标；`PER_FCS` 与 `PER_PSDU` 双口径 |
| S2B-06 | 独立 verifier + 负样本 | **CLOSED** | `tools/sic/verify_s2b.py` | `--self-test`：好集通过，6 个故意损坏变体全部被抓 |
| S2B-07 | B0/B1 名义效果 | **CLOSED** | `s2b_b0_smoke`、`s2b_matrix` | B0 clean 8/8；B1 clean=100/100、SIC=100/100、raw=48/100 |
| S2B-08 | B2 瀑布与横轴收益 | **MEASURED**（见 §7） | matrix B2（15 cells × 1000） | clean≈SIC 在 SNR≈-4 dB 陡降；raw(ISR10) 干扰地板 ~0.5、ISR20 恒 1.0；固定-PER SNR 差 N/A |
| S2B-09 | B3 边界/困难 | **MEASURED**（见 §8） | matrix B3（6 cells × 500） | early/late 41 dB、rescue ~290、sic 0；CFO±50k 四径抑制度降到 3.64 dB 但 damage=0 |
| S2B-10 | B4 负样本误提交 | **CLOSED**（见 §9） | matrix B4（3 cells × 1000） | 三负样本 detect=0、commit=0、mask 改变 0；误提交 0 ≤ 1% |
| S2B-11 | 性能/算力 | **实测**（见 §12） | `stage_times.csv` | SIC 核心 ~3.1 ms/窗；本任务不声明吞吐验收 |

---

## 1. 版本与 provenance

来源：`gr-uwb/build-sic-payload/results/s2b_*/*/run_manifest.json` 的 `provenance`。

| 项 | 值 |
|---|---|
| commit / 分支 | `93c80b3` / `feature/sic-payload-collision` |
| dirty | 每 run 记 `git.head` 与 `git.status_sha256`（本任务在基线之上新增文件） |
| 独立构建 | `gr-uwb/build-sic-payload`（不与父仓库 `build-sic` / `/usr/local` 混用） |
| 实际加载库 | 由 `run_manifest.json:provenance.loaded_library(+sha256)` 逐 run 记录，`ldd` 指向本 worktree `build-sic-payload/lib` |
| 共享核心 / adapter | **未修改**：`uwb_preamble_sic_core.{h,cc}`、`uwb_preamble_sic.{h,cc}`、`sis_reference.py`、S1 core/adapter QA 均保持基线；`ctest -R preamble_sic` 在本 worktree 通过 |
| MATLAB | **未实跑**（`matlab.status=未实跑`；`testdata/sic_payload/matlab/*.m` 未运行） |

`libgnuradio-uwb.so` 的实际路径与 sha256 由每个 manifest 记录；cf. §10 的 verifier
`manifest_library_in_worktree` 检查。

---

## 2. 本轮新增文件与边界（薄 wrapper）

| 文件 | 角色 |
|---|---|
| `docs/sic/S2B_设计与测试契约.md` | 先于代码的设计 + QA 契约 |
| `gr-uwb/apps/sic_payload_sim.py` | 真值侧信号生成（复用 `sic_preamble_sim` 公共件，角色反转） |
| `gr-uwb/apps/uwb_sim_sic_payload.py` | GR flowgraph/实验驱动/CLI（单 run + `--matrix` + `--self-test`） |
| `gr-uwb/apps/test_sic_payload.py` | 针对性 QA（纯 + 依赖构建） |
| `tools/sic/payload_metrics.py` | PER/Wilson/抑制度指标 |
| `tools/sic/verify_s2b.py` | 独立 verifier + 负样本 self-test |
| `tools/sic/s2b_aggregate.py` | 矩阵聚合/复核/画图（复算而非信任 summary） |
| `testdata/sic_payload/` | README、config_schema、cases、fixtures（CF32）、MATLAB 对照脚本 |
| `gr-uwb/apps/CMakeLists.txt` | 仅**新增**一条 CTest `uwb_qa_sic_payload_py`，未改旧测试 |

未复制整套 `sic_preamble_sim.py` / `uwb_sim_preamble_sic.py`：生成器复用
`work_fullwin_geometry/apply_path/PathSpec/recover_preamble_code/shape_finite_sync/pulse_taps`；
驱动复用 `ResultSink/provenance/call_sic_make_from_templates/wilson_ci`。SIC 数学核心与
GNU Radio block **未新增**（沿用 S1 的 `uwb.preamble_sic` 与 `uwb.realtime_demodulator`）。

---

## 3. 包几何与 FCS 契约（真实 PHY）

工作网格 `Fs=998.4e6`，`sps=1016`，2 样点/chip。**PSDU=96 B（不含 FCS 也是 96，
含 FCS）→ 见表**。96 B wire = 94 随机数据 + 2 B FCS。字段区间：

| 段 | 样点 |
|---|---:|
| SYNC (64) | 65024 |
| SFD (4z2) | 8128 |
| PHR (21) | 21504 |
| payload (`nsym·128`, `nsym=912`) | 116736 |
| **整包** | **211392** |

payload 绝对区 `[96653,213389)`（含 `pre_guard=1997`）。整窗 `n=245440`。
FCS：`crc16_802154` 反射多项式 0x8408、init 0，低字节在前，**只追加一次**；
modulator `append_fcs=False`；解调按 PHR 长度解出**整段含 FCS** 并校验（不剥离）。
`test_sic_payload.py::crc16_matches_canonical`（canonical PSDU 的末 2 B = 0x2112）。

---

## 4. 信号模型、角色与真值防火墙

* `y = s + i + w`；`clean = s + w`；`sic` = `y` 经 S1 SIC adapter（code9 模板）。
* `s` = code10 完整包（每包**随机 PSDU**，确定性子流，含 00/FF）；
  `i` = code9 感知前导-only 有限 SYNC 列（65064 样点），置于 payload 内；
  `w` = AWGN，三路共用同 realization。
* ISR/SNR 事先定义（契约 §3）：SNR 在 payload 区 `[96653,213389)`；ISR 在真实重叠区。
* 公开信息：混合 IQ、公开 comm profile、公开 sensing code/名义长度、公开粗 ROI、
  公开 guard 与可见前缀。**不传**注入起点/CFO/路径/干扰 IQ/payload 真值。
  解调 `predicted_start_sample` 用公开**调度起点** `pre_guard`，与注入无关。

在线 meta 只含：`window_id/packet_id/code_index/sync_repetitions/sfd_mode/
sample_count/sample_rate/observed_end_sample/sensing_roi_lo/hi/predicted_start_sample`。
QA `no_truth_in_online_meta` 断言无 `start_sample/cfo_hz/paths/gain/psdu_wire_hex/isr_db` 等键。

---

## 5. `coarse_stride`：一个必须记录的实测发现

S1 用 `coarse_stride=14`（cs-sync 主瓣被 coarse 命中）。S2-B 中 code9 sync 的
归一化相关主瓣在全速率下**极窄**（±1 样点内衰减：`0.95 / 0.59 / 0.16 / ~0.01`），
ROI 起点与注入起点不对齐时 coarse(14) 会**整体错过**主瓣，导致
`below_detect_threshold`。校准集实测后把 `coarse_stride` 固定为 **2**（公开参数，
非阈值调参），检测在 ISR=0 仍有 0.69 的峰值裕度（无干扰 ~0.046）。
此发现独立于阈值，仅改变检测搜索粒度，已写入契约与报告。

---

## 6. B0 closure 与 B1 名义效果

**B0（无干扰 closure，8 窗）**：`s2b_b0_smoke`，clean `success` 8/8、`fcs_pass` 8/8、
PSDU 逐字节相等。

**B1（acceptance，SNR30/ISR10/center/CFO0/single，100 包）**：

| 路径 | PER_PSDU | Wilson95 | FCS 通过 | 状态分布 |
|---|---:|---|---:|---|
| clean | 0/100 = 0.00 | [0.963, 1.0] | 100/100 | success 100 |
| raw | 52/100 = 0.52 | [0.385, 0.577] | 48/100 | payload_failed 48, fcs_failed 4, success 48 |
| sic | **0/100 = 0.00** | [0.963, 1.0] | 100/100 | success 100 |

即工程目标 clean=100/100、sic=100/100 达成；raw 实测失败（非硬编码）。实际抑制度
（重叠区）中位数 ~40+ dB（逐窗见 `windows.jsonl:suppression_db_overlap`）。

---

## 7. B2 瀑布（acceptance，5 SNR × ISR{0,10,20}，每 cell 1000 包）

**校准过程（calibration seed，未接触 acceptance）**：clean 扫描（40→100 窗）
得 SNR=-4 为 100/100 失败、-3.5 为 1/100、≥-3 为 0/100，**clean 瀑布在 SNR≈-3.5 dB
且极陡**；raw（ISR10）在 SNR=2..8 恒 ~50% 失败（`payload_failed`/`fcs_failed`），
**不随 SNR 改善**——干扰主导。据此冻结 5 个 acceptance SNR 点 `{-5, -3.5, 0, 4, 8}`，
raw/SIC 使用同一横轴。PER_PSDU（错/1000，Wilson95 见 `matrix_cells.csv`）：

| SNR(dB) | ISR0 clean | ISR0 raw | ISR0 sic | ISR10 clean | ISR10 raw | ISR10 sic | ISR20 clean | ISR20 raw | ISR20 sic |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| -5.0 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 |
| -3.5 | 38 | 38 | 38 | 38 | 670 | 38 | 38 | 1000 | 38 |
| 0.0 | 0 | 0 | 0 | 0 | 608 | 0 | 0 | 1000 | 0 |
| 4.0 | 0 | 0 | 0 | 0 | 572 | 0 | 0 | 1000 | 0 |
| 8.0 | 0 | 0 | 0 | 0 | 551 | 0 | 0 | 1000 | 0 |

配对（同 cell 内，ISR>0）：`raw_fail_sic_ok` 632/962（-3.5）、608/1000（0）、572/1000（4）、
551/1000（8）；`raw_ok_sic_fail`（损伤）**全部 0**。SIC 中位抑制度随 ISR 上升
（-5/ISR10 ≈ 24 dB → 8/ISR20 ≈ 45 dB），且随 SNR 上升（同 ISR 下噪声受限残余更小）。

**结论与横轴收益**：
* `clean≈sic` 曲线在 **SNR≈-4 dB** 处陡降（-5 全错，-3.5 为 3.8%，≥0 为 0），
  SIC 把干扰受限的问题**还原为噪声受限的 clean 曲线**。
* `raw`（ISR10）**不随 SNR 改善**：从 -3.5 到 8 dB 仅由 0.670 缓降到 0.551，
  趋近 ~0.5 的**干扰地板**（干扰是结构性信号，加噪不抵消）。ISR20 恒为 1.0。
* **固定 PER 所需的 SNR 差：N/A**。raw 在采样区间内从未降到目标 PER（甚至未到 0.5），
  曲线不跨越，按契约不外推 dB 增益。可报告的事实是：在 `sic=0` 的全部 SNR≥0 点，
  `raw_PER≥0.55`；ISR20 时 `raw_PER=1.0` 而 `sic_PER≤0.038`。
* ISR0 时 raw≈clean（干扰与信号等强、且噪声在高 SNR 下不占主导时干扰仍可解码），
  说明 raw 的失败来自 ISR，而非 SNR。

图：`s2b_matrix/figure_per_vs_snr.png`（三路径 × 三 ISR，Wilson95 带）、
`figure_paired.png`（救回/损伤）、`matrix_cells.csv`、`matrix_recomputed.json`。

---

## 8. B3 边界与困难条件（acceptance，每 cell 500 包）

| cell | clean | raw | sic | rescue | damage | 中位抑制度 |
|---|---:|---:|---:|---:|---:|---:|
| early / ISR10 / SNR30 | 0/500 | 293/500 | 0/500 | 293 | 0 | 41.33 dB |
| late / ISR10 / SNR30 | 0/500 | 281/500 | 0/500 | 281 | 0 | 40.93 dB |
| CFO-50k single | 0/500 | 29/500 | 0/500 | 29 | 0 | 25.11 dB |
| CFO+50k single | 0/500 | 29/500 | 0/500 | 29 | 0 | 24.81 dB |
| CFO-50k four | 0/500 | 2/500 | 0/500 | 2 | 0 | **3.64 dB** |
| CFO+50k four | 0/500 | 2/500 | 0/500 | 2 | 0 | **3.64 dB** |

**结论**：early/late 与 center 一致（SIC 全救回、抑制度 ~41 dB、损伤 0）。
CFO=±50 kHz 时 raw 的失败率**反而降低**（0.058）：50 kHz 相位在一个 payload BPM
判决窗内旋转，削弱了干扰与 payload 判决的相干性，这是**实测**现象，非算法假设。
四径叠加 CFO 时，8 符号 CFO/路径估计误差使实际抑制度降到 **3.64 dB**（S1 的
"8 符号 CFO 深度限制"在 payload 场景复现），但因 raw 本身已近乎不出错，
SIC 仍 0/500 失败、损伤 0——即深度不足**未转化为 PER 损失**，如实记录不外推。

---

## 9. B4 负样本（no_sensing / wrong_code11 / no_overlap，各 1000 包）

| cell | detected | committed | 误提交 | mask 外改变 | clean | raw | sic | rescue | damage |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| no_sensing | 0/1000 | **0** | **0** | 0 | 0/1000 | 0/1000 | 0/1000 | 0 | 0 |
| wrong_code11 | 0/1000 | **0** | **0** | 0 | 0/1000 | 551/1000 | 551/1000 | 0 | 0 |
| no_overlap | 0/1000 | **0** | **0** | 0 | 0/1000 | 0/1000 | 0/1000 | 0 | 0 |

* no_sensing（无干扰）：SIC 全旁路，`out==y` 逐位不变；clean/raw/sic 同结果。
* wrong_code11（注入 code11 干扰、SIC 只有 code9 模板）：SIC **不提交、全旁路**，
  `b4_wrong_code11` 的 raw=sic=551/1000 失败——即 SIC 对没有模板的干扰**不帮忙也
  不伤害**（rescue=0、damage=0），误提交 0 ≤ 1% 门限。
* no_overlap（真实干扰被放在包尾、不在公开 payload ROI 内）：SIC 不检测、全旁路、误提交 0。

即：`误 commit ≤ 1%`、`旁路逐位不变` 两条负样本门限均达成。

---

## 10. 独立 verifier 与错误产物拒绝

`tools/sic/verify_s2b.py`：

* 只读 `run_manifest.json`/`windows.jsonl`/`summary.json`/`index.json`/`recon/*.npz`；
* **独立复算**每路 PER_PSDU/PER_FCS、Wilson CI、paired 计数、中位抑制度，
  与 `summary` 交叉核对；分母 = 计划窗数；
* 从 `recon/*.npz` **独立复算**真实抑制度（`out-clean` vs `i_true`），并检查
  `out == raw` 在 mask 外逐位一致（用归档的 `raw`）；
* provenance：库必须在 worktree `build-sic-payload`（拒绝 `/usr/local`）；MATLAB 未跑声明；
* seed 唯一性与 calibration/acceptance 分离；`uses_truth=false`、`apply_refit_count=0`。

`--self-test`：构造一致好集（通过），再制造 6 个故意损坏变体，**全部被抓**：
`wrong_denominator`、`missing_failure_row`、`bad_roi_order`、`double_fcs_length`、
`zeroed_out_array`、`wrong_library`。

对真实 run 的复核：`s2b_b0_smoke` 33 pass/0 fail；B1 33 pass/0 fail；负样本 run 33 pass/0 fail。

---

## 11. 针对性 QA

`gr-uwb/apps/test_sic_payload.py`（也经 CTest `uwb_qa_sic_payload_py`）：

```
PASS crc16_matches_canonical / fcs_appended_once / random_psdu_has_00_ff_and_varies
PASS payload_geometry_and_short_rejection
PASS interferer_is_preamble_only            (len=65064 conv=0 tail_rel=7.5e-7 sfd_leak=0.47)
PASS positions_inside_payload_and_training_unknown
PASS no_truth_in_online_meta / observation_budget_covers_jitter
PASS clean_bit_identical_when_no_interferer
PASS wrong_code_low_correlation              (code11 metric 0.114)
PASS build_clean_code10_closure / build_code_role_mismatch_fails
PASS build_sic_freeze_and_mask              (commit=1 mask_outside_changed=0 refit=0 fp_equal=1)
== 13 pass, 0 fail, 0 skip ==
```

`ctest -R preamble_sic` 的 S1 core/adapter QA 在本 worktree 亦通过（未放宽旧测试）。

---

## 12. 性能/算力（本环境实测，窄口径）

同一 worktree、同一构建、`-j` 无关；SIC 核心为真实负载（23000+ 窗）。单位 µs。

| 阶段 | mean | p95 | 说明 |
|---|---:|---:|---|
| 生成 `gen_us` | ~22000 | ~25000 | 离线 Python 生成 `y=s+i+w`（含每包随机 PSDU 调制、路径、噪声），**不在在线路径** |
| SIC 核心 `core_total_us` | 3106–3302 | 3478–5296 | `estimate8_frozen`（8 符号估计 + 全支持相减） |
| clean 解调 `stage_total_us` | 2893–3017 | 3464–4074 | 完整包 demod |
| sic 解调 `stage_total_us` | 2877–3091 | 3353–4826 | 完整包 demod（SIC 输出） |

整 run 墙钟：B1 100 窗 7.46 s（~75 ms/窗，**被离线生成主导**）。SIC 核心 ~3.1 ms/窗
沿用 S1 的真实负载量级，**未达 S1 的 mean≤0.5 ms/200 窗/s 门槛**；本任务是开发与效果
验证，未声明吞吐验收。正式吞吐 benchmark 需串行（`flock -n /tmp/uwb-sic-benchmark.lock`）
并与另一 session 协调；本任务期间 load avg ≈ 0.4–1.0，未与另一 session 的正式 benchmark 重叠。

---

## 13. 未实跑 / 失败 / OPEN

* **MATLAB 未实跑**：`testdata/sic_payload/matlab/*.m` 提供但未执行，其输出未验证。
* **native/UHD/FPGA/TWR/多干扰源/连续 1 GS/s** 不在本轮（继承 S0/S1 边界）。
* 正式**吞吐 benchmark**（SIC mean≤0.5 ms 等）不在本轮效果矩阵口径；如需，须串行
  `flock -n /tmp/uwb-sic-benchmark.lock` 并记录 CPU/load（本任务期间 load avg ≈ 0.4–1.0）。
* B2/B3 按正式计数采集：B2 每 cell 1000、B3 每 cell 500、B4 每 cell 1000、
  B1 100；共 25 cells、21100 个配对窗。所有 cell 经 `verify_s2b.py` 独立复核通过。
* 已知边界：raw 的 PER 曲线在采样 SNR 区间**不跨越目标**，故固定-PER 的 SNR 差记 **N/A**；
  CFO/四径只测到 ±50 kHz 与 4 径，未扫更宽的 CFO/更多径。
* `PU`：无。

---

## 14. 可复现命令

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-payload-collision
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2 --target gnuradio-uwb uwb_python
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-payload --output-on-failure -R 'preamble_sic|sic_payload'

# QA / verifier self-test
env -u LD_LIBRARY_PATH UWB_BUILD_DIR=gr-uwb/build-sic-payload python3 gr-uwb/apps/test_sic_payload.py
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --self-test

# smoke + verifier
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py --build-dir gr-uwb/build-sic-payload \
    --smoke --output gr-uwb/build-sic-payload/results/s2b_b0_smoke
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --results gr-uwb/build-sic-payload/results/s2b_b0_smoke

# full staged matrix (serial) + aggregate
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_payload.py --build-dir gr-uwb/build-sic-payload \
    --matrix --output gr-uwb/build-sic-payload/results/s2b_matrix
env -u LD_LIBRARY_PATH python3 tools/sic/s2b_aggregate.py --matrix gr-uwb/build-sic-payload/results/s2b_matrix
```
