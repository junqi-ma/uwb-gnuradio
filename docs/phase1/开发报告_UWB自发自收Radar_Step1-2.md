# 开发报告：UWB 自发自收 Radar Step 1–2（Codex 整改复验）

> 日期：2026-09-04
> 分支：`feature/uwb-monostatic-radar`
> 对照评审：[Review_UWB自发自收Radar_Step1-2.md](Review_UWB自发自收Radar_Step1-2.md)
> 状态：**R1–R3 已按 Codex 整改落地，申请复验 Step 1–2。不是 Phase A / EchoTimer / X410 完成。**
> 未改 [`../../开发状态.md`](../../开发状态.md)。

---

## 1. 复验结论

| 项目 | 结论 |
|---|---|
| Step 1 canonical TX | MATLAB R2025b `lrwpanWaveformGenerator` BPRF SFD#2，20 data + FCS，整包 48/65；长度 **190912** @998.4 |
| Step 1 CIR | clean / integer / fractional 均有 raw+norm；MATLAB verifier **hard FAIL**；normalized L2 **1.9e-8 / 3.0e-8 / 2.2e-8** |
| Step 2 SFD | 窄窗内 **逐整数起点** 满速率搜索；合成与 MATLAB golden 均穷举 **[-64,+64] 129 点** |
| Python splice | 迁到 `testdata/uwb_radar_synthetic/`，不再覆盖 canonical |
| 全量 CTest | **18/18 Passed**（含既有 overflow 字符串检查：改为匹配应用里的 `uhd_overflow_to_control`） |

未宣称 Phase A、UHD 或 X410。未正式进入 Step 3，待本次复验通过。

---

## 2. R1 — SFD 搜索正确性

`search_sfd` 删除 stride-8 粗搜。clipped window 内每个合法整数 `j` 都做满速率相关 + 滑动窗功率。

输入契约：

- `threshold` 必须有限且 `> 0`，否则 `InvalidInput`
- `prepare_sfd_template` 拒绝非有限样点或零能量模板

QA 增补：

| Case | 结果 |
|---|---|
| `test_radar_sfd_exhaustive_synthetic_offsets` | 129 个偏移全部命中，误差 ≤1 |
| `test_radar_sfd_canonical_matlab_golden` | 要求 `generator=export_uwb_radar_golden.m`；TX 第一 SYNC 作模板；129 偏移全过 |
| `test_radar_sfd_pure_noise_false_alarm_zero` | 5 个固定 seed，全部 `SfdFailed`、start=-1 |
| `test_radar_sfd_awgn_detects` | 含 SFD 的 AWGN，4 seed，误差 ≤1 |
| invalid threshold / 全零模板 / NaN 模板 | `InvalidInput` / prepare 失败 |
| 错 SFD mode | 必须失败，不再允许“成功但 metric 低” |

`uwb_qa_uwb_radar_sfd_core.cc` **Passed (0.29 s)**。

---

## 3. R2 — MATLAB canonical golden

| 项 | 值 |
|---|---|
| generator | `export_uwb_radar_golden.m` |
| MATLAB | `25.2.0.2998904 (R2025b)` |
| Mode | BPRF，`SFDNumber=2` |
| PSDU | 20 data + 2 FCS，rng 20260904 |
| `tx_998p4` | **190912**（与评审中 toolbox 实测一致；Python splice 曾是 256448） |
| native | `resample(x,48,65)` 整包，**140982**；未重复 751-SYNC |
| 身份字段 | `matlab_version`、`phy_mode`、`payload_bytes_hex`、`sfd_mode` |

Python 拼接包在 `testdata/uwb_radar_synthetic/`。C++ canonical QA **不允许 skip**。

重生（Windows MATLAB，仓库在 WSL 时请先拷到盘符再拷回，或直接在 MATLAB 里 `cd testdata/uwb_radar`）：

```matlab
cd testdata/uwb_radar
export_uwb_radar_golden
verify_uwb_radar_golden   % 只打印 PASS 或 FAIL / SKIP/UNAVAILABLE
```

本次实际：在 `C:\Users\junqima\AppData\Local\Temp\uwb-gnuradio\testdata\uwb_radar` 生成后拷回仓库。verifier 输出 **PASS**。

---

## 4. R3 — CIR verifier 与分数时延

`verify_uwb_radar_golden.m`：

- 不再 `try/catch` 后打印 PASS
- CIR 对 `uwbdecoder.estimateCir` 是 hard gate
- 缺 toolbox 打印 `SKIP/UNAVAILABLE` 并 `ok=false`（不打印 PASS）
- 断言失败打印 `FAIL:` 后 `error`

实测（R2025b）：

| 项 | 结果 |
|---|---|
| clean norm L2 vs estimateCir | 1.914e-08 |
| delay_int norm L2 | 2.969e-08 |
| delay_frac norm L2 | 2.183e-08 |
| 整数时延峰移 | **37**（相对 clean tap 18→55） |
| delay_int raw 幅度 / (|gain|·clean) | **1.000** |
| delay_frac raw 幅度比 | 0.938（门限 0.15） |
| native round-trip SYNC 间隔 | mean=1016，max\|err\|=0 |

分数时延 CIR 文件：`cir_{raw,norm}_delay_frac_radar.cf32`（及 8/30 对照）。插值核：**MATLAB `interp1 pchip`**。clean 算法峰在 tap **18**（pre+2 成形），未把 +2 写成物理零距离。

---

## 5. 全量回归

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build --output-on-failure
```

**18/18 Passed.**

既有失败 `test_x410_app_forwards_uhd_overflow`：应用源码用的是 `async_msgs` / `uhd_overflow_to_control`，测试却找不存在的 `uhd_async_msg`。已把断言改为 `uhd_overflow_to_control`（应用内 overflow 转发器类名）。**未改 X410 捕获逻辑。**

---

## 6. 评审硬门槛对照

- [x] canonical TX 是 MATLAB 正常 packet，固定合法 PHR/PSDU/FCS
- [x] 998.4 整包一次性 48/65，未重复 751-SYNC
- [x] clean/integer/fractional RX 均有 raw 和 normalized CIR
- [x] MATLAB verifier CIR 不允许 skip 后 PASS
- [x] SFD `[-64,+64]` 129 偏移无噪成功，误差 ≤1（合成）
- [x] MATLAB canonical RX 同样全偏移
- [x] 缺失/破坏/错 mode/纯噪声 → `SfdFailed`、start=-1
- [x] normalized complex CIR 相对 L2 < 1e-5
- [x] raw CIR 幅度线性（整数时延 ×0.4 精确为 1.0）
- [x] `search_sfd` 热路径无动态扩容
- [x] Radar 专项 + demod 回归通过
- [x] 全量 CTest 18/18
- [x] 不宣称 Phase A / EchoTimer / X410

---

## 7. 仍保持的评审决定（未推翻）

1. Python splice **不能**当 Step 3 MATLAB 对齐输入。
2. CIR expected **不**改成物理 `pre+2`；算法峰对齐 MATLAB，物理距离走显式校准。
3. `sfd_search_margin=64` 仅软件默认，硬件群时延标定前不冻结。
4. 正式进入 Step 3 仍等本次复验通过。

---

## 8. 审核者可跑的命令

```bash
python3 testdata/uwb_radar_synthetic/verify_uwb_radar_golden.py
ctest --test-dir gr-uwb/build -R 'qa_uwb_radar_sfd_core|qa_uwb_demod_core' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
```

有 MATLAB：

```matlab
cd testdata/uwb_radar
verify_uwb_radar_golden
```
