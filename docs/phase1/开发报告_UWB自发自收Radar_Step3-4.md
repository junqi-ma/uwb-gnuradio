# 开发报告：UWB 自发自收 Radar Step 3–4

> 日期：2026-09-04
> 分支：`feature/uwb-monostatic-radar`
> 对照评审：[Review_UWB自发自收Radar_Step3-4.md](Review_UWB自发自收Radar_Step3-4.md)
> 状态：**R1–R3 已按 Codex 整改落地，申请复验 Step 3–4。不是 Phase A / EchoTimer / UHD / X410 完成。**
> 未改 [`../../开发状态.md`](../../开发状态.md)。未开始 Step 5（PDU 65/48）。

---

## 1. 复验结论

| 项目 | 结论 |
|---|---|
| R1 prepared profile | `prepare_radar_cir_core` 冻结 code/SFD/SYNC/sps/`tx_profile_id`；runtime 错配一律 `InvalidInput` |
| R1 64 配 32 | canonical 64-SYNC + `sync_repetitions=32` → `InvalidInput`，`tap_count=0` |
| R2 32/64/128 完整包 | 64 = MATLAB canonical；32/128 = MATLAB 64-SYNC 完整包裁剪/拼接，含 PHR/PSDU/FCS 与整包 48/65 |
| R3 多路径 | 从 MATLAB clean raw CIR 按已知延时/复增益构造 expected；完整 complex L2 + 相对增益 |
| R3 delay=0/1 与窗边界 | delay=0/1 峰位精确；首/末合法 tap 与越界截断有契约 |
| Radar 专项 | **4/4 Passed** |
| demod 回归 | `qa_uwb_demod_core`、`qa_uwb_realtime_demodulator` Passed |
| 全量 CTest | **21/21 Passed** |

未宣称 Phase A、UHD 或 X410。未进入 Step 5。

---

## 2. R1 — prepared profile 绑定

`RadarCirCoreScratch.prepared` 在 `prepare_radar_cir_core(cfg, ...)` 时写入不可变身份：

- `code_index`、`sfd_mode`、`n_sfd_symbols`
- `sync_repetitions`、`samples_per_symbol`
- `tx_profile_id`
- `max_pre` / `max_post`

`radar_cir_one` 比较 runtime `RadarCirConfig` 与 prepared profile，任一不一致返回 `InvalidInput` 并 `tap_count=0`，不输出 origin/CIR。算法身份只从 prepared 读取。切换 32/64/128 必须重新 prepare。

负向 QA：`ieee` vs `4z2`、code 10 vs 9、64 配 32/128、`samples_per_symbol`、`tx_profile_id`。

坐标乘法/加法走 `radar_i64_*`，溢出 `InvalidInput`。

---

## 3. R2 — 32/64/128 完整 packet

BPRF `lrwpanHRPConfig` 的 `PreambleDuration` 只有 16/64/1024/4096；32/128 是 HPRF 专属。未改用 HPRF。

路径：MATLAB 先生成 **BPRF 64-SYNC 完整包**（SYNC+SFD+PHR+PSDU/FCS），再

- 32：保留前 32 个 pulse-shaped SYNC + 原 SFD/PHR/PSDU/FCS
- 128：SYNC 字段拼接两次 + 原 SFD/PHR/PSDU/FCS
- 对**整包**一次性 `resample(x,48,65)`

`metadata.profile_kind`：64 canonical 为 `standard`（`export_uwb_radar_golden.m`）；32/128 为 `custom_sync_length`（`export_uwb_radar_packet.m`）。

| N | tx @998.4 | native | SFD（RX） | CIR valid | peak |
|---|---:|---:|---:|---:|---:|
| 32 | 158400 | 116973 | 34509 | 22 | 18 |
| 64 | 190912 | 140982 | 67021 | 54 | 18 |
| 128 | 255936 | 188999 | 132045 | 54 | 18 |

现有 `N×SYNC+SFD` 合成片段仍作为快速结构单测，不是 Step 4 完整包验收。

---

## 4. R3 — 多路径与边界

独立 expected：把已审核 MATLAB `cir_raw_clean_radar.cf32` 按整数延时平移并乘复增益。不是 C++ 自比自。

- delay=0：峰 18，对齐 MATLAB clean
- delay=1：峰 19，raw/norm L2 vs 平移 MATLAB CIR
- 3 path：`(0, 0.30)`、`(25, 1.00)`、`(50, 0.25+j0.40)`；峰在最强径；相对复增益含相位
- 首径落到 tap 0 / 末 tap；再向外 1 tap 为截断（仍 `Ok`，峰贴边）
- `pre=0, post=1`；prepare 容量不足 → `InvalidInput`

窗边缘路径的 L2 容差放宽到 `5e-3`（有限 CIR 窗截断），峰位仍精确。delay=1 / 多路径主门限 `1e-4`。

---

## 5. Step 4 整合契约

```text
RX window + predicted SFD
  → search_sfd
  → SFD success ? refine_sync_origin : SfdFailed
  → timing success ? estimate_radar_cir : TimingFailed
  → CIR success ? Ok (raw+norm taps) : CirFailed
```

- 失败：`tap_count=0`，失败阶段下标保持 `-1`，scratch taps 清零。
- `preamble_start_sample`：RX refine origin。
- `cir_origin_sample`：TX-time origin（`predicted_sfd - N×sps`）。
- 整数延时 37 → 峰 **55 = 18+37**。
- `sfd_search_margin=64` 仅为软件默认。

---

## 6. 测试命令与结果

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build -R 'uwb_qa_uwb_radar_' --output-on-failure
ctest --test-dir gr-uwb/build -R 'qa_uwb_demod_core|qa_uwb_realtime_demodulator' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
```

Radar 4/4、demod 回归、全量 **21/21 Passed**。

32/128 重生：

```matlab
cd testdata/uwb_radar
run_export_radar_packets
```

---

## 7. 未做

- Step 5 PDU 65/48 metadata / 坐标契约
- PacketSource、LoopbackEcho、message CIR block、CirWriter
- Phase A、EchoTimer、UHD、X410
- 32/128 不是 toolbox 原生 BPRF `PreambleDuration`；已在 metadata 标明 `custom_sync_length`
- 未改 `开发状态.md`
