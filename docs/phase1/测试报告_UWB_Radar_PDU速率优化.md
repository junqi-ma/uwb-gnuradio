# 测试报告：UWB Radar PDU 模式速率优化（整改后）

> 依据：`docs/phase1/开发指南_UWB_Radar_PDU模式速率优化.md`
> 与 `docs/phase1/整改指南_UWB_Radar_PDU速率优化验收.md`。
> 设备：X410 CG400（原生 491.52 MS/s，FPGA `CG_400`），`addr=192.168.10.2`，
> TX/RX0 ch0、RX1 ch3。日期：2026-09-13。
> 代码：`cd2f45b`（P0 算法+QA）、`d197466`（接线）；基线 `0dbfda5`。

本报告按整改指南 §1/§10 重新分级，逐项区分 **已验证 / 未验证 / 环境阻塞**。
固定参数（除注明）：`--preamble-length 128 --pulse-shape minphase
--gain-tx 50 --gain-rx 60 --cal-delay-native 334 --no-udp`。

## 1. 里程碑状态（整改后）

| 项目 | 状态 |
|---|---|
| M0 | 100/200 Hz Python 基线已采集；500/1000 Hz 在 `0dbfda5` 已采集 |
| M1 | C++ PDU fixed 实现 + SC16/FC32 数值契约已修并有 QA；**200 Hz 60 s 与 10 min soak 均通过** |
| M2 | ROI 机制 + 同输入 full/auto QA；require-SFD auto ROI 已强制 full fallback |
| M3 | 持久 worker + QA；200 Hz 1 worker 达标 |
| M4 | 复制审计完成（`analysis_outputs/pdu_copy_audit.md`） |
| 200 Hz | **fixed 验收通过**（60 s + 10 min soak，0 late/0 overflow/0 drop，队列无正斜率） |
| 500 Hz | 压力测试，未验收（§5） |

## 2. P0：SC16/FC32 幅度契约（已修）

原 PDU 65/32 对 SC16 输入用 `float(int16)`，使 cpp-pdu 的 raw CIR、`cir.cf32`、
`raw_l2_norm`、`cir_peak_metric`、UDP payload 比旧 Python（UHD fc32）链大约
32768×。整改（commit `cd2f45b`）：

- 新增 `Sc16ScalePolicy { RawInteger（默认，兼容通信链）, UnitRange =
  float(int16)/32768 }`；block 输出 `input_iq_scale`/`output_iq_scale`；
- app `--res-sc16-scale {auto,raw,unit}`，cpp-pdu 自动选 `UnitRange`；
- `published_samples` 透传。

QA（`uwb_qa_uwb_pdu_sc16_scale_65_32.cc`，通过）：
- A=SC16 `UnitRange` 与 B=FC32 `complex(int16/32768)` 经 65/32 后逐样本一致
  （`<=1e-7`），几何 metadata 逐字段一致；
- C=SC16 `RawInteger` == A×32768；
- `published_samples` 透传。

上板结果（200 Hz，100 pulses，cpp-pdu）：

| 指标 | 整改前(RawInteger) | 整改后(UnitRange) | 旧 Python 链 |
|---|---:|---:|---:|
| `cir_peak_metric` mean | ~1734 | **0.0539** | 0.0529 |
| CIR ok | 100/100 | 100/100 | — |

raw taps / `cir.cf32` / UDP 现在与旧链同量纲。

## 3. P1：ROI 正确性（M2）

- **同输入 full-vs-auto QA**（`uwb_qa_uwb_pdu_roi_cir_65_32.cc`，通过）：
  同一输入窗，FULL（payload=全窗）与 ROI（payload=前 K，`sample_count` 仍为物理
  窗）经 65/32 + Radar CIR estimator 后 `status`/`cir_origin_sample`/
  `predicted_sfd_start_sample`/`peak_tap`/raw taps/normalized taps/
  `cir_peak_metric`/`raw_l2_norm` **逐帧一致**；覆盖 predicted timing 与
  require-SFD。
- **require-SFD 安全策略**：app 在 `--require-sfd --publish-native -1` 时打印
  警告并**退回 full window**（auto ROI 的推导只覆盖 predicted-timing 读取范围，
  没有经证明的 SFD 搜索上界）。
- metadata 区分保留：`rx_samples_requested/received`、`sample_count`（物理窗）/
  `published_samples`（payload 前缀）/ `input_sample_count`（实际读入）/
  `full_output_sample_count`（实际输出）。

未验证：不同 preamble/cal 极值的 full/auto 逐帧对照（QA 只跑了单组几何）；实机
`--require-sfd` ROI 对照。

## 4. X410 实测：fixed 200 Hz

### 4.1 smoke 与 60 s soak（cpp-pdu，`UnitRange`）

| 用例 | 规模 | wall_s | echo_ok | late | res/est drop | CIR ok | est_q_hwm | 队列 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 200 Hz smoke | 100 | — | 100 | 0 | 0 | 100 | 1 | 平坦 |
| **200 Hz soak-1** | 12000 (60 s) | 60.254 | **12000** | **0** | **0** | **12000** | 1 | `res_in-res_out`≤1、`est_in-est_done`≤1，无正斜率 |

60 s soak 关键量：`wr_ok=12000`、`unique_ids=12000`、`missing_count=0`、
`dup_count=0`、`peak_tap 20–21`、`metric_mean=0.0545`、`echo_queue_hwm=1`。
backlog 时间序列（每 5 s）首/末：`res 1→1`、`est 0→0`（12 个采样点）。

### 4.2 10 min soak（soak-2，已通过）

命令：cpp-pdu，`--pulses 120000 --pri-s 0.005`（≈600 s），rc=0。

| 指标 | 值 |
|---|---:|
| `schedule_wall_s` | 600.250 |
| `echo_ok` / `echo_fail` / **`echo_late`** | 120000 / 0 / **0** |
| `res_rx`/`res_tx`/`res_drop` | 120000/120000/**0** |
| `est_rx`/`est_done`/`est_drop` | 120000/120000/**0** |
| `est_queue_hwm` / `echo_queue_hwm` | 1 / 1 |
| `wr_ok` / `wr_fail` | 120000 / 0 |
| CIR ok / unique_ids / missing / dup | 120000 / 120000 / 0 / 0 |
| `peak_tap` | 17–18 |
| `metric_mean` | 0.05159 |
| worker 分段均值 (us) | total 4886、rx_collect 1397、tx_send 3285、pdu_build 180、pdu_publish 5 |

backlog（120 个 5 s 采样点）：`res_in-res_out` ∈ [0,1]，前半/后半均值
0.567/0.583；`est_in-est_done` ∈ [0,1]，前半/后半均值 0.067/0.083 →
**无长期正斜率**。无 UHD overflow、无 late、无 estimator drop。

**结论：200 Hz fixed X410 验收通过（60 s + 10 min 双档，0 late/0 overflow/
0 downstream drop，队列无正斜率）。** 仍受限于 §9 的未验证项（scan/servo/
dump/UDP 实机、分位数、单流图 QA），故只对 fixed 200 Hz 声明验收。

### 4.3 Python/C++ A/B 多轮（200 Hz，1000 pulses，各 3 轮）

Python 臂用**基线 commit `0dbfda5` 的 worktree**（`/tmp/opencode/wt-base`），
C++ 臂用当前提交。

| 轮 | backend | echo_ok | late | res/est drop | wr_ok | metric_mean |
|---|---|---:|---:|---:|---:|---:|
| py1 | python@0dbfda5 | 999 | 1 | 0 | 999 | 0.05394 |
| py2 | python@0dbfda5 | 998 | 2 | 0 | 998 | 0.05243 |
| py3 | python@0dbfda5 | 999 | 1 | 0 | 999 | 0.05412 |
| cpp1 | cpp-pdu | 1000 | 0 | 0 | 1000 | 0.05158 |
| cpp2 | cpp-pdu | 1000 | 0 | 0 | 1000 | 0.05219 |
| cpp3 | cpp-pdu | 1000 | 0 | 0 | 1000 | 0.05361 |

波动范围：Python late 1–2、cpp late 恒 0；两臂 `metric_mean` 均
0.052–0.054（SC16 契约已统一）。

## 5. P1：500 Hz 压力（未验收）

- cpp-pdu w1 短跑曾 2500/2500、0 late，但 `worker_us_mean≈PRI 2 ms`；
  w8 出现 50 late（多 worker 抢 UHD 核）。
- **`0dbfda5` 真基线**（`analysis_outputs/m0_baseline_500hz/`）：500 Hz
  2500 pulses → echo_ok **1227**、late 1273、`res_rx==echo_ok`、`res/est_drop=0`、
  rc=3；1000 Hz → 599/2500。Python 链在 500 Hz 就是调度层失败，下游不积压。
- 结论：500 Hz 未验收；逐 burst UHD 路径未稳定 <2 ms，属后续 M6。

## 6. P1：worker 分段计时与 backlog（部分完成）

新增 cpp-pdu worker 分段 timing（`slot_lead / issue_rx / tx_send / rx_collect /
pdu_build / pdu_publish / worker_total`）并写入 `summary.json`；live 每 5 s 输出
backlog 时间序列。

示例（200 Hz，100 pulses，均值被 pulse 0 的 0.25 s arm 拉高）：

| 段 | mean (us) |
|---|---:|
| worker_total | 7239 |
| rx_collect（含等待设备时钟） | 3764 |
| tx_send | 3259 |
| pdu_build（metadata+PMT） | 191 |
| pdu_publish | 5 |

**未完成**：p50/p95/p99 每段分位数、各线程 CPU/上下文切换、slot_wait 与 UHD
service 的严格分离、end-to-end（RX 完成→CIR 发布）延迟。当前只有 mean/max。

## 7. P0：queue-full 与生命周期 QA

`uwb_qa_uwb_echo_timer.cc`（通过）：capacity=1、worker 被 Timeout 占住时快速提交
3 个 schedule → handler 非阻塞（每次 <50 ms）、`schedules_received=3`、
`schedules_dropped=1`、HWM≥1、`queue_full` status；随后按 0,1,2 顺序完成
（timeout/ok/ok）；stop-during-busy 无死锁。EchoTimer 的 partial/late/timeout/
overflow/broken-chain/stop/固定 scratch 既有用例保持通过。

**未完成（P0 剩余）**：FakeBurstBackend→EchoTimer→65/32→CIR→writer 的**单一
端到端流图 QA**（当前是分 block QA + ROI roundtrip QA，未在一条流图里串
EchoTimer 与 CIR）。

## 8. 构建与全量测试

- `cmake --build gr-uwb/build -j` 通过（UHD backend ON）。
- CTest **41/42 通过**。唯一失败 = 已知环境相关
  `uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值（本机 `pdus/s≈216` vs 阈值
  400；改动前即失败）。新增 `qa_uwb_pdu_sc16_scale_65_32`、
  `qa_uwb_pdu_roi_cir_65_32`、echo queue-full 用例均通过。
- Python QA：`test_echo_stream_buffers`/`test_freq_plan`/`test_cir_udp_format`
  通过。

## 9. 未验证 / 环境阻塞（诚实清单）

- 每段 timing 的 p50/p95/p99、CPU、end-to-end p99：未插桩（只有 mean/max）。
- **scan/manual retune、peak servo、SC16 dump、UDP UCR2** 在 cpp-pdu 下的实机
  验收：未做（sweep app 已接 `set_freq`/`set_cal_delay_native`，但未上板；
  dump 在 cpp-pdu 仍禁用；UDP 量纲随本修复正确，未做接收端 loopback）。
- 单一端到端 PDU 流图 QA：未做（§7）。
- ROI 多几何（preamble 64/128、cal 极值）逐帧对照：未做。
- 物理 RX 缩窗/UHD 事务优化（M6）：未做。
- `uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值：环境阻塞。

## 9b. 默认后端变更（cpp-pdu）

自本轮起 `x410_cg400_hrp_echo_cir.py` 与 `x410_cg400_hrp_echo_cir_sweep.py`
的 `--echo-backend` **默认改为 `cpp-pdu`**（Python 保留为 fallback）。动机：
scan retune + peak servo 会放大 Python 定时路径的 TX underflow。实测同一
300 kHz 降序 scan：

| 后端 | UHD `U`（TX underflow） | 结果 |
|---|---:|---|
| python | 5656（大量 `UUUU`） | late=22 / 21000 |
| **cpp-pdu（默认）** | **0** | 200/200 ok、late=0、retune=2、servo 锁 tap 30 |

注意：`--dump-rx/--dump-sc16` 仅 python 后端支持；cpp-pdu 下会告警并禁用。
scan 的 `--freq-step` 单位是 **Hz**（不是 kHz）；300 kHz 写 `300e3`。

## 10. 改动文件（本轮）

- `gr-uwb/include/gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h`、
  `gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_32.cc`（SC16 scale policy +
  `published_samples`）
- `gr-uwb/lib/qa_uwb_pdu_sc16_scale_65_32.cc`（新）、
  `gr-uwb/lib/qa_uwb_pdu_roi_cir_65_32.cc`（新）、
  `gr-uwb/lib/qa_uwb_echo_timer.cc`（queue-full）
- `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`、
  `gr-uwb/lib/uwb_realtime_echo_timer.cc`（分段 timing）
- `gr-uwb/python/uwb/bindings/python_bindings.cc`（scale enum + telemetry）
- `gr-uwb/apps/x410_cg400_hrp_echo_cir.py`（`--res-sc16-scale`、require-SFD
  fallback、telemetry/backlog）、
  `gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py`（cpp-pdu retune/servo）
- `gr-uwb/lib/CMakeLists.txt`
- `analysis_outputs/{cpp_pdu_ab/round2,cpp_pdu_ab/README.md,m0_baseline_500hz,
  m0_baseline_1000hz,pdu_copy_audit.md,README.md}`
