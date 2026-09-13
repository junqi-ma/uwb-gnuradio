# 测试报告：UWB Radar PDU 模式速率优化（M0–M4）

> 依据：`docs/phase1/开发指南_UWB_Radar_PDU模式速率优化.md`
> 设备：X410 CG400（原生 491.52 MS/s），`addr=192.168.10.2`
> TX/RX0 ch0，RX RX1 ch3
> 日期：2026-09-13
> 代码：本次改动（见文末 commit），基线 commit `0dbfda5`

本报告逐项区分 **已验证 / 未验证 / 环境阻塞**。凡无硬件长跑的结论一律不写
“硬件验收通过”。

## 1. 交付物与状态

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | Python PDU 链可信基线（100/200 Hz） | 已完成（`analysis_outputs/m0_baseline/`） |
| M1 | C++ `UwbRealtimeEchoTimer` UHD 工厂 + 绑定 + app 接线 + 雷达 metadata | 已完成并上板 A/B |
| M2 | C++ echo 端 `publish_native`/ROI 发布 | 已完成，QA 覆盖；上板用默认 auto ROI |
| M3 | PDU 65/32 持久多 worker（`num_workers`） | 已完成；200 Hz 1 worker 已达标 |
| M4 | 复制审计 | 已完成（`analysis_outputs/pdu_copy_audit.md`），不建议本阶段改 |
| M5 | ROI 重采样 + CIR 融合块 | 未做（200 Hz 已达标，无必要） |
| M6 | 物理 RX 缩窗/批量 RX | 未做（属独立课题） |

## 2. M0 基线（python backend）

见 `analysis_outputs/README.md` / `m0_baseline/`：

| 指标 | 100 Hz | 200 Hz |
|---|---:|---:|
| pulses | 1000 | 1000 |
| schedule_wall_s | 10.242 | 5.248 |
| echo_ok / late | 1000 / 0 | 999 / **1** |
| est_drop | 0 | 0 |
| CIR ok | 1000 | 999 |
| est service mean/max (us) | 381 / 538 | 379 / 531 |
| publisher tolist/pmt/total mean (ms) | 1.08 / 0.79 / 1.98 | 1.08 / 0.82 / 2.01 |

结论：200 Hz 已处 Python 主线程临界（`uhd_ms` 均值 5.08 ms ≈ PRI 5 ms，1 个
late）；publisher 每脉冲 `tolist()+init_c32vector` ~1.9 ms 且持 GIL。

## 3. M1 C++ PDU echo（上板 A/B）

命令固定 `--preamble-length 128 --pulse-shape minphase --gain-tx 50
--gain-rx 60 --cal-delay-native 334 --no-udp`，`--echo-backend cpp-pdu`。
详见 `analysis_outputs/cpp_pdu_ab/README.md`。

| case | backend | res-workers | pulses | wall_s | echo_ok | late | res_drop | est_drop | wr_ok |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 Hz | python | – | 1000 | 10.242 | 1000 | 0 | 0 | 0 | 1000 |
| 100 Hz | cpp-pdu | 1 | 1000 | 10.260 | 1000 | 0 | 0 | 0 | 1000 |
| 200 Hz | python | – | 1000 | 5.248 | 999 | 1 | 0 | 0 | 999 |
| 200 Hz | cpp-pdu | 1 | 1000 | **5.266** | **1000** | **0** | 0 | 0 | **1000** |
| 200 Hz | cpp-pdu | 4 | 1000 | 5.246 | 1000 | 0 | 0 | 0 | 1000 |

**M1 验收（fixed 100/200 Hz）**：
- ✅ 200 Hz：`0 late`、`0 overflow`（无 UHD overflow 标记）、`0 res/est drop`、
  CIR 1000/1000；
- ✅ CIR 几何与 python 链逐字段一致（origin/cal/predicted SFD）；
- ✅ C++ PDU 与旧 Python PDU 的 payload/metadata 契约一致（SC16 + 全几何）；
- ✅ 运行期 RX scratch 地址/capacity 不变、无热路径分配（QA）；
- ⚠️ `publish_burst` 的 SC16 路径使 `cir_peak_metric`/`raw_l2_norm` 比 Python 链
  大 ~32768×（int16 满量程）；CIR taps 归一化、判决功率归一化，不受影响（§5）。
- ⚠️ 逐帧 CIR 数值对照无法用跨进程硬件抓取完成（亚样本抖动使 `cir_norm`
  余弦 ~0.27）；数值一致性用离线/同输入 QA 覆盖。

## 4. M2 ROI 发布

- C++ echo `publish_burst` 支持 `publish_native`（schedule 覆盖 > 块级 > 0=全窗）；
  `sample_count`/`rx_samples_received` 保持**物理窗**不变，新增
  `published_samples`。`--publish-native -1` 由 app 用 `cir_publish_native()`
  解析后显式下发。
- QA：`test_echo_timer_publish_roi`（长度=2·K、`sample_count` 仍物理值）。
- 上板：默认 auto ROI（128 preamble → 64992 native），
  `res_drop=0`、CIR 1000/1000，说明 ROI 前缀覆盖了 CIR 读取范围。

**未验证**：`--require-sfd`/更大 search margin 下的 ROI 上界单独推导；全窗
(`--publish-native 0`) 与 auto ROI 的逐帧复数 taps 对照（应离线做）。

## 5. M3 PDU 65/32 持久 worker

- `make/make_from_taps/ctor` 增加 `int num_workers=1`；`set_num_workers`/
  `num_workers`/`resampler_kernel`；构造时建持久 pool（core 既有实现）。
- QA `qa_uwb_pdu_rational_resampler_65_32_workers.cc`：1/4/8 worker 输出
  **逐样本一致**（n_in=1000 与 100000）、默认=1、`set_num_workers` 等价、
  malformed/oversize 仍 drop、重复 start 安全；并新增 **SC16 输入与 FC32 输入
  逐样本一致**（同一 int16 码字）的用例，直接验证 `cpp-pdu` 发布的 native SC16
  载荷契约（几何 metadata 亦一致，仅 `input_sample_format` 不同）。
- app 增加 `--res-workers`（默认 1）。
- 上板：200 Hz 下 1 与 4 worker 均 0 late / 0 est_drop → **达到 0 late 的最小
  worker 数 = 1**；4 无收益。
- ⚠️ 500 Hz：res-workers=1 本次 2500/2500 通过，但 `worker_us_mean=1981 us ≈
  PRI 2000 us`，未满足“逐 burst UHD <2 ms”；res-workers=8 反而 50 late / 1 fail
  （多 worker 抢占 UHD 核）。**500 Hz 未验收通过**，只作瓶颈定位。

## 6. M4 复制审计

`analysis_outputs/pdu_copy_audit.md`：链上大数组复制逐项列明；`d_rx_buf_→
s16vector` 与 `d_scratch_→c32vector` 因 PMT 所有权必须保留；core `work_` 组装在
one-shot 下有一次冗余 memcpy，但依赖流式状态机，收益有限。本阶段**不声称消除
PDU 复制**。

## 7. 构建与测试

- `cmake --build gr-uwb/build -j` 通过（UHD backend ON）。
- CTest：**38/39 通过**。唯一失败 = 已知环境相关
  `uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值（`pdus/s=216` vs 阈值 400；
  在改动前即失败，`开发状态.md` 已记录）。相关定向 QA 全过。
- Python 绑定冒烟：`uwb.realtime_echo_timer`、`uwb.realtime_echo_timer_uhd`、
  `fake_burst_backend`、PDU 65/32 `num_workers`/`set_num_workers` 均可导入调用。
- Python QA：`test_echo_stream_buffers.py`、`test_freq_plan.py`、
  `test_cir_udp_format.py`（本报告未改动这些模块）。

## 8. 未验证 / 环境阻塞

- **X410 soak（≥60 s / 10 min）**：未做；本报告只做 5–10 s 短跑。不得写成
  soak 验收通过。
- **500 Hz 稳定达标**：未通过（见 §5）。
- **scan/manual retune 与 peak servo** 在 `cpp-pdu` 下的实机验收：未做
  （`sweep` app 尚未接 `--echo-backend`；本次只做 fixed）。
- **跨频点群时延 / retune settling**：未做。
- **ROI 逐帧复数 taps 对照**（full vs auto）：未做。
- **多进程/ASan/TSan**：未做。
- `uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值：本机环境阻塞（改动前即失败）。

## 9. 复现命令

```bash
# Python 基线
python3 gr-uwb/apps/bench_pdu_throughput.py --run --rates 100,200 \
    --pulses 1000 --out-root analysis_outputs/m0_baseline

# C++ PDU echo（fixed）
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --args addr=192.168.10.2 --echo-backend cpp-pdu --res-workers 1 \
  --pulses 1000 --pri-s 0.005 --preamble-length 128 --pulse-shape minphase \
  --gain-tx 50 --gain-rx 60 --cal-delay-native 334 --no-udp \
  --output /tmp/cpp_200
```

## 10. 改动文件

- `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
- `gr-uwb/lib/uwb_realtime_echo_timer.cc`
- `gr-uwb/lib/qa_uwb_echo_timer.cc`
- `gr-uwb/include/gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h`
- `gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_32.cc`
- `gr-uwb/lib/qa_uwb_pdu_rational_resampler_65_32_workers.cc`（新）
- `gr-uwb/python/uwb/bindings/python_bindings.cc`
- `gr-uwb/grc/uwb_realtime_echo_timer.block.yml`（新）、
  `gr-uwb/grc/uwb_pdu_rational_resampler_ccf_65_32.block.yml`、
  `gr-uwb/grc/CMakeLists.txt`
- `gr-uwb/apps/x410_cg400_hrp_echo_cir.py`（`--echo-backend`、`--res-workers`）
- `gr-uwb/apps/bench_pdu_throughput.py`（新）
- `gr-uwb/lib/CMakeLists.txt`
- `analysis_outputs/{README.md,m0_baseline/,cpp_pdu_ab/,pdu_copy_audit.md}`
