# 测试报告：QM35 盲捕获与自动周期锁定（mixed 737.28）

> 日期：2026-08-14
> 依据方案：[`GROK_X410_QM35盲捕获与自动周期锁定开发方案.md`](GROK_X410_QM35盲捕获与自动周期锁定开发方案.md)
> 代码基线：`origin/main` `af97089`（本机 `master` 已快进）
> 主线入口：[`../../开发状态.md`](../../开发状态.md)

## 1. 结论

在真实 X410 mixed 捕获上，**不指定 `first_packet_sample` / `t0`**，现有
`UwbAutoScheduledExtractorSc16` 能自动确认第一个 QM35 包，锁定已知 5 ms 周期，
并按 EverySlot 截取后续 slot。

这是离线 0.5 s 文件闭环，不是方案 §18 的完整硬件完成定义。X410 随机启动、
60 s / 10 min soak、overflow 失锁重捕获仍未验收。

| 项 | 结果 |
|---|---|
| 是否需要用户提供 t0 | **否** |
| 首锁依据 | code-9 候选 + 解调 `success` + **FCS pass** + QM35 profile |
| 最终状态 | `locked`，`identity_confirmed=true` |
| 0.5 s 窗口 | 1 acquisition + 99 scheduled，**0 drop** |
| 解调 | 100 收、99 success、**99/100 FCS pass**、1 个 `payload_failed` |
| 锁定后能量 Region | **0**（通信包不再建 Region） |

## 2. 本轮做了什么

1. 从远程拉取 `main`（`e607e16` → `af97089`），得到盲捕获自动锁定实现。
2. 阅读方案与当前 C++ / Python 实现，确认热路径是统一块，不是 Detector 与
   ScheduledExtractor 并联。
3. 重新配置并编译 `gr-uwb`（新块此前未进入本机 build）。
4. 新增真实解调闭环脚本 [`testdata/offline_qm35_auto_lock.py`](../../testdata/offline_qm35_auto_lock.py)。
   现有 `x410_auto_scheduled_capture.py` 的 file 路径默认是 **loopback 假确认**，
   不能回答 mixed 数据上的真实身份锁定。
5. 在本地 mixed 737.28 SC16 文件上跑完整 0.5 s（先用 80 ms 前缀冒烟）。

## 3. 当前实现摘要

统一块：`gr::uwb::UwbAutoScheduledExtractorSc16`
Python / GRC：`uwb.auto_scheduled_extractor_sc16`

```text
UNLOCKED_ACQUIRE   整数能量门 + code-9 粗/细相关
        ↓ 发出 acquisition PDU（保留首包）
CANDIDATE_VERIFY   PDU 65/48 + UwbRealtimeDemodulator
        ↓ success + FCS + code 9 / 64 SYNC / 4z2
PROVISIONAL_TRACK  立即按 nominal 5 ms 截后续窗（加宽 guard）
        ↓ 约 3 次 timing-ok
LOCKED             关闭全局能量门，只按 t0+kT EverySlot 截窗
```

第一版只做 **盲 t0、已知 T=5 ms**。锁定依据不是能量门限，也不是单独的 code-9
相关峰。DW1000 即使触发能量门，只要不能通过 QM35 解调 + FCS，就不会锁定。

坐标约定：schedule 只存在于 native 737.28 MHz。解调 `detected_start` 在 998.4
MHz，经已有 65/48 群时延逆映射回 native 后再更新 tracker。

生产默认窗（时间换算，不是直接复用 998.4 的 9984/189696/4096）：

```text
pre  = 7373    # 10 us
body = 140083  # 190 us
post = 3023    # 4.1 us
T    = 0.005 s # 3686400 native samples
```

## 4. 测试条件

| 项 | 值 |
|---|---|
| 输入 | `/mnt/f/UWB基带数据/dw1000_qm35_mixed_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat` |
| 场景 | metadata：`DW1000 and QM35 mixed` |
| 格式 | SC16 interleaved int16，737.28 MS/s，6489.6 MHz，0.5 s |
| 样点数 | 368,640,000 complex |
| `first_packet_sample` | **未提供** |
| 提取器模板 | `testdata/reference_preamble_code9_737p28.cf32`（751 点） |
| 解调模板 | `testdata/reference_preamble.bin`（1016 点 @998.4） |
| 重采样 | PDU 65/48，`quality_minorder` |
| 解调 | code 9、64 SYNC、4z2、`cir_filter_mode=bypass`、2 workers |
| 能量门 | threshold `0.001`，decimation 100 |
| lock_observations | 3 |

链路：

```text
file_source SC16
  -> UwbAutoScheduledExtractorSc16   # 无 t0
  -> UwbPduRationalResamplerCcf65_48
  -> UwbRealtimeDemodulator
  -> schedule_feedback -> lock_obs
```

复现：

```bash
PYTHONPATH=$PWD/gr-uwb/build/test_modules \
LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/offline_qm35_auto_lock.py --max-seconds=0.5
```

`--max-seconds=0.08` 的前缀冒烟同样锁定，16/16 FCS pass。

## 5. 结果

### 5.1 锁定过程

| 事件 | 流坐标 `abs_sample` | 状态 |
|---|---:|---|
| `acquisition_started` | 0 | `unlocked_acquire` |
| `candidate_emitted` | 1,884,045 | `candidate_verify` |
| `qm35_identity_confirmed` | 2,178,940 | `provisional` |
| `provisional_schedule_started` | 2,178,940 | `provisional` |
| `schedule_locked` | 9,346,502 | `locked` |

- 首个 acquisition PDU：`start_sample=1,690,126`（约 **2.292 ms**），fine metric 0.671。
- 身份确认墙钟：**0.10 s**。
- 进入 `LOCKED` 时流已走到约 12.7 ms。异步解调反馈到达前，流按设计继续前进；
  第一个仍在未来的 slot 是 `k=1`，预测点 `5,376,546`。
- 终态锁定参数：`t0=1,690,719.757`，`T=0.005000034 s`（3,686,425 native samples，
  相对标称 5 ms 约 **+6.8 ppm**）。timing feedback 在文件过程中持续微调 t0。

### 5.2 计数

| 计数 | 值 |
|---|---:|
| 墙钟（全 0.5 s 文件） | 12.33 s |
| energy_regions | 5 |
| energy_regions_after_lock | **0** |
| candidates_emitted / rejected | 1 / 0 |
| scheduled_windows / emitted / dropped | 99 / 100 / **0** |
| resampler emitted / dropped | 100 / 0 |
| demod rx / completed / failed / dropped | 100 / 99 / 1 / 0 |
| stale_feedback / unmapped / discontinuity | 0 / 0 / 0 |

PDU `capture_mode`：`acquisition=1`，`provisional=2`，`scheduled=97`。

### 5.3 解调

| 状态 | 数量 |
|---|---:|
| success + FCS pass | 99 |
| payload_failed | 1 |
| timing_failed | 0 |

最后一个 `payload_failed` 符合 mixed 碰撞 slot 仍按 EverySlot 输出的约定，
不能记成提取失败。

acquisition 包的 `det-pred` 为 −3021（998.4 域）：首包 seed 是 detector
timing seed，不是周期预测点，该偏差不表示失锁。进入 scheduled 后前几个 slot
的 `det-pred` 为 +13、+67、+74，之后随 clock 残差缓慢增大，tracker 同步抬升 t0。

### 5.4 80 ms 前缀

同一脚本 `--max-seconds=0.08`：

- 同样在约 0.10 s 墙钟确认身份并进入 `locked`；
- 5 个能量 Region，1 个候选，0 拒绝；
- 15 个 scheduled + 1 个 acquisition，16/16 FCS pass。

说明锁定发生在文件最前面几个 QM35 slot，不依赖读完整 0.5 s。

## 5.5 自动锁定时间开销（2026-08-14）

两套口径必须分开：**RF/流坐标时间**（方案 §15 的 lock time）和 **host 墙钟**（离线算力 + 读盘）。
`lock_observations=3`，所以 `LOCKED` 至少要等首包之后再成功解调 2 个 5 ms slot。

同一 20 ms 前缀、同一脚本，输入分别来自 `/mnt/f` 和 `/dev/shm`：

| 事件 | RF 时间 | 墙钟 `/mnt/f` | 墙钟 `/dev/shm` | 间隔含义 |
|---|---:|---:|---:|---|
| `acquisition_started` | 0 | 1.2 ms | 2.1 ms | `tb.start` |
| `candidate_emitted` | **2.555 ms** | 76.5 ms | **12.1 ms** | 能量门 + 5～9 次 code-9 验证 + PMT 拷贝 |
| `qm35_identity_confirmed` | 2.93 / 4.62 ms | 85.5 ms | **20.0 ms** | 首包 realtime demod + FCS |
| `schedule_locked` | 12.7 / 14.3 ms | 313 ms | **46.0 ms** | 再等 2 个 timing-ok slot |

首包（acquisition，190106 点 SC16）解调元数据，`/dev/shm`：

| 阶段 | 耗时 |
|---|---:|
| PDU 65/48 FIR | 2.89 ms |
| demod core | 3.13 ms |
| 其中 timing / CFO / SFD / CIR bypass | 0.49 / 0.88 / 0.92 / 0.73 ms |
| demod latency（入队到出结果） | 3.41 ms |
| 候选发布 → 身份确认（墙钟） | 7.94 ms |

后续 provisional/scheduled 窗更短（约 150～187 k 点）：65/48 ≈ 1.5～2.1 ms，demod ≈ 1.7～2.0 ms。

要点：

1. **算法 lock time 很短。** 首包在 2.29 ms，3 次 observation 后 `LOCKED` 约 **13～14 ms RF**，低于方案 P95 ≤50 ms。
2. **身份确认的纯计算约 8～9 ms**，几乎就是「一次 65/48 + 一次 QM35 demod」。能量门本身不是大头。
3. **捕获期 `work()` 会 `wait_for_worker_idle()`。** 等的是 code-9 `verify` +
   `init_s16vector` 拷贝，不是 65/48。GNU Radio 3.10 的 `message_port_pub` 只
   `insert_tail` 到 resampler 消息队列，FIR 在 resampler 自己的 TPB 线程跑。
   `/dev/shm` 上从 0 到候选约 10 ms 墙钟只吃了 1.88 M 样点，约 **190 MS/s = 0.26× 实时**。
4. `/mnt/f` 的 75 ms 候选墙钟主要是盘（约 100 MB/s），不能拿来判断 X410。
5. 捕获期越慢，能量门开得越久，mixed 里每个 DW1000 突发都会变成一次 code-9 粗扫。实时 737.28 MS/s 上，解调那 8 ms 里会再涌入约 16 个通信 Region，进一步拖住 `work()`，有 overflow 风险。
6. **进入 `LOCKED` 后不再 `wait_for_worker_idle`，也不再跑能量门。** 0.5 s 全文件 12.3 s 墙钟是 100 个窗的 65/48+解调 + 读 1.4 GB，不是锁定算法。

实时 X410 上更可能的时间线：身份反馈约在首包后 8～10 ms 返回，紧接着的 5 ms slot（k=1）大概率已经错过，k=2 仍可截到；`LOCKED` 大约在首包后 15～20 ms。前提是捕获期算力跟得上，否则先 overflow。

### 5.6 65/48 与 extractor worker 的关系（更正）

线程上二者**已经分离**：extractor worker 发布 PDU 后立即 `release_region`；
65/48 在 `UwbPduRationalResamplerCcf65_48` 的消息线程执行。再拆一层线程
不会自动加快捕获期。

当前 IQ 拷贝链（acquisition 约 190 k 点）：

| 步骤 | 动作 | 字节 |
|---|---|---:|
| 流 → Region 池 | 必要 | 760 KB SC16 |
| Region → PMT `s16vector` | 为立刻还池 | 760 KB |
| PMT → resampler `d_input_scratch_` | SC16→FC32 | 1.52 MB |
| FIR → `d_scratch_` | 计算写 | ~2.06 MB |
| scratch → PMT `c32vector` | 交给 demod | ~2.06 MB |
| demod job | 持有 PMT，不再拷 IQ | 0 |

真正可省的是「Region→PMT」：让 Region 多占池约 3 ms，直到 FIR 读完。
不要再经中间 `std::vector` 队列转一手，那才会多一次复制。捕获期 0.26×
实时的主因仍是 `verify` + `wait_for_worker_idle`，不是这条 PMT 拷贝。

### 5.7 只计计算、不计读盘：能否实时

`/dev/shm` 上 100 ms mixed 前缀（无磁盘，20 窗、20/20 FCS）：

| 阶段 | 样点 / 墙钟 | 吞吐 | 相对 737.28 MS/s |
|---|---|---:|---:|
| 捕获到候选 | 1.88 M / 12.16 ms | 155 MS/s | **0.21×** |
| 捕获到身份确认 | 3.78 M / 22.05 ms | 171 MS/s | **0.23×** |
| 到 `LOCKED` | 10.6 M / 46.46 ms | 228 MS/s | **0.31×** |
| 锁定后本测试整段消费 | 63.1 M / 255 ms | 247 MS/s | 0.34× |

锁定后 247 MS/s 与历史 scheduled「默认小 chunk GR plumbing ~240–250 MS/s」一致，
不是 skip 算法上限。CORE / 大 buffer 下 scheduled 曾到 2.5–3.3 GS/s。
PDU 65/48 + QM35 demod 约 1.9 + 1.8 ms/窗，200 slot/s 只需 5 ms，余量约 2.5×。

结论：锁定后的周期截窗 + 65/48 + 解调，按计算跟得上 200 slot/s。
**捕获期跟不上 737.28 MS/s**，mixed 上约 0.21×。端到端不能宣称实时。

## 6. 对照方案完成定义

方案 §18 只有全部满足才能宣称“X410 能从盲流锁定 QM35 并自动捕获后续包”。

| # | 条款 | 本轮 |
|---:|---|---|
| 1 | 无需 `first_packet_sample` | **通过**（离线文件） |
| 2 | 首锁含 FCS-pass QM35 identity | **通过** |
| 3 | 首个确认包被输出 | **通过**（acquisition PDU） |
| 4 | 自动切到 5 ms scheduled capture | **通过** |
| 5 | 锁定后关闭全局能量 Region | **通过**（after_lock=0） |
| 6 | timing feedback 微调 native t0/T | **通过**（T +6.8 ppm，t0 持续更新） |
| 7 | collision/FCS fail 仍 EverySlot | **通过**（1 个 payload_failed 仍出窗） |
| 8 | overflow / rx discontinuity 失锁重捕获 | **未测**（文件无 UHD 中断） |
| 9 | 737/998 坐标与 MATLAB 对照 | **未做**本文件逐样本对照 |
| 10 | 20 次随机启动、60 s、10 min soak | **未测** |
| 11 | 全部 QA/CTest 通过 | **未通过**（见 §7） |

因此本轮结论应写成：**离线 mixed 737.28 上，盲 t0 + 已知 5 ms 的最小闭环已经打通。**
不得写成“X410 硬件盲捕获已完成”。

## 7. QA 与实现缺口

本机构建后定向 CTest：

| 测试 | 结果 |
|---|---|
| `uwb_qa_uwb_detector_sc16.cc` | 通过 |
| `uwb_qa_uwb_realtime_demodulator.cc` | 通过 |
| `uwb_qa_uwb_defaults.cc` | 通过 |
| `uwb_qa_uwb_auto_scheduled_extractor_sc16.cc` | **4 个用例失败** |

已知失败原因（不影响上述真实文件结论）：

- `test_native_reference_template_detects_placed_start` 把模板写死为
  `/home/oi/Desktop/uwb-gnuradio/testdata/reference_preamble_code9_737p28.cf32`；
- holdover / discontinuity / X410 overflow 转发用例未在本机环境通过。

`x410_auto_scheduled_capture.py --identity demod` 的 file 路径并未接上
`UwbRealtimeDemodulator`；真实身份闭环以 `offline_qm35_auto_lock.py` 为准。

## 8. 下一步

锁定后原生 SC16 落盘（头 300 µs、尾 100 µs）已在本文件 0.5 s 上打通（100/100
bit-exact，0 drop）。实现与复现见
[`下一步_锁定后SC16截取与DW1000头尾预留.md`](下一步_锁定后SC16截取与DW1000头尾预留.md)。

```bash
python3 testdata/offline_qm35_auto_lock.py --max-seconds=0.5 \
  --write-sc16 /tmp/qm35_sc16_dump
```

硬件验收（未完成，不能用写盘代替）：

1. 修正 QA 中的本机模板路径，并修 holdover / discontinuity 用例。
2. 把 `x410_auto_scheduled_capture.py` 的 file/x410 `--identity demod` 接到
   与离线脚本相同的 65/48 + realtime demod + `lock_obs` 闭环。
3. 对同一 mixed 文件做 MATLAB 逐包 `detected_start` / `t0` 对照。
4. X410 实机：随机启动 20 次、60 s 连续、10 min soak、人为 overflow 后重捕获。

不得在上述 4 项完成前，把本报告写成硬件生产验收通过。
