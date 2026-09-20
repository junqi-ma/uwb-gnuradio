# Jamming运行 estimator queue_full 丢包分析（供 Codex 深查）

日期：2026-09-20
来源：用户粘贴的 `x410_cg400_hrp_echo_cir_jam.py` 复用 `base.start_live_stats` 的 live 日志。
`jam` 本体没有自己的 live 循环，调用点见 `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py:1386`，格式定义在 `gr-uwb/apps/x410_cg400_hrp_echo_cir.py:438`。

## 1. 原始日志（逐行保留）

```text
live dt=1.000 echo_ok_hz=152.0 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=2408 tx_err=0 wr_hz=1236.8 udp_hz=1276.7 udp_ok_hz=1236.8 udp_eagain=0 service_us_mean=101287 max=107878 pri_hz=200.0
NOTE estimator dropped 142 frames (queue_full); radio ok is not CIR/UDP ok
live dt=1.000 echo_ok_hz=152.0 cir_ok_hz=9.0 cir_fail_hz=0.0 est_q=64 est_drop=2551 tx_err=0 wr_hz=1215.8 udp_hz=1254.8 udp_ok_hz=1215.8 udp_eagain=0 service_us_mean=101338 max=107878 pri_hz=200.0
NOTE estimator dropped 143 frames (queue_full); radio ok is not CIR/UDP ok
live dt=1.000 echo_ok_hz=152.0 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=2693 tx_err=0 wr_hz=1223.8 udp_hz=1260.7 udp_ok_hz=1223.8 udp_eagain=0 service_us_mean=101341 max=107878 pri_hz=200.0
NOTE estimator dropped 142 frames (queue_full); radio ok is not CIR/UDP ok
backlog res_in=2953 res_out=2952 est_in=2952 est_done=195 est_drop=2693 est_q=64 wr=24164 echo_pub=2953
live dt=1.000 echo_ok_hz=152.0 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=2835 tx_err=0 wr_hz=1221.7 udp_hz=1261.7 udp_ok_hz=1221.7 udp_eagain=0 service_us_mean=101334 max=107878 pri_hz=200.0
NOTE estimator dropped 142 frames (queue_full); radio ok is not CIR/UDP ok
live dt=1.000 echo_ok_hz=135.0 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=2961 tx_err=0 wr_hz=1233.8 udp_hz=1273.8 udp_ok_hz=1233.8 udp_eagain=0 service_us_mean=101299 max=107878 pri_hz=200.0
NOTE estimator dropped 126 frames (queue_full); radio ok is not CIR/UDP ok
live dt=1.000 echo_ok_hz=149.9 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=3101 tx_err=0 wr_hz=1218.5 udp_hz=1258.5 udp_ok_hz=1218.5 udp_eagain=0 service_us_mean=101332 max=107878 pri_hz=200.0
NOTE estimator dropped 140 frames (queue_full); radio ok is not CIR/UDP ok
live dt=1.001 echo_ok_hz=151.9 cir_ok_hz=10.0 cir_fail_hz=0.0 est_q=64 est_drop=3242 tx_err=0 wr_hz=1208.3 udp_hz=1247.3 udp_ok_hz=1207.3 udp_eagain=0 service_us_mean=101372 max=107878 pri_hz=200.0
NOTE estimator dropped 141 frames (queue_full); radio ok is not CIR/UDP ok
```

## 2. 字段定义（代码位置）

* `live ...` 打印：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py:438-456`
  * `echo_ok_hz = delta(echo._ok)/dt`，radio TX/RX 成功脉冲速率。
  * `cir_ok_hz = delta(est.pdus_completed())/dt`，estimator 核心状态 Ok 速率。
  * `cir_fail_hz = delta(est.pdus_failed())/dt`，核心状态非 Ok 但仍 publish 的速率。
  * `est_q = est.queue_depth()`，当前队列深度。
  * `est_drop = est.pdus_dropped()`，累计 queue-full 丢弃（单调增）。
  * `wr_hz = delta(wr.frames_written())/dt`，CIR writer 成功帧速率（注意 repetitions 模式下 1 脉冲 = N 记录）。
  * `udp_hz / udp_ok_hz / udp_eagain = udp.sent / udp.sent_ok / udp.dropped`。
  * `service_us_mean/max = est.service_mean_us()/service_max_us()`，estimator `radar_cir_one + per-repetition + publish_frame` 耗时统计。
  * `pri_hz = 1/pri_s`，需求脉冲速率。
* `NOTE estimator dropped ... (queue_full)`：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py:457-460`，`est_drop` 差量 >0 才打印。
* `backlog ...` 每 5s：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py:464-471`
  * `res_in/out = res.pdus_received()/pdus_emitted()`，PDU resampler 进/出。
  * `est_in/done = est.pdus_received()/pdus_completed()`。
  * `wr = wr.frames_written()` 累计，`echo_pub = echo._pub_ok` 累计。
* estimator 计数器语义：`gr-uwb/include/gnuradio/uwb/uwb_radar_cir_estimator_block.h:134-143`
  * `pdus_dropped() // queue full`，`pdus_completed() // core ok`，`pdus_failed() // core not ok`。
* worker 模型：`gr-uwb/lib/uwb_radar_cir_estimator_block.cc:710-808`
  * 单 worker 线程 `worker_loop`，`radar_cir_one` + `emit_individual_repetitions` 循环 + `publish_frame` 都计入 `service_us`（`:805-806`）。
* jam 侧构造：
  * `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py:1334-1342`，`est_q = max(8, --est-queue)`，`radar_cir_estimator(..., est_q, use_pred, cir_output=="repetitions")`。
  * `cir_records_per_pulse = sync_reps - skip` 见 `gr-uwb/apps/x410_cg400_hrp_echo_cir.py:1789-1790`。
* 默认参数：
  * `--est-queue default=64`：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py:1386`。
  * `--cir-output default="repetitions"`：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py:1246,1389`。
  * `--preamble-length default=64`：`:128`；`--cir-pre 16 --cir-post 100`（116 taps）：`:211-213`。

## 3. 定量推导

1. 需求 `pri_hz=200` 即 `pri_s=5ms`。`echo_ok~135-152Hz` 已低于需求约 24%，但 `tx_err=0`，先排除 TX send 错误；剩余缺口需查 `echo._fail/_late`（本次日志没打印）。
2. 守恒关系成立：`echo_ok - cir_ok ≈ est_drop增量`。
   * 例 `152.0 - 10.0 = 142.0`，`NOTE ... 142 frames` 完全对上；`152.0-9.0=143` 对第二行 `143`。
3. backlog 守恒：`est_done(195) + est_drop(2693) + est_q(64) = 2952 = est_in(2952)`。
   * `res_in 2953 ≈ res_out 2952 ≈ est_in 2952 ≈ echo_pub 2953`，证明 resampler 前级正常，瓶颈在 estimator 之后。
   * 处理率仅 `195/2952 ≈ 6.6%`。
4. 服务时间决定吞吐上限：`service_mean ~101.3ms`，单 worker 上限 `1/0.101 ≈ 9.9Hz`，和观测 `cir_ok 9-10Hz` 吻合。
   * 对比基线：`analysis_outputs/m0_baseline` 及 `cpp_pdu_ab` 系列 `est_service_us_mean ~360-470us`；`logs/x410_jam_clip_check_20260919/summary.json:27` 为 `639us`。当前 101ms 约为基线 250 倍。
   * `mean 101ms / max 107ms` 接近，说明是确定性单包成本，不是偶发毛刺。
5. `wr/udp ~1200Hz` 是扇出假象：`repetitions` 模式下每个 `cir_ok` 发布 N 条（默认 `sync_reps=64`，若 `--preamble-length 128` 则 N=128；观测 `wr/cir ≈ 122`，需结合本次实际命令行确认）。不能用 `wr_hz` 高来证明系统健康。
6. 健康信号：`cir_fail=0`（算法状态机没报失败）、`udp_eagain=0`（UDP 非阻塞发送没丢）、`tx_err=0`。

## 4. 结论（已确认部分）

* 是丢包，且是持续性过载丢包：生产者 `~150Hz` >> 消费者 `~10Hz`，队列常满 `est_q=64`，约 93% 脉冲在 estimator 入口 `queue_full` 丢弃。
* 不是射频丢包、不是 UDP 丢包、不是 CIR 算法 `SfdFailed/CirFailed`。
* 瓶颈是 estimator 单包服务时间 101ms 远超 5ms PRI 预算。

## 5. 未确认、请 Codex 深查

1. 本次 jam 实际命令行是什么？（`--preamble-length/--sync-reps`、`--cir-output`、`--cir-pre/--cir-post`、`--publish-native`、`--sfd-search-margin/--sync-refine-margin`、`--res-workers`、`--echo-backend`、`--pulses/--pri-s/--duration-s`）。`wr/cir≈122` 暗示可能用了 128 preamble 或 jam 双波形加长，需核对。
2. 为什么 `service_us` 从 0.4-0.6ms 膨胀到 101ms？
   * `publish_native` 是否为 0（全 RX 窗）导致 `radar_cir_one` 输入 `n` 巨大？jam 双 TX 波形是否加长了 RX 窗？
   * `emit_individual_repetitions=true` 时 `publish_frame × N` 的 PMT/`message_port_pub`/GIL 开销占比多少？`service_us` 包含 publish（`:774-806`），需拆分 `radar_cir_one` vs repetition loop vs publish。
   * 是否开了 `--cir-output repetitions` + 大 `cir_pre/post` + 大 `sfd_search_margin` 组合导致逐 repetition 全相关？
   * CPU 争用：`res_workers`、UHD 收发、writer、UDP 是否和 estimator 抢核？`service_max` 是否含调度延迟？
   * 构建类型：Release vs Debug？是否有 ASan/日志/MAT 文件写入拖慢 worker？
3. `echo_ok 152Hz < pri 200Hz` 的缺口是另一回事：查 `echo._fail/_late`、UHD `U/O` 溢出、`tx_lead/min-lead-s`、jam `freq_settle/retune` 是否吃掉了 25% 脉冲。
4. `udp_hz - udp_ok_hz ≈ 30-40Hz` 差值（例 `1247.3 vs 1207.3`）在 `udp_eagain=0` 下是什么？是 `sent` vs `sent_ok` 定义差异还是 UDP 帧校验失败？需看 `CirUdpSink` 计数器。
5. `backlog wr=24164` 与 `est_done×records_per_pulse` 是否对得上？若对不上，writer 是否还收了别的源？

## 6. 建议 Codex 复现/取证清单

* 粘贴完整启动命令行 + `summary.json`（`est_rx/enqueued/done/fail/drop/invalid`、`est_service_us_mean/max`、`res_rx/tx/drop`、`wr_ok/fail`、`cir` 统计）。
* 跑对照：同一机器 `--cir-output average` vs `repetitions`，`--pulses` 小批量，看 `service_us` 是否从 101ms 回落到 <1ms。
* 加拆分计时：`radar_cir_one`、repetition fanout、每 `publish_frame` 三段分别打点；或先用 `bench_pdu_throughput.py` 离线压测 estimator。
* 录 `top/htop` 或 `perf`：estimator worker 单核是否 100%，resampler workers 是否抢核。
* 确认 `est_queue` 不要调大掩盖问题（help 原话：`Do not set this to pulses — that hides lag as 'no loss'`）。

## 7. Codex 复核、整改与实机结果（2026-09-20）

### 7.1 对原分析的判断

队列守恒、约 10 Hz 的单 worker 吞吐上限及 queue-full 是直接丢包点的判断均正确。
需要修正的是：原日志的 `service_us` 不只包含 `radar_cir_one`，还包含 128 次
逐 repetition 相关、PMT 字典/向量构造及 `message_port_pub`；因此 101 ms 不能
归因于一次平均 CIR 核心计算。Release 离线基准中旧平均 CIR 约 1.8--2.0 ms。

### 7.2 根因与代码整改

1. `estimate_radar_cir_repetition` 原先每个 tap 扫描全部 1016 个 sampled-code
   样点，再分支跳过绝大多数零值。现改为 prepare 阶段预计算 64 个非零索引和值；
   worker 热路径不分配内存，累加次序和 MATLAB/旧实现数值保持一致。
2. 原实现为每个 repetition 重建约 30 项公共 PMT metadata 并发布一个 PDU。
   新增可选 packed repetition PDU：estimator 每 pulse 发布一个内部 batch，writer
   仍落成 128 个有序 FC32 CIR record + 每 pulse 一行 columnar JSON，UDP sink 仍
   扇出 128 个 UCR4 datagram；外部文件/UDP 格式不变。base/jam 实时 app 的
   `repetitions` 模式启用 batch，其他依赖逐 PDU 的 sweep/align 路径保持旧接口。
3. 实机首轮发现 auto ROI 仍按平均模式 `cir_skip=10` 算长度，导致每拍最后 4 个
   repetition 固定 `cir_failed`。repetition 模式现显式用 `cir_skip=0` 计算
   `publish_native`（737.28 MS/s 本轮由 96032 修正为 103535）。

### 7.3 验证结果

- 离线：逐 repetition 均值对平均 CIR、逐 tap bit-exact、writer binary/JSON
  布局、UDP batch 顺序/状态、100 pulse × 200 Hz paced replay 均通过；paced
  replay `est_drop=0`，服务时间 `<5 ms`。
- X410/CG600/DPDK 双 TX，128 SYNC，200 Hz，jam offset 491.34 kHz，UDP 开启，
  3 s：`echo_ok=600/600`、`echo_late=0`、`res_drop=0`、`est_done=600/600`、
  `est_drop=0`、`est_q=0`、`est_service_us_mean=3536`、`max=4453`；writer
  `76800/76800 ok`、UDP `76800/76800 ok`、`udp_eagain=0`，进程退出码 0。
- 安装 Release 后同配置 60 s：实际完成 12000 pulse、1536000 repetition，
  `res_drop=0`、`est_done=12000/12000`、`est_fail=0`、`est_drop=0`，writer 与
  UDP 均 `1536000/1536000 ok`；estimator mean/max 3.452/6.133 ms。调度线程有
  `echo_late=300`，导致 pulse ID 跨过 300 个编号（实际采集数仍为 12000），因此
  应用完整性硬门槛退出码为 1；这是既有宿主实时调度问题，不是 estimator 丢包。
  结果：`/tmp/x410_jam_rep_batch_udp_200hz_60s_20260920/summary.json`。
- 同配置 no-UDP 5 s：1000/1000 estimator 完成、128000/128000 repetition ok、
  `est_drop=0`；该轮有既知且独立的 `echo_late=20`，不得据此宣称 late/soak 验收。

结论：本文原始日志中的 estimator queue-full 丢包已在 X410 200 Hz、60 s 实测消除；
整体实时链仍未通过 `echo_late=0` 硬门槛，且尚未执行 10 min soak 或 300 Hz 多轮验收。
