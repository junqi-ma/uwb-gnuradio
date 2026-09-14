# UWB Radar PDU 吞吐优化 — M0 baseline（SA4）

本目录保存 M0（python backend）基线：`bench_pdu_throughput.py` 的解析结果、
X410 100/200 Hz 实测日志与 A/B 对比表。不放原始大 IQ。

## 1. Git commit

```
0dbfda5089cf33e0a124db1de95dc1e4abda3f9e
0dbfda5 perf(radar): multi-worker 65/32 FIR for the stream chain (--res-workers)
```

## 2. 运行环境（每个 shell 都要）

```bash
cd /home/oi/Desktop/uwb-gnuradio
export PYTHONPATH=/usr/local/lib/python3.10/site-packages:/tmp/opencode/uwbshim:$PWD/gr-uwb/build/test_modules
export LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib
```

旧的 `/tmp/uhd_eal_noret` AVX-512 补丁库**已废弃**：本机 DPDK 库与 libuhd
都已按 AVX2 重编，`bootstrap_uhd_env()` 不再加载它，见
[`../docs/DPDK_X410_CG600启用.md`](../docs/DPDK_X410_CG600启用.md)。

设备：kernel UDP `addr=192.168.10.2`，TX/RX0 ch0，RX1 ch3，`--no-udp`；要更高
TX 吞吐可加 `--use-dpdk --mgmt-addr 192.168.20.133`（需 root）。同一时刻只
允许一个进程使用 X410。

## 3. M0 baseline 命令

一键（脚本内部串行调用 app，每个 rate 一个进程）：

```bash
python3 gr-uwb/apps/bench_pdu_throughput.py \
    --run --rates 100,200 --pulses 1000 \
    --out-root analysis_outputs/m0_baseline
```

脚本实际 spawn 的两个子进程（固定 preamble 128 / minphase / gain 50-60 /
`--no-udp` / 相同 `--pulses 1000`，`--pri-s` 决定 rate）：

```text
/usr/bin/python3 .../gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --args addr=192.168.10.2 \
  --output .../analysis_outputs/m0_baseline/100hz \
  --pri-s 0.01 --pulses 1000 --preamble-length 128 --pulse-shape minphase \
  --gain-tx 50.0 --gain-rx 60.0 --tx-channel 0 --rx-channel 3 \
  --tx-antenna TX/RX0 --rx-antenna RX1 --cal-delay-native 334.0 \
  --timing-detail --no-udp

/usr/bin/python3 .../gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --args addr=192.168.10.2 \
  --output .../analysis_outputs/m0_baseline/200hz \
  --pri-s 0.005 --pulses 1000 --preamble-length 128 --pulse-shape minphase \
  --gain-tx 50.0 --gain-rx 60.0 --tx-channel 0 --rx-channel 3 \
  --tx-antenna TX/RX0 --rx-antenna RX1 --cal-delay-native 334.0 \
  --timing-detail --no-udp
```

退出码：100 Hz = 0（全 1000 完成），200 Hz = 3（app 在「并非每个计划 pulse
都完成」时返回 3，本例因 1 个 late）。

## 4. Baseline 结果

来源：`analysis_outputs/m0_baseline/bench_summary.{md,json}`（本次实测）。

| metric | 100hz | 200hz |
|---|---|---|
| rate (Hz) | 100 | 200 |
| pulses | 1000 | 1000 |
| schedule_wall_s | 10.242 | 5.248 |
| echo_ok | 1000 | 999 |
| echo_late | 0 | 1 |
| est_drop | 0 | 0 |
| cir_ok | 1000 | 999 |
| CIR/s | 97.63 | 190.36 |
| est service mean (us) | 381 | 379 |
| est service max (us) | 538 | 531 |
| est service p95 (us) | 420 | 435 |

Echo 每 burst 主线程步骤（成功 burst，ms）：

| metric | 100hz mean | 100hz p99 | 200hz mean | 200hz p99 |
|---|---|---|---|---|
| get_time_ms | 0.144 | 0.187 | 0.115 | 0.168 |
| issue_ms | 0.020 | 0.035 | 0.019 | 0.048 |
| send_ms | 6.514 | 9.124 | 3.344 | 4.087 |
| recv_ms | 3.508 | 7.852 | 1.722 | 2.907 |
| uhd_ms | 10.042 | 10.032 | 5.084 | 5.042 |
| enqueue_ms | 0.013 | 0.026 | 0.012 | 0.021 |
| sc16_ms | 0.001 | 0.005 | 0.001 | 0.002 |

Publisher 各阶段（来自 stdout `pub pulse=` 行；只对 `pulse_id<3` 与每 100 个
pulse 打印，故 100 Hz n=13 / 200 Hz n=12，非全样本）：

| metric | 100hz mean | 100hz max | 200hz mean | 200hz max |
|---|---|---|---|---|
| contig_ms | 0.003 | 0.012 | 0.003 | 0.010 |
| tolist_ms | 1.076 | 1.610 | 1.077 | 1.580 |
| pmt_ms | 0.792 | 2.590 | 0.818 | 2.550 |
| pub_ms | 0.113 | 0.148 | 0.113 | 0.142 |
| total_ms | 1.984 | 4.360 | 2.012 | 4.200 |

## 5. 设备现象与风险

- 100 Hz：1000/1000 ok，0 late，无 est_drop，1 s 内 est_q=0，服务时间
  均值 ~381 us，无 UFC/UHD 报错。
- 200 Hz：999/1000 ok，1 个 late（pulse 1，`lead_min=-0.000507 s`），
  stdout 出现 UHD RX overflow 标记 `U`；无 est_drop、无 tx_send_error。
  说明 200 Hz 已处于 python backend 的临界：单 burst 主线程 `uhd_ms` 均值
  5.08 ms 已接近 PRI 5 ms。
- 两个 rate 的 `recv_ms.max`/`uhd_ms.max` ≈ 250 ms 是 pulse 0 的固定
  `--arm-delay-s=0.25` 造成，不是稳态异常。
- **resampler 服务时间当前不可观测**：app 的 `summary.json` 未查询
  `resample_total_us()/resample_max_us()`，CIR meta 也未透传 `resample_us`，
  故表中为 `-`。脚本已支持从 summary 键
  (`res_service_us_mean` 等) 与 CIR 每帧 `resample_us` 自动解析，等主 agent
  把 resampler 统计接入 app 后可直接出数。

## 6. 文件清单

```
analysis_outputs/
  README.md                       # 本文件
  m0_baseline/
    run_meta.json                 # 精确命令 / commit / 参数 / rc
    bench_summary.md              # A/B 对比表（Markdown）
    bench_summary.json            # 同上（JSON，含每个 case 的完整统计）
    100hz/  summary.json  echo_timing.jsonl  cir.jsonl  run.json
    100hz.stdout.log
    200hz/  summary.json  echo_timing.jsonl  cir.jsonl  run.json
    200hz.stdout.log
```

原始 `cir.cf32` / `cir_norm.cf32` / `sync_template_live.cf32` 已删除，只保留
JSON/文本摘要。

## 7. 无硬件复现（--parse-only）

`--out-dir` 可以是单次 run 目录，也可以是包含多个 run 子目录的 tag 目录：

```bash
python3 gr-uwb/apps/bench_pdu_throughput.py --parse-only \
    --out-dir analysis_outputs/m0_baseline
```

脚本输出 `bench_summary.md` + `bench_summary.json`，统计每项
count/mean/p50/p95/p99/max；stdout 中的 `pub pulse=` 行即使与 `sched` 行粘连
也能被正确解析。默认 `--run` rates 为 `100,200,500`，可复现 spec §D 的三档。

## 整改轮补充证据（round 2）

- `m0_baseline_500hz/`、`m0_baseline_1000hz/`：**基线 commit `0dbfda5`** 的
  Python PDU 压力测试（worktree），含 `run.json`（argv/commit/设备/环境/hash）。
- `cpp_pdu_ab/round2/`：整改后 cpp-pdu 的 200 Hz scalecheck / 60 s soak /
  **10 min soak** 与 **3 轮 Python/C++ A/B**，每个 case 含 `summary.json` +
  `stdout.log` + `run.json`（backlog 统计）。
- `pdu_copy_audit.md`：M4 大数组复制审计。
- 结论见 `docs/phase1/测试报告_UWB_Radar_PDU速率优化.md`。
