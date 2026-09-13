# UWB PDU throughput

Generated: 2026-09-13T19:07:51

Fixed parameters: preamble=128, pulse-shape=minphase, gain_tx=50.0, gain_rx=60.0, no_udp=True, pulses=1000, device=addr=192.168.10.2, tx_ch=0, rx_ch=3, git=0dbfda5 perf(radar): multi-worker 65/32 FIR for the stream chain (--res-workers)

## Per-case summary

| metric | 100hz | 200hz |
|---|---|---|
| rate (Hz) | 100.000 | 200.000 |
| pulses | 1000 | 1000 |
| schedule_wall_s | 10.242 | 5.248 |
| echo_ok | 1000 | 999 |
| echo_late | 0 | 1 |
| est_drop | 0 | 0 |
| cir_ok | 1000 | 999 |
| CIR/s | 97.634 | 190.363 |
| echo_ok/s | 97.634 | 190.363 |
| est service mean (us) | 381 | 379 |
| est service max (us) | 538 | 531 |
| est service p95 (us) | 420.000 | 435.000 |
| publisher samples | 13 | 12 |

## Echo per-burst host steps (ms)

| metric | 100hz | 200hz |
|---|---|---|
| get_time_ms.mean | 0.144 | 0.115 |
| get_time_ms.p95 | 0.179 | 0.149 |
| get_time_ms.p99 | 0.187 | 0.168 |
| get_time_ms.max | 0.710 | 0.190 |
| issue_ms.mean | 0.020 | 0.019 |
| issue_ms.p95 | 0.027 | 0.025 |
| issue_ms.p99 | 0.035 | 0.048 |
| issue_ms.max | 0.059 | 0.059 |
| send_ms.mean | 6.514 | 3.344 |
| send_ms.p95 | 9.074 | 4.032 |
| send_ms.p99 | 9.124 | 4.087 |
| send_ms.max | 9.254 | 5.333 |
| recv_ms.mean | 3.508 | 1.722 |
| recv_ms.p95 | 7.722 | 2.757 |
| recv_ms.p99 | 7.852 | 2.907 |
| recv_ms.max | 250.414 | 250.441 |
| uhd_ms.mean | 10.042 | 5.084 |
| uhd_ms.p95 | 9.906 | 4.917 |
| uhd_ms.p99 | 10.032 | 5.042 |
| uhd_ms.max | 250.899 | 250.982 |
| enqueue_ms.mean | 0.013 | 0.012 |
| enqueue_ms.p95 | 0.021 | 0.018 |
| enqueue_ms.p99 | 0.026 | 0.021 |
| enqueue_ms.max | 0.038 | 0.036 |
| sc16_ms.mean | 0.001 | 0.001 |
| sc16_ms.p95 | 0.002 | 0.002 |
| sc16_ms.p99 | 0.005 | 0.002 |
| sc16_ms.max | 0.009 | 0.005 |

## Publisher stages (ms)

| metric | 100hz | 200hz |
|---|---|---|
| pub.contig_ms.mean | 0.003 | 0.003 |
| pub.contig_ms.p95 | 0.007 | 0.007 |
| pub.contig_ms.p99 | 0.011 | 0.009 |
| pub.contig_ms.max | 0.012 | 0.010 |
| pub.tolist_ms.mean | 1.076 | 1.077 |
| pub.tolist_ms.p95 | 1.604 | 1.536 |
| pub.tolist_ms.p99 | 1.609 | 1.571 |
| pub.tolist_ms.max | 1.610 | 1.580 |
| pub.pmt_ms.mean | 0.792 | 0.818 |
| pub.pmt_ms.p95 | 1.486 | 1.560 |
| pub.pmt_ms.p99 | 2.369 | 2.352 |
| pub.pmt_ms.max | 2.590 | 2.550 |
| pub.pub_ms.mean | 0.113 | 0.113 |
| pub.pub_ms.p95 | 0.135 | 0.140 |
| pub.pub_ms.p99 | 0.145 | 0.142 |
| pub.pub_ms.max | 0.148 | 0.142 |
| pub.total_ms.mean | 1.984 | 2.012 |
| pub.total_ms.p95 | 3.166 | 3.171 |
| pub.total_ms.p99 | 4.121 | 3.994 |
| pub.total_ms.max | 4.360 | 4.200 |

## Notes

- Echo step stats are over successful bursts in `echo_timing.jsonl`.
- Publisher per-pulse lines are emitted only for `pulse_id < 3` or every 100th pulse; `pub.n` is the number of parsed lines.
- `est service` comes from `summary.json` (`est_service_us_mean/max`); percentiles come from `cir.jsonl` `estimator_us` when present.
- Resampler `resample_us` is reported only when the app or CIR meta exposes it; otherwise it is `-`.

