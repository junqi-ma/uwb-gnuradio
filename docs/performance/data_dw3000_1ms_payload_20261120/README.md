# DW3000 1 ms 实板采集与 payload 失败分析数据（2026-11-20）

对应报告：
[../DW3000_1ms实板采集_Payload失败分析_2026-11-20.md](../DW3000_1ms实板采集_Payload失败分析_2026-11-20.md)

## 采集

原始 SC16 文件未入库（2.75 GiB）：

```text
/tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16
size    : 2,949,120,000 bytes (737,280,000 complex int16)
sha256  : 66b2a0d51669305d828548c8e3fb32d9e475c5a22fa7c2f2fe8220f519b4bfb4
```

采集命令（`uwb_uhd_sc16_capture`，本地新增 `--uhd-antenna` 选项）：

```sh
uwb_uhd_sc16_capture --out /tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16 \
  --ms 1000 --settle-ms 2 --uhd-args addr=192.168.10.2 \
  --uhd-channel 3 --uhd-antenna RX1 --freq 6489.6e6 --gain 60 \
  --native-rate 737.28e6 --no-detect
```

采集观测：wall 1002.183 ms，737.15 MS/s，overflows=0，timeouts=0，
mean power 1.452e-4，max |x| 0.1411。

## 日志

`logs/*.log.gz` 为 `uwb_dw3000_live --print packets` 的原始输出。除特别注明外，
公共参数为 `--replay dw3000_1ms_1s.sc16 --period-ms 1 --workers 1 --seconds 2`。

| 文件 | 配置 | 用途 |
| --- | --- | --- |
| `offline-w1.log.gz` | 默认（`eq16`、sparse timing、acquire、fused on） | 基线 |
| `var-eq32.log.gz` | `--taps eq32` | 重采样滤波器对照 |
| `var-quality.log.gz` | `--taps quality` | 重采样滤波器对照 |
| `var-nospatial.log.gz` | `--timing-sparse off` | 稠密 timing 模板对照 |
| `var-locked.log.gz` | `--sync locked` | 锁定网格对照 |
| `var-cfoauto.log.gz` | `--cfo-refit auto` | CFO 二次重估对照 |
| `var-cfoskip.log.gz` | `--cfo-refit skip` | CFO 二次重估对照 |
| `var-ondemandoff.log.gz` | `--ondemand off` | on-demand 前缀对照 |
| `var-eq32legacy.log.gz` | `--taps eq32 --fused off --workers 4 --queue 256` | 隔离：legacy 解码 + 正确周期估计 |
| `var-nospatiallegacy.log.gz` | `--timing-sparse off --fused off --workers 4 --queue 256` | 隔离：legacy 解码 + 稠密 timing |
| `var-legacy-acq.log.gz` | `--fused off --workers 4 --queue 256` | 隔离：legacy 解码 + acquire 周期估计 |
| `var-legacy-locked.log.gz` | `--sync locked --fused off --workers 4 --queue 256` | 隔离：legacy 解码 + 锁定周期估计 |
| `var-fusedoff.log.gz` | `--fused off`，1 worker | 仅说明 legacy 全流在 1 ms 下会队列溢出，不作成功率对比 |

`w1/w2/r2-w1/r2-w2.log.gz` 是同一设备 1 ms 周期下 15 s 的 live 单/双 worker 记录，
不属于本次 1 s 文件回放统计，保留作上下文。

## 复现

```sh
python3 analyze_failures.py logs/offline-w1.log.gz
python3 analyze_failures.py logs/var-*.log.gz --json summary_variants.json
```

脚本只解析 `PACKET` 行；成功定义为 `status=OK`、`fcs=1`、`crc=1`，
并区分 `path=target` 与采集/候选窗口。

`summary_baseline.json` 与 `summary_variants.json` 是本次报告使用的解析结果；
`baseline_summary.txt`、`variants_summary.txt` 是脚本的文本输出。
