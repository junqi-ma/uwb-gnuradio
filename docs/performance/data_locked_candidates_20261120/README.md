# Locked 模式候选窗口数据（2026-11-20）

对应报告：
[../Locked模式候选窗口与目标PRR分析_2026-11-20.md](../Locked模式候选窗口与目标PRR分析_2026-11-20.md)

## 来源

用户提供的 live 运行：

```sh
env -u LD_LIBRARY_PATH gr-uwb/build-native-only/apps/uwb_dw3000_live \
  --args addr=192.168.10.2 --period-ms 1 --workers 2 \
  --print summary --summary 1000 --sync locked
```

- 设备：X410，channel 3/RX1，6489.6 MHz，gain 60
- 时长：10,721,165,312 / 737,280,000 ≈ 14.54 s
- 期望包数：约 14,534（≈ 1000 包/s）
- 配置：`eq16`、fused on、interference on、lock_fcs on

## 文件

- `live-locked-2w-key.log`：从终端输出整理的关键行（START、LOCK、PRE_GUARD、
  全部 SUMMARY、COST_TOTAL、LOCK_STATS、FINAL）。不是逐包日志。
- `analyze_locked.py`：解析脚本。
- `summary.txt`、`summary.json`：本次解析结果。

## 复现

```sh
python3 analyze_locked.py live-locked-2w-key.log
```

输出摘要：

```text
acq_seeds=14920 candidates=14912 target=12220 unknown=2692 admitted=2699
target: 14534/14534 = 100.00%
aggregate: 15003/17234 = 87.05%
extra candidate windows≈2700, candidate_ok=469, candidate_fail≈2231
failures=2231: TIMING_FAILED=1827, SFD_FAILED=402,
                INCOMPLETE_WINDOW=1, PAYLOAD_FAILED=1
```

## 说明

- `SUMMARY`/`FINAL` 的 `fcs/completed` 统计的是**所有解调尝试**，
  包含约 2700 个未知 autocorr 候选窗口，不能当作目标包 PRR。
- 目标口径看 `LOCK_STATS expected_slots / fcs_ok`。
- “华硕路由器”是待验证的外部干扰假设，当前日志没有 Wi-Fi 功率或频谱数据，
  不能据此下结论；验证方法见报告第 6 节。
