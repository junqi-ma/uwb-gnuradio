# QM35 timing coarse-stride 扫描

输入为 `qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat`。所有档位使用相同的
737.28→998.4 MHz `quality_minorder` 重采样、固定 `t0/T`、no-lock 调度、64 SYNC、4z2
和 CFO 24-SYNC 丢弃策略。99 个调度窗中末窗未发布，因此每档有 98 条结果；漏检率的
分母统一为 99 个调度窗。

| coarse stride | timing failed | 漏检率 | FCS pass / 98 | timing 中位时间 (µs) |
|---:|---:|---:|---:|---:|
| 1 | 0 | 0.0% | 97 | 19170 |
| 2 | 0 | 0.0% | 97 | 9804 |
| 3 | 0 | 0.0% | 97 | 6932 |
| 4 | 0 | 0.0% | 97 | 5307 |
| 5 | 0 | 0.0% | 97 | 4498 |
| 6 | 0 | 0.0% | 97 | 3865 |
| 7 | 0 | 0.0% | 97 | 3605 |
| 8 | 17 | 17.2% | 80 | 3065 |
| 10 | 0 | 0.0% | 97 | 2732 |
| 12 | 0 | 0.0% | 97 | 2422 |
| 13 | 0 | 0.0% | 97 | 2368 |
| 14 | 0 | 0.0% | 97 | 2154 |
| 15 | 0 | 0.0% | 97 | 2167 |
| 16（旧默认） | 13 | 13.1% | 84 | 2078 |
| 24 | 15 | 15.2% | 82 | 1693 |
| 32 | 11 | 11.1% | 86 | 1512 |

结果表明漏检率并不随 stride 单调变化：粗采样相位会与窄相关峰发生确定性混叠。
在这份捕获上，`stride=14` 是已测的零漏检、最低中位 timing 耗时配置，仅比 `stride=16`
慢约 3.7%。这只是单捕获结论；在改默认值前，应使用另一段数据验证 `14` 的零漏检。

每档逐 slot 结果保存在 `stride_<N>.csv`。复跑示例：

```bash
PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib \
  python3 testdata/offline_qm35_gr_demod.py --workers=4 --no-lock \
  --timing-coarse-stride=14 \
  --output-csv=testdata/qm35_timing_stride_sweep/stride_14.csv
```
