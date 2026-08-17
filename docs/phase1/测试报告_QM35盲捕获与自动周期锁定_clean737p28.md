# 测试报告：QM35 盲捕获与自动周期锁定（无干扰 737.28）

> 日期：2026-08-15
> 流程与 mixed 相同：[`测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md`](测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md)
> 主线入口：[`../../开发状态.md`](../../开发状态.md)

## 1. 结论

在无 DW1000 的 X410 QM35 捕获上，**不指定 `t0`**，同一条
`UwbAutoScheduledExtractorSc16` → 原生 SC16 dump → 逐窗消单音覆盖
`capture.iq` 路径能锁定并落盘。

这仍是离线 0.5 s 文件闭环，不是 X410 硬件 / soak / overflow 验收。

| 项 | 结果 |
|---|---|
| 是否需要用户提供 t0 | **否** |
| 首锁依据 | code-9 候选 + 解调 `success` + **FCS pass** |
| 最终状态 | `locked`，`identity_confirmed=true` |
| 0.5 s 窗口 | 1 acquisition + 98 scheduled/provisional 写出 |
| extractor drop | **1**（最后一个 300/190/100 µs 窗超出文件末尾） |
| writer drop | **0** |
| 消单音前与源文件 | **99/99 bit-exact** |
| 解调 | live 与 dump 再解均为 **98/99 FCS**，id=24 `payload_failed` |
| `capture.iq` | 只保留去单音后的文件，无 `capture_raw.iq` / `capture_notch.iq` |

## 2. 测试条件

| 项 | 值 |
|---|---|
| 输入 | `/mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat` |
| 场景 | metadata：`QM35`（无 DW1000） |
| 格式 | SC16 interleaved int16，737.28 MS/s，6489.6 MHz，0.5 s |
| 样点数 | 368,640,000 complex |
| `first_packet_sample` | **未提供** |
| dump 窗 | 头 300 µs / 体 190 µs / 尾 100 µs |
| 脚本 | `testdata/offline_qm35_auto_lock.py --max-seconds=0.5 --write-sc16 DIR` |

## 3. 锁定与落盘

```text
file_source SC16 @737.28
  -> UwbAutoScheduledExtractorSc16   # 不提供 first_packet_sample
  -> PDU 65/48 quality_minorder
  -> UwbRealtimeDemodulator (code 9, 64 SYNC, 4z2, bypass)
  -> schedule_feedback -> lock_obs
并行：extractor packet -> UwbPacketWriter
之后：cancel_capture_tone.process_dump 覆盖 capture.iq
```

| 项目 | 值 |
|---|---|
| acquisition `start` | 3543362（约 4.804 ms），metric 0.807 |
| 身份确认墙钟 | 0.20 s |
| 锁定 T | 0.005000069 s（3,686,451 native samples，约 +13.8 ppm） |
| 能量 Region | 捕获期 1，锁定后 0 |
| 写出几何 | acq `2032/200000/0`；provisional `239616/140083/92160`；locked `221184/140083/73728` |
| payload 前缀（FCS 过的窗） | `2D00003200000000090004E8` |

末窗 drop 是预期：首包比 mixed 晚约 2.5 ms，k=99 的窗尾超过 368,640,000。

## 4. 去单音

与 mixed 同一 CW：基带 −232.960 MHz，RF **6256.640 MHz**。
无干扰窗里该单音只占总功率约 0.01 dB，但 tone bin 仍被压掉（约 319 dB）。
写出后直接 `os.replace` 覆盖 `capture.iq`，不保留未处理副本。

## 5. 落盘位置与复现

```text
F:\UWB基带数据\qm35_clean_scheduled_sc16_dump\
  capture.iq
  capture.jsonl
  capture_notch_report.json
  scheduled_dump_cpp.csv
  scheduled_dump_cpp_summary.json

F:\USRP数据解调\decoded_results\scheduled_sc16_dump_qm35_clean\
  scheduled_dump_cpp.csv
  scheduled_dump_cpp_summary.json
```

```bash
PYTHONPATH=$PWD/gr-uwb/build/test_modules \
LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/offline_qm35_auto_lock.py --max-seconds=0.5 \
    --write-sc16 /tmp/qm35_clean_sc16_dump \
    /mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat

python3 testdata/decode_scheduled_sc16_dump.py /tmp/qm35_clean_sc16_dump \
  --csv /tmp/qm35_clean_sc16_dump/scheduled_dump_cpp.csv
```

MATLAB：`run_decode_scheduled_sc16_dump.m`，`dumpDir` 填上面的 dump 目录。
