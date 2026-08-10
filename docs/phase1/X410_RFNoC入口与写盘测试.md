# X410 RFNoC 入口与持续写盘测试

## 采样率边界

当前 X410 FPGA radio 输出为 737.28 MS/s，而 UWB MATLAB/reference pipeline
使用 998.4 MS/s。二者为精确有理数关系：

```text
737.28 MS/s × 65 / 48 = 998.4 MS/s
```

生产路径采用：

```text
X410 Radio 737.28M
→ RFNoC custom Upsampler 65/48
→ RFNoC RxStreamer fc32 998.4M
→ UwbScheduledExtractor
→ UwbPacketWriter (SC16)
```

禁止在主机 GNU Radio 中对 737.28 MS/s 连续流做 65/48 重采样。入口脚本
只接受 RFNoC 上采样后的 998.4 MS/s 语义，RFNoC block ID 由
`--upsampler-block` 指定，默认 `0/Upsampler#0`。

## 实机入口

```bash
PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib \
python3 gr-uwb/apps/x410_scheduled_capture.py \
  --args 'addr=192.168.10.2' \
  --frequency 6.5e9 --gain 20 --antenna RX2 \
  --upsampler-block 0/Upsampler#0 \
  --first-packet-sample 10000000 \
  --packet-interval 0.005 \
  --output /data/uwb_capture
```

首次接机先增加 `--dry-run` 检查参数。实际运行前用 `uhd_usrp_probe` 确认
FPGA image 中的 block ID、端口号和 Radio 实例。脚本会检查 Radio rate 未被
UHD coercion，并打印完整速率契约。

当前入口以 sample-domain `t0` 启动；UHD `rx_time` discontinuity/overflow
重映射仍是下一阶段，未完成前发生 overflow 必须停止采集并重新 seed，不能
继续信任旧 schedule。

也可直接打开 `gr-uwb/examples/x410_rfnoc_uwb_scheduled.grc` 组合或修改。
另有通用 detector 和 scheduled file replay 示例，说明见
`gr-uwb/examples/README.md`。

## 30 秒持续写盘仿真

测试不生成 30 秒全速原始 IQ，而以真实时间节拍向 Writer 发送 scheduled
window PDU；因此磁盘负载与生产路径一致。

```bash
PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib \
python3 gr-uwb/apps/simulate_scheduled_write.py \
  --output /tmp/uwb-write-30s \
  --duration 30 --slot-rate 200 --samples 203776
```

2026-08-09 本机结果：

| 项目 | 结果 |
|---|---:|
| 逻辑/墙钟时长 | 30 s |
| 窗口 | 6,000 |
| JSONL 行 | 6,000 |
| SC16 字节 | 4,890,624,000 |
| 平均磁盘数据率 | 163.02 MB/s |
| 数据/metadata 完整性 | 通过 |

脚本退出码非零表示 packet 数、drop、文件大小或 JSONL 行数任一不匹配。
