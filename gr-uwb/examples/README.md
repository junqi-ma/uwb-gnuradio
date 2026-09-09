# UWB GNU Radio Companion flowgraphs

## `uwb_detector_to_writer.grc`

通用未知时刻通信包路径：CF32 File Source → UWB Detector → Packet Writer。
默认测试文件末尾没有静默样本，因此图中 Stream Mux 自动追加 5,000 个零样本，
保证最后一个能量 Region 正常关闭。换输入文件时同步修改 `input_samples`，或删除
Mux 并确保实时源持续提供后续样本。

## `uwb_scheduled_file_capture.grc`

已知 `t0/T` 的文件回放路径：CF32 File Source → Scheduled Extractor → Writer。
默认使用 0-based `first_packet_sample=4992000`。用于逐 slot/MATLAB 回归，不通过
能量门为每个通信包创建 Region。

## `x410_rfnoc_uwb_scheduled.grc`

实机生产骨架：

```text
X410 RFNoC Radio 737.28M (SC16)
→ UWB RFNoC Upsampler 65/48 (SC16)
→ RFNoC Rx Streamer (CF32 998.4M)
→ Scheduled Extractor
→ Packet Writer
```

打开图后必须检查：设备地址、中心频率、增益、天线、`first_packet_sample`、
输出目录，以及 FPGA block type/instance。包装块默认查找 UHD block type
`Upsampler`；若 `uhd_usrp_probe` 显示不同名称，应修改对应参数。

## `x410_rfnoc_uwb_detector_sc16.grc`

未知 packet 时刻的 SC16 生产路径。RFNoC RxStreamer 保持 SC16 host
output，全速率只运行整数能量门；候选 Region 在后台转换一次 CF32 做相关，
最终原始 SC16 PDU 直接写盘。与 scheduled 图不同，它会处理所有通过能量门的
通信/雷达候选，因此密集通信环境下仍优先使用已知 `t0/T` 的 scheduled 路径。

## `uwb_radar_loopback_cir.grc`（读文件 TX）

纯消息链路的单站雷达软件回环（无 UHD）。发送端 **加载** 完整 packet 文件：

```text
Message Strobe (每 PRI 一个 emit)
→ Radar Packet Source（testdata/uwb_radar/tx_998p4.cf32）
→ Loopback Echo（整数/分数时延、多径、AWGN）
→ Radar CIR Estimator（998.4 MS/s，SYNC 模板 sync_template_998p4.cf32）
→ CIR Writer（cir.cf32 / cir_norm.cf32 / cir.jsonl / run.json）
```

## `uwb_radar_loopback_cir_synth.grc`（C++ 合成 TX）

同一条 CIR 回环，发送端换成 `UwbHrpPacketSource` 实时组包。默认 PHY 对齐雷达
golden：code 9、64 SYNC、4z2、STS SP1、22 B PSDU
（`47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221`，已含 FCS）。

```text
Message Strobe
→ HRP Packet Source（998.4 MS/s 合成）
→ Loopback Echo → Radar CIR Estimator → CIR Writer
```

两张图不要合在一张里切换：读文件走 `uwb_radar_packet_source`，合走
`uwb_hrp_packet_source`。Estimator 的 `status` 接 Message Debug。把 strobe
删掉并将 Packet Source 的 `auto_emit` 置 True 即为单脉冲；`echo_delays` /
`echo_gains` 控制回波。CIR 用 `testdata/uwb_radar/read_uwb_cir.m` 读取。
合成图的 `psdu_hex` / `sync_repetitions` / `insert_sts` 可改；改 SFD 时 CIR
Estimator 的 `sfd_mode` 必须一起改。

## `uwb_radar_cir_udp.grc`（CIR 经 UDP 送到另一台机器）

同一条软件回环 CIR 链，估计器的 `cir` 口并联到 `network.socket_pdu`
（`UDP_CLIENT`）。默认目的地 **133.133.133.132:12345**。

```text
Message Strobe
→ HRP Packet Source
→ Loopback Echo
→ Radar CIR Estimator
    → CIR Writer
    → Socket PDU UDP_CLIENT  →  133.133.133.132:12345
```

`socket_pdu` 只发送 PDU 的 `c32vector` 原始字节，**不含** `pulse_id` / status
等 meta。当前窗 16+100 tap = **928 字节 / 脉冲**，小于 1472，一帧一个 UDP 包。
对端：

```bash
python3 gr-uwb/apps/cir_udp_recv.py --bind 0.0.0.0 --port 12345
```

源码树直接跑（不必先 `grcc`，会加载 `gr-uwb/build` 的 Python 绑定）：

```bash
python3 gr-uwb/apps/uwb_radar_cir_udp.py \
  --udp-host 133.133.133.132 --udp-port 12345 --seconds 5
```

GRC 参数 `udp_host` / `udp_port` 可改。X410 实机路径用同一目的地：

```bash
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --udp-host 133.133.133.132 --udp-port 12345 ...
```

`--no-udp` 关闭发送。

## 编译校验

从源码树使用自定义 block YAML：

```bash
GRC_BLOCKS_PATH=$PWD/gr-uwb/grc grcc -o /tmp/uwb-grcc \
  gr-uwb/examples/uwb_detector_to_writer.grc \
  gr-uwb/examples/uwb_scheduled_file_capture.grc \
  gr-uwb/examples/x410_rfnoc_uwb_scheduled.grc \
  gr-uwb/examples/uwb_radar_loopback_cir.grc \
  gr-uwb/examples/uwb_radar_loopback_cir_synth.grc
```

安装 OOT module 后，GRC 会从标准 block path 自动发现这些块。

运行雷达回环图生成的脚本（未安装 OOT 时）：

```bash
PYTHONPATH=$PWD/gr-uwb/build/test_modules python3 /tmp/uwb-grcc/uwb_radar_loopback_cir.py
PYTHONPATH=$PWD/gr-uwb/build/test_modules python3 /tmp/uwb-grcc/uwb_radar_loopback_cir_synth.py
```
