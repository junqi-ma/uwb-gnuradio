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

X410 实机自发自收雷达用法见
[`docs/phase1/使用说明_X410_CG400自发自收雷达.md`](../../docs/phase1/使用说明_X410_CG400自发自收雷达.md)。
本机 UHD/DPDK SIGILL 见
[`docs/本机UHD_DPDK_AVX512问题与绕过.md`](../../docs/本机UHD_DPDK_AVX512问题与绕过.md)。

## `uwb_sim_cg400_echo_cir.py`（无 UHD 复刻 CG400 X410 链路 + 原始落盘）

与 `x410_cg400_hrp_echo_cir.py` 同一条链路，只把 UHD 定时收发换成 numpy
仿真回波，可把每脉冲原始 RX 窗按 X410 格式落盘，直接与实采对比：

```text
UwbHrpPacketSource (998.4, code 9, 64 SYNC, 4z2)
  -> 32/65 下采样到 491.52 原生
  -> SimTimedEcho（X410 rx_geometry 窗 + 时延/多径/复增益/AWGN）
  -> PDU 65/32 -> UwbRadarCirEstimator -> UwbCirWriter
```

窗长与 X410 fullwin 一致：`pre_guard + tx_native + rx_pad(8 µs) +
range_guard + tail_guard`，491.52 原生下为 110752 样点；回波默认放在
`pre_guard + --cal-delay-native`（334）。`--dump-rx` 落盘为参考 dump 相同的
三文件契约（复用 `UwbPacketWriter`）：

```text
<output>/capture.iq            # 100 包拼接的 native SC16（IQ 交织，小端）
<output>/capture.jsonl         # 每包一行，字段与 X410 fullwin dump 逐字段一致
<output>/capture_metadata.json # 运行级配置
<output>/tx_491p52.sc16        # 仿真 TX 原生波形
```

```bash
python3 gr-uwb/apps/uwb_sim_cg400_echo_cir.py \
  --rate-hz 100 --duration-s 10 --dump-rx \
  --echo-delays "334,600" --echo-gains "1.0,0.4+0.2j" \
  --noise-std 0.01 --output <dir>
```

默认按 802.15.4z 插入 STS；加 `--no-sts` 生成不含 STS 的帧（linear
核 native 61219 / 窗 77980）。

发射成形脉冲默认 `--pulse-shape linear`（加载
`testdata/uwb_hrp_tx/pulse_trunc_linear_rc183_240.f32`，1025 taps）：
线性相位 RC 原型截断到峰位 tap 32，RC 通带 183 / 阻带 240 MHz，−3 dB
单边 200 MHz（−6/−10 dB 220/232 MHz），99% 占用双边 226 MHz，
245.76 MHz 以上 −59 dB。实测 CIR 10–100 ns 峰前/峰后 **−27.8/−27.7 dB**
（min-phase 前代 −58.0/−17.4），近距 +5/+6 tap 从 −14/−13 提升到
−29/−19。代价是峰前从 −58 抬到 −28（单站雷达负延迟无目标，且两侧对称）。
脉冲群时延 32 work taps（16 native）：dump 流程加
`--cal-delay-native 350 --echo-delays 334` 可使 CIR 峰回到 tap 16；
live SYNC 模板取 `pulse_center_taps()` 附近。`legacy` 是拟合 737.28 MS/s
参考的 48 点 Butterworth 核，频谱在 ±245.76 MHz 边缘只衰减约 3 dB，
限带后振铃；`gaussian`（`--pulse-sigma-ns`，默认 2.5 ns）拖尾最低
（−67 dB）但主瓣 4.3 ns、带宽只有 ±53 MHz。`--pulse-shape minphase`
仍可选（−58/−17 的取舍）。`--pulse-taps <file.f32>` 可加载任意预计算
核（覆盖 `--pulse-shape`）。设计脚本 `design_tx_pulse.py`，设计依据与
复算见
[`docs/固定491p52采样率_发射脉冲低拖尾方案.md`](../../docs/固定491p52采样率_发射脉冲低拖尾方案.md)、
`analysis_outputs/fixed_rate_tx_pulse/`。

产物：上述 raw 文件、`cir.cf32`、`cir_norm.cf32`、`cir.jsonl`、
`run.json`、`summary.json`（含 raw/CIR 计数、pacing、窗口几何）。
`--echo-delays` 超出 `--sfd-search-margin` 时须加大 margin，否则本脉冲
`status=sfd_failed`。

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
X410 实机脚本已改为 `UCR1` 头 + 116 tap，失败帧也发（taps 填 0），避免
对端把 `sfd_failed` 的 0 字节包当成丢包。对端：

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
