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

## 编译校验

从源码树使用自定义 block YAML：

```bash
GRC_BLOCKS_PATH=$PWD/gr-uwb/grc grcc -o /tmp/uwb-grcc \
  gr-uwb/examples/uwb_detector_to_writer.grc \
  gr-uwb/examples/uwb_scheduled_file_capture.grc \
  gr-uwb/examples/x410_rfnoc_uwb_scheduled.grc
```

安装 OOT module 后，GRC 会从标准 block path 自动发现这些块。
