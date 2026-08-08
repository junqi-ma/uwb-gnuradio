# UWB GNU Radio 测试信号说明

## 文件

- uwb_code9_preamble64_payload128_standard_sfd.cfile：GNU Radio Complex Float32，按 I/Q/I/Q 交错排列。
- uwb_code9_preamble64_payload128_standard_sfd_int16.dat：小端序交错 int16 I/Q，满幅度缩放到 ±32767。
- uwb_code9_preamble64_payload128_standard_sfd_metadata.mat：MATLAB 元数据。

## PHY 参数

- 模式：IEEE 802.15.4a HRP（legacy）
- 采样率：998.400 MHz
- 平均 PRF：62.4 MHz
- 数据率：6.81 Mbps
- Preamble Code Index：9
- SYNC 前导长度：64 symbols
- SamplesPerPulse：2
- SFD：标准 IEEE 802.15.4a legacy SFD（由 lrwpanWaveformGenerator 内置生成）

## 时间布局

- 前置空白：4992000 samples = 5.000000 ms
- 包起始位置：第 4992001 个样本（1-based）
- 包波形长度：256448 samples = 256.858974 us
- 总长度：5248448 samples = 5.256859 ms
- 文件开头只有精确的复数零样本；未追加尾部空白。

## Payload 说明

- 请求的测试 payload：128 bytes。
- 标准 IEEE 802.15.4a PHR/PSDU 部分：127 bytes（125 data bytes + 2-byte FCS）。
- 额外测试扩展：1 byte，接在标准 PSDU 后的 pulse-level 波形扩展。
- 原因：802.15.4a HRP 的 PHR 长度字段和 MATLAB 生成器的 PSDULength 上限为 127 bytes。
- 因此该样本的前导、SFD、PHR、127-byte PSDU 和 FCS 是标准帧；最后 1 byte 是为满足 128-byte 测试长度而添加的非标准尾部。
- 随机种子：20260807（可复现）
- FCS：0x4E07（小端字节序：074E）
- Payload hex：F9A358C7F3FEB8D8EFBBABF5AEB07E5F77CE624994D6E5F02AC9500E8BC5E7D1A2E5788C1260EBDF4727FCDC1366A2180B3468C52B1AE63DD9DD18FC5B0E07FA6CC1A43CF1B59CFCE42DABF76B2709C25369DA3F5D5B0EAC56552BD7C901600CCE098CA06FDAB81629AED984A2F45795463EE055B76A208A1DBB12EA41074E1F

## GNU Radio 使用

1. 使用 File Source 打开 uwb_code9_preamble64_payload128_standard_sfd.cfile。
2. Item type 选择 Complex Float32，采样率设置为 998.4e6。
3. 连接 QT GUI Frequency Sink、Time Sink 或自定义 UWB 解调模块。
4. 如果使用 uwb_code9_preamble64_payload128_standard_sfd_int16.dat，请选择交错 Complex Short/int16 I/Q 类型，并注意它已按峰值归一化到 int16 满幅度。

## 重新生成

- 在 MATLAB 当前目录运行：generate_uwb_gnuradio_sample。
- 修改脚本顶部的采样率、payload、随机种子或输出目录参数后重新运行即可。

> 注意：5 ms 空白在 998.4 MHz 下对应 4,992,000 个采样点，文件会比较大。
