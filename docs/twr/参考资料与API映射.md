# TWR 参考资料与 API 映射

核对日期：2026-09-28。此文件记录需求设计参考，尚未选定实际商用模组的 SDK 版本。
API 表为功能映射，X410 实现不仿制芯片寄存器或宣称 `dwt_*` ABI 兼容。

## 1. 已取得的公开资料

| 资料 | 版本/用途 | 来源 |
|---|---|---|
| DW1000 Device Driver API Guide | Version 2.7，文内对应 driver 05.00.xx；配置、收发和时间戳语义 | [Qorvo 官方论坛托管的指南](https://forum.qorvo.com/uploads/short-url/jQPkxawHn3LzbJln0QqjPShspZP.pdf) |
| DW3xxx/QM33xxx Device Driver API Guide | 公开 PDF 封面为 4.9，修订表包含后续条目；按章节参考，不能据此认定用户 SDK 版本 | [Qorvo 官方论坛托管的指南](https://forum.qorvo.com/uploads/short-url/xD3TlXKvkujjdUaJWXv2b4E7GQN.pdf) |
| APS013：The implementation of two-way ranging with the DW1000 | Version 2.3；三报文 asymmetric DS-TWR 公式与交换 | [Decawave APS013](https://forum.qorvo.com/uploads/short-url/5yIaZ3A99NNf2uPHsUPjoBLr2Ua.pdf) |
| DW1000 产品页 | 查找实际驱动与应用示例包，不以第三方移植作为唯一协议依据 | [Qorvo DW1000](https://www.qorvo.com/products/p/DW1000) |
| DW3110 产品页 | 共同 channel 5 与扩展 channel 9 能力边界 | [Qorvo DW3110](https://www.qorvo.com/products/p/DW3110) |
| UHD Timed Commands | 本次参考 UHD 4.5.0.0 文档；实现时对照本机 UHD 版本 | [Ettus UHD](https://files.ettus.com/manual_archive/v4.5.0.0/html/page_timedcmds.html) |
| USRP X4x0 Series | 官方 RF/FPGA 能力；本地定制 image 另行读回/验证 | [Ettus X4x0](https://files.ettus.com/manual/page_usrp_x4xx.html) |

DW3xxx 指南第 6.3.26/27 节介绍 05a/05b DS initiator/responder，
第 6.3.30/31 节介绍 06a/06b SS initiator/responder。
SS 时钟估计参考 `dwt_readclockoffset()` / `dwt_readcarrierintegrator()`，具体符号与
换算系数必须绑定芯片/驱动版本。DW1000 与 DW3000 的示例格式分别验证。

## 2. API 功能映射草案

下表芯片端名称参考上述 DW1000/DW3xxx 指南；有些调用仅在特定系列/版本可用。
本项目列为拟实现的抽象职责，不代表当前已有这些 TWR 接口。

| 芯片 API 参考 | 本项目职责 |
|---|---|
| `dwt_configure()` | PHY/profile 的类型化配置、能力检查和有效配置快照 |
| `dwt_configuretxrf()` | gain、IQ amplitude、pulse shape 与已标定功率控制 |
| `dwt_writetxdata()`、`dwt_writetxfctrl()` | 帧编码、长度/FCS、ranging bit、HRP 调制 |
| `dwt_setdelayedtrxtime()`、`dwt_starttx()` | 量化后的 timed TX、最小提前量、deadline 错误 |
| `dwt_setrxaftertxdelay()` | 每个 TX 后的 RX 打开窗口 |
| `dwt_setrxtimeout()`、`dwt_rxenable()` | 限定窗口 RX 与超时；首次 Poll 可持续监听 |
| `dwt_readtxtimestamp()`、`dwt_readrxtimestamp()` | 带时间域、marker、校准及来源的时间戳接口 |
| `dwt_settxantennadelay()`、`dwt_setrxantennadelay()` | 校准模型，区分芯片天线延迟与 X410 完整链路延迟 |
| `dwt_readcarrierintegrator()`、`dwt_readclockoffset()` | CFO/SFO 诊断和有方向定义的 clock ratio |
| `dwt_setcallbacks()`、状态/中断、接收数据读取 | typed event、RX 帧与 radio error；短回调后进入 C++ FSM |
| `dwt_forcetrxoff()` | 有定义的 cancel/stop/reset 及资源释放 |

DW1000 指南中的 delayed TX 输入不是普通微秒数，而是系统时间高位并有量化要求；
其时间单位约为 `1/(499.2e6×128)` 秒。adapter 必须依据目标版本处理截位和天线延迟。
例如若空口仅携带 32 位该单位计数，整个模周期仅约 67 ms；不能直接容纳任意
长 host 回复等待。必须定义模差的最大无歧义 interval，或选用双方明确支持的扩展帧。
这些规则不能未经核对套用到所有 DW3xxx 变体。

## 3. 实现前需冻结的资料

选择目标模组后，保存 `references_manifest`：模块/芯片/板卡型号、SDK release/hash、
driver API version、示例名、固件 hash、编译选项、PHY 配置、原版与本项目修改差异。
参考 PDF 的名称/URL/版本与获取日期也进入 manifest；如归档本地文件，记录 SHA256。

第二阶段需实际检查：

- `deca_device_api.h` 或该 release 等价头文件，及配置结构、枚举与相关实现说明。
- `ss_twr_init/resp`、`ds_twr_init/resp` 的帧数组、时间戳字段、长度、字节序、FCS 约定。
- 示例采用的 clock offset 计算、天线延迟、RX timeout、TX/RX turnaround 与定时量化。
- 实际固件是否允许改 profile/回复时间，及获取结果和原始 timestamps 的接口。

公开资料足以开始本分支的需求和通用核心开发。用户所用 release 若不能公开获取，
再请求该 SDK/API/示例包；其缺失只阻塞对应模组 profile 冻结与硬件互通验收。
复用第三方源码前核对适用许可，默认在仓库保存引用和本项目适配代码。

## 4. 本地 GNU Radio 与算法参考

已定位本机 `/home/oi/gnuradio/` 中：

- `gr-uhd/lib/usrp_sink_impl.cc`：`tx_time` → timed TX metadata。
- `gr-uhd/lib/usrp_source_impl.cc`：接收 `rx_time` tag 与流命令。
- `gr-pdu/lib/tagged_stream_to_pdu_impl.cc`：tagged stream 到 PDU 的边界。
- `gr-digital/lib/header_payload_demux_impl.cc`：帧头/载荷状态与跨调度分段。

移到其他机器时重新定位源树并记录版本；实现 block 前必须实际阅读相关执行路径。
UWB 数值参考为 `UWB_demodulation/decode_uwb.m`、
`+uwbdecoder/buildUwbReference.m`、`+uwbdecoder/estimateCir.m`。
新增 TWR/ToA 模型和 MATLAB 对照脚本放在 `testdata/twr/`，保留独立 ground truth。
