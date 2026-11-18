# DW3000 接收工具

更新：2026-11-17，主分支 `feature/sic-research`，低 SNR 优化合入 `63c057f`。
低 SNR 完整帧与 ABBA 实验见 [复现工具](low_snr_eval/README.md)。
完整构建、安装资源与接口边界见
[原生 UHD/C++ 构建使用说明](../../docs/原生UHD与C++构建使用说明.md)。

## 直接使用当前原生 C++ live

在仓库根目录构建和运行：

```sh
cmake -S gr-uwb -B gr-uwb/build-native-only \
  -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_EXPERIMENTS=OFF \
  -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native-only --target uwb_dw3000_live -j4
gr-uwb/build-native-only/apps/uwb_dw3000_live \
  --replay /path/to/capture.sc16 --workers 4 --sync locked --period-ms 5
```

省略 `--replay` 会连接 X410；SC16 输入须为 737.28 MS/s。C++ 程序默认
`--mode continuous --sync acquire --fused on --cir full --ondemand auto`。
整数 chip 网格准入门限约 6.468 ppm，可用 `--fused-gate-ppm` 收紧，拒绝时按包
Full→Bypass；PACKET 的 `grid_reject` / `st_fused` 用于观察实际分支。
单音默认开启；周期默认 5 ms，可用 `--period-ms` 改为实际发包周期。

`--mode timed` 仅支持 replay，且关闭单音消除；真实 UHD 定时 RX 尚未实现。
安装程序可通过 `--res-dir DIR` 指定 taps/template；详见上面的构建使用说明。
`--taps quality|eq32|realtime|eq16` 默认 `eq16`；安装包含全部四套系数文件。

**入口差异**：`live_dw3000_packets.py` 是兼容入口，显式默认 `--cir auto`，
使用 `build-live-wrapper` 构建原生程序。新增 C++ 参数放在 `--` 后透传：

```sh
python3 tools/rx/live_dw3000_packets.py --replay capture.sc16 --cir full \
  -- --sync locked --period-ms 1 --taps eq16
```

## 接收信噪比与链路余量

`measure_rx_snr.py` 从原始 SC16 统计接收 SNR，不依赖解调器任何指标：按周期
定位包与包间噪声区，噪声区 PSD 中位数作热噪声底，包区减噪声区得信号 PSD，
在 ±250 MHz（信道 5）内积分。−232.96 MHz 的 CW 单音会先拟合减掉，积分时再挖掉。

```sh
python3 tools/rx/measure_rx_snr.py \
  --input testdata/x410_fixed_tone_live_20261115/rx_1ms_1s.sc16 --period-ms 1
```

输出带内 SNR、全带宽 SNR、不消单音时的 SNR、热噪声底（LSB 与 dBFS）和单音电平。
`--json` 输出机器可读结果，`--band-mhz` 改积分带宽（默认 250）。

当前实测（DW3000 衰减 10 dB、1.5 m、X410 增益 60）：带内 SNR **+13.7 dB**，
热噪声底 −47.5 dBFS，单音比热噪声高 12–13 dB。详见
[`X410_接收信噪比与链路余量_2026-11-16.md`](../../docs/performance/X410_接收信噪比与链路余量_2026-11-16.md)。

## DW3000 逐包离线解码

仓库根目录执行：

```sh
python3 tools/rx/print_dw3000_packets.py
```

默认重新解码 `testdata/x410_fixed_tone_live_20261115/clean_1s.sc16`，使用同目录的
`packet_starts_1s.txt`（200 个包窗）。首次运行通过 CMake 构建原生 `uwb_dw3000_decode`（`build-offline-decoder`），
关闭 UHD 和实验目标；后续增量构建会跟踪头文件和编译后的解调入口。默认 CFO 配置，无 skip=216 临时参数。

逐包打印候选时间、检测到的 SYNC 数、SFD/PHR 状态、PHR 包长、CFO、CFO 拟合点数、
FCS、独立 Python CRC 和失败阶段。首尾窗口不足也占一行并报告原因。最后汇总成功率。
`time_ms` 为候选种子坐标，不是精确 ToA；`demod_us` 仅在 JSONL 中保存，属于解调
内核诊断时间，不包含截窗、重采样或排队。

历史夹具验证结果为默认输入 **200/200**、同窗原始数据 **193/200**；
这是对应夹具/配置的记录，不代表任意当前采集：

```sh
python3 tools/rx/print_dw3000_packets.py --raw
python3 tools/rx/print_dw3000_packets.py --payload --json-out out/dw3000_packets.jsonl
```

其他 SC16 采集需要显式提供与其对应的候选起点（每行一个非负整数 native 样点）：

```sh
python3 tools/rx/print_dw3000_packets.py --input /path/rx.sc16 --starts /path/starts.txt
```

工具限定为 737.28 MS/s SC16、code10，重采样至 998.4 MS/s。默认 256 SYNC、4z2、
Full CIR；可用 `--preamble`、`--sfd`、`--pre-guard`、`--window` 调整包窗配置。
它处理已采集文件，不访问 X410，不重新运行单音消除，也不执行新的盲捕获。

退出码：0=所有候选均完整解码并通过 FCS/独立 CRC；1=有包失败；2=参数、文件、编译
或执行错误。默认原始数据对照 193/200，因此 `--raw` 返回 1 是预期结果。

## Python 包装脚本：实时逐包打印，每 50 包汇总

```sh
python3 tools/rx/live_dw3000_packets.py
```

首次运行会自动构建当前源码，然后直接启动 UHD。默认配置：

- X410 `192.168.10.2`，通道 **3**，天线 **RX1**（第二接收连接器）。
- 6489.6 MHz、737.28 MS/s、增益 60 dB；code10、256 SYNC、4z2、6.8 Mb/s、STS 关闭。
- 连续固定单音消除开启，20 ms 更新、0.5 ms 训练；自动自相关捕获，无需手填起点。
- `--cir auto`：先按现有最强径采样方式解码；有有效前导/SFD但解码失败时，完整 CIR 重试一次。
- 两个解码 worker，24 个预分配包窗口，128×65536 样点原始队列；打印使用独立的有界队列。

按 **Ctrl+C** 停止，先关闭 UHD，再处理已提交的包并打印最终汇总。也可以指定时长：

```sh
python3 tools/rx/live_dw3000_packets.py --seconds 10
python3 tools/rx/live_dw3000_packets.py --gain 55 --workers 4
python3 tools/rx/live_dw3000_packets.py --cir full --workers 6
```

输出示例：

```text
PACKET id=50 time_ms=248.806 status=OK sync=256 sfd=1 phr=1 len=127 cfo_khz=177.100 fit=40 fcs=1 crc=1 latency_ms=7.200 full_retry=0
SUMMARY completed=50 batch=50 batch_fcs=50/50 batch_pct=100.0 total_fcs=50/50 total_pct=100.0 full_retry=0 raw_drop_samples=0 source_errors=0 gaps=0 ...
```

`SUMMARY` 每完成 50 个候选包输出一次，包含本批和累计成功率、失败阶段、完整 CIR 重试数、
原始队列丢样数、源读取错误和不连续次数。停止时不足 50 个的余数输出 `SUMMARY_PARTIAL`，
最后输出 `FINAL`。`HEARTBEAT` 每秒显示一次接收进度，没有包时也会打印。

多 worker 按完成顺序打印，`id` 是捕获顺序；候选时间不是精确 ToA。
成功率以已捕获候选为分母，未检测到的发射包不在分母内，不能直接作为 PER。
`source_errors` 包含 UHD 读取错误；`RX_GAP` 单独打印不连续边界。传输统计约每 100 ms
取快照，最终值在停止接收后刷新。`latency_ms` 从完整窗口就绪到解码完成，含解码排队，
不含前导到达至完整窗形成的时间。退出瞬间窗口不完整会显示 `INCOMPLETE_WINDOW`。

接收、截窗、解码、打印各有有界缓冲：解码池满会明确输出 `DROPPED_QUEUE`；终端过慢导致
省略输出时，`output_omitted` 会递增，成功/失败计数仍保留。

同一程序也支持无需硬件的按采样率回放（原始 SC16，单音消除同样开启）：

```sh
python3 tools/rx/live_dw3000_packets.py --replay testdata/x410_fixed_tone_live_20261115/rx_ch3_rx1_1s.sc16
```

早期包装脚本配置的验证记录：该 1 秒回放自动捕获 200 包，200/200 FCS/独立 CRC 通过，其中 1 包
完整 CIR 重试；3 秒 X410 实测捕获 599 包，599/599 通过、0 源错误、0 原始队列丢样。
开发中较长测试曾出现 UHD 不连续，因此这些结果只代表已测片段，不是长时间零溢出的保证。
本工具不保存连续 IQ。

## 单 worker / 多 worker 耗时对比

实时逐包输出增加 `worker`、`queue_us`、`resample_us`、`demod_us`、`retry_us`、
`compute_us` 和 `cpu_us`。前五个耗时及总计算耗时为墙钟时间，`cpu_us` 为当前 worker
的线程 CPU 时间。`queue_us` 包含窗口复制、线程唤醒和队列等待；`compute_us` 包含
重采样和解调（包括可选 Full CIR 重试），不含打印。结束输出 `COST_TOTAL`，记录连续
前端的自相关/环形存储/截窗、单音估计及单音相减累计墙钟时间。

先启动一次实时脚本以构建当前源码，然后可串行测试 1、2、4、4、2、1 worker：

```sh
python3 tools/rx/benchmark_dw3000_workers.py --output out/workers_new_run --seconds 3
```

每个配置运行两次，写逐包日志、配置、分位数到新目录；默认连接 X410。
加 `--replay testdata/x410_fixed_tone_live_20261115/rx_ch3_rx1_1s.sc16` 可对同一份 IQ
做按采样率回放。基准独占 `/tmp/uwb-sic-benchmark.lock`，历史输出目录不能复用。
统计计时排除每轮前 20 个候选，但全部成功/失败仍计数；耗时只对实际处理的包统计，
被队列丢弃的包单独计数。3 秒上板是短时诊断，不代表长时间稳定性或完整 PER。
详细结果见 `docs/performance/X410_实时单多worker耗时统计_2026-11-15.md`。

补测指定 worker 配置可加 `--workers 8 8`，按顺序运行两次 8 worker。默认序列保持不变。
