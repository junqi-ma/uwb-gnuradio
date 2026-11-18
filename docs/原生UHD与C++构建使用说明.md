# 原生 UHD/C++ 构建使用说明

当前分支只提供原生 C++ 库和应用。GNU Radio 实现与旧用法保存在
`archive/gnuradio-adapter`（`d5dfd30`）。下面的命令请使用新的构建目录，
避免继承旧目录中的 GR 配置或二进制文件。

## 构建与测试

```sh
cmake -S gr-uwb -B gr-uwb/build-native-only \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native-only -j4
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-native-only --output-on-failure -j1
```

| 选项 | 默认 | 用途 |
| --- | --- | --- |
| `ENABLE_UHD_BACKEND` | `AUTO` | `ON` 要求 UHD 开发包。仅回放可用 `OFF`。 |
| `UWB_SIMD` | `AUTO` | 按 CPU 选择指令集。也可指定 `AVX2`、`AVX512` 或 `SCALAR`。 |
| `UWB_BUILD_EXPERIMENTS` | `ON` | 允许构建原生实验应用。生产部署可设 `OFF`。 |

C++17、CMake、VOLK、Threads 和 Boost.Test 是构建依赖。Python 解释器用于
辅助工具和 Python 测试。SCALAR 用于正确性检查和可移植回退。

## 原生路径

- `uwb_core` 提供调制、解调、重采样、CIR 和 SIC。
- `uwb_native` 提供 UHD 连续采样和 SC16 回放、单音消除、窗口装配和 worker。
- `uwb_twr_core` 和 `uwb_twr_phy` 提供独立的离线 TWR 协议及 PHY。

接收入口是 `uwb_dw3000_live`。原生速率为 737.28 MS/s，窗口内重采样到
998.4 MS/s。固定周期的 SIC/CIR 窗口可用 `uwb_uhd_sc16_pipeline_runner`。
两条应用路径的职责见[窗口执行架构](架构_解调与窗口执行.md)。

## Live 入口与默认值

直接启动 C++ 程序（省略 `--replay` 会连接硬件）：

```sh
gr-uwb/build-native-only/apps/uwb_dw3000_live \
  --replay /path/to/capture.sc16 --workers 4 --sync locked --period-ms 5
```

输入为 737.28 MS/s complex SC16。默认 X410 `192.168.10.2`、channel 3/RX1、
6489.6 MHz、增益 60；code10、256 SYNC、4z2、6.8 Mb/s、STS off。
默认 2 worker；默认 `--mode continuous --sync acquire`。

| 参数 | C++ live 默认 / 行为 |
| --- | --- |
| `--fused` / `--cir` | `on` / `full`；Full 是融合 CMF 的前提 |
| `--fused-gate-ppm` | 默认为物理漂移上限，约 6.468 ppm；必须正值且不超上限 |
| `--ondemand` | `auto`，跟随 `--fused` |
| `--tone` | `on`；20 ms 更新、0.5 ms 训练；只在连续模式启用 |
| `--sync locked` | 锁定后使用局部 ROI，默认半宽 256 样点 |
| `--taps` | `eq16`；可选 `quality/eq32/realtime/eq16` |
| `--acq-span / --acq-step` | `192000 / 64`；积分对数 / 环表块粒度，不是抽取积分 |
| `--acq-low / --acq-peak` | `0.00015 / 0.00030`；归一化延迟自相关门限 |
| `--lock-fcs` | `on`；`off` 允许前导+SFD 身份用于进锁 |
| `--stop-after-sfd` | `off`；`on` 跳过 PHR/payload 与 auto Full 重试，结果不能计作完整 FCS |
| `--period-ms` | `5`；可按发送周期改为其他值 |
| `--mode timed` | 仅 replay；无 replay 返回 2，单音消除关闭 |

每包用 timing 周期估计统一决定整数网格准入，同时控制 CFO 旋转范围、前缀及融合。
拒绝准入时，该包请求的 Full soft-chip 路径降级到 Bypass，输出 `grid_reject` 诊断；
保留分数网格/解调回退。`Qm35825Profile` 本身的融合标志仍默认关闭，不能将 live 的
默认配置推广到所有调用者。无参整数网格辅助函数的容差为 1.5 ppm，profile 默认门限
为 6.468 ppm，而 live 默认取物理上限常量；调用时显式容差决定实际准入。

**Python 包装入口有区别**：`tools/rx/live_dw3000_packets.py` 仍显式传入默认
`--cir auto`，构建使用独立的 `build-live-wrapper` 原生目录，关闭实验目标。
兼容入口之外的 C++ 参数通过 `--` 传入，例如：

```sh
python3 tools/rx/live_dw3000_packets.py --replay /path/to/capture.sc16 \
  --cir full -- --sync locked --period-ms 1 --taps eq16
```

透传参数追加在兼容参数之后，同名参数最后一次取值生效。
`tools/rx/check_clock_offset.py` 汇总时钟统计和逐包 `grid_reject` 计数；
不再按旧 `1e-9` 门限对均值打印 PASS/FAIL。实际融合准入查看 PACKET 的
`grid_reject/st_fused` 和启动配置。

### 重采样系数与低 SNR 复现

`--taps quality|eq32|realtime|eq16` 已提交，默认 `eq16`（Hpad 16）。
安装包含全部四套系数和前导模板；`--res-dir` 可显式指定资源根目录。
性能与滤波质量依据见[实采报告](performance/X410_2ms周期收包解调报告_2026-11-15.md)。
低 SNR 完整帧夹具、ABBA runner 和统计说明见
[复现工具](../tools/rx/low_snr_eval/README.md)。

## 安装与 C++ 消费者

```sh
cmake --install gr-uwb/build-native-only --prefix /tmp/uwb-native-install
cmake -S gr-uwb -B gr-uwb/build-native-only \
  -DUWB_NATIVE_INSTALL_PREFIX=/tmp/uwb-native-install
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-native-only \
  -R uwb_qa_native_install_consumer --output-on-failure
```

外部项目使用 `find_package(uwb CONFIG REQUIRED)` 和 `uwb::uwb_native`，
配置时可设 `CMAKE_PREFIX_PATH` 指向安装前缀。头文件延用
`<gnuradio/uwb/...>`，C++ 符号延用 `gr::uwb`。整包解调入口
`uwb_demod_pipeline.h` 链接 `uwb::uwb_core`。

安装包含四套 live 重采样系数及前导模板。使用 `cmake --install --prefix`
临时重定位时，可通过 `--res-dir /tmp/uwb-native-install/share/gr-uwb` 指定资源。

原有 GR 流程和测试数据可在归档分支复现。历史性能报告中的 GR 结果保留原日期和范围。
需要旧 GR 生成器的 SIC 实验可从归档分支生成夹具，再由当前分支的原生比较工具读取。
