# openUWB

原生 C++17 UWB 基带与运行时：HRP 调制/解调、重采样、CIR、SIC、Sensing 和离线
TWR，入口是 `openuwb_rx`（实时接收或 SC16 回放）。GNU Radio blocks、PMT/PDU
适配与 GRC 流程归档在 `archive/gnuradio-adapter`（`d5dfd30`）。

2026-11-20 完成 Phase 0–4 架构重构：仓库根即 CMake 工程，公共头文件在
`include/openuwb/<domain>/`，库源码在 `lib/<domain>/`，命名空间为 `openuwb`，
可执行程序统一 `openuwb_*`。验收结论与 ABBA 数据见
[重构验收报告](docs/performance/openUWB架构重构验收报告_2026-11-20.md)。

## 快速开始

依赖 C++17、CMake、VOLK、Boost.Test；连接 USRP 时需 UHD 开发包，仅回放可用
`no-uhd` 预设。Python 3 用于辅助工具和部分测试。

```sh
env -u LD_LIBRARY_PATH cmake --preset native-avx2
env -u LD_LIBRARY_PATH cmake --build --preset native-avx2 -j
env -u LD_LIBRARY_PATH ctest --preset native-avx2 --output-on-failure

build/native-avx2/apps/openuwb_rx --replay capture.sc16
```

预设：`native-avx2`（默认）、`native-avx512`、`scalar-debug`、`no-uhd`。
完整参数、安装和消费者示例见
[原生 UHD/C++ 构建使用说明](docs/原生UHD与C++构建使用说明.md)。

## 结构

```
include/openuwb/{support,dsp,phy/hrp/{common,tx,rx},sic,sensing,twr,radio,runtime}
lib/<domain>/            与 include 镜像的静态库源码
apps/                    openuwb_rx / pipeline_runner / capture / twr_*
tests/{unit,integration,benchmarks}
tools/{rx,sic,perf,performance,twr,_common}
python/openuwb/          twr_config 纯 Python 层
```

单向依赖：`support → dsp → phy_hrp → {sic, sensing, twr_core}`；`radio → dsp`
（唯一可链接 UHD）；`runtime → phy_hrp + sic + sensing + radio`。

外部工程用 `find_package(OpenUWB CONFIG REQUIRED)` +
`openuwb::runtime`（或 `openuwb::twr_core`）。旧包名 `find_package(uwb)`、
旧头文件 `<gnuradio/uwb/...>` 与 `gr::uwb` 命名空间在过渡期保留，全部标注
`TODO(openuwb-compat-removal)`。

## 文档

- [原生构建、运行与安装](docs/原生UHD与C++构建使用说明.md)
- [当前开发状态](开发状态.md)
- [解调与窗口执行边界](docs/架构_解调与窗口执行.md)
- [文档索引](docs/README.md)
- [接收与离线回放工具](tools/rx/README.md)
