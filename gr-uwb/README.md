# 原生 UWB C++ 源码

本目录提供 `uwb_core`、`uwb_native`、离线 TWR 库和原生应用。构建、UHD 接收、文件回放和安装用法见 [原生 UHD/C++ 使用说明](../docs/原生UHD与C++构建使用说明.md)。

公共头文件继续安装到 `include/gnuradio/uwb/`。整包解调包含 `uwb_demod_pipeline.h` 并链接 `uwb::uwb_core`。原生应用通过 `uwb::uwb_native` 获得设备源与回放支持。

GNU Radio 适配模块和示例由 `archive/gnuradio-adapter` 保留。
