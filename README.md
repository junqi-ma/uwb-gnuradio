# UWB 接收与信号处理

当前分支以原生 UHD/C++ 为开发基线。可以直接从 UHD 接收 SC16，也可以回放同格式文件，再执行单音消除、候选捕获、窗口重采样、DW3000 解调和周期锁定。原生库还提供 HRP 调制、SIC、CIR 和离线 TWR。GNU Radio blocks、PMT/PDU 适配、Python 绑定及 GRC 流程保存在本地分支 `archive/gnuradio-adapter`（`d5dfd30`）。

在仓库根目录构建。依赖 C++17、CMake、VOLK、Boost.Test。连接 USRP 时需 UHD 开发包。Python 3 用于辅助工具和部分测试。

```sh
cmake -S gr-uwb -B gr-uwb/build-native-only -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native-only -j4
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-native-only --output-on-failure -j1
```

仅做文件回放时，配置 `-DENABLE_UHD_BACKEND=OFF`。默认 `AUTO` 会按机器上是否可用 UHD 开发包选择后端。实时接收建议使用默认 ISA 自动选择或明确指定受机器支持的 SIMD。以下示例从 737.28 MS/s SC16 文件解调。

```sh
gr-uwb/build-native-only/apps/uwb_dw3000_live --replay capture.sc16
```

`uwb_core` 提供 DSP，`uwb_native` 提供原生设备源、回放和窗口运行时。外部 C++ 工程可安装后通过 `find_package(uwb CONFIG REQUIRED)` 链接 `uwb::uwb_native`。源码目录 `gr-uwb/`、公共头文件 `<gnuradio/uwb/...>` 和命名空间 `gr::uwb` 沿用既有名称，方便原生调用者继续编译。

- [原生构建、运行与安装说明](docs/原生UHD与C++构建使用说明.md)
- [当前开发状态](开发状态.md)
- [接收与离线回放工具](tools/rx/README.md)
- [解调与窗口执行的代码边界](docs/架构_解调与窗口执行.md)
- [文档索引](docs/README.md)
