# 原生接收与 GNU Radio 构建使用说明

更新：2026-11-17；最新优化合入 `63c057f`，当前主分支 `feature/sic-research`。
本页描述现行接口；按日期保存的实验报告描述当时配置，不自动代表当前默认值。

## 构建选择

在仓库根目录执行：

```sh
cmake -S gr-uwb -B gr-uwb/build-native \
  -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=OFF -DUWB_BUILD_EXPERIMENTS=OFF \
  -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native -j4
ctest --test-dir gr-uwb/build-native --output-on-failure -j1
```

| 选项 | 默认 | 作用 |
| --- | --- | --- |
| `UWB_BUILD_GNURADIO` | `ON` | `OFF` 不查找 GNU Radio，不构建 GR blocks、Python bindings 或 GRC |
| `UWB_BUILD_EXPERIMENTS` | `ON` | `OFF` 不构建部分历史 benchmark/诊断目标，保留核心 QA |
| `UWB_SIMD` | `AUTO` | `AUTO/AVX2/AVX512/SCALAR`，统一选择编译期 ISA；SCALAR 不作为实时配置 |
| `ENABLE_UHD_BACKEND` | `AUTO` | `ON` 必须找到 UHD；`AUTO` 找到才启用；`OFF` 不构建硬件后端 |

原生模式依赖 C++17、CMake、VOLK、Threads、Boost.Test；当前 CMake 会要求 Boost.Test。
Python 3 用于 Python QA；开启 UHD 时需 UHD 开发包。原生模式无需 GNU Radio/PMT。
GR 模式另需 GNU Radio 3.10+（blocks/fft/filter），Python bindings 使用 Python/pybind11。
离线机器可设 `-DENABLE_UHD_BACKEND=OFF`，仍可使用 replay。

保留 GNU Radio 的构建使用独立目录：

```sh
cmake -S gr-uwb -B gr-uwb/build-gr -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=ON -DUWB_BUILD_EXPERIMENTS=ON
cmake --build gr-uwb/build-gr -j4
ctest --test-dir gr-uwb/build-gr --output-on-failure -j1
```

原生模式减少 GR/绑定/实验目标的编译工作量；尚未做配对构建计时，不宣称具体加速比例。
完整包解调入口现编译进 uwb_core；生产调用者包含 `uwb_demod_pipeline.h`，
单阶段 QA 可继续包含兼容聚合头 `uwb_demod_core.h`。依赖与状态边界见
[解调与窗口执行](架构_解调与窗口执行.md)。

## 库与原生数据路径

| 库 | 内容 | 主要依赖 |
| --- | --- | --- |
| `uwb_core`（静态） | CIR、重构、固定单音及 SIC 核心 | VOLK、Threads，无 GR/PMT/UHD |
| `uwb_native`（静态） | SC16 source、回放、单音装饰器、窗口流水线、可选 UHD 后端 | core、Threads、可选 UHD |
| `gnuradio-uwb`（共享） | GR blocks、PMT/PDU 适配 | native、GNU Radio；仅 GR 模式 |

`uwb_twr_core`、`uwb_twr_phy` 等独立库仍保留。头文件继续使用
`<gnuradio/uwb/...>` 路径和 `gr::uwb` 命名空间；这些名称本身不表示需要 GR runtime。

原生 SC16 流程为：

```text
UHD / replay SC16 @737.28 MS/s
  → 可选固定单音消除（连续模式）
  → 原生候选/目标窗装配
  → worker 内 SC16 65/48 重采样 @998.4 MS/s
  → C++ 解调/消费
```

它绕过 GR scheduler 和 PMT。`UhdSc16Pipeline` 提供原生流水线；live 接收器还包含
自己的候选窗/worker 调度，二者未在本轮统一。GR 的 RFNoC、流式重采样、PDU 重采样
和捕获应用仍保留，不应将某个 RFNoC 示例的 FPGA 要求推广到全部 host 路径。

## Live 入口与默认值

直接启动 C++ 程序（省略 `--replay` 会连接硬件）：

```sh
gr-uwb/build-native/apps/uwb_dw3000_live \
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
`--cir auto`，构建使用独立的 `build-live-wrapper` 原生目录，关闭 GR 和实验目标。
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

## 安装、资源与消费者

设定可写前缀后安装（也可换成自己的绝对路径）：

```sh
cmake -S gr-uwb -B gr-uwb/build-native \
  -DCMAKE_INSTALL_PREFIX=/tmp/uwb-native-install
cmake --build gr-uwb/build-native -j4
cmake --install gr-uwb/build-native
```

原生安装导出 `uwb` CMake 包：

```cmake
find_package(uwb CONFIG REQUIRED)
target_link_libraries(my_receiver PRIVATE uwb::uwb_native)
```

消费者使用 `-DCMAKE_PREFIX_PATH=/tmp/uwb-native-install`。GR 模式保留原
`gnuradio-uwb` 包，独立 `uwb` 包由原生模式安装。拆库后原生消费者须重新链接，
只替换旧 GR 共享库不保证其二进制 ABI 兼容；GR/Python 回归证明的是重新构建后的兼容。

Live 的资源根目录按以下顺序选择第一个存在的目录：

1. `--res-dir DIR`
2. `UWB_RESOURCE_DIR` 环境变量
3. `UWB_TESTDATA_DIR` 环境变量
4. 编译时确定的安装资源目录
5. build-tree 的 testdata 兜底

目录必须含 `resampler_65_48/taps_quality_minorder.txt` 和
`reference_preamble_code10_998p4.cf32`。安装包含这些资源；资源根目录存在但缺文件会报错，
不会逐文件尝试下一个目录。改变安装前缀应重新配置/构建，或显式传 `--res-dir`。

## 已清理与保留范围

已移除 `convert_cache` / `both` 重采样实验及专属缓存、内核、QA，以及 pipeline 的
`resampler_workers`/runner `--resampler-workers`。旧 CLI 用法会报错；
`--dsp-workers` 仍用于 runner 的 burst 级并行，live 使用 `--workers`。
分段 FIR 诊断开关移到 `DemodScratch::diagnose_fir_segments`。

内部重采样线程池仍有 GR 调用者；`scalar_legacy` / `volk_legacy` 对照、各 SIC 后端、
AcquireLegacy、replay 和 GR adapter 仍保留。两套调度未合并，不能宣称实验路径全部移除。
真实 UHD finite timed RX 尚未实现，不能把 replay timed 的证据当硬件验收。

## 验证入口与边界

- 主路径 `build-native-cleanup-main`：GNU Radio OFF / UHD ON / experiments OFF，
  构建通过；50 项 CTest 中 49 通过、1 安装消费者按设计跳过，0 失败。
- 回合前同版源码在 worktree：原生 ON/UHD 模式 53/53，GR 模式 87/87；原生测试曾在
  隐藏系统 GR/PMT 的环境执行。不同配置目标数量不同，不能直接比较总数。
- 动态 locked 回放未建立严格逐包一致性；固定窗口/IQ 对照不替代动态验证。
- 主线已有 X410 连续接收记录；**本轮拆库回合未重新上板**，不是整个项目从未上板。

详细出处：[独立验收](performance/UHD原生链路_GNURadio解耦独立验收_2026-11-15.md)、
[回合记录](performance/UHD原生解耦清理_回合主路径记录_2026-11-16.md)、
[X410 算法/实采报告](performance/X410_目标周期锁定与随机干扰双路径开发报告_2026-11-15.md)。
