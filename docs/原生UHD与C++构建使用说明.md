# OpenUWB 原生 UHD/C++ 构建使用说明

仓库根目录就是 `OpenUWB` 工程（Phase 3 去掉了 `gr-uwb/` 外壳）。GNU Radio
实现与旧用法保存在 `archive/gnuradio-adapter`（`d5dfd30`）。下面的命令都从
仓库根目录执行，构建产物在 `build/<preset>`。

## 构建与测试

```sh
env -u LD_LIBRARY_PATH cmake --preset native-avx2
env -u LD_LIBRARY_PATH cmake --build --preset native-avx2 -j
env -u LD_LIBRARY_PATH ctest --preset native-avx2 --output-on-failure
```

| 预设 | 用途 |
| --- | --- |
| `native-avx2` | Release + AVX2/FMA 基线，UHD 自动探测;日常开发与验收默认档 |
| `dev-avx2` | Release + AVX2/FMA，实验目标关闭（`UWB_BUILD_EXPERIMENTS=OFF`），日常编译 + `ctest -L rx_sic` 闭环的变体 |
| `native-avx512` | Release + AVX-512 宽内核 TU（基线仍是 AVX2/FMA） |
| `scalar-debug` | Debug + 关闭自写 SIMD 内核，用于单步与参考对照 |
| `no-uhd` | Release 且不构建 UHD 后端；回放、库和全部非硬件测试仍可运行 |

所有预设现在使用 Ninja 生成器（Stage 1）。首次切换到 Ninja 后，请删除旧的
Makefile build 目录（如 `build/native-avx2`）并重新配置。QA 目标默认启用
预编译头（`UWB_ENABLE_PCH=ON`）；可用 ccache 时自动探测并加速
（`UWB_USE_CCACHE=ON`）。

也可以不用预设，直接传参：

```sh
env -u LD_LIBRARY_PATH cmake -S . -B build/custom \
  -DCMAKE_BUILD_TYPE=Release -DUWB_SIMD=AVX2 -DENABLE_UHD_BACKEND=OFF
```

| 选项 | 默认 | 用途 |
| --- | --- | --- |
| `ENABLE_UHD_BACKEND` | `AUTO` | `ON` 要求 UHD 开发包。仅回放可用 `OFF`。 |
| `UWB_SIMD` | `AUTO` | 在 CMake 配置时选择 DSP 编译档位。也可指定 `AVX2`、`AVX512` 或 `SCALAR`。 |
| `UWB_BUILD_EXPERIMENTS` | `ON` | 允许构建原生实验应用。生产部署可设 `OFF`。 |

C++17、CMake、VOLK、Threads 和 Boost.Test 是构建依赖。Python 解释器用于
辅助工具和 Python 测试。

### SIMD 自动选择

同一分支包含 AVX2 和 AVX-512 内核。`native-avx2` / `native-avx512` 预设显式
指定档位；自行配置时 `AUTO` 会检查编译器能力与所有可见逻辑 CPU 的标志，并在
`UWB SIMD:` 日志中输出 `selected` 和 `required_isa`。

| CPU 和编译器能力 | 选中的档位 |
| --- | --- |
| AVX2/FMA 和 AVX-512 F、BW、DQ、VL 均可用 | `AVX512`，已启用的宽内核使用 AVX-512，其余代码保留 AVX2/FMA 基线 |
| AVX2/FMA 可用 | `AVX2` |
| AVX2/FMA 不可用，或未检测到 Linux CPU 标志 | `SCALAR`（VOLK 仍可自行选择 SIMD） |

产物的指令集要求在配置时固定。把产物复制到其他机器时，目标 CPU 和操作系统也要满足
该要求。交叉编译时，使用 `-DUWB_SIMD=AVX2`、`-DUWB_SIMD=AVX512` 或
`-DUWB_SIMD=SCALAR` 显式指定目标档位。

## 目录与目标

公共头文件在 `include/openuwb/<domain>/`，库源码在 `lib/<domain>/`，应用在
`apps/`，测试在 `tests/`，离线工具在 `tools/`。命名空间是 `openuwb`，子域
`openuwb::support`、`dsp`、`phy::hrp`、`sic`、`radio`、`runtime`。

库目标严格单向依赖（详见 `lib/CMakeLists.txt`）：

```
openuwb_support -> openuwb_dsp -> openuwb_phy_hrp -> {openuwb_sic, openuwb_sensing,
                                                      openuwb_twr_core}
openuwb_twr_phy -> openuwb_twr_core + openuwb_dsp
openuwb_radio   -> openuwb_dsp（仅它可链接 UHD::UHD）
openuwb_runtime -> openuwb_phy_hrp + openuwb_sic + openuwb_sensing + openuwb_radio
```

应用入口：

| 二进制 | 用途 |
| --- | --- |
| `openuwb_rx` | 实时接收与 SC16 回放主入口 |
| `openuwb_pipeline_runner` | 固定周期的 SIC/CIR 窗口流水线 |
| `openuwb_capture` | UHD SC16 直接采集与离线分析 |
| `openuwb_twr_fake_demo` / `openuwb_twr_native_phy_demo` | 离线 TWR 协议 / 原生速率 PHY 演示 |

过渡期旧名 `uwb_dw3000_live`、`uwb_uhd_sc16_pipeline_runner`、
`uwb_uhd_sc16_capture` 仍以构建树/安装树软链提供
（`TODO(openuwb-compat-removal)`）。`apps/openuwb_tx/` 只是预留目录，没有目标。

原生速率为 737.28 MS/s，窗口内重采样到 998.4 MS/s。两条应用路径的职责见
[窗口执行架构](架构_解调与窗口执行.md)。

## Live 入口与默认值

直接启动 C++ 程序（省略 `--replay` 会连接硬件）：

```sh
build/native-avx2/apps/openuwb_rx \
  --replay /path/to/capture.sc16 --workers 4 --sync locked --period-ms 5
```

输入为 737.28 MS/s complex SC16。默认 X410 `192.168.10.2`、channel 3/RX1、
6489.6 MHz、增益 60；code10、256 SYNC、4z2、6.8 Mb/s、STS off。
默认 2 worker；默认 `--mode continuous --sync acquire`。

| 参数 | C++ live 默认 / 行为 |
| --- | --- |
| `--fused` / `--cir` | `on` / `full`；Full 是融合 CMF 的前提 |
| `--fused-gate-ppm` | 默认为物理漂移上限，约 6.468 ppm；必须正值且不超上限 |
| `--sfo-from-cfo` | `on`；默认由 Stage-2 CFO 与 `--freq`（`carrier_freq_hz`）推导 `measured_period`（`-cfo_hz / carrier_freq_hz`），避免整数峰位量化离群；`off` 回退到 Stage-1 SYNC 峰位 OLS 拟合 |
| `--ondemand` | `auto`，跟随 `--fused` |
| `--tone` | `on`；20 ms 更新、0.5 ms 训练；只在连续模式启用 |
| `--sync locked` | 锁定后使用局部 ROI，默认半宽 256 样点 |
| `--taps` | `eq16`；可选 `quality/eq32/realtime/eq16` |
| `--acq-span / --acq-step` | `192000 / 64`；积分对数 / 环表块粒度，不是抽取积分 |
| `--acq-low / --acq-peak` | `0.00015 / 0.00030`；归一化延迟自相关门限 |
| `--autocorr-when-locked` | `scan`；`suspend` 在锁跟踪（Verify/Locked/Holdover）期间暂停自相关扫描，失锁后回看积压再扫描；`auto` 在 `--interference off` 时等同 `suspend`，否则等同 `scan`。`suspend` 与 `--interference on` 同时使用会在启动时报错退出 |
| `--autocorr-resume-lookback` | 默认 `192000+450000=642000` 原生样点；失锁恢复时从 `max(0, valid_begin, end-N)` 重新扫描，`0` 表示从当前 `end` 开始 |
| `--lock-fcs` | `on`；`off` 允许前导+SFD 身份用于进锁 |
| `--stop-after-sfd` | `off`；`on` 跳过 CIR/PHR/payload 与 auto Full 重试，结果不能计作完整 FCS |
| `--stop-after-cir` | `off`；`on` 在 SFD+独立 CIR 估计后返回，跳过 CMF/PHR/payload（Sensing 模式） |
| `--timing-sparse` | `on`；使用 64 点稀疏三元码做 timing/SFD/fold 匹配滤波，`off` 切回稠密脉冲成形模板 |
| `--lock-residual-limit` | `0`（自动取 `max(512, locked_roi_native)`）；显式指定网格残差异常门限（原生样点） |
| `--period-ms` | `5`；可按发送周期改为其他值 |
| `--mode timed` | 仅 replay；无 replay 返回 2，单音消除关闭 |
| `--sic` | `off`；`preamble/full/auto` 在目标 demod 前重建并减去干扰源。`off` 不分配 SIC scratch/残差缓冲，既有输出不变 |
| `--sic-backend` | `ipatov`；可选 `discrete/path` |
| `--sic-code` | `11`；干扰源 HRP code（9..12） |
| `--sic-reps` | `128`；干扰源 SYNC 数；Ipatov 仅 64/128/256 |
| `--sic-sfd` | `dw-8`（映射 `decawave`）；可选 `4z1/4z2/4z3/4z4/ieee` |

`--autocorr-when-locked` 的 `START` 输出解析后的
`autocorr_when_locked=scan|suspend`；`LOCK_STATS` 末尾新增
`autocorr_suspend_events/resume_events/suspended_samples/resume_backlog_samples`。
`--interference off` 且锁定时 `suspend` 会停止 `acq.scan`，因此
`acq_seeds`/`autocorr_ignored` 下降是统计口径变化，不代表检测能力变化。
这两个参数只影响 continuous 模式；`--mode timed` 本就不跑自相关，门控
计数恒为 0。`--autocorr-resume-lookback -1` 是默认哨兵，等同
`acq_span+window`；除 `-1` 外的负值会被拒绝。

`--sic != off` 时 `START` 追加 `sic/sic_backend/sic_code/sic_reps/sic_sfd`，
`PACKET` 追加 `sic_us/sic_lvl/sic_exec`，`SUMMARY` 追加
`sic_mean/sic_p95`，`FINAL` 前输出独立 `SIC_STATS`。SIC 失败或落
`L0_Bypass` 时原样使用未修改的重采样窗口（fail-open）。SIC 全窗盲搜的
定时核跟随 `--timing-sparse/--timing-shaped/--cfo-refit`（与 acq 一致；
默认 `--timing-sparse on`），并在宽窗上启用间隔局部探测 + K=4 相干折叠
（`rx_wide_interval_probe`，M=42 SYMBOL，仍受 `sync_reps/3`/回溯上限
约束；全 miss 快速落 `L0_Bypass`）。live 路径还通过
`copy_out_on_bypass=false` + `i_hat=nullptr` 跳过 L0 整窗拷贝/清零。
25 ms 回放夹具上 `sic_mean` 从约 10.6 ms 降到约 **0.124 ms**
（K=4/M=42、L0 免拷贝；旧 L0 路径约 0.70 ms），单 worker
`dropped=0`。当前结果仅为文件回放，尚未实时化，未上板。
当前默认仍为 `scan`；回放证据见
[锁定后暂停自相关_实现与回放验证_2026-11-20](performance/锁定后暂停自相关_实现与回放验证_2026-11-20.md)，
未上板。

每包在 Stage-2 CFO 拟合之后统一计算周期估计（`--sfo-from-cfo on` 时由 `-cfo_hz / carrier_freq_hz` 推导，否则使用 Stage-1 timing 周期拟合），据此决定整数网格准入，同时控制 CFO 旋转范围、前缀及融合。
拒绝准入时输出 `grid_reject`，按请求的 CIR 模式走分数网格的完整 soft-chip 路径。
`sync` 记录初始定时实测的峰数，`sync_folded=1` 表示峰来自多符号折叠锚点，
`sync_grid` 记录 CIR 所用拟合网格覆盖的符号数。`Qm35825Profile` 本身的融合标志默认关闭，不能将 live 的
默认配置推广到所有调用者。无参整数网格辅助函数的容差为 1.5 ppm，profile 默认门限
为 6.468 ppm，而 live 默认取物理上限常量；调用时显式容差决定实际准入。

**Python 包装入口有区别**：`tools/rx/live_dw3000_packets.py` 仍显式传入默认
`--cir auto`。它复用 `$OPENUWB_BUILD_DIR` 或 `build/native-avx2`，构建目标为
`openuwb_rx`（不再创建私有 `build-live-wrapper` 目录）。
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
cmake --install build/native-avx2 --prefix /tmp/openuwb-install
env -u LD_LIBRARY_PATH tests/integration/install_consumer/run_native_install_consumer.sh \
  --prefix /tmp/openuwb-install
```

外部项目优先使用主包：

```cmake
find_package(OpenUWB CONFIG REQUIRED)
target_link_libraries(app PRIVATE openuwb::runtime)   # 或 openuwb::twr_core
```

公共头文件是 `<openuwb/...>`，命名空间是 `openuwb`。整包解调入口
`<openuwb/phy/hrp/rx/demod_pipeline.h>` 链接 `openuwb::phy_hrp`
（经 `openuwb::runtime` 传递）。

旧包名仍可用（`TODO(openuwb-compat-removal)`）：`find_package(uwb CONFIG REQUIRED)`
+ `uwb::uwb_native` / `uwb::uwb_core` / `uwb::uwb_twr_core`；头文件可继续包含
`<gnuradio/uwb/...>`，并通过 `include/gnuradio/uwb/compat.h` 使用 `gr::uwb`。
两种写法指向同一组导入目标，不会出现第二套实现。

安装包含四套 live 重采样系数及前导模板，资源目录是 `share/openuwb`。使用
`cmake --install --prefix` 临时重定位时，可通过
`--res-dir /tmp/openuwb-install/share/openuwb` 指定资源。

日志使用 `<openuwb/support/log.h>`（`OPENUWB_LOG_ERR/WARN/INFO/DEBUG`），
默认写 `stderr`，可用 `OPENUWB_LOG_LEVEL`（优先）或 `UWB_LOG_LEVEL` 调整；
`openuwb_rx` 的 `Reporter` 独占 `stdout`，事件行格式不变。

原有 GR 流程和测试数据可在归档分支复现。历史性能报告中的 GR 结果保留原日期和范围。
需要旧 GR 生成器的 SIC 实验可从归档分支生成夹具，再由当前分支的原生比较工具读取。
