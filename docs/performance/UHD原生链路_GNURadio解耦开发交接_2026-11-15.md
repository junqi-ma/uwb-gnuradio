# UHD 原生链路 GNU Radio 解耦与历史路径清理 — 开发交接（2026-11-15）

实施范围：`docs/performance/UHD原生链路_GNURadio解耦与历史路径清理修改规划_2026-11-15.md` 的
**P1 + P2 全部 + P3 的 1–3**。P3 的 4–5 只做了兼容评估，**未实施**，剩余工作见 §7。P4 未触碰。
基线 `a8f0016`；全部改动未提交、未推送、未全局 install、未上板。

## 0. 首轮独立验收修复（A/B/C，2026-11-15 第二轮）

- **A. 版本顺序**：`VERSION_MAJOR/API/ABI/PATCH` 移到 `include(GrVersion)` 之前定义，GR 配置恢复输出
  `Building for version: 1.0.0.0 / 1.0.0`；`libgnuradio-uwb.so` SONAME 回到 `libgnuradio-uwb.so.1.0.0`，
  版本化文件 `libgnuradio-uwb.so.1.0.0.0` 存在。新增回归测试 `uwb_qa_gnuradio_uwb_version`
  （`lib/qa_uwb_gnuradio_uwb_version.sh`，GR 模式注册）钉住 SONAME 与版本文件名。
- **B. 安装包 UHD 依赖闭包**：新增 `cmake/Modules/uwbConfigUhdDep.cmake` 并随 `uwbConfig.cmake` 安装；
  当消费者的 UHD 包不定义 `UHD::UHD` 时（只给 `UHD_LIBRARIES`/`UHD_INCLUDE_DIRS` 或仅 pkg-config），
  与顶层构建相同的 fallback 在消费者侧重建 `UHD::UHD`。已用独立消费者复现原失败场景：
  `find_package(uwb CONFIG REQUIRED)` + `target_link_libraries(... uwb::uwb_native)` 在
  `/tmp/uwb-native-prefix`（UHD=ON）上 configure/generate/build/运行全部成功。
  原生安装消费者脚本也改为通过 `find_package(uwb CONFIG)` 生成微型 CMake 工程构建，不再手工拼 `-l`。
- **B 补充（pkg-config fallback 可达性）**：`uwbConfigUhdDep.cmake` 不再用 `find_dependency(UHD ...)`
  （会继承父包 REQUIRED，在 `CMAKE_DISABLE_FIND_PACKAGE_UHD=TRUE` 或无 UHD config 时直接报错、
  pkg-config 分支不可达），改为可失败的 `find_package(UHD CONFIG QUIET)`，然后
  `pkg_check_modules(UHD_PC QUIET IMPORTED_TARGET uhd)`（优先用 `PkgConfig::UHD_PC` 导入目标，支持非系统路径），
  再退到 `UHD_LIBRARIES`/`UHD_INCLUDE_DIRS`/`UHD_PC_*` 变量；三者皆无时设置
  `uwb_FOUND FALSE` + `uwb_NOT_FOUND_MESSAGE` 并 return，不创建空导入 target，
  `uwbConfig.cmake`据 `UWB_UHD_DEP_OK` 跳过 `uwbTargets.cmake` 包含。
  已验证四种场景：① 正常 UHD=ON 前缀（config target 路径）构建运行 OK；② 审查复现
  `-DCMAKE_DISABLE_FIND_PACKAGE_UHD=TRUE` 经 pkg-config fallback 构建运行 OK 且链接 libuhd；
  ③ REQUIRED 且完全无 UHD → 以 `uwb_NOT_FOUND_MESSAGE` 明确失败；
  ④ 非 REQUIRED 且无 UHD → `uwb_FOUND=0`、无 `UHD::UHD`、无 `uwb::uwb_native`，配置不硬错；
  UHD=OFF 前缀消费者不受影响（ALL OK）。
- **C. 原生 live console QA**：`uwb_qa_dw3000_live_console` 的注册条件从 `if(ENABLE_TESTING)` 改为
  `if(CMAKE_TESTING_ENABLED OR ENABLE_TESTING)`（与 runner exit QA 一致），原生模式现为 **53 项全过**
  （含 live console），UHD=OFF 原生模式 52 项。
- 另按审查记录第 5 项准确说明：`rx_diagnose_fir_segments` 只是**配置开关**从公共 production profile
  迁到 `DemodScratch`；诊断实现 `uwb_softchip_fir_diag.h` 与 demod core 内的分支仍在公共核心头中，
  生产路径默认永远关闭——本文档不宣称诊断已从生产核心完全移除。

## 1. 组件开关与目标结构

```
uwb_core        STATIC  纯 C++ 核心（SIC/CIR/离散估计/单音核心），链接仅 VOLK+Threads
uwb_native      STATIC  原生 RX 运行时（pipeline、Replay/UHD 源、单音装饰器），PUBLIC 链接 uwb_core，可选 UHD
uwb_twr_core / uwb_twr_fake_link / uwb_twr_phy   STATIC（原样保留）
gnuradio-uwb    SHARED  仅 GNU Radio adapter（gr::block/PMT），UWB_BUILD_GNURADIO=ON 时构建，
                        PUBLIC 链接 uwb_native；保留 GR block 源码/API，但不承诺旧共享库全部原生符号的 ABI
原生 apps       链接 uwb_native/uwb_core，不链 gnuradio-uwb
```

CMake 选项（`gr-uwb/CMakeLists.txt` 顶层）：

| 选项 | 默认 | 含义 |
|---|---|---|
| `UWB_BUILD_GNURADIO` | ON | OFF 时不执行 `find_package(Gnuradio)`、不 include 任何 `Gr*` 模块；独立发现 Volk/Threads/Boost/Python3；UHD 仍按 `ENABLE_UHD_BACKEND` AUTO/ON/OFF 发现 |
| `UWB_BUILD_EXPERIMENTS` | ON | OFF 时跳过历史 benchmark/诊断实验 app（见 §5.3） |
| `ENABLE_UHD_BACKEND` | AUTO | 上移到顶层，两种模式共用，语义不变 |

## 2. 正确构建 / 测试 / 安装命令

```bash
cd gr-uwb
# A. GR adapter 模式（回归基线，行为应与改动前一致）
cmake -S . -B build-cleanup-gr -DCMAKE_BUILD_TYPE=Release
cmake --build build-cleanup-gr -j13
env -u LD_LIBRARY_PATH ctest --test-dir build-cleanup-gr -j1

# B. 原生模式（无 GR 包发现；本机 GR 装着也不会被找）
cmake -S . -B build-cleanup-native -DCMAKE_BUILD_TYPE=Release -DUWB_BUILD_GNURADIO=OFF
cmake --build build-cleanup-native -j13
env -u LD_LIBRARY_PATH ctest --test-dir build-cleanup-native -j1

# C. 原生 + 无 UHD（回放/QA 路径）
cmake -S . -B build-cleanup-native-nouhd -DUWB_BUILD_GNURADIO=OFF -DENABLE_UHD_BACKEND=OFF
cmake --build build-cleanup-native-nouhd -j13
env -u LD_LIBRARY_PATH ctest --test-dir build-cleanup-native-nouhd -j1

# D. 临时前缀安装 + 消费者
cmake --install build-cleanup-native --prefix /tmp/uwb-native-prefix
cmake -S . -B build-cleanup-native -DUWB_BUILD_GNURADIO=OFF -DUWB_NATIVE_INSTALL_PREFIX=/tmp/uwb-native-prefix
env -u LD_LIBRARY_PATH ctest --test-dir build-cleanup-native -R uwb_qa_native_install_consumer
# 或直接：gr-uwb/apps/install_consumer/run_native_install_consumer.sh --prefix /tmp/uwb-native-prefix

# E. GR 模式安装 + 原有消费者门（回归）
cmake --install build-cleanup-gr --prefix /tmp/uwb-gr-prefix
cmake -S . -B build-cleanup-gr -DUWB_TWR_INSTALL_PREFIX=/tmp/uwb-gr-prefix
env -u LD_LIBRARY_PATH ctest --test-dir build-cleanup-gr -R uwb_qa_install_consumer
```

## 3. 实际改动（P1/P2）

头文件解耦：

- `api.h` 改为自包含：不再 include `<gnuradio/attributes.h>`，宏语义与 GR 的 attributes.h 在 GCC/Clang 上逐位相同（`visibility("default")`），MSVC 仍按 `gnuradio_uwb_EXPORTS` 等 dllexport/dllimport。adapter 导出语义不变。
- 新增 `uwb_radar_meta_core.h`：纯 C++ 常量/校验（速率、span、SYNC reps、code 校验、白名单 key 数组），不含 PMT。`uwb_radar_pdu_meta.h` 只留 PMT 字典映射并 include 新头，**不复制常量**；`uwb_hrp_mod_core.h` 及 6 个只用常量的 QA/app 改为 include 新头。
- 新增 `uwb_sc16_stream_source.h`：`StreamReadMeta` + `ISc16StreamSource` 最小契约从 pipeline 头迁出；`uwb_fixed_tone_cancel_source.h` 改为只 include 它 + 单音核心头，**不再被迫带入 SIC/雷达/解调全部消费者**。
- 新增 `uwb_gr_complex.h`（`typedef std::complex<float> gr_complex;`，与 GR 头同名重定义合法）；6 个核心 QA 的 `<gnuradio/gr_complex.h>` 换成它。
- `uwb_preamble_verifier_sc16.h` 不再 include `<gnuradio/filter/fir_filter.h>`：`fir_filter_ccc` 只被当作 tap 存储（set_taps/taps），改为 `std::vector<gr_complex> fir_taps_`，相关性仍走原生 detector-core kernel，**坐标逐位不变**（qa_uwb_detector_sc16 等 GR QA 全绿为证）。

库与 app 拆分（`lib/CMakeLists.txt`、`apps/CMakeLists.txt` 重写）：

- 每个源文件只编译进一个目标（core 6 个 / native 2 个+UHD 后端 / adapter 23 个），适配层 SHARED 链接 uwb_native。GR 模式下 `GR_LIBRARY_FOO(gnuradio-uwb)` 照旧，静态档案加入同一 export set。
- 原生 app（live、runner、capture、steady/burst benchmark、tone 工具、hrp_mod、offline_postprocess、TWR demo）全部改链 `uwb_native/uwb_core`；5 个 GR flowgraph benchmark 只在 GR 模式构建。
- QA：GR 模式全部 86 项照旧 `GR_ADD_CPP_TEST`；原生模式核心 QA 用裸 `add_executable/add_test` 链接 `uwb_native/uwb_core/uwb_twr_phy + Boost`（目标名不变，per-target compile definitions 不变），31 个 GR-block QA 不编译并打印跳过清单。`qa_uwb_twr_timing_budget.cc` 因含 PDU adapter 头，已从核心 QA 移入 adapter QA。
- 安装/包配置：GR 模式照旧安装 `gnuradio-uwbConfig.cmake` + `gnuradio-uwbTargets.cmake`；原生模式安装 `uwbConfig.cmake` + `uwbTargets.cmake`（namespace `uwb::`）到 `<prefix>/lib/cmake/uwb/`，config 只 `find_dependency(Threads/Volk/UHD)`，**不查找 GR**。新增头文件（`uwb_radar_meta_core.h`、`uwb_sc16_stream_source.h`、`uwb_gr_complex.h`、pipeline/tone/lock/timed 头、`uwb_discrete_cir_sic.h`、`uwb_sc16_scale.h`）加入安装列表。
- live 资源定位：`uwb_dw3000_live` 新增 `--res-dir`，解析顺序 `--res-dir` > `$UWB_RESOURCE_DIR` > `$UWB_TESTDATA_DIR` > 编译期安装资源目录 `UWB_INSTALL_RESOURCE_DIR` > 编译期 build 树 testdata 兜底。两个资源文件（taps、模板）安装到 `<prefix>/share/gr-uwb/`。runner 默认夹具支持 `$UWB_TESTDATA_DIR` 运行时覆盖。产品代码中无任何主工作区硬编码。
- 新增原生安装消费者：`apps/install_consumer/uwb_native_consumer.cc` + `run_native_install_consumer.sh`，CTest `uwb_qa_native_install_consumer`（`-DUWB_NATIVE_INSTALL_PREFIX=<prefix>`，空则 SKIP 77）。通过 `find_package(uwb CONFIG REQUIRED)` + `uwb::uwb_native` 构建（验证导出依赖闭包，含 UHD::UHD fallback，见 §0-B），只用前缀，再用 readelf/ldd + `-H` 头轨迹证明无 GR/PMT 依赖和头文件。新增版本契约回归 `lib/qa_uwb_gnuradio_uwb_version.sh`（GR 模式，§0-A）。另修 `run_install_consumer.sh`：python 消费者运行时把前缀 lib 置于 LD_LIBRARY_PATH 最前，避免 /usr/local 旧库影子（临时前缀下原会失败）。

## 4. P3 删除项与兼容行为

1. **C2 `convert_cache`/`both` 删除**：`uwb_rational_resampler_core.h` 删除 set_convert_cache/cc 缓冲/专属 AVX2 f32 内核/调度分支（C1 `fixed_phase` 保留且逐位一致仍由 QA 门控）；pipeline 的 `ResamplerExperiment` 只剩 none/fixed_phase，`convert_cache|both` 打印明确 removed 诊断并 fail-closed 到 none；runner/steady 的 `--resampler-experiment` 对旧值 **退出码 2 明确报错**；`benchmark_sc16_burst_core --convert-cache` 退出码 2；`qa_uwb_resampler_burst_opt` 的 C2/both 用例删除，C1 bit-identity 矩阵保留。
2. **`--resampler-workers` 删除**：`PipelineConfig::resampler_workers` 字段、pipeline 内强制-1 的告警、runner 解析/打印全部删除；旧参数 **退出码 2 明确报错**（"use --dsp-workers"）。Python 仿真器 `uwb_sim_preamble_sic.py --native-resampler-workers` 是独立仿真参数，未改（见 §7）。
3. **实验/诊断构建隔离**：`UWB_BUILD_EXPERIMENTS=ON|OFF` 控制历史 benchmark/诊断 app（fullpacket compare/cfo/segment、discrete_cir_taps、bench_unified_timing、experiment_preamble_recovery、benchmark_chain_workers 及其 py QA）。`rx_diagnose_fir_segments` 从公共生产 profile（`uwb_phy_profile.h`）迁出到 `demod::core::DemodScratch::diagnose_fir_segments`，由实验包装 `apps/fullpacket_rx_pipeline.h` 设置；生产路径默认永远关闭。注意：诊断实现（`uwb_softchip_fir_diag.h`）和 demod core 内的分支仍在公共核心头中，即只迁移了配置入口而非完全隔离实现（见 §0）。

## 5. 已跑自测（本 worktree 新构建实测）

| 场景 | 结果 |
|---|---|
| GR=ON 全量 CTest | **87 项：86 通过 + 1 按设计跳过**（新增 `uwb_qa_gnuradio_uwb_version`；`uwb_qa_native_install_consumer` 空前缀 SKIP；`uwb_qa_install_consumer` 配前缀 PASS）；configure 输出恢复 `Building for version: 1.0.0.0 / 1.0.0` |
| GR=OFF、UHD=AUTO 全量 CTest | **53/53 通过**（含 live console QA——验收项 C；runner exit；native install consumer 配前缀） |
| GR=OFF、UHD=OFF 全量 CTest | **52/52 通过**（consumer SKIP），runner NEEDED 仅 `libvolk+libstdc++...`，无 uhd/gnuradio/pmt；UHD=OFF 前缀消费者同样 ALL OK |
| 独立包消费者（验收项 B 复现） | 外部工程 `find_package(uwb CONFIG REQUIRED)` + `uwb::uwb_native` 对 `/tmp/uwb-native-prefix` configure/generate/build/运行全成功，NEEDED 仅 volk/uhd/std |
| 原生 app NEEDED（GR=ON 与 OFF 两个构建） | `uwb_dw3000_live` / runner / capture / steady：仅 `libuhd`（UHD=ON 时）、`libvolk`、std —— **无 `libgnuradio-*`/pmt**；`libgnuradio-uwb.so` 仍含 GR（符合预期） |
| 头文件独立性 | 对全部核心/native 公共头做 `g++ -E -H` 轨迹：0 个 GR/PMT 外部头（仅 `gnuradio/uwb/*` 自己） |
| 原生安装消费者 | 对 `/tmp/uwb-native-prefix`：仅用前缀编译 `uwb_native_consumer.cc`，运行全 OK，NEEDED 只有 volk/uhd/std，`-H` 无 GR 头，CTest PASS |
| GR 模式临时前缀安装消费者 | `/tmp/uwb-gr-prefix` 上 `uwb_qa_install_consumer` PASS（C++ + pybind） |
| 输出一致性 | `benchmark_sc16_burst_core`（56000 样点、quality_minorder taps）在 GR 与 native 两个构建、fixed_phase 0/1 下 checksum 全部 `0xa4f3654771a28a06`，length_ok，C1=default 逐位一致；runner 在回放夹具上两个构建的 ledger（offered=8/admitted=7/completed=7、invariants PASS）一致 |
| 删除入口行为 | `--resampler-workers`、`--resampler-experiment convert_cache|both`、`--convert-cache` 均退出码 2 并打印明确 removed 原因 |
| live 资源定位 | `--res-dir /tmp/resdir` 与 `UWB_RESOURCE_DIR=/tmp/resdir` 均可启动并加载 taps/模板（回放 0.2 s 正常出 PACKET/SUMMARY 行） |

## 6. 保留行为（未动）

宽搜索/LockedBounded（locked ROI 失败不回宽搜）、fractional SPC/CIR/融合 CMF、三种 SIC 后端（DiscreteCir/PathFractional/Ipatov）、SC16 缩放、长度/群时延公式、gap/时间/Ready/守恒契约、VOLK 与 scalar fallback、65/32、回放源、GR adapter 全部源码与 GR 构建模式、timed 无 replay fail-closed 等均未改动；Python/GRC 只在 `UWB_BUILD_GNURADIO=ON AND ENABLE_PYTHON` 时加入子目录。

## 7. 剩余工作与限制（不伪报完成）

- **P3.4 未实施**：内部线程池（`run_mt`/`run_mt_sc16`/`set_num_workers`/`pool_parallel_for`）仍被 GR adapter 调用——`uwb_rational_resampler_ccf_65_48.cc:149/152`、`uwb_rational_resampler_ccf_65_32.cc:151/156`、`uwb_pdu_rational_resampler_ccf_65_32.cc:221/243` 以及 `qa_uwb_pdu_rational_resampler_65_32_workers.cc`。按规划须先把这些调用方迁到非池路径再删池；本轮为保 GR 功能不动而未做。
- **P3.5 未实施**：`scalar_legacy`/`volk_legacy` 内核仍在 `uwb_rational_resampler_core.h::set_kernel` 中，且 `qa_uwb_rational_resampler_core.cc:430` 用 `volk_legacy` 作对照参考、`benchmark_resampler_65_48.cc` 用作 A/B。删除需把 QA 改为独立 float64/golden 参考，改动面较大，留待单独一轮。
- `apps/uwb_sim_preamble_sic.py` 的 `--native-resampler-workers` 是纯 Python 仿真器参数，未随 C++ 入口删除。
- `tools/performance/run_sc16_round2_matrix.py` 仍可传 `--resampler-experiment convert_cache`——二进制会明确报错退出 2（兼容行为即"明确错误"）。
- 未验证：无 GR 开发头的干净环境（容器/chroot）下的配置编译——本机用 `-H` 头轨迹 + 原生构建无 GR cache + readelf/ldd 证明四层（构建/头文件/运行时/安装消费者）；主 Codex 可在无 GR 环境复核。未上板；MATLAB 交叉验证未跑；性能无申报变化（拆库未动算法，AVX2 `-march=native` 编译选项全部保留在原 target 上）。
- 大型实采 IQ 未复制进本 worktree，回放对照用 `testdata/` 小夹具；如主测试需要大夹具，可只读引用主目录并显式传参（产品代码无硬编码）。

## 8. 需要主 Codex 独立验证的项目

1. 四种构建矩阵（GR=ON/OFF × UHD AUTO/OFF）的 configure/build/ctest，对照 §5 数字（87/53/52 项）；GR 配置输出应为 `1.0.0.0 / 1.0.0` 且 `uwb_qa_gnuradio_uwb_version` 通过。
2. `readelf -d`/`ldd` 检查 live、runner、capture、steady 及 `libuwb_native.a` 消费者——应无 `libgnuradio-*`/pmt。
3. 原生临时前缀安装 + `uwb_qa_native_install_consumer`（脚本内部经 `find_package(uwb CONFIG)` 构建消费者，覆盖 UHD::UHD fallback）；另用独立 CMake 工程对同一前缀做 `find_package(uwb CONFIG REQUIRED)` 闭包验证；GR 前缀 + `uwb_qa_install_consumer`。
4. 回放逐包对照（PSDU/FCS/状态/窗口/锁定字段）对比基线 `a8f0016` 的旧构建——拆库应逐位一致；`benchmark_sc16_burst_core` checksum 跨构建一致。
5. 删除入口的退出码与报错文本（§4）。
6. 无 GR 环境（如容器）重复 B 场景。

主代理最终审查补正：`pkg_check_modules(UHD_PC ... IMPORTED_TARGET)` 的正确目标名为 `PkgConfig::UHD_PC`；已修正目标名，使 pkg-config 分支实际使用导入目标的包含/链接接口。

主代理符号审查：冻结基线共享库导出 `UhdSc16Pipeline`，拆分后的 GR adapter 共享库不再导出该类的全部原生实现。原生 C++ 消费者应链接 `uwb_native`/`uwb_core`（安装包目标 `uwb::uwb_native`）并重新构建；不能仅替换旧 `libgnuradio-uwb.so` 来宣称原生二进制 ABI 兼容。现有 GR/Python 回归验证的是重新构建后的适配层。
