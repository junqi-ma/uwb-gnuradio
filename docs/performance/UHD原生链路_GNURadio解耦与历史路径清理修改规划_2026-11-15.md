# UHD 原生链路：GNU Radio 解耦与历史路径清理修改规划

日期：2026-11-15。状态：规划，尚未实施。

## 1. 工作区与基线

- 分支：`refactor/native-gr-decouple`。
- worktree：`/tmp/uwb-gnuradio-native-cleanup`。
- Git 基线：`a8f0016267cdaa62a00786db66e5e296a3988f56`。
- 主工作区最新目标锁定、timed、解调及 QA 改动尚未提交。本 worktree 在 HEAD 上复制了这些已修改的受控文件，以及两个新核心头文件、两个新 QA 和相关规划报告，作为待实现基线；这些复制内容仍为未提交改动，不是本规划实施成果。
- 精确文件清单和 SHA-256 见 `native_cleanup_baseline_20261115.json`。实现前应将所复制的既有功能与后续重构分别保存为可审查提交，不将它们混成一个重构提交。
- 未复制大型 IQ、构建目录、日志、无关未跟踪文件；运行回放前必须显式准备夹具。原工作区源码保持不变。

## 2. 目标和范围

首要目标：不安装 GNU Radio 开发包或运行库，仍可配置、构建和运行原生 UHD/C++ 接收程序及核心 QA；GNU Radio block、PMT、Python 绑定和 GRC 变为可选适配层。

次要目标：移除已否决的实验路径、失效配置入口和重复历史对照实现，减少生产配置组合。第一轮保持算法行为、默认 FIR 系数、群时延、SC16 缩放、窗口边界和锁定契约。

本规划不安排上板、install、推送或部署；不删除原始实验数据和历史报告；不把 timed replay 视为已实现的 UHD 定时采样。统一两套任务调度放在后续独立阶段，不与首轮依赖拆分同时改动。

## 3. 已确认的依赖与清理依据

| 位置 | 当前事实 | 处理方向 |
| --- | --- | --- |
| `gr-uwb/CMakeLists.txt` | 顶层强制 `find_package(Gnuradio ... REQUIRED COMPONENTS blocks fft filter)`，并使用 Gr 系列构建模块 | 原生模式不能执行 GR 包发现及 Gr 宏；VOLK、Threads、UHD 独立发现 |
| `gr-uwb/lib/CMakeLists.txt` | 原生 pipeline、单音、SIC 与 GR block 混编进 `gnuradio-uwb`，链接 runtime/filter | 拆分核心、原生运行层和适配层，源文件只编译归属清晰的一份 |
| `api.h` | 核心导出宏经 `gnuradio/attributes.h` 引入 GR 开发头 | 核心独立导出宏；兼容现有 adapter 导出语义 |
| `uwb_hrp_mod_core.h` | 引用 `uwb_radar_pdu_meta.h` 中的采样率、长度和 profile 校验，间接包含 PMT | 纯 C++ 常量/校验与 PMT 字典映射分离，保持原校验行为 |
| `uwb_fixed_tone_cancel_source.h` | 为继承数据源接口而包含整个 pipeline 头，带入 SIC/雷达/解调 | 独立最小数据源接口和配置，避免单音装饰器包含全部消费者 |
| 测试构建 | 部分 QA 依赖统一加入 GR 库 | 核心测试直接链接核心；GR block QA 单独注册 |
| `convert_cache/both` | 历史结论 C2 不采用，核心与 CLI 仍可选择 | 删除缓存及专属分支，保留 fixed_phase |
| `resampler_workers` | 原生 pipeline 保留解析/打印/校验，但各 worker 实际初始化为 1 | 删除失效入口；与核心线程池删除分开提交 |
| 内部线程池/legacy 循环 | 内部多线程在目标窗口上退化，GR 接口仍调用；legacy 是历史对照 | 先迁移调用方，再删除；独立数值参考接替对照职责 |

现有 `uwb_twr_core` 和 `uwb_twr_phy` 已有独立静态目标，可复用其分层经验；但当前顶层 CMake 仍要求 GR，不能据其链接接口宣称全项目已可无 GR 构建。

现有 `build-cfo-tail-fix` 二进制只作为初始依赖证据：live 依赖 `libgnuradio-uwb`，该库的 NEEDED 含 runtime/filter/pmt。最终结论必须来自新 worktree 新构建，不能使用旧目录替代。

## 4. 目标依赖结构

```text
uwb_core       -> C++ 标准库、VOLK；线程相关需要则显式声明 Threads
uwb_native     -> uwb_core、Threads、UHD（可选，用于设备后端）
原生 apps      -> uwb_native / uwb_core
GNU Radio adapter（可选） -> uwb_core / uwb_native、GNU Radio、PMT
Python/GRC（可选）        -> GNU Radio adapter
```

- core：检测/调制/重采样/解调/CIR/SIC、锁定、timed 计划和纯配置。
- native：源接口、回放、UHD 数据源、单音装饰器、有界流水线及任务调度。
- adapter：`gr::block`、stream tag、PMT 字典/PDU、消息端口、GR Python 绑定。
- VOLK 保留，它可独立使用；不为清理依赖重写现有 SIMD 算法。
- 初期保留 `gnuradio/uwb/` 项目头路径及 `gr::uwb` 命名空间，避免无必要的批量改名。判断依赖以外部头文件、链接符号和干净环境构建为准。
- 不以减少动态库加载数量承诺 DSP 吞吐改善；主要收益是构建、部署、测试和维护简化。

## 5. 分阶段实施

### P0：冻结行为及独立构建契约

1. 核对复制清单、未提交功能与新文件；保存基线状态和构建命令。
2. 在全新独立 build 中运行现有回归，重新记录测试数量、跳过项和耗时；AGENTS 中 86 项结果是历史记录，不直接当作本 worktree 的结果。
3. 选定有限且可复现的合成、实采回放夹具，记录散列、profile、输入长度和命令。通过显式参数引用原目录只读夹具，或复制必要的小夹具；不把源工作区硬编码进产品。
4. 保存逐包 PSDU、FCS、状态、包起点、SYNC/SFD/CFO/SPC、窗口几何、锁定状态及队列守恒。性能字段单独比较。

交付：基线记录、依赖表、回放比较字段与数据定位，不改算法。

### P1：拆除头文件与构建耦合

涉及：顶层/库/apps/include CMake、`api.h`、`uwb_radar_pdu_meta.h`、`uwb_hrp_mod_core.h`，新增纯配置及导出头。

1. 为原生构建提供不执行 GR 模块的 CMake 路径，建议 `UWB_BUILD_GNURADIO=OFF`；adapter 启用时才发现 GR 并加载其安装/测试/Python 宏。
2. 独立发现 VOLK/Threads；UHD 按现有 AUTO/ON/OFF 语义发现。原生可在 UHD=OFF 下运行回放和 QA。
3. 拆分纯 profile 常量/校验与 PMT 映射，不复制两套常量；保留旧头作为 adapter 兼容入口。
4. 引入核心符号导出定义；静态、共享构建和原适配层均检查符号可见性。
5. 分离默认/可选组件的 install/export/package config；原生包配置不得隐含查找 GR。

验收：无 GR 开发头的环境可配置并编译核心；adapter 开启时仍可构建。仅设置查包禁用变量不足以证明头文件独立，还须环境隔离验证。

### P2：拆库并接通原生应用

涉及：`lib/CMakeLists.txt`、`apps/CMakeLists.txt`、pipeline/单音数据源头文件及实现、SIC 源文件。

1. 将核心源文件编入 `uwb_core`，原生源文件编入 `uwb_native`；adapter 链接这些目标。避免同一导出实现重复出现在多个共享库。
2. 将 `ISc16StreamSource`、读取结果/统计的最小契约独立；UHD 源实现可从 pipeline 翻译单元分离，保持 open/start/stop/时间戳/gap 语义。
3. live、capture、pipeline runner 和必要回放工具改为直接链接原生目标，不链接 `gnuradio-uwb`。
4. 核心 QA 迁移到普通 CMake/CTest + 独立测试依赖；GR QA 仅在 adapter 模式注册。
5. 将 taps/template 等运行资源与 QA 大数据区分；处理当前 live 的源码绝对路径 `UWB_TESTDATA_DIR`。以显式资源路径或可安装资源目录支撑原生程序，禁止依赖旧 build/RPATH/PYTHONPATH 才能运行。

验收：新 live、runner 及其原生自有库的传递动态依赖无 `libgnuradio-*`/PMT；核心公共头不含 GR 外部头；旧 adapter 回归仍通过。

### P3：删除明确废弃的实验入口

按独立提交执行，每项后运行对应 QA 和回放比较：

1. 删除 C2 `convert_cache`、组合 `both`、专属缓存/内核/统计及相关无效 QA 矩阵，保留 fixed_phase 的正确性覆盖。
2. 删除原生 `--resampler-workers`、配置字段和误导性输出；旧参数应返回明确错误，不能静默忽略。
3. 历史 benchmark/诊断归入可选实验构建；`rx_diagnose_fir_segments` 从公共生产接口迁出。其 QA 归入实验模式，不能悄悄减少默认功能验证。
4. 迁移 GNU Radio 65/48、65/32 及 PDU 接口对内部线程池的调用后，再删除 pool、run_mt、逐输出调度表；受影响 API 明确兼容/报错策略。不能仅因 UHD 不使用而直接删除其他调用方。
5. 删除生产 `scalar_legacy/volk_legacy` 循环；修订基准、TWR 验证工具和 QA，保留独立 float64/golden、流式分块及长度公式验证。

首轮可先交付 1–3；4–5 如兼容范围较大，单独完成，不能提前宣称已清理。

### P4：调度公共化（后续独立重构）

live 自有 Workers 与 UhdSc16Pipeline 在窗口池、Ready、队列及 worker 生命周期上重复，但业务窗口/提交语义不同。先设计可复用执行器契约，再分步迁移；不得将目标锁反馈、多消费者 SIC 和 timed 间隙处理硬塞成同一业务逻辑。

单独验证 Ready 屏障、非阻塞满载策略、有界内存、异常退出、提交顺序、gap 传播和队列守恒。P1/P2 的 GR 解耦不等待本阶段完成。

## 6. 保留路径和行为边界

- 宽捕获/重捕获与 LockedBounded 同时保留；locked ROI 失败不能偷偷回宽搜。
- 分数 SPC、完整 CIR、稀疏 RAKE、Bypass 和必要失败回退不因“旧路径”命名而删除。
- 融合 CMF 有适用范围，保留；若优化支持条件预检，作为额外行为改动独立验证，不混入纯拆库。
- DiscreteCir、PathFractional、Ipatov 后端保留，不能用非碰撞收益取代碰撞场景证据。
- scalar/portable SIMD 回退、65/32、回放源和原 GR 回归能力保留。
- timed 无 replay 仍 fail-closed；timed 单音调度和真实 UHD finite RX 未接通的限制原样报告。
- 不因出现 PMT 头就删除元数据的时间、缩放、群时延及质量检查；只移动适配代码。

## 7. 验证矩阵和验收门槛

| 构建/场景 | 必须验证 |
| --- | --- |
| 原生、GR=OFF、UHD=OFF | 在无 GR 开发头/运行库环境配置、构建；合成/回放及核心 QA 通过 |
| 原生、GR=OFF、UHD=ON | 编译并链接设备后端；原生 apps 的传递依赖无 GR/PMT。没有上板不得宣称硬件验证 |
| GR adapter=ON | 原 GR block/Python 及原生测试按依赖注册并通过；跳过原因明确 |
| 安装消费者 | 临时前缀内安装并运行最小 C++ 消费者；不使用源目录、旧 build 或 GR 头；无需全局 install |
| 输出一致性 | 同编译选项下 IQ/重构输出优先逐位一致；逐包 payload/FCS/状态/窗口/锁定字段一致。若数值差异必须单独归因并审批验收口径，不能自动放宽 |
| 调度/失败 | 小容量池、满载、EOS、不连续、初始化失败和取消；守恒律、退出码与 fail-closed 均保持 |
| 性能 | 相同 ISA/优化参数、同夹具、独占基准锁；避免拆库意外丢失 AVX2 或跨目标优化。测得变化如实报告 |

使用 `readelf -d` 检查直接 NEEDED，结合 `ldd` 或等价递归工具检查传递依赖；保留头文件编译依赖记录。最终“无 GR”必须同时满足构建、头文件、运行时和安装消费者四个层面。

已有算法检测的 MATLAB 交叉验证要求继续适用；本轮未执行时列为未验证。纯依赖拆分优先用逐样点/逐包等价证据，不冒充新算法或硬件验收。

## 8. 提交和交付顺序

1. 保存复制而来的既有功能基线，明确其自身验证状态。
2. P0 基线与独立构建合同。
3. P1 头文件/CMake 解耦。
4. P2 拆库、原生应用与 QA、运行资源/安装消费者。
5. P3 每类删除分别提交，保留验证记录。
6. P4 独立方案与实施，不并入首轮解耦验收。

最终交付：变更清单、构建命令/组件选项、原生依赖报告、QA 与回放对照、资源准备说明、删除入口及兼容影响、未验证清单。历史报告留档，开发报告以新构建实测结果为准。
