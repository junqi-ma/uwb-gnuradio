# UHD 原生链路 GNU Radio 解耦：主代理独立验收

日期：2026-11-15。状态：本轮 P1/P2 与 P3.1–3 开发及独立验证完成；动态 live 锁定逐包一致性仍有明确限制，见 §5/§6。

## 1. 执行分工与源码范围

用户授权 OpenCode 在 `/tmp/uwb-gnuradio-native-cleanup` 开发，主 Codex 独立测试。OpenCode session 为 `ses_e205e058fffeJ4n8lfF3DWoxQd`；默认服务额度耗尽后，经用户明确授权改用 Mistral。

开发规划见 `UHD原生链路_GNURadio解耦与历史路径清理修改规划_2026-11-15.md`。既有未提交功能基线见 `native_cleanup_baseline_20261115.json`，不是本轮重构成果。

冻结对照树为 `/tmp/uwb-native-cleanup-baseline`：由 a8f0016 的 Git archive 与清单所列 14 个文件组合，逐文件 SHA-256 校验通过，不依赖开发 session 后续编辑。

## 2. 基线构建与完整回归

```sh
cmake -S /tmp/uwb-native-cleanup-baseline/gr-uwb \
  -B /tmp/uwb-native-cleanup-baseline/build \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
cmake --build /tmp/uwb-native-cleanup-baseline/build -j 3
```

完整串行 CTest：**86 项，85 通过，1 跳过，0 失败；108.96 秒**。跳过项为未配置安装前缀的 `uwb_qa_install_consumer`。

首轮七项失败均为环境/定位问题，后续逐项及完整复测已通过：

- 两项 legacy demod QA 读取 `../../../testdata`，要求特定 build 深度。仅在冻结树的生成 CTest 文件中修正运行目录及脚本绝对路径，未修改测试断言或算法源码。
- 四项 apps/Python 测试继承的 `LD_LIBRARY_PATH` 指向旧安装库，引起 undefined symbol。完整复测清除 `LD_LIBRARY_PATH` 和 `PYTHONPATH`。
- 一项 provenance QA 在 archive 树无法读取 revision；以 `UWB_REPO_DIR` 指向原始 Git 仓库提供版本元数据。实际冻结文件由 SHA-256 清单标识，不能仅用 revision 代表所有未提交代码。

修正环境后的完整日志：`/tmp/uwb-native-cleanup-baseline/ctest-clean.log`。这些环境问题必须在重构验收中单独检查，不能靠少注册测试来隐藏。

## 3. 实采回放基线

输入：原工作区 `testdata/x410_fixed_tone_live_20261115/rx_ch3_rx1_100ms.sc16`，只读访问。参数：4 workers，tone on，cir auto。

| 同步策略 | 完成包 | FCS | 原始样点 | 丢样/source errors/gap |
| --- | --- | --- | --- | --- |
| acquire | 20 | 20/20 | 73,728,000 | 全 0 |
| locked | 20 | 19/20 | 73,728,000 | 全 0 |

仅作为这份输入和配置的对照，不将其解释为 PER 或硬件验收。逐包日志保存在新 worktree 的 `.opencode-task/validation/baseline-live-*.log`。

另用通用 pipeline runner 对同一输入截取 8 个窗口，保存约 32 MB post-resample IQ 和 JSON 功能记录。用于后续逐字节和逐窗字段比较，不以该截窗实验的 SIC 抑制数字评价算法质量。

## 4. 最终验收安排

尚待开发结束后执行：

1. 原生 GR=OFF/UHD=OFF 的隔离环境配置、构建、核心 QA。
2. 原生 GR=OFF/UHD=ON 的配置、构建、核心 QA与真实回放。
3. GR adapter=ON、实验组件开启的完整兼容回归。
4. post-resample IQ 逐字节比较、逐包功能字段及锁定契约比较。
5. 临时前缀安装与脱离源码/build 的消费者、资源定位验证。
6. 原生 ELF 直接及传递依赖，公共核心头依赖和被删除 CLI 的明确报错。

无 GR 环境由 `.opencode-task/validation/without_gnuradio.py` 使用 bwrap 实现：遮蔽 GR/PMT 开发头、GR CMake 包和安装的 libgnuradio ELF 文件；该隔离机制已实测可用。未修改或卸载系统安装。

尚未进行上板、全局 install、推送、MATLAB 新交叉验证或性能提升验收。

## 5. 首轮主代理独立验收结果（修复待复核）

两组从空 build 开始的 bwrap 隔离构建成功，构建过程隐藏系统 GR/PMT 头、GR CMake 配置和全部已安装 `libgnuradio*.so*`；不是仅关闭一个构建开关。

| 配置 | 独立 CTest | 备注 |
| --- | --- | --- |
| GR=OFF/UHD=OFF/实验=OFF | 48 项，47 通过、1 安装门跳过，0 失败，19.64s | 尚未配置本组安装前缀 |
| GR=OFF/UHD=ON/实验=ON | 52 项全部通过，0 失败，21.73s | 原生安装消费者实际执行通过 |

原生模式首版漏注册 live console QA，主代理在同一隔离环境下对两组二进制手工执行该 QA，均通过；已交回 subagent 修复注册条件。

对上述两组的 live、capture、pipeline runner 共 6 个 ELF 检查传递动态依赖：无 GR/PMT，无缺失库。ON 保留 UHD，OFF 不加载 UHD。

### 固定输入比较

- pipeline：8 窗 post-resample IQ 共 **32,623,872 字节逐字节相同**。
- pipeline：12 个非计时功能字段/窗（含状态、窗口几何、SIC 提交、抑制值和 radar 字段），8 窗精确相等。
- 独立固定窗口 demod 对照程序：两边以相同 `-O3 -march=native` 编译头文件核心，对相同 8 窗输入运行；包状态、起点、SYNC 数、measured period、CFO、SFD/PHR/FCS 及**完整 PSDU 字节**的输出文本完全相同。其中 6 窗解出 127 B 且 FCS 通过，2 窗失败状态也相同。这些窗口含未消除单音，不作为该场景解调率验收。

### 实采 live 调度波动

100ms/4-worker 的动态 live 回放没有达到逐包锁定字段完全相等：首轮 acquire 两边 20/20 FCS，但初始锁定切换的时刻/任务 ID 不同；locked 首轮基线19/20、新18/20。随后3次交替配对，基线为19/20、18/20、19/20，新构建为18/20、18/20、18/20；全部完成20包、无丢样/source error/gap。

已观察到冻结基线本身也有初期锁定波动，因此不能把单次动态 live 差异等同于算法数值变化，也不能宣称动态锁定验收已逐包一致。固定窗口的完整 PSDU 对照与 IQ 一致性支持核心数值未改变；动态锁定稳定性保留为独立限制，不在依赖拆分中擅自调整 ROI、锁定门限或算法来掩盖。

### 已交回修复的问题

1. GR 版本变量初始化晚于 `GrVersion`，产生 `...0 / ..`。
2. 安装包配置只 `find_dependency(UHD)`，本机 UHD 包没有导出 `UHD::UHD`，导致外部 CMake 消费者生成失败；手工链接消费者通过不能替代导出目标验证。
3. 原生模式 `if(ENABLE_TESTING)` 导致 live console QA 漏注册。

删除入口已验证：`--resampler-workers` 和 `--resampler-experiment convert_cache|both` 均返回2并明确报错。

证据位于 `.opencode-task/validation/`。以上为首次独立验收记录，最终结果以修复后的后续章节为准。


## 6. 修复后的最终结果

OpenCode 完成开发及两轮针对性修复，主 Codex 独立执行构建/测试、补正 pkg-config 导入目标名，并核对记录。没有提交、推送、全局安装或上板。

| 独立构建 | 测试注册数 | 最终通过/跳过/失败 | 说明 |
| --- | --- | --- | --- |
| GR=OFF、UHD=OFF、实验=OFF | 49 | 48 / 1 / 0 | GR/PMT 头、配置、ELF 隔离；live QA 已纳入 CTest；19.35s |
| GR=OFF、UHD=ON、实验=ON | 53 | 53 / 0 / 0 | 相同隔离，安装消费者实际执行；22.54s |
| GR=ON、UHD=ON、实验=ON | 87 | 87 / 0 / 0 | 全套首次运行其余项通过；版本测试刷新生成脚本、原生消费者改用原生前缀后两项复测通过。完整日志与修正项日志均保留 |
| GR=ON、UHD=OFF、实验=ON | 86 | 84 / 2 / 0 | 两个未配前缀的安装门跳过；首次版本测试使用开发中间态生成命令，刷新 CMake 后该项复测通过；完整首轮107.51s |

表中 GR 两组是完整首轮与针对性失败项复测的合并结果，并非两次完整单轮全绿；修改仅涉及最终测试注册/生成与前缀参数，未改变已通过的 C++ 内核。证据：`gr-{on,off}-ctest.log` 与 `gr-{on,off}-ctest-corrected-gates.log`。

### 安装包与资源验证

1. 无 GR 隔离环境中，外部工程通过 `find_package(uwb CONFIG REQUIRED)` 和 `uwb::uwb_native` 完成配置、编译、运行；仅引用临时安装前缀。
2. UHD=ON 的正常 config 发现与禁用 UHD CMake 后的 pkg-config-only 两种方式均完成消费者构建/运行。主代理修正导入目标名为 `PkgConfig::UHD_PC`，并以 target property 检查实际使用该接口。
3. UHD=ON 前缀且 config/pkg-config 均不可用：REQUIRED 查找按明确缺依赖信息失败；可选查找正常返回 `uwb_FOUND=FALSE`，没有空的 UHD/native imported target。
4. UHD=OFF 前缀的外部消费者独立编译运行通过，不需要 UHD 查找。
5. 安装后的 live 在隐藏原工作区、冻结基线和新 worktree 源码、并隐藏 GR 安装的条件下，用安装资源默认路径和极短合成回放正常启动、结束。此项只证明资源定位与启动退出，不证明该短输入能解码。
6. 原有 GR 安装消费者（C++/Python）在临时 GR 前缀通过；原生消费者使用单独原生前缀。当前 `uwb` 原生 CMake 包由 GR=OFF 构建安装，GR 安装前缀使用其原有 `gnuradio-uwb` 包，二者不能混用。

### 数值与行为结论

§5 的 8 窗 IQ、12 字段/窗、固定窗口完整 PSDU 对照全部精确一致。删除 CLI 的错误码/诊断通过，6 个原生 app 的 ELF 闭包无 GR/PMT。锁定/timed 核心与对应 QA、AGENTS 基线文件逐字节未变。

动态 live 的逐包一致性没有通过，不能把固定窗口证据扩大为实时锁定验收。补充1-worker对照：冻结基线17/20、新18/20，无丢样/gap；与4-worker配对一起说明基线本身有调度相关波动，但仍不足以证明重构对动态尾部完全无影响。后续如要关闭此项，应对固定到达/反馈顺序做独立调度复现，不调整默认 ROI 来掩盖失败。

### 本轮范围与剩余工作

- 完成 GNU Radio 依赖解耦、core/native/adapter 拆分、独立 QA、临时安装消费者和资源定位；删除 C2/失效参数并隔离一部分历史实验。
- P3.4 内部线程池、P3.5 legacy 数值对照循环仍保留，需迁移 GR 调用方及独立参考后另行删除；P4 两套任务调度公共化尚未实施。
- FIR 分段诊断只从公共 PHY profile 迁至 scratch 开关，诊断头与实现仍在 demod core；不宣称已彻底迁出核心。
- 未进行上板、MATLAB 新交叉验证、性能提升验收；不宣称均值或尾延迟提升。

独立验证产物保留于 `.opencode-task/validation/`（已忽略，避免把 IQ、构建日志和 session 内容误加入 Git）。两个文档分别记录开发自测和主代理验收，最终边界以本记录为准。

补充二进制兼容边界：`nm -D --defined-only` 确认冻结基线的 GR 共享库导出 `UhdSc16Pipeline`，拆分后的 adapter 不再导出该类的全部实现。GR block/Python 重新构建回归通过；原生 C++ 旧消费者需重新链接 `uwb_native`/`uwb_core`，不承诺直接替换旧共享库的原生 ABI 兼容。
