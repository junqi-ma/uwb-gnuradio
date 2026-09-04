# Review：UWB 自发自收 Radar Step 1–2

> Review 日期：2026-09-04  
> 被审查报告：[开发报告_UWB自发自收Radar_Step1-2.md](开发报告_UWB自发自收Radar_Step1-2.md)  
> 被审查提交：`41aa698..487da0b`  
> 结论：**暂不通过，整改后复验；在 Step 1–2 退出条件满足前，不正式进入 Step 3。**

## 1. Review 范围

本次只验收：

1. Step 1：固定正常 UWB TX packet、998.4/737.28 golden、整数/分数时延 RX 与 CIR golden；
2. Step 2：已知 TX 时刻附近的 SFD 有界窄窗搜索 core；
3. 构建、专项 QA、全量回归以及 MATLAB 对照是否满足开发计划的退出条件。

不验收 GNU Radio Radar block、EchoTimer/UHD/X410、CIR writer 或 Phase A 端到端。

## 2. 验收结论

| 项目 | 结论 | 说明 |
|---|---|---|
| Step 1 | **不通过** | 当前入库的是 Python 拼接包，不是 generator-of-record 生成的正常 packet；缺少分数时延 CIR；MATLAB CIR verifier 会吞掉失败 |
| Step 2 | **不通过** | stride-8 粗搜存在可稳定复现的无噪漏检；纯噪声 QA 没有断言必须失败 |
| 编译 | 通过 | `cmake --build gr-uwb/build -j4` 成功 |
| Radar SFD 专项 CTest | 表面通过 | `uwb_qa_uwb_radar_sfd_core.cc` 通过，但 QA 未覆盖已发现的粗搜盲区 |
| Python golden 自检 | 通过 | 只能说明 Python 文件和 Python metadata 自洽，不能代替 MATLAB/标准 packet 验收 |
| MATLAB generator | 可运行 | 本机 MATLAB R2025b、Communications Toolbox 可用，已在临时目录成功生成 BPRF/SFD#2 packet |
| 全量 CTest | 17/18 | 既有 X410 overflow 转发检查失败；不是本次 radar 提交引入，但仓库“全绿”门槛仍未满足 |

当前实现中以下设计可以保留：

- SFD core 是纯算法而不是 GNU Radio block，调度边界清楚；
- 搜索域严格裁剪在调用方给定的 margin 内；
- 不做 CFO 估计或补偿；
- 正常失败路径返回 `SfdFailed` 和 `sfd_start_sample=-1`，没有用预测位置冒充检测结果；
- `prepare_sfd_template` 与 hot-path `search_sfd` 分离，`search_sfd` 中未发现显式动态分配；
- 4z2 序列及 `kron(4z2, SYNC)` 模板与 MATLAB BPRF/SFD#2 波形在正确起点能够高相关。

## 3. 阻断问题 P0：stride-8 SFD 搜索会漏掉真实 SFD

### 3.1 问题位置

`uwb_radar_sfd_core.h` 当前使用：

```text
stride-8 coarse search
  → 只选择一个最佳 coarse 点
  → 仅在该 coarse 点附近 ±7 样点 full-rate refine
```

实现注释假设“2x oversampled SFD peak is broad”，该假设对当前 UWB 成形模板不成立。

### 3.2 实测结果

使用 `testdata/reference_preamble.bin`、`GetSfdSequence("4z2")`、无噪声、完全匹配模板，在 `margin=64` 内穷举真实 SFD 相对预测位置的全部 129 个整数偏移：

| 输入 | 结果 |
|---|---:|
| C++ core + 合成完全匹配 SFD | **46/129 个偏移检测失败** |
| 同一搜索逻辑 + MATLAB R2025b 标准 BPRF/SFD#2 golden | **45/129 个偏移检测失败** |

典型相关 metric：

| 相对真实起点偏移 | metric |
|---:|---:|
| 0 | 约 1.0 |
| 1 | 约 0.386 |
| 2 | 接近 0 |
| 3 | 约 0.030 |
| 5 | 约 0.0057 |

因此粗网格可能完全踩不到主峰；此时算法会选择远处的弱旁瓣，随后的局部细搜也不会覆盖真实起点。现有 `predicted ±37` 和左右边界用例只是恰好没有覆盖全部粗网格相位。

### 3.3 必须整改

先建立正确性基线，再做性能优化：

1. 第一版建议直接对窄窗内所有整数起点做 full-rate 搜索；
2. 新增表驱动 QA，穷举 `true_start-predicted ∈ [-margin,+margin]`，每个偏移都必须找到真实起点，误差不超过 1；
3. 对合成完全匹配模板和 MATLAB canonical golden 各做一次上述穷举；
4. 再叠加多个固定种子的 AWGN 梯度，记录检测率、误差和 false alarm；
5. 若 full-rate 窄窗搜索性能不足，再单独优化。任何 stride/decimation 优化都必须保持上述全偏移 QA 通过，不能只验证少数偏移。

不建议只把 stride 从 8 改成另一个常数后结束，因为多径和噪声下仍需通过完整 QA 证明不存在同类盲区。

## 4. 阻断问题 P0：正式 TX golden 不符合 Step 1 身份要求

### 4.1 当前文件的真实身份

当前入库 `testdata/uwb_radar/*.cf32` 由 Python fallback 生成：

- 从旧的 128-byte 测试 cfile 截取；
- 保留 64 个旧 SYNC；
- 用 `kron(4z2, first_SYNC)` 替换原 IEEE SFD；
- 保留旧 PHR/PSDU；
- 原源包是 127-byte 标准 PSDU 加 1-byte pulse-level 非标准扩展。

它适合做 synthetic fixture，但不满足开发计划中的“固定合法 PHR/PSDU/FCS、完整 normal UWB packet”要求。

### 4.2 MATLAB 实测

本机实际存在：

- MATLAB `25.2.0.2998904 (R2025b)`；
- Communications Toolbox license；
- `lrwpanHRPConfig`。

`export_uwb_radar_golden.m` 已成功在临时目录生成：

| 波形 | 998.4 MS/s 长度 |
|---|---:|
| MATLAB BPRF、SFDNumber=2、20 data bytes + FCS | **190912** |
| 当前仓库 Python fallback | **256448** |

两者显然不是同一个 packet，不能用 Python 自检替代 MATLAB canonical golden。

### 4.3 必须整改

1. 用 `export_uwb_radar_golden.m` 重新生成并提交 canonical golden；
2. canonical metadata 必须明确记录 `generator=export_uwb_radar_golden.m`、MATLAB 版本、profile、payload/FCS、坐标和滤波信息；
3. Python fallback 若保留，必须使用独立目录或不同文件名，例如 `uwb_radar_synthetic/`，不得与 MATLAB canonical 文件互相覆盖；
4. C++ canonical-golden QA 不得 optional skip，并应检查 metadata 的 generator/profile；
5. 重新生成后，C++ SFD QA 必须直接读取 MATLAB RX golden，而不是只读 Python 拼接波形。

## 5. 阻断问题 P1：MATLAB CIR verifier 会错误打印 PASS

`verify_uwb_radar_golden.m` 当前把整个 `compare_cir_if_possible` 放在 `try/catch` 中。以下情况都会被当成“skip”并最终打印 `PASS`：

- toolbox/internal symbol 探测错误；
- 文件读取错误；
- tap 数错误；
- complex CIR 数值误差超过门限；
- `estimateCir` 本身抛出异常。

本次实际运行结果为：

```text
skip CIR vs estimateCir: Communications Toolbox HRPCodes unavailable
PASS
```

但同一次 generator 已成功调用 `lrwpan.internal.HRPCodes`，说明 verifier 的 `exist(...)` 探测方法不可靠。

### 必须整改

1. generator-of-record 环境中，CIR 对比必须是 hard gate，任何异常或超差都返回非零/抛错；
2. 不要捕获数值 comparison 的断言后继续 PASS；
3. 至少验证 clean、整数时延、分数时延的 normalized complex CIR；
4. normalized CIR 使用复数相对 L2 误差 `<1e-5`，或文档化与现有 demod golden 同级的明确容差；
5. raw CIR 单独检查幅度线性，不能用 normalized 结果代替；
6. verifier 最终只允许输出明确的 `PASS` 或 `FAIL`。如果环境缺依赖，应输出 `SKIP/UNAVAILABLE` 并以非验收状态退出，不能输出 PASS。

## 6. 阻断问题 P1：缺少分数时延 CIR golden

开发计划 Step 1 要求已知整数/分数时延 RX 同时包含：

- SFD 位置；
- SYNC origin；
- raw complex CIR；
- normalized complex CIR。

当前 Python 和 MATLAB generator 都只导出了 fractional-delay RX IQ，没有 fractional-delay CIR 文件。

必须补齐分数时延 CIR，并在 metadata 中记录：

- fractional delay 与复增益；
- interpolation 方法；
- CIR pre/post、tap count、有效 repetition 数；
- peak/sub-sample 参考约定；
- 与 MATLAB `estimateCir` 的 complex L2 对照结果。

注意：Python `ndimage.shift(order=3)` 与 MATLAB `interp1(...,'pchip')` 不是同一插值核。canonical fractional golden 应以 MATLAB 路径为准，或明确规定唯一的跨语言参考模型。

## 7. QA 问题 P1：纯噪声误检时测试仍会通过

`test_radar_sfd_awgn` 当前逻辑为：

```cpp
const bool ok = search_sfd(...);
if (!ok) {
    // 检查失败状态
}
```

如果纯噪声被误检为成功，测试不会执行任何失败断言，因此仍然通过。

必须直接断言：

```cpp
BOOST_REQUIRE(!ok);
BOOST_CHECK(buried.status == SfdStatus::SfdFailed);
BOOST_CHECK_EQUAL(buried.sfd_start_sample, int64_t(-1));
```

建议使用多个固定 seed，并把“纯噪声 false alarm 为 0”与“含 SFD 的 AWGN 检测成功”拆成两个测试用例。

## 8. 建议同时补齐的输入契约 P2

以下不一定单独阻断 Step 2，但应在 core 对外使用前补齐：

1. `threshold` 必须检查 `std::isfinite`，并限制在合理范围；当前负数或 NaN 可能导致无效输入返回成功；
2. `prepare_sfd_template` 应拒绝零能量或非有限模板，不能对全零模板归一化后仍返回 true；
3. 增加上述 invalid-input QA；
4. hot-path QA 当前只检查 vector capacity 不变。代码审查确认 `search_sfd` 没有显式分配，但若后续重构，应考虑用 allocation counter 或预分配契约测试防止回归。

## 9. CIR 坐标与 +2 tap 的决定

当前 sampled-code CIR 对 pulse-shaped packet 的 clean peak 位于 `pre+2`，不建议通过修改 expected 字段把它简单解释成物理零距离。

后续契约应区分：

1. **算法 tap 坐标**：C++ 必须逐复数 tap 对齐 MATLAB `estimateCir`；如果 MATLAB 输出峰在 `pre+2`，C++ 也应一致；
2. **物理距离坐标**：通过 `calibration_delay_work_samples` 或等价字段扣除 TX pulse shaping、48/65、65/48 和 RF/数字链路的确定性群时延；
3. 已知信道额外时延 `D` 应表现为相对 clean CIR 的峰移 `D`，当前整数时延 37 的相对峰移验证是正确的；
4. 不允许把一次测试测得的 `+2` 隐式硬编码成所有 profile 的常数，应由模板/profile 或校准 metadata 明确给出。

## 10. `sfd_search_margin=64` 的决定

`64 @ 998.4 MS/s` 可以暂时作为软件开发默认值，但暂不冻结为硬件验收参数。

冻结前必须测量或计算：

- TX/RX 固定时序误差；
- 48/65 TX 和 65/48 RX 的坐标映射与残余群时延；
- EchoTimer/UHD/device-time 调度误差；
- 校准后的剩余不确定度和安全余量。

无论 margin 最终取多少，SFD 搜索都必须覆盖窗内每一个合法整数起点，不能再依赖 stride 网格碰巧命中。

## 11. 全量回归状态

本次实际执行：

```bash
cmake --build gr-uwb/build -j4
ctest --test-dir gr-uwb/build -R uwb_qa_uwb_radar_sfd_core --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
```

结果：

- build：成功；
- Radar SFD 专项：通过；
- 全量：17/18；
- 失败项：`uwb_qa_uwb_auto_scheduled_extractor_sc16.cc/test_x410_app_forwards_uhd_overflow`，检查应用源码中 `uhd_async_msg` 失败。

`41aa698..487da0b` 没有修改该失败测试或对应 X410 app，因此它不是本次 radar delta 引入。但项目需求写明“现有 CTest 必须全绿”，在最终合并/阶段验收前仍须处理或形成经确认的基线豁免，不能把 17/18 写成全绿。

## 12. 给 Grok 的整改顺序

建议保持小提交、逐步验证：

### R1 — 修正 SFD 搜索正确性

- 先改为无盲区的窄窗搜索；
- 增加全部 129 个整数偏移 QA；
- 修正纯噪声断言；
- 补 threshold/零模板输入检查；
- 运行 Radar QA 和 demod core 回归。

### R2 — 建立 MATLAB canonical golden

- 用 MATLAB generator 生成正式 normal packet；
- Python synthetic fixture 与 canonical 分离；
- 添加 generator/profile 身份检查；
- 让 C++ QA 必须读取 canonical RX golden；
- 验证 native round-trip 坐标无累计漂移。

### R3 — 修正 MATLAB verifier 并补齐 fractional CIR

- CIR mismatch/异常必须 hard fail；
- 导出 clean、integer、fractional 的 raw/normalized CIR；
- 对 `estimateCir` 做逐 complex tap 对照；
- 记录复数 L2、峰位、相对峰移、raw 幅度线性结果。

### R4 — 复验 Step 1–2

只有 R1–R3 全部满足退出条件后，更新开发报告并申请复验。复验通过后再开始 Step 3 的 SYNC 回推与 C++ CIR core。

## 13. 复验硬门槛

下次报告必须给出以下可复现证据：

- [ ] canonical TX 是 MATLAB/已验证工具生成的正常 UWB packet，固定合法 PHR/PSDU/FCS；
- [ ] 998.4 完整 packet 一次性 48/65，未重复 751-sample SYNC；
- [ ] clean/integer/fractional RX 均有 raw 和 normalized CIR；
- [ ] MATLAB verifier 对 CIR comparison 不允许 skip 后 PASS；
- [ ] SFD 在 `[-64,+64]` 全部 129 个整数偏移下无噪检测成功，误差 ≤1；
- [ ] MATLAB canonical golden 上执行同样的全偏移验证；
- [ ] 缺失、破坏、错误模式和纯噪声均 100% 返回 `SfdFailed`、start=-1；
- [ ] normalized complex CIR 对 MATLAB 相对 L2 达到计划门限；
- [ ] raw CIR 幅度线性通过；
- [ ] `search_sfd` hot path 无动态扩容；
- [ ] Radar 专项、demod 回归通过；
- [ ] 全量 CTest 全绿，或对既有失败有明确、经确认的基线处置结论；
- [ ] 不宣称 Phase A、EchoTimer/UHD 或 X410 已验收。

## 14. 对原报告六个问题的正式答复

1. **Python golden 能否作为 Step 3 输入？** 只能作为 synthetic/smoke fixture，不能作为 MATLAB 对齐或正式验收 golden。
2. **SFD C++ 是否满足 Step 2？** 否。stride-8 已证实存在无噪漏检，且缺少强制 MATLAB canonical QA。
3. **`kron(4z2,SYNC)` 拼接包能否作为一期 TX？** 不能作为正式 TX；可以保留为明确标注的合成测试数据。
4. **CIR expected 是否直接改成 `pre+2`？** 不直接改成物理零点。算法结果先对齐 MATLAB，再通过显式 calibration delay 映射物理距离。
5. **margin=64 是否冻结？** 仅作临时软件默认；硬件链路群时延和调度误差标定后再冻结。
6. **是否允许进入 Step 3？** 暂不允许正式进入。先完成 R1–R3 并复验 Step 1–2。

