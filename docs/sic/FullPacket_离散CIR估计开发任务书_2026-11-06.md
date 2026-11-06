# Full-packet SIC：直接估计离散等效 CIR

## 1. 目标与执行范围

用前 8 个 SYNC 直接估计整数采样网格上的复数 FIR 系数，替换 full-packet 路径的“分数路径估计 → 分数延迟波形 → 路径累加”。保持快速通信解调、FCS 门槛、整包重调制、VOLK CFO 旋转与事务相减。评估消除效果、通信/感知解码率和总 SIC 延迟，不能只比较卷积耗时。

本文件是下一轮执行任务书；本次交付只编写计划，不实现算法。执行时主 agent + 3 个 subagent，按第 4 节调度，无须逐阶段等待用户确认；失败则输出证据与下一步，不隐瞒失败或自动降低验收标准。

- 工作目录 `.worktrees/sic-fullpacket-compare`，分支 `feature/sic-fullpacket-compare`。
- 已提交基线 `887e7bb`；生产算法 `1adf401`，分段诊断 `32473f2`。保留旧路径、旧 build 与历史 run。
- 新算法仅作为 `discrete_cir` 显式实验选项；旧 `path_fractional` 与原 preamble-only 对照保持。通过后才能建议改默认。
- 不改冻结 preamble 核心、adapter、`tools/sic/sic_reference.py`；不改硬件、TWR、resampler。不新增 GNU Radio block。
- 新 build：`gr-uwb/build-sic-fullpacket-discrete`，从当前源码独立配置，不复制旧 cache。

## 2. 原理与必须保持的数学约定

参考 x 是重调制器输出的完整、已脉冲成形 work-grid 波形，不是 chip 序列，也不是相关器输出。定义整数包起点 q、有符号整数 lag ℓ、CFO ω 与绝对相位锚点 r：

    y_hat[n] = exp(j*ω*(n-r)) * Σℓ h[ℓ] x[n-q-ℓ]

h 是相对于这个 x 的离散等效信道，吸收分数传播时延与复增益；不能把相关 CIR 直接当 h，也不能将原路径时延四舍五入。K 条物理路径通常不等于 K 个离散 tap。新路径不从旧 tau/gain 合成 h，否则仍是第一条路线。

只用前 8 SYNC 的合法观测构造训练问题：

    z[n] = y[n] * exp(-j*ω*(n-r))
    X[n,ℓ] = x_sync_train[n-q-ℓ]
    h = argmin ||Xh-z||² + λ ||h||²

- 严格限制所有检测、CFO 与拟合读取在 observed_end 之前；训练行只能使用已知训练模板支持，处理实际起始瞬态，不能任意循环延拓。后续 SYNC/PHR/payload 不反馈估计。
- q、ω 来自混合 IQ 与公开 ROI。新增独立 acquisition helper，参考已有检测/CFO逻辑，只运行所需步骤；不改冻结源码。可临时用旧 estimate_model 验证接口，但全部成本必须计入，最终交付需移除旧分数路径搜索/增益拟合依赖。
- 第一版采用连续有限 lag 支持，包含负 lag 的分数时延前沿；S0 阶段根据公开最大延迟扩展、粗定位误差与模板支持确定边界。支持宽度须小于一个 SYNC 周期，并报告设计矩阵有效秩/条件性。重复 8 次增加 SNR，不自动增加可辨识维度。
- 默认用 double 精度正则化最小二乘。缓存公开模板对应的矩阵/分解（以模板、lag、训练行、λ 为完整 key）；按包计算 Xᴴz 和求解，不显式求逆，不为每包构造大矩阵。λ 用独立校准固定，禁止用验收窗口选最优 λ/长度。
- Sensing 与噪声不是待拟合目标；短支持与正则化用于限制过拟合。既测残余干扰，也测弱目标保护。模型不可辨识、非有限、支持不完整等情况显式旁路，不静默退回旧算法掩盖失败。
- 拟合与整包重调制必须使用同一幅度归一化；h 已包含的缩放不得再乘一次。CFO 只在上述位置应用一次，保持绝对锚点。
- 全包按有限线性卷积处理，字段之间共享正确 halo，包外参考为零。新后端第一版全包统一卷积，无前缀相位行缓存；允许缓存模板矩阵，不能跨包缓存 h。负 lag、包尾、ROI 截断均须明确。

## 3. 接口与实现边界（S0 冻结后并行）

新增 `gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h`，至少定义：

- `DiscreteCirConfig`：公开训练/搜索范围、lag_lo、tap_count、λ、门控和 profile。
- `DiscreteCirModel`：valid/reason、q、ω、r、observed_end、lag_lo、复数 taps、归一化与模板身份、模型 fingerprint、拟合诊断。冻结后 apply 不重拟合。
- `estimate_discrete_cir(...)`：仅混合 IQ 前缀、公开模板/config、预分配 scratch；禁止 sidecar 真值参数。
- `apply_discrete_cir(...)`：输入已重调制波形与冻结模型，返回完整重构/相减结果；支持与有限性检查失败时 out==y、ihat==0。
- 分开的 estimator/reconstruction scratch，明确 capacity 与冷初始化计时。在线热调用不新增大分配。

重构先实现直接 FIR 参考/向量化内核（目标 O(NL)，L 为实际 tap 数），以实测决定是否值得继续优化；本轮不同时开发 FFT、稀疏裁剪和混合缓存三套后端。密集 FIR 可能更慢，必须如实报告。

## 4. 分工、依赖和每环节交付

| 阶段/负责人 | 文件所有权与工作 | 完成证据 / 下一步 |
|---|---|---|
| S0 主 agent | 阅读现有 fullpacket/preamble 核心、快速 RX runner 与 MATLAB 参考；冻结上述新头文件、支持/λ校准规则和基准命令；记录模板归一化、训练行、整数坐标 | 接口说明与一个短数值原型证实拟合可辨识；若失败先解决模型，不派实现；成功后并行 A/B/C |
| A subagent | 新 `gr-uwb/lib/uwb_discrete_cir_estimator.cc` 及 `qa_uwb_discrete_cir_estimator.cc`；独立 acquisition、去 CFO、LS、模型冻结与诊断 | 估计器针对性 QA、拟合条件性和估计耗时；交给主 agent 集成 |
| B subagent | 新 `gr-uwb/lib/uwb_discrete_cir_reconstruct.cc` 及 `qa_uwb_discrete_cir_reconstruct.cc`；先用人工 taps 对接口开发线性 FIR、CFO、边界、数值安全 | 与独立直接卷积参考对齐；交给主 agent 接真实估计模型 |
| C subagent | 新 `tools/sic/run_discrete_cir_compare.py` 与独立离线 verifier；先准备数据、统计/schema 和必要用例，不等 A/B 完成；不得编辑生产 runner | 相同源窗配对、解码率/效果/延迟报表脚本就绪；集成后执行一次效果评价 |
| S2 主 agent | 唯一负责新公共头文件后续修改、CMake、`benchmark_fullpacket_compare.cc`、原 fullpacket wrapper 接线；保留原选项 | 构建并运行受影响 QA 一次；确认新路径不调用旧分数估计，交给 C 做效果验收 |
| S3 主 agent | 效果通过后独占性能锁串行计时；归档报告、更新入口并提交 | 输出是否达到质量/性能目标、失败项与唯一建议下一步；不自动扩大实验 |

依赖：S0 → (A ∥ B ∥ C) → S2 → C 效果验收 → S3。共享文件只有主 agent 写；接口变更先发消息同步，subagent 不自行改公共头文件、不提交他人文件。计时期间其他 agent 不运行 CPU 密集任务。主 agent 集中提交各阶段代码。

## 5. 必要验证与停止规则

每次有意义实现修改，只构建受影响 target 并运行对应 QA；文档/统计展示改动不跑算法测试。不要为“保险”重复 smoke、跑全部历史矩阵、或每个 subagent 各跑一遍全流水线。

1. A/B 单测覆盖实际风险：已知整数 FIR；非整数多径产生的独立接收波形；非零 CFO/负 lag；训练前缀隔离；病态/非有限输入；字段交界和包尾；FCS 失败的事务旁路。合并相近用例，不能只用新估计器自身生成 golden。整数模型用独立线性卷积验证，分数模型沿用既有生成器，oracle 只在离线 verifier。
2. 集成只跑一次既有 10 格、每格 16 源窗功能对照（复用 VOLK 报告输入）；若缺少分数多径/困难 CFO/弱目标保护，则只补缺少的代表用例。16/32/127 B 的边界长度检查放单测，不扩展性能矩阵。明确测试集与校准集隔离。
3. 本轮工程准入：配对 comm/sensing 正确解码数不低于旧路径；负样本无新增误提交；每格实际干扰抑制与弱目标保护指标相对旧路径劣化不超过 1 dB。逐格列出结果与失败，不能用平均值掩盖；困难 CFO 原有失败照实保留。这是有限样本准入，不是总体解码率证明。
4. 仅质量通过后，sync_base/payload_base 两格、16 源窗，各变体 16 次预热 + 128 次观测 × 2 轮，顺序 AB/BA；采用相同环境与同一计时层次，记录源窗重复并非独立样本。无需反复计时到获得更好结果。
5. 报告估计器 acquisition/去 CFO/投影/求解，重构卷积/CFO检查，以及解调/重调制/相减/总 SIC；父子不重复累加。缓存构建与热态、峰值 scratch、失败延迟单列；mean/P50/P95/P99 如实报告，小样本 P99 仅描述性。
6. 目标：保持质量，配对总 SIC mean 降低至少 10%，P95 不增加；卷积快但总 SIC 慢不算成功。未达到即报告瓶颈，不无限扫参、不宣称 0.5 ms 或 200 窗/s 已达标。

正式计时须绑定 code-tested commit、profile、样本长度、实际加载库 hash、编译参数；独占 `/tmp/uwb-sic-benchmark.lock`。用 `env -u LD_LIBRARY_PATH` 并检查 ldd。历史报告的 8.89 ms 总 SIC 仅背景，正式结论以本轮同时配对旧后端为准。分段诊断存在计时扰动，不能作为生产精确基线。大产物留新 build/results 的唯一 run 目录。MATLAB 未运行明确标注；不以此阻止本轮离线 C++ 实验，也不宣称 MATLAB/硬件通过。

## 6. 每个阶段必须输出的反馈

主 agent 在 S0、A/B/C 各自完成、S2、S3 时各输出一条简短中文反馈，并追加到 `docs/sic/DiscreteCIR_开发进度.md`。长阶段至少每 60 秒提供进展；反馈不作为等待审批的门槛。

    [阶段 Sx / 负责人] 完成 / 阻塞 / 未达标
    完成内容：一句话说明实际行为变化。
    证据：commit 或文件；必要测试结果/测量数值；未验证项。
    问题：无，或具体失败及影响（不写泛泛风险）。
    下一步：负责人 + 具体任务 + 进入条件。

最终交付代码、一次功能对照、一次串行性能对照与简短报告。报告首先回答“是否值得替换现有 CIR 估计器”，再给效果/解码率/总耗时表和剩余瓶颈；不得把阶段完成写成算法达标。
