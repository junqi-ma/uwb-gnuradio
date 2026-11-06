# 当前任务：通信解码加速已交付

结果见 `docs/sic/通信解码加速开发报告_2026-11-07.md`，code-tested SHA
`d2d18be`。三项显式实验路径：A 同步/SFD（VOLK 点积、模板缓存、尾部 ±8 seed
与回退）、B CFO（1024 点绝对相位块 + VOLK 旋转）、C 按需 soft-chip（PHR 前缀
+ 长度驱动扩展 + 完整旧路径回退）。正式三格配对 comm demod mean：
`sync_base` 2226.8→1718.9 µs（0.772）、`payload_base` 2816.0→1933.1 µs
（0.686）、`payload_four_frac` 2658.3→1715.7 µs（0.645）；P95 均未增加。
8 格×16 源窗逐窗 comm/sensing PSDU、FCS、status 一致，0 次 soft-chip fallback。
默认选项全关，旧 build 未覆盖；MATLAB/native/FPGA/硬件/paced 与
mean≤0.5 ms/P99≤1 ms/200 窗/s 未验证、未宣称。以下为历史记录。

# 当前任务：FIR 乘加融合优化已交付

结果见 `docs/sic/DiscreteCIR_FIR乘加融合报告_2026-11-07.md`，code-tested SHA
`ddf335d`。融合 `dst += h*src` 后，12 tap 纯 FIR mean 1.072→0.548 ms
（比值 0.511），3 tap FIR 0.296→0.161 ms，3 tap wrapper 1.199→1.085 ms；
所有 tap 数 FIR/channel/wrapper mean 与 P95 下降。独立 double 卷积最大相对
L2 1.3524e-7（门槛 2e-5），针对性 QA 通过。新 build
`gr-uwb/build-sic-fullpacket-fused`，旧 build 未覆盖；未改估计器、tap 选择、
CFO、FCS/ROI 或默认后端，未上板，不宣称实时目标通过。下方为历史开发记录。

# 当前任务：离散等效 CIR full-packet 后端已交付

执行 `docs/sic/FullPacket_离散CIR估计开发任务书_2026-11-06.md`。本任务优先于下方历史入口。
code-tested SHA `10b660b`，新 build `gr-uwb/build-sic-fullpacket-discrete`。新
`discrete_cir` 用前 8 SYNC 直接拟合整数 lag 离散 FIR，冻结 preamble 路径与原
`path_fractional` 保持；不新增 GNU Radio block，不上板。正式结果见
`docs/sic/DiscreteCIR_开发报告_2026-11-06.md`：10 格×16 源窗功能 PASS，
负样本 PASS；两格串行配对总 SIC mean 为旧路径的 0.308/0.343，P95/P99 同时下降。
当前主瓶颈已转为 comm demod（约 2.0–2.6 ms）。`discrete_cir` 仍是显式实验选项，
不自动改默认；MATLAB/native/FPGA/硬件与 paced 到达未验证。

# 当前任务：SYNC+SFD / PHR / payload 分段实验已交付

结果见 `docs/sic/FullPacket_字段重构计时报告_2026-11-06.md`，code-tested SHA `32473f2`；payload_base热缓存分段为0.874/0.321/1.836ms，公共0.249ms，分段总计比原库低4.5%，不得冒充生产精确分摊。用户授权离线分段实验；生产重构库保持 `1adf401` 的VOLK实现。新增独立诊断exe，按波形字段划分滤波，按最早估计路径锚定输出时间区间划分路径叠加与CFO；保留完整波形halo。公共开销单列，冷/热cache分别观察，原库逐次配对验证输出与计时扰动。只测两格16源窗，每格每变体128次×2轮，独占基准锁；针对性自测通过后绑定提交运行，不扩大历史矩阵。

# 当前任务：Full-packet CFO 的 VOLK 加速已交付

结果见 `docs/sic/FullPacket_VOLK加速报告_2026-11-06.md`，code-tested SHA `1adf401`；CFO阶段约7.6–7.8倍加速，双检查保留，10格功能通过。用户已授权在本实验分支加速乘法。保留1024点绝对相位锚定、双精度累加、转换/试减有限性检查与事务提交；采用本机VOLK分派，不新增GNU Radio block，不修改冻结前导核心。新build `gr-uwb/build-sic-fullpacket-volk`；旧build与结果保留。仅做针对性QA、10格功能对照与两格串行配对计时，不自动上板或扩大历史矩阵。

# 当前任务：CFO/转换/数值检查的隔离诊断已交付

结果见 `docs/sic/FullPacket_CFO拆分诊断报告_2026-11-06.md`，code-tested SHA `4b5da96`。已新增离线scratch重放工具，两处通用复乘为主要开销；生产融合循环与在线检查保持不变，原数据保留。不自动开始下一轮优化。

# 当前任务：payload 前缀缓存对照已交付

本轮结果见 `docs/sic/FullPacket_payload滤波对照报告_2026-11-06.md`，正式代码 `cea0fc3`；不自动开始下一轮优化。用户已授权测试仅对payload及边界区逐包滤波，固定前缀相位行按实际波形缓存；完整包仍逐路径处理和消除。新build `gr-uwb/build-sic-fullpacket-payload-cache` 保留旧build与结果，不上板。快速RX接入与复测已交付：`docs/sic/FullPacket_快速RX复测报告_2026-11-06.md`，正式代码 `28c7734`；不自动进入重构优化。历史报告 `docs/sic/FullPacket_开发报告_2026-11-06.md` 与 `5dd2cfb` 结果保留；新run独立归档。仅调整实验runner的RX路径，不改冻结SIC数学，不上板。

执行 docs/sic/FullPacket_vs_PreambleOnly_开发计划_2026-11-06.md。仅本实验分支允许新增整包路径；原 preamble 行为与历史基线保持。主 agent 与三个 subagent 按文件所有权协作。

# S2 A/B 集成基线入口

当前源码基线同时包含已验收的两个方向：

- A：累加后 sensing CIR。历史报告 `docs/sic/S2A_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2a-accepted-20261018`（`2c383d5`）。
- B：sensing 前导干扰通信 payload。历史报告 `docs/sic/S2B_整改开发报告_2026-10-18.md`，冻结 tag `sic-s2b-accepted-20261018`（`ac3849e`）。

执行与验收记录：

- `docs/sic/S2_AB冻结与合并执行指南_2026-10-18.md`
- `docs/sic/S2_AB冻结清单_2026-10-18.md`
- `docs/sic/S2_AB集成验收报告_2026-10-18.md`
- `docs/sic/S2_AB集成回归入口_2026-10-18.md`

先读根目录 `AGENTS.md`。共享 SIC 数学核心保持冻结。不要只做 A 或只做 B。
单方向历史操作说明留在：

- `docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`
- `docs/sic/OpenCode开发指示_S2B_感知前导干扰通信Payload.md`

原 A/B worktree 继续作为历史锚点，不在这里改它们的结果目录。

```bash
pwd
git status --short --branch
git rev-parse HEAD
git worktree list
```

两个 build 都从当前源码根串行构建。核对 `ldd` 指向本 worktree 的对应 `build-sic-*/lib`，不要指向父目录 `build-sic` 或 `/usr/local`。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-accum -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-accum -j2
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-payload -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-payload -j2
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-accum/lib/libgnuradio-uwb.so.1.0.0.0
env -u LD_LIBRARY_PATH ldd gr-uwb/build-sic-payload/lib/libgnuradio-uwb.so.1.0.0.0
```

`UWB_demodulation` 是指向主仓库资料的只读软链接，禁止写入。
大产物写各自 `gr-uwb/build-sic-*/results`，不覆盖历史 run，不推送远程。
