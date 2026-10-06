# OpenCode 开发指示：M2-A 非 MATLAB 子项收尾

日期：2026-10-06。性质：状态核验与下一步任务单；本回合仅规划，未修改生产代码。

## 1. 核验结论

**当前为 M2-A 干净信道离线闭环已交付，整体尚未完成；不进入 M2-B。**

依据：[开发报告](M2-A_native_PHY开发报告.md)、[G0](M2-A_G0接口与数字坐标.md)、
[评审报告](M2-A_评审报告.md)、[原任务单](OpenCode开发指示_M2-A_native_PHY闭环.md)。
继续遵守 [需求](需求_UWB_SS_DS_TWR.md)、[总路线](开发路线与验收矩阵.md)、
[参考资料与 API 映射](参考资料与API映射.md) 和仓库 AGENTS.md。

### 1.1 本轮实际检查

- HEAD：`bcdec85`（M2-A 主链集成）；已跟踪工作区在规划前无修改，已有未跟踪文档与实验数据保留。
- `uwb_twr_phy.cc` SHA256：`d4264480303aa909ab3d1f31c0ad88a553aa594419318a52a191c7560c8bb8f0`。
- `uwb_twr_phy.h` SHA256：`b274d55755ad97dfa28fe510b3ced8dda488a0c493c7a064615b7ac46546acc6`。
  与开发报告的缩写一致。
- 在现有 build 实跑以下 CTest，**4/4 通过，3.58 秒**：
  `rational_resampler_m2a`、`twr_native_phy`、`twr_core.cc`、`twr_m2a_native_phy_verify_py`。
- 实跑 Final / 491.52 MS/s / SC16：bytes 精确、FCS 通过；work TX=132544、native=65294、
  work RX=132711；SC16 饱和计数=31。独立 verifier **72/72**，self-test **24 个变异全部检出**。
- M2-A QA 的 ldd：无 GNU Radio/UHD 动态依赖，实际使用 `/usr/local/lib/libvolk.so.3.3`。
- 本轮没有重新构建、全量 62 项回归、安装消费、sanitizer、MATLAB 或 SIMD 缺陷复现。
  开发报告的 **61/62、历史吞吐失败**属于此前运行证据，不改称本轮实测。

本轮命令：

```bash
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build \
  -R 'rational_resampler_m2a|twr_native_phy|twr_m2a_native_phy|twr_core.cc' -j1 --output-on-failure
env -u LD_LIBRARY_PATH gr-uwb/build/apps/twr_native_phy_demo \
  --frame final --native-rate 491520000 --iq sc16 --output /tmp/twr-m2a-status-20261006-final.json
python3 tools/twr/verify_m2_a_native_phy.py \
  --output /tmp/twr-m2a-status-20261006-final.json --self-test
```

### 1.2 完成状态要分开报告

| 项目 | 当前事实 | 下一步 |
|---|---|---|
| A01–A04、A06–A09 | 有实现和 QA，干净闭环已交付；本轮定向回归通过 | 保持基线，复核测试实际覆盖 |
| A05 数字坐标 | 开发报告正文标为“部分”，不能被摘要“A01–A09 完成”覆盖 | 补非零 origin、padding/裁剪、精确映射与 verifier |
| A10 信道损伤 | 未实现；CLI 明确拒绝非 clean | 下一主开发项 |
| A11 MATLAB | 用户指示忽略；没有实跑证据 | **本轮不开发、不运行、不安装环境**，不改称通过 |
| A12 manifest/verifier | clean 路径通过；未消费 TwrConfig 是 G0 允许的降级 | 保持说明，补损伤/资源/坐标字段验证，不扩展完整配置系统 |
| A13 分配与延迟 | 未实测；重采样 helper 每次调用新建 core | 补 prepared context、buffer 复用及实际观测 |
| A14 回归/安装 | 有此前结果，报告仍标部分 | 对收尾 revision 重新验收 |
| 独立 review | 现有报告审核的是 G0 / `4edf6c2`，不是最终 `bcdec85` 实现 | 补最终实现审查，不能沿用 G0 裁决代签 |
| SIMD 非确定性 | 开发报告记录已复现、未修；本轮未重定位根因 | 归档复现与限制，**不修改核** |

## 2. 本任务边界（优先于旧任务单的本轮安排）

1. 保持用户“忽略 MATLAB”的指示：不再安排 oracle agent，不把安装 MATLAB 作为开工条件。
   `matlab.executed=false`，保留既有脚本，不删除或伪造证据。
2. 保持“SIMD 核缺陷只记录不修”：不修改 `uwb_rational_resampler_core.h` 的 FIR/SIMD/
   padding/宏块算法，不顺手修 shared kernel，不悄悄改生产默认 kernel。
   本任务也不将“小于 1e-5”解释为未初始化读取可接受或已经消除。
3. 不做 M2-B：不增加首径、RMARKER、硬件校准、ToF 有效性或 timestamp patch；
   不实现 controller/UHD/两 RX/pybind；不更改 M1-B 事件准入来接收 decode-only 数据。
4. 只补 **A05/A10/A13、相关 A12/A14 和最终审查**。原最小 profile、两 rate、
   三帧、两格式、FCS 唯一归属及失败语义保持。
5. 按原完整验收口径 A11 仍未满足。收尾后最多声明
   **“M2-A 本轮非 MATLAB 子项完成；独立 MATLAB 对照未做，SIMD 已知风险保留”**。
   不擅自把“忽略 MATLAB”解释为已获得其验证证据或整体无条件验收。

## 3. S0：先补证据与实现设计

- 归档 `/tmp/opencode/dbg_core.cc`、`det_size_k.cc` 等有效复现程序到
  `testdata/twr/m2a/diagnostics/`，补输入/taps hash、实际 kernel、编译选项、库版本、
  allocator 环境、运行次数、finite 检查及复现命令；探针不得依赖另一机器的临时文件。
- 区分“相同输入不同输出”的实测与“未初始化读取发生在何处”的定位证据。
  如没有明确读地址/源码证据，根因精度如实注明；不因 ASAN/UBSan 通过宣称排除了
  未初始化读取。已知失败单独归档，不伪装成普通通过 QA。
- scalar / VOLK / AVX2 的观测分开；kernel 不可用写 SKIP 与原因，不把 fallback 当该 kernel 通过。
  相同 kernel 重复运行确定性，与不同 kernel 的正常浮点归约差异是两件事。
- 先补 `M2-A_G0接口与数字坐标.md` 收尾附录：prepared context 生命周期、stage 坐标、
  impairment schema/SNR 定义、观测窗口和准入规则，经 reviewer 确认再改主链。

SIMD 修复是**待用户另行授权的独立任务**；本轮只记录其对证据与性能解释的影响。
若它实际阻止任一必选子项，报告该项阻塞，不扩大容差或静默绕过以宣布完成。

## 4. S1：A13 生命周期与缓冲复用

源码依据：`uwb_twr_phy.cc::resample_causal` 在每次调用中执行 `Core core(...)`；
`m2a_demod_work` 每次创建模板、scratch，并在搜索候选循环中创建 padding buffer。
因此当前 helper 是一次性离线执行路径，还不是 prepared、可持续复用的包处理路径。

### 4.1 实现要求

- 新增或最小扩展一个 **M2-A prepared context**，冻结 profile/taps/容量/格式，
  持有 TX/RX core、调制 scratch/prefix、IQ buffer、解调模板和可复用搜索 workspace。
  命名由协调者在 G0 冻结；不复制另一套调制/重采样/解调算法。
- 原便捷 API 可保留并明确是 cold wrapper；新 demo/benchmark 必须实际使用 prepared API，
  不能只在独立核 benchmark 中复用而生产链仍每帧重新分配。
- context 不是并发共享对象；每 worker/端点独立实例。prepare 失败原子化；
  帧失败后可恢复，独立 burst 间 reset；不能带入前一帧滤波历史或旧 bytes。
- 扩容/配置变更只在 prepare；尺寸超限显式拒绝；所有 guard、tail、损伤滤波长度
  计入容量，不能只有原输入满足 2^20、临时 padding 却越界。
- 新 TX/重采样调度及 IQ workspace 热路径目标为零分配；既有解调内部临时 vector/
  结果字符串分配独立统计。不要为追求“整链零分配”重构整个 demod core。
- 本步骤只复用既有核实例，不修改 SIMD 算法；若复用使已知异常更明显，单独报告。

### 4.2 观测要求

- 分开计数：prepare、prepare 后首帧、后续帧；调制、TX resample、量化、RX resample、
  搜索/解调、结果/JSON 序列化。注明计数器覆盖 `new/new[]/aligned new` 与 malloc 的范围。
  未拦截到的分配不宣称为零，计数器本身不可递归分配。
- 耗时计数与分配计数分次运行；日志/文件 I/O 不计入核心计算时间，端到端 wall time 另列。
- 两 rate × 三帧 × 两格式；相同帧重复与 timestamp/seq/地址变化各测一组；
  clean 与长搜索/失败路径分别报告，不只测缓存命中的最快路径。
- 默认每组至少 1000 次热调用，保存样本数、P50/P95/P99/max、cold 延迟、RSS/峰值内存。
  若样本不足，不报告 P99.9；重复循环 RSS/容量不能持续增长。
- 同环境串行比较当前一次性 wrapper 与 prepared 路径，保留原始 CSV。
  不从软件数据推出硬件 reply-delay 下限、1 GS/s 实时率或双 RX 可运行。

## 5. S2：A10 native 域损伤压力

损伤模型放测试/离线 harness，不给解码器传入 truth 或精确帧起点，不新增 Python FSM。
冻结顺序：TX native CF32 → 信道模型 → RX SC16 量化（若选）→ RX resampler → demod。
若要模拟 TX DAC 量化，必须作为独立选项，不混用现有 native SC16 往返语义。

### 5.1 最小场景（G0 附录先冻结）

| 场景 | 必测点 / 定义 |
|---|---|
| clean | 现有全部基本单元，输出保持兼容 |
| CFO | 0、+20 kHz、−20 kHz；相位按 native rate、样点坐标推进；跨 chunk 不重置 |
| AWGN | 建议固定 30 dB（常规）和 10 dB（压力）；冻结 seed 列表与复噪声方差公式 |
| 延迟 | 整数 0/1/17 native samples，分数 1/4、1/2、3/4 sample；滤波器阶数/群时延/边界约定固定 |
| 多径 | 最少一组弱首径强后径，例如复振幅 0.35 与 1.0、后径 +8 native samples；相位也记录 |
| 组合 | CFO+AWGN、分数延迟+多径各至少一组；不得只测单项 |

建议参数不是已验证能力。不得按跑出的成功种子挑子集；每基本单元固定至少 10 个 seed
用于含随机项的场景，seed 0 若特殊也必须明确。SNR 以**信号有效区间**平均功率计算，
不能因前置零填充变长而改变同一信号的噪声水平；记录区间、功率及每维方差。
分数延迟/多径的长度、尾部与数字群时延进入 trace，不把已知延迟直接输入 ToF。

### 5.2 判据

- clean 仍全部 bytes 精确/FCS 成功；零 CFO/零噪声/单位信道等退化路径与 clean 一致。
- 单位测试先独立验证损伤模型：CFO 相位/频偏、AWGN 方差、impulse 延迟和多径系数。
  不只依赖“解调似乎成功”来验证信道模型正确。
- 压力测试允许真实解码失败，但每次必须记录终态，
  `attempted = exact_success + explicit_failure`；非精确帧不能计入成功。
- 明确分离“模型/测试机制通过”与“该信道参数可解码的通过范围”。失败场景不能取得
  native evidence；clean 成功也不替压力失败背书。
- 更换帧、空窗、坏 PHR/编码合法但 FCS 错的 PSDU、截尾仍须无旧结果重用；
  弱首径强后径只检验解码和诊断，不声明首径判决正确。
- CLI 原 `--scenario` 的拒绝行为仅在有真实实现后替换；JSON 记录参数/seed/kernel/
  饱和次数与状态。测试驱动必须实际走同一生产 helper，不能存在第二套 PHY 流水线。

## 6. S3：A05 坐标与 A12 证据补齐

当前 `stages[]` 存在不等于坐标已独立验证；多数 `origin=0`，解调使用前后 guard
调整内部搜索中心，`padding` 只记录总和。应以独立单位核验证而非仅核对 JSON 字段存在。

- 明确 stage input/output 的不同单位（bytes / samples）、坐标所处网格、0-based origin；
  记录前后 padding 分量、裁剪及搜索有效区间，不能用总 padding 猜前端偏移。
- 有理数滤波延迟和组合映射独立推导；保留 exact 坐标与显示用 rounded index 的区别。
  对逆映射，只断言定义内可逆/误差界，不要求有损取整也精确可逆。
- 测非零 origin、全部多相余数、odd/even taps、chunk 起始相位、guard/crop、两方向
  组合映射；坐标落在 padding/无效区间必须可识别，不当真实检测结果。
- 测解码失败后诊断字段清空/无效，不允许保留未换算的 padded 索引并标为输入坐标。
  不加首径或硬件时间戳，不把 sample-grid bookkeeping 命名为 RMARKER。
- verifier 增加 origin、前后 guard、区间单位/上下界、损伤参数、实际 kernel 和
  capacity/配置执行值的变异检查；schema 改动版本化或显式向后兼容。
- `--config/TwrConfig` 继续保持当前冻结 profile 退化方案；未支持就明确拒绝。
  不把此阶段扩展为另一轮完整配置系统开发，文档也不能勾选配置链路完成。
- 本轮 SC16 实跑饱和计数为 31，说明“clean 可解码”不等于“无削顶”。保留计数，
  对正常/严重削顶测试分别报告；不把它自动变成测距质量 passed，也不必因任意单个
  饱和值就将 bytes 精确的离线 decode 判错。

## 7. OpenCode 分工与执行顺序

本表供 OpenCode 执行时派发，本规划回合不启动实现 subagent。
**所有 agent 共享代码库，不得覆盖或撤销他人修改；公共接口由单一 owner 维护。**

| 角色 | 唯一写入责任 | 禁止事项 |
|---|---|---|
| 协调者 | `uwb_twr_phy.h`、G0、CMake/安装、状态与整合 | 不并发让多个 agent 改公共头 |
| A `m2a_context` | `lib/uwb_twr_phy.cc`；prepared context/复用/坐标输出 | 不改 shared resampler/HRP/demod 算法 |
| B `m2a_impairment` | 新 test-support 信道模型及模型独立 QA | 不复制主链、不向解码器传 truth |
| C `m2a_qa_perf` | `qa_uwb_twr_native_phy.cc`、新增分配/延迟观测程序 | 先补失败/边界 QA，不通过放宽容差消除问题 |
| D `m2a_cli_verify` | `apps/twr_native_phy_demo.cc`、`tools/twr/verify_m2_a_native_phy.py`、配套 CLI QA | 不自己另实现 context 或信道模型 |
| E `m2a_final_review` | 生产/QA 只读；新最终实现评审报告、诊断复现归档 | 不实现 SIMD 修复，不代写生产代码 |

未列文件默认不改；B 的信道模型通过 G0 冻结的 stage seam/runner 串接，A 和 D 复用，
不让 A/B/D 同写一个函数。JSON 与 coordinate schema 由协调者先冻结。

执行顺序：

1. **P0**：保存基线、归档已知 SIMD 复现、补 G0 收尾附录；E 先审核。
2. **P1**：A 实现 context；B 独立写模型；C 先写复用/分配/坐标测试；D 按 schema 准备。
3. **P2**：整合 A10 场景及 A05；保持干净回归；运行 verifier 变异。
4. **P3**：代码稳定后串行做 A13 观测，不能和编译/其他性能测试竞争 CPU。
5. **P4**：E 审最终 revision，复现边界与反例；协调者做 A14 回归/安装并锁定报告。

## 8. 验收与交付

每次有意义修改后构建并测受影响 QA；最终运行：

```bash
cmake -S gr-uwb -B gr-uwb/build
cmake --build gr-uwb/build -j8
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build \
  -R 'twr|rational_resampler|hrp_mod_core|demod_core' -j1 --output-on-failure
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build -j1 --output-on-failure
python3 gr-uwb/apps/test_twr_config.py
python3 tools/twr/verify_m0_1_findings.py
python3 tools/twr/verify_m1_a_tof.py
python3 gr-uwb/apps/test_twr_fake_demo.py
install_prefix=$(mktemp -d /tmp/twr-m2a-closeout-XXXXXX)
env -u LD_LIBRARY_PATH bash tools/twr/verify_install_consumer.sh "$install_prefix"
```

M1-A verifier 消费已有 golden，不要求本轮再启动 MATLAB。
补新 context 实际安装消费；ASAN+UBSan 覆盖容量/寿命/失败恢复，但不冒充 SIMD
未初始化读取已关闭。ldd 确认实际库；不使用 sudo install 作测试前置。
历史 PDU 吞吐门槛保持，单列实际结果；同环境前后对照才能说明新增代码性能影响。
新 benchmark 输出 build/临时目录，选定日志/CSV/hash 显式归档。

新增 `docs/twr/M2-A_收尾开发报告_2026-10-06.md`（跨日用实际日期）及
`docs/twr/M2-A_最终实现评审报告.md`。按 A01–A14 给出最终通过/部分/排除/未测，
关联最终代码 revision、全长 hash、命令、样本量、输出与 reviewer 裁决。
修正文档互相矛盾：摘要“A01–A09 完成”、A05 部分、A12 完整配置未做、总路线
“M2–M7 尚未实施”不可继续同时作为最新状态。

本轮收尾门槛：A05/A10/A13 证据齐全，A12/A14 对最终实现复核，reviewer 无新阻断；
未关闭 SIMD 风险和未做 A11 显式保留。未经另行同意，不恢复整体无条件完成，
不启动 M2-B。后续应单独决定 SIMD 修复范围及完整独立验证安排，再制定 M2-B 任务。
