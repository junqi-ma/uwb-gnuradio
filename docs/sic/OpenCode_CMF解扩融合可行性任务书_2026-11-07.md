# CMF 与解扩融合：可行性开发任务书

## 1. 要回答的问题

在现有 GNU Radio C++ 接收机中，将 payload 的 CMF 与 PN 解扩融合，能否在保持解码结果的同时减少耗时？首要交付是有证据的“可用且更快 / 正确但不快 / 尚不等价”结论。先证明内核，再做最小集成；不要求全面重构或默认启用。

- 工作树：`.worktrees/sic-fullpacket-compare`；分支 `feature/sic-fullpacket-compare`；起始代码 `6ac593c`。
- 当前组合：快速 RX 四选项全开 + `discrete_cir` + 融合 FIR。
- 32 B 参考：单径 3 tap 总 SIC 3.599 ms，四径 12 tap 4.958 ms；通信解码约 1.70/1.67 ms。数字来自 `gr-uwb/build-sic-fullpacket-rxopt/results/best_combined_20261107_113020/`，仅供定位，正式收益本轮配对测量。
- 本轮只优化通信 RX 的 payload 指标生成；接收 CMF 为约 38 tap，与 SIC 重构的 3/12 tap 不同。

## 2. 原理与参考

现有计算（省略固定对齐偏移）：

```text
z[k] = Σq h*[q] × rx[nk + q]
M    = Σk PN[k] × z[k]
```

交换求和可先按每个延迟解扩，再做信道合并。进一步将重叠权重预先相加，可构造一个有限长模板，直接从候选 IQ 窗口求 M。实现使用融合模板，不显式生成解扩后的延迟矩阵。

payload 为 8 chip/burst、2 sample/chip、38 tap：单个模板长度为 38+(8−1)×2=52；原逐 chip 计算约需 8×38=304 次复数加权，融合相关约 52 次。收益必须计入每包模板构造。支持范围、索引、共轭和相位必须从现有 C++ 公式推导，不照抄 MATLAB 的 1-based 索引。

只读参考：

- `/mnt/f/USRP数据解调/markdowns/CMF解扩顺序对照实验.md`
- `/mnt/f/USRP数据解调/run_compare_cmf_despread_order.m`：`makePlan`、`updateWeights`、`frontField`、`backEnd`。
- `/mnt/f/USRP数据解调/decoded_results/cmf_order_extended/summary.csv`

MATLAB `fused-conv` 已完成 4,000 次配对，软分数归一化误差约 9.9e-16、硬比特/PSDU 零分歧；含模板更新的相关加速为单径约 2 倍、多径约 1.3 倍。其 front 使用全采样率 FIR 和 128/145 tap，而 C++ 已按 chip 取样并 SIMD 化，速度须重新验证。

## 3. 集成契约：先冻结再并行

主 agent 先读 `uwb_demod_core.h` 的 `stage_cir_softchips`、`bprf_demod`、`bprf_demod_soft`、`stage_payload_fcs`，用一个短步骤确定 helper API 与计时字段。

- 输入：CFO 补偿后的 IQ、实际接收 CMF 系数、chip 起点、spc、相位 rot、PN、候选位置和输出 scratch。在线数据不来自注入真值。
- 输出：按当前顺序排列的两个 hop × 两个 BPM 位置的有符号相关指标；沿用现有硬判决的候选比较及平局规则。
- MATLAB 根据 PN 选择 hop；C++ 当前搜索两个 hop。本轮保留 C++ 四候选，不顺带改成 PN 单 hop。
- 只支持当前整数网格 spc=2 和完整 CMF；分数 spc、RAKE、bypass 走原路径。显式开关 `--rx-fused-payload-cmf`，默认关闭。
- SYNC/SFD/PHR 保持当前前缀路径，复用其 CIR、相位和已校验 PHR。包长只取接收解码结果。
- 复用 C++ 现有 FEC/FCS，先接成功路径的硬判决。失败后至多一次完整旧路径重试，统一发布最终结果并计数，不另造译码器。
- 当前 soft-chip 用全局最大值归一化。G0 比较未归一化复指标；G1 硬判决路径可用统一正尺度，复用前缀 rot。若需要软回退或依赖幅度门限，调用旧流；不能为复现全流最大值而偷偷生成完整 soft-chip，抵消优化。
- 保留旧流的范围不足/短窗检查；仅“候选窗口可读”不能放宽原包长准入。固定偏移、负索引、尾部 halo 均显式检查。
- CIR、相位、PN 或配置改变时更新模板。仅与配置有关的 PN/位置可跨包缓存；本轮计时按每包更新信道模板。

## 4. 多 subagent 分工

主 agent + 3 个 subagent，接口确定后并行。不要多人修改大 header。

| 负责人 | 独占范围 | 交付 |
|---|---|---|
| A：融合内核 | 新 `gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h` | 模板构造、四候选相关、预分配 scratch、portable + 复用现有 SIMD/VOLK；简述索引公式 |
| B：独立数值验证 | 新 `gr-uwb/lib/qa_uwb_rx_fused_cmf.cc` | 直接双重求和的 double 参考、候选指标比较、边界及判决验证；给主 agent CMake 接线建议 |
| C：实验与报告 | 新 `tools/sic/run_fused_cmf_compare.py`、实验摘要/报告 | 配对运行、失败/回退统计、分阶段与总 SIC 汇总；等待主 agent 提供真实计时字段 |
| 主 agent | `uwb_demod_core.h`、profile/result/scratch、`fullpacket_rx_pipeline.h`、benchmark、CMake | 公共接口、最小集成、统一构建和串行计时、最终结论 |

A/B/C 可先并行准备内核、参考和驱动；G0 通过后主 agent 接入。共享文件修改由主 agent 完成。subagent 交付后报告“完成内容、证据、问题、下一步”，避免仅报测试通过。

## 5. 执行顺序与停止条件

### G0：证明内核正确、值得接入

1. A/B 对同一 IQ、同一权重比较四候选的全部复指标。包括真实截取片段和确定性随机输入，覆盖复 CIR、相位、非对齐地址、末尾合法窗口和越界拒绝。
2. 使用独立 double 双重求和作参考；缩放误差可定义为 `max|new−ref| / max(max|ref|, 1e-12)`，float 初始门槛 2e-5。近零抵消样本同时报告绝对误差；失败先查原因，不自动放宽门槛。模拟真实判决检查硬比特。
3. 对真实 payload 数据比较“旧 payload CMF+后处理+解扩”与“每包模板更新+融合相关+判决”；旧流不能重复承担前导计算，新流不能省略更新成本。同时报告相关内核和包级工作量，避免只对单个 dot 宣称整段收益。
4. 若有可复现数值错误，修复后再接入；若正确但含更新成本无收益，最多一次有针对性的布局/复用优化，然后交付“不值得集成”的结论。

### G1：验证最小接入有效

1. 接通显式选项；按既有前缀完成 PHR 后直接产生 payload 指标，不再为 payload 扩展 soft-chip。
2. 前缀当前有额外预算，首轮不缩短，以控制改动。记录已生成 chip 数和实际减少的计算范围。
3. 单独记录 `template_update_us`、`fused_correlate_us`、`fused_fallback_count`，保留原阶段语义；若重新分配计时，报告映射。父子耗时不得重复相加。
4. 对照基线为同一新 binary、四个 RX 优化开关均开启，只切换融合开关；两边均使用 `--reconstruction discrete_cir`。

### G2：给出能否继续的结论

建议继续条件：功能一致，含模板更新的 payload 指标生成 mean 降低至少 20%，通信解码及总 SIC 无明显退化（mean 增加不超过 5%）；P95 如实报告。20% 是开发目标，不是正确性的必要条件。小幅收益、无收益或退化均可作为完成的可行性结论，不追加无边界优化。

## 6. 最小验证与计时

- G0 一个针对性 QA；接入后只跑相关 demod/fast-RX QA。出现新失败才扩展排查。
- 功能复用已有 inputs16：`payload_base`、`payload_four_frac`、`payload_cfo_m50k`、`payload_cfo_p50k`、`payload_len127`、`payload_snr8`，各 16 源窗，新旧逐窗比较 bytes/FCS/status/committed/reason；检查行数和 source ID 唯一性。
- 加一个确定性损坏包，验证失败/回退不误提交；复用已有 fixture，优先不新增生成器。保留失败样本，统计尝试融合、成功融合、回退次数。
- 正式计时只测 `payload_base`、`payload_four_frac`、`payload_len127`，每变体预热 16、观测 128×2，old→new→new→old；失败与回退计入整体耗时，并另列成功路径。读入/初始化开销和样本等待单独说明。
- 新 build `gr-uwb/build-sic-fullpacket-fused-cmf`，不覆盖旧 build/run。独占 `/tmp/uwb-sic-benchmark.lock`，暂停其他构建和重负载，串行测量；`env -u LD_LIBRARY_PATH` 并核对 ldd，绑定 commit、选项、窗口、实际二进制和库哈希。
- 驱动必须真实执行功能与性能判定；空结果、缺行、未满足目标不能统一标 PASS。分别输出“正确性通过”和“性能目标达到”。
- 不重跑历史全矩阵，不做硬件、长时间 soak、FFT、PHR 融合或译码器改造。本轮不用重新跑 MATLAB 4,000 包，现有实验作公式参考。

## 7. 最终交付与反馈

交付显式实验代码（或正确但不快的独立原型）、必要 QA、可复现命令及简短报告 `docs/sic/CMF解扩融合可行性报告_2026-11-07.md`。

报告只需：是否可用、是否更快；四候选误差与解码差异；模板更新/相关/通信解码/总 SIC 的 mean、P95；失败与回退；下一步建议。完整数据放新 build/results，不提交大 IQ。

主 agent 在接口确定、G0、G1、G2 完成时各输出一次：

> 本阶段完成了什么 → 数值/耗时证据 → 当前结论 → 下一步做什么。

若 G0 已否定性能可行性，报告原因后结束，不为完成清单强行改动主接收流程。
