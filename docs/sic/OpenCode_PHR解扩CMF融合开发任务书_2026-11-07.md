# PHR 解扩与 CMF 融合开发任务书

## 1. 目标与范围

将通信 RX 的 PHR 指标生成改为等价的“先按延迟解扩，再 CMF 合并”，沿用 payload 已验证的融合模板实现思路。保持 PHR、通信 PSDU/FCS 和 SIC 提交结果一致，实际减少 soft-chip FIR 计算，并分别量出 PHR 与 payload 的成本。

- 工作树 `.worktrees/sic-fullpacket-compare`，分支 `feature/sic-fullpacket-compare`，起始 HEAD `9ff3be1`；既有计时代码 `57f97d5`。
- 基线为快速 RX 四选项 + `discrete_cir` + 已开启的 payload 融合；本轮新增 `--rx-fused-phr-cmf`，默认关闭。
- 首轮新选项要求同时开启 payload 融合和 on-demand；由配置层自动启用依赖并输出有效配置，避免新增大量组合。
- 只改实验接收链，不改冻结 SIC 核心、重构 FIR、CIR 估计数学、同步算法或 FEC 算法；不上板。
- 参考当前 `uwb_demod_core.h`、`uwb_rx_fused_cmf.h`、`fullpacket_rx_pipeline.h` 及《CMF解扩融合可行性报告_2026-11-07.md》。MATLAB 公式参考 `/mnt/f/USRP数据解调/run_compare_cmf_despread_order.m`，只读，不要求重跑大实验。

当前 32 B／12 个有效重构 tap：通信解码 1.579 ms，总 SIC 4.934 ms；前缀 soft-chip FIR 388 µs，PHR 阶段约 9 µs，payload 模板+相关 39.2 µs。388 µs 含 SYNC/SFD/PHR 和部分 payload，不能当作 PHR FIR；9 µs 不含此前 FIR。接收 CMF 是 38 tap，与重构 12 tap 不同。

## 2. 原理与准确参数

令 h 为现有 CMF 已处理共轭和归一化的系数，x 为 CFO 补偿 IQ，a 为当前符号 PN。固定候选起点 b：

```text
旧：z[c] = Σq h[q] x[b + 2c + q]
    M    = Σc a[c] z[c]
新：t[m] = Σ(q+2c=m) a[c] h[q]
    M    = Σm t[m] x[b+m]
判决指标 = polarity × Re(rot × M)
```

这是交换线性求和的融合实现，不显式生成完整解扩延迟矩阵，也不能先压成一个标量再忽略信道延迟。

PHR 固定 21 symbol、64 chip/burst、512 chip/symbol、PN offset=0；spc=2、38 tap 时模板长 `38+63×2=164`。四候选 chip 偏移按既有顺序为 `0,256,64,320`；符号起点为 `sfd_end_chip+1+s×512`。payload 保持 8 chip/burst、PN offset=1344、52 点模板。

主 agent 从现有代码确认 IQ 基址、chip 对齐、共轭及 rot；复用当前两 hop×两 BPM 搜索、符号极性和平局规则。不能把 payload 常量机械替换后留下 52 点专用 kernel 或不足的 scratch。

## 3. 必须完成的接入

1. **先明确前缀依赖。** 列出 CIR 估计、相位估计、NS-SFD 搜索各自读取的区间及 halo，推导最小安全 soft-chip 前缀。保持它们的输入区间、搜索范围和相位定义；不能凭 PHR 名义起点硬截断搜索窗。正尺度归一化对硬判决可抵消，但须检查 NS-SFD 门限和相位估计是否依赖它。
2. **真正减少 FIR。** 新路径只生成上述必要前缀；PHR 和 payload 由融合相关产生指标。不继续无条件生成原 59,740 chip 前缀。若必要依赖确实跨入 PHR，保留并量出重叠范围，不声称整段 FIR 全部省掉。原逻辑预算准入与实际生成长度分开，仍拒绝原路径不允许的短窗。
3. **复用译码。** 将 PHR 的指标判决与 Viterbi/SECDED 解析做最小拆分，共享后者；缓存 PHR 的 21 对 g0/g1。payload 联合 Viterbi 仍需要原始 PHR 编码判决，必须使用缓存，不能从译码后 19 bit 代替，也不能再次调用依赖 PHR soft-chip 的 `bprf_demod`。保留旧路径入口和行为。
4. **统一回退。** 首轮只支持完整 CMF、spc=2。分数 spc、RAKE、bypass、不支持配置、范围不足、非有限值、PHR 纠错或后续解码/FCS 失败，至多执行一次完整旧流，发布最终结果。尤其保留 SECDED 纠错长度涉及尾能量检查的既有行为；不要另写近似规则。回退包含全部耗时，不嵌套重复重试。
5. **模板与存储。** PN 可按配置缓存，信道模板每包更新；PHR 21 个符号使用各自 PN。复用 portable/SIMD 基础，预分配 scratch，保持 payload 专用快路径。无需新增 GNU Radio block。
6. **显式边界。** 验证负索引、长度计算、最后候选偏移 320、模板尾 halo、名义 21×512 chip 范围、tap 数和实际 IQ 长度。在线只用接收结果和公开配置。

## 4. 独立计时：必须能回答“PHR FIR 花多久”

增加诊断字段并给出父子关系，保留原字段语义；PHR 与 payload 各自记录 template、correlate、decode、attempt、fallback。新路径的 PHR 独立 soft-chip FIR 为零或实际保留量，不能把融合计算记作零开销。

旧路径按解码所得字段边界，将 soft-chip FIR 分为前导/SFD、PHR、payload及额外预算区。分类按输出 chip 位置，输入 halo 不重复归属。若 FIR 时刻尚未知准确 SFD 末端，用独立诊断重放在已确定的同一输入、CIR、rot 上分段；明确为诊断成本，不能冒充生产精确分摊。不要逐 chip 调时钟。

报告至少给出：

- 旧 PHR FIR、后处理、解扩/译码；新 PHR 模板、相关、译码。
- 保留前缀 FIR、PHR 与 payload 融合成本、实际生成 chip 数及覆盖范围。
- 通信解码与总 SIC 的 mean/P95；重试时间和次数单列且纳入整体。

端到端收益由同一 binary 开关配对确定。缩前缀会同时移除多种字段的冗余计算，这部分单列，不能全归为 PHR 内核收益。

## 5. 并行任务与交付顺序

主 agent 先冻结 helper API、scratch 大小、PHR 判决缓存及计时字段，再调度三个 subagent。共享头文件和构建接线归主 agent，避免交叉修改。

| 负责人 | 文件所有权与任务 | 交付 |
|---|---|---|
| A：内核 | `uwb_rx_fused_cmf.h` | PHR 164 点融合、参数化几何与边界，保留 payload 快路径 |
| B：独立验证 | 新 `qa_uwb_rx_fused_phr.cc` | double 双重求和参考、四候选和判决对照、边界与集成用例；提供 CMake 建议 |
| C：统计 | 新 `tools/sic/run_fused_phr_compare.py`、本轮报告 | 复用已有输入和驱动结构，完整性/功能/性能分别判定，字段分段统计 |
| 主 agent | `uwb_demod_core.h`、profile/result/scratch、pipeline、runner、CMake | 前缀依赖、共享译码与判决缓存、开关、一次回退、集成和串行计时 |

执行三步：G0 公式与内核对照 → G1 缩前缀及解码接入 → G2 配对计时与报告。每步完成输出“做了什么 → 数值/功能/耗时证据 → 结论 → 下一步”；发现关键错误先关闭，不扩大无关测试。

## 6. 最小验收，优先开发效率

- 内核一个针对性 QA：复 CIR/PN/相位、code 9–12、两种极性、近零/平局、非对齐、合法尾边界及少一个 sample、非法几何。对独立 double 参考报告缩放误差和近零绝对误差，沿用 2e-5 相对尺度门槛；不通过时查原因，不自动放宽。
- 集成只跑相关 demod、fast-RX、payload/PHR 融合 QA。新增少量确定性案例覆盖 PHR SECDED 纠错与不可纠错、短窗、payload 失败、关闭开关和不支持模式；核对最终结果及一次回退。复用 fixture，不搭新生成平台。
- 功能复用既有六格 inputs16：`payload_base`、`payload_four_frac`、`payload_cfo_m50k`、`payload_cfo_p50k`、`payload_len127`、`payload_snr8`。逐窗比较 PHR bits/长度/纠错状态、comm/sensing bytes/FCS/status、committed/reason。正常可支持样本须实际走融合，不能靠全部回退通过。
- 计时只测 `payload_base`、`payload_four_frac`、`payload_len127`；旧=payload 融合已开，新=再开 PHR 融合及必要前缀收缩。同 binary、同输入，预热 16，观测 128×2，ABBA。复用已验收设置，不另扫 tap/SNR 矩阵。
- 驱动先检查预期行数、source ID 和重复轮次、解码一致性及失败率，再判性能。拒绝/失败不静默过滤；空结果、缺行、解码退化应非零退出，性能未达明确输出目标未达，不能统一 PASS。
- 新 build `gr-uwb/build-sic-fullpacket-fused-phr`，历史 run 不覆盖。独占 `/tmp/uwb-sic-benchmark.lock`，正式计时不并行构建或测试；`env -u LD_LIBRARY_PATH` 并核对 ldd，记录 commit、有效选项、窗口、CPU、binary/lib hash。大 IQ/build 不进 Git，不安装 `/usr/local`。

功能一致且实际减少 FIR 为接入验收要求；性能目标为含模板更新的 PHR 路径降低至少 20%，通信解码/总 SIC mean 不增加超过 5%，P95 如实报告。目标未达时最多进行一轮明确瓶颈优化，再交付真实结论；不改为默认启用，不做硬件、soak、FFT、全面矩阵或重写 FEC。

## 7. 最终交付

代码、针对性 QA、复现命令、CSV/JSON 摘要及 `docs/sic/PHR解扩CMF融合开发报告_2026-11-07.md`。报告简要回答：解码是否一致，PHR/payload 各花多久，减少多少 FIR，通信与 SIC 快多少，下一步是否值得继续。更新入口为真实完成状态；本任务书本身不表示已经实现。
