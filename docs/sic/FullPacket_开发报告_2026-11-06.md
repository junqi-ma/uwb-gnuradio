# Full-packet 与 preamble-only SIC 开发报告（2026-11-06）

已完成 P0–P4：完整通信干扰生成、在线解码/重构/事务相减、配对效果校验、真实 paced 延迟及正式串行计时。软件功能闭环通过；两组均未达到 SIC mean≤0.5 ms / P99≤1 ms。full-packet 在本次200窗/s负载下明显过载。

## 1. 交付与版本

- 实验分支 `feature/sic-fullpacket-compare`，worktree `.worktrees/sic-fullpacket-compare`，从 `794056c` 集成 tag 创建；未合并回冻结基线、未推送远程。
- 功能代表集绑定干净提交 `fab0f68`；正式计算与paced绑定干净提交 `5dd2cfb`。二者仅计时分区与CLI限制不同，消除数学没有变化。
- 原 preamble core/header/adapter、Python参考及原QA源码相对冻结tag保持不变。新接口位于 `uwb_fullpacket_sic_core.h`，新runner `benchmark_fullpacket_compare.cc`；不新增 GNU Radio block。
- 主agent负责接口/CMake/runner/集成测量；三个subagent分别交付整包core与QA、生成器与编排器、独立verifier与QA。构建及正式测量串行，正式测量独占 `/tmp/uwb-sic-benchmark.lock`。
- 新build `gr-uwb/build-sic-fullpacket` 从源码配置；ldd指向本worktree库，库SHA256 `bcba46a84664265a51909e6d08641d6c7ea3ffab876e95e821a87f832bd329e0`。未安装到 `/usr/local`、未复制旧CMakeCache。

## 2. 比较合同与实现边界

两种主组使用同一混合IQ、公开模板/粗ROI和前8 SYNC冻结模型。preamble组只消除SYNC；full组在原混合IQ上解通信包，校验PHR与FCS，再生SYNC/SFD/PHR/PSDU，施加同一冻结多径/分数时延/CFO模型并从原IQ减一次。在线路径不读sidecar、生成器payload、注入起点/CFO/路径或分量IQ。raw/clean是独立离线消费者对照，不辅助SIC。

首版支持 code10、64 SYNC、4z2、legacy脉冲、无STS/无ranging、单个完整通信干扰，wire PSDU含且只含一次FCS。没有引入长窗口重拟合、fallback或迭代。波形幅度比例显式校验；33tap/32phase分数算子与原core的对照QA通过；相减区间外逐位不变、失败原样旁路。

512样本公开粗ROI下，legacy窄脉冲的真实相关度约0.99995，但stride14会漏峰；本实验三处搜索统一stride1，保留0.40/0.25门槛。两组配置一致，这里的时间不等同于历史A/B或perf分支数字。

## 3. 固定代表集：10格各16个独立窗口

以下计数为PSDU exact且FCS通过；通信列是full组通信解码。所有格clean sensing均16/16。小样本16/16的95% Wilson区间约[80.64%,100%]，不能声称普遍零错误。

| 条件 | 通信解码 | raw sensing | preamble sensing | full sensing | CIR收益 preamble/full (dB) |
|---|---:|---:|---:|---:|---:|
| sync_base | 16/16 | 0/16 | 0/16 | 16/16 | 33.45 / 33.45 |
| payload_base | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 52.18 |
| payload_len16 | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 52.77 |
| payload_len127 | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 47.43 |
| payload_snr8 | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 30.19 |
| payload_isr0 | 16/16 | 16/16 | 16/16 | 16/16 | 0.00 / 42.18 |
| payload_isr20 | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 62.17 |
| payload_cfo_m50k | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 51.91 |
| payload_cfo_p50k | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 52.88 |
| payload_four_frac | 16/16 | 0/16 | 0/16 | 16/16 | 0.00 / 27.95 |

CIR指标是独立匹配滤波、reps9…62相对同窗clean的误差能量改善。SYNC重叠时两组CIR收益接近，但preamble组保留后续字段干扰，完整sensing解码仍失败；payload重叠时preamble组在目标区无相减收益。payload场景的通信前8 SYNC先于sensing到达，估计比SYNC碰撞容易；其高抑制度不能泛化到任意时间重叠、信道或硬件。四径格使用既有official四径和PCHIP生成器，与core分数算子存在模型差异。

## 4. 正式处理耗时

机器：Intel Core i7-12700。每格每轮每组1024次，2轮，每组每轮32次热身；同窗串行交错组顺序。复用16个独立输入，模型/解码每次重算；处理观测数不作为独立包数量。OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1、去除LD_LIBRARY_PATH。

下表为SIC自身的service wall time，不含下游CIR及sensing解调，也不含初始化、模板准备、生成器、磁盘I/O。CPU线程总时间另存CSV/JSON，包含下游消费者，不能与SIC时间混称。

| 场景/轮次 | preamble mean / P99 (ms) | full mean / P99 (ms) | mean倍数 |
|---|---:|---:|---:|
| sync_base/round1 | 2.269 / 3.144 | 16.225 / 23.939 | 7.15× |
| sync_base/round2 | 2.605 / 6.174 | 18.324 / 29.734 | 7.03× |
| payload_base/round1 | 2.851 / 6.532 | 21.841 / 31.804 | 7.66× |
| payload_base/round2 | 2.887 / 7.056 | 21.826 / 32.121 | 7.56× |

full主要增量为通信解调（约4.0–7.0 ms）及逐包多径/分数/CFO重构（约9.8–11.8 ms）；TX再调制约0.30–0.36 ms。前导模板的分数相位表可预先缓存，未知payload的相位行要逐包生成。本轮没有为追速度另换算法。首版沿用调制器局部临时分配，未承诺全路径零分配。轮间绝对耗时及P99有波动，但约7–8倍的相对成本结论未改变，按任务书未追加测量轮次。

## 5. 真实到达回放与延迟

各场景两组分开运行200窗/s、10秒，单worker、队列容量2、raw输入池16；每个scheduled窗口均有完成或拒绝终态，source_late均0。包括下游CIR及sensing完整解调的工作量；成功路径和失败路径成本不同。

| 场景/组 | 完成 / 提交 | queue_full | 平均排队 (ms) | 最终结果P99 (ms) |
|---|---:|---:|---:|---:|
| payload_base/full | 425/2000 | 1575 | 44.600 | 79.793 |
| payload_base/preamble | 2000/2000 | 0 | 0.061 | 5.474 |
| sync_base/full | 486/2000 | 1514 | 38.691 | 77.562 |
| sync_base/preamble | 1891/2000 | 109 | 6.957 | 22.038 |

最终结果延迟从共同的公开窗口首样点到CIR+完整sensing解调结果，含排队、真实到达与处理；P99只针对完成窗，拒绝数量同表保留。full接受窗的通信与sensing解码都成功，但总体交付率受拒绝限制；preamble接受窗的完整sensing解码在这两格均失败，较快终态不代表同等服务质量。payload组preamble零拒绝也不能写成完整功能200窗/s通过。

preamble逻辑观察终点10560样本≈10.577 us；full采用保守的完整公开context到齐策略，SYNC/payload context分别约139.936/236.795 us，不是精确PHR推导的最小帧尾等待。first_clean_us是首个可用前缀，不等于目标CIR或整包结果；独立预缓冲计时也不是到达延迟。拒绝行的同名时间是即时状态占位，无净化IQ，其值不进入输出延迟统计。

配置IQ内存：输入池SYNC约17.88 MB、payload约30.26 MB（十进制），另有两份输出/重构IQ约2.24/3.78 MB及算法scratch；未实测进程总RSS峰值，不把上述容量称为总内存。

## 6. 验证与异常处理记录

- 针对性CTest：9/9通过，含原core/adapter、demod/mod、radar CIR及新增fullpacket core；新增Python verifier QA单独1/1通过。core新增4组契约QA，不新增生产block测试。
- smoke：4格各4窗，配对IQ与独立raw/clean对照通过。功能代表集10格×16窗，输出IQ、输入/模板hash、FCS/PSDU、支持边界、真实残余与CIR独立校验通过。
- 两个小型负样本各1窗：no_interference整窗逐位旁路；no_overlap目标SYNC区域逐位不变（允许在目标区外消除通信包）。
- 既有A：12格×8窗、96次提交，verifier64项0失败；旁路8窗，verifier65项0失败。此开发期回归manifest如实保留dirty状态，不冒充干净提交验收；原共享源码未改，manifest记录实际加载路径位于新build。该旧A入口未记录库hash，不把此开发期记录当作独立的正式hash绑定证据。
- 既有B：11格×8窗驱动均0退出。旧wrapper默认信任build-sic-payload，首次路径校验失败；不改判据、不重采IQ，使用原verifier的显式--trusted-build复核，11格全通过。记录见regression_b/explicit_build_verification.json；不是正式32格重跑。
- A的额外reject-QA曾启动，耗时偏长后按效率要求中止，未计通过；随后仅完成上述基础校验，不重复采集96窗。
- paced旧verifier将立即queue_full状态误当作已输出净化IQ，触发错误的早输出判定。修正为只对接受输出检查样本因果性；新增一组正反例QA通过。四份已存paced记录各2000条，使用新版verifier独立复核通过，不重跑测量、不覆盖原verification.json。
- 正式测量原provenance绑定测量时的verifier；后置修正版本SHA256为 `9720e8fc32628db8652c5de73d40a21180e3b430f5aff1a5e36181c8a4dffa6d`。复核结果与hash记录在paced/reverification_index.json，原experiment_summary保留最初失败结果以供审计。
- MATLAB/native/UHD/FPGA未实跑；本轮无SFO、PLL transient或真实硬件互通验证。功能、性能、未验证项分别报告。

## 7. 产物与复现

小型版本化结果：`FullPacket_结果摘要_2026-11-06.json`、`FullPacket_计时摘要_2026-11-06.csv`。大IQ和完整记录均在新build/results，不进入Git、不覆盖历史run。

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-fullpacket-compare
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket --target benchmark_fullpacket_compare uwb_python -j2
# 选择全新输出目录；functional按需生成输入，timing/paced独占锁
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket/results/functional_NEW --kind functional
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket/results/timing_NEW --kind timing
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket/results/paced_NEW --kind paced
```

本轮未因统计波动扩展长soak、多worker、全矩阵或硬件实验。可在本交付基础上后续优化整包分数重构，但不把未来优化当作当前结果。
