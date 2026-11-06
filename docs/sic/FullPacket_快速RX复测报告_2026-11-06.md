# Full-packet SIC 快速 RX 接入与复测报告（2026-11-06）

已完成成熟快速RX接入及配对复测。32B通信包的完整解调平均耗时降低约48%–57%；full-packet SIC平均约15–16 ms，逐包信道重构仍占66%–69%。mean≤0.5 ms、P99≤1 ms与单worker200窗/s的完整功能目标仍未通过。

## 1. 实现与控制变量

- 分支 `feature/sic-fullpacket-compare`，code-tested SHA `28c773446595d22316f56d24f27273790d6c6be1`，所有本轮正式run均 `git_dirty=false`。
- 成熟RX算法已存在于当前 `demod::core::demodulate_one`，与TWR分支相比保留了后来加入的诊断。新增 `apps/fullpacket_rx_pipeline.h` 做预分配封装，通信code10及下游sensing code9同时使用。没有回移旧算法、裁短窗口、关闭同步/CFO修正或降低FCS门槛。
- fast target `benchmark_fullpacket_compare_fast_rx` 使用 `-O3 -DNDEBUG -march=native`，编入既有38tap AVX2/FMA FIR。portable target为同一runner源码，不加本机ISA；两版均无fast-math。编译选项作用于实验runner及inline RX/消费者，未更改生产adapter。
- 两种target调用同一个共享SIC/整包重构库，库SHA256仍为 `bcba46a84664265a51909e6d08641d6c7ea3ffab876e95e821a87f832bd329e0`；SIC数学、八符号估计、分数延迟模型、整包再调制、相减支持与真值隔离未变。
- 编排器新增 `--rx-pipeline fast|portable`，默认fast；逐行核验 `rx_pipeline`、`rx_avx2`，记录target flags.make、二进制和ldd实际库hash。
- 旧计时/大IQ保留，旧测量二进制按原sha256归档；旧报告复现时需显式选择portable。

## 2. 验证与测量范围

新增native wrapper两组针对性QA（32B/code10、127B/code9解码；非法输入/容量/恢复复用）及Python verifier QA，CTest 2/2通过。portable做单窗接线检查；fast在固定10格各16窗上全部通信PSDU/FCS及净化后sensing解码成功。320份SIC输出与上一轮功能集逐位一致。没有重跑共享源码未改的历史A/B全套模拟，也未扩展全矩阵/soak。

正式计算计时：i7-12700，同一16源窗、同一公开profile，各场景每组每轮1024次，32次热身；顺序portable→fast→fast→portable，锁内串行。每个run均同时测preamble与full两组并交错组顺序。OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1，去除LD_LIBRARY_PATH。重复曝光不作为独立包样本数。

主场景wire PSDU为32B（30B数据+2B FCS），64SYNC+8SFD+PHR+payload，波形133568样本≈133.782us；两场景处理context仍139712/236416样本。full仍等待整个公开context，未使用注入起点或真值帧尾缩短延迟。

## 3. 通信解调与 full SIC 总耗时

以下为两轮等样本数平均的service wall time（ms），不是排队或空口延迟。SIC总时间不含下游CIR/sensing解调。

| 场景 | 通信解调 portable→fast | 解调降低 | full SIC portable→fast | 总时间降低 |
|---|---:|---:|---:|---:|
| sync_base | 4.211 → 2.211 | 47.5% | 17.002 → 15.198 | 10.6% |
| payload_base | 6.415 → 2.764 | 56.9% | 19.848 → 15.914 | 19.8% |

逐轮数据（不平均P99）：

| 场景/轮次 | portable SIC mean/P99 | fast SIC mean/P99 |
|---|---:|---:|
| sync_base/r1 | 17.841/29.004 | 15.853/27.367 |
| sync_base/r2 | 16.164/23.509 | 14.543/22.546 |
| payload_base/r1 | 20.930/29.362 | 16.142/25.430 |
| payload_base/r2 | 18.766/23.715 | 15.686/22.056 |

绝对时间和尾延迟有轮间波动，两格的解调与总成本改善方向均一致；保留两个轮次，未追加测到满意为止的重复轮次。历史约21.8ms与本轮约15.9ms可作阶段记录，正式改善比例使用上述本轮portable配对结果，避免混入跨run机器状态差异。

## 4. 更新后的瓶颈

fast full组各阶段两轮平均（ms）：

| 场景 | estimate8 | 通信解调 | 校验 | TX再调制 | 信道重构 | 相减 | 重构占SIC |
|---|---:|---:|---:|---:|---:|---:|
| sync_base | 1.817 | 2.211 | 0.187 | 0.322 | 10.477 | 0.183 | 68.9% |
| payload_base | 1.827 | 2.764 | 0.301 | 0.314 | 10.521 | 0.183 | 66.1% |

主要剩余开销是逐包多径/分数时延/CFO重构：约10.5ms。此库的源码、编译配置与hash均未改，轮间小幅变化不归因于RX算法改动。未知payload的相位行仍逐包计算；前导模板则可预缓存。这里只接入RX加速，没有优化重构器或减少拟合路径/消除支持。

通信解调的既有七阶段均已计时；这些计数是comm_demod的子项，不能再次加到SIC总时间：

| 场景/配置 | timing | CFO | SFD父阶段 | CIR/soft-chip | ns_sfd | PHR | payload/FCS |
|---|---:|---:|---:|---:|---:|---:|
| sync_base/portable | 0.408 | 0.183 | 0.986 | 2.550 | 0.037 | 0.010 | 0.034 |
| sync_base/fast | 0.430 | 0.159 | 0.865 | 0.660 | 0.046 | 0.011 | 0.035 |
| payload_base/portable | 0.404 | 0.307 | 1.122 | 4.491 | 0.039 | 0.010 | 0.037 |
| payload_base/fast | 0.413 | 0.261 | 0.952 | 1.045 | 0.044 | 0.010 | 0.034 |

RX提速主要来自CIR/soft-chip阶段的既有AVX2/FMA路径。快速RX内部的主要项为SFD父阶段及CIR/soft-chip；SFD包含尾部timing/CFO重拟合，不能把该列全当纯SFD搜索。payload译码/RS/FCS本身只有约0.03–0.04ms，不能把完整解调的数毫秒全归到FEC。

## 5. 快速 RX 下的真实 paced 延迟

200窗/s、10秒、单worker、队列容量2、输入池16，source_late均0。以下最终结果延迟从公开窗口首样点到CIR及sensing完整解调结束；P99仅针对完成窗，拒绝仍列入总体账本。

| 场景/组 | 完成/提交 | queue_full | 平均排队ms | 结果P99 ms | sensing成功/提交 |
|---|---:|---:|---:|---:|---:|
| sync_base/preamble | 1997/2000 | 3 | 0.656 | 14.025 | 0/2000 |
| sync_base/full | 574/2000 | 1426 | 32.341 | 65.269 | 574/2000 |
| payload_base/preamble | 1999/2000 | 1 | 0.195 | 7.738 | 0/2000 |
| payload_base/full | 543/2000 | 1457 | 34.262 | 69.600 | 543/2000 |

full接受窗均完成通信解码与sensing恢复，但拒绝仍占多数；单worker完整功能200窗/s未通过。preamble虽然处理更快，这两个整包干扰场景的完整sensing解码仍失败，不能与full的成功终态等价。CPU服务时间与队列/最终延迟分列，未用多worker吞吐代替单窗成本。

## 6. 产物与复现

- 结果：`FullPacket_快速RX复测摘要_2026-11-06.json`、`FullPacket_快速RX计时_2026-11-06.csv`。
- 完整归档在新build/results的 `fast_rx_functional`、`fast_rx_timing`、`fast_rx_paced`；正式run均通过hash、终态、阶段计时及RX配置校验。
- `fast_rx_functional/portable_iq_equivalence.json`记录320输出逐位一致；原始归档未覆盖。
- MATLAB/native重采样/UHD/FPGA与硬件互通未运行；native RX在本文指CPU本机ISA，不表示native采样链路通过。

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-fullpacket-compare
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket --target benchmark_fullpacket_compare benchmark_fullpacket_compare_fast_rx uwb_qa_uwb_fullpacket_fast_rx.cc -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket -R "fullpacket_fast_rx|fullpacket_compare_py" --output-on-failure
# 全新输出目录；fast默认，portable必须显式指定
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket/results/fast_NEW --kind timing --rx-pipeline fast
```

当前结论支持继续单独研究整包重构的热点；未自动开始下一轮重构优化。
