# Full-packet SIC：VOLK 乘法加速报告（2026-11-06）

已将两处逐样本通用复乘和double→float转换替换为VOLK批量内核。payload_base的CFO/转换/数值检查由2.316ms降至0.297ms，full SIC由11.062ms降至8.893ms。完整有限性检查与失败旁路保留。

## 实现与边界

- code-tested SHA `1adf4012422a217ab964661d0a0146be5961330d`，正式测量时工作区干净。代码位于 `gr-uwb/lib/uwb_fullpacket_sic_core.cc`。
- 本地GNU Radio `gr-blocks/lib/rotator_cc_impl.cc`及 `/usr/include/gnuradio/blocks/rotator.h` 表明Rotator调用VOLK旋转内核。当前路径本来就在C++，没有Python逐点补偿。
- 本轮采用与冻结前导路径相同的 `volk_32fc_s32fc_multiply_32fc`、`volk_64f_convert_32f` 和 `volk_32fc_x2_multiply_32fc`，按本机VOLK配置自动分派。未直接使用有状态Rotator：保留现有1024点双精度基表及绝对块相位锚定，避免改成float持续递推的误差特性。
- 双精度信道路由累加保持；栈上复用1024点旋转缓冲，模型使用已有scratch，无逐块动态分配。模型和试减结果仍逐点有限性检查，全部成功后才提交相减。未加fast-math或修改库的generic编译选项。
- 原前导SIC核心、信道FIR、前缀缓存策略、解码器和在线输入边界未改。本轮不新增GNU Radio block。

## 有限验证

- Release源码新配build `gr-uwb/build-sic-fullpacket-volk`，未复用CMakeCache或覆盖旧build。CTest 3/3通过：fullpacket core、payload cache、Python verifier。核心新增两组QA覆盖CFO0/±50kHz、负相对相位/块边界、分数多径与旧标量相位算子的误差，以及转换溢出/试减溢出的事务旁路。
- 10格各16源窗，通信精确解码160/160、净化后sensing精确解码160/160；全部verifier通过，状态、解码字节及输入hash与旧功能归档一致。
- 比较320份完整SIC输出IQ（含前导和全包两模式），163份逐位相同。最大绝对IQ差 9.53676135e-07；逐文件最大相对L2误差 4.91457354e-07。这是VOLK与通用复乘的浮点舍入差异，不能声称逐位等价。完整误差清单存 `results/volk_compare/iq_equivalence.json`。
- 保持测试预算：未重跑历史A/B全矩阵、MATLAB或硬件测试。

## 配对计时

32B wire PSDU/code10/64SYNC/4z2，998.4MS/s，快速RX，两组均启用payload_cached。同机串行old→VOLK→VOLK→old；每格每模式每轮256次、两轮，32次热身，16个raw源窗复用。每轮由runner独占 `/tmp/uwb-sic-benchmark.lock`。以下是两轮等样本均值，单位ms，初始化、文件I/O与队列不计入SIC服务时间。重复曝光不作为新独立包。

| 场景 | CFO/转换/检查 旧→新 | 阶段加速 | 信道重构 旧→新 | full SIC 旧→新 | SIC降幅 |
|---|---:|---:|---:|---:|---:|
| sync_base | 2.338 → 0.307 | 7.62× | 5.330 → 3.355 | 9.947 → 7.869 | 20.9% |
| payload_base | 2.316 → 0.297 | 7.81× | 5.552 → 3.518 | 11.062 → 8.893 | 19.6% |

新实现重构阶段分布：

| 场景 | 分数相位滤波/缓存 | 路径叠加 | CFO/转换/检查 | 信道重构父计时 |
|---|---:|---:|---:|---:|
| sync_base | 1.764 | 1.117 | 0.307 | 3.355 |
| payload_base | 1.808 | 1.157 | 0.297 | 3.518 |

CFO已显著缩小，信道重构中剩余主要开销变为分数相位滤波/缓存和路径叠加；通信解码也仍占整包SIC的重要部分。父计时还包含支持计算、缓存key比较、清零等，不能将父子时间二次累加。

本轮验证的是稳态计算收益；没有重新测200窗/s队列或端到端延迟，未证明mean≤0.5ms/P99≤1ms目标。256样本/轮的P99仅保留在CSV供观察，不作为稳定尾延迟结论。

## 版本、库与复现

旧build为 `gr-uwb/build-sic-fullpacket-payload-cache`，生产库来自 `cea0fc3`，其后 `4b5da96` 仅增加隔离诊断工具。旧库hash与上一轮归档一致。runner的provenance.git_head记录本次工作区提交，不能据此误认为旧二进制也由新源码构建；实际绑定以下库hash。

| 变体 | 实际加载库SHA256 |
|---|---|
| old_r1 | `512224833f7ebb0c05c6e54617c26091cfe4f1975d1ff0296fea763f3de841e1` |
| volk_r1 | `3cf40db33eb69649af496763226c2f0e50e46e96ae41689a1229ed34518cc3d1` |

两组均以ldd确认加载各自build/lib，去除LD_LIBRARY_PATH，线程环境固定。库使用generic `-O3 -DNDEBUG -fPIC`；只有RX runner沿用原有 `-march=native`，VOLK自行做运行时分派。

原始记录：`gr-uwb/build-sic-fullpacket-volk/results/volk_compare/{functional,old_r1,volk_r1,volk_r2,old_r2}`；旧run保留。小型结果为同目录 `FullPacket_VOLK加速摘要_2026-11-06.json` 与 `FullPacket_VOLK加速计时_2026-11-06.csv`。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-volk -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-volk --target benchmark_fullpacket_compare_fast_rx uwb_qa_uwb_fullpacket_sic_core.cc uwb_qa_uwb_fullpacket_payload_cache.cc -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-volk -R "fullpacket_payload_cache|fullpacket_sic_core|fullpacket_compare_py" --output-on-failure
# 使用全新输出目录
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket-volk --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket-volk/results/volk_NEW --kind timing --rx-pipeline fast --reconstruction payload_cached --observations 256 --rounds 2
```
