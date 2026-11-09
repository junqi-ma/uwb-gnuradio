# Full-packet CFO/转换/数值检查：拆分诊断报告（2026-11-06）

已定位主要开销：两次逐样本通用复数乘法。拆分实现中旋转因子生成和信号复乘合计约85%；double→float及显式有限性扫描占比较小。生产融合循环未修改，安全检查没有关闭。

## 1. 方法与口径

- code-tested SHA `4b5da9678503bae0a3b5ddd4c784d304829bf523`，运行时工作区干净。新工具 `benchmark_fullpacket_cfo_diag` 与重构库使用相同generic `-O3 -DNDEBUG -fPIC`，不加native/fast-math。
- 共享生产库SHA256 `512224833f7ebb0c05c6e54617c26091cfe4f1975d1ff0296fea763f3de841e1`，与缓存实验原库一致；新代码只在独立诊断exe及QA中。
- 公开profile为32B/code10/64SYNC/4z2，两格各16个raw窗口，在线重新估计前8SYNC、解通信包、调用原库重构。捕获实际accum、IQ、原model、绝对支持区/相位参考；不读注入sidecar或分量IQ。
- 每变体每轮128次、两轮、每变体8次热身，交错/反转变体顺序，正式运行独占 `/tmp/uwb-sic-benchmark.lock`。计时外将相同快照复制到复用热缓冲。
- 各支持区约13.36万样本。四变体共2048条观测，全部逐次在计时外检查model/试减有限性并与原库model逐位比较，通过率2048/2048；只有16个独立源窗/格，不能把重放数当新包数。
- 新增3组针对性QA（CTEST1/1）：绝对1024grid、负相对相位、CFO0/±50kHz、逐位一致、转换/复乘/试减溢出、非法参数与恢复复用，通过。无新硬件/整包吞吐验收。

## 2. 拆分实现的阶段耗时

以payload_base为例，以下是**拆分实现**的均值；拆循环增加中间缓冲读写，不能将这些子时间直接当成原融合2.3ms的精确分摊。

| 阶段 | 平均耗时us | 占拆分总时间 |
|---|---:|---:|
| 1024基表生成 | 7.44 | 0.3% |
| 逐块相位与旋转因子生成（复乘1） | 1031.14 | 42.9% |
| double→float转换 | 135.23 | 5.6% |
| 信号乘旋转因子（复乘2） | 1000.30 | 41.7% |
| model有限性扫描 | 75.59 | 3.1% |
| 试减 y-model | 64.63 | 2.7% |
| 试减结果有限性扫描 | 86.15 | 3.6% |
| 拆分总时间（含少量计时/分派开销） | 2401.48 | 100% |

旋转因子阶段包含每1024点的块相位指数计算及逐样本base×block；不是每样本sin/cos。生成基表仅约7us，不能把两毫秒级成本归到基表。两处复乘合计约2.03ms；转换约0.135ms，两个有限性扫描约0.162ms，试减约0.065ms。

## 3. 保持融合形态的诊断对照

| 场景 | 原双检查融合 us | 仅model检查 us | 去掉显式检查 us | 拆分双检查 us |
|---|---:|---:|---:|---:|
| sync_base | 2315.17 | 2120.13 | 1958.85 | 2422.48 |
| payload_base | 2275.27 | 2127.74 | 1966.80 | 2401.48 |

payload_base去掉两处显式检查表达式的变体仍需1.967ms，相对原融合仅降低13.6%；主要计算开销仍然存在。仅model检查同时省去了试减表达式，不把差值全部称为“isfinite成本”。编译器代码与流水变化也影响差值。
拆分总时间比原融合增加约5.5%，所以拆循环本身不是提速方案。分阶段数值只用于定位，不能拿拆分/无检查变体替换当前生产循环后宣称完整SIC已提速。

## 4. 编译和反汇编证据

检查了诊断工具及实际生产库的反汇编。生产 `fullpacket_cancel` CFO样本循环在当前库的 `0x144d98`、`0x144dd4` 有两处无条件 `__mulsc3@plt` 调用，分别对应base×block及converted×rotation。

```text
call __mulsc3@plt        # 第一次复乘
cvtsd2ss ...             # 两个double→float分量转换
call __mulsc3@plt        # 第二次复乘
```

每包约13.36万点，对应约26.7万次通用float复数乘法调用。耗时包含复乘算术、通用异常值语义及调用/寄存器搬运，未测出“纯函数调用开销”单独占比。编译器向量化报告显示复乘循环未跨样本向量化（unsupported complex float/control flow）；这不等于完全没有指令级SIMD。

这直接支持优先研究块级VOLK/SIMD复乘，复用已验证前导路径里的批量scalar×complex及complex×complex接口，并保留model/试减有限性检查。不能用全局fast-math或关闭检查作为默认优化。当前未实现该优化。

## 5. 校验、数值状态与局限

- FTZ/DAZ在两组中均关闭；捕获的转换后float、最终model与试减结果分量中，subnormal计数均0。未统计全部中间乘积或硬件assist，因此不将此当作所有非正规数路径的绝对排除。
- 原库采集期间CFO父计时与融合重放量级接近，但只是采集16窗，且缓存布局/调用位置不同；不是第二套稳定全流水线性能验收。
- 计时限定于真实中间量的隔离热缓冲核重放，不含解码、估计、FIR、I/O、队列及最终IQ相减提交；不可据此直接重算完整SIC服务时间。
- 原融合检查循环、FullPacketScratch接口和在线pipeline均未改；诊断去检查变体只写scratch，计时后仍完整检查，不能进入在线IQ提交。
- 稳态短测不证明稳定P99、全CFO范围吞吐或硬件实时性，未重跑历史A/B矩阵或200窗/s回放。

## 6. 归档与复现

完整数据在 `gr-uwb/build-sic-fullpacket-payload-cache/results/cfo_split_diagnostic`，含逐次记录、provenance、输入/工具/库hash、原库反汇编及编译器向量化报告。小型版本化结果：`FullPacket_CFO拆分诊断摘要_2026-11-06.json`、`FullPacket_CFO拆分计时_2026-11-06.csv`。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-payload-cache -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-payload-cache --target benchmark_fullpacket_cfo_diag uwb_qa_uwb_fullpacket_cfo_diag.cc -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-payload-cache -R uwb_qa_uwb_fullpacket_cfo_diag.cc --output-on-failure
# 选择新输出目录；正式计时先独占锁
flock -n /tmp/uwb-sic-benchmark.lock env -u LD_LIBRARY_PATH gr-uwb/build-sic-fullpacket-payload-cache/apps/benchmark_fullpacket_cfo_diag --case gr-uwb/build-sic-fullpacket/results/inputs16/payload_base --out gr-uwb/build-sic-fullpacket-payload-cache/results/cfo_NEW --observations 128 --rounds 2 --warmup 8
```
