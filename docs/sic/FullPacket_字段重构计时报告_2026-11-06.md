# Full-packet SIC：SYNC+SFD / PHR / payload 重构计时（2026-11-06）

热缓存时，payload_base中SYNC+SFD约0.874ms、PHR约0.321ms、payload约1.836ms，公共开销约0.249ms。payload占分段重构总耗时约56%；前导缓存省掉了大部分前导FIR计算，但前导仍需路径累加与CFO处理。

## 实验方法与边界

- code-tested SHA `32473f28a421bdf6f3f63eeba77fff807b28beca`；生产库未修改，与VOLK加速报告实际加载的库hash相同。新工具 `benchmark_fullpacket_segment_diag` 只链接独立分段诊断副本，不替换线上函数，不新增GNU Radio block。
- 复用已有 `gr-uwb/build-sic-fullpacket-volk` 增量构建诊断exe，generic `-O3 -DNDEBUG -fPIC`，未加native或fast-math。生产库与诊断函数由同一exe交替调用、分别维护scratch和前缀缓存。
- 两格各16个raw IQ源窗，模型只从公开前8SYNC估计，通信包现场解码后捕获冻结模型和wire PSDU；不读注入sidecar/IQ真值。模型估计与解码在本实验计时外，抓取解码采用portable路径，不影响被测重构。
- 32B wire PSDU含FCS；64SYNC+8SFD，前导 `[0,73152)`；21个物理PHR符号 `[73152,94656)`；payload `[94656,133568)`，共38912样本。不能按19个PHR信息位划分空口波形。
- 滤波按上述发射波形输出坐标分段，每段仍读取完整wave及halo，不分别零填充生成三个包。路径叠加/CFO在接收输出时间坐标按 `floor(min estimated tau_abs)+字段边界` 分段，尾部多径延展归入payload。边界附近可含相邻字段的多径贡献，这是计算时间区间归因，不是三个独立信号分量的物理分离。
- 热缓存前每变体16次热身，正式每轮128次、两轮，交替并反转原库/诊断顺序。16源窗重复曝光不计为更多独立包。每格每变体256条稳态记录；本实验全程独占 `/tmp/uwb-sic-benchmark.lock`。
- 冷缓存记录单独每源一次：调用前清key/phase mask，分配排除；只是相位结果缓存冷，不代表CPU缓存冷或进程首包延迟。热身和正式计时前两变体均重新从相同空缓存开始。

## 热缓存分段结果

下表全部来自分段诊断函数的均值，单位ms。滤波包含缓存命中复制、动态区域滤波及必要的相位行构造；路径项含复增益叠加，CFO项包含旋转、类型转换、模型与试减有限性检查。

| 场景 | 字段 | 相位滤波/缓存 | 路径叠加 | CFO/转换/检查 | 字段合计 | 占重构总计 |
|---|---|---:|---:|---:|---:|---:|
| sync_base | SYNC+SFD | 0.138 | 0.555 | 0.165 | 0.857 | 27.5% |
| sync_base | PHR | 0.110 | 0.159 | 0.045 | 0.314 | 10.1% |
| sync_base | payload | 1.418 | 0.293 | 0.077 | 1.788 | 57.3% |
| sync_base | 公共开销 | — | — | — | 0.159 | 5.1% |
| sync_base | **重构总计** | | | | **3.118** | **100%** |
| payload_base | SYNC+SFD | 0.143 | 0.561 | 0.170 | 0.874 | 26.6% |
| payload_base | PHR | 0.113 | 0.162 | 0.046 | 0.321 | 9.8% |
| payload_base | payload | 1.455 | 0.299 | 0.083 | 1.836 | 56.0% |
| payload_base | 公共开销 | — | — | — | 0.249 | 7.6% |
| payload_base | **重构总计** | | | | **3.280** | **100%** |

公共开销是channel父计时减去三字段全部子计时的余量，包括清零、支持计算、缓存key校验、公共CFO基表与分段/计时开销等；没有进一步声称这些余量的精确内部比例。不包含估计、解码、输入校验、重新调制或最终相减。

## 拆分对计时的影响

分段把原连续循环拆成多个区间，重复少量FIR系数生成并改变访存局部性/VOLK块边界，因此不能把分段数值当作原库精确剖面。同步运行未修改的原库作为对照：

| 场景 | 原库重构均值ms | 分段诊断均值ms | 分段相对原库变化 |
|---|---:|---:|---:|
| sync_base | 3.236 | 3.118 | -3.7% |
| payload_base | 3.436 | 3.280 | -4.5% |

这里的分段总时间更短约3.7–4.5%，包括循环拆分引起的访存/编译影响；没有测得各因素的独立因果贡献。未按总时间比例把分段数字强行缩放为生产值，也未将本诊断副本宣布为新的生产优化。

## 缓存冷启动对照

下列仅为每格16次相位缓存冷诊断的均值，样本少且冷对照总先原库后诊断，仅说明缓存影响，不作为稳定延迟结论：

| 场景 | SYNC+SFD ms | PHR ms | payload ms | 公共 ms | 分段总计 ms | 同轮原库总计 ms |
|---|---:|---:|---:|---:|---:|---:|
| sync_base | 4.284 | 1.010 | 2.023 | 0.233 | 7.550 | 8.722 |
| payload_base | 3.940 | 0.882 | 1.860 | 0.306 | 6.988 | 7.363 |

热缓存正式记录中两变体均无phase miss（sync_base624次hit，payload_base640次hit，每变体），所以前导滤波项主要是复制而不是重新卷积。冷记录会重新计算大段前缀FIR，前导时间明显增加。动态PHR段为2128样本、payload38912样本：PHR末两物理符号依赖payload，不能全部缓存。缓存区的读取本身仍有成本。

## 输出验证与测试预算

- 新增诊断自测CTest1/1：16/32/127B、冷/热cache、分数多径、负相对相位、PHR/payload边界和FCS失败旁路，对照原库。
- 32个实际源窗上，冷/热身/稳态共1152次输出检查均有限且与捕获的原库模型/相减输出在2e-6绝对阈值内；其中972个模型逐位一致，最大模型绝对差2.3283e-10、最大相对L2差1.2337e-12。差异来自分段后VOLK块边界/浮点路径，未宣称逐位等价。
- 每条记录检查公共项非负及分项加总与父计时一致。未重跑10格功能矩阵、历史A/B矩阵、MATLAB、硬件或200窗/s端到端测试。本实验只回答重构字段的计算分布。

## 归档与复现

原始记录位于 `gr-uwb/build-sic-fullpacket-volk/results/segment_diagnostic`；包含provenance、两格windows.jsonl/summary.json与运行日志。源码、二进制、实际加载库、输入hash及编译选项均绑定，旧run保留。小型摘要JSON与CSV在本报告同目录。

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-volk -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-volk --target benchmark_fullpacket_segment_diag -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-volk -R "uwb_qa_fullpacket_segment_diag$" --output-on-failure
# 在干净代码提交上运行，选择全新输出目录
env -u LD_LIBRARY_PATH python3 tools/sic/run_fullpacket_segment_diag.py --build-dir gr-uwb/build-sic-fullpacket-volk --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket-volk/results/segments_NEW --observations 128 --rounds 2 --warmup 16
```
