# Full-packet：payload 滤波与固定前缀缓存对照报告（2026-11-06）

已实测：在当前固定32B/同PHY条件下，缓存固定前缀相位行、逐包只滤波数据相关尾部，确实加速。完整包仍施加原有多径/分数延迟/CFO并相减，320份SIC输出与原实现逐位一致。

## 1. 实现、边界与版本

- 分支 `feature/sic-fullpacket-compare`，code-tested SHA `cea0fc3df82e00ea9050c7210710c0e07e1a2a36`，正式run均干净；新build `gr-uwb/build-sic-fullpacket-payload-cache` 从源码串行配置，未覆盖旧build/历史run。
- 开关 `--reconstruction full|payload_cached`，默认full，RX两组均为已验fast AVX2/FMA。新增cache选项未切换重构库ISA、未修改前导SIC核心、未裁减路径或相减支持。
- 缓存的是发射波形固定前缀的分数相位基函数，不是信道输出；路径时延、增益、CFO始终使用当前窗口的冻结模型。缓存key是实际再调制波形的精确前缀字节，PHR/长度/全包幅度归一化改变时失效。没有读取sidecar真值或缓存解码payload。
- 当前编码器连续输入PHR19+RS/payload，再把前21个符号作为物理PHR，因此末两PHR符号含payload依赖。不能把名义payload边界前的所有样本都视为固定。
- 新模式缓存FIR输出 `[0,92528)`，key覆盖输入 `[0,92544)`；名义payload从94656开始，动态区还含末两PHR符号及80样本保护区，共比payload提前2128样本。33tap FIR仍读取完整波形及边界halo，逐tap累加顺序与float32舍入点保持。
- 每个所需非零相位首次使用才填充，缓存命中时复制前缀并计算动态尾部；phase0仍用原直接复制。冷缓存、新相位、key失效重算均计入调用服务时间。
- 相减仍在全部生成/有限性检查通过后一次提交；失败输出原样。完整包CFO、复增益叠加与末尾支持不因缓存减少。

## 2. 验证与预算

针对性CTest 3/3：新增缓存三组QA、原fullpacket core QA、Python verifier QA。覆盖冷/热缓存、不同payload、PHR16/127B与幅度变化、分数多径、31→0相位回绕、整窗out+i_hat逐位对照、FCS/非法模型/NaN/截尾失败旁路。PHR尾部payload依赖也有针对性检查。

固定10格各16窗的候选通信解码和净化后sensing解码均16/16；320份SIC IQ与上一轮fast/整包滤波归档逐位一致。未重跑历史A/B全矩阵或长soak。

正式对照只测32B两个代表场景，16源窗复用、每格每模式每轮512次、两轮，32次热身，顺序full→cached→cached→full；同机i7-12700、同fastRX、同共享库，独占 `/tmp/uwb-sic-benchmark.lock` 串行运行。重复曝光不等于新的独立包。

下列为两轮等样本均值（ms），不含初始化、I/O或队列；P99仅保留512样本尾部观察，不宣称稳定尾延迟验收。

| 场景 | 信道重构 full→cached | 重构降低 | full SIC full→cached | 总时间降低 |
|---|---:|---:|---:|---:|
| sync_base | 9.911 → 5.267 | 46.9% | 14.347 → 9.899 | 31.0% |
| payload_base | 10.269 → 5.526 | 46.2% | 15.652 → 10.916 | 30.3% |

## 3. 收益来源与剩余热点

| 场景/模式 | 分数相位构造 | 路径叠加 | CFO/数值检查 | 信道重构父计时 |
|---|---:|---:|---:|---:|
| sync_base/full | 6.412 | 1.102 | 2.289 | 9.911 |
| sync_base/payload_cached | 1.633 | 1.147 | 2.326 | 5.267 |
| payload_base/full | 6.625 | 1.150 | 2.285 | 10.269 |
| payload_base/payload_cached | 1.729 | 1.161 | 2.371 | 5.526 |

phase计时包含相位行生成及命中复制，channel还含支持计算、key比较和清零；子项不再次加到父计时。分数滤波明显减少后，CFO/数值检查成为重构内较大的剩余项，不能把整个重构耗时继续全归给FIR。

| 场景 | 按相位调用计算的命中率 | 实际FIR输出样本量降低 | 正式计时内key变化次数 |
|---|---:|---:|---:|
| sync_base | 99.28% | 68.77% | 0 |
| payload_base | 99.22% | 68.73% | 0 |

32B波形长133568，命中时动态FIR输出41040样本/相位（payload38912+前置2128），理论上约69.27%的相位FIR输出可复用；少量新相位miss使实测工作量降低略小。工作量计数不等于DRAM流量。phase0本来就不执行33tap FIR，未把它冒充节省的FIR。

## 4. 冷缓存与适用范围

- 标准32次热身排除于稳态均值；初始cache分配另记engine_init_us，新相位或key失效若发生在计时窗仍完整计入。QA及未热身功能记录保留首次填充，不能把稳态均值作为首包保证。
- 固定PHR/幅度条件下可高命中；包长或归一化频繁改变会使这个单key缓存失效，收益会下降。失败不会把旧前缀用于新波形。当前未测混合包长的正式性能。
- 缓存额外预分配约 24.43 MB/worker（32相位行+key）；这不是总RSS。收益以增加常驻内存换取，不能称零成本。
- 本轮只回答计算开销，未重测200窗/s队列/端到端延迟，也未运行MATLAB/native采样链路/UHD/FPGA。SIC均值仍远高于0.5ms，不宣称实时目标通过。

## 5. 归档与复现

完整记录在新build/results/functional_cached、reference_check、timing_abba；旧build/results未改。`functional_cached/equivalence.json`记录320输出一致。小型结果为同目录JSON/CSV：`FullPacket_payload滤波对照摘要_2026-11-06.json`、`FullPacket_payload滤波计时_2026-11-06.csv`。

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-fullpacket-compare
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-payload-cache -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-payload-cache --target benchmark_fullpacket_compare_fast_rx uwb_qa_uwb_fullpacket_payload_cache.cc uwb_qa_uwb_fullpacket_sic_core.cc -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-payload-cache -R "fullpacket_payload_cache|fullpacket_sic_core|fullpacket_compare_py" --output-on-failure
# 选择全新输出目录；full是参考，payload_cached是候选
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/sic/run_fullpacket_compare.py --build-dir gr-uwb/build-sic-fullpacket-payload-cache --inputs gr-uwb/build-sic-fullpacket/results/inputs16 --out gr-uwb/build-sic-fullpacket-payload-cache/results/cached_NEW --kind timing --rx-pipeline fast --reconstruction payload_cached --observations 512 --rounds 2
```

保留显式开关，默认仍为full；未自动推进CFO或其他重构优化。
