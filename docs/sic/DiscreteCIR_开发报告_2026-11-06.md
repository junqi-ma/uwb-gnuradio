# Discrete CIR full-packet SIC 开发报告（2026-11-06）

本轮执行 `FullPacket_离散CIR估计开发任务书_2026-11-06.md`：用前 8 个 SYNC 直接
估计整数采样网格上的离散等效 FIR，替换 full-packet 路径的“分数路径估计 → 分数
延迟波形 → 路径累加”。旧 `path_fractional`、preamble-only、冻结 preamble
core/adapter、`tools/sic/sic_reference.py` 未改；新后端只作为
`--reconstruction discrete_cir` 显式实验选项。

**结论先说**：在本轮有限样本上，`discrete_cir` 值得作为 full-packet 路径的
替换候选。10 格×16 源窗配对中 comm/sensing 正确解码数不低于旧路径，目标区抑制
和 CIR 恢复全部在 1 dB 容差内（多数持平，`sync_base`/`payload_four_frac` 改善）；
两格正式串行计时中配对总 SIC mean 降到旧路径的 **30.8% / 34.3%**，P95/P99
同时下降。但 `payload_four_frac` 的 12 个 active taps 使重构仍比单径重，且本轮
只覆盖有限信道/无硬件，不能据此直接改默认。

## 1. 交付与版本

- 分支/worktree：`feature/sic-fullpacket-compare`，
  `.worktrees/sic-fullpacket-compare`。
- code-tested commit：`10b660b0f4007864fcb0e6a57ab5f0a86a0a3447`
  （`fix(sic): bind timing quality by measured sources`）。
- 新 build：`gr-uwb/build-sic-fullpacket-discrete`，从当前源码独立配置，未复制
  旧 CMakeCache、未安装 `/usr/local`。
- 实际加载库：`gr-uwb/build-sic-fullpacket-discrete/lib/libgnuradio-uwb.so.1.0.0.0`，
  SHA256 `43e12c18629b9a11c254f85820b0b11e87d4c41312735aae363faed2f5e5ed29`。
- runner binary：`gr-uwb/build-sic-fullpacket-discrete/apps/benchmark_fullpacket_compare_fast_rx`，
  SHA256 `84ddef9f2dd7ac9518c9eef8e739a5389bba399bb966b5db399c15b58267ef7b`。
- 正式 run（大 IQ 留在 build/results，不进 Git）：
  - `results/functional_discrete_20261106_r2`
  - `results/negative_discrete_20261106_r2`
  - `results/timing_discrete_20261106_r2`
- 小型摘要：`docs/sic/DiscreteCIR_结果摘要_2026-11-06.json`、
  `docs/sic/DiscreteCIR_计时摘要_2026-11-06.csv`。

## 2. 实现边界

- 新公共头文件 `gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h`；
  估计器 `gr-uwb/lib/uwb_discrete_cir_estimator.cc`；
  重构器 `gr-uwb/lib/uwb_discrete_cir_reconstruct.cc`。
- 在线估计只读 `y[0, observed_end)`、公开 `sync`/前 8 SYNC `train`、公开 ROI 和
  config；不接收 sidecar、注入时延、CFO、路径或 IQ 真值。
- 独立 acquisition：coarse/fine 相关检测、周期回退、8 SYNC CFO；随后按 `sps`
  折叠 8 个 SYNC 得粗 CIR。**粗 CIR 只用于选择 active lag 组，不把 CIR 当 h**。
- 支持选择：粗 CIR 局部极大值中保留 `>= peak_floor*peak` 的峰，按幅度贪心保持
  `peak_min_spacing` 间距，每个峰激活 `peak±active_radius`；active lag 的并集
  形成连续 hull，hull 内未激活 tap 严格为零。
- 拟合：只在 active lag 上做 double 精度 ridge LS；`X^H X + λI` 的 K×K Cholesky
  分解缓存在固定槽位，key 为 template/config/active lag set/training rows/λ；
  热调用只算 `X^H z` 和三角求解，不显式求逆、不构造 M×L 大矩阵、不分配。
- 重构：直接 FIR，单次用公开 SYNC 参考归一化整包波形；`h` 已含包幅度，不再
  二次缩放。CFO 使用 1024 样本绝对相位块。FIR/CFO 使用 VOLK 内核；tap-major
  连续 AXPY + 预分配 accumulator。
- 事务：FCS/PHR/profile 校验失败、模型不可用、ROI 无重叠、上下文截断、非有限
  输入/重构均 `out==y` 逐位、`ihat==0`；只有 committed 时才在 `[sub_lo,sub_hi)`
  内修改 `out`。

参数由独立 seed `3000000` 的 3 个校准格各 2 窗固定：
`lag_lo=-8`、最大 `tap_count=64`、`peak_floor=0.15`、`peak_min_spacing=5`、
`active_radius=1`、`lambda=1e-6`。校准表见
`docs/sic/DiscreteCIR_校准记录.md`；未用 10 格验收窗口选参。

## 3. 功能对照（10 格×16 源窗）

同源窗配对；旧路径 `--reconstruction full`（path_fractional），新路径
`--reconstruction discrete_cir`；comm/sensing 计数为 PSDU exact 且 FCS 通过。
功能 run `passed=true`，所有 16 窗均满足 1 dB 支持容差。

| 条件 | comm 旧/新 | sensing 旧/新 | 目标区抑制 旧→新 (dB) | CIR 恢复 旧→新 (dB) | 弱目标误差 旧→新 (dB) |
|---|---:|---:|---:|---:|---:|
| sync_base | 16/16 / 16/16 | 16/16 / 16/16 | 29.68 → **35.74** | 29.05 → **36.16** | 0.07 → 0.02 |
| payload_base | 16/16 / 16/16 | 16/16 / 16/16 | 48.23 → 48.23 | 47.74 → 47.74 | 0.00 → 0.00 |
| payload_len16 | 16/16 / 16/16 | 16/16 / 16/16 | 48.92 → 48.93 | 48.56 → 48.56 | 0.00 → 0.00 |
| payload_len127 | 16/16 / 16/16 | 16/16 / 16/16 | 43.36 → 43.36 | 43.97 → 43.97 | 0.00 → 0.00 |
| payload_snr8 | 16/16 / 16/16 | 16/16 / 16/16 | 26.23 → 26.23 | 25.74 → 25.74 | 0.00 → 0.00 |
| payload_isr0 | 16/16 / 16/16 | 16/16 / 16/16 | 38.23 → 38.23 | 37.74 → 37.74 | 0.00 → 0.00 |
| payload_isr20 | 16/16 / 16/16 | 16/16 / 16/16 | 58.23 → 58.23 | 57.74 → 57.74 | 0.00 → 0.00 |
| payload_cfo_m50k | 16/16 / 16/16 | 16/16 / 16/16 | 44.30 → 44.30 | 43.72 → 43.71 | 0.00 → 0.00 |
| payload_cfo_p50k | 16/16 / 16/16 | 16/16 / 16/16 | 44.56 → 44.56 | 44.19 → 44.19 | 0.00 → 0.00 |
| payload_four_frac | 16/16 / 16/16 | 16/16 / 16/16 | 25.00 → **25.89** | 27.94 → **33.03** | 0.01 → 0.01 |

说明：

- 计数是 16 个源窗，95% Wilson 区间约 `[80.64%,100%]`；这是有限样本功能门，
  不是总体解码率证明。
- `sync_base` 的改善来自峰值组 active 支持：旧连续宽支持会把 sensing 前导
  拟合进 h，新路径只保留 comm 粗 CIR 峰。
- 四径分数格 `payload_four_frac` 目标区 +0.90 dB、CIR +5.10 dB；其余 payload
  格与旧路径基本持平。全部格里相对旧路径的最小 delta 是 `payload_cfo_m50k` 的
  CIR -0.01 dB，远小于 1 dB 容差；没有格接近失败边界。
- 目标区/CIR 数值由本轮独立 verifier 对旧/新同时复算；与历史报告使用不同
  verifier，绝对数值不可直接与历史表拼接。

负样本：

| 条件 | 结果 |
|---|---|
| no_interference | committed=0，输出与 raw 整窗逐位一致 |
| no_overlap | committed=1（在 target 区外消除通信包），target SYNC 区逐位不变 |

## 4. 正式串行计时

两格、16 源窗、每变体每轮 16 次预热 + 128 次观测、2 轮、AB/BA，旧/新使用同一
binary/library；独占 `/tmp/uwb-sic-benchmark.lock`；无拒绝行。下表为 SIC service
wall time，不含下游 CIR、sensing 解调、初始化、生成器与 I/O；观测复用 16 个源窗，
不作为独立包样本。

| 格 | 轮次 | 旧 mean / P95 / P99 (µs) | 新 mean / P95 / P99 (µs) | mean 比 | 门槛 |
|---|---|---:|---:|---:|---|
| sync_base | r1 | 12239.7 / 18939.1 / 19358.2 | 3813.2 / 4251.2 / 6096.6 | 0.312 | PASS |
| sync_base | r2 | 12396.1 / 18924.8 / 19660.8 | 3770.0 / 4176.9 / 4848.7 | 0.304 | PASS |
| payload_base | r1 | 13566.3 / 15493.1 / 16065.5 | 4707.4 / 5138.0 / 5529.1 | 0.347 | PASS |
| payload_base | r2 | 13434.4 / 15230.7 / 15910.7 | 4566.8 / 5017.8 / 5567.8 | 0.340 | PASS |

两轮合并：

| 格 | 旧 mean/P50/P95/P99 (µs) | 新 mean/P50/P95/P99 (µs) | mean 比 |
|---|---:|---:|---:|
| sync_base | 12317.9 / 11399.3 / 18958.7 / 19556.1 | **3791.6 / 3715.4 / 4203.7 / 5013.0** | **0.308** |
| payload_base | 13500.3 / 14533.7 / 15404.1 / 16013.6 | **4637.1 / 4586.7 / 5065.0 / 5614.1** | **0.343** |

目标“mean 降低至少 10%、P95 不增加”在两格四轮均通过；实际 mean 降低约
66%–69%，P95 降低约 67%–78%。P99 只有 256 个配对观测，仅作描述性。

新路径阶段（两轮 256 行均值，µs）：

| 阶段 | sync_base | payload_base |
|---|---:|---:|
| estimate（含 acquisition/support/LS） | 470.8 | 468.6 |
| 其中 detect / support / projection / solve / fit_check | 122.9 / 204.4 / 46.1 / 1.0 / 84.0 | 124.7 / 204.3 / 45.0 / 1.2 / 81.0 |
| comm demod | 2000.3 | 2605.5 |
| validate | 156.6 | 275.1 |
| tx（整包重调制） | 268.7 | 278.9 |
| channel（归一化+FIR+CFO+检查） | 669.1 | 675.0 |
| 其中 scale / convolution / cfo_rot | 119.8 / 337.5 / 211.6 | 124.1 / 343.0 / 207.7 |
| subtract | 82.4 | 81.1 |
| copy（含事务双拷贝未归属部分） | 143.7 | 253.0 |
| **总 SIC** | **3791.6** | **4637.1** |

旧路径对应 estimate 约 1.78/1.87 ms、channel 约 7.88/8.19 ms、总 SIC 约
12.32/13.50 ms。新路径的估计和重构都已不再是主瓶颈；**当前最大单项是 comm
demod（2.0–2.6 ms）**，其次是 estimate 的 support 选择（约 0.20 ms）、
channel 的 FIR/CFO（约 0.34/0.21 ms）和事务双拷贝（约 0.14–0.25 ms）。
若下一轮继续优化，应优先减少重复 y/out 拷贝和 support 选择成本，而不是继续压
FIR；本轮不扩大优化范围。

缓存/冷启动、失败路径和 scratch：

- `engine_init_us` 在 timing run 中为 48.6–56.4 ms/进程，包含模板 Gram/factor
  cache 构建、RX/radar scratch 和读模板；在 SIC service 计时窗口之外，不摊入
  mean/P95。
- 每次计时 run 的每个变体每轮先做 16 次 warmup，warmup 行不进入统计；正式
  128 行是热态。首窗冷态单次实测约 7.9 ms（payload_base），明显高于热态均值，
  故不把冷态混入热态分布。
- 失败/旁路：`no_interference` 的 `below_detect_threshold` 路径 `sic_total`
  约 0.678 ms；`no_overlap` 正常提交（在 target 外消除）约 7.60 ms（含 comm
  demod 等完整工作）。失败路径不会被伪造成成功延迟。
- scratch 容量在 `reserve/prepare` 固定；按当前 capacity 估算，算法缓冲约
  15 MB（Gram 1.0 MB、16 槽 factor cache 1.0 MB、energy prefix 1.9 MB、
  z 3.8 MB、waveform/scaled/accumulator/fir_tmp 各约 1.9 MB，另加 modulator
  和输入输出缓冲）。未测进程 RSS 峰值，不把估算容量称为总内存。

## 5. 复现

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-fullpacket-compare
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-discrete \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-discrete \
  --target gnuradio-uwb benchmark_fullpacket_compare_fast_rx uwb_python -j12

# targeted QA
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-discrete \
  -R "discrete_cir|fullpacket_sic_core" --output-on-failure

# functional (10 cells x 16 windows; fresh output)
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 tools/sic/run_discrete_cir_compare.py \
  --build-dir gr-uwb/build-sic-fullpacket-discrete \
  --inputs gr-uwb/build-sic-fullpacket/results/inputs16 \
  --out gr-uwb/build-sic-fullpacket-discrete/results/functional_NEW --kind functional

# timing (requires the passed functional summary; 128 x 2, AB/BA)
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 tools/sic/run_discrete_cir_compare.py \
  --build-dir gr-uwb/build-sic-fullpacket-discrete \
  --inputs gr-uwb/build-sic-fullpacket/results/inputs16 \
  --out gr-uwb/build-sic-fullpacket-discrete/results/timing_NEW --kind timing \
  --quality-summary gr-uwb/build-sic-fullpacket-discrete/results/functional_NEW/experiment_summary.json \
  --observations 128 --rounds 2 --warmup 16

# negative
env -u LD_LIBRARY_PATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 tools/sic/run_discrete_cir_compare.py \
  --build-dir gr-uwb/build-sic-fullpacket-discrete \
  --inputs gr-uwb/build-sic-fullpacket/results/inputs_negative \
  --out gr-uwb/build-sic-fullpacket-discrete/results/negative_NEW --kind negative
```

每次 run 输出 `provenance.json`、`experiment_plan.json`、`experiment_summary.json`；
功能 run 另有 per-cell `comparison.json`/`.csv`、`comparison_report.md`；timing
另有 `timing_summary.json`、`timing.csv`、`timing_report.md`。

## 6. 未验证项与边界

- MATLAB 未运行；native 重采样、FPGA/UHD/硬件、TWR 均未测试。
- 正式计时是预缓冲 service wall time，不是 paced 真实到达延迟；不宣称
  mean≤0.5 ms / P99≤1 ms / 200 窗/s 已达标。
- 功能对照是有限样本（16 源窗/格）门，不是总体解码率证明；跨信道、跨硬件、
  多干扰、SFO/PLL 瞬态未覆盖。
- `payload_four_frac` 的 12 个 active taps 在单径格之外增加了重构成本；本轮正式
  计时未包含该格，不能把两格数字外推到全部信道。
- 旧路径/新路径的模型 ID、q、CFO 独立估计，不要求相等；配对只要求同源窗、
  同输入 hash、同 binary/library。
- 本轮没有把 `discrete_cir` 改为默认；是否改默认应由下一轮在更宽信道/ paced
  到达/硬件证据上决定。

## 7. 建议的下一步（不自动执行）

1. 先做一次 paced/分块到达对照，确认新路径的总 SIC 优势在真实样本等待下仍成立；
2. 在保持质量门的前提下，减少 `fullpacket_cancel_discrete` 与 `apply` 的重复
   y/out 清零拷贝（当前约 0.14–0.25 ms/窗）；
3. 若要继续扩大信道覆盖，优先补多径/弱路径和 SFO 诊断，不用验收窗口扫 λ 或
   support 参数；
4. 不改默认，直到上述证据齐全。
