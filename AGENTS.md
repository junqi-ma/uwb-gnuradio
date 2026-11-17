# 前端自相关 SIMD 化与 `-12 dB` 参数升级（2026-11-17）

已按 `docs/performance/前端SIMD与-12dB自相关参数升级方案_2026-11-17.md` 交付，
报告 `docs/performance/前端SIMD与-12dB自相关参数升级开发报告_2026-11-17.md`，
数据 `docs/performance/data_frontend_simd_20261117/measurements.json`。
只改 `gr-uwb/apps/uwb_dw3000_live.cc` 的流式前端捕获与
`tools/rx/qa_live_dw3000_console.py`；后端解调/锁定/融合/SIC 未动。

- **验收**：`--tone on` 0.5 s 纯 AWGN 回放 `frontend_us` **87.9–94.7 ms**
  （5 次，中位 92.6；目标 `<=125`，任务书 `-12 dB` 标量调参版是 `310.6 ms`），
  `--tone off` `91.4–99.5 ms`（中位 96.5，并发负载时见过 116.8 的尖峰）；`raw_drop_samples=0`、`gaps=0`、虚警 `0`。捕获率
  `+10/0/-12/-14 dB` 全部 **`acq_seeds=100/100`**，`INCOMPLETE_WINDOW` 恰好 1
  （第 0 包在样点 0），`OVERLAPPING_CANDIDATE=0`。CTest **52 项 51 通过 1 跳过**。
- **默认参数**：`acq_span 64000→192000`（192k 全速率积分对，不是 24k 抽算）、
  `acq_step 8→64`（只作环表粒度）、`acq_low 0.0015→0.00015`、
  `acq_peak 0.0028→0.00030`；`acq_persist` 不变。
- **触发改为三角形峰顶**：新增 `peak_at`，`fall_frac=0.6`、`fall_after=65536`；
  提交判定放在 `metric>=low_thr` 之前，无噪突发从 1.0 直落 0.0 也能提交；
  种子 `peak_at+lag/2`。旧的 leading-edge `+span/2` 在 192k 窗下随 SNR 漂移
  十余万样点。
- **两处偏离任务书（都是实测逼出来的）**：①累加核用整数 `vpmaddwd` 而非
  float——Alder Lake 的 `int16→float` 加宽转换只走唯一 shuffle 端口，光转换
  就 50 ms、float 版 92 ms，整数版 **66 ms**；输入先 `vpaddsw +4` 四舍五入再
  `>>3`（13 位），int32 每 64 样点冲入 float，任意 `--acq-step` 上界 `2.7e8`，
  且 `|x|<=4096` 使 `vpsignw` 取反精确，`-32768` 溢出不可能；度量是比值，
  均匀缩放精确抵消，噪底偏差 **0.37%**（直接截断是 10.5%）。②接收**直写环形
  缓冲**，取消整条流 staging 拷贝（实测拷贝 45–70 ms，任务书估 75–85 ms 偏高），
  稳态 `frontend_copy_us=0.13–0.22 ms`；回绕块仍走 staging，gap 用 `memmove`
  重定位，`INVALID_GAP`/`valid_begin`/`acq.reset` 语义不变。
- **顺带修掉**：标量尾与环回绕回退曾按原始 int16 累加（与 SIMD 的 `>>3` 差
  64 倍，`--acq-step` 非 8 倍数时直接产生虚警）；每块一次串行 double 除法
  （19 ms）改为门限/峰值/回落全部交叉相乘，纯噪下 0 次除法。
- **新增诊断**：`COST_TOTAL` 增加 `frontend_copy_us`、`frontend_scan_us`、
  `acq_seeds`。`acq_seeds` 是前端产生的种子数，`completed` 只是 EOF 前跑完的
  任务数（前导-only 夹具下 `+10 dB` 每包约 8 ms 后端重试，必然积压）。
- **任务书预期不成立的两点**：`proto_low_snr_signal.py` 生成的是**纯前导**
  （无 SFD/PHR/payload），`+10/0 dB` 的 `fcs=100/100` 不可能达成，可判读指标
  只有 `acq_seeds`；`completed>=99` 只在 `-12/-14 dB`（后端快速 TIMING_FAILED）
  成立。
- **`SCALAR` 构建不再是实时配置**：全速率标量 `frontend_us≈462 ms/0.5 s`
  （4.9× AVX2），非 paced 回放会丢样；只作可编译/可验证回退。实时必须 AVX2。
- **未验证**：上板/UHD、paced 到达、8 worker、负载下 P99、多径/AWGN/ISR、
  CFO 扫频与 SFO、MATLAB 交叉验证、`-16/-18 dB`、gap 与块回绕同时发生的组合。

---

# 当前版本：原生 UHD/C++ 与 GNU Radio 适配层已拆分（2026-11-16）

主分支 `feature/sic-research`。`4ecb4f3` 保存原主路径源码/新文档；`5d5ba26`
回合 GNU Radio 解耦与历史路径清理。现行构建、依赖、参数与安装入口见
[原生接收与 GNU Radio 构建使用说明](docs/原生接收与GNU_Radio构建使用说明.md)。

- `uwb_core` / `uwb_native` 与 `gnuradio-uwb` adapter 拆分；
  `UWB_BUILD_GNURADIO=OFF` 可无 GR/PMT 构建，VOLK 保留。
- `UWB_BUILD_EXPERIMENTS=OFF` 关闭部分实验目标；两开关默认仍为 ON。
  UHD 使用 `ENABLE_UHD_BACKEND=AUTO|ON|OFF`。
- 已移除 `convert_cache`、`both` 和 pipeline `resampler_workers`/旧 runner 参数；
  FIR 诊断开关在 `DemodScratch`，内部线程池和 legacy 对照尚未移除。
- C++ live 默认 `--fused on --cir full`，每包网格拒绝时 Full→Bypass；Python
  wrapper 仍传 `--cir auto`，完整新参数用 C++ 入口。不得删除分数网格/融合回退。
- 主路径原生精简构建：50 项 CTest，49 通过、1 安装消费者跳过；此前同版 worktree
  原生 53/53、GR 87/87。动态 locked 逐包一致性限制仍保留。
- 主线连续 UHD 已有上板记录；本轮拆库未上板。timed 实际 UHD 命令仍未实现，
  必须保持无 replay 时 fail-closed。原生消费者须重新链接新库，不保证旧二进制 ABI。
- 编译期 ISA 由 `UWB_SIMD=AUTO|AVX2|AVX512|SCALAR` 统一决定（默认 AUTO，本机
  Zen 5 解析为 `avx512`）；`-march=native` 已从构建中移除。AVX-512 只出现在独立
  TU（函数 target attribute），输出与 AVX2 逐位一致并有 QA 守护；默认只对重采样
  `H_pad 24/48` 与 demod/SIC 的 `sync` 组启用。

以下为按阶段保留的历史快照；其中旧分支名、构建路径、未提交/未上板、默认参数和
测试计数均以对应报告日期与范围解释。后文开发规则继续适用。

---

# 距离接收优化：多 Subagent 并行执行（2026-11-16）

执行 `docs/performance/多Subagent并行开发方案_距离接收优化_2026-11-16.md`。
基线 `717b351`，集成分支 `exp/rx-opt-integration`，最终 `d16f6db`。
报告 `docs/performance/距离接收优化_多Subagent执行报告_2026-11-16.md`。
**只做软件/文件回放，未上板。**

- **7 距离 ABBA（3 次配对，全部 `raw_drop_samples=0`）**：`compute` 1687.2 →
  **1414.8（−16.1%）**、`demod` 964.3 → **749.1（−22.3%）**、`st_timing` 288.8 →
  **157.2（−45.6%）**、`sfd_cforef` 237.9 → **159.9（−32.8%）**、`resample` 722.8 →
  **665.7（−7.9%）**；CRC-OK 13894 → **13912**；5.2 m PRR 96.27% → **97.00%**、
  `TIMING_FAILED` 20 → **4**。规划目标 1250–1300 未达成。
- **建议 1（去掉 SFD 二次 CFO/全帧去旋转）不成立**：Δf p90 318–348 Hz、跨度
  279,467 样点上漂移 p90 0.70 rad（max 1.50），只有 19% 的包在 0.10 rad 内；
  跳过第二次旋转只是把工作搬到第一次（`st_cfo` 22→240、`st_sfd` 249→34、`demod` 不变）；
  收紧区间后 `cforef` −27 µs 被 `st_payload` +28 µs 抵消。**去旋转只能变快不能变少**，
  因为它是把下游要读的样点物化到 `scratch.derotated`。
- **建议 7（周期累加捕获）在 5.2 m 无对象**：`NOWIN=50` 中 45 个是文件前导静默
  （前 45.656 ms 文件里没有包，三条独立证据），4 个是锁定入口流水线延迟，1 个是文件尾。
- **建议 5（锁定态 SFD 定点验证）主动取消**：`st_sfd` 的 87% 是 `sfd_cforef`，
  搜索本身只有 ~35 µs。该 worktree 改投去旋转内核。
- **实际交付**：①锁定态 `pre_guard` 36864→8192（`--locked-pre-guard`，按锁状态生效，
  `set_pre_guard` 越界 fail-closed）；②payload 子阶段打点（证明 Viterbi+RS+CRC 只占
  6.8%，热点是 fused CMF 120 µs 与 payload 去旋转 114 µs）；③锁定态稀疏锚点网格跟踪
  （16 锚点 bootstrap + 每符号一次 8-lag argmax，`grid_fb=0`）+ 失败路径稀疏尾部重探
  （挽救 17/21 个 5.2 m TIM）；④CFO 去旋转内核块锚点折进表、数据单遍
  （0.250→0.190 ns/样点，max abs ≤2.67e-7、rel L2 ≤6.5e-8）。
- **`measured_period` 的一处偏离**：任务书要求只由锚点拟合，实测 16 锚点 SE 是 256 点
  的 4 倍、在 2.5 m 造成 1 包/2000 payload 翻转；改为「锚点 bootstrap + 全 refined 峰
  拟合」（与 legacy 同一估计器），`integer_chip_grid`/`grid_reject` 语义不变。
- **验收**：AVX-512 与 AVX2 各 55 项 CTest、54 通过 1 跳过；AVX2/AVX-512 逐包解码
  0 差异（0.8 m 2035 窗、3.8 m 2006 窗的 status/PSDU/FCS 全等）；`zmm` 1770 vs 0。
- **夹具不可逐次复现**：4 worker + locked 反馈下同一二进制两次运行 CRC-OK 差 ±1…4，
  单次对单次比较无效，必须 ABBA 配对。负载下 2.5/5.2 m 偶发整段失锁（基线与新二进制
  都出现）；空载 2.5 m 基线 10/10、新二进制 10/10 全 OK。
- **未验证**：上板/UHD、paced、8 worker、负载下 P99、多径/AWGN/ISR、CFO 扫频/SFO、
  MATLAB 交叉验证、`--locked-pre-guard` < 8192、`--locked-roi 64/128`、
  `--cfo-refit auto|skip` 在 27 ppm 未 trim 时钟下的行为。
- **下一步 P0**：把 CFO 相位斜坡折进 fused CMF / soft-chip 内核（按 chip 递推，
  误差 <0.002 rad/chip），这是消除剩余约 280 µs 去旋转的唯一途径；需先立误差合同。

---

# AVX-512 内核合并入主分支（2026-11-16）

已把 `/tmp/uwb-gnuradio-avx512`（`perf/avx512-kernels`）的两轮 AVX-512 工作
合回 `feature/sic-research`：`ef247fa`（编译期 ISA 选择 + 65/48、65/32 重采样）、
`d5598d9`（demod/SIC 内核组 + 宽内核），合并提交 `9fc9c49`。规划/报告见
`docs/performance/AVX512内核与编译期CPU选择开发报告_2026-11-16.md` 与
`docs/performance/解调与SIC_AVX512内核扩展开发报告_2026-11-16.md`。
- **构建开关**：`UWB_SIMD=AUTO|AVX2|AVX512|SCALAR`（默认 AUTO）取代散落的
  `-march=native`；生成的 `uwb_simd_config.h` 是唯一真源，header-only 内核不会
  在不同 TU 编成不同后端。`AUTO` 原生 Linux 读 `/proc/cpuinfo`，交叉编译拒绝猜测；
  基线 ISA 的探针对象在任何宽指令可达之前就能拒绝不支持的机器（不是 SIGILL）。
- **逐位一致**：AVX-512 的 lane 映射、FMA 顺序、累加树与 AVX2 完全相同，输出是
  逐位一致而非容差一致，QA 对每个已发布 `H_pad` 强制该性质；`-ffp-contract=off`
  未改，未加 `-ffast-math`，既有 QA 容差未放宽。
- **默认只开两处**：重采样 `H_pad 24/48`（ABBA core +9.40/+9.43% 与
  +20.11/+21.62%），demod/SIC 的 `sync` 组（同二进制 A/B：整包 −6.13/−5.77/−5.62%、
  preamble −7.68/−8.08/−7.32%，`timing` −25.21%，`comm_rx` −8.14%）。
  `H_pad 16/32`（+2.22/+2.87%、+6.21/+4.39%）、`cfo`/`cir`/`sic_*` 组保持 AVX2；
  FC32 `process()` 默认也保持 AVX2（逐宏块调用曾退化 ~8%）。
- **被否决的候选**：融合 CMF 52 点复点积 +29%、38 抽头 CIR FIR +94%、
  20 抽头 SIC 脉冲累加 +2%（store-forwarding 限制）。本机 Zen 5 的 512 位
  `moveldup/movehdup/permute_ps` 半速，所以复乘减半指令数无收益。
- **故意没动**：`timing` 的 `pw` 与 `sfd` 的 `pwr` 串行浮点功率求和。向量化会改
  归一化分母、可能翻转近等峰 tie-break，需先立误差合同（实测 365.6 ns → 45.9 ns，
  7.97×，两项合计约 65 µs ≈ 整包 18%）。
- **等价性**：SIC dump `sic_out_preamble.bin` / `sic_out_fullpacket.bin` 各
  1,084,544 B 在 A/D 之间逐字节相同，抑制比 63.29/63.33 dB 与 `OUTPUT_L2` 不变。
- **合并后主路径复核**：`UWB_SIMD=AVX512` 与 `AVX2` 两个 in-tree 构建各
  **54 项 CTest 全部通过**（`uwb_qa_native_install_consumer` 按设计跳过）；
  AVX-512 构建 `libuwb_core.a` 有 1770 条 `zmm`、AVX2 构建 0 条；
  `uwb_kernel_backend.cc` / `uwb_simd_runtime.cc` 两个基线 TU 均 0 条。
  `UWB_SIMD=AUTO` 在本机解析为 `avx512`。
- **未验证**：上板/UHD 实机、paced 到达、MATLAB 交叉验证、8 worker、负载下 P99、
  多径/AWGN/ISR、CFO 扫频与 SFO。本轮只做软件/回放验收。
- 分支带入的旧文档快照（`AGENTS.md`、`README.md`、`docs/README.md`、
  `gr-uwb/README.md`、`tools/rx/README.md`、`开发状态.md`、`uwb_dw3000_live.cc`）
  **未**随合并进入主分支，主线保留自己更新的版本。

---

# 接收信噪比与链路余量实测（2026-11-16）

- 条件：DW3000 **发送衰减 10 dB、距离 1.5 m**；X410 ch3/RX1、6489.6 MHz、增益 60、
  737.28 MS/s；code 10、256 SYNC、127 B、6.81 Mb/s。无功率计标定，只有相对量。
- **带内 SNR = +13.7 dB**（信号/热噪声，均在 ±250 MHz 信道 5 带宽内）。5/2/1 ms
  三次采集 **+13.62 / +13.70 / +13.71**，信号 rms 541/542/544 LSB、热噪声
  rms 139.5/138.3/138.8 —— **周期缩短不损失链路预算**。
- **热噪声底 −47.5 dBFS**（rms 139 LSB），比 16 bit 量化底高 48 dB；峰值 −17.5 dBFS、
  **0 削顶**。加接收增益不会改善 SNR。
- **−232.96 MHz CW 单音比热噪声高 12–13 dB**，与信号同量级（rms 576–640 vs 541）。
  不消单音带内 SNR 掉到 **−1 dB**（全带宽 −0.8…−1.6 dB）。
- **单音消除是必需项，不是优化**：`--tone on` 99.5% FCS，`--tone off` **33%**。
  机制不只是 SNR：单音在采集器 `lag=750` 处整周期相干（−91/288×750 = −237 周期），
  归一化延迟自相关被持续触发，`candidates` 989→**1239**（0.8 ms holdoff 限速上限），
  `dropped_queue=490/500`，真包窗口被假候选挤掉。
- **链路余量 ≈ +10 dB**（噪声注入等效额外路损）：0/3/6/10 dB 均 99.5% FCS；
  14 dB 掉到 93.2% 且 `candidates=1`（靠单个包偶然过门限）；**18 dB 时
  `candidates=0`、`expected_slots=0`，采集彻底失效**。瓶颈是
  `Acquisition::scan` 的固定门限（`metric>=0.15`、`peak>0.5`、跨度≥36864），
  **不是解码器**（解码器在 −0.5 dB 带内 SNR 下仍过 93%）。
- 噪声区另有 WiFi 6E 类宽带凸起（−178/−100/−93/−88/−58.88/+94.72 MHz，各占热噪声
  10–20%），对 SNR 影响 <0.2 dB，但会限制远距离下限。
- 新增 `tools/rx/measure_rx_snr.py`；数据 `docs/performance/data_rx_snr_20261116/`；
  报告 `docs/performance/X410_接收信噪比与链路余量_2026-11-16.md`。
- **未验证**：功率计标定、距离/衰减扫描（余量是注入等效的）、多径与遮挡、
  余量实验只在 1 ms 复验、长时间 soak 的噪声底漂移。

---

# 重采样默认抽头改为 Hpad 16（2026-11-16）

- **原默认不是 32 而是 Hpad 48**：`uwb_dw3000_live` 加载 `taps_quality_minorder.txt`
  （T=2707）。内核成本正比于 `Hpad = 8*ceil(ceil(T/65)/8)`，不是 T。
- 新增 `--taps eq16|quality|eq32|realtime`，**默认 `eq16`**（T=1039，Hpad 16，
  纹波 0.0868 dB、全阻带 −74.56 dB、通带 290 MHz）。
- **速度（2 ms/W=4，3 轮，`dropped_queue=0`）**：quality compute 3844.7 /
  resample 2332.6；eq16 compute **2558.8（−33.4%）** / resample **1106.4（−52.6%）**；
  eq32 −20.2%/−34.2%；realtime −29.6%/−47.8%。5 ms/W=1 同向：resample
  1925.9→958.9（−50.2%）。重采样降幅与 Hpad 不成正比（3 倍 Hpad 只换 2 倍速度），
  说明内核有固定装载/写回开销。
- **1 ms 下这是能否跑通的分界**：W=4 每 worker 从 **92.8% 降到 64.3%**；
  实机 1 ms 10 s 达 **99.81% FCS（8203/8219）、0 丢样点、0 丢队列**，
  compute 2398.8 / resample 1062.7 / 每 worker 60.0%。
- **eq16 没有系统性解码损失**：四个系数集的失败包**互不重叠**，且每个失败包的特征
  都是该系数集下 `tmetric` 掉到 ~0.67（同包在其他系数集 ~0.70–0.79）→ 是群时延
  对齐的边缘包（48→1353、32→1039、24→659、16→519 work 样点），不是滤波器质量。
  ~1500 包/配置下失败 0–3 个（0–0.2%）。
- **未验证**：SIC + eq16（等波纹报告要求 realtime 档先补 SIC 验证）、
  多径/AWGN/ISR、MATLAB 交叉验证。`--taps quality` 可随时回退。
- 报告 `docs/performance/X410_2ms周期收包解调报告_2026-11-15.md` §9。
- 全量串行 CTest 87 项全部通过。未提交、未推送、未 install。

---

# 当前提交快照：X410 目标周期锁定与随机干扰双路径（2026-11-15）

已按 `docs/performance/X410_目标周期锁定与随机干扰双路径修改规划_2026-11-15.md`
交付 P0–P4 软件部分。报告
`docs/performance/X410_目标周期锁定与随机干扰双路径开发报告_2026-11-15.md`。
基线 `a8f0016`；未 install、未推送、**未上板**。
- **新增原生核心**（GNU Radio 运行时无关、先 QA 再接 live，不新增 GR block）：
  `uwb_target_lock_core.h`（`ACQUIRE→VERIFY→LOCKED→HOLDOVER→REACQUIRE`、有界反馈
  队列、每 k 独立取整的预测、holdover 扩窗、干扰分类/去重）、
  `uwb_timed_rx_plan.h`（设备定时计划/装配/8 种互斥终端、单音与目标窗物理区间合并）。
- **`LockedBounded` 同步策略**：`Qm35825Profile::sync_strategy`，默认
  `AcquireLegacy` 行为逐条不变。锁定态只在 `seed±locked_roi_samples`（默认 256）内
  搜索，可按已知符号网格探测 `locked_inner_symbols`（默认 4）个内侧 SYNC 并换算回
  包起点；**不**粗扫、**不**整网格探测、默认**不**回溯；ROI 失败即跟踪失败。
  仍保留 256 SYNC 逐符号跟踪（CFO/CIR 需要）。
- **live 入口**：`--mode continuous|timed`、`--sync acquire|locked`、`--period-ms`、
  `--locked-roi`、`--lock-observations/--holdover/--reacquire`、`--interference`、
  `--timed-lead-us/--timed-lookahead/--timed-alignment/--timed-t0-ms`。
  候选分类**推迟到提交时刻**（目标窗比自相关晚约 0.46 ms 才提交），失败 slot 的
  候选显式报 `INTERFERENCE_AMBIGUOUS` 而非丢弃。
- **QA**：全量串行 CTest **86 项 85 通过 1 跳过**（`uwb_qa_install_consumer` 按设计
  跳过），112.98 s。新增 `qa_uwb_target_lock_core.cc`（15 例）、
  `qa_uwb_timed_rx_plan.cc`（10 例），并扩展
  `tools/rx/qa_live_dw3000_console.py`（locked 策略、timed 计划、fail-closed）。
- **实采 A/B**（`rx_ch3_rx1_1s.sc16`，4 worker，3 次配对，197 目标时隙/次）：
  demod 均值 1838.6→1813.8 µs（−1.3%）、compute 3699.9→3671.9 µs（−0.8%）；
  **最坏 demod 7383→3555 µs（−51.8%）、最坏 compute 12944→7469 µs（−42.3%）**；
  FCS 199/200 vs 199/200（一次 198）。均值差异在噪声内——256 SYNC 跟踪 + CIR +
  soft-chip 占 ~1.8 ms，策略只约束首次 SYNC 搜索，**收益在尾部不在均值**。
- **契约计数**：locked 下 `bounded_legacy_scans=0`、`backtrack=0`（acquire 下 197
  时隙合计回溯 136–149 符号）、`roi=256`、残差均值 −0.02 样点（−1…+2）。
  一次 `bounded_fail` 对应残差 −301 的包：±256 ROI 的已知代价，约 0.5%。
- **周期**：学习 3,686,299.5 vs nominal 3,686,400 = **−27.25 ppm**，与实采
  4.999864 ms 一致；锁定耗时 3 包；1 s 内 0 重捕获 / 0 holdover / 0 gap。
- **干扰路径**：`cand_target=197`、`cand_unknown=0`、`autocorr_dedup=197`、
  `dedup_confirmed=194`、`dedup_ambiguous=2`，未切出第二个重复窗口。
- **timed_burst**（paced fake device，1 s 同夹具）：197 目标时隙、0 丢弃/过期、
  FCS 192/199（96.5%）、204 窗下发（199 完整 + 5 文件尾部分）、203 计划空档、
  0 非预期 gap / 迟到命令 / 溢出 / 超时 / 设备时间跳变；上传样点 75,000,240 =
  连续模式 737,280,000 的 **10.2%**（只证明主机可见窗口与负载下降，不是 ADC/RF 停止）。
- **未做**：UHD 定时命令路径（`STREAM_MODE_NUM_SAMPS_AND_DONE`）**未实现**，
  `--mode timed` 无 `--replay` 时 fail-closed 返回 2；timed 模式**关闭**单音消除
  （训练窗会落在包上），设备定时单音调度只有核心算术 + QA、未接入 live source；
  未接 SIC；`--locked-roi` 64/128 未验证；未上板/UHD 溢出/paced/8 worker/P99/
  MATLAB 交叉验证/多径/AWGN/ISR/CFO 扫频与 SFO 均未验证。
- **实采 vs 仿真夹具的差距已归因到根因（报告 §5.6）**：同一波形配置下 X410 实采
  单包解调 1871 µs，仿真夹具 1086 µs（1.72×），差距几乎全在 CIR/soft-chip
  （902 vs 115）与 SFD（489 vs 200）。**根因是采样时钟偏差，不是噪声/多径/CFO**：
  融合 CMF 用固定 `kSamplesPerChip=2` 整数网格，门限 `|spc-2|<=1e-9`；实采
  `spc=1.9999465`（**−26.7 ppm**，门限的 5.3 万倍）。127 B payload 共 154,624
  样点，−26.7 ppm 累积 4.1 样点 ≈ 2.1 chip 漂移；要 <1 样点需 ≤6.5 ppm。
  仿真夹具用 `resample_poly(x,48,65)` 从标称速率精确生成，网格恰为 2.0，所以通过。
  该 −26.7 ppm 与独立测得的包周期 −27.25 ppm 是同一时钟偏差的两面。
  第二个独立阻塞：`--cir auto` 映射为 `Bypass`，融合门限也显式拒绝 Bypass。
  四次对照：`fused=off/auto` 1800 µs（无回退）；`fused=on/full` 17204 µs
  （`st_cir` 仅 72 µs，但 spc 检查失败 → 回退 16.2 ms）；`fused=off/full`
  17054 µs（228,422 chip × 38 抽头，不可用）；`fused=on/bypass` 1870 µs
  （配置门限先拒）。**调松门限无用**：物理上限约 6.5 ppm，实采 26.7 ppm。
  另修掉 `rx_softchip_on_demand` 在 `fused=off` 时每包失败再回退的白跑，
  `--ondemand` 默认跟随 `--fused`，demod **−7.3%**（1938.6→1835.4 µs 三次同向，
  FCS 持平）。
- **受控实验（同波形同噪声种子，只改时钟）**：自造 0 ppm 与 −27 ppm 两份 40 包
  夹具。demod 1225.9→1580.4 µs（+29%），其中 CIR/soft-chip 399.3→688.5
  （+289.2，占 82%）；timing −11.7、CFO +19.5、SFD +55.1、NS-SFD/PHR/payload
  +2.6、resample −3.6；检出 SYNC 256/256、CFO 拟合峰 40/40、FCS 39/40 两边相同。
  **机制**：`spc≠2` → `fractional_grid=true` → 软比特 FIR 从 AVX2 整数网格内核切到
  标量分数插值（每 chip 两次 38 抽头点积 + 线性插值）。实采与 −27 ppm 夹具残差
  仅 +61.5 µs（+3.9%），来自真实噪声/多径/CFO。**除 CIR/soft-chip 外无实质差别。**
- **时钟修好即可用融合路径（实测）**：0 ppm 下 `--fused on` demod 1225.9→**980.5 µs
  （−20%）**、`st_cir` 399→95 µs、零回退、FCS 39/40；同一开关在 −27 ppm 下触发
  15.8 ms 整条流回退。故"trim 回 ±1 ppm"可让实采 1637→约 1000 µs。
- **X410 不能微调晶振（本机只读查询）**：clock sources `mboard internal external
  nsync gpsdo`；master clock rate range 单点 737.28 MHz（step 0）；
  `set_rx_rate(737.28e6)` 精确无 coercion。`get_rx_rate()` 是配置值不是测量值，
  "0 ppm error" 不证明晶振准确；要准只能用 external 10 MHz / gpsdo / nsync。
  本次偏差在 DW3000 发送端，trim X410 是修错的一头。
- **重采样与单音消除：无"实机 vs 仿真"算法差异**（报告 §5.7）。
  重采样同一夹具 0 ppm vs −27 ppm 差 0.2%（4.661 vs 4.652 ns/样点）；实采
  4.928 相对夹具 `tone off` 的 +5.7% 可用同一夹具打开 decorator 复现（4.877），
  即内存竞争而非数据。单音消除：孤立内核基准 38.1 ms/s，live 夹具 82.1、
  实采 90.9（2.2–2.4×），`bypassed_samples=0` 证明两边都在真减；差距来自内存带宽
  （环形拷贝 ≈5.9 + 重采样 ≈7.9 + 单音 ≈5.9 GB/s ≈ 19.6 GB/s）。实采比夹具再高
  10.7%，因实采 demod 更重、worker 占更多带宽。`clean_comm` 仿真基准**不含**单音
  消除，故两者之间没有可比的单音数字。
- **整数 chip 网格门限已改为物理推导的漂移预算（报告 §5.8）**：原
  `|spc-2| <= 1e-9`（0.0005 ppm）比 256 峰周期拟合的估计器噪声（实测 1.5–2.2 ppm
  逐包 stdev）低三个数量级，**真实数据永远无法满足**。硬上限
  `kIntegerGridCeilingPpm = 1/154624 = 6.468 ppm`（1 样点漂移），默认准入取该上限
  （`Qm35825Profile::fused_gate_ppm`，CLI `--fused-gate-ppm`）。新增
  `integer_chip_grid()` / `integer_chip_grid_at()`（各带显式容差重载）替换
  `uwb_demod_core.h` 全部 5 处 `1e-9`。QA 新增
  `integer_chip_grid_tolerance_matches_the_drift_budget`（0.66/1.24/1.4 ppm 接受、
  1.6/27 ppm 拒绝、显式容差重载生效、边界=漂移预算、退化输入 fail-closed）。
- **门限不能收紧到 1.5 ppm（实测反例）**：紧门限拒掉的不是坏时钟而是估计噪声，
  且被拒的包会 `fractional_grid=true` → 软比特内核从 AVX2 整数网格切到标量分数插值
  → 228,416 chip × 38 抽头 = **15.3 ms/包**。trim6b 实测：1.5 ppm 走融合 84%、
  demod **3549/3693 µs**；6.468 ppm 走融合 100%、demod **1103/1128 µs**（快 3.2 倍）。
  要收紧门限必须先有比周期拟合更准的时钟估计（CFO/载频实测 ±0.14 ppm）。
- **顺带修的耦合缺陷**：原实现里 on-demand 前缀由 `--ondemand`（跟随 `--fused`）决定、
  融合准入由门限决定，两者独立 → 门限拒包时前缀仍被截短到 NS-SFD，PHR 拿不到软比特
  → FCS 失败 → 整条流重试 **16.2 ms/包**（1.5 ppm 下 16% 的包）。改为 Stage 1 之后
  每包只判定一次 `integer_grid_ok`（用 `timing.measured_period`），同时控制第二次 CFO
  旋转范围、on-demand 前缀、融合 PHR/payload；被拒包走完整前缀（2.6 ms）。
- **默认配置已改为 `--fused on --cir full`（报告 §5.10）**：融合要求
  `cir != Bypass`，两者必须同时改。trim6b，4 worker，3 次交替配对（`--sync locked`）：
  旧默认（`fused off`+`cir auto`）demod **1560.0 µs** / `st_cir` 497.2 / p95 2385；
  新默认 demod **1218.1 µs（−21.9%，省 342 µs）** / `st_cir` **117.2（−76%）** /
  p95 **1721.5（−27.8%）** / FCS 同为 203/204。
- **默认必须带降级保护（否则 27 ppm 晶振会慢 10 倍）**：门限拒包时 `--cir full`
  仍对整帧跑 38 抽头 → 27 ppm 采集 **17,716 µs/包**（旧默认 1800）。已改为
  `demodulate_one` 每包判定 `integer_grid_ok` 时把 `Full` 降级为 `Bypass`
  （Full 软比特只为喂融合而存在），新增诊断字段 `grid_reject`。
  27 ppm 实测 **1974 µs**（p99 3208、FCS 199/200、`grid_reject=197`、`fused=0`），
  比不降级改善 9 倍且仍能解码。
- **新默认实机复核（X410 连续流 10 s，4 worker，loadavg 1.9）**：1998 目标时隙、
  1997 解码（99.9%）、demod mean **1177.8** / p50 1092.3 / p95 1647.5 /
  p99 **1960.5** / max 4436.8、`st_cir` 115.4、**0 丢样点**、`grid_reject=1`、
  `fused=1996`、锁 1 次进入 / 0 holdover / 0 重捕获 / 0 不连续、周期 −0.606 ppm、
  FCS 2018/2021。此前实机 demod mean 在 1357–1472、p99 在 3194–4492。
  **机器有外部并行编译负载（loadavg 5–7）时会掉 6–20% 样点、demod 抬到 2.7 ms，
  那是环境不是算法**——实机数字必须在 loadavg < 2 下测。
- **target lock 已在 X410 实机验证（报告 §5.9）**。1 s 实采回放 A/B（trim6b，4 worker，
  3 次配对，每次 591 包，`--fused on --cir full`）：acquire demod mean 1169.6 /
  p99 **4040.0** / max 5205 / `st_timing` max 3608 / backtrack 423；locked mean 1216.9 /
  p99 **2286.1** / max 4126 / timing max 1691 / backtrack **0** →
  **p99 −43.4%、最坏 −20.7%、timing 最坏 −53.1%、均值 +4.0%（噪声内）**。
  契约逐条成立 591/591：`bounded_ok=591`、`bounded_fail=0`、`legacy_scans=0`、
  `backtrack=0`、`roi=256`、`sync_strategy=locked_bounded`。
  X410 连续流 10 s ×2：1993/1882 时隙、99.3%/99.7% 解码、**0 丢样点**、
  `st_fused` 100%、锁 1/5 次进入、0 holdover、0 重捕获、周期 −0.63/−0.64 ppm。
  **实机 A/B 不能比均值**：所有阶段（含策略不碰的 sfd/cir/payload）整体差 ~10%，
  属机器状态差异；受控比较只能用同一份回放夹具。早期两次
  `acquire + fused + cir full` 实机出现 16% 丢样点/1290 次不连续，**不可复现**
  （第三次 0 丢样点、2034 包），不能作为"acquire 撑不住"的证据。
- **DW3000 晶振 trim 全过程（同一 1 s 流程，`tools/rx/capture_x410_sc16.py` +
  `check_clock_offset.py`）**：CFO 176.7→43.5→19.5→8.08→12.0(过冲)→**4.25 kHz**；
  偏差 27.1→6.65→2.94→1.24→1.85→**0.66 ppm**；payload 漂移 4.20→**0.10 样点**；
  FCS 99.0→90.9→95.1→99.5→95.4→**99.5%**；"恰好 2.0"的包 0%→**80%**。
  trim6 上两个独立估计完全一致：`4.25 kHz / 6489.6 MHz = 0.66 ppm` 与
  `spc_ppm = -0.66 ppm`。**原始 177 kHz 载波 CFO 全部来自晶振**（27 ppm × 6.4896 GHz）。
  中间 trim2/trim5 的 FCS 回落到 90.9%/95.4% 说明 payload 失败确实由时钟偏差引起。
- **门限放宽 + 融合路径实测收益（trim6b，4 worker，3 次配对）**：默认
  (`--fused off --cir auto`) demod 1555.9 µs / `st_cir` 494.4；`--fused on
  --cir full` demod **1223.7 µs（−21.3%）** / `st_cir` **118.7（−76%）** /
  零回退 / FCS 同为 203/204。耦合：融合要求 `cir != Bypass`，故必须 `--cir full`；
  而 `--cir full` 无融合时是 2623 µs（不可用），**门限放宽是让它可用的前提**。
- **顺带**：新增 `--tone on|off`（关闭时不再套 decorator，避免无意义的有界环丢样）；
  新增 `--tone-period-ms` / `--tone-training-ms`（诊断扫描用，实测对 FCS 无影响）；
  PACKET 行新增 `spc_ppm`、SFD 子阶段（`sfd_boot/sfd_tail/sfd_cforef/sfd_final/
  sfd_other`）、`chips/taps`，用于直接判定融合路径被拒的原因；COST_TOTAL 新增
  `bypassed_samples/model_updates/model_rejects/training_windows`。

---

# 历史提交快照：clean comm 256 SYNC/127 B/CFO 多 worker 耗时与分解（2026-11-15）

已交付干净通信接收（无干扰、无 SIC、无 sensing）在 256 SYNC、127 B PSDU、
+50 kHz CFO 下 1/2/4 worker 的耗时与阶段分解。报告
`docs/performance/clean_comm_256_127_CFO_多worker耗时与分解_2026-11-15.md`，
数据 `docs/performance/data_clean_comm_256_127_cfo_20261115/`，驱动
`tools/sic/run_clean_comm_workers.py`，基准新增 `clean_comm` 任务
（`tools/sic/benchmark_chain_workers.cc`）。基线 `acffbba`。
- **结果**：完成间隔 2766.6 / 1446.7 / 818.4 µs（1.91× / 3.38×），等效原生吞吐
  119.8 / 229.0 / 404.9 MS/s；单包均值 2764.3 / 2889.6 / 3255.2 µs（+4.5% /
  +17.8%）。该包在原生流中占 449.4 µs，单 worker 慢 6.16×、4 worker 仍慢 1.82×。
- **分解（N1 均值）**：重采样 1677.2 µs（60.7%，其中 `process_sc16` 占 99.9%）
  + comm 1086.4 µs（39.3%）。comm 内 timing 427.4（39.3%）、payload 290.9
  （26.8%）、SFD 200.4（18.4%）、CIR 115.2（10.6%）、PHR 19.0、CFO 15.9、
  NS-SFD 10.0、other 4.2。SFD 里 167.2 µs（83%）是锚定后第二次 CFO 拟合+去旋转；
  CIR 里估计 60.1 / soft-chip FIR 26.9 / 后处理 27.1。去旋转跨阶段合计 267.4 µs，
  是诊断桶，不可与父阶段相加。
- **CFO 对照**：+50 kHz 与 0 Hz 总时间差在 ±1% 内且方向不一致（首轮串行跑 CFO 块
  得到 2.9% 假差异，改为跨轮交替顺序后消失）。全部 18 次运行 × 512 任务 =
  **9216 个任务 FCS 通过、PSDU 逐字节一致**，0 回退；CFO 估计精确等于注入值；
  SFD 起点在所有 worker 数下一致。
- **计时口径**：本路径**不含** `FixedToneCancelSc16Source`，夹具也无单音；
  `resample_us` 只包住 `reset+process_sc16+flush`。生产 pipeline 里单音消除是
  窗口装配前的源装饰器，有独立 `cancel_us`，不在 `resample_us`/`e2e_us` 内。
- **未验证**：上板/UHD、RX/队列/提交/paced、`quality_eq32`/`realtime_*`、
  多径/AWGN/ISR、CFO 扫频与 SFO、8 worker、P99、MATLAB 交叉验证。

---

# 历史提交快照：固定频率单音消除 C++ 模块集成（2026-11-15）

已把"已知固定接收基带单音的幅度/相位估计 + 20 ms 外推 + 连续相减"接入原生
`UhdSc16Pipeline`，作为**截窗之前**的公共前处理。报告
`docs/performance/固定频率单音消除_C++模块集成开发报告_2026-11-15.md`，数据
`docs/performance/data_fixed_tone_cancel_20261115/`，任务书
`docs/performance/OpenCode_固定频率单音消除模块开发任务书_2026-11-15.md`。
独立 build `gr-uwb/build-fixed-tone-cancel`；未 install、未推送、未上板。
- **落点**：`UHD recv → [装饰器] 有界原始 SC16 块环(默认32×65536=8 MiB) →
  装饰器 read_samples(训练/估计/外推 + 融合 copy_cancel) → 现有窗口拼装 → 原 DSP`。
  `start_stream()` 仍只在 worker Ready 屏障后；`open()` 只分配。
- **数学**：`c=(1/Nt)Σx·e^{-jωn}`，int32 折叠 + double 投影（对任意 Nt 与全窗投影
  等价），288 点周期由 `f0·P/fs` 整数**推导并校验**（默认 −91/288），无短周期
  显式 unsupported。LUT 量化固定 nearest-even（与 fenv 无关），分量超 int16
  拒绝且不改旧表。热路径 = load + `_mm256_subs_epi16` + store；AVX2 运行时分派
  （target attribute，未给全库开 AVX2），scalar/AVX2 逐位相等。
- **语义**：每周期起点暂存并回放训练窗（不丢样、不等整 20 ms），256 样点交叉渐变；
  gap/时间戳跳变在正确输出边界给 `is_gap` 并按新 epoch 重新训练（不跨不连续学习）；
  EOS 训练不足 `partial_training_bypass` 原样输出；队列满非阻塞整块丢弃 + gap。
  两层 read 计数分开，新增 `source_samples == queued+dropped`、
  `queued == consumed+pending` 守恒并纳入 runner 的 runtime_ok。
- **验证**：全量串行 CTest **83 项 82 通过 1 跳过**（`uwb_qa_install_consumer`
  按设计跳过），0 失败；新增 QA：核心 12 例 + 装饰器/pipeline 8 例。
  40 ms 实采（固定 −232.96 MHz、20 ms/0.5 ms）与**独立 numpy 固定频率参考
  29,491,200 复样点逐字节一致（0 差异）**；1 ms 相干抑制 43.0–59.7 dB；
  自相关中位数 0.640→0.0041；code10 模板 8/8 保留（5 组峰值时间不变，
  3 组移动 −198/−83/−28 µs，属曲面峰值位置，不是 ToA）。
- **性能**：核心 65536 样点融合 3.38–3.46 µs、两遍 5.16–5.19 µs
  （ABBA −32.6…−34.7%）；估算 64.7–66.1 µs、建表 6.7 µs。
  单通道 737.28 MS/s **paced 回放 1 s：0 丢块、0 gap、50 次模型更新**（默认 8 MiB
  队列，不绑核，3 次 stage-only + 4 次 pipeline 全 0 丢；整进程绑单核不稳定，
  同一命令 0 或 ~0.8% 丢）；首样点延迟 0.64–0.83 ms；完整 pipeline 窗口服务时间
  31.8→32.1/33.0 µs（+0.9%/+3.9%）；另一时段 34.0→35.6/40.7 µs（+3.0%/+4.9%，
  关闭基线本身也漂到 34.0–34.6 µs）；前处理 `cancel_cpu` 66–69 ms/s（6.6–6.9% 单核）。
- **顺带修的缺陷**：`ReplaySc16Source` 的 paced 时间基准原本取自 `open()`，
  而 pipeline 在 Ready 屏障之前 open → `start_stream()` 后全速突发补数据，
  会打满 8 MiB 环（实测 0.08–0.5% 丢块、首样点 3.48 ms）。改为
  `start_stream()` 重置基准后同一用例 0 丢块、首样点 0.69 ms（unpaced 路径逐位不变）。
- **QA 反复运行暴露并修掉一个竞态**：训练窗"还没收到更多数据"曾被误判成 gap
  （`contiguous_end` 与 `published_end` 取自两次不同 head/tail 观测）→ 模型作废、
  epoch 前移、该周期旁路。改为同一快照的 `RunSpan`，新增 QA 连跑 12 次与全量复测全绿。
- **未验证**：上板/UHD 真实溢出、MATLAB、PSDU/FCS/PER、CIR/弱目标保护
  （40 ms 固定 5 ms 窗网格不对齐包 → SIC committed 0/8、SFD 0/8，两边相同，
  不是回归证据）、8 Worker、通用 NCO 后端、训练窗碰包鲁棒性。

---

# 历史提交快照：Ipatov 前导重构 `template_scale_mismatch` 修复（2026-11-12）

已交付单 worker 测试配置修正后的重构错误定位与修复。报告见
`docs/performance/单worker配置修正_Ipatov前导重构错误定位与修复报告_2026-11-12.md`，
原始数据 `docs/performance/data_sic_ipatov_prefix_fix_20261112/`，任务书
`docs/performance/OpenCode_单worker测试配置修正_2026-11-12.md`。code-tested SHA
`80ac19e`（文档 `314d48c`）。
- **根因是三段链**：`modulate_prefix_pulse_list()` 用 `packet_samples_998p4(...,0,...)`
  把 PHR 的 21504 样点算进「前缀」（报 94656 vs 实际 73152）→ 窗口容量 77429
  小于它 → 事件缓存被清空、稀疏分支跳过 → 非稀疏分支把**成形**前缀喂给
  **单位冲激**参考，尺度检查报 `template_scale_mismatch`。安全检查本身没错，
  拦下的是错误输入。
- **第二个独立缺陷**：Ipatov 非稀疏 L1 路径原本**永远不可能提交**，只是
  `sparse_pulse_reconstruct` 默认 true 掩盖了它。现在冷路径一次性展开单位冲激
  前缀（585 KB/scratch），提交原因 `committed_preamble_ipatov`，缓存缺失时
  fail-closed 为 `impulse_prefix_unavailable`。
- **两个入口此前都没有 QA**，这是缺陷进主干的原因；已补
  `prefix_pulse_list_span_is_sync_plus_sfd` 与
  `test_unified_ipatov_preamble_uses_unit_impulse_prefix`。
- **验证**：全量 CTest **80 项 79 通过 1 跳过**（`uwb_qa_install_consumer` 按设计
  跳过，122.85 s）。任务书单配置检查 **30/30 提交、44.11 dB**（修复前 0/30、
  0.00 dB）；discrete 对照 30/30、32.54 dB。cost-split 12 次全部提交
  （41.02–44.11 dB）；g3 4/4 窗功能字段一致，EVM 0.730%/0.425%，lag 0。
- **`sensing SFD 30/30 → 0/30` 不是检测退化**：该 fixture 是同一个 code-9 包的
  重复，`both` 模式下包被消掉；`--consumer sensing`（不跑 SIC）仍 3/3 检出。
- **耗时口径**：修复前 307 µs 是失败退出耗时，不能当基线；修复后 413 µs 才是
  真实消除负载。`queue_us`（16.7–26.3 ms）是全速回放排队，不是 paced 延迟。
- **未验证**：混合码（sensing 9 / comm 10）重叠、SIC 后 sensing CIR/弱目标保护、
  PSDU/FCS/PER、上板、MATLAB 交叉验证、paced 到达、8 Worker。L2 离散事务仍硬编码
  `code_index==10` + Legacy 脉冲，code 9 整包路径仍会被 `unsupported_profile` 拒绝。

---

# 历史提交快照：65/48 等波纹 FIR 设计实验（2026-11-11）

已交付等波纹（Remez）重设计实验：设计脚本、独立频响验证、独立 float64 参考、
候选系数、ABBA 性能矩阵与应用层波形对照。报告见
`docs/performance/65_48等波纹FIR实验报告_2026-11-11.md`，任务书
`docs/performance/OpenCode_65_48等波纹FIR设计实验任务书_2026-11-11.md`。
- **核心结论**：内核成本正比于 `Hpad = 8*ceil(ceil(T/65)/8)`，不是 `T`。跨档位才有
  收益：realtime `Hpad` 24→16（`T=1039`）、quality 48→32（`T=2079`），
  **每输出样点乘加 −33.3%**。
- **频响通过**：4 个候选独立复验（全阻带扫到 `Fv/2`），纹波 0.0868/0.0849/
  0.0095/0.0097 dB，全阻带 −74.6/−76.0/−108.1/−73.9 dB，`dc` 误差 ≤5.2e-7。
  基线 Kaiser 全阻带复验通过，但余量仅 0.070/0.029 dB。
- **数值通过**：192 例独立 float64 参考对照（6 系数 × 4 输入 × 8 长度），
  0 失败、0 长度不符，最差相对 L2 **6.28e-8**（出现在 Kaiser 基线）。
- **速度通过**：3 个 session × 4 格 ABBA 配对，全部超过 5% 门槛。
  quality 稳定 **−24%**（−23.9…−25.0%）；realtime 不稳（−12.5…−18.9%），
  只宣称 **≥−12%**。绝对：56k realtime 154.8→125.5 µs，quality 259.2→195.9 µs。
- **波形质量通过**：按理论群时延差对齐后残差 lag **0 样点**，幅度
  **+0.047/+0.042 dB**，EVM 0.73%/0.43%（均在 0.1 样点 / 0.1 dB 门槛内）。
  不对齐时 153% 的"误差"几乎全是 2.92/6.54 样点时延。
- **新档位模板展开已否决**：能让等波纹再快 1.8–7.3%，但同一 build 内让现有
  Hpad 24/48 内核慢 3.4–4.7%（改结构后 quality_56k 达 +12.9%），已回退；
  回退后二进制与改动前逐字节相同，四个 tap 集 `--dump-iq` 与改动前 `memcmp` 全等。
- **未验证**：SIC 抑制、PSDU/FCS、PER。仓库 SIC 场景是 998.4M work-grid fixture，
  使用即绕过被测重采样器（任务书 §5 禁止）；唯一 native fixture 不触发检测。
  PER（3 SNR × 1000 包）已论证**功效不足**：分辨率 ±1.35 pp 而期望效应约
  0.0007 dB SNR 等效，差约 3 个数量级，改用 EVM/相对 L2 直接测量。
- **群时延变化未集成**：realtime 13.73→10.81、quality 28.19→21.65 输出样点，
  解调器时延补偿、capture/halo 几何与时间原点均未改。
- 未上板、未做 MATLAB 交叉验证、未改默认 profile（任务书要求本轮不自动替换）。

---

# 历史提交快照：UHD SC16 重采样与流水线第二轮优化交付（2026-11-11）

已交付 Phase 0 测量合同、A1 惰性调度表、A2 就绪协调、B1 RX 预分配缓冲、
C1 固定相位调度（默认）与 C2 转换复用（未保留）。开发报告与全部数字见
`docs/performance/UHD_SC16重采样与流水线第二轮优化报告_2026-11-11.md`，
接口/测量合同见 `docs/performance/接口冻结_UHD_SC16第二轮优化_2026-11-11.md`。
- **串行全量 CTest：79 项，78 通过，1 跳过**（`uwb_qa_install_consumer` 按设计
  跳过，124.06 s），0 失败。
- **最终矩阵**（control = HEAD+插桩+A2 vs final = A1+B1+C1；12 配置 × 5 组 ABBA
  × 10000 稳态测量窗，零丢弃）：56,000 点窗口稳态 **−8.3% / −7.8% / −8.4%**
  （realtime N=1/2/4）、**−3.3% / −6.2% / −6.4%**（quality N=1/2/4）；
  4,000 点 realtime N=1 **−8.0%**，其余 4k 格在噪声内。绝对吞吐：
  56k N=1 约 233 MS/s，56k N=4 约 710 MS/s，4k N=4 约 739 MS/s。
- **启动开销**：`startup_ready` 中位改善 −19.9%…−84.0%（4k 路径 −77%…−84%）。
- **正确性**：C1/C2 相对默认 SC16 内核逐样点位级一致（3252 次比较、
  16,180,905 输出样点，`memcmp` 全等）；流水线 control vs final 11 个功能字段
  相等；**重采样 IQ 直接字节比对 14.5 MB、0 字节不同**（A1 旧 RX vs B1 新 RX，
  N=1 与 N=4；`none` vs 默认 `fixed_phase` 4.86 MB 全等）。
- **C2 未保留**：转换复用在 realtime 变慢、quality 变快（−7.6%…+7.6%），仅保留
  显式 `--resampler-experiment convert_cache`。
- **B1 未通过 5% 性能门槛**，按内存有界与消除冗余工作保留：每窗 224 KB
  "先清零再覆盖" → 整个 run 一次 448 KB（−99.9%），逐窗 `resize` → 1 次；
  端到端吞吐改善在噪声内，报告如实记录。
- **未验证**：未上板；UHD `start_stream()` 时序、`recv` 溢出、paced 到达、8 Worker、
  AVX-512、MATLAB 交叉验证均未做；提交路径仍在 `commit_mutex_` 内，不宣称
  RX 已无阻塞。
- 插桩扰动已量化：`--diag-rx --diag-commit` 全开 +1.8%（240.8 vs 236.5 µs/窗）。

---

# 历史提交快照：UHD SC16 Burst 级多核并行流水线交付（2026-11-11）

已交付 Burst 级多核分发（Per-Burst Run-to-Completion），取代此前被实测证明会
退化的「单 Burst 内部切分」多核。开发报告详见
`docs/sic/UHD_SC16_Burst级多核并行开发报告_2026-11-11.md`，契约见
`docs/performance/开发方案_UHD_SC16_Burst级多核并行_2026-11-11.md`。
- **全量回归通过**：串行 CTest **78/78** 全部通过（`uwb_qa_install_consumer`
  按设计跳过，116.13 s）。新增 4 项多核 QA 全部通过。
- **逐窗位级无损**：`dsp_workers ∈ {1,2,4}` 在 737.28M→998.4M 路径上，11 个
  `WindowResult` 字段逐窗 **位级相等**（0.00e+00，无容差）。
- **单包延迟不升**：56,000 点 `realtime_minorder` 的 Resample P50 为
  185.65 / 183.37 / 184.39 µs（N=1/2/4），4,000 点为 13.39 / 15.34 / 13.28 µs。
- **持续吞吐显著提升**（56,000 点 × 100 窗，`realtime_minorder`）：
  275.19 → 154.13 → 129.05 µs/窗（**1.79× / 2.13×**）；
  `quality_minorder` 367.59 → 204.49 → 136.73 µs/窗（**1.80× / 2.69×**）。
  4,000 点 × 1000 窗：19.83 → 11.77 → 8.66 µs/窗（**1.68× / 2.29×**）。
- **旧方案退化被完整重现**：56,000 点 `--resampler-workers 2` 总墙钟 41 ms、
  `4` 为 30 ms，均差于单核 28 ms；`--dsp-workers 2` 为 13 ms（**2.15×**）。
- **状态账本守恒律 100% 成立**，含 2 槽小池 4 Worker 的
  `offered=200 / admitted=8 / dropped=192` 满载丢弃场景。
- **新瓶颈已定位**：N=4 后不再线性扩展，瓶颈转为**串行 RX 生产者**
  （56k 窗 RX+提交下限约 45–50 µs/窗，100 窗约 13 ms）；N=8 回落，不建议 N>4。
- **未验证**：未上板；UHD 实机多核、`recv` 溢出交互、paced 到达均未验证。
  本文数字为文件回放的吞吐上限，非实时调度下的实际出包间隔。

---

# 历史提交快照：UHD SC16 65/48 重采样（737.28M→998.4M）向量化融合优化交付（2026-11-10）

已交付基于 AVX2 融合展开与直写流水线的高性能 65/48 重采样优化，开发报告详见 `docs/sic/UHD_SC16_65_48重采样优化性能评估报告_2026-11-10.md`。
- **全量独立测试通过**：在独立编译目录 `gr-uwb/build-sc16-pipeline` 下构建，CTest 包含 7 组 Golden 向量回归、长度公式契约、分块流式一致性、逐样点精度对比等测试 100% 全部通过。
- **逐样点位级无损精度**：融合 `process_sc16` 与标准分步路径（`sc16_to_cf32` + `process`）的最大绝对误差为 **0.00e+00**（位级精确无损）。
- **转换开销彻底归零**：`process_sc16` 直写使得独立 `convert_us` 耗时压降至 **0.00 µs**，完全消除了中间缓冲与二次内存拷贝。
- **重采样提速 1.8× – 2.5×**：
  - **4,000 样点窗口**：Resample P50 降至 **16.55 µs**（`realtime_minorder`，**2.53×**）与 **22.47 µs**（`quality_minorder`，**1.86×**）；端到端延迟 P50 仅 **16.65–22.57 µs**。
  - **56,000 样点整包窗口**：Resample P50 从 592.93 µs 降至 **234.47 µs**（`realtime_minorder`，**2.53×**，净压降 **358.5 µs**）与 **324.12 µs**（`quality_minorder`，**1.83×**，净压降 **268.8 µs**）；端到端延迟 P50 降至 **266.35–358.87 µs**。
- **状态账本守恒律 100% 成立**：`offered == admitted + dropped + invalid_gap + cancelled` 在全量测试中 100% 满足，满载零非预期丢弃。

---

# 历史提交快照：UHD SC16 原生双线程流水线交付（2026-11-10）

已交付绕过 GNU Radio scheduler 和 PMT 消息传递的高性能原生数据流处理流水线，开发报告详见 `docs/sic/UHD_SC16原生接收流水线开发报告_2026-11-10.md`。
- **全量独立测试通过**：在独立编译目录 `gr-uwb/build-sc16-pipeline` 下构建，CTest `uwb_qa_uwb_uhd_sc16_pipeline.cc` 包含 5 项针对性测试全部 100% 通过（耗时 0.04 s）。
- **极低转换与调度时延**：
  - 4000 样点窗口的 SC16 $\to$ CF32 转换实测均值 **1.51 µs**（P50 **1.44 µs**，P95 **1.63 µs**），标量与 AVX2 逐点精确一致（$0.00\times 10^0$ 误差）。
  - 无锁 SPSC 队列排队延迟中位数（P50）仅 **0.86 µs**。
  - 端到端调度耗时中位数（P50）仅 **2.49–2.64 µs**（P95 **3.73–8.55 µs**），相比 GNU Radio 框架损耗实现两个数量级压降。
- **状态账本与零背压保障**：严格遵守状态账本守恒律（`offered == admitted + dropped + invalid_gap + cancelled` 100% 成立）；满载时非阻塞丢弃，绝不上溯背压至硬件驱动缓冲区，避免 USRP 溢出（`O`）。

---

# 历史提交快照：SIC 第三阶段核心瓶颈优化（2026-11-10）

已交付 SIC 第三阶段核心瓶颈优化（SFD 候选窗 AVX2 预折叠点积、Tail 40-SYNC 重跟踪峰值条件等价复用与回退安全、SYNC 相位/增益保序向量化累加、NS-SFD 稀疏符号匹配与 $O(1)$ 滑动能量、Ipatov 训练样点去旋转共享旋转表向量化与零 CFO 旁路、`norm_at` 坐标缓存），开发报告详见 `docs/sic/SIC_第三阶段核心瓶颈优化开发报告_2026-11-10.md`。
- **Preamble-Only 稳固突破 500 µs（0.5 ms）关键实时门槛**：核心耗时均值降至 **436.34 µs**（P50 **428.54 µs**，分批 P95 **500.03 µs**，合并 100 轮 P95 **493.40 µs**），抑制比保持 **63.29 dB**（0.00 dB 差异）。
- **Full-Packet SIC 均值降至 790 µs 级别**：核心耗时均值从 1,009.63 µs 降至 **797.64 µs**（加速比 1.27×，P50 **769.46 µs**，分批 P95 **928.72 µs**，合并 100 轮 P95 **936.87 µs**），抑制比保持 **63.33 dB**（0.00 dB 差异）。
- **SFD 与 Tail 重跟踪提速 5.5 倍**：SFD 阶段耗时从 194.10 µs 降至 **34.91 µs**（5.56× 加速）。
- **信道估计提速 2.5 倍**：`estimate_channel` 耗时从 124.84 µs 降至 **49.40 µs**（2.53× 加速）。
- **验证与数值安全收尾**：新增 5 项针对性单元测试（含开启 `ipatov_correlation=true` 的标量对照、Tail 40-SYNC 复用与回退专门用例），12/12 项 CTest 全部 100% 通过（耗时 13.91 s）；基准逐样点有限性 100% PASS，新旧输出样点相对 $L_2$ 差异 **0.00e+00**（位级精确无损）。

---

# 历史提交快照：SIC 第二阶段全链路算力优化（2026-11-10）

已交付 SIC 第二阶段全链路算力优化（信道估计实数展开/局部能量/CFO 双精度向量化、SFD 细搜 4-lag 批处理与相干累加 SIMD、前导持有型持久缓存与 64 位哈希、ROI 抑制能量 SIMD），开发报告详见 `docs/sic/SIC_第二阶段全链路算力优化开发报告_2026-11-10.md`。
- **Preamble-Only 耗时大幅压降至 630 µs 级别**：核心耗时均值从 797.98 µs 降至 **634.84 µs**（加速比 1.26×，净减 ~163 µs，P50 622.65 µs，P95 744.55 µs），抑制比保持 **63.29 dB**（0.00 dB 差异）。
- **Full-Packet SIC P50 进入亚毫秒**：核心耗时均值从 1,162.30 µs 降至 **1,025.23 µs**（P50 **986.18 µs** 进入亚毫秒，P95 从 1,434.22 µs 降至 **1,151.56 µs**，净降 ~283 µs），抑制比保持 **63.33 dB**（0.00 dB 差异）。
- **信道估计提速 2 倍**：`estimate_channel` 从 213.30 µs 降至 **106.03 µs**（2.01×），P95 从 258.62 µs 降至 132.84 µs。
- **验证与数值安全**：新增 11 项针对性单元测试，12/12 项 CTest 全部 100% 通过（耗时 13.46 s）。

---

# 历史提交快照：前导码与 SFD 互相关向量化优化（2026-11-10）

已交付前导码与 SFD 专用向量化优化（AVX2 4 滞后实数展开点积与 SFD 符号级 Kronecker 分解），绑定提交 `4697980`，测试报告详见 `docs/sic/前导码与SFD互相关优化开发报告_2026-11-10.md`。
- **Preamble-Only SIC 突破 1 ms**：核心耗时均值从 1.15–1.25 ms 降低至 **828.12 µs**（P50 812.35 µs，P95 999.00 µs），抑制比保持 **63.29 dB**。
- **Full-Packet SIC 进一步优化**：核心耗时均值从 1.58 ms 降至 **1,142.42 µs**（P50 1,128.12 µs，P95 1,276.16 µs），抑制比保持 **63.33 dB**。
- **通信前导解调提速**：`stage_timing` 从 ~380 µs 降至 97.7 µs（3.9×），`stage_sfd` 从 ~340 µs 降至 212 µs（1.6×）。
- **验证与数值安全**：实数展开 AVX2/FMA 点积与 VOLK 误差 $< 4.8 \times 10^{-7}$，9/9 项 CTest 全部通过。

---

# SIC 全包与前导优化统一集成基线（2026-11-09）

本分支为 `integration/sic-unified-20261109`，由 `feature/sic-fullpacket-compare`（全包 SIC 与离散 CIR）与 `feature/sic-perf`（Preamble-only 核心优化与 PMT/dcons 优化）合并而成。

- **全包与离散 CIR 基线**：包含 Full-packet SIC、离散等效 CIR 估计（`discrete_cir`）、快速通信接收机（Fast SFD/CFO/Softchip、Fused CMF/PHR）以及 AVX2 脉冲重构内核。
- **前导核心与系统开销优化**：包含 Preamble SIC 核心的列/旋转/self-energy 预计算缓存加速，以及 GNU Radio PDU 块的 PMT 静态符号表缓存和 O(1) dcons 前插优化。
- **统一模型愿景**：Preamble-only 被视为已知字段覆盖仅为 SYNC 的全包 SIC 特例。

---

# 当前提交快照：整包 SIC 稀疏重构与验证精简（2026-11-08）

本次提交汇总当前实验代码与报告。紧凑四径、32 B、20 tap、lag_lo=-1：总 SIC mean 1.822 ms，调制+重构 227.7 µs。相较最初整包版本历史 21.83 ms 约快 12 倍；不同信道/运行的阶段对比，详见 `docs/sic/SIC_首版至当前性能对比_2026-11-08.md`。下方“未提交”为各轮测量时的历史状态。PHR R1–R3 与 SYNC 碰撞退化仍未关闭；实验选项不自动替代默认路径。

---

# 最新变更：单位实数脉冲重构（2026-11-07）

Ipatov SIC 新路径已移除峰值扫描、归一化及整包实数转复数，并直接用实数脉冲累加复数 CIR。三项 QA 通过，七格与损坏包合计 128 个新旧实际输出逐位一致。32 B 分数多径：调制 246.8→61.3 µs、信道/CFO 891.7→747.2 µs、总 SIC 3.116→2.782 ms。报告：`docs/sic/SIC_单位实数脉冲重构优化报告_2026-11-07.md`。仍为 128 点相关 CIR；SYNC 碰撞退化未改变，3 参数约束尚未集成，前次 PHR R1–R3 未处理。修改未提交。

---

# 最新实验：SIC Ipatov 相关 CIR（2026-11-07）

新增显式 `--reconstruction ipatov_cir`：用 Ipatov 冲激序列相关估计包含 shaping 的响应，整包使用单位脉冲重构，省去投影/LS。七格×16 窗解码一致，损坏包逐位旁路；分数多径参数估计 1.482→0.277 ms、总 SIC 4.415→3.148 ms。但 SYNC 碰撞抑制 35.63→19.63 dB，单径总耗时略增，**尚不替换默认后端、不宣布基线替换验收通过**。报告：`docs/sic/SIC_Ipatov相关CIR开发报告_2026-11-07.md`。保留旧路径与历史数据；本轮仍未提交，前次 PHR R1–R3 未处理。

---

# 最新变更：SYNC 逐 chip FIR 已移除（2026-11-07）

用户要求已落实到 PHR 融合快路径：仅保留 SFD 搜索 FIR，实际输出 36,585→4,081 chip；SYNC 相位改为先累加对应 IQ 再计算等价统计。四项 QA、六格功能及损坏包对照通过。详见 `docs/sic/SYNC逐chip_FIR移除报告_2026-11-07.md`。本轮未提交；前次 PHR 复核 R1–R3 仍待关闭，不把下方历史“已交付”解读为这些问题已修复。

---

# 当前任务：PHR 解扩与 CMF 融合已交付

结果见 [报告](docs/sic/PHR解扩CMF融合开发报告_2026-11-07.md)，code-tested SHA
`609dfca`。新增显式 `--rx-fused-phr-cmf`（默认关闭）：在 payload 融合基础上把
PHR 改为 164 点融合模板 + 四候选相关，缓存 21 对 g0/g1 供联合 Viterbi，并把
soft-chip 前缀从 59,740 缩到 36,585 chip（只保留 NS-SFD +8 halo 的 9 个名义
PHR chip）。6 格×16 源窗逐窗 PHR bits/长度/纠错、comm/sensing PSDU/FCS/status、
committed/reason 一致，96/96 PHR 融合、0 回退；损坏包 16/16 融合尝试、16 次
完整回退、0 误提交。正式三格 `phr_path_with_diag` 87.1/84.3/89.5→18.0/17.5/
17.5 µs（0.206/0.208/0.196），通信 mean 0.8575/0.9065/0.8735，总 SIC mean
0.9375/0.9842/0.9487，prefix FIR 0.5741/0.6555/0.5848。PHR 内核单独（不含旧
FIR）约为旧解扩/译码的 2.0–2.2 倍，收益来自移除 PHR soft-chip FIR 和冗余前缀。
MATLAB、native、FPGA/硬件、paced、P99 与实时目标未验证。

---

# 历史任务：CMF 与解扩融合可行性已交付

结果见 `docs/sic/CMF解扩融合可行性报告_2026-11-07.md`，code-tested SHA
`57f97d5`。新增显式 `--rx-fused-payload-cmf`（默认关闭）：前缀 PHR/CIR/相位
不变，payload 用 52 点融合模板 + 四候选相关直接产生 BPM 指标；分数 spc、
RAKE、bypass 和失败路径保留旧流一次。6 格×16 源窗逐窗 comm/sensing
PSDU/FCS/status 一致，0 次正常格回退；损坏包 16/16 融合尝试、16 次回退、
0 误提交。正式三格 `payload_path` mean 98.4→56.4（0.573）、99.1→57.6
（0.581）、653.6→207.5 µs（0.317）；comm mean 0.945/0.955/0.801，总 SIC
mean 0.968/0.992/0.921，P95 同向下降。`stage_payload` 单独约 1.90× 旧值，
收益来自不再扩展 payload soft-chip；独立复核发现的尾部差一 chip 已修复。
MATLAB、native、FPGA/硬件、paced 与实时目标未验证。

# 历史任务：通信解码加速已交付

结果见 `docs/sic/通信解码加速开发报告_2026-11-07.md`，code-tested SHA
`f753f17`。A/B/C 三个显式选项：SFD VOLK 点积 + 模板缓存 + 尾部 ±8 seed、
CFO 1024 点块旋转、按 PHR 长度两阶段 soft-chip。正式三格配对 comm demod mean：
`sync_base` 2025.9→1656.6 µs（0.818）、`payload_base` 2764.8→1925.5 µs
（0.696）、`payload_four_frac` 2762.3→1757.4 µs（0.636）；P95 均未增加。
8 格×16 源窗逐窗 comm/sensing PSDU、FCS、status 一致，0 次 soft-chip fallback；
补充 16/127 B 两格 comm mean 为 0.576/0.809。默认选项全关，旧 build
`gr-uwb/build-sic-fullpacket-fused` 未覆盖；MATLAB、native、FPGA/硬件、
paced 到达与 mean≤0.5 ms/P99≤1 ms/200 窗/s 未验证、未宣称。以下为历史记录。

# 当前任务：FIR 乘加融合优化已交付

结果见 `docs/sic/DiscreteCIR_FIR乘加融合报告_2026-11-07.md`，code-tested SHA
`ddf335d`。融合 `dst += h*src` 后，12 tap 纯 FIR mean 1.072→0.548 ms
（比值 0.511），3 tap FIR 0.296→0.161 ms，3 tap wrapper 1.199→1.085 ms；
所有 tap 数 FIR/channel/wrapper mean 与 P95 下降。独立 double 卷积最大相对
L2 1.3524e-7（门槛 2e-5），针对性 QA 通过。新 build
`gr-uwb/build-sic-fullpacket-fused`，旧 build 未覆盖；未改估计器、tap 选择、
CFO、FCS/ROI 或默认后端，未上板，不宣称实时目标通过。下方为历史开发记录。

# 当前任务：离散等效 CIR full-packet 后端已交付

执行 `docs/sic/FullPacket_离散CIR估计开发任务书_2026-11-06.md`。本任务优先于下方历史入口。
code-tested SHA `10b660b`，新 build `gr-uwb/build-sic-fullpacket-discrete`。新
`discrete_cir` 用前 8 SYNC 直接拟合整数 lag 离散 FIR，冻结 preamble 路径与原
`path_fractional` 保持；不新增 GNU Radio block，不上板。正式结果见
`docs/sic/DiscreteCIR_开发报告_2026-11-06.md`：10 格×16 源窗功能 PASS，
负样本 PASS；两格串行配对总 SIC mean 为旧路径的 0.308/0.343，P95/P99 同时下降。
当前主瓶颈已转为 comm demod（约 2.0–2.6 ms）。`discrete_cir` 仍是显式实验选项，
不自动改默认；MATLAB/native/FPGA/硬件与 paced 到达未验证。

# 当前任务：SYNC+SFD / PHR / payload 分段实验已交付

结果见 `docs/sic/FullPacket_字段重构计时报告_2026-11-06.md`，code-tested SHA `32473f2`；payload_base热缓存分段为0.874/0.321/1.836ms，公共0.249ms，分段总计比原库低4.5%，不得冒充生产精确分摊。用户授权离线分段实验；生产重构库保持 `1adf401` 的VOLK实现。新增独立诊断exe，按波形字段划分滤波，按最早估计路径锚定输出时间区间划分路径叠加与CFO；保留完整波形halo。公共开销单列，冷/热cache分别观察，原库逐次配对验证输出与计时扰动。只测两格16源窗，每格每变体128次×2轮，独占基准锁；针对性自测通过后绑定提交运行，不扩大历史矩阵。

# 当前任务：Full-packet CFO 的 VOLK 加速已交付

结果见 `docs/sic/FullPacket_VOLK加速报告_2026-11-06.md`，code-tested SHA `1adf401`；CFO阶段约7.6–7.8倍加速，双检查保留，10格功能通过。用户已授权在本实验分支加速乘法。保留1024点绝对相位锚定、双精度累加、转换/试减有限性检查与事务提交；采用本机VOLK分派，不新增GNU Radio block，不修改冻结前导核心。新build `gr-uwb/build-sic-fullpacket-volk`；旧build与结果保留。仅做针对性QA、10格功能对照与两格串行配对计时，不自动上板或扩大历史矩阵。

# 当前任务：CFO/转换/数值检查的隔离诊断已交付

结果见 `docs/sic/FullPacket_CFO拆分诊断报告_2026-11-06.md`，code-tested SHA `4b5da96`。已新增离线scratch重放工具，两处通用复乘为主要开销；生产融合循环与在线检查保持不变，原数据保留。不自动开始下一轮优化。

# 当前任务：payload 前缀缓存对照已交付

本轮结果见 `docs/sic/FullPacket_payload滤波对照报告_2026-11-06.md`，正式代码 `cea0fc3`；不自动开始下一轮优化。用户已授权测试仅对payload及边界区逐包滤波，固定前缀相位行按实际波形缓存；完整包仍逐路径处理和消除。新build `gr-uwb/build-sic-fullpacket-payload-cache` 保留旧build与结果，不上板。快速RX接入与复测已交付：`docs/sic/FullPacket_快速RX复测报告_2026-11-06.md`，正式代码 `28c7734`；不自动进入重构优化。历史报告 `docs/sic/FullPacket_开发报告_2026-11-06.md` 与 `5dd2cfb` 结果保留；新run独立归档。仅调整实验runner的RX路径，不改冻结SIC数学，不上板。

执行 docs/sic/FullPacket_vs_PreambleOnly_开发计划_2026-11-06.md。仅本实验分支允许新增整包路径；原 preamble 行为与历史基线保持。主 agent 与三个 subagent 按文件所有权协作。

---

# SIC 性能优化（feature/sic-perf，2026-10-19）

本 worktree 从 annotated tag `sic-s2-ab-integrated-20261018`（`794056c`）创建，分支 `feature/sic-perf`。任务文件是 `docs/sic/Grok开发指示_SIC性能优化_2026-10-19.md`。

冻结基线的算法、阈值、种子、指标和信道仍然约束历史对照。只有本性能分支可以按该任务改共享 SIC 核心。不要把本分支自动合并回 `feature/sic-research`，不要移动 tag，不要推送。

只读对照 worktree 是 `.worktrees/sic-perf-baseline`（detached `794056c`）。两边各自构建 `gr-uwb/build-sic-accum` 与 `gr-uwb/build-sic-payload`，不覆盖旧集成 build，不复制 CMakeCache。mean≤0.5 ms、P99≤1 ms、200 窗/s 仍是未达成目标，不是本轮承诺。

---

# 已验收 A/B 集成基线（2026-10-18）

本文件是 S2-A 与 S2-B 合并后的共同研究基线。下方 `---` 之后的 S0/S1/TWR
条文是继承规范，不表示当前任务仍是单方向整改或 S1 实现。新的算法实验从集成
tag 另开 worktree，同时保留 A 与 B 的评价；本基线本身不自动开始下一轮实验，
也不上板。

| 方向 | 历史锚点 | 最终报告 | 评价 |
|---|---|---|---|
| A：累加后 sensing CIR | tag `sic-s2a-accepted-20261018`（`2c383d5`），worktree `.worktrees/sic-sensing-accum` | `docs/sic/S2A_整改开发报告_2026-10-18.md` | 12 格代表回归、旁路、A-N02 基线 |
| B：sensing 前导干扰通信 payload | tag `sic-s2b-accepted-20261018`（`ac3849e`），worktree `.worktrees/sic-payload-collision` | `docs/sic/S2B_整改开发报告_2026-10-18.md` | 11 格 integration smoke，不是正式 32-cell 矩阵 |

统一入口是根目录 `OPENCODE_START.md`。冻结与合并过程见
`docs/sic/S2_AB冻结与合并执行指南_2026-10-18.md`、
`docs/sic/S2_AB冻结清单_2026-10-18.md` 和
`docs/sic/S2_AB集成验收报告_2026-10-18.md`。
回归命令见 `docs/sic/S2_AB集成回归入口_2026-10-18.md`。
各方向原来的 `OPENCODE_START` 内容已并入统一入口；单方向开发指示保留为历史文件。
2026-10-18 的集成回归绑定 code-tested SHA `2cea67b`。验收记录在上面的报告里。
annotated tag `sic-s2-ab-integrated-20261018` 指向包含该报告的提交。

必须同时保留的结论边界：

- 共享 SIC 核心、adapter、`tools/sic/sic_reference.py` 和原 core/adapter QA 门槛保持冻结。在线路径只接收混合 IQ、公开模板和公开粗 ROI。注入时延、CFO、路径和 IQ 真值只进 sidecar 与离线 verifier。
- A 的名义 CIR 抑制约 28.4 dB，残余高度相干。困难 CFO 的约 12.61 dB 改善是离线有界诊断，不是新的在线结果，也不把 CFO=0 说成已经局部最优。
- B 的 ISR=20 dB 恢复只适用于已测单径、零 CFO 条件。B-N01 之后，库信任根来自 verifier 所在源码根或显式 `--repo`/`--trusted-build`，不能由 manifest 自报的 `build_dir` 给自己背书。
- MATLAB、native、FPGA/硬件和正式实时吞吐都未作为本基线的通过项。mean≤0.5 ms、P99≤1 ms、200 窗/s 仍是目标，不是已实现事实。功能回归通过不等于性能通过。
- 正式性能数字必须独占 `/tmp/uwb-sic-benchmark.lock`。大 IQ、build 和冻结包不进入 Git。历史 run 不被新 run 覆盖。

两个 build 都从本基线源码串行配置，不复制旧 CMakeCache，不软链旧 build，不安装到 `/usr/local`：

- A：`gr-uwb/build-sic-accum`
- B：`gr-uwb/build-sic-payload`

本地 `UWB_demodulation` 仍是指向主仓库 MATLAB 资料的只读软链接，禁止写入。

---

# UWB GNU Radio Project

## 当前分支：SIC 论文研究（优先执行，2026-10-16）

当前分支 `feature/sic-research`，从 `feature/uwb-ds-twr` 的 `d0c8138` 创建。
当前用户任务是 **S1 八符号估计与全前导消除**，不是继续 TWR 里程碑。
先读根目录 **UWB_Preamble_SIC_GNU_Radio_Design.md**，再读
**docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md** 与
**docs/sic/S1_八符号模型API.md**；前者为 OpenCode 可执行任务与验收依据。
S0 的指示/合同/整改报告保留为历史基线，S1 语义优先于旧 fit/validation 耦合。

### S0 任务与约束

- 复用现有 sensing RX 仿真，在时间上重叠的 SYNC 区注入不同 preamble code 的
  comm 前导：默认 sensing code 9、comm code 10、64 SYNC、998.4 MS/s。
  “重合”是时间重叠，不是同码；同码只在显式困难诊断用例中启用。
- 先做 work-grid CF32，再接现有 CG400 native 491.52 → work 998.4 PDU 65/32。
  保留完整 sensing RX 给解调器，不能把只覆盖 preamble 的 CIR 窗拿去完整包解调。
- comm 是 **preamble-only**；SIC 不要求解码 comm PHR/payload/FCS，不复用旧
  full-packet SIC 的 FCS 门槛。旧 `feature/sic-validation` 保留，不整体合并。
- clean/raw/sic 按同一窗口配对，报告 sensing CIR/弱目标保护及完整 sensing 包
  解调状态。在线 SIC 只接收公开模板/粗 ROI，不得读取注入时延/CFO/路径/IQ 真值；
  oracle 只作单列诊断。相减总功率下降不等于消除了干扰。
- 新增 SIC 数学核心与 GNU Radio adapter 分离；先 Python 数学参考与 C++ QA，
  再薄 PDU block、benchmark。执行范围和顺序按 S0 指示，不自动上板。
- 完整解调、radar CIR-only、SIC 前端分开计时。当前 `stage_sfd_us` 还包含
  tail timing/CFO refit，`stage_cir_us` 含 soft-chip；补诊断子计时，保留旧字段语义，
  父子计时不二次累加。失败路径、PMT/队列与生成器开销必须单列。
- 默认单 worker，scratch 预分配；同时限制 worker 队列与 PMT 入口 in-flight。
  回放源延迟提交也算 missed admission，不能靠降低实际输入率掩盖过载。
- 原有 MATLAB canonical fixture 不覆盖；新信号/脚本存 `testdata/sic_preamble/`。
  MATLAB 未运行明确写未验证。既有 resampler SIMD 非确定性只记录不修，保留
  scalar/reference 对照，不把其风险混同为 SIC 算法失败。
- 所有性能数字重新绑定 commit/profile/实际加载库/窗口长度。SIC mean≤0.5 ms、
  P99≤1 ms 和 200 窗/s 是验证目标，不能预先写为实现事实；不宣称连续 1 GS/s 已通过。
- 新 block 前仍须查本地 GNU Radio 类似实现、说明 scheduler 语义、先补针对性 QA，
  每次有意义修改后构建/测试。吞吐 benchmark 串行、同环境配对。

S0 已实现并完成一轮独立整改（R01–R10）。开发结果见
**docs/sic/S0_前导碰撞仿真与耗时开发报告.md**（含 superseded 说明）与
复核依据 **docs/sic/S0_整改意见报告_2026-10-16.md**、整改结果
**docs/sic/S0_整改开发报告_R01-R10.md**、测量合同
**docs/sic/S0_整改_测量合同.md**。效果以“真实输出”指标（`actual_suppression_db`）
为准，不是重构 NMSE；工作/采样网格与效果/算力/CIR-only/full demod 分列，通过项、
失败项、未实跑项（MATLAB 未实跑等）均列明。关键结论：主矩阵 68/72 cell 通过、
负样本误提交 0/3000（重叠门控修复后）；SIC 核心实测 ~5 ms/窗（真实负载），
未达 mean≤0.5 ms/200 窗/s 相关门槛（core/GR 达标旧口径已 superseded）。

### S1 八符号估计与全前导消除（当前默认路径）

依据 **docs/sic/OpenCode修改指示_S1_八符号估计与全前导消除_2026-10-17.md**，
契约 **docs/sic/S1_八符号模型API.md**。默认路径 `estimate8_frozen`：

- 严格只读前 8 个 SYNC 的 `observed_end_sample` 前缀完成检测/CFO/CIR/分数时延/
  增益估计，冻结为 `SicModel`（`model_id`/fingerprint）；`apply_model` 不再重拟合。
- 支持由 **configured nominal prediction**（`origin + sync_reps*sps + post_tail`）
  预测，不再被 `confirmed*sps` 截断；相减覆盖完整前导，含独立评估区与末 4 SYNC。
- 默认 `online_validation_reps=0`（不等 holdout 才相减）；可选显式保守对照。
  旧 `--sic-cover-preamble` 仅为 `confirmed_legacy` 别名，不得当作 S1 结果。
- 四区分离：acquisition/search、training `[0,8)`、cancellation（全支持）、
  evaluation（离线 `[8,64)`，只评估不反馈）。效果以真实输出残余为准。
- 验证：core QA 28 例（前缀隔离含 guard page、全支持、末 4 SYNC 实减、
  chunk 逐位一致、外推失配、旁路、R05 数值安全）；verifier 增加 S1 检查。
- 关键风险（如实保留）：8 符号 CFO 精度限制远端深度；CFO 非零 cell 的
  tail 抑制明显低于名义 CFO=0 的 20–25 dB 目标。性能门槛仍未达。

S1 阶段结论以 **docs/sic/S1_八符号估计与全前导消除开发报告.md** 为准；
不能只引用本轮阶段性数字。

## 继承基线：UWB SS-TWR / DS-TWR

`feature/uwb-ds-twr` 从 radar 分支 `b897677` 创建（首个提交 `38fbe2c` 为纯文档），
同时开发 SS-TWR 与 DS-TWR。
主要需求是 **docs/twr/需求_UWB_SS_DS_TWR.md**，开发顺序和验收见
**docs/twr/开发路线与验收矩阵.md**，芯片资料和 API 映射见
**docs/twr/参考资料与API映射.md**。先读这三份文档和 **开发状态.md**。

**M0.1 第一轮已关闭 R1–R8，第二轮已关闭 N01–N06，第三轮已关闭 N07（枚举域
fail-open）；M0.1 契约层完成，TWR 协议能力仍未实现**（2026-09-29）。先读
**docs/twr/评审意见_M0.1_2026-09-29.md**（N01–N06）、
**docs/twr/M0.1第二轮整改报告_N01-N06.md**、以及
**docs/twr/M0.1_N07修复报告_2026-09-29.md**（N07：审核报 1 处，实际 20 处）。
当前进行 **M2-A native PHY 闭环**（未完成：A10 损伤压力、A13 分配/延迟、A11 MATLAB 按用户
指示忽略）。见 **docs/twr/M2-A_G0接口与数字坐标.md**、**docs/twr/M2-A_评审报告.md**、
**docs/twr/M2-A_native_PHY开发报告.md**。M2-A 只证 native 往返与数字坐标，**不证**首径
ToA/硬件测距；`allows(NativeRoundtripVerified, Ranging)` 仍为 false。**不自动进入 M2-B**。
另：重采样核 SIMD 路径存在已复现的未初始化读取（输出非逐位可复现），按任务单只记录不修。

此前已完成 **M1-A 独立 ToF 数学 + MATLAB oracle**，以及
**M1-B 离线协议核心与双端仿真**（2026-09-30 交付，经第二轮独立复核发现 R01–R09，
已全部整改关闭，见
**docs/twr/M1-B_复核整改报告_2026-09-30.md** 与
**docs/twr/M1-B_复核整改审核意见_2026-09-30.md**）。报告见
**docs/twr/M1-B_协议核心开发报告.md**（其"完成"结论仅在整改后成立）。
下一阶段是 M2-A native PHY 闭环，再是 M2-B RMARKER/首径/时映射/timestamp patch，
随后 M3/M4 上板；**不自动进入 M2**。
**M1-B（复核整改后）**：纯 C++ SS/DS 两端协议核心、wire claim 边界、确定性 fake link、
B01–B18 QA、离线 demo/独立 verifier、独立安装消费者。复核整改关闭了：计划可携带
Completed 提前成功、DS initiator 遗漏 Poll outcome、非估计端绕过本端准入、
reset 清身份屏障、responder 未预留终态容量、空域外 outcome 生成无主终态、
evidence 覆盖绝对 deadline、计划无数值映射、TwrConfig 校验后丢弃，以及复审的
计划 source 未约束与 `exchange_timeout==0` 无绝对上限。
**它不是硬件测距**：结果是带 simulation/wire-claim 标注的协议估计，
`measurement_valid=false`，ToF 公式未对硬件/native/首径验证。
M2-A native PHY / M2-B RMARKER·首径·时映射·timestamp patch / 两 RX 路由 /
controller / UHD / pybind 大迁移均**不在** M1-B 范围；不自动进入 M2。

第二/三轮的关键教训（不要再犯）：

- **把错误行为写进测试约定**：N02 的 QA 曾显式断言"`scheduled_calibrated` TX 必须被拒"
  并因此把 REQ-TIME-03 明确允许的路径堵死；N01 的语料 note 描述了代码里**不存在**的机制。
  语料一致只能证明指定输入上的一致性，**不能证明这些共同约定正确**。
- **枚举的"成员"不是它的"域"**：一个枚举变量可以持有枚举没有的值。所有判定必须
  **显式做域检查并 fail-closed**，且**先于**依赖该值的检查（否则报的是无关字段）。
  `switch` 用**无 `default`** 形式以保留 `-Wswitch` 保护，域测试放在 switch 之前。
- **验证要用穷举而不是抽样**：N07 的 12 处 `TimedField.marker` 是抽样会漏、按类型注解
  穷举才抓到的；可选字段（默认 `None`）尤其容易漏。`validate()` **必须报告，不能抛异常**。
- 逐包时序由 C++/硬件定时驱动；scheduled TX 必须带 `TxSendEvidence`
  （command time / 量化时刻 / marker offset / 校准空口时间 + `outcome == Completed`），
  **`send()` 成功或 BURST_ACK 单独不构成证据**，且入门的值**绝不改标**
  `hardware_measured`。
- **16 SYNC 和 64 SYNC 一样没有 ToA 证据**，"窗更长"不是证据；
  `allows(work_decode, Ranging) = 0` 对全部 48 行成立。

M0 初版设计、回归和调度说明分别见 **docs/twr/M0_设计契约.md**、
**docs/twr/M0_回归基线.md**、**docs/twr/M0_调度语义.md**。
其中与复核报告冲突的结论不再作为冻结约束；不能用“已冻结”阻止修复已复现缺陷。

第一阶段：一台 X410 两个 channel 各作为独立逻辑端点，实现真实空口/有线的
SS/DS 测距及角色互换。第二阶段：X410 与指定 DW1000、DW3000 模组，分别完成
两种协议和两端角色互通。现有 radar、TX/RX、解调和 MATLAB 代码是复用与回归基线。

### 当前事实与必须关闭的缺口

- frame v1 为 14 B 头 + 40-bit LE timestamps；Poll/Response/Final 空口长度为
  16/26/31 B，FCS 由 HRP 层追加。它是阶段一内部协议，商用适配需另验具体固件。
  Report 保留且未实现；STS 本阶段不开发。
- 配置/codec 的 14 B 几何、session 收窄、独立 PHR 速率约束已整改；现有调制器
  是 0.85 Mb/s PHR + 6.81 Mb/s payload。frame 的运行 seq 是 uint16，配置初值
  `session.sequence` 为 uint8；M1-B 必须显式管理运行身份和旧帧复用屏障。
- Python effective/snapshot 的修改隔离和 capability 公开解冻漏洞已整改，保留回归。
- 时间数学的精确投影、半周期和分数边界已整改；ToF 不使用有损整数 ns 投影，
  本端输入显式校验 range-capable marker、校准和质量，不能只依赖 interval 返回 Ok。
- 48 行白名单证明的是 998.4 MS/s work-grid 解码自洽，未证明 native/ToA/硬件互通。
  16 SYNC 在 ToA 验证前不得用于测距。128 等暂因当前链路限制拒绝，不能写成芯片
  不支持；能力按 work/native/ToA/hardware/vendor 证据分别升级。
- 仓库已有 Python work→native resample_poly；缺口是反应式 TWR 所需的 C++ 集成、
  buffer 复用与时间映射。两路 RX、分数首径和 delayed-frame timestamp patch 仍待实现。
- controller 可采用零流端口 gr::block + 短 handler + 有界队列 + 单 owner；
  还必须限制 PMT 入口积压，其他 adapter 按实际 scheduler 语义选型。
- X4xx ctrlport `EXEC_LATE_CMDS` 与 timed TX 数据是不同路径；TX 需处理设备
  `ERR_TX_LATE_DATA` 和主机 deadline。具体 FPGA/UHD 行为需绑定版本上板验证。

### 验证环境

检查测试二进制实际加载的库。本机默认 LD_LIBRARY_PATH 可能命中旧 /usr/local 库，
CTest 脚本也会继承它；本地验证使用 `env -u LD_LIBRARY_PATH ctest ...` 并用 ldd 确认
build/lib。手工测试在环境正确时有效；不把 sudo install 作为普通测试前置条件。
吞吐 benchmark 串行、同环境对照，失败单列；CSV 输出到临时/build 目录后显式归档。

### TWR 开发必须遵守

1. 将协议 FSM、frame codec、时间戳/ToF 数学核心与 GNU Radio/UHD 适配分开。
   Python 做配置和结果消费，逐包时序由 C++ 及硬件定时驱动。
2. 复用现有 HRP TX、detector/extractor、resampler、demod 和 burst backend。
   现有双 TX/单 RX radar 路径不等于双端 TWR；需显式设计两个 RX 路由和共享设备资源。
3. 端点只使用本端硬件事件及对端空口帧携带的信息。不得用共享 schedule、私有时间戳
   或已知距离计算待测 ToF；真实有效 RX 帧才能触发 Response/Final。
4. 每个时间字段明确 clock domain、epoch、tick rate、marker、量化和校准。
   区分 RX 第一 IQ 样点、RMARKER、首径、预期时间和 TX command time。
   禁止用 radar `predicted_sfd`、最大 CIR tap 或主机收包时间直接当作实际 ToA。
5. delayed TX deadline 不满足时显式失败，不即时补发沿用旧 timestamp。
   回复延迟预算覆盖帧尾、解码、构帧、传输和设备提前量；不得承诺未经验证的短时序。
6. SS clock ratio 的方向/符号/换算及 DS 不等回复时间公式均须独立测试。
   单台 X410 共钟不替代独立时钟、回绕、CFO/SFO 和延迟校准验证。
7. 参数必须经统一 capability/profile 校验并记录 readback。未知或不支持的模式拒绝，
   不静默采用 QM35/radar 默认值；芯片 power word、TX gain、IQ amplitude、dBm 分开。
8. 商用互通绑定具体模组、SDK/固件 hash、帧格式、时间戳单位和 PHY profile。
   优先共同 channel 5 无 STS 配置；每种新增能力单独验证。原版与改时序固件分开报告。
9. 每个 exchange 有唯一 terminal result，保留 timeout、FCS 错、late、underflow、
   overflow、队列丢弃和取消。不得通过跳号补采或重试掩盖失败测量。
10. `testdata/twr/` 保存短信号/帧/time golden 和独立 MATLAB 验证脚本。
    实现前新增针对性 QA，改动后构建/测试。未完成的模组测试、精度标定与 soak
    明确写为待验证；禁止宣称厘米级精度或“完整互通”而没有相应证据。

## 开发规范与参考

### 主要开发需求参考
本分支优先参考上述 TWR 需求；同目录 **开发需求参考.md** 保留早期检测截取需求。
其中“只截取、不做实时解调”“暂不连接 USRP”等历史范围不限制当前 TWR 开发。

### 当前开发状态
请参考同目录下的 **开发状态.md**（已完成的模块、算法参数、测试结果、构建运行方法与下一步计划）。

### 周期旁路雷达截取（Phase-1 已完成）
请参考 **docs/phase1/开发总结_QM35825周期旁路雷达截取.md** 与
**docs/phase1/GROK_X410_QM35825周期旁路开发方案.md**。
已知 `t0/T` 时使用 `UwbScheduledExtractor`，不要用能量门对每个通信包建 Region。

### QM35 盲捕获与自动周期锁定（离线最小闭环已通）
未知 `t0`、已知 T=5 ms 时使用 `UwbAutoScheduledExtractorSc16`，不要并联
Detector + ScheduledExtractor。方案见
**docs/phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md**；
mixed 737.28 离线实测见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_mixed737p28.md**；
无干扰 QM35 同一流程见
**docs/phase1/测试报告_QM35盲捕获与自动周期锁定_clean737p28.md**。
硬件 soak / overflow 重捕获未完成，不得写成 X410 盲捕获验收通过。

### UWB 算法与验证
- UWB 检测算法（能量门限、粗检测、细相关、preamble 匹配等）**必须**参考 `UWB_demodulation/` 目录下的 MATLAB 实现（包括 `buildUwbReference.m`、`decode_uwb.m`、`analyze_*` 等文件）。
- 算法正确性验证优先：使用 `testdata/` 下的已知 UWB 测试信号（`.cfile`、`*.dat`、`*_metadata.mat`、`UWB_test_signal_description.md`）进行 MATLAB 对照。
- 检测结果（packet start、IQ 截取范围）必须与 MATLAB 实现逐样本/逐 packet 对比。

### 开发规则（必须严格遵守）

Before implementing any GNU Radio block:

1. Search the local GNU Radio source tree for similar blocks.
2. Determine the correct block type:
   - sync_block
   - block
   - tagged_stream_block
   - message/PDU
3. Explain scheduler semantics before implementation.
4. Avoid allocating memory inside work/general_work.
5. Add QA tests before declaring implementation complete.
6. Run build and tests after each meaningful modification.

### Performance goal

Input data may eventually approach 1 GS/s.

Do NOT assume a naive full-rate correlation implementation is acceptable.

Preferred detection pipeline:

energy / coarse detection
→ candidate region
→ preamble verification
→ packet extraction

### 验证要求

- Synthetic UWB signals should be generated for unit testing (use `testdata/`).
- Detection results **must** eventually be cross-checked against MATLAB implementation in `UWB_demodulation/`.
- Add MATLAB scripts in `testdata/` or root for automated verification.

### 其他

- All test data is located in `testdata/`.
- Full MATLAB demodulation pipeline is in `UWB_demodulation/`.

## Goal

Current branch: validate detection-triggered preamble-only SIC in GNU Radio
simulation, with different sensing/communication codes overlapping in time.
Measure sensing CIR recovery, target preservation, full sensing demodulation
and SIC costs separately. Follow the S0 instruction above; do not advance
to hardware or TWR milestones automatically.

### Inherited TWR goal

Build configurable SS-TWR and DS-TWR on the existing GNU Radio C++ UWB TX/RX
chain, with Python APIs, timestamp calibration, explicit error handling and
MATLAB cross-checks. First validate two logical endpoints on one X410, then
validate both roles against specified DW1000 and DW3000 modules. Preserve
the existing packet capture, demodulation and radar regression baselines.
