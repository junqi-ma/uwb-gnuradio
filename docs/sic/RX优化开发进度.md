# RX 优化开发进度（2026-11-07）

格式：`[阶段/负责人] 状态；实际改动；验证/数值证据；下一步。`

- [接口冻结/主] 完成。新增 `Qm35825Profile::rx_fast_sfd/rx_fast_cfo_rotation/rx_seed_refine_tail/rx_softchip_on_demand/timing_seed_refine_radius`（默认关闭）、`DemodResult`/`TimingResult`/`CfoResult`/`CirResult` 诊断字段、`DemodScratch` SFD 缓存与两阶段 soft-chip 状态、`FastRxPipeline::Options`；benchmark 增加 `--rx-*` 选项和子计时/fallback 输出；新增 `tools/sic/run_rx_opt_compare.py` 配对驱动。新建 `gr-uwb/build-sic-fullpacket-rxopt`，旧 build 未覆盖。`uwb_demod_core`/`fullpacket_fast_rx` QA 通过，默认路径选项全关。
- [A/同步与 SFD] 完成 helper 与接线。`uwb_rx_sync_opt.h` 提供 VOLK 复共轭点积、SFD 模板内容缓存、窄 seed 半径；`stage_sfd` 粗/细相关均走 `rxopt::conjugate_dot`，候选/阈值/顺序不变；`stage_timing` 首次窄搜（默认仅尾部 ±8）失败自动回退原 `max(256,stride)`，首轮窄搜与回退分别计数，`seed_search_us` 不重复计入 `coarse_search_us`；尾跟踪不足和最终 SFD 失败各有一次原半径重试。局部 QA：点积相对误差 2.8e-6，模板与原构造逐位一致，seed +3 窄搜命中、+12 回退恢复、+400 两路径都失败不伪造捕获。
- [B/CFO 旋转] 完成 helper 与接线。`uwb_rx_cfo_opt.h` 用 1024 点绝对相位块 + VOLK 复乘替换四条逐样本递推链；相位拟合、两次调用顺序、符号和失败行为不变；`fit_us`/`rotate_us`/kernel 位单独输出。独立 double 参考最大绝对误差 1.09e-6、相对 8.8e-7；option 关闭时与旧循环逐位一致。
- [C/按需 soft-chip] helper 与两阶段接线完成。`uwb_rx_softchip_opt.h` 给出 PHR 长度 chip 公式、旧预算、前缀预算、PHR 后最终预算和前缀重标定；`stage_cir_softchips` 增加显式 `max_chips/begin_chip`，前缀只生成 SYNC/SFD/PHR + guard，PHR 后按校验长度扩展；前缀/扩展复用同一 CIR/tap/网格，最终统一归一化；分数 spc、RAKE、bypass 和修正 PHR 回退完整旧路径一次并计数。默认 profile 预算：旧 184758、前缀 59741、32B 最终 67872、127B 最终 124704 chips。
- [集成/功能] 完成首轮。8 格×16 源窗 `sync_base/payload_base/payload_four_frac/payload_cfo_m50k/payload_cfo_p50k/payload_len16/payload_len127/payload_snr8` 逐源窗 comm/sensing PSDU、FCS、status、SFD/CIR 位置一致；0 次 soft-chip fallback（本轮全部整数 spc=2）。新增 QA：全选项与旧路径 PSDU/FCS/位置一致、32B 前缀 chip 流对旧流缩放后最大误差 <1e-5、PHR/SFD 损坏时回退一次且状态与旧路径一致、尾部 seed +12 回退。
- [配对性能/主] 待执行。计划：`sync_base/payload_base/payload_four_frac` 三格，16 源窗，每变体 16 预热、128 观测×2 轮，旧→新/新→旧，独占 `/tmp/uwb-sic-benchmark.lock`。
