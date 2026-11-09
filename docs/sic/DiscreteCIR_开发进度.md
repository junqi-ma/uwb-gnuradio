# Discrete CIR 开发进度

任务书：`FullPacket_离散CIR估计开发任务书_2026-11-06.md`。
基线：`887e7bb`；本实验分支 `feature/sic-fullpacket-compare`，worktree
`.worktrees/sic-fullpacket-compare`。新 build：`gr-uwb/build-sic-fullpacket-discrete`。

    [阶段 S0 / 负责人 主 agent] 完成
    完成内容：冻结新公共头文件、数学/支持选择/归一化合同，建立粗 CIR 峰值组
    active lag + ridge LS 的接口；连续 hull 只用于表示支持，未激活 tap 为零，避免把
    sensing 前导拟合进 h；旧 preamble/path_fractional 路径未改。
    证据：gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h、
    docs/sic/DiscreteCIR_接口合同.md；离线原型（仅设计，不作为验收）在
    payload_base/sync_base/payload_four_frac 上验证整数/四径分数信道可由离散 FIR 表示，
    并确认必须用公开模板归一化，不能重复乘 waveform scale。
    问题：连续宽支持会把 sensing 前导拟合进去并损伤 payload 区；独立 seed 3000000
    校准后改为峰值组 active 集合（floor 0.15、间距 5、半径 1），单径约 3 taps、
    四径约 12 taps/hull 54，原型不再退化。
    下一步：subagent A/B/C 并行实现估计器、重构器、评价工具；主 agent 接 benchmark。

    [阶段 A / 负责人 subagent A] 完成
    完成内容：独立 acquisition（检测/CFO/折叠粗 CIR）、峰值组 active lag 选择、
    固定 factor cache 的 ridge LS、拟合条件/残差门控、模型 hash；无旧分数路径依赖。
    证据：gr-uwb/lib/uwb_discrete_cir_estimator.cc；
    gr-uwb/lib/qa_uwb_discrete_cir_estimator.cc 7 例通过（整数 FIR、独立分数多径、
    负 lag/CFO、前缀隔离、非法输入、错模板、第二前导不被拟合）。
    问题：连续宽支持会把 sensing 拟合进 h；已用峰值组 active 集合解决。
    下一步：主 agent 集成 benchmark。

    [阶段 B / 负责人 subagent B] 完成
    完成内容：直接 FIR 重构、单次波形归一化、绝对锚点 CFO、边界/事务旁路、
    FCS/PHR/profile 包装；主 agent 追加 tap-major + VOLK 内核。
    证据：gr-uwb/lib/uwb_discrete_cir_reconstruct.cc；
    gr-uwb/lib/qa_uwb_discrete_cir_reconstruct.cc 6 例通过（独立双精度卷积、
    截断/ROI/包尾、归一化、FCS/PHR/profile、16/127B、数值安全）。
    问题：无。
    下一步：主 agent 集成。

    [阶段 C / 负责人 subagent C] 完成
    完成内容：独立离线比较 runner/verifier/QA；同源窗配对、逐格质量门、负样本
    target 区不变、timing AB/BA 与质量摘要绑定。
    证据：tools/sic/run_discrete_cir_compare.py、
    tools/sic/verify_discrete_cir_compare.py、tools/sic/qa_discrete_cir_compare.py；
    verifier 自检 baseline + 20 mutations PASS。
    问题：initial negative no_overlap 检查过严（要求整窗不变），已改为 target SYNC
    区逐位不变，允许区外消除。
    下一步：主 agent 正式功能对照与计时。

    [阶段 S2 / 负责人 主 agent] 完成
    完成内容：新 build `gr-uwb/build-sic-fullpacket-discrete` 配置/构建；
    `benchmark_fullpacket_compare --reconstruction discrete_cir` 接线；
    新 QA + 原 fullpacket/preamble core QA 4/4 通过；10 格×16 源窗功能对照 PASS，
    负样本 no_interference/no_overlap PASS；配对 comm/sensing 16/16 不低旧路径。
    证据：`docs/sic/DiscreteCIR_开发报告_2026-11-06.md`、
    `gr-uwb/build-sic-fullpacket-discrete/results/functional_discrete_20261106_r2`。
    问题：初版连续支持在 sync_base 会拟合 sensing；已用峰值组 active 集合修复。
    下一步：干净提交后正式功能/计时。

    [阶段 S3 / 负责人 主 agent] 完成
    完成内容：code-tested `10b660b` 上重跑正式功能、负样本和串行配对计时；
    两格 mean 比为 0.308/0.343，P95/P99 同时下降，质量门全部通过。
    证据：`results/functional_discrete_20261106_r2`、
    `results/negative_discrete_20261106_r2`、`results/timing_discrete_20261106_r2`；
    `docs/sic/DiscreteCIR_结果摘要_2026-11-06.json`、
    `docs/sic/DiscreteCIR_计时摘要_2026-11-06.csv`。
    问题：当前主瓶颈已转为 comm demod 约 2.0–2.6 ms；事务双拷贝约 0.14–0.25 ms。
    下一步：不自动改默认；若继续，先做 paced 到达与双拷贝优化，扩大信道覆盖。
