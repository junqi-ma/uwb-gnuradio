# Full-packet SIC 快速 RX 接入

用户已授权在 `feature/sic-fullpacket-compare` 接入成熟快速解调并重新分析耗时。
复用当前 `demod::core::demodulate_one`（成熟 TWR/RX pipeline 加既有诊断），
不回移历史分支算法、不删长窗口同步/CFO修正、不裁短窗口、不改FCS门控。

- `apps/fullpacket_rx_pipeline.h`：预分配两路RX scratch、完整输入窗口、公开/估计seed。
- `benchmark_fullpacket_compare_fast_rx`：同一runner加 `-march=native`，启用既有38tap AVX2/FMA FIR及编译器本机向量化；不启用fast-math。
- `benchmark_fullpacket_compare`：保留portable对照，不加本机ISA选项。
- 新编排器 `--rx-pipeline fast|portable`，默认fast。通信解调及下游sensing解调同时切换，队列、输入与SIC模型不变。
- 共享SIC核心、整包重构库保持原编译配置。只有实验runner及native wrapper QA使用本机ISA；不把整包重构的提速归到RX。
- 行记录和summary含 `rx_pipeline` / `rx_avx2`；编排器逐行校验实际编译能力，记录对应target的flags.make和二进制/实际库hash。
- `comm_stage_us` 为解调七个父阶段的诊断，子阶段不二次计入SIC总时间。

验证预算：新增native wrapper两组契约QA和原Python verifier QA；fast的固定10格各16窗输出校验；portable仅做短接线回归。
正式计时复用同16源窗，每格每组1024次、两轮；portable/fast串行交错，锁内运行，32次热身。
随后fast单worker进行两代表格的200窗/s、10秒回放。不重跑历史A/B整套模拟或全矩阵，原共享代码未改。

历史数据保留在原 results/functional、timing、paced。新结果写独立 fast_rx_* 目录。
原测量二进制已按sha256归档到 results/archived_binaries/，旧报告复现需显式 `--rx-pipeline portable`。
这轮只接入已有RX加速，不优化未知payload的信道重构；剩余瓶颈由重测结果判断。
