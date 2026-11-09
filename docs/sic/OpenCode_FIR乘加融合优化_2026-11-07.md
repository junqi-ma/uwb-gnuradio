# FIR 乘加融合：快速开发任务书

## 目标与范围

在 `.worktrees/sic-fullpacket-compare` / `feature/sic-fullpacket-compare` 开发。
基线 `fd93b54`，计时工具提交 `2449032`。只优化离散 CIR 的 FIR 内循环，减少
临时数组读写；不改估计器、tap 选择、CFO、FCS 门槛、数值检查、相减 ROI 或默认后端。
不新增 GNU Radio block，不上板，不扩展解码/评价框架。

目标：同轮对照中 12 tap 纯 FIR mean 至少降低 20%；3 tap 的 FIR mean 与整包
wrapper mean 不明显退化（不超过 5%）。这是工程目标，不预先承诺实现。

## 原理与最小实现

当前 `gr-uwb/lib/uwb_discrete_cir_reconstruct.cc` 的每个 tap 执行：

    tmp = h * src       // volk_32fc_s32fc_multiply_32fc
    dst += tmp         // volk_32fc_x2_add_32fc

改为融合复数乘加 `dst[i] += h * src[i]`，消除 FIR 对 tmp 的写入和再读取。
保留 tmp 给 CFO 使用，不要误删。整数延迟仍由 src/dst 偏移表示；保留每个输出
样点的 tap 累加顺序、现有边界及预分配方式，热路径不增加分配。

先检查本地 VOLK 是否已有适用的融合内核；有则复用。否则实现小型 C++ 融合 helper，
先检查 Release 编译器向量化结果。只有确实未向量化且计时未达目标时，再考虑一个
有运行时能力检查和 scalar fallback 的 AVX2/FMA 实现。不要给整个库加
`-march=native`，不要用 fast-math 绕过 NaN/Inf 检查，不一次开发多个 SIMD 后端。

第一版仅融合，不分块；若融合正确但未达性能目标，最多试一个固定块长
（建议 1024 个 complex<float>），以输出块为外循环、active tap 为内循环，
保持同一样点 tap 顺序与边界。两版均无收益就报告瓶颈，停止无休止调参。

## 执行顺序与反馈

1. 保留旧 build `gr-uwb/build-sic-fullpacket-discrete`；新建
   `gr-uwb/build-sic-fullpacket-fused`，使用相同 Release 编译参数。
   先核对旧库 hash 与 tap 数报告一致，不覆盖旧库。完成最小融合实现。
2. 构建受影响 target，执行下述必要验证；正确后提交代码，再正式配对计时。
3. 写一页结果报告并提交：旧/新 FIR、channel、wrapper 的 mean/P95，精度差异、
   编译器实际向量化情况，以及是否值得保留；更新入口状态。

每步结束输出：`[阶段] 完成内容；验证证据；问题；下一步`。
无需逐阶段等用户确认。任务小，默认单 agent；如用 subagent，最多一个只读检查
编译器/SIMD 选择，主 agent 写代码和跑测试，不启动多套 QA/benchmark。

## 只做必要验证

- 运行现有 `discrete_cir_reconstruct` 针对性 QA（涵盖边界/独立卷积/事务旁路），
  不重复全部历史 smoke。改了测试工具才运行其新增检查。
- 复用 `benchmark_discrete_cir_taps` 的完整输出独立 double 卷积检查；现有最大
  相对 L2 门槛 2e-5 不放宽，有限性/相减检查保持。FMA 可能改变末位，不要求逐位相等。
- 若使用 SIMD，检查不足一向量的尾部与非对齐偏移；现有 QA 未覆盖时只补一例。
- 不改算法参数，不重新跑 10 格解码率矩阵；若误差超门槛先修实现，不放宽标准。

## 性能对照：复用现有工具

工具：`gr-uwb/apps/benchmark_discrete_cir_taps.cc`。
固定 32 B、133568 样点波形、64 样点支持、CFO +50 kHz；测有效 tap
1/3/6/12/24/48/64，每组 16 次预热、128 次观测，正序/倒序各一轮。
旧/新都运行同一工具：顺序旧→新→新→旧，每次沿用工具内部两轮，合并时注明
重复调用不是独立信道样本。无需创建新 runner/verifier 框架，可用短脚本汇总 CSV。

正式计时独占 `/tmp/uwb-sic-benchmark.lock`，同机串行，其他 CPU 密集任务暂停；
使用 `env -u LD_LIBRARY_PATH`，ldd 确认各自 build 库，记录代码提交、二进制/库 hash、
编译参数及 CPU。新 run 存独立目录，不覆盖旧 run。初始化与文件输出不进入计时。

历史参考（不是本轮对照分母）：3 tap FIR 0.286 ms，12 tap 1.045 ms，
64 tap 5.358 ms。正式收益必须使用本轮旧/新配对结果。
`channel` 包含 FIR/CFO/归一化/检查，`wrapper` 还包含校验/重调制/相减/拷贝；
二者均不包含 CIR 估计与通信解码，不称为总 SIC。父子计时不重复累加。

构建与 QA 示例：

```bash
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-fullpacket-fused -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-fullpacket-fused --target benchmark_discrete_cir_taps uwb_qa_uwb_discrete_cir_reconstruct.cc -j2
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-sic-fullpacket-fused -R 'discrete_cir_reconstruct' --output-on-failure
```

交付：最小代码改动 + CSV/简短报告 + commit。若未达目标，如实给出是否有收益，
不要自动扩展到 FFT、tap 裁剪、估计器或通信解调优化。
