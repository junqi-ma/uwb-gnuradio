# Full-packet：payload 滤波与前缀缓存对照

用户授权在当前实验分支测试“只对 payload 做 CIR/分数延迟滤波”是否加速。
实现保留完整包逐路径相减；缓存的是固定前缀的分数相位基函数，时延、增益与 CFO
仍由当前窗口的冻结模型逐次施加。没有缓存接收干扰或注入真值。

模式：`--reconstruction full|payload_cached`，默认 full；RX 两组均使用已验的 fast 路径。
新 build 为 `gr-uwb/build-sic-fullpacket-payload-cache`，旧 build 和结果保持原样。

边界：当前卷积编码连续输入 PHR19+RS/payload，物理 PHR21 的最后两个符号依赖 payload。
因此动态区域包括这两个 PHR 符号、80 样本保护区和 payload，而非在名义 payload 起点硬切。
固定输入 key 到92544，缓存 FIR 输出 `[0,92528)`；名义payload从94656开始。
33tap FIR 仍读取完整波形及跨边界halo，tap顺序和float32舍入点不变。

缓存 key 为实际再调制前缀的精确字节，包含归一化与 PHR 影响；变化时清空有效相位标志。
缓存按需填充，但32行内存事先分配；冷缓存/新相位/失效重算均计入channel服务时间。
新增phase/path/CFO子计时、hit/miss、key变化和实际/参考FIR输出样本计数；子项不二次加总。

验证预算：缓存3组契约QA、原整包core QA、Python verifier QA；候选固定10格各16窗，
与历史fast输出逐位对照。仅对32B两个代表格作两轮full→cached→cached→full串行计算计时，
每格每组每轮512次、32次热身，复用16输入，独占benchmark锁。P99仅作样本尾部观察。
本轮回答计算开销问题，不重复历史A/B全矩阵或200窗/s队列验收。
