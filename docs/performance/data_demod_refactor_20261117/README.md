# 解调与调度分层重构验证数据

verification.json 包含：

- 基线 2a4e244 与重构版的二进制 SHA-256；
- +4/-12/-14 dB 三份固定窗口夹具哈希、逐行差异数、去掉耗时后的规范 JSON 哈希；
- 21 个搬移函数体与起点逐字一致的审计清单；
- 三档动态 live ABBA 的原始每次值、汇总、启动信息、CPU 和输入哈希。

工具为 tools/rx/low_snr_eval/compare_packets.py 和 ab.py；
生成夹具见该目录 README。数据属于软件/合成文件回放，不是上板证据。
ABBA 阶段耗时来自单独 packets 模式，不能与 summary 均值按包配对。

基线固定窗口 decoder：从 2a4e244 导出旧 uwb_demod_core.h 和
 tools/rx/decode_dw3000_packets.cc，以旧头优先、原 build 生成头为配置，
链接基线 libuwb_core.a，使用 -O3 -mavx2 -mfma -ffp-contract=off -std=c++17。
当前 decoder：cmake --build gr-uwb/build-native --target uwb_dw3000_decode。

详见[开发报告](../解调与调度分层重构开发报告_2026-11-17.md)。
