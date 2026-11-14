# 检测失败定位（2026-11-12）

输入 6,000,000 个 SC16 样点；与 testdata/uwb_radar/tx_737p28.sc16（140982 点）对照，首、次两个完整周期逐字节相同。原生参考的元数据 testdata/uwb_radar/metadata.json 标明 code_index=9、64 SYNC、SFD=4z2。默认加载 reference_preamble.bin 与 work-grid 参考首个 SYNC 的归一化相关约 0.9999999；模板未丢失。

原计时配置 period=100000、t0=0、pre_guard=400，SIC ROI 固定 [478,606)。它与真实重复周期 140982 不一致。首窗还在文件边界被裁到 start=0，实际包头经 quality Kaiser 重采样后在输出 28 点，ROI 却仍假定约 542 点。原始首窗 dump 的 ROI 内最大单 SYNC 归一化相关仅 0.000781，真正峰在 28、1044、2060……，相关约 0.985。

诊断均使用当前 runner、单 worker、consumer=both、独占基准锁；采用 preamble 策略以排除整包负载因素。

| 对照 | SIC 状态 | sensing SFD |
|---|---|---|
| 原窗口与码配置，6 窗 | 5 below_detect_threshold，1 fit_residual_exceeded | 0/6 |
| period=140982、t0=140982，仍 code=10，6 窗 | 6 fit_residual_exceeded | 6/6 |
| 同上 + code=9，Ipatov，6 窗 | 6 template_scale_mismatch | 6/6 |
| 同上 + quality_eq32，Ipatov，6 窗 | 6 template_scale_mismatch | 6/6 |
| 同上 + discrete 后端，3 窗 | 3 committed_preamble_discrete | 成功消除后另行评估 |

修正几何后包头在质量 Kaiser 输出约 570 点，位于 ROI 内（542+28 群时延），ROI 最大单 SYNC 相关约 0.991。默认 comm_code=10 与真实 code=9 不符；Ipatov 内部单位脉冲参考按 comm_code 建立，故即使外部 shaped 模板能检测，后续拟合仍失败。

template_scale_mismatch 是检测/拟合通过后的重构拒绝；uwb_discrete_cir_reconstruct.cc 的该状态来自波形与重构参考的尺度/形状一致性检查。它是另一个待定位的 Ipatov 集成问题，此轮没有修复或关闭。不能把其当作检测阈值问题，也不能据此降低安全门槛。discrete 成功对照支持输入本身可被当前链路检测和消除。

此前 0.84–1.00 ms/窗数字仅为失败/旁路路径耗时；不能当作有效 SIC 性能。原脚本与生产代码均未修改。逐窗 JSON、日志和两个首窗 IQ dump 在本目录；本诊断样本量用于定位行为，不作为正式性能结果。
