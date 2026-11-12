# 单 worker 测试配置修正

已修正当前原生采样率 fixture 的两个测试配置错误：

- `period=140982`，`t0=140982`：与重复包长度一致，从第二包开始，保留完整 pre-guard。
- `--sic-comm-code 9`：与 `testdata/uwb_radar/metadata.json` 的真实码一致。

入口为 `tools/performance/analyze_sc16_cost_split.py` 和
`tools/performance/run_equiripple_g3.py`。默认 SIC 策略显式设置为 `preamble`，
可用 `--sic-policy` 覆盖。单 DSP worker、逐次独占 benchmark 锁，队列容量至少
等于请求窗口数，避免短文件全速回放时只处理少数窗口。

两入口各用每配置 2 窗完成冒烟。所有包含 sensing 的配置均检出 SFD；包含 SIC 的
配置均已进入重构阶段，仍报 `template_scale_mismatch`。这两个配置修正已经生效，
但有效 Ipatov 消除尚未通过。OpenCode 应先定位该重构错误，不能用失败退出耗时
作为有效 SIC 性能。`g3` 的功能字段比较也不应预设新旧滤波器逐位相等：群时延
变化会移动检测位置，需按时间原点校正后评价。

建议 OpenCode 首先执行以下单配置功能检查（从仓库根目录运行，输出使用新目录）：

```bash
gr-uwb/build-sc16-r1r2/apps/uwb_uhd_sc16_pipeline_runner \
  --benchmark \
  --file testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin \
  --native-rate 737280000 --work-rate 998400000 \
  --period 140982 --t0 140982 --capture 56000 \
  --pre-guard 400 --post-guard 400 \
  --consumer both --dsp-workers 1 --resampler-workers 1 \
  --pool-size 32 --max-windows 30 \
  --sic-policy preamble --sic-backend ipatov --sic-comm-code 9 \
  --resampler-profile quality_minorder \
  --json-out /tmp/uwb-single-worker-fixed-20261112.json
```

前序诊断中同一窗口与码配置改为 `--sic-backend discrete`，3/3 窗返回
`committed_preamble_discrete`，可作定位后端问题的功能对照。

后续性能统计应使用逐窗 `e2e_us` 计算整体分位数，不可相加各阶段 P50。
现有 cost-split 脚本的 `worker_us` 是阶段 P50 之和，`msps` 字段还多乘了
1000，均不应作为正式整体耗时/吞吐结论；原始 JSON 可供独立统计。
本次仅修正配置及运行保障，未执行正式性能矩阵。
