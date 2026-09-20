# 测试报告：X410 双 TX random-delay 连续 Buffer（软件整改）

> 日期：2026-09-20
> 方案：[`整改方案_X410双TX_random_delay连续Buffer.md`](整改方案_X410双TX_random_delay连续Buffer.md)
> 基线 commit：`6af97a3563a1af7bf78551b19a821db84486eea3`
> 工作区：未 reset / checkout / 清理用户修改

## 结论

软件整改已落地：multi-TX 热路径每拍只发 **一个长度为 L 的 data fragment**；
uniform delay 只滑动 jammer backing 指针，TX0 的 buffer 指针、长度和样本位置
在整个 grid 内不变。`jam_delay_native` 等字段经 65/48 与 CIR estimator 透传到
`cir.jsonl` 候选 metadata。

**硬件 A/B/C/D 功能门与 soak 未做**，不得写成整改验收通过。旧报告中
random-delay `peak_tap=19` 视为待重新验收。

## 每拍 data send

`max_fragment_size >= L` 时 `tx_cmd.fragment_count == 1`。UHD backend 仍对
该 fragment 自行 packetize，并在全部数据之后发一次 zero-length EOB。
`max_fragment_size < L` 在 schedule 入口拒绝（`max_fragment_size_lt_L`）。

## 内存增量

未在构造函数按 `max_tx_samples` 预分配 4 路 dense。Window bank 在
`handle_schedule()` 物化一次，由 `Job` 的 `shared_ptr<const MultiTxWindowBank>`
持有。典型 L≈189003、D≈3686（±5 µs @ 737.28 MS/s）：

- sense dense：L × 4 B ≈ 0.72 MiB
- 其它 dense 行（jam 在 uniform 下为全零占位）：同量级
- jam backing：`(L+2D) × 4 B` ≈ 0.75 MiB

合计约 **2–3 MiB / 已武装 schedule**，远小于按 cap 预分配的 40–48 MiB。
热路径不 resize / assign / fill / copy。

## 软件测试

```text
ctest -R 'uwb_qa_uwb_(echo_multitx_layout|echo_timer.cc$|uhd_backend|pdu_rational_resampler_65|radar_cir_estimator|radar_e2e)'
→ 9/9 passed

uwb_qa_uwb_pdu_rational_resampler.cc --run_test=test_pdu_radar_metadata_whitelist,test_pdu_radar_metadata_sync_32_64_128
→ passed（全量该文件仍失败于既有吞吐阈值 test_pdu_resampler_throughput_sanity）

python3 gr-uwb/apps/test_echo_cir_jam_plan.py  → 75/75
python3 gr-uwb/apps/test_jam_app_args.py       → 15/15
python3 gr-uwb/apps/test_echo_tx_channels.py   → 10/10

ctest --test-dir gr-uwb/build  → 42/43
（唯一失败为既有环境吞吐阈值 test_pdu_resampler_throughput_sanity）
```

Fake backend 256 拍：TX0 逐样本相同、TX1 起点 = `D+jam_delay_native`、同 seed
re-arm 序列 bit-exact、静默 jammer 整行全零、`fragment_count==1`、partial send
两路同一 continuation cursor、bank 地址/capacity 不变。

## 硬件（未执行）

方案 §7/§8 的 A/B/C/D 组与 100–500 Hz soak **未跑**。通过条件仍是：

- 静默 jammer 下 sensing peak 不随 delay 跳变（禁止再出现 -4..+4 → 25/36/…）
- soak 的 `echo_late` 必须单独记账，不得写成性能通过

## 改动文件

- `gr-uwb/include/gnuradio/uwb/uwb_echo_multitx.h`（WindowBank + helpers）
- `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
- `gr-uwb/lib/uwb_realtime_echo_timer.cc`
- `gr-uwb/include/gnuradio/uwb/uwb_radar_pdu_meta.h`
- `gr-uwb/lib/uwb_radar_cir_estimator_block.cc`（透传走 kPassthrough）
- `gr-uwb/lib/qa_uwb_echo_multitx_layout.cc`（新）
- `gr-uwb/lib/qa_uwb_echo_timer.cc`
- `gr-uwb/lib/qa_uwb_pdu_rational_resampler.cc`
- `gr-uwb/lib/qa_uwb_radar_cir_estimator_block.cc`
- `gr-uwb/lib/CMakeLists.txt`
- `gr-uwb/apps/echo_cir_jam_plan.py`
- `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`
- `gr-uwb/apps/test_echo_cir_jam_plan.py`
- `gr-uwb/apps/test_jam_app_args.py`
- `gr-uwb/grc/uwb_realtime_echo_timer.block.yml`
