# 测试报告：X410 C++ 双 TX 高性能 Radar CIR（M1–M5 代码完成，Fake QA 全绿）

> 状态：**代码实现完成，Fake/QA/构建全绿；硬件 soak 未做，不得写成验收通过**
> 基线 commit：`6af97a3`（工作区有大量预先存在的未提交改动，本任务未 reset/覆盖）
> 日期：2026-09-19
> 规划：`docs/phase1/开发规划_X410双TX_C++高性能Radar_CIR.md`
> 分支：`feature/uwb-monostatic-radar`（未提交，保持工作区状态）

---

## 1. 结论

- M1（纯 C++ 多通道契约 + Fake backend）、M2（真实 UHD 多通道 fixed TX 代码）、
  M3（EchoTimer 多 TX schedule + jam app fixed align）、M4（无拷贝 per-pulse
  random delay）、M5（jammer-only scan retune）**代码完成**，全部有 Fake QA。
- 主构建通过；`ctest -R uwb_qa_uwb_(echo_timer|uhd_backend)` **4/4**；
  全量 CTest **41/42**（唯一失败是既有环境吞吐阈值，见 §5）；
  Python QA `test_echo_cir_jam_plan` **70/70**、`test_echo_tx_channels` **10/10**、
  `test_jam_app_args` **13/13**；UHD-OFF 构建 + targeted QA **3/3**。
- **未做**：M0 硬件基线冻结、M6 端到端 200/300 Hz soak 与全扫描验收（无硬件会话）。
  `continuous` 独立 jammer streamer 按规划不在本任务范围（cpp-pdu 明确拒绝）。

## 2. 改动清单（只列本任务文件；其余工作区改动为任务前已存在）

| 文件 | 内容 |
|---|---|
| `gr-uwb/include/gnuradio/uwb/uwb_echo_multitx.h` | **新增**：`kEchoMaxTxChannels=4`、`TxBurstFragment`、确定性 `JamDelayRng`（PCG-XSH-RR + Lemire 无偏映射）、零拷贝布局 planner（`prepare_multitx_geometry` / `multitx_burst_bounds` / `multitx_channel_ptr`） |
| `…/uwb_echo_burst_backend.h` | `TxCommand` 加 `tx_channel_count` + `tx_multi_fragments`（count==1 走旧 `fragments`，bit-exact）；`BurstResult` 加 `tx_channel_count` / `tx_wire_*`（旧 `tx_samples_*` 保持每通道语义）；新增 `TxAsyncCounts` + `tune_tx_channel()` / `tx_async_counts()` 虚接口（默认拒绝/零） |
| `…/uwb_uhd_backend_config.h` | 新增 `UhdTxChannelConfig` + `UhdBurstBackendConfig::tx_channels`；`effective_tx_channels()` 兼容转换；`validate_tx_channels()`（1–4 路、物理通道不重复、频率/增益有限）；dry-run plan 预留多通道字段 |
| `…/uwb_uhd_burst_backend.h/.cc` | `prepare()` 逐 TX 通道 rate 严格回读 + 单条 `channels=[sense,jam]` SC16 streamer + `get_num_channels` 校验；`issue_tx()` 用 `std::array` + `ref_vector` 零分配 send（SOB/time 仅首次、data 永不带 EOB、单次 zero-length EOB、partial 全通道同 offset 续发）；async 线程记录 channel/ticks（无 time spec 记 unmatched）；`collect_result()` 固定容量 `std::array` 归属 + 六计数器；`tune_tx_channel()` 逻辑→物理映射 |
| `…/uwb_realtime_echo_timer.h/.cc` | schedule PDU 双格式（旧 `cons(meta,s16vector)` + 新 `cons(meta,pmt_vector[…])`，§5.3 全键验证，arm 时证最坏 fragment 数 ≤64）；`MultiTxGeometry` + PRNG seed 冻结；每拍 dwell 边界仅 `tune_tx_channel(jam)` → settle 重锚（index/pulse 连续，不计 late）；`JamDelayRng` per-pulse delay；边界切分 + 固定 `d_mtxf_[64]`；先 issue_rx 再 issue_tx；burst metadata 新增 `tx_channel_count` / `tx_wire_*` / `tx_async_*` / `jam_delay_native/us` / `jam_freq_plan/actual/offset_hz` / `jam_retune_seq`；单 TX 旧键/样本不变 |
| `…/uwb_fake_burst_backend.h` | 多通道 `issue_tx`（同上校验 + per-channel stitched + wire 计数）；`tune_tx_channel` 记录顺序/回读；`tx_async_counts`（ack/unmatched 有界 + dropped）；单通道行为逐样本不变 |
| `gr-uwb/lib/uwb_uhd_dry_run.cc` | `--tx-channels "0,1"` + 每通道 gains/antennas/freqs；打印通道映射、每通道/聚合字节、最坏 fragment 数；旧 `--tx-channel` 兼容 |
| `gr-uwb/python/uwb/bindings/python_bindings.cc` | 多通道 overload（`tx_channels/tx_antennas/tx_gains/tx_freqs`）+ 旧 scalar 兼容 wrapper；新增 `realtime_echo_timer_uhd_multitx` 别名（app 的 `tx_gains_db` / `tx_freqs_hz` kwargs）；`realtime_echo_timer` 新增 `tx_channel_count` / `jam_*_last` / `jam_retunes(_failures)` / `tx_async_counts`（转 `py::dict`） |
| `gr-uwb/grc/uwb_realtime_echo_timer.block.yml` | 多通道参数（默认空 = 单通道旧路径） |
| `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py` | align 允许 `--echo-backend cpp-pdu`（删硬拒绝；`continuous` + cpp-pdu 仍明确拒绝）；`JamCppPduEcho` 启动时一次归一化 + 一次多 TX schedule PDU（`jp.build_cpp_schedule_meta`），无逐拍 loop/`tolist`/composite 重建；summary 新增 `tx_channel_count` / 每路有效长度 / 物理 L / `cpp_delay_updates` / `cpp_jam_retune(_fail)` / `cpp_tx_async`；cpp 下 `--dump-rx/--dump-sc16` 告警禁用；默认后端仍为 python（M6 通过后才切） |
| `gr-uwb/apps/echo_cir_jam_plan.py` | 纯加法（numpy-only）：`CPP_SCHEDULE_META_KEYS`、`CPP_JAM_DELAY_MODES`、`cpp_jam_delay_mode()`、`build_cpp_schedule_meta()`（浮点 us/s → native 整数 ticks 在 Python 侧换算） |
| `gr-uwb/lib/qa_uwb_uhd_backend.cc` | +5：`test_uhd_backend_multitx_config`、`test_uhd_backend_effective_single_equiv`、`test_multitx_rng_repeat_bounds`、`test_multitx_geometry_L_bounds`、`test_multitx_channel_ptr_slices` |
| `gr-uwb/lib/qa_uwb_echo_timer.cc` | +4：`test_fake_multitx_two_channel_stitch`、`test_fake_multitx_fragment_partial_flags`、`test_fake_multitx_tune_async`、`test_fake_multitx_scratch_stable` |
| `gr-uwb/apps/test_echo_cir_jam_plan.py` | +`BuildCppScheduleMetaTest`（13 例） |
| `gr-uwb/apps/test_jam_app_args.py` | `test_rejects_cpp_pdu` → `test_allows_cpp_pdu_align` |

## 3. 验证结果

```text
cmake --build gr-uwb/build -j                                   # 通过
ctest -R 'uwb_qa_uwb_(echo_timer|uhd_backend)'                  # 4/4 通过
ctest --test-dir gr-uwb/build                                   # 41/42（§5）
test_echo_cir_jam_plan.py / test_echo_tx_channels.py /
  test_jam_app_args.py                                          # 70/70、10/10、13/13
UHD-OFF（/tmp/opencode/uhd_off_final，-DENABLE_UHD_BACKEND=OFF）  # 构建通过，targeted 3/3
绑定烟测（build 树 .so，Fake backend + multitx factory 构造）     # 通过
app --dry-run --jam-enable --echo-backend cpp-pdu               # 通过（L=140982，pmt_vector[s16:281964,s16:96036]）
```

另有一组仓外一次性 C++ harness（M3 自检，未入仓）：单 TX 语义+新键、fixed 双通道逐样本布局/partial/SOB-EOB、uniform 范围/seed 可复现/sense 驻留、dwell retune 序列/settle 重锚连续 0 late、四类非法 PDU 拒绝、单元素 vector≡legacy、失败重试语义——ALL PASS，并在编写中断言反向定位 2 个真 bug（fixed 模式 span 误拒、失败 retune 虚报频率），已修复。

## 4. 修过的真 bug（实现中发现）

1. fixed 模式 `jam_delay_out_of_span` 误拒：`[-D,+D]` span 检查是 uniform 专属，曾拦在 fixed 之前（app 固定模式 sense@0/jam@delay≠0/D=0 必拒）。现仅 uniform 分支检查。
2. 失败 retune 虚报频率：失败分支仍用未生效的新目标写 metadata。现失败时回落到仍生效的旧值 + `tune_failed` 状态 + 下拍重试。
3. `apply_schedule` 空指针崩溃：multi Job 的 `payload` 为 NIL，曾无条件 `s16vector_elements`（worker 线程抛 = terminate）。现 multi 时 `d_tx_ptr_=nullptr`。
4. `UhdBurstBackend::tune()` 多通道调错通道：曾用 legacy scalar `tx_channel`。现用 frozen `tx_phys_channels[0]`（单 TX 恒等）。
5. `publish_burst` 头/cc 签名不一致 + `PubExtra` 默认实参 GCC 闭合错误：以后者（3 参、无默认值）为准统一。

## 5. 唯一失败项（非回归）

`uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值失败（`pdus/s=218` vs 400 等三阈值；容器性能型）。该文件及被测实现均未被本任务改动（`git diff --name-only` 无此二文件），属规划 §8.2 所述既有环境失败。

## 6. 未完成与硬件会话前置条件（M0/M6，明确未做）

1. **上板前必须 `sudo cmake --install gr-uwb/build`**：系统已安装的 `libgnuradio-uwb` / `uwb_python` 是 8 月旧版（无 multitx factory/新访问器）；本任务无 root，未安装。`sudo -E python3` 硬件运行时加载的是旧版，不装则 cpp-pdu 双 TX 不可用。
2. M0 同会话基线（标准单 TX 200/300 Hz 3×30 s + Python 双 TX 短测）未采。
3. M6 验收矩阵（§9：200 Hz 10 min、300 Hz 3×60 s + 10 min、1638×100 全扫描、async/CIR 完整性门）未跑。
4. jammer `continuous` 模式仍仅 Python（规划 §11 明确不做）。

## 7. 复现命令

```bash
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build -R 'uwb_qa_uwb_(echo_timer|uhd_backend)' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
python3 gr-uwb/apps/test_echo_cir_jam_plan.py
python3 gr-uwb/apps/test_echo_tx_channels.py
python3 gr-uwb/apps/test_jam_app_args.py
# UHD-OFF：
cmake -S gr-uwb -B /tmp/opencode/uhd_off_final -DENABLE_UHD_BACKEND=OFF
cmake --build /tmp/opencode/uhd_off_final -j
ctest --test-dir /tmp/opencode/uhd_off_final -R 'uwb_qa_uwb_(echo_timer|uhd_backend)' --output-on-failure
# dry-run（无硬件）：
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py --dry-run --jam-enable \
  --echo-backend cpp-pdu --output /tmp/jam_dry
# 上板前（需 root）：
sudo cmake --install gr-uwb/build
```
