# 合并说明：长窗落盘 / 离线 65/48 → X410 实时解调 main

> 日期：2026-08-19  
> 本机分支：`feature/native-long-dump-offline-resample`  
> 本机分叉点：`6c137b9`（`origin/main` / `master`：`fix: anchor CFO and CIR timing from SFD tail`）  
> 对端：X410 电脑上**更新后的 `main`**（实时解调、不落盘已通过）  
> 主线入口：[`../../开发状态.md`](../../开发状态.md)

在 **X410 那台**把本分支并进它的新 `main`。不要在本机用旧 `6c137b9` 去覆盖那边已经通过的实时解调。

本分支持仓：`git@github.com:junqi-ma/uwb-gnuradio.git`。若尚未 push，先在本机：

```text
git push -u origin feature/native-long-dump-offline-resample
```

## 1. 两边各保留什么

| 侧 | 必须保住 | 不要丢掉的约束 |
|---|---|---|
| X410 `main` | 实时解调闭环（不落盘已通过）及其后对 demod / extractor / x410 脚本的修复 | 5 ms 槽内只对 **204.1 µs** 做 65/48 |
| 本分支 | 落盘与解调几何拆开；`UwbPduWindowCrop`；`uwb_offline_postprocess_dump` | 590 µs 原生窗**禁止**进实时 FIR |

本分支**不替代**实时解调，只补三件事：

1. 热路径允许 737.28 SC16 长窗落盘（300/190/100 µs），同时 65/48 仍只吃 crop 后的 10/190/4.1 µs。
2. 采集结束后 C++ 按窗去单音 + 65/48 → 998.4 **SC16**（`capture_998p4.iq`）。
3. `offline_qm35_auto_lock.py --write-sc16` 已按上述接线；本机 x410 脚本的 `--identity demod` **仍未接** 65/48 + writer。

旧 `--write-sc16` 曾把同一条 590 µs PDU 送进 65/48，handler ~7.4 ms，X410 overflow。合并后若再出现这种接法，视为回退。

## 2. 在 X410 电脑上的合并步骤

```text
cd <uwb-gnuradio>
git fetch origin
git checkout main
git pull --ff-only origin main          # 保住已通过的实时解调
git merge --no-ff origin/feature/native-long-dump-offline-resample
```

若该 feature 还没在 `origin` 上，用 U 盘 / 另一 remote 的同名分支代替 `origin/feature/...`。

冲突时 **以 X410 main 的实时解调接线为底**，再按 §3 把 crop / dump 几何嵌进去。不要用本分支整文件覆盖他们已改过的 `x410_auto_scheduled_capture.py` 或解调器。

合完后在 `gr-uwb/build` 重新 cmake + 编译（本分支新增 app、header、GRC、bindings）：

```text
cd gr-uwb/build
cmake ..
cmake --build . -j"$(nproc)"
ctest --output-on-failure
```

至少确认新增用例在：`uwb_qa_uwb_pdu_window_crop`、`uwb_qa_uwb_tone_canceller`、`uwb_qa_uwb_scheduled_dump_io`。实时解调相关旧测试也必须仍绿。

## 3. 冲突怎么判

### 3.1 热路径契约（不可协商）

```text
extractor 锁定后
  packet ──► UwbPacketWriter          590 µs 原生 SC16 → capture.iq
  packet ──► UwbPduWindowCrop         按 predicted_start 切 10/190/4.1
                └──► PDU 65/48        quality_minorder，只吃短窗
                       └──► RealtimeDemod
采集结束（不占 UHD）
  uwb_offline_postprocess_dump DIR    去单音 + 65/48 → capture_998p4.iq
```

- `--write-sc16` / 打开 Writer **只开写盘**，不得把 extractor 的解调几何改成 590 µs。
- `CaptureOnly` 只裁输出，不算少算力，**不能**代替 crop。
- 默认不覆盖 `capture.iq`。998.4 是派生文件，SC16 + 按窗 `iq_scale`。
- 实时 5 ms 路径不要开离线 `--workers` 线程池，也不要给每窗 FIR 再套 `nworkers`。

### 3.2 按文件

| 文件 | 本分支做了什么 | 对端若也改了 |
|---|---|---|
| `testdata/offline_qm35_auto_lock.py` | dump/demod 几何拆分、crop、自动后处理 | 保留双方意图：dump 长、FIR 短、写 `demod_results.jsonl` |
| `gr-uwb/apps/x410_auto_scheduled_capture.py` | 只加了 dump-pre/post 和「demod 未接线」注释 | **保留 X410 的 `--identity demod` 实现**，再插入 crop，见 §4 |
| `gr-uwb/python/uwb/bindings/python_bindings.cc` | 增加 `bind_pdu_window_crop` | 两边 bind 都留下 |
| `gr-uwb/{apps,lib,include,grc}/CMakeLists.txt` | 新 app / QA / yml | 两边 `add_*` 都留下 |
| `开发状态.md` / `docs/README.md` | 本分支进度与计时 | 手写合并：X410 实时结论 + 本分支落盘/离线结论都保留 |
| `uwb_defaults.h`、extractor、demod、65/48 | 本分支几乎未改算法 | 以 X410 main 为准 |
| 下列**本分支新文件** | 直接接纳 | 一般无冲突 |

新文件（应原样进入 main）：

```text
gr-uwb/include/gnuradio/uwb/uwb_window_crop_core.h
gr-uwb/include/gnuradio/uwb/uwb_pdu_window_crop.h
gr-uwb/include/gnuradio/uwb/uwb_tone_canceller.h
gr-uwb/include/gnuradio/uwb/uwb_scheduled_dump_io.h
gr-uwb/lib/uwb_pdu_window_crop.cc
gr-uwb/lib/qa_uwb_pdu_window_crop.cc
gr-uwb/lib/qa_uwb_tone_canceller.cc
gr-uwb/lib/qa_uwb_scheduled_dump_io.cc
gr-uwb/grc/uwb_pdu_window_crop.block.yml
gr-uwb/apps/uwb_offline_postprocess_dump.cc
docs/phase1/开发需求_原生率长窗落盘与离线65_48去单音.md
docs/phase1/合并说明_原生率长窗落盘与X410实时解调main.md   # 本文
```

## 4. 若 X410 main 已经接上 `--identity demod`

本机脚本里 demod 仍未接线。对端若已经是

```text
ext.packet → 65/48 → demod
```

合并后必须改成：

```text
ext.packet → writer（仅当要落盘；590 µs）
ext.packet → UwbPduWindowCrop(7373 / 140083 / 3023) → 65/48 → demod
```

extractor 在打开 Writer 时用 dump 几何（221184 / 140083 / 73728），**不要**再把 FIR 输入换成长窗。crop 更新 `window_start` / pre/cap/post / `sample_count`，**保持** `predicted_start`。acquisition 窗 crop 直通。

对照实现：`testdata/offline_qm35_auto_lock.py`。

采集 0 drop 之后再调：

```text
uwb_offline_postprocess_dump DIR --tone-rf-hz 6256.640e6
```

默认 `--out-format sc16`，写 `DIR/capture_998p4.iq` + `capture_998p4.jsonl`。工具失败不要删 raw，脚本用退出码 4 区分。

## 5. 和在线 65/48 的对齐（避免合完误判）

- 每一帧 jsonl 的 `predicted_start_sample`（998.4）与在线 PDU 65/48 用同一 `map(p)`，是同一绝对时刻。
- 默认离线产物是**整段 590 µs** 再升采样，窗内 QM35 起点约在第 30 万点，不是在线短窗的 ~9985。
- 默认还做按窗去单音和 SC16 量化；IQ **不是**在线 CF32 PDU 的逐样本拷贝。
- 要对着在线 FIR 逐点比：`--emit qm35 --skip-notch --out-format cf32`。

## 6. 合并后验收（仍不是 soak）

1. **不落盘**再跑一遍 X410 上已经通过的实时解调。这条退了就先别开 dump。
2. 本机/文件源：`offline_qm35_auto_lock.py --write-sc16 DIR`  
   期望：锁定后 scheduled crop 约 150479 点；FIR mean ~1.7 ms；writer 0 drop；`capture.iq` 与源 bit-exact；自动写出 `capture_998p4.iq`。
3. 再开 X410 live dump 时看 overflow：单帧 65/48 必须仍是短窗。若 handler 又回到 ~7 ms，就是长窗进了 FIR。
4. 不得把本次合并写成方案 §18 硬件盲捕获 / 60 s / 10 min soak 通过。

## 7. 本分支提交（供对端核对）

| 提交 | 内容 |
|---|---|
| `b725f38` | 需求规划 |
| `f3a23d0` | crop + 几何拆分 + 离线工具 + 自动调用 |
| 本提交 | 离线按窗线程池、去单音递推、998.4 默认 SC16、本文 |

离线工具：`--workers N`（0=auto，封顶 8）。不要在实时路径用。
