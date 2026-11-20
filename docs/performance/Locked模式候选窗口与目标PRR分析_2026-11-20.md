# Locked 模式候选窗口与目标 PRR 分析（2026-11-20）

本报告分析一次 DW3000 1 ms 连续发送、X410 `--sync locked`、2 worker 的
live 运行，解释为什么 `SUMMARY/FINAL` 只有 87.1%，以及多出来的候选窗口
从哪里来。数据来自用户终端输出，不是逐包日志；原始关键行保存在
[data_locked_candidates_20261120](data_locked_candidates_20261120/README.md)。

> 一句话结论：1 ms 周期对应约 1000 包/s。目标包成功率约 100%；
> 87.1% 是“所有解调尝试”的成功率，被约 2700 个未关联上的 autocorr
> 候选窗口拉低。候选窗口不是额外发包，而是前端 seed 没和目标 slot
> 对上号后仍然被送去解调。

---

## 1. 数据来源

```sh
env -u LD_LIBRARY_PATH gr-uwb/build-native-only/apps/uwb_dw3000_live \
  --args addr=192.168.10.2 --period-ms 1 --workers 2 \
  --print summary --summary 1000 --sync locked
```

| 项目 | 值 |
| --- | --- |
| 设备 | X410，channel 3 / RX1，6489.6 MHz，gain 60 |
| 接收时长 | 10,721,165,312 / 737,280,000 ≈ **14.54 s** |
| 发送周期 | `period_samples=737,279` ≈ 1 ms |
| 理论包数 | 约 14,534（≈ 1000 包/s） |
| 配置 | `eq16`、fused on、ondemand on、interference on、lock_fcs on |
| 接收状态 | locked，`period_ppm≈-1.06`，无 gap/source error/queue drop |

注意：`--period-ms 1` 是 **1000 包/s**，不是 100 包/s。若实际目标是
100 包/s，发送周期应配置为 10 ms。

---

## 2. 两个口径：目标 PRR 与尝试成功率

| 口径 | 数值 |
| --- | ---: |
| 锁定后的预期目标 slot | 14,534 |
| 目标 slot FCS 成功（`LOCK_STATS fcs_ok`） | 14,534 |
| 目标包成功率 | **约 100%**（`not_detected=2`、`fcs_failed=1`、`identity_rejects=2`） |
| 总提交窗口（`completed`） | 17,234 |
| 总 FCS（`FINAL fcs`） | 15,003（87.1%） |
| 多出的未知候选窗口 | 约 2,699 |
| 候选成功 | 469（17.4%） |
| 候选失败 | 约 2,230（82.6%） |

`FINAL fcs=15,003` 比目标 slot 多 469，正好是候选窗口里成功解码的数量。
这些成功窗口很可能是同一目标包的重复解码；否则应被计为干扰，但日志中
`cand_interference=0`、`interference_events=0`。

因此：

- **不能用 `FINAL fcs/completed=87.1%` 当作目标包 PRR。**
- 目标侧只有约 2～3 个异常，约 99.98%～100%。

---

## 3. 失败分布

按全部 SUMMARY（含最后一个 partial batch）汇总，共 2,231 次失败：

| 状态 | 数量 | 占全部失败 |
| --- | ---: | ---: |
| `TIMING_FAILED` | 1,827 | 81.9% |
| `SFD_FAILED` | 402 | 18.0% |
| `PAYLOAD_FAILED` | 1 | 0.04% |
| `INCOMPLETE_WINDOW` | 1 | 0.04% |

- 失败几乎全部是 timing/SFD，而不是 payload/FCS。
- `full_retry=0`、`out_of_grid=0`、`queue_full=0`。
- 目标侧 payload 失败只有 1 个，说明 fused payload 链路本身健康。

---

## 4. 候选窗口从哪里来

### 4.1 前端 seed 数量正常

```text
acq_seeds      = 14,920   ≈ 1.03 个/包
candidates     = 14,912
cand_target    = 12,220
cand_unknown   =  2,692
cand_admitted  =  2,699
```

前端延迟自相关在连续模式下**一直在运行**，不是只在 acquire 阶段运行。
锁定后它仍然每个包产生一个 seed；正常情况下 seed 会和锁定栅格匹配，
然后被丢弃，不再解调。

### 4.2 锁定后的关联规则

`TargetLockCore::classify_candidate()` 的规则近似：

```text
k      = round((seed - t0) / period)
offset = seed - predicted(k)
near   = |offset| <= target_association_window

if near 且该 slot 已提交或已成功 -> TargetAssociated（丢弃）
否则                              -> Unknown（仍然提交解调）
```

本次 `target_association_window` 默认等于初始 `pre_guard`：

```text
36,864 native samples / 737.28 MHz = 50 µs
```

所以只要 seed 与预测包起点相差超过 50 µs，或者该 slot 的 target plan
还没写入 lock 表，就会被判成 `Unknown`。

### 4.3 Unknown 仍然会切一个完整窗口

每个 unknown seed 都会由 `submit_candidate_window()` 切一个：

```text
count      = 450,000 samples  ≈ 610 µs
window_start = seed - 8,192   ≈ seed 前 11 µs
```

而锁定目标窗口是：

```text
8,192 + 329,161 + 8,192 = 345,545 samples
```

候选窗口比目标窗口大约多 104,455 samples。候选窗口用 acquire 配置解调
（`target_slot=false` 时走 `acq` profile，timing search margin 131,072），
但如果 seed 本身不是包起点、窗口只提前 11 µs，窗口很可能已经切掉前导
开头，或者根本没有 UWB 包，最终就是 `TIMING_FAILED` / `SFD_FAILED`。

这也解释了“这些窗口里像没包”：

- seed 偏得太多 → 窗口没有完整前导 → timing 失败；
- seed 恰好还套住包 → 重复解出（469 个候选成功）；
- 非 UWB 干扰触发 seed → 窗口里没有可解调的 UWB 包。

### 4.4 为什么 acquire 时少、locked 时多

此前同一设备的 `--sync acquire` 15 s 运行：

```text
cand_target = 14,471
cand_unknown =    529   (约 3.5%)
```

本次 locked：

```text
cand_target = 12,220
cand_unknown =  2,692   (约 18.1%)
```

seed 总数仍然约一包一个，所以不是空口包变多，而是 **seed→target 的关联率
明显下降**。可能原因：

1. 锁定后 `pre_guard` 从 36,864 缩到 8,192，候选窗口只提前 11 µs；
2. 候选窗口 450k 与目标窗口 345k 的提交顺序/时机不同；
3. seed 来自 192k 样点（260 µs）宽相关平台，位置本身有几十 µs 抖动；
4. `--sync locked` 下目标 origin 的更新方式与 acquire 不同；
5. 外部非 UWB 干扰（见第 6 节）。

现有汇总日志只能看到 `cand_unknown=2692`，不能区分“offset 超窗”和
“slot plan 尚未提交”。要定位需要在 `classify_candidate()` 前打印
`offset_from_target`、`near_target`、该 slot 是否已提交。

---

## 5. 批次现象

| 完成数 | batch FCS | TIMING | SFD | 备注 |
| ---: | ---: | ---: | ---: | --- |
| 1,000 | 79.1% | 165 | 43 | 启动/候选收敛 |
| 2,000 | 85.0% | 124 | 26 | |
| 3,000 | 92.8% | 61 | 11 | 最好 |
| 4,000 | 90.9% | 80 | 11 | |
| 5,000 | 91.2% | 74 | 14 | |
| 6,000 | 88.9% | 91 | 20 | |
| 7,000 | 86.3% | 115 | 22 | |
| 8,000 | 89.9% | 83 | 18 | |
| 9,000 | 88.5% | 92 | 23 | |
| 10,000 | 89.6% | 87 | 17 | |
| 11,000 | 88.7% | 92 | 21 | |
| 12,000 | 80.9% | 154 | 37 | 候选突增 |
| 13,000 | 82.3% | 146 | 31 | 候选突增 |
| 14,000 | 86.9% | 108 | 22 | payload 失败 1 |
| 15,000 | 88.5% | 93 | 22 | |
| 16,000 | 87.1% | 105 | 24 | |
| 17,000 | 84.2% | 124 | 34 | |
| 17,234 | 83.3% | 33 | 6 | 最后 partial batch |

- 第一个 batch 偏低是锁定/候选收敛期，不代表稳态；
- 12k–13k 的凹陷来自候选窗口突增，不是目标包丢失；
- 目标 `fcs_ok` 始终跟得上 `expected_slots`。

---

## 6. 外部干扰假设：华硕路由器（待验证）

用户提到现场有华硕路由器。这个假设值得单独列出，但**当前日志不能证明**。

### 6.1 频段重叠

- UWB channel 5 中心频率 6489.6 MHz；
- Wi-Fi 6E/7 的 6 GHz 频段约 5925–7125 MHz，包含 6489.6 MHz；
- 支持 6 GHz 的华硕路由器（Wi-Fi 6E/7）如果近距离工作，可能落在
  接收带宽内，抬高噪声底或产生非 UWB 的相关峰。

### 6.2 现有计数器为什么看不出来

- `cand_interference=0`：只有“确认是不同 UWB code”的候选才计为干扰；
- `interference_events=0`：只有 UWB 身份不匹配才计事件；
- Wi-Fi OFDM 不是 UWB 包，不会通过 UWB SFD/timing，因此不会进入这两个计数器；
- 它更可能表现为：`acq_seeds` 偏多、`cand_unknown` 偏多、`TIMING_FAILED`
  偏多，这正是本次看到的形态，但本次也可能主要由 seed 关联失败造成。

### 6.3 建议的 A/B 验证

保持其它参数完全相同，每组跑 30 s，记录
`acq_seeds / cand_unknown / timing_failed / target fcs_ok`：

| 组 | 路由器 | `--interference` |
| --- | --- | --- |
| A | 开 | on |
| B | 关 | on |
| C | 开 | off |
| D | 关 | off |

判读：

- B 相对 A 的 `cand_unknown`/`TIMING_FAILED` 明显下降 → 路由器有贡献；
- C/D 的 `completed≈expected_slots`、FCS 接近 100% → 候选窗口主要是
  unknown seed 提交，不是目标丢包；
- 若 B 与 A 差异很小，则主因仍是 seed→slot 关联逻辑。

如果条件允许，再用频谱仪或 USRP 看 6 GHz 频段是否有持续 Wi-Fi 能量，
比只看解调计数更直接。

---

## 7. 验证与改进建议

1. **先跑 `--interference off`**（5–10 s）：
   lock 建立后前端 seed 会被直接忽略（`autocorr_ignored`），
   `completed` 应回到 `expected_slots` 附近，FCS 应接近 100%。
   这能确认 2,700 个候选窗口只是多余提交。
2. **短时 `--print packets`**（2–3 s）：
   比较 `path=acquire` 成功包和 `path=target` 的 `psdu/time_ms`，
   确认 469 个候选成功是否为重复解码。
3. **增加诊断**：
   在候选分类处记录 `offset_from_target`、`near_target`、
   slot 是否已有 plan/结果，拆分 2,692 个 unknown 的真实原因。
4. **减少候选**：
   - lock 稳定后不提交 `Unknown` 候选，或只用于干扰检测、不送完整解调；
   - 调整 `target_association_window`（当前 50 µs）或按最近 slot 关联；
   - 如果 SNR 足够，缩小 `--acq-span` 让 seed 更尖锐。
5. **报告口径**：
   SUMMARY 最好分开 target FCS 和 candidate FCS；否则
   `fcs/completed` 会随候选数量波动，不能直接当 PRR。

---

## 8. 局限

- 只有一次 14.5 s 运行的汇总日志，没有逐包日志和 seed offset；
- `fcs_ok=14,534` 与 `not_detected=2`、`fcs_failed=1` 存在计数器重叠，
  唯一目标包成功率只能估计在 99.98%～100%；
- 华硕路由器假设未验证，没有频谱或 Wi-Fi 开关对照；
- 本报告不涉及 TWR，也未做 MATLAB 逐样点验证。

---

## 9. 复现

```sh
# 本次运行
uwb_dw3000_live --args addr=192.168.10.2 --period-ms 1 --workers 2 \
  --sync locked --print summary --summary 1000

# 候选抑制验证
uwb_dw3000_live --args addr=192.168.10.2 --period-ms 1 --workers 2 \
  --sync locked --interference off --print summary --summary 1000

# 逐包对比（短时）
uwb_dw3000_live --args addr=192.168.10.2 --period-ms 1 --workers 2 \
  --sync locked --print packets --summary 1000
```

数据与解析：
[data_locked_candidates_20261120](data_locked_candidates_20261120/README.md)。
