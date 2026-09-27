# 真实 UWB Packet 随机 Preamble Overlap 实现方案

日期：2026-09-21  
实施对象：OpenCode  
目标应用：`gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`，重点支持
`--echo-backend cpp-pdu`、X410 CG600/DPDK 双 TX。

## 1. 目标与关键语义

新增一种离散随机碰撞模式：每个 sensing pulse 都发送一个**完整的 jammer UWB
packet**，随机选择 jammer packet 相对 sensing packet 的起始时刻，使两个 packet 的
preamble 恰好重叠 `L` 个 SYNC repetition。

本方案中的 `L` 只表示 **jammer preamble 与 sensing preamble 的交集长度**，不是把
干扰门控为 L 个 repetition：

- jammer packet 不得截短，仍包含其配置所生成的 preamble、SFD、PHR、PSDU、STS；
- jammer 提前到达（lead）时，其 SFD/PHR/PSDU 可能继续干扰 sensing preamble；
- jammer 延后到达（lag）时，其 packet 尾部可能继续干扰 sensing 的后续字段；
- 这正是两个真实 UWB packet 异步碰撞的行为。若只发 L 个 repetition，那是门控
  干扰实验，不属于本方案。

现有模式必须保持兼容：

- fixed `--jam-delay-us`；
- uniform native-sample jitter `--jam-delay-random-us`；
- frequency offset/scan；
- single-TX、Python backend 和未启用 overlap 的命令行行为。

## 2. 数学定义

设：

- sensing preamble repetition 数为 `Ns`；
- jammer preamble repetition 数为 `Nj`；
- 每个 repetition 在 998.4 MS/s work grid 上固定为 `SPS=1016` samples；
- jammer packet 起点相对 sensing packet 起点为 `d`，`d<0` 表示 jammer lead；
- `1 <= L <= min(Ns, Nj)`。

两种碰撞方向定义如下：

```text
lead（干扰 sensing preamble 的前部）：
    d_work = -(Nj - L) * SPS

lag（干扰 sensing preamble 的后部）：
    d_work = +(Ns - L) * SPS
```

当 `Ns=Nj=128` 时：

```text
L=128: d=0，完全对齐
L=64 : lead=-64*Trep 或 lag=+64*Trep
L=1  : lead=-127*Trep 或 lag=+127*Trep
Trep = 1016 / 998.4e6 ≈ 1.017628205 us
```

`L=Ns=Nj` 时 lead/lag 都映射为 aligned；仍要固定 RNG 消耗规则，不能因该特殊值
改变后续 pulse 的随机序列。

### 2.1 work offset 到 native offset

不能在 C++ 热路径用浮点微秒换算，也不能简单累计四舍五入后的 `Trep_native`。
Python 控制路径应为每个候选 `(side,L)` 预计算 native delay lookup table。边界映射应
使用与 `scipy.signal.resample_poly` 长度和现有 RX geometry 一致的整数有理映射：

```text
B(q) = ceil(q * SPS * tx_interp / tx_decim)

lead: d_native = -B(Nj-L)
lag : d_native = +B(Ns-L)
```

CG600 为 `tx_interp/tx_decim=48/65`，CG400 为 `32/65`。OpenCode 必须用脉冲/切片
QA 验证 `B(q)` 与实际 `profile.tx_native()` 中 repetition 边界一致；若实测表明
resample_poly 的相位原点要求不同的整数律，应修正统一 helper，而不是在多个位置
分别加补偿常数。

## 3. 用户接口

建议新增：

```text
--jam-overlap-mode off|random-reps       默认 off
--jam-overlap-min-reps N                 默认 1
--jam-overlap-max-reps N                 默认 min(Ns,Nj)
--jam-overlap-side lead|lag|random       默认 random
--jam-overlap-seed U32                   默认沿用/显式生成 jam delay seed
```

示例：128-SYNC sensing 与 128-SYNC 完整 jammer packet，每拍随机 1..128 个
preamble repetition overlap，方向随机：

```bash
sudo -E taskset -c 2-19 python3 -u \
  gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py \
  --use-dpdk --mgmt-addr 192.168.20.133 --args addr=192.168.10.2 \
  --echo-backend cpp-pdu --jam-enable --jam-waveform packet \
  --preamble-length 128 --jam-preamble-length 128 \
  --jam-overlap-mode random-reps \
  --jam-overlap-min-reps 1 --jam-overlap-max-reps 128 \
  --jam-overlap-side random --jam-overlap-seed 20260921 \
  --max-fragment-size 524288 --rate-hz 200 --duration-s 60 \
  --output /tmp/x410_random_overlap_200hz_60s
```

参数规则必须 fail-fast：

1. overlap 模式必须使用 `--jam-enable --jam-waveform packet`；不得静默把 preamble
   waveform 当作真实 packet。
2. `min/max` 必须满足 `1 <= min <= max <= min(Ns,Nj)`。
3. overlap 模式不得同时使用 `--jam-delay-random-us`；也不得叠加非零
   `--jam-delay-us`。以后若需要额外传播时延，应单独设计 `base_delay`，本次不要混合。
4. `--jam-overlap-side lead/lag` 只生成对应方向；`random` 每拍随机方向。
5. seed 必须写入 summary，固定 seed、相同成功/失败序列时必须 bit-exact 重现。
6. overlap 模式首期只要求 `cpp-pdu`。Python backend 应明确报“不支持”，不要产生
   语义不同的近似结果。

## 4. 随机分布与确定性

受控实验需要 `L` 均匀，而不是 native delay 均匀：

```text
L ~ UniformInteger[min_reps, max_reps]
side ~ Bernoulli(lead, lag)       # 仅 side=random
```

每个 attempted burst 固定消费两次 PCG 输出：第一次选择 L，第二次选择 side。即使
side 固定或 `L` 导致 aligned，也消费第二次输出，确保序列不因配置分支或特殊值漂移。
沿用 `JamDelayRng` 的 PCG-XSH-RR 和无偏整数映射，不使用 `std::uniform_*`，避免不同
STL 实现产生不同序列。

定义 attempted burst 与现有 uniform delay 一致：进入实时 worker、准备该 pulse 时
即抽样；之后即使 UHD 返回失败也不回滚 RNG。必须加 QA 锁定这一契约。

## 5. Scheduler 与实时实现

`UwbRealtimeEchoTimer` 是 message/PDU 调度块，不是 `sync_block`、普通 stream block
或 `tagged_stream_block`。Schedule PDU 在控制路径完成验证、波形持有和 Buffer
物化；实时 worker 只负责选择本拍参数、设置固定数组中的指针并调用 backend。

热路径硬约束：

- 不分配/释放内存；
- 不构造或裁剪 NumPy/PMT waveform；
- 不重采样；
- 不清零或复制整条 TX row；
- 保持每拍一个 multi-channel TX fragment；
- TX0 的实际 sensing 起点、样本内容、长度和 `sense_tx_ticks` 不随 L/side 改变；
- 每拍只允许固定次数 RNG、lookup table 索引和 jammer backing 指针移动。

### 5.1 扩展 delay mode

不要把 overlap 冒充现有 `Uniform`。建议：

```cpp
enum class JamDelayMode : uint8_t {
    Fixed = 0,
    Uniform = 1,
    PreambleOverlap = 2,
};
```

Schedule metadata 增加：

```text
jam_overlap_min_reps
jam_overlap_max_reps
jam_overlap_side_mode          # lead / lag / random
jam_overlap_seed
sense_preamble_reps
jam_preamble_reps
jam_overlap_lead_delays_native # 按 L-min 索引的 s64 vector
jam_overlap_lag_delays_native  # 按 L-min 索引的 s64 vector
```

若现有 PMT 没有方便的 `s64vector`，可用严格长度检查的 PMT vector/integer；不要把
负 delay 塞进 `u64vector` 后依赖隐式转换。Schedule handler 必须拷贝/持有表，worker
不得访问临时 Python 内存。

### 5.2 非对称连续窗口几何

完整 packet 的 lead/lag 会真实扩展物理发送时间；这部分传输量无法在不裁掉 jammer
packet 的前提下消除。但不应再额外用对称 `D=max(abs(delay))` 过度 padding。

令全部候选 native delay 的边界为：

```text
dmin = min(delay_table) <= 0
dmax = max(delay_table) >= 0
P = -dmin                         # sense 在物理 burst 中的固定 offset
R = dmax - dmin                   # jammer pointer 可滑动范围
Ltx = max(P + sense_len,
          P + dmax + jam_len)     # 固定物理 burst 长度
```

推广当前 `MultiTxWindowBank`：

```text
jam_backing_length = Ltx + R
jam_backing_wave_begin = P + dmax
jam_window_begin(delay) = dmax - delay
```

窗口中 jammer 的相对起点为：

```text
jam_backing_wave_begin - jam_window_begin = P + delay
```

它对 `[dmin,dmax]` 都成立，并在 `dmin=-D,dmax=+D` 时严格退化为当前
`wave_begin=2D`、`window_begin=D-delay`、`backing=L+2D`。因此应把 geometry helper
一般化，而不是另写一套未经测试的 overlap Buffer。

`lead` 或 `lag` 单方向运行时，非对称几何会明显小于双方向 symmetric Buffer，应在
dry-run 中打印并建议资源受限实验按方向分两轮执行。

### 5.3 资源与性能预期

以 CG600、`Ns=Nj=128`、完整 sensing/jammer packet 约 189k native samples 为例，
随机 `L=1..128` 且 side=random 时最大绝对 offset 约 95.3k native samples：

```text
Ltx 约 380k samples/channel
jam backing 约 570k samples
window bank 约 5--6 MiB（以实际 waveform 长度为准）
200 Hz、双通道 SC16 主机 TX 约 0.61 GB/s
```

内存不是主要瓶颈，物理发送样本数和 DPDK/UHD 调度负载才是。当前默认
`--max-fragment-size 262144` 不足；应用必须在 arm 前准确计算 Ltx 并给出类似错误：

```text
random overlap needs max_fragment_size >= 379xxx; use 524288
```

不得自动拆成多个 begin/end fragment，因为此前已证明分片会改变 X410 双 TX 时序并
引入 underflow/late。若 backend/设备不允许一个 fragment 覆盖 Ltx，应直接拒绝配置。

dry-run 和启动日志必须输出：`dmin/dmax`、最大 lead/lag us、`Ltx`、backing samples、
window bank MiB、每 pulse TX bytes、目标速率下 TX GB/s、fragment count（必须为 1）。

## 6. 每拍 metadata 与输出格式

每个回波/CIR PDU 至少携带：

```text
jam_delay_mode = "preamble_overlap"
jam_delay_native
jam_delay_us
jam_overlap_reps
jam_overlap_side = "lead" | "lag" | "aligned"
sense_preamble_reps
jam_preamble_reps
sense_tx_ticks
jam_tx_ticks
jam_overlap_seed
```

这些字段必须通过：EchoTimer → RX result → PDU resampler → CIR estimator → CIR writer。
更新 `uwb_radar_pdu_meta.h` 的 allowlist，并为 average/repetitions/packed repetition 三条
路径都加 QA。`cir.jsonl` 的 pulse 公共字段必须保存 overlap 信息，不能让 128 条
repetition 各自重复写一份。

summary 增加：

```text
jam_overlap_mode/min/max/side/seed
jam_overlap_histogram            # L -> pulse count
jam_overlap_side_histogram
jam_delay_native_min/max
cpp_delay_updates                # overlap 每次 attempted burst 更新一次
```

### 6.1 UDP

UCR4 不含 delay/overlap，无法仅凭实时 UDP 数据还原实验条件。新增向后兼容的 UCR5，
receiver 继续解析 UCR4/UCR3/UCR2/UCR1：

```text
UCR5 = UCR4 fields
     + jam_delay_native i32
     + jam_overlap_reps u16
     + sense_preamble_reps u16
     + jam_preamble_reps u16
     + overlap_side u8            # 0 none, 1 lead, 2 lag, 3 aligned
     + flags/reserved u8
```

新增 12 bytes 后 header 为 64 bytes；116 taps SC16 时 datagram 为 528 bytes，仍远低于
常规 MTU。base 非 jam 模式发送 side=none、overlap=0、delay=0。更新
`cir_udp_recv.py`、raw dump JSONL 与单元测试；不得复用 UCR4 magic 却悄悄改变 header。

## 7. 修改范围

至少检查并按职责修改：

- `gr-uwb/apps/echo_cir_jam_plan.py`
  - 参数纯函数、work→native overlap lookup、非对称几何和资源报告。
- `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`
  - CLI/校验、完整 packet 强制、schedule metadata、summary/histogram。
- `gr-uwb/include/gnuradio/uwb/uwb_echo_multitx.h`
  - 新 mode、非对称 geometry/backing、固定 RNG 抽样 helper。
- `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
- `gr-uwb/lib/uwb_realtime_echo_timer.cc`
  - schedule 解析/持有、arm-time bank、per-pulse lookup、metadata。
- `gr-uwb/include/gnuradio/uwb/uwb_radar_pdu_meta.h`
  - overlap metadata 透传 allowlist。
- `gr-uwb/apps/x410_cg400_hrp_echo_cir.py`
- `gr-uwb/apps/cir_udp_recv.py`
  - UCR5 sender/receiver，旧版本只读兼容。
- Python bindings/docstrings（若增加 block accessor/统计计数器）。
- 对应 C++ QA、Python QA、GRC/API 文档和 `开发状态.md`。

先搜索本机 GNU Radio/UHD 源码中多通道 timed burst 和 message handler 的相似实现；
不得改变当前 block 类型，也不得把随机逻辑移到 Python 每拍控制循环。

## 8. 测试要求

### 8.1 Python 纯函数 QA

1. 对 CG400/CG600、`Ns/Nj` 的所有标准组合、所有合法 L、lead/lag 穷举：
   - work-grid preamble 区间交集严格为 L repetitions；
   - delay table 单调、端点和符号正确；
   - L=full 映射 delay=0/aligned；
   - 非对称 Ltx/backing 所有加法做溢出检查。
2. 用 impulse/标记 repetition waveform 经 `profile.tx_native()` 验证 lookup 的 native
   边界误差不超过已定义的一个 polyphase 边界样本，且约定在文档和测试中固定。
3. CLI 冲突、范围、错误 backend/waveform、fragment 不足全部 fail-fast。

### 8.2 C++ layout/RNG QA

1. 固定 seed 序列 golden test；相同 seed bit-exact，不同 seed 不同。
2. 每拍固定消费两次 RNG，包括 fixed side、aligned、backend failure。
3. 至少 100k draw 检查 L/side 均在范围内；统计检查只能用宽容阈值，不能做脆弱的
   精确概率断言。
4. 对 delay table 每项检查：
   - TX0 指针、内容、count、sense sample 位置逐样本不变；
   - jammer 是完整原 packet，位于 `P+delay`，首尾没有被裁掉；
   - one fragment，指针范围始终在 backing 内；
   - `sense_tx_ticks` 恒定，`jam_tx_ticks-sense_tx_ticks=delay`。
5. fake backend 注入 send/RX failure，验证 RNG 契约和 metadata 不错位。
6. 在 realtime worker 的测试 allocator/hook 下证明无分配；至少保留现有性能 benchmark
   并比较改造前 fixed/uniform 模式无显著回归。

### 8.3 端到端与 MATLAB 对照

1. fake backend 运行至少 10k pulses，按每拍 metadata 重新计算期望 L/side/delay，
   逐拍一致；pulse ID 不重不漏。
2. average、individual repetitions、packed repetitions 三种 CIR 输出都保留 metadata。
3. writer binary record 数量/顺序不变；JSONL histogram 与逐拍字段守恒。
4. UCR5 pack/unpack bit-exact；UCR4/3/2/1 legacy parse 不回归；batch fanout 的 128
   datagram 都带同一 pulse overlap metadata。
5. 导出代表性的 lead/lag、L={1,32,64,127,128} SC16 双 TX 波形；MATLAB 脚本按
   work/native rate 计算 packet/preamble 区间，逐样本或逐 packet 验证实际交集。

每次有意义修改后运行构建和 targeted tests；完成前运行 Release build、相关 CTest、
Python tests、`git diff --check`。

## 9. X410 验收

分三步，不能直接宣称硬件通过：

1. 20 pulses smoke：检查双 TX、UCR5、metadata、无 UHD underflow/time error。
2. 200 Hz、60 s、固定 seed、`L=1..128`、side=random、UDP 开启：
   - 12000/12000 pulse；
   - `res_drop=0`、`est_drop=0`、CIR fail=0；
   - writer/UDP 应为 `12000*128` records；
   - overlap histogram 总数=12000，side histogram 总数=12000；
   - `tx_fragment_count=1`；
   - UHD underflow/time/seq error=0；
   - 单独报告 `echo_late`，不得用 estimator 无 drop 掩盖 late。
3. 若 60 s 通过，再做至少 10 min soak；300 Hz 另作性能档，不与功能正确性混写。

同时做三组 A/B：fixed aligned、random lead-only、random both-sides。记录 Ltx、实际
TX GB/s、CPU、estimator mean/max、UDP drain 时间，确认性能变化来自完整 packet 的
物理发送窗口增长，而不是 worker 分配或复制回归。

## 10. 完成标准与禁止事项

完成标准：完整 jammer packet、离散 L 均匀随机、lead/lag 可控、固定 seed 可重现、
每拍 metadata/JSONL/UDP 可追溯，TX0 时序不随碰撞模式变化，热路径无分配且保持单
fragment，并通过上述 QA 与 X410 分级验收。

禁止事项：

- 不得通过截短 jammer preamble 冒充真实 packet collision；
- 不得用 `--jam-freq-offset` 表达时间 overlap；
- 不得在 worker 中生成、裁剪、重采样或复制 packet；
- 不得恢复多 fragment begin/end 分片；
- 不得只记录 seed 而不记录每拍实际 L/side/delay；
- 不得增加 queue 或忽略 `echo_late` 来宣称性能通过；
- 不得为节省 Buffer 裁掉处于 sensing packet 之前/之后的 jammer packet 部分。完整包
  碰撞的物理时间跨度是真实成本，只能用非对称几何避免额外 padding。
