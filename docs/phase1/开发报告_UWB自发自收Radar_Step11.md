# 开发报告：UWB 自发自收 Radar Step 11（真实 UHD backend，无目标物验收）

> 日期：2026-09-05
> 分支：`feature/uwb-monostatic-radar`
> 依据：[开发计划_UWB自发自收Radar.md](开发计划_UWB自发自收Radar.md) Step 11（§14）、
> [开发需求_UWB自发自收Radar.md](开发需求_UWB自发自收Radar.md) §6.2/§8、
> [开发报告_UWB自发自收Radar_Step9-10.md](开发报告_UWB自发自收Radar_Step9-10.md)
> 状态：**Step 11 完成（无目标物验收）**：`UhdBurstBackend` 已实现并隔离编译；
> 本机（无 UHD 头文件）OFF 路径全量 CTest **30/30** 通过；UHD ON 路径在本机用
> **真实 UHD 4.1.0.5 头文件**（apt 下 载 dev 包解包到临时前缀，不动系统）完成
> **编译+链接+QA 验证**：4/4 uhd 测试通过，含**有 UHD 无 USRP → `prepare()`
> 明确报 `device unavailable: ...`、不崩溃不卡死**的实机运行时验证。
> **未做任何 X410/OTA/电缆/实测射频验证**（Step 12 范围）。未改
> [`../../开发状态.md`](../../开发状态.md)。未提交。

---

## 1. 复验结论

| 项目 | 结论 |
|---|---|
| 主构建（本机，无 UHD 头，AUTO→禁用） | 全量构建 0 error；CTest **30/30**（原 27 + 新 3） |
| `git diff --check` | 干净 |
| UHD ON（真实 UHD 4.1 头，/tmp 前缀） | 编译+链接通过（backend 符号入 lib，动态链到系统 `libuhd.so.4.1.0`）；`ctest -R uhd` **4/4** |
| 有 UHD 无 USRP | `prepare()` 返回 false + `"device unavailable: ..."`，3/3 设备 QA 通过，实测 ~1.6 s |
| 强制 `ENABLE_UHD_BACKEND=ON` 无 UHD | configure 阶段 `FATAL_ERROR`，报错含处置建议（不静默降级） |
| 越界/无关改动 | 未改 Step 12 专属 `apps/x410_uwb_echo_radar.py`、未改 Step 12 报告；共享 CMake 仅 `gr-uwb/lib/CMakeLists.txt` 与 `gr-uwb/include/gnuradio/uwb/CMakeLists.txt` 两个文件 |
| 未宣称 | X410、OTA、电缆、任何 timed-burst 实射频行为均未验证 |

---

## 2. 实现前调查（本机源码，语义记录）

1. **gr-uhd TX 侧**（`/home/junqima/workspace/gnuradio/gr-uhd/lib/usrp_sink_impl.cc`）：
   stream tags `tx_time`/`tx_sob`/`tx_eob` → `tx_metadata_t{has_time_spec, time_spec,
   start_of_burst, end_of_burst}`；SOB/time-spec 只在首包，EOB 终结突发；
   `stop()`/`start()` 用 **0 长度 EOB/SOB 包** 发送元数据（本实现复用该机制终结突发）。
   `async_event_loop()` 以 100 ms 轮询 `recv_async_msg`，按位测试
   `EVENT_CODE_BURST_ACK/UNDERFLOW/SEQ_ERROR/TIME_ERROR/UNDERFLOW_IN_PACKET/SEQ_ERROR_IN_BURST`。
2. **gr-uhd RX 侧**（`usrp_source_impl.cc`）：timed burst =
   `issue_stream_cmd(STREAM_MODE_NUM_SAMPS_AND_DONE, stream_now=false,
   time_spec=…)`；完成点是阻塞 `recv()`；`ERROR_CODE_TIMEOUT` 视为可重试，
   `ERROR_CODE_OVERFLOW` 计数并继续，其他错误码上浮 warn——本项目按雷达契约
   把 overflow/late/broken-chain 全部升级为"本脉冲作废"的 per-burst status。
   首 packet 的 `md.time_spec` 是首样点设备时刻（→ `rx_time_ticks`）。
3. **gr-radar `usrp_echotimer_cc`**：仅参考 timed-burst 几何（需求 §3 已记录），
   不照搬其 work 内建线程与单次 send/recv。
4. **multi_usrp/streamer API 名以 gr-uhd 3.10（系统版本，链 UHD 4.1）调用为准**：
   `set_tx_rate(rate,chan)/get_tx_rate/set_rx_rate/get_rx_rate`、
   `set_{tx,rx}_{freq,gain,antenna}`、`set_{clock,time}_source`、
   `get_{tx,rx}_stream(stream_args)`（otw `sc16`/cpu `s16`）、
   `rx_streamer::recv(ref_vector<void*>, n, md, timeout, one_packet)`、
   `tx_streamer::send(ref_vector<const void*>, n, md, timeout)`、
   `recv_async_msg(md, timeout)`、`get_time_now(mboard)`。
5. **UHD 4.1 头布局与本机 ABI 实测**（决定性发现）：4.1 用合并头
   `uhd/stream.hpp` + `uhd/types/metadata.hpp`（无 `uhd/rx_streamer.hpp`/`rx_info.hpp`，
   那是 4.2+ 布局）；**RX error code 非 0..4 连续**：`NONE=0x0, TIMEOUT=0x1,
   LATE_COMMAND=0x2, BROKEN_CHAIN=0x4, OVERFLOW=0x8, ALIGNMENT=0xc,
   BAD_PACKET=0xf`；TX async 位掩码 `BURST_ACK=0x1, UNDERFLOW=0x2,
   SEQ_ERROR=0x4, TIME_ERROR=0x8, UNDERFLOW_IN_PACKET=0x10,
   SEQ_ERROR_IN_BURST=0x20, USER_PAYLOAD=0x40`。设计中的"UHD-free 映射 +
   UHD TU 里 static_assert 交叉核对"机制**实际拦截了一次错误假设**
   （初版按连续 0..4 写的映射在真实头下无法通过 static_assert），随后按实测
   ABI 修正——这正是计划 §2"先查本机源码"要防的错误。

---

## 3. 分层与文件

```text
UwbRealtimeEchoTimer (gr::block, 0 流端口; Step 10, 未改)
  └─ 单 worker：EchoGrid 设备时刻调度 core → IRadioBurstBackend
       ├─ FakeBurstBackend    (Step 10, 未改; CI)
       └─ UhdBurstBackend     (Step 11, 本次新增; UWB_HAVE_UHD 才编译)
```

| 动作 | 文件 | 内容 |
|---|---|---|
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_uhd_backend_config.h` | **UHD-free** 契约面：`UhdBurstBackendConfig`+校验、严格 rate 读回策略、rx/async 错误码映射（真实 ABI 常量）、tick↔(full,frac) 转换、fragment flag 契约校验、device-unavailable 消息契约、dry-run plan 构建/打印 |
| 新增 | `gr-uwb/include/gnuradio/uwb/uwb_uhd_burst_backend.h` | 类声明（pimpl，全部 UHD 类型留在 .cc）；未定义 `UWB_HAVE_UHD` 时 `#error`；`UWB_API` 导出 |
| 新增 | `gr-uwb/lib/uwb_uhd_burst_backend.cc` | UHD 实现（仅 ON 时编译；ABI static_assert 交叉核对） |
| 新增 | `gr-uwb/lib/qa_uwb_uhd_backend.cc` | 9 case UHD-free QA |
| 新增 | `gr-uwb/lib/qa_uwb_uhd_backend_device.cc` | 3 case UHD-gated 运行时 QA（无 USRP 场景） |
| 新增 | `gr-uwb/lib/uwb_uhd_dry_run.cc` | dry-run CLI：不含任何 UHD 头、不链 UHD/`gnuradio-uwb` → **结构上不可能开设备** |
| 修改 | `gr-uwb/lib/CMakeLists.txt` | `ENABLE_UHD_BACKEND`(CACHE STRING AUTO/ON/OFF)+`find_package(UHD CONFIG)`+pkg-config 回退、条件源/链接/`UWB_HAVE_UHD` PUBLIC 定义、UHD-gated QA、dry-run 可执行+2 个 CTest |
| 修改 | `gr-uwb/include/gnuradio/uwb/CMakeLists.txt` | 两个新头的安装行 |

Step 10 代码（EchoTimer/core/fake backend/QA）**零改动**；Step 12 专属文件零改动。

---

## 4. UhdBurstBackend 语义（与 Step 10 报告遗留项的对应）

- **同一 `multi_usrp`，两条 SC16 streamer**：`prepare()` 创建 device（缺设备 →
  `device unavailable: <what>`，绝不跨接口抛异常）、可选 clock/time source、
  TX/RX 通道 rate **set 后读回严格校验**（`rel_tol=1e-9`，737280000.0 精确
  double；任何 UHD 静默 coercion → prepare 失败，报 requested/readback）、
  可选 freq/gain/antenna（负 gain = 不设置，显式 0 dB 是真设置）、
  `get_tx_stream/get_rx_stream("s16","sc16")` 单通道。
- **RX 先于 TX**：`issue_rx()` 非阻塞下发
  `NUM_SAMPS_AND_DONE, stream_now=false, time_spec=rx_ticks`（ticks→time 部件
  由确定性转换完成），**先于** `issue_tx()` 被调度 worker 调用（顺序由
  Step 10 的 worker 保证）。
- **partial send**：按 plan 的 fragment 逐个 send，`send()` 返回不足即原地
  re-issue 余量（不带 SOB/time）；**time_spec+SOB 只在首个 send 调用**；
  **EOB 不放在数据调用上**——全部数据被接受后用**专用 0 长度 EOB send**
  终结突发（gr-uhd `stop()` 机制），杜绝"部分 send 却声明了 EOB"的破链。
  Step 10 报告 §5.1 遗留的 EOB 尾包问题以此机制关闭。
- **partial recv / collect**：`collect_result(index, wait_ms)` 阻塞
  `recv()` 循环，每次超时 = min(recv_timeout, 剩余预算)；部分收包计数
  `rx_reissues`；收满即完成。**错误码映射**：`TIMEOUT`→继续等（受总预算
  约束，超时由 worker 合成 Timeout）、`OVERFLOW`→Overflow（脉冲作废，
  需求 §8.3）、`LATE_COMMAND`→LateCommand、`BROKEN_CHAIN`→BrokenChain、
  alignment/bad_packet/未知→BackendError；**故障态完成自动 `abort_rx()`
  （STOP_CONTINUOUS）** 保证下一突发干净。首 packet `time_spec`→
  `rx_time_ticks`。
- **TX async 映射**：独立 async 线程（对齐 gr-uhd async_event_loop，100 ms
  轮询）记录事件；`collect_result` 按 **event.time_spec ↔ 本突发 tx_ticks**
  精确匹配归因：`TIME_ERROR`→LateCommand、underflow/seq→BackendError、
  `BURST_ACK`/`USER_PAYLOAD`→正常、未知高位→BackendError（不静默丢弃）。
  归因不受"事件晚于 RX 窗结束"影响（按 ticks 匹配，不按在飞窗口猜测）。
- **停止/错误安全**：`request_stop()` 置粘性 flag（未 prepared 也成立，
  对齐 fake 语义）并停 async 线程（≤100 ms join）；collect 循环每轮检查
  flag → StopDuringIo + abort；`prepare()` 清 flag（restart 安全，块的
  start() 先 request_stop 再 prepare）。
- **设备时刻**：`device_time_ticks()` = `get_time_now()` 经确定性转换；
  未 prepared/异常 → 0（worker 视为过期、跳槽，不崩溃）。转换在 GPS 量级
  时刻有 ±~64 tick 的 double 间距误差（文档化；不影响 PRI 栅格整数几何，
  t0 由同一转换产生、time_spec 由同一转换消费，突发执行不受影响）。

---

## 5. dry-run CLI（不打开设备的纯契约输出）

```text
$ gr-uwb/build/lib/uwb_uhd_burst_dry_run \
    --device-args addr=192.168.10.2 --rate 737280000 --tx-channel 0 \
    --rx-channel 1 --clock-source internal --time-source internal \
    --pri-num 3686400 --pri-den 1 --pre-guard-ticks 1475 \
    --tx-samples 140982 --rx-samples 55591 --max-fragment-size 65536 \
    --t0-ticks 36864000 --schedule-index 0
```

输出（节选，实测）：

```text
uwb_uhd_burst_backend DRY-RUN (no device opened)
device_args              = addr=192.168.10.2
sample_rate_hz           = 737280000
rate_readback            = strict rel_tol=1.0000000000000001e-09 (silent coercion rejected)
otw_format               = sc16 / cpu_format = s16
pri_ticks                = 3686400/1
pre_guard_ticks          = 1475 (2.0005967881944446e-06 s)
tx_ticks / rx_ticks      = 36864000 / 36862525 (rx BEFORE tx)
tx_samples / rx_samples  = 140982 / 55591
tx_fragments / rx_fragments = 3 / 1 (max_fragment_size=65536)
tx_bytes / rx_bytes      = 563928 / 222364 (sc16 = 4 B/sample)
command_order            = issue_rx(NUM_SAMPS_AND_DONE, timed) BEFORE issue_tx(send loop)
tx_metadata              = SOB+time_spec on first send call; EOB on a dedicated zero-length call after all data
rx_error_map             = 0x0:none 0x1:timeout 0x2:late_command 0x4:broken_chain 0x8:overflow else:backend_error
tx_async_map             = time_error:late_command underflow/seq_error:backend_error burst_ack:normal
dry-run complete: no device was opened
```

默认值即项目雷达契约：TX=完整 64-SYNC native 包 140982（testdata metadata）、
RX≈75.4 µs=55591（需求 §5.3/§8.2）、t0=0.05 s arm delay、PRI=5 ms、
pre_guard=2 µs=1475 tick。分数 PRI（`--pri-num 73728 --pri-den 10`）输出
精确有理数且 fragment 重规划正确（实测 35/14 fragments）。非法参数 exit 2
（如 `--rate 0`：`invalid contract: sample_rate_hz must be > 0 and finite`）。

---

## 6. QA 与结果

### `qa_uwb_uhd_backend`（9 case，UHD-free，两配置均运行）

| case | 覆盖 |
|---|---|
| config_validation | rate 0/负/inf/NaN、tol 0/≥1e-3、send/recv timeout 越界、freq 负/NaN、gain NaN/>120；**负 gain=不设置 sentinel、显式 0 dB 合法** |
| rate_strict | 精确相等/亚容差 fp 噪声通过；1 kHz 偏移、0.75×、容差 0 拒绝 |
| error_mapping | 真实 ABI 常量断言（0x0/0x1/0x2/0x4/0x8；async 0x1..0x40）；RX 全表 + 未知码/负码→BackendError；TX 单 bit/组合位（TIME|ACK→late）、未知高位→BackendError、USER_PAYLOAD 非错误 |
| device_unavailable_message | 前缀契约 `device unavailable: ...` |
| tick_conversions | ticks→(full,frac)→ticks 往返精确（含 1e12 大值）；负 ticks/frac 越界/rate≤0 显式失败 |
| fragment_flags | 单 fragment（TIME\|SOB\|EOB）、3-fragment 合法；缺 EOB/中间 SOB/零长/null 拒绝 |
| dry_run_plan | 默认契约逐字段精确断言（pri 3686400/1、rx_ticks=t0−1475、fragments 3/1、bytes、pri_s≈0.005）；分数 PRI 有理数；t0<pre_guard、零长度、fragment 上限、非法 config 全部拒绝 |
| dry_run_print | 关键行在 stdout（rate 737280000、coercion rejected、rx BEFORE tx、NUM_SAMPS_AND_DONE、错误映射行、`dry-run complete: no device was opened`） |
| fake_adapter_geometry | **EchoTimer+FakeBurstBackend 用 UHD 形状参数**（TX 140982/RX 55591、fragment 65536、io chunk 30000 强制 TX 5 次/RX 2 次 partial re-issue）：2 burst 全部 `partial_handled`、rx 计数精确、拼接 RX 逐点正确、0 失败——真实 backend 参数空间被注入式契约接受（计划 §14"参数校验、错误映射继续用 fake adapter QA"） |

### `qa_uwb_uhd_backend_device`（3 case，**仅 UHD ON 编译**，无 USRP 机器实测）

| case | 覆盖 |
|---|---|
| unprepared_is_inert | 未 prepared：issue_rx/issue_tx→BackendError、collect→true+BackendError、abort_rx 无副作用、device_time_ticks=0、request_stop→stop_requested=true |
| constructor_validation | 非法 config 构造抛 `invalid_argument` |
| device_unavailable | `uhd::device::find` 无设备时：`prepare()` 实跑 → false + `"device unavailable: ..."` 前缀；**实测本机 ~1.6 s 返回，无崩溃无卡死**（有 USRP 的机器自动 skip，硬件机器安全） |

结果（UHD ON 树实测）：`Running 3 test cases... *** No errors detected`。

---

## 7. CMake UHD ON/OFF 双路径验证

```bash
# (a) 默认 AUTO，本机无 UHD 头 → 禁用 + 明确状态行，主库/Phase A/fake QA 照常
$ rm -rf /tmp/opencode/uwb-auto && cmake -S gr-uwb -B /tmp/opencode/uwb-auto
-- UHD not found: UhdBurstBackend disabled; the library, EchoTimer fake-backend QA and dry-run build without UHD.
-- UHD burst backend (ENABLE_UHD_BACKEND=AUTO): 0

# (b) 强制 ON 无 UHD → configure 明确失败（不静默降级、不崩溃）
$ cmake -S gr-uwb -B /tmp/opencode/uwb-on -DENABLE_UHD_BACKEND=ON
CMake Error: ENABLE_UHD_BACKEND=ON but UHD was not found (install libuhd-dev /
UHD CMake or pkg-config support). Use ENABLE_UHD_BACKEND=AUTO to skip the backend instead.

# (c) 显式 OFF → 同 AUTO 结果，UWB_HAVE_UHD=0
$ cmake -S gr-uwb -B /tmp/opencode/uwb-off -DENABLE_UHD_BACKEND=OFF
-- UHD burst backend (ENABLE_UHD_BACKEND=OFF): 0
```

**真实 UHD ON 编译验证**（本机无 UHD 头文件；不装系统包，apt 仅 `download` +
`dpkg -x` 解到 `/tmp` 前缀，`libuhd.so` 软链到系统运行时库）：

```bash
$ cd /tmp/opencode/uhd-dev && apt-get download libuhd-dev   # 4.1.0.5-3
$ dpkg -x libuhd-dev_4.1.0.5-3_amd64.deb /tmp/opencode/uhd-prefix
$ ln -sf /lib/x86_64-linux-gnu/libuhd.so.4.1.0 /tmp/opencode/uhd-prefix/usr/lib/libuhd.so
$ export UHD_DIR=/tmp/opencode/uhd-prefix/usr
$ cmake -S gr-uwb -B /tmp/opencode/uwb-uhd-real -DENABLE_UHD_BACKEND=ON \
        -DUHD_DIR=/tmp/opencode/uhd-prefix/usr/lib/x86_64-linux-gnu/cmake/uhd
-- UHD burst backend (ENABLE_UHD_BACKEND=ON): 1
$ cmake --build /tmp/opencode/uwb-uhd-real -j
# 0 error；nm -D libgnuradio-uwb.so | grep -c UhdBurstBackend → 7；
# ldd → /lib/x86_64-linux-gnu/libuhd.so.4.1.0（真实 UHD 4.1 运行时）
```

该编译过程**用 static_assert 实测了真实 UHD 4.1 ABI**，拦截并纠正了设计阶段
对 RX error code 的错误假设（见 §2.5）。UHD-ON 树 `ctest -R uhd` 4/4 通过；
其余 6 个失败（demod_core/realtime_demodulator/rational_resampler 系/
tone_canceller）为**out-of-tree 相对路径 testdata 加载问题**——同位置
UHD-OFF 对照树（`/tmp/opencode/uwb-off-ctrl`）失败完全同类（对照树另有
1 个已知时序敏感 flake），证明与 UHD ON/OFF 无关；in-tree 规范构建
30/30 全绿。

---

## 8. 精确命令与结果

```bash
$ cmake --build gr-uwb/build -j$(nproc)          # 全量，0 error（OFF/AUTO 路径）
$ ctest --test-dir gr-uwb/build --output-on-failure
   → 100% tests passed, 0 tests failed out of 30   # 原 27 + 新 3，复跑通过
$ ctest --test-dir gr-uwb/build -R 'uhd' --output-on-failure
   → 3/3（qa 9 case + dry-run 正例 + dry-run 负例 WILL_FAIL）
$ git diff --check                                # exit 0
$ ctest --test-dir /tmp/opencode/uwb-uhd-real -R 'uhd' --output-on-failure
   → 4/4（含 UHD-gated 设备 QA；UHD 4.1 banner 可见）
```

注意：主构建目录曾因 `option()` 缓存 BOOL 出现过 `ENABLE_UHD_BACKEND=OFF`
的旧值，已改为 CACHE STRING（AUTO/ON/OFF）并显式 reconfigure 为 AUTO；
主构建在全部代码变更后复跑 30/30。

---

## 9. 未做 / 未验证 / 遗留

1. **未做任何硬件验证**：X410 smoke/单帧 timed burst/多帧/200 pulse/s/
   校准/电缆/OTA 全部属于 Step 12，本报告不作任何声明。
2. UhdBurstBackend 仅在本机完成 **编译+链接+无设备运行时** 验证；真实
   timed send/recv 数据通路、rate 读回对真机 coercion 的拒绝、overflow/
   late 的真机表现——全部未验证，待 Step 12 分层验证。
3. TX async 事件按 `time_spec↔tx_ticks` 精确匹配归因；与任何突发都不匹配的
   残留事件保留在有界队列（128，最旧丢弃，计数上报字段存在但未接 status
   事件）——真机如果有"事件迟到且 ticks 不匹配"的场景需要 Step 12 观察后决定
   是否升级为显式 status。
4. `device_time_ticks()` 在 GPS 量级全秒下的 double 间距误差（±~64 tick）
   已文档化；若 Step 12 实测显示影响 late 判定，可改用 (full,frac)→tick 的
   高精度整数路径。
5. TSan/ASan 未在本环境完成（系统库未插桩，同 Step 10 遗留）。
6. UhdBurstBackend 的 GRC YAML / Python 绑定仍未提供（按 Step 10 R2 决议，
   待 backend 工厂与 Step 12 应用定型后一并补齐，避免再次暴露不可用接口）。
7. 未提交、未 push；未改 `开发状态.md`；未改 Step 12 专属 app/报告。
