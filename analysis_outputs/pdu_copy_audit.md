# PDU 雷达链大数组复制审计（M4）

> 只读审计，基于提交 `0dbfda5` + 本次 M1–M3 改动后的源码。
> 每个“复制”标注 file:line；结论只有「保留一次（PMT 所有权要求）」或
> 「可选优化（需改 API，风险自评）」两类，不含“零拷贝一定可行”。

链路：

```
UHD SC16 scratch → 输出 s16vector → PDU resampler 输入 scratch
  → core work_ 组装 → resampler 输出 scratch → 输出 c32vector
  → estimator 队列/任务 → CIR PDU / writer
```

## 1. UHD 接收写入（无复制）

`UhdBurstBackend::collect_result` 直接把 `rx_stream->recv(bufs, ...)` 收到
`BurstFragment.rx_data` 指向的地址，也就是 `UwbRealtimeEchoTimer::d_rx_buf_`
的分片（`uwb_uhd_burst_backend.cc:617-620`，分片地址在
`uwb_realtime_echo_timer.cc` `run_one_burst()` 里由 `d_rx_buf_.data()+offset*2`
给出）。**无中间缓冲复制**。

## 2. echo 发布：`d_rx_buf_` → `s16vector`

`publish_burst()` 调 `pmt::init_s16vector(2*published, d_rx_buf_.data())`
（`gr-uwb/lib/uwb_realtime_echo_timer.cc`，M2 后的 ROI 分支）。

- 复制大小：`2*published` int16（ROI 后约 130k int16 ≈ 260 KB）。
- **必须保留一次**：PMT uniform vector 拥有自己的存储，必须在 `publish_burst`
  返回后继续有效；worker 的 `d_rx_buf_` 会被下一个 burst 覆写。不能把
  `d_rx_buf_` 的指针直接塞进 PMT（悬空/数据竞争）。
- 未发现 GNU Radio/PMT 公开“把已有缓冲零拷贝包装成 uniform vector 且转移
  所有权”的 API；`pmt::init_*vector` 一律复制。

## 3. resampler 输入：SC16 → FC32 转换缓冲

`UwbPduRationalResamplerCcf65_32::handle_packet` 对 `s16vector` 输入逐点写入
`d_input_scratch_`（`uwb_pdu_rational_resampler_ccf_65_32.cc:416-431`）。

- 这既是复制也是格式转换（int16 对 → `gr_complex`）。
- **本次没有消除**：core 的 `process()` 只接受 `const gr_complex*`，所以必须
  有一块 FC32 输入缓冲；即便直接把 PMT 的 int16 数据“就地”读成 float 也不
  可行（类型/对齐不同）。
- 可选优化：给 core 增加“直接从 int16 输入指针转换+FIR”的重载，把转换融合进
  `work_` 组装，可省掉这一遍独立写缓冲。属于 M5 融合块的范畴，本次不做。

## 4. core `work_` 组装（复制）

`RationalResamplerLmCore::process` 把 `hist_ + current_ + input` `memcpy` 进
`work_`（`uwb_rational_resampler_core.h:279-296`）。

- 一-shot PDU（每包 `reset()`）时 `hist_` 全零，这次 memcpy 实际是把输入整体
  复制一遍。
- **保留**：core 的流式状态机（history/phase）与 `run_mt` 的
  `sched_win_buf_` 索引都依赖这块连续 `work_`；去掉它需要重写 core 的窗口
  寻址，收益（一遍 ~400 KB memcpy）小于回归风险。文档记录为 M5 之前的已知成本。

## 5. resampler 输出：`d_scratch_` → `c32vector`

`handle_packet` 末尾 `pmt::init_c32vector(emit_len, d_scratch_.data()+emit_off)`
（`uwb_pdu_rational_resampler_ccf_65_32.cc:693-696`）。

- 复制大小：`emit_len` c32（ROI + CaptureOnly 时约 222k ≈ 1.8 MB；FullWindow）。
- **必须保留一次**：`d_scratch_` 每包复用，PMT 必须独立拥有载荷。
- 可选优化（高风险）：让 core 直接写入“将来要发布的 PMT storage”，但 PMT 没有
  稳定的可写 uniform-vector 视图；若用自定义 blob 契约则破坏 PDU 兼容性，
  spec §M4 明确禁止。

## 6. estimator 队列/任务

`UwbRadarCirEstimator::handle_rx` 只取 `car/cdr` 的 `pmt::pmt_t` 引用并按
引用入队；worker 用 `pmt::c32vector_elements(payload)` 直接读
（`uwb_radar_cir_estimator_block.cc:522-537` 及任务入队处）。

- **无深拷贝**：PMT 引用计数保证生命周期覆盖 worker。这是链上唯一已经“零额外
  复制”的大数组。

## 7. CIR 输出

`est` 只发布 116-tap 的小 PDU；writer/UDP 再消费。量级 ~1 KB，不是瓶颈。

## 结论

| 阶段 | 复制 | 处理 |
|---|---|---|
| UHD recv → `d_rx_buf_` | 无 | — |
| `d_rx_buf_` → echo `s16vector` | 有（ROI 后 ~260 KB int16） | 保留（PMT 所有权） |
| SC16 → FC32 `d_input_scratch_` | 有（转换性质） | 保留；融合进 core 属 M5 |
| `hist+input` → core `work_` | 有（one-shot 冗余） | 保留（状态机依赖） |
| `d_scratch_` → resampler `c32vector` | 有（~1.8 MB） | 保留（PMT 所有权） |
| estimator 入队 | 无 | — |

**本阶段不声称“消除 PDU 复制”**：可证明必须保留的复制占多数，唯一已知可省的是
§3/§4（每个 PDU 各一遍 ~400 KB / ~1.8 MB 的内存往返），但都需要改 core/融合块
API，收益相对 200 Hz 的主瓶颈（每 burst UHD ~5 ms）有限，留待 M5。
