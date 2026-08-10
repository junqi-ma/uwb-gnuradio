# UWB GNU Radio stream rate — performance report

Host: WSL2 Ubuntu, 12th Gen Intel Core i7-12700 (20 threads), GNU Radio 3.10, CF32  
Date: 2026-08-08  
Scheme: `GROK_性能分析测试改进方案.md`  
Context: `性能瓶颈分析_门限与Detector合路.md`  
Commits: `6246758` (work-chunk stats + max_noutput), `3558fc3` (layered benchmark)

## 1. Executive summary

The previous ~200–264 MS/s “ceiling” was **not** a fundamental PatternSource or SEARCH limit. It was caused by **incorrect buffer configuration** (`set_max_output_buffer(4M)` does not enlarge GR 3.10 buffers) so work chunks stayed ~4k samples.

With the correct API:

```cpp
src->set_max_output_buffer(0, -1);
src->set_min_output_buffer(0, buffer_items);
tb->set_max_noutput_items(max_noutput_items);
// and raise UwbDetector block max_noutput beyond the old 64k self-cap
```

layered GR rates at 500M samples (median of 5) reach:

| Mode | Default buffer | Best (1M items) |
|------|---------------:|----------------:|
| source-null | **236 MS/s** | **4447 MS/s** |
| source-search | **216 MS/s** | **3479 MS/s** |
| detector-sparse (1 pkt/s) | **212 MS/s** | **3468 MS/s** |

**1 GS/s is met** on this host for sparse and SEARCH-only CF32 GR paths when min buffer ≥ ~128k and max noutput is raised accordingly. Best knee is **512k–1M items**; 4M is worse.

## 2. Answers to scheme §11 questions

### Q1. Default vs enlarged work chunks?

| Config | src mean noutput (null) | det mean noutput (search/sparse) |
|--------|------------------------:|---------------------------------:|
| default | **~4096** | **~4080–4090** (hist all ≤8k) |
| 32k min+max | ~16383 | ~16300 |
| 128k | ~65531 | ~48k–52k |
| 512k | ~262055 | ~245k–250k |
| 1M | ~524109 | ~503k–509k |
| 4M buffer / 4M nout | ~2.09M | ~2.0M |

Default effective work size is **~4k**, not 8192 request folklore alone (source often grants 4096). Correct min buffer makes mean chunk track roughly half of configured max (double-mapped buffer write space).

### Q2. Layer throughputs and incremental cost (500M, median 5)

**Default (old-equivalent plumbing):**

| Layer | MS/s med | Δ vs previous |
|-------|--------:|---------------|
| source-null | 236.1 | — |
| source-search | 215.9 | −8.6% |
| detector-sparse | 212.1 | −1.8% vs search |

**Best chunk (1M):**

| Layer | MS/s med | Δ vs previous |
|-------|--------:|---------------|
| source-null | 4447.0 | — |
| source-search | 3478.5 | **−21.8%** (SEARCH + pre_ring + det thread) |
| detector-sparse | 3468.4 | **≈ search** (worker free at 1 pkt/s) |

At large chunks the residual cost is SEARCH/pre_ring (~22% of null), **not** region/worker/message.

### Q3. PatternSource vs vector_source (100M, buffer 512k, 5 runs)

| Topology | pattern med | vector med |
|----------|------------:|-----------:|
| source-null | ~3698 MS/s | ~1269 MS/s |
| source-search | ~3042 MS/s | ~1233 MS/s |

**PatternSource is faster** here (streaming memset/memcpy vs reading an 800 MiB prebuilt CF32 vector). PatternSource generation is **not** the default-buffer bottleneck; large in-memory vectors can be *worse* (cache / TLB).

### Q4. Best buffer knee?

**512k–1M items** with matching `max_noutput_items`.

- 128k already >2 GS/s for sparse  
- 512k → ~3.3 GS/s sparse; 1M → ~3.5 GS/s sparse  
- 4M **regresses** to ~1.7–2.0 GS/s (likely cache / memory bandwidth / scheduling)

Pick **1M** for max rate tests; **512k** is a good latency/memory tradeoff.

### Q5. perf hotspots?

`perf` **not installed** on this WSL image (`perf: command not found`). Attribution uses wall/CPU% + work-chunk stats + layered A/B instead.

### Q6. Sparse vs 200 pkt/s bottleneck same?

**No.**

| Conceptual pkt/s | gap samples | med MS/s (500M, 1M buf, 5 runs) | det | drop |
|-----------------:|------------:|--------------------------------:|----:|-----:|
| 1 | 998143552 | ~3557 | 1 | 0 |
| 10 | 99743552 | ~3210 | 5 | 0 |
| 100 | 9743552 | ~2552 | 50 | 0 |
| 200 | 4743552 | ~1603 | 100 | 0 |

- **1 pkt/s:** limited by full-rate source + SEARCH/pre_ring + GR buffer path.  
- **200 pkt/s:** still ≥1.5 GS/s with 0 drop, but rate falls as region/worker/PDU load grows — worker path becomes visible while still not blocking 1 GS/s on this host.

### Q7. Stable rate on this host (i7-12700 / WSL2 / CF32)?

With **min_output_buffer=1M, max_noutput=1M**, PatternSource:

- source-null: **~4.5–4.7 GS/s** (1G target)  
- source-search / detector-sparse 1 pkt/s: **~3.4–3.6 GS/s** (1G target)  
- detector-sparse 200 pkt/s: **~1.5–1.6 GS/s**, 0 drop  

Without enlarged buffers (default): **~200–240 MS/s** (matches historical reports).

### Q8. Path to 1 GS/s — more GR tuning or fused ingest/SC16?

**1 GS/s is already achieved** on the real GR path with correct buffers. No fused ingest required for the 1 GS/s goal on this hardware.

Next steps only if targeting multi-GS/s headroom or real UHD ingest:

1. Production flowgraphs must set **min_output_buffer** on the source (max alone is wrong).  
2. Optional SEARCH/pre_ring batching (~22% gap vs null at large chunks).  
3. SC16 / fused UHD only if radio path or power/memory dominate — not to clear 1 GS/s.

## 3. Comparison table (scheme §11)

Medians; MS/s; work mean from det when present else src; 500M unless noted.

| Version | source-null | search-only | 1 pkt/s sparse | 200 pkt/s | work mean | CPU% | drop |
|---------|------------:|------------:|---------------:|----------:|----------:|-----:|-----:|
| Original baseline (default buf) | 236 | 216 | 212 | (not re-run at default) | ~4.1k | ~90 | 0 |
| Correct min buffer 512k | 4260 | 3347 | 3340 | — | ~250k | ~166 | 0 |
| Best chunk 1M | 4447 | 3479 | 3468 | **1603** | ~509k | ~169 | 0 |
| SEARCH micro-opt | *not applied* (already >1 GS/s) | | | | | | |
| Architecture prototype | *not needed for 1 GS/s* | | | | | | |

1G target confirmation (best 1M, 3 runs): null ~4600–4690; search ~3435–3523; sparse ~3474–3614 MS/s; det=2; drop=0.

## 4. Full buffer scan matrix (500M, gap≈1 pkt/s, 5 repeats)

### source-null

| config | buffer | max_noutput | MS/s min | med | max | work_mean | CPU% |
|--------|-------:|-----------:|--------:|----:|----:|----------:|-----:|
| default | 0 | 0 | 234.6 | **236.1** | 238.5 | 4095.5 | 87 |
| 32k | 32768 | 32768 | 900.3 | **916.0** | 939.3 | 16383.2 | 94 |
| 128k | 131072 | 131072 | 3495.5 | **3593.8** | 3689.1 | 65530.8 | 118 |
| 512k | 524288 | 524288 | 4242.0 | **4260.3** | 4321.0 | 262054.5 | 108 |
| 1M | 1048576 | 1048576 | 4409.7 | **4447.0** | 4479.2 | 524109.0 | 105 |
| 4M_n1M | 4194304 | 1048576 | 2067.7 | **2167.7** | 2196.8 | 1048218.0 | 101 |
| 4M_n4M | 4194304 | 4194304 | 2035.2 | **2073.8** | 2172.2 | 2092050.2 | 101 |

### source-search (threshold never fires)

| config | buffer | max_noutput | MS/s min | med | max | work_mean | CPU% | det | drop |
|--------|-------:|-----------:|--------:|----:|----:|----------:|-----:|----:|-----:|
| default | 0 | 0 | 212.6 | **215.9** | 217.1 | 4089.4 | 91 | 0 | 0 |
| 32k | 32768 | 32768 | 647.8 | **661.8** | 686.8 | 16307.9 | 97 | 0 | 0 |
| 128k | 131072 | 131072 | 1986.8 | **2187.3** | 2269.7 | 49786.7 | 146 | 0 | 0 |
| 512k | 524288 | 524288 | 3273.3 | **3346.6** | 3399.8 | 249751.2 | 166 | 0 | 0 |
| 1M | 1048576 | 1048576 | 3423.0 | **3478.5** | 3507.1 | 503019.1 | 169 | 0 | 0 |
| 4M_n1M | 4194304 | 1048576 | 1624.9 | **1725.0** | 1760.1 | 1039502.0 | 175 | 0 | 0 |
| 4M_n4M | 4194304 | 4194304 | 1709.6 | **1747.3** | 1791.1 | 2040817.3 | 177 | 0 | 0 |

### detector-sparse (normal threshold, ~1 pkt/s)

| config | buffer | max_noutput | MS/s min | med | max | work_mean | CPU% | det | drop |
|--------|-------:|-----------:|--------:|----:|----:|----------:|-----:|----:|-----:|
| default | 0 | 0 | 202.7 | **212.1** | 212.6 | 4086.5 | 90 | 1 | 0 |
| 32k | 32768 | 32768 | 660.0 | **677.9** | 692.0 | 16328.6 | 96 | 1 | 0 |
| 128k | 131072 | 131072 | 2132.9 | **2281.3** | 2317.3 | 52417.2 | 147 | 1 | 0 |
| 512k | 524288 | 524288 | 3226.8 | **3340.0** | 3432.5 | 250376.5 | 166 | 1 | 0 |
| 1M | 1048576 | 1048576 | 3442.8 | **3468.4** | 3569.9 | 508648.0 | 169 | 1 | 0 |
| 4M_n1M | 4194304 | 1048576 | 1819.8 | **1939.8** | 2000.0 | 1014199.8 | 170 | 1 | 0 |
| 4M_n4M | 4194304 | 4194304 | 1933.1 | **1963.0** | 1975.6 | 1992032.8 | 170 | 1 | 0 |

## 5. Sparse packet-rate scan (500M, buffer 1M, 5 runs)

| Conceptual pkt/s | gap samples | MS/s range (approx) | detections | drop |
|-----------------:|------------:|--------------------:|-----------:|-----:|
| 1 | 998143552 | 3258–3675 (med ~3557) | 1 | 0 |
| 10 | 99743552 | 2999–3416 (med ~3210) | 5 | 0 |
| 100 | 9743552 | 2362–2596 (med ~2552) | 50 | 0 |
| 200 | 4743552 | 1532–1618 (med ~1603) | 100 | 0 |

## 6. 1G target confirmation (best 1M buffer, 3 runs)

| Mode | MS/s | detections | drop |
|------|-----:|-----------:|-----:|
| source-null | 4609 / 4694 / 4574 | 0 | 0 |
| source-search | 3448 / 3523 / 3435 | 0 | 0 |
| detector-sparse | 3474 / 3614 / 3476 | 2 | 0 |

processed samples always equal target (exact PatternSource accounting).

## 7. Code changes

1. **`benchmark_detector`**: layered modes `source-null` / `source-search` / `detector-sparse`; CLI `--buffer-items`, `--max-noutput-items`, `--source pattern|vector`, `--threshold`, `--repeat`; PatternSource **exact target** (no last-chunk overshoot); work-chunk stats + hist; correct **min_output_buffer** API.  
2. **`UwbDetector`**: work-chunk stats API (`work_calls`, min/mean/max, histogram); default `max_noutput_items` raised **65536 → 1048576**.  
3. **QA**: `test_detector_work_chunk_stats` drives real `work()` and asserts stats.  
4. CTest: **4/4 pass** before and after.

## 8. How to reproduce

```bash
cd gr-uwb/build
export LD_LIBRARY_PATH=$PWD/lib:$LD_LIBRARY_PATH
CFILE=../../testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile
BENCH=./apps/benchmark_detector

# Layered A/B
$BENCH $CFILE source-null --target 500000000 --gap 998143552 --repeat 5
$BENCH $CFILE source-null --target 500000000 --gap 998143552 \
  --buffer-items 1048576 --max-noutput-items 1048576 --repeat 5
$BENCH $CFILE source-search --target 500000000 --gap 998143552 \
  --buffer-items 1048576 --max-noutput-items 1048576 --repeat 5
$BENCH $CFILE detector-sparse --target 500000000 --gap 998143552 \
  --buffer-items 1048576 --max-noutput-items 1048576 --repeat 5

ctest --output-on-failure
```

## 9. Decision log

- Phase B: all three GR modes rise together with buffer size → primary issue was **scheduler/buffer granularity**, not worker.  
- Phase C: PatternSource faster than vector → do not blame generator for 264 MS/s era.  
- Phase D SEARCH opts: **skipped** for production algorithm changes; only raised detector max noutput. Gap source-null vs search ~22% at 1M is secondary while still multi-GS/s.  
- Phase F fused architecture: **not required** for 1 GS/s acceptance on this host.

## 10. Questions for GPT (optional follow-up)

1. Given residual ~22% SEARCH/pre_ring cost at multi-GS/s, is further SEARCH batching worth it before real UHD ingest tests?  
2. For production UHD flowgraphs, recommended min_output_buffer / max_noutput defaults and latency tradeoffs?  
3. Why 4M buffers regress vs 1M on this WSL host — expected cache effect or GR-specific?  
4. When worker becomes the limit (dense / high pkt/s), what headroom formula should gate region pool size?
