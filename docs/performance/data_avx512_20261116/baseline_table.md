# AVX2 基线汇总（3 轮中位数，µs）

被测二进制 /tmp/opencode/bench_avx2_baseline
SHA256 dc425102a10518ea6b4fde7f1e6b9032e1749466dbabf4bb38605b4a02d0ab6a
逐轮原始值见 baseline_table.csv；单次运行 JSON 见同目录。

## 65/48

| taps (T, Hpad) | len | fp | core mean | core P50 | core P95 | core P99 | total mean | total P50 | total P95 | total P99 | ns/native sample | MS/s (total mean) | JSON (r1 / r2 / r3) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| realtime_eq16 (1039, 16) | 4096 | 1 | 7.37 | 7.17 | 7.68 | 9.29 | 7.47 | 7.27 | 7.80 | 9.40 | 1.825 | 548.0 | 65_48_realtime_eq16_4096_fp1_r1 / 65_48_realtime_eq16_4096_fp1_r2 / 65_48_realtime_eq16_4096_fp1_r3 |
| realtime_eq16 (1039, 16) | 4096 | 0 | 8.58 | 8.49 | 9.30 | 10.04 | 8.68 | 8.59 | 9.43 | 10.16 | 2.120 | 471.8 | 65_48_realtime_eq16_4096_fp0_r1 / 65_48_realtime_eq16_4096_fp0_r2 / 65_48_realtime_eq16_4096_fp0_r3 |
| realtime_eq16 (1039, 16) | 56000 | 1 | 93.80 | 93.44 | 95.13 | 97.12 | 93.95 | 93.58 | 95.28 | 97.27 | 1.678 | 596.1 | 65_48_realtime_eq16_56000_fp1_r1 / 65_48_realtime_eq16_56000_fp1_r2 / 65_48_realtime_eq16_56000_fp1_r3 |
| realtime_eq16 (1039, 16) | 56000 | 0 | 111.05 | 110.04 | 118.30 | 120.62 | 111.20 | 110.18 | 118.67 | 120.77 | 1.986 | 503.6 | 65_48_realtime_eq16_56000_fp0_r1 / 65_48_realtime_eq16_56000_fp0_r2 / 65_48_realtime_eq16_56000_fp0_r3 |
| realtime_minorder (1319, 24) | 4096 | 1 | 9.18 | 9.17 | 9.23 | 9.59 | 9.32 | 9.32 | 9.39 | 10.46 | 2.276 | 439.4 | 65_48_realtime_minorder_4096_fp1_r1 / 65_48_realtime_minorder_4096_fp1_r2 / 65_48_realtime_minorder_4096_fp1_r3 |
| realtime_minorder (1319, 24) | 4096 | 0 | 10.70 | 10.67 | 10.75 | 11.65 | 10.85 | 10.82 | 10.90 | 11.80 | 2.648 | 377.6 | 65_48_realtime_minorder_4096_fp0_r1 / 65_48_realtime_minorder_4096_fp0_r2 / 65_48_realtime_minorder_4096_fp0_r3 |
| realtime_minorder (1319, 24) | 56000 | 1 | 120.36 | 119.26 | 126.68 | 136.13 | 120.52 | 119.42 | 126.87 | 136.34 | 2.152 | 464.6 | 65_48_realtime_minorder_56000_fp1_r1 / 65_48_realtime_minorder_56000_fp1_r2 / 65_48_realtime_minorder_56000_fp1_r3 |
| realtime_minorder (1319, 24) | 56000 | 0 | 139.19 | 138.94 | 142.04 | 143.06 | 139.39 | 139.13 | 142.24 | 143.24 | 2.489 | 401.8 | 65_48_realtime_minorder_56000_fp0_r1 / 65_48_realtime_minorder_56000_fp0_r2 / 65_48_realtime_minorder_56000_fp0_r3 |
| quality_eq32 (2079, 32) | 4096 | 1 | 11.42 | 11.35 | 11.44 | 13.04 | 11.66 | 11.59 | 11.69 | 13.29 | 2.846 | 351.4 | 65_48_quality_eq32_4096_fp1_r1 / 65_48_quality_eq32_4096_fp1_r2 / 65_48_quality_eq32_4096_fp1_r3 |
| quality_eq32 (2079, 32) | 4096 | 0 | 12.84 | 12.80 | 12.93 | 14.33 | 13.08 | 13.04 | 13.21 | 14.57 | 3.193 | 313.2 | 65_48_quality_eq32_4096_fp0_r1 / 65_48_quality_eq32_4096_fp0_r2 / 65_48_quality_eq32_4096_fp0_r3 |
| quality_eq32 (2079, 32) | 56000 | 1 | 144.78 | 144.09 | 146.53 | 151.68 | 145.05 | 144.35 | 146.82 | 151.96 | 2.590 | 386.1 | 65_48_quality_eq32_56000_fp1_r1 / 65_48_quality_eq32_56000_fp1_r2 / 65_48_quality_eq32_56000_fp1_r3 |
| quality_eq32 (2079, 32) | 56000 | 0 | 166.57 | 166.17 | 169.88 | 176.42 | 166.85 | 166.46 | 170.16 | 176.70 | 2.980 | 335.6 | 65_48_quality_eq32_56000_fp0_r1 / 65_48_quality_eq32_56000_fp0_r2 / 65_48_quality_eq32_56000_fp0_r3 |
| quality_minorder (2707, 48) | 4096 | 1 | 15.38 | 15.34 | 15.50 | 16.67 | 15.73 | 15.69 | 15.90 | 17.05 | 3.841 | 260.3 | 65_48_quality_minorder_4096_fp1_r1 / 65_48_quality_minorder_4096_fp1_r2 / 65_48_quality_minorder_4096_fp1_r3 |
| quality_minorder (2707, 48) | 4096 | 0 | 16.45 | 16.45 | 16.52 | 17.82 | 16.83 | 16.80 | 16.88 | 18.52 | 4.108 | 243.4 | 65_48_quality_minorder_4096_fp0_r1 / 65_48_quality_minorder_4096_fp0_r2 / 65_48_quality_minorder_4096_fp0_r3 |
| quality_minorder (2707, 48) | 56000 | 1 | 229.78 | 229.36 | 232.48 | 233.23 | 230.19 | 229.75 | 232.88 | 233.63 | 4.111 | 243.3 | 65_48_quality_minorder_56000_fp1_r1 / 65_48_quality_minorder_56000_fp1_r2 / 65_48_quality_minorder_56000_fp1_r3 |
| quality_minorder (2707, 48) | 56000 | 0 | 242.31 | 241.99 | 245.12 | 250.29 | 242.69 | 242.39 | 245.51 | 250.97 | 4.334 | 230.7 | 65_48_quality_minorder_56000_fp0_r1 / 65_48_quality_minorder_56000_fp0_r2 / 65_48_quality_minorder_56000_fp0_r3 |

## 65/32（仅 fixed-phase 1）

| taps (T, Hpad) | len | fp | core mean | core P50 | core P95 | core P99 | total mean | total P50 | total P95 | total P99 | ns/native sample | MS/s (total mean) | JSON (r1 / r2 / r3) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| realtime_eq16 (1039, 16) | 4096 | 1 | 10.66 | 10.63 | 10.77 | 11.71 | 10.89 | 10.85 | 11.00 | 12.10 | 2.657 | 376.3 | 65_32_realtime_eq16_4096_fp1_r1 / 65_32_realtime_eq16_4096_fp1_r2 / 65_32_realtime_eq16_4096_fp1_r3 |
| realtime_eq16 (1039, 16) | 56000 | 1 | 136.51 | 136.09 | 138.35 | 140.02 | 136.77 | 136.34 | 138.60 | 140.28 | 2.442 | 409.4 | 65_32_realtime_eq16_56000_fp1_r1 / 65_32_realtime_eq16_56000_fp1_r2 / 65_32_realtime_eq16_56000_fp1_r3 |
| quality_minorder (2707, 48) | 4096 | 1 | 23.22 | 23.15 | 23.42 | 24.69 | 23.88 | 23.82 | 24.07 | 25.54 | 5.830 | 171.5 | 65_32_quality_minorder_4096_fp1_r1 / 65_32_quality_minorder_4096_fp1_r2 / 65_32_quality_minorder_4096_fp1_r3 |
| quality_minorder (2707, 48) | 56000 | 1 | 340.34 | 340.02 | 343.81 | 347.59 | 341.05 | 340.69 | 344.52 | 348.30 | 6.090 | 164.2 | 65_32_quality_minorder_56000_fp1_r1 / 65_32_quality_minorder_56000_fp1_r2 / 65_32_quality_minorder_56000_fp1_r3 |
