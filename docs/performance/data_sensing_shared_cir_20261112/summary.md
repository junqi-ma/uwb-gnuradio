# Combined summary (3 independent sessions)

All values in microseconds. `ratio` = candidate mean / control mean for the same metric.

`mean`/`P50`/`P95` are the average of the per-session values; the bracketed range is the min..max of those per-session values. `min`/`max` are the extreme single observations seen in any session.


## sync64_psdu32

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| control | total | 520 | 855.77 [854.18..857.53] | 831.04 | 985.74 | 776.97 | 3015.35 |  |
| control | sfd | 520 | 667.85 [663.87..670.47] | 648.69 | 768.64 | 605.83 | 2834.03 |  |
| control | refine | 520 | 13.04 [12.18..14.49] | 11.68 | 15.35 | 10.98 | 1137.44 |  |
| control | cir_scalar_kernel | 520 | 174.89 [173.96..175.83] | 168.24 | 211.24 | 157.32 | 634.46 |  |
| candidate | total | 520 | 648.07 [641.18..652.31] | 628.12 | 761.35 | 590.90 | 2656.86 | 0.7573 |
| candidate | frontend_timing | 520 | 95.82 [93.85..97.50] | 91.67 | 112.00 | 85.00 | 767.00 |  |
| candidate | frontend_cfo | 520 | 15.31 [15.14..15.45] | 14.67 | 19.00 | 14.00 | 82.00 |  |
| candidate | frontend_sfd | 520 | 521.61 [517.06..524.09] | 505.67 | 622.33 | 477.00 | 2346.00 |  |
| candidate | coverage | 520 | 0.00 [0.00..0.01] | 0.00 | 0.00 | 0.00 | 6.00 |  |
| candidate | cir_us_reported | 520 | 12.45 [12.28..12.57] | 12.00 | 17.67 | 11.00 | 72.00 |  |
| candidate | cir_shared_kernel | 520 | 13.03 [12.80..13.24] | 12.32 | 15.78 | 11.74 | 177.73 |  |

## sync256_psdu127

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| control | total | 520 | 899.25 [891.77..905.72] | 873.24 | 1068.83 | 810.48 | 4318.85 |  |
| control | sfd | 520 | 668.97 [664.77..675.25] | 647.97 | 789.60 | 605.57 | 4042.41 |  |
| control | refine | 520 | 12.45 [12.28..12.66] | 11.78 | 15.67 | 11.00 | 147.12 |  |
| control | cir_scalar_kernel | 520 | 217.82 [214.72..220.94] | 208.99 | 265.69 | 192.46 | 696.93 |  |
| candidate | total | 520 | 1176.61 [1168.43..1187.97] | 1134.37 | 1393.66 | 1044.77 | 5408.27 | 1.3084 |
| candidate | frontend_timing | 520 | 387.93 [380.84..396.30] | 370.67 | 466.33 | 336.00 | 4525.00 |  |
| candidate | frontend_cfo | 520 | 17.03 [16.73..17.29] | 16.00 | 21.67 | 15.00 | 114.00 |  |
| candidate | frontend_sfd | 520 | 716.50 [710.95..720.01] | 689.00 | 860.33 | 647.00 | 4151.00 |  |
| candidate | coverage | 520 | 0.17 [0.11..0.22] | 0.00 | 1.00 | 0.00 | 45.00 |  |
| candidate | cir_us_reported | 520 | 51.24 [50.34..52.13] | 45.67 | 78.00 | 41.00 | 507.00 |  |
| candidate | cir_shared_kernel | 520 | 46.67 [45.97..47.67] | 42.56 | 65.77 | 38.37 | 268.93 |  |

## comm64_psdu32

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| comm_legacy_profile | total | 520 | 1311.03 [1292.09..1342.62] | 1272.78 | 1476.25 | 1204.79 | 5853.50 |  |
| comm_legacy_profile | stage_cir | 520 | 551.16 [543.17..562.23] | 535.00 | 640.33 | 505.00 | 3676.00 |  |
| comm_fast_profile | total | 520 | 344.24 [342.12..348.07] | 327.99 | 429.05 | 309.93 | 1828.74 |  |
| comm_fast_profile | stage_cir | 520 | 51.64 [50.89..52.47] | 48.67 | 65.00 | 47.00 | 350.00 |  |

## Key ratios (candidate / control)

| config | metric | control mean | candidate mean | ratio | speedup |
|---|---|---|---|---|---|
| sync64_psdu32 | total | 855.77 | 648.07 | 0.7573 | 1.320x |
| sync64_psdu32 | cir_scalar_kernel -> cir_shared_kernel | 174.89 | 13.03 | 0.0745 | 13.425x |
| sync256_psdu127 | total | 899.25 | 1176.61 | 1.3084 | 0.764x |
| sync256_psdu127 | cir_scalar_kernel -> cir_shared_kernel | 217.82 | 46.67 | 0.2142 | 4.667x |
| comm64_psdu32 | total | 1311.03 | 344.24 | 0.2626 | 3.808x |
| comm64_psdu32 | stage_cir | 551.16 | 51.64 | 0.0937 | 10.672x |
