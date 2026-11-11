# Raw benchmark data (2026-11-11 round 2)

This directory keeps only the **aggregated, reproducible** tables:

- `<variant>/runs.csv`    — one row per run: every metric the driver reported,
                            the return code, the validity flag and the raw JSON path.
- `<variant>/summary.csv` — per config/variant medians.
- `environment.txt`      — host, compiler, cmake args, CPU topology, input/taps hashes.

The per-run JSON/LOG files (one JSON per window plus the console log) are **not**
committed: they are 0.1–1.1 GB per variant.  They are archived at

    /tmp/opencode/archive/<variant>/raw/

and can be regenerated exactly with

    python3 tools/performance/run_sc16_round2_matrix.py ...

using the binaries and configuration recorded in the report.  Every number in
`docs/performance/UHD_SC16重采样与流水线第二轮优化报告_2026-11-11.md` is
re-derivable from `runs.csv` alone with
`tools/performance/summarize_sc16_round2.py`.

Variant directories:

| dir | comparison | note |
|---|---|---|
| `A1` | control vs A1 | superseded: cold protocol dropped windows (see report §5.2) |
| `A1b` | control vs A1 | superseded: same defect |
| `A1c` | control vs A1 | **authoritative** A1 cold-start data, zero drops |
| `A1warm` | control vs A1 | 2000-window steady warm comparison |
| `B1` | A1 vs B1 | superseded: run while another process was active |
| `B1clean` | A1 vs B1 | superseded: feeder still used the yield() gate |
| `B1final` | A1 vs B1 | **authoritative** Phase-B data |
| `C1` | B1 vs B1+C1 | kernel C1 at pipeline level (same binary, switch A/B) |
| `C2` | none vs convert_cache | kernel C2 at pipeline level (same binary) |
| `FINAL` | control vs final | **authoritative** final matrix, 12 configs x 5 ABBA pairs |
