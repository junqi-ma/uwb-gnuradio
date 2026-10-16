# S0 SIC core API + algorithm contract (frozen)

Implementations: `gr-uwb/include/gnuradio/uwb/uwb_preamble_sic_core.h` (+ `.cc`
in `gr-uwb/lib/`), namespace `gr::uwb::sic`. A NumPy reference mirror lives at
`tools/sic/sic_reference.py`. Both implement **this** contract so the C++ core
can be checked against the Python reference.

Separate the **math core** (this file, no GNU Radio scheduler) from the **PDU
adapter** (`uwb_preamble_sic.*`, W3). The core does no I/O, no heap growth in
`sic_one`, and no GNU Radio types.

## Modes and truth isolation
- `SicMode::Off` — bypass.
- `SicMode::Estimate` — the only mode usable for effect/real-time acceptance.
  It MUST NOT receive or read injected delay / CFO / paths / ISR / interferer IQ
  or any generator event. It sees only `y`, the comm template/preamble and config.
- `SicMode::Oracle` — diagnostic upper bound only; consumes an explicit truth
  struct and sets `uses_truth=true`. Never counted in effect/real-time gates.

## Config
```cpp
enum class SicMode : uint8_t { Off = 0, Estimate = 1, Oracle = 2 };
enum class SicState : uint8_t { Bypassed, Committed, Failed, Ambiguous };

struct SicConfig {
    SicMode mode = SicMode::Off;
    size_t  comm_code = 10;
    size_t  sync_reps = 64;            // nominal; the true start is unknown
    size_t  samples_per_symbol = 1016;
    double  sample_rate = 998.4e6;
    size_t  k_paths = 4;               // 1,2,4,8,16
    size_t  fit_reps = 8;              // estimation window in SYNC (8/16/32)
    size_t  validation_reps = 8;       // holdout, disjoint from fit_reps, >=8
    int64_t roi_lo = 0, roi_hi = 0;    // public candidate search ROI [lo,hi)
    size_t  coarse_stride = 14;        // detection coarse stride (samples)
    size_t  path_min_spacing = 0;      // 0 => derive from template autocorr mainlobe
    size_t  post_tail = 100;           // defined reconstruction tail (samples)
    bool    enable_fractional = true;
    bool    enable_cfo = true;
    double  detect_threshold = 0.0;    // fixed from the CALIBRATION set
    double  validate_threshold = 0.0;  // fixed from the CALIBRATION set
};
```

## Result
```cpp
struct SicTimings {
    uint64_t detect_coarse_us, detect_fine_us, cfo_us, cir_us, path_select_us,
             fractional_refine_us, matrix_build_us, gain_fit_us, reconstruct_us,
             validate_commit_us, core_total_us;
};
struct SicResult {
    SicState state = SicState::Bypassed;
    bool detected = false, committed = false, partial = false, ambiguous = false;
    const char* bypass_reason = nullptr;   // stable string literal
    bool uses_truth = false;
    int64_t detected_start = -1;           // estimated comm SYNC origin (work samples)
    int64_t support_lo = -1, support_hi = -1; // confirmed interferer support [lo,hi)
    double  cfo_hz = 0.0;
    size_t  n_paths = 0, n_path_candidates = 0;
    double  gain_condition = 0.0;          // cond2(A) of the final gain fit; see below
    double  residual_corr = 0.0;           // post-cancel comm residual correlation (holdout)
    uint64_t coarse_candidates = 0, fine_candidates = 0, correlations = 0;
    size_t  roi_samples = 0;
    const char* reject_reason = nullptr;   // when not committed
    SicTimings timings;
};
```

## Templates (both supplied by the caller; independent comm namespace)
- `sync` / `sync_len`: ONE shaped SYNC waveform (e.g. 1016 samples) for `comm_code`.
- `preamble` / `preamble_len`: the FULL comm preamble waveform (real repetition
  pattern, leading edge transient, pulse/multipath tail) for reconstruction.
  Detection correlates `sync`; reconstruction delays `preamble`.

## Scratch
`SicScratch` is allocated/`prepare_sic_core`d once. `sic_one` never grows a
vector, does no file I/O, and does not allocate. State the peak scratch bytes.

## Algorithm (both languages must match)
1. **detect_coarse / detect_fine**: direct normalized correlation of `y` with
   `sync` inside `roi` at `coarse_stride`, then a ±stride fine search at stride 1
   around the best coarse candidate. Count `coarse_candidates`, `fine_candidates`,
   `correlations`, `roi_samples`. All candidates below `detect_threshold` →
   `state=Bypassed`, `detected=false`.
2. **Repetition confirmation**: at `detected_start`, verify repeated peaks at
   `+k*sps` for `k=1..fit_reps-1` (and the holdout reps) are consistent. Not
   enough evidence → `partial`/`ambiguous`; only subtract in the confirmed region
   or bypass. Never treat a mid-period peak as the full-preamble origin.
3. **cfo** (if `enable_cfo`): from the repeated-peak complex correlations
   `c_k`, `dphi = arg(c_{k+1}·conj(c_k))`, `cfo = dphi/(2π·sps/Fs)` averaged over
   the stable peaks. Apply CFO by rotating the **reconstruction basis**, not by
   derotating `y`. Use a recurrence rotation with periodic absolute re-anchor.
4. **cir / path_select**: matched-filter CIR of `y` against `sync` over a local
   delay range; local peaks with `path_min_spacing` (default = template
   autocorrelation mainlobe width) and at most `k_paths` DISTINCT delays; a single
   path must not be counted twice. Record `n_path_candidates`, `roi_samples`.
5. **fractional_refine** (if enabled): refine each `tau_k` to fractional-sample by
   a local search (bounded ±0.5 sample) minimizing the residual on the FIT window,
   using a fixed bandlimited interpolation owned by the core. Integer-only path
   must be closed first and gated separately.
6. **matrix_build / gain_fit**: columns are the `preamble` delayed by `tau_k`
   (fractional) and CFO-rotated, restricted to the FIT window; solve the complex
   least squares `A h ≈ y_fit` with a Householder **QR** of `A` (no normal
   equations and **no explicit matrix inverse**). Check rank/conditioning
   (`gain_condition`) and finite values; fail-closed on ill-conditioning or
   capacity overflow.

   **`gain_condition` semantics (corrected, R05).** It is `cond2(A)`, the ratio
   of the largest to smallest singular value of the M×K design matrix — exactly
   the `numpy.linalg.cond(A)` quantity the reference reports. The C++ core
   computes it from the K×K QR factor `R`: `A = Q R` with `Q` orthogonal implies
   `AᴴA = RᴴR`, so `cond2(A) == cond2(R)` exactly, and the singular values of `R`
   come from a self-contained one-sided complex Jacobi SVD (K ≤ 16, no LAPACK).
   It is **not** the `max|R_kk|/min|R_kk|` diagonal ratio, which underestimates
   `cond(A)` (R05 counterexample `A = [[1,1],[0,1.5e-6]]`: diagonal ratio
   666666.7 vs true `cond2(A)` 1333333.3). The commit gate is
   `gain_condition < 1e6`; a non-finite or rank-deficient factor, or a failed
   solve, is fail-closed (no commit).
7. **reconstruct**: `i_hat[n] = Σ_k h_k · preamble[n - tau_k]·exp(j(2π f_cfo n/Fs))`.
   Subtraction region = intersection(estimated comm support, public sensing ROI)
   plus `post_tail`; bounds from config + signal evidence only. Outside that
   region `out[n] == y[n]` byte-for-byte (including sensing payload not in mask).
8. **validate_commit**: fit and validation use DISJOINT reps. Compute the holdout
   comm residual correlation of `y - i_hat` against `preamble`. Commit
   (`state=Committed`) only if the residual is below `validate_threshold` AND
   parameters are plausible. Total-energy drop alone is NOT sufficient. On any
   failure: `state=Bypassed`, `out == y` exactly. The online gate reads only `y`,
   the template and config — never `s`/`i`/`clean`, and never decides commit from
   the evaluation outcome. Thresholds are fixed on the calibration set and
   unchanged on the acceptance set.

## Timings
Every `SicTimings` field is measured; failed stages are timed; not-run stages
stay 0 with a reason. `core_total_us` is the outer steady_clock total and is the
authority; `core_total` is not the naive sum of children when children overlap.

## QA required (added BEFORE each capability is claimed done)
- integer single path → integer four path → fractional → CFO, each with a
  synthetic model-consistent signal; assert reconstruction NMSE / suppression and
  that `out==y` on a bypass.
- K = 1/2/4/8/16; fit_reps = 8/16/32; narrow vs wide ROI.
- model mismatch: generator uses one fractional-delay model, core another
  (independent bandlimited vs PCHIP) — the estimate must still converge or the
  case must be reported as a declared stress failure, not silently passed.
- truth isolation: assert `estimate` mode never receives truth and that
  `uses_truth` is false; a wrong comm code must not commit on a code-9 signal.
- no double-count: children vs parents in the timings.
- Cross-check against `tools/sic/sic_reference.py` golden vectors where present.
