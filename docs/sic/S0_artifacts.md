# S0 artifact schemas (frozen, rev 2 — post G0 reconciliation)

Outputs of the S0 chain. `run_id`/`window_id` must be joinable across every
table. Large raw benchmark output stays under `build-sic/sic-results/<...>` and
is archived explicitly before being referenced; no simulation IQ is committed.

Consumer keys are exactly `clean`, `raw`, `sic`. `sic` is absent when SIC is off.
Every window has exactly one terminal state per consumer (no silent drops).

Rev 2 resolves the G0 producer↔verifier disagreements; see "Adjudications".

## run_manifest.json  (one per run)
```
commit, dirty(bool), build_type, compiler, flags[],
lib_paths{ gnuradio_uwb, loaded[] },         # `env -u LD_LIBRARY_PATH ldd` output
cpu, os, gnuradio_version, volk_version,
config_path, config_hash,
fixture_hashes{},                            # see hash convention
template_hashes{},                           # template_id -> sha256 hex (NO lengths)
template_meta{},                             # template_id -> {length, source, ...}
seed_lists{calibration,acceptance},
sample_rate_work, sample_rate_native, input_grid, preset, sic_mode
```

### Hash convention (frozen)
- `fixture_hashes`: map key -> sha256 hex. A produced file key is a path
  relative to the results dir (e.g. `fixtures/window_0000.npz`). A repo fixture
  key is `repo:` + repo-relative path (e.g. `repo:testdata/uwb_radar/
  rx_clean_998p4.cf32`); the verifier resolves `repo:` against the repo root.
- `template_hashes`: template_id -> sha256 hex of the CF32 waveform bytes.
  Values are ALWAYS 64-hex. Never carry lengths or non-hash values here.
- `template_meta`: template_id -> {length(int), source(string), sha256(same)}.

## windows.jsonl  (one JSON object per window)
```
schema_version, run_id, window_id, epoch_id, sample_rate, sample_count,
window_start_sample, predicted_sensing_start,
sensing_profile_id, sensing_code, sensing_sync_reps, sensing_sfd_mode, template_id,
overlap{lo,hi,samples},                       # [lo,hi) work samples, realized
isr_db (nullable), snr_db, comm_code, sic_k,
fit_reps (nullable iff sic off), validation_reps (nullable iff sic off),
roi (nullable iff sic off; [lo,hi) when SIC on), demod_softchip_mode,
clean{status, cir_status, fcs_pass, psdu_ok, psdu_hex, cir_peak_tap, target_amp,
      input_sha256, input_samples, stage_*},   # consumer result + posted-IQ hash
raw{...}, sic{...},                           # sic omitted when off
sic_state{detected, committed, partial, ambiguous, bypass_reason, uses_truth},
admission{published_ns, admitted_ns, completion_ns, missed_admission(bool)},
recon{...} (nullable iff sic off)
```
`native` adds `coord{work_to_native, native_to_work, filter_delay, valid_lo, valid_hi}`.

When `sic_mode == off`: `sic_k == 0`, `fit_reps`/`validation_reps`/`roi`/`recon`
MUST be `null` (not a fake positive value), and `clean`/`raw` only.

When interference is off, the generator MUST make `i` exactly zero and compute
`y == s + w` **with the same expression used for the clean consumer**, so
`clean.input_sha256 == raw.input_sha256` in the baseline preset.

## stage_times.csv  (long form; rev 2 adds a `scope` column)
```
run_id,window_id,consumer,scope,tier,stage,us,detail
```
- `scope` ∈ `demod`, `radar`, `sic`, `outer`, `adapter`. It disambiguates the
  `cir`/`cfo` names that exist in both the demod and radar/SIC paths.
- `tier` = `cxx`, `adapter`, or `outer`.
- demod `stage` values (scope=demod): `timing`, `timing_coarse`,
  `timing_fine_track`, `cfo`, `sfd`, `sfd_bootstrap`, `sfd_tail_retrack`,
  `sfd_cfo_refit`, `sfd_final`, `sfd_anchor_other`, `cir`, `cir_estimate`,
  `soft_fir`, `postprocess`, `ns_sfd`, `phr`, `payload`, `total`.
  `sfd_*` children sum to <= `sfd`; `cir_*` children sum to <= `cir`; the demod
  `total` row >= the sum of the top-level demod stages only.
- radar `stage` (scope=radar): `cir` (radar_cir_only), plus its own subfields.
- SIC `stage` values (scope=sic): `detect_coarse`, `detect_fine`, `cfo`, `cir`,
  `path_select`, `fractional_refine`, `matrix_build`, `gain_fit`, `reconstruct`,
  `validate_commit`, `core_total`.
- Failed stages are timed; not-run stages are `not_run` (us empty).
- Instrumentation on/off short comparison reported; existing values kept.

## summary.json
```
run_id, preset, input_grid, sic_mode, windows_expected, windows_completed,
clean{..., mean/p50/p95/p99/max}, raw{...}, sic{...},
effect{...}, demod{clean_ok, raw_ok, sic_ok, raw_ok_sic_fail, raw_fail_sic_ok,
       failing_stages},
negative{...}, runtime{...}, gates{...}, not_run[]
```
- Every listed key MUST be present. When a key is not applicable (e.g. SIC off),
  emit it as `null` and add a matching `not_run[]` reason. This keeps
  "missing key = failure" strict.
- `failing_stages`: sorted set of consumer names (`clean`/`raw`/`sic`) that
  produced at least one non-success terminal status in `windows.jsonl`. Empty
  list when all succeed.

## truth.jsonl  (generator only; NEVER read by the SIC API or any online gate)
```
run_id, window_id, seed, sensing_code, comm_code,
comm{start_sample_exact, cfo_hz, paths[{delay_samples, gain}], isr_db, scale,
     real_overlap_lo, real_overlap_hi, frac_model},
sensing{paths[...]},
hashes{template_id, template_sha256}
```
`oracle` mode reads `truth.jsonl` and MUST emit `uses_truth=true`; diagnostic
only, never in effect/real-time acceptance.

## verifier contract
`tools/sic/verify_s0.py` checks only **declared required** artifacts:
input hashes, sample counts, completion counts, FCS/PSDU where applicable,
three-way pairing on the same window_id, and timer completeness. Missing
required artifacts are a failure; missing `summary` keys are never silently
treated as success. Every unexecuted item must appear in `not_run`.

## Adjudications (frozen decisions behind rev 2)
1. Timing scope: add `scope`; the demod `total >=` check sums demod-scoped
   stages only (radar `cir` no longer pollutes it).
2. SIC-off nullability: `fit_reps`/`validation_reps`/`roi`/`recon` null when
   `sic_mode==off`; the verifier accepts null iff off.
3. summary completeness: SIC-conditional keys present as `null` + a `not_run`
   reason when off (never omitted).
4. baseline `raw == clean`: enforced exactly via `input_sha256` equality, plus
   exact equality of `status/fcs_pass/psdu_ok/psdu_hex/cir_peak_tap`. Timing
   fields are excluded. (Generator guarantees `y == s+w` bitwise when off.)
5. Manifest hashes: `repo:` prefix resolves against the repo root;
   `template_hashes` are sha256 hex only; lengths move to `template_meta`.
6. `failing_stages` = consumers with >=1 non-success terminal status.
```
