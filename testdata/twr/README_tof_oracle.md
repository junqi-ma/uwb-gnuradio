# M1-A ToF oracle vectors

`tof_oracle_vectors.json` is the independent golden for the M1-A SS/DS ToF
mathematics (`gr-uwb/include/gnuradio/uwb/uwb_twr_math.h`).  It is produced
from an explicit PHYSICAL model, never from the C++ output (REQ-QA-01).

## STATUS: the MATLAB oracle HAS been run

The oracle of record, `generate_tof_oracle.m`, **was executed** and produced the
checked-in `tof_oracle_vectors.json`:

```bash
matlab -batch "cd('testdata/twr'); generate_tof_oracle"
```

| field | value |
|---|---|
| MATLAB | `25.2.0.2998904 (R2025b)`, release `2025b` |
| host | Windows install at `F:\MATLAB`, invoked from WSL through `matlab.exe -batch` |
| command | `matlab -batch "cd('testdata/twr'); generate_tof_oracle"` |
| exit status | `0` |
| output | `wrote .../tof_oracle_vectors.json (12 vectors)` |
| vector count | 12 |
| JSON `provenance.matlab_executed` | `true` |

The first execution also fixed latent bugs in the script, which had been
written but never run: single-argument `R(n)` calls, a struct addition
(`R(3000)+R(2,7)`), and -- most importantly -- the wrap fold, which computed the
sub-tick fraction against the *un-modded* integer and therefore emitted a bogus
large `num` for the `ss8_wrapped_a_interval` vector.  `jsonencode` also required
the vector list to be a **cell array** rather than `[V{:}]`, so that SS vectors
carry only `ra`/`db` and DS vectors all four intervals.

### Independent agreement

`gen_tof_oracle.py` is an exact `fractions.Fraction` implementation of the
**same** physical model.  Its 12 vectors are identical to the MATLAB output
field for field; `tools/twr/verify_m1_a_tof.py` and `gr-uwb/lib/qa_uwb_twr_math.cc`
then check the C++ against both.  The Python file additionally gives an
independent re-derivation from the raw endpoint ticks, including the wrap.

### Two things that are NOT failures

* MATLAB's `jsonencode` **sorts object keys**; the Python writer does not.
  Object order is not significant.
* MATLAB writes large magnitudes in **exponent notation** (e.g.
  `"tick_rate_hz": 6.38976E+10`).  That is valid JSON; the in-tree reader
  (`uwb_twr_config.h`, `json::parse_number` via `strtod`) and Python both parse
  it, and every value round-trips exactly (all magnitudes here are integers
  below 2^53).

## Files

| file | role |
|---|---|
| `generate_tof_oracle.m` | the oracle of record (MATLAB). **Executed** on R2025b; produced the checked-in vectors. |
| `gen_tof_oracle.py` | exact `fractions.Fraction` reference of the same model; independent second check. Re-running it overwrites the MATLAB golden with an equivalent Python one. |
| `tof_oracle_vectors.json` | the golden. 12 vectors, MATLAB output. |
| `README_tof_oracle.md` | this file |

## The model

Two clocks `fA`, `fB`.  `k = fA/fB` converts B ticks into A ticks.

```
RA = t4A - t1A = 2*tau_A + dB_A          (A ticks)
DA = t5A - t4A = dA_A                    (A ticks)
DB = t3B - t2B = dB_A / k                (B ticks)
RB = t6B - t3B = (2*tau_A + dA_A) / k    (B ticks)

SS: ToF_A = (RA - k*DB) / 2                              = tau_A
DS: ToF_A = (RA*k*RB - DA*k*DB)/(RA + k*RB + DA + k*DB)  = tau_A
```

Both identities are exact, so the generator re-evaluates the formula and
asserts it equals `tau_A` before writing anything.

## Vector set

| id | protocol | covers |
|---|---|---|
| `ss1_common_clock` | ss | one physical clock, `k=1` stated; the install-consumer example |
| `ss2_nominal_rate_ratio` | ss | `k = 625/624` from 1.0 GHz vs 998.4 MHz; sub-tick DB |
| `ss3a_a_faster_30ppm` | ss | `k > 1` (finest representable deviation) |
| `ss3b_a_slower_30ppm` | ss | `k < 1` |
| `ss5f_fractional_tof` | ss | ToF = 5/2 ticks (no truncation) |
| `ss8_wrapped_a_interval` | ss | A interval crosses the 40-bit wrap |
| `ss7_negative_tof` | ss | negative ToF retained signed |
| `ds1_symmetric` | ds | `DA == DB`, agrees with `ss1` |
| `ds2_asymmetric` | ds | `DA != DB` (APS013) |
| `ds3_tiny_tof_long_turnaround` | ds | tiny ToF, long asymmetric turnaround, fractional |
| `ds7_asymmetric_rate_ratio` | ds | `k != 1` and asymmetric -- the unit conversion is load bearing |
| `ds6_negative_tof` | ds | negative ToF retained signed |

The failure matrix (`ss4`, `ss5`, `ss6`, `ds4`, `ds5`, `com1`..`com3`) is
exercised directly in `gr-uwb/lib/qa_uwb_twr_math.cc` with hand-written
expectations, because it depends on the admission context (missing ratio,
expired window, epoch mismatch, all-zero intervals, 128-bit overflow,
out-of-domain enum, type-level `Timestamp` rejection) rather than on a
physical value.

## Reproduce

```bash
# the oracle of record: this is what produced the checked-in vectors
matlab -batch "cd('testdata/twr'); generate_tof_oracle"

# the Python reference (independent; overwrites the golden with an
# equivalent file -- do not run it if you want to keep MATLAB provenance)
python3 testdata/twr/gen_tof_oracle.py

# the C++ QA and the independent verifier, against the MATLAB vectors
cd gr-uwb/build && cmake . && cmake --build . -j"$(nproc)"
env -u LD_LIBRARY_PATH ctest -R uwb_qa_uwb_twr_math --output-on-failure
python3 tools/twr/verify_m1_a_tof.py
```

## Hashes (of the checked-in files)

```
sha256(tof_oracle_vectors.json) = 9d4dd0036240d8abaaeae1d7655eb2045c0da3970613645063909d65d14249d4
sha256(generate_tof_oracle.m)   = 2aa8ce31cbe3f13e6ea8b2da9747dee7d967a90b9b18a939402f950e50d2405c
sha256(gen_tof_oracle.py)       = 5e71cc5fc6e19e883925cfa1189e9d6f487f285852596ff9d8d7b9a1b4238f42
```
