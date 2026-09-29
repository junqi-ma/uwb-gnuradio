# M1-A ToF oracle vectors

`tof_oracle_vectors.json` is the independent golden for the M1-A SS/DS ToF
mathematics (`gr-uwb/include/gnuradio/uwb/uwb_twr_math.h`).  It is produced
from an explicit PHYSICAL model, never from the C++ output (REQ-QA-01).

## STATUS: the MATLAB oracle has NOT been run

The M1-A instruction requires the MATLAB script to be executed on this machine
(`matlab -batch "cd('testdata/twr'); generate_tof_oracle"`).  **It was not**,
because this machine has no runnable MATLAB:

* `/usr/local/MATLAB/R2024a/` exists but contains no `bin/matlab` launcher and
  no `bin/glnxa64/MATLAB` binary -- only support libraries and licence files;
* `matlab` is not on `PATH`;
* there is no MATLAB Runtime / MCR and no Octave.

Per the instruction, an unrun `.m` is **not** "MATLAB verified", so this file
does not claim it is.  The checked-in vectors were produced by
`gen_tof_oracle.py`, an exact `fractions.Fraction` reference of the **same**
model.  That is a weaker authority than MATLAB and is recorded as such in the
JSON `provenance` block.

To close this properly, run the MATLAB script on a machine that has MATLAB and
commit the regenerated vectors (object key order will differ; object order is
not significant).

## Files

| file | role |
|---|---|
| `generate_tof_oracle.m` | the oracle of record (MATLAB, R2024a). **Written, not executed.** |
| `gen_tof_oracle.py` | exact `fractions.Fraction` reference of the same model; produced the checked-in vectors |
| `tof_oracle_vectors.json` | the golden. 12 vectors. |
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
# regenerate with the Python reference (what produced the checked-in file)
python3 testdata/twr/gen_tof_oracle.py

# the MATLAB oracle of record (needs a working MATLAB)
matlab -batch "cd('testdata/twr'); generate_tof_oracle"

# the C++ QA and the independent verifier
env -u LD_LIBRARY_PATH ctest -R uwb_qa_uwb_twr_math --output-on-failure
python3 tools/twr/verify_m1_a_tof.py
```

## Hashes (of the checked-in files)

```
sha256(tof_oracle_vectors.json) = 673592679848e6a37a4340316d4986019bc1c19ef31372fb4fb5dfe3a55924ea
sha256(gen_tof_oracle.py)       = d09859b88cbd7ed5eac4e90acc5d90dc23057adf6dbe017086317f57dba18023
sha256(generate_tof_oracle.m)   = 91eb8cb05e937684dea82facab34082d70cd697ecb5bb7e92cdd3d5b1109a418
```
