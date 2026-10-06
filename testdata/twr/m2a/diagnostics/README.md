# M2-A diagnostics: resampler core is not bitwise reproducible

Recorded 2026-10-06.  Status: **known, reproduced, NOT fixed, NOT localised to a
source line.**  Per the M2-A closeout task §2/§3 the SIMD kernel is **not**
touched; fixing it is a separate task that needs the user's authorisation.

## 1. What was measured, and what was not

**Measured**: two FRESH `RationalResamplerLmCore` instances, given the SAME taps
and the SAME input, do not always produce the same full-upfirdn output.  The
number of differing output samples and the magnitude change from run to run.

**NOT measured / NOT claimed**: WHERE an uninitialised read happens.  There is no
read-address, no source line and no MSAN/valgrind trace here.  The evidence is
the observable symptom only.  `ASAN`/`UBSAN` are clean, and that does **not**
exclude uninitialised reads: ASan tracks bounds/use-after-free, UBSan tracks UB,
neither tracks uninitialised *values* (that is MSan's job, which is not available
here).  No "the read is gone" claim may be derived from a green ASan run.

## 2. Environment (recorded, because the symptom depends on it)

| item | value |
|---|---|
| compiler | `c++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0`, `-std=c++17 -O2` |
| libc | glibc 2.35 (`ldd (Ubuntu GLIBC 2.35-0ubuntu3.14)`) |
| VOLK | `/usr/local/lib/libvolk.so.3.3.0` (symlink `libvolk.so.3.3`) |
| CPU | AVX2 + FMA present (`/proc/cpuinfo`) |
| allocator env | varies with `MALLOC_PERTURB_` |

## 3. Inputs (hashes)

| input | sha256 |
|---|---|
| `testdata/resampler_65_48/taps_quality_minorder.txt` (T=2707, DC=65) | `07b09983f072a06604ec08a1fee19317e010b2b10d2bf79077b6f2b6a39a2667` |
| `testdata/resampler_65_32/taps_quality_minorder.txt` (T=2707, DC=65) | `07b09983f072a06604ec08a1fee19317e010b2b10d2bf79077b6f2b6a39a2667` |
| `testdata/twr/m2a/taps/tx_48_65.f32` (T=2707, DC=48) | `edc40fa1427adbb1a4d140aa3e9824431ee1812e687af61515129256595b0ab5` |
| `gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h` | `d0b9af704164a5c6e1f8d3451dfcda065e7af4f173605bf66803cfcd9045e1d4` |

Probe inputs: `N = 4096`, deterministic input from `std::mt19937(0x5EED)` with
`uniform_real_distribution<float>(-0.5, 0.5)`, output buffer `Lout + 256` where
`Lout = ceil(((N-1)*65 + 2707)/48) = 5602`.

## 4. Observations

`repro_det_size_k.cc`, taps `resampler_65_48/taps_quality_minorder.txt`, `65/48`:

| kernel | runs | result |
|---|---|---|
| `scalar_macroblock` | 6 | `ndiff = 0`, `max_abs = 0` — **always** |
| `volk_macroblock` | 10 | `ndiff = 1221` (7 runs) or `ndiff = 4` (3 runs), `max_abs` 1.794e-07 / 3.072e-08 |
| `avx2_fma_macroblock` | 10 | `ndiff = 1221` (2 runs) or `ndiff = 4` (8 runs) |

The **scalar kernel is deterministic**; only the SIMD/VOLK macroblock FIR paths
vary.  `ndiff` is the count of output samples where the two cores differ.

**Within one process vs between processes.**  Two fresh cores inside the SAME
process usually agree (they tend to reuse the same heap block); the *value*
differs BETWEEN processes.  A probe that only compares two cores in one process
therefore **under-reports** this.  `resampler_nondeterminism_probe.cc` is such a
probe and did NOT reproduce (12/12 runs, all kernels, identical `out_hash`); it is
archived to show that the symptom is **allocation-pattern sensitive**, which is
itself consistent with reading uninitialised memory but is not a root-cause proof.

## 5. Reproduce

```bash
cd /home/oi/Desktop/uwb-gnuradio
c++ -std=c++17 -O2 -I gr-uwb/include \
    testdata/twr/m2a/diagnostics/repro_det_size_k.cc \
    -o /tmp/repro_size -L/usr/local/lib -lvolk -lpthread -ldl -lm

# scalar: deterministic
for i in $(seq 1 6); do
  env -u LD_LIBRARY_PATH /tmp/repro_size testdata/resampler_65_48/taps_quality_minorder.txt 256 scalar_macroblock
done | sort | uniq -c

# volk / avx2: alternating ndiff
for i in $(seq 1 10); do
  env -u LD_LIBRARY_PATH /tmp/repro_size testdata/resampler_65_48/taps_quality_minorder.txt 256 volk_macroblock
done | sort | uniq -c
for i in $(seq 1 10); do
  env -u LD_LIBRARY_PATH /tmp/repro_size testdata/resampler_65_48/taps_quality_minorder.txt 256 avx2_fma_macroblock
done | sort | uniq -c

# the allocation-pattern-sensitive third probe (does not reproduce here)
c++ -std=c++17 -O2 -I gr-uwb/include \
    testdata/twr/m2a/diagnostics/resampler_nondeterminism_probe.cc \
    -o /tmp/rsnd -L/usr/local/lib -lvolk -lpthread -ldl -lm
for i in $(seq 1 12); do
  env -u LD_LIBRARY_PATH /tmp/rsnd testdata/resampler_65_48/taps_quality_minorder.txt 65 48 4096 256 volk_macroblock 1
done | sort | uniq -c
```

## 6. Impact

* LSB-level (<= 1.8e-07) non-determinism in the resampler output.
* **Byte-level decode is unaffected**: the M2-A `A01` round trips and the M1-B /
  radar regressions are byte-exact.  A `1e-5` comparison bound is therefore used
  for the M2-A chunk-invariance QA instead of bitwise equality; a real state leak
  would be O(0.1+) and is still caught.
* The existing 65/48 and 65/32 goldens compare within a documented tolerance.

## 7. Files

| file | role |
|---|---|
| `repro_det_size_k.cc` | the reproducing probe (two fresh cores, same input) |
| `resampler_nondeterminism_probe.cc` | a third probe that did NOT reproduce; archived to show allocation-pattern sensitivity |
| `README.md` | this record |
