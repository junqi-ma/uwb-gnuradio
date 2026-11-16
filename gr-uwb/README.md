# gr-uwb

This is the CMake source tree for native UWB DSP/UHD libraries and the optional
GNU Radio adapter. See the [root README](../README.md) and the
[current build/architecture guide](../docs/原生接收与GNU_Radio构建使用说明.md).

## Native build

From the repository root:

```bash
cmake -S gr-uwb -B gr-uwb/build-native \
  -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=OFF -DUWB_BUILD_EXPERIMENTS=OFF \
  -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native -j4
ctest --test-dir gr-uwb/build-native --output-on-failure -j1
```

Requires C++17, CMake ≥ 3.8, standalone VOLK, Threads and Boost.Test. Python 3
runs Python QA. `ENABLE_UHD_BACKEND=ON` requires UHD; `OFF` builds without the
hardware backend, while `AUTO` (default) detects it. No GNU Radio/PMT dependency
is needed for this mode.

## SIMD backend

`-DUWB_SIMD=AUTO|AVX2|AVX512|SCALAR` (default `AUTO`) selects the compile-time ISA
for the hand-written DSP kernels. `AUTO` reads `/proc/cpuinfo` on a native Linux
build and refuses to guess when cross-compiling. `AVX2` passes `-mavx2 -mfma`
explicitly and never `-march=native`; `AVX512` adds a separate kernel translation
unit built with `avx512f/bw/dq/vl` function target attributes while the rest of
the program stays on the AVX2/FMA baseline; `SCALAR` disables the self-written
SIMD kernels (VOLK's own runtime dispatch is not affected). Non-`SCALAR` binaries
carry a target-CPU requirement, and `gr::uwb::core::simd::require_simd_backend()`
refuses an unsupported machine with a clear error instead of SIGILL. AVX-512
kernels are bit-identical to AVX2 and QA enforces it. Per-`H_pad` default kernels
and configure examples are in the
[build guide](../docs/原生接收与GNU_Radio构建使用说明.md).

The generated `uwb_simd_config.h` is the single source of truth for the backend,
so every translation unit (including the header-only resampler core) agrees on
it. `UWB_KERNEL_BACKEND` (for example `sync=avx2,cir=avx512`) overrides one
kernel group at a time for QA and benchmarks; it is not a production PHY switch.

## GNU Radio adapter build

```bash
cmake -S gr-uwb -B gr-uwb/build-gr -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=ON -DUWB_BUILD_EXPERIMENTS=ON
cmake --build gr-uwb/build-gr -j4
ctest --test-dir gr-uwb/build-gr --output-on-failure -j1
```

Also requires GNU Radio 3.10+, Python/pybind11 for bindings. Both component
options default to `ON`; experiments `OFF` disables a subset of benchmark and
diagnostic apps. It does not remove core regression tests.

```python
from gnuradio import uwb
# GRC: category [UWB]
```

The adapter includes detector/detector_sc16, scheduled extractor, packet writer,
realtime demodulator, radar and SIC blocks. Examples are in
[examples/README.md](examples/README.md).

## Install and link

Set `CMAKE_INSTALL_PREFIX` when configuring and run `cmake --install <build-dir>`.
A native install exports `find_package(uwb CONFIG REQUIRED)` with
`uwb::uwb_core` and `uwb::uwb_native`. A GNU Radio install retains the
`gnuradio-uwb` package. Native consumers must relink after the library split;
replacing only the old adapter shared library does not preserve their binary ABI.

The native live executable installs its taps/template resources and supports
`--res-dir`. See [tools/rx/README.md](../tools/rx/README.md) for runtime defaults
and replay examples. Defaults for older GR capture blocks remain in
`include/gnuradio/uwb/uwb_defaults.h`.

## License

GPL-3.0-or-later — see [LICENSE](../LICENSE).
