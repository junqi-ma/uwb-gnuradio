# gr-uwb — Native UHD/C++ and GNU Radio UWB processing

Native C++ processing libraries and an optional GNU Radio out-of-tree (OOT)
module for **IEEE 802.15.4a/z HRP UWB** streams at
~1 GS/s host rate. The native path processes bounded IQ windows directly.
GNU Radio blocks detect or schedule-extract packets, emit PDUs, optionally
demodulate them online, and write captures for offline MATLAB processing.

This repository also holds MATLAB demodulation references (`UWB_demodulation/`),
test vectors (`testdata/`), and design notes (`docs/`, Chinese status docs at the
root). The installable OOT tree is **`gr-uwb/`**.

The current branch is **`feature/sic-research`**. Native UHD/SC16 reception,
windowed 65/48 resampling, DW3000 demodulation, target locking and SIC cores are
available alongside the GNU Radio adapter. Since `5d5ba26`, native applications
can build and run without GNU Radio or PMT. See the
[current build and architecture guide](docs/原生接收与GNU_Radio构建使用说明.md).

TWR includes offline protocol and native PHY work; hardware ranging remains
outside the validated scope. See the [TWR development records](docs/twr/开发路线与验收矩阵.md).

| Item | Value |
|------|--------|
| Module | `gr-uwb` |
| Python import | `from gnuradio import uwb` |
| C++ namespace | `gr::uwb` |
| Headers | `#include <gnuradio/uwb/...>` |
| GRC category | `[UWB]` |
| GNU Radio | **3.10+** for adapter builds; optional for native builds |
| License | GPL-3.0-or-later |
| Version | 1.0.0 |

---

## Features

- **Native receive path** — SC16 source → optional fixed-tone cancellation →
  bounded windows → fused SC16 65/48 resampling → C++ demodulation, with replay
  and burst-level DSP workers; no GNU Radio scheduler or PMT in this path.
- **Target locking** — acquire/locked synchronization and target/interference
  classification; device-timed mode currently supports replay only.

- **Unknown-time detection** — energy gate → coarse preamble → fine correlation →
  fixed capture PDU (`UwbDetector` CF32, `UwbDetectorSc16` SC16).
- **Known-period radar slots** — when `t0` / period `T` are known, use
  `UwbScheduledExtractor` so communication bursts do **not** create extra windows.
- **Async packet writer** — CF32 PDUs quantized to SC16 + JSONL metadata
  (`UwbPacketWriter`).
- **Realtime demodulator** — worker-pool message block for QM35825-style BPRF
  profiles (`UwbRealtimeDemodulator`), with Full / Top-K RAKE / Bypass soft-chip
  modes.
- **X410 / RFNoC entry** — host expects **998.4 MS/s** after FPGA 65/48 upsample
  from radio **737.28 MS/s** (see examples and `apps/x410_scheduled_capture.py`).
- **Shared production defaults** — window geometry and host sample rate live in
  `include/gnuradio/uwb/uwb_defaults.h` and are mirrored by GRC YAML defaults.

Host-side native-to-998.4 MS/s resampling is also available through the existing
65/48 and 65/32 C++ paths used by the radar applications. Supported rates depend
on the radio/FPGA profile. TWR offline configuration/protocol and native PHY libraries are present.
SIC cores and GNU Radio adapters are integrated on the current branch.

---

## Dependencies

- Common: CMake ≥ 3.8, C++17 toolchain, standalone VOLK, Threads and Boost
  `unit_test_framework` (required by the current CMake configuration).
- GNU Radio adapter: GNU Radio **3.10+** development packages (blocks, FFT,
  filter), Python 3 and pybind11.
- Native mode: GNU Radio and pybind11 are unnecessary; Python 3 enables Python QA.
- UHD: `ENABLE_UHD_BACKEND=ON` requires UHD; `AUTO` uses it when found;
  `OFF` builds replay/core paths without hardware backends.
- Optional: MATLAB for reference validation and offline decode.

For a native build on Ubuntu/Debian:

```bash
sudo apt install cmake g++ libvolk2-dev libboost-test-dev python3 libuhd-dev
```

For GNU Radio blocks and bindings, also install `gnuradio-dev python3-pybind11`.

---

## Building and installing

From the repository root, build the native UHD/C++ path:

```bash
cmake -S gr-uwb -B gr-uwb/build-native \
  -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=OFF -DUWB_BUILD_EXPERIMENTS=OFF \
  -DENABLE_UHD_BACKEND=ON
cmake --build gr-uwb/build-native -j4
ctest --test-dir gr-uwb/build-native --output-on-failure -j1
```

Use `-DENABLE_UHD_BACKEND=OFF` for a replay-only machine. To build GNU Radio
blocks, Python bindings and GRC definitions, use a separate build directory:

```bash
cmake -S gr-uwb -B gr-uwb/build-gr -DCMAKE_BUILD_TYPE=Release \
  -DUWB_BUILD_GNURADIO=ON -DUWB_BUILD_EXPERIMENTS=ON
cmake --build gr-uwb/build-gr -j4
ctest --test-dir gr-uwb/build-gr --output-on-failure -j1
```

Both component options default to `ON`; experiments `OFF` excludes some
benchmarks and diagnostics, not core QA. To install either build, set its
`CMAKE_INSTALL_PREFIX` and run `cmake --install <build-dir>`. The native package
exports `uwb::uwb_core` and `uwb::uwb_native`; native C++ consumers must relink
against these libraries after the split. GNU Radio Python APIs belong to the
adapter build.

PyBOMBS users: if `PYBOMBS_PREFIX` is set, CMake uses that install prefix
automatically.

After a GNU Radio adapter install, Companion discovers blocks under category
**`[UWB]`**. For an uninstalled adapter build, from the repository root:

```bash
export GRC_BLOCKS_PATH="$PWD/gr-uwb/grc${GRC_BLOCKS_PATH:+:$GRC_BLOCKS_PATH}"
export PYTHONPATH="$PWD/gr-uwb/build-gr/test_modules${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$PWD/gr-uwb/build-gr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

---

## Quick start (native replay)

```bash
gr-uwb/build-native/apps/uwb_dw3000_live \
  --replay /path/to/capture.sc16 --workers 4 --sync locked --period-ms 5
```

The file must be native-rate 737.28 MS/s complex SC16. Omit `--replay` to access
hardware. The C++ executable defaults to `--fused on --cir full`; a rejected
integer chip grid falls back per packet. `--mode timed` requires replay and
turns tone cancellation off. See the [receive tools guide](tools/rx/README.md)
for the Python wrapper's differing defaults and additional parameters.

## Quick start (GNU Radio Python)

```python
from gnuradio import gr, blocks, uwb

# Unknown-time detector: template is one L2-normalized SYNC symbol (CF32 file)
det = uwb.detector.make_from_file(
    "testdata/reference_preamble.bin",
    pre_trigger=2032,
    capture=200000,
    energy_threshold=0.001,
    energy_gate_decimation=100,
    coarse_decimation=4,
    sample_rate=998.4e6,  # written into PDU metadata
)
writer = uwb.packet_writer("/tmp/uwb_out", "capture", False)

tb = gr.top_block()
src = blocks.file_source(gr.sizeof_gr_complex, "testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile", False)
tb.connect(src, det)
tb.msg_connect(det, "packet", writer, "packet")
tb.run()
print("packets", writer.packets_written(), "dropped", writer.packets_dropped())
```

Known `t0` / period radar capture:

```python
ext = uwb.scheduled_extractor(
    998.4e6,          # sample_rate
    0.01,             # packet_interval_s (T)
    0,                # first_packet_sample (t0, 0-based)
    9984, 189696, 4096, 8,
    uwb.scheduled_extractor.EmitPolicy.EverySlot,
    False,
)
```

Realtime demodulator (message ports `samples` → `result` / `status`):

```python
demod = uwb.realtime_demodulator(
    "testdata/reference_preamble.bin",
    num_workers=2,
    queue_capacity=64,
    sfd_mode="4z2",          # QM35825 production; use "ieee" for MATLAB golden
    cir_rake_top_k=0,
    cir_filter_mode="auto",  # "full" | "rake" | "bypass"
)
```

---

## Blocks

| GRC / Python name | Role |
|-------------------|------|
| `uwb_energy_detector` | Sliding-window mean \|x\|² energy gate (stream) |
| `uwb_preamble_detector` | Normalized matched-filter preamble metric (stream) |
| `uwb_detector` | Full PDU detector path on **CF32** |
| `uwb_detector_sc16` | Same pipeline on **SC16** stream in; SC16 PDU out |
| `uwb_scheduled_extractor` | Fixed-interval radar slot extract (CF32) |
| `uwb_packet_writer` | PDU → `capture.iq` (SC16) + `capture.jsonl` |
| `uwb_realtime_demodulator` | Async demod on window PDUs |
| `uwb_rfnoc_upsampler` | GRC helper label for RFNoC 65/48 path |

Production default geometry (also in GRC) is defined once in
`gr-uwb/include/gnuradio/uwb/uwb_defaults.h`:

| Constant | Default | Use |
|----------|---------|-----|
| Host sample rate | `998.4e6` | Detector metadata, scheduled defaults |
| Detector pre / capture | `2032` / `200000` | Unknown-time capture |
| Scheduled pre / body / post | `9984` / `189696` / `4096` | QM35825 slot window |
| Scheduled period | `0.01` s | 100 radar slots/s |

**Detector tip:** pass `sample_rate` so PDU metadata matches the actual host rate
(default remains 998.4e6). **Scheduled tip:** when `t0/T` are known, prefer
`UwbScheduledExtractor` over energy-gated detection for every communication packet.

---

## Recommended pipelines

### Unknown packet time (communications)

```text
IQ (CF32) → UwbDetector → UwbPacketWriter
IQ (SC16) → UwbDetectorSc16 → UwbPacketWriter
```

### Known radar schedule (QM35825-style)

```text
IQ (CF32 @ 998.4 MS/s) → UwbScheduledExtractor → UwbPacketWriter
                      └→ (optional) UwbRealtimeDemodulator
```

### X410 / RFNoC (FPGA resample)

```text
X410 Radio 737.28 MS/s (SC16)
  → RFNoC Upsampler 65/48
  → RxStreamer (CF32 or SC16 @ 998.4 MS/s)
  → Scheduled Extractor  or  Detector SC16
  → Packet Writer
```

This RFNoC example requires the FPGA rate change. The native host path and
GNU Radio host/PDU resamplers also implement 65/48; they use different graphs.

---

## Examples and apps

| Path | Description |
|------|-------------|
| `gr-uwb/examples/uwb_detector_to_writer.grc` | File → detector → writer |
| `gr-uwb/examples/uwb_scheduled_file_capture.grc` | File → scheduled extract → writer |
| `gr-uwb/examples/x410_rfnoc_uwb_scheduled.grc` | X410 + RFNoC scheduled capture |
| `gr-uwb/examples/x410_rfnoc_uwb_detector_sc16.grc` | X410 SC16 detector path |
| `gr-uwb/apps/x410_scheduled_capture.py` | CLI X410 scheduled capture |
| `gr-uwb/apps/benchmark_detector*.cc` | Throughput / format benchmarks |

See `gr-uwb/examples/README.md` for parameter notes (`first_packet_sample`,
RFNoC block names, silent tail for file EOS).

Compile-check GRC without installing:

```bash
GRC_BLOCKS_PATH=$PWD/gr-uwb/grc grcc -o /tmp/uwb-grcc \
  gr-uwb/examples/uwb_detector_to_writer.grc
```

---

## Test data and MATLAB

| Path | Contents |
|------|----------|
| `testdata/reference_preamble.bin` | CF32 SYNC template |
| `testdata/uwb_code9_*.cfile` | Synthetic code-9 packets |
| `testdata/realtime_demod_golden/` | Stage-wise golden for demod core |
| `UWB_demodulation/` | Reference MATLAB pipeline (`decode_uwb.m`, …) |
| `read_uwb_packet.m` | Read writer output (`capture.iq` + JSONL) |

Algorithmic detection/demod parameters should stay consistent with the MATLAB
reference; see root docs `开发需求参考.md` and `AGENTS.md`.

---

## Development layout (`gr-uwb/`)

```text
gr-uwb/
  include/gnuradio/uwb/   Public headers (blocks + header-only cores)
  lib/                    DSP/native libraries, GR adapters and Boost QA
  grc/                    GRC YAML definitions
  python/uwb/             Python package + pybind11 bindings
  apps/                   Benchmarks and capture scripts
  examples/               Flowgraphs
  docs/                   Doxygen stubs
```

Header-only cores (`uwb_detector_core.h`, `uwb_scheduled_extractor_core.h`,
`uwb_demod_core.h`) are GNU Radio–independent for unit tests and benchmarks.

Run tests from the repository root:

```bash
ctest --test-dir gr-uwb/build-native --output-on-failure -j1
```

---

## Documentation map

| Doc | Topic |
|-----|--------|
| [`docs/原生接收与GNU_Radio构建使用说明.md`](docs/原生接收与GNU_Radio构建使用说明.md) | Current build, library layout and native receiver |
| [`开发状态.md`](开发状态.md) | Current status and historical milestones |
| [`开发需求参考.md`](开发需求参考.md) | Detection & extract requirements |
| [`开发方案_UWB实时解调.md`](开发方案_UWB实时解调.md) | Realtime demod architecture |
| [`docs/README.md`](docs/README.md) | Current entry points and historical records |
| [`docs/phase1/`](docs/phase1/) | Scheduled extractor & X410 entry |
| [`AGENTS.md`](AGENTS.md) | Contributor rules (MATLAB cross-check, block hygiene) |

---

## License

This project is licensed under the **GNU General Public License v3.0 or later**
(same as GNU Radio OOT modules generated by `gr_modtool`).

- Full text: [`LICENSE`](LICENSE)
- Source files use SPDX: `GPL-3.0-or-later`
