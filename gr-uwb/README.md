# gr-uwb

GNU Radio **out-of-tree (OOT)** module for high-rate UWB packet detection,
scheduled radar-slot extraction, capture, and realtime demodulation.

This directory is the installable OOT tree. The parent repository also contains
MATLAB references, golden test data, and design documents — see the
[repository root README](../README.md).

## Install

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(nproc)"
ctest --output-on-failure
sudo cmake --install .
sudo ldconfig
```

**Requirements:** GNU Radio 3.10+, CMake ≥ 3.8, Boost, VOLK, Python 3 / pybind11.

## Use

```python
from gnuradio import uwb
# GRC: category [UWB]
```

| Block | Purpose |
|-------|---------|
| `detector` / `detector_sc16` | Unknown-time packet PDUs (CF32 / SC16) |
| `scheduled_extractor` | Fixed `t0/T` radar windows |
| `packet_writer` | SC16 IQ + JSONL metadata |
| `realtime_demodulator` | Async worker-pool demod |
| `energy_detector` / `preamble_detector` | Stream building blocks |

Defaults (host rate, capture geometry) are defined in
`include/gnuradio/uwb/uwb_defaults.h`.

## Examples

See `examples/` and `examples/README.md`. CLI capture helper:
`apps/x410_scheduled_capture.py`.

## License

GPL-3.0-or-later — see [`../LICENSE`](../LICENSE) for the full text.
