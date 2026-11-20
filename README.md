# OpenUWB — 802.15.4a/z-Compatible UWB Software Baseband

English | [简体中文](README.zh-CN.md)

> [!NOTE]
> **Work in Progress**: This project is under active early development. Interfaces, CLI options, and features are evolving rapidly; API stability is not guaranteed, and no performance benchmarks are listed here yet.

## Motivation & Vision

Most existing UWB research relies on dedicated commercial transceivers such as the DW1000 and DW3000. Because their baseband processing is fixed in hardware and closed firmware, researchers have very limited freedom to inspect or modify physical-layer waveforms, synchronization, channel estimation, interference cancellation, or protocol timing.

Our goal is to implement an open, software-defined UWB radio on modern wide-bandwidth SDRs (such as the USRP X410 and X420) while maintaining IEEE 802.15.4a/z compliance so that it can interoperate directly with commercial standard UWB devices (e.g., NXP, Qorvo, Apple).

We believe that bringing full SDR programmability together with commercial standard interoperability will unlock many new research opportunities across UWB communication, wireless sensing, ranging (TWR), and physical-layer signal processing.

## Development Timeline (Core Milestones)

- **2026-08 — Real-Time Demodulation, Commercial Interop & Windowed Resampling**
  - Established the windowed native-rate capture (`737.28 MS/s`) + per-window `65/48` polyphase resampling (`998.4 MS/s` working rate) architecture for blind acquisition, periodic slot locking, and decoding of real QM35825 and DW1000 captures.
  - Aligned HRP BPRF demodulation (preamble codes, SFD search, CFO compensation) with MATLAB references.
- **2026-09 — X410 Timed Echo CIR Sensing, Dual-TX Interference & DPDK**
  - Built timed echo-CIR sensing on USRP X410 (CG400/CG600) with per-repetition CIR output, runtime frequency sweep/hopping, and UHD DPDK support.
  - Implemented C++ dual-TX timed transmission (sensing + synchronized interference generation) with continuous multi-TX buffer scheduling.
- **2026-09 ~ 2026-10 — SS-TWR / DS-TWR Protocol Core & Offline Native PHY Loopback**
  - Delivered the UHD-free TWR configuration contract, ToF math library (verified against a MATLAB oracle), SS/DS-TWR protocol FSM, and offline native PHY loopback (hardware ToA/ranging is not yet implemented).
- **2026-11 (Mid) — Low-SNR Demodulation, SIMD Sparse Correlation & Unified SIC**
  - Enhanced low-SNR acquisition and decoding (folded sync, discrete SFD grid rescue, BPM soft decisions) and added AVX2 / AVX-512 sparse ternary correlation kernels.
  - Unified single-packet Successive Interference Cancellation (SIC: decode → CIR estimation → waveform reconstruction → cancellation) with the standard demodulation pipeline (runtime integration pending).
- **2026-11-20 — Native C++17 Refactor & Rename to OpenUWB (Phase 0–4)**
  - Removed the runtime dependency on GNU Radio (archived under `archive/gnuradio-adapter`) and promoted the native C++17 / UHD codebase to the repository root.
  - Renamed the project to **OpenUWB**, introducing the `openuwb` namespace, unidirectional domain libraries (`support → dsp → phy_hrp → {sic, sensing, twr_core}`, with `openuwb_radio` isolating UHD and `openuwb_runtime` assembling the pipeline), and unified `openuwb_*` applications.

## Repository Navigation

For current build instructions and detailed status (currently documented in Chinese):

- Build & usage guide: [docs/原生UHD与C++构建使用说明.md](docs/原生UHD与C++构建使用说明.md)
- Detailed development status: [开发状态.md](开发状态.md)
- Documentation index: [docs/README.md](docs/README.md)
