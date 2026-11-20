# openuwb_tx — reserved application slot (Phase 3)

This directory is intentionally empty of code.  Phase 3 only reserves the
layout for the **Simple TX** application so the repository stops growing a new
top-level concept every time TX lands.

There is no CMake target, no `main()`, and no placeholder algorithm here on
purpose: a fake TX would be indistinguishable from a working one in a smoke
test.  The TX chain will be registered when it has real profile data, a
byte-exact HRP modulator test, and a documented UHD burst contract.

Intended shape (see `docs/OpenCode执行方案_openUWB全仓库架构重构_2026-11-20.md`):

```
apps/openuwb_tx/
├── main.cc            CLI -> openuwb::runtime::TxConfig
├── tx_session.{h,cc}  schedule + burst loop (runtime/)
└── CMakeLists.txt     openuwb_tx -> openuwb_radio (+ openuwb_phy_hrp)
```

Constraints for that future work:

- reuse `include/openuwb/phy/hrp/tx/mod_core.h` for modulation;
- reuse `include/openuwb/runtime/config.h` (`TxConfig`) instead of a second
  option parser;
- link `openuwb_radio` only when UHD is present, exactly like `openuwb_rx`;
- do **not** link `openuwb_sic` or the RX runtime into the TX app.
