#!/usr/bin/env python3
"""Rebuild /tmp/uhd_eal_noret so PyUHD can load on this AVX2-only host.

UHD 4.6 is linked against DPDK 21. Those .so files contain AVX-512
constructors. The i7-12700 has no AVX-512, so `import uhd` SIGILLs
even when /etc/uhd/uhd.conf uses kernel UDP.

This copies the DPDK libs UHD needs and patches two load-time
functions to `endbr64; ret`. Radar app bootstrap then puts this
directory first on LD_LIBRARY_PATH.

See docs/本机UHD_DPDK_AVX512问题与绕过.md.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

DST = Path("/tmp/uhd_eal_noret")
SRC_DIRS = (
    Path("/usr/local/lib/x86_64-linux-gnu"),
    Path("/usr/local/lib"),
    Path("/usr/lib/x86_64-linux-gnu"),
)
NEEDED = (
    "librte_eal.so.21.0",
    "librte_hash.so.21.0",
    "librte_ethdev.so.21.0",
    "librte_mbuf.so.21.0",
    "librte_mempool.so.21.0",
    "librte_ring.so.21.0",
    "librte_kvargs.so.21.0",
    "librte_telemetry.so.21.0",
    "librte_rcu.so.21.0",
    "librte_net.so.21.0",
    "librte_meter.so.21.0",
)
# (library file, symbol) — both are AVX-512 and run on .so load.
PATCH_SYMS = (
    ("librte_eal.so.21.0", "rte_srand"),
    ("librte_ethdev.so.21.0", "eth_dev_init_cb_lists"),
)
ENDBR64 = bytes.fromhex("f30f1efa")


def find_src(name: str) -> Path:
    for d in SRC_DIRS:
        p = d / name
        if p.is_file():
            return p
    raise SystemExit("missing %s (looked in %s)" % (name, SRC_DIRS))


def symbol_offset(lib: Path, name: str) -> int:
    out = subprocess.check_output(["nm", str(lib)], text=True, stderr=subprocess.DEVNULL)
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] == name:
            return int(parts[0], 16)
    raise SystemExit("symbol %s not in %s" % (name, lib))


def patch_ret(lib: Path, off: int) -> None:
    data = bytearray(lib.read_bytes())
    if off + 5 > len(data):
        raise SystemExit("%s offset %#x past EOF" % (lib, off))
    if data[off:off + 4] != ENDBR64:
        raise SystemExit("%s:%#x expected endbr64, got %s" % (
            lib, off, data[off:off + 8].hex()))
    data[off + 4] = 0xc3
    lib.write_bytes(data)


def main() -> int:
    if os.geteuid() == 0:
        print("refusing to run as root", file=sys.stderr)
        return 2
    if DST.exists():
        shutil.rmtree(DST)
    DST.mkdir(parents=True)
    for name in NEEDED:
        src = find_src(name)
        shutil.copy2(src, DST / name)
        base = name.replace(".21.0", "")
        os.symlink(name, DST / (base + ".21"))
        os.symlink(base + ".21", DST / base)
        print("copied", src, flush=True)
    for name, sym in PATCH_SYMS:
        lib = DST / name
        off = symbol_offset(lib, sym)
        patch_ret(lib, off)
        print("patched %s %s @ %#x -> endbr64; ret" % (name, sym, off),
              flush=True)

    env = os.environ.copy()
    env["PYTHONPATH"] = "/usr/local/lib/python3.10/site-packages"
    env["LD_LIBRARY_PATH"] = str(DST) + (
        (":" + env["LD_LIBRARY_PATH"]) if env.get("LD_LIBRARY_PATH") else "")
    r = subprocess.run(
        [sys.executable, "-c", "import uhd; print('import_uhd_ok', uhd.__file__)"],
        env=env)
    if r.returncode != 0:
        print("import uhd still failed; see docs/本机UHD_DPDK_AVX512问题与绕过.md",
              file=sys.stderr)
        return r.returncode or 1
    print("ready", DST, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
