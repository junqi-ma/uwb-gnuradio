"""External consumer for the INSTALLED uwb.twr_config module.

Companion to install_consumer.cc.  Run with PYTHONPATH set to the install
prefix's configured Python install directory and the source checkout
deliberately NOT on the path, so
a twr_config.py missing from GR_PYTHON_INSTALL(...) is a hard failure rather
than something the source tree quietly satisfies.

Checks the M0 contract the installed module must honour:
  * it imports at all, and from the PREFIX;
  * the capability whitelist is populated and still DEFAULT DENY (128 SYNC is
    rejected, with a reason);
  * it does NOT drag in GNU Radio (its own design contract: config and result
    consumption in Python, per-packet timing in C++);
  * the public API surface named by REQ-API-01 is present.

The module is installed as `gnuradio/uwb/twr_config.py` inside the prefix's
configured Python install directory, next to the package `__init__.py` and the
pybind11 extension -- i.e. in exactly the directory `import uwb.twr_config` /
`from gnuradio.uwb import twr_config` resolves through.  That directory is
`dist-packages` on a stock local install (review defect N05: the shell wrapper
used to hardcode `site-packages`); the wrapper/CTest pass the actually
configured directory rather than guessing.

Why this consumer imports it as a TOP-LEVEL `uwb` package rather than as
`gnuradio.uwb`:

  * `gnuradio` is a pkgutil-style namespace spread over several roots.  This
    machine also has a stale /usr/local/lib/python3.10/dist-packages/gnuradio/uwb
    from an earlier `sudo install`, and that root is found FIRST, so a
    `gnuradio.uwb` import would silently exercise the OLD install and prove
    nothing about this prefix.  Importing `uwb` directly from the prefix's
    configured Python install directory puts the prefix in control of the
    name resolution.
  * `twr_config` is deliberately importable with NO GNU Radio and NO pybind11
    extension (its own design contract).  Going through `gnuradio.uwb.__init__`
    would import the extension first, which is the opposite of what is being
    tested.

The resolved `__file__` is asserted against the prefix, so the import cannot
silently come from the source checkout or from the stale system install.
"""

import os
import sys

PREFIX = os.environ.get("UWB_INSTALL_PREFIX_PY", "")
assert PREFIX, "UWB_INSTALL_PREFIX_PY is not set"
# The shell wrapper builds paths by concatenation, which can leave a doubled
# separator.  Normalise both sides before comparing.
PREFIX = os.path.normpath(PREFIX)

import uwb
import uwb.twr_config as T

mod = sys.modules["uwb.twr_config"]
print("  imported from : %s" % mod.__file__)
assert "gr-uwb/python" not in mod.__file__, (
    "imported from the SOURCE tree, not the install prefix: %s" % mod.__file__)
assert mod.__file__.startswith(PREFIX), (
    "imported from %s, not from the install prefix %s"
    % (mod.__file__, PREFIX))
print("  package       : %s" % uwb.__file__)

caps = T.capabilities()
print("  schema=%s profile=%s channels=%s sync_reps=%s phy_rows=%d"
      % (caps.schema_version, caps.profile_version, caps.channels,
         caps.sync_repetitions, len(caps.phy_matrix)))

# The M0.1 measured profile must be intact after installation.  The schema is
# CURRENT (`twr-config/2`); a stale "twr-config/1" expectation here is review
# defect N05.
assert caps.schema_version == T.SCHEMA_VERSION, (
    "installed module advertises %r but its own SCHEMA_VERSION is %r"
    % (caps.schema_version, T.SCHEMA_VERSION))
assert caps.schema_version == "twr-config/2", caps.schema_version
assert T.LEGACY_SCHEMA_VERSIONS == ("twr-config/1",), T.LEGACY_SCHEMA_VERSIONS
assert "BREAKING" in T.SCHEMA_V1_MIGRATION_REASON, T.SCHEMA_V1_MIGRATION_REASON
assert caps.channels == [5], caps.channels
assert caps.sync_repetitions == [16, 64], caps.sync_repetitions
assert len(caps.phy_matrix) == 48, len(caps.phy_matrix)
assert caps.sts_supported is False
assert caps.ranging_bit_required is True

# DEFAULT DENY: 128 SYNC is refused, and the refusal is explained.
lk = caps.lookup_phy(737280000.0, 9, 128, T.SfdMode.R4Z2, 127, True)
assert lk.allowed is False, "128 SYNC was admitted by the installed module"
print("  128 SYNC rejected with: %s" % lk.reason)
assert lk.reason, "the rejection carries no reason"

# The admitted combination still works.
ok = caps.lookup_phy(737280000.0, 9, 64, T.SfdMode.R4Z2, 127, True)
assert ok.allowed is True, "64 SYNC/4z2/code9 was rejected: %s" % ok.reason

# `twr_config` must be usable with NO GNU Radio and NO pybind11 extension:
# that is the M0 design split (REQ-API-01 -- Python does configuration and
# result consumption, per-packet timing is C++/hardware).  Importing the
# package `uwb` runs its __init__, which DOES load the extension when it can,
# so the standalone property is checked in a clean subprocess that imports
# ONLY the module file, with the package __init__ bypassed.
import subprocess

probe = r"""
import importlib.util, sys
spec = importlib.util.spec_from_file_location(
    "twr_config_standalone", %r)
mod = importlib.util.module_from_spec(spec)
sys.modules["twr_config_standalone"] = mod      # dataclasses() needs this
spec.loader.exec_module(mod)
assert not [m for m in sys.modules if m.startswith("gnuradio")], \
    sorted(m for m in sys.modules if m.startswith("gnuradio"))
assert "uwb_python" not in sys.modules
caps = mod.capabilities()
print("standalone_ok rows=%%d" %% len(caps.phy_matrix))
""" % (mod.__file__,)

res = subprocess.run([sys.executable, "-c", probe],
                     capture_output=True, text=True, cwd="/")
assert res.returncode == 0, (
    "uwb.twr_config is not standalone:\n%s%s" % (res.stdout, res.stderr))
print("  twr_config alone (no __init__, no extension, no GNU Radio): %s"
      % res.stdout.strip())

# The public surface REQ-API-01 names.
for name in ("validate", "effective_config", "to_json_string",
             "from_json_string", "TwrConfigSnapshot", "capabilities",
             "apply_calibration_once", "PerMessageOverrides", "ExchangeGate"):
    assert name in T.__all__, "missing public name: %s" % name
print("  public API present: validate/effective_config/JSON/snapshot")

print("  PYTHON CONSUMER OK")
