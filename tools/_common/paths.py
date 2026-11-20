"""Shared path discovery for the openUWB tools.

Phase 3 lifted the CMake project to the repository root, renamed the
applications to ``openuwb_*`` and standardised the preset output on
``<repo>/build/<preset>``.  Scripts that used to hard-code
``gr-uwb/build-something/apps/uwb_dw3000_live`` should ask this module
instead, so a fresh checkout works without editing the script.

Resolution order for a build directory
--------------------------------------
1. ``$OPENUWB_BUILD_DIR`` (explicit override, used by CI and ABBA drivers).
2. ``<repo>/build/<preset>`` for the presets that actually have a CMakeCache,
   preferring ``native-avx2``.
3. TODO(openuwb-compat-removal): legacy ``<repo>/gr-uwb/build*`` trees from
   checkouts made before Phase 3.  They are searched last and only because
   historical experiment drivers may still point at them.

Application lookup accepts either the official name (``openuwb_rx``) or the
pre-Phase-3 name (``uwb_dw3000_live``); the official name wins when both
exist.

TODO(openuwb-compat-removal): delete the legacy trees/names once the tools,
the docs and downstream scripts all use the openuwb_* names.
"""

from __future__ import annotations

import os
from pathlib import Path

__all__ = [
    "PRESETS",
    "APP_ALIASES",
    "repo_root",
    "build_dirs",
    "build_dir",
    "find_app",
    "find_binary",
]

PRESETS = ("native-avx2", "native-avx512", "no-uhd", "scalar-debug")

#: official binary name -> names it had before the Phase 3 rename.
#: TODO(openuwb-compat-removal): keep only the official names.
APP_ALIASES = {
    "openuwb_rx": ("uwb_dw3000_live",),
    "openuwb_pipeline_runner": ("uwb_uhd_sc16_pipeline_runner",),
    "openuwb_capture": ("uwb_uhd_sc16_capture",),
    "openuwb_twr_fake_demo": ("twr_fake_demo",),
    "openuwb_twr_native_phy_demo": ("twr_native_phy_demo",),
}

#: Where CMake drops executables.  ``apps`` is the Phase 3 layout; the second
#: entry is the legacy nested tree.  TODO(openuwb-compat-removal).
_BINARY_SUBDIRS = ("apps", "gr-uwb/apps")


def repo_root() -> Path:
    """Repository root (``tools/_common/paths.py`` -> three levels up)."""
    return Path(__file__).resolve().parents[2]


def build_dirs() -> list[Path]:
    """Candidate build directories, most-preferred first."""
    override = os.environ.get("OPENUWB_BUILD_DIR", "").strip()
    if override:
        return [Path(override).expanduser().resolve()]
    root = repo_root()
    dirs = [root / "build" / preset for preset in PRESETS]
    # TODO(openuwb-compat-removal): legacy build trees.
    dirs.extend(sorted(root.glob("gr-uwb/build*")))
    return dirs


def _is_build_dir(path: Path) -> bool:
    return (path / "CMakeCache.txt").is_file()


def build_dir(required: bool = True) -> Path | None:
    """First configured build directory, or ``None`` when ``required`` is false."""
    for candidate in build_dirs():
        if _is_build_dir(candidate):
            return candidate
    if not required:
        return None
    raise SystemExit(
        "no configured build directory found; configure one with "
        "`cmake --preset native-avx2` or set OPENUWB_BUILD_DIR"
    )


def find_binary(name: str, required: bool = True) -> Path | None:
    """Locate an executable by exact name across the known build trees."""
    for directory in build_dirs():
        for sub in _BINARY_SUBDIRS:
            candidate = directory / sub / name
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return candidate
        candidate = directory / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    if not required:
        return None
    raise SystemExit(
        f"cannot find executable {name!r} in any of: "
        + ", ".join(str(d) for d in build_dirs())
        + " (build it, or set OPENUWB_BUILD_DIR)"
    )


def find_app(name: str, required: bool = True) -> Path | None:
    """Locate an application, accepting its official or legacy name."""
    names = (name,) + APP_ALIASES.get(name, ())
    # If a legacy name was asked for, also try the official one.
    for official, legacy_names in APP_ALIASES.items():
        if name in legacy_names:
            names = (official,) + names
            break
    for candidate_name in names:
        found = find_binary(candidate_name, required=False)
        if found is not None:
            return found
    if not required:
        return None
    raise SystemExit(
        f"cannot find application {name!r} (tried {', '.join(names)}) in any "
        "known build tree; build it with `cmake --build --preset native-avx2` "
        "or set OPENUWB_BUILD_DIR"
    )
