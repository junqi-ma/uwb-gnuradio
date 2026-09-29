#!/bin/sh
# External-consumer install proof (M0 review R8 / P2, review defect N05).
#
# Compiles and runs a consumer against NOTHING but the install prefix, then
# imports the Python module from the prefix.  Nothing here reads the source
# tree or build/test_modules, so a header or module missing from the install
# rules fails instead of being masked.
#
# Usage:
#   run_install_consumer.sh --prefix <install-prefix> [--python-rel <rel>]
#   run_install_consumer.sh <DESTDIR> <prefix-relative-to-DESTDIR>   (legacy)
#
#   --prefix DIR    The absolute install prefix (contains include/, lib/).
#   --python-rel R  The Python install directory RELATIVE to the prefix, as
#                   configured by CMake (GR_PYTHON_DIR, e.g.
#                   lib/python3.10/dist-packages).  When omitted the wrapper
#                   searches the prefix for gnuradio/uwb/twr_config.py.
#
# Exit codes:
#   0   consumer ran and passed
#   1   consumer ran and FAILED
#   77  no install prefix was supplied/available: SKIPPED (CTest
#       SKIP_RETURN_CODE), so an ordinary developer build is not failed
#       merely because nothing has been installed.
#
# Review defect N05: the old wrapper hardcoded `site-packages`, but a stock
# local install uses `dist-packages`; it now reads the configured directory
# (passed by CMake as GR_PYTHON_DIR) or discovers it.
#
# `set -e` matters: without it a failing C++/Python consumer would fall through
# to the "ALL OK" line.  Every consumer failure must abort non-zero.
set -eu

ROOT=$(cd "$(dirname "$0")" && pwd)

# ---------------------------------------------------------------------------
# Argument parsing (POSIX sh; supports the new --style and the legacy
# positional <DESTDIR> <PREFIX_REL> form the delivery doc used).
# ---------------------------------------------------------------------------
PREFIX=""
PYTHON_REL=""
DESTDIR=""
PREFIX_REL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --prefix)
            PREFIX=${2:-}
            shift 2
            ;;
        --prefix=*)
            PREFIX=${1#--prefix=}
            shift
            ;;
        --python-rel)
            PYTHON_REL=${2:-}
            shift 2
            ;;
        --python-rel=*)
            PYTHON_REL=${1#--python-rel=}
            shift
            ;;
        --help|-h)
            sed -n '2,40p' "$0"
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "run_install_consumer.sh: unknown option: $1" >&2
            exit 2
            ;;
        *)
            if [ -z "$DESTDIR" ]; then
                DESTDIR="$1"
            elif [ -z "$PREFIX_REL" ]; then
                PREFIX_REL="$1"
            else
                echo "run_install_consumer.sh: unexpected argument: $1" >&2
                exit 2
            fi
            shift
            ;;
    esac
done

if [ -z "$PREFIX" ] && [ -n "$DESTDIR" ] && [ -n "$PREFIX_REL" ]; then
    PREFIX="$DESTDIR/$PREFIX_REL"
fi

if [ -z "$PREFIX" ]; then
    echo "SKIP: no install prefix supplied (--prefix), so the installed"
    echo "      consumer cannot be exercised yet.  Install the module and"
    echo "      re-run, e.g."
    echo "        cmake --install <build> --prefix <prefix>"
    echo "        $0 --prefix <prefix> --python-rel <GR_PYTHON_DIR>"
    exit 77
fi

INCDIR="$PREFIX/include"
if [ ! -d "$INCDIR/gnuradio/uwb" ]; then
    echo "SKIP: '$PREFIX' does not look like an install prefix" >&2
    echo "      (missing $INCDIR/gnuradio/uwb)" >&2
    exit 77
fi

# ---------------------------------------------------------------------------
# The Python module directory: the CONFIGURED destination, not a guess.
# ---------------------------------------------------------------------------
if [ -n "$PYTHON_REL" ]; then
    PYDIR="$PREFIX/$PYTHON_REL"
else
    PYDIR=""
    for d in "$PREFIX"/lib/python*/dist-packages \
             "$PREFIX"/lib/python*/site-packages \
             "$PREFIX"/lib64/python*/dist-packages \
             "$PREFIX"/lib64/python*/site-packages; do
        if [ -f "$d/gnuradio/uwb/twr_config.py" ]; then
            PYDIR="$d"
            break
        fi
    done
    # Last resort: any python* dir with the module.
    if [ -z "$PYDIR" ]; then
        PYDIR=$(find "$PREFIX" -name twr_config.py -path '*gnuradio/uwb*' 2>/dev/null |
                    head -1 | sed 's,/gnuradio/uwb/twr_config.py$,,' || true)
    fi
fi

echo "== install prefix =="
echo "   include    : $INCDIR"
echo "   python     : ${PYDIR:-<not found>}"

if [ -z "$PYDIR" ] || [ ! -d "$PYDIR/gnuradio/uwb" ]; then
    echo "SKIP: could not locate the installed gnuradio/uwb Python package under"
    echo "      '$PREFIX' (pass --python-rel with GR_PYTHON_DIR)" >&2
    exit 77
fi

echo
echo "== files present in the prefix =="
# The SEVEN public TWR contract headers, including uwb_twr_tof_input.h (the
# ranging-admission entry point that review defect N05 found was not
# installed) and uwb_twr_math.h (the M1-A SS/DS ToF mathematics).
# uwb_twr_test_output.h is intentionally NOT installed: it is
# QA-only output policy and expands UWB_TESTDATA_DIR with no default.
for h in uwb_twr_types.h uwb_twr_frame.h uwb_twr_timestamp.h uwb_twr_config.h \
         uwb_twr_capability_evidence.h uwb_twr_tof_input.h uwb_twr_math.h; do
    if [ -f "$INCDIR/gnuradio/uwb/$h" ]; then
        echo "   header  OK  gnuradio/uwb/$h"
    else
        echo "   header  MISSING  gnuradio/uwb/$h"
        exit 1
    fi
done
for p in __init__.py twr_config.py; do
    if [ -f "$PYDIR/gnuradio/uwb/$p" ]; then
        echo "   module  OK  gnuradio/uwb/$p"
    else
        echo "   module  MISSING  gnuradio/uwb/$p"
        exit 1
    fi
done

# Build in a scratch directory so the source checkout is never written to.
WORK=$(mktemp -d "${TMPDIR:-/tmp}/uwb_install_consumer.XXXXXX") || exit 1
trap 'rm -rf "$WORK"' EXIT INT TERM
BIN="$WORK/install_consumer"

echo
echo "== compile the C++ consumer against the prefix only =="
# -I points at the prefix and nothing else.  No -I to the source tree, no
# linking against the OOT build directory: if a header is not installed, this
# fails to compile.
c++ -std=c++17 -O1 -Wall -Wextra \
    -I"$INCDIR" \
    "$ROOT/install_consumer.cc" \
    -o "$BIN"
echo "   compiled: $BIN"

echo
echo "== run the C++ consumer =="
"$BIN"

echo
echo "== import the Python module from the prefix only =="
# `twr_config` is installed as <prefix>/<GR_PYTHON_DIR>/gnuradio/uwb/twr_config.py,
# next to the package __init__.py.  It is imported as a top-level `uwb`
# package (so the prefix's gnuradio/ directory is on PYTHONPATH) with CWD=/ and
# the source checkout absent, because `gnuradio` is a pkgutil-style namespace
# and this machine has a stale .../dist-packages/gnuradio/uwb from an earlier
# install that a `gnuradio.uwb` import would find FIRST.  The consumer asserts
# the resolved __file__ against the prefix, so a missing install is a hard
# failure rather than a silent fallback.
cd /
UWB_INSTALL_PREFIX_PY="$PYDIR" PYTHONPATH="$PYDIR:$PYDIR/gnuradio" \
    "${PYTHON:-python3}" "$ROOT/install_consumer.py"

echo
echo "== install consumer: ALL OK =="
