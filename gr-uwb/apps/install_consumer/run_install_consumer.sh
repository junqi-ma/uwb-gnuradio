#!/bin/sh
# External-consumer install proof (M0 review R8 / P2).
#
# Compiles and runs a consumer against NOTHING but the install prefix, then
# imports the Python module from the prefix.  Nothing here reads the source
# tree or build/test_modules, so a header or module missing from the install
# rules fails instead of being masked.
set -e

DESTDIR=${1:?usage: run_install_consumer.sh <DESTDIR>}
PREFIX_REL=${2:?usage: run_install_consumer.sh <DESTDIR> <prefix-relative-to-DESTDIR>}

ROOT=$(cd "$(dirname "$0")" && pwd)
PREFIX="$DESTDIR/$PREFIX_REL"
INCDIR="$PREFIX/include"
PYDIR=$(ls -d "$PREFIX"/lib/python*/site-packages 2>/dev/null | head -1)

echo "== install prefix =="
echo "   include    : $INCDIR"
echo "   python     : $PYDIR"

echo
echo "== files present in the prefix =="
for h in uwb_twr_types.h uwb_twr_frame.h uwb_twr_timestamp.h uwb_twr_config.h \
         uwb_twr_capability_evidence.h uwb_twr_test_output.h; do
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

echo
echo "== compile the C++ consumer against the prefix only =="
# -I points at the prefix and nothing else.  No -I to the source tree, no
# linking against the OOT build directory: if a header is not installed, this
# fails to compile.
c++ -std=c++17 -O1 -Wall -Wextra \
    -I"$INCDIR" \
    "$ROOT/install_consumer.cc" \
    -o "$ROOT/install_consumer"
echo "   compiled: $ROOT/install_consumer"

echo
echo "== run the C++ consumer =="
"$ROOT/install_consumer"

echo
echo "== import the Python module from the prefix only =="
# `twr_config` is installed as <prefix>/.../gnuradio/uwb/twr_config.py, next to
# the package __init__.py.  It is imported as a top-level `uwb` package (so
# the prefix's gnuradio/ directory is on PYTHONPATH) with CWD=/ and the source
# checkout absent, because `gnuradio` is a pkgutil-style namespace and this
# machine has a stale /usr/local/lib/python3.10/dist-packages/gnuradio/uwb
# from an earlier install that a `gnuradio.uwb` import would find FIRST.  The
# consumer asserts the resolved __file__ against the prefix, so a missing
# install is a hard failure rather than a silent fallback.
cd /
UWB_INSTALL_PREFIX_PY="$PYDIR" PYTHONPATH="$PYDIR:$PYDIR/gnuradio" python3 "$ROOT/install_consumer.py"

echo
echo "== install consumer: ALL OK =="
