#!/bin/sh
# Native (GNU-Radio-free) installed-consumer proof (2026-11-15 native cleanup).
#
# Compiles uwb_native_consumer.cc against an install prefix ONLY -- no source
# tree, no build tree, no GNU Radio headers on the include path -- links the
# installed uwb_native / uwb_core archives, runs the consumer, and then proves
# with readelf/ldd that the produced binary has NO libgnuradio-* / pmt NEEDED
# entry.  A GNU Radio development package is NOT required for any of this.
#
# Usage:
#   run_native_install_consumer.sh --prefix <install-prefix>
#
# Exit codes:
#   0   consumer compiled, ran and passed; dependency check clean
#   1   consumer failed or the dependency check found GNU Radio
#   77  no install prefix supplied/available: SKIPPED (CTest SKIP_RETURN_CODE)
#
# `set -e` matters: without it a failing consumer would fall through to the
# "ALL OK" line.
set -eu

ROOT=$(cd "$(dirname "$0")" && pwd)

PREFIX=""
while [ $# -gt 0 ]; do
    case "$1" in
        --prefix)   PREFIX=${2:-}; shift 2 ;;
        --prefix=*) PREFIX=${1#--prefix=}; shift ;;
        --help|-h)  sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "run_native_install_consumer.sh: unexpected argument: $1" >&2; exit 2 ;;
    esac
done

if [ -z "$PREFIX" ]; then
    echo "SKIP: no install prefix supplied (--prefix)."
    echo "      Install the native build and re-run, e.g."
    echo "        cmake -S gr-uwb -B build-native -DUWB_BUILD_GNURADIO=OFF"
    echo "        cmake --build build-native && cmake --install build-native --prefix <prefix>"
    echo "        $0 --prefix <prefix>"
    exit 77
fi

INCDIR="$PREFIX/include"
LIBDIR=""
for d in "$PREFIX/lib" "$PREFIX/lib64"; do
    if [ -d "$d" ]; then LIBDIR="$d"; break; fi
done
if [ ! -d "$INCDIR/gnuradio/uwb" ] || [ -z "$LIBDIR" ]; then
    echo "SKIP: '$PREFIX' does not look like an install prefix" >&2
    echo "      (need $INCDIR/gnuradio/uwb and a lib dir)" >&2
    exit 77
fi
for h in uwb_rational_resampler_core.h uwb_uhd_sc16_pipeline.h \
         uwb_fixed_tone_cancel_source.h uwb_sc16_stream_source.h \
         uwb_target_lock_core.h uwb_demod_pipeline.h uwb_hrp_codec.h api.h; do
    if [ ! -f "$INCDIR/gnuradio/uwb/$h" ]; then
        echo "FAIL: installed header missing: $INCDIR/gnuradio/uwb/$h" >&2
        exit 1
    fi
done
for l in libuwb_native.a libuwb_core.a; do
    if [ ! -f "$LIBDIR/$l" ]; then
        echo "FAIL: installed archive missing: $LIBDIR/$l" >&2
        exit 1
    fi
done

WORK=$(mktemp -d "${TMPDIR:-/tmp}/uwb_native_consumer.XXXXXX")
trap 'rm -rf "$WORK"' EXIT

CXX=${CXX:-c++}
VOLK_INC=""
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists volk 2>/dev/null; then
    VOLK_INC=$(pkg-config --cflags volk)
fi

# Build the consumer through the INSTALLED package config, exactly like an
# external consumer would: find_package(uwb CONFIG REQUIRED) must close the
# whole dependency closure (Threads/Volk and, for a UHD build, UHD::UHD --
# including the UHD::UHD fallback when the UHD package does not define the
# target).  No hand-picked -l flags, no source/build tree on the path.
cat > "$WORK/CMakeLists.txt" <<EOF
cmake_minimum_required(VERSION 3.16)
project(uwb_native_consumer_check CXX)
find_package(uwb CONFIG REQUIRED)
add_executable(uwb_native_consumer "$ROOT/uwb_native_consumer.cc")
target_link_libraries(uwb_native_consumer PRIVATE uwb::uwb_native)
EOF

echo "configuring consumer via find_package(uwb CONFIG) against prefix only: $PREFIX"
cmake -S "$WORK" -B "$WORK/build" -DCMAKE_PREFIX_PATH="$PREFIX" \
    -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$WORK/build" >/dev/null

BIN="$WORK/build/uwb_native_consumer"
echo "running native consumer"
"$BIN"

# Dependency proof: no GNU Radio / PMT in the direct NEEDED set ...
if command -v readelf >/dev/null 2>&1; then
    NEEDED=$(readelf -d "$BIN" | sed -n 's/.*NEEDED.*\[\(.*\)\]/\1/p')
else
    NEEDED=$(ldd "$BIN" | awk '{print $3}' | grep -v '^$')
fi
echo "NEEDED entries: $NEEDED"
if echo "$NEEDED" | grep -Eq 'gnuradio|pmt'; then
    echo "FAIL: consumer binary depends on GNU Radio/PMT:" >&2
    echo "$NEEDED" | grep -E 'gnuradio|pmt' >&2
    exit 1
fi
# ... and none transitively either (uwb_native/uwb_core are static archives,
# so there is nothing to recurse into, but check ldd anyway).
if command -v ldd >/dev/null 2>&1; then
    if ldd "$BIN" | grep -Eq 'gnuradio|pmt'; then
        echo "FAIL: transitive dependency on GNU Radio/PMT" >&2
        ldd "$BIN" >&2
        exit 1
    fi
fi

# Header-independence proof: preprocessing the consumer must not pull in any
# GNU Radio / PMT header even though the machine has them installed.
if command -v "$CXX" >/dev/null 2>&1; then
    HDRS=$("$CXX" -std=c++17 $VOLK_INC -I"$INCDIR" -E -H -x c++ \
        "$ROOT/uwb_native_consumer.cc" -o /dev/null 2>&1 >/dev/null | \
        awk '{print $2}' | grep -v '^<' || true)
    # -H writes included files to stderr with leading dots; extract paths.
    HDRS=$("$CXX" -std=c++17 $VOLK_INC -I"$INCDIR" -E -H -x c++ \
        "$ROOT/uwb_native_consumer.cc" -o /dev/null 2>&1 >/dev/null | \
        sed -n 's/^\.* \(\/.*\)$/\1/p' | sort -u || true)
    # the project's own gnuradio/uwb headers are of course allowed
    if echo "$HDRS" | grep -E '/gnuradio/|/pmt/' | grep -v 'gnuradio/uwb' | grep -q .; then
        echo "FAIL: consumer pulls in GNU Radio/PMT headers:" >&2
        echo "$HDRS" | grep -E '/gnuradio/|/pmt/' | grep -v 'gnuradio/uwb' >&2
        exit 1
    fi
fi

echo "NATIVE INSTALL CONSUMER: ALL OK (prefix=$PREFIX, no GNU Radio)"
