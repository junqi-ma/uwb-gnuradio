#!/usr/bin/env bash
# Verify the INSTALLED gr-uwb TWR consumer deliverable (review defect N05).
#
# This is the self-contained release-gate run for the installed public API.
# It does NOT rebuild the project: it stages a DESTDIR install from the
# EXISTING build tree into an isolated prefix, then proves that
#
#   * uwb_twr_tof_input.h (the ranging-admission entry header) is installed;
#   * a throwaway external C++ consumer can include it and CALL the admission
#     entry points against the prefix only;
#   * the repository's full C++ consumer compiles and runs against the prefix;
#   * the Python consumer imports twr_config from the prefix's ACTUAL
#     configured Python directory (dist-packages, not a site-packages guess)
#     and validates the CURRENT schema (twr-config/2).
#
# Usage:
#   tools/twr/verify_install_consumer.sh [install-prefix]
#
# The prefix defaults to a temp directory under /tmp/opencode.  When it is a
# temp path it is removed first, so repeated runs are clean and idempotent; a
# non-temp prefix is installed over, never deleted.
#
# Environment overrides:
#   UWB_BUILD_DIR   The configured build tree (default <repo>/gr-uwb/build).
#   PYTHON          Python interpreter (default python3).
#   CXX             C++ compiler (default c++).
#
# Known build-tree caveat: a `sudo install` in this checkout left a
# root-owned gr-uwb/build/install_manifest.txt.  Passing --component
# Unspecified makes CMake write install_manifest_Unspecified.txt instead of
# touching that unwritable file, so no sudo and no build-tree chmod is needed.
# This script never writes to a tracked source file.
#
# Exit status: 0 on success; non-zero with a clear message on any failure.

set -euo pipefail

REPO_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
BUILD_DIR=${UWB_BUILD_DIR:-"$REPO_ROOT/gr-uwb/build"}
PYTHON=${PYTHON:-python3}
CXX=${CXX:-c++}
CONSUMER_DIR="$REPO_ROOT/gr-uwb/apps/install_consumer"

PREFIX=${1:-/tmp/opencode/uwb_install_consumer}

fail() {
    echo "VERIFY FAILED: $*" >&2
    exit 1
}

# Strip any trailing slash so a prefix like "/tmp/" cannot match the temp-path
# case below and cause a delete of /tmp.  "/" collapses to "".
while [ "${PREFIX%/}" != "$PREFIX" ]; do
    PREFIX=${PREFIX%/}
done
if [ -z "$PREFIX" ] || [ "$PREFIX" = "/" ]; then
    fail "refusing to use '$PREFIX' as the install prefix"
fi

echo "== verify_install_consumer =="
echo "   repo       : $REPO_ROOT"
echo "   build tree : $BUILD_DIR"
echo "   prefix     : $PREFIX"

# ---------------------------------------------------------------------------
# Preconditions: an existing configured build tree with the library built.
# We deliberately do not build here; the orchestrator owns the build tree.
# ---------------------------------------------------------------------------
[ -f "$BUILD_DIR/CMakeCache.txt" ] || fail \
    "no configured build tree at $BUILD_DIR (set UWB_BUILD_DIR or configure it)"
[ -d "$BUILD_DIR/lib" ] || fail "no $BUILD_DIR/lib"
if ! ls "$BUILD_DIR"/lib/libgnuradio-uwb.so* >/dev/null 2>&1; then
    fail "libgnuradio-uwb.so is not built in $BUILD_DIR (build it first; this script does not build)"
fi

# The configured Python install directory, read from the generated install
# rules (this is what GR_PYTHON_DIR produced).  NOT a site-packages guess.
PYTHON_REL=$(sed -n \
    's#.*DESTINATION "${CMAKE_INSTALL_PREFIX}/\(.*\)/gnuradio/uwb".*#\1#p' \
    "$BUILD_DIR/python/uwb/cmake_install.cmake" 2>/dev/null | head -1 || true)
if [ -z "$PYTHON_REL" ]; then
    PYTHON_REL=$(grep -oE 'lib(64)?/python[0-9]+(\.[0-9]+)?/(dist|site)-packages' \
        "$BUILD_DIR/python/uwb/cmake_install.cmake" 2>/dev/null | head -1 || true)
fi
echo "   python dir : ${PYTHON_REL:-<will discover>}"

# ---------------------------------------------------------------------------
# Stage the install into the isolated prefix.  Clean only temp prefixes.
# ---------------------------------------------------------------------------
case "$PREFIX" in
    /tmp/opencode/*|/tmp/*|"${TMPDIR:-/tmp}"/*)
        rm -rf "$PREFIX"
        ;;
    *)
        echo "   note: $PREFIX is not a temp path; installing over it (not deleting)"
        ;;
esac
mkdir -p "$PREFIX"

# Avoid a root-owned stale component manifest if one exists.
MANIFEST="$BUILD_DIR/install_manifest_Unspecified.txt"
if [ -e "$MANIFEST" ] && [ ! -w "$MANIFEST" ]; then
    rm -f "$MANIFEST" || fail "cannot remove non-writable $MANIFEST"
fi

echo
echo "== DESTDIR install into the isolated prefix =="
# --component Unspecified selects exactly the default (untagged) install rules
# while redirecting the manifest away from the root-owned install_manifest.txt.
# The DESTDIR form is tried first; a --prefix fallback covers any CMake that
# does not apply DESTDIR to an overridden root prefix.
if ! DESTDIR="$PREFIX" cmake --install "$BUILD_DIR" --prefix / --component Unspecified; then
    echo "   DESTDIR install failed; retrying with --prefix $PREFIX"
    cmake --install "$BUILD_DIR" --prefix "$PREFIX" --component Unspecified
fi
HDR="$PREFIX/include/gnuradio/uwb/uwb_twr_tof_input.h"
if [ ! -f "$HDR" ]; then
    echo "   header not at $HDR after DESTDIR install; retrying with --prefix $PREFIX"
    cmake --install "$BUILD_DIR" --prefix "$PREFIX" --component Unspecified
fi

# ---------------------------------------------------------------------------
# The admission header MUST be installed (review defect N05).
# ---------------------------------------------------------------------------
for h in uwb_twr_types.h uwb_twr_frame.h uwb_twr_timestamp.h uwb_twr_config.h \
         uwb_twr_capability_evidence.h uwb_twr_tof_input.h uwb_twr_math.h; do
    [ -f "$PREFIX/include/gnuradio/uwb/$h" ] || fail \
        "installed public header is missing: include/gnuradio/uwb/$h"
done
echo "   all seven public TWR headers present"

# The QA-only header must NOT be installed (it cannot compile standalone).
if [ -f "$PREFIX/include/gnuradio/uwb/uwb_twr_test_output.h" ]; then
    fail "uwb_twr_test_output.h was installed, but it is QA-only and expands \
UWB_TESTDATA_DIR with no default; it must not be a public install header"
fi
echo "   QA-only uwb_twr_test_output.h correctly absent"

# ---------------------------------------------------------------------------
# Throwaway external consumer: include ONLY the admission header and call the
# admission entry points.  Its include path is the prefix and nothing else.
# ---------------------------------------------------------------------------
PYDIR="$PREFIX/${PYTHON_REL:-}"
if [ -z "$PYTHON_REL" ] || [ ! -f "$PYDIR/gnuradio/uwb/twr_config.py" ]; then
    PYDIR=$(find "$PREFIX" -name twr_config.py -path '*gnuradio/uwb*' 2>/dev/null |
                head -1 | sed 's,/gnuradio/uwb/twr_config.py$,,' || true)
fi
[ -n "$PYDIR" ] && [ -f "$PYDIR/gnuradio/uwb/twr_config.py" ] || fail \
    "could not locate the installed gnuradio/uwb/twr_config.py under $PREFIX"
echo "   python package dir: $PYDIR"

WORK=$(mktemp -d "${TMPDIR:-/tmp}/uwb_verify_install.XXXXXX")
trap 'rm -rf "$WORK"' EXIT INT TERM

cat > "$WORK/tiny_admission_consumer.cc" <<'EOF'
// Throwaway external consumer: proves uwb_twr_tof_input.h is installed AND
// usable against the prefix alone.  No source tree, no build tree, no link.
#include <gnuradio/uwb/uwb_twr_math.h>
#include <gnuradio/uwb/uwb_twr_tof_input.h>

#include <cstdio>
#include <string>

using namespace gr::uwb::twr;

int main()
{
    ClockDomain d;
    if (!ClockDomain::make("verify_ch5_uus", 499.2e6 * 128.0, 1u, 40u, d)) {
        std::printf("FAIL: cannot build a clock domain\n");
        return 1;
    }

    // R6 / default deny: a host capture coordinate is never a ranging input,
    // whatever correction bits it claims.
    Timestamp raw;
    if (!Timestamp::from_ticks(1000, d, TimestampMarker::UhdRxFirstIqSample,
                               TimestampSource::HardwareMeasured, kCorrectionAll, raw)) {
        std::printf("FAIL: cannot build a raw timestamp\n");
        return 1;
    }
    RangeAdmissionContext empty; // no calibration, no reference, no first path
    const RangeAdmission denied = admit_range_capable_time(raw, empty);
    if (denied.admitted || denied.value.has_value()) {
        std::printf("FAIL: a default context admitted a raw sample\n");
        return 1;
    }

    // A fully evidenced RMARKER pair is admitted and yields the exact interval.
    const uint32_t rx_bits = kCorrectionRxSampleToFirstPath | kCorrectionWindowCrop |
                             kCorrectionSampleRateConversion |
                             kCorrectionFirstPathFraction | kCorrectionWaveformGeometry |
                             kCorrectionRmarkerOffset | kCorrectionFirstPathQualityGate;
    const uint32_t tx_bits = kCorrectionWaveformGeometry | kCorrectionTxCommandToAir |
                             kCorrectionDelayedTxQuantization;
    Timestamp rx, tx;
    if (!Timestamp::from_ticks(9000, d, TimestampMarker::RmarkerRx,
                               TimestampSource::HardwareMeasured, rx_bits, rx) ||
        !Timestamp::from_ticks(5000, d, TimestampMarker::RmarkerTx,
                               TimestampSource::HardwareMeasured, tx_bits, tx)) {
        std::printf("FAIL: cannot build the RMARKER timestamps\n");
        return 1;
    }
    const std::string cal_id = "verify-cal-r1";
    if (apply_calibration_ticks(rx, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied ||
        apply_calibration_ticks(tx, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied) {
        std::printf("FAIL: cannot apply the calibration exactly once\n");
        return 1;
    }
    CalibrationStamp cal;
    cal.id = cal_id;
    cal.calibrated_epoch = 1u;
    cal.valid_from_ticks = 1000;
    cal.valid_until_ticks = 2000;
    CalibrationApplication app;
    app.calibration_id = cal_id;
    app.result = CalibrationResult::Applied;
    cal.applications.push_back(app);

    RangeAdmissionContext ctx;
    ctx.calibration = &cal;
    ctx.reference_ticks = 1500;
    ctx.reference_ticks_recorded = true;
    ctx.rx_first_path = FirstPathQuality::passed(18.5, 9.0, 0.82);

    const RangingIntervalAdmission pair = admit_ranging_interval(rx, tx, ctx);
    if (!pair.admitted || !pair.value.has_value() ||
        pair.value->interval().ticks != 4000) {
        std::printf("FAIL: a fully evidenced RMARKER pair was not admitted\n");
        return 1;
    }
    std::printf("TINY CONSUMER OK: admit_ranging_interval -> exact %lld ticks\n",
                static_cast<long long>(pair.value->interval().ticks));

    // M1-A: the installed math header computes the hand-checked SS example.
    // Common clock (k=1 STATED), RA = 2000, DB = 1000 -> ToF = 500 A ticks.
    Timestamp ra_l, ra_e, db_l, db_e;
    if (!Timestamp::from_ticks(2000, d, TimestampMarker::RmarkerRx,
                               TimestampSource::HardwareMeasured, rx_bits, ra_l) ||
        !Timestamp::from_ticks(0, d, TimestampMarker::RmarkerTx,
                               TimestampSource::HardwareMeasured, tx_bits, ra_e) ||
        !Timestamp::from_ticks(1000, d, TimestampMarker::RmarkerTx,
                               TimestampSource::HardwareMeasured, tx_bits, db_l) ||
        !Timestamp::from_ticks(0, d, TimestampMarker::RmarkerRx,
                               TimestampSource::HardwareMeasured, rx_bits, db_e)) {
        std::printf("FAIL: cannot build the M1-A timestamps\n");
        return 1;
    }
    if (apply_calibration_ticks(ra_l, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied ||
        apply_calibration_ticks(ra_e, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied ||
        apply_calibration_ticks(db_l, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied ||
        apply_calibration_ticks(db_e, cal_id, 0, 0, 0u, 0u) != CalibrationResult::Applied) {
        std::printf("FAIL: cannot calibrate the M1-A timestamps\n");
        return 1;
    }
    const RangingIntervalAdmission ra = admit_ranging_interval(ra_l, ra_e, ctx);
    const RangingIntervalAdmission db = admit_ranging_interval(db_l, db_e, ctx);
    if (!ra.admitted || !db.admitted || !ra.value.has_value() || !db.value.has_value()) {
        std::printf("FAIL: the M1-A intervals were not admitted\n");
        return 1;
    }
    ClockRatio k;
    if (!ClockRatio::unity_same_clock(d, k)) {
        std::printf("FAIL: cannot state a common clock\n");
        return 1;
    }
    const TofResult tof = compute_ss_tof(*ra.value, *db.value, k);
    if (!tof.ok || tof.tof.num != 500 || tof.tof.den != 1) {
        std::printf("FAIL: compute_ss_tof expected 500/1, got %s (%s)\n",
                    tof.tof.to_string().c_str(), tof_status_to_string(tof.status));
        return 1;
    }
    std::printf("M1-A CONSUMER OK: compute_ss_tof -> %s A ticks\n",
                tof.tof.to_string().c_str());
    return 0;
}
EOF

echo
echo "== compile + run the throwaway admission consumer against the prefix only =="
"$CXX" -std=c++17 -O1 -Wall -Wextra -I"$PREFIX/include" \
    "$WORK/tiny_admission_consumer.cc" -o "$WORK/tiny_admission_consumer"
"$WORK/tiny_admission_consumer"

# ---------------------------------------------------------------------------
# The repository's full consumer (header set + contract + Python module),
# driven through the wrapper with the ACTUAL configured Python directory.
# ---------------------------------------------------------------------------
echo
echo "== full installed consumer (C++ + Python) =="
PYTHON="$PYTHON" sh "$CONSUMER_DIR/run_install_consumer.sh" \
    --prefix "$PREFIX" \
    --python-rel "$PYTHON_REL"

echo
echo "VERIFY INSTALL CONSUMER: ALL OK"
