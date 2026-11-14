#!/bin/sh
# End-to-end QA for `--output-dir` / `--live-log` on uwb_uhd_sc16_pipeline_runner.
#
# Owner: Track 3 of
# docs/performance/OpenCode_Pipeline落盘与实时日志_多Agent执行计划_2026-11-14.md.
#
# Cases
#   1. --consumer sensing --output-dir <dir> --live-log summary
#      --live-log-interval-ms 50 on the 737.28 MS/s fixture:
#        cir.ucr4 / cir.jsonl / run.json exist, the independent Python verifier
#        accepts them, and the live log produced at least one line.
#   2. --consumer comm --output-dir <dir>: demod_results.jsonl exists and the
#      verifier accepts the whole directory (a comm-only run still leaves a
#      cir.jsonl, possibly with tap_count == 0 on every row).
#   3. No --output-dir: the run exits 0 and creates NO *.ucr4 / *.jsonl /
#      run.json anywhere in its working directory.
#   4. Usage errors: `--live-log bogus` and `--writer-queue 0` must exit
#      non-zero instead of silently falling back to a default.
#
# Cases 3 and 4 run in EVERY build.  Cases 1 and 2 need the real writer: when
# this build only carries the Phase 0 writer stub the runner reports
# output_offered == 0 && output_written == 0, and the script exits 77 (CMake
# SKIP_RETURN_CODE) AFTER 3 and 4 passed, so a stub can never look like a pass.
# A real ledger violation (output_ledger_ok == false) is a FAILURE, never a
# skip: masking it would hide exactly the defect this test exists to catch.
#
# Usage:
#   qa_uwb_pipeline_output_runner.sh --runner <bin> --fixture <sc16 bin> \
#                                    --verifier <verify_ucr4_output.py>
#
# Everything is written under ${TMPDIR:-/tmp}; the source/build tree is never
# written to.  Exit 0 == all cases passed, 77 == writer unavailable, 1 == FAIL.
set -eu

RUNNER=""
FIXTURE=""
VERIFIER=""
PYTHON=${PYTHON:-python3}

usage()
{
    echo "Usage: $0 --runner <uwb_uhd_sc16_pipeline_runner> --fixture <sc16 file> --verifier <verify_ucr4_output.py>" >&2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --runner)
            RUNNER=${2:-}
            shift 2
            ;;
        --runner=*)
            RUNNER=${1#*=}
            shift
            ;;
        --fixture)
            FIXTURE=${2:-}
            shift 2
            ;;
        --fixture=*)
            FIXTURE=${1#*=}
            shift
            ;;
        --verifier)
            VERIFIER=${2:-}
            shift 2
            ;;
        --verifier=*)
            VERIFIER=${1#*=}
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unknown argument '$1'" >&2
            usage
            exit 2
            ;;
    esac
done

if [ -z "$RUNNER" ] || [ -z "$FIXTURE" ] || [ -z "$VERIFIER" ]; then
    usage
    exit 2
fi
if [ ! -x "$RUNNER" ]; then
    echo "FAIL: runner '$RUNNER' is not an executable file" >&2
    exit 1
fi
if [ ! -f "$FIXTURE" ]; then
    echo "FAIL: replay fixture '$FIXTURE' not found" >&2
    exit 1
fi
if [ ! -f "$VERIFIER" ]; then
    echo "FAIL: verifier '$VERIFIER' not found" >&2
    exit 1
fi
if ! command -v "$PYTHON" >/dev/null 2>&1; then
    echo "FAIL: '$PYTHON' is required to run the verifier" >&2
    exit 1
fi

# Case 3 runs the runner from a scratch working directory, so every path handed
# to it must be absolute (CTest already passes absolute paths; a manual run may
# not).
abspath()
{
    case "$1" in
        /*) printf '%s\n' "$1" ;;
        *) printf '%s/%s\n' "$(pwd)" "$1" ;;
    esac
}
RUNNER=$(abspath "$RUNNER")
FIXTURE=$(abspath "$FIXTURE")
VERIFIER=$(abspath "$VERIFIER")

TMPROOT=${TMPDIR:-/tmp}
WORK=$(mktemp -d "$TMPROOT/uwb-pipeline-output.XXXXXX") || {
    echo "FAIL: cannot create a temp dir under $TMPROOT" >&2
    exit 1
}
cleanup()
{
    rm -rf "$WORK"
}
trap cleanup EXIT HUP INT TERM

fail()
{
    echo "FAIL: $*" >&2
    exit 1
}

dump_log()
{
    [ -f "$1" ] || return 0
    echo "----- tail of $1 -----" >&2
    tail -n 40 "$1" >&2
    echo "---------------------" >&2
}

# check_json_regex <field> <ERE value> <file>
#
# The runner writes one "meta"/"ledger" line of the form `"key": value`, so a
# whitespace-tolerant ERE is enough and keeps this test free of a jq
# dependency (the Python verifier is invoked separately, where it is the point).
check_json_regex()
{
    field=$1
    pattern=$2
    file=$3
    if ! grep -Eq "\"$field\"[[:space:]]*:[[:space:]]*$pattern" "$file"; then
        echo "FAIL: JSON field '$field' does not match '$pattern' in $file" >&2
        dump_log "$file"
        exit 1
    fi
}

# json_number <field> <file> -> prints the first unsigned integer value, or
# nothing when the field is absent.  The meta object is a single line, so a
# greedy prefix is safe: every key appears exactly once.
json_number()
{
    field=$1
    file=$2
    sed -n "s/.*\"$field\"[[:space:]]*:[[:space:]]*\([0-9][0-9]*\).*/\1/p" "$file" |
        head -n 1
}

# writer_disabled <json> -> 0 when the writer produced nothing at all, which is
# the Phase 0 stub signature (offered == written == 0).
writer_disabled()
{
    offered=$(json_number output_offered "$1")
    written=$(json_number output_written "$1")
    [ "$offered" = "0" ] && [ "$written" = "0" ]
}

run_runner()
{
    # run_runner <log> <json> [extra args...]
    log=$1
    json=$2
    shift 2
    set +e
    "$RUNNER" \
        --file "$FIXTURE" \
        --period 56000 --capture 56000 --max-windows 1 \
        "$@" \
        --json-out "$json" >"$log" 2>&1
    rc=$?
    set -e
    return $rc
}

# ---------------------------------------------------------------------------
# Case 3: no --output-dir must create no files at all.
# ---------------------------------------------------------------------------
echo "[case 3] no --output-dir must create no *.ucr4 / *.jsonl / run.json"
mkdir -p "$WORK/nofiles"
set +e
(
    cd "$WORK/nofiles" &&
        "$RUNNER" \
            --file "$FIXTURE" \
            --period 56000 --capture 56000 --max-windows 1 --consumer sensing \
            --json-out "$WORK/nofiles.json" >"$WORK/nofiles.log" 2>&1
)
rc=$?
set -e
if [ "$rc" -ne 0 ]; then
    dump_log "$WORK/nofiles.log"
    fail "run without --output-dir exited $rc; expected 0"
fi
stray=$(find "$WORK/nofiles" \( -name '*.ucr4' -o -name '*.jsonl' -o -name 'run.json' \) -print)
if [ -n "$stray" ]; then
    echo "$stray" >&2
    fail "run without --output-dir created output files: $stray"
fi
if [ ! -s "$WORK/nofiles.json" ]; then
    dump_log "$WORK/nofiles.log"
    fail "run without --output-dir wrote no --json-out document"
fi
check_json_regex output_offered '0([^0-9]|$)' "$WORK/nofiles.json"
check_json_regex output_written '0([^0-9]|$)' "$WORK/nofiles.json"
echo "  ok: exit code 0, no output files, output_offered=0 output_written=0"

# ---------------------------------------------------------------------------
# Case 4: usage errors must be rejected, never silently defaulted.
# ---------------------------------------------------------------------------
echo "[case 4] usage errors must exit non-zero"
set +e
"$RUNNER" \
    --file "$FIXTURE" --period 56000 --capture 56000 --max-windows 1 \
    --consumer sensing --live-log bogus >"$WORK/badlivelog.log" 2>&1
rc=$?
set -e
if [ "$rc" -eq 0 ]; then
    dump_log "$WORK/badlivelog.log"
    fail "--live-log bogus exited 0; an unknown live-log mode must be a usage error"
fi
echo "  ok: --live-log bogus exited $rc"

set +e
"$RUNNER" \
    --file "$FIXTURE" --period 56000 --capture 56000 --max-windows 1 \
    --consumer sensing --writer-queue 0 >"$WORK/badqueue.log" 2>&1
rc=$?
set -e
if [ "$rc" -eq 0 ]; then
    dump_log "$WORK/badqueue.log"
    fail "--writer-queue 0 exited 0; a zero queue depth must be a usage error"
fi
echo "  ok: --writer-queue 0 exited $rc"

# ---------------------------------------------------------------------------
# Case 1: sensing CIR persistence + live log.
# ---------------------------------------------------------------------------
echo "[case 1] --consumer sensing --output-dir + --live-log summary"
OUT1="$WORK/out-sensing"
if ! run_runner "$WORK/sensing.log" "$WORK/sensing.json" --consumer sensing \
        --output-dir "$OUT1" --live-log summary --live-log-interval-ms 50; then
    dump_log "$WORK/sensing.log"
    fail "sensing run with --output-dir failed"
fi
if writer_disabled "$WORK/sensing.json"; then
    echo "SKIP: output writer disabled in this build (Phase 0 stub)" >&2
    echo "      cases 3 and 4 passed; cases 1 and 2 were not exercised" >&2
    exit 77
fi
check_json_regex output_ledger_ok true "$WORK/sensing.json"
for name in cir.ucr4 cir.jsonl run.json; do
    [ -f "$OUT1/$name" ] || {
        dump_log "$WORK/sensing.log"
        fail "sensing run did not create $OUT1/$name"
    }
done
if ! "$PYTHON" "$VERIFIER" --dir "$OUT1"; then
    dump_log "$WORK/sensing.log"
    fail "verifier rejected $OUT1"
fi
live_lines=$(sed -n 's/.*live_log_lines:[[:space:]]*\([0-9][0-9]*\).*/\1/p' \
    "$WORK/sensing.log" | head -n 1)
if [ -z "$live_lines" ] || [ "$live_lines" -lt 1 ]; then
    dump_log "$WORK/sensing.log"
    fail "--live-log summary produced no live log line (live_log_lines='$live_lines')"
fi
echo "  ok: cir.ucr4/cir.jsonl/run.json verified, live_log_lines=$live_lines"

# ---------------------------------------------------------------------------
# Case 2: comm decode persistence.
# ---------------------------------------------------------------------------
echo "[case 2] --consumer comm --output-dir"
OUT2="$WORK/out-comm"
if ! run_runner "$WORK/comm.log" "$WORK/comm.json" --consumer comm \
        --output-dir "$OUT2"; then
    dump_log "$WORK/comm.log"
    fail "comm run with --output-dir failed"
fi
if writer_disabled "$WORK/comm.json"; then
    echo "SKIP: output writer disabled in this build (Phase 0 stub)" >&2
    echo "      cases 3 and 4 passed; cases 1 and 2 were not exercised" >&2
    exit 77
fi
check_json_regex output_ledger_ok true "$WORK/comm.json"
[ -f "$OUT2/demod_results.jsonl" ] || {
    dump_log "$WORK/comm.log"
    fail "comm run did not create $OUT2/demod_results.jsonl"
}
if ! "$PYTHON" "$VERIFIER" --dir "$OUT2"; then
    dump_log "$WORK/comm.log"
    fail "verifier rejected $OUT2"
fi
echo "  ok: demod_results.jsonl verified"

echo "PASS: uwb_uhd_sc16_pipeline_runner output persistence QA (4/4 cases)"
exit 0
