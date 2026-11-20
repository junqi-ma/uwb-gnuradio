#!/bin/sh
# Exit-code QA for uwb_uhd_sc16_pipeline_runner (round-2 review R2).
#
# The review found that the runner computed ready_ok / ledger invariants and
# still returned 0 unconditionally, so "a failed run exits non-zero" was not
# actually implemented.  The runner now distinguishes:
#
#   * worker readiness  -> ready_ok()          (DSP workers only)
#   * worker/source failure -> failed()        (includes a source failure)
#   * source lifecycle  -> source_started() / source_failed() / source_error()
#
# and exits 1 when runtime_ok is false.  A normal EOF or user stop after a
# successful start must still exit 0, which is what case 2 below pins down.
#
# Cases:
#   1. A nonexistent --file fails initialize(); the process must exit non-zero
#      (the historical bug exited 0 here).
#   2. One valid 56k-sample window from the 737.28 MS/s replay fixture must
#      exit 0, and its JSON must prove the source really started and the
#      ledger balances.
#
# The start_stream() == false path cannot be triggered through a file source;
# it is covered by the library QA (fake source with set_start_stream_result
# (false)), so it is deliberately not faked here.
#
# Usage:
#   qa_uwb_uhd_sc16_pipeline_runner_exit.sh --runner <bin> --fixture <sc16 bin>
#
# Everything is written under ${TMPDIR:-/tmp}; the source/build tree is never
# written to.  Exit 0 == both cases passed.
set -eu

RUNNER=""
FIXTURE=""

usage()
{
    echo "Usage: $0 --runner <uwb_uhd_sc16_pipeline_runner> --fixture <sc16 file>" >&2
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

if [ -z "$RUNNER" ] || [ -z "$FIXTURE" ]; then
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

TMPROOT=${TMPDIR:-/tmp}
WORK=$(mktemp -d "$TMPROOT/uwb-sc16-runner-exit.XXXXXX") || {
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
# whitespace-tolerant ERE is enough and keeps this test free of a Python/jq
# dependency.  Values are matched against the exact JSON spelling the runner
# emits (true/false/1), never against a substring of a longer token.
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

echo "[case 1] nonexistent --file must exit non-zero (initialize failure)"
MISSING="$WORK/does-not-exist-sc16.bin"
if [ -e "$MISSING" ]; then
    fail "'$MISSING' unexpectedly exists; the case needs a nonexistent path"
fi

set +e
"$RUNNER" \
    --file "$MISSING" \
    --period 56000 --capture 56000 --max-windows 1 --consumer sensing \
    --json-out "$WORK/missing.json" >"$WORK/missing.log" 2>&1
rc=$?
set -e

if [ "$rc" -eq 0 ]; then
    dump_log "$WORK/missing.log"
    fail "nonexistent --file exited 0; an initialize failure must exit non-zero"
fi
if ! grep -q "Failed to initialize" "$WORK/missing.log"; then
    dump_log "$WORK/missing.log"
    fail "nonexistent --file exited $rc but the log has no 'Failed to initialize' line"
fi
echo "  ok: exit code $rc, initialize failure reported"

echo "[case 2] one valid 56k window must exit 0 and report a healthy source"
set +e
"$RUNNER" \
    --file "$FIXTURE" \
    --period 56000 --capture 56000 --max-windows 1 --consumer sensing \
    --json-out "$WORK/ok.json" >"$WORK/ok.log" 2>&1
rc=$?
set -e

if [ "$rc" -ne 0 ]; then
    dump_log "$WORK/ok.log"
    fail "valid 56k window run exited $rc; expected 0"
fi
if [ ! -s "$WORK/ok.json" ]; then
    dump_log "$WORK/ok.log"
    fail "run exited 0 but wrote no JSON to $WORK/ok.json"
fi

check_json_regex source_started true "$WORK/ok.json"
check_json_regex source_failed false "$WORK/ok.json"
check_json_regex source_error '""' "$WORK/ok.json"
check_json_regex ledger_invariants_ok true "$WORK/ok.json"
# The window must actually have run, not merely have exited 0 with an empty
# ledger: one offered / admitted / completed 56k window.
check_json_regex offered_windows '1([^0-9]|$)' "$WORK/ok.json"
check_json_regex admitted_windows '1([^0-9]|$)' "$WORK/ok.json"
check_json_regex completed_windows '1([^0-9]|$)' "$WORK/ok.json"
echo "  ok: exit code 0, source_started=true, source_failed=false, ledger ok"

echo "PASS: uwb_uhd_sc16_pipeline_runner exit-code QA (2/2 cases)"
exit 0
