#!/bin/bash
# P0 AVX2 baseline matrix driver (serial, one process at a time).
# Does not modify sources/CMake/build dirs; writes only JSON + one log here.
set -u
cd /tmp/uwb-gnuradio-avx512 || exit 1
# Private, frozen copy of the AVX2 benchmark binary.  The build tree may be
# reconfigured/rebuilt by another agent; the dataset must stay tied to ONE
# binary, so every run below uses this copy and never the build-tree path.
BIN=/tmp/opencode/bench_avx2_baseline
BIN_SHA256=dc425102a10518ea6b4fde7f1e6b9032e1749466dbabf4bb38605b4a02d0ab6a
OUT=docs/performance/data_avx512_20261116
TAPS=testdata/resampler_65_48
LOG=$OUT/matrix_stdout.log
: > "$LOG"
echo "# binary $(sha256sum "$BIN")" >> "$LOG"
if [ "$(sha256sum "$BIN" | cut -d' ' -f1)" != "$BIN_SHA256" ]; then
  echo "FATAL: benchmark copy SHA256 does not match the frozen value"; exit 1
fi
echo "# start $(date -Is) load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$LOG"

run() { # rate taps len fp round
  local rate=$1 taps=$2 len=$3 fp=$4 r=$5
  local name="${rate}_${taps}_${len}_fp${fp}_r${r}"
  local json="$OUT/${name}.json"
  echo "### $name $(date -Is) load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$LOG"
  "$BIN" --taps "$TAPS/taps_${taps}.txt" --rate "$rate" --kernel avx2 \
         --length "$len" --fixed-phase "$fp" --reps 50 --warmup 10 \
         --json "$json" >> "$LOG" 2>&1
  local rc=$?
  if [ $rc -ne 0 ]; then echo "FAILED rc=$rc $name" >> "$LOG"; echo "FAILED $name"; fi
}

for r in 1 2 3; do
  for taps in realtime_eq16 realtime_minorder quality_eq32 quality_minorder; do
    for len in 4096 56000; do
      for fp in 1 0; do
        run 65_48 "$taps" "$len" "$fp" "$r"
      done
    done
  done
  # second instantiation: 65/32
  for taps in realtime_eq16 quality_minorder; do
    for len in 4096 56000; do
      run 65_32 "$taps" "$len" 1 "$r"
    done
  done
done
echo "# end $(date -Is) load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$LOG"
echo DONE
