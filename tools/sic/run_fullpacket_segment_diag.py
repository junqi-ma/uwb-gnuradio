#!/usr/bin/env python3
"""Run the offline field-timing diagnostic under the exclusive SIC lock."""
import argparse
import datetime
import os
import platform
import subprocess
from pathlib import Path

from run_fullpacket_compare import (ROOT, benchmark_lock, check_input, clean_env,
                                    git_output, library_binding, sha256, write_json)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--cell', action='append')
    parser.add_argument('--observations', type=int, default=128)
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--warmup', type=int, default=16)
    args = parser.parse_args()
    build, inputs, out = (p.resolve() for p in (args.build_dir, args.inputs, args.out))
    cells = args.cell or ['sync_base', 'payload_base']
    if len(set(cells)) != len(cells):
        parser.error('duplicate cell')
    if out.exists() and any(out.iterdir()):
        parser.error('choose an empty output directory')
    binary = build / 'apps/benchmark_fullpacket_segment_diag'
    env = clean_env()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    ldd, libraries = library_binding(binary, build, env)
    flags = build / 'apps/CMakeFiles/benchmark_fullpacket_segment_diag.dir/flags.make'
    sources = ['tests/benchmarks/benchmark_fullpacket_segment_diag.cc',
               'tests/benchmarks/fullpacket_segment_diagnostics.h',
               'tests/benchmarks/fullpacket_segment_diagnostics.cc',
               'lib/sic/uwb_fullpacket_sic_core.cc',
               'include/openuwb/sic/fullpacket_sic_core.h',
               'include/openuwb/phy/hrp/tx/hrp_mod_core.h',
               'include/openuwb/phy/hrp/rx/demod_core.h',
               'tools/sic/run_fullpacket_segment_diag.py']
    status = git_output('status', '--porcelain=v1', '--untracked-files=normal')
    if status:
        raise RuntimeError('commit diagnostic code before formal measurement')
    prov = dict(schema='fullpacket-segment-provenance/1',
                git_head=git_output('rev-parse', 'HEAD'), git_dirty=False,
                created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                machine=platform.platform(), affinity=sorted(os.sched_getaffinity(0)),
                binary=str(binary), binary_sha256=sha256(binary),
                libraries=libraries, ldd=ldd, compile_flags=flags.read_text(),
                source_sha256={p: sha256(ROOT / p) for p in sources},
                capture_policy='public raw IQ only; model and wire PSDU captured once per source; portable decode outside reconstruction timing',
                interval_policy='phase: waveform cuts; path/CFO: same cuts shifted by floor(min estimated tau); delayed tail belongs to payload',
                waveform_cuts=[0, 73152, 94656, 133568],
                cold_policy='reset key/mask before each call; allocations excluded',
                steady_policy=f'same source IQ, separate variant scratch; warmup{args.warmup} then alternating/reversed production and segmented',
                scope='reconstruction channel only; validation/TX/subtraction/estimation/decode outside timing',
                lock='/tmp/uwb-sic-benchmark.lock', runs=[])
    out.mkdir(parents=True, exist_ok=True)
    with benchmark_lock(True):
        for cell in cells:
            case = inputs / cell
            pub, windows, hashes, templates = check_input(case)
            command = [str(binary), '--case', str(case), '--out', str(out / cell),
                       '--observations', str(args.observations), '--rounds', str(args.rounds),
                       '--warmup', str(args.warmup)]
            with (out / (cell + '.log')).open('w') as log:
                rc = subprocess.run(command, cwd=ROOT, env=env, stdout=log,
                                    stderr=subprocess.STDOUT).returncode
            prov['runs'].append(dict(cell=cell, argv=command, returncode=rc,
                                     public_profile=pub, source_hashes=hashes,
                                     template_hashes=templates))
            write_json(out / 'provenance.json', prov)
            print(cell, rc, flush=True)
            if rc:
                raise RuntimeError(f'{cell} failed; see {out}/{cell}.log')
    write_json(out / 'experiment_summary.json', dict(complete=True, passed=True,
               cells=cells, observations=args.observations, rounds=args.rounds,
               warmup=args.warmup, provenance=str(out / 'provenance.json')))


if __name__ == '__main__':
    main()
