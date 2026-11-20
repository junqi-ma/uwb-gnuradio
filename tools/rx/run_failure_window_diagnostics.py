#!/usr/bin/env python3
"""Build private diagnostic instrumentation and replay selected failure windows."""
import argparse
import json
from pathlib import Path
import re
import subprocess


def build(root):
    work = Path('/tmp/uwb_window_diag')
    private = work/'include/gnuradio/uwb'
    private.mkdir(parents=True, exist_ok=True)
    source_dir = root/'include/openuwb'
    files = {name: (source_dir/name).read_text() for name in
             ('uwb_demod_timing.h', 'uwb_demod_cfo.h', 'uwb_demod_sfd.h',
              'uwb_demod_phr.h')}
    files['uwb_demod_pipeline.cc'] = (root/'lib/phy/hrp/uwb_demod_pipeline.cc').read_text()
    patches = [
        ('    out = TimingResult{};', '    ++window_diag::timing_call;\n    out = TimingResult{};'),
        ('        if (bm < min_metric)', '        window_diag::local(want, radius, bm, bp, min_metric);\n        if (bm < min_metric)'),
        ('    out.cfo_hz = slope / (2.0 * M_PI);', '    out.cfo_hz = window_diag::force_cfo != 0 ? window_diag::force_cfo : slope / (2.0 * M_PI);'),
        ('    out = SfdResult{};', '    ++window_diag::sfd_call;\n    out = SfdResult{};'),
        ('    const auto execution = plan_packet_execution(profile, res.timing);', '    if (window_diag::nominal_period) res.timing.measured_period = 1016.0;\n    const auto execution = plan_packet_execution(profile, res.timing);'),
        ('    if (best < profile.sfd_detection_threshold)\n        return false;\n\n    out.ok = true;\n    out.sfd_start_chip', '    window_diag::nsfds.push_back({best, profile.sfd_detection_threshold, best_k});\n    if (best < profile.sfd_detection_threshold)\n        return false;\n\n    out.ok = true;\n    out.sfd_start_chip'),
        ('        return window_passed;', '        window_diag::sfd(expected, center, window_best, window_best_j, profile.sfd_detection_threshold);\n        return window_passed;')]
    for before, after in patches:
        matches = [name for name, text in files.items() if before in text]
        assert len(matches) == 1, (before, matches)
        name = matches[0]
        assert files[name].count(before) == 1, before
        files[name] = files[name].replace(before, after)
    for name, text in files.items():
        (private/name).write_text(text)
    # The diagnostic entry has a distinct symbol, so the core archive cannot
    # accidentally replace the instrumented implementation at link time.
    build_dir = root/'build/window-diag'
    subprocess.run(['cmake', '-S', str(root), '-B', str(build_dir),
                    '-DCMAKE_BUILD_TYPE=Release', '-DUWB_BUILD_EXPERIMENTS=OFF', '-DENABLE_UHD_BACKEND=OFF',
                    '-DUWB_SIMD=AVX2'], check=True)
    subprocess.run(['cmake', '--build', str(build_dir), '--target', 'uwb_core', '-j4'],
                   check=True)
    command = ['c++', '-std=c++17', '-O3', '-mavx2', '-mfma', '-ffp-contract=off',
               '-Ddemodulate_one=window_diagnostic_demodulate_one',
               '-include', str(root/'tools/rx/window_diag_state.h'),
               '-Wno-deprecated-declarations', '-I'+str(work/'include'),
               '-I'+str(root/'include'), '-I'+str(build_dir/'include'),
               str(root/'tools/rx/diagnose_failure_window.cc'),
               str(private/'uwb_demod_pipeline.cc'),
               str(build_dir/'lib/libuwb_core.a'), '-lvolk', '-ldl', '-lm', '-pthread',
               '-o', str(work/'diagnose')]
    subprocess.run(command, cwd=root, check=True)
    return work/'diagnose'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--examples', type=Path, required=True)
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[2]
    binary = build(root)
    examples = json.loads((args.examples/'manifest.json').read_text())
    out = args.examples/'analysis'
    out.mkdir(exist_ok=True)
    result = []
    names = ['OK', 'INVALID_INPUT', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
             'PHR_FAILED', 'PAYLOAD_FAILED', 'FCS_FAILED', 'QUEUE_FULL', 'INTERNAL_ERROR', 'CIR_FAILED']
    for a in examples:
        attempts = a['selected_attempt']
        seed = int(attempts['pred']) if attempts else a['predicted_start_sample']
        ws = int(attempts['win']) if attempts else seed-36864
        before = a['anchors'][2]
        good = dict(re.findall(r'(\w+)=([^ ]+)', before['log_line']))
        cfo = float(good['cfo_khz'])*1000
        modes = ['baseline', 'acquire', 'timing020', 'sfd025', 'both', 'dense', 'nominal', f'cfo={cfo:g}', 'good_control']
        runs = {}
        for mode in modes:
            prefix = out/(a['id']+'_'+mode.replace('=', '_'))
            actual_seed, actual_ws = seed, ws
            if mode == 'good_control':
                actual_seed = int(good.get('pred', good['start']))
                actual_ws = int(good.get('win', actual_seed-36864))
            command = [str(binary), a['source'], str(actual_ws), '374217', str(actual_seed),
                       mode, str(prefix), str(root/'testdata')]
            completed = subprocess.run(command, capture_output=True, text=True, check=True)
            r = json.loads(completed.stdout)
            r.update(status_name=names[r['status']], seed=actual_seed, window_start=actual_ws,
                     prefix=str(prefix))
            runs[mode] = r
            if mode not in ('baseline', 'good_control'):
                prefix.with_suffix('.cf32').unlink()
        record = dict(id=a['id'], kind=a['kind'], runs=runs)
        if attempts:
            base = runs['baseline']
            record['original_status_reproduced'] = base['status_name'] == attempts['status']
            record['original_sync_reproduced'] = base['sync'] == int(attempts['sync'])
        result.append(record)
        (out/'replay_results.json').write_text(json.dumps(result, indent=2)+'\n')
        print(a['id'], ' '.join(f'{m}:{r["status_name"]}/{r["sync"]}' for m, r in runs.items()), flush=True)


if __name__ == '__main__':
    main()
