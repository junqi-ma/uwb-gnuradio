#!/usr/bin/env python3
"""Build private diagnostic instrumentation and replay selected failure windows."""
import argparse
import json
from pathlib import Path
import re
import subprocess


def build(root):
    work = Path('/tmp/uwb_window_diag')
    header = work/'include/gnuradio/uwb/uwb_demod_core.h'
    header.parent.mkdir(parents=True, exist_ok=True)
    text = (root/'gr-uwb/include/gnuradio/uwb/uwb_demod_core.h').read_text()
    patches = [
        ('    out = TimingResult{};', '    ++window_diag::timing_call;\n    out = TimingResult{};'),
        ('        if (bm < min_metric)', '        window_diag::local(want, radius, bm, bp, min_metric);\n        if (bm < min_metric)'),
        ('    out.cfo_hz = slope / (2.0 * M_PI);', '    out.cfo_hz = window_diag::force_cfo != 0 ? window_diag::force_cfo : slope / (2.0 * M_PI);'),
        ('    out = SfdResult{};', '    ++window_diag::sfd_call;\n    out = SfdResult{};'),
        ('    constexpr size_t kSfdStride = 8;', '    const size_t kSfdStride = window_diag::dense_sfd ? 1 : 8;'),
        ('    bool integer_grid_ok = true;', '    if (window_diag::nominal_period) res.timing.measured_period = 1016.0;\n    bool integer_grid_ok = true;'),
        ('    if (best < profile.sfd_detection_threshold)\n        return false;\n\n    out.ok = true;\n    out.sfd_start_chip', '    window_diag::nsfds.push_back({best, profile.sfd_detection_threshold, best_k});\n    if (best < profile.sfd_detection_threshold)\n        return false;\n\n    out.ok = true;\n    out.sfd_start_chip'),
        ('        return window_best >= profile.sfd_detection_threshold;', '        window_diag::sfd(expected, center, window_best, window_best_j, profile.sfd_detection_threshold);\n        return window_best >= profile.sfd_detection_threshold;')]
    for before, after in patches:
        assert text.count(before) == 1, before
        text = text.replace(before, after)
    header.write_text(text)
    command = ['c++', '-std=c++17', '-O3', '-mavx2', '-mfma', '-ffp-contract=off',
               '-Wno-deprecated-declarations', '-I'+str(work/'include'),
               '-Igr-uwb/include', '-Iinclude', '-Igr-uwb/build-avx512-smtcheck/include',
               'tools/rx/diagnose_failure_window.cc',
               'gr-uwb/build-avx512-smtcheck/lib/libuwb_core.a', '-L/usr/local/lib',
               '-lvolk', '-ldl', '-lm', '-pthread', '-Wl,-rpath,/usr/local/lib',
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
