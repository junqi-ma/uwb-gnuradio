#!/usr/bin/env python3
"""Exercise the real threaded live receiver without hardware or private IQ."""
from array import array
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile

binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory(prefix='uwb-live-console-') as temporary:
    path = Path(temporary) / 'cw_burst.sc16'
    # A finite 6 ms stream with one 0.3 ms DC burst. Autocorrelation sees a
    # candidate; the real UWB matched template must reject it as timing_failed.
    count = round(737.28e6 * .006)
    iq = array('i', [0]) * count
    start, length = round(737.28e6 * .001), round(737.28e6 * .0003)
    iq[start:start+length] = array('i', [1000]) * length
    with path.open('wb') as f:
        iq.tofile(f)
    command = [binary, '--replay', str(path), '--summary', '1']
    result = subprocess.run(command, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr + result.stdout
    packets = re.findall(r'^PACKET .*$', result.stdout, re.M)
    assert len(packets) == 1, result.stdout
    assert 'status=TIMING_FAILED' in packets[0], result.stdout
    costs = dict(part.split('=', 1) for part in packets[0].split()[1:] if '=' in part)
    for key in ['queue_us', 'resample_us', 'demod_us', 'retry_us', 'compute_us', 'cpu_us']:
        assert float(costs[key]) >= 0, packets[0]
    assert abs(float(costs['latency_ms'])*1000-float(costs['queue_us'])-float(costs['compute_us'])) < 2, packets[0]
    assert float(costs['compute_us']) >= float(costs['resample_us'])+float(costs['demod_us'])-1, packets[0]
    assert re.search(r'^COST_TOTAL frontend_us=.* estimate_us=.* cancel_us=', result.stdout, re.M), result.stdout
    assert re.search(r'^SUMMARY completed=1 batch=1 batch_fcs=0/1 ', result.stdout, re.M), result.stdout
    assert re.search(r'^FINAL completed=1 fcs=0/1 .*raw_drop_samples=0 source_errors=0 gaps=0 ', result.stdout, re.M), result.stdout
    # Real SIGINT follows a successful start; shutdown must join every thread.
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        lines = []
        assert process.stdout is not None
        for line in process.stdout:
            lines.append(line)
            if line.startswith('STREAM_STARTED'):
                process.send_signal(signal.SIGINT)
                break
        tail, errors = process.communicate(timeout=15)
        output = ''.join(lines) + tail
        assert process.returncode == 0, errors + output
        assert 'FINAL completed=' in output, output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    missing = subprocess.run([binary, '--replay', str(path)+'.missing'],
                             capture_output=True, text=True, timeout=15)
    assert missing.returncode == 2, missing.stdout + missing.stderr

    # ------------------------------------------------------------------
    # Target lock + LockedBounded synchronization strategy.
    # The synthetic stream has no target, so the lock must stay in acquire and
    # the run must still finish with the same single rejected candidate.
    # ------------------------------------------------------------------
    locked = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1', '--sync', 'locked',
         '--lock-observations', '3', '--holdover', '3', '--reacquire', '8',
         '--locked-roi', '256'],
        capture_output=True, text=True, timeout=30)
    assert locked.returncode == 0, locked.stderr + locked.stdout
    assert re.search(r'^FINAL completed=1 fcs=0/1 ', locked.stdout, re.M), locked.stdout
    assert re.search(r'^LOCK_STATS final_state=acquire ', locked.stdout, re.M), locked.stdout
    assert re.search(r'^LOCK_STATS .*reacquire_entries=0 ', locked.stdout, re.M), locked.stdout
    assert re.search(r'^LOCK_STATS .*unexpected_gaps=', locked.stdout) is None
    assert re.search(r'^START .*sync=locked .*period_ms=5\.000000 ', locked.stdout), locked.stdout

    # ------------------------------------------------------------------
    # Device-timed burst on a timestamped fake source: planned gaps are not
    # errors, no unexpected gap, and the blind grid needs an explicit anchor.
    # ------------------------------------------------------------------
    timed = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1', '--mode', 'timed',
         '--sync', 'locked', '--timed-t0-ms', '1.0', '--timed-lookahead', '2',
         '--timed-alignment', '4', '--timed-lead-us', '300'],
        capture_output=True, text=True, timeout=60)
    assert timed.returncode == 0, timed.stderr + timed.stdout
    assert 'TONE_CANCEL disabled reason=timed_burst_training_unscheduled' in timed.stdout
    plan = re.search(r'^TIMED_PLAN .*$', timed.stdout, re.M)
    assert plan is not None, timed.stdout
    fields = dict(part.split('=', 1) for part in plan.group(0).split()[1:] if '=' in part)
    assert int(fields['issued']) >= 1, plan.group(0)
    assert int(fields['unexpected_gaps']) == 0, plan.group(0)
    assert int(fields['late_commands']) == 0, plan.group(0)
    assert int(fields['overflows']) == 0, plan.group(0)
    assert int(fields['device_time_jumps']) == 0, plan.group(0)
    assert int(fields['alignment_expanded_samples']) >= 0, plan.group(0)
    assert re.search(r'^RX_PLANNED_GAP ', timed.stdout, re.M), timed.stdout
    assert re.search(r'^RX_GAP ', timed.stdout, re.M) is None, timed.stdout

    # The device-timed UHD path is not validated on hardware: it must fail
    # closed rather than silently fall back to a host timer.
    no_replay = subprocess.run([binary, '--mode', 'timed'],
                               capture_output=True, text=True, timeout=15)
    assert no_replay.returncode == 2, no_replay.stdout + no_replay.stderr
    assert 'timed' in no_replay.stderr.lower(), no_replay.stderr

    # ------------------------------------------------------------------
    # State-dependent pre-guard.
    # The job buffer is allocated for the widest geometry, so the LOCKED guard
    # may only shrink inside that budget and must never fall below the bounded
    # ROI plus the resampler transient.  Every rejected configuration must
    # fail closed instead of silently cutting a truncated window.
    # ------------------------------------------------------------------
    def start_fields(stdout):
        line = re.search(r'^START .*$', stdout, re.M)
        assert line is not None, stdout
        return dict(part.split('=', 1) for part in line.group(0).split()[1:]
                    if '=' in part)

    narrow = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1', '--sync', 'locked',
         '--locked-pre-guard', '8192'],
        capture_output=True, text=True, timeout=30)
    assert narrow.returncode == 0, narrow.stderr + narrow.stdout
    fields = start_fields(narrow.stdout)
    # Acquisition keeps the wide guard; only LOCKED uses the narrow one.
    assert fields['pre_guard'] == '36864', fields
    assert fields['locked_pre_guard'] == '8192', fields
    assert int(fields['locked_window_capacity']) < int(fields['window_capacity']), fields
    assert int(fields['locked_window_capacity']) <= 450000, fields
    # The synthetic stream never locks, so the guard must never switch.
    assert re.search(r'^PRE_GUARD ', narrow.stdout, re.M) is None, narrow.stdout
    candidate = re.search(r'^PACKET .*path=acquire .*$', narrow.stdout, re.M)
    assert candidate is not None, narrow.stdout
    assert 'guard=36864' in candidate.group(0), candidate.group(0)
    # The window actually cut for the candidate starts one guard before the
    # seed, i.e. the reported guard is the one that produced the window.
    assert re.search(r'^LOCK_STATS final_state=acquire ', narrow.stdout, re.M), narrow.stdout
    assert re.search(r'^FINAL completed=1 fcs=0/1 ', narrow.stdout, re.M), narrow.stdout

    for bad, why in [('0', 'must be > 0'),
                     ('256', 'must be >='),
                     ('400000', 'exceeds the job buffer')]:
        rejected = subprocess.run(
            [binary, '--replay', str(path), '--summary', '1', '--sync', 'locked',
             '--locked-pre-guard', bad],
            capture_output=True, text=True, timeout=30)
        assert rejected.returncode == 2, (bad, rejected.stdout + rejected.stderr)
        assert why in rejected.stderr, (bad, rejected.stderr)
        assert 'START ' not in rejected.stdout, rejected.stdout

    # Autocorrelation acquisition parameter validation.
    for extra, why in [
        (['--acq-span', '48000', '--acq-step', '7'], 'positive multiple'),
        (['--acq-persist', '0'], 'must be > 0'),
        (['--acq-low', '0.10', '--acq-peak', '0.05'], 'low <= peak'),
        (['--acq-low', '0.0'], 'low <= peak'),
    ]:
        rejected = subprocess.run(
            [binary, '--replay', str(path), '--summary', '1'] + extra,
            capture_output=True, text=True, timeout=30)
        assert rejected.returncode == 2, (extra, rejected.stdout + rejected.stderr)
        assert why in rejected.stderr, (extra, rejected.stderr)
        assert 'START ' not in rejected.stdout, rejected.stdout

print('live console: acquisition/rejection, batch summary, SIGINT, source failure, '
      'locked strategy, timed-burst plan and locked pre-guard PASS')
