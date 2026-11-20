#!/usr/bin/env python3
"""Exercise the real threaded live receiver without hardware or private IQ."""
from array import array
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile

# Phase 3: discover the receiver when no path is given, so the documented
# `python3 tools/rx/qa_live_dw3000_console.py` works from a fresh preset build.
# OPENUWB_BUILD_DIR overrides the search.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _common.paths import find_app  # noqa: E402

binary = (str(Path(sys.argv[1]).resolve()) if len(sys.argv) > 1
          else str(find_app("openuwb_rx")))
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

    # ------------------------------------------------------------------
    # Locked-state autocorrelation gate:
    #   - default is scan and the four counters are present but zero;
    #   - auto resolves by --interference;
    #   - suspend + --interference on fails closed before START;
    #   - bad mode/lookback fail closed.
    # ------------------------------------------------------------------
    gate_keys = ('autocorr_suspend_events', 'autocorr_resume_events',
                 'autocorr_suspended_samples',
                 'autocorr_resume_backlog_samples')
    scan = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1'],
        capture_output=True, text=True, timeout=30)
    assert scan.returncode == 0, scan.stderr + scan.stdout
    fields = start_fields(scan.stdout)
    assert fields['autocorr_when_locked'] == 'scan', fields
    lock_line = re.search(r'^LOCK_STATS .*$', scan.stdout, re.M)
    assert lock_line is not None, scan.stdout
    for key in gate_keys:
        assert f' {key}=0 ' in lock_line.group(0) + ' ', lock_line.group(0)

    auto_off = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1',
         '--autocorr-when-locked', 'auto', '--interference', 'off'],
        capture_output=True, text=True, timeout=30)
    assert auto_off.returncode == 0, auto_off.stderr + auto_off.stdout
    assert start_fields(auto_off.stdout)['autocorr_when_locked'] == 'suspend'
    auto_on = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1',
         '--autocorr-when-locked', 'auto', '--interference', 'on'],
        capture_output=True, text=True, timeout=30)
    assert auto_on.returncode == 0, auto_on.stderr + auto_on.stdout
    assert start_fields(auto_on.stdout)['autocorr_when_locked'] == 'scan'

    for extra, why in [
        (['--autocorr-when-locked', 'suspend', '--interference', 'on'],
         'requires --interference off'),
        (['--autocorr-when-locked', 'on'], 'invalid --autocorr-when-locked'),
        (['--autocorr-when-locked', 'SUSPEND'], 'invalid --autocorr-when-locked'),
        (['--autocorr-resume-lookback', '-2'], 'must be >= -1'),
    ]:
        rejected = subprocess.run(
            [binary, '--replay', str(path), '--summary', '1'] + extra,
            capture_output=True, text=True, timeout=30)
        assert rejected.returncode == 2, (extra, rejected.stdout + rejected.stderr)
        assert why in rejected.stderr, (extra, rejected.stderr)
        assert 'START ' not in rejected.stdout, (extra, rejected.stdout)

    # ------------------------------------------------------------------
    # Low-SNR lock/capture switches.
    #
    # Both are strict on|off: they change what the lock and the demodulator
    # consider a success, so a typo must fail closed instead of silently
    # choosing a mode (unlike --tone/--fused, which treat anything but "off"
    # as on).
    # ------------------------------------------------------------------
    for flag in ('--lock-fcs', '--stop-after-sfd'):
        for bad in ('invalid', 'true', '1', 'ON', ''):
            rejected = subprocess.run(
                [binary, '--replay', str(path), '--summary', '1', flag, bad],
                capture_output=True, text=True, timeout=30)
            assert rejected.returncode == 2, (flag, bad, rejected.stdout + rejected.stderr)
            assert flag + ' must be on or off' in rejected.stderr, (flag, bad, rejected.stderr)
            assert 'START ' not in rejected.stdout, (flag, bad, rejected.stdout)

    # The defaults are the historical behaviour and must stay byte-identical:
    # the START line reports both effective modes.
    baseline = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1'],
        capture_output=True, text=True, timeout=30)
    assert baseline.returncode == 0, baseline.stderr + baseline.stdout
    fields = start_fields(baseline.stdout)
    assert fields['fused'] == 'on', fields
    assert fields['cir'] == 'full', fields
    assert fields['taps'] == 'eq16', fields
    assert fields['ondemand'] == 'on', fields
    assert fields['lock_fcs'] == 'on', fields
    assert fields['stop_after_sfd'] == 'off', fields

    # Python wrapper default (--cir auto) and `--` passthrough to C++ live.
    wrapper = Path(__file__).resolve().parent / 'live_dw3000_packets.py'
    wrapper_env = dict(**subprocess.os.environ, UWB_LIVE_BINARY=binary)
    wrap_default = subprocess.run(
        [sys.executable, str(wrapper), '--replay', str(path), '--summary', '1'],
        capture_output=True, text=True, timeout=30, env=wrapper_env)
    assert wrap_default.returncode == 0, wrap_default.stderr + wrap_default.stdout
    wfields = start_fields(wrap_default.stdout)
    assert wfields['cir'] == 'auto', wfields
    assert wfields['fused'] == 'on', wfields
    assert wfields['taps'] == 'eq16', wfields

    wrap_passthrough = subprocess.run(
        [sys.executable, str(wrapper), '--replay', str(path), '--summary', '1',
         '--', '--sync', 'locked', '--taps', 'quality'],
        capture_output=True, text=True, timeout=30, env=wrapper_env)
    assert wrap_passthrough.returncode == 0, wrap_passthrough.stderr + wrap_passthrough.stdout
    wpfields = start_fields(wrap_passthrough.stdout)
    assert wpfields['cir'] == 'auto', wpfields
    assert wpfields['sync'] == 'locked', wpfields
    assert wpfields['taps'] == 'quality', wpfields

    wrap_timed_no_replay = subprocess.run(
        [sys.executable, str(wrapper), '--', '--mode', 'timed'],
        capture_output=True, text=True, timeout=15, env=wrapper_env)
    assert wrap_timed_no_replay.returncode == 2, (
        wrap_timed_no_replay.stdout + wrap_timed_no_replay.stderr)

    # Low-SNR capture-only smoke: FCS is dropped from lock identity and the
    # demodulator stops after the SFD.  The synthetic DC burst is still only a
    # rejected candidate, so the run must start, complete and exit cleanly.
    # `--cir auto` is passed explicitly because that is the only setting where
    # the 15 ms Full retry could fire; capture-only must suppress it.
    capture_only = subprocess.run(
        [binary, '--replay', str(path), '--summary', '1', '--cir', 'auto',
         '--lock-fcs', 'off', '--stop-after-sfd', 'on'],
        capture_output=True, text=True, timeout=60)
    assert capture_only.returncode == 0, capture_only.stderr + capture_only.stdout
    fields = start_fields(capture_only.stdout)
    assert fields['lock_fcs'] == 'off', fields
    assert fields['stop_after_sfd'] == 'on', fields
    assert re.search(r'^FINAL completed=1 fcs=0/1 ', capture_only.stdout, re.M), \
        capture_only.stdout
    # No FCS means no identity comparison, so no interference event may be
    # reported even though the candidate has a preamble+SFD-shaped metric.
    assert 'reason=identity_mismatch' not in capture_only.stdout, capture_only.stdout
    # And the capture-only run never pays for the --cir auto Full retry.
    assert re.search(r'^PACKET .*full_retry=0 ', capture_only.stdout, re.M), \
        capture_only.stdout

    # ------------------------------------------------------------------
    # Triangle-peak acquisition trigger.
    #
    # The 0.3 ms DC burst is longer than the 192k window, so the delayed
    # autocorrelation has a flat top over [burst_start, burst_start+len-span]
    # and the reported seed must land inside that top plus half a lag.  The
    # legacy leading-edge + span/2 seed would land ~96000 samples earlier.
    burst_start = start
    burst_len = length
    seeds = {}
    for step in (64, 3):
        run = subprocess.run(
            [binary, '--replay', str(path), '--summary', '1',
             '--acq-step', str(step)],
            capture_output=True, text=True, timeout=60)
        assert run.returncode == 0, run.stdout + run.stderr
        found = re.findall(r'^PACKET id=\d+ time_ms=([0-9.]+) status=(\S+)',
                           run.stdout, re.M)
        assert len(found) == 1, run.stdout
        seeds[step] = round(float(found[0][0]) * 737.28e3)
        assert found[0][1] == 'TIMING_FAILED', run.stdout
        # Inside the flat top (plus half a lag for the centred seed).
        assert burst_start - 750 <= seeds[step] <= burst_start + (burst_len - 192000) + 750, \
            (step, seeds[step])
        # Not the legacy leading-edge seed.
        assert seeds[step] > burst_start - 60000, (step, seeds[step])
    # The block kernel and the scalar tail must agree on one scale: any
    # --acq-step (a multiple of 8 or not) reports the same seed within one step.
    assert abs(seeds[64] - seeds[3]) <= 64, seeds

    # ------------------------------------------------------------------
    # Submission accounting invariants under queue overload.
    #
    # 10 bursts spaced by 1.1 ms (> 0.80 ms holdoff), processed with 1 worker,
    # queue capacity 1, and --taps quality (resampling alone > 1.5 ms/window),
    # guarantee queue saturation while fitting all 10 windows before EOF.
    # Every candidate ID must reach a unique terminal state and balance.
    # ------------------------------------------------------------------
    overload_path = Path(temporary) / 'overload_10_bursts.sc16'
    ov_count = round(737.28e6 * 0.016)
    ov_iq = array('i', [0]) * ov_count
    ov_len = round(737.28e6 * 0.00028)
    burst_val = array('i', [1000]) * ov_len
    for k in range(10):
        b_start = round(737.28e6 * (0.001 + k * 0.0011))
        ov_iq[b_start:b_start + ov_len] = burst_val
    with overload_path.open('wb') as f:
        ov_iq.tofile(f)

    ov_run = subprocess.run(
        [binary, '--replay', str(overload_path), '--workers', '1',
         '--queue', '1', '--taps', 'quality', '--tone', 'off',
         '--summary', '50', '--print', 'packets'],
        capture_output=True, text=True, timeout=60)
    assert ov_run.returncode == 0, ov_run.stderr + ov_run.stdout
    ov_packets = re.findall(r'^PACKET .*$', ov_run.stdout, re.M)
    ov_final_line = re.search(r'^FINAL .*$', ov_run.stdout, re.M)
    assert ov_final_line is not None, ov_run.stdout
    ov_final = dict(part.split('=', 1) for part in ov_final_line.group(0).split()[1:] if '=' in part)
    assert int(ov_final['acq_seeds']) == 10, ov_final
    assert int(ov_final['submit_attempts']) == 10, ov_final
    assert int(ov_final['admitted']) + int(ov_final['dropped_queue']) == 10, ov_final
    assert int(ov_final['dropped_queue']) > 0, ov_final
    assert int(ov_final['demod_completed']) == int(ov_final['admitted']), ov_final
    assert int(ov_final['completed']) == 10, ov_final
    assert len(ov_packets) == 10, (len(ov_packets), ov_run.stdout)
    seen_ids = []
    dropped_lines = 0
    for pkt in ov_packets:
        pfields = dict(part.split('=', 1) for part in pkt.split()[1:] if '=' in part)
        seen_ids.append(int(pfields['id']))
        assert 'path=acquire' in pkt, pkt
        if pfields['status'] == 'DROPPED_QUEUE':
            dropped_lines += 1
    assert sorted(seen_ids) == list(range(1, 11)), seen_ids
    assert dropped_lines == int(ov_final['dropped_queue']), (dropped_lines, ov_final)

print('live console: acquisition/rejection, batch summary, SIGINT, source failure, '
      'locked strategy, timed-burst plan, locked pre-guard, low-SNR '
      'lock/capture switches and submission accounting PASS')
