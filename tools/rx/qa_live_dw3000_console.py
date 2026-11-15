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
print('live console: acquisition/rejection, batch summary, SIGINT and source failure PASS')
