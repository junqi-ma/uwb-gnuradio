% verify_s2b_payload.m — S2-B independent MATLAB cross-check (STATUS: 未实跑)
%
% Given a C++ run directory produced by gr-uwb/apps/uwb_sim_sic_payload.py,
% reproduce the code-9 interferer finite SYNC train from the shared toolbox and
% compare it to the templates written by the driver, and re-check the archived
% actual suppression.  NOT executed for the S2-B delivery (未实跑).

function verify_s2b_payload(runDir)
    if nargin < 1
        here = fileparts(mfilename('fullpath'));
        runDir = fullfile(here, '..');
    end
    runDir = char(runDir);
    tplDir = fullfile(runDir, 'templates');

    % --- 1. code-9 finite SYNC train vs MATLAB reference ------------------
    cfg9 = lrwpanHRPConfig('Mode', '802.15.4a', 'MeanPRF', 62.4, ...
        'DataRate', 6.81, 'PreambleDuration', 64, 'CodeIndex', 9, ...
        'SamplesPerPulse', 2, 'PSDULength', 1);
    ref9 = lrwpanWaveformGenerator(uint8(zeros(1, 1)), cfg9);
    sync9 = ref9(1:1016);

    cxxSync9 = fullfile(tplDir, 'sensing_sync_code9_998p4.cf32');
    if exist(cxxSync9, 'file')
        x = read_cf32(cxxSync9);
        r = abs(sum(conj(sync9(:)) .* x(1:numel(sync9)))) / ...
            (norm(sync9) * norm(x(1:numel(sync9))));
        fprintf('[MATLAB] code-9 SYNC correlation = %.6f (expect ~1.0)\n', r);
    else
        fprintf('[MATLAB] missing %s\n', cxxSync9);
    end

    % --- 2. archived actual suppression recompute -------------------------
    reconDir = fullfile(runDir, 'recon');
    files = dir(fullfile(reconDir, 'window_*.npz'));
    nBad = 0;
    for k = 1:numel(files)
        p = fullfile(reconDir, files(k).name);
        % NOTE: MATLAB cannot read .npz directly; export to .mat first
        % (python3 -c "import numpy as np; ...") in a real run.
        fprintf('[MATLAB] (placeholder) %s\n', files(k).name);
        nBad = nBad + 0;
    end
    fprintf('[MATLAB] S2-B payload cross-check script NOT RUN for delivery\n');
end

function x = read_cf32(path)
    fid = fopen(path, 'rb');
    raw = fread(fid, [2, inf], 'single');
    fclose(fid);
    x = complex(raw(1, :).', raw(2, :).');
end
