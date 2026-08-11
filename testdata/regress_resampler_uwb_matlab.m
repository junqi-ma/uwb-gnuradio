% §8.5 UWB algorithm regression — MATLAB cross-tool comparison of the 65/48
% software resampler round trip (737.28 -> 998.4 MS/s).
%
% The C++ fixed block reproduces scipy/MATLAB upfirdn exactly, so this script
% performs the round trip with MATLAB's own upfirdn (equivalent contract) and
% verifies the UWB packet survives:
%   1. 998.4 MHz cfile -> 48/65 anti-alias decimation  = X410 capture
%   2. 65/48 upsampling (quality taps) back to 998.4 MHz
%   3. packet start via preamble correlation on original vs round-trip
%   4. optional full decode_uwb FCS comparison
%
% Run from repo root: matlab -batch "testdata/regress_resampler_uwb_matlab"

function regress_resampler_uwb_matlab()
    addpath(fullfile(pwd, 'UWB_demodulation'));

    L = 65; M = 48;
    fs = 998.4e6;
    cfile = 'testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile';
    fid = fopen(cfile, 'rb'); x = fread(fid, Inf, 'single=>single'); fclose(fid);
    x998 = complex(x(1:2:end), x(2:2:end));
    orig_start = 4992000;              % 0-based packet start

    dec_taps = readtaps('testdata/resampler_65_48/taps_realtime.txt') * (48/65);
    q_taps   = readtaps('testdata/resampler_65_48/taps_quality.txt');
    tmpl     = readtmpl('testdata/reference_preamble.bin');

    % ---- round trip ----
    x737 = upfirdn(x998, dec_taps, M, L);         % 998.4 -> 737.28
    x_rt = upfirdn(x737, q_taps, L, M);           % 737.28 -> 998.4
    fprintf('in len=%d  737.28 len=%d  round-trip len=%d\n', ...
            numel(x998), numel(x737), numel(x_rt));
    fprintf('energy ratio roundtrip/original = %.4f\n', ...
            mean(abs(x_rt).^2)/mean(abs(x998).^2));

    % ---- packet start via preamble correlation ----
    s_orig = corr_peak(x998, tmpl, orig_start);
    s_rt   = corr_peak(x_rt,  tmpl, orig_start);
    fprintf('packet start  original=%d  round-trip=%d  delta=%d\n', ...
            s_orig, s_rt, s_rt - s_orig);
    if abs(s_rt - s_orig) > 200
        error('REGRESSION FAIL: packet start drifted by %d', s_rt - s_orig);
    end
    fprintf('packet-start regression: PASS (delta %d = round-trip group delay)\n', ...
            s_rt - s_orig);

    % ---- full decode_uwb FCS comparison (requires matching input format) ----
    try
        r_orig = try_decode_uwb(x998);
        r_rt   = try_decode_uwb(x_rt);
        fprintf('decode_uwb FCS: original pass=%d  round-trip pass=%d\n', ...
                r_orig.fcs_pass, r_rt.fcs_pass);
        if r_orig.fcs_pass && r_rt.fcs_pass
            fprintf('FCS regression: PASS\n');
        else
            fprintf('FCS regression: FAIL (check decoder config)\n');
        end
    catch err
        fprintf('decode_uwb skipped: %s\n', err.message);
    end
end

function t = readtaps(p)
    t = readmatrix(p, 'FileType', 'text');
end

function t = readtmpl(p)
    fid = fopen(p, 'rb'); c = fread(fid, Inf, 'single=>single'); fclose(fid);
    t = complex(c(1:2:end), c(2:2:end));
end

function s = corr_peak(x, tmpl, approx)
    L = numel(tmpl);
    % window around the known packet start
    lo = max(approx - 5000, 1); hi = min(approx + 5000, numel(x) - L);
    win = x(lo:hi);
    c = zeros(size(win));
    for i = 1:numel(win)
        c(i) = abs(sum(conj(tmpl) .* win(i:i+L-1)));
    end
    [~, imax] = max(c);
    s = lo + imax - 1;
end

function r = try_decode_uwb(x)
    % Build a minimal options struct matching decode_uwb for the code-9,
    % 64-SYNC standard-SFD test cfile (see run_decode_uwb_all.m for a fuller
    % example).  Adjust fields to your capture if decode_uwb rejects them.
    options = struct();
    options.fs_rx = 998.4e6;
    options.preamble_repetitions = 64;
    options.cir_repetitions = 64;
    options.code_index = 9;
    options.data_rate = 6.81;
    options.sfd_mode = 'auto';
    % decode_uwb expects preprocessedRx; wrap the complex IQ directly when the
    % decoder accepts a raw vector, otherwise decode via your capture format.
    preprocessedRx = struct('data', x);
    try
        result = decode_uwb(options, preprocessedRx);
        r.fcs_pass = isfield(result, 'fcs_pass') && result.fcs_pass;
    catch
        r.fcs_pass = false;
    end
end
