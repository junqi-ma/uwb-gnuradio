function ok = verify_s2a_cir(fixtureDir)
%VERIFY_S2A_CIR  Independent MATLAB cross-check of one exported S2-A window.
%
%   OK = VERIFY_S2A_CIR(FIXTUREDIR) reads the self-contained fixture written by
%   testdata/sic_accum/export_matlab_fixture.py and re-derives, in MATLAB with
%   an explicit double loop, the frozen S2-A fixed-operator CIR
%
%       H[r,t] = sum_m y[origin + r*sps + t + m] * conj(code[m]) / code_energy
%
%   for the three paired branches (clean, raw, sic).  It compares the result
%   against the archived expected_H_{clean,raw,sic} tensors (max abs error and
%   relative error) and prints an explicit PASS/FAIL per branch against a frozen
%   tolerance.  It then recomputes the CIR SIC gain
%
%       C_CIR(N) = 10*log10( ||E_raw,N||^2 / ||E_sic,N||^2 ),
%       E_x,N = mean_{r in [0,N)} H_x[r] - mean_{r in [0,N)} H_clean[r]
%
%   at N=1 and N=64 over the full tap grid and over the target ROI offsets
%   [35,44), and prints the values so they can be compared with the Python-side
%   windows.jsonl numbers.  NOTE: this script CANNOT read windows.jsonl; it only
%   prints its own numbers for a human/automated diff.
%
%   Only base MATLAB is required (fread / jsondecode / file IO); no toolbox.
%
%   STATUS: **未实跑 / NOT EXECUTED.**  No MATLAB or Octave is installed on the
%   development host.  This script was reviewed for syntax only and has NOT been
%   run; no MATLAB-verified result is claimed.  MATLAB is NOT required for the
%   normal S2-A pipeline (the NumPy reference + tools/sic/verify_s2a.py are the
%   primary path).  See matlab/README.md.
%
%   Frozen tolerance (may only be tightened by an explicit revision + re-run):
%       max_abs <= 1e-4  and  rel (vs max|expected|) <= 1e-4
%   (the expected tensors are stored complex64; a double-precision recompute of
%   the same samples agrees to ~1e-6).

    if nargin < 1 || isempty(fixtureDir)
        fixtureDir = fullfile(fileparts(mfilename('fullpath')), 'out');
    end
    tolAbs = 1e-4;
    tolRel = 1e-4;

    fprintf('================================================================\n');
    fprintf('S2-A independent MATLAB CIR cross-check\n');
    fprintf('  MATLAB : %s\n', version);
    fprintf('  fixture: %s\n', fixtureDir);
    fprintf('  WARNING: this script was NOT executed by the author of the\n');
    fprintf('           change (未实跑). Output below is a fresh run only if you\n');
    fprintf('           are reading it while executing the script yourself.\n');
    fprintf('================================================================\n');

    jsonPath = fullfile(fixtureDir, 'fixture.json');
    if exist(jsonPath, 'file') ~= 2
        error('verify_s2a_cir:NoFixture', 'missing %s', jsonPath);
    end
    f = jsondecode(fileread(jsonPath));

    origin     = double(f.origin);
    sps        = double(f.sps);
    sync_reps  = double(f.sync_reps);
    tap_lo     = double(f.tap_lo);
    tap_hi     = double(f.tap_hi);
    ntaps      = double(f.ntaps);
    codeEnergy = double(f.code_energy);

    fprintf('cell_id=%s  window_id=%d  origin=%d  sps=%d  sync_reps=%d\n', ...
        f.cell_id, double(f.window_id), origin, sps, sync_reps);
    fprintf('tap grid [%d,%d)  ntaps=%d  code_energy=%.6f  samples=%d\n', ...
        tap_lo, tap_hi, ntaps, codeEnergy, double(f.sample_count));

    code = local_read_cf32(fullfile(fixtureDir, 'code.cf32'));
    if numel(code) ~= sps
        error('verify_s2a_cir:CodeLength', ...
            'code length %d != sps %d', numel(code), sps);
    end
    y = struct();
    H = struct();
    Hexp = struct();
    branches = {'clean', 'raw', 'sic'};
    for b = 1:numel(branches)
        name = branches{b};
        y.(name) = local_read_cf32(fullfile(fixtureDir, [name '.cf32']));
        n = numel(y.(name));
        if origin < 0 || origin + sync_reps * sps > n
            error('verify_s2a_cir:TooShort', ...
                '%s window %d too short for origin %d', name, n, origin);
        end
        Hexp.(name) = local_read_cf32( ...
            fullfile(fixtureDir, ['expected_H_' name '.cf32']));
        if numel(Hexp.(name)) ~= sync_reps * ntaps
            error('verify_s2a_cir:Shape', ...
                'expected_H_%s has %d elements, want %d', ...
                name, numel(Hexp.(name)), sync_reps * ntaps);
        end
        Hexp.(name) = reshape(Hexp.(name), sync_reps, ntaps);
    end

    % ---- independent matched filter: explicit double loop over (r, t) -----
    nfail = 0;
    fprintf('\n-- H recompute (explicit (r,t) loop, sum over the code) --\n');
    for b = 1:numel(branches)
        name = branches{b};
        Hr = complex(zeros(sync_reps, ntaps));
        for r = 0:sync_reps - 1
            base = origin + r * sps;          % 0-based start of repetition r
            for ti = 0:ntaps - 1
                t = tap_lo + ti;              % tap offset in work samples
                s0 = base + t + 1;            % MATLAB 1-based first sample
                seg = y.(name)(s0:s0 + sps - 1);
                Hr(r + 1, ti + 1) = sum(seg .* conj(code)) / codeEnergy;
            end
        end
        H.(name) = Hr;
        expd = Hexp.(name);
        finiteMask = ~isnan(real(expd)) & ~isnan(imag(expd));
        d = abs(Hr(finiteMask) - expd(finiteMask));
        maxAbs = max(d);
        scale = max(abs(expd(finiteMask)));
        relErr = maxAbs / max(scale, eps);
        pass = (maxAbs <= tolAbs) && (relErr <= tolRel);
        if ~pass
            nfail = nfail + 1;
        end
        fprintf(['  %-5s  max|H|=%9.5f  max_abs_err=%.3e  rel=%.3e  ' ...
                 'nan_cells=%d  -> %s\n'], ...
            name, scale, maxAbs, relErr, ...
            numel(expd) - nnz(finiteMask), local_verdict(pass));
    end

    % ---- C_CIR at N = 1 and N = 64 ----------------------------------------
    fprintf('\n-- C_CIR(N) = 10log10(||E_raw,N||^2 / ||E_sic,N||^2) --\n');
    fprintf('  (printed for comparison; this script cannot read windows.jsonl)\n');
    roiLo = 35; roiHi = 44;                   % frozen target ROI offsets
    cLo = roiLo - tap_lo + 1;
    cHi = roiHi - tap_lo;                     % [cLo, cHi] inclusive columns
    if cLo < 1 || cHi > ntaps
        error('verify_s2a_cir:Roi', 'target ROI outside the tap grid');
    end
    Ns = [1, 64];
    for k = 1:numel(Ns)
        N = Ns(k);
        hc = mean(H.clean(1:N, :), 1);
        hr = mean(H.raw(1:N, :), 1);
        hs = mean(H.sic(1:N, :), 1);
        Er = hr - hc;
        Es = hs - hc;
        cFull = local_db(sum(abs(Er).^2), sum(abs(Es).^2));
        cTgt = local_db(sum(abs(Er(cLo:cHi)).^2), sum(abs(Es(cLo:cHi)).^2));
        fprintf(['  N=%2d  C_CIR_full=%12.6f dB  C_CIR_targetROI[%d,%d)=%12.6f ' ...
                 'dB\n'], N, cFull, roiLo, roiHi, cTgt);
    end

    ok = (nfail == 0);
    fprintf('\n');
    if ok
        fprintf('RESULT: PASS (all branches within tol_abs=%.1e, tol_rel=%.1e)\n', ...
            tolAbs, tolRel);
    else
        fprintf('RESULT: FAIL (%d branch(es) outside tolerance)\n', nfail);
    end
    fprintf('Reminder: 未实跑 unless this is your own execution; MATLAB is not\n');
    fprintf('required for the normal S2-A pipeline.\n');
end


function v = local_db(num, den)
    if num <= 0 && den <= 0
        v = NaN;                              % N/A (exact 0/0, no eps floor)
    elseif den <= 0
        v = Inf;                              % positive numerator / 0
    elseif num <= 0
        v = -Inf;                             % 0 / positive denominator
    else
        v = 10 * log10(num / den);
    end
end


function s = local_verdict(pass)
    if pass
        s = 'PASS';
    else
        s = 'FAIL';
    end
end


function x = local_read_cf32(path)
    fid = fopen(path, 'rb');
    if fid < 0
        error('verify_s2a_cir:ReadCf32', 'cannot open %s', path);
    end
    raw = fread(fid, inf, 'float32=>single');   % interleaved I/Q
    fclose(fid);
    raw = raw(:);
    if mod(numel(raw), 2) ~= 0
        error('verify_s2a_cir:Cf32Odd', '%s has odd element count', path);
    end
    x = complex(double(raw(1:2:end)), double(raw(2:2:end)));
end
