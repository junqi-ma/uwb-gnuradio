function matlab_resample_reference(varargin)
% MATLAB_RESAMPLE_REFERENCE  Independent upfirdn / polyphase reference for the
% frozen causal full-convolution resampler contract (G0 §3.1).
%
% Contract under test (gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h):
%     Lout = ceil(((N-1)*L + T) / M)
%     y[m] = sum_k h[(m*M mod L) + L*k] * x[floor(m*M/L) - k],  x[j]=0 outside [0,N)
%
% This script is the INDEPENDENT MATLAB oracle for A11 cross-check #1.  It:
%   1. loads the frozen M2-A taps (testdata/twr/m2a/taps/ by default),
%   2. runs MATLAB upfirdn AND an explicit polyphase implementation of the
%      formula above and cross-checks them against each other,
%   3. compares the MATLAB result sample-by-sample against a C++ resampler
%      output file (given as input) when that file is present.
%
% It does NOT run in this repository's CI: MATLAB is unavailable here, so the
% committed manifest records matlab_executed=false.  This .m file existing is
% not a pass.
%
% Usage (on a machine with MATLAB):
%   matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_resample_reference"
%   matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_resample_reference('cpp_dir','/path/to/cpp','taps_dir','/path/to/taps')"
%
% Name/value options:
%   'taps_dir'  directory of frozen taps (default ../taps, env TWR_M2A_TAPS_DIR)
%   'cpp_dir'   directory of C++ resampler outputs (default ../cpp, env TWR_M2A_CPP_DIR)
%   'out_dir'   where MATLAB writes its outputs (default ./out)
%   'fail_on_blocked'  logical, error out when a comparison cannot run (default false)
%
% C++ output naming contract (documented in ../README.md):
%   <cpp_dir>/<dirName>_<case>_cpp.cf32     e.g. tx_48_65_random_cpp.cf32
% where dirName is one of tx_48_65, tx_32_65, rx_65_48, rx_65_32 and case is
% one of impulse, zeros, random, twr_waveform.  The input is the committed
% inputs/<case>_in.cf32; the C++ side must have consumed exactly that file.

    opt = parse_opts(varargin{:});
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(fileparts(here))));
    opt = resolve_defaults(opt, here);
    if ~exist(opt.out_dir, 'dir'), mkdir(opt.out_dir); end

    have_upfirdn = (exist('upfirdn', 'file') == 2);
    fprintf('== matlab_resample_reference ==\n');
    fprintf('MATLAB %s; upfirdn available=%d\n', version, have_upfirdn);

    dirs = { ...
        struct('name', 'tx_48_65', 'L', 48, 'M', 65, ...
               'rate_in', 998.4e6, 'rate_out', 737.28e6), ...
        struct('name', 'tx_32_65', 'L', 32, 'M', 65, ...
               'rate_in', 998.4e6, 'rate_out', 491.52e6), ...
        struct('name', 'rx_65_48', 'L', 65, 'M', 48, ...
               'rate_in', 737.28e6, 'rate_out', 998.4e6), ...
        struct('name', 'rx_65_32', 'L', 65, 'M', 32, ...
               'rate_in', 491.52e6, 'rate_out', 998.4e6)};

    cases = {'impulse', 'zeros', 'random', 'twr_waveform'};

    max_abs_tol = 1e-4;
    rel_l2_tol = 1e-5;

    records = struct('direction', {}, 'case', {}, 'status', {}, ...
                     'n_in', {}, 'n_expected', {}, 'n_matlab', {}, ...
                     'n_cpp', {}, 'length_ok', {}, 'self_ok', {}, ...
                     'upfirdn_vs_polyphase_max_abs', {}, ...
                     'max_abs_error', {}, 'relative_l2', {}, ...
                     'zeros_exact', {}, 'cpp_file', {}, 'matlab_file', {}, ...
                     'taps_file', {}, 'note', {});

    n_failed = 0;
    n_blocked = 0;
    n_pass = 0;

    for di = 1:numel(dirs)
        d = dirs{di};
        [h, taps_file, taps_note] = load_taps_for(opt.taps_dir, d.L, d.M, repo);
        if isempty(h)
            fprintf('[BLOCKED] %s: %s\n', d.name, taps_note);
            for ci = 1:numel(cases)
                records(end+1) = blank_record(d.name, cases{ci}, ...
                    'blocked_no_taps', taps_note); %#ok<AGROW>
            end
            n_blocked = n_blocked + numel(cases);
            continue;
        end
        T = numel(h);
        fprintf('-- %s: L=%d M=%d T=%d taps=%s\n', d.name, d.L, d.M, T, taps_file);

        for ci = 1:numel(cases)
            c = cases{ci};
            in_file = fullfile(here, 'inputs', [c '_in.cf32']);
            if ~exist(in_file, 'file')
                records(end+1) = blank_record(d.name, c, ...
                    'blocked_no_input', in_file); %#ok<AGROW>
                n_blocked = n_blocked + 1;
                continue;
            end
            x = read_cf32(in_file);
            N = numel(x);
            Lout = ceil(((N - 1) * d.L + T) / d.M);

            if have_upfirdn
                y_up = upfirdn(x, h, d.L, d.M);
            else
                y_up = polyphase_ref(x, h, d.L, d.M);
            end
            y_poly = polyphase_ref(x, h, d.L, d.M);
            self_max = max_abs_diff(y_up, y_poly);

            rec = blank_record(d.name, c, 'unknown', '');
            rec.taps_file = taps_file;
            rec.n_in = N;
            rec.n_expected = Lout;
            rec.n_matlab = numel(y_up);
            rec.length_ok = (numel(y_up) == Lout);
            rec.upfirdn_vs_polyphase_max_abs = self_max;
            rec.self_ok = (self_max <= 1e-9) || (~have_upfirdn);
            rec.matlab_file = fullfile(opt.out_dir, sprintf('%s_%s_matlab.cf32', d.name, c));
            write_cf32(rec.matlab_file, y_up);

            cpp_file = fullfile(opt.cpp_dir, sprintf('%s_%s_cpp.cf32', d.name, c));
            rec.cpp_file = cpp_file;
            if ~exist(cpp_file, 'file')
                rec.status = 'blocked_no_cpp';
                rec.note = 'C++ resampler output not present';
                n_blocked = n_blocked + 1;
                fprintf('   [BLOCKED] %s/%s: %s\n', d.name, c, rec.note);
                records(end+1) = rec; %#ok<AGROW>
                continue;
            end

            y_cpp = read_cf32(cpp_file);
            rec.n_cpp = numel(y_cpp);
            if ~rec.length_ok || numel(y_cpp) ~= Lout
                rec.status = 'failed_length';
                rec.note = sprintf('length mismatch: expected %d, matlab %d, cpp %d', ...
                    Lout, numel(y_up), numel(y_cpp));
                n_failed = n_failed + 1;
                records(end+1) = rec; %#ok<AGROW>
                fprintf('   [FAIL] %s/%s: %s\n', d.name, c, rec.note);
                continue;
            end

            if strcmp(c, 'zeros')
                rec.zeros_exact = all(y_cpp == 0) && all(y_up == 0);
                rec.max_abs_error = 0;
                rec.relative_l2 = 0;
                if rec.zeros_exact
                    rec.status = 'pass';
                    n_pass = n_pass + 1;
                else
                    rec.status = 'failed_zeros';
                    rec.note = 'all-zero input must produce exact zero output';
                    n_failed = n_failed + 1;
                end
            else
                diff = y_up(:) - y_cpp(:);
                rec.max_abs_error = max(abs(diff));
                denom = max(norm(y_cpp(:)), eps);
                rec.relative_l2 = norm(diff) / denom;
                if rec.length_ok && rec.self_ok && ...
                        rec.max_abs_error <= max_abs_tol && ...
                        rec.relative_l2 <= rel_l2_tol
                    rec.status = 'pass';
                    n_pass = n_pass + 1;
                else
                    rec.status = 'failed_tolerance';
                    rec.note = sprintf('max_abs=%.3e rel_l2=%.3e', ...
                        rec.max_abs_error, rec.relative_l2);
                    n_failed = n_failed + 1;
                end
            end
            records(end+1) = rec; %#ok<AGROW>
            fprintf('   [%s] %s/%s  n=%d  max_abs=%.3e rel_l2=%.3e\n', ...
                upper(rec.status), d.name, c, Lout, rec.max_abs_error, rec.relative_l2);
        end
    end

    result = struct();
    result.schema = 'twr-m2a-matlab-resample-reference/1';
    result.generated_utc = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
    result.matlab = struct('executed', true, 'version', version, ...
        'command', 'matlab_resample_reference', 'exit_code', 0);
    result.have_upfirdn = have_upfirdn;
    result.tolerances = struct('max_abs_error', max_abs_tol, ...
        'relative_l2', rel_l2_tol, 'zeros', 'exact');
    result.taps_dir = opt.taps_dir;
    result.cpp_dir = opt.cpp_dir;
    result.out_dir = opt.out_dir;
    result.counts = struct('pass', n_pass, 'failed', n_failed, 'blocked', n_blocked);
    result.records = records;
    out_json = fullfile(opt.out_dir, 'matlab_resample_reference_result.json');
    write_json(out_json, result);
    fprintf('wrote %s\n', out_json);
    fprintf('== pass=%d failed=%d blocked=%d ==\n', n_pass, n_failed, n_blocked);

    if n_failed > 0
        error('matlab_resample_reference:Failed', ...
            '%d comparison(s) failed; see %s', n_failed, out_json);
    end
    if n_blocked > 0 && opt.fail_on_blocked
        error('matlab_resample_reference:Blocked', ...
            '%d comparison(s) blocked; see %s', n_blocked, out_json);
    end
end

% ---------------------------------------------------------------------------
function opt = parse_opts(varargin)
    opt = struct('taps_dir', '', 'cpp_dir', '', 'out_dir', '', ...
                 'fail_on_blocked', false);
    if mod(numel(varargin), 2) ~= 0
        error('options must be name/value pairs');
    end
    for i = 1:2:numel(varargin)
        name = varargin{i};
        if ~isfield(opt, name)
            error('unknown option: %s', name);
        end
        opt.(name) = varargin{i+1};
    end
end

function opt = resolve_defaults(opt, here)
    if isempty(opt.taps_dir)
        opt.taps_dir = getenv('TWR_M2A_TAPS_DIR');
        if isempty(opt.taps_dir)
            opt.taps_dir = fullfile(here, '..', 'taps');
        end
    end
    if isempty(opt.cpp_dir)
        opt.cpp_dir = getenv('TWR_M2A_CPP_DIR');
        if isempty(opt.cpp_dir)
            opt.cpp_dir = fullfile(here, '..', 'cpp');
        end
    end
    if isempty(opt.out_dir)
        opt.out_dir = fullfile(here, 'out');
    end
    opt.taps_dir = char(opt.taps_dir);
    opt.cpp_dir = char(opt.cpp_dir);
    opt.out_dir = char(opt.out_dir);
end

% ---------------------------------------------------------------------------
% Explicit polyphase implementation of the G0 §3.1 formula (independent of
% upfirdn).  O(N*T/L), only used on small/medium vectors.
function y = polyphase_ref(x, h, L, M)
    x = x(:);
    h = h(:);
    N = numel(x);
    T = numel(h);
    if N == 0 || T == 0
        y = complex(zeros(0, 1));
        return;
    end
    Lout = ceil(((N - 1) * L + T) / M);
    y = complex(zeros(Lout, 1));
    for m = 0:Lout-1
        arm = mod(m * M, L);
        base = floor(m * M / L);
        acc = complex(0);
        k = 0;
        while arm + L * k < T
            j = base - k;
            if j >= 0 && j < N
                acc = acc + h(arm + L * k + 1) * x(j + 1);
            end
            k = k + 1;
        end
        y(m + 1) = acc;
    end
end

function d = max_abs_diff(a, b)
    a = a(:);
    b = b(:);
    if numel(a) ~= numel(b)
        d = Inf;
        return;
    end
    if isempty(a)
        d = 0;
        return;
    end
    d = max(abs(a - b));
end

% ---------------------------------------------------------------------------
function [h, src, note] = load_taps_for(tapsDir, L, M, repoRoot)
    h = [];
    src = '';
    note = sprintf('no taps file for %d/%d under %s', L, M, tapsDir);
    if ~exist(tapsDir, 'dir')
        note = sprintf('taps dir does not exist: %s', tapsDir);
        return;
    end

    % 1) design.json discovery (preferred: records the exact file per direction).
    %    Agent A's schema uses frozen_file (the canonical independent design);
    %    file/taps_file are also accepted.
    dj = fullfile(tapsDir, 'design.json');
    if exist(dj, 'file')
        try
            d = jsondecode(fileread(dj));
            names = {};
            if isfield(d, 'directions') && ~isempty(d.directions)
                names = {'directions'};
            elseif isfield(d, 'taps') && ~isempty(d.taps)
                names = {'taps'};
            end
            for ni = 1:numel(names)
                arr = d.(names{ni});
                for ai = 1:numel(arr)
                    e = arr(ai);
                    [eL, eM] = entry_lm(e);
                    if eL == L && eM == M
                        for fld = {'frozen_file', 'file', 'taps_file'}
                            if isfield(e, fld{1})
                                f = fullfile(tapsDir, e.(fld{1}));
                                if exist(f, 'file')
                                    h = read_taps_file(f);
                                    src = f;
                                    note = '';
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        catch err
            note = sprintf('design.json parse failed (%s); falling back', err.message);
        end
    end

    % 2) conventional file names
    base = {sprintf('taps_%d_%d', L, M), ...
            sprintf('tx_taps_%d_%d', L, M), ...
            sprintf('rx_taps_%d_%d', L, M), ...
            sprintf('m2a_taps_%d_%d', L, M), ...
            sprintf('tx_%d_%d', L, M), ...
            sprintf('rx_%d_%d', L, M)};
    exts = {'.f32', '.bin', '.txt', '.csv'};
    for bi = 1:numel(base)
        for ei = 1:numel(exts)
            f = fullfile(tapsDir, [base{bi} exts{ei}]);
            if exist(f, 'file')
                h = read_taps_file(f);
                src = f;
                note = '';
                return;
            end
        end
    end

    % 3) repository RX prototype fallback (M2-A canonical RX taps)
    if ~isempty(repoRoot)
        if L == 65 && M == 48
            cand = fullfile(repoRoot, 'testdata', 'resampler_65_48', ...
                'taps_quality_minorder.txt');
        elseif L == 65 && M == 32
            cand = fullfile(repoRoot, 'testdata', 'resampler_65_32', ...
                'taps_quality_minorder.txt');
        else
            cand = '';
        end
        if ~isempty(cand) && exist(cand, 'file')
            h = read_taps_file(cand);
            src = cand;
            note = 'repo RX minorder fallback';
            return;
        end
    end
end

function [L, M] = entry_lm(e)
    L = NaN;
    M = NaN;
    if isfield(e, 'interp'), L = e.interp; end
    if isfield(e, 'l'), L = e.l; end
    if isfield(e, 'decim'), M = e.decim; end
    if isfield(e, 'm'), M = e.m; end
    if isfield(e, 'L'), L = e.L; end
    if isfield(e, 'M'), M = e.M; end
end

function h = read_taps_file(f)
    [~, ~, ext] = fileparts(f);
    ext = lower(ext);
    if strcmp(ext, '.f32') || strcmp(ext, '.bin')
        fid = fopen(f, 'rb');
        if fid < 0, error('cannot open taps %s', f); end
        h = fread(fid, Inf, 'float32=>double');
        fclose(fid);
    else
        h = readmatrix(f, 'FileType', 'text');
        h = h(:);
    end
    if isempty(h)
        error('empty taps file %s', f);
    end
end

% ---------------------------------------------------------------------------
function z = read_cf32(p)
    fid = fopen(p, 'rb');
    if fid < 0, error('cannot open %s', p); end
    raw = fread(fid, Inf, 'float32=>double');
    fclose(fid);
    if mod(numel(raw), 2) ~= 0
        error('cf32 file has odd float count: %s', p);
    end
    z = complex(raw(1:2:end), raw(2:2:end));
end

function write_cf32(p, z)
    z = z(:);
    inter = zeros(2 * numel(z), 1);
    inter(1:2:end) = real(z);
    inter(2:2:end) = imag(z);
    fid = fopen(p, 'wb');
    if fid < 0, error('cannot write %s', p); end
    fwrite(fid, inter, 'float32');
    fclose(fid);
end

function write_json(p, s)
    fid = fopen(p, 'w');
    if fid < 0, error('cannot write %s', p); end
    fwrite(fid, jsonencode(s));
    fclose(fid);
end

function rec = blank_record(direction, caseName, status, note)
    rec = struct('direction', direction, 'case', caseName, 'status', status, ...
        'n_in', 0, 'n_expected', 0, 'n_matlab', 0, 'n_cpp', 0, ...
        'length_ok', false, 'self_ok', false, ...
        'upfirdn_vs_polyphase_max_abs', NaN, ...
        'max_abs_error', NaN, 'relative_l2', NaN, 'zeros_exact', false, ...
        'cpp_file', '', 'matlab_file', '', 'taps_file', '', 'note', note);
end
