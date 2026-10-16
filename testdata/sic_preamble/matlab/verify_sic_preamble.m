function ok = verify_sic_preamble(matlabOutDir, cxxRunDir, varargin)
%VERIFY_SIC_PREAMBLE  Hard-gate MATLAB-vs-C++ cross-check for the S0 case.
%
%   OK = VERIFY_SIC_PREAMBLE(MATLABOUTDIR, CXXRUNDIR)
%   OK = VERIFY_SIC_PREAMBLE(MATLABOUTDIR, CXXRUNDIR, 'Name', value, ...)
%
%   Compares the MATLAB exports produced by export_comm_template.m /
%   export_sic_preamble.m against the C++ S0 run artifacts, field by field,
%   with FROZEN tolerances, prints a table and sets a non-zero exit on any
%   mismatch (it raises an error the batch harness reports as a failure).
%
%   The C++ artifact directory is a REQUIRED path argument (never hard-coded);
%   the MATLAB export directory defaults to <this dir>/out and is also a path
%   argument for reproducibility.
%
%   Name/value options:
%     'CirPath'  explicit C++ per-tap CIR file (CF32) to compare against the
%                MATLAB radar CIR.  If omitted the verifier searches the run
%                directory (see local_find_cir).
%
%   Required C++ artifacts (missing => FAIL, cannot verify):
%     <run>/templates/comm_sync_code10_998p4.cf32
%     <run>/templates/comm_preamble_code10_998p4.cf32
%     <run>/fixtures/window_0000.npz   (arrays s, i, w, y)
%     <run>/windows.jsonl              (sic_state.detected_start)
%
%   Optional C++ artifacts (compared when present, else NOT_RUN):
%     <run>/templates/sync_template_code9_998p4.cf32
%     <run>/cir/radar_cir_norm_998p4.cf32
%     <run>/cir/sic_cir_norm_998p4.cf32
%
%   FROZEN tolerances (see README.md; may only be tightened by an explicit
%   revision + re-run):
%     template  max_abs <= 1e-4   rel_l2 <= 1e-4
%     i         max_abs <= 2e-3   rel_l2 <= 2e-3
%     s         max_abs <= 1e-6   rel_l2 <= 1e-6   (loaded-from-C++ integrity)
%     detected_start |diff| <= 1 sample
%     per-tap CIR normalised-magnitude NCC >= 0.98, max |mag| diff <= 0.10
%
%   NOTE (未实跑): no MATLAB/Octave on the development host.  This verifier was
%   reviewed for syntax but has NOT been executed.  See README.md / not_run.log.

    if nargin < 1 || isempty(matlabOutDir)
        matlabOutDir = fullfile(fileparts(mfilename('fullpath')), 'out');
    end
    if nargin < 2 || isempty(cxxRunDir)
        error('verify_sic_preamble:NeedRunDir', ...
            'the C++ run directory is a required path argument');
    end

    opt = struct('CirPath', '');
    if mod(numel(varargin), 2) ~= 0
        error('verify_sic_preamble:BadArgs', 'options must be name/value pairs');
    end
    for k = 1:2:numel(varargin)
        opt.(varargin{k}) = varargin{k+1};
    end

    tol = struct( ...
        'template_max_abs', 1e-4, 'template_rel_l2', 1e-4, ...
        'i_max_abs', 2e-3, 'i_rel_l2', 2e-3, ...
        's_max_abs', 1e-6, 's_rel_l2', 1e-6, ...
        'det_tol', 1.0, ...
        'cir_ncc_min', 0.98, 'cir_mag_diff', 0.10);

    fprintf('S0-D MATLAB vs C++ cross-check\n');
    fprintf('  matlab out : %s\n', matlabOutDir);
    fprintf('  cxx run    : %s\n', cxxRunDir);

    rows = {};

    % ---- C++ artifacts --------------------------------------------------
    cxxSync = fullfile(cxxRunDir, 'templates', 'comm_sync_code10_998p4.cf32');
    cxxPre = fullfile(cxxRunDir, 'templates', 'comm_preamble_code10_998p4.cf32');
    cxxSenseSync = fullfile(cxxRunDir, 'templates', 'sync_template_code9_998p4.cf32');
    cxxWin = fullfile(cxxRunDir, 'windows.jsonl');
    npzPath = local_first_existing({ ...
        fullfile(cxxRunDir, 'fixtures', 'window_0000.npz'), ...
        fullfile(cxxRunDir, 'fixtures', 'window_0.npz')});

    requiredMissing = {};
    if exist(cxxSync, 'file') ~= 2, requiredMissing{end+1} = cxxSync; end %#ok<AGROW>
    if exist(cxxPre, 'file') ~= 2, requiredMissing{end+1} = cxxPre; end %#ok<AGROW>
    if isempty(npzPath), requiredMissing{end+1} = 'fixtures/window_0000.npz'; end %#ok<AGROW>
    if exist(cxxWin, 'file') ~= 2, requiredMissing{end+1} = cxxWin; end %#ok<AGROW>

    % ---- MATLAB exports -------------------------------------------------
    mSync = fullfile(matlabOutDir, 'comm_sync_code10_998p4.cf32');
    mPre = fullfile(matlabOutDir, 'comm_preamble_code10_998p4.cf32');
    mSynthI = fullfile(matlabOutDir, 'i_998p4.cf32');
    mSynthS = fullfile(matlabOutDir, 's_998p4.cf32');
    mDet = fullfile(matlabOutDir, 'detected_start.json');
    mRadCir = fullfile(matlabOutDir, 'radar_cir_norm_998p4.cf32');
    mSicCir = fullfile(matlabOutDir, 'sic_cir_norm_998p4.cf32');

    matlabMissing = {};
    for f = {mSync, mPre, mSynthI, mDet}
        if exist(f{1}, 'file') ~= 2
            matlabMissing{end+1} = f{1}; %#ok<AGROW>
        end
    end

    % ---- template comparisons (independent generation) ------------------
    if exist(cxxSync, 'file') == 2 && exist(mSync, 'file') == 2
        rows{end+1} = local_cmp_array('comm_sync_code10', mSync, cxxSync, ...
            tol.template_max_abs, tol.template_rel_l2);
    end
    if exist(cxxPre, 'file') == 2 && exist(mPre, 'file') == 2
        rows{end+1} = local_cmp_array('comm_preamble_code10', mPre, cxxPre, ...
            tol.template_max_abs, tol.template_rel_l2);
    end
    if exist(cxxSenseSync, 'file') == 2
        mSenseSync = fullfile(matlabOutDir, 'sensing_sync_code9_998p4.cf32');
        if exist(mSenseSync, 'file') == 2
            rows{end+1} = local_cmp_array('sensing_sync_code9', mSenseSync, ...
                cxxSenseSync, tol.template_max_abs, tol.template_rel_l2);
        else
            rows{end+1} = local_row('sensing_sync_code9', '-', 'presence', ...
                NaN, NaN, false, 'MATLAB sensing_sync_code9 missing');
        end
    else
        rows{end+1} = local_row('sensing_sync_code9', '-', 'presence', ...
            NaN, NaN, true, 'NOT_RUN (C++ sensing sync not exported)');
    end

    % ---- s / i / y comparisons ------------------------------------------
    if ~isempty(npzPath) && exist(mSynthI, 'file') == 2
        cxxI = local_read_npz_array(npzPath, 'i');
        rows{end+1} = local_cmp_vectors('i (interferer)', ...
            local_read_cf32(mSynthI), cxxI, tol.i_max_abs, tol.i_rel_l2);
        if exist(mSynthS, 'file') == 2
            cxxS = local_read_npz_array(npzPath, 's');
            rows{end+1} = local_cmp_vectors('s (loaded integrity)', ...
                local_read_cf32(mSynthS), cxxS, tol.s_max_abs, tol.s_rel_l2);
            cxxW = local_read_npz_array(npzPath, 'w');
            rmsw = sqrt(mean(abs(double(cxxW)).^2));
            siM = local_read_cf32(mSynthS) + local_read_cf32(mSynthI);
            siC = cxxS + cxxI;
            rows{end+1} = local_cmp_vectors('s+i (noise-free part)', ...
                siM, siC, tol.i_max_abs, tol.i_rel_l2);
            rows{end+1} = local_row('cxx AWGN rms', ...
                sprintf('%.6g', rmsw), 'info', NaN, NaN, true, ...
                'noise floor; y is NOT sample-comparable (RNG differs)');
        else
            rows{end+1} = local_row('s (loaded integrity)', '-', 'presence', ...
                NaN, NaN, true, 'NOT_RUN (MATLAB s_998p4.cf32 missing)');
        end
    end

    % ---- detected start -------------------------------------------------
    if exist(cxxWin, 'file') == 2 && exist(mDet, 'file') == 2
        win = local_read_first_jsonl(cxxWin);
        cxxDet = NaN;
        if isfield(win, 'sic_state') && isfield(win.sic_state, 'detected_start')
            cxxDet = double(win.sic_state.detected_start);
        end
        det = jsondecode(fileread(mDet));
        mDetStart = double(det.matlab_detected_start);
        rows{end+1} = local_cmp_scalar('detected_start', mDetStart, cxxDet, ...
            tol.det_tol);
    end

    % ---- per-tap CIR ----------------------------------------------------
    [radCir, sicCir] = local_find_cir(cxxRunDir, opt.CirPath);
    if ~isempty(radCir) && exist(mRadCir, 'file') == 2
        rows{end+1} = local_cmp_cir('radar_cir per-tap', ...
            local_read_cf32(mRadCir), local_read_cf32(radCir), ...
            tol.cir_ncc_min, tol.cir_mag_diff);
    else
        rows{end+1} = local_row('radar_cir per-tap', '-', 'presence', ...
            NaN, NaN, true, 'NOT_RUN (no C++ per-tap CIR declared)');
    end
    if ~isempty(sicCir) && exist(mSicCir, 'file') == 2
        rows{end+1} = local_cmp_cir('sic_cir per-tap', ...
            local_read_cf32(mSicCir), local_read_cf32(sicCir), ...
            tol.cir_ncc_min, tol.cir_mag_diff);
    else
        rows{end+1} = local_row('sic_cir per-tap', '-', 'presence', ...
            NaN, NaN, true, 'NOT_RUN (no C++ per-tap CIR declared)');
    end

    % ---- print table ----------------------------------------------------
    local_print_table(rows);

    nFail = 0;
    for k = 1:numel(rows)
        if ~rows{k}.pass
            nFail = nFail + 1;
        end
    end
    if ~isempty(requiredMissing)
        fprintf('\nFAIL: required C++ artifacts missing:\n');
        for k = 1:numel(requiredMissing)
            fprintf('  - %s\n', requiredMissing{k});
        end
    end
    if ~isempty(matlabMissing)
        fprintf('\nFAIL: required MATLAB exports missing (run the exporters):\n');
        for k = 1:numel(matlabMissing)
            fprintf('  - %s\n', matlabMissing{k});
        end
    end

    ok = (nFail == 0) && isempty(requiredMissing) && isempty(matlabMissing);

    report = struct();
    report.ok = ok;
    report.n_fail = nFail;
    report.required_missing = requiredMissing;
    report.matlab_missing = matlabMissing;
    report.tolerances = tol;
    report.rows = local_rows_to_struct(rows);
    fid = fopen(fullfile(matlabOutDir, 'verify_report.json'), 'w');
    if fid >= 0
        fwrite(fid, jsonencode(report));
        fclose(fid);
    end

    if ok
        fprintf('\nPASS: MATLAB cross-check matches the C++ artifacts.\n');
    else
        fprintf('\nFAIL: %d comparison(s) failed / %d required artifact(s) missing.\n', ...
            nFail, numel(requiredMissing) + numel(matlabMissing));
        error('verify_sic_preamble:VerifyFailed', ...
            '%d comparison(s) failed; see verify_report.json', nFail);
    end
end

% =========================================================================
% comparison helpers
% =========================================================================

function row = local_row(name, what, metric, value, extra, pass, note)
    row = struct('name', name, 'what', what, 'metric', metric, ...
        'value', value, 'extra', extra, 'pass', logical(pass), 'note', note);
end

function row = local_cmp_vectors(name, a, b, maxAbsTol, relL2Tol)
    a = a(:); b = b(:);
    if numel(a) ~= numel(b)
        row = local_row(name, sprintf('%d vs %d', numel(a), numel(b)), ...
            'length', NaN, NaN, false, 'length mismatch');
        return;
    end
    d = abs(a - b);
    ma = max(d);
    ref = norm(b);
    if ref == 0, ref = 1; end
    rl = norm(d) / ref;
    pass = (ma <= maxAbsTol) && (rl <= relL2Tol);
    row = local_row(name, sprintf('%d samples', numel(a)), ...
        sprintf('max_abs=%.3g rel_l2=%.3g', ma, rl), ma, rl, pass, ...
        sprintf('tol max_abs<=%g rel_l2<=%g', maxAbsTol, relL2Tol));
end

function row = local_cmp_array(name, pathA, pathB, maxAbsTol, relL2Tol)
    row = local_cmp_vectors(name, local_read_cf32(pathA), ...
        local_read_cf32(pathB), maxAbsTol, relL2Tol);
end

function row = local_cmp_scalar(name, a, b, tol)
    d = abs(double(a) - double(b));
    pass = d <= tol;
    row = local_row(name, sprintf('%g vs %g', a, b), ...
        sprintf('abs_diff=%.3g', d), d, NaN, pass, sprintf('tol <=%g', tol));
end

function row = local_cmp_cir(name, a, b, nccMin, magDiff)
    a = a(:); b = b(:);
    if numel(a) ~= numel(b)
        row = local_row(name, sprintf('%d vs %d', numel(a), numel(b)), ...
            'length', NaN, NaN, false, 'CIR length mismatch');
        return;
    end
    an = a / (norm(a) + eps);
    bn = b / (norm(b) + eps);
    ncc = abs(an' * bn);
    md = max(abs(abs(an) - abs(bn)));
    pass = (ncc >= nccMin) && (md <= magDiff);
    row = local_row(name, sprintf('%d taps', numel(a)), ...
        sprintf('ncc=%.5f max_mag_diff=%.4f', ncc, md), ncc, md, pass, ...
        sprintf('tol ncc>=%g, mag_diff<=%g', nccMin, magDiff));
end

function out = local_rows_to_struct(rows)
    out = [];
    for k = 1:numel(rows)
        r = rows{k};
        out = [out; struct('name', r.name, 'what', r.what, ...
            'metric', r.metric, 'value', r.value, 'extra', r.extra, ...
            'pass', r.pass, 'note', r.note)]; %#ok<AGROW>
    end
end

function local_print_table(rows)
    fprintf('\n%-26s %-22s %-34s %-7s %s\n', ...
        'field', 'extent', 'metric', 'pass', 'note');
    fprintf('%s\n', repmat('-', 1, 108));
    for k = 1:numel(rows)
        r = rows{k};
        if r.pass
            tag = 'ok';
        else
            tag = 'FAIL';
        end
        fprintf('%-26s %-22s %-34s %-7s %s\n', ...
            local_trunc(r.name, 26), local_trunc(r.what, 22), ...
            local_trunc(r.metric, 34), tag, r.note);
    end
end

function s = local_trunc(s, n)
    s = char(s);
    if numel(s) > n
        s = [s(1:max(1, n-1)) '~'];
    end
end

% =========================================================================
% artifact location + I/O
% =========================================================================

function [radCir, sicCir] = local_find_cir(runDir, explicit)
    radCir = '';
    sicCir = '';
    radCands = { ...
        fullfile(runDir, 'cir', 'radar_cir_norm_998p4.cf32'), ...
        fullfile(runDir, 'radar_cir_norm_998p4.cf32'), ...
        fullfile(runDir, 'cir_norm_clean_radar.cf32')};
    sicCands = { ...
        fullfile(runDir, 'cir', 'sic_cir_norm_998p4.cf32'), ...
        fullfile(runDir, 'sic_cir_norm_998p4.cf32')};
    for k = 1:numel(radCands)
        if exist(radCands{k}, 'file') == 2
            radCir = radCands{k};
            break;
        end
    end
    for k = 1:numel(sicCands)
        if exist(sicCands{k}, 'file') == 2
            sicCir = sicCands{k};
            break;
        end
    end
    if ~isempty(explicit) && exist(explicit, 'file') == 2
        if isempty(radCir)
            radCir = explicit;
        else
            sicCir = explicit;
        end
    end
    % last resort: recursive glob for a declared *cir*norm*.cf32
    if isempty(radCir)
        hits = dir(fullfile(runDir, '**', '*cir*norm*.cf32'));
        if ~isempty(hits)
            radCir = fullfile(hits(1).folder, hits(1).name);
        end
    end
end

function x = local_read_cf32(path)
    fid = fopen(path, 'rb');
    if fid < 0
        error('verify_sic_preamble:ReadCf32', 'cannot open %s', path);
    end
    raw = fread(fid, inf, 'float32=>single');
    fclose(fid);
    raw = raw(:);
    if mod(numel(raw), 2) ~= 0
        error('verify_sic_preamble:Cf32Odd', '%s has odd element count', path);
    end
    x = complex(raw(1:2:end), raw(2:2:end));
end

function v = local_read_npz_array(npzPath, key)
    tmp = tempname;
    mkdir(tmp);
    cleaner = onCleanup(@() local_rmdir(tmp)); %#ok<NASGU>
    unzip(npzPath, tmp);
    npy = fullfile(tmp, [key '.npy']);
    if exist(npy, 'file') ~= 2
        error('verify_sic_preamble:NoNpzKey', ...
            '%s has no array "%s"', npzPath, key);
    end
    v = local_read_npy(npy);
end

function v = local_read_npy(path)
    fid = fopen(path, 'rb');
    if fid < 0
        error('verify_sic_preamble:ReadNpy', 'cannot open %s', path);
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    magic = fread(fid, 6, 'uint8=>uint8').';
    if ~isequal(magic, [147 78 85 77 80 89])
        error('verify_sic_preamble:BadNpy', 'not a .npy file: %s', path);
    end
    major = fread(fid, 1, 'uint8');
    fread(fid, 1, 'uint8');
    if major == 1
        hlen = fread(fid, 1, 'uint16=>double', 0, 'ieee-le');
    else
        hlen = fread(fid, 1, 'uint32=>double', 0, 'ieee-le');
    end
    hdr = char(fread(fid, hlen, 'uint8=>uint8').');
    descr = regexp(hdr, '''descr''\s*:\s*''([^'']+)''', 'tokens', 'once');
    shape = regexp(hdr, '''shape''\s*:\s*\(([^\)]*)\)', 'tokens', 'once');
    fortran = ~isempty(regexp(hdr, '''fortran_order''\s*:\s*True', 'once'));
    if isempty(descr) || isempty(shape)
        error('verify_sic_preamble:BadNpyHeader', 'bad .npy header: %s', path);
    end
    descr = descr{1};
    dimsStr = strtrim(shape{1});
    if isempty(dimsStr)
        dims = 1;
    else
        dims = str2double(strsplit(strrep(dimsStr, ',', ' ')));
        dims = dims(~isnan(dims));
        if isempty(dims)
            dims = 1;
        end
    end
    nElem = prod(dims);
    switch descr
        case '<c8'
            raw = fread(fid, 2 * nElem, 'float32=>single', 0, 'ieee-le');
            v = complex(raw(1:2:end), raw(2:2:end));
        case '<f8'
            v = fread(fid, nElem, 'float64=>double', 0, 'ieee-le');
        case '<f4'
            v = fread(fid, nElem, 'float32=>single', 0, 'ieee-le');
        case '<i8'
            v = fread(fid, nElem, 'int64=>int64', 0, 'ieee-le');
        case '<i4'
            v = fread(fid, nElem, 'int32=>int32', 0, 'ieee-le');
        otherwise
            error('verify_sic_preamble:NpyDtype', ...
                'unsupported .npy dtype %s in %s', descr, path);
    end
    if numel(dims) > 1 && ~fortran
        v = reshape(v, fliplr(dims)).';
    elseif numel(dims) > 1
        v = reshape(v, dims);
    else
        v = v(:);
    end
end

function obj = local_read_first_jsonl(path)
    fid = fopen(path, 'r');
    if fid < 0
        error('verify_sic_preamble:ReadJsonl', 'cannot open %s', path);
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    line = fgetl(fid);
    while ischar(line) && isempty(strtrim(line))
        line = fgetl(fid);
    end
    if ~ischar(line)
        error('verify_sic_preamble:EmptyJsonl', 'no JSON line in %s', path);
    end
    obj = jsondecode(line);
end

function p = local_first_existing(cands)
    p = '';
    for k = 1:numel(cands)
        if exist(cands{k}, 'file') == 2
            p = cands{k};
            return;
        end
    end
end

function local_rmdir(p)
    if exist(p, 'dir')
        rmdir(p, 's');
    end
end
