function matlab_decode_cpp_iq(varargin)
% MATLAB_DECODE_CPP_IQ  Decode a C++ native-chain IQ file with the repository's
% independent decoder UWB_demodulation/decode_uwb.m and compare to the frozen
% twr-m2a-native/1 result JSON (G0 §8).
%
% This is A11 cross-check #2.  It:
%   1. reads the C++ twr-m2a-native/1 JSON and the native CF32 IQ it names,
%   2. resamples native->work with the frozen RX contract
%      (upfirdn(x, rx_taps, L, M), Lout = ceil(((N-1)*L+T)/M)) unless the file
%      is already at the 998.4 MS/s work rate,
%   3. calls decode_uwb on the work-rate vector (preprocessedRx is a numeric
%      vector), and
%   4. compares PHR length, the full PSDU bytes, FCS and the diagnostic
%      coordinates.
%
% INDEX BASES: decode_uwb reports 1-based MATLAB sample/chip indices.  The C++
% twr-m2a-native/1 diagnostic coordinates are 0-based absolute samples.  The
% conversion (subtract 1) is done explicitly below; it is never implicit.
%
% FAILURE POLICY: this script never catches an exception and reports PASS.
% A decoder exception writes a failed record and is rethrown, so matlab -batch
% exits non-zero.  Missing inputs are reported as BLOCKED, never as PASS.
%
% Usage (on a machine with MATLAB):
%   matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_decode_cpp_iq('result_json','/path/result.json')"
%
% Name/value options:
%   'result_json'  required path to a twr-m2a-native/1 JSON
%   'iq_file'      optional native CF32 path (defaults to JSON-named file)
%   'taps_dir'     frozen taps dir (default ../taps, env TWR_M2A_TAPS_DIR)
%   'out_dir'      where to write the result JSON (default ./out)
%   'tol_samples'  packet-start / SFD sample tolerance (default 1, G0 §6)
%   'sfd_mode'     decoder SFD mode (default 'ieee', the frozen profile)

    opt = parse_opts(varargin{:});
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(fileparts(here))));
    addpath(fullfile(repo, 'UWB_demodulation'));
    opt = resolve_defaults(opt, here);
    if ~exist(opt.out_dir, 'dir'), mkdir(opt.out_dir); end

    if ~exist(opt.result_json, 'file')
        error('result_json not found: %s', opt.result_json);
    end
    cpp = jsondecode(fileread(opt.result_json));

    % ---- expected bytes, computed INDEPENDENTLY from the input MAC frame ----
    mac = hex_to_bytes(require_field(cpp, 'frame', 'mac_hex'));
    psdu_expected = [mac; fcs_le_bytes(mac)];
    psdu_bytes_expected = numel(psdu_expected);
    if isfield(cpp.frame, 'psdu_bytes') && cpp.frame.psdu_bytes ~= psdu_bytes_expected
        error('frame.psdu_bytes=%d disagrees with mac_hex length+2=%d', ...
            cpp.frame.psdu_bytes, psdu_bytes_expected);
    end

    % ---- native IQ file ----
    iq_file = opt.iq_file;
    if isempty(iq_file)
        iq_file = find_iq_file(cpp, opt.result_json);
    end
    if isempty(iq_file) || ~exist(iq_file, 'file')
        record_blocked(opt, cpp, psdu_expected, iq_file, ...
            'native CF32 IQ file not found');
        if opt.fail_on_blocked
            error('matlab_decode_cpp_iq:Blocked', 'native IQ file not found');
        end
        return;
    end

    native_hz = getfield_default(cpp, 'rate', 'native_hz', 998.4e6);
    x = read_cf32(iq_file);

    % ---- resample native->work with the frozen causal contract ----
    resample_info = struct('applied', false, 'L', 1, 'M', 1, ...
        'taps_file', '', 'n_in', numel(x), 'n_out', numel(x));
    y = x;
    if abs(native_hz - 998.4e6) > 1
        if abs(native_hz - 737.28e6) < 1
            L = 65; M = 48;
        elseif abs(native_hz - 491.52e6) < 1
            L = 65; M = 32;
        else
            record_blocked(opt, cpp, psdu_expected, iq_file, ...
                sprintf('unsupported native rate %.0f Hz', native_hz));
            if opt.fail_on_blocked
                error('matlab_decode_cpp_iq:Blocked', 'unsupported native rate');
            end
            return;
        end
        [h, taps_file] = load_taps_for(opt.taps_dir, L, M, repo);
        if isempty(h)
            record_blocked(opt, cpp, psdu_expected, iq_file, ...
                sprintf('no RX taps for %d/%d', L, M));
            if opt.fail_on_blocked
                error('matlab_decode_cpp_iq:Blocked', 'no RX taps');
            end
            return;
        end
        T = numel(h);
        n_expected = ceil(((numel(x) - 1) * L + T) / M);
        y = upfirdn(x, h, L, M);
        if numel(y) ~= n_expected
            error('resample length mismatch: expected %d, got %d', ...
                n_expected, numel(y));
        end
        resample_info = struct('applied', true, 'L', L, 'M', M, ...
            'taps_file', taps_file, 'n_in', numel(x), 'n_out', numel(y));
    end

    % ---- decode (no catch-and-pass) ----
    o = struct();
    o.fs_rx = 998.4e6;
    o.code_index = 9;
    o.data_rate = 6.81;
    o.preamble_repetitions = 64;
    o.cir_skip_initial_repetitions = 10;
    o.cir_repetitions = 54;
    o.sfd_mode = opt.sfd_mode;
    o.max_psdu_bytes = 127;
    o.enable_frame_crop = true;
    o.verbose = false;
    o.show_plots = false;
    o.ant_num = 1;
    o.channel_index = 1;

    try
        result = decode_uwb(o, y, struct(), [], 'single');
    catch err
        rec = base_record(cpp, psdu_expected, iq_file, resample_info);
        rec.source_json = opt.result_json;
        rec.status = 'failed_decode_exception';
        rec.note = err.message;
        write_json(fullfile(opt.out_dir, 'matlab_decode_cpp_iq_result.json'), rec);
        rethrow(err);
    end

    % ---- compare ----
    got_bytes = uint8(result.payload.bytes(:));
    checks = struct();
    checks.phr_psdu_length = (double(result.phr.psdu_length_bytes) == psdu_bytes_expected);
    checks.bytes_exact = isequal(got_bytes, psdu_expected);
    checks.fcs_pass = logical(result.payload.fcs_pass);
    expected_fcs = fcs_le_bytes(mac);
    expected_fcs_u16 = uint16(expected_fcs(1)) + bitshift(uint16(expected_fcs(2)), 8);
    checks.fcs_matches_expected = (uint16(result.payload.fcs_received) == expected_fcs_u16) ...
        && (uint16(result.payload.fcs_calculated) == expected_fcs_u16);

    % MATLAB 1-based -> 0-based, explicitly
    matlab_packet_start_0based = double(result.preamble.start_sample_uncropped) - 1;
    spc = double(result.soft_chip_timing.samples_per_chip);
    first_chip_1based = double(result.soft_chip_timing.first_chip_sample_uncropped);
    matlab_sfd_start_0based = (first_chip_1based - 1) + ...
        (double(result.sfd.start_chip) - 1) * spc;

    cpp_packet = getfield_default(getfield_default(cpp, 'decode', struct()), ...
        'diagnostic', struct());
    cpp_packet = getfield_default(cpp_packet, 'packet_start_sample', NaN);
    cpp_sfd = getfield_default(getfield_default(cpp, 'decode', struct()), ...
        'diagnostic', struct());
    cpp_sfd = getfield_default(cpp_sfd, 'sfd_start_sample', NaN);

    d_packet = NaN; d_sfd = NaN;
    checks.packet_start_ok = false;
    checks.sfd_start_ok = false;
    if ~isnan(cpp_packet)
        d_packet = matlab_packet_start_0based - double(cpp_packet);
        checks.packet_start_ok = abs(d_packet) <= opt.tol_samples;
    end
    if ~isnan(cpp_sfd)
        d_sfd = matlab_sfd_start_0based - double(cpp_sfd);
        checks.sfd_start_ok = abs(d_sfd) <= opt.tol_samples;
    end

    % C++ self-consistency: what the C++ JSON itself claims it decoded.
    cpp_payload_hex = getfield_default(getfield_default(cpp, 'decode', struct()), ...
        'payload_hex', '');
    cpp_self_ok = false;
    if ~isempty(cpp_payload_hex)
        cpp_self_ok = isequal(hex_to_bytes(cpp_payload_hex), psdu_expected);
    end
    cpp_status = getfield_default(getfield_default(cpp, 'decode', struct()), ...
        'status', '');

    rec = base_record(cpp, psdu_expected, iq_file, resample_info);
    rec.source_json = opt.result_json;
    rec.matlab.executed = true;
    rec.matlab.version = version;
    rec.matlab.command = 'matlab_decode_cpp_iq';
    rec.expected = struct('mac_hex', bytes_to_hex(mac), ...
        'psdu_hex', bytes_to_hex(psdu_expected), ...
        'psdu_bytes', psdu_bytes_expected, ...
        'fcs', sprintf('0x%04x', expected_fcs_u16));
    rec.decoded = struct('psdu_length_bytes', double(result.phr.psdu_length_bytes), ...
        'payload_hex', bytes_to_hex(got_bytes), ...
        'fcs_received', sprintf('0x%04x', uint16(result.payload.fcs_received)), ...
        'fcs_calculated', sprintf('0x%04x', uint16(result.payload.fcs_calculated)), ...
        'fcs_pass', logical(result.payload.fcs_pass), ...
        'packet_start_0based', matlab_packet_start_0based, ...
        'sfd_start_0based', matlab_sfd_start_0based, ...
        'samples_per_chip', spc, ...
        'detected_repetitions', double(result.preamble.detected_repetitions));
    rec.checks = checks;
    rec.deltas = struct('packet_start_samples', d_packet, 'sfd_start_samples', d_sfd, ...
        'tol_samples', opt.tol_samples);
    rec.cpp = struct('status', cpp_status, 'packet_start_sample', cpp_packet, ...
        'sfd_start_sample', cpp_sfd, 'payload_hex', cpp_payload_hex, ...
        'self_consistent', cpp_self_ok);

    all_ok = checks.phr_psdu_length && checks.bytes_exact && checks.fcs_pass && ...
        checks.fcs_matches_expected && checks.packet_start_ok && checks.sfd_start_ok;
    if all_ok
        rec.status = 'pass';
    else
        rec.status = 'failed';
        rec.note = 'one or more checks failed; see checks/deltas';
    end
    out_json = fullfile(opt.out_dir, 'matlab_decode_cpp_iq_result.json');
    write_json(out_json, rec);
    fprintf('== matlab_decode_cpp_iq: %s ==\n', upper(rec.status));
    fprintf('   bytes_exact=%d fcs_pass=%d phr_len=%d d_packet=%.1f d_sfd=%.1f\n', ...
        checks.bytes_exact, checks.fcs_pass, checks.phr_psdu_length, ...
        d_packet, d_sfd);
    fprintf('wrote %s\n', out_json);

    if ~all_ok
        error('matlab_decode_cpp_iq:Failed', ...
            'M2-A MATLAB decode cross-check failed; see %s', out_json);
    end
end

% ---------------------------------------------------------------------------
function opt = parse_opts(varargin)
    opt = struct('result_json', '', 'iq_file', '', 'taps_dir', '', ...
                 'out_dir', '', 'tol_samples', 1, 'sfd_mode', 'ieee', ...
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
    if isempty(opt.out_dir)
        opt.out_dir = fullfile(here, 'out');
    end
    opt.taps_dir = char(opt.taps_dir);
    opt.out_dir = char(opt.out_dir);
end

function f = find_iq_file(cpp, result_json)
    f = '';
    [d, base] = fileparts(result_json);
    cands = {};
    if isfield(cpp, 'artifacts') && isfield(cpp.artifacts, 'native_cf32')
        cands{end+1} = cpp.artifacts.native_cf32;
    end
    if isfield(cpp, 'native_cf32'), cands{end+1} = cpp.native_cf32; end
    if isfield(cpp, 'iq_file'), cands{end+1} = cpp.iq_file; end
    cands{end+1} = fullfile(d, [base '.cf32']);
    cands{end+1} = fullfile(d, 'native.cf32');
    for i = 1:numel(cands)
        c = char(cands{i});
        if ~isempty(c) && exist(c, 'file')
            f = c;
            return;
        end
    end
end

function rec = base_record(cpp, psdu_expected, iq_file, resample_info)
    rec = struct();
    rec.schema = 'twr-m2a-matlab-decode-cpp-iq/1';
    rec.generated_utc = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
    rec.source_json = '';
    rec.matlab = struct('executed', false, 'version', version, ...
        'command', '', 'exit_code', 0);
    rec.iq_file = iq_file;
    rec.native_hz = getfield_default(cpp, 'rate', 'native_hz', NaN);
    rec.resample = resample_info;
    rec.status = 'unknown';
    rec.note = '';
    rec.expected_psdu_bytes = numel(psdu_expected);
    rec.expected_psdu_hex = bytes_to_hex(psdu_expected);
end

function record_blocked(opt, cpp, psdu_expected, iq_file, reason)
    if ~exist(opt.out_dir, 'dir'), mkdir(opt.out_dir); end
    rec = base_record(cpp, psdu_expected, iq_file, struct());
    rec.source_json = opt.result_json;
    rec.status = 'blocked';
    rec.note = reason;
    rec.matlab.executed = true;
    rec.matlab.version = version;
    rec.matlab.command = 'matlab_decode_cpp_iq';
    write_json(fullfile(opt.out_dir, 'matlab_decode_cpp_iq_result.json'), rec);
    fprintf('== matlab_decode_cpp_iq: BLOCKED (%s) ==\n', reason);
end

% ---------------------------------------------------------------------------
function b = hex_to_bytes(h)
    h = char(h);
    h = h(h ~= ' ' & h ~= sprintf('\n') & h ~= sprintf('\r'));
    if mod(numel(h), 2) ~= 0
        error('odd-length hex string');
    end
    b = uint8(zeros(numel(h) / 2, 1));
    for i = 1:numel(b)
        b(i) = uint8(hex2dec(h(2*i-1:2*i)));
    end
end

function s = bytes_to_hex(b)
    b = uint8(b(:));
    s = lower(reshape(dec2hex(b, 2)', 1, []));
end

function f = fcs_le_bytes(mac)
    crc = uwbdecoder.ieee802154CRC16(uint8(mac(:)));
    f = uint8([bitand(crc, uint16(255)); bitshift(crc, -8)]);
end

function v = getfield_default(s, f, d)
    if isstruct(s) && isfield(s, f)
        v = s.(f);
    else
        v = d;
    end
end

function [h, src] = load_taps_for(tapsDir, L, M, repoRoot)
    h = [];
    src = '';
    if ~exist(tapsDir, 'dir')
        tapsDir = '';
    end
    % design.json discovery first (records the exact file per direction).
    % Agent A's schema uses frozen_file; file/taps_file are also accepted.
    if ~isempty(tapsDir)
        dj = fullfile(tapsDir, 'design.json');
        if exist(dj, 'file')
            try
                d = jsondecode(fileread(dj));
                for key = {'directions', 'taps'}
                    if isfield(d, key{1}) && ~isempty(d.(key{1}))
                        arr = d.(key{1});
                        for ai = 1:numel(arr)
                            e = arr(ai);
                            [eL, eM] = entry_lm(e);
                            if eL == L && eM == M
                                for fld = {'frozen_file', 'file', 'taps_file'}
                                    if isfield(e, fld{1})
                                        f = fullfile(tapsDir, e.(fld{1}));
                                        if exist(f, 'file')
                                            [h, src] = read_taps_any(f);
                                            return;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            catch
                % fall through to conventional names
            end
        end
        base = {sprintf('taps_%d_%d', L, M), ...
                sprintf('rx_taps_%d_%d', L, M), ...
                sprintf('m2a_taps_%d_%d', L, M), ...
                sprintf('rx_%d_%d', L, M)};
        exts = {'.f32', '.bin', '.txt', '.csv'};
        for bi = 1:numel(base)
            for ei = 1:numel(exts)
                f = fullfile(tapsDir, [base{bi} exts{ei}]);
                if exist(f, 'file')
                    [h, src] = read_taps_any(f);
                    return;
                end
            end
        end
    end
    % repository RX prototype fallback (M2-A canonical RX taps)
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
            [h, src] = read_taps_any(cand);
            return;
        end
    end
end

function [L, M] = entry_lm(e)
    L = NaN; M = NaN;
    for f = {'interp', 'l', 'L'}
        if isfield(e, f{1}), L = e.(f{1}); end
    end
    for f = {'decim', 'm', 'M'}
        if isfield(e, f{1}), M = e.(f{1}); end
    end
end

function [h, src] = read_taps_any(f)
    [~, ~, ext] = fileparts(f);
    if any(strcmpi(ext, {'.f32', '.bin'}))
        fid = fopen(f, 'rb');
        if fid < 0, error('cannot open taps %s', f); end
        h = fread(fid, Inf, 'float32=>double');
        fclose(fid);
    else
        h = readmatrix(f, 'FileType', 'text');
        h = h(:);
    end
    src = f;
end

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

function write_json(p, s)
    fid = fopen(p, 'w');
    if fid < 0, error('cannot write %s', p); end
    fwrite(fid, jsonencode(s));
    fclose(fid);
end

function v = require_field(s, parent, child)
    if ~isfield(s, parent) || ~isfield(s.(parent), child)
        error('JSON missing %s.%s', parent, child);
    end
    v = s.(parent).(child);
end
