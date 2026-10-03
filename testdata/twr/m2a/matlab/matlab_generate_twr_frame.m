function matlab_generate_twr_frame(varargin)
% MATLAB_GENERATE_TWR_FRAME  Independently generate the frozen minimal-profile
% TWR frames (Poll/Response/Final) on the 998.4 MS/s work grid.
%
% This is A11 cross-check #3.  It:
%   1. serialises frame v1 (gr-uwb/include/gnuradio/uwb/uwb_twr_frame.h) from
%      its field table, independently of any C++ code;
%   2. appends the FCS exactly once via the repository's
%      UWB_demodulation/+uwbdecoder/ieee802154CRC16.m;
%   3. builds the standard minimal-profile HRP config through the repository's
%      UWB_demodulation/+uwbdecoder/buildUwbReference.m (802.15.4a, mean PRF
%      62.4 MHz, data rate 6.81 Mb/s, code 9, 64 SYNC, 2 samples/pulse,
%      standard IEEE SFD) and calls lrwpanWaveformGenerator;
%   4. emits the work-grid CF32 IQ plus a JSON with the expected bytes/FCS so
%      the C++ chain can decode it and an independent reference can check it;
%   5. self-decodes the generated IQ with decode_uwb and compares bytes.
%
% The frozen profile also sets the PHR ranging bit and the MAC flags ranging
% bit (G0 §1).  If the installed Communications Toolbox cannot express the
% ranging bit, the script records a BLOCKED reason and does NOT write a golden
% waveform: fabricating one from C++ output is forbidden.
%
% FAILURE POLICY: no catch-and-pass.  A missing toolbox, an inexpressible
% profile, or a self-decode byte mismatch is a hard error (non-zero exit).
%
% Usage (on a machine with MATLAB):
%   matlab -batch "cd('testdata/twr/m2a/matlab'); matlab_generate_twr_frame"
%
% Name/value options:
%   'frames'         cellstr subset of {'poll','response','final'}
%   'out_dir'        output directory (default ./generated)
%   'leading_zeros'  leading silence in samples (default 2048)
%   'trailing_zeros' trailing silence in samples (default 4096)
%   'peak_amplitude' frozen peak amplitude (default 0.8)

    opt = parse_opts(varargin{:});
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(fileparts(here))));
    addpath(fullfile(repo, 'UWB_demodulation'));
    if isempty(opt.out_dir)
        opt.out_dir = fullfile(here, 'generated');
    end
    if ~exist(opt.out_dir, 'dir'), mkdir(opt.out_dir); end

    % ---- toolbox availability (hard gate) ---------------------------------
    if exist('lrwpanHRPConfig', 'file') ~= 2 && exist('lrwpanHRPConfig', 'class') ~= 8
        blocked(repo, opt, 'lrwpanHRPConfig (Communications Toolbox) not available');
    end
    if exist('lrwpanWaveformGenerator', 'file') ~= 2 && ...
            exist('lrwpanWaveformGenerator', 'class') ~= 8
        blocked(repo, opt, 'lrwpanWaveformGenerator (Communications Toolbox) not available');
    end

    % ---- repository reference: canonical HRP config + code geometry --------
    rparams = struct('data_rate', 6.81, 'code_index', 9);
    reference = uwbdecoder.buildUwbReference(rparams);
    if abs(reference.fs - 998.4e6) > 1
        blocked(repo, opt, sprintf('reference.fs=%.3f MHz, expected 998.4 MHz', ...
            reference.fs / 1e6));
    end
    if reference.samples_per_symbol ~= 1016
        blocked(repo, opt, sprintf('samples_per_symbol=%d, expected 1016', ...
            reference.samples_per_symbol));
    end
    % Verify the repository reference really is the frozen minimal profile.
    assert(reference.cfg.CodeIndex == 9, 'reference code index is not 9');
    assert(reference.cfg.PreambleDuration == 64, 'reference preamble is not 64');
    assert(abs(reference.cfg.DataRate - 6.81) < 1e-9, 'reference rate is not 6.81');
    assert(abs(reference.cfg.MeanPRF - 62.4) < 1e-9, 'reference PRF is not 62.4');
    assert(reference.cfg.SamplesPerPulse == 2, 'reference samples/pulse is not 2');

    % ---- frozen frame field tables (independent of C++) -------------------
    specs = frame_specs();
    golden = load_golden(repo);

    n_pass = 0; n_blocked = 0;
    for fi = 1:numel(opt.frames)
        name = opt.frames{fi};
        idx = find(strcmp({specs.name}, name), 1);
        if isempty(idx)
            error('unknown frame: %s', name);
        end
        spec = specs(idx);
        mac = encode_twr_v1(spec);
        psdu = [mac; fcs_le_bytes(mac)];

        % independent serializer cross-check against the frozen golden
        if ~isempty(golden)
            g = find_golden(golden, spec);
            if ~isempty(g) && ~strcmpi(bytes_to_hex(mac), lower(char(g.mac_payload_hex)))
                error(['independent frame serializer disagrees with ', ...
                    'testdata/twr/frame_golden_v1.json for %s'], name);
            end
        end

        cfg = lrwpanHRPConfig(Mode='802.15.4a', ...
            MeanPRF=reference.cfg.MeanPRF, ...
            DataRate=reference.cfg.DataRate, ...
            PreambleDuration=reference.cfg.PreambleDuration, ...
            CodeIndex=reference.cfg.CodeIndex, ...
            SamplesPerPulse=reference.cfg.SamplesPerPulse, ...
            PSDULength=numel(psdu));
        if ~isprop(cfg, 'Ranging')
            fprintf('[BLOCKED] %s: lrwpanHRPConfig has no Ranging property\n', name);
            n_blocked = n_blocked + 1;
            continue;
        end
        cfg.Ranging = true;
        if ~logical(cfg.Ranging)
            fprintf('[BLOCKED] %s: Ranging could not be set true\n', name);
            n_blocked = n_blocked + 1;
            continue;
        end

        payload_bits = psdu_to_lsb_bits(psdu);
        packet = lrwpanWaveformGenerator(payload_bits, cfg);
        packet = complex(packet(:));
        pk = max(abs(packet));
        if ~(pk > 0)
            error('generated waveform is all zero for %s', name);
        end
        packet = packet * (opt.peak_amplitude / pk);

        iq = complex(zeros(opt.leading_zeros + numel(packet) + opt.trailing_zeros, 1));
        iq(opt.leading_zeros + (1:numel(packet))) = packet;

        % ---- self-decode with the repository decoder (no catch-and-pass) ---
        o = struct();
        o.fs_rx = 998.4e6;
        o.code_index = 9;
        o.data_rate = 6.81;
        o.preamble_repetitions = 64;
        o.cir_skip_initial_repetitions = 10;
        o.cir_repetitions = 54;
        o.sfd_mode = 'ieee';
        o.max_psdu_bytes = 127;
        o.enable_frame_crop = true;
        o.verbose = false;
        o.show_plots = false;
        o.ant_num = 1;
        o.channel_index = 1;
        result = decode_uwb(o, iq, struct(), [], 'single');
        got = uint8(result.payload.bytes(:));
        if ~isequal(got, psdu) || ~logical(result.payload.fcs_pass)
            error(['self-decode of generated %s failed: got %d bytes ', ...
                'fcs_pass=%d, expected %d bytes'], name, numel(got), ...
                logical(result.payload.fcs_pass), numel(psdu));
        end

        iq_file = fullfile(opt.out_dir, sprintf('%s_work.cf32', name));
        write_cf32(iq_file, iq);
        meta = frame_meta(name, spec, mac, psdu, cfg, reference, opt, iq, result);
        json_file = fullfile(opt.out_dir, sprintf('%s_expected.json', name));
        write_json(json_file, meta);
        n_pass = n_pass + 1;
        fprintf('[PASS] %s: mac=%dB psdu=%dB fcs=0x%04x work=%d samples\n', ...
            name, numel(mac), numel(psdu), meta.frame.fcs_u16, numel(iq));
    end

    fprintf('== matlab_generate_twr_frame: generated=%d blocked=%d ==\n', ...
        n_pass, n_blocked);
    if n_blocked > 0
        error('matlab_generate_twr_frame:Blocked', ...
            '%d frame(s) blocked by toolbox profile limitations', n_blocked);
    end
end

% ---------------------------------------------------------------------------
function opt = parse_opts(varargin)
    opt = struct('frames', {{'poll', 'response', 'final'}}, ...
                 'out_dir', '', 'leading_zeros', 2048, ...
                 'trailing_zeros', 4096, 'peak_amplitude', 0.8);
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

function specs = frame_specs()
    % Frozen field values from testdata/twr/frame_golden_v1.json.  Kept here
    % (not read from C++) so the serializer is independent; the JSON is then
    % used only as a cross-check.
    specs = struct('name', {}, 'function_code', {}, 'session_id', {}, ...
        'seq', {}, 'pan_id', {}, 'src_addr', {}, 'dst_addr', {}, ...
        'flags', {}, 'timestamps', {});
    specs(1) = struct('name', 'poll', 'function_code', 0, 'session_id', 48879, ...
        'seq', 258, 'pan_id', 4660, 'src_addr', 1, 'dst_addr', 2, ...
        'flags', 1, 'timestamps', []);
    specs(2) = struct('name', 'response', 'function_code', 1, 'session_id', 48879, ...
        'seq', 258, 'pan_id', 4660, 'src_addr', 2, 'dst_addr', 1, ...
        'flags', 1, 'timestamps', [4886718345, 16702650]);
    specs(3) = struct('name', 'final', 'function_code', 2, 'session_id', 48879, ...
        'seq', 258, 'pan_id', 4660, 'src_addr', 1, 'dst_addr', 2, ...
        'flags', 1, 'timestamps', [73588229205, 659419522645, 4328719365]);
end

function mac = encode_twr_v1(spec)
    % frame v1, little-endian: 1 B version, 1 B function, then six u16 fields,
    % then 5-byte little-endian timestamps in wire order.  All bit operations
    % use explicit unsigned integer types so the result is not implementation
    % dependent on double bitwise semantics.
    mac = uint8([1, spec.function_code]);
    for v = [spec.session_id, spec.seq, spec.pan_id, spec.src_addr, ...
             spec.dst_addr, spec.flags]
        vv = uint32(v);
        mac(end+1) = uint8(bitand(vv, uint32(255))); %#ok<AGROW>
        mac(end+1) = uint8(bitand(bitshift(vv, -8), uint32(255))); %#ok<AGROW>
    end
    for t = spec.timestamps
        tt = uint64(t);
        for b = 0:4
            mac(end+1) = uint8(bitand(bitshift(tt, -8*b), uint64(255))); %#ok<AGROW>
        end
    end
end

function f = fcs_le_bytes(mac)
    crc = uwbdecoder.ieee802154CRC16(uint8(mac(:)));
    f = uint8([bitand(crc, uint16(255)); bitshift(crc, -8)]);
end

function bits = psdu_to_lsb_bits(psdu)
    psdu = uint8(psdu(:));
    bits = zeros(numel(psdu) * 8, 1);
    for i = 1:numel(bits)
        bits(i) = double(bitand(bitshift(uint16(psdu(floor((i-1)/8)+1)), ...
            -(mod(i-1, 8))), 1));
    end
end

function golden = load_golden(repo)
    golden = [];
    p = fullfile(repo, 'testdata', 'twr', 'frame_golden_v1.json');
    if exist(p, 'file')
        golden = jsondecode(fileread(p));
    end
end

function g = find_golden(golden, spec)
    g = [];
    if ~isfield(golden, 'cases')
        return;
    end
    for i = 1:numel(golden.cases)
        if golden.cases(i).function_code == spec.function_code
            g = golden.cases(i);
            return;
        end
    end
end

function meta = frame_meta(name, spec, mac, psdu, cfg, reference, opt, iq, result)
    fcs_u16 = uint16(psdu(end-1)) + bitshift(uint16(psdu(end)), 8);
    meta = struct();
    meta.schema = 'twr-m2a-matlab-generated/1';
    meta.generated_utc = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
    meta.frame_type = name;
    meta.phy = struct('work_hz', 998.4e6, 'code_index', 9, ...
        'sync_repetitions', 64, 'sfd', 'ieee', 'ranging', logical(cfg.Ranging), ...
        'data_rate_mbps', 6.81, 'mean_prf_mhz', 62.4, ...
        'samples_per_pulse', 2, 'peak_amplitude', opt.peak_amplitude, ...
        'samples_per_symbol', reference.samples_per_symbol, ...
        'chips_per_symbol', reference.chips_per_symbol);
    meta.frame = struct('type', name, 'mac_hex', bytes_to_hex(mac), ...
        'mac_bytes', numel(mac), 'psdu_hex', bytes_to_hex(psdu), ...
        'psdu_bytes', numel(psdu), 'fcs', sprintf('0x%04x', fcs_u16), ...
        'fcs_u16', double(fcs_u16), ...
        'version', 1, 'function_code', spec.function_code, ...
        'session_id', spec.session_id, 'seq', spec.seq, ...
        'pan_id', spec.pan_id, 'src_addr', spec.src_addr, ...
        'dst_addr', spec.dst_addr, 'flags', spec.flags);
    meta.iq = struct('file', sprintf('%s_work.cf32', name), ...
        'leading_zeros', opt.leading_zeros, ...
        'packet_samples', numel(iq) - opt.leading_zeros - opt.trailing_zeros, ...
        'total_samples', numel(iq), 't0_0based', opt.leading_zeros, ...
        'format', 'cf32 interleaved');
    meta.self_check = struct('psdu_length_bytes', double(result.phr.psdu_length_bytes), ...
        'payload_hex', bytes_to_hex(uint8(result.payload.bytes(:))), ...
        'fcs_pass', logical(result.payload.fcs_pass), ...
        'packet_start_0based', double(result.preamble.start_sample_uncropped) - 1, ...
        'sfd_start_chip_1based', double(result.sfd.start_chip));
    meta.matlab = struct('executed', true, 'version', version, ...
        'command', 'matlab_generate_twr_frame', 'exit_code', 0);
end

function blocked(repo, opt, reason)
    rec = struct('schema', 'twr-m2a-matlab-generated/1', ...
        'status', 'blocked', 'reason', reason, ...
        'matlab', struct('executed', true, 'version', version, ...
            'command', 'matlab_generate_twr_frame', 'exit_code', 0));
    if ~exist(opt.out_dir, 'dir'), mkdir(opt.out_dir); end
    write_json(fullfile(opt.out_dir, 'BLOCKED.json'), rec);
    error('matlab_generate_twr_frame:Blocked', '%s', reason);
end

% ---------------------------------------------------------------------------
function s = bytes_to_hex(b)
    b = uint8(b(:));
    s = lower(reshape(dec2hex(b, 2)', 1, []));
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
