function meta = export_comm_template(outDir)
%EXPORT_COMM_TEMPLATE  Independent MATLAB export of the S0 comm code-10 template.
%
%   META = EXPORT_COMM_TEMPLATE() writes into
%   testdata/sic_preamble/matlab/out/:
%
%     comm_sync_code10_998p4.cf32        1016 samples  (ONE shaped SYNC symbol)
%     comm_preamble_code10_998p4.cf32    65072 samples (64 SYNC + 48-tap tail)
%     sensing_sync_code9_998p4.cf32      1016 samples  (code-9 sensing SYNC)
%     comm_template_meta.json
%
%   META = EXPORT_COMM_TEMPLATE(OUTDIR) writes into OUTDIR instead.
%
%   This is an INDEPENDENT implementation: the waveform is generated from the
%   Communications Toolbox lrwpanWaveformGenerator / lrwpan.internal.HRPCodes,
%   NOT read from the C++ / Python outputs.  It is the MATLAB-side reference for
%   the G0 requirement "code10 comm SYNC must be independently generated and
%   checked against MATLAB sample by sample"
%   (docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md §8.1 item 2).
%
%   Conventions frozen against the C++ producer
%   (gr-uwb/apps/uwb_sim_preamble_sic.py, template_meta source strings):
%     - sample rate 998.4 MS/s, samples_per_symbol = 1016, 64 SYNC,
%       mean PRF 62.4, data rate 6.81, SamplesPerPulse 2, 4z2 SFD,
%       peak-normalised to 0.8 (peakAmp/pk of the whole packet).
%     - comm_sync     = packet(SYNC(1) : SYNC(1)+1015)
%     - comm_preamble = packet(SYNC(1) : SYNC(end)+47)  (= 65024 + 48 taps,
%       keeping the leading-edge transient and the shaping-filter tail)
%     - sensing_sync  = code-9 packet(SYNC(1) : SYNC(1)+1015)
%
%   Required toolbox: Communications Toolbox (lrwpanWaveformGenerator,
%   lrwpanHRPConfig, lrwpanHRPFieldIndices, lrwpan.internal.HRPCodes).
%
%   NOTE (未实跑): no MATLAB/Octave is installed on the development host, so
%   this script has NOT been executed.  See README.md and not_run.log.

    if nargin < 1 || isempty(outDir)
        outDir = fullfile(fileparts(mfilename('fullpath')), 'out');
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    if exist('lrwpanHRPConfig', 'file') ~= 2 && ...
            exist('lrwpanHRPConfig', 'class') == 0
        error('export_comm_template:NoToolbox', ...
            ['Communications Toolbox lrwpanHRPConfig is missing; ' ...
             'cannot generate the independent HRP template']);
    end

    fs = 998.4e6;                       %#ok<NASGU> recorded in meta
    peakAmp = 0.8;
    syncReps = 64;
    samplesPerPulse = 2;
    meanPrf = 62.4;
    dataRate = 6.81;
    psduBytes = 22;                     % canonical 20 data + 2 FCS bytes
    sfdNumber = 2;                      % 4z2
    commCode = 10;
    sensingCode = 9;
    tailTaps = 48;                      % C++ legacy pulse tap count
    spsExpected = 1016;

    % --- comm code-10 template ------------------------------------------
    cfg10 = local_hrp_config(meanPrf, dataRate, syncReps, commCode, ...
        samplesPerPulse, psduBytes, sfdNumber);
    packet10 = local_packet(cfg10, psduBytes, peakAmp);
    idx10 = lrwpanHRPFieldIndices(cfg10);
    sps = (idx10.SYNC(end) - idx10.SYNC(1) + 1) / syncReps;
    if sps ~= spsExpected
        error('export_comm_template:SpsMismatch', ...
            'samples_per_symbol is %g, expected %d', sps, spsExpected);
    end
    sync10 = packet10(idx10.SYNC(1) : idx10.SYNC(1) + sps - 1);
    pre10 = packet10(idx10.SYNC(1) : idx10.SYNC(end) + tailTaps - 1);
    if numel(pre10) ~= syncReps * sps + tailTaps
        error('export_comm_template:PreambleLength', ...
            'full preamble is %d samples, expected %d', ...
            numel(pre10), syncReps * sps + tailTaps);
    end

    % --- sensing code-9 SYNC (same independent generator) ----------------
    cfg9 = local_hrp_config(meanPrf, dataRate, syncReps, sensingCode, ...
        samplesPerPulse, psduBytes, sfdNumber);
    packet9 = local_packet(cfg9, psduBytes, peakAmp);
    idx9 = lrwpanHRPFieldIndices(cfg9);
    sps9 = (idx9.SYNC(end) - idx9.SYNC(1) + 1) / syncReps;
    if sps9 ~= spsExpected
        error('export_comm_template:SensingSpsMismatch', ...
            'sensing samples_per_symbol is %g, expected %d', sps9, spsExpected);
    end
    sync9 = packet9(idx9.SYNC(1) : idx9.SYNC(1) + sps9 - 1);

    % --- write CF32 ------------------------------------------------------
    files = struct();
    files.comm_sync_code10 = local_write_cf32( ...
        fullfile(outDir, 'comm_sync_code10_998p4.cf32'), sync10);
    files.comm_preamble_code10 = local_write_cf32( ...
        fullfile(outDir, 'comm_preamble_code10_998p4.cf32'), pre10);
    files.sensing_sync_code9 = local_write_cf32( ...
        fullfile(outDir, 'sensing_sync_code9_998p4.cf32'), sync9);

    meta = struct();
    meta.schema_version = 1;
    meta.generator = 'export_comm_template.m';
    meta.description = ['Independent MATLAB lrwpanWaveformGenerator export of ' ...
        'the S0 comm code-10 template (998.4 MS/s work grid, 64 SYNC).'];
    meta.matlab_version = version;
    try
        meta.matlab_release = version('-release');
    catch
        meta.matlab_release = '';
    end
    meta.sample_format = 'fc32';
    meta.dtype = 'complex64';
    meta.byte_order = 'little-endian';
    meta.layout = 'interleaved_iq';
    meta.sample_index_base = 0;
    meta.rate_work_hz = fs;
    meta.comm_code = commCode;
    meta.sensing_code = sensingCode;
    meta.sync_repetitions = syncReps;
    meta.samples_per_symbol = sps;
    meta.sfd_number = sfdNumber;
    meta.peak_amplitude = peakAmp;
    meta.tail_taps = tailTaps;
    meta.files = files;
    meta.comm_sync_length = numel(sync10);
    meta.comm_sync_peak = double(max(abs(sync10)));
    meta.comm_sync_energy = double(sum(abs(sync10).^2));
    meta.comm_sync_sha256 = local_sha256(sync10);
    meta.comm_preamble_length = numel(pre10);
    meta.comm_preamble_peak = double(max(abs(pre10)));
    meta.comm_preamble_energy = double(sum(abs(pre10).^2));
    meta.comm_preamble_sha256 = local_sha256(pre10);
    meta.sensing_sync_length = numel(sync9);
    meta.sensing_sync_sha256 = local_sha256(sync9);
    meta.source = ['independent lrwpanWaveformGenerator; ' ...
        'comm_sync=SYNC(1:1016); comm_preamble=SYNC(1:end)+48 tap tail'];
    meta.verified_by = 'verify_sic_preamble.m';

    fid = fopen(fullfile(outDir, 'comm_template_meta.json'), 'w');
    if fid < 0
        error('export_comm_template:WriteMeta', ...
            'cannot write comm_template_meta.json in %s', outDir);
    end
    fwrite(fid, jsonencode(meta));
    fclose(fid);

    fprintf(['Wrote comm templates to %s\n' ...
        '  comm_sync      = %d samples (sha256 %s)\n' ...
        '  comm_preamble  = %d samples (sha256 %s)\n' ...
        '  sensing_sync   = %d samples (sha256 %s)\n'], ...
        outDir, numel(sync10), meta.comm_sync_sha256, ...
        numel(pre10), meta.comm_preamble_sha256, ...
        numel(sync9), meta.sensing_sync_sha256);
end

% =========================================================================
% local helpers
% =========================================================================

function cfg = local_hrp_config(meanPrf, dataRate, syncReps, codeIndex, ...
        samplesPerPulse, psduBytes, sfdNumber)
    attempts = {'BPRF', '802.15.4zBPRF', '802.15.4z', '802.15.4a'};
    lastErr = [];
    for k = 1:numel(attempts)
        try
            cfg = lrwpanHRPConfig('Mode', attempts{k}, 'MeanPRF', meanPrf, ...
                'DataRate', dataRate, 'PreambleDuration', syncReps, ...
                'CodeIndex', codeIndex, 'SamplesPerPulse', samplesPerPulse, ...
                'PSDULength', psduBytes, 'SFDNumber', sfdNumber);
            return;
        catch err
            lastErr = err;
        end
    end
    error('export_comm_template:NoHRPConfig', ...
        'lrwpanHRPConfig chain failed: %s', lastErr.message);
end

function packet = local_packet(cfg, psduBytes, peakAmp)
    payloadBits = zeros(psduBytes * 8, 1);
    [packet, ~] = lrwpanWaveformGenerator(payloadBits, cfg);
    packet = complex(double(packet(:)), 0);
    pk = max(abs(packet));
    if ~(pk > 0)
        error('export_comm_template:EmptyPacket', 'generated packet is empty');
    end
    packet = packet * (peakAmp / pk);
end

function n = local_write_cf32(path, x)
    x = x(:);
    inter = zeros(2 * numel(x), 1, 'single');
    inter(1:2:end) = real(x);
    inter(2:2:end) = imag(x);
    fid = fopen(path, 'wb');
    if fid < 0
        error('export_comm_template:WriteCf32', 'cannot write %s', path);
    end
    fwrite(fid, inter, 'float32');
    fclose(fid);
    n = numel(x);
end

function hex = local_sha256(x)
    hex = '';
    try
        md = java.security.MessageDigest.getInstance('SHA-256');
        x = x(:);
        inter = zeros(2 * numel(x), 1, 'single');
        inter(1:2:end) = real(x);
        inter(2:2:end) = imag(x);
        bytes = reshape(typecast(single(inter), 'int8'), 1, []);
        digest = md.digest(bytes);               % MATLAB int8 row, 32 bytes
        u8 = typecast(digest(:), 'uint8');
        hex = lower(sprintf('%02x', u8));
    catch
        hex = '';
    end
end
