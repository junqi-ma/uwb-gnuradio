function meta = export_uwb_radar_packet(outDir, preambleSymbols, varargin)
%EXPORT_UWB_RADAR_PACKET  Complete pulse-shaped packet for one SYNC / PSDU.
%
%   META = EXPORT_UWB_RADAR_PACKET(OUTDIR, NSYNC) writes SYNC+SFD+STS+PHR+PSDU
%   at 998.4 MS/s, one-shot native TX at 737.28 (48/65) and 491.52 (32/65),
%   plus a clean RX loopback and radar CIR goldens.
%
%   NSYNC is 32, 64, 128, 256, 512, 1024 or 2048.
%   IEEE BPRF PreambleDuration native in MATLAB is 16/64/1024/4096.  Other
%   lengths crop or tile the pulse-shaped SYNC field of a 64- or 1024-SYNC
%   complete packet (never 751-sample template repeats).
%
%   Name-value:
%     'PSDUBytes'   (default 0)  random data-byte count.  0 → PSDULength=0.
%                                N>0 appends IEEE 802.15.4 FCS (PSDU=N+2)
%                                unless 'AppendFCS' is false.
%     'PSDUHex'     exact PSDU bytes (hex string).  Overrides PSDUBytes.
%     'RandomSeed'  (default 20260904)
%     'AppendFCS'   (default true when using PSDUBytes>0)
%
%   Does not overwrite testdata/uwb_radar/ canonical 64-SYNC 20-byte files
%   unless OUTDIR points there.

    if nargin < 1 || isempty(outDir)
        error('export_uwb_radar_packet:Arg', 'outDir required');
    end
    if nargin < 2 || isempty(preambleSymbols)
        error('export_uwb_radar_packet:Arg', 'preambleSymbols required');
    end
    allowedSync = [32 64 128 256 512 1024 2048];
    if ~ismember(preambleSymbols, allowedSync)
        error('export_uwb_radar_packet:Sync', ...
            'SYNC length must be 32, 64, 128, 256, 512, 1024 or 2048');
    end

    p = inputParser;
    addParameter(p, 'PSDUBytes', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'PSDUHex', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'RandomSeed', 20260904, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'AppendFCS', true, @(x) islogical(x) && isscalar(x));
    parse(p, varargin{:});
    nDataBytes = double(p.Results.PSDUBytes);
    psduHex = char(p.Results.PSDUHex);
    rngSeed = double(p.Results.RandomSeed);
    appendFcs = logical(p.Results.AppendFCS);

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    here = fileparts(mfilename('fullpath'));
    testdataDir = fileparts(here);
    repo = fileparts(testdataDir);
    add_uwbdecoder(repo);

    fs = 998.4e6;
    fsNative = 737.28e6;
    fsCg400 = 491.52e6;
    peakAmp = 0.8;
    codeIndex = 9;
    samplesPerPulse = 2;
    meanPrf = 62.4;
    dataRate = 6.81;
    sfdSeq = [-1; -1; -1; 1; -1; -1; 1; -1];
    c = 299792458;
    radarRangeM = 15;
    cirSkip = 10;
    cirRadarPre = 16;
    tailSamples = 4096;
    preGuard = round(2e-6 * fs);
    cirRadarPost = ceil(2 * radarRangeM / c * fs);

    [payloadBytes, psduMode] = build_psdu(nDataBytes, psduHex, rngSeed, appendFcs);
    psduBytes = numel(payloadBytes);
    if psduBytes > 127
        error('export_uwb_radar_packet:PSDU', 'PSDULength %d exceeds 127', psduBytes);
    end
    payloadBits = bytes_to_lsb_bits(payloadBytes);

    nativeBase = pick_native_base(preambleSymbols);
    cfg = make_hrp_config(meanPrf, dataRate, nativeBase, ...
        codeIndex, samplesPerPulse, psduBytes);

    [basePkt, ~] = lrwpanWaveformGenerator(payloadBits, cfg);
    basePkt = complex(basePkt(:), zeros(numel(basePkt), 1));
    pk = max(abs(basePkt));
    if ~(pk > 0)
        error('export_uwb_radar_packet:EmptyPacket', 'generated packet is empty');
    end
    basePkt = basePkt * (peakAmp / pk);

    idx = lrwpanHRPFieldIndices(cfg);
    samplesPerSymbol = (idx.SYNC(end) - idx.SYNC(1) + 1) / cfg.PreambleDuration;
    if samplesPerSymbol ~= 1016
        error('export_uwb_radar_packet:Sps', 'expected 1016 samples/symbol, got %g', ...
            samplesPerSymbol);
    end
    syncField = basePkt(idx.SYNC(1):idx.SYNC(end));
    restField = basePkt(idx.SYNC(end)+1:end); % SFD + STS + PHR + payload
    [packet, profileKind, construction] = assemble_sync( ...
        syncField, restField, preambleSymbols, nativeBase, samplesPerSymbol);
    packet = packet(:);
    syncOriginTx = 0;
    sfdStartTx = preambleSymbols * samplesPerSymbol;

    native737 = single(resample(double(packet), 48, 65));
    native491 = single(resample(double(packet), 32, 65));
    ntaps48 = 2 * 10 * max(48, 65) + 1;
    ntaps32 = 2 * 10 * max(32, 65) + 1;

    rxClean = embed_tx(packet, preGuard, tailSamples, 0, 1);
    ref = local_reference(cfg, codeIndex, fs, samplesPerSymbol);
    originPred = preGuard;
    cirClean = local_estimate_cir(rxClean, originPred, ref, ...
        cirRadarPre, cirRadarPost, cirSkip, preambleSymbols);

    files = struct();
    files.tx_998p4 = write_cf32(fullfile(outDir, 'tx_998p4.cf32'), packet);
    files.tx_737p28 = write_cf32(fullfile(outDir, 'tx_737p28.cf32'), native737);
    files.tx_491p52 = write_cf32(fullfile(outDir, 'tx_491p52.cf32'), native491);
    files.rx_clean_998p4 = write_cf32(fullfile(outDir, 'rx_clean_998p4.cf32'), rxClean);
    files.cir_raw_clean_radar = write_cf32(fullfile(outDir, 'cir_raw_clean_radar.cf32'), cirClean.raw);
    files.cir_norm_clean_radar = write_cf32(fullfile(outDir, 'cir_norm_clean_radar.cf32'), cirClean.norm);

    meta = struct();
    meta.description = sprintf(['Complete UWB radar packet: code 9, %d SYNC, ', ...
        'SFDNumber=2 (4z2), PSDU %d bytes (%s), 998.4 MS/s + one-shot ', ...
        '48/65 and 32/65 native.'], preambleSymbols, psduBytes, psduMode);
    meta.matlab_version = version;
    try
        meta.matlab_release = version('-release');
    catch
        meta.matlab_release = '';
    end
    meta.phy_mode = cfg.Mode;
    meta.profile_kind = profileKind;
    meta.base_preamble_duration = nativeBase;
    meta.construction = construction;
    meta.sfd_number = 2;
    meta.psdu_mode = psduMode;
    meta.psdu_data_bytes = nDataBytes;
    meta.psdu_fcs_bytes = double(appendFcs && isempty(psduHex) && nDataBytes > 0) * 2;
    if isempty(payloadBytes)
        meta.payload_bytes_hex = '';
    else
        meta.payload_bytes_hex = sprintf('%02X', payloadBytes);
    end
    meta.psdu_length_bytes = psduBytes;
    meta.sample_format = 'fc32';
    meta.dtype = 'complex64';
    meta.byte_order = 'little-endian';
    meta.layout = 'interleaved_iq';
    meta.bytes_per_complex = 8;
    meta.sample_index_base = 0;
    meta.rate_work_hz = fs;
    meta.rate_native_hz = fsNative;
    meta.rate_native_cg400_hz = fsCg400;
    meta.code_index = codeIndex;
    meta.sync_repetitions = preambleSymbols;
    meta.sfd_mode = '4z2';
    meta.sfd_sequence = sfdSeq(:).';
    meta.samples_per_symbol = samplesPerSymbol;
    meta.peak_amplitude = peakAmp;
    meta.rng_seed = rngSeed;
    meta.tx_length_998p4 = numel(packet);
    meta.tx_length_737p28 = numel(native737);
    meta.tx_length_491p52 = numel(native491);
    meta.coordinates_0based = struct( ...
        'tx_998p4', struct('sync_origin', syncOriginTx, 'sfd_start', sfdStartTx), ...
        'rx_clean_998p4', struct('sync_origin', preGuard + syncOriginTx, ...
            'sfd_start', preGuard + sfdStartTx));
    meta.resample = struct('interp_uc200', 48, 'decim_uc200', 65, ...
        'interp_cg400', 32, 'decim_cg400', 65, ...
        'half_length', 10, 'window', {{'kaiser', 5.0}}, ...
        'ntaps_48_65', ntaps48, 'ntaps_32_65', ntaps32);
    meta.rx_window = struct('pre_guard_samples', preGuard, 'pre_guard_s', 2e-6, ...
        'tail_samples', tailSamples, 'length_998p4', numel(rxClean), 'cfo_hz', 0);
    meta.cir = struct();
    meta.cir.algorithm = 'estimateCir.m (skip, coherent average, forward sampled_code, /code_energy)';
    meta.cir.sampled_code_length = numel(ref.sampled_code);
    meta.cir.code_energy = ref.code_energy;
    meta.cir.skip_initial_repetitions = cirSkip;
    meta.cir.repetitions_used = min(54, preambleSymbols - cirSkip);
    meta.cir.radar = struct('pre', cirRadarPre, 'post', cirRadarPost, ...
        'tap_count', cirRadarPre + cirRadarPost, ...
        'peak_tap_measured_clean', cirClean.peak_tap, ...
        'valid_repetitions_clean', cirClean.valid);
    meta.files = struct('tx_998p4_cf32', files.tx_998p4, ...
        'tx_737p28_cf32', files.tx_737p28, ...
        'tx_491p52_cf32', files.tx_491p52, ...
        'rx_clean_998p4_cf32', files.rx_clean_998p4, ...
        'cir_raw_clean_radar_cf32', files.cir_raw_clean_radar, ...
        'cir_norm_clean_radar_cf32', files.cir_norm_clean_radar);
    meta.generator = 'export_uwb_radar_packet.m';
    meta.cfg_mode = cfg.Mode;
    meta.note = ['Complete pulse-shaped packet (SYNC+SFD+STS+PHR+PSDU). ', ...
        'Native is one-shot resample of the entire packet. ', construction];

    fid = fopen(fullfile(outDir, 'metadata.json'), 'w');
    if fid < 0
        error('cannot write metadata.json');
    end
    fwrite(fid, jsonencode(meta));
    fclose(fid);
    fprintf('Wrote %d-SYNC PSDU=%d to %s (tx=%d, 737=%d, 491=%d, sfd=%d)\n', ...
        preambleSymbols, psduBytes, outDir, numel(packet), numel(native737), ...
        numel(native491), preGuard + sfdStartTx);
end

function nativeBase = pick_native_base(nSync)
    % MATLAB BPRF generator accepts 16/64/1024/4096.
    if nSync <= 256
        nativeBase = 64;
    else
        nativeBase = 1024;
    end
end

function [packet, kind, construction] = assemble_sync(syncField, restField, ...
        nWant, nBase, sps)
    nHave = numel(syncField) / sps;
    if abs(nHave - nBase) > 1e-9
        error('export_uwb_radar_packet:SyncField', 'SYNC field length mismatch');
    end
    if nWant == nBase
        packet = [syncField; restField];
        kind = 'standard';
        construction = sprintf('lrwpanWaveformGenerator BPRF %d-SYNC complete packet', nBase);
        return;
    end
    kind = 'custom_sync_length';
    one = syncField(1:sps);
    if nWant < nBase && mod(nBase, nWant) == 0
        packet = [syncField(1:nWant * sps); restField];
        construction = sprintf(['MATLAB BPRF %d-SYNC complete packet; keep first %d ', ...
            'pulse-shaped SYNC symbols + original SFD/STS/PHR/PSDU'], nBase, nWant);
    elseif nWant > nBase && mod(nWant, nBase) == 0
        reps = nWant / nBase;
        tiled = repmat(syncField, reps, 1);
        packet = [tiled; restField];
        construction = sprintf(['MATLAB BPRF %d-SYNC complete packet; concatenate the ', ...
            'pulse-shaped SYNC field %d times + original SFD/STS/PHR/PSDU'], nBase, reps);
    else
        % Generic: tile unit symbols from the base SYNC field.
        nCopy = min(nWant, nBase);
        head = syncField(1:nCopy * sps);
        if nWant > nCopy
            extra = repmat(one, nWant - nCopy, 1);
            head = [head; extra];
        end
        packet = [head; restField];
        construction = sprintf(['MATLAB BPRF %d-SYNC complete packet; rebuilt %d ', ...
            'pulse-shaped SYNC symbols + original SFD/STS/PHR/PSDU'], nBase, nWant);
    end
end

function [payloadBytes, mode] = build_psdu(nDataBytes, psduHex, rngSeed, appendFcs)
    if ~isempty(psduHex)
        payloadBytes = hex_to_bytes(psduHex);
        mode = 'hex';
        return;
    end
    if nDataBytes == 0
        payloadBytes = uint8.empty(0, 1);
        mode = 'empty';
        return;
    end
    rng(rngSeed, 'twister');
    dataBytes = uint8(randi([0 255], nDataBytes, 1));
    if appendFcs
        fcs = uwbdecoder.ieee802154CRC16(dataBytes);
        fcsBytes = uint8([bitand(fcs, 255); bitshift(fcs, -8)]);
        payloadBytes = [dataBytes; fcsBytes];
        mode = 'random_data_plus_fcs';
    else
        payloadBytes = dataBytes;
        mode = 'random_data';
    end
end

function bytes = hex_to_bytes(s)
    s = regexprep(upper(char(s)), '[^0-9A-F]', '');
    if mod(numel(s), 2) ~= 0
        error('export_uwb_radar_packet:Hex', 'PSDUHex must have an even number of hex digits');
    end
    if isempty(s)
        bytes = uint8.empty(0, 1);
        return;
    end
    bytes = uint8(zeros(numel(s) / 2, 1));
    for i = 1:numel(bytes)
        bytes(i) = uint8(hex2dec(s(2*i-1:2*i)));
    end
end

function bits = bytes_to_lsb_bits(payloadBytes)
    psduBytes = numel(payloadBytes);
    bits = zeros(psduBytes * 8, 1);
    for i = 1:numel(bits)
        bits(i) = bitand(bitshift(uint16(payloadBytes(floor((i-1)/8)+1)), ...
            -(mod(i-1, 8))), 1);
    end
end

function cfg = make_hrp_config(meanPrf, dataRate, preambleSymbols, ...
        codeIndex, samplesPerPulse, psduLength)
    attempts = { {'BPRF'}, {'802.15.4zBPRF'}, {'802.15.4z'}, {'802.15.4a'} };
    lastErr = [];
    for k = 1:numel(attempts)
        mode = attempts{k}{1};
        try
            cfg = lrwpanHRPConfig('Mode', mode, 'MeanPRF', meanPrf, ...
                'DataRate', dataRate, 'PreambleDuration', preambleSymbols, ...
                'CodeIndex', codeIndex, 'SamplesPerPulse', samplesPerPulse, ...
                'PSDULength', psduLength, 'SFDNumber', 2);
            return;
        catch err
            lastErr = err;
        end
    end
    error('export_uwb_radar_packet:NoHRPConfig', '%s', lastErr.message);
end

function add_uwbdecoder(repo)
    candidates = { ...
        fullfile(repo, 'UWB_demodulation'), ...
        '/home/junqima/workspace/uwb-gnuradio/UWB_demodulation'};
    for k = 1:numel(candidates)
        if exist(candidates{k}, 'dir')
            addpath(candidates{k});
            return;
        end
    end
    error('export_uwb_radar_packet:NoDemod', 'Cannot find UWB_demodulation');
end

function n = write_cf32(path, x)
    x = x(:);
    inter = zeros(2 * numel(x), 1, 'single');
    inter(1:2:end) = real(x);
    inter(2:2:end) = imag(x);
    fid = fopen(path, 'wb');
    if fid < 0
        error('cannot write %s', path);
    end
    fwrite(fid, inter, 'float32');
    fclose(fid);
    n = numel(x);
end

function rx = embed_tx(tx, preGuard, tail, delaySamples, gain)
    n = preGuard + numel(tx) + tail;
    rx = complex(zeros(n, 1));
    start = preGuard + delaySamples + 1;
    stop = start + numel(tx) - 1;
    if start < 1 || stop > n
        error('embed_tx:Window', 'delayed TX does not fit in RX window');
    end
    rx(start:stop) = tx * gain;
end

function ref = local_reference(cfg, codeIndex, fs, samplesPerSymbol)
    code = lrwpan.internal.HRPCodes(codeIndex);
    spread = zeros(numel(code) * cfg.PreambleSpreadingFactor, 1);
    spread(1:cfg.PreambleSpreadingFactor:end) = code(:);
    sampled = zeros(numel(spread) * cfg.SamplesPerPulse, 1);
    sampled(1:cfg.SamplesPerPulse:end) = spread;
    ref = struct('fs', fs, 'samples_per_symbol', samplesPerSymbol, ...
        'sampled_code', sampled, 'code_energy', real(sampled' * sampled));
end

function cir = local_estimate_cir(rx, syncOrigin0, ref, pre, post, skip, nSync)
    code = ref.sampled_code(:);
    codeEnergy = ref.code_energy + eps;
    tapCount = pre + post;
    wlen = numel(code) + tapCount - 1;
    period = ref.samples_per_symbol;
    acc = complex(zeros(wlen, 1));
    valid = 0;
    rx = rx(:);
    last = skip + min(54, nSync - skip) - 1;
    for k = skip:last
        rs = syncOrigin0 + k * period;
        lo = rs - pre;
        hi = lo + wlen - 1;
        if lo < 0 || hi > numel(rx)-1
            continue;
        end
        acc = acc + rx(lo+1:hi+1);
        valid = valid + 1;
    end
    if valid < 1
        error('export_uwb_radar_packet:NoCir', 'no complete SYNC windows');
    end
    avg = acc / valid;
    raw = conv(avg, flipud(conj(code)), 'valid') / codeEnergy;
    nrm = raw / (norm(raw) + eps);
    [~, peakTap] = max(abs(raw));
    cir = struct('raw', raw, 'norm', nrm, 'valid', valid, ...
        'peak_tap', peakTap - 1);
end
