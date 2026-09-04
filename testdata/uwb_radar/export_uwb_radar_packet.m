function meta = export_uwb_radar_packet(outDir, preambleSymbols)
%EXPORT_UWB_RADAR_PACKET  Complete pulse-shaped packet golden for one SYNC length.
%
%   META = EXPORT_UWB_RADAR_PACKET(OUTDIR, NSYNC) writes a full
%   lrwpanWaveformGenerator packet (SYNC+SFD+PHR+PSDU/FCS), one-shot
%   resample(x,48,65) native TX, clean RX loopback, and radar CIR goldens.
%
%   NSYNC is 32, 64, or 128. Does not overwrite testdata/uwb_radar/ canonical
%   64-SYNC files unless OUTDIR points there.

    if nargin < 1 || isempty(outDir)
        error('export_uwb_radar_packet:Arg', 'outDir required');
    end
    if nargin < 2 || isempty(preambleSymbols)
        error('export_uwb_radar_packet:Arg', 'preambleSymbols required');
    end
    if ~ismember(preambleSymbols, [32 64 128])
        error('export_uwb_radar_packet:Sync', 'SYNC length must be 32, 64, or 128');
    end
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    here = fileparts(mfilename('fullpath'));
    testdataDir = fileparts(here);
    repo = fileparts(testdataDir);
    add_uwbdecoder(repo);

    fs = 998.4e6;
    fsNative = 737.28e6;
    peakAmp = 0.8;
    rngSeed = 20260904;
    codeIndex = 9;
    samplesPerPulse = 2;
    meanPrf = 62.4;
    dataRate = 6.81;
    nDataBytes = 20;
    sfdSeq = [-1; -1; -1; 1; -1; -1; 1; -1];
    c = 299792458;
    radarRangeM = 15;
    cirSkip = 10;
    cirRadarPre = 16;
    tailSamples = 4096;
    preGuard = round(2e-6 * fs);
    cirRadarPost = ceil(2 * radarRangeM / c * fs);
    up = 48;
    down = 65;
    half = 10;

    % BPRF / 802.15.4a PreambleDuration is 16/64/1024/4096. 32 and 128 are
    % HPRF-only in lrwpanHRPConfig. Build those from a MATLAB 64-SYNC
    % complete packet (SYNC+SFD+PHR+PSDU/FCS), then crop/repeat the
    % pulse-shaped SYNC field. This is a custom profile, not a SYNC+SFD
    % fragment and not HPRF.
    baseSync = 64;
    cfg = make_hrp_config(meanPrf, dataRate, baseSync, ...
        codeIndex, samplesPerPulse, nDataBytes + 2);

    rng(rngSeed, 'twister');
    dataBytes = uint8(randi([0 255], nDataBytes, 1));
    fcs = uwbdecoder.ieee802154CRC16(dataBytes);
    fcsBytes = uint8([bitand(fcs, 255); bitshift(fcs, -8)]);
    payloadBytes = [dataBytes; fcsBytes];
    psduBytes = numel(payloadBytes);
    payloadBits = zeros(psduBytes * 8, 1);
    for i = 1:numel(payloadBits)
        payloadBits(i) = bitand(bitshift(uint16(payloadBytes(floor((i-1)/8)+1)), ...
            -(mod(i-1, 8))), 1);
    end

    [basePkt, ~] = lrwpanWaveformGenerator(payloadBits, cfg);
    basePkt = complex(basePkt(:), zeros(numel(basePkt), 1));
    pk = max(abs(basePkt));
    if ~(pk > 0)
        error('export_uwb_radar_packet:EmptyPacket', 'generated packet is empty');
    end
    basePkt = basePkt * (peakAmp / pk);

    idx = lrwpanHRPFieldIndices(cfg);
    samplesPerSymbol = (idx.SYNC(end) - idx.SYNC(1) + 1) / cfg.PreambleDuration;
    syncField = basePkt(idx.SYNC(1):idx.SYNC(end));
    sfdField = basePkt(idx.SFD(1):idx.SFD(end));
    restField = basePkt(idx.SFD(end)+1:end);
    if preambleSymbols == 64
        packet = basePkt;
        profileKind = 'standard';
        construction = 'lrwpanWaveformGenerator BPRF 64-SYNC complete packet';
    elseif preambleSymbols == 32
        nKeep = 32 * samplesPerSymbol;
        packet = [syncField(1:nKeep); sfdField; restField];
        profileKind = 'custom_sync_length';
        construction = ['MATLAB BPRF 64-SYNC complete packet; keep first 32 ', ...
            'pulse-shaped SYNC symbols + original SFD/PHR/PSDU/FCS'];
    else
        packet = [syncField; syncField; sfdField; restField];
        profileKind = 'custom_sync_length';
        construction = ['MATLAB BPRF 64-SYNC complete packet; concatenate the ', ...
            'pulse-shaped SYNC field twice + original SFD/PHR/PSDU/FCS'];
    end
    packet = packet(:);
    syncOriginTx = 0;
    sfdStartTx = preambleSymbols * samplesPerSymbol;

    native = resample(double(packet), up, down);
    native = single(native(:));
    ntaps = 2 * half * max(up, down) + 1;
    gdScipy = (ntaps - 1) / 2 / down;

    rxClean = embed_tx(packet, preGuard, tailSamples, 0, 1);
    ref = local_reference(cfg, codeIndex, fs, samplesPerSymbol);
    originPred = preGuard;
    cirClean = local_estimate_cir(rxClean, originPred, ref, ...
        cirRadarPre, cirRadarPost, cirSkip, preambleSymbols);

    files = struct();
    files.tx_998p4 = write_cf32(fullfile(outDir, 'tx_998p4.cf32'), packet);
    files.tx_737p28 = write_cf32(fullfile(outDir, 'tx_737p28.cf32'), native);
    files.rx_clean_998p4 = write_cf32(fullfile(outDir, 'rx_clean_998p4.cf32'), rxClean);
    files.cir_raw_clean_radar = write_cf32(fullfile(outDir, 'cir_raw_clean_radar.cf32'), cirClean.raw);
    files.cir_norm_clean_radar = write_cf32(fullfile(outDir, 'cir_norm_clean_radar.cf32'), cirClean.norm);

    meta = struct();
    meta.description = sprintf(['Complete UWB radar packet golden: code 9, %d SYNC, ', ...
        'SFDNumber=2 (4z2), 20 data bytes + IEEE 802.15.4 FCS, 998.4 MS/s + ', ...
        'one-shot 48/65 native.'], preambleSymbols);
    meta.matlab_version = version;
    try
        meta.matlab_release = version('-release');
    catch
        meta.matlab_release = '';
    end
    meta.phy_mode = cfg.Mode;
    meta.profile_kind = profileKind;
    meta.base_preamble_duration = baseSync;
    meta.construction = construction;
    meta.sfd_number = 2;
    meta.psdu_data_bytes = nDataBytes;
    meta.psdu_fcs_bytes = 2;
    meta.payload_bytes_hex = sprintf('%02X', payloadBytes);
    meta.dtype = 'complex64';
    meta.byte_order = 'little-endian';
    meta.layout = 'interleaved_iq';
    meta.sample_index_base = 0;
    meta.rate_work_hz = fs;
    meta.rate_native_hz = fsNative;
    meta.code_index = codeIndex;
    meta.sync_repetitions = preambleSymbols;
    meta.sfd_mode = '4z2';
    meta.sfd_sequence = sfdSeq(:).';
    meta.samples_per_symbol = samplesPerSymbol;
    meta.peak_amplitude = peakAmp;
    meta.rng_seed = rngSeed;
    meta.tx_length_998p4 = numel(packet);
    meta.tx_length_737p28 = numel(native);
    meta.coordinates_0based = struct( ...
        'tx_998p4', struct('sync_origin', syncOriginTx, 'sfd_start', sfdStartTx), ...
        'rx_clean_998p4', struct('sync_origin', preGuard + syncOriginTx, ...
            'sfd_start', preGuard + sfdStartTx));
    meta.resample = struct('interp', up, 'decim', down, 'half_length', half, ...
        'window', {{'kaiser', 5.0}}, ...
        'group_delay_samples', 0, ...
        'group_delay_domain', 'native_737p28_matlab_resample_compensated', ...
        'ntaps', ntaps, ...
        'scipy_equivalent_group_delay_samples', gdScipy);
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
        'rx_clean_998p4_cf32', files.rx_clean_998p4, ...
        'cir_raw_clean_radar_cf32', files.cir_raw_clean_radar, ...
        'cir_norm_clean_radar_cf32', files.cir_norm_clean_radar);
    meta.generator = 'export_uwb_radar_packet.m';
    meta.matlab_reference = 'testdata/uwb_radar/export_uwb_radar_packet.m';
    meta.cfg_mode = cfg.Mode;
    meta.psdu_length_bytes = psduBytes;
    meta.note = ['Complete pulse-shaped packet (SYNC+SFD+PHR+PSDU/FCS). ', ...
        'Native is one-shot resample(x,48,65) of the entire packet. ', ...
        construction];

    fid = fopen(fullfile(outDir, 'metadata.json'), 'w');
    if fid < 0
        error('cannot write metadata.json');
    end
    fwrite(fid, jsonencode(meta));
    fclose(fid);
    fprintf('Wrote %d-SYNC packet to %s (tx=%d, native=%d, sfd=%d, peak=%d)\n', ...
        preambleSymbols, outDir, numel(packet), numel(native), ...
        preGuard + sfdStartTx, cirClean.peak_tap);
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
