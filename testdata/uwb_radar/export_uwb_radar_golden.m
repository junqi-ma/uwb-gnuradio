function meta = export_uwb_radar_golden(outDir)
%EXPORT_UWB_RADAR_GOLDEN  Generator-of-record UWB radar TX/RX/CIR goldens.
%
%   META = EXPORT_UWB_RADAR_GOLDEN() writes testdata/uwb_radar/*.cf32 and
%   metadata.json using Communications Toolbox lrwpanWaveformGenerator.
%
%   Profile: BPRF (or 802.15.4a + SFDNumber=2), CodeIndex=9, 64 SYNC,
%   SamplesPerPulse=2, MeanPRF=62.4, DataRate=6.81, 4z2 SFD, rng 20260904,
%   20 data bytes + IEEE 802.15.4 FCS, peak-normalize to 0.8.
%
%   Native TX is resample(x,48,65) of the *entire* packet (MATLAB resample
%   delay-compensates; the Python fallback records scipy group delay).
%
%   This script is the generator of record for testdata/uwb_radar/.
%   The Python splice fixture lives in testdata/uwb_radar_synthetic/ and
%   must not overwrite these canonical files.

    if nargin < 1 || isempty(outDir)
        outDir = fileparts(mfilename('fullpath'));
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
    preambleSymbols = 64;
    samplesPerPulse = 2;
    meanPrf = 62.4;
    dataRate = 6.81;
    nDataBytes = 20;
    sfdSeq = [-1; -1; -1; 1; -1; -1; 1; -1];
    c = 299792458;
    radarRangeM = 15;
    cirSkip = 10;
    cirRadarPre = 16;
    cirDemodPre = 8;
    cirDemodPost = 30;
    tailSamples = 4096;
    delayInt = 37;
    delayFrac = 12.4;
    gainInt = 0.4 * exp(1j * 0.7);
    gainFrac = 0.55 * exp(-1j * 1.1);
    preGuard = round(2e-6 * fs);
    cirRadarPost = ceil(2 * radarRangeM / c * fs);
    up = 48;
    down = 65;
    half = 10;

    cfg = make_hrp_config(meanPrf, dataRate, preambleSymbols, ...
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

    [packet, ~] = lrwpanWaveformGenerator(payloadBits, cfg);
    packet = complex(packet(:), zeros(numel(packet), 1));
    pk = max(abs(packet));
    if ~(pk > 0)
        error('export_uwb_radar_golden:EmptyPacket', 'generated packet is empty');
    end
    packet = packet * (peakAmp / pk);

    idx = lrwpanHRPFieldIndices(cfg);
    samplesPerSymbol = (idx.SYNC(end) - idx.SYNC(1) + 1) / cfg.PreambleDuration;
    syncOriginTx = idx.SYNC(1) - 1;          % 0-based
    sfdStartTx = idx.SFD(1) - 1;

    native = resample(double(packet), up, down);
    native = single(native(:));
    % MATLAB resample delay-compensates; scipy resample_poly does not.
    ntaps = 2 * half * max(up, down) + 1;
    gdScipy = (ntaps - 1) / 2 / down;

    rxClean = embed_tx(packet, preGuard, tailSamples, 0, 1);
    rxDelayInt = embed_tx(packet, preGuard, tailSamples, delayInt, gainInt);
    rxDelayFrac = embed_tx_frac(packet, preGuard, tailSamples, delayFrac, gainFrac);

    ref = local_reference(cfg, codeIndex, fs, samplesPerSymbol);
    originPred = preGuard; % TX-time origin in the RX window (0-based)
    cirCleanRadar = local_estimate_cir(rxClean, originPred, ref, ...
        cirRadarPre, cirRadarPost, cirSkip, preambleSymbols);
    cirClean830 = local_estimate_cir(rxClean, originPred, ref, ...
        cirDemodPre, cirDemodPost, cirSkip, preambleSymbols);
    cirIntRadar = local_estimate_cir(rxDelayInt, originPred, ref, ...
        cirRadarPre, cirRadarPost, cirSkip, preambleSymbols);
    cirInt830 = local_estimate_cir(rxDelayInt, originPred, ref, ...
        cirDemodPre, cirDemodPost, cirSkip, preambleSymbols);
    cirFracRadar = local_estimate_cir(rxDelayFrac, originPred, ref, ...
        cirRadarPre, cirRadarPost, cirSkip, preambleSymbols);
    cirFrac830 = local_estimate_cir(rxDelayFrac, originPred, ref, ...
        cirDemodPre, cirDemodPost, cirSkip, preambleSymbols);

    files = struct();
    files.tx_998p4 = write_cf32(fullfile(outDir, 'tx_998p4.cf32'), packet);
    files.tx_737p28 = write_cf32(fullfile(outDir, 'tx_737p28.cf32'), native);
    files.rx_clean_998p4 = write_cf32(fullfile(outDir, 'rx_clean_998p4.cf32'), rxClean);
    files.rx_delay_int_998p4 = write_cf32(fullfile(outDir, 'rx_delay_int_998p4.cf32'), rxDelayInt);
    files.rx_delay_frac_998p4 = write_cf32(fullfile(outDir, 'rx_delay_frac_998p4.cf32'), rxDelayFrac);
    files.cir_raw_clean_radar = write_cf32(fullfile(outDir, 'cir_raw_clean_radar.cf32'), cirCleanRadar.raw);
    files.cir_norm_clean_radar = write_cf32(fullfile(outDir, 'cir_norm_clean_radar.cf32'), cirCleanRadar.norm);
    files.cir_raw_clean_8_30 = write_cf32(fullfile(outDir, 'cir_raw_clean_8_30.cf32'), cirClean830.raw);
    files.cir_norm_clean_8_30 = write_cf32(fullfile(outDir, 'cir_norm_clean_8_30.cf32'), cirClean830.norm);
    files.cir_raw_delay_int_radar = write_cf32(fullfile(outDir, 'cir_raw_delay_int_radar.cf32'), cirIntRadar.raw);
    files.cir_norm_delay_int_radar = write_cf32(fullfile(outDir, 'cir_norm_delay_int_radar.cf32'), cirIntRadar.norm);
    files.cir_raw_delay_int_8_30 = write_cf32(fullfile(outDir, 'cir_raw_delay_int_8_30.cf32'), cirInt830.raw);
    files.cir_norm_delay_int_8_30 = write_cf32(fullfile(outDir, 'cir_norm_delay_int_8_30.cf32'), cirInt830.norm);
    files.cir_raw_delay_frac_radar = write_cf32(fullfile(outDir, 'cir_raw_delay_frac_radar.cf32'), cirFracRadar.raw);
    files.cir_norm_delay_frac_radar = write_cf32(fullfile(outDir, 'cir_norm_delay_frac_radar.cf32'), cirFracRadar.norm);
    files.cir_raw_delay_frac_8_30 = write_cf32(fullfile(outDir, 'cir_raw_delay_frac_8_30.cf32'), cirFrac830.raw);
    files.cir_norm_delay_frac_8_30 = write_cf32(fullfile(outDir, 'cir_norm_delay_frac_8_30.cf32'), cirFrac830.norm);

    meta = struct();
    meta.description = ['Canonical UWB monostatic-radar golden from ', ...
        'lrwpanWaveformGenerator: code 9, 64 SYNC, SFDNumber=2 (4z2), ', ...
        '20 data bytes + IEEE 802.15.4 FCS, 998.4 MS/s + one-shot 48/65 native.'];
    meta.matlab_version = version;
    try
        meta.matlab_release = version('-release');
    catch
        meta.matlab_release = '';
    end
    meta.phy_mode = cfg.Mode;
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
    meta.tx_998p4_peak = double(max(abs(packet)));
    meta.tx_998p4_energy = double(packet' * packet);
    meta.tx_737p28_peak = double(max(abs(native)));
    meta.tx_737p28_energy = double(double(native)' * double(native));
    meta.coordinates_0based = struct( ...
        'tx_998p4', struct('sync_origin', syncOriginTx, 'sfd_start', sfdStartTx), ...
        'rx_clean_998p4', struct('sync_origin', preGuard + syncOriginTx, ...
            'sfd_start', preGuard + sfdStartTx), ...
        'rx_delay_int_998p4', struct('sync_origin', preGuard + syncOriginTx + delayInt, ...
            'sfd_start', preGuard + sfdStartTx + delayInt), ...
        'rx_delay_frac_998p4', struct('sync_origin', preGuard + syncOriginTx + delayFrac, ...
            'sfd_start', preGuard + sfdStartTx + delayFrac));
    meta.resample = struct('interp', up, 'decim', down, 'half_length', half, ...
        'window', {{'kaiser', 5.0}}, ...
        'group_delay_samples', 0, ...
        'group_delay_domain', 'native_737p28_matlab_resample_compensated', ...
        'ntaps', ntaps, ...
        'scipy_equivalent_group_delay_samples', gdScipy);
    meta.rx_window = struct('pre_guard_samples', preGuard, 'pre_guard_s', 2e-6, ...
        'tail_samples', tailSamples, 'length_998p4', numel(rxClean), 'cfo_hz', 0);
    meta.delay_int = struct('samples', delayInt, 'gain_mag', 0.4, 'gain_phase_rad', 0.7);
    meta.delay_frac = struct('samples', delayFrac, 'gain_mag', 0.55, ...
        'gain_phase_rad', -1.1, 'interpolation', 'pchip', ...
        'interpolation_note', ...
        'canonical fractional delay uses MATLAB interp1 pchip; Python ndimage.shift is not the reference');
    meta.cir = struct();
    meta.cir.algorithm = 'estimateCir.m (skip, coherent average, forward sampled_code, /code_energy)';
    meta.cir.sampled_code_length = numel(ref.sampled_code);
    meta.cir.code_energy = ref.code_energy;
    meta.cir.skip_initial_repetitions = cirSkip;
    meta.cir.repetitions_used = preambleSymbols - cirSkip;
    meta.cir.c_m_per_s = c;
    meta.cir.radar_max_range_m = radarRangeM;
    meta.cir.range_m_per_tap = c / (2 * fs);
    meta.cir.radar = struct('pre', cirRadarPre, 'post', cirRadarPost, ...
        'tap_count', cirRadarPre + cirRadarPost, ...
        'peak_tap_expected_clean', cirRadarPre, ...
        'peak_tap_expected_delay_int', cirRadarPre + delayInt, ...
        'peak_tap_measured_clean', cirCleanRadar.peak_tap, ...
        'peak_tap_measured_delay_int', cirIntRadar.peak_tap, ...
        'peak_tap_measured_delay_frac', cirFracRadar.peak_tap, ...
        'valid_repetitions_clean', cirCleanRadar.valid, ...
        'valid_repetitions_delay_int', cirIntRadar.valid, ...
        'valid_repetitions_delay_frac', cirFracRadar.valid, ...
        'peak_convention', ...
        ['algorithm tap index is argmax(|raw|) on the estimateCir grid; ', ...
         'clean peak is typically pre + pulse-shape offset, not physical zero. ', ...
         'Known extra delay D must shift the peak by D relative to clean.']);
    meta.cir.demod_8_30 = struct('pre', cirDemodPre, 'post', cirDemodPost, ...
        'tap_count', cirDemodPre + cirDemodPost, ...
        'peak_tap_expected_clean', cirDemodPre, ...
        'peak_tap_measured_clean', cirClean830.peak_tap, ...
        'peak_tap_measured_delay_int', cirInt830.peak_tap, ...
        'peak_tap_measured_delay_frac', cirFrac830.peak_tap, ...
        'note', 'delay_int/frac peaks can fall outside post=30; radar 16/100 is the acceptance window');
    meta.files = struct( ...
        'tx_998p4_cf32', files.tx_998p4, ...
        'tx_737p28_cf32', files.tx_737p28, ...
        'rx_clean_998p4_cf32', files.rx_clean_998p4, ...
        'rx_delay_int_998p4_cf32', files.rx_delay_int_998p4, ...
        'rx_delay_frac_998p4_cf32', files.rx_delay_frac_998p4, ...
        'cir_raw_clean_radar_cf32', files.cir_raw_clean_radar, ...
        'cir_norm_clean_radar_cf32', files.cir_norm_clean_radar, ...
        'cir_raw_clean_8_30_cf32', files.cir_raw_clean_8_30, ...
        'cir_norm_clean_8_30_cf32', files.cir_norm_clean_8_30, ...
        'cir_raw_delay_int_radar_cf32', files.cir_raw_delay_int_radar, ...
        'cir_norm_delay_int_radar_cf32', files.cir_norm_delay_int_radar, ...
        'cir_raw_delay_int_8_30_cf32', files.cir_raw_delay_int_8_30, ...
        'cir_norm_delay_int_8_30_cf32', files.cir_norm_delay_int_8_30, ...
        'cir_raw_delay_frac_radar_cf32', files.cir_raw_delay_frac_radar, ...
        'cir_norm_delay_frac_radar_cf32', files.cir_norm_delay_frac_radar, ...
        'cir_raw_delay_frac_8_30_cf32', files.cir_raw_delay_frac_8_30, ...
        'cir_norm_delay_frac_8_30_cf32', files.cir_norm_delay_frac_8_30);
    % Filename keys with dots: also emit a cell list the verifier can ignore.
    meta.file_lengths = { ...
        'tx_998p4.cf32', files.tx_998p4; ...
        'tx_737p28.cf32', files.tx_737p28; ...
        'rx_clean_998p4.cf32', files.rx_clean_998p4; ...
        'rx_delay_int_998p4.cf32', files.rx_delay_int_998p4; ...
        'rx_delay_frac_998p4.cf32', files.rx_delay_frac_998p4; ...
        'cir_raw_clean_radar.cf32', files.cir_raw_clean_radar; ...
        'cir_norm_clean_radar.cf32', files.cir_norm_clean_radar; ...
        'cir_raw_clean_8_30.cf32', files.cir_raw_clean_8_30; ...
        'cir_norm_clean_8_30.cf32', files.cir_norm_clean_8_30; ...
        'cir_raw_delay_int_radar.cf32', files.cir_raw_delay_int_radar; ...
        'cir_norm_delay_int_radar.cf32', files.cir_norm_delay_int_radar; ...
        'cir_raw_delay_int_8_30.cf32', files.cir_raw_delay_int_8_30; ...
        'cir_norm_delay_int_8_30.cf32', files.cir_norm_delay_int_8_30; ...
        'cir_raw_delay_frac_radar.cf32', files.cir_raw_delay_frac_radar; ...
        'cir_norm_delay_frac_radar.cf32', files.cir_norm_delay_frac_radar; ...
        'cir_raw_delay_frac_8_30.cf32', files.cir_raw_delay_frac_8_30; ...
        'cir_norm_delay_frac_8_30.cf32', files.cir_norm_delay_frac_8_30};
    meta.generator = 'export_uwb_radar_golden.m';
    meta.matlab_reference = 'testdata/uwb_radar/export_uwb_radar_golden.m';
    meta.note = ['Canonical golden: MATLAB lrwpanWaveformGenerator + resample(x,48,65). ', ...
        'Python splice fixtures are under testdata/uwb_radar_synthetic/ and are not this packet.'];
    meta.cfg_mode = cfg.Mode;
    meta.psdu_length_bytes = psduBytes;

    txt = jsonencode(meta);
    fid = fopen(fullfile(outDir, 'metadata.json'), 'w');
    if fid < 0
        error('cannot write metadata.json');
    end
    fwrite(fid, txt);
    fclose(fid);
    fprintf('Wrote radar goldens to %s (tx=%d samples, native=%d)\n', ...
        outDir, numel(packet), numel(native));
end

function cfg = make_hrp_config(meanPrf, dataRate, preambleSymbols, ...
        codeIndex, samplesPerPulse, psduLength)
    attempts = {
        {'BPRF'}
        {'802.15.4zBPRF'}
        {'802.15.4z'}
        {'802.15.4a'}
        };
    lastErr = [];
    for k = 1:numel(attempts)
        mode = attempts{k}{1};
        try
            if strcmp(mode, '802.15.4a')
                cfg = lrwpanHRPConfig('Mode', mode, 'MeanPRF', meanPrf, ...
                    'DataRate', dataRate, 'PreambleDuration', preambleSymbols, ...
                    'CodeIndex', codeIndex, 'SamplesPerPulse', samplesPerPulse, ...
                    'PSDULength', psduLength, 'SFDNumber', 2);
            else
                cfg = lrwpanHRPConfig('Mode', mode, 'MeanPRF', meanPrf, ...
                    'DataRate', dataRate, 'PreambleDuration', preambleSymbols, ...
                    'CodeIndex', codeIndex, 'SamplesPerPulse', samplesPerPulse, ...
                    'PSDULength', psduLength, 'SFDNumber', 2);
            end
            return;
        catch err
            lastErr = err;
        end
    end
    error('export_uwb_radar_golden:NoHRPConfig', ...
        'lrwpanHRPConfig BPRF/4z/4a failed: %s', lastErr.message);
end

function add_uwbdecoder(repo)
    candidates = { ...
        fullfile(repo, 'UWB_demodulation'), ...
        fullfile(fileparts(repo), 'uwb-gnuradio', 'UWB_demodulation'), ...
        '/home/junqima/workspace/uwb-gnuradio/UWB_demodulation'};
    for k = 1:numel(candidates)
        if exist(candidates{k}, 'dir')
            addpath(candidates{k});
            return;
        end
    end
    error('export_uwb_radar_golden:NoDemod', ...
        'Cannot find UWB_demodulation (need +uwbdecoder/ieee802154CRC16.m)');
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
    start = preGuard + delaySamples + 1; % 1-based
    stop = start + numel(tx) - 1;
    if start < 1 || stop > n
        error('embed_tx:Window', 'delayed TX does not fit in RX window');
    end
    rx(start:stop) = tx * gain;
end

function rx = embed_tx_frac(tx, preGuard, tail, delaySamples, gain)
    n = preGuard + numel(tx) + tail;
    base = complex(zeros(n, 1));
    base(preGuard+1:preGuard+numel(tx)) = tx;
    t = (0:n-1).';
    rx = interp1(t, base, t - delaySamples, 'pchip', 0) * gain;
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
    % 0-based syncOrigin0; port of estimateCir.m with raw + L2 outputs.
    code = ref.sampled_code(:);
    codeEnergy = ref.code_energy + eps;
    tapCount = pre + post;
    wlen = numel(code) + tapCount - 1;
    period = ref.samples_per_symbol;
    acc = complex(zeros(wlen, 1));
    valid = 0;
    rx = rx(:);
    for k = skip:(nSync-1)
        rs = syncOrigin0 + k * period; % 0-based SYNC start
        lo = rs - pre;                 % 0-based
        hi = lo + wlen - 1;
        if lo < 0 || hi > numel(rx)-1
            continue;
        end
        acc = acc + rx(lo+1:hi+1);
        valid = valid + 1;
    end
    if valid < 1
        error('export_uwb_radar_golden:NoCir', 'no complete SYNC windows');
    end
    avg = acc / valid;
    raw = conv(avg, flipud(conj(code)), 'valid') / codeEnergy;
    nrm = raw / (norm(raw) + eps);
    [~, peakTap] = max(abs(raw));
    cir = struct('raw', raw, 'norm', nrm, 'valid', valid, ...
        'peak_tap', peakTap - 1); % 0-based tap index
end
