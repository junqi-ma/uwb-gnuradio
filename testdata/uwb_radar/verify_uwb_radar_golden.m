function ok = verify_uwb_radar_golden(inDir)
%VERIFY_UWB_RADAR_GOLDEN  Read cf32+metadata and check SFD/SYNC arithmetic.
%
%   OK = VERIFY_UWB_RADAR_GOLDEN() checks testdata/uwb_radar goldens.
%   CIR vs estimateCir is skipped when UWB_demodulation / Communications
%   Toolbox is not on the path (this machine has no MATLAB).

    if nargin < 1 || isempty(inDir)
        inDir = fileparts(mfilename('fullpath'));
    end
    ok = true;
    meta = jsondecode(fileread(fullfile(inDir, 'metadata.json')));
    fprintf('UWB radar golden verification (MATLAB)\n');
    fprintf('  metadata generator=%s\n', string(meta.generator));

    if meta.sample_index_base ~= 0
        error('sample_index_base must be 0');
    end
    if meta.sync_repetitions ~= 64
        error('sync_repetitions must be 64');
    end
    if ~strcmp(char(meta.sfd_mode), '4z2')
        error('sfd_mode must be 4z2');
    end
    sps = meta.samples_per_symbol;
    nSync = meta.sync_repetitions;

    names = required_files(meta);
    for k = 1:size(names, 1)
        fname = names{k, 1};
        nExpect = names{k, 2};
        x = read_cf32(fullfile(inDir, fname));
        if numel(x) ~= nExpect
            error('%s length %d != metadata %d', fname, numel(x), nExpect);
        end
        fprintf('  %s: %d samples\n', fname, numel(x));
    end

    tx = read_cf32(fullfile(inDir, 'tx_998p4.cf32'));
    pk = max(abs(tx));
    if abs(pk - meta.peak_amplitude) > 1e-5
        error('tx_998p4 peak %g != %g', pk, meta.peak_amplitude);
    end
    fprintf('  tx_998p4 peak=%.9f\n', pk);

    coords = meta.coordinates_0based;
    check_pair('tx_998p4', coords.tx_998p4, nSync, sps);
    check_pair('rx_clean_998p4', coords.rx_clean_998p4, nSync, sps);
    check_pair('rx_delay_int_998p4', coords.rx_delay_int_998p4, nSync, sps);
    d = meta.delay_int.samples;
    if coords.rx_delay_int_998p4.sync_origin ~= coords.rx_clean_998p4.sync_origin + d
        error('delay_int SYNC origin not shifted by %d', d);
    end
    if coords.rx_delay_int_998p4.sfd_start ~= coords.rx_clean_998p4.sfd_start + d
        error('delay_int SFD start not shifted by %d', d);
    end
    fprintf('  SFD = sync_origin + %d*%g in all three coordinate systems\n', nSync, sps);

    native = read_cf32(fullfile(inDir, 'tx_737p28.cf32'));
    rt = resample(double(native), meta.resample.decim, meta.resample.interp);
    sync = double(tx(1:sps));
    corr = abs(conv(rt, flipud(conj(sync)), 'valid'));
    % Allow filter group delay; search a wide window for the first SYNC.
    % Cover both delay-compensated (peak~0) and uncompensated FIR (peak~gd).
    [~, first] = max(corr(1:min(numel(corr), 128)));
    first = first - 1; % 0-based into corr
    peaks = zeros(nSync, 1);
    peaks(1) = first;
    for k = 1:nSync-1
        center = first + k * sps + 1; % 1-based
        lo = max(1, round(center) - 2);
        hi = min(numel(corr), round(center) + 2);
        [~, rel] = max(corr(lo:hi));
        peaks(k+1) = (lo + rel - 1) - 1;
    end
    spacing = diff(peaks);
    maxErr = max(abs(spacing - sps));
    fprintf('  native round-trip SYNC spacing mean=%.6f max|err|=%.6f\n', ...
        mean(spacing), maxErr);
    if maxErr > 1 + 1e-9
        error('native round-trip SYNC spacing drifted by more than 1 sample');
    end

    % Optional CIR vs estimateCir (skip if demod path missing).
    try
        compare_cir_if_possible(inDir, meta, tx);
    catch err
        fprintf('  skip CIR vs estimateCir: %s\n', err.message);
    end
    fprintf('PASS\n');
end

function names = required_files(meta)
    radarTaps = meta.cir.radar.tap_count;
    demodTaps = meta.cir.demod_8_30.tap_count;
    rxn = meta.rx_window.length_998p4;
    names = {
        'tx_998p4.cf32', meta.tx_length_998p4
        'tx_737p28.cf32', meta.tx_length_737p28
        'rx_clean_998p4.cf32', rxn
        'rx_delay_int_998p4.cf32', rxn
        'rx_delay_frac_998p4.cf32', rxn
        'cir_raw_clean_radar.cf32', radarTaps
        'cir_norm_clean_radar.cf32', radarTaps
        'cir_raw_clean_8_30.cf32', demodTaps
        'cir_norm_clean_8_30.cf32', demodTaps
        'cir_raw_delay_int_radar.cf32', radarTaps
        'cir_norm_delay_int_radar.cf32', radarTaps
        'cir_raw_delay_int_8_30.cf32', demodTaps
        'cir_norm_delay_int_8_30.cf32', demodTaps
        };
end

function check_pair(label, c, nSync, sps)
    expect = c.sync_origin + nSync * sps;
    if c.sfd_start ~= expect
        error('%s: sfd_start %g != sync_origin %g + %d*%g', ...
            label, c.sfd_start, c.sync_origin, nSync, sps);
    end
    fprintf('  %s: sync_origin=%g sfd_start=%g\n', label, c.sync_origin, c.sfd_start);
end

function x = read_cf32(path)
    fid = fopen(path, 'rb');
    if fid < 0
        error('cannot read %s', path);
    end
    raw = fread(fid, inf, 'float32=>single');
    fclose(fid);
    x = complex(raw(1:2:end), raw(2:2:end));
end

function compare_cir_if_possible(inDir, meta, tx)
    if exist('uwbdecoder.estimateCir', 'file') ~= 2 ...
            && exist('+uwbdecoder/estimateCir.m', 'file') ~= 2
        error('estimateCir.m not on path');
    end
    rx = read_cf32(fullfile(inDir, 'rx_clean_998p4.cf32'));
    ref = struct();
    ref.fs = meta.rate_work_hz;
    ref.sampled_code = []; % would need HRPCodes
    % Without toolbox HRPCodes we cannot rebuild sampled_code; skip.
    if exist('lrwpan.internal.HRPCodes', 'class') == 0 ...
            && exist('lrwpan.internal.HRPCodes', 'file') == 0
        error('Communications Toolbox HRPCodes unavailable');
    end
    code = lrwpan.internal.HRPCodes(meta.code_index);
    spread = zeros(numel(code) * 4, 1);
    spread(1:4:end) = code(:);
    sampled = zeros(numel(spread) * 2, 1);
    sampled(1:2:end) = spread;
    reference = struct('fs', meta.rate_work_hz, 'sampled_code', sampled, ...
        'code_energy', real(sampled' * sampled));
    preamble = struct('start_sample', meta.coordinates_0based.rx_clean_998p4.sync_origin + 1, ...
        'measured_period', meta.samples_per_symbol, ...
        'detected_repetitions', meta.sync_repetitions, ...
        'search_half_width', meta.cir.radar.pre);
    params = struct('preamble_repetitions', meta.sync_repetitions, ...
        'cir_skip_initial_repetitions', meta.cir.skip_initial_repetitions, ...
        'cir_repetitions', meta.cir.repetitions_used, ...
        'cir_pre_samples', meta.cir.radar.pre, ...
        'cir_post_samples', meta.cir.radar.post, ...
        'cir_max_path_m', [], 'cir_store_individual_values', false, ...
        'show_plots', false, 'cir_timing', false);
    cir = uwbdecoder.estimateCir(double(rx), preamble, reference, params);
    gold = read_cf32(fullfile(inDir, 'cir_norm_clean_radar.cf32'));
    rel = norm(cir.values(:) - gold(:)) / (norm(gold) + eps);
    fprintf('  CIR vs estimateCir relative L2=%.3e (tx unused=%d)\n', rel, numel(tx));
    if rel > 1e-4
        error('CIR does not match estimateCir');
    end
end
