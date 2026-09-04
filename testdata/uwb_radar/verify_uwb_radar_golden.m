function ok = verify_uwb_radar_golden(inDir)
%VERIFY_UWB_RADAR_GOLDEN  Hard-gate check of canonical MATLAB radar goldens.
%
%   OK = VERIFY_UWB_RADAR_GOLDEN() reads testdata/uwb_radar.
%   Prints exactly one of PASS / FAIL / SKIP/UNAVAILABLE.
%   CIR comparison is a hard gate: mismatch or exception is FAIL, not skip.
%
%   Exit intent:
%     PASS             ok=true
%     FAIL             error() after printing FAIL
%     SKIP/UNAVAILABLE ok=false, no PASS (toolbox missing)

    if nargin < 1 || isempty(inDir)
        inDir = fileparts(mfilename('fullpath'));
    end
    ok = false;

    if exist('lrwpanHRPConfig', 'file') ~= 2 && ...
            exist('lrwpanHRPConfig', 'class') == 0
        fprintf('SKIP/UNAVAILABLE: Communications Toolbox lrwpanHRPConfig missing\n');
        return;
    end

    here = fileparts(mfilename('fullpath'));
    testdataDir = fileparts(here);
    repo = fileparts(testdataDir);
    demodDir = fullfile(repo, 'UWB_demodulation');
    if exist(demodDir, 'dir')
        addpath(demodDir);
    else
        fprintf('FAIL: UWB_demodulation not found at %s\n', demodDir);
        error('verify_uwb_radar_golden:NoDemod', 'UWB_demodulation missing');
    end

    metaPath = fullfile(inDir, 'metadata.json');
    if exist(metaPath, 'file') ~= 2
        fprintf('FAIL: missing %s\n', metaPath);
        error('verify_uwb_radar_golden:NoMeta', 'metadata.json missing');
    end
    meta = jsondecode(fileread(metaPath));
    fprintf('UWB radar canonical golden verification (MATLAB)\n');
    fprintf('  generator=%s\n', string(meta.generator));

    if ~contains(string(meta.generator), 'export_uwb_radar_golden.m')
        fprintf('FAIL: generator is %s, need export_uwb_radar_golden.m\n', ...
            string(meta.generator));
        error('verify_uwb_radar_golden:NotCanonical', ...
            'this verifier only accepts MATLAB canonical goldens');
    end
    if meta.sample_index_base ~= 0
        fail_here('sample_index_base must be 0');
    end
    if meta.sync_repetitions ~= 64
        fail_here('sync_repetitions must be 64');
    end
    if ~strcmp(char(meta.sfd_mode), '4z2')
        fail_here('sfd_mode must be 4z2');
    end

    names = required_files(meta);
    for k = 1:size(names, 1)
        fname = names{k, 1};
        nExpect = names{k, 2};
        x = read_cf32(fullfile(inDir, fname));
        if numel(x) ~= nExpect
            fail_here(sprintf('%s length %d != metadata %d', fname, numel(x), nExpect));
        end
        fprintf('  %s: %d samples\n', fname, numel(x));
    end

    tx = read_cf32(fullfile(inDir, 'tx_998p4.cf32'));
    pk = max(abs(tx));
    if abs(pk - meta.peak_amplitude) > 1e-5
        fail_here(sprintf('tx_998p4 peak %g != %g', pk, meta.peak_amplitude));
    end
    fprintf('  tx_998p4 peak=%.9f\n', pk);

    sps = meta.samples_per_symbol;
    nSync = meta.sync_repetitions;
    coords = meta.coordinates_0based;
    check_pair('tx_998p4', coords.tx_998p4, nSync, sps);
    check_pair('rx_clean_998p4', coords.rx_clean_998p4, nSync, sps);
    check_pair('rx_delay_int_998p4', coords.rx_delay_int_998p4, nSync, sps);
    d = meta.delay_int.samples;
    if abs(coords.rx_delay_int_998p4.sync_origin - (coords.rx_clean_998p4.sync_origin + d)) > 1e-9
        fail_here(sprintf('delay_int SYNC origin not shifted by %d', d));
    end
    if abs(coords.rx_delay_int_998p4.sfd_start - (coords.rx_clean_998p4.sfd_start + d)) > 1e-9
        fail_here(sprintf('delay_int SFD start not shifted by %d', d));
    end
    if isfield(coords, 'rx_delay_frac_998p4')
        df = meta.delay_frac.samples;
        if abs(coords.rx_delay_frac_998p4.sync_origin - ...
                (coords.rx_clean_998p4.sync_origin + df)) > 1e-9
            fail_here('delay_frac SYNC origin not shifted by fractional delay');
        end
    end

    native = read_cf32(fullfile(inDir, 'tx_737p28.cf32'));
    rt = resample(double(native), meta.resample.decim, meta.resample.interp);
    sync = double(tx(1:sps));
    corr = abs(conv(rt, flipud(conj(sync)), 'valid'));
    [~, first] = max(corr(1:min(numel(corr), 128)));
    first = first - 1;
    peaks = zeros(nSync, 1);
    peaks(1) = first;
    for k = 1:nSync-1
        center = first + k * sps + 1;
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
        fail_here('native round-trip SYNC spacing drifted by more than 1 sample');
    end

    compare_cir_hard(inDir, meta);

    fprintf('PASS\n');
    ok = true;
end

function fail_here(msg)
    fprintf('FAIL: %s\n', msg);
    error('verify_uwb_radar_golden:Fail', '%s', msg);
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
        'cir_raw_delay_frac_radar.cf32', radarTaps
        'cir_norm_delay_frac_radar.cf32', radarTaps
        };
end

function check_pair(label, c, nSync, sps)
    expect = c.sync_origin + nSync * sps;
    if abs(c.sfd_start - expect) > 1e-9
        fail_here(sprintf('%s: sfd_start %g != sync_origin %g + %d*%g', ...
            label, c.sfd_start, c.sync_origin, nSync, sps));
    end
    fprintf('  %s: sync_origin=%g sfd_start=%g\n', label, c.sync_origin, c.sfd_start);
end

function x = read_cf32(path)
    fid = fopen(path, 'rb');
    if fid < 0
        fail_here(sprintf('cannot read %s', path));
    end
    raw = fread(fid, inf, 'float32=>single');
    fclose(fid);
    x = complex(raw(1:2:end), raw(2:2:end));
end

function compare_cir_hard(inDir, meta)
    code = lrwpan.internal.HRPCodes(meta.code_index);
    spread = zeros(numel(code) * 4, 1);
    spread(1:4:end) = code(:);
    sampled = zeros(numel(spread) * 2, 1);
    sampled(1:2:end) = spread;
    reference = struct('fs', meta.rate_work_hz, 'sampled_code', sampled, ...
        'code_energy', real(sampled' * sampled));

    cases = {
        'clean', 'rx_clean_998p4.cf32', 'cir_raw_clean_radar.cf32', ...
            'cir_norm_clean_radar.cf32', 0, 1.0
        'delay_int', 'rx_delay_int_998p4.cf32', 'cir_raw_delay_int_radar.cf32', ...
            'cir_norm_delay_int_radar.cf32', meta.delay_int.samples, meta.delay_int.gain_mag
        'delay_frac', 'rx_delay_frac_998p4.cf32', 'cir_raw_delay_frac_radar.cf32', ...
            'cir_norm_delay_frac_radar.cf32', meta.delay_frac.samples, meta.delay_frac.gain_mag
        };

    origin0 = meta.coordinates_0based.rx_clean_998p4.sync_origin;
    pre = meta.cir.radar.pre;
    post = meta.cir.radar.post;
    rawClean = [];
    peakClean = NaN;

    for k = 1:size(cases, 1)
        label = cases{k, 1};
        rx = double(read_cf32(fullfile(inDir, cases{k, 2})));
        goldRaw = read_cf32(fullfile(inDir, cases{k, 3}));
        goldNorm = read_cf32(fullfile(inDir, cases{k, 4}));
        extraDelay = cases{k, 5};
        gainMag = cases{k, 6};

        preamble = struct('start_sample', origin0 + 1, ...
            'measured_period', meta.samples_per_symbol, ...
            'detected_repetitions', meta.sync_repetitions, ...
            'search_half_width', pre);
        params = struct('preamble_repetitions', meta.sync_repetitions, ...
            'cir_skip_initial_repetitions', meta.cir.skip_initial_repetitions, ...
            'cir_repetitions', meta.cir.repetitions_used, ...
            'cir_pre_samples', pre, ...
            'cir_post_samples', post, ...
            'cir_max_path_m', [], 'cir_store_individual_values', false, ...
            'show_plots', false, 'cir_timing', false);
        cir = uwbdecoder.estimateCir(rx, preamble, reference, params);
        if numel(cir.values) ~= numel(goldNorm)
            fail_here(sprintf('%s CIR tap count %d != golden %d', ...
                label, numel(cir.values), numel(goldNorm)));
        end
        rel = norm(cir.values(:) - goldNorm(:)) / (norm(goldNorm) + eps);
        fprintf('  %s normalized vs estimateCir relative L2=%.3e\n', label, rel);
        if rel >= 1e-5
            fail_here(sprintf('%s normalized CIR L2 %g >= 1e-5', label, rel));
        end

        [~, pk] = max(abs(goldRaw));
        peakTap = pk - 1;
        fprintf('  %s raw peak_tap=%d |raw|=%.6g\n', label, peakTap, abs(goldRaw(pk)));
        if strcmp(label, 'clean')
            rawClean = goldRaw;
            peakClean = peakTap;
        else
            if isempty(rawClean)
                fail_here('clean CIR must be compared before delayed CIR');
            end
            shift = extraDelay;
            if abs(shift - round(shift)) < 1e-9
                relPeak = peakTap - peakClean;
                fprintf('  %s peak shift vs clean=%d (channel delay %g)\n', ...
                    label, relPeak, shift);
                if relPeak ~= round(shift)
                    fail_here(sprintf('%s integer peak shift %d != %g', ...
                        label, relPeak, shift));
                end
                amp = abs(goldRaw(pk)) / (abs(rawClean(peakClean + 1)) * gainMag + eps);
                fprintf('  %s raw amplitude vs |gain| ratio=%.6f\n', label, amp);
                if abs(amp - 1) > 0.05
                    fail_here(sprintf('%s raw amplitude ratio %g not ~1', label, amp));
                end
            else
                fprintf('  %s fractional delay %g: peak_tap=%d (sub-sample, no integer shift gate)\n', ...
                    label, shift, peakTap);
                amp = abs(goldRaw(pk)) / (abs(rawClean(peakClean + 1)) * gainMag + eps);
                fprintf('  %s raw amplitude vs |gain| ratio=%.6f\n', label, amp);
                if abs(amp - 1) > 0.15
                    fail_here(sprintf('%s frac raw amplitude ratio %g not ~1', label, amp));
                end
            end
        end
    end
end
