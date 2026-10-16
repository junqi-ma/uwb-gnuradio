function meta = export_comm_template(outDir, varargin)
%EXPORT_COMM_TEMPLATE  Independent MATLAB export of the S0 comm code-10 template.
%
%   META = EXPORT_COMM_TEMPLATE() writes into
%   testdata/sic_preamble/matlab/out/:
%
%     comm_sync_code10_998p4.cf32        samples_per_symbol (1016)
%                                        = first shaped SYNC symbol
%     comm_preamble_code10_998p4.cf32    sync_reps*1016 + n_taps (65072)
%                                        = finite 64-SYNC code-pulse train
%                                          convolved with the shaping FIR
%     sensing_sync_code9_998p4.cf32      samples_per_symbol (1016)
%                                        = code-9 sensing SYNC symbol
%     comm_pulse_taps_998p4.f32          n_taps (48) real float32 shaping taps
%     comm_template_meta.json            convention + length/energy/taps/hash
%
%   META = EXPORT_COMM_TEMPLATE(OUTDIR) writes into OUTDIR instead.
%
%   META = EXPORT_COMM_TEMPLATE(OUTDIR, 'Name', value, ...) options:
%     'PulseTapsFile'  float32 LE tap file used to CROSS-CHECK the fitted
%                      pulse (default testdata/uwb_hrp_tx/pulse_48.f32; ''
%                      disables the cross-check).  It is never read into the
%                      template directly unless the least-squares fit fails.
%     'PeakAmp'        peak normalisation target of the *preamble-only*
%                      waveform (default 0.8).
%     'SyncReps'       SYNC repetitions (default 64).
%     'CommCode'       comm HRP code index (default 10).
%     'SensingCode'    sensing HRP code index (default 9).
%
%   -----------------------------------------------------------------------
%   R07 CONVENTION (finite-length SYNC code-pulse train, NOT a truncated
%   full packet):
%
%     The comm preamble is the *natural response of the shaping filter to
%     the finite 64-SYNC code-pulse train*.  No SFD / PHR / payload sample
%     ever enters it, so editing the downstream packet cannot change it.
%
%       sample rate            fs                = 998.4e6            [Hz]
%       samples per chip       samplesPerChip    = 2                  [-]
%       preamble spread factor spreadingFactor   = 4                  [-]
%       Ipatov code length     codeLength        = 127 (ternary)      [-]
%       chips per symbol       chipsPerSymbol    = 127*4 = 508        [-]
%       samples per symbol     sps               = 508*2 = 1016       [-]
%       shaping FIR            n_taps            = 48 real causal,    [-]
%                              centre tap = 2,   peakAmp = 0.8
%
%     Placement (identical to the C++ `uwb_hrp_mod_core.h` SYNC builder):
%       for repetition r = 0 .. syncReps-1
%         for code entry  c = 0 .. codeLength-1  with code(c+1) ~= 0
%           chip_index   = r * chipsPerSymbol + c * spreadingFactor
%           sample_index = chip_index * samplesPerChip = r*1016 + c*8
%           train(sample_index+1) = code(c+1)          (zero-based 0..65016)
%     The causal FIR is placed with `taps(1)` at the pulse's sample index
%     (`add_shaped_chip`): y(n) = sum_k taps(k)*train(n-k+1).
%
%     Preamble = full linear convolution, kept to length
%       L = syncReps*sps + n_taps = 64*1016 + 48 = 65072 samples
%     The leading edge starts at sample 0 = taps(1)*code(1); the natural
%     filter tail runs to sample syncReps*sps - samplesPerChip*spreadingFactor
%     + n_taps - 1 = 65063 (zero-based), i.e. 8 zero pad samples close the
%     65072 convention.  Only the SYNC pulses contribute there -- there is
%     no SFD/PHR/payload.
%
%     Normalisation: the preamble is peak-normalised to PeakAmp (0.8) using
%     its OWN peak, NOT the full-packet peak, so it is independent of the
%     SFD/PHR/payload.  `comm_sync` is the first sps samples scaled with the
%     same factor.
%
%   Independence:  the ternary code comes from the Communications Toolbox
%   `lrwpan.internal.HRPCodes`, and the 48 shaping taps are recovered by a
%   MATLAB least-squares fit of the toolbox waveform's *SYNC region only*
%   against the code-pulse train.  Nothing is read from the C++ / Python
%   output, so this is the MATLAB-side reference for the G0 requirement
%   "code10 comm SYNC must be independently generated and checked against
%   MATLAB sample by sample"
%   (docs/sic/OpenCode开发指示_S0_前导碰撞仿真与耗时验证.md §8.1 item 2).
%
%   Required toolbox: Communications Toolbox (lrwpanWaveformGenerator,
%   lrwpanHRPConfig, lrwpanHRPFieldIndices, lrwpan.internal.HRPCodes).
%
%   NOTE (未实跑): no MATLAB/Octave is installed on the development host, so
%   this script has NOT been executed.  See README.md and not_run.log.

    if nargin < 1 || isempty(outDir)
        outDir = fullfile(fileparts(mfilename('fullpath')), 'out');
    end
    here = fileparts(mfilename('fullpath'));
    repo = fileparts(fileparts(fileparts(here)));   % .../testdata/sic_preamble/matlab

    opt = local_defaults(repo);
    opt = local_parse_options(opt, varargin);

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    if exist('lrwpanHRPConfig', 'file') ~= 2 && ...
            exist('lrwpanHRPConfig', 'class') == 0
        error('export_comm_template:NoToolbox', ...
            ['Communications Toolbox lrwpanHRPConfig is missing; ' ...
             'cannot generate the independent HRP template']);
    end
    if exist('lrwpan.internal.HRPCodes', 'file') ~= 2 && ...
            exist('lrwpan.internal.HRPCodes', 'class') == 0
        error('export_comm_template:NoHrpCodes', ...
            'lrwpan.internal.HRPCodes is missing from the Communications Toolbox');
    end

    fs = 998.4e6;                       % recorded in meta.rate_work_hz
    samplesPerChip = 2;                 % kQm35SamplesPerChip
    spreadingFactor = 4;                % kQm35SpreadingFactor
    codeLength = 127;                   % kQm35CodeLength
    chipsPerSymbol = 508;               % kQm35ChipsPerSymbol
    samplesPerSymbol = 1016;            % kQm35SamplesPerSymbol
    tailTapsExpected = 48;              % kPulseTaps (legacy)
    meanPrf = 62.4;
    dataRate = 6.81;
    samplesPerPulse = 2;
    psduBytes = 22;                     % canonical 20 data + 2 FCS bytes
    sfdNumber = 2;                      % 4z2

    syncReps = opt.SyncReps;
    peakAmp = opt.PeakAmp;
    commCode = opt.CommCode;
    sensingCode = opt.SensingCode;

    if chipsPerSymbol ~= codeLength * spreadingFactor
        error('export_comm_template:Geometry', 'chips/symbol mismatch');
    end
    if samplesPerSymbol ~= chipsPerSymbol * samplesPerChip
        error('export_comm_template:Geometry', 'samples/symbol mismatch');
    end

    % --- toolbox packets (code table + tap fit + whole-packet diagnostic) -
    cfg10 = local_hrp_config(meanPrf, dataRate, syncReps, commCode, ...
        samplesPerPulse, psduBytes, sfdNumber);
    idx10 = lrwpanHRPFieldIndices(cfg10);
    sps = (idx10.SYNC(end) - idx10.SYNC(1) + 1) / syncReps;
    if sps ~= samplesPerSymbol
        error('export_comm_template:SpsMismatch', ...
            'samples_per_symbol is %g, expected %d', sps, samplesPerSymbol);
    end
    raw10 = local_packet_raw(cfg10, psduBytes);
    wholePacketPeak = max(abs(raw10));
    syncRegion10 = raw10(idx10.SYNC(1) : idx10.SYNC(end));   % SYNC only
    if numel(syncRegion10) ~= syncReps * samplesPerSymbol
        error('export_comm_template:SyncRegionLength', ...
            'SYNC region is %d samples, expected %d', ...
            numel(syncRegion10), syncReps * samplesPerSymbol);
    end

    % --- finite code-pulse train -----------------------------------------
    code10 = local_hrp_code(commCode, codeLength);
    train10 = local_sync_train(code10, syncReps, samplesPerSymbol, ...
        spreadingFactor, samplesPerChip);

    % --- shaping FIR: independent least-squares from the SYNC region -----
    [pulseTaps, fitResidual, tapsSource, frozenDelta] = local_pulse_taps( ...
        syncRegion10, train10, tailTapsExpected, opt.PulseTapsFile);
    nTaps = numel(pulseTaps);
    if nTaps ~= tailTapsExpected
        error('export_comm_template:TapsLength', ...
            'pulse has %d taps, expected %d', nTaps, tailTapsExpected);
    end

    % --- finite-SYNC preamble: conv(train, taps), NO SFD/PHR/payload -----
    pre10raw = local_fit_length(conv(train10, pulseTaps), ...
        syncReps * samplesPerSymbol + nTaps);
    rawPeak = max(abs(pre10raw));
    if ~(rawPeak > 0)
        error('export_comm_template:EmptyPreamble', 'generated preamble is empty');
    end
    scale = peakAmp / rawPeak;
    pre10 = single(pre10raw * scale);

    % `comm_sync` is the first SYNC symbol, same construction, same scale.
    toolboxSync10 = raw10(idx10.SYNC(1) : idx10.SYNC(1) + samplesPerSymbol - 1);
    sync10 = single(toolboxSync10 * scale);

    % --- edge / tail self-checks (fail fast, they are hard invariants) ----
    % The natural SYNC-only response ends at sample `expectedLast` (1-based):
    %   last pulse sample (zero-based) + n_taps - 1, i.e. 65016 + 47 + 1.
    expectedLast = (syncReps - 1) * samplesPerSymbol + ...
        (codeLength - 1) * spreadingFactor * samplesPerChip + nTaps;
    ampTol = 1e-12 * (max(abs(pre10raw)) + eps);
    firstNz = find(abs(pre10raw) > ampTol, 1, 'first');
    lastNz = find(abs(pre10raw) > ampTol, 1, 'last');
    if numel(pre10raw) ~= syncReps * samplesPerSymbol + nTaps
        error('export_comm_template:PreambleLength', ...
            'preamble is %d samples, expected %d', numel(pre10raw), ...
            syncReps * samplesPerSymbol + nTaps);
    end
    if isempty(firstNz) || firstNz ~= 1
        error('export_comm_template:LeadingEdge', ...
            'preamble leading edge is not sample 0');
    end
    if isempty(lastNz) || lastNz > expectedLast
        error('export_comm_template:TailSupport', ...
            'preamble support reaches sample %g, past the natural tail %d', ...
            lastNz - 1, expectedLast - 1);
    end
    % Every sample strictly past the natural SYNC tail must be exactly zero:
    % this is the direct "no SFD/PHR/payload leaked into the tail" proxy.
    tailMaxAbs = max(abs(pre10raw(expectedLast + 1:end)));
    if tailMaxAbs ~= 0
        error('export_comm_template:TailLeak', ...
            'samples past the natural tail are not zero (max %g)', tailMaxAbs);
    end
    syncEdgeDiff = max(abs(double(sync10) - double(pre10(1:samplesPerSymbol))));

    % --- SFD-independence diagnostic (best effort, never fatal) -----------
    sfdDiag = local_sfd_independence(meanPrf, dataRate, syncReps, commCode, ...
        samplesPerPulse, psduBytes, sfdNumber, syncRegion10);

    % --- sensing code-9 SYNC (unchanged convention) ----------------------
    cfg9 = local_hrp_config(meanPrf, dataRate, syncReps, sensingCode, ...
        samplesPerPulse, psduBytes, sfdNumber);
    idx9 = lrwpanHRPFieldIndices(cfg9);
    sps9 = (idx9.SYNC(end) - idx9.SYNC(1) + 1) / syncReps;
    if sps9 ~= samplesPerSymbol
        error('export_comm_template:SensingSpsMismatch', ...
            'sensing samples_per_symbol is %g, expected %d', sps9, samplesPerSymbol);
    end
    packet9 = local_packet(cfg9, psduBytes, peakAmp);
    sync9 = packet9(idx9.SYNC(1) : idx9.SYNC(1) + sps9 - 1);

    % --- write CF32 + taps ----------------------------------------------
    files = struct();
    files.comm_sync_code10 = local_write_cf32( ...
        fullfile(outDir, 'comm_sync_code10_998p4.cf32'), sync10);
    files.comm_preamble_code10 = local_write_cf32( ...
        fullfile(outDir, 'comm_preamble_code10_998p4.cf32'), pre10);
    files.sensing_sync_code9 = local_write_cf32( ...
        fullfile(outDir, 'sensing_sync_code9_998p4.cf32'), sync9);
    files.comm_pulse_taps = local_write_f32( ...
        fullfile(outDir, 'comm_pulse_taps_998p4.f32'), pulseTaps);

    meta = struct();
    meta.schema_version = 2;
    meta.revision = 'R07';
    meta.generator = 'export_comm_template.m';
    meta.description = ['Independent MATLAB export of the S0 comm code-10 ' ...
        'template: finite 64-SYNC code-pulse train shaped by a 48-tap FIR, ' ...
        'no SFD/PHR/payload (R07).'];
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
    meta.samples_per_symbol = samplesPerSymbol;
    meta.sfd_number = sfdNumber;
    meta.peak_amplitude = peakAmp;
    meta.files = files;

    % --- the exact R07 generation convention -----------------------------
    meta.convention = struct( ...
        'kind', 'finite_sync_code_pulse_convolution', ...
        'contains_sfd', false, ...
        'contains_phr', false, ...
        'contains_payload', false, ...
        'samples_per_chip', samplesPerChip, ...
        'spreading_factor', spreadingFactor, ...
        'code_length', codeLength, ...
        'chips_per_symbol', chipsPerSymbol, ...
        'samples_per_symbol', samplesPerSymbol, ...
        'n_taps', nTaps, ...
        'tap_center', 2, ...
        'chip_index_formula', 'r*508 + c*4', ...
        'sample_index_formula', 'r*1016 + c*8', ...
        'conv_formula', 'y(n) = sum_k taps(k)*train(n-k+1)', ...
        'length_formula', 'sync_reps*samples_per_symbol + n_taps', ...
        'length', syncReps * samplesPerSymbol + nTaps, ...
        'first_pulse_sample', 0, ...
        'last_pulse_sample', (syncReps - 1) * samplesPerSymbol + ...
            (codeLength - 1) * spreadingFactor * samplesPerChip, ...
        'natural_tail_last_sample', expectedLast - 1, ...
        'last_significant_sample', lastNz - 1, ...
        'tail_region_max_abs', tailMaxAbs, ...
        'trailing_zero_samples', numel(pre10raw) - lastNz, ...
        'normalisation', 'own_peak', ...
        'scale', scale, ...
        'raw_peak', rawPeak, ...
        'whole_packet_peak_diagnostic', wholePacketPeak);

    meta.taps = struct( ...
        'source', tapsSource, ...
        'count', nTaps, ...
        'fit_max_abs_residual_rel', fitResidual, ...
        'file', files.comm_pulse_taps, ...
        'sha256', local_sha256_real(pulseTaps), ...
        'values', double(pulseTaps(:).'));
    meta.taps_frozen_cross_check = struct( ...
        'file', opt.PulseTapsFile, ...
        'max_abs_delta', frozenDelta);

    meta.comm_sync_length = numel(sync10);
    meta.comm_sync_peak = double(max(abs(sync10)));
    meta.comm_sync_energy = double(sum(abs(sync10).^2));
    meta.comm_sync_sha256 = local_sha256(sync10);
    meta.comm_preamble_length = numel(pre10);
    meta.comm_preamble_peak = double(max(abs(pre10)));
    meta.comm_preamble_energy = double(sum(abs(pre10).^2));
    meta.comm_preamble_sha256 = local_sha256(pre10);
    meta.comm_preamble_train_nonzero = double(sum(train10 ~= 0));
    meta.sensing_sync_length = numel(sync9);
    meta.sensing_sync_sha256 = local_sha256(sync9);
    meta.sync_edge_max_abs_diff = syncEdgeDiff;
    meta.sfd_independence = sfdDiag;
    meta.source = ['independent lrwpan HRPCodes + 48-tap lsq fit of the ' ...
        'toolbox SYNC region; preamble = conv(code-pulse train, taps), ' ...
        'no SFD/PHR/payload'];
    meta.verified_by = 'verify_sic_preamble.m';

    fid = fopen(fullfile(outDir, 'comm_template_meta.json'), 'w');
    if fid < 0
        error('export_comm_template:WriteMeta', ...
            'cannot write comm_template_meta.json in %s', outDir);
    end
    fwrite(fid, jsonencode(meta));
    fclose(fid);

    fprintf(['Wrote comm templates to %s\n' ...
        '  convention: %d SYNC * %d samples/symbol + %d taps = %d samples, ' ...
        'no SFD/PHR/payload\n' ...
        '  taps: %d (source %s, lsq residual %.3g, frozen delta %g)\n' ...
        '  comm_sync      = %d samples (sha256 %s)\n' ...
        '  comm_preamble  = %d samples (sha256 %s)\n' ...
        '  sensing_sync   = %d samples (sha256 %s)\n'], ...
        outDir, syncReps, samplesPerSymbol, nTaps, ...
        syncReps * samplesPerSymbol + nTaps, ...
        nTaps, tapsSource, fitResidual, frozenDelta, ...
        numel(sync10), meta.comm_sync_sha256, ...
        numel(pre10), meta.comm_preamble_sha256, ...
        numel(sync9), meta.sensing_sync_sha256);
end

% =========================================================================
% local helpers
% =========================================================================

function opt = local_defaults(repo)
    opt = struct( ...
        'PulseTapsFile', fullfile(repo, 'testdata', 'uwb_hrp_tx', 'pulse_48.f32'), ...
        'PeakAmp', 0.8, ...
        'SyncReps', 64, ...
        'CommCode', 10, ...
        'SensingCode', 9);
end

function opt = local_parse_options(opt, args)
    if mod(numel(args), 2) ~= 0
        error('export_comm_template:BadArgs', 'options must be name/value pairs');
    end
    for k = 1:2:numel(args)
        name = args{k};
        if ~isfield(opt, name)
            error('export_comm_template:UnknownOption', 'unknown option %s', name);
        end
        opt.(name) = args{k+1};
    end
end

function code = local_hrp_code(codeIndex, codeLength)
    code = lrwpan.internal.HRPCodes(codeIndex);
    code = int8(code(:));
    if numel(code) ~= codeLength
        error('export_comm_template:CodeLength', ...
            'HRPCodes(%d) has %d entries, expected %d', ...
            codeIndex, numel(code), codeLength);
    end
end

function train = local_sync_train(code, syncReps, samplesPerSymbol, ...
        spreadingFactor, samplesPerChip)
    train = zeros(syncReps * samplesPerSymbol, 1);
    step = spreadingFactor * samplesPerChip;   % samples between code entries
    for r = 0:syncReps-1
        base = r * samplesPerSymbol;
        for c = 0:numel(code)-1
            v = double(code(c + 1));
            if v ~= 0
                train(base + c * step + 1) = v;
            end
        end
    end
end

function [taps, resid, source, frozenDelta] = local_pulse_taps(target, train, ...
        nTaps, frozenFile)
    % Independent MATLAB least-squares recovery of the shaping FIR from the
    % toolbox waveform's SYNC region (never from the C++ output).
    target = target(:);
    train = train(:);
    n = numel(target);
    A = zeros(n, nTaps);
    for k = 1:nTaps
        A(k:n, k) = train(1:n - k + 1);
    end
    taps = A \ target;
    resid = max(abs(A * taps - target)) / (max(abs(target)) + eps);

    frozenDelta = NaN;
    frozen = [];
    if ~isempty(frozenFile) && exist(frozenFile, 'file') == 2
        frozen = local_read_f32(frozenFile);
        if numel(frozen) == nTaps
            frozenDelta = max(abs(taps(:) - frozen(:)));
        else
            frozen = [];
        end
    end

    source = 'lsq_fit_toolbox_sync_region';
    if ~all(isfinite(taps)) || resid > 1e-3
        % Never emit a bad independent fit: fall back to the frozen profile
        % taps, and say so in the meta.
        if ~isempty(frozen)
            taps = frozen(:);
            resid = NaN;
            source = 'frozen_file_fallback_lsq_residual_too_large';
        else
            error('export_comm_template:PulseFitFailed', ...
                ['least-squares pulse fit residual %.3g is too large and no ' ...
                 'frozen tap file is available'], resid);
        end
    end
end

function diag = local_sfd_independence(meanPrf, dataRate, syncReps, commCode, ...
        samplesPerPulse, psduBytes, sfdNumber, syncRegion)
    % Best-effort evidence that the SYNC region (and therefore the fitted
    % pulse + preamble) does not depend on the downstream SFD.  Never fatal.
    diag = struct('tested', false, 'max_abs_diff', NaN, ...
        'alt_sfd_number', NaN, 'note', 'skipped');
    opts = unique([1, 3, 4, sfdNumber + 1, sfdNumber - 1]);
    for k = 1:numel(opts)
        alt = opts(k);
        if alt == sfdNumber
            continue;
        end
        try
            cfg = local_hrp_config(meanPrf, dataRate, syncReps, commCode, ...
                samplesPerPulse, psduBytes, alt);
            idx = lrwpanHRPFieldIndices(cfg);
            raw = local_packet_raw(cfg, psduBytes);
            altRegion = raw(idx.SYNC(1) : idx.SYNC(end));
            if numel(altRegion) ~= numel(syncRegion)
                continue;
            end
            diag.tested = true;
            diag.alt_sfd_number = alt;
            diag.max_abs_diff = ...
                max(abs(altRegion - syncRegion)) / (max(abs(syncRegion)) + eps);
            diag.note = sprintf('SYNC region identical across SFDNumber %d vs %d', ...
                sfdNumber, alt);
            return;
        catch
            % invalid SFDNumber -> try the next candidate
        end
    end
end

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

function packet = local_packet_raw(cfg, psduBytes)
    payloadBits = zeros(psduBytes * 8, 1);
    [packet, ~] = lrwpanWaveformGenerator(payloadBits, cfg);
    packet = complex(double(packet(:)), 0);
    if isempty(packet)
        error('export_comm_template:EmptyPacket', 'generated packet is empty');
    end
end

function packet = local_packet(cfg, psduBytes, peakAmp)
    packet = local_packet_raw(cfg, psduBytes);
    pk = max(abs(packet));
    if ~(pk > 0)
        error('export_comm_template:EmptyPacket', 'generated packet is empty');
    end
    packet = packet * (peakAmp / pk);
end

function x = local_fit_length(x, L)
    x = x(:);
    if numel(x) < L
        x(end+1:L, 1) = 0; %#ok<AGROW>
    else
        x = x(1:L);
    end
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

function n = local_write_f32(path, x)
    x = single(x(:));
    fid = fopen(path, 'wb');
    if fid < 0
        error('export_comm_template:WriteF32', 'cannot write %s', path);
    end
    fwrite(fid, x, 'float32');
    fclose(fid);
    n = numel(x);
end

function x = local_read_f32(path)
    fid = fopen(path, 'rb');
    if fid < 0
        error('export_comm_template:ReadF32', 'cannot open %s', path);
    end
    x = fread(fid, inf, 'float32=>double');
    fclose(fid);
    x = x(:);
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

function hex = local_sha256_real(x)
    hex = '';
    try
        md = java.security.MessageDigest.getInstance('SHA-256');
        bytes = reshape(typecast(single(x(:)), 'int8'), 1, []);
        digest = md.digest(bytes);
        u8 = typecast(digest(:), 'uint8');
        hex = lower(sprintf('%02x', u8));
    catch
        hex = '';
    end
end
