function results = analyze_qm35_sc16_matlab()
%ANALYZE_QM35_SC16_MATLAB Decode the scheduled QM35 slots with MATLAB.
%   Reads the real 737.28 MS/s SC16 capture by fixed t0/T windows, applies
%   the same quality_minorder 65/48 upfirdn contract as the GNU Radio
%   offline path, decodes each 998.4 MS/s PDU, and writes a slot-level CSV.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'UWB_demodulation'));

capture = 'F:\UWB基带数据\qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat';
tapsFile = fullfile(root, 'testdata', 'resampler_65_48', ...
    'taps_quality_minorder.txt');
outputDir = fullfile(root, 'testdata', 'qm35_matlab_compare');
if ~isfolder(outputDir), mkdir(outputDir); end

fs998 = 998.4e6;
interp = 65;
decim = 48;
t0 = 3543552;                  % zero-based, 737.28 MHz domain
period = 3686400;              % 5 ms, 737.28 MHz domain
pre = 30000;
captureSamples = 160000;
post = 10000;
windowSamples = pre + captureSamples + post;

info = dir(capture);
assert(~isempty(info), 'Capture not found: %s', capture);
totalSamples = floor(info.bytes/4);
slotCount = 0;
while t0 + slotCount*period + captureSamples + post <= totalSamples
    slotCount = slotCount + 1;
end

fid = fopen(tapsFile, 'rb');
assert(fid >= 0, 'Cannot open taps: %s', tapsFile);
taps = fread(fid, Inf, 'single=>single');
fclose(fid);
assert(~isempty(taps), 'Empty taps: %s', tapsFile);
filterDelay = (numel(taps)-1)/2;

options = struct();
options.fs_rx = fs998;
options.data_rate = 6.81;
options.preamble_repetitions = 64;
options.code_index = 9;
options.sfd_mode = '4z2';
options.cir_skip_initial_repetitions = 10;
options.cir_repetitions = 54;
options.max_psdu_bytes = 127;
options.enable_frame_crop = true;
options.show_plots = false;
reference = uwbdecoder.buildUwbReference( ...
    uwbdecoder.mergeOptions(uwbdecoder.defaultOptions(), options));

fprintf(['MATLAB QM35 scheduled decode: slots=%d, input=%d samples, ', ...
    'window=%d, taps=%d\n'], slotCount, totalSamples, windowSamples, ...
    numel(taps));

cells = cell(slotCount, 1);
if isempty(gcp('nocreate'))
    parpool('local');
end
parfor slotOne = 1:slotCount
    slot = slotOne - 1;
    cells{slotOne} = decodeOne(capture, taps, reference, options, slot, ...
        t0, period, pre, captureSamples, post, interp, decim, ...
        filterDelay);
end

results = vertcat(cells{:});
tableOut = struct2table(results);
csvFile = fullfile(outputDir, 'matlab_results.csv');
writetable(tableOut, csvFile);
save(fullfile(outputDir, 'matlab_results.mat'), 'results', '-v7');

decodeOk = [results.decode_ok];
fcsOk = [results.fcs_pass];
fprintf('MATLAB summary: decoded=%d/%d, FCS=%d/%d\n', ...
    nnz(decodeOk), slotCount, nnz(fcsOk), slotCount);
fprintf('Wrote %s\n', csvFile);
end

function out = decodeOne(capture, taps, reference, options, slot, ...
        t0, period, pre, captureSamples, post, interp, decim, ...
        filterDelay)
predNative = t0 + slot*period;
windowStartNative = predNative - pre;
nNative = pre + captureSamples + post;

out = emptyResult(slot, predNative, windowStartNative, interp, decim, ...
    filterDelay);
try
    fid = fopen(capture, 'rb');
    if fid < 0, error('qm35:OpenFailed', 'Cannot open capture'); end
    cleanup = onCleanup(@() fclose(fid));
    status = fseek(fid, 4*windowStartNative, 'bof');
    if status ~= 0, error('qm35:SeekFailed', 'Cannot seek slot %d', slot); end
    raw = fread(fid, 2*nNative, 'int16=>single');
    if numel(raw) ~= 2*nNative
        error('qm35:ShortRead', 'Short read in slot %d', slot);
    end
    x737 = complex(raw(1:2:end), raw(2:2:end));
    x998 = upfirdn(x737, taps, interp, decim);

    % Match GNU Radio's seed convention: the mapped schedule position is
    % pre*65/48 samples into the PDU. The FIR delay is discovered by timing.
    seededStartOne = round(pre*interp/decim) + 1;
    result = decode_uwb(options, x998, [], reference, 'single', ...
        seededStartOne);

    windowStartOut = round((windowStartNative*interp + filterDelay)/decim);
    predictedOut = round((predNative*interp + filterDelay)/decim);
    detectedOut = windowStartOut + ...
        double(result.preamble.start_sample_uncropped) - 1;

    out.decode_ok = true;
    out.fcs_pass = logical(result.payload.fcs_pass);
    if out.fcs_pass
        out.status = "success";
    elseif result.phr.secded_pass
        out.status = "fcs_or_payload_failed";
    else
        out.status = "phr_failed";
    end
    out.predicted_start_out = predictedOut;
    out.detected_start_out = detectedOut;
    out.det_minus_pred = detectedOut - predictedOut;
    out.detected_repetitions = result.preamble.detected_repetitions;
    out.timing_metric = result.preamble.metric_peak;
    out.clock_error_ppm = result.preamble.sample_clock_error_ppm;
    out.cfo_hz = result.preamble.carrier_frequency_offset_hz;
    out.sfd_correlation = result.preamble.sfd_waveform_correlation;
    out.phr_secded_pass = logical(result.phr.secded_pass);
    out.psdu_length = result.phr.psdu_length_bytes;
    out.fcs_received = double(result.payload.fcs_received);
    out.fcs_calculated = double(result.payload.fcs_calculated);
    if isempty(result.payload.bytes)
        out.payload_hex = "";
    else
        out.payload_hex = string(sprintf('%02X', result.payload.bytes));
    end
catch err
    out.status = "decode_error";
    out.error_id = string(err.identifier);
    out.error_message = string(err.message);
end
end

function out = emptyResult(slot, predNative, windowStartNative, ...
        interp, decim, filterDelay)
out = struct( ...
    'slot', slot, ...
    'status', "decode_error", ...
    'decode_ok', false, ...
    'fcs_pass', false, ...
    'predicted_start_native', predNative, ...
    'window_start_native', windowStartNative, ...
    'predicted_start_out', ...
        round((predNative*interp + filterDelay)/decim), ...
    'detected_start_out', -1, ...
    'det_minus_pred', NaN, ...
    'detected_repetitions', 0, ...
    'timing_metric', NaN, ...
    'clock_error_ppm', NaN, ...
    'cfo_hz', NaN, ...
    'sfd_correlation', NaN, ...
    'phr_secded_pass', false, ...
    'psdu_length', 0, ...
    'fcs_received', 0, ...
    'fcs_calculated', 0, ...
    'payload_hex', "", ...
    'error_id', "", ...
    'error_message', "");
end
