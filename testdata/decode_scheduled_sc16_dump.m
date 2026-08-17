function results = decode_scheduled_sc16_dump(dumpDir, opts)
%DECODE_SCHEDULED_SC16_DUMP Parse a scheduled-extractor SC16 dump and decode.
%   RESULTS = DECODE_SCHEDULED_SC16_DUMP(DUMPDIR) reads
%       DUMPDIR/capture.iq + DUMPDIR/capture.jsonl
%   written by UwbPacketWriter after UwbAutoScheduledExtractorSc16 /
%   UwbScheduledExtractorSc16, slices each window into head / QM35 body /
%   tail, upsamples 65/48 with the same quality_minorder taps as
%   analyze_qm35_sc16_matlab.m, and calls decode_uwb (UWB_demodulation).
%
%   RESULTS = DECODE_SCHEDULED_SC16_DUMP(DUMPDIR, OPTS) accepts:
%     .decode_dw1000   (false) also run a DW1000 decode_uwb on the same
%                      998.4 window (code 10 / 256 SYNC / decawave)
%     .max_slots       ([]) limit scheduled+provisional packets
%     .output_dir      (DUMPDIR) CSV / MAT destination
%     .taps_file       ([]) quality_minorder 65/48 taps; default searches
%                      this repo then ../testdata/resampler_65_48/
%     .show_plots      (false)
%
%   Canonical copy: UWB_demodulation/decode_scheduled_sc16_dump.m
%   (acceleration). Keep this testdata copy path-aware so either location
%   works.
%
%   Seed convention matches analyze_qm35_sc16_matlab.m:
%     seededStartOne = round(pre * 65/48) + 1
%   so the QM35 search starts at the predicted radar origin inside the PDU.
%
%   See also ANALYZE_QM35_SC16_MATLAB, DECODE_UWB, READ_UWB_PACKET.

if nargin < 1 || isempty(dumpDir)
    error('decode_scheduled_sc16_dump:usage', ...
        'usage: results = decode_scheduled_sc16_dump(dumpDir [, opts])');
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'decode_dw1000'), opts.decode_dw1000 = false; end
if ~isfield(opts, 'max_slots'), opts.max_slots = []; end
if ~isfield(opts, 'output_dir'), opts.output_dir = dumpDir; end
if ~isfield(opts, 'taps_file'), opts.taps_file = []; end
if ~isfield(opts, 'show_plots'), opts.show_plots = false; end

thisDir = fileparts(mfilename('fullpath'));
if exist(fullfile(thisDir, 'decode_uwb.m'), 'file')
    addpath(thisDir);
else
    addpath(fullfile(fileparts(thisDir), 'UWB_demodulation'));
end

iqFile = fullfile(dumpDir, 'capture.iq');
jsonlFile = fullfile(dumpDir, 'capture.jsonl');
assert(isfile(iqFile), 'Missing %s', iqFile);
assert(isfile(jsonlFile), 'Missing %s', jsonlFile);

tapsFile = opts.taps_file;
if isempty(tapsFile)
    candidates = {
        fullfile(thisDir, 'testdata', 'resampler_65_48', ...
            'taps_quality_minorder.txt')
        fullfile(fileparts(thisDir), 'testdata', 'resampler_65_48', ...
            'taps_quality_minorder.txt')
        };
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            tapsFile = candidates{i};
            break
        end
    end
end
assert(~isempty(tapsFile) && isfile(tapsFile), ...
    'Cannot find taps_quality_minorder.txt; set opts.taps_file');
fid = fopen(tapsFile, 'rb');
assert(fid >= 0, 'Cannot open taps: %s', tapsFile);
taps = fread(fid, Inf, 'single=>single');
fclose(fid);
assert(~isempty(taps), 'Empty taps: %s', tapsFile);
filterDelay = (numel(taps) - 1) / 2;
interp = 65;
decim = 48;
fs998 = 998.4e6;

metas = readDumpJsonl(jsonlFile);
nAll = numel(metas);
keep = true(1, nAll);
if ~isempty(opts.max_slots)
    scheduled = false(1, nAll);
    for k = 1:nAll
        scheduled(k) = isScheduledMeta(metas(k));
    end
    idx = find(scheduled);
    if numel(idx) > opts.max_slots
        drop = idx((opts.max_slots + 1):end);
        keep(drop) = false;
    end
end
metas = metas(keep);
nPkt = numel(metas);

if ~isfolder(opts.output_dir)
    mkdir(opts.output_dir);
end

qm35Opt = struct();
qm35Opt.fs_rx = fs998;
qm35Opt.data_rate = 6.81;
qm35Opt.preamble_repetitions = 64;
qm35Opt.code_index = 9;
qm35Opt.sfd_mode = '4z2';
qm35Opt.cir_skip_initial_repetitions = 10;
qm35Opt.cir_repetitions = 54;
qm35Opt.max_psdu_bytes = 127;
qm35Opt.enable_frame_crop = true;
qm35Opt.show_plots = opts.show_plots;
qm35Ref = uwbdecoder.buildUwbReference( ...
    uwbdecoder.mergeOptions(uwbdecoder.defaultOptions(), qm35Opt));

dwRef = [];
dwOpt = struct();
if opts.decode_dw1000
    dwOpt.fs_rx = fs998;
    dwOpt.data_rate = 6.81;
    dwOpt.preamble_repetitions = 256;
    dwOpt.code_index = 10;
    dwOpt.sfd_mode = 'decawave';
    dwOpt.cir_skip_initial_repetitions = 10;
    dwOpt.cir_repetitions = 64;
    dwOpt.max_psdu_bytes = 127;
    dwOpt.enable_frame_crop = true;
    dwOpt.show_plots = false;
    dwRef = uwbdecoder.buildUwbReference( ...
        uwbdecoder.mergeOptions(uwbdecoder.defaultOptions(), dwOpt));
end

fprintf(['Scheduled SC16 dump decode: dir=%s packets=%d/%d ', ...
    'taps=%d dw1000=%d\n'], dumpDir, nPkt, nAll, numel(taps), ...
    opts.decode_dw1000);

cells = cell(nPkt, 1);
usePar = ~isempty(ver('parallel')) && ~isempty(gcp('nocreate'));
if usePar
    parfor k = 1:nPkt
        cells{k} = decodeOne(iqFile, metas(k), taps, filterDelay, ...
            interp, decim, qm35Opt, qm35Ref, dwOpt, dwRef, ...
            opts.decode_dw1000);
    end
else
    for k = 1:nPkt
        cells{k} = decodeOne(iqFile, metas(k), taps, filterDelay, ...
            interp, decim, qm35Opt, qm35Ref, dwOpt, dwRef, ...
            opts.decode_dw1000);
        if mod(k, 10) == 0 || k == nPkt
            fprintf('  ... %d/%d\n', k, nPkt);
        end
    end
end

results = vertcat(cells{:});
tableOut = struct2table(results);
csvFile = fullfile(opts.output_dir, 'scheduled_dump_matlab.csv');
writetable(tableOut, csvFile);
save(fullfile(opts.output_dir, 'scheduled_dump_matlab.mat'), ...
    'results', '-v7');

qm35Ok = [results.qm35_decode_ok];
qm35Fcs = [results.qm35_fcs_pass];
fprintf('QM35: decoded=%d/%d  FCS=%d/%d\n', ...
    nnz(qm35Ok), nPkt, nnz(qm35Fcs), nPkt);
if opts.decode_dw1000
    dwOk = [results.dw_decode_ok];
    dwFcs = [results.dw_fcs_pass];
    fprintf('DW1000: decoded=%d/%d  FCS=%d/%d\n', ...
        nnz(dwOk), nPkt, nnz(dwFcs), nPkt);
end
fprintf('Wrote %s\n', csvFile);
end

function metas = readDumpJsonl(jsonlFile)
% Field sets differ between acquisition and scheduled lines; union them.
fid = fopen(jsonlFile, 'r');
assert(fid >= 0, 'Cannot open %s', jsonlFile);
cleanup = onCleanup(@() fclose(fid));
raw = {};
while true
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    line = strtrim(line);
    if isempty(line)
        continue
    end
    raw{end + 1} = jsondecode(line); %#ok<AGROW>
end
if isempty(raw)
    error('decode_scheduled_sc16_dump:EmptyJsonl', 'no JSONL rows in %s', ...
        jsonlFile);
end
keys = {};
for k = 1:numel(raw)
    keys = union(keys, fieldnames(raw{k}), 'stable');
end
template = cell2struct(repmat({[]}, numel(keys), 1), keys, 1);
metas = repmat(template, numel(raw), 1);
for k = 1:numel(raw)
    f = fieldnames(raw{k});
    for i = 1:numel(f)
        metas(k).(f{i}) = raw{k}.(f{i});
    end
end
end

function tf = isScheduledMeta(meta)
mode = '';
if isfield(meta, 'capture_mode') && ~isempty(meta.capture_mode)
    mode = char(meta.capture_mode);
end
tf = any(strcmp(mode, {'scheduled', 'provisional'}));
end

function [pre, body, post] = geometryOf(meta, n)
pre = fieldOr(meta, 'pre_trigger_samples', []);
if isempty(pre)
    pre = fieldOr(meta, 'pre_guard_samples', 0);
end
body = fieldOr(meta, 'capture_samples', []);
post = fieldOr(meta, 'post_guard_samples', []);
if isempty(body)
    if isempty(post)
        body = max(0, n - pre);
        post = 0;
    else
        body = max(0, n - pre - post);
    end
end
if isempty(post)
    post = max(0, n - pre - body);
end
pre = max(0, min(n, round(double(pre))));
body = max(0, min(n - pre, round(double(body))));
post = max(0, n - pre - body);
end

function v = fieldOr(s, name, fallback)
if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = fallback;
end
end

function out = decodeOne(iqFile, meta, taps, filterDelay, interp, decim, ...
        qm35Opt, qm35Ref, dwOpt, dwRef, doDw)
n = double(meta.sample_count);
[pre, body, post] = geometryOf(meta, n);
windowStart = fieldOr(meta, 'window_start_sample', ...
    fieldOr(meta, 'start_sample', 0));
predicted = fieldOr(meta, 'predicted_start_sample', ...
    windowStart + pre);
packetId = fieldOr(meta, 'packet_id', -1);
schedIdx = fieldOr(meta, 'schedule_index', -1);
mode = '';
if isfield(meta, 'capture_mode') && ~isempty(meta.capture_mode)
    mode = string(meta.capture_mode);
end

out = emptyResult(packetId, schedIdx, mode, windowStart, predicted, ...
    pre, body, post, n, interp, decim, filterDelay);

try
    x737 = readDumpIq(iqFile, meta);
    if numel(x737) ~= n
        error('dump:ShortIq', 'IQ length %d != sample_count %d', ...
            numel(x737), n);
    end
    x998 = upfirdn(x737, taps, interp, decim);
    seededStartOne = round(pre * interp / decim) + 1;
    result = decode_uwb(qm35Opt, x998, [], qm35Ref, 'single', ...
        seededStartOne);

    windowStartOut = round((double(windowStart) * interp + filterDelay) / decim);
    predictedOut = round((double(predicted) * interp + filterDelay) / decim);
    detectedOut = windowStartOut + ...
        double(result.preamble.start_sample_uncropped) - 1;

    out.qm35_decode_ok = true;
    out.qm35_fcs_pass = logical(result.payload.fcs_pass);
    if out.qm35_fcs_pass
        out.qm35_status = "success";
    elseif result.phr.secded_pass
        out.qm35_status = "fcs_or_payload_failed";
    else
        out.qm35_status = "phr_failed";
    end
    out.qm35_predicted_start_out = predictedOut;
    out.qm35_detected_start_out = detectedOut;
    out.qm35_det_minus_pred = detectedOut - predictedOut;
    out.qm35_detected_repetitions = result.preamble.detected_repetitions;
    out.qm35_timing_metric = result.preamble.metric_peak;
    out.qm35_cfo_hz = result.preamble.carrier_frequency_offset_hz;
    out.qm35_sfd_correlation = result.preamble.sfd_waveform_correlation;
    out.qm35_phr_secded_pass = logical(result.phr.secded_pass);
    out.qm35_psdu_length = result.phr.psdu_length_bytes;
    if isempty(result.payload.bytes)
        out.qm35_payload_hex = "";
    else
        out.qm35_payload_hex = string(sprintf('%02X', result.payload.bytes));
    end

    if doDw
        dw = decode_uwb(dwOpt, x998, [], dwRef, 'single', []);
        out.dw_decode_ok = true;
        out.dw_fcs_pass = logical(dw.payload.fcs_pass);
        if out.dw_fcs_pass
            out.dw_status = "success";
        elseif dw.phr.secded_pass
            out.dw_status = "fcs_or_payload_failed";
        else
            out.dw_status = "phr_failed";
        end
        out.dw_detected_start_out = windowStartOut + ...
            double(dw.preamble.start_sample_uncropped) - 1;
        out.dw_detected_repetitions = dw.preamble.detected_repetitions;
        out.dw_timing_metric = dw.preamble.metric_peak;
        out.dw_cfo_hz = dw.preamble.carrier_frequency_offset_hz;
        out.dw_sfd_correlation = dw.preamble.sfd_waveform_correlation;
        if isempty(dw.payload.bytes)
            out.dw_payload_hex = "";
        else
            out.dw_payload_hex = string(sprintf('%02X', dw.payload.bytes));
        end
    end
catch err
    out.qm35_status = "decode_error";
    out.error_id = string(err.identifier);
    out.error_message = string(err.message);
end
end

function x = readDumpIq(iqFile, meta)
if isfield(meta, 'file') && ~isempty(meta.file)
    f = fullfile(fileparts(iqFile), char(meta.file));
    offset = 0;
else
    f = iqFile;
    offset = double(meta.file_offset_samples);
end
fid = fopen(f, 'rb', 'ieee-le');
if fid < 0
    error('dump:OpenFailed', 'cannot open %s', f);
end
cleanup = onCleanup(@() fclose(fid));
n = double(meta.sample_count);
status = fseek(fid, offset * 4, 'bof');
if status ~= 0
    error('dump:SeekFailed', 'cannot seek offset %d', offset);
end
raw = fread(fid, 2 * n, 'int16=>single');
if numel(raw) ~= 2 * n
    error('dump:ShortRead', 'short read at offset %d', offset);
end
% Same as analyze_qm35_sc16_matlab: int16 as float, no 1/32768.
% decode_uwb is scale-invariant for timing / FCS.
x = complex(raw(1:2:end), raw(2:2:end));
end

function out = emptyResult(packetId, schedIdx, mode, windowStart, ...
        predicted, pre, body, post, n, interp, decim, filterDelay)
out = struct( ...
    'packet_id', double(packetId), ...
    'schedule_index', double(schedIdx), ...
    'capture_mode', string(mode), ...
    'window_start_native', double(windowStart), ...
    'predicted_start_native', double(predicted), ...
    'pre_samples', double(pre), ...
    'body_samples', double(body), ...
    'post_samples', double(post), ...
    'sample_count', double(n), ...
    'qm35_status', "decode_error", ...
    'qm35_decode_ok', false, ...
    'qm35_fcs_pass', false, ...
    'qm35_predicted_start_out', ...
        round((double(predicted) * interp + filterDelay) / decim), ...
    'qm35_detected_start_out', -1, ...
    'qm35_det_minus_pred', NaN, ...
    'qm35_detected_repetitions', 0, ...
    'qm35_timing_metric', NaN, ...
    'qm35_cfo_hz', NaN, ...
    'qm35_sfd_correlation', NaN, ...
    'qm35_phr_secded_pass', false, ...
    'qm35_psdu_length', 0, ...
    'qm35_payload_hex', "", ...
    'dw_status', "", ...
    'dw_decode_ok', false, ...
    'dw_fcs_pass', false, ...
    'dw_detected_start_out', -1, ...
    'dw_detected_repetitions', 0, ...
    'dw_timing_metric', NaN, ...
    'dw_cfo_hz', NaN, ...
    'dw_sfd_correlation', NaN, ...
    'dw_payload_hex', "", ...
    'error_id', "", ...
    'error_message', "");
end
