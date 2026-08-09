%% Export QM35825 realtime-demod golden vectors from decode_uwb.
%
% Feeds the clean code-9 radar testdata through the reference MATLAB decoder
% and exports per-stage reference outputs that the C++ demod core must match.
%
% Uses decode_uwb_all's fixed-interval window mode (same as
% run_decode_uwb_radar.m): each predicted slot is a short ~190 us window, so
% the adaptive energy gate works and the full pipeline completes cleanly.
%
% Outputs (under testdata/realtime_demod_golden/):
%   manifest.csv           sample / profile / result summary
%   window.cfile           the cropped frame IQ fed to the decoder
%   stage_timing.mat       preamble start/period/peaks/metric
%   stage_cfo.mat          CFO Hz + residual phase
%   stage_sfd.mat          SFD start/end/polarity/metric
%   stage_cir.mat          CIR taps + first path
%   stage_softchips.mat    soft-chip stream
%   stage_ns_sfd.mat       NS-SFD location
%   stage_phr.mat          coded/decoded PHR bits, SECDED, PSDU length
%   stage_payload.mat      payload bits/bytes, received/calculated FCS
%
% Coordinate convention: decode_uwb reports 1-based indices internally; this
% script converts all sample/chip positions to 0-based absolute.

clear; close all; clc;

proj = fileparts(mfilename('fullpath'));
addpath(proj);
uwbcode = fullfile(proj, 'UWB_demodulation');
addpath(uwbcode);

outdir = fullfile(proj, 'realtime_demod_golden');
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% ---- input: clean code-9 radar sample ----
sample.cfile = fullfile(proj, 'uwb_code9_preamble64_payload128_standard_sfd.cfile');
info = dir(sample.cfile);
sample.n = info.bytes / 8;
sample.fs = 998.4e6;
sample.t0_0based = 4992000;
sample.first_packet_time_s = sample.t0_0based / sample.fs;
sample.packet_interval_s = 1 / 200; % only the first slot matters for golden

fprintf('Using %s : %d samples\n', sample.cfile, sample.n);

%% ---- QM35825 radar profile (matches run_decode_uwb_radar.m) ----
options = struct();
options.fs_rx = sample.fs;
options.data_rate = 6.81;
options.preamble_repetitions = 16;
options.code_index = 9;
options.sfd_mode = '4z2';
options.cir_skip_initial_repetitions = 10;
options.cir_repetitions = options.preamble_repetitions - ...
    options.cir_skip_initial_repetitions;
options.max_psdu_bytes = 127;
options.verbose = false;
options.show_plots = false;
options.ant_num = 1;
options.channel_index = 1;
options.sample_offset = 0;
options.sample_num = sample.n;
options.file_name = sample.cfile;

% batch: fixed-interval windows centred on the known first SYNC.
batch = struct();
batch.detection_mode = 'fixed_interval';
batch.first_packet_time_s = sample.first_packet_time_s;
batch.packet_interval_s = sample.packet_interval_s;
batch.fixed_max_packets = 3; % only need the first clean slot
batch.pre_packet_guard_samples = round(10e-6 * options.fs_rx);
batch.window_samples = round(200e-6 * options.fs_rx); % covers full frame
batch.min_window_samples = 2.0e5;
batch.require_fcs_pass = false;
batch.save_individual_cir = false;
batch.output_directory = fullfile(outdir, 'decode_all_out');
batch.mat_file = fullfile(batch.output_directory, 'all_frames_cir.mat');
batch.summary_csv = fullfile(batch.output_directory, 'frame_summary.csv');

%% ---- run the reference decoder (windowed mode) ----
t_total = tic;
results = decode_uwb_all(options, batch);
elapsed = toc(t_total);

fprintf('decode_uwb_all done in %.3f s | scheduled=%d decoded=%d fcs_ok=%d\n', ...
    elapsed, results.candidate_count, results.packet_count, ...
    results.fcs_pass_count);

% ---- DEBUG: show results structure when nothing decoded ----
fprintf('results fields: '); disp(fieldnames(results)');
if isfield(results, 'frame_records') && ~isempty(results.frame_records)
    fr = results.frame_records{1};
elseif isfield(results, 'frames') && ~isempty(results.frames)
    fr = results.frames(1);
elseif isfield(results, 'decode_results') && ~isempty(results.decode_results)
    fr = results.decode_results{1};
else
    fprintf('candidate_count=%d packet_count=%d\n', ...
        results.candidate_count, results.packet_count);
    if isfield(results, 'candidates')
        fprintf('candidates fields: '); disp(fieldnames(results.candidates)');
    end
    if isfield(results, 'statuses')
        disp('statuses:'); disp(results.statuses');
    end
    error('export_realtime_demod_golden:noDecode', ...
        'No packets decoded; cannot export golden vectors.');
end
% Prefer first FCS-passing frame
for i = 1:numel(results.frame_records)
    if results.frame_records{i}.phr.fcs_pass
        fr = results.frame_records{i};
        break;
    end
end

% The per-window result struct fr has the same fields as decode_uwb's
% output (preamble, sfd, cir, chips, phr, payload, soft_chip_timing).
result = fr;

%% ---- export per-stage golden vectors (convert to 0-based) ----
% window IQ: re-derive the cropped frame the decoder used.
ref = uwbdecoder.buildUwbReference(options);
[cropped, preamble_cr, cropInfo] = uwbdecoder.cropToFrame(...
    uwbdecoder.readIqRaw(sample.cfile, 0, sample.n, 1, 'single'), ...
    result.preamble, ref, options);
write_iq_cfile(fullfile(outdir, 'window.cfile'), cropped);

% timing
timing.start_0based = result.preamble.start_sample - 1;
timing.start_uncropped_0based = result.preamble.start_sample_uncropped - 1;
timing.crop_start_0based = cropInfo.crop_start - 1;
timing.period = result.preamble.samples_per_repetition;
timing.metric = result.preamble.metric_peak;
timing.peaks_0based = result.preamble.peak_indices(:) - 1;
timing.detected_repetitions = result.preamble.detected_repetitions;
save(fullfile(outdir, 'stage_timing.mat'), '-struct', 'timing');

% CFO
cfo.hz = result.preamble.carrier_frequency_offset_hz;
save(fullfile(outdir, 'stage_cfo.mat'), '-struct', 'cfo');

% SFD
sfd.start_0based = result.sfd.start - 1;
sfd.end_0based = result.sfd.end - 1;
sfd.polarity = result.sfd.polarity;
sfd.metric = result.sfd.metric;
save(fullfile(outdir, 'stage_sfd.mat'), '-struct', 'sfd');

% CIR
cir.first_path = result.cir.first_path;
cir.pre_samples = result.cir.pre_samples;
cir.post_samples = result.cir.post_samples;
cir.values = result.cir.values;
save(fullfile(outdir, 'stage_cir.mat'), '-struct', 'cir');

% soft chips
softchips.stream = result.soft_chips;
softchips.samples_per_chip = result.soft_chip_timing.samples_per_chip;
softchips.first_chip_0based = result.soft_chip_timing.first_chip_sample_uncropped - 1;
softchips.num_chips = result.soft_chip_timing.num_chips;
save(fullfile(outdir, 'stage_softchips.mat'), '-struct', 'softchips');

% NS-SFD
ns_sfd.start_chip = result.sfd.start_chip - 1;
ns_sfd.end_chip = result.sfd.end_chip - 1;
ns_sfd.polarity = result.sfd.polarity;
ns_sfd.metric = result.sfd.metric;
save(fullfile(outdir, 'stage_ns_sfd.mat'), '-struct', 'ns_sfd');

% PHR
phr.start_chip = result.phr.start_chip - 1;
phr.end_chip = result.phr.end_chip - 1;
phr.coded_bits = result.phr.coded_bits;
phr.secded_pass = result.phr.secded_pass;
phr.psdu_length = result.phr.psdu_length_bytes;
save(fullfile(outdir, 'stage_phr.mat'), '-struct', 'phr');

% payload
payload.bits = result.payload.bits;
payload.bytes = result.payload.bytes;
payload.fcs_received = result.payload.fcs_received;
payload.fcs_calculated = result.payload.fcs_calculated;
payload.fcs_pass = result.payload.fcs_pass;
save(fullfile(outdir, 'stage_payload.mat'), '-struct', 'payload');

%% ---- manifest ----
fid = fopen(fullfile(outdir, 'manifest.csv'), 'w');
fprintf(fid, 'key,value\n');
fprintf(fid, 'sample,%s\n', sample.cfile);
fprintf(fid, 'fs_rx,%g\n', sample.fs);
fprintf(fid, 'code_index,%d\n', options.code_index);
fprintf(fid, 'preamble_repetitions,%d\n', options.preamble_repetitions);
fprintf(fid, 'sfd_mode,%s\n', options.sfd_mode);
fprintf(fid, 'data_rate,%g\n', options.data_rate);
fprintf(fid, 't0_0based,%d\n', sample.t0_0based);
fprintf(fid, 'decode_total_s,%.4f\n', elapsed);
fprintf(fid, 'scheduled,%d\n', results.candidate_count);
fprintf(fid, 'decoded,%d\n', results.packet_count);
fprintf(fid, 'fcs_ok,%d\n', results.fcs_pass_count);
fprintf(fid, 'preamble_start_0based,%d\n', timing.start_0based);
fprintf(fid, 'preamble_period,%.4f\n', timing.period);
fprintf(fid, 'preamble_metric,%.6f\n', timing.metric);
fprintf(fid, 'detected_peaks,%d\n', timing.detected_repetitions);
fprintf(fid, 'cfo_hz,%.4f\n', cfo.hz);
fprintf(fid, 'sfd_start_0based,%d\n', sfd.start_0based);
fprintf(fid, 'sfd_polarity,%d\n', sfd.polarity);
fprintf(fid, 'sfd_metric,%.6f\n', sfd.metric);
fprintf(fid, 'psdu_length,%d\n', phr.psdu_length);
fprintf(fid, 'secded_pass,%d\n', phr.secded_pass);
fprintf(fid, 'payload_bytes,%d\n', numel(payload.bytes));
fprintf(fid, 'fcs_pass,%d\n', payload.fcs_pass);
fprintf(fid, 'fcs_received,0x%04x\n', payload.fcs_received);
fprintf(fid, 'fcs_calculated,0x%04x\n', payload.fcs_calculated);
fclose(fid);

fprintf('Golden vectors written to %s\n', outdir);

%% ---- helpers ----
function write_iq_cfile(path, x)
    x = x(:);
    inter = zeros(2 * numel(x), 1, 'single');
    inter(1:2:end) = real(x);
    inter(2:2:end) = imag(x);
    fid2 = fopen(path, 'wb');
    if fid2 < 0, error('cannot open %s', path); end
    fwrite(fid2, inter, 'float32');
    fclose(fid2);
end
