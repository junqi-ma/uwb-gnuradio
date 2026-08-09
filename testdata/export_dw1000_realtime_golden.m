%% Export one real, preprocessed DW1000 frame for C++ S1 verification.
%
% Source capture contract:
%   F:\UWB基带数据\dw1000_new_processed_1.dat
%   CF work rate 998.4 MHz after resampling, tone removal and frequency shift.
% Existing MATLAB batch results select the first FCS-valid code-11 frame.

clear; clc;

testdata_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(testdata_dir);
addpath(fullfile(project_dir, 'UWB_demodulation'));

capture_file = 'F:\UWB基带数据\dw1000_new_processed_1.dat';
results_file = ['F:\USRP数据解调\decoded_results\', ...
    'dw1000_new_processed_1\all_frames_cir.mat'];
outdir = fullfile(testdata_dir, 'dw1000_realtime_golden');
if ~exist(outdir, 'dir'), mkdir(outdir); end

saved = load(results_file, 'results');
results = saved.results;
params = results.params;
assert(params.fs_rx == 998.4e6);
assert(params.code_index == 11);
assert(params.preamble_repetitions == 128);
assert(strcmpi(params.sfd_mode, 'decawave'));

frame_index = find([results.frames.fcs_pass], 1, 'first');
assert(~isempty(frame_index));
frame = results.frames(frame_index);

guard = 10000;
window_start_0based = max(0, frame.abs_start_sample - guard);
window_end_0based = frame.abs_end_sample + guard;
sample_count = window_end_0based - window_start_0based + 1;
raw = uwbdecoder.readIqRaw(capture_file, window_start_0based, ...
    sample_count, params.ant_num, 'single');
rx = uwbdecoder.selectIqChannel(raw, params.channel_index);

reference = uwbdecoder.buildUwbReference(params);
write_complex_f32(fullfile(outdir, 'window.cfile'), rx);
write_complex_f32(fullfile(outdir, 'reference_preamble.cfile'), ...
    reference.preamble_waveform);
write_complex_f32(fullfile(outdir, 'stage_cir.cfile'), frame.cir.values);

fid_payload = fopen(fullfile(outdir, 'payload.bin'), 'wb');
assert(fid_payload >= 0);
payload_cleanup = onCleanup(@() fclose(fid_payload));
fwrite(fid_payload, uint8(frame.payload_bytes), 'uint8');
clear payload_cleanup;

fid = fopen(fullfile(outdir, 'manifest.csv'), 'w');
assert(fid >= 0);
manifest_cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'key,value\n');
fprintf(fid, 'profile,dw1000-production\n');
fprintf(fid, 'matlab_release,%s\n', version('-release'));
fprintf(fid, 'source_capture,%s\n', capture_file);
fprintf(fid, 'source_results,%s\n', results_file);
fprintf(fid, 'source_frame_index,%d\n', frame_index);
fprintf(fid, 'sample_rate,%.0f\n', params.fs_rx);
fprintf(fid, 'code_index,%d\n', params.code_index);
fprintf(fid, 'preamble_repetitions,%d\n', params.preamble_repetitions);
fprintf(fid, 'sfd_mode,%s\n', params.sfd_mode);
fprintf(fid, 'window_start_0based,%d\n', window_start_0based);
fprintf(fid, 'window_sample_count,%d\n', sample_count);
fprintf(fid, 'preamble_start_absolute_0based,%d\n', frame.abs_start_sample);
fprintf(fid, 'preamble_start_window_0based,%d\n', ...
    frame.abs_start_sample - window_start_0based);
fprintf(fid, 'frame_end_absolute_0based,%d\n', frame.abs_end_sample);
fprintf(fid, 'detected_repetitions,%d\n', frame.detected_repetitions);
fprintf(fid, 'samples_per_repetition,%.9f\n', ...
    frame.samples_per_repetition);
fprintf(fid, 'cfo_hz,%.9f\n', frame.carrier_frequency_offset_hz);
fprintf(fid, 'sfd_name,%s\n', frame.sfd_name);
fprintf(fid, 'sfd_correlation,%.9f\n', frame.sfd_correlation);
fprintf(fid, 'psdu_length_bytes,%d\n', frame.psdu_length_bytes);
fprintf(fid, 'fcs_received,0x%04X\n', frame.fcs_received);
fprintf(fid, 'fcs_calculated,0x%04X\n', frame.fcs_calculated);
fprintf(fid, 'fcs_pass,%d\n', frame.fcs_pass);
fprintf(fid, 'cir_tap_count,%d\n', numel(frame.cir.values));
fprintf(fid, 'cir_pre_samples,%d\n', frame.cir.pre_samples);
fprintf(fid, 'cir_post_samples,%d\n', frame.cir.post_samples);
fprintf(fid, 'cir_repetition_count,%d\n', frame.cir.repetition_count);

fprintf('DW1000 realtime golden written to %s\n', outdir);

function write_complex_f32(path, values)
    values = single(values(:));
    interleaved = zeros(2 * numel(values), 1, 'single');
    interleaved(1:2:end) = real(values);
    interleaved(2:2:end) = imag(values);
    fid = fopen(path, 'wb');
    if fid < 0, error('Cannot open %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid, interleaved, 'single');
end
