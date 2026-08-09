%% Export one real, preprocessed DW1000 frame for C++ S1 verification.
%
% Source capture contract:
%   F:\UWB基带数据\dw1000_new_processed_1.dat
%   CF work rate 998.4 MHz after resampling, tone removal and frequency shift.
% Existing MATLAB batch results select the first FCS-valid code-11 frame.

clear; clc;

testdata_dir = fileparts(mfilename('fullpath'));
project_dir = fileparts(testdata_dir);
algorithm_dir = fullfile(project_dir, 'UWB_demodulation');
if ~isfolder(algorithm_dir)
    % The SIC worktree intentionally does not track the ignored MATLAB tree.
    algorithm_dir = fullfile(fileparts(project_dir), ...
        'uwb-gnuradio', 'UWB_demodulation');
end
assert(isfolder(algorithm_dir), ...
    'Cannot locate the UWB_demodulation MATLAB algorithm source.');
addpath(algorithm_dir);

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

% Reconstruct the exact production DW1000 pulse train and apply the decoded
% complex CIR. This is the direct-first, integer-alignment S2 baseline; the
% offline fractional/PLL/CIR-slow/CFO2/SFO enhancements are intentionally
% excluded.
decoded = struct();
decoded.payload = struct('bytes', uint8(frame.payload_bytes(:)), ...
    'fcs_pass', logical(frame.fcs_pass));
decoded.sfd = struct('name', frame.sfd_name);
decoded.cir = frame.cir;
tx_options = struct( ...
    'fs_tx', params.fs_rx, ...
    'phy_mode', '802.15.4a', ...
    'ranging', true, ...
    'preamble_repetitions', params.preamble_repetitions, ...
    'code_index', params.code_index, ...
    'sfd_number', 0, ...
    'sfd_sequence', [-1; -1; -1; -1; 1; -1; 0; 0], ...
    'peak_amplitude', 1, ...
    'guard_samples', 0, ...
    'require_fcs_pass', true);
tx = generate_uwb_tx_from_decode(decoded, tx_options);
channel = apply_estimated_cir_to_uwb(tx, decoded.cir);
replica = channel.waveform_x410(:);
assert(numel(replica) == numel(tx.pulse_impulses_work));

fit_options = struct( ...
    'search_radius', 128, ...
    'alignment_first_repetition_0based', 24, ...
    'alignment_repetitions', 32, ...
    'cfo_first_repetition_0based', 24, ...
    'cfo_last_repetition_exclusive', 128, ...
    'gain_first_repetition_0based', 24, ...
    'gain_last_repetition_exclusive', 128, ...
    'min_alignment_correlation', 0.70, ...
    'min_suppression_db', 0.20, ...
    'max_abs_cfo_hz', 100e3);
nominal_start_0based = frame.abs_start_sample - window_start_0based;
trial = fitIntegerTrial(rx, replica, nominal_start_0based, ...
    round(frame.samples_per_repetition), params.fs_rx, fit_options);
assert(trial.alignment_correlation >= fit_options.min_alignment_correlation);
assert(abs(trial.fitted_cfo_hz) <= fit_options.max_abs_cfo_hz);
assert(trial.suppression_db >= fit_options.min_suppression_db);

write_real_f32(fullfile(outdir, 'tx_pulse_impulses.f32'), ...
    tx.pulse_impulses_work);
write_complex_f32(fullfile(outdir, 'tx_cir_replica.cfile'), replica);
write_complex_f32(fullfile(outdir, 'trial_received.cfile'), trial.received);
write_complex_f32(fullfile(outdir, 'trial_model.cfile'), trial.model);
write_complex_f32(fullfile(outdir, 'trial_residual.cfile'), trial.residual);

fid_fields = fopen(fullfile(outdir, 'field_bounds.csv'), 'w');
assert(fid_fields >= 0);
fields_cleanup = onCleanup(@() fclose(fid_fields));
fprintf(fid_fields, 'field,begin_0based,end_exclusive_0based\n');
field_names = {'SYNC', 'SFD', 'PHR', 'Payload'};
for k = 1:numel(field_names)
    bounds = tx.field_indices_work.(field_names{k});
    fprintf(fid_fields, '%s,%d,%d\n', field_names{k}, ...
        bounds(1) - 1, bounds(2));
end
clear fields_cleanup;

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
fprintf(fid, 'golden_schema,dw1000-tx-replica-v1\n');
fprintf(fid, ['preprocessing_contract,', ...
    'cf32-998.4mhz-tone-removed-10mhz-shift-v1\n']);
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
fprintf(fid, 'tx_phy_mode,802.15.4a\n');
fprintf(fid, 'tx_ranging,1\n');
fprintf(fid, 'tx_sfd_number,0\n');
fprintf(fid, 'tx_pulse_impulse_count,%d\n', ...
    numel(tx.pulse_impulses_work));
fprintf(fid, 'tx_replica_sample_count,%d\n', numel(replica));
fprintf(fid, 'tx_cir_zero_delay_index_0based,%d\n', ...
    channel.zero_delay_index - 1);
fprintf(fid, 'trial_nominal_start_window_0based,%d\n', ...
    nominal_start_0based);
fprintf(fid, 'trial_fitted_start_window_0based,%d\n', ...
    trial.fitted_start_0based);
fprintf(fid, 'trial_sample_count,%d\n', numel(trial.received));
fprintf(fid, 'trial_search_radius,%d\n', fit_options.search_radius);
fprintf(fid, 'trial_alignment_first_repetition_0based,%d\n', ...
    fit_options.alignment_first_repetition_0based);
fprintf(fid, 'trial_alignment_repetitions,%d\n', ...
    fit_options.alignment_repetitions);
fprintf(fid, 'trial_alignment_correlation,%.12g\n', ...
    trial.alignment_correlation);
fprintf(fid, 'trial_cfo_first_repetition_0based,%d\n', ...
    fit_options.cfo_first_repetition_0based);
fprintf(fid, 'trial_cfo_last_repetition_exclusive,%d\n', ...
    fit_options.cfo_last_repetition_exclusive);
fprintf(fid, 'trial_fitted_cfo_hz,%.12g\n', trial.fitted_cfo_hz);
fprintf(fid, 'trial_gain_first_repetition_0based,%d\n', ...
    fit_options.gain_first_repetition_0based);
fprintf(fid, 'trial_gain_last_repetition_exclusive,%d\n', ...
    fit_options.gain_last_repetition_exclusive);
fprintf(fid, 'trial_global_gain_real,%.12g\n', real(trial.global_gain));
fprintf(fid, 'trial_global_gain_imag,%.12g\n', imag(trial.global_gain));
fprintf(fid, 'trial_power_before,%.12g\n', trial.power_before);
fprintf(fid, 'trial_power_after,%.12g\n', trial.power_after);
fprintf(fid, 'trial_suppression_db,%.12g\n', trial.suppression_db);
fprintf(fid, 'trial_min_alignment_correlation,%.12g\n', ...
    fit_options.min_alignment_correlation);
fprintf(fid, 'trial_min_suppression_db,%.12g\n', ...
    fit_options.min_suppression_db);
fprintf(fid, 'trial_max_abs_cfo_hz,%.12g\n', ...
    fit_options.max_abs_cfo_hz);

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

function write_real_f32(path, values)
    fid = fopen(path, 'wb');
    if fid < 0, error('Cannot open %s', path); end
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid, single(values(:)), 'single');
end

function trial = fitIntegerTrial(rx, replica, nominalStart0, period, fs, o)
    templateFirst = o.alignment_first_repetition_0based * period + 1;
    templateLast = min(numel(replica), templateFirst + ...
        o.alignment_repetitions * period - 1);
    template = replica(templateFirst:templateLast);
    candidates = nominalStart0 + (-o.search_radius:o.search_radius);
    scores = -inf(size(candidates));
    for k = 1:numel(candidates)
        first = candidates(k) + templateFirst;
        last = first + numel(template) - 1;
        if first < 1 || last > numel(rx), continue; end
        segment = rx(first:last);
        scores(k) = abs(template' * segment) / ...
            (norm(template) * norm(segment) + eps);
    end
    [correlation, best] = max(scores);
    fittedStart0 = candidates(best);
    available = min(numel(replica), numel(rx) - fittedStart0);
    replica = replica(1:available);
    received = rx(fittedStart0 + (1:available));

    firstRep = o.cfo_first_repetition_0based;
    lastRep = min(o.cfo_last_repetition_exclusive, ...
        floor(available / period));
    repetitions = firstRep:lastRep-1;
    correlations = complex(zeros(numel(repetitions), 1));
    times = zeros(numel(repetitions), 1);
    for q = 1:numel(repetitions)
        first = repetitions(q) * period + 1;
        last = min((repetitions(q) + 1) * period, available);
        idx = first:last;
        correlations(q) = replica(idx)' * received(idx);
        times(q) = ((first + last) / 2 - 1) / fs;
    end
    phase = unwrap(angle(correlations));
    lineFit = polyfit(times, phase, 1);
    cfoHz = lineFit(1) / (2 * pi);
    n = (0:available-1).';
    replicaCfo = replica .* exp(1j * 2 * pi * cfoHz * n / fs);

    gainFirst = o.gain_first_repetition_0based * period + 1;
    gainLast = min(o.gain_last_repetition_exclusive * period, available);
    gainIdx = gainFirst:gainLast;
    gain = (replicaCfo(gainIdx)' * received(gainIdx)) / ...
        (replicaCfo(gainIdx)' * replicaCfo(gainIdx) + eps);
    model = gain * replicaCfo;
    residual = received - model;
    powerBefore = mean(abs(received).^2);
    powerAfter = mean(abs(residual).^2);

    trial = struct( ...
        'fitted_start_0based', fittedStart0, ...
        'alignment_correlation', correlation, ...
        'fitted_cfo_hz', cfoHz, ...
        'global_gain', gain, ...
        'power_before', powerBefore, ...
        'power_after', powerAfter, ...
        'suppression_db', 10 * log10(powerBefore / (powerAfter + eps)), ...
        'received', received, 'model', model, 'residual', residual);
end
