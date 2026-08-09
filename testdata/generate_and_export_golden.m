%% Generate a clean 64-SYNC code-9 signal and decode it immediately.
%
% Avoids the energy-gate failure on long-silence files by generating the
% signal in-memory and decoding it right away.  Uses standard IEEE SFD
% (lrwpanWaveformGenerator default) and sfd_mode='auto' so the decoder picks
% the matching SFD template.
%
% Outputs golden vectors to C:\Users\junqima\AppData\Local\Temp\uwb_golden\

clear; close all; clc;
addpath('C:\Users\junqima\AppData\Local\Temp\uwb_golden\UWB_demodulation');
addpath('C:\Users\junqima\AppData\Local\Temp\uwb_golden\UWB_demodation\helpers');

fs = 998.4e6;
mean_prf = 62.4;
data_rate = 6.81;
code_index = 9;
preamble_symbols = 64;
samples_per_pulse = 2;
psdu_bytes = 127;
peak_amp = 0.8;

outdir = 'C:\Users\junqima\AppData\Local\Temp\uwb_golden';
if ~exist(outdir, 'dir'), mkdir(outdir); end

%% Build PHY config and generate packet
cfg = lrwpanHRPConfig(Mode='802.15.4a', MeanPRF=mean_prf, ...
    DataRate=data_rate, PreambleDuration=preamble_symbols, ...
    CodeIndex=code_index, SamplesPerPulse=samples_per_pulse, ...
    PSDULength=psdu_bytes);
fprintf('cfg SampleRate=%.3f MHz\n', cfg.SampleRate/1e6);

% Deterministic payload (125 data + 2 FCS = 127)
rng(20260809, 'twister');
data_bytes = uint8(randi([0 255], psdu_bytes-2, 1));
fcs = uwbdecoder.ieee802154CRC16(data_bytes);
fcs_bytes = uint8([bitand(fcs, 255); bitshift(fcs, -8)]);
payload_bytes = [data_bytes; fcs_bytes];
payload_bits = zeros(psdu_bytes*8, 1);
for i = 1:numel(payload_bits)
    payload_bits(i) = bitand(bitshift(uint16(payload_bytes(floor((i-1)/8)+1)), -(mod(i-1,8))), 1);
end

[packet, ~] = lrwpanWaveformGenerator(payload_bits, cfg);
packet = complex(packet(:), zeros(numel(packet), 1));
pk = max(abs(packet));
packet = packet * (peak_amp / pk);

% Leading + trailing silence so soft chips fully cover payload budget
pre_samples = round(10e-6 * fs);   % 10 us leading
post_samples = round(60e-6 * fs);  % 60 us trailing guard
iq = complex(zeros(pre_samples + numel(packet) + post_samples, 1, 'single'));
iq(pre_samples+1:pre_samples+numel(packet)) = single(packet);
t0_0based = pre_samples;

fprintf('Generated packet: %d samples, total %d samples, t0=%d (0-based)\n', ...
    numel(packet), numel(iq), t0_0based);

%% Decode immediately (clean signal, energy gate works)
o = struct();
o.fs_rx = fs; o.code_index = code_index; o.data_rate = data_rate;
o.preamble_repetitions = preamble_symbols; o.sfd_mode = 'auto';
o.cir_skip_initial_repetitions = 10;
o.cir_repetitions = preamble_symbols - o.cir_skip_initial_repetitions;
o.max_psdu_bytes = 127; o.enable_frame_crop = true;
o.verbose = false; o.show_plots = false;
o.ant_num = 1; o.channel_index = 1;

result = decode_uwb(o, iq, struct(), [], 'single');
fprintf('\n=== DECODE RESULT ===\n');
fprintf('preamble start=%d (1-based) period=%.4f peaks=%d metric=%.4f\n', ...
    result.preamble.start_sample, result.preamble.samples_per_repetition, ...
    result.preamble.detected_repetitions, result.preamble.metric_peak);
fprintf('CFO=%.3f Hz\n', result.preamble.carrier_frequency_offset_hz);
fprintf('SFD start_chip=%d end_chip=%d polarity=%d correlation=%.4f\n', ...
    result.sfd.start_chip, result.sfd.end_chip, result.sfd.polarity, result.sfd.correlation);
fprintf('PHR psdu_length=%d secded_pass=%d\n', ...
    result.phr.psdu_length_bytes, result.phr.secded_pass);
fprintf('payload: %d bytes fcs_pass=%d recv=%04x calc=%04x\n', ...
    numel(result.payload.bytes), result.payload.fcs_pass, ...
    result.payload.fcs_received, result.payload.fcs_calculated);

%% Export golden vectors
write_iq_cfile(fullfile(outdir, 'window.cfile'), iq);

% timing — ABSOLUTE (full/window) coordinates, 0-based.  The detector's
% start_sample / peak_indices are cropped; shift by crop_start so the C++
% core (which works in absolute coordinates, matching PDU metadata) can
% compare directly.  crop_start_1based=6936, start_uncropped_1based=9985.
crop0 = result.preamble.crop_start_sample - 1; % 0-based crop offset
timing.start_0based = result.preamble.start_sample_uncropped - 1; % absolute
timing.period = result.preamble.samples_per_repetition;
timing.metric = result.preamble.metric_peak;
timing.peaks_0based = (result.preamble.peak_indices(:) - 1) + crop0; % absolute
timing.detected_repetitions = result.preamble.detected_repetitions;
timing.crop_start_0based = crop0;
save(fullfile(outdir, 'stage_timing.mat'), '-struct', 'timing');

% CFO
cfo.hz = result.preamble.carrier_frequency_offset_hz;
save(fullfile(outdir, 'stage_cfo.mat'), '-struct', 'cfo');

% SFD (chip coordinates)
sfd.start_chip = result.sfd.start_chip - 1; % 0-based
sfd.end_chip = result.sfd.end_chip - 1;
sfd.polarity = result.sfd.polarity;
sfd.correlation = result.sfd.correlation;
save(fullfile(outdir, 'stage_sfd.mat'), '-struct', 'sfd');

% CIR
cir.values = result.cir.values;
cir.pre_samples = result.cir.pre_samples;
cir.post_samples = result.cir.post_samples;
cir.first_repetition = result.cir.first_repetition;
cir.last_repetition = result.cir.last_repetition;
cir.timing = result.cir.timing;
save(fullfile(outdir, 'stage_cir.mat'), '-struct', 'cir');

% soft chips
softchips.stream = result.soft_chips;
softchips.samples_per_chip = result.soft_chip_timing.samples_per_chip;
softchips.first_chip_0based = result.soft_chip_timing.first_chip_sample_uncropped - 1;
softchips.num_chips = result.soft_chip_timing.num_chips;
save(fullfile(outdir, 'stage_softchips.mat'), '-struct', 'softchips');

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

% manifest
fid = fopen(fullfile(outdir, 'manifest.csv'), 'w');
fprintf(fid, 'key,value\n');
fprintf(fid, 'fs_rx,%g\n', fs);
fprintf(fid, 'code_index,%d\n', code_index);
fprintf(fid, 'preamble_repetitions,%d\n', preamble_symbols);
fprintf(fid, 'sfd_mode,%s\n', 'ieee-legacy');
fprintf(fid, 'data_rate,%g\n', data_rate);
fprintf(fid, 't0_0based,%d\n', t0_0based);
fprintf(fid, 'packet_samples,%d\n', numel(packet));
fprintf(fid, 'preamble_start_0based,%d\n', timing.start_0based);
fprintf(fid, 'preamble_period,%.6f\n', timing.period);
fprintf(fid, 'preamble_metric,%.6f\n', timing.metric);
fprintf(fid, 'detected_peaks,%d\n', timing.detected_repetitions);
fprintf(fid, 'cfo_hz,%.4f\n', cfo.hz);
fprintf(fid, 'sfd_start_chip,%d\n', sfd.start_chip);
fprintf(fid, 'sfd_end_chip,%d\n', sfd.end_chip);
fprintf(fid, 'sfd_polarity,%d\n', sfd.polarity);
fprintf(fid, 'sfd_correlation,%.6f\n', sfd.correlation);
fprintf(fid, 'psdu_length,%d\n', phr.psdu_length);
fprintf(fid, 'secded_pass,%d\n', phr.secded_pass);
fprintf(fid, 'payload_bytes,%d\n', numel(payload.bytes));
fprintf(fid, 'fcs_pass,%d\n', payload.fcs_pass);
fprintf(fid, 'fcs_received,0x%04x\n', payload.fcs_received);
fprintf(fid, 'fcs_calculated,0x%04x\n', payload.fcs_calculated);
fclose(fid);

fprintf('\nGolden vectors written to %s\n', outdir);

%% helpers
function write_iq_cfile(path, x)
    x = x(:);
    inter = zeros(2*numel(x), 1, 'single');
    inter(1:2:end) = real(x);
    inter(2:2:end) = imag(x);
    fid2 = fopen(path, 'wb');
    if fid2 < 0, error('cannot open %s', path); end
    fwrite(fid2, inter, 'float32');
    fclose(fid2);
end
