% export_payload_geometry.m  — S2-B reference export (STATUS: 未实跑 / NOT RUN)
%
% Emits the frozen S2-B communication-packet geometry and a code-10 SYNC /
% full-packet reference from the shared SRD/MATLAB toolbox, for independent
% cross-check of gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h.
%
% This script was NOT executed for the S2-B delivery.  It is provided for the
% analyst; do not cite its output as validated until it has actually run.

clear; clc;
here = fileparts(mfilename('fullpath'));
outdir = fullfile(here, 'out');
if ~exist(outdir, 'dir'); mkdir(outdir); end

% Frozen profile (must match docs/sic/S2B_设计与测试契约.md).
codeIndex    = 10;       % desired communication packet
syncReps     = 64;
sfdMode      = '4z2';
psduBytes    = 96;       % wire PSDU incl. 2-byte FCS
dataRate     = 6.81;     % Mb/s payload
samplesPerPulse = 2;     % 998.4 MS/s work grid
meanPrfMHz   = 62.4;

cfg = lrwpanHRPConfig('Mode', '802.15.4a', 'MeanPRF', meanPrfMHz, ...
    'DataRate', dataRate, 'PreambleDuration', syncReps, ...
    'CodeIndex', codeIndex, 'SamplesPerPulse', samplesPerPulse, ...
    'PSDULength', psduBytes);
cfg.SFD = sfdMode;

psdu = uint8(mod((0:psduBytes-3).' * 37 + 11, 256));    % deterministic
fcs  = uwbdecoder.ieee802154CRC16(psdu);                 % single FCS
psduWire = [psdu; uint8(mod(double(fcs), 256)); ...
            uint8(floor(double(fcs)/256))];
assert(numel(psduWire) == psduBytes);

wave = lrwpanWaveformGenerator(psduWire, cfg);

% One shaped SYNC symbol = samplesPerSymbol samples, matching the modulator.
samplesPerSymbol = 127 * 4 * samplesPerPulse;            % 1016
sync = wave(1:samplesPerSymbol);

write_cf32(fullfile(outdir, 'comm_sync_code10_998p4.cf32'), sync);
write_cf32(fullfile(outdir, 'comm_packet_code10_psdu96.cf32'), wave);

sfdSyms   = numel(cfg.SFDSequence);
phrSyms   = 21;
payloadSyms = psduBytes * 8 + 48 * ceil(psduBytes * 8 / 330);
offsets = struct( ...
    'sync_lo', 0, ...
    'sync_hi', syncReps * samplesPerSymbol, ...
    'sfd_hi',  (syncReps + sfdSyms) * samplesPerSymbol, ...
    'phr_hi',  (syncReps + sfdSyms) * samplesPerSymbol + phrSyms * 512 * samplesPerPulse, ...
    'payload_hi', (syncReps + sfdSyms) * samplesPerSymbol + phrSyms * 512 * samplesPerPulse ...
                  + payloadSyms * 64 * samplesPerPulse, ...
    'fcs_bytes', 2, 'psdu_bytes', psduBytes, 'fcs_crc', double(fcs));
fid = fopen(fullfile(outdir, 'payload_field_offsets.json'), 'w');
fprintf(fid, '%s\n', jsonencode(offsets));
fclose(fid);

fprintf('S2-B geometry written to %s (UNVALIDATED — script not run for delivery)\n', outdir);

function write_cf32(path, x)
    fid = fopen(path, 'wb');
    fwrite(fid, [real(x(:)).'; imag(x(:)).'], 'single');
    fclose(fid);
end
