%RUN_EXPORT_RADAR_PACKETS  Write SYNC-length packet goldens (default PSDU=0).
here = fileparts(mfilename('fullpath'));
ns = [32 64 128 256 512 1024 2048];
for n = ns
    out = fullfile(here, 'packets', sprintf('sync%d_psdu0', n));
    export_uwb_radar_packet(out, n, 'PSDUBytes', 0);
end
% Keep the historical 32/128 20-byte goldens used by e2e_sync_repetitions.
export_uwb_radar_packet(fullfile(here, 'packets', 'sync32'), 32, ...
    'PSDUBytes', 20, 'RandomSeed', 20260904);
export_uwb_radar_packet(fullfile(here, 'packets', 'sync128'), 128, ...
    'PSDUBytes', 20, 'RandomSeed', 20260904);
fprintf('DONE radar packets\n');
