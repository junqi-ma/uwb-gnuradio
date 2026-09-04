%RUN_EXPORT_RADAR_PACKETS  Write 32- and 128-SYNC complete packet goldens.
here = fileparts(mfilename('fullpath'));
export_uwb_radar_packet(fullfile(here, 'packets', 'sync32'), 32);
export_uwb_radar_packet(fullfile(here, 'packets', 'sync128'), 128);
fprintf('DONE packets 32/128\n');
