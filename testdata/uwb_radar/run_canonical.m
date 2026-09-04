% One-shot canonical export + hard-gate verify (Windows or POSIX MATLAB).
here = fileparts(mfilename('fullpath'));
cd(here);
fprintf('canonical dir: %s\n', here);
meta = export_uwb_radar_golden(here);
disp(meta.generator);
ok = verify_uwb_radar_golden(here);
if ~ok
    error('run_canonical:VerifyFailed', 'verify_uwb_radar_golden returned false');
end
