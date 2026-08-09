%% Export the production DW1000 PHY constants used by Phase-2 SIC.
%
% Requires MATLAB Communications Toolbox.  The output is intentionally small
% and text-only so C++ QA can verify it without a MATLAB runtime.

clear; clc;

testdata_dir = fileparts(mfilename('fullpath'));
outdir = fullfile(testdata_dir, 'sic_profile_golden');
if ~exist(outdir, 'dir'), mkdir(outdir); end

code_index = 11;
preamble_repetitions = 128;
sfd_mode = 'decawave';
sfd = int8([-1; -1; -1; -1; 1; -1; 0; 0]);
code = int8(lrwpan.internal.HRPCodes(code_index));

assert(numel(code) == 127);
assert(nnz(code) == 64);
assert(sum(double(code).^2) == 64);

spreading_factor = 4;
samples_per_pulse = 2;
spread = zeros(numel(code) * spreading_factor, 1, 'int8');
spread(1:spreading_factor:end) = code(:);
sampled = zeros(numel(spread) * samples_per_pulse, 1, 'int8');
sampled(1:samples_per_pulse:end) = spread;

writematrix(code(:).', fullfile(outdir, 'dw1000_code11.csv'));
writematrix(sampled(:).', fullfile(outdir, 'dw1000_sampled_code11.csv'));
writematrix(sfd(:).', fullfile(outdir, 'dw1000_sfd_decawave.csv'));
for scrambler_code = [9, 10, 11]
    pn = lrwpan.internal.createScrambler(scrambler_code, 64, 0);
    prefix = int8(pn());
    writematrix(prefix(:).', fullfile(outdir, sprintf( ...
        'bprf_scrambler_code%d.csv', scrambler_code)));
end

fid = fopen(fullfile(outdir, 'manifest.csv'), 'w');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'key,value\n');
fprintf(fid, 'profile,dw1000-production\n');
fprintf(fid, 'generator,lrwpan.internal.HRPCodes\n');
fprintf(fid, 'matlab_release,%s\n', version('-release'));
fprintf(fid, 'sample_rate,998400000\n');
fprintf(fid, 'mean_prf_mhz,62.4\n');
fprintf(fid, 'code_index,%d\n', code_index);
fprintf(fid, 'preamble_repetitions,%d\n', preamble_repetitions);
fprintf(fid, 'sfd_mode,%s\n', sfd_mode);
fprintf(fid, 'data_rate_mbps,6.81\n');
fprintf(fid, 'code_length,%d\n', numel(code));
fprintf(fid, 'code_nonzero,%d\n', nnz(code));
fprintf(fid, 'code_energy,%g\n', sum(double(code).^2));
fprintf(fid, 'spreading_factor,%d\n', spreading_factor);
fprintf(fid, 'samples_per_pulse,%d\n', samples_per_pulse);
fprintf(fid, 'samples_per_symbol,%d\n', numel(sampled));

fprintf('SIC production profile golden written to %s\n', outdir);
