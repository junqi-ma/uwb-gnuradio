%% Run SYNC polarity-coded validation (alternating, notched, 64 SYNC).
% Compares RX-decoded mixes (q + g*c*d) against uncoded baseline.
% The Python generator must have been run first:
%
%   python3 testdata/generate_sync_polarity_mix.py \
%     --clean /mnt/f/.../qm35_clean_tone_removed.dat \
%     --interference /mnt/f/.../dw1000_clean_tone_removed.dat \
%     --geometry-json /mnt/f/.../qm35_clean_scheduled_sc16_dump/capture.jsonl \
%     --csv /mnt/f/.../qm35_clean_scheduled_sc16_dump/scheduled_dump_cpp.csv \
%     --output-dir /mnt/f/.../qm35_sync_polarity_notched_20260824 --write-dumps
%
% Then this script decodes each dump_gain* with the same 65/48 + decode_uwb
% chain as analyze_qm35_sc16_matlab.m.  MATLAB must have Communications
% Toolbox.  Run via:
%
%   matlab -batch "run_analyze_sync_polarity"
%
% Outputs per dump:
%   <dump>/scheduled_dump_matlab.csv + .mat  (from decode_scheduled_sc16_dump)
%   run-level summary: <dumpRoot>/sync_polarity_summary.csv
%
% The summary aggregates QM35 FCS rate, timing_metric and SFD correlation
% so the suppression gap between coded/decoded and uncoded mixes is visible
% without re-running the full batch.  Deeper CIR pre-path SIR can be added
% by calling run_search_uwb_periodic_cir-style per-slot CIR analysis on the
% same dumps (see §6 in UWB模拟域SYNC极性编码干扰抑制分析.md).

clear; close all; clc;
project_dir = fileparts(mfilename('fullpath'));
addpath(project_dir);
addpath(fullfile(project_dir, '..', 'testdata'));

% -------------------------------------------------------------------------
% Config
% -------------------------------------------------------------------------
dumpRoot = 'F:\UWB基带数据\qm35_sync_polarity_notched_20260824';
% Also compare against the uncoded notched mix and the clean dump
baselines = {
    'qm35_clean', 'F:\UWB基带数据\qm35_clean_scheduled_sc16_dump'
    'notched_gain1', 'F:\UWB基带数据\qm35_high_power_dw1000_mix_notched_20260817'  % whole-file .dat, not a dump
    };
gains = {'gain1','gain2','gain3'};
% For whole-file .dat baselines use analyze_qm35 path; for dump dirs use dump decoder
decode_dw1000 = false;  % set true to also score DW1000 head/tail (optional)

fprintf('=== SYNC polarity validation: dumpRoot=%s ===\n', dumpRoot);
records = {};

% 1) Decoded polarity dumps (the experiment)
for k = 1:numel(gains)
    g = gains{k};
    dumpDir = fullfile(dumpRoot, ['dump_' char(g)]);
    if ~isfolder(dumpDir)
        fprintf('[skip] missing %s\n', dumpDir);
        continue
    end
    fprintf('\n--- Decoding %s ---\n', dumpDir);
    opts = struct('decode_dw1000', decode_dw1000, 'output_dir', dumpDir, 'show_plots', false);
    results = decode_scheduled_sc16_dump(dumpDir, opts);
    summary = summarize_results(results, g, dumpDir, 'polarity_decoded');
    records{end+1} = summary; %#ok<AGROW>
end

% 2) Optional: clean dump for target-preservation baseline
cleanDump = 'F:\UWB基带数据\qm35_clean_scheduled_sc16_dump';
if isfolder(cleanDump)
    fprintf('\n--- Decoding clean baseline %s ---\n', cleanDump);
    results = decode_scheduled_sc16_dump(cleanDump, struct('output_dir', cleanDump, 'show_plots', false));
    records{end+1} = summarize_results(results, 'clean', cleanDump, 'baseline'); %#ok<AGROW>
end

% 3) Write aggregate CSV
if ~isempty(records)
    T = vertcat(records{:});
    outCsv = fullfile(dumpRoot, 'sync_polarity_summary.csv');
    writetable(T, outCsv);
    save(fullfile(dumpRoot, 'sync_polarity_summary.mat'), 'T', '-v7');
    fprintf('\nWrote %s (%d rows)\n', outCsv, height(T));
    disp(T(:, {'gain','dataset','n_windows','qm35_fcs_pass','qm35_fcs_rate','qm35_timing_median','qm35_sfd_median'}));
else
    fprintf('No records collected - check dumpRoot\n');
end

% -------------------------------------------------------------------------
function S = summarize_results(results, gain, dumpDir, tag)
    n = numel(results);
    fcs = sum([results.qm35_fcs_pass]);
    S = table({char(gain)}, {char(tag)}, {char(dumpDir)}, n, fcs, fcs/max(n,1), ...
        median([results.qm35_timing_metric]), median([results.qm35_sfd_correlation]), ...
        'VariableNames', {'gain','dataset','dumpDir','n_windows','qm35_fcs_pass','qm35_fcs_rate','qm35_timing_median','qm35_sfd_median'});
end
