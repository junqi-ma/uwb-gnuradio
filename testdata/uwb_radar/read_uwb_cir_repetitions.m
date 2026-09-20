function [cirMatrix, meta] = read_uwb_cir_repetitions(outDir, pulseId, wantNorm)
%READ_UWB_CIR_REPETITIONS Read every landed repetition CIR for one pulse.
%   [X,META] = READ_UWB_CIR_REPETITIONS(OUTDIR,PULSEID) returns a
%   tap_count-by-repetition_count complex-single matrix ordered by
%   repetition_ordinal. META is the matching expanded metadata struct array;
%   on disk, all repetitions for the pulse occupy one compact JSONL line.
%
%   Pass WANTNORM=true to read cir_norm.cf32 instead of cir.cf32.

    if nargin < 3
        wantNorm = false;
    end
    [~, allMeta] = read_uwb_cir(outDir, []);
    keep = [allMeta.pulse_id] == pulseId;
    meta = allMeta(keep);
    if isempty(meta)
        error('read_uwb_cir_repetitions:noPulse', ...
              'pulse %d not found in %s', pulseId, outDir);
    end
    if ~isfield(meta, 'repetition_index') || ...
            any(arrayfun(@(x) isempty(x.repetition_index), meta))
        error('read_uwb_cir_repetitions:notIndividual', ...
              'pulse %d does not contain per-repetition CIR records', pulseId);
    end
    if isfield(meta, 'repetition_ordinal')
        [~, order] = sort([meta.repetition_ordinal]);
    else
        [~, order] = sort([meta.repetition_index]);
    end
    meta = meta(order);
    tapCounts = [meta.tap_count];
    if any(tapCounts == 0) || any(tapCounts ~= tapCounts(1))
        error('read_uwb_cir_repetitions:failedRecord', ...
              'pulse %d has failed or inconsistent CIR records', pulseId);
    end
    cirMatrix = complex(zeros(tapCounts(1), numel(meta), 'single'));
    for k = 1:numel(meta)
        cirMatrix(:, k) = read_uwb_cir(
            outDir, pulseId, wantNorm, meta(k).repetition_index);
    end
end
