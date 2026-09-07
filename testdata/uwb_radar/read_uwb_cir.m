function [cir, meta] = read_uwb_cir(outDir, pulseId, wantNorm)
%READ_UWB_CIR Read one pulse's CIR from a UwbCirWriter output directory.
%   [CIR, META] = READ_UWB_CIR(OUTDIR, PULSEID) reads the CIR frame whose
%   pulse_id == PULSEID from OUTDIR/cir.cf32 + OUTDIR/cir.jsonl.
%
%   [CIR, META] = READ_UWB_CIR(OUTDIR, PULSEID, TRUE) instead reads the
%   L2-normalized CIR from OUTDIR/cir_norm.cf32 (UwbCirWriter must have been
%   started with write_normalized=true).
%
%   CIR is tap_count x 1 complex single (raw CIR: divided by code_energy,
%   NOT L2-normalized).  For failed frames (sfd_failed / timing_failed /
%   cir_failed / length-mismatch frames) tap_count is 0 and CIR is empty;
%   the metadata line is still returned so failures stay observable.
%
%   META is the cir.jsonl line for the pulse as a struct, including
%   file_offset_taps, tap_count, status, zero_delay_tap, peak_tap and
%   range_m_per_tap.  With an empty PULSEID, META returns all lines as a
%   struct array and CIR is empty.
%
%   Files (UwbCirWriter contract):
%     cir.cf32      concatenated raw complex CIR taps (complex64 LE, 8 B/tap)
%     cir_norm.cf32 concatenated normalized taps (optional)
%     cir.jsonl     one JSON object per pulse
%     run.json      static run configuration
%
%   Example:
%     [cir, meta] = read_uwb_cir('cir_out', 3);
%     [nrm, meta] = read_uwb_cir('cir_out', 3, true);

    if nargin < 2
        error('read_uwb_cir:usage', ...
              'usage: [cir,meta]=read_uwb_cir(outDir,pulseId[,wantNorm])');
    end
    if nargin < 3
        wantNorm = false;
    end

    jsonlFile = fullfile(outDir, 'cir.jsonl');
    metaAll = readAllJsonl(jsonlFile);

    if isempty(pulseId)
        cir = [];
        meta = metaAll;
        return;
    end

    idx = find([metaAll.pulse_id] == pulseId, 1);
    if isempty(idx)
        error('read_uwb_cir:noPulse', ...
              'pulse %d not found in %s', pulseId, jsonlFile);
    end
    meta = metaAll(idx);

    if ~isfield(meta, 'tap_count') || isempty(meta.tap_count)
        error('read_uwb_cir:badMeta', ...
              'cir.jsonl line for pulse %d has no tap_count', pulseId);
    end
    if meta.tap_count == 0
        cir = [];
        return;
    end

    if wantNorm
        binFile = fullfile(outDir, 'cir_norm.cf32');
        if ~isfield(meta, 'file_offset_norm_taps') || ...
                isempty(meta.file_offset_norm_taps)
            error('read_uwb_cir:noNorm', ...
                  'run has no file_offset_norm_taps; was cir_norm written?');
        end
        offset = meta.file_offset_norm_taps;
    else
        binFile = fullfile(outDir, 'cir.cf32');
        if ~isfield(meta, 'file_offset_taps') || ...
                isempty(meta.file_offset_taps)
            error('read_uwb_cir:badMeta', ...
                  'cir.jsonl line for pulse %d has no file_offset_taps', ...
                  pulseId);
        end
        offset = meta.file_offset_taps;
    end

    fid = fopen(binFile, 'rb', 'ieee-le');
    if fid < 0
        error('read_uwb_cir:open', 'cannot open %s', binFile);
    end
    fseek(fid, offset * 8, 'bof');
    % 'float32=>single': R2025b fread returns double unless the output
    % class is stated explicitly; the CIR contract is complex single.
    raw = fread(fid, meta.tap_count * 2, 'float32=>single');
    fclose(fid);
    if numel(raw) < 2 * meta.tap_count
        error('read_uwb_cir:short', ...
              '%s truncated: expected %d taps at offset %d', ...
              binFile, meta.tap_count, offset);
    end
    cir = raw(1:2:end) + 1i * raw(2:2:end);
    cir = cir(:);
end

function meta = readAllJsonl(jsonlFile)
%READALLJSONL Parse a cir.jsonl file into a struct array.
%   Lines are decoded into a cell array first and missing fields are
%   back-filled with [] so that a plain struct array can always be built
%   (a struct() cannot be extended by "s(k) = jsondecode(line)").
    fid = fopen(jsonlFile, 'r');
    if fid < 0
        error('read_uwb_cir:open', 'cannot open %s', jsonlFile);
    end
    rows = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line))
            rows{end+1, 1} = jsondecode(line); %#ok<AGROW>
        end
    end
    fclose(fid);

    if isempty(rows)
        meta = struct('pulse_id', {}, 'schedule_index', {}, ...
                      'status', {}, 'tap_count', {}, ...
                      'file_offset_taps', {});
        return;
    end

    % Union of all fields; missing fields become [].
    fields = {};
    for i = 1:numel(rows)
        fields = union(fields, fieldnames(rows{i}));
    end
    for i = 1:numel(rows)
        for f = 1:numel(fields)
            if ~isfield(rows{i}, fields{f})
                rows{i}.(fields{f}) = [];
            end
        end
    end
    meta = [rows{:}]; %#ok<AGROW>
    meta = meta(:);
end
