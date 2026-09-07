function verify_e2e_cir(writerOutDir, nFrames, groupA, groupB, expectedGainRatio, expectFailedPids, refDir)
%VERIFY_E2E_CIR Verify a UwbCirWriter output directory from the Radar
%Phase-A offline end-to-end QA (gr-uwb/lib/qa_uwb_radar_e2e.cc).
%   VERIFY_E2E_CIR(WRITEROUTDIR, NFRAMES) verifies EVERY frame of the run
%   (packet source -> loopback [-> 65/48] -> estimator -> writer):
%
%     1. cir.jsonl has exactly NFRAMES lines, pulse_id == 0..NFRAMES-1
%        in order (no missing / reordered frames).
%     2. Every ok frame has tap_count > 0 and
%        tap_count == cir_pre_samples + cir_post_samples (when the line
%        carries those fields); every failed frame has tap_count == 0.
%     3. file_offset_taps advances only on ok frames and matches the
%        running sum; cir.cf32 size == 8 * total taps (complex64 LE).
%     4. If cir_norm.cf32 exists, every ok frame's normalized CIR has
%        L2 norm ~= 1 (within 1e-3) and the file size matches.
%     5. Raw CIR magnitude linearity: the ratio of the raw CIR L2 norms
%        of GROUPA and GROUPB equals EXPECTEDGAINRATIO (within 1e-3
%        relative).  With the optional 7th argument REFDIR, GROUPA ids
%        are read from WRITEROUTDIR and GROUPB ids from REFDIR
%        (cross-directory comparison, e.g. loopback gain 2.0 vs 0.5).
%     6. Failed frames have an empty CIR but a present line; when
%        EXPECTFAILEDPIDS is given the failed pulse ids must match it
%        exactly (all other frames ok).
%     7. Unified coordinate convention (Step 9): for ok frames
%        |sfd_start_sample - predicted_sfd_start_sample| <= 2 and
%        |preamble_start_sample - cir_origin_sample| <= 2 (no systematic
%        65/48 FIR-head bias), zero_delay_tap == cir_pre_samples and
%        zero_delay_tap <= peak_tap <= tap_count.
%
%   Performance: the JSONL is parsed ONCE (bulk read) and both binary
%   files are opened once; per-frame CIR taps are then read by
%   (offset, tap_count) with fseek/fread.  Total cost is O(N) decodes,
%   so the full 6000-frame soak directory verifies in seconds.
%
%   All metadata vectors extracted with [metaAll.x] are explicitly
%   columnized with (:) — concatenation yields row vectors, and mixing
%   them with column masks (okMask) previously triggered implicit
%   expansion in the PASS 7 find() expressions.
%
%   Usage (MATLAB R2025b via WSL interop, UNC paths):
%     addpath('\\wsl.localhost\Ubuntu-22.04\home\junqima\workspace\uwb-gnuradio\testdata\uwb_radar');
%     % 6000-frame soak run (all ok):
%     verify_e2e_cir('\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_radar_e2e_soak', 6000);
%     % missing-SFD run (pulse 1 is sfd_failed):
%     verify_e2e_cir('\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_radar_e2e_missfd', 2, [], [], [], 1);
%     % cross-directory amplitude linearity (gain 2.0 vs gain 0.5):
%     verify_e2e_cir('\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_radar_e2e_gain20', 1, 0, 0, 4.0, [], ...
%                    '\\wsl.localhost\Ubuntu-22.04\tmp\uwb_qa_radar_e2e_gain05');

    if nargin < 2
        error('verify_e2e_cir:usage', ...
              'usage: verify_e2e_cir(writerOutDir, nFrames [, groupA, groupB, expectedGainRatio, expectFailedPids, refDir])');
    end
    if nargin < 3
        groupA = [];
    end
    if nargin < 4
        groupB = [];
    end
    if nargin < 5
        expectedGainRatio = [];
    end
    if nargin < 6
        expectFailedPids = [];
    end
    if nargin < 7
        refDir = '';
    end

    % ---- one single JSONL parse for the whole run (O(N) overall) --------
    [~, metaAll] = read_uwb_cir(writerOutDir, []);

    % ---- 1) one line per frame, in pulse order --------------------------
    assert(numel(metaAll) == nFrames, ...
           'cir.jsonl must have %d lines, got %d', nFrames, numel(metaAll));
    % [metaAll.x] concatenates to a ROW vector; force column orientation
    % for every metadata vector used in vectorized expressions.
    pid = double([metaAll.pulse_id]);
    pid = pid(:);
    assert(isequal(pid(:), (0:(nFrames - 1))'), ...
           'pulse_id order mismatch (frames missing or reordered)');
    fprintf('PASS 1: %d JSONL lines, pulse_id 0..%d in order\n', ...
            nFrames, nFrames - 1);

    status = cellstr(string({metaAll.status}));
    tapCount = nan(nFrames, 1);
    if isfield(metaAll, 'tap_count')
        tapCount = double([metaAll.tap_count]);
        tapCount = tapCount(:);
    end
    okMask = strcmp(status, 'ok');
    okMask = okMask(:);
    failed = find(~okMask);

    % ---- 6) failed-frame set --------------------------------------------
    if ~isempty(expectFailedPids)
        assert(isequal(sort(double(expectFailedPids(:))), ...
                       sort(pid(failed))), ...
               'failed pulse ids %s do not match the expected set %s', ...
               mat2str(pid(failed)'), mat2str(expectFailedPids(:)'));
    end
    fprintf('PASS 6: %d failed frames (%s), others ok\n', numel(failed), ...
            strjoin(unique(status(failed)), ','));

    % ---- 2) per-frame metadata contract ----------------------------------
    running = 0;
    for k = 1:nFrames
        before = running;
        if okMask(k)
            assert(~isnan(tapCount(k)) && tapCount(k) > 0, ...
                   'pulse %d: ok frame must have tap_count > 0', pid(k));
            if isfield(metaAll, 'cir_pre_samples') && ...
                    ~isempty(metaAll(k).cir_pre_samples) && ...
                    isfield(metaAll, 'cir_post_samples') && ...
                    ~isempty(metaAll(k).cir_post_samples)
                assert(tapCount(k) == ...
                       metaAll(k).cir_pre_samples + metaAll(k).cir_post_samples, ...
                       'pulse %d: tap_count != pre + post', pid(k));
            end
            running = running + tapCount(k);
        else
            assert(~isnan(tapCount(k)) && tapCount(k) == 0, ...
                   'pulse %d: failed frame must have tap_count == 0', pid(k));
        end
        % file_offset_taps is the offset *before* this frame's write.
        assert(metaAll(k).file_offset_taps == before, ...
               'pulse %d: file_offset_taps %d != %d', ...
               pid(k), metaAll(k).file_offset_taps, before);
    end
    fprintf('PASS 2: tap_count contract and running offsets for all %d frames\n', ...
            nFrames);

    % ---- 3) raw binary: one open, per-frame offset reads (O(N)) ---------
    rawFile = fullfile(writerOutDir, 'cir.cf32');
    rawInfo = dir(rawFile);
    assert(numel(rawInfo) == 1, 'cir.cf32 missing in %s', writerOutDir);
    assert(rawInfo.bytes == 8 * running, ...
           'cir.cf32 size %d != 8 * %d taps', rawInfo.bytes, running);
    rawAll = cell(nFrames, 1);
    fid = fopen(rawFile, 'rb', 'ieee-le');
    assert(fid > 0, 'cannot open %s', rawFile);
    cleanupRaw = onCleanup(@() fclose(fid));
    for k = 1:nFrames
        if tapCount(k) == 0
            continue;
        end
        fseek(fid, double(metaAll(k).file_offset_taps) * 8, 'bof');
        buf = fread(fid, 2 * tapCount(k), 'float32=>single');
        assert(numel(buf) == 2 * tapCount(k), ...
               'pulse %d: cir.cf32 truncated at offset %d', ...
               pid(k), metaAll(k).file_offset_taps);
        rawAll{k} = buf(1:2:end) + 1i * buf(2:2:end);
    end
    fprintf('PASS 3: cir.cf32 size and per-frame offsets read back (%d taps)\n', ...
            running);

    % ---- 7) unified coordinate convention --------------------------------
    hasCoords = isfield(metaAll, 'sfd_start_sample') && ...
                isfield(metaAll, 'predicted_sfd_start_sample') && ...
                isfield(metaAll, 'preamble_start_sample') && ...
                isfield(metaAll, 'cir_origin_sample');
    if hasCoords
        % Row-vector concatenation, forced to columns (see header note).
        sfdV = double([metaAll.sfd_start_sample]);
        sfdV = sfdV(:);
        predV = double([metaAll.predicted_sfd_start_sample]);
        predV = predV(:);
        preV = double([metaAll.preamble_start_sample]);
        preV = preV(:);
        orgV = double([metaAll.cir_origin_sample]);
        orgV = orgV(:);
        bad = find(okMask & (abs(sfdV - predV) > 2), 1);
        assert(isempty(bad), ...
               'pulse %d: |sfd - predicted| = %d > 2 (65/48 head bias?)', ...
               pid(bad), abs(sfdV(bad) - predV(bad)));
        bad = find(okMask & (abs(preV - orgV) > 2), 1);
        assert(isempty(bad), ...
               'pulse %d: |preamble - cir_origin| = %d > 2 (65/48 head bias?)', ...
               pid(bad), abs(preV(bad) - orgV(bad)));
        if isfield(metaAll, 'zero_delay_tap')
            % Extract scalar columns row-wise (jsondecode rows are flat;
            % [] backfill becomes NaN) so no per-row isempty is needed.
            zeroV = nan(nFrames, 1);
            preS = nan(nFrames, 1);
            peakV = nan(nFrames, 1);
            for k = 1:nFrames
                if ~isempty(metaAll(k).zero_delay_tap)
                    zeroV(k) = double(metaAll(k).zero_delay_tap);
                end
                if isfield(metaAll, 'cir_pre_samples') && ...
                        ~isempty(metaAll(k).cir_pre_samples)
                    preS(k) = double(metaAll(k).cir_pre_samples);
                end
                if isfield(metaAll, 'peak_tap') && ...
                        ~isempty(metaAll(k).peak_tap)
                    peakV(k) = double(metaAll(k).peak_tap);
                end
            end
            bad = find(okMask & preS == preS & zeroV ~= preS, 1);
            assert(isempty(bad), ...
                   'pulse %d: zero_delay_tap %d must be cir_pre %d', ...
                   pid(bad), zeroV(bad), preS(bad));
            bad = find(okMask & peakV == peakV & ...
                       (peakV < zeroV | peakV > tapCount), 1);
            assert(isempty(bad), ...
                   'pulse %d: peak_tap %d outside [zero_delay %d, tap_count %d]', ...
                   pid(bad), peakV(bad), zeroV(bad), tapCount(bad));
        end
        fprintf('PASS 7: |sfd-predicted|<=2, |preamble-cir_origin|<=2, zero_delay==cir_pre (all ok frames)\n');
    else
        fprintf('SKIP 7: no coordinate fields in cir.jsonl\n');
    end

    % ---- 4) normalized CIR: one open, per-frame offset reads -------------
    normFile = fullfile(writerOutDir, 'cir_norm.cf32');
    if exist(normFile, 'file')
        normInfo = dir(normFile);
        assert(normInfo.bytes == 8 * running, ...
               'cir_norm.cf32 size %d != 8 * %d taps', normInfo.bytes, ...
               running);
        nfid = fopen(normFile, 'rb', 'ieee-le');
        assert(nfid > 0, 'cannot open %s', normFile);
        cleanupNorm = onCleanup(@() fclose(nfid));
        nOk = 0;
        for k = 1:nFrames
            if tapCount(k) == 0
                continue;
            end
            nrm = readSpan(nfid, metaAll(k).file_offset_taps, tapCount(k));
            nrmNorm = double(norm(nrm));
            assert(abs(nrmNorm - 1) < 1e-3, ...
                   'pulse %d: normalized CIR L2 norm %.6f != 1', ...
                   pid(k), nrmNorm);
            nOk = nOk + 1;
        end
        fprintf('PASS 4: normalized CIR L2 norm ~= 1 for all %d ok frames\n', ...
                nOk);
    else
        fprintf('SKIP 4: no cir_norm.cf32 in %s\n', writerOutDir);
    end

    % ---- 5) raw CIR magnitude linearity ----------------------------------
    if ~isempty(groupA) && ~isempty(groupB) && ~isempty(expectedGainRatio)
        if ~isempty(refDir)
            % Cross-directory comparison: GROUPA from writerOutDir,
            % GROUPB from refDir (single binary open per directory;
            % writerOutDir reuses the already-parsed metadata).
            rawA = readRawById(writerOutDir, metaAll, groupA);
            [~, refMeta] = read_uwb_cir(refDir, []);
            rawB = readRawById(refDir, refMeta, groupB);
        else
            rawA = [];
            for k = groupA(:)'
                assert(~isempty(rawAll{k}), 'pulse %d: no raw CIR', pid(k));
                rawA = [rawA; rawAll{k}]; %#ok<AGROW>
            end
            rawB = [];
            for k = groupB(:)'
                assert(~isempty(rawAll{k}), 'pulse %d: no raw CIR', pid(k));
                rawB = [rawB; rawAll{k}]; %#ok<AGROW>
            end
        end
        ratio = norm(rawA) / norm(rawB);
        assert(abs(ratio - expectedGainRatio) < ...
               1e-3 * expectedGainRatio, ...
               'raw CIR magnitude ratio |A|/|B| = %g, expected %g', ...
               ratio, expectedGainRatio);
        if ~isempty(refDir)
            fprintf('PASS 5: cross-directory raw CIR linearity |%s A|/|%s B| = %.6f ~= %g\n', ...
                    writerOutDir, refDir, ratio, expectedGainRatio);
        else
            fprintf('PASS 5: raw CIR linearity |A|/|B| = %.6f ~= %g\n', ...
                    ratio, expectedGainRatio);
        end
    else
        fprintf('SKIP 5: no gain groups given\n');
    end

    fprintf('verify_e2e_cir: ALL CHECKS PASSED for %s (%d frames)\n', ...
            writerOutDir, nFrames);
end

function taps = readRawById(dirName, metaRows, ids)
%READRAWBYID Read the raw CIR of the given pulse ids from dirName with a
%   single open of cir.cf32 (offset reads; O(#ids)).  metaRows is the
%   already-parsed JSONL struct array of that directory.
    fid = fopen(fullfile(dirName, 'cir.cf32'), 'rb', 'ieee-le');
    assert(fid > 0, 'cannot open %s', fullfile(dirName, 'cir.cf32'));
    cleanup = onCleanup(@() fclose(fid));
    taps = [];
    for k = ids(:)'
        idx = find([metaRows.pulse_id] == k, 1);
        assert(~isempty(idx), 'dir %s has no pulse %d', dirName, k);
        assert(metaRows(idx).tap_count > 0, ...
               'dir %s pulse %d is not an ok frame', dirName, k);
        fseek(fid, double(metaRows(idx).file_offset_taps) * 8, 'bof');
        buf = fread(fid, 2 * metaRows(idx).tap_count, 'float32=>single');
        assert(numel(buf) == 2 * metaRows(idx).tap_count, ...
               'dir %s truncated at pulse %d', dirName, k);
        taps = [taps; buf(1:2:end) + 1i * buf(2:2:end)]; %#ok<AGROW>
    end
end

function cir = readSpan(fid, offsetTaps, tapCount)
%READSPAN Read tapCount complex taps at tap offset offsetTaps (0-based).
    fseek(fid, double(offsetTaps) * 8, 'bof');
    buf = fread(fid, 2 * tapCount, 'float32=>single');
    assert(numel(buf) == 2 * tapCount, 'truncated read at offset %d', ...
           offsetTaps);
    cir = buf(1:2:end) + 1i * buf(2:2:end);
end
