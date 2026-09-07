function verify_read_uwb_cir(mixedDir, normDir)
%VERIFY_READ_UWB_CIR Verify read_uwb_cir.m against real UwbCirWriter output.
%   MIXEDDIR is a UwbCirWriter QA output directory written with
%   write_normalized=false (mixed ok/failed frames); NORMDIR one written
%   with write_normalized=true.  Both are produced by
%   gr-uwb/lib/qa_uwb_cir_writer.cc (test_writer_mixed_frames and
%   test_writer_normalized).
%
%   Checks:
%     1. ok raw CIR == the QA tap pattern, sample by sample (complex single)
%        pattern: (pulse_id*1000+i)*0.001 + 1i*i*(-0.0005), float32 math
%     2. failed pulse -> empty cir, meta.tap_count == 0, status kept
%     3. normalized CIR == same pattern, via cir_norm.cf32
%     4. truncated cir.cf32 -> error 'read_uwb_cir:short'
%     5. missing file_offset_norm_taps -> error 'read_uwb_cir:noNorm'
%
%   Usage:
%     addpath(testdata_uwb_radar_dir);
%     verify_read_uwb_cir('/tmp/uwb_qa_cir_writer_mixed', ...
%                         '/tmp/uwb_qa_cir_writer_norm');

    tap_count = 116;

    % ---- 1) ok raw CIR, sample-by-sample against the QA pattern --------
    for pulse_id = [0 1]
        [cir, meta] = read_uwb_cir(mixedDir, pulse_id);
        assert(isequal(size(cir), [tap_count, 1]), ...
               'pulse %d: cir must be %dx1, got %s', ...
               pulse_id, tap_count, mat2str(size(cir)));
        assert(isa(cir, 'single') && ~isreal(cir), ...
               'pulse %d: cir must be complex single', pulse_id);
        expect = expect_taps(pulse_id, tap_count);
        assert(isequal(cir, expect), ...
               'pulse %d: CIR samples differ from QA writer pattern', ...
               pulse_id);
        assert(strcmp(char(meta.status), 'ok'), 'pulse %d: status must be ok', pulse_id);
        assert(meta.tap_count == tap_count, ...
               'pulse %d: tap_count must be %d', pulse_id, tap_count);
        fprintf('PASS 1: pulse %d raw CIR %dx1 complex single, per-sample exact\n', ...
                pulse_id, tap_count);
    end

    % ---- 2) failed pulse -> empty CIR, metadata kept -------------------
    % Mixed dir statuses (test_writer_mixed_frames):
    %   0 ok / 1 ok / 2 sfd_failed / 3 ok / 4 timing_failed / 5 ok /
    %   6 cir_failed / 7 ok / 8 invalid_input / 9 ok
    failed_map = {2, 'sfd_failed'; 4, 'timing_failed'; ...
                  6, 'cir_failed'; 8, 'invalid_input'};
    for r = 1:size(failed_map, 1)
        pid = failed_map{r, 1};
        [cir, meta] = read_uwb_cir(mixedDir, pid);
        assert(isempty(cir), 'pulse %d: cir must be empty', pid);
        assert(meta.tap_count == 0, ...
               'pulse %d: meta.tap_count must be 0', pid);
        assert(strcmp(char(meta.status), failed_map{r, 2}), ...
               'pulse %d: status %s != %s', pid, meta.status, ...
               failed_map{r, 2});
        fprintf('PASS 2: pulse %d (%s) -> empty cir, tap_count==0\n', ...
                pid, failed_map{r, 2});
    end

    % ---- 3) normalized CIR from the normalized run ---------------------
    % Normalized dir has exactly two ok frames (0 and 6): their
    % normalized offsets are 0 and tap_count.
    for pulse_id = [0 6]
        [nrm, meta] = read_uwb_cir(normDir, pulse_id, true);
        assert(isequal(size(nrm), [tap_count, 1]), ...
               'pulse %d: norm must be %dx1', pulse_id, tap_count);
        assert(isequal(nrm, expect_taps(0, tap_count)), ...
               'pulse %d: normalized samples differ', pulse_id);
        assert(meta.file_offset_norm_taps == pulse_id / 6 * tap_count, ...
               'pulse %d: wrong file_offset_norm_taps', pulse_id);
        [raw, ~] = read_uwb_cir(normDir, pulse_id);
        assert(isequal(raw, nrm), ...
               'pulse %d: raw != norm (QA writes the same pattern)', ...
               pulse_id);
        fprintf('PASS 3: pulse %d normalized CIR, per-sample exact\n', ...
                pulse_id);
    end
    % failed frame in the normalized run also returns empty cir.
    [cir5, meta5] = read_uwb_cir(normDir, 2);
    assert(isempty(cir5) && meta5.tap_count == 0, ...
           'normalized run pulse 2 must be a failed frame');
    fprintf('PASS 3b: normalized-run failed pulse 2 -> empty cir\n');

    % ---- 4) truncated cir.cf32 -> error read_uwb_cir:short -------------
    tmp = tempdir;
    truncDir = fullfile(tmp, sprintf('uwb_verify_trunc_%d', randi(1e9)));
    if exist(truncDir, 'dir'); rmdir(truncDir, 's'); end
    mkdir(truncDir);
    copyfile(fullfile(mixedDir, 'cir.jsonl'), ...
             fullfile(truncDir, 'cir.jsonl'));
    copyfile(fullfile(mixedDir, 'cir.cf32'), fullfile(truncDir, 'cir.cf32'));
    % keep only the first tap's bytes -> line for pulse 0 cannot be read
    fid = fopen(fullfile(truncDir, 'cir.cf32'), 'rb');
    allb = fread(fid, inf, 'uint8');
    fclose(fid);
    fid = fopen(fullfile(truncDir, 'cir.cf32'), 'wb');
    fwrite(fid, allb(1:8), 'uint8');
    fclose(fid);
    threw = '';
    try
        read_uwb_cir(truncDir, 0);
    catch e
        threw = e.identifier;
    end
    rmdir(truncDir, 's');
    assert(strcmp(threw, 'read_uwb_cir:short'), ...
           'truncated cir.cf32 must raise read_uwb_cir:short, got %s', threw);
    fprintf('PASS 4: truncated cir.cf32 -> read_uwb_cir:short\n');

    % ---- 5) missing normalized offset -> error read_uwb_cir:noNorm ----
    assert(~exist(fullfile(mixedDir, 'cir_norm.cf32'), 'file'), ...
           'mixed run must not contain cir_norm.cf32');
    threw = '';
    try
        read_uwb_cir(mixedDir, 0, true);
    catch e
        threw = e.identifier;
    end
    assert(strcmp(threw, 'read_uwb_cir:noNorm'), ...
           'missing norm must raise read_uwb_cir:noNorm, got %s', threw);
    fprintf('PASS 5: no file_offset_norm_taps -> read_uwb_cir:noNorm\n');

    % ---- 6) missing pulse -> error read_uwb_cir:noPulse ----------------
    threw = '';
    try
        read_uwb_cir(mixedDir, 12345);
    catch e
        threw = e.identifier;
    end
    assert(strcmp(threw, 'read_uwb_cir:noPulse'), ...
           'unknown pulse must raise read_uwb_cir:noPulse, got %s', threw);
    fprintf('PASS 6: unknown pulse id -> read_uwb_cir:noPulse\n');

    % ---- 7) bulk read: all lines as a struct array ---------------------
    [~, metaAll] = read_uwb_cir(mixedDir, []);
    assert(numel(metaAll) == 10, ...
           'mixed run must contain 10 lines, got %d', numel(metaAll));
    assert(isequal([metaAll.pulse_id]', (0:9)'), ...
           'pulse_id order mismatch in bulk read');
    assert(isequal(double([metaAll.tap_count]), ...
                   [116 116 0 116 0 116 0 116 0 116]), ...
           'tap_count pattern mismatch in bulk read');
    fprintf('PASS 7: bulk read returns 10-line struct array\n');

    fprintf('verify_read_uwb_cir: ALL CHECKS PASSED\n');
end

function taps = expect_taps(pulse_id, n)
%EXPECT_TAPS The deterministic QA writer tap pattern (float32 arithmetic,
% identical to gr-uwb/lib/qa_uwb_cir_writer.cc make_taps()).
    i = single(0:(n - 1));
    re = (single(double(pulse_id) * 1000) + i) .* single(0.001);
    im = i .* single(-0.0005);
    taps = (re + 1i * im);
    taps = taps(:);
end
