function meta = generate_qm35_reference_737p28(srcPath, outPath, metaPath)
%GENERATE_QM35_REFERENCE_737P28  Native-rate QM35 code-9 SYNC template.
%
%   META = GENERATE_QM35_REFERENCE_737P28() writes
%     testdata/reference_preamble_code9_737p28.cf32
%     testdata/reference_preamble_code9_737p28_metadata.json
%   from testdata/reference_preamble.bin using the same 48/65 polyphase
%   contract as testdata/generate_qm35_reference_737p28.py.
%
%   The 998.4 MS/s source is the one-SYNC waveform produced by
%   UWB_demodulation/+uwbdecoder/buildUwbReference.m (L2-normalized).
%   Energy, group delay and sample-index base are recorded in META so C++
%   and MATLAB detector starts can be compared within one sample or the
%   documented filter-delay tolerance.
%
%   This file is the MATLAB generator of record.  When MATLAB is absent
%   the Python companion produces bit-identical metadata fields (length,
%   energy, group delay) under the same resample_poly / Kaiser-5 FIR.

    if nargin < 1 || isempty(srcPath)
        here = fileparts(mfilename('fullpath'));
        srcPath = fullfile(here, 'reference_preamble.bin');
    end
    here = fileparts(srcPath);
    if nargin < 2 || isempty(outPath)
        outPath = fullfile(here, 'reference_preamble_code9_737p28.cf32');
    end
    if nargin < 3 || isempty(metaPath)
        metaPath = fullfile(here, 'reference_preamble_code9_737p28_metadata.json');
    end

    fid = fopen(srcPath, 'rb');
    if fid < 0
        error('cannot open %s', srcPath);
    end
    raw = fread(fid, inf, 'float32=>single');
    fclose(fid);
    src = complex(raw(1:2:end), raw(2:2:end));

    up = 48;
    down = 65;
    half = 10;
    native = resample(double(src), up, down);
    native = single(native(:));
    ntaps = 2 * half * max(up, down) + 1;
    gd = (ntaps - 1) / 2 / down;
    energy_before = double(native' * native);
    nrm = norm(double(native)) + eps;
    native = single(double(native) / nrm);
    energy = double(native' * native);

    fid = fopen(outPath, 'wb');
    if fid < 0
        error('cannot write %s', outPath);
    end
    interleaved = zeros(2 * numel(native), 1, 'single');
    interleaved(1:2:end) = real(native);
    interleaved(2:2:end) = imag(native);
    fwrite(fid, interleaved, 'float32');
    fclose(fid);

    meta = struct();
    meta.description = ['One QM35 / IEEE 802.15.4a HRP code-9 SYNC symbol ' ...
        'at the X410 native rate 737.28 MS/s.'];
    meta.code_index = 9;
    meta.preamble_repetitions = 64;
    meta.sfd_mode = '4z2';
    meta.input_file = 'reference_preamble.bin';
    meta.output_file = 'reference_preamble_code9_737p28.cf32';
    meta.input_rate_hz = 998.4e6;
    meta.output_rate_hz = 737.28e6;
    meta.interp = up;
    meta.decim = down;
    meta.resample_half_length = half;
    meta.source_length = numel(src);
    meta.template_length = numel(native);
    meta.energy = energy;
    meta.energy_before_normalize = energy_before;
    meta.group_delay_samples = gd;
    meta.group_delay_domain = 'native_737p28';
    meta.sample_index_base = 0;
    meta.dtype = 'complex64';
    meta.filter_delay_tolerance_samples = 1;
    meta.generator = 'testdata/generate_qm35_reference_737p28.m';
    meta.matlab_reference = 'UWB_demodulation/+uwbdecoder/buildUwbReference.m';

    txt = jsonencode(meta);
    fid = fopen(metaPath, 'w');
    fwrite(fid, txt);
    fclose(fid);
end
