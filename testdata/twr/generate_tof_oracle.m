function generate_tof_oracle()
%GENERATE_TOF_ORACLE  M1-A independent SS/DS ToF oracle (MATLAB of record).
%
%   STATUS ON THIS MACHINE: NOT EXECUTED.  The M1-A instruction requires this
%   script to be RUN and its vectors checked in.  The machine used to develop
%   M1-A has no runnable MATLAB: /usr/local/MATLAB/R2024a has no `bin/matlab`
%   launcher and no main binary, there is no MATLAB Runtime, and there is no
%   Octave.  The checked-in `tof_oracle_vectors.json` was therefore produced
%   by `gen_tof_oracle.py`, an exact `fractions.Fraction` reference of the SAME
%   physical model.  That substitution is recorded in the JSON provenance
%   block and in README_tof_oracle.md.  It is NOT MATLAB verification.
%
%   RUN (once MATLAB exists):
%       matlab -batch "cd('testdata/twr'); generate_tof_oracle"
%   It overwrites tof_oracle_vectors.json in the same directory and prints the
%   per-vector expected values.  JSON object KEY ORDER differs from the Python
%   writer (jsonencode sorts keys); object order is not significant.
%
%   THE MODEL (must match gen_tof_oracle.py statement for statement)
%   -----------------------------------------------------------------
%   Two clocks fA, fB.  k = fA/fB converts B ticks into A ticks.
%
%       RA = t4A - t1A = 2*tau_A + dB_A          (A ticks)
%       DA = t5A - t4A = dA_A                    (A ticks)
%       DB = t3B - t2B = dB_A / k                (B ticks)
%       RB = t6B - t3B = (2*tau_A + dA_A) / k    (B ticks)
%
%       SS: ToF_A = (RA - k*DB) / 2                     = tau_A
%       DS: ToF_A = (RA*k*RB - DA*k*DB)/(RA + k*RB + DA + k*DB) = tau_A
%
%   Both identities are exact, so the script re-evaluates the formula and
%   asserts it equals tau_A before writing anything.  A mismatch is a model
%   bug and aborts with an error rather than emitting a wrong golden.

HERE = fileparts(mfilename('fullpath'));
OUT  = fullfile(HERE, 'tof_oracle_vectors.json');

WRAP_BITS   = 40;
WRAP_PERIOD = 2 ^ WRAP_BITS;         % exact: 2^40 is an integer in double
UUS_RATE    = 499.2e6 * 128.0;

domA    = dom('dw1000_snA_ch5_uus', UUS_RATE, 7, WRAP_BITS);
domB    = dom('dw3000_snB_ch5_uus', UUS_RATE, 7, WRAP_BITS);
domA_rr = dom('dw1000_snA_ch5_uus', 1.0e9,      7, WRAP_BITS);
domB_rr = dom('dw3000_snB_ch5_uus', 998.4e6,    7, WRAP_BITS);

V = {};
V{end+1} = mk('ss1_common_clock','ss', R(1,1), domA, domA, ...
    R(500), R(1000), [], 'nominal_same_clock', false, 'ok', ...
    'hand check: RA=2000, DB=1000, k=1 -> ToF=500');
V{end+1} = mk('ss2_nominal_rate_ratio','ss', R(625,624), domA_rr, domB_rr, ...
    R(500), R(1000), [], 'nominal_rate_ratio', false, 'ok', ...
    'hand check: k*DB = (625/624)*(4992/5) = 1000 A ticks; (2000-1000)/2=500');
V{end+1} = mk('ss3a_a_faster_30ppm','ss', R(32768,32767), domA_rr, domB_rr, ...
    R(500), R(32768), [], 'nominal_rate_ratio', false, 'ok', ...
    'k>1 (A runs fast): DB=32767 B ticks -> back to 32768 A ticks');
V{end+1} = mk('ss3b_a_slower_30ppm','ss', R(32767,32768), domA_rr, domB_rr, ...
    R(500), R(32767), [], 'nominal_rate_ratio', false, 'ok', ...
    'k<1 (A runs slow): DB=32768 B ticks -> back to 32767 A ticks');
V{end+1} = mk('ss5f_fractional_tof','ss', R(1,1), domA, domA, ...
    R(5,2), R(1000), [], 'nominal_same_clock', false, 'ok', ...
    'RA=1005, DB=1000, k=1 -> ToF=5/2 (must not truncate)');
V{end+1} = mk('ss8_wrapped_a_interval','ss', R(1,1), domA, domA, ...
    R(105), R(1000), [], 'nominal_same_clock', true, 'ok', ...
    't1A=P-100, t4A=1110 -> RA=1210 across wrap; (1210-1000)/2=105');
V{end+1} = mk('ss7_negative_tof','ss', R(1,1), domA, domA, ...
    R(-500), R(2000), [], 'nominal_same_clock', false, 'negative_tof', ...
    'RA=1000, DB=2000 -> ToF=-500, kept signed (never clamped to 0)');
V{end+1} = mk('ds1_symmetric','ds', R(1,1), domA, domA, ...
    R(500), R(1000), R(1000), 'nominal_same_clock', false, 'ok', ...
    'RA=2000,RB=2000,DA=DB=1000 -> (4e6-1e6)/6000=500, same as SS-1');
V{end+1} = mk('ds2_asymmetric','ds', R(1,1), domA, domA, ...
    R(500), R(1000), R(3000), 'nominal_same_clock', false, 'ok', ...
    'RA=2000,RB=4000,DA=3000,DB=1000 -> (8e6-3e6)/10000=500');
V{end+1} = mk('ds3_tiny_tof_long_turnaround','ds', R(1,1), domA, domA, ...
    R(1,7), R(3000)+R(2,7), R(5000)+R(3,7), 'nominal_same_clock', false, 'ok', ...
    'tau=1/7; long asymmetric turnaround; no ns truncation anywhere');
V{end+1} = mk('ds7_asymmetric_rate_ratio','ds', R(625,624), domA_rr, domB_rr, ...
    R(500), R(1000), R(1000), 'nominal_rate_ratio', false, 'ok', ...
    'DB=4992/5, RB=9984/5 B ticks; converted back -> 500 A ticks');
V{end+1} = mk('ds6_negative_tof','ds', R(1,1), domA, domA, ...
    R(-450), R(1000), R(1000), 'nominal_same_clock', false, 'negative_tof', ...
    'RA=RB=100, DA=DB=1000 -> N=-990000, D=2200 -> -450');

doc = struct();
doc.schema = 'uwb-twr-tof-oracle/1';
doc.provenance = struct( ...
    'generator', 'testdata/twr/generate_tof_oracle.m', ...
    'generator_language', 'MATLAB R2024a, int64 rational arithmetic', ...
    'matlab_script', 'testdata/twr/generate_tof_oracle.m', ...
    'matlab_executed', true, ...
    'matlab_executed_note', 'produced by running this script', ...
    'units', 'ticks', ...
    'tof_domain', 'A (clock ratio k = fA/fB converts B ticks to A ticks)', ...
    'formulas', struct('ss', '(RA_A - k*DB_B) / 2', ...
                       'ds', '(RA_A*k*RB_B - DA_A*k*DB_B) / (RA_A + k*RB_B + DA_A + k*DB_B)'));
doc.vectors = [V{:}];

fid = fopen(OUT, 'w');
if fid < 0, error('cannot open %s for writing', OUT); end
fprintf(fid, '%s\n', jsonencode(doc, 'PrettyPrint', true));
fclose(fid);

fprintf('wrote %s (%d vectors)\n', OUT, numel(V));
for i = 1:numel(V)
    e = V{i}.expected;
    if isempty(e.tof)
        tn = '-'; td = '-';
    else
        tn = e.tof.num; td = e.tof.den;
    end
    fprintf('  %-30s %-3s status=%-13s tof=%s/%s\n', ...
        V{i}.id, V{i}.protocol, e.status, num2str(tn), num2str(td));
end
end

% ===========================================================================
% local helpers
% ===========================================================================
function d = dom(name, rate, epoch, bits)
d = struct('name', name, 'tick_rate_hz', rate, 'epoch', epoch, ...
           'timestamp_bits', bits);
end

function r = R(n, d)
%R  Exact rational in lowest terms, positive denominator.
if d == 0, error('zero denominator'); end
if d < 0, n = -n; d = -d; end
g = gcd(abs(n), d);
r = struct('num', int64(n / g), 'den', int64(d / g));
end

function r = radd(a, b), r = R(int64(a.num)*int64(b.den) + int64(b.num)*int64(a.den), ...
                               int64(a.den)*int64(b.den)); end
function r = rsub(a, b), r = R(int64(a.num)*int64(b.den) - int64(b.num)*int64(a.den), ...
                               int64(a.den)*int64(b.den)); end
function r = rmul(a, b), r = R(int64(a.num)*int64(b.num), int64(a.den)*int64(b.den)); end
function r = rdiv(a, b)
if b.num == 0, error('division by zero'); end
r = R(int64(a.num)*int64(b.den), int64(a.den)*int64(b.num));
end
function b = rlt(a, b)   % a < b for exact rationals
b = int64(a.num)*int64(b.den) < int64(b.num)*int64(a.den);
end

function e = endpoint(x, base, wrap, WRAP_PERIOD)
%ENDPOINT  (ticks,num,den) of `x + base`, folded when `wrap`.
y = radd(x, base);
if wrap
    % `y` is a non-negative rational; fold the INTEGER part.
    ti = floor(double(y.num) / double(y.den));
    ti = mod(ti, WRAP_PERIOD);
    fr = rsub(y, R(ti, 1));
    if fr.num == 0
        e = struct('ticks', int64(ti), 'num', int64(0), 'den', int64(0));
    else
        e = struct('ticks', int64(ti), 'num', int64(fr.num), 'den', int64(fr.den));
    end
else
    ti = floor(double(y.num) / double(y.den));
    fr = rsub(y, R(ti, 1));
    if fr.num == 0
        e = struct('ticks', int64(ti), 'num', int64(0), 'den', int64(0));
    else
        e = struct('ticks', int64(ti), 'num', int64(fr.num), 'den', int64(fr.den));
    end
end
if e.den ~= 0 && e.den > 32767
    error('interval fraction denominator %d exceeds 32767', e.den);
end
end

function s = kv(x)
s = struct('num', int64(x.num), 'den', int64(x.den));
end

function iv = interval(later, earlier, lm, em)
iv = struct('later', later, 'earlier', earlier, ...
            'later_marker', lm, 'earlier_marker', em);
end

function v = mk(id, proto, k, domA, domB, tau, db, da, source, wrap_a, status, note)
WRAP_PERIOD = 2 ^ 40;
% --- intervals from the physical model -----------------------------------
if strcmp(proto, 'ss')
    ra = radd(rmul(R(2), tau), db);
    dbb = rdiv(db, k);
    daa = [];
    rbb = [];
else
    ra = radd(rmul(R(2), tau), db);
    daa = da;
    dbb = rdiv(db, k);
    rbb = rdiv(radd(rmul(R(2), tau), da), k);
end

% --- the model re-evaluates its own formula (independent of C++) ---------
if strcmp(proto, 'ss')
    derived = rdiv(rsub(ra, rmul(k, dbb)), R(2));
else
    rb_in_a = rmul(k, rbb);
    db_in_a = rmul(k, dbb);
    num = rsub(rmul(ra, rb_in_a), rmul(daa, db_in_a));
    den = radd(radd(ra, rb_in_a), radd(daa, db_in_a));
    derived = rdiv(num, den);
end
if derived.num ~= tau.num || derived.den ~= tau.den
    error('model self-check failed for %s: derived=%d/%d tau=%d/%d', ...
        id, derived.num, derived.den, tau.num, tau.den);
end

% --- absolute timestamps -------------------------------------------------
t1a = R(0); t4a = ra; t5a = [];
if ~isempty(daa), t5a = radd(ra, daa); end
t2b = R(0); t3b = dbb; t6b = [];
if ~isempty(rbb), t6b = radd(dbb, rbb); end
if wrap_a, base_a = R(WRAP_PERIOD - 100, 1); else, base_a = R(0); end

ivs = struct();
ivs.ra = interval(endpoint(t4a, base_a, wrap_a, WRAP_PERIOD), ...
                  endpoint(t1a, base_a, wrap_a, WRAP_PERIOD), ...
                  'rmarker_rx', 'rmarker_tx');
ivs.db = interval(endpoint(t3b, R(0), false, WRAP_PERIOD), ...
                  endpoint(t2b, R(0), false, WRAP_PERIOD), ...
                  'rmarker_tx', 'rmarker_rx');
if strcmp(proto, 'ds')
    ivs.da = interval(endpoint(t5a, base_a, wrap_a, WRAP_PERIOD), ...
                      endpoint(t4a, base_a, wrap_a, WRAP_PERIOD), ...
                      'rmarker_tx', 'rmarker_rx');
    ivs.rb = interval(endpoint(t6b, R(0), false, WRAP_PERIOD), ...
                      endpoint(t3b, R(0), false, WRAP_PERIOD), ...
                      'rmarker_rx', 'rmarker_tx');
end

if strcmp(status, 'ok') || strcmp(status, 'negative_tof')
    tof = kv(derived);
else
    tof = [];
end

v = struct();
v.id = id;
v.protocol = proto;
v.note = note;
v.clock_ratio = struct('source', source, 'k', kv(k));
v.domain_a = domA;
v.domain_b = domB;
v.intervals = ivs;
v.expected = struct('status', status, 'tof_domain', 'a', 'tof', tof);
end
