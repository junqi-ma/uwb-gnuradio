# Phase-2 SIC profile golden

These text fixtures are exported by `testdata/export_sic_profile_golden.m`
from MATLAB Communications Toolbox. They freeze the production DW1000 PHY
identity used by realtime SIC: code 11, 128 SYNC repetitions and Decawave
DW-8 SFD. The BPRF scrambler prefixes for code 9/10/11 also freeze the
code-index-dependent initialization used by PHR/payload de-spreading.

They are profile constants only. They do not claim that C++ DW1000 decoding,
waveform reconstruction or cancellation is MATLAB-golden verified.

The committed fixtures were regenerated with MATLAB R2025b. Run
`export_sic_profile_golden` after a Communications Toolbox upgrade and review
all resulting diffs before accepting them.
