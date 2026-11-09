# Full-packet SIC paired fixtures

Generate short fixtures with `gr-uwb/apps/sic_fullpacket_sim.py`; large CF32,
manifest and output data belong under the new build's results directory.
The generator refuses to overwrite any nonempty output directory.

```
python3 gr-uwb/apps/sic_fullpacket_sim.py --build-dir gr-uwb/build-sic-fullpacket --self-test
python3 gr-uwb/apps/sic_fullpacket_sim.py --build-dir gr-uwb/build-sic-fullpacket --out gr-uwb/build-sic-fullpacket/results/input_smoke --cell payload_base --windows 4
```

`--matrix` creates the fixed 10 representative cases, 16 windows each by
default. `--cell sync_isr20`, `no_overlap`, `no_interference` are additional
small smoke/negative cases. Online runner reads only public metadata,
shared public templates, and raw.cf32. clean/sensing/interference CF32 and
sidecar contain truth and are offline-verifier inputs only.

Both signals have full SYNC/SFD/PHR/PSDU. Wire PSDU includes 2-byte IEEE FCS,
appended exactly once. Sensing is code9 with weak target +37 samples/0.25;
communication is code10/64 SYNC/4z2, with private ±64-sample jitter inside a
public ±256-sample coarse search ROI. The 8-symbol observation endpoint is
computed from the public ROI upper bound, not the private start. PCHIP is the
fractional-delay generator convention, matching the inherited A/B generator.

ISR is relative to each component's own 48-SYNC [8,56) reference power.
One amplitude scale multiplies the entire communication packet; packet
length never changes its declared ISR normalization interval.
