#!/usr/bin/env python3
"""Small contract mutation QA for the independent full-packet verifier."""
import copy
import json
from pathlib import Path
import tempfile

import numpy as np
import verify_fullpacket_compare as V


def main():
    mutations = 0
    with tempfile.TemporaryDirectory(prefix="fullpacket-verifier-qa-") as tmp:
        case, run = Path(tmp)/"case", Path(tmp)/"run"
        source = case/"windows"/"w000000"
        source.mkdir(parents=True)
        run.mkdir()
        public = dict(n=256, observed_end=128, target_lo=0, target_hi=256,
                      sensing_seed=32, sample_rate=998400000.)
        (case/"public.json").write_text(json.dumps(public))
        clean = np.full(256, complex(.1, .2), dtype=np.complex64)
        raw = clean + np.complex64(.3+.4j)
        clean.tofile(source/"clean.cf32")
        raw.tofile(source/"raw.cf32")
        for name in ("comm_sync.cf32", "comm_preamble.cf32", "sensing_sync.cf32"):
            np.ones(16, dtype=np.complex64).tofile(case/name)
        payload = b"qa"
        wire = (payload+V.crc16(payload).to_bytes(2, "little")).hex()
        (source/"sidecar.json").write_text(json.dumps(dict(comm_psdu_hex=wire,
                    sensing_psdu_hex=wire, hashes={"raw.cf32": V.sha256(source/"raw.cf32")})))
        template = dict(window_id=0, source_window=0, mode="preamble", committed=False,
                        reason="test", model_valid=True, model_id=1, n=256, observed_end=128,
                        support=[0, 256], comm_fcs=None, comm_psdu_hex="", sensing_fcs=True,
                        sensing_psdu_hex=wire, times_us={k: 1. for k in V.STAGES},
                        events=dict(required_sample=128, submit_us=0., start_us=1., finish_us=2., result_us=3.))
        template["times_us"]["sic_total"] = 7.
        template["times_us"]["service"] = 9.
        full = copy.deepcopy(template)
        full.update(mode="full", committed=True, comm_fcs=True, comm_psdu_hex=wire)
        full["events"]["required_sample"] = 256
        rows = [template, full]
        raw.tofile(run/"w000000_preamble.cf32")
        clean.tofile(run/"w000000_full.cf32")
        def evaluate(values):
            (run/"windows.jsonl").write_text("".join(json.dumps(v)+"\n" for v in values))
            return V.verify(case, run)
        baseline = evaluate(rows)
        assert baseline["passed"], baseline["errors"]
        assert baseline["modes"]["full"]["comm_exact"]["rate"] == 1.
        assert baseline["iq_effects_verified"]
        for name, mutate in [
            ("support", lambda r: r[1].update(support=[0,1])),
            ("bypass", lambda r: r[1].update(committed=False)),
            ("FCS", lambda r: r[1].update(comm_psdu_hex="01000000")),
            ("time", lambda r: r[1]["events"].update(start_us=9.)),
            ("duplicate", lambda r: r.append(copy.deepcopy(r[1]))),
            ("input hash", lambda r: r[1].update(input_hash="0"*64)),
        ]:
            values = copy.deepcopy(rows)
            mutate(values)
            assert not evaluate(values)["passed"], name+" mutation escaped"
            mutations += 1
        # Immediate rejection status may precede availability of the full
        # context. The same timestamp on an accepted output is invalid.
        immediate = copy.deepcopy(full)
        immediate.update(committed=False, model_valid=False, comm_fcs=False,
                         comm_psdu_hex="", sensing_fcs=False, sensing_psdu_hex="",
                         support=[0,0], reason="queue_full", rejected=True, times_us={})
        immediate["events"] = dict(required_sample=256, submit_us=1., start_us=1.,
                                   finish_us=1., result_us=1., first_clean_us=1., sample_ready_us=2.)
        assert not V.validate_record(immediate, public)
        accepted_early = copy.deepcopy(full)
        accepted_early["events"].update(first_clean_us=1., sample_ready_us=2.)
        assert "early clean before samples" in V.validate_record(accepted_early, public)
        evaluate(rows)
        summary = V.verify(case, run, summary_only=True)
        assert summary["passed"] and not summary["iq_effects_verified"]
        repeated = [copy.deepcopy(row) for row in rows+rows]
        for row in repeated[2:]:
            row["window_id"] = 1
        (run/"windows.jsonl").write_text("".join(json.dumps(v)+"\n" for v in repeated))
        repeated_summary = V.verify(case, run, summary_only=True)
        assert repeated_summary["passed"], repeated_summary["errors"]
        fs = repeated_summary["modes"]["full"]
        assert fs["n"] == 2 and fs["unique_sources"] == 1
        assert fs["comm_exact"]["ci95"] is None
        assert fs["unique_source_rates"]["comm_exact"]["n"] == 1
        assert V.rate(0,16)["ci95"][1] > 0
        assert V.rate(1,1)["ci95"][0] < 1
    print(f"qa_fullpacket_compare: baseline + {mutations} mutations + rejection causality + summary-only PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
