#!/usr/bin/env python3
"""Full-packet interference generator; truth-bearing and never an online API.

See docs/sic/FullPacket_接口合同.md. Both cancellation modes consume the SAME
raw IQ. Shared templates and public.json contain only receiver configuration;
channel, bytes, true starts and component IQ live in offline sidecars.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import sic_preamble_sim as S
import sic_payload_sim as P

SCHEMA = "sic-fullpacket-case/1"
SPS = S.SPS
RATE = S.WORK_HZ
SYNC_REPS = 64
COMM_NOMINAL = 2048
SEARCH_HALF = 256
JITTER_HALF = 64
FOUR = [[0, [1.0, 0.0]], [7.4, [0.0, 0.5]],
        [23, [0.276, -0.116]], [51.2, [0.2, 0.0]]]
SENSING_PATHS = [[0, [1.0, 0.0]], [37, [0.25, 0.0]]]

# The 10 cells are fixed before observing outcomes. Extra cells are ONLY
# interface smoke/negative QA, never silently included in the representative set.
CELLS = {
    "sync_base": dict(position="sync"),
    "payload_base": dict(position="payload"),
    "payload_len16": dict(position="payload", psdu_bytes=16),
    "payload_len127": dict(position="payload", psdu_bytes=127),
    "payload_snr8": dict(position="payload", snr_db=8.0),
    "payload_isr0": dict(position="payload", isr_db=0.0),
    "payload_isr20": dict(position="payload", isr_db=20.0),
    "payload_cfo_m50k": dict(position="payload", cfo_hz=-50000.0),
    "payload_cfo_p50k": dict(position="payload", cfo_hz=50000.0),
    "payload_four_frac": dict(position="payload", channel="four", frac=0.4),
}
EXTRA = {
    "sync_isr20": dict(position="sync", isr_db=20.0),
    "no_overlap": dict(position="none"),
    "no_interference": dict(position="sync", present=False),
}


def dump_json(path: Path, obj):
    path.write_text(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + "\n")


def config(cell: str):
    cfg = dict(cell=cell, position="sync", psdu_bytes=32, sensing_psdu_bytes=32,
               snr_db=30.0, isr_db=10.0, cfo_hz=0.0, channel="single", frac=0.0,
               present=True, frac_model="pchip", pulse_shape="legacy")
    cfg.update((CELLS | EXTRA)[cell])
    if not (2 <= cfg["psdu_bytes"] <= 127):
        raise ValueError("unsupported wire PSDU length")
    return cfg


def fields(start: float, nbytes: int):
    f = P.comm_field_offsets(SYNC_REPS, 8, nbytes)
    return {name: [start + f[lo], start + f[hi]] for name, lo, hi in (
        ("SYNC", "sync_lo", "sync_hi"), ("SFD", "sfd_lo", "sfd_hi"),
        ("PHR", "phr_lo", "phr_hi"), ("payload", "payload_lo", "payload_hi"))}


def make_packet(uwb, wire: bytes, code: int):
    return P.comm_packet_samples(uwb, wire, sync_reps=SYNC_REPS,
                                 sfd_mode="4z2", comm_code=code,
                                 pulse_shape="legacy", peak_amplitude=0.8)


def templates(uwb):
    comm = P.build_finite_sync_template(
        uwb, code_index=10, sync_reps=SYNC_REPS, sfd_mode="4z2",
        pulse_shape="legacy", peak_amplitude=0.8)
    sensing = P.build_finite_sync_template(
        uwb, code_index=9, sync_reps=SYNC_REPS, sfd_mode="4z2",
        pulse_shape="legacy", peak_amplitude=0.8)
    return {"comm_sync.cf32": comm["sync"],
            "comm_preamble.cf32": comm["waveform"],
            "sensing_sync.cf32": sensing["sync"]}, {"comm": comm, "sensing": sensing}


def geometry(cfg):
    comm_fields = P.comm_field_offsets(SYNC_REPS, 8, cfg["psdu_bytes"])
    if cfg["position"] == "sync":
        desired = COMM_NOMINAL
    elif cfg["position"] == "payload":
        desired = COMM_NOMINAL + comm_fields["payload_lo"] + 2048
    else:
        desired = COMM_NOMINAL + comm_fields["packet_len"] + 8192
    desired_fields = P.comm_field_offsets(SYNC_REPS, 8, cfg["sensing_psdu_bytes"])
    # 256 guard covers private jitter and official path delays; 4096 tail
    # covers all shaping, desired weak target and truncated-context negatives.
    n = max(COMM_NOMINAL + comm_fields["packet_len"],
            desired + desired_fields["packet_len"]) + 4096
    n = (n + 3) // 4 * 4
    return dict(sample_rate=RATE, n=int(n), roi_lo=COMM_NOMINAL - SEARCH_HALF,
                roi_hi=COMM_NOMINAL + SEARCH_HALF,
                observed_end=COMM_NOMINAL + SEARCH_HALF + 8 * SPS + 128,
                sensing_seed=int(desired), consumer_lo=0, consumer_hi=int(n),
                target_lo=int(desired), target_hi=int(desired + SYNC_REPS * SPS),
                comm_code=10, sensing_code=9, sync_reps=64, sfd_mode="4z2",
                pulse_shape="legacy", peak_amplitude=0.8,
                max_psdu_bytes=127, max_context_samples=int(n))


def realize(uwb, cfg, public, seed: int, window_id: int):
    # Split RNG streams: changing noise or geometry cannot expose packet bytes
    # to the online path, or silently replace one branch's payload.
    streams = np.random.SeedSequence([int(seed), int(window_id)]).spawn(4)
    comm_rng, sensing_rng, timing_rng, noise_rng = [np.random.default_rng(x) for x in streams]
    comm_wire = P.append_fcs(comm_rng.integers(0, 256, cfg["psdu_bytes"] - 2,
                                             dtype=np.uint8).tobytes())
    sensing_wire = P.append_fcs(sensing_rng.integers(0, 256, cfg["sensing_psdu_bytes"] - 2,
                                                   dtype=np.uint8).tobytes())
    comm_tx = make_packet(uwb, comm_wire, 10)
    sensing_tx = make_packet(uwb, sensing_wire, 9)
    n = public["n"]
    sensing = np.zeros(n, dtype=np.complex64)
    for delay, gain in SENSING_PATHS:
        S.apply_path(sensing, sensing_tx, public["sensing_seed"], delay, complex(*gain))
    interference = np.zeros(n, dtype=np.complex64)
    jitter = int(timing_rng.integers(-JITTER_HALF, JITTER_HALF + 1))
    comm_start = COMM_NOMINAL + jitter
    paths = FOUR if cfg["channel"] == "four" else [[0, [1.0, 0.0]]]
    for delay, gain in paths:
        S.apply_path(interference, comm_tx, comm_start, float(delay) + cfg["frac"],
                     complex(*gain), cfg["frac_model"])
    desired_ref = slice(public["sensing_seed"] + 8 * SPS,
                        public["sensing_seed"] + 56 * SPS)
    comm_ref = slice(comm_start + 8 * SPS, comm_start + 56 * SPS)
    ps = float(np.mean(np.abs(sensing[desired_ref]).astype(np.float64) ** 2))
    pi = float(np.mean(np.abs(interference[comm_ref]).astype(np.float64) ** 2))
    amplitude = math.sqrt(ps * 10 ** (cfg["isr_db"] / 10) / pi)
    interference *= np.float32(amplitude)
    if cfg["cfo_hz"]:
        rot = np.exp(2j * np.pi * cfg["cfo_hz"] *
                     (np.arange(n, dtype=np.float64) - comm_start) / RATE)
        interference = (interference * rot).astype(np.complex64)
    if not cfg["present"]:
        interference.fill(0)
    sigma = math.sqrt(ps * 10 ** (-cfg["snr_db"] / 10) / 2)
    noise = (sigma * (noise_rng.standard_normal(n) +
                      1j * noise_rng.standard_normal(n))).astype(np.complex64)
    clean = (sensing + noise).astype(np.complex64)
    raw = (clean + interference).astype(np.complex64)
    arrays = {"raw.cf32": raw, "clean.cf32": clean, "sensing.cf32": sensing,
              "interference.cf32": interference}
    sidecar = dict(schema=SCHEMA, window_id=window_id, seed=int(seed),
                   comm_psdu_hex=comm_wire.hex(), sensing_psdu_hex=sensing_wire.hex(),
                   comm_start=comm_start, comm_fractional_offset=cfg["frac"],
                   sensing_start=public["sensing_seed"], desired_seed=public["sensing_seed"],
                   cfo_hz=cfg["cfo_hz"], cfo_phase_reference=comm_start,
                   paths=paths, sensing_paths=SENSING_PATHS, frac_model=cfg["frac_model"],
                   amplitude_scale=amplitude, sensing_sync_power=ps,
                   comm_sync_power_before_scale=pi, noise_component_variance=sigma * sigma,
                   isr_reference="48_SYNC_[8,56)_each_signal_own_origin",
                   fields=fields(comm_start + cfg["frac"], cfg["psdu_bytes"]),
                   sensing_fields=fields(public["sensing_seed"], cfg["sensing_psdu_bytes"]),
                   target_delay_samples=37, target_gain=[0.25, 0.0],
                   injected=cfg, hashes={k: S.sha256_array(v) for k, v in arrays.items()})
    return arrays, sidecar


def generate_case(uwb, out: Path, cell: str, windows: int, seed: int, template_data=None):
    # Refuse overwrite, including partially generated historical runs.
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"output is not empty: {out}")
    out.mkdir(parents=True, exist_ok=True)
    cfg = config(cell)
    public = geometry(cfg)
    waves, evidence = template_data or templates(uwb)
    public.update(schema=SCHEMA, cell=cell,
                  template_hashes={name: S.sha256_array(wave) for name, wave in waves.items()})
    for name, wave in waves.items():
        np.asarray(wave, dtype=np.complex64).tofile(out / name)
    dump_json(out / "public.json", public)
    for wid in range(windows):
        wdir = out / "windows" / f"w{wid:06d}"
        wdir.mkdir(parents=True)
        arrays, sidecar = realize(uwb, cfg, public, seed, wid)
        for name, a in arrays.items():
            a.tofile(wdir / name)
        dump_json(wdir / "sidecar.json", sidecar)
    manifest = dict(schema=SCHEMA, cell=cell, config=cfg, seed=seed, windows=windows,
                    fixed_representative_cells=list(CELLS),
                    generator_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    template_evidence={role: x["meta"] for role, x in evidence.items()},
                    online_files=["public.json", *waves, "windows/*/raw.cf32"],
                    offline_only=["windows/*/sidecar.json", "windows/*/clean.cf32",
                                  "windows/*/sensing.cf32", "windows/*/interference.cf32"],
                    not_run=["MATLAB", "native", "hardware"])
    dump_json(out / "manifest.json", manifest)
    return manifest


def self_test(uwb):
    # One short integration QA. Independent CRC check-vector plus code-pulse
    # shift-add vs dense convolution protect shared-generator self-consistency.
    assert P.crc16_802154(b"123456789") == 0x2189
    wave_data, evidence = templates(uwb)
    for x in evidence.values():
        assert x["meta"]["convolve_max_abs_diff"] < 1e-10
        assert x["meta"]["recovery"]["max_int_dev"] < 1e-4
    cfg = config("payload_base")
    public = geometry(cfg)
    arrays, side = realize(uwb, cfg, public, 2000000, 0)
    assert np.array_equal(arrays["raw.cf32"], arrays["clean.cf32"] + arrays["interference.cf32"])
    wire = bytes.fromhex(side["comm_psdu_hex"])
    assert len(wire) == 32 and P.crc16_802154(wire) == 0
    tx = make_packet(uwb, wire, 10)
    other_tx = make_packet(uwb, P.append_fcs(bytes(range(30))), 10)
    assert np.array_equal(tx[:8 * SPS], other_tx[:8 * SPS])
    assert not np.array_equal(tx[64 * SPS:], other_tx[64 * SPS:])
    # The finite template cannot contain any SFD/PHR/PSDU data.
    finite = wave_data["comm_preamble.cf32"]
    sync = wave_data["comm_sync.cf32"]
    ratio = np.vdot(sync, tx[:SPS]) / np.vdot(sync, sync)
    assert np.max(np.abs(tx[:8 * SPS] - ratio * finite[:8 * SPS])) < 1e-6
    assert side["fields"]["payload"][0] < public["target_hi"]
    assert side["fields"]["payload"][1] > public["target_lo"]
    assert public["observed_end"] == public["roi_hi"] + 8 * SPS + 128
    assert side["comm_start"] != public["roi_lo"]
    for cell in CELLS:
        p = geometry(config(cell))
        assert 0 <= p["target_lo"] < p["target_hi"] <= p["n"]
    print("sic_fullpacket_sim self-test PASS (CRC, independent pulse operator, fields, truth isolation, paired IQ)")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--cell", choices=list(CELLS | EXTRA), default="sync_base")
    ap.add_argument("--matrix", action="store_true")
    ap.add_argument("--windows", type=int, default=16)
    ap.add_argument("--seed", type=int, default=2000000)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.windows < 1 or args.seed < 0:
        ap.error("windows must be positive and seed nonnegative")
    uwb = S.load_uwb(str(Path(args.build_dir).resolve()))
    if args.self_test:
        self_test(uwb)
        return
    if args.out is None:
        ap.error("--out is required unless --self-test")
    td = templates(uwb)
    if args.matrix:
        if args.out.exists() and any(args.out.iterdir()):
            raise FileExistsError(f"matrix output is not empty: {args.out}")
        args.out.mkdir(parents=True, exist_ok=True)
        summaries = [generate_case(uwb, args.out / cell, cell, args.windows, args.seed, td)
                     for cell in CELLS]
        dump_json(args.out / "matrix.json", dict(schema=SCHEMA, cells=list(CELLS),
                  windows_per_cell=args.windows, seed=args.seed,
                  formal_cartesian_matrix=False, case_count=len(summaries)))
    else:
        generate_case(uwb, args.out, args.cell, args.windows, args.seed, td)
    print(f"generated {args.out}")


if __name__ == "__main__":
    main()
