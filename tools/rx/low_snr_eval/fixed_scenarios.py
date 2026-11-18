#!/usr/bin/env python3
"""Generate and compare the 6x40 fixed-window low-SNR/CFO/SFO scenarios.

Scenarios match the 2026-11-18 low-SNR boundary review fixtures:
  control  : +4 dB, seed 1729/11729, 0 Hz CFO, 0 ppm SFO
  seed1729 : -12 dB, seed 1729/11729, 0 Hz CFO, 0 ppm SFO
  seed2718 : -12 dB, seed 2718/12718, 0 Hz CFO, 0 ppm SFO
  cfo10k   : -12 dB, seed 1729/11729, +10 kHz CFO, 0 ppm SFO
  cfo40k   : -12 dB, seed 1729/11729, +40 kHz CFO, 0 ppm SFO
  sfo27ppm : -12 dB, seed 1729/11729, 0 Hz CFO, +27 ppm SFO (37038/37037)
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile

import numpy as np
from scipy.signal import resample_poly

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "rx"))
import proto_low_snr_signal as sig  # noqa: E402

SLOT_NATIVE = 737280
PACKET_COUNT = 41  # generate 41, evaluate sequence indices 1..40
PKT_WORK = 444352  # modulate_one length for 127 B / 256 SYNC / 4z2

SCENARIOS = [
    ("control", 1729, 4.0, 0, False, 40),
    ("seed1729", 1729, -12.0, 0, False, 38),
    ("seed2718", 2718, -12.0, 0, False, 40),
    ("cfo10k", 1729, -12.0, 10000, False, 37),
    ("cfo40k", 1729, -12.0, 40000, False, 38),
    ("sfo27ppm", 1729, -12.0, 0, True, 37),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def find_build_dir(explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit
    for cand in (
        "gr-uwb/build-baseline-a",
        "gr-uwb/build-refactor-b",
        "gr-uwb/build-native-only",
        "gr-uwb/build-native",
    ):
        p = REPO / cand
        if (p / "include").exists():
            return p
    raise RuntimeError(
        "No build directory with generated headers found; pass --build-dir"
    )


def generate_fixtures(fixtures_dir: Path, build_dir: Path, force: bool = False) -> dict:
    fixtures_dir.mkdir(parents=True, exist_ok=True)
    needed = []
    for name, seed, *_ in SCENARIOS:
        if (
            force
            or not (fixtures_dir / f"{name}.sc16").exists()
            or not (fixtures_dir / f"{name}-starts.txt").exists()
            or not (fixtures_dir / f"truth-{seed}.txt").exists()
        ):
            needed.append(name)

    if not needed:
        manifest = {
            "fixtures_dir": str(fixtures_dir),
            "generated": False,
            "cases": {
                name: {
                    "sc16_sha256": sha256(fixtures_dir / f"{name}.sc16"),
                    "starts_sha256": sha256(fixtures_dir / f"{name}-starts.txt"),
                    "truth_sha256": sha256(fixtures_dir / f"truth-{seed}.txt"),
                }
                for name, seed, *_ in SCENARIOS
            },
        }
        return manifest

    gen_src = (REPO / "tools/rx/low_snr_eval/gen_full_frame.cc").read_text()
    needle = "        gr::uwb::mod::append_ieee_fcs(wire);"
    if gen_src.count(needle) != 1:
        raise RuntimeError("Unexpected gen_full_frame.cc format")
    gen_src = gen_src.replace(
        needle,
        needle
        + """
        for (const auto byte : wire)
            std::printf("%02x", static_cast<unsigned>(byte));
        std::printf("\\n");""",
    )

    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    manifest = {"fixtures_dir": str(fixtures_dir), "generated": True, "cases": {}}

    with tempfile.TemporaryDirectory(prefix="uwb-gen-6x40-") as tmp:
        tmp_dir = Path(tmp)
        src_path = tmp_dir / "gen_truth.cc"
        bin_path = tmp_dir / "gen_truth"
        src_path.write_text(gen_src)
        subprocess.run(
            [
                "c++",
                "-std=c++17",
                "-O2",
                "-I" + str(REPO / "gr-uwb/include"),
                "-I" + str(build_dir / "include"),
                str(src_path),
                "-lvolk",
                "-o",
                str(bin_path),
            ],
            env=env,
            check=True,
        )

        for name, seed, snr, cfo, sfo, _ in SCENARIOS:
            base = tmp_dir / f"work-{seed}.cf32"
            truth = fixtures_dir / f"truth-{seed}.txt"
            if not base.exists():
                done = subprocess.run(
                    [
                        str(bin_path),
                        "--packets",
                        str(PACKET_COUNT),
                        "--seed",
                        str(seed),
                        "--out",
                        str(base),
                    ],
                    env=env,
                    text=True,
                    capture_output=True,
                    check=True,
                )
                lines = done.stdout.splitlines()[:PACKET_COUNT]
                if len(lines) != PACKET_COUNT:
                    raise RuntimeError(f"Expected {PACKET_COUNT} truth lines, got {len(lines)}")
                truth.write_text("\n".join(lines) + "\n")

            iq_path = fixtures_dir / f"{name}.sc16"
            starts_path = fixtures_dir / f"{name}-starts.txt"
            up, down = (37038, 37037) if sfo else (1, 1)

            if force or not iq_path.exists():
                x = np.fromfile(base, dtype=np.complex64)
                x *= 542.0 / float(np.sqrt(np.mean(np.abs(x[:PKT_WORK]) ** 2)))
                x = sig.resample_to_native(x)
                if sfo:
                    x = resample_poly(x, up, down)
                if cfo:
                    for start in range(0, x.size, 1_000_000):
                        stop = min(x.size, start + 1_000_000)
                        phase = 2.0 * np.pi * cfo / sig.FS_NATIVE * np.arange(start, stop)
                        x[start:stop] *= np.exp(1j * phase).astype(np.complex64)
                x = sig.add_awgn(x, snr, seed=seed + 10000)
                sig.quantize_sc16(x).tofile(iq_path)
                del x

            if force or not starts_path.exists():
                starts_path.write_text(
                    "".join(
                        f"{round(i * SLOT_NATIVE * up / down)}\n"
                        for i in range(1, PACKET_COUNT)
                    )
                )

            manifest["cases"][name] = {
                "snr_nominal_db": snr,
                "payload_seed": seed,
                "noise_seed": seed + 10000,
                "cfo_hz": cfo,
                "sfo_ppm": (up / down - 1.0) * 1e6,
                "sc16_sha256": sha256(iq_path),
                "starts_sha256": sha256(starts_path),
                "truth_sha256": sha256(truth),
            }
    return manifest


def compare_decoders(
    decoder_a: Path,
    decoder_b: Path,
    fixtures_dir: Path,
    out_path: Path,
    check_expected: bool = True,
) -> dict:
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    data = REPO / "testdata"
    tmpl = data / "reference_preamble_code10_998p4.cf32"
    taps = data / "resampler_65_48/taps_realtime_eq16.txt"

    summary = {
        "binaries": {
            "A": {"path": str(decoder_a), "sha256": sha256(decoder_a)},
            "B": {"path": str(decoder_b), "sha256": sha256(decoder_b)},
        },
        "fixtures_dir": str(fixtures_dir),
        "conditions": (
            "Native SC16; 1 ms; 127 bytes/code10/256 SYNC/4z2; "
            "noise seed = payload seed + 10000; sequence indices 1..40; "
            "pre_guard=36864 window=450000; eq16"
        ),
        "cases": {},
    }

    failures = []
    for name, seed, snr, cfo, sfo, expected_exact in SCENARIOS:
        iq = fixtures_dir / f"{name}.sc16"
        starts = fixtures_dir / f"{name}-starts.txt"
        truth_path = fixtures_dir / f"truth-{seed}.txt"
        truth_hex = truth_path.read_text().splitlines()
        if len(truth_hex) != PACKET_COUNT:
            raise RuntimeError(f"{truth_path}: expected {PACKET_COUNT} lines, got {len(truth_hex)}")
        eval_truth = truth_hex[1:]

        rows_by_side = []
        for side, binary in (("A", decoder_a), ("B", decoder_b)):
            cmd = [
                str(binary),
                str(iq),
                str(starts),
                str(tmpl),
                str(taps),
                "256",
                "4z2",
                "36864",
                "450000",
            ]
            done = subprocess.run(
                cmd, env=env, check=True, text=True, capture_output=True
            )
            rows = [json.loads(line) for line in done.stdout.splitlines()]
            if len(rows) != PACKET_COUNT - 1:
                raise RuntimeError(f"{side}/{name}: expected 40 rows, got {len(rows)}")
            for idx, row in enumerate(rows):
                if row.get("id") != idx or "status" not in row:
                    raise RuntimeError(f"{side}/{name}: invalid row at index {idx}")
                row.pop("demod_us", None)
            rows_by_side.append(rows)

        rows_a, rows_b = rows_by_side
        differences = [i for i, (ra, rb) in enumerate(zip(rows_a, rows_b)) if ra != rb]

        def metrics_for(rows):
            exact = sum(bool(r["fcs"]) and r.get("hex") == t for r, t in zip(rows, eval_truth))
            wrong = sum(bool(r["fcs"]) and r.get("hex") != t for r, t in zip(rows, eval_truth))
            est = [r["cfo_hz"] for r in rows if r.get("cfo_points")]
            return {
                "frames": len(rows),
                "status_counts": dict(collections.Counter(str(r["status"]) for r in rows)),
                "exact_psdu_fcs": exact,
                "wrong_psdu_fcs": wrong,
                "sfd_ok": sum(int(bool(r.get("sfd_ok", 0))) for r in rows),
                "median_cfo_est_hz": float(statistics.median(est)) if est else None,
            }

        up, down = (37038, 37037) if sfo else (1, 1)
        m_a = metrics_for(rows_a)
        m_b = metrics_for(rows_b)
        case_rec = {
            "snr_nominal_db": snr,
            "payload_seed": seed,
            "noise_seed": seed + 10000,
            "cfo_hz": cfo,
            "sfo_ppm": (up / down - 1.0) * 1e6,
            "sha256": sha256(iq),
            "starts_sha256": sha256(starts),
            "truth_sha256": sha256(truth_path),
            "expected_exact_psdu_fcs": expected_exact,
            "A": m_a,
            "B": m_b,
            "differences": differences,
        }
        summary["cases"][name] = case_rec
        print(
            f"{name:10s} exact A={m_a['exact_psdu_fcs']}/40 B={m_b['exact_psdu_fcs']}/40 "
            f"wrong={m_b['wrong_psdu_fcs']} diff={len(differences)}",
            flush=True,
        )
        if differences:
            failures.append(f"{name}: A and B differ at frames {differences}")
        if m_a["wrong_psdu_fcs"] != 0 or m_b["wrong_psdu_fcs"] != 0:
            failures.append(f"{name}: wrong PSDU passed FCS")
        if check_expected and m_b["exact_psdu_fcs"] != expected_exact:
            failures.append(
                f"{name}: expected exact_psdu_fcs={expected_exact}, got {m_b['exact_psdu_fcs']}"
            )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print("wrote", out_path)
    if failures:
        raise SystemExit("; ".join(failures))
    return summary


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("decoders", nargs="*", help="DECODER_A DECODER_B")
    ap.add_argument("--fixtures", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--generate", action="store_true", help="generate missing 6x40 fixtures")
    ap.add_argument("--force", action="store_true", help="regenerate existing 6x40 fixtures")
    ap.add_argument("--build-dir", type=Path, default=None, help="build dir with generated headers")
    ap.add_argument(
        "--allow-regression",
        action="store_true",
        help="do not fail if exact_psdu_fcs differs from the frozen baseline count",
    )
    args = ap.parse_args()

    if args.generate:
        build_dir = find_build_dir(args.build_dir)
        manifest = generate_fixtures(args.fixtures, build_dir, force=args.force)
        if args.out is not None and not args.decoders:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(manifest, indent=2) + "\n")
            print("wrote", args.out)
        elif not args.decoders:
            print(json.dumps(manifest, indent=2))

    if args.decoders:
        if len(args.decoders) != 2:
            ap.error("provide both DECODER_A and DECODER_B")
        if args.out is None:
            ap.error("--out is required when comparing decoders")
        compare_decoders(
            Path(args.decoders[0]),
            Path(args.decoders[1]),
            args.fixtures,
            args.out,
            check_expected=not args.allow_regression,
        )
    elif not args.generate:
        ap.error("provide DECODER_A DECODER_B or --generate")


if __name__ == "__main__":
    main()
