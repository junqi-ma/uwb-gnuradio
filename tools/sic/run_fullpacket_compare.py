#!/usr/bin/env python3
"""Sequential, provenance-bound full-packet versus preamble-only experiments.

This is orchestration, not an online signal processor. It may read sidecars and
hash source IQ after the C++ online runner has completed. Formal timing/paced
runs require exclusive benchmark lock; no build or parallel work is launched.
"""
from __future__ import annotations

import argparse
import contextlib
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
VERIFIER = ROOT / "tools/sic/verify_fullpacket_compare.py"
REPRESENTATIVE = ["sync_base", "payload_base", "payload_len16", "payload_len127",
                  "payload_snr8", "payload_isr0", "payload_isr20", "payload_cfo_m50k",
                  "payload_cfo_p50k", "payload_four_frac"]
DEFAULT_CELLS = {"smoke": ["sync_base", "sync_isr20", "payload_base", "payload_isr20"],
                 "functional": REPRESENTATIVE,
                 "timing": ["sync_base", "payload_base"],
                 "paced": ["sync_base", "payload_base"]}
LOCK = Path("/tmp/uwb-sic-benchmark.lock")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True,
                                    allow_nan=False) + "\n")


def git_output(*args):
    return subprocess.run(["git", "-C", str(ROOT), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout.strip()


def clean_env():
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    return env


def run_logged(command, log, env):
    # List arguments preserve exact bytes; never shell-interpolate filenames.
    with Path(log).open("w") as f:
        completed = subprocess.run([str(x) for x in command], cwd=ROOT, env=env,
                                   stdout=f, stderr=subprocess.STDOUT)
    return completed.returncode


def library_binding(binary, build, env):
    completed = subprocess.run(["ldd", str(binary)], env=env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               check=True)
    loaded = []
    for line in completed.stdout.splitlines():
        if "libgnuradio-uwb" in line and "=>" in line:
            candidate = line.split("=>", 1)[1].strip().split()[0]
            if candidate == "not":
                raise RuntimeError("C++ runner uwb library cannot be resolved")
            path = Path(candidate).resolve()
            if not path.is_relative_to(build.resolve()):
                raise RuntimeError(f"runner resolves uwb library outside trusted build: {path}")
            loaded.append({"path": str(path), "sha256": sha256(path)})
    if not loaded:
        raise RuntimeError("ldd did not identify the runner's uwb library")
    return completed.stdout, loaded


def provenance(binary, build, env, rx_pipeline, reconstruction):
    flags_file = build / "apps/CMakeFiles" / (binary.name + ".dir") / "flags.make"
    if not flags_file.is_file():
        raise RuntimeError(f"selected runner compile flags missing: {flags_file}")
    flags_text = flags_file.read_text()
    ldd, libraries = library_binding(binary, build, env)
    status = git_output("status", "--porcelain=v1", "--untracked-files=normal")
    cpu = "unknown"
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    sources = [ROOT / p for p in (
        "gr-uwb/apps/benchmark_fullpacket_compare.cc",
        "gr-uwb/apps/fullpacket_rx_pipeline.h",
        "gr-uwb/include/gnuradio/uwb/uwb_fullpacket_sic_core.h",
        "gr-uwb/lib/uwb_fullpacket_sic_core.cc",
        "gr-uwb/lib/uwb_preamble_sic_core.cc",
        "gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h",
        "gr-uwb/include/gnuradio/uwb/uwb_demod_core.h")]
    sources.extend([VERIFIER, Path(__file__).resolve()])
    return dict(schema="fullpacket-experiment-provenance/1",
                created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                repo=str(ROOT), git_head=git_output("rev-parse", "HEAD"),
                git_dirty=bool(status), git_status=status.splitlines(),
                build_dir=str(build), binary={"path": str(binary), "sha256": sha256(binary)},
                requested_rx_pipeline=rx_pipeline,
                requested_reconstruction=reconstruction,
                expected_rx_avx2=rx_pipeline == "fast",
                compile_flags=dict(path=str(flags_file), sha256=sha256(flags_file),
                                   content=flags_text),
                libraries=libraries, ldd=ldd,
                sources={str(p.relative_to(ROOT)): sha256(p) for p in sources},
                machine=dict(system=platform.platform(), cpu=cpu, cpu_count=os.cpu_count(),
                             affinity=sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None),
                environment={k: env.get(k) for k in (
                    "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VOLK_GENERIC")},
                ld_library_path_removed=True,
                full_wait_policy="whole_public_context",
                early_prefix_policy="public_observed_end_then_full_context_forward_apply",
                time_kind="stages: steady-clock service wall us; thread_cpu: CLOCK_THREAD_CPUTIME_ID total pipeline us",
                acquisition_profile=dict(pulse_shape="legacy", sic_coarse_stride=1, comm_timing_coarse_stride=1, sensing_timing_coarse_stride=1,
                                         rationale="legacy narrow-pulse correlation peaks require stride1 in the public 512-sample search ROI; gates .40/.25 preserved"),
                source_pool_policy="repeat raw IQ pool; model and decoded payload re-estimated per observation",
                lock_path=str(LOCK))


def case_directory(inputs, cell, selected_count):
    # A direct case directory is accepted for convenient one-cell reruns.
    if (inputs / "public.json").is_file():
        if selected_count != 1:
            raise ValueError("direct --inputs case requires exactly one --cell")
        declared = json.loads((inputs / "public.json").read_text()).get("cell")
        if declared != cell:
            raise ValueError(f"input cell {declared!r} does not match selected {cell!r}")
        return inputs
    return inputs / cell


def check_input(case):
    pub = json.loads((case / "public.json").read_text())
    windows = sorted(p for p in (case / "windows").iterdir() if p.is_dir())
    if not windows:
        raise ValueError(f"no input windows in {case}")
    hashes = {}
    for window in windows:
        for name in ("raw.cf32", "clean.cf32", "sensing.cf32", "interference.cf32", "sidecar.json"):
            if not (window / name).is_file():
                raise ValueError(f"incomplete source window {window}: missing {name}")
        if (window / "raw.cf32").stat().st_size != int(pub["n"]) * 8:
            raise ValueError(f"raw size mismatch in {window}")
        hashes[window.name] = sha256(window / "raw.cf32")
    actual_templates = {name: sha256(case / name) for name in (
        "comm_sync.cf32", "comm_preamble.cf32", "sensing_sync.cf32")}
    if actual_templates != pub.get("template_hashes"):
        raise ValueError(f"public template hash mismatch in {case}")
    return pub, windows, hashes, actual_templates


def check_rx_metadata(record, expected, label):
    """Bind selected executable to the kernel it actually reports using."""
    if record.get("rx_pipeline") != expected:
        raise RuntimeError(f"{label}: RX pipeline {record.get('rx_pipeline')!r} != requested {expected!r}")
    if type(record.get("rx_avx2")) is not bool or record["rx_avx2"] != (expected == "fast"):
        raise RuntimeError(f"{label}: RX AVX2/FMA metadata does not match requested {expected}")
    return {"rx_pipeline": record["rx_pipeline"], "rx_avx2": record["rx_avx2"]}


def check_reconstruction_metadata(record, expected, label):
    if record.get("reconstruction") != expected:
        raise RuntimeError(f"{label}: reconstruction {record.get('reconstruction')!r} != requested {expected!r}")
    return record["reconstruction"]


def annotate(run, source_windows, source_hashes, rx_pipeline, reconstruction):
    log = run / "windows.jsonl"
    original = run / "windows.runner.jsonl"
    log.rename(original)
    with original.open() as src, log.open("w") as dst:
        for line in src:
            row = json.loads(line)
            check_rx_metadata(row, rx_pipeline, f"{run.name}/row {row.get('window_id')}")
            check_reconstruction_metadata(row, reconstruction, f"{run.name}/row {row.get('window_id')}")
            source = row["source_window"]
            if not isinstance(source, int) or not 0 <= source < len(source_windows):
                raise ValueError("runner source_window outside declared pool")
            row["input_hash"] = source_hashes[source_windows[source].name]
            row["input_hash_added_offline"] = True
            dst.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")


def reference_summary(run, windows, mode):
    """Offline reference decode counts; SIC verifier deliberately excludes refs."""
    rows = [json.loads(line) for line in (run / "windows.jsonl").read_text().splitlines() if line]
    expected = {i: json.loads((w / "sidecar.json").read_text())["sensing_psdu_hex"].lower()
                for i, w in enumerate(windows)}
    errors, seen = [], set()
    fcs_count = exact_count = 0
    for row in rows:
        source = row.get("source_window")
        if row.get("mode") != mode or source not in expected:
            errors.append("unexpected reference mode/source")
            continue
        if source in seen:
            errors.append("duplicate reference source")
        seen.add(source)
        fcs_count += row.get("sensing_fcs") is True
        exact_count += (row.get("sensing_fcs") is True and
                        row.get("sensing_psdu_hex", "").lower() == expected[source])
    if len(rows) != len(windows) or len(seen) != len(windows):
        errors.append("reference source coverage mismatch")
    def rate(count):
        n = len(rows)
        if not n:
            return dict(successes=count, n=0, fraction=None, ci95=None)
        z = 1.959963984540054
        phat = count / n
        den = 1 + z*z/n
        center = (phat + z*z/(2*n)) / den
        half = z * math.sqrt(phat*(1-phat)/n + z*z/(4*n*n)) / den
        return dict(successes=int(count), n=n, fraction=phat,
                    ci95=[max(0., center-half), min(1., center+half)])
    stages = sorted({k for row in rows for k in row.get("times_us", {})})
    means = {k: sum(row.get("times_us", {}).get(k, 0) for row in rows) / len(rows)
             for k in stages} if rows else {}
    result = dict(schema="fullpacket-reference-summary/1", mode=mode,
                  structural_pass=not errors, errors=errors, rows=len(rows),
                  unique_sources=len(seen), sensing_fcs=rate(fcs_count), sensing_exact=rate(exact_count),
                  times_mean_us=means, iq_effects_verified=False,
                  verification_note="offline sidecar decode counts only; reference modes do not run SIC verifier")
    write_json(run / "reference_summary.json", result)
    return result


@contextlib.contextmanager
def benchmark_lock(required):
    if not required:
        yield
        return
    with LOCK.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("another benchmark owns /tmp/uwb-sic-benchmark.lock") from exc
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--inputs", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--kind", required=True, choices=DEFAULT_CELLS)
    parser.add_argument("--rx-pipeline", choices=("portable", "fast"), default="fast",
                        help="fast uses the mature native AVX2/FMA RX; portable keeps the baseline runner")
    parser.add_argument("--reconstruction", choices=("full", "payload_cached"), default="full",
                        help="full reference FIR or exact cached-prefix plus payload/boundary FIR")
    parser.add_argument("--cell", action="append", help="repeat to select multiple cells")
    parser.add_argument("--windows", type=int, help="input source windows when generating; existing cases reused intact")
    parser.add_argument("--seed", type=int, default=2000000)
    parser.add_argument("--observations", type=int, default=1024, help="timing observations per mode per round")
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--warmup", type=int, default=32)
    parser.add_argument("--rate", type=float, default=200)
    parser.add_argument("--seconds", type=float, default=10)
    args = parser.parse_args(argv)
    if min(args.observations, args.rounds) < 1 or args.warmup < 0 or args.rate <= 0 or args.seconds <= 0:
        parser.error("invalid timing/arrival budget")
    source_count = args.windows if args.windows is not None else (4 if args.kind == "smoke" else 16)
    if source_count < 1:
        parser.error("--windows must be positive")
    cells = args.cell or DEFAULT_CELLS[args.kind]
    if len(cells) != len(set(cells)):
        parser.error("duplicate --cell")
    inputs, out, build = args.inputs.resolve(), args.out.resolve(), args.build_dir.resolve()
    binary_name = ("benchmark_fullpacket_compare_fast_rx" if args.rx_pipeline == "fast"
                   else "benchmark_fullpacket_compare")
    binary = build / "apps" / binary_name
    if out.exists() and any(out.iterdir()):
        parser.error(f"output is not empty; choose a new run: {out}")
    if not binary.is_file():
        parser.error(f"runner missing: {binary}")
    env = clean_env()
    out.mkdir(parents=True, exist_ok=True)
    prov = provenance(binary, build, env, args.rx_pipeline, args.reconstruction)
    write_json(out / "provenance.json", prov)
    plans = []
    for cell in cells:
        case = case_directory(inputs, cell, len(cells))
        if not (case / "public.json").is_file():
            raise RuntimeError(f"missing case {case}; supply existing fixtures "
                               "or generate them on archive/gnuradio-adapter")
        pub, windows, hashes, templates = check_input(case)
        if len(windows) != source_count:
            print(f"reuse {cell}: {len(windows)} existing sources (requested generation size {source_count})", flush=True)
        plans.append((cell, case, pub, windows, hashes, templates))
    write_json(out / "experiment_plan.json", dict(kind=args.kind, rx_pipeline=args.rx_pipeline, reconstruction=args.reconstruction, cells=cells, seed=args.seed,
        source_windows_requested=source_count, observations=args.observations,
        rounds=args.rounds, warmup_per_mode_per_round=args.warmup,
        paced_rate=args.rate, paced_seconds=args.seconds,
        formal_cartesian_matrix=False, formal_lock=args.kind in ("timing", "paced"),
        cases=[dict(cell=c, path=str(p), public=pub, raw_hashes=h, template_hashes=t)
               for c,p,pub,w,h,t in plans]))
    results = []
    try:
        with benchmark_lock(args.kind in ("timing", "paced")):
            for cell, case, pub, windows, hashes, templates in plans:
                if args.kind == "timing":
                    jobs = [(f"round{r + 1}", "both", args.observations, args.warmup)
                            for r in range(args.rounds)]
                elif args.kind == "paced":
                    jobs = [(mode, mode, None, args.warmup) for mode in ("preamble", "full")]
                else:
                    jobs = [("paired", "both", None, 0),
                            ("reference_raw", "raw", None, 0),
                            ("reference_clean", "clean", None, 0)]
                for label, mode, count, warmup in jobs:
                    run = out / cell / label
                    command = [binary, "--case", case, "--out", run,
                               "--mode", mode, "--warmup", str(warmup),
                               "--reconstruction", args.reconstruction]
                    if count is not None:
                        command += ["--repeat", str(count)]
                    if args.kind == "paced":
                        command += ["--paced-rate", str(args.rate), "--paced-seconds", str(args.seconds)]
                    if args.kind in ("smoke", "functional") and mode == "both":
                        command += ["--dump-iq"]
                    run.mkdir(parents=True)
                    command_record = dict(argv=[str(x) for x in command], cwd=str(ROOT),
                        input_case=str(case), source_count=len(windows), template_hashes=templates,
                        public_profile=pub, provenance_file=str(out / "provenance.json"),
                        requested_rx_pipeline=args.rx_pipeline,
                        requested_reconstruction=args.reconstruction,
                        expected_rx_avx2=args.rx_pipeline == "fast",
                        service_observations_are_unique_packets=args.kind not in ("timing", "paced"),
                        full_context_arrival_policy="whole_public_context")
                    write_json(run / "command.json", command_record)
                    rc = run_logged(command, run / "runner.log", env)
                    item = dict(cell=cell, label=label, run=str(run), runner_rc=rc)
                    if rc == 0:
                        actual_hashes = ({w.name: sha256(w / "clean.cf32") for w in windows}
                                         if mode == "clean" else hashes)
                        runner_summary = json.loads((run / "runner_summary.json").read_text())
                        item["rx_kernel"] = check_rx_metadata(runner_summary, args.rx_pipeline,
                                                              str(run / "runner_summary.json"))
                        item["reconstruction"] = check_reconstruction_metadata(
                            runner_summary, args.reconstruction, str(run / "runner_summary.json"))
                        annotate(run, windows, actual_hashes, args.rx_pipeline, args.reconstruction)
                        if mode in ("raw", "clean"):
                            item["reference_summary"] = reference_summary(run, windows, mode)
                        else:
                            verify_command = [sys.executable, VERIFIER, "--case", case, "--run", run, "--require-provenance"]
                            if args.kind in ("timing", "paced"):
                                verify_command += ["--summary-only"]
                            vrc = run_logged(verify_command, run / "verify.log", env)
                            item["verifier_rc"] = vrc
                            result_path = run / "verification.json"
                            if result_path.exists():
                                verification = json.loads(result_path.read_text())
                                # Detailed windows stay in per-job verification.json.
                                item["verification"] = {k: v for k, v in verification.items() if k != "windows"}
                        item["runner_summary"] = runner_summary
                    results.append(item)
                    write_json(out / "experiment_summary.json", dict(
                        schema="fullpacket-experiment/1", kind=args.kind, rx_pipeline=args.rx_pipeline, reconstruction=args.reconstruction,
                        complete=False, results=results, provenance=str(out / "provenance.json")))
                    print(f"{cell}/{label}: runner={rc} verifier={item.get('verifier_rc', 'not-run')}", flush=True)
    except BaseException:
        write_json(out / "experiment_summary.json", dict(
            schema="fullpacket-experiment/1", kind=args.kind, rx_pipeline=args.rx_pipeline, reconstruction=args.reconstruction,
            complete=False, results=results, provenance=str(out / "provenance.json")))
        raise
    passed = all(r["runner_rc"] == 0 and
                 (r.get("reference_summary", {}).get("structural_pass") is True
                  if "reference_summary" in r else r.get("verifier_rc") == 0)
                 for r in results)
    write_json(out / "experiment_summary.json", dict(schema="fullpacket-experiment/1",
        kind=args.kind, rx_pipeline=args.rx_pipeline, reconstruction=args.reconstruction, complete=True, passed=passed, results=results,
        provenance=str(out / "provenance.json"),
        repeated_exposure_note="timing/paced reuse a finite raw IQ pool; do not treat observations as independent packets",
        rate_aggregation="per-run verifier unique_source_rates; repeated rounds must not be pooled as independent trials"))
    return 0 if passed else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"experiment failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
