#!/usr/bin/env python3
"""Benchmark OpenCode Go (OpenAI-compatible) streaming token speed.

Measures, per run:
  - TTFT            time to first streamed token (reasoning or content)
  - latency         total wall time of the request
  - output tokens   from the API `usage` block (includes reasoning tokens)
  - gen tok/s       output tokens / (latency - TTFT), i.e. pure decode speed
  - total tok/s     output tokens / latency, i.e. end-to-end speed

The API key is read from $OPENCODE_API_KEY or, failing that, from the local
OpenCode credential database. It is never written to disk by this script.

Usage:
    python3 bench_token_speed.py
    python3 bench_token_speed.py --model deepseek-v4.1-flash --runs 5
    python3 bench_token_speed.py --model muse-spark-1.3-contributor --runs 3 --json
"""
from __future__ import annotations

import argparse
import json
import os
import sqlite3
import statistics
import sys
import time
import urllib.error
import urllib.request
import uuid

DEFAULT_BASE = "https://opencode.ai/zen/go/v1"
DEFAULT_DB = os.path.expanduser("~/.local/share/opencode/opencode.db")
DEFAULT_MODEL = "deepseek-v4.1-flash"
DEFAULT_PROMPT = (
    "Write a detailed technical explanation of how a sliding-window "
    "energy detector works for UWB packet detection. Aim for a few "
    "hundred words."
)
USER_AGENT = "opencode-bench/1.0"


def get_api_key(explicit: str | None) -> str:
    if explicit:
        return explicit
    env = os.environ.get("OPENCODE_API_KEY")
    if env:
        return env.strip()
    db = os.environ.get("OPENCODE_DB", DEFAULT_DB)
    if not os.path.exists(db):
        sys.exit(f"No OPENCODE_API_KEY and no credential db at {db}")
    con = sqlite3.connect(db)
    row = con.execute(
        "select value from credential where integration_id='opencode-go' "
        "order by time_updated desc limit 1"
    ).fetchone()
    if not row:
        sys.exit("No 'opencode-go' credential found in the OpenCode db")
    return json.loads(row[0])["key"]


def stream_once(base: str, key: str, model: str, prompt: str, max_tokens: int,
                temperature: float | None) -> dict:
    payload = {
        "model": model,
        "messages": [{"role": "user", "content": prompt}],
        "max_tokens": max_tokens,
        "stream": True,
        "stream_options": {"include_usage": True},
    }
    if temperature is not None:
        payload["temperature"] = temperature

    headers = {
        "Authorization": f"Bearer {key}",
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
        "User-Agent": USER_AGENT,
        # Required by the OpenCode Go gateway for efficient routing.
        "x-opencode-session": str(uuid.uuid4()),
    }
    req = urllib.request.Request(
        base.rstrip("/") + "/chat/completions",
        data=json.dumps(payload).encode(),
        headers=headers,
        method="POST",
    )

    t0 = time.perf_counter()
    ttft = None
    usage = None
    n_content = 0
    n_reasoning = 0
    n_chunks = 0
    first_byte = None

    try:
        with urllib.request.urlopen(req, timeout=300) as resp:
            for raw in resp:
                if first_byte is None:
                    first_byte = time.perf_counter()
                line = raw.decode("utf-8", "replace").strip()
                if not line or not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if data == "[DONE]":
                    break
                try:
                    chunk = json.loads(data)
                except json.JSONDecodeError:
                    continue
                if chunk.get("usage"):
                    usage = chunk["usage"]
                for choice in chunk.get("choices") or []:
                    delta = choice.get("delta") or {}
                    piece = delta.get("content")
                    reason = delta.get("reasoning_content") or delta.get("reasoning")
                    produced = False
                    if piece:
                        n_content += 1
                        produced = True
                    if reason:
                        n_reasoning += 1
                        produced = True
                    if produced:
                        n_chunks += 1
                        if ttft is None:
                            ttft = time.perf_counter()
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", "replace")
        raise RuntimeError(f"HTTP {e.code}: {body[:400]}") from None
    except urllib.error.URLError as e:
        raise RuntimeError(f"connection error: {e.reason}") from None

    t_end = time.perf_counter()
    latency = t_end - t0
    t_first = ttft if ttft is not None else t_end
    gen_time = max(t_end - t_first, 1e-9)

    out_tokens = None
    reasoning_tokens = None
    if usage:
        out_tokens = usage.get("completion_tokens")
        detail = usage.get("completion_tokens_details") or {}
        reasoning_tokens = detail.get("reasoning_tokens")
    if out_tokens is None:
        # Fall back to chunk count if the gateway omitted usage.
        out_tokens = n_chunks

    return {
        "ttft_s": None if ttft is None else ttft - t0,
        "first_byte_s": None if first_byte is None else first_byte - t0,
        "latency_s": latency,
        "output_tokens": out_tokens,
        "reasoning_tokens": reasoning_tokens,
        "gen_tokens_per_s": (out_tokens / gen_time) if out_tokens else None,
        "total_tokens_per_s": (out_tokens / latency) if out_tokens else None,
        "chunks": n_chunks,
    }


def summarize(runs: list[dict]) -> dict:
    def col(name):
        return [r[name] for r in runs if r.get(name) is not None]

    def stats(name):
        vals = col(name)
        if not vals:
            return None
        return {
            "mean": statistics.mean(vals),
            "median": statistics.median(vals),
            "min": min(vals),
            "max": max(vals),
        }

    return {
        "ttft_s": stats("ttft_s"),
        "latency_s": stats("latency_s"),
        "output_tokens": stats("output_tokens"),
        "gen_tokens_per_s": stats("gen_tokens_per_s"),
        "total_tokens_per_s": stats("total_tokens_per_s"),
    }


def fmt(v, unit="", nd=2):
    return "  n/a" if v is None else f"{v:.{nd}f}{unit}"


def print_report(model, runs, summary):
    print(f"\n=== OpenCode Go token-speed benchmark ===")
    print(f"model: {model}   runs: {len(runs)}\n")
    header = (f"{'run':>3} {'TTFT(s)':>8} {'lat(s)':>8} "
              f"{'out_tok':>8} {'gen tok/s':>10} {'e2e tok/s':>10}")
    print(header)
    print("-" * len(header))
    for i, r in enumerate(runs, 1):
        print(f"{i:>3} {fmt(r['ttft_s']):>8} {fmt(r['latency_s']):>8} "
              f"{str(r['output_tokens']):>8} "
              f"{fmt(r['gen_tokens_per_s']):>10} "
              f"{fmt(r['total_tokens_per_s']):>10}")
    print("-" * len(header))
    g = summary["gen_tokens_per_s"]
    if g:
        print(f"\ndecode speed (gen tok/s):  mean {g['mean']:.1f} | "
              f"median {g['median']:.1f} | min {g['min']:.1f} | max {g['max']:.1f}")
    t = summary["ttft_s"]
    if t:
        print(f"TTFT (s):                  mean {t['mean']:.3f} | "
              f"median {t['median']:.3f} | min {t['min']:.3f} | max {t['max']:.3f}")
    e = summary["total_tokens_per_s"]
    if e:
        print(f"end-to-end (e2e tok/s):    mean {e['mean']:.1f} | "
              f"median {e['median']:.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default=os.environ.get("OPENCODE_GO_BASE", DEFAULT_BASE))
    ap.add_argument("--api-key", default=None,
                    help="override; default $OPENCODE_API_KEY or the OpenCode db")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--max-tokens", type=int, default=512)
    ap.add_argument("--prompt", default=DEFAULT_PROMPT)
    ap.add_argument("--temperature", type=float, default=None)
    ap.add_argument("--warmup", type=int, default=1,
                    help="warmup requests excluded from stats (default 1)")
    ap.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = ap.parse_args()

    key = get_api_key(args.api_key)

    for _ in range(max(args.warmup, 0)):
        try:
            stream_once(args.base, key, args.model, "ping", 8, args.temperature)
        except RuntimeError as e:
            sys.exit(f"warmup failed: {e}")

    runs = []
    for i in range(args.runs):
        while True:
            try:
                runs.append(stream_once(args.base, key, args.model, args.prompt,
                                        args.max_tokens, args.temperature))
                break
            except RuntimeError as e:
                msg = str(e)
                if "rate" in msg.lower() or "429" in msg:
                    print(f"run {i+1}: rate limited, retrying in 3s...", file=sys.stderr)
                    time.sleep(3)
                    continue
                sys.exit(f"run {i+1} failed: {e}")
        if not args.json:
            print(f"run {i+1}: TTFT {fmt(runs[-1]['ttft_s'],'s',3)}  "
                  f"latency {fmt(runs[-1]['latency_s'],'s',2)}  "
                  f"out_tokens {runs[-1]['output_tokens']}  "
                  f"gen {fmt(runs[-1]['gen_tokens_per_s'],' tok/s')}",
                  file=sys.stderr)

    summary = summarize(runs)
    if args.json:
        print(json.dumps({"model": args.model, "runs": runs, "summary": summary},
                         indent=2))
    else:
        print_report(args.model, runs, summary)


if __name__ == "__main__":
    main()
