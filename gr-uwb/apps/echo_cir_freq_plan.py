#!/usr/bin/env python3
"""Frequency plan / manual command parsing for the X410 echo-CIR sweep app.

This module deliberately has **no** GNU Radio, UHD or numpy dependency so it
can be unit-tested on a bare host (see ``test_freq_plan.py``).  The RF side of
the sweep lives in ``x410_cg400_hrp_echo_cir_sweep.py``.

Modes
-----
fixed
    Always the nominal centre frequency (base-app behaviour).
scan
    ``start + i*step`` for ``i = 0..n-1`` while ``<= stop``; ``dwell`` pulses
    per point.  ``once`` stops after one sweep, otherwise it wraps.
manual
    A background thread reads commands from stdin; the latest command takes
    effect at the next burst boundary and is held until changed.

Command grammar (manual mode)
-----------------------------
    +50              offset from nominal in the default unit (kHz by default)
    6489600kHz       absolute
    6489.6MHz        absolute
    f 6489.6e6       absolute (explicit)
    off +2.5MHz      offset (explicit)
    status | ?       print current target
    q | quit | exit  stop the run

Bare numbers (no unit suffix, no exponent) use ``--freq-unit`` (default
``khz``): ``+50`` -> +50 kHz, ``491`` -> 491 kHz.  Scientific notation is
always Hz (``-10e6`` -> -10 MHz).  Units ``GHz/MHz/kHz/Hz`` are accepted
case-insensitively and override the default.
"""
from __future__ import annotations

import json
import math
import os
import queue
import re
import sys
import threading


_FREQ_RE = re.compile(
    r"^([+-]?)(\d+(?:\.\d*)?|\.\d+)(?:[eE]([+-]?\d+))?\s*([a-zA-Z]*)$")

# Unit used for a bare manual-mode number (no suffix, no exponent).
DEFAULT_FREQ_UNIT = "khz"
FREQ_UNIT_CHOICES = ("hz", "khz", "mhz", "ghz")
_UNIT_SCALE = {"hz": 1.0, "khz": 1e3, "mhz": 1e6, "ghz": 1e9}


def parse_freq_value(token, default_unit=DEFAULT_FREQ_UNIT):
    """Parse a bare/token number with an optional unit into Hz.

    ``default_unit`` applies only when the token has no unit suffix and no
    exponent (scientific notation is always Hz).  Raises ValueError on
    anything that is not a number(+unit).
    """
    m = _FREQ_RE.match(str(token).strip())
    if not m:
        raise ValueError("cannot parse frequency %r" % (token,))
    if default_unit not in _UNIT_SCALE:
        raise ValueError("unknown default unit %r" % (default_unit,))
    sign, mant, exp, unit = m.groups()
    val = float(mant + ("e" + exp if exp else ""))
    if sign == "-":
        val = -val
    u = unit.lower()
    if u == "":
        # No suffix: scientific notation is Hz, otherwise the default unit.
        u = "hz" if exp is not None else default_unit
    if u not in _UNIT_SCALE:
        raise ValueError("unknown frequency unit %r" % (unit,))
    return val * _UNIT_SCALE[u]


def parse_freq_command(line, nominal_hz, default_unit=DEFAULT_FREQ_UNIT):
    """Parse one manual-mode line.

    Returns ``(action, value)`` where action is ``set`` (value = absolute Hz),
    ``delta`` (value = Hz relative to nominal), ``status`` or ``stop``.
    Raises ValueError for malformed input.
    """
    s = str(line).strip()
    if not s:
        return ("status", None)
    low = s.lower()
    if low in ("q", "quit", "exit"):
        return ("stop", None)
    if low in ("status", "?", "s", "freq", "f?"):
        return ("status", None)
    if low.startswith("f "):
        return ("set", parse_freq_value(s[2:].strip(), default_unit))
    if low.startswith("freq "):
        return ("set", parse_freq_value(s[5:].strip(), default_unit))
    if low.startswith("off "):
        return ("delta", parse_freq_value(s[4:].strip(), default_unit))
    if low.startswith("offset "):
        return ("delta", parse_freq_value(s[7:].strip(), default_unit))
    if s[0] in "+-":
        return ("delta", parse_freq_value(s, default_unit))
    return ("set", parse_freq_value(s, default_unit))


class FreqPlan:
    """Decide the centre frequency for every pulse id."""

    def __init__(self, mode, nominal_hz, start_hz=None, stop_hz=None,
                 step_hz=0.0, dwell=1, once=True, manual_q=None,
                 freq_unit=DEFAULT_FREQ_UNIT):
        if freq_unit not in _UNIT_SCALE:
            raise SystemExit("unknown --freq-unit %r" % (freq_unit,))
        self.mode = str(mode)
        self.nominal = float(nominal_hz)
        self.dwell = max(1, int(dwell))
        self.once = bool(once)
        self._manual_q = manual_q
        self._manual_freq = float(nominal_hz)
        self.freq_unit = str(freq_unit)
        self.stop_requested = False
        self.freqs = []
        self.start = None
        self.stop = None

        if self.mode == "fixed":
            self.freqs = [self.nominal]
        elif self.mode == "scan":
            self.start = self.nominal if start_hz is None else float(start_hz)
            if stop_hz is None:
                raise SystemExit("--freq-mode scan requires --freq-stop")
            if not (step_hz and float(step_hz) > 0.0):
                raise SystemExit("--freq-mode scan requires --freq-step > 0")
            self.stop = float(stop_hz)
            if self.stop < self.start:
                raise SystemExit("--freq-stop must be >= --freq-start")
            n = int(math.floor((self.stop - self.start) / float(step_hz)
                               + 1e-9)) + 1
            self.freqs = [self.start + i * float(step_hz) for i in range(n)]
        elif self.mode == "manual":
            self.freqs = [self.nominal]
        else:
            raise SystemExit("unknown --freq-mode %r" % self.mode)

        for f in self.freqs:
            if not (f > 0.0):
                raise SystemExit("frequency plan has non-positive entry %r" % f)

    @property
    def n_points(self):
        return len(self.freqs)

    def total_pulses(self):
        """Fixed length of a single scan sweep, else None."""
        if self.mode == "scan" and self.once:
            return self.n_points * self.dwell
        return None

    def current(self):
        if self.mode == "manual":
            return self._manual_freq
        return self.freqs[0]

    def freq_for(self, pulse_id):
        if self.mode == "manual":
            self._drain_manual()
            return self._manual_freq
        if self.mode == "fixed":
            return self.freqs[0]
        idx = int(pulse_id) // self.dwell
        if self.once:
            idx = min(idx, self.n_points - 1)
        else:
            idx %= self.n_points
        return self.freqs[idx]

    def dwell_index(self, pulse_id):
        if self.mode == "scan":
            return int(pulse_id) // self.dwell
        return 0

    def _drain_manual(self):
        if self._manual_q is None:
            return
        while True:
            try:
                line = self._manual_q.get_nowait()
            except queue.Empty:
                break
            if line is None:
                self.stop_requested = True
                break
            try:
                action, val = parse_freq_command(
                    line, self.nominal, self.freq_unit)
            except ValueError as exc:
                print("[freq] %s" % exc, flush=True)
                continue
            if action == "stop":
                self.stop_requested = True
                print("[freq] stop requested", flush=True)
                break
            if action == "status":
                print("[freq] current=%.3f kHz offset=%+.3f kHz (unit=%s)"
                      % (self._manual_freq / 1e3,
                         (self._manual_freq - self.nominal) / 1e3,
                         self.freq_unit),
                      flush=True)
                continue
            new = self.nominal + val if action == "delta" else val
            if not (new > 0.0):
                print("[freq] ignore non-positive target %.3f kHz"
                      % (new / 1e3), flush=True)
                continue
            self._manual_freq = new
            print("[freq] manual -> %.3f kHz (offset %+.3f kHz)"
                  % (new / 1e3, (new - self.nominal) / 1e3), flush=True)


def start_manual_reader(q):
    """Daemon thread: stdin lines -> queue; EOF pushes None (stop)."""

    def loop():
        try:
            for line in sys.stdin:
                q.put(line.rstrip("\n"))
        except Exception:
            pass
        finally:
            q.put(None)

    th = threading.Thread(target=loop, name="freq_stdin", daemon=True)
    th.start()
    return th


def iter_jsonl(path):
    if not path or not os.path.isfile(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                yield json.loads(ln)


def analyze_cir_by_freq(cir_jsonl, freq_records):
    """Join cir.jsonl with the per-pulse frequency log and aggregate.

    ``freq_records`` is the list of dicts from the sweep echo block, each with
    ``pulse_id`` and ``freq_hz``.  Returns a list sorted by frequency.
    """
    by_pulse = {}
    for r in freq_records or []:
        try:
            by_pulse[int(r["pulse_id"])] = r
        except (KeyError, TypeError, ValueError):
            continue

    groups = {}
    order = []
    for rec in iter_jsonl(cir_jsonl):
        pid = rec.get("pulse_id")
        fr = None
        if pid is not None:
            try:
                fr = by_pulse.get(int(pid))
            except (TypeError, ValueError):
                fr = None
        key = None if fr is None else round(float(fr["freq_hz"]), 3)
        g = groups.get(key)
        if g is None:
            g = {
                "freq_hz": None if key is None else float(key),
                "freq_offset_hz": None if fr is None
                else float(fr["freq_offset_hz"]),
                "n": 0, "ok": 0, "fail": 0,
                "status_hist": {},
                "_peak_sum": 0.0, "_peak_n": 0,
                "_metric_sum": 0.0, "_metric_n": 0, "metric_max": 0.0,
            }
            groups[key] = g
            order.append(key)
        g["n"] += 1
        st = rec.get("status", "unknown")
        g["status_hist"][st] = g["status_hist"].get(st, 0) + 1
        if st == "ok":
            g["ok"] += 1
            if "peak_tap" in rec:
                g["_peak_sum"] += float(rec["peak_tap"])
                g["_peak_n"] += 1
            if "cir_peak_metric" in rec:
                m = float(rec["cir_peak_metric"])
                g["_metric_sum"] += m
                g["_metric_n"] += 1
                g["metric_max"] = max(g["metric_max"], m)
        else:
            g["fail"] += 1

    out = []
    for key in order:
        g = groups[key]
        g["peak_tap_mean"] = (g.pop("_peak_sum") / g["_peak_n"]
                              if g["_peak_n"] else None)
        g["metric_mean"] = (g.pop("_metric_sum") / g["_metric_n"]
                            if g["_metric_n"] else None)
        g.pop("_peak_n")
        g.pop("_metric_n")
        out.append(g)
    out.sort(key=lambda x: (x["freq_hz"] is None, x["freq_hz"] or 0.0))
    return out
