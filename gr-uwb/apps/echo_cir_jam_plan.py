#!/usr/bin/env python3
"""Jamming plan / geometry helpers for the X410 echo-CIR + HRP jammer app.

This module is **pure Python** (numpy only).  It deliberately does *not*
import GNU Radio, pmt or UHD, so it can be unit-tested on a bare host (see
``test_echo_cir_jam_plan.py``).  The RF side of the jamming app lives in the
corresponding ``x410_*`` app.

Contract
--------
The jammer transmits a second HRP waveform on another X410 TX channel while
the sensing path keeps its own TX/RX pair.  ``compose_tx_native`` builds the
two-row host buffer handed to the sink: row 0 is the sensing waveform, row 1
is the jammer placed at a chosen sample delay.  ``validate_jam_args`` is the
fail-fast gate used by the app before it touches hardware.

Modes (``JAM_MODES``)
---------------------
off
    Jammer channel stays silent (sense only).
align
    Jammer is re-armed to the sensing pulse period (same PRI).
continuous
    Jammer repeats every ``jam_repeat_pri_us`` independently of the sensing
    burst, so its row can outlast the sensing row.

All time/frequency inputs are validated here; bad values raise ``ValueError``
and the message includes the offending value.
"""
from __future__ import annotations

import math

import numpy as np


JAM_MODES = ("off", "align", "continuous")
DEFAULT_JAM_CODE_INDEX = 10
DEFAULT_JAM_CHANNEL = 1

# Channels occupy the X410 RF ports 0..3 (one TX used for sensing, one RX).
_CHANNEL_MAX = 3
# Same order of magnitude as an X410 tune step; larger offsets are rejected.
_MAX_FREQ_OFFSET_HZ = 1e6
_JAM_CODE_INDICES = (9, 10, 11, 12)


def _as_finite(name, value):
    """``float(value)`` that must be finite, else ``ValueError``."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(
            "%s must be a finite number, got %r" % (name, value))
    if not math.isfinite(v):
        raise ValueError("%s must be finite, got %r" % (name, value))
    return v


def jam_freq_hz(center_hz, offset_hz):
    """Absolute jammer frequency: ``center_hz + offset_hz``.

    Both inputs must be finite; raises ``ValueError`` otherwise.
    """
    return _as_finite("center_hz", center_hz) + _as_finite("offset_hz", offset_hz)


def placement_native(delay_us, native_hz):
    """Convert a jammer delay in microseconds to native samples.

    Rounds half away from zero and returns an ``int``.  ``delay_us`` must be
    finite and >= 0, ``native_hz`` must be finite and > 0; raises
    ``ValueError`` otherwise.
    """
    d = _as_finite("delay_us", delay_us)
    n = _as_finite("native_hz", native_hz)
    if d < 0.0:
        raise ValueError("delay_us must be >= 0, got %r" % (delay_us,))
    if not (n > 0.0):
        raise ValueError("native_hz must be > 0, got %r" % (native_hz,))
    x = d * 1e-6 * n
    # Round half away from zero (x is >= 0 here, but keep it explicit).
    return int(math.floor(x + 0.5)) if x >= 0.0 else int(math.ceil(x - 0.5))


def combined_tx_len(sense_len, jam_len, delay_native):
    """Length of the two-row TX buffer.

    With no jammer (``jam_len <= 0``) this is just ``sense_len``.  Otherwise
    the jammer row ends at ``jam_len + delay_native``; the buffer is long
    enough for whichever row is longer.  ``sense_len`` must be > 0 and
    ``delay_native`` >= 0; raises ``ValueError`` otherwise.
    """
    s = int(sense_len)
    j = int(jam_len)
    d = int(delay_native)
    if s <= 0:
        raise ValueError("sense_len must be > 0, got %r" % (sense_len,))
    if d < 0:
        raise ValueError("delay_native must be >= 0, got %r" % (delay_native,))
    if j <= 0:
        return s
    return max(s, j + d)


def compose_tx_native(sense, jam, delay_native):
    """Build the ``(2, L)`` complex64 TX buffer.

    Row 0 is ``sense`` zero-padded to ``L``; row 1 is ``jam`` placed starting
    at sample ``delay_native`` with zeros elsewhere.  ``jam`` may be ``None``
    or empty, in which case row 1 is all zeros.  ``sense`` must be a
    non-empty 1-D array-like; raises ``ValueError`` otherwise.  Inputs are
    never mutated.
    """
    sense_arr = np.asarray(sense, dtype=np.complex64)
    if sense_arr.ndim != 1 or sense_arr.size == 0:
        raise ValueError(
            "sense must be a non-empty 1-D array, got shape %r"
            % (sense_arr.shape,))
    jam_len = 0 if jam is None else len(jam)
    length = combined_tx_len(sense_arr.size, jam_len, delay_native)
    out = np.zeros((2, length), dtype=np.complex64)
    out[0, :sense_arr.size] = sense_arr
    if jam_len > 0:
        jam_arr = np.asarray(jam, dtype=np.complex64)
        if jam_arr.ndim != 1:
            raise ValueError(
                "jam must be a 1-D array, got shape %r" % (jam_arr.shape,))
        delay = int(delay_native)
        out[1, delay:delay + jam_arr.size] = jam_arr
    return out


def validate_jam_args(args):
    """Fail-fast validation of the jammer CLI/config dict.

    Reads keys with ``.get()``: ``jam_enabled``, ``jam_mode``,
    ``jam_channel``, ``tx_channel``, ``rx_channel``,
    ``jam_freq_offset_hz``, ``jam_code_index``, ``jam_scale`` and
    ``jam_repeat_pri_us``.  Returns ``None`` when OK, otherwise raises
    ``ValueError`` naming the offending value.  When ``jam_enabled`` is
    falsy nothing else is checked.
    """
    if not args.get("jam_enabled"):
        return

    mode = args.get("jam_mode")
    if mode not in JAM_MODES:
        raise ValueError(
            "jam_mode must be one of %r, got %r" % (JAM_MODES, mode))

    for name in ("jam_channel", "tx_channel", "rx_channel"):
        v = args.get(name)
        if isinstance(v, bool) or not isinstance(v, int):
            raise ValueError(
                "%s must be an int in [0, %d], got %r"
                % (name, _CHANNEL_MAX, v))
        if v < 0 or v > _CHANNEL_MAX:
            raise ValueError(
                "%s must be in [0, %d], got %r" % (name, _CHANNEL_MAX, v))

    jam_ch = args.get("jam_channel")
    tx_ch = args.get("tx_channel")
    rx_ch = args.get("rx_channel")
    if jam_ch == tx_ch or jam_ch == rx_ch:
        raise ValueError(
            "jam_channel %r must differ from tx_channel %r and rx_channel %r"
            % (jam_ch, tx_ch, rx_ch))

    off = _as_finite("jam_freq_offset_hz", args.get("jam_freq_offset_hz"))
    if abs(off) > _MAX_FREQ_OFFSET_HZ:
        raise ValueError(
            "jam_freq_offset_hz must satisfy abs(...) <= 1e6, got %r" % (off,))

    code = args.get("jam_code_index")
    if code not in _JAM_CODE_INDICES:
        raise ValueError(
            "jam_code_index must be in %r, got %r" % (_JAM_CODE_INDICES, code))

    scale = _as_finite("jam_scale", args.get("jam_scale"))
    if not (scale > 0.0):
        raise ValueError("jam_scale must be > 0, got %r" % (scale,))

    if mode == "continuous":
        pri = _as_finite(
            "jam_repeat_pri_us", args.get("jam_repeat_pri_us"))
        if not (pri > 0.0):
            raise ValueError(
                "jam_repeat_pri_us must be > 0, got %r" % (pri,))
