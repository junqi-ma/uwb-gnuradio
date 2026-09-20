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
# In-buffer jammer delay (not an independent PRI).  2 ms @ 737.28 MS/s is
# already ~1.5e6 extra native samples; larger values blow up the TX burst.
_MAX_DELAY_RANDOM_US = 2000.0
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


def make_freq_offset_scan(start_hz, stop_hz, step_hz):
    """Inclusive jammer-offset grid from ``start_hz`` to ``stop_hz``.

    ``step_hz`` is a positive magnitude; direction follows start/stop
    (``stop < start`` sweeps down).  The stop value is always included,
    even when it does not land on the step grid.  Raises ``ValueError``
    on non-finite inputs or a zero step.
    """
    start = _as_finite("start_hz", start_hz)
    stop = _as_finite("stop_hz", stop_hz)
    step = _as_finite("step_hz", step_hz)
    if step == 0.0:
        raise ValueError("step_hz must be != 0, got %r" % (step_hz,))
    step = abs(step)
    if start == stop:
        return [start]
    direction = 1.0 if stop > start else -1.0
    span = abs(stop - start)
    n = int(math.floor(span / step + 1e-9)) + 1
    out = [start + direction * i * step for i in range(n)]
    if abs(out[-1] - stop) > 0.5:
        out.append(float(stop))
    return out


def jam_freq_hz(center_hz, offset_hz):
    """Absolute jammer frequency: ``center_hz + offset_hz``.

    Both inputs must be finite; raises ``ValueError`` otherwise.
    """
    return _as_finite("center_hz", center_hz) + _as_finite("offset_hz", offset_hz)


def placement_native(delay_us, native_hz):
    """Convert a jammer delay in microseconds to native samples.

    Rounds half away from zero and returns an ``int``.  ``delay_us`` may be
    negative (jammer leads the sensing burst).  ``native_hz`` must be finite
    and > 0; raises ``ValueError`` otherwise.
    """
    d = _as_finite("delay_us", delay_us)
    n = _as_finite("native_hz", native_hz)
    if not (n > 0.0):
        raise ValueError("native_hz must be > 0, got %r" % (native_hz,))
    x = d * 1e-6 * n
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


# Host float → SC16 scale used by the jam app (`x410_cg400_hrp_echo_cir.py`).
SC16_IQ_SCALE = 32768.0


def fc32_to_sc16(iq, iq_scale=SC16_IQ_SCALE):
    """UHD-style host float → interleaved little-endian int16 I/Q.

    Matches ``x410_cg400_hrp_echo_cir.fc32_to_sc16`` so a jam-scale that
    quantizes to digital zero can be checked without importing GNU Radio.
    """
    x = np.asarray(iq, dtype=np.complex64)
    interleaved = np.empty(x.size * 2, dtype=np.float64)
    interleaved[0::2] = np.real(x)
    interleaved[1::2] = np.imag(x)
    scaled = np.rint(interleaved * float(iq_scale))
    return np.clip(scaled, -32768, 32767).astype(np.int16)


def bipolar_tx_len(sense_len, jam_len, half_span_native):
    """TX length when the jammer delay is in ``[-D, D]`` and sense sits at ``D``.

    ``half_span_native`` is ``D >= 0``.  With no jammer this is just
    ``sense_len`` (no pad).  Otherwise
    ``L = max(D + sense_len, 2D + jam_len)``.
    """
    s = int(sense_len)
    j = int(jam_len)
    d = int(half_span_native)
    if s <= 0:
        raise ValueError("sense_len must be > 0, got %r" % (sense_len,))
    if d < 0:
        raise ValueError(
            "half_span_native must be >= 0, got %r" % (half_span_native,))
    if j <= 0:
        return s
    return max(d + s, 2 * d + j)


def compose_tx_native_at(sense, jam, sense_offset, jam_offset, length=None):
    """Build a ``(2, L)`` payload with explicit row start indices.

    ``length`` defaults to the smallest L that fits both rows.  Inputs are
    never mutated.
    """
    sense_arr = np.asarray(sense, dtype=np.complex64)
    if sense_arr.ndim != 1 or sense_arr.size == 0:
        raise ValueError(
            "sense must be a non-empty 1-D array, got shape %r"
            % (sense_arr.shape,))
    so = int(sense_offset)
    jo = int(jam_offset)
    if so < 0 or jo < 0:
        raise ValueError(
            "offsets must be >= 0, got sense_offset=%r jam_offset=%r"
            % (sense_offset, jam_offset))
    jam_len = 0 if jam is None else len(jam)
    need = so + int(sense_arr.size)
    if jam_len > 0:
        need = max(need, jo + int(jam_len))
    L = int(need if length is None else length)
    if L < need:
        raise ValueError(
            "length %d is shorter than required %d" % (L, need))
    out = np.zeros((2, L), dtype=np.complex64)
    out[0, so:so + sense_arr.size] = sense_arr
    if jam_len > 0:
        jam_arr = np.asarray(jam, dtype=np.complex64)
        if jam_arr.ndim != 1:
            raise ValueError(
                "jam must be a 1-D array, got shape %r" % (jam_arr.shape,))
        out[1, jo:jo + jam_arr.size] = jam_arr
    return out


def parse_delay_random_us(text):
    """Parse ``--jam-delay-random-us``.

    ``None`` / empty → ``None`` (fixed ``--jam-delay-us``).  A single
    number ``T >= 0`` means the per-pulse delay is uniform on
    ``[-T, +T]`` microseconds.  Raises ``ValueError`` naming the
    offending value.  Pair syntax is rejected.
    """
    if text is None:
        return None
    s = str(text).strip()
    if not s:
        return None
    if "," in s:
        raise ValueError(
            "jam_delay_random_us is a single ±span in us, got %r" % (text,))
    t = _as_finite("jam_delay_random_us", s)
    if t < 0.0:
        raise ValueError(
            "jam_delay_random_us must be >= 0, got %r" % (t,))
    if t > _MAX_DELAY_RANDOM_US:
        raise ValueError(
            "jam_delay_random_us must be <= %g us (in-buffer ±offset, "
            "not an independent PRI), got %r"
            % (_MAX_DELAY_RANDOM_US, t))
    return t


def delay_half_span_native(span_us, native_hz):
    """Native samples ``D`` for a ±``span_us`` window (``[-D, D]``)."""
    d = placement_native(span_us, native_hz)
    if d < 0:
        d = -d
    return d


def draw_delay_native(rng, lo_native, hi_native):
    """Uniform integer draw on ``[lo_native, hi_native]``.

    ``rng`` must implement ``integers(low, high)`` with ``high`` exclusive
    (``numpy.random.Generator``).  A degenerate span returns ``lo_native``.
    ``lo_native`` may be negative (jammer leads sensing).
    """
    lo = int(lo_native)
    hi = int(hi_native)
    if hi < lo:
        raise ValueError(
            "hi_native must be >= lo_native, got lo=%r hi=%r"
            % (lo_native, hi_native))
    if lo == hi:
        return lo
    return int(rng.integers(lo, hi + 1))


def place_jam_row(payload, jam, delay_native):
    """Write ``jam`` into row 1 of an existing ``(2, L)`` payload.

    Row 0 is left untouched.  Row 1 is zeroed, then ``jam`` is copied
    starting at ``delay_native``.  Mutates and returns ``payload``.
    ``jam`` may be ``None`` or empty (row 1 stays zero).  The placement
    must fit in ``L``; raises ``ValueError`` otherwise.
    """
    arr = np.asarray(payload)
    if arr.ndim != 2 or arr.shape[0] != 2 or arr.shape[1] == 0:
        raise ValueError(
            "payload must have shape (2, L) with L>0, got %r" % (arr.shape,))
    arr[1, :] = 0
    if jam is None:
        return arr
    jam_arr = np.asarray(jam, dtype=arr.dtype)
    if jam_arr.size == 0:
        return arr
    if jam_arr.ndim != 1:
        raise ValueError(
            "jam must be a 1-D array, got shape %r" % (jam_arr.shape,))
    delay = int(delay_native)
    if delay < 0:
        raise ValueError("delay_native must be >= 0, got %r" % (delay_native,))
    end = delay + int(jam_arr.size)
    if end > arr.shape[1]:
        raise ValueError(
            "jam placement [%d:%d] exceeds payload length %d"
            % (delay, end, arr.shape[1]))
    arr[1, delay:end] = jam_arr
    return arr


def validate_jam_args(args):
    """Fail-fast validation of the jammer CLI/config dict.

    Reads keys with ``.get()``: ``jam_enabled``, ``jam_mode``,
    ``jam_channel``, ``tx_channel``, ``rx_channel``,
    ``jam_freq_offset_hz``, ``jam_code_index``, ``jam_scale``,
    ``jam_repeat_pri_us`` and optional ``jam_delay_random_us``.  Returns
    ``None`` when OK, otherwise raises ``ValueError`` naming the offending
    value.  When ``jam_enabled`` is falsy nothing else is checked.
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

    span = args.get("jam_delay_random_us")
    if span is not None:
        t = _as_finite("jam_delay_random_us", span)
        if t < 0.0:
            raise ValueError(
                "jam_delay_random_us must be >= 0, got %r" % (t,))
        if t > _MAX_DELAY_RANDOM_US:
            raise ValueError(
                "jam_delay_random_us must be <= %g us, got %r"
                % (_MAX_DELAY_RANDOM_US, t))


# --- C++ dual-TX schedule metadata (§5.3, M3 app layer) --------------------

#: Schedule-PDU metadata keys consumed by the C++ multi-TX EchoTimer
#: (planning §5.3).  All float ``us``/``s`` inputs are converted to native
#: integer ticks here in Python; the hot path never does float geometry.
CPP_SCHEDULE_META_KEYS = (
    "tx_channel_count",
    "tx_samples",
    "tx_waveform_samples",
    "tx_base_offsets_native",
    "jam_logical_channel",
    "jam_delay_mode",
    "jam_delay_lo_native",
    "jam_delay_hi_native",
    "jam_delay_seed",
    "jam_freq_offsets_hz",
    "jam_dwell",
    "jam_freq_settle_ticks",
)

#: Schedule-PDU ``jam_delay_mode`` values (C++ ``JamDelayMode``).
CPP_JAM_DELAY_MODES = ("fixed", "uniform")


def cpp_jam_delay_mode(delay_random_us):
    """Schedule-PDU delay mode name for a ``--jam-delay-random-us`` span.

    ``None`` (fixed ``--jam-delay-us``) → ``"fixed"``; any span value
    (including ``0.0``) → ``"uniform"`` on ``[-D, +D]``.
    """
    return "uniform" if delay_random_us is not None else "fixed"


def build_cpp_schedule_meta(*, sense_len, jam_len, native_hz,
                            delay_us=0.0, delay_random_us=None,
                            delay_seed=0, jam_offsets_hz=(),
                            jam_dwell=0, freq_settle_s=0.05,
                            jam_logical_channel=1):
    """Build the §5.3 multi-TX schedule metadata (plain Python, PMT-free).

    Geometry mirrors ``JamTimedUhdEcho.prepare_jam`` so the Python timed
    path and the C++ grid agree sample-exactly:

    * fixed (``delay_random_us is None``): sense sits at 0, jammer at
      ``placement_native(delay_us)``,
      ``L = combined_tx_len(sense, jam, delay)``;
    * uniform: sense is parked at ``D = delay_half_span_native(span)``,
      jammer base at ``D`` with per-pulse ``delay in [-D, +D]``,
      ``L = bipolar_tx_len(sense, jam, D)``.

    The payload handed to C++ holds the two *effective* waveforms (no
    per-pulse ``(2, L)`` composite); ``tx_base_offsets_native`` gives each
    channel's base start inside the physical burst ``L``.  ``freq_settle_s``
    is converted to ``jam_freq_settle_ticks`` here.

    Returns a JSON-serialisable ``dict`` with exactly
    ``CPP_SCHEDULE_META_KEYS``.  Raises ``ValueError`` on bad lengths,
    rate, delay span, seed, dwell or offsets.  Existing
    ``compose_*``/``validate_*`` semantics are untouched.
    """
    s = int(sense_len)
    j = int(jam_len)
    if s <= 0:
        raise ValueError("sense_len must be > 0, got %r" % (sense_len,))
    if j <= 0:
        raise ValueError(
            "jam_len must be > 0 for dual-TX, got %r" % (jam_len,))
    n = _as_finite("native_hz", native_hz)
    if not (n > 0.0):
        raise ValueError("native_hz must be > 0, got %r" % (native_hz,))
    mode = cpp_jam_delay_mode(delay_random_us)
    if mode == "uniform":
        span = _as_finite("delay_random_us", delay_random_us)
        if span < 0.0:
            raise ValueError(
                "delay_random_us must be >= 0, got %r" % (delay_random_us,))
        if span > _MAX_DELAY_RANDOM_US:
            raise ValueError(
                "delay_random_us must be <= %g us, got %r"
                % (_MAX_DELAY_RANDOM_US, span))
        half = delay_half_span_native(span, n)
        length = bipolar_tx_len(s, j, half)
        sense_off, jam_base = half, half
        lo_n, hi_n = -half, half
    else:
        delay_n = placement_native(delay_us, n)
        length = combined_tx_len(s, j, delay_n)
        sense_off, jam_base = 0, delay_n
        lo_n = hi_n = delay_n
    try:
        seed = int(delay_seed or 0)
    except (TypeError, ValueError):
        raise ValueError(
            "delay_seed must be an int, got %r" % (delay_seed,))
    if seed < 0 or seed >= 2**32:
        raise ValueError(
            "delay_seed must be in [0, 2**32), got %r" % (delay_seed,))
    offsets = [float(_as_finite("jam_offsets_hz[%d]" % i, o))
               for i, o in enumerate(list(jam_offsets_hz))]
    dwell = int(jam_dwell)
    if dwell < 0:
        raise ValueError("jam_dwell must be >= 0, got %r" % (jam_dwell,))
    settle_s = _as_finite("freq_settle_s", freq_settle_s)
    if settle_s < 0.0:
        raise ValueError(
            "freq_settle_s must be >= 0, got %r" % (freq_settle_s,))
    settle_ticks = int(math.floor(settle_s * n + 0.5))
    return {
        "tx_channel_count": 2,
        "tx_samples": int(length),
        "tx_waveform_samples": [s, j],
        "tx_base_offsets_native": [int(sense_off), int(jam_base)],
        "jam_logical_channel": int(jam_logical_channel),
        "jam_delay_mode": mode,
        "jam_delay_lo_native": int(lo_n),
        "jam_delay_hi_native": int(hi_n),
        "jam_delay_seed": seed,
        "jam_freq_offsets_hz": offsets,
        "jam_dwell": dwell,
        "jam_freq_settle_ticks": settle_ticks,
    }


def contiguous_window_layout(*, sense_len, jam_len, native_hz,
                             delay_us=0.0, delay_random_us=None):
    """C++ contiguous-window send geometry (no delay-dependent fragments).

    Returns a JSON-serialisable dict with ``layout='contiguous-window'``,
    ``L``, ``D``, ``backing_length``, jam-window range and
    ``data_fragments=1``.  Uniform: backing is ``L+2D`` and the jammer
    read window start is ``D-delta`` in ``[0, 2D]``.  Fixed: two dense
    rows of length ``L``, jammer parked at the fixed delay.
    """
    meta = build_cpp_schedule_meta(
        sense_len=sense_len, jam_len=jam_len, native_hz=native_hz,
        delay_us=delay_us, delay_random_us=delay_random_us)
    L = int(meta["tx_samples"])
    D = int(meta["tx_base_offsets_native"][0])
    mode = str(meta["jam_delay_mode"])
    if mode == "uniform":
        backing = L + 2 * D
        win_lo, win_hi = 0, 2 * D
    else:
        backing = L
        jam_at = int(meta["tx_base_offsets_native"][1])
        win_lo = win_hi = jam_at
    return {
        "layout": "contiguous-window",
        "L": L,
        "D": D,
        "backing_length": int(backing),
        "jam_window_begin_min": int(win_lo),
        "jam_window_begin_max": int(win_hi),
        "data_fragments": 1,
        "jam_delay_mode": mode,
    }


def require_fragment_covers_L(max_fragment_size, L):
    """Contiguous mode needs one data send covering the whole burst."""
    f = int(max_fragment_size)
    n = int(L)
    if f < n:
        raise ValueError(
            "max_fragment_size %d < L %d; contiguous-window dual-TX "
            "requires one data fragment (raise --max-fragment-size)"
            % (f, n))
    return 1
