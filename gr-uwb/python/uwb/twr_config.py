# Copyright 2026
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Typed TWR configuration, capability whitelist, validator and JSON codec.

This is the pure-Python half of REQ-API-01.  It is a field-for-field mirror of
the frozen C++ layer

    gr-uwb/include/gnuradio/uwb/uwb_twr_config.h
    gr-uwb/include/gnuradio/uwb/uwb_twr_types.h

and it must agree with that header: same field names, same enum spellings,
same dotted field paths, same ``ConfigReason`` taxonomy, same
``"fnv1a64:%016x"`` config hash, same JSON key set.  Every number format and
every hash is byte-compatible with the C++ side, so a config hashed here and a
config hashed there produce the same identifier.

It is deliberately **pure Python + stdlib**:

  * no pybind11 and no C++ extension -- the pybind exposure is deferred to M1
    to avoid colliding with the binding work; this module therefore imports
    from a source checkout with no built OOT module and no radio present;
  * no ``numpy`` (nothing here needs it) and no ``jsonschema``;
  * the JSON codec is built on the stdlib ``json`` module with hooks that
    preserve the int64 / double distinction the C++ parser makes.

Requirements traceability (docs/twr/需求_UWB_SS_DS_TWR.md):
  REQ-SCOPE-01  unsupported combinations are rejected, never defaulted
  REQ-SCOPE-04  no STS in phase 1
  REQ-PHY-01    common profile is ch5 / 6489.6 MHz / 64 MHz PRF class
  REQ-PHY-02    native rate, work rate, symbol rate and RF bandwidth are four
                different quantities
  REQ-PHY-03    channel 5 first; other channels are per-channel claims
  REQ-BASE-01   one physical channel, one owner
  REQ-BASE-02   a supported code index is not a supported preamble length
  REQ-API-01    typed config, one validator, capabilities() / validate() /
                effective_config(), versions, requested vs effective, readback,
                startup rejection
  REQ-API-02    reply delay / post-TX RX enable / RX timeout are three
                distinct things, each with a reference event
  REQ-API-03    per-message overrides take effect at exchange boundaries on
                an immutable snapshot
  REQ-PROTO-01  a two-endpoint session needs one initiator and one responder
  REQ-PROTO-05  a TWR frame sets the ranging bit; the Report message is
                reserved and NOT implemented
  REQ-ERR-01    every rejection maps onto the frozen status taxonomy
  REQ-OUT-01    64-bit fields keep integer precision through JSON
  REQ-CAL-01    calibration is versioned and applicable exactly once
  REQ-GR-02     diagnostic I/O never runs on the realtime thread
  REQ-GR-03     bounded queues / diagnostic buffers
  REQ-GR-04     reply-delay budget feasibility is checked, not assumed
  REQ-QA-01/02  targeted QA; no self-generated ground truth

WHAT IS *NOT* CLAIMED HERE
--------------------------
  * Nothing here says a DW1000 / DW3000 / any vendor module interoperates.  A
    loopback round trip proves this repository's TX and RX agree with each
    other, nothing more (see ``testdata/twr/phy_matrix_whitelist_737280000.csv``).
  * ``min_tx_lead_time`` has NO measured value.  ``testdata/twr/
    timing_budget_turnaround_estimate.csv`` records ``uhd_min_lead_time`` as
    ``UNMEASURED`` and ``min_feasible_reply_delay`` as
    ``BLOCKED_ON_HOST_MEASUREMENT`` for all four reply scopes.  The validator
    therefore has no default for it: the operator must supply the measured
    value, and a reply delay that does not cover it is rejected
    (deadline_missed) rather than sent late.
  * No accuracy claim of any kind.  Nothing in this file is evidence that any
    centimetre-level range was ever produced.
  * **16 SYNC is admitted but is NOT claimed to be fit for ranging.**  The
    re-measurement against a freshly rebuilt library is done and the verdict is
    positive on every axis that was measured: 16 SYNC round-trips with a
    byte-exact FCS (98/98 rows), its ``cfo_zero_peak_corr`` is 0 on every row so
    no synthesised zero-phase point can enter the carrier-offset fit, and it is
    self-describing (``phr_preamble_idx == 0`` = 16, a legal 802.15.4a BPRF
    duration).  What is still UNMEASURED is its first-path / ToA accuracy over
    the shorter 16.3 us integration window (64 SYNC has 65.1 us) -- and ranging
    accuracy is a function of exactly that quantity.  So 16 SYNC sits in
    ``SYNC_REPS_NEEDING_TOA_VALIDATION``: a caller building a ranging profile
    must consult ``sync_reps_needing_toa_validation()`` and refuse it until M2
    measures the first path.  See ``SYNC_16_TOA_UNVERIFIED_REASON``.
  * No AWGN noise ceiling is asserted for either length.  The shipped CSVs record
    one sigma point per length, not a sweep, so a ceiling could not be sourced
    from the data this file mirrors.

VALUE SEMANTICS AT THE CONFIG BOUNDARY (M0.1, closes R1)
-------------------------------------------------------
A validated config is a VALUE.  Nothing this module hands out is a handle onto
one:

  * ``effective_config()`` reads the caller's config ONCE and immediately turns
    it into a private deep copy; ``requested``, ``effective`` and ``readback``
    are three further independent copies.  Mutating the caller's object
    afterwards -- a scalar, a nested dataclass, a list element, the readback --
    cannot move the snapshot, its hash, its ``ok`` flag or its quantised values.
  * ``TwrConfigSnapshot`` deep-copies the ``EffectiveConfig`` it is given, and
    ``get()`` returns a FRESH deep copy on every call.  There is no public
    attribute, cache or iterator onto the internal config.
  * The process-wide capability tables are FROZEN: every container is a
    ``ReadOnlyList`` with no mutator at all, attribute assignment is refused,
    and a ``PhyCapabilityRow`` is a frozen dataclass.  ``capabilities()`` and
    ``default_deny_capabilities()`` return the one shared read-only instance,
    because two endpoints validating concurrently must not disagree about what
    the hardware supports.  Derive a different table with
    ``CapabilitiesBuilder(base)`` -- never by editing the shipped one.

``set_overrides()`` is the only way to change a validated config, and only at an
exchange boundary; a call while an exchange is in flight is refused and the
running exchange keeps the value it started with (REQ-API-03).

WHERE COPYING IS ALLOWED, AND WHERE IT IS NOT
----------------------------------------------
Copying belongs to CONFIGURE/EXCHANGE BOUNDARIES ONLY.  It is allowed when a
config is read from JSON or built by the operator, inside ``effective_config()``,
inside ``TwrConfigSnapshot``, and once per ``set_overrides()``.  It is FORBIDDEN
on any per-sample, per-IQ or per-packet path: M2's work<->native rate
conversion and M3's timed-TX controller must hold the effective value they were
given and read it, because a per-packet copy is a per-packet heap allocation and
belongs in no realtime budget.  A caller that needs the config on a hot path
takes it once with ``get()`` and keeps it for the lifetime of the work.  If that
is ever too expensive, the answer is a fixed-size POD struct on the C++ side
(M1 owns that), not a weaker Python guarantee.  ``copy_config`` documents this
rule where a caller will actually read it.

THE SCHEMA VERSION (M0.1, R3 / R5)
-----------------------------------
``twr-config/2`` is a BREAKING change and the version says so:
``frame.frame_profile`` became REQUIRED (it selects the MAC layout the geometry
claim is checked against, and a free-form profile name is not a layout), and
``phy.phr_rate`` changed value domain -- it used to be a ``DataRate`` required
to EQUAL ``phy.data_rate`` and is now a :class:`PhrRate` with its own two
members.  A ``twr-config/1`` document is REFUSED with a migration message that
names the fields that changed, never silently misread: the two schemas disagree
about what a PHR rate is, and reading one as the other would produce a config
that validates and transmits something the author never asked for.

THE FRAME CODEC IS THE GEOMETRY AUTHORITY (M0.1, R3)
----------------------------------------------------
M0 review R3 found TWO geometries for one format: this schema carried a
pre-codec "7-byte header" :class:`FrameGeometry` with 9/19/24-byte budgets while
``uwb_twr_frame.h`` had emitted a 14-byte header and 16/26/31-byte on-air frames
since it was frozen.  Both suites passed, because nothing ever asked whether an
ACCEPTED configuration can losslessly build the real frame.

The dependency now points in the one direction that can be wrong in one place
instead of two:

  * :class:`FrameGeometry` is a CLAIM -- "this is what I believe the layout to
    be" -- checked field by field through :func:`frame_geometry_check`, and
    every disagreement becomes its own rejection naming that exact field.  It
    never decides anything.
  * Every byte count this module reports is a DELEGATION to
    :class:`FrameProfileGeometry`.  There is no second ``header + n * timestamp``
    sum anywhere in this file, and no fixture keeps its own copy of the numbers.
  * ``frame.frame_profile`` is a :class:`FrameProfileId`, not a free-form
    string, so "an arbitrary profile name is an executable geometry" is
    UNREPRESENTABLE rather than merely discouraged.
  * :func:`frame_psdu_bytes` and :func:`frame_bytes_on_air` keep their old
    three- and four-argument call SHAPES for source compatibility, but the
    geometry argument is deliberately IGNORED -- a claim does not decide a
    length, and keeping the parameter means an existing call site keeps working
    while losing the ability to pass a number of its own.

THE PARITY CORPUS (M0.1, plan 2.A item 4)
------------------------------------------
C++ is the runtime authority of record.  A retained pure-Python validator is
only defensible if it is pinned to the SAME profile data and compared ITEM BY
ITEM against the SAME JSON corpus: accepted/rejected, the reason, and the
effective values -- including the ``config_hash``, which is an fnv1a64 over the
canonical listing of every field and so catches any single wrong spelling,
missing entry or ordering difference.  Comparing enums, or grepping source
strings, does not count.

The corpus is ``testdata/twr/config_parity_corpus.json`` and is read by BOTH
implementations: by this module through :func:`load_parity_corpus` and
:func:`run_parity_case` (driven by ``gr-uwb/apps/test_twr_config.py``), and by
``gr-uwb/lib/qa_uwb_twr_config_parity.cc`` through the C++ header's own JSON
parser.  Section 11 of this module is the loader both sides share.

DELIBERATE DIVERGENCES FROM THE C++ HEADER (and why)
----------------------------------------------------
1. Each rejection carries the MEASURED reason the CSV recorded (or, for a
   length the matrix never swept, the derived tokens in
   ``sync_repetition_reasons``) instead of the single generic "matrix not
   measured yet" string.  Same ``ConfigReason``, better message.  The human
   MESSAGE is the ONLY thing that differs, and the parity corpus therefore
   compares the machine-readable triple -- field path, ``ConfigReason``,
   ``ExchangeStatus`` -- rather than the prose.
2. Rejected combinations are carried in ``Capabilities.phy_matrix_rejections``
   so the rejection reason can be recovered from the table instead of being
   recomputed.  A row in that list can only ever produce a *better* message; it
   can never turn a rejection into an acceptance.  C++ has no such list.
3. ``build_unmeasured_capabilities()`` keeps the PRE-matrix empty-list state the
   C++ header used to ship, so the default-deny mechanism stays testable.  It is
   not the C++ default any more -- ``build_default_capabilities()`` is, and it is
   the measured whitelist, byte-for-byte the same 48 rows / 2 SYNC lengths /
   2 reason strings / profile version.
4. ``Opt[T]`` is ``Optional[T]`` with ``None`` meaning absent.
5. The JSON codec uses the stdlib ``json`` module rather than a hand-written
   parser, with ``parse_float``/``parse_constant`` hooks that reproduce the C++
   reader's int64-vs-double distinction and its refusal of ``NaN`` /
   ``Infinity`` / ``-Infinity``.
6. TWO BEHAVIOURS THAT LOOK LIKE DIVERGENCES AND ARE NOT -- they are MIRRORS of
   the C++ reader, and the parity corpus pins both:
   * an unknown or mistyped enum leaves the enumeration's ZERO member behind
     (C++ ``static_cast<E>(0)``) and always records a machine-readable
     violation.  It is not a silent default: the import report is the authority
     and is what every entry point returns.  Substituting a "sensible" member
     instead would make the two implementations disagree about a document they
     both reject.
   * an unsigned field NARROWS to its width (C++
     ``static_cast<uint16_t>(get_i64(...))``), so ``"local_address": -1`` is
     65535 and trips the reserved-address rule rather than storing a number no
     C++ build can hold.
"""

from __future__ import annotations

import copy as _copy
import json
import math
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = [
    # enums
    "Protocol", "Role", "FrameType", "PrfClass", "DataRate", "SfdMode",
    "PhrMode", "PhrRate", "PreambleLength", "FrameProfileId",
    "StsMode", "FcsAppender", "TxPowerPolicy", "PulseShaping",
    "AgcMode", "FirstPathAlgorithm", "CompensationFlag", "TimeReferenceEvent",
    "TimeDomain", "TimeUnit", "TimestampMarker", "TimestampSource",
    "EndpointState", "ExchangeStatus", "CapabilityStatus", "ConfigReason",
    "EvidenceLevel", "EvidenceUse", "SessionIdError", "GeometryField",
    "FcsOwner",
    # constants
    "JSON_MAX_SAFE_INTEGER", "NATIVE_RATE_UC200_HZ", "NATIVE_RATE_CG400_HZ",
    "NATIVE_RATE_TOLERANCE_REL", "CHANNEL_FREQUENCY_TOLERANCE_HZ",
    "WORK_SAMPLE_RATE_HZ", "WORK_SAMPLES_PER_SYMBOL", "EXISTING_MEAN_PRF_HZ",
    "NOMINAL_MEAN_PRF_HZ", "CODE_INDEX_MIN", "CODE_INDEX_MAX",
    "MAX_PSDU_BYTES", "MAX_IN_FLIGHT_EXCHANGES", "MAX_DIAGNOSTIC_BYTES",
    "MAX_QUEUE_ENTRIES", "MAX_MEASUREMENT_COUNT", "MAX_ATTEMPTS_PER_EXCHANGE",
    "MAX_PHYSICAL_CHANNELS", "MAX_PEERS_PER_CONFIG", "SCHEMA_VERSION",
    "LEGACY_SCHEMA_VERSIONS", "SCHEMA_V1_MIGRATION_REASON",
    "FRAME_HEADER_BYTES", "FRAME_FCS_BYTES", "FRAME_MAX_PSDU_BYTES",
    "FRAME_TIMESTAMP_BITS", "FRAME_TIMESTAMP_BYTES", "FRAME_MAX_TIMESTAMPS",
    "FRAME_VERSION", "PHR_STANDARD_INFO_BYTES", "PHR_STANDARD_CODED_BITS",
    "PHR_STANDARD_SYMBOLS", "SESSION_ID_WIRE_BITS", "SESSION_ID_WIRE_MAX",
    "SESSION_ID_RESERVED_LOCAL", "SESSION_ID_BIRTHDAY_SESSIONS_50PCT",
    "NATIVE_TICK_NS", "UWB_CHANNEL_CENTER_FREQUENCY_HZ",
    "RADAR_SYNC_REPETITIONS", "IEEE_802154A_PREAMBLE_SYMBOLS",
    "ALL_PHR_RATES", "ALL_PREAMBLE_LENGTHS", "ALL_FRAME_PROFILE_IDS",
    "MEASURED_CODE_INDICES", "MEASURED_SYNC_REPETITIONS", "MEASURED_SFD_MODES",
    "MEASURED_DATA_RATES", "MEASURED_PHY_NATIVE_RATE_HZ",
    "CFO_TAIL_REPETITIONS", "CFO_SKIP_INITIAL_REPETITIONS",
    "SYNC_16_TOA_UNVERIFIED_REASON", "SYNC_16_CAVEAT_REASON",
    "SYNC_REPS_NEEDING_TOA_VALIDATION", "sync_reps_needing_toa_validation",
    "MEASURED_ROW_REASON", "MEASURED_ROW_REASON_TOA_UNVERIFIED",
    "MEASURED_ROW_EVIDENCE_LEVEL", "MEASURED_ROW_EVIDENCE_SOURCE",
    "MEASURED_ROW_EVIDENCE_STATEMENT",
    "CFO_FIT_REASON_TEMPLATE",
    "PHR_DURATION_REASON_TEMPLATE", "ILLEGAL_LENGTH_REASON_TEMPLATE",
    "API_LIST_REASON", "CODE_INDEX_REASON", "STAGE_CFO_FAILED_REASON",
    "STAGE_CIR_FAILED_REASON", "CFO_MIN_PEAKS_REASON_TEMPLATE",
    "CIR_SKIP_REASON_TEMPLATE", "CFO_MIN_MEASURED_PEAKS",
    "CIR_SKIP_INITIAL_REPETITIONS", "DATA_RATE_REJECT_REASON",
    "NATIVE_ROUNDTRIP_BLOCKED_REASON",
    "PENDING_MATRIX_REASON", "UNSUPPORTED_STS_REASON",
    "PHY_MATRIX_WHITELIST_CSV",
    # time
    "Duration", "quantise_duration", "device_field", "host_field",
    # reporting
    "ConfigViolation", "ValidationReport",
    # timed fields
    "TimedField", "timed_field_to_text",
    # capabilities
    "PhyCapabilityRow", "CapabilityLookup", "ReadOnlyList", "Capabilities",
    "CapabilitiesBuilder", "capabilities", "unmeasured_capabilities",
    "default_deny_capabilities",
    "build_measured_capabilities", "build_default_capabilities",
    "build_unmeasured_capabilities",
    "is_allowed_native_rate", "is_allowed_quantisation_hz",
    "code_index_supported", "sync_reps_supported",
    "uwb_channel_center_frequency_hz", "sfd_mode_symbols",
    "phr_advertised_sync_symbols", "phr_preamble_duration_index",
    "sync_repetition_reasons", "phr_self_description_reason",
    "cfo_fit_leak_reason",
    "phr_rate_is_implemented", "phr_rate_symbols",
    "phr_rate_unsupported_reason", "phr_mode_unsupported_reason",
    "preamble_length_symbols", "preamble_length_from_symbols",
    "preamble_length_unsupported_reason", "preamble_length_supported_by_vendor",
    "preamble_length_reject_note", "reject_scope_current_software",
    "reject_scope_out_of_scope", "reject_scope_not_measured",
    "reject_scope_api_contract", "vendor_preamble_length_citation",
    "evidence_level_implies", "evidence_level_valid",
    "evidence_level_index", "evidence_required_level", "evidence_allows",
    "evidence_explain", "not_established_list", "EVIDENCE_LEVEL_ORDER",
    "EVIDENCE_USE_ORDER",
    # frame geometry authority
    "FrameProfileGeometry", "FrameGeometryClaim", "GeometryMismatch",
    "GeometryCheckResult", "GEOMETRY_FIELD_ORDER",
    "frame_geometry_for", "frame_geometry_mac_appends_fcs",
    "frame_geometry_check", "frame_geometry_claim",
    "frame_geometry_from_authority", "frame_geometry_of_profile",
    "frame_profile_id_is_supported",
    # session id wire mapping
    "session_id_fits_wire", "session_id_to_wire",
    "session_id_error_to_exchange_status", "wire_session_id_collides",
    # config
    "FrameGeometry", "SessionConfig", "PhyConfig", "FrameFormatConfig",
    "TransmitConfig", "ReceiveConfig", "RadioReadback", "EndpointBinding",
    "RadioConfig", "PerMessageTiming", "TimeoutConfig", "CalibrationRecord",
    "TimestampCalibrationConfig", "DiagnosticsConfig", "ConfigMeta",
    "TwrConfig", "frame_psdu_bytes", "frame_type_timestamp_count",
    "frame_bytes_on_air",
    # field listing / hash
    "FieldRecord", "FieldChange", "flatten_fields", "canonical_text",
    "config_hash", "diff_fields", "int_to_text", "double_to_text",
    "bool_to_text", "duration_to_text",
    # validation
    "ConfigValidator", "validate",
    # effective / runtime
    "EffectiveConfig", "effective_config", "copy_config", "ExchangeGate",
    "PerMessageOverrides", "TwrConfigSnapshot", "apply_calibration_once",
    # json
    "ConfigJsonError", "to_json_dict", "from_json_dict", "to_json_string",
    "from_json_string", "effective_to_json_dict", "parse_int64_strict",
]

# ===========================================================================
# 0. Mirrored constants
# ===========================================================================
#
# Everything in this block is a duplication of a value that already exists in
# C++.  The duplication is deliberate and load-bearing: this module must be
# importable with no GNU Radio and no built library, so it cannot include the
# headers.  The C++ QA (qa_uwb_twr_config.cc) cross-checks the same constants
# against the code they mirror; the Python QA (test_twr_config.py) checks the
# same properties from this side.

# IEEE 754 JSON-safe integer limit (2**53 - 1).  Larger integers are written
# as decimal STRINGS (REQ-OUT-01).
JSON_MAX_SAFE_INTEGER = 9007199254740991

INT64_MIN = -(2 ** 63)
INT64_MAX = 2 ** 63 - 1

# Mirrors is_allowed_uhd_native_rate() in uwb_uhd_backend_config.h.
NATIVE_RATE_UC200_HZ = 737280000.0  # X410 / UC200
NATIVE_RATE_CG400_HZ = 491520000.0  # CG400
NATIVE_RATE_TOLERANCE_REL = 1e-9

# Mirrors kTwrChannelFrequencyToleranceHz.
CHANNEL_FREQUENCY_TOLERANCE_HZ = 100.0

# Mirrors twr::kTwrWorkSampleRateHz / kTwrWorkSamplesPerSymbol /
# kTwrExistingMeanPrfHz / kTwrNominalMeanPrfHz.
WORK_SAMPLE_RATE_HZ = 998.4e6
WORK_SAMPLES_PER_SYMBOL = 1016
EXISTING_MEAN_PRF_HZ = 62.4e6
NOMINAL_MEAN_PRF_HZ = 64.0e6

# Mirrors radar_meta::code_index_supported() and kMaxPsduBytes.
CODE_INDEX_MIN = 9
CODE_INDEX_MAX = 12
MAX_PSDU_BYTES = 127

# Phase-1 hard limit (REQ-SCOPE-01 "同端初版一次仅一个在途 exchange").
MAX_IN_FLIGHT_EXCHANGES = 1

# Diagnostics memory bounds (REQ-GR-03: bounded buffers).
MAX_DIAGNOSTIC_BYTES = 64 * 1024 * 1024
MAX_QUEUE_ENTRIES = 1 << 20
MAX_MEASUREMENT_COUNT = 1000000
MAX_ATTEMPTS_PER_EXCHANGE = 16
MAX_PHYSICAL_CHANNELS = 16
MAX_PEERS_PER_CONFIG = 4

# ---------------------------------------------------------------------------
# THE SCHEMA VERSION (M0.1, R3 / R5)
# ---------------------------------------------------------------------------
# ``twr-config/2`` is a BREAKING change and the version says so:
#
#   * ``frame.frame_profile`` became REQUIRED.  It selects the MAC layout the
#     geometry claim is checked against, and a free-form profile name is not a
#     layout (M0 review R3).
#   * ``phy.phr_rate`` changed value domain: it used to be a ``DataRate`` that
#     had to EQUAL ``phy.data_rate``, and is now a ``PhrRate`` with its own two
#     members.  A document that says ``"phr_rate": "6p8m"`` meant a 6.81 Mb/s
#     PHR, which this modulator cannot transmit, so the string is no longer a
#     legal value on that axis (M0 review R2).
#   * the frame geometry is checked field by field against the codec, so a
#     pre-codec "7-byte header" claim is now a per-field rejection rather than
#     an accepted 9/19/24-byte budget.
#
# A ``twr-config/1`` document is REJECTED with a migration message, never
# silently misread: the two schemas disagree about what a PHR rate is, and
# reading one as the other would produce a config that validates and transmits
# something the author never asked for.
SCHEMA_VERSION = "twr-config/2"

#: The schema(s) this build knows how to MIGRATE FROM.  A document naming one
#: of these is rejected with a message naming the specific field that changed,
#: not with the generic "this build implements X" text used for a version it
#: has never heard of.
LEGACY_SCHEMA_VERSIONS: Tuple[str, ...] = ("twr-config/1",)

#: Why the v1 -> v2 migration is not automatic, stated once so both languages
#: emit the same sentence.
SCHEMA_V1_MIGRATION_REASON = (
    "twr-config/2 is a BREAKING change: frame.frame_profile is now REQUIRED "
    "(it selects the MAC layout the geometry claim is checked against), "
    "phy.phr_rate changed value domain from a payload DataRate that had to "
    "equal phy.data_rate to its own PhrRate enumeration, and the frame "
    "geometry is compared field by field with the frame codec "
    "(14-byte header, 5-byte timestamps, 2-byte PHR information, "
    "16/26/31-byte on-air frames). A twr-config/1 document is refused rather "
    "than silently misread, because the two schemas disagree about what a PHR "
    "rate is and misreading one as the other would accept a configuration that "
    "transmits something the author never asked for. To migrate: add "
    "frame.frame_profile = \"frame_v1\"; set phy.phr_rate = \"850k\" (the rate "
    "uwb_hrp_mod_core.h actually modulates) and leave phy.data_rate at the "
    "payload rate \"6p8m\"; then replace the frame.geometry object with "
    "frame_geometry_of_profile(frame.frame_profile) instead of a hand-written "
    "byte count; and set meta.schema_version = \"twr-config/2\"."
)

# Mirrors radar_meta::kRadarSyncRepetitions / sync_reps_supported().  These
# are the lengths the *modulator API* will accept; being in this list says
# nothing at all about whether the length transmits and decodes correctly.
RADAR_SYNC_REPETITIONS: Tuple[int, ...] = (
    1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048,
)

# IEEE 802.15.4a-2007 Table 68a BPRF preamble durations.  Anything else is a
# custom cropped/tiled SYNC field, not a standard preamble duration.
IEEE_802154A_PREAMBLE_SYMBOLS: Tuple[int, ...] = (16, 64, 1024, 4096)

# The CFO fit re-measures only the last max(cfo_min_fit_repetitions=32, 40) = 40
# repetitions and SYNTHESISES the earlier peak coordinates from the SFD-anchored
# grid (uwb_demod_core.h, demodulate_one).  Synthesised peaks carry a zero
# matched-filter value, so they must be excluded from the least-squares fit.
CFO_TAIL_REPETITIONS = 40

# Qm35825Profile::cfo_skip_initial_repetitions is hardcoded to 24, which
# coincides with 64 - 40 ONLY at 64 SYNC repetitions.  At any longer preamble
# the skip no longer covers every synthesised peak and the zero-phase points
# leak into the fit.
CFO_SKIP_INITIAL_REPETITIONS = 24

# Reason tokens, byte-identical to the ones the measured QA writes into
# testdata/twr/phy_matrix_737280000.csv, so a Python-side message and a CSV
# cell can be grepped against each other.
CFO_FIT_REASON_TEMPLATE = (
    "cfo_fit_includes_{tail_first}_zero_peak_corr_generated_peaks_because_"
    "cfo_skip_{skip}_lt_tail_first_{tail_first}_needs_cfo_skip_eq_preamble_"
    "minus_40"
)
# NOTE the capital "Y" in "syMBOLs": that typo is in the measured QA's reason
# string (qa_uwb_twr_phy_matrix.cc / phy_matrix_whitelist_737280000.csv) and is
# carried here VERBATIM on purpose, so a Python rejection message and a CSV cell
# can be grepped against each other.  Do not "fix" the spelling in one place
# only; fix it in the QA and regenerate the CSV.
PHR_DURATION_REASON_TEMPLATE = (
    "phr_preamble_duration_index_{idx}_advertises_{advertised}_syMBOLs_not_"
    "{requested}_ieee_802154a_cannot_express_this_length"
)
# The whitelist CSV's own cell for a length the modulator API refuses before
# it even modulates.  Carried verbatim, with no length number, because the
# surrounding message already names the requested length.
API_LIST_REASON = "sync_repetitions_outside_api_list_rejected_before_modulating"
#: The whitelist CSV's cell for a preamble code outside the API range.
CODE_INDEX_REASON = "code_index_outside_api_range_rejected_before_modulating"
#: The whitelist CSV's cells for the two stage failures.  The SHORT forms are
#: what the whitelist CSV records; the full matrix
#: (testdata/twr/phy_matrix_737280000.csv) records the LONG explanatory forms
#: below.  A rejection ROW must carry the short form to stay byte-equal to the
#: whitelist CSV; ``sync_repetition_reasons()`` uses the long form because its
#: job is to explain the mechanism.
STAGE_CFO_FAILED_REASON = "stage2_cfo_failed"
STAGE_CIR_FAILED_REASON = "stage4_cir_failed"
#: A derived explanation, matching the FULL matrix's reason cell.  A preamble
#: shorter than this leaves the CFO stage with fewer than four measured peaks,
#: so demodulate_one() cannot estimate a carrier offset at all (measured:
#: 1 and 2 SYNC -> CfoFailed, peaks == 1 and 2).
CFO_MIN_MEASURED_PEAKS = 4
CFO_MIN_PEAKS_REASON_TEMPLATE = (
    "stage2_cfo_failed_stage_cfo_requires_at_least_{min}_measured_peaks"
)
#: A derived explanation, matching the FULL matrix's reason cell.
#: Qm35825Profile::cir_skip_initial_repetitions is 10, so a preamble shorter
#: than that leaves no repetitions for the CIR estimator (measured: 4 and 8
#: SYNC -> CirFailed, peaks == 4 and 8).
CIR_SKIP_INITIAL_REPETITIONS = 10
CIR_SKIP_REASON_TEMPLATE = (
    "stage4_cir_failed_cir_skip_initial_repetitions_{skip}_not_smaller_than_"
    "available_{n}"
)
#: DERIVED ONLY -- not a measured CSV cell.  128/256/512/2048 are custom
#: profiles built by cropping/tiling a pulse-shaped SYNC field; they are not
#: IEEE 802.15.4a BPRF preamble durations (legal: 16/64/1024/4096).  The
#: measured matrix rejects them for OTHER, concrete reasons; this token records
#: the additional structural objection so an operator sees every reason at once.
ILLEGAL_LENGTH_REASON_TEMPLATE = (
    "sync_repetitions_{n}_is_not_a_legal_ieee_802154a_bprf_preamble_duration_"
    "legal_values_16_64_1024_4096"
)

# mod::encode_phr19 hardcodes PHR data-rate index 2 (6.81 Mb/s) and
# core::stage_payload_fcs always de-maps the payload with the 6.81 Mb/s
# geometry, so every other data rate is refused (qa_uwb_twr_phy_matrix.cc).
DATA_RATE_REJECT_REASON = (
    "mod_encode_phr19_hardcodes_6_81_mbps_index_2_and_stage_payload_fcs_"
    "always_uses_6_81_mbps_geometry"
)

# ---------------------------------------------------------------------------
# SYNC lengths that are decode-verified but whose FIRST-PATH / ToA ACCURACY
# has not been measured yet.
# ---------------------------------------------------------------------------
# Ranging accuracy is a function of first-path ToA, so a length may sit in the
# capability whitelist -- it round-trips with a byte-exact FCS -- and still be
# unfit as a RANGING profile.  16 SYNC is in that set because its integration
# window is 16.3 us against 64 SYNC's 65.1 us, so the first path is estimated
# over a quarter of the observation length.
#
# This is ADMITTED, not rejected: the measurement is unambiguous that 16 SYNC
# decodes, self-describes and tracks carrier offset.  What is unverified is
# only the ToA accuracy, and that is a statement about M2's first-path
# validation, not about this whitelist.
#
# Mirrors C++ twr::kTwrSyncRepsNeedingToaValidation and
# twr::twr_sync_reps_needs_toa_validation().
SYNC_REPS_NEEDING_TOA_VALIDATION = (16,)

#: The whitelist CSV's cell carried on every 16 SYNC row.
SYNC_16_CAVEAT_REASON = (
    "decode_verified_only_first_path_toa_accuracy_unverified_for_16_sync_"
    "shorter_integration_window"
)
#: The honest final statement about 16 SYNC, replacing the earlier
#: "verdict pending re-measurement" text.  The re-measurement against a
#: freshly rebuilt library is DONE; what remains open is the first-path / ToA
#: accuracy measurement, which is a different question and belongs to M2.
SYNC_16_TOA_UNVERIFIED_REASON = (
    "sync_16_is_measured_and_admitted_decode_verified_and_self_describing_"
    "phr_preamble_duration_index_0_equals_16_which_is_legal_802154a_bprf_and_"
    "cfo_zero_peak_corr_is_0_because_its_measured_tail_covers_the_whole_"
    "preamble_so_no_synthesised_zero_phase_peak_can_enter_the_fit_but_its_"
    "first_path_toa_accuracy_is_UNVERIFIED_for_the_shorter_16p3_us_"
    "integration_window_versus_65p1_us_at_64_sync_and_ranging_accuracy_"
    "depends_on_exactly_that_quantity_so_16_sync_must_not_be_used_as_a_"
    "ranging_profile_until_m2_measures_it"
)
#: The C++ capability-row reason strings, byte-identical to
#: uwb_twr_config.h::build_default_capabilities().
MEASURED_ROW_REASON = (
    "measured_fcs_pass_round_trip_see_testdata/twr/phy_matrix_737280000.csv"
)
MEASURED_ROW_REASON_TOA_UNVERIFIED = (
    "measured_fcs_pass_round_trip_but_first_path_toa_accuracy_unverified_see_"
    "testdata/twr/phy_matrix_737280000.csv"
)

PENDING_MATRIX_REASON = (
    "PHY capability matrix not measured yet; see "
    "testdata/twr/phy_matrix_whitelist_737280000.csv"
)

UNSUPPORTED_STS_REASON = (
    "STS is out of scope for phase 1 (REQ-SCOPE-04, explicit user decision)"
)

PHY_MATRIX_WHITELIST_CSV = "testdata/twr/phy_matrix_whitelist_737280000.csv"

# Measured native-rate round trip of a TWR-sized PSDU is impossible in this
# repository: there is no work -> native decimator, only native -> work
# resamplers.  A 737.28 MS/s loopback therefore proves the *work grid* only.
NATIVE_ROUNDTRIP_BLOCKED_REASON = (
    "no_work_to_native_decimator_in_repo_so_a_twr_psdu_cannot_be_modulated_"
    "on_the_native_grid"
)


# ---------------------------------------------------------------------------
# THE FRAME CODEC'S FROZEN LAYOUT CONSTANTS (M0.1, R3)
# ---------------------------------------------------------------------------
# Mirrors uwb_twr_frame.h.  These are the ONE set of numbers in the Python
# layer: there is no second "7-byte header" geometry anywhere in this file, and
# a test fixture fills a claim from them rather than typing literals.
#
#   kFrameHeaderBytes = kOffTimestamps = 14
#   kFrameFcsBytes    = 2            (IEEE 802.15.4, appended by the PHY)
#   kPhyMaxPsduBytes  = 127
#   timestamp_bits    = 40           -> 5 bytes
#   kPhrStandardInfoBytes = 2, coded to kPhrStandardCodedBits = 19 over 21
#   symbols
#
# M0 review R3 is the reason this block exists at all: the Python schema
# carried a pre-codec geometry with 7-byte headers and 9/19/24-byte budgets
# while the codec had emitted a 14-byte header and 16/26/31-byte frames since
# it was frozen.  Both suites passed because nothing compared them.
FRAME_HEADER_BYTES = 14
FRAME_FCS_BYTES = 2
FRAME_MAX_PSDU_BYTES = 127
FRAME_TIMESTAMP_BITS = 40
FRAME_TIMESTAMP_BYTES = FRAME_TIMESTAMP_BITS // 8  # 5
FRAME_MAX_TIMESTAMPS = 3                            # Final carries three
PHR_STANDARD_INFO_BYTES = 2
PHR_STANDARD_CODED_BITS = 19
PHR_STANDARD_SYMBOLS = 21
FRAME_VERSION = 0x01
#: Offsets are the specification; the header size IS the first timestamp's
#: offset, so there is exactly one definition of 14 here.
FRAME_OFF_TIMESTAMPS = FRAME_HEADER_BYTES


# ===========================================================================
# 1. Text and number formatting (byte-compatible with the C++ helpers)
# ===========================================================================
#
# `double_to_text` uses "%.17g" rather than repr() on purpose: the C++ side
# uses snprintf("%.17g") and the two must agree character for character or the
# canonical field text -- and therefore config_hash -- would differ between
# languages for the same config.


def double_to_text(v: float) -> str:
    """``%.17g`` -- round-trips every finite IEEE 754 double exactly."""
    return "%.17g" % v


def int_to_text(v: int) -> str:
    """Plain decimal int64 text, as the C++ helper spells it."""
    return "%d" % v


def bool_to_text(v: bool) -> str:
    """``true``/``false``, the spelling the C++ helper and JSON share."""
    return "true" if v else "false"


def duration_to_text(d: "Duration") -> str:
    """A duration as integer nanoseconds with the ``ns`` suffix."""
    return int_to_text(d.nanos()) + "ns"


def _is_finite(v: float) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def rate_matches_strict(requested: float, readback: float,
                        rel_tol: float = NATIVE_RATE_TOLERANCE_REL) -> bool:
    """Relative comparison; the X410 NCO resolves far below 1 Hz."""
    if not (rel_tol > 0.0) or not _is_finite(requested) or not _is_finite(readback):
        return False
    scale = requested if requested > readback else readback
    return abs(readback - requested) <= rel_tol * scale


def is_allowed_native_rate(hz: float) -> bool:
    """Only 737.28 MS/s (X410/UC200) and 491.52 MS/s (CG400).

    REQ-PHY-02: the native device rate is a DIFFERENT quantity from the
    998.4 MS/s work rate, the symbol rate and the RF bandwidth.
    """
    return rate_matches_strict(NATIVE_RATE_UC200_HZ, hz) or \
        rate_matches_strict(NATIVE_RATE_CG400_HZ, hz)


def is_allowed_quantisation_hz(hz: float) -> bool:
    """Quantisation rates a ``TimedField`` may name.

    Anything finer than 500 MHz (i.e. a sub-2 ns device tick) is rejected: the
    device tick cannot express it, so demanding it is a REQ-API-01 timing
    precision failure rather than something to round away.
    """
    if not _is_finite(hz) or hz <= 0.0:
        return False
    if hz == 1.0e9:  # integer nanoseconds
        return True
    if hz == WORK_SAMPLE_RATE_HZ:
        return True
    return is_allowed_native_rate(hz)


def code_index_supported(n: int) -> bool:
    """True for the preamble codes the modulator accepts (9..12)."""
    return CODE_INDEX_MIN <= n <= CODE_INDEX_MAX


def sync_reps_supported(n: int) -> bool:
    """Mirrors radar_meta::sync_reps_supported() (modulator API list only)."""
    return n in RADAR_SYNC_REPETITIONS


# Device tick periods, for documentation and for the precision messages.
NATIVE_TICK_NS = {
    NATIVE_RATE_UC200_HZ: 1.0e9 / NATIVE_RATE_UC200_HZ,   # 1.3563 ns
    NATIVE_RATE_CG400_HZ: 1.0e9 / NATIVE_RATE_CG400_HZ,   # 2.0345 ns
    WORK_SAMPLE_RATE_HZ: 1.0e9 / WORK_SAMPLE_RATE_HZ,     # 1.0016 ns
    1.0e9: 1.0,                                          # 1 ns
}


# ===========================================================================
# 2. Enums
# ===========================================================================


class _StrEnum(str, Enum):
    """A string-valued enum with a stable, round-trippable spelling.

    ``str(x)`` / ``f"{x}"`` / ``x == "text"`` all give the wire spelling, so a
    value can go into a message, an f-string or a dict key without a
    conversion step that could be forgotten.
    """

    def __str__(self) -> str:  # pragma: no cover - trivial
        return str.__str__(self)

    __format__ = str.__format__  # type: ignore[assignment]

    @classmethod
    def from_string(cls, s: str) -> Optional["_StrEnum"]:
        """Exact-spelling lookup; returns ``None`` for anything unknown."""
        for member in cls:
            if member.value == s:
                return member
        return None

    @classmethod
    def from_string_any(cls, *spellings: str) -> Optional["_StrEnum"]:
        for s in spellings:
            found = cls.from_string(s)
            if found is not None:
                return found
        return None

    @classmethod
    def values(cls) -> List[str]:
        return [m.value for m in cls]


class Protocol(_StrEnum):
    """Two-Way Rounding algorithm.

    SS computes at the initiator using the responder's (t2B, t3B); DS computes
    at the responder using its own (t2B, t3B, t6B) plus the initiator's times
    carried in Final.
    """

    SS = "ss"
    DS = "ds"

    @classmethod
    def from_string(cls, s: str) -> Optional["Protocol"]:
        # Accepts the C++ spellings, including the upper-case and long forms.
        if s in ("ss", "SS", "single"):
            return cls.SS
        if s in ("ds", "DS", "double"):
            return cls.DS
        return None


class Role(_StrEnum):
    """Physical channel map is a separate config axis, never encoded here."""

    INITIATOR = "initiator"
    RESPONDER = "responder"


class FrameType(_StrEnum):
    """Wire function codes.  The values are part of the versioned profile."""

    POLL = "poll"
    RESPONSE = "response"
    FINAL = "final"
    #: RESERVED.  Phase 1 deliberately has no Result/Report message: the
    #: three-message reference profile does not emit one and adding a fourth
    #: would invalidate the interop claim (REQ-PROTO-05).  The code point
    #: exists so a future profile can claim it, and the validator rejects it.
    REPORT = "report"


def frame_type_timestamp_count(t: FrameType) -> int:
    """Number of 40-bit timestamps a frame type carries (profile-fixed)."""
    return {
        FrameType.POLL: 0,
        FrameType.RESPONSE: 2,   # t2B, t3B
        FrameType.FINAL: 3,      # t1A, t4A, t5A
        FrameType.REPORT: 0,     # reserved / not implemented
    }[t]


class TimestampMarker(_StrEnum):
    """REQ-TIME-02: these are physically different instants, never one number.

    ``UHD_RX_FIRST_IQ_SAMPLE`` needs resampler group delay, filter delay,
    window crop and rate conversion before it means anything on air; the
    ``RMARKER_*`` values are the protocol markers exchanged between endpoints.
    """

    UHD_RX_FIRST_IQ_SAMPLE = "uhd_rx_first_iq_sample"
    PREAMBLE_START = "preamble_start"
    SFD_START = "sfd_start"
    PHR_START = "phr_start"
    RMARKER_TX = "rmarker_tx"
    RMARKER_RX = "rmarker_rx"
    ANTENNA_PLANE = "antenna_plane"


class TimestampSource(_StrEnum):
    """A timed command that returned success is NOT a measured timestamp."""

    UNKNOWN = "unknown"
    HARDWARE_MEASURED = "hardware_measured"
    SCHEDULED_CALIBRATED = "scheduled_calibrated"
    ESTIMATED = "estimated"
    RECONSTRUCTED = "reconstructed"


def timestamp_source_is_measurement(s: TimestampSource) -> bool:
    return s in (TimestampSource.HARDWARE_MEASURED,
                 TimestampSource.SCHEDULED_CALIBRATED)


class EndpointState(_StrEnum):
    IDLE = "idle"
    POLL_SENT = "poll_sent"
    RESPONSE_RECEIVED = "response_received"
    FINAL_SENT = "final_sent"
    POLL_RECEIVED = "poll_received"
    FINAL_RECEIVED = "final_received"


class ExchangeStatus(_StrEnum):
    """Every exchange terminates with exactly ONE of these (REQ-ERR-01).

    Numeric values are part of the JSON/CSV schema (REQ-OUT-01); append only.
    There is no "recovered" path: a failed measurement is reported as a
    failure, never patched up by skipping a slot and continuing.
    """

    OK = "ok"

    # --- configuration / lifecycle -----------------------------------------
    CONFIG_REJECTED = "config_rejected"
    UNSUPPORTED = "unsupported"
    CANCELLED = "cancelled"
    QUEUE_FULL = "queue_full"
    INTERNAL_ERROR = "internal_error"

    # --- PHY / frame --------------------------------------------------------
    PHY_FCS_FAILED = "phy_fcs_failed"
    PHY_DECODE_FAILED = "phy_decode_failed"
    WRONG_PEER = "wrong_peer"
    UNEXPECTED_FRAME_TYPE = "unexpected_frame_type"
    STALE_SESSION = "stale_session"

    # --- receive ------------------------------------------------------------
    RX_TIMEOUT = "rx_timeout"
    RX_OVERFLOW = "rx_overflow"
    RX_CHAIN_BROKEN = "rx_chain_broken"

    # --- transmit -----------------------------------------------------------
    TX_LATE = "tx_late"
    TX_UNDERFLOW = "tx_underflow"
    TX_SEQ_ERROR = "tx_seq_error"
    TX_CHAIN_BROKEN = "tx_chain_broken"

    # --- timing / signal quality -------------------------------------------
    INVALID_TIME_DOMAIN = "invalid_time_domain"
    CLOCK_ESTIMATE_INVALID = "clock_estimate_invalid"
    FIRST_PATH_UNRELIABLE = "first_path_unreliable"

    # --- configuration state at runtime ------------------------------------
    CALIBRATION_MISSING = "calibration_missing"
    CALIBRATION_EXPIRED = "calibration_expired"

    # --- protocol -----------------------------------------------------------
    PROTOCOL_TIMEOUT = "protocol_timeout"
    DEADLINE_MISSED = "deadline_missed"


def exchange_status_is_ok(s: ExchangeStatus) -> bool:
    return s == ExchangeStatus.OK


def exchange_status_yields_range(s: ExchangeStatus) -> bool:
    """A validated range is only defined for a fully successful exchange."""
    return s == ExchangeStatus.OK


_STATUS_FAMILY = {
    ExchangeStatus.OK: "ok",
    ExchangeStatus.CONFIG_REJECTED: "config",
    ExchangeStatus.UNSUPPORTED: "config",
    ExchangeStatus.CANCELLED: "lifecycle",
    ExchangeStatus.QUEUE_FULL: "host",
    ExchangeStatus.INTERNAL_ERROR: "host",
    ExchangeStatus.PHY_FCS_FAILED: "phy",
    ExchangeStatus.PHY_DECODE_FAILED: "phy",
    ExchangeStatus.WRONG_PEER: "protocol",
    ExchangeStatus.UNEXPECTED_FRAME_TYPE: "protocol",
    ExchangeStatus.STALE_SESSION: "protocol",
    ExchangeStatus.RX_TIMEOUT: "rx",
    ExchangeStatus.RX_OVERFLOW: "rx",
    ExchangeStatus.RX_CHAIN_BROKEN: "rx",
    ExchangeStatus.TX_LATE: "tx",
    ExchangeStatus.TX_UNDERFLOW: "tx",
    ExchangeStatus.TX_SEQ_ERROR: "tx",
    ExchangeStatus.TX_CHAIN_BROKEN: "tx",
    ExchangeStatus.INVALID_TIME_DOMAIN: "signal",
    ExchangeStatus.CLOCK_ESTIMATE_INVALID: "signal",
    ExchangeStatus.FIRST_PATH_UNRELIABLE: "signal",
    ExchangeStatus.CALIBRATION_MISSING: "calibration",
    ExchangeStatus.CALIBRATION_EXPIRED: "calibration",
    ExchangeStatus.PROTOCOL_TIMEOUT: "deadline",
    ExchangeStatus.DEADLINE_MISSED: "deadline",
}


def exchange_status_family(s: ExchangeStatus) -> str:
    return _STATUS_FAMILY.get(s, "invalid")


# --- config-level enums ----------------------------------------------------


class PrfClass(_StrEnum):
    """A PRF CLASS, not a rate: 802.15.4a pairs a class with a channel.

    62.4 MHz is the BPRF value MEASURED by this code base; 64 MHz is the HRP
    nominal.  Neither may be silently substituted for the other.
    """

    BPRF64 = "bprf64"
    HPRF64 = "hprf64"
    HPRF400 = "hprf400"


class DataRate(_StrEnum):
    """PAYLOAD data-rate CLASS.

    Only ``6p8m`` is enabled; see ``DATA_RATE_REJECT_REASON``.  This is the
    rate of the frame BODY, and it is a DIFFERENT axis from :class:`PhrRate`
    below: M0 typed ``phr_rate`` as one of these and required it to equal
    ``data_rate``, which was the exact inverse of what this repository
    transmits.
    """

    R850K = "850k"
    R6P8M = "6p8m"
    R27M = "27m"
    R7P8M = "7p8m"
    R27P2M = "27p2m"
    R6P8M_HPRF = "6p8m_hprf"


# ---------------------------------------------------------------------------
# PHR RATE -- a SEPARATE quantity from the payload data rate (M0.1, R2)
# ---------------------------------------------------------------------------
# Mirrors ``enum class PhrRate`` in uwb_twr_config.h, and the argument there in
# full: uwb_hrp_mod_core.h modulates a 0.85 Mb/s PHR (21 SEC-DED-coded symbols
# of 512 chips) ahead of a 6.81 Mb/s payload (8 chips per burst of 64 per
# symbol).  The PHR is a different modulation at a different rate.  The 2-bit
# data-rate field that lives INSIDE the PHR describes the PAYLOAD that follows
# it and says nothing about how the PHR itself is transmitted.
#
# So the M0 rule accepted a 6.81 Mb/s PHR -- which this modulator cannot
# produce -- and rejected the 0.85 Mb/s PHR it does produce.  A dedicated enum
# is the fix: the two axes are separately typed, separately named and
# separately validated, and this one carries only PHR rates.
class PhrRate(_StrEnum):
    #: 851.2 kb/s, the fixed PHR rate of an HRP BPRF profile.  THIS IS THE ONE
    #: uwb_hrp_mod_core.h modulates.
    STANDARD_850K = "850k"
    #: "Transmit the PHR at the payload data rate."  A legal option in IEEE
    #: 802.15.4 and an explicit enum in the Qorvo DW3xxx API guide.
    #:
    #: It is an enum member so the config surface can NAME an option it does
    #: not implement, instead of encoding the wish as a payload rate.  It is
    #: rejected, and the reason is a limit of THIS IMPLEMENTATION: the
    #: modulator has exactly one PHR code path and it is the
    #: 512-chips-per-symbol one.
    SAME_AS_DATA = "same_as_data"


#: The complete set, so a validator can enumerate what it may offer.
ALL_PHR_RATES: Tuple[PhrRate, ...] = (PhrRate.STANDARD_850K, PhrRate.SAME_AS_DATA)


def phr_rate_is_implemented(v: PhrRate) -> bool:
    """True only for a PHR rate this build can actually TRANSMIT.

    A member that is a legitimate protocol option with no implementation is
    refused; being a named option is not the same as being a runnable one.
    """
    return v is PhrRate.STANDARD_850K


def phr_rate_symbols(v: PhrRate) -> int:
    """PHR symbols per frame of the implemented PHR: 21.

    0 for a rate with no implementation, so it can never be mistaken for a
    real budget.
    """
    return PHR_STANDARD_SYMBOLS if phr_rate_is_implemented(v) else 0


def phr_rate_unsupported_reason(v: PhrRate) -> str:
    """Why a PHR-rate member is refused, in its own words.

    Every member that is not implemented has its own reason here, so no
    rejection of this field is ever a bare "unsupported" with nothing behind
    it.  The wording is deliberately a statement about THIS SOFTWARE, never
    about any Qorvo part.
    """
    if v is PhrRate.STANDARD_850K:
        return "the 851.2 kb/s PHR is the rate uwb_hrp_mod_core.h modulates"
    if v is PhrRate.SAME_AS_DATA:
        return (
            "phr_rate=same_as_data is a legal 802.15.4 / DW3xxx API option, but "
            "this build has exactly ONE PHR code path and it is the fixed "
            "0.85 Mb/s one (21 symbols of 512 chips, uwb_hrp_mod_core.h "
            "kPhrSymbols / kPhrChipsPerSymbol). There is no same-as-data PHR "
            "implementation to select, so the request is refused rather than "
            "silently transmitted as a 0.85 Mb/s PHR. This is a limit of the "
            "software, not a claim about any Qorvo part.")
    return ("phr_rate %s is not a member of the PHR-rate enumeration" % (v,))


# ---------------------------------------------------------------------------
# PREAMBLE (SYNC) LENGTH -- the lengths THIS decoder / CFO path implements
# ---------------------------------------------------------------------------
# Mirrors ``enum class PreambleLength`` in uwb_twr_config.h.
#
# This is deliberately an enumeration of what the current software can RUN,
# not of what the standard or any vendor part can do.  What limits this build
# is the DEMODULATOR: the CFO re-measurement in uwb_demod_core.h re-fits only
# the last max(cfo_min_fit_repetitions, 40) SYNCs and synthesises the earlier
# peaks at phase 0, so the number of zero-phase points that leak into the
# least-squares fit is max(0, reps - 64) and 64 is the first clean length.
# Separately, mod::encode_phr19 maps 128 / 256 / 512 to preamble-duration
# index 1 (= 64), so the PHR does not self-describe those lengths.
#
# Qorvo's own API accepts 64 / 128 / 256 / 512 / 1024 / 2048, and the DW3xxx
# guide lists 128 / 256 / 512 as non-standard-but-supported.  None of that is
# in dispute, which is exactly why "the chip cannot do 128" would be a false
# statement about hardware this project has never spoken to.
class PreambleLength(_StrEnum):
    #: The other measured-clean length: its tail covers the whole preamble, so
    #: no zero-phase point reaches the fit, and it is self-describing
    #: (preamble-duration index 0 = 16).  Its first-path / ToA accuracy is
    #: still unmeasured, a separate caveat carried by
    #: ``sync_reps_needing_toa_validation``.
    SYM16 = "16"
    #: 64 SYNC: 65.1 us integration window, preamble-duration index 1.
    SYM64 = "64"


ALL_PREAMBLE_LENGTHS: Tuple[PreambleLength, ...] = (
    PreambleLength.SYM16, PreambleLength.SYM64)


def preamble_length_symbols(v: PreambleLength) -> int:
    """The SYNC repetition count of a length.  0 for a non-member."""
    return {PreambleLength.SYM16: 16, PreambleLength.SYM64: 64}.get(v, 0)


def preamble_length_from_symbols(symbols: int) -> Optional[PreambleLength]:
    """The reverse mapping, for the raw ``phy.preamble_symbols`` field.

    ``None`` means "not a length this build implements" -- never "round to the
    nearest".
    """
    for v in ALL_PREAMBLE_LENGTHS:
        if preamble_length_symbols(v) == symbols:
            return v
    return None


def preamble_length_unsupported_reason(symbols: int) -> str:
    """Why a SYNC repetition count is refused.

    Each refused length gets its own measured reason, and every one of them
    names a limit of THIS SOFTWARE rather than of a chip.  Byte-identical to
    C++ ``preamble_length_unsupported_reason()``, so the two languages cannot
    disagree about what was measured.
    """
    n = int_to_text(symbols)
    if symbols == 32:
        return ("preamble length %s SYNC is refused: mod::encode_phr19 "
                "advertises preamble-duration index 1 (= 64) for it, so the "
                "PHR does not self-describe a 32-symbol preamble" % n)
    if symbols in (128, 256, 512):
        return ("preamble length %s SYNC is refused by the CURRENT DEMODULATOR,"
                " not by any hardware: uwb_demod_core.h re-measures only the "
                "last max(cfo_min_fit_repetitions, 40) SYNCs and synthesises "
                "the earlier peaks at phase 0, so the zero-phase points that "
                "leak into the least-squares CFO fit number max(0, reps - 64) "
                "-- measured at 128 SYNC as an injected 20 kHz offset coming "
                "back as 3791 Hz; and mod::encode_phr19 maps %s to "
                "preamble-duration index 1 (= 64), so the PHR does not "
                "self-describe it. Qorvo's API accepts these lengths (DW3xxx "
                "lists 128/256/512 as non-standard preamble lengths), so this is "
                "a limit of this software path, not a statement about what a "
                "DW1000 or DW3000 can transmit" % (n, n))
    if symbols == 1024:
        return ("preamble length %s SYNC is PHR-legal (preamble-duration index "
                "2) but refused by the current demodulator for the same "
                "CFO-fit bias as 128/256/512; a software limit, not a hardware "
                "one" % n)
    if symbols == 2048:
        return ("preamble length %s SYNC exceeds the work-grid TX buffer this "
                "build sizes for one frame (mod::kMaxHrpTxSamples); a length "
                "this large is a buffer and scheduling decision, not a hardware "
                "capability statement" % n)
    if symbols in (1, 2):
        return ("preamble length %s SYNC leaves fewer than the 4 measured peaks "
                "uwb_demod_core.h stage_cfo needs" % n)
    if symbols in (4, 8):
        return ("preamble length %s SYNC is shorter than the 10 initial "
                "repetitions uwb_demod_core.h skips before the CIR search" % n)
    return ("preamble length %s is not a SYNC repetition count this build "
            "implements: the enumeration is 16 or 64, bounded by this "
            "repository's demodulator/CFO path (see the PreambleLength "
            "comment). That is a software limit; the Qorvo parts' own API "
            "accepts 64 through 2048" % n)


# ---------------------------------------------------------------------------
# Rejection scope, and the vendor citation behind it (M0.1, R7)
# ---------------------------------------------------------------------------
# A rejection is always attributed to something.  Naming the scope is what
# stops "our decoder can't" from hardening into "the chip can't".
def reject_scope_current_software() -> str:
    return "current_software_implementation_limit"


def reject_scope_out_of_scope() -> str:
    """Reject-scope tag: deliberately out of scope for phase 1 (e.g. STS)."""
    return "out_of_scope_phase1"


def reject_scope_not_measured() -> str:
    """Reject-scope tag: nobody looked, distinct from looked-and-failed."""
    return "not_measured"


def reject_scope_api_contract() -> str:
    """Reject-scope tag: refused by the API contract, not by measurement."""
    return "this_build_api_contract"


#: Kept as data so the claim is greppable and can be re-verified against the
#: PDFs rather than living only in a comment.
def vendor_preamble_length_citation() -> str:
    return ("qorvo_supports_these_lengths_see_dw1000_api_guide_v2.7_sec5.12_"
            "txPreambLength_and_dw3xxx_api_guide_pdf_pp32-34_preamble_"
            "enumeration_including_128_256_512_listed_as_non_standard_"
            "preamble_lengths")


def preamble_length_supported_by_vendor(n: int) -> bool:
    """True for the lengths this build refuses for decoder/encoder reasons.

    It is emphatically NOT a statement that the hardware lacks them.
    """
    return n in (64, 128, 256, 512, 1024, 2048)


def preamble_length_reject_note(n: int) -> str:
    """The note carried alongside a 128/256/512/1024/2048 rejection.

    Byte-identical to C++ ``preamble_length_reject_note()``.  The word
    "structurally impossible" appears nowhere: what fails is THIS decoder, and
    the citation says so.
    """
    s = reject_scope_current_software()
    s += "_preamble_symbols_%d_refused_by_this_build_only_" % n
    if n == 1024:
        s += ("uwb_demod_core_stage_cfo_refits_only_the_last_40_reps_while"
              "_cfo_skip_initial_repetitions_stays_24_so_synthesised_zero_phase"
              "_peaks_enter_the_phase_fit_measured_error_19901_Hz_at_20_kHz_")
    else:
        s += ("uwb_demod_core_stage_cfo_refits_only_the_last_40_reps_while"
              "_cfo_skip_initial_repetitions_stays_24_so_synthesised_zero_phase"
              "_peaks_enter_the_phase_fit_and_mod_encode_phr19_maps_the_length"
              "_onto_the_4_value_preamble_duration_index_so_the_phr_does_not_"
              "self_describe_")
    if preamble_length_supported_by_vendor(n):
        s += "not_a_hardware_claim_" + vendor_preamble_length_citation()
    return s


def phr_mode_unsupported_reason(v: "PhrMode") -> str:
    """Why a PHR-form member is refused, in its own words.

    M0 caught only ``none`` and let ``extended`` through as a valid profile.
    Every member now carries its own reason.
    """
    if v is PhrMode.STANDARD:
        return ("the standard 13-bit (2 octet) HRP PHR is the one frame v1 "
                "encodes")
    if v is PhrMode.EXTENDED:
        return ("phr_mode=extended asks for a vendor PHR layout longer than the "
                "13 information bits frame v1 encodes (2 octets, SEC-DED coded "
                "to 19 bits over 21 symbols). No extended-PHR layout exists in "
                "this codec or in uwb_hrp_mod_core.h, so the request is refused "
                "rather than transmitted as a standard PHR. This is a limit of "
                "this software, not a claim about any Qorvo part")
    if v is PhrMode.NONE:
        return ("a TWR frame without a PHR is not a valid TWR profile; the PHR "
                "carries the RANGING bit and the PSDU length the receiver needs "
                "before it can decode the payload")
    return "phr_mode %s is not a member of the PHR-mode enumeration" % (v,)


class SfdMode(_StrEnum):
    R4Z1 = "4z1"
    R4Z2 = "4z2"
    R4Z3 = "4z3"
    R4Z4 = "4z4"
    DWT8 = "decawave"
    IEEE8 = "ieee"


def sfd_mode_symbols(v: SfdMode) -> int:
    """SYMBOL count of each SFD (the sequence lengths in GetSfdSequence)."""
    return {
        SfdMode.R4Z1: 4,
        SfdMode.R4Z2: 8,
        SfdMode.R4Z3: 16,
        SfdMode.R4Z4: 32,
        SfdMode.DWT8: 8,
        SfdMode.IEEE8: 8,
    }[v]


class PhrMode(_StrEnum):
    """PHR PRESENCE/FORM -- a different axis from the PHR RATE.

    This says whether a PHR is transmitted at all and which layout carries it;
    :class:`PhrRate` says at what rate.  Both axes were present in M0; what was
    missing is that each member gets its own reason, instead of only ``none``
    being caught while ``extended`` passed validation silently.
    """

    STANDARD = "standard"
    #: A vendor PHR layout longer than the 13 information bits frame v1
    #: encodes.  This codec and this modulator have no layout for it.
    EXTENDED = "extended"
    #: Not a UWB frame at all: the PHR is what carries the RANGING bit and the
    #: PSDU length a receiver needs before it can decode the payload.
    NONE = "none"


# ---------------------------------------------------------------------------
# THE EVIDENCE LADDER (M0.1, R7)
# ---------------------------------------------------------------------------
# Mirrors uwb_twr_capability_evidence.h.  The M0 whitelist overstates its
# evidence: every one of the 318 rows in testdata/twr/phy_matrix_737280000.csv
# carries ``path = work_direct_998p4*`` -- a full modulate -> loopback ->
# demodulate -> byte-exact-FCS round trip on THIS repo's 998.4 MS/s work grid.
# That is a real, useful result, and it is exactly ONE level of evidence.
#
# The ladder is MONOTONIC and CUMULATIVE: establishing a higher level
# establishes every lower one, because each stage strictly contains the
# previous one as a precondition.
class EvidenceLevel(_StrEnum):
    #: nothing measured
    NONE = "none"
    #: a TWR-sized PSDU was modulated, looped back and demodulated
    #: byte-exactly on this repo's work grid (998.4 MS/s).  Proves TX and RX
    #: agree with each OTHER.  Proves nothing about a radio.
    WORK_DECODE_VERIFIED = "work_decode_verified"
    #: additionally: the frame actually traversed the native sample rate
    #: (work -> native -> work) with the resampler and its time mapping under
    #: test.  Not established by any M0 row.
    NATIVE_ROUNDTRIP_VERIFIED = "native_roundtrip_verified"
    #: additionally: first-path / ToA accuracy measured against an independent
    #: oracle.  Ranging accuracy is a function of exactly this quantity, which
    #: is why a decode-verified row may NOT be used to produce a range.
    TOA_VERIFIED = "toa_verified"
    #: additionally: measured on real hardware, with the FPGA/UHD versions and
    #: the clock source recorded.
    HARDWARE_VERIFIED = "hardware_verified"
    #: additionally: measured against a NAMED module with its SDK / firmware
    #: hash and frame bytes recorded.
    VENDOR_INTEROP_VERIFIED = "vendor_interop_verified"


#: The complete ladder, weakest first.  ``not_established_list()`` walks it.
EVIDENCE_LEVEL_ORDER: Tuple[EvidenceLevel, ...] = (
    EvidenceLevel.NONE,
    EvidenceLevel.WORK_DECODE_VERIFIED,
    EvidenceLevel.NATIVE_ROUNDTRIP_VERIFIED,
    EvidenceLevel.TOA_VERIFIED,
    EvidenceLevel.HARDWARE_VERIFIED,
    EvidenceLevel.VENDOR_INTEROP_VERIFIED,
)

EVIDENCE_LEVEL_COUNT = len(EVIDENCE_LEVEL_ORDER)


def evidence_level_index(v: EvidenceLevel) -> int:
    """Position of `l` on the ladder, or -1 when it is not a level."""
    return EVIDENCE_LEVEL_ORDER.index(v)


def evidence_level_valid(v: EvidenceLevel) -> bool:
    """True when `l` is one of the five ladder levels (None excluded)."""
    return v in EVIDENCE_LEVEL_ORDER


def evidence_level_implies(a: EvidenceLevel, b: EvidenceLevel) -> bool:
    """``a`` is at least as strong as ``b``."""
    return evidence_level_index(a) >= evidence_level_index(b)


class EvidenceUse(_StrEnum):
    """The level a given USE requires before it may be attempted.

    A decode-only row is usable for offline codec work; producing a range
    needs ToA; putting it on a radio needs hardware; talking to a DW1000 needs
    a named module.
    """

    WORK_DECODE = "work_decode"          # offline codec / loopback self-consistency
    NATIVE_ROUNDTRIP = "native_roundtrip"  # frame must survive the native grid
    RANGING = "ranging"                  # must yield a first path / ToA
    HARDWARE = "hardware"                # must run on a real radio
    VENDOR_INTEROP = "vendor_interop"    # must interop with a named module


EVIDENCE_USE_ORDER: Tuple[EvidenceUse, ...] = (
    EvidenceUse.WORK_DECODE, EvidenceUse.NATIVE_ROUNDTRIP, EvidenceUse.RANGING,
    EvidenceUse.HARDWARE, EvidenceUse.VENDOR_INTEROP)


def evidence_required_level(u: EvidenceUse) -> EvidenceLevel:
    """The ladder rung a given use requires; Ranging needs ToaVerified."""
    return {
        EvidenceUse.WORK_DECODE: EvidenceLevel.WORK_DECODE_VERIFIED,
        EvidenceUse.NATIVE_ROUNDTRIP: EvidenceLevel.NATIVE_ROUNDTRIP_VERIFIED,
        EvidenceUse.RANGING: EvidenceLevel.TOA_VERIFIED,
        EvidenceUse.HARDWARE: EvidenceLevel.HARDWARE_VERIFIED,
        EvidenceUse.VENDOR_INTEROP: EvidenceLevel.VENDOR_INTEROP_VERIFIED,
    }[u]


def evidence_allows(established: EvidenceLevel, u: EvidenceUse) -> bool:
    """Never widens a claim: it only ever fails closed."""
    return evidence_level_implies(established, evidence_required_level(u))


def evidence_explain(established: EvidenceLevel, u: EvidenceUse) -> str:
    """The fail-closed explanation.

    Names both the missing level and where it would have to be measured, so an
    operator can tell "not looked at" from "looked at and failed".
    """
    need = evidence_required_level(u)
    if evidence_allows(established, u):
        return "evidence_ok:%s covers use=%s" % (established, u)
    return ("evidence_insufficient:use=%s_requires_%s_but_only_%s_is_established"
            "_unverified_stays_rejected_REQ_SCOPE_01" % (u, need, established))


def not_established_list(established: EvidenceLevel) -> str:
    """The levels a row explicitly does NOT claim, ``;``-separated.

    Written out so the absence is DATA rather than a comment: a row that says
    "work decode verified" must also say, in the same record, that native /
    ToA / hardware / vendor are not.
    """
    out: List[str] = []
    for level in EVIDENCE_LEVEL_ORDER:
        if level is EvidenceLevel.NONE:
            continue
        if evidence_level_implies(established, level):
            continue
        out.append(str(level))
    return ";".join(out)  # empty when every level is established


#: M0.1 R7: the level the 48 measured rows ACTUALLY establish, and the
#: artifact that establishes it.  Byte-identical to the C++ row fields.  The
#: ``native_rate_hz`` a row is KEYED on is the device rate the profile targets,
#: NOT proof that a frame was ever resampled down to that grid and back, which
#: is why the established level stops at the work grid.
MEASURED_ROW_EVIDENCE_LEVEL = EvidenceLevel.WORK_DECODE_VERIFIED
MEASURED_ROW_EVIDENCE_SOURCE = (
    "work_direct_998p4_modulate_loopback_demodulate_fcs"
)
#: The same statement, for a human: what the 48 rows do and do not establish.
MEASURED_ROW_EVIDENCE_STATEMENT = (
    "the 48 measured rows were produced by a full modulate -> "
    "UwbLoopbackEcho -> demodulate -> FCS-pass, byte-exact round trip at "
    "998.4 MS/s on this repository's WORK grid. That proves this repo's TX and "
    "RX agree with each other and nothing more: no DW1000 / DW3000, no SDK "
    "frame profile and no firmware hash was measured. The native_rate_hz column "
    "is the rate the row is KEYED on, not proof of a native round trip; nothing "
    "was measured for first-path / ToA accuracy; nothing was measured on "
    "hardware."
)


class StsMode(_StrEnum):
    """Phase-2 capability.  The values exist; phase 1 enables none of them."""

    OFF = "off"
    SP64 = "sp64"
    SP128 = "sp128"
    SP256 = "sp256"
    SP512 = "sp512"
    SP1024 = "sp1024"


class FcsAppender(_StrEnum):
    """WHO appends the FCS.  Exactly one layer may do it."""

    NONE = "none"      # invalid: the profile must append somewhere
    MAC = "mac"        # the MAC builds the CRC, mac_psdu_includes_fcs is true
    PHY = "phy"        # the PHY appends it, mac_psdu_includes_fcs is false


class TxPowerPolicy(_StrEnum):
    LEAVE_UNTOUCHED = "leave_untouched"
    MANUAL_GAIN_DB = "manual_gain_db"
    IQ_AMPLITUDE = "iq_amplitude"
    CALIBRATED_DBM = "calibrated_dbm"


class PulseShaping(_StrEnum):
    EXISTING_HRP = "existing_hrp"
    RECTANGULAR = "rectangular"
    ROOT_RAISED_COSINE = "root_raised_cosine"


class AgcMode(_StrEnum):
    MANUAL = "manual"
    DISABLED = "disabled"
    VENDOR_DEFAULT = "vendor_default"


class FirstPathAlgorithm(_StrEnum):
    LEADING_EDGE = "leading_edge"
    PEAK = "peak"
    ENERGY_CENTROID = "energy_centroid"
    INTERPOLATED_PEAK = "interpolated_peak"
    STRONGEST_CLUSTER = "strongest_cluster"


class CompensationFlag(_StrEnum):
    """``REQUIRED`` means a measurement without the compensation FAILS.

    It is not a lower-quality success (REQ-TIME-05).
    """

    OFF = "off"
    ON = "on"
    REQUIRED = "required"


class TimeReferenceEvent(_StrEnum):
    """The event a timed field is measured FROM (REQ-API-02).

    This is what makes "the protocol reply delay", "the post-TX RX enable
    delay" and "the timeout waiting for an expected RX frame" three
    different, non-interchangeable numbers.
    """

    POLL_TX_RMARKER = "poll_tx_rmarker"
    POLL_RX_RMARKER = "poll_rx_rmarker"
    RESPONSE_TX_RMARKER = "response_tx_rmarker"
    RESPONSE_RX_RMARKER = "response_rx_rmarker"
    FINAL_TX_RMARKER = "final_tx_rmarker"
    FINAL_RX_RMARKER = "final_rx_rmarker"
    REPORT_TX_RMARKER = "report_tx_rmarker"
    REPORT_RX_RMARKER = "report_rx_rmarker"
    FRAME_TAIL = "frame_tail"          # last transmitted / received sample
    RX_ENABLE = "rx_enable"            # the RX stream command was armed
    HOST_MONOTONIC = "host_monotonic"  # steady_clock, never the RF plane


class TimeDomain(_StrEnum):
    """Which clock the number lives in.  ``UNSPECIFIED`` is never acceptable."""

    UNSPECIFIED = "unspecified"
    DEVICE_TICKS = "device_ticks"
    MONOTONIC_HOST = "monotonic_host"


class TimeUnit(_StrEnum):
    """So ns, seconds and native ticks can never be added by accident."""

    NANOSECONDS = "ns"
    SECONDS = "s"
    NATIVE_TICKS = "native_ticks"


class CapabilityStatus(_StrEnum):
    MEASURED = "measured"            # established in this repo / frozen reqs
    MEASURED_PENDING = "measured_pending"  # the measured matrix has not landed
    BY_DEFINITION = "by_definition"  # fixed by the TWR requirement
    OUT_OF_SCOPE = "out_of_scope"    # deliberately excluded from phase 1


class ConfigReason(_StrEnum):
    """Every rejection carries exactly one of these.  Append-only (REQ-OUT-01).

    Grouped by numeric block so the JSON/CSV value of a reason is stable even
    if members are appended: 1x value, 2x cross-field, 4x capability, 6x
    calibration, 8x lifecycle, 10x JSON import.
    """

    NONE = "none"

    # --- value level ------------------------------------------------------
    NOT_FINITE = "not_finite"              # NaN / +Inf / -Inf in a float field
    OUT_OF_RANGE = "out_of_range"          # outside the documented numeric range
    INDEX_OUT_OF_RANGE = "index_out_of_range"
    NEGATIVE_VALUE = "negative_value"
    EMPTY_VALUE = "empty_value"
    OVER_CAPACITY = "over_capacity"
    ZERO_VALUE = "zero_value"

    # --- cross-field ------------------------------------------------------
    FIELD_CONFLICT = "field_conflict"
    CHANNEL_FREQUENCY_MISMATCH = "channel_frequency_mismatch"
    DUPLICATE_RESOURCE = "duplicate_resource"
    TIMING_ORDER_VIOLATION = "timing_order_violation"
    TIMING_BUDGET_INFEASIBLE = "timing_budget_infeasible"
    TIMING_PRECISION_INSUFFICIENT = "timing_precision_insufficient"
    QUANTISATION_UNSUPPORTED = "quantisation_unsupported"
    UNIT_MISMATCH = "unit_mismatch"
    FRAME_LENGTH_OVERFLOW = "frame_length_overflow"

    # --- capability -------------------------------------------------------
    UNSUPPORTED = "unsupported"            # not in the capability whitelist
    OUT_OF_SCOPE = "out_of_scope"          # real, but out of phase-1 scope
    IN_FLIGHT_NOT_SUPPORTED = "in_flight_not_supported"

    # --- calibration ------------------------------------------------------
    CALIBRATION_MISSING = "calibration_missing"
    CALIBRATION_EXPIRED = "calibration_expired"
    CALIBRATION_ALREADY_APPLIED = "calibration_already_applied"
    CALIBRATION_MISMATCH = "calibration_mismatch"

    # --- lifecycle / mutation ---------------------------------------------
    MID_EXCHANGE_MUTATION = "mid_exchange_mutation"
    EXCHANGE_ALREADY_IN_FLIGHT = "exchange_already_in_flight"

    # --- JSON import ------------------------------------------------------
    MALFORMED_JSON = "malformed_json"
    UNKNOWN_KEY = "unknown_key"
    MISSING_KEY = "missing_key"
    TYPE_MISMATCH = "type_mismatch"
    UNKNOWN_ENUM_VALUE = "unknown_enum_value"
    INTEGER_PRECISION_LOSS = "integer_precision_loss"


# ===========================================================================
# 3. Duration -- the only time type that crosses the public API
# ===========================================================================
#
# REQ-API-02: the API speaks SI seconds or this named type.  Vendor units
# (DW UUS at 1/(499.2e6*128) s, USRP device ticks, the host monotonic clock)
# are adapter-internal and never appear here.  Integer nanoseconds keep the
# JSON output free of binary floating point drift; the tick quantisation is a
# separate, explicit, CHECKED conversion.


class Duration:
    """Immutable integer-nanosecond duration with checked tick conversion."""

    __slots__ = ("_ns",)

    def __init__(self, ns: int = 0):
        if not isinstance(ns, int) or isinstance(ns, bool):
            raise TypeError("Duration takes an int64 nanosecond count")
        if ns < INT64_MIN or ns > INT64_MAX:
            raise OverflowError("Duration out of the int64 nanosecond range")
        self._ns = ns

    # -- construction ----------------------------------------------------
    @staticmethod
    def from_nanos(ns: int) -> "Duration":
        return Duration(ns)

    @staticmethod
    def from_seconds(seconds: float) -> Optional["Duration"]:
        """``None`` for NaN / +-Inf / overflow instead of a silent saturation."""
        if not _is_finite(seconds):
            return None
        ns = seconds * 1e9
        if ns > 9.2233720368547758e18 or ns < -9.2233720368547758e18:
            return None
        return Duration(int(ns))

    @staticmethod
    def from_ticks(ticks: int, tick_rate_hz: float) -> Optional["Duration"]:
        if not _is_finite(tick_rate_hz) or tick_rate_hz <= 0.0:
            return None
        return Duration.from_seconds(float(ticks) / tick_rate_hz)

    # -- accessors --------------------------------------------------------
    def nanos(self) -> int:
        return self._ns

    def seconds(self) -> float:
        return self._ns * 1e-9

    def is_zero(self) -> bool:
        return self._ns == 0

    def negative(self) -> bool:
        return self._ns < 0

    # -- device ticks -----------------------------------------------------
    def ticks_at(self, tick_rate_hz: float) -> Optional[int]:
        """Integer device ticks, half-away-from-zero, or ``None`` on failure.

        ``None`` means the tick rate is unusable or the value overflows the
        int64 tick range; a caller must treat that as a hard failure.
        """
        if not _is_finite(tick_rate_hz) or tick_rate_hz <= 0.0:
            return None
        t = self.seconds() * tick_rate_hz
        if t > 9.2233720368547758e18 or t < -9.2233720368547758e18:
            return None
        return int(t - 0.5 if t < 0.0 else t + 0.5)

    def representable_at(self, tick_rate_hz: float, max_error_ns: int) -> bool:
        """True when a round trip through ``tick_rate_hz`` costs <= the budget.

        The worst case is half a tick on each side, so the tick period must be
        within ``2 * max_error_ns``.  At 491.52 MS/s the period is 2.0345 ns,
        which is why ``max_quantisation_error_ns = 1`` is REJECTED there and
        2 (or 3) is required.
        """
        if not _is_finite(tick_rate_hz) or tick_rate_hz <= 0.0:
            return False
        period_ns = 1e9 / tick_rate_hz
        return period_ns <= float(max_error_ns) * 2.0

    def quantisation_error_ns(self, tick_rate_hz: float) -> Optional[float]:
        """Actual |value - round_trip(round_trip(value))| in ns."""
        ticks = self.ticks_at(tick_rate_hz)
        if ticks is None:
            return None
        back = Duration.from_ticks(ticks, tick_rate_hz)
        if back is None:
            return None
        return float(abs(back.nanos() - self.nanos()))

    # -- operators --------------------------------------------------------
    def __add__(self, o: "Duration") -> "Duration":
        return Duration(self._ns + o._ns)

    def __sub__(self, o: "Duration") -> "Duration":
        return Duration(self._ns - o._ns)

    def __lt__(self, o: "Duration") -> bool:
        return self._ns < o._ns

    def __le__(self, o: "Duration") -> bool:
        return self._ns <= o._ns

    def __gt__(self, o: "Duration") -> bool:
        return self._ns > o._ns

    def __ge__(self, o: "Duration") -> bool:
        return self._ns >= o._ns

    def __eq__(self, o: object) -> bool:
        return isinstance(o, Duration) and o._ns == self._ns

    def __ne__(self, o: object) -> bool:
        return not self.__eq__(o)

    def __hash__(self) -> int:
        return hash(("Duration", self._ns))

    def __repr__(self) -> str:
        return "Duration(%dns)" % self._ns

    def to_text(self) -> str:
        return duration_to_text(self)

    def __deepcopy__(self, memo: Dict[int, Any]) -> "Duration":
        """A Duration is already an immutable value, so the copy is trivial.

        Stated explicitly because :func:`copy_config` deep-copies whole
        ``TwrConfig`` trees containing dozens of these, and the generic
        ``__reduce_ex__`` path is a measurable share of the cost of taking a
        snapshot.  It is a real copy, not a shared reference: there is nothing
        mutable inside a Duration to reach through, and the new object carries
        its own ``_ns`` in its own ``__slots__`` storage.
        """
        out = Duration.__new__(Duration)
        memo[id(self)] = out
        object.__setattr__(out, "_ns", self._ns)
        return out


ZERO = Duration(0)


def quantise_duration(d: Duration, rate_hz: float) -> Tuple[Optional[int],
                                                            Optional[Duration]]:
    """Round to the nearest device tick and round-trip back.

    This is the single place a Duration becomes device ticks; adapters convert
    from here, never at the call site (REQ-API-02).  Returns
    ``(ticks, round_tripped_value)``; either may be ``None``.
    """
    ticks = d.ticks_at(rate_hz)
    if ticks is None:
        return (None, None)
    back = Duration.from_ticks(ticks, rate_hz)
    if back is None:
        return (ticks, None)
    return (ticks, back)


# ===========================================================================
# 4. Machine-readable rejections and the validation report
# ===========================================================================


@dataclass(frozen=True)
class ConfigViolation:
    """One rejection: a field path, a reason, a status and a human message."""

    field: str = ""
    reason: ConfigReason = ConfigReason.NONE
    status: ExchangeStatus = ExchangeStatus.CONFIG_REJECTED
    message: str = ""
    requirement: str = ""

    def to_text(self) -> str:
        s = self.field if self.field else "<config>"
        s += ": %s [%s]" % (self.reason, self.status)
        if self.message:
            s += " " + self.message
        if self.requirement:
            s += " (%s)" % self.requirement
        return s


def config_violation_to_string(v: ConfigViolation) -> str:
    return v.to_text()


class ValidationReport:
    """The full report.  ``ok()`` is never the only information a caller gets."""

    __slots__ = ("valid", "violations")

    def __init__(self) -> None:
        self.valid = True
        self.violations: List[ConfigViolation] = []

    def ok(self) -> bool:
        return self.valid

    def has(self, r: ConfigReason, field: Optional[str] = None) -> bool:
        """``has(reason)`` or ``has(reason, field)``."""
        if field is None:
            return any(v.reason == r for v in self.violations)
        return self.find(field, r) is not None

    def has_field(self, path: str) -> bool:
        return any(v.field == path for v in self.violations)

    def find(self, path: str, r: ConfigReason) -> Optional[ConfigViolation]:
        for v in self.violations:
            if v.field == path and v.reason == r:
                return v
        return None

    def first_for_field(self, path: str) -> Optional[ConfigViolation]:
        for v in self.violations:
            if v.field == path:
                return v
        return None

    def first_status(self) -> ExchangeStatus:
        """First non-Ok status, in report order.  Never a bare bool."""
        if not self.violations:
            return ExchangeStatus.OK
        return self.violations[0].status

    def messages(self) -> List[str]:
        return [v.to_text() for v in self.violations]

    def to_string(self) -> str:
        if not self.violations:
            return "ok"
        return "; ".join(v.to_text() for v in self.violations)

    def add(self, v: ConfigViolation) -> None:
        self.valid = False
        self.violations.append(v)

    def add_rejection(self, field: str, reason: ConfigReason,
                      status: ExchangeStatus, message: str, requirement: str) -> None:
        self.add(ConfigViolation(field=field, reason=reason, status=status,
                                 message=message, requirement=requirement))

    def __repr__(self) -> str:
        return "ValidationReport(%s)" % self.to_string()


# ===========================================================================
# 5. Timed fields (reference event + marker + unit + quantisation)
# ===========================================================================


@dataclass
class TimedField:
    """A duration that knows what it is measured FROM, in WHICH clock, at
    WHICH RF marker, and to WHAT resolution the device must express it.

    ``marker`` is REQUIRED when ``domain`` is ``DEVICE_TICKS`` and FORBIDDEN
    when it is ``MONOTONIC_HOST`` -- there is no RF marker on a host monotonic
    clock, and putting one there is an invitation to a cross-domain
    subtraction (REQ-TIME-01).
    """

    value: Duration = field(default_factory=Duration)
    domain: TimeDomain = TimeDomain.UNSPECIFIED
    reference: TimeReferenceEvent = TimeReferenceEvent.HOST_MONOTONIC
    marker: Optional[TimestampMarker] = None
    #: Device tick rate the value is quantised to, or 1e9 for integer ns.
    required_quantisation_hz: float = 0.0
    #: Largest quantisation error tolerated, in ns.  0 demands an exact
    #: nanosecond and is therefore rejected (TIMING_PRECISION_INSUFFICIENT);
    #: 1 is unreachable at 491.52 MS/s, whose tick is 2.0345 ns.
    max_quantisation_error_ns: int = 1
    #: Free text: how the number was obtained / converted (REQ-API-02).
    note: str = ""

    def to_text(self) -> str:
        return "%s@%s/%s/%s/q=%s/e=%dns" % (
            duration_to_text(self.value),
            self.domain,
            self.reference,
            self.marker if self.marker is not None else "(no marker)",
            double_to_text(self.required_quantisation_hz),
            self.max_quantisation_error_ns,
        )


def timed_field_to_text(f: TimedField) -> str:
    """The compact ``value@domain/reference/marker/q=../e=..ns`` rendering."""
    return f.to_text()


def device_field(ns: int, ref: TimeReferenceEvent, marker: TimestampMarker,
                 quantisation_hz: float = NATIVE_RATE_UC200_HZ,
                 max_quantisation_error_ns: int = 1,
                 note: str = "") -> TimedField:
    """A device-tick field with its RF marker stated (the normal case)."""
    return TimedField(value=Duration(ns), domain=TimeDomain.DEVICE_TICKS,
                      reference=ref, marker=marker,
                      required_quantisation_hz=quantisation_hz,
                      max_quantisation_error_ns=max_quantisation_error_ns,
                      note=note)


def host_field(ns: int, max_quantisation_error_ns: int = 1,
               note: str = "") -> TimedField:
    """A host-monotonic field: integer nanoseconds, NO RF marker."""
    return TimedField(value=Duration(ns), domain=TimeDomain.MONOTONIC_HOST,
                      reference=TimeReferenceEvent.HOST_MONOTONIC,
                      marker=None, required_quantisation_hz=1.0e9,
                      max_quantisation_error_ns=max_quantisation_error_ns,
                      note=note)


# ===========================================================================
# 6. Capability whitelist (DEFAULT DENY)
# ===========================================================================


@dataclass(frozen=True)
class PhyCapabilityRow:
    """One row of testdata/twr/phy_matrix_whitelist_737280000.csv.

    A row enters the accepting ``phy_matrix`` only when a full
    modulate -> loopback -> demodulate -> FCS-pass round trip was observed for
    it AND the PHR preamble-duration field actually describes the transmitted
    preamble length (REQ-PHY-01: "不能仅据 API 枚举认定兼容").

    FROZEN (M0.1, defect R1): a row IS the whitelist claim -- ``reason`` is the
    measured evidence a rejection quotes, ``verdict`` is the accept/reject
    decision, ``max_psdu_bytes`` and ``ranging`` are the conditions it admits.
    A mutable row reachable from a read-only table is therefore a hole straight
    through the freeze: ``capabilities().phy_matrix[0].verdict = "supported"``
    would have turned a measured rejection into an acceptance without touching
    a single container.  Being frozen closes that at the row itself rather than
    hoping every container around it is protected.

    To derive a different row, build a new one (``dataclasses.replace`` or a
    fresh constructor call); a whitelist row is evidence and evidence is not
    edited in place.
    """

    native_rate_hz: float = 0.0
    code_index: int = 0
    sync_repetitions: int = 0
    sfd_mode: SfdMode = SfdMode.R4Z2
    max_psdu_bytes: int = 0
    ranging: bool = True
    status: CapabilityStatus = CapabilityStatus.MEASURED_PENDING
    reason: str = ""
    sfd_symbols: int = 0
    sfd_len_ieee_802154a_standard: bool = False
    verdict: str = ""
    # M0.1 R7: what this row was ACTUALLY measured to establish.  A row may
    # be work-decode verified and nothing more -- which is exactly the M0
    # state, and the reason it must not be read as ranging capability.
    evidence_level: EvidenceLevel = EvidenceLevel.NONE
    evidence_source: str = ""
    evidence_provenance_id: str = ""
    #: the levels this row explicitly does NOT claim, so a reader can never
    #: mistake a work-decode row for a ranging or vendor claim
    evidence_not_established: str = ""

    def __deepcopy__(self, memo: Dict[int, Any]) -> "PhyCapabilityRow":
        """A value copy, cheaply.

        Every field is an immutable scalar or enum, so copying the fields IS a
        deep copy -- there is no nested container to share.  The generic
        dataclass ``deepcopy`` goes through ``__reduce_ex__`` and is the single
        largest cost in freezing a whitelist (84 rows), and a whitelist is
        copied by every ``TwrConfigSnapshot``, so this is worth stating rather
        than leaving to the default.
        """
        out = PhyCapabilityRow.__new__(PhyCapabilityRow)
        memo[id(self)] = out
        for name in self.__dataclass_fields__:
            object.__setattr__(out, name, getattr(self, name))
        return out


@dataclass
class CapabilityLookup:
    """Outcome of a whitelist lookup.

    ``allowed == False`` with ``MEASURED_PENDING`` means "not yet known", which
    is still a rejection (REQ-SCOPE-01) and never a silent fallback.
    ``allowed == False`` with ``MEASURED`` means the measurement exists and
    its verdict is "no"; ``reason`` then carries the measured reason tokens.

    M0.1 R7: the EVIDENCE behind this answer is carried as data too.  A
    refusal states what is missing and at which level it would have to be
    measured, so an operator can tell "not looked at" from "looked at and
    failed".
    """

    allowed: bool = False
    status: CapabilityStatus = CapabilityStatus.MEASURED_PENDING
    reason: str = ""
    established: EvidenceLevel = EvidenceLevel.NONE
    requested_use: EvidenceUse = EvidenceUse.WORK_DECODE
    evidence_not_established: str = ""
    provenance_id: str = ""


class ReadOnlyList(Sequence):
    """An immutable sequence: the type every capability container has.

    R1 (docs/twr/M0_复核报告.md) was not only that the config was shared: the
    PROCESS-WIDE whitelist was reachable too, so one caller could widen the
    whitelist for every later validation in the process.  A plain ``list`` gives
    no protection at all -- ``caps.channels.append(9)`` is a one-liner nobody
    thinks of as a mutation of a global.

    So the containers here are this instead: it is a :class:`Sequence` (it
    compares equal to a list, iterates, indexes and slices), it has no
    ``append`` / ``__setitem__`` / ``clear`` / ``sort`` at all, and neither the
    instance nor its backing tuple can be rebound or deleted.  Mutation is
    therefore refused at the point of the call rather than merely discouraged.

    It is not hashable, exactly like the list it replaces; nothing in this
    module needs to hash a capability container.
    """

    __slots__ = ("_items",)

    def __init__(self, items: Iterable[Any] = ()) -> None:
        object.__setattr__(self, "_items", tuple(items))

    # -- read-only by construction --------------------------------------
    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError(
            "ReadOnlyList is immutable; the capability table it belongs to is a "
            "value, not a workspace (use CapabilitiesBuilder to derive a "
            "different table)")

    def __delattr__(self, name: str) -> None:
        raise AttributeError("ReadOnlyList is immutable")

    def __repr__(self) -> str:
        return "ReadOnlyList(%r)" % (list(self._items),)

    # -- Sequence --------------------------------------------------------
    def __len__(self) -> int:
        return len(self._items)

    def __getitem__(self, index: Any) -> Any:
        return self._items[index]

    def __iter__(self) -> Any:
        return iter(self._items)

    def __reversed__(self) -> Any:
        return reversed(self._items)

    def index(self, value: Any, *args: Any) -> int:
        return self._items.index(value, *args)

    def count(self, value: Any) -> int:
        return self._items.count(value)

    def __contains__(self, value: Any) -> bool:
        return value in self._items

    def __eq__(self, other: object) -> bool:
        if isinstance(other, ReadOnlyList):
            return self._items == other._items
        if isinstance(other, (list, tuple)):
            return self._items == tuple(other)
        return NotImplemented

    def __ne__(self, other: object) -> bool:
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result

    def __hash__(self) -> int:
        raise TypeError("ReadOnlyList is unhashable, like the list it replaces")

    # -- a deep copy of a read-only container is still read-only ---------
    def __deepcopy__(self, memo: Dict[int, Any]) -> "ReadOnlyList":
        return ReadOnlyList(_copy.deepcopy(self._items, memo))


#: Every container a :class:`Capabilities` owns.  Freezing iterates exactly
#: this list, so a new list-valued field cannot be shipped unfrozen by accident
#: as long as it is declared here.
_CAPABILITY_LIST_FIELDS: Tuple[str, ...] = (
    "native_rates_hz", "code_indices", "max_psdu_bytes_seen", "prf_classes",
    "channels", "data_rates", "phy_matrix", "sync_repetitions", "sfd_modes",
    "phy_matrix_rejections", "sts_modes", "quantisation_rates_hz",
)

#: Every SCALAR field a :class:`Capabilities` owns, including the provenance
#: strings.  They are copied so that the reason a config was refused keeps
#: naming the evidence that produced it.
_CAPABILITY_SCALAR_FIELDS: Tuple[str, ...] = (
    "work_sample_rate_hz", "work_samples_per_symbol", "mean_prf_hz",
    "ranging_bit_required", "sts_supported", "schema_version",
    "profile_version", "build_id", "pending_reason", "unsupported_sts_reason",
    "phy_matrix_source",
)


class Capabilities:
    """The whitelist.  Everything not listed here is rejected.

    Two states, and the difference is load-bearing:

    * **frozen** (what :func:`capabilities`, :func:`unmeasured_capabilities`,
      :func:`default_deny_capabilities` and every ``build_*_capabilities()``
      return): every container is a :class:`ReadOnlyList` and every attribute
      is locked, so no caller can widen or narrow the table through a returned
      reference.  ``caps.channels.append(9)`` raises instead of silently
      changing every later validation in the process.
    * **mutable** (only inside :class:`CapabilitiesBuilder`): plain lists, so a
      measured row can be appended.  A builder never sees a frozen table; it
      works on :meth:`copy` of one.
    """

    # ---- MEASURED lists -------------------------------------------------
    def __init__(self) -> None:
        # Must be the FIRST assignment: __setattr__ consults it, and every
        # attribute set below runs through __setattr__.
        self._frozen = False
        self.native_rates_hz: List[float] = []
        self.code_indices: List[int] = []
        self.max_psdu_bytes_seen: List[int] = []
        self.prf_classes: List[PrfClass] = []
        self.channels: List[int] = []
        self.work_sample_rate_hz: float = 0.0
        self.work_samples_per_symbol: int = 0
        self.mean_prf_hz: float = 0.0
        self.ranging_bit_required: bool = True

        # ---- measured (accepting) lists ---------------------------------
        self.data_rates: List[DataRate] = []
        self.phy_matrix: List[PhyCapabilityRow] = []
        self.sync_repetitions: List[int] = []
        self.sfd_modes: List[SfdMode] = []

        # ---- measured (REJECTING) rows ---------------------------------
        # Carried so a rejection can quote the measurement that produced it.
        # Adding a row here can only improve a message; it can never turn a
        # rejection into an acceptance.
        self.phy_matrix_rejections: List[PhyCapabilityRow] = []

        # ---- explicitly out of scope ------------------------------------
        self.sts_supported: bool = False
        self.sts_modes: List[StsMode] = []

        # ---- quantisation ------------------------------------------------
        self.quantisation_rates_hz: List[float] = []

        # ---- provenance ---------------------------------------------------
        self.schema_version: str = SCHEMA_VERSION
        self.profile_version: str = "unfrozen"
        self.build_id: str = "m0"
        self.pending_reason: str = PENDING_MATRIX_REASON
        self.unsupported_sts_reason: str = UNSUPPORTED_STS_REASON
        self.phy_matrix_source: str = PHY_MATRIX_WHITELIST_CSV

    # ---- frozen / mutable state ----------------------------------------
    @property
    def read_only(self) -> bool:
        """True when this table refuses every mutation attempt."""
        return self._frozen

    def freeze(self) -> "Capabilities":
        """Make this table read-only IN PLACE and return it.

        Every container becomes a :class:`ReadOnlyList` and every attribute
        assignment is refused afterwards.  Idempotent.  Freezing is what
        ``freeze()`` on an already-frozen table must mean: a no-op, never a
        second wrapping.
        """
        if self._frozen:
            return self
        for name in _CAPABILITY_LIST_FIELDS:
            current = getattr(self, name)
            if isinstance(current, ReadOnlyList):
                continue
            setattr(self, name, ReadOnlyList(current))
        # Set last: every setattr above is a plain attribute store, which
        # freeze() itself forbids once the flag is on.
        self._frozen = True
        return self

    def _check_mutable(self, name: str) -> None:
        if self._frozen:
            raise AttributeError(
                "the capability table is read-only (%s): a measured whitelist "
                "is a value shared by every validation in the process, so it "
                "cannot be widened in place -- derive a different table with "
                "CapabilitiesBuilder(base)" % name)

    def __setattr__(self, name: str, value: Any) -> None:
        if getattr(self, "_frozen", False):
            self._check_mutable(name)
        object.__setattr__(self, name, value)

    def __delattr__(self, name: str) -> None:
        if getattr(self, "_frozen", False):
            self._check_mutable(name)
        object.__delattr__(self, name)

    # ---- queries --------------------------------------------------------
    def native_rate_supported(self, hz: float) -> bool:
        return any(rate_matches_strict(v, hz) for v in self.native_rates_hz)

    def code_index_supported(self, n: int) -> bool:
        return n in self.code_indices

    def prf_class_supported(self, c: PrfClass) -> bool:
        return c in self.prf_classes

    def channel_supported(self, ch: int) -> bool:
        return ch in self.channels

    def data_rate_supported(self, r: DataRate) -> bool:
        return r in self.data_rates

    def sync_repetitions_supported(self, n: int) -> bool:
        return n in self.sync_repetitions

    def sfd_mode_supported(self, m: SfdMode) -> bool:
        return m in self.sfd_modes

    def phr_rate_supported(self, r: PhrRate) -> bool:
        """True for a PHR rate this build can actually TRANSMIT.

        Deliberately NOT derived from the payload ``data_rates`` list: the PHR
        is a different modulation at a different rate, and the M0 rule that
        conflated the two accepted a PHR this modulator cannot produce.
        """
        return phr_rate_is_implemented(r)

    def preamble_length_supported(self, v: PreambleLength) -> bool:
        """True for a SYNC length this decoder / CFO path can actually run."""
        return v in ALL_PREAMBLE_LENGTHS

    def max_psdu_bytes(self) -> int:
        return max(self.max_psdu_bytes_seen) if self.max_psdu_bytes_seen else 0

    def quantisation_supported(self, hz: float) -> bool:
        return any(rate_matches_strict(v, hz) for v in self.quantisation_rates_hz)

    def copy(self) -> "Capabilities":
        """A MUTABLE deep copy, so a builder can never mutate the shipped one.

        The copy is deliberately NOT frozen: a :class:`CapabilitiesBuilder` has
        to append to it.  Handing that copy to a caller is safe precisely
        because it is private to the builder -- the shipped table is never
        reachable from it.
        """
        return _copy.deepcopy(self).thaw()

    def thaw(self) -> "Capabilities":
        """Undo :meth:`freeze` IN PLACE and return the table.

        Every container becomes a plain ``list`` again and attribute assignment
        works.  Only a builder calls this, on a table it just copied.  Calling
        it on a shipped table would be an API bug, not a feature: there is
        exactly one shipped table per process and it must stay read-only.
        """
        if not self._frozen:
            return self
        # The flag comes off first: every setattr below is otherwise refused.
        object.__setattr__(self, "_frozen", False)
        for name in _CAPABILITY_LIST_FIELDS:
            current = getattr(self, name)
            if isinstance(current, ReadOnlyList):
                setattr(self, name, list(current))
        return self

    def frozen_copy(self) -> "Capabilities":
        """An independent, read-only copy of this table.

        Used where a caller must hold a table that cannot move underneath it
        even if the table it was derived from is later thawed by its owner.
        """
        return _copy.deepcopy(self).freeze()

    # ---- the joint lookup -----------------------------------------------
    def lookup_phy(self, native_rate_hz: float, code_index: int,
                   sync_repetitions: int, sfd_mode: SfdMode, psdu_bytes: int,
                   ranging: bool) -> CapabilityLookup:
        """The full (rate, code, SYNC, SFD, PSDU, ranging) joint lookup.

        Every axis is cross-validated together: "code supported" alone never
        implies "this preamble length transmits and decodes both ways"
        (REQ-BASE-02).
        """
        out = CapabilityLookup()
        if self.phy_matrix:
            for row in self.phy_matrix:
                if not rate_matches_strict(row.native_rate_hz, native_rate_hz):
                    continue
                if row.code_index != code_index:
                    continue
                if row.sync_repetitions != sync_repetitions:
                    continue
                if row.sfd_mode != sfd_mode:
                    continue
                if row.ranging != ranging:
                    continue
                if psdu_bytes > row.max_psdu_bytes:
                    continue
                out.allowed = True
                out.status = row.status
                out.reason = row.reason
                # M0.1 R7: the evidence travels with the answer, so a caller
                # can never read "decode round-trips" as "produces a range".
                out.established = row.evidence_level
                out.evidence_not_established = row.evidence_not_established
                out.provenance_id = row.evidence_provenance_id
                return out

        out.allowed = False
        # A row the measurement rejected: quote its reason.
        for row in self.phy_matrix_rejections:
            if not rate_matches_strict(row.native_rate_hz, native_rate_hz):
                continue
            if row.code_index != code_index:
                continue
            if row.sync_repetitions != sync_repetitions:
                continue
            if row.sfd_mode != sfd_mode:
                continue
            if row.ranging != ranging:
                continue
            if psdu_bytes > row.max_psdu_bytes:
                continue
            out.status = row.status
            out.reason = "measured_reject: " + (row.reason or "no reason recorded")
            out.reason += " (%s)" % self.phy_matrix_source
            out.established = row.evidence_level
            out.evidence_not_established = row.evidence_not_established
            out.provenance_id = row.evidence_provenance_id
            return out

        # No row at all for this (rate, code, length, sfd): still a rejection.
        derived = sync_repetition_reasons(sync_repetitions)
        if derived:
            out.status = CapabilityStatus.MEASURED
            out.reason = "measured_reject: " + "; ".join(derived)
            out.reason += " (%s)" % self.phy_matrix_source
            out.evidence_not_established = not_established_list(EvidenceLevel.NONE)
            return out

        out.status = CapabilityStatus.MEASURED_PENDING
        out.reason = (
            "no measured row for native_rate=%s code=%d sync=%d sfd=%s psdu<=%d; "
            "see %s%s" % (
                double_to_text(native_rate_hz), code_index, sync_repetitions,
                sfd_mode, psdu_bytes, self.phy_matrix_source,
                ("; " + self.pending_reason) if self.pending_reason else "",
            )
        )
        out.evidence_not_established = not_established_list(EvidenceLevel.NONE)
        return out

    def to_string(self) -> str:
        s = []
        s.append("schema=%s profile=%s build=%s\n" % (
            self.schema_version, self.profile_version, self.build_id))
        s.append("native_rates_hz=" + ",".join(double_to_text(v)
                                               for v in self.native_rates_hz) + "\n")
        s.append("code_indices=" + ",".join(int_to_text(v)
                                            for v in self.code_indices) + "\n")
        s.append("channels=" + ",".join(int_to_text(v) for v in self.channels) + "\n")
        s.append("prf_classes=" + ",".join(str(v) for v in self.prf_classes) + "\n")
        s.append("work_sample_rate_hz=%s work_samples_per_symbol=%d "
                 "mean_prf_hz=%s\n" % (
                     double_to_text(self.work_sample_rate_hz),
                     self.work_samples_per_symbol,
                     double_to_text(self.mean_prf_hz)))
        s.append("max_psdu_bytes=%d\n" % self.max_psdu_bytes())
        s.append("data_rates=" + ",".join(str(v) for v in self.data_rates) + "\n")
        s.append("sync_repetitions=" + ",".join(int_to_text(v)
                                                for v in self.sync_repetitions) + "\n")
        s.append("sfd_modes=" + ",".join(str(v) for v in self.sfd_modes) + "\n")
        s.append("ranging_bit_required=%s\n" % bool_to_text(self.ranging_bit_required))
        s.append("sts_supported=%s%s\n" % (
            bool_to_text(self.sts_supported),
            "" if self.sts_supported else " (%s)" % self.unsupported_sts_reason))
        s.append("quantisation_rates_hz=" + ",".join(
            double_to_text(v) for v in self.quantisation_rates_hz) + "\n")
        s.append("phy_matrix_rows=%d from %s" % (len(self.phy_matrix),
                                                 self.phy_matrix_source))
        s.append("phy_matrix_rejection_rows=%d" % len(self.phy_matrix_rejections))
        if not self.phy_matrix:
            s.append(" [MEASURED-PENDING] " + self.pending_reason)
        return "".join(s) + "\n"


class CapabilitiesBuilder:
    """Installs measured rows without touching a shipped default.

    Nothing here relaxes a default: adding a row only ever turns a rejection
    into an acceptance.
    """

    def __init__(self, base: Capabilities):
        # copy() thaws, so the builder has a MUTABLE workspace even when the
        # base is the frozen process-wide table.  This is the ONLY supported way
        # to change what the whitelist allows.
        self._caps = base.copy()

    def add_native_rate(self, hz: float) -> "CapabilitiesBuilder":
        self._caps.native_rates_hz.append(hz)
        return self

    def add_code_index(self, n: int) -> "CapabilitiesBuilder":
        self._caps.code_indices.append(n)
        return self

    def add_channel(self, ch: int) -> "CapabilitiesBuilder":
        self._caps.channels.append(ch)
        return self

    def add_prf_class(self, c: PrfClass) -> "CapabilitiesBuilder":
        self._caps.prf_classes.append(c)
        return self

    def add_data_rate(self, r: DataRate) -> "CapabilitiesBuilder":
        self._caps.data_rates.append(r)
        return self

    def add_sync_repetitions(self, n: int) -> "CapabilitiesBuilder":
        self._caps.sync_repetitions.append(n)
        return self

    def add_sfd_mode(self, m: SfdMode) -> "CapabilitiesBuilder":
        self._caps.sfd_modes.append(m)
        return self

    def add_max_psdu_bytes(self, n: int) -> "CapabilitiesBuilder":
        self._caps.max_psdu_bytes_seen.append(n)
        return self

    def add_quantisation_rate(self, hz: float) -> "CapabilitiesBuilder":
        self._caps.quantisation_rates_hz.append(hz)
        return self

    def add_phy_row(self, row: PhyCapabilityRow) -> "CapabilitiesBuilder":
        self._caps.phy_matrix.append(row)
        return self

    def add_phy_rejection_row(self, row: PhyCapabilityRow) -> "CapabilitiesBuilder":
        self._caps.phy_matrix_rejections.append(row)
        return self

    def set_profile_version(self, v: str) -> "CapabilitiesBuilder":
        self._caps.profile_version = v
        return self

    def set_build_id(self, v: str) -> "CapabilitiesBuilder":
        self._caps.build_id = v
        return self

    def build(self) -> Capabilities:
        """The derived table, FROZEN.

        Frozen because it is a decision, not a workspace: handing a mutable
        table to a caller would let them widen it after the fact and invalidate
        every validation that already used it.  Build another one to change it.
        """
        return self._caps.freeze()


# --- measured reasons for a preamble length -------------------------------


def phr_advertised_sync_symbols(idx: int) -> int:
    """PHR preamble-duration indices are 0=16, 1=64, 2=1024, 3=4096
    (802.15.4a-2007 Table 68a).  Returns the repetition count the PHR
    actually advertises, or 0 when the field cannot express the length.
    """
    return {0: 16, 1: 64, 2: 1024, 3: 4096}.get(idx, 0)


def phr_preamble_duration_index(sync_repetitions: int) -> int:
    """Mirrors mod::encode_phr19 (uwb_hrp_mod_core.h).

    Non-native SYNC lengths still advertise the nearest native duration
    (16 / 64 / 1024) exactly like MATLAB BPRF, so the PHR does NOT
    self-describe a length such as 128, 256, 512 or 2048.
    """
    if sync_repetitions >= 1024:
        return 2
    if sync_repetitions <= 16:
        return 0
    return 1


def sync_repetition_reasons(n: int) -> List[str]:
    """Every INDEPENDENT reason a preamble length must be refused for.

    Returned as a list so the validator can report all of them at once instead
    of stopping at the first.

    This is the DERIVED, explanatory view.  The MEASURED verdict strings live
    in ``Capabilities.phy_matrix_rejections`` and are byte-equal to the
    whitelist CSV cells; where the two overlap (the PHR and CFO-fit tokens) the
    text is identical, and where they differ the CSV's short stage-failure
    tokens are deliberately not repeated here -- this function instead carries
    the long explanatory forms the full matrix records.

    An empty list means "no structural objection".  For 16 and 64 that is the
    final answer: both are measured and admitted.  For every other API length it
    means the whitelist -- not this function -- is what refuses it.
    """
    reasons: List[str] = []

    # (0) A preamble this short does not survive the measured chain at all:
    #     too few measured peaks for the CFO stage, or fewer repetitions than
    #     the CIR estimator skips.  (Measured: 1/2 -> CfoFailed with 1/2 peaks,
    #     4/8 -> CirFailed with 4/8 peaks.)
    if 0 < n < CFO_MIN_MEASURED_PEAKS:
        reasons.append(CFO_MIN_PEAKS_REASON_TEMPLATE.format(
            min=CFO_MIN_MEASURED_PEAKS, n=n))
    elif 0 < n < CIR_SKIP_INITIAL_REPETITIONS:
        reasons.append(CIR_SKIP_REASON_TEMPLATE.format(
            skip=CIR_SKIP_INITIAL_REPETITIONS, n=n))

    # (1) The PHR preamble-duration field cannot describe this length.  This is
    #     the ONLY measured objection against 32: it decodes with a byte-exact
    #     FCS and tracks carrier offset exactly (32 measured peaks, zero
    #     synthesised), but encode_phr19 advertises index 1 = 64.
    idx = phr_preamble_duration_index(n)
    advertised = phr_advertised_sync_symbols(idx)
    if advertised != n:
        reasons.append(PHR_DURATION_REASON_TEMPLATE.format(
            idx=idx, advertised=advertised, requested=n))

    # (2) The CFO least-squares fit.  The demodulator re-measures only the last
    #     max(cfo_min_fit_repetitions=32, 40) = 40 repetitions and SYNTHESISES
    #     the earlier peak coordinates from the SFD-anchored grid.  Synthesised
    #     peaks carry a zero matched-filter value, so they must be excluded.
    #     The profile's cfo_skip_initial_repetitions is hardcoded to 24, which
    #     coincides with 64 - 40 ONLY at 64 SYNC, so nothing leaks there.  Above
    #     64 the skip no longer covers every zero-phase point and they leak into
    #     the fit: measured tail_first = 88 at 128 (20 kHz came back as 3791 Hz,
    #     error -16209 Hz) and 984 at 1024 (error -19901 Hz at 20 kHz).
    #     Note 1024 gets NO PHR reason: index 2 advertises 1024 exactly, so the
    #     PHR does self-describe it and the fit bias is the only objection.
    if n > 64:
        tail_first = n - CFO_TAIL_REPETITIONS
        reasons.append(CFO_FIT_REASON_TEMPLATE.format(
            tail_first=tail_first, skip=CFO_SKIP_INITIAL_REPETITIONS))

    # (3) DERIVED, not measured: 32/128/256/512/2048 are custom profiles built
    #     by cropping/tiling a pulse-shaped SYNC field and are not standard
    #     802.15.4a BPRF preamble durations.  The whitelist already rejects each
    #     of them for a concrete measured reason; this token records the
    #     additional structural objection alongside it.
    if n not in IEEE_802154A_PREAMBLE_SYMBOLS:
        reasons.append(ILLEGAL_LENGTH_REASON_TEMPLATE.format(n=n))

    # (4) Not even in the modulator's API list, so the matrix QA refused to
    #     modulate it in the first place.
    if not sync_reps_supported(n):
        reasons.append(API_LIST_REASON)

    return reasons


def sync_reps_needing_toa_validation(n: int) -> bool:
    """True when a length decodes but its first-path / ToA accuracy is unmeasured.

    Mirrors C++ ``twr::twr_sync_reps_needs_toa_validation``.  Being in the
    capability whitelist and being fit as a RANGING profile are two different
    questions: ranging accuracy is a function of first-path ToA, so a length can
    round-trip with a byte-exact FCS and still be unfit to produce a range until
    its first-path accuracy has been measured.  Callers building a ranging
    profile must consult this and refuse such a length rather than reporting a
    range from it.
    """
    return n in SYNC_REPS_NEEDING_TOA_VALIDATION


# --- the shipped whitelists -----------------------------------------------

#: SFD modes the measured matrix accepted (all six, at 64 SYNC).
MEASURED_SFD_MODES: Tuple[SfdMode, ...] = (
    SfdMode.R4Z1, SfdMode.R4Z2, SfdMode.R4Z3, SfdMode.R4Z4,
    SfdMode.DWT8, SfdMode.IEEE8,
)

#: The preamble lengths the measured matrix accepted.  BOTH 16 and 64 round
#: trip; 16 additionally carries the first-path / ToA caveat, which is a
#: question about RANGING FITNESS, not about whether the length is supported.
MEASURED_SYNC_REPETITIONS: Tuple[int, ...] = (16, 64)

#: The only data rate the modulator can actually produce (index 2 = 6.81 Mb/s).
MEASURED_DATA_RATES: Tuple[DataRate, ...] = (DataRate.R6P8M,)

#: The only preamble codes the API accepts, and all four are in the matrix.
MEASURED_CODE_INDICES: Tuple[int, ...] = (9, 10, 11, 12)

#: The only native rate with measured PHY rows.  491.52 MS/s is a *supported*
#: native rate (it mirrors is_allowed_uhd_native_rate) but has no measured PHY
#: row, so the joint lookup refuses the combination and says so instead of
#: quietly re-running the other rate.
MEASURED_PHY_NATIVE_RATE_HZ: float = NATIVE_RATE_UC200_HZ

#: Byte-identical to C++ uwb_twr_config.h::build_default_capabilities().
MEASURED_PROFILE_VERSION = "m0-ch5-64sync-4z2"


def _measured_row_reason(sync: int) -> str:
    """The C++ capability-row reason for a measured length."""
    return (MEASURED_ROW_REASON_TOA_UNVERIFIED
            if sync_reps_needing_toa_validation(sync) else MEASURED_ROW_REASON)


def phr_self_description_reason(n: int) -> str:
    """The measured PHR self-description token for a preamble length.

    Empty string when the length DOES self-describe (16, 64, 1024, 4096).
    """
    idx = phr_preamble_duration_index(n)
    advertised = phr_advertised_sync_symbols(idx)
    if advertised == n:
        return ""
    return PHR_DURATION_REASON_TEMPLATE.format(
        idx=idx, advertised=advertised, requested=n)


def cfo_fit_leak_reason(n: int) -> str:
    """The measured carrier-offset-fit leakage token for a preamble length.

    Empty string for ``n <= 64``: at 64 SYNC the synthesised tail is exactly
    ``cfo_skip_initial_repetitions`` long, and at 16 SYNC the measured tail
    covers the whole preamble, so nothing leaks at either length.
    """
    if n <= 64:
        return ""
    return CFO_FIT_REASON_TEMPLATE.format(
        tail_first=n - CFO_TAIL_REPETITIONS, skip=CFO_SKIP_INITIAL_REPETITIONS)


def _build_unmeasured_table() -> Capabilities:
    """The MUTABLE pre-matrix table, before :meth:`Capabilities.freeze`.

    Private on purpose: it is the base :func:`build_measured_capabilities`
    extends, and a mutable base cannot be a frozen one.  Both public builders
    call it and freeze their result.
    """
    c = Capabilities()
    c.schema_version = SCHEMA_VERSION
    c.profile_version = "unfrozen-m0"
    c.build_id = "m0"

    # ---- MEASURED (existing code) ----
    c.native_rates_hz = [NATIVE_RATE_UC200_HZ, NATIVE_RATE_CG400_HZ]
    c.code_indices = list(range(CODE_INDEX_MIN, CODE_INDEX_MAX + 1))
    c.max_psdu_bytes_seen = [MAX_PSDU_BYTES]
    c.work_sample_rate_hz = WORK_SAMPLE_RATE_HZ
    c.work_samples_per_symbol = WORK_SAMPLES_PER_SYMBOL
    c.mean_prf_hz = EXISTING_MEAN_PRF_HZ
    c.quantisation_rates_hz = [1.0e9, WORK_SAMPLE_RATE_HZ,
                               NATIVE_RATE_UC200_HZ, NATIVE_RATE_CG400_HZ]

    # ---- MEASURED-PENDING: empty, so everything PHY is refused ----
    c.data_rates = []
    c.sync_repetitions = []
    c.sfd_modes = []
    c.phy_matrix = []
    c.phy_matrix_rejections = []

    # ---- from the frozen TWR requirements ----
    # REQ-PHY-01 / REQ-PHY-03: channel 5 at 6489.6 MHz with the 64 MHz PRF
    # class is the common phase-1 baseline.  Other channels are per-channel
    # acceptance items and are NOT claimed.
    c.channels = [5]
    c.prf_classes = [PrfClass.BPRF64]
    c.ranging_bit_required = True

    # ---- out of scope for phase 1 ----
    c.sts_supported = False
    c.sts_modes = []
    return c


def build_unmeasured_capabilities() -> Capabilities:
    """The PRE-matrix default-deny state: every measured list EMPTY.

    This is the state uwb_twr_config.h had before the PHY matrix landed, and it
    is retained deliberately so the default-deny MECHANISM stays testable: the
    same config must be refused against these empty lists and accepted against
    the measured whitelist, and nothing in between.

    It is NOT the current C++ default any more; ``build_default_capabilities()``
    is.  Use it only to exercise the refusal path.

    The table is FROZEN before it is returned, so a caller cannot turn the
    default-deny state into something else by accident; derive a different
    table with ``CapabilitiesBuilder``.
    """
    return _build_unmeasured_table().freeze()


def build_measured_capabilities() -> Capabilities:
    """The whitelist the measured QA produced -- the C++ shipped default.

    Mirrors ``testdata/twr/phy_matrix_whitelist_737280000.csv`` (84 rows) and
    adds no combination the CSV lacks.

      * 48 ``supported`` rows: native 737.28 MS/s x {16, 64} SYNC x codes
        {9,10,11,12} x SFD {4z1,4z2,4z3,4z4,decawave,ieee}, ranging = true,
        max_psdu_bytes = 127.  Row order matches the C++ loop
        (sync, then code, then sfd).  The 16 SYNC rows carry the extra
        first-path / ToA caveat token, exactly as C++ does.
      * 36 ``unsupported`` rows kept as REJECTIONS so a refusal can quote the
        measured cell verbatim instead of a generic message.

    Measured evidence behind the admitted rows (from
    ``testdata/twr/phy_matrix_737280000.csv``, 318 rows):

      * 16 SYNC -- 98/98 PASS, ``bytes_exact`` and ``fcs_pass`` on every row,
        PSDU 16/26/31/127 B, ``ranging`` 0 and 1.  ``cfo_zero_peak_corr == 0``
        on EVERY row: the measured tail covers the whole 16-repetition
        preamble, so the hardcoded ``cfo_skip_initial_repetitions = 24``
        degenerates to an effective 0 and no synthesised zero-phase point can
        enter the fit.  Worst |CFO error| 0.007 Hz (at 100 kHz injected).
        SELF-DESCRIBING: ``phr_preamble_idx == 0`` on every row, i.e.
        ``encode_phr19`` advertises 16, which is a legal 802.15.4a BPRF
        duration.  CAVEAT: first-path / ToA accuracy is unverified over the
        shorter 16.3 us integration window (64 SYNC has 65.1 us) -- see
        ``sync_reps_needing_toa_validation``.
      * 64 SYNC -- 98 PASS, ``cfo_zero_peak_corr == 24`` at 64 SYNC, which is
        exactly ``64 - 40 == cfo_skip_initial_repetitions``, so nothing leaks.
        Worst |CFO error| 0.008 Hz (at 100 kHz injected).
      * Both lengths pass the integer-delay, fractional-delay, 2-tap and 3-tap
        multipath, delayed-strong-second and AWGN(sigma=0.05) sweeps the full
        matrix records.  No AWGN noise CEILING is asserted here: the shipped
        CSVs record one sigma point per length, not a sweep, so a ceiling would
        be a claim this file cannot support.

    Rejected lengths and their MEASURED cause (the whitelist CSV cell):

      * 1 / 2   -> ``stage2_cfo_failed``       (fewer than 4 measured peaks)
      * 4 / 8   -> ``stage4_cir_failed``       (cir skip 10 > available)
      * 32      -> PHR advertises 64 (index 1).  It decodes byte-exactly and is
        CFO-exact, so this is its ONLY objection.
      * 100     -> outside the modulator API list (not a power of two)
      * 128     -> PHR advertises 64 AND CFO-fit leakage (tail_first 88 > 24)
      * 256/512 -> PHR advertises 64 AND CFO-fit leakage
      * 1024    -> CFO-fit leakage ONLY (tail_first 984).  PHR-legal: index 2
        advertises 1024 exactly, so it must NOT be given a PHR reason.
      * 2048    -> PHR advertises 1024 AND CFO-fit leakage

    The CSV carries no PSDU column, so each row's ``max_psdu_bytes`` is the
    frozen capability bound ``kMaxPsduBytes = 127``; the 127-byte PHR/FCS
    golden is part of the same measured matrix (``phr_psdu_length == 127``).

    The table is FROZEN before it is returned (R1: a caller must not be able to
    widen the process-wide whitelist through a returned reference).
    """
    c = _build_unmeasured_table()
    c.profile_version = MEASURED_PROFILE_VERSION
    c.phy_matrix_source = PHY_MATRIX_WHITELIST_CSV

    c.data_rates = list(MEASURED_DATA_RATES)
    c.sync_repetitions = list(MEASURED_SYNC_REPETITIONS)
    c.sfd_modes = list(MEASURED_SFD_MODES)

    # ---- the 48 measured-supported rows, in the C++ loop order -------------
    #
    # M0.1 R7: every one of these 48 rows was measured ONLY on the 998.4 MS/s
    # work grid (a full modulate -> loopback -> demodulate -> byte-exact-FCS
    # round trip).  The established level is therefore exactly
    # WORK_DECODE_VERIFIED and nothing above it, and the absent levels are
    # recorded AS DATA so nothing downstream can read a decode-verified row as
    # ranging, hardware or vendor capability.
    for sync in MEASURED_SYNC_REPETITIONS:
        for code in MEASURED_CODE_INDICES:
            for sfd in MEASURED_SFD_MODES:
                c.phy_matrix.append(PhyCapabilityRow(
                    native_rate_hz=MEASURED_PHY_NATIVE_RATE_HZ,
                    code_index=code,
                    sync_repetitions=sync,
                    sfd_mode=sfd,
                    max_psdu_bytes=MAX_PSDU_BYTES,
                    ranging=True,
                    status=CapabilityStatus.MEASURED,
                    reason=_measured_row_reason(sync),
                    sfd_symbols=sfd_mode_symbols(sfd),
                    sfd_len_ieee_802154a_standard=sfd in (
                        SfdMode.R4Z2, SfdMode.R4Z3, SfdMode.DWT8,
                        SfdMode.IEEE8),
                    verdict="supported",
                    evidence_level=MEASURED_ROW_EVIDENCE_LEVEL,
                    evidence_source=MEASURED_ROW_EVIDENCE_SOURCE,
                    evidence_not_established=not_established_list(
                        MEASURED_ROW_EVIDENCE_LEVEL),
                ))

    # ---- the measured REJECTIONS, cell strings verbatim --------------------
    def reject(code: int, sync: int, sfd: SfdMode, reason: str) -> None:
        c.phy_matrix_rejections.append(PhyCapabilityRow(
            native_rate_hz=MEASURED_PHY_NATIVE_RATE_HZ,
            code_index=code,
            sync_repetitions=sync,
            sfd_mode=sfd,
            max_psdu_bytes=MAX_PSDU_BYTES,
            ranging=True,
            status=CapabilityStatus.MEASURED,
            reason=reason,
            sfd_symbols=sfd_mode_symbols(sfd),
            verdict="unsupported",
            # A rejection row was also produced on the work grid: it establishes
            # the same level, and the fact that it was NOT established is why
            # the row refuses.  Every level above work-decode is absent.
            evidence_level=MEASURED_ROW_EVIDENCE_LEVEL,
            evidence_source=MEASURED_ROW_EVIDENCE_SOURCE,
            evidence_not_established=not_established_list(
                MEASURED_ROW_EVIDENCE_LEVEL),
        ))

    # 1 / 2 -- the CFO stage needs at least 4 measured peaks.
    for n in (1, 2):
        reject(9, n, SfdMode.R4Z2, STAGE_CFO_FAILED_REASON)
    # 4 / 8 -- the CIR estimator skips 10 repetitions, more than are available.
    for n in (4, 8):
        reject(9, n, SfdMode.R4Z2, STAGE_CIR_FAILED_REASON)
    # 32 -- decodes byte-exactly and is CFO-exact, but does not self-describe.
    reject(9, 32, SfdMode.R4Z2, phr_self_description_reason(32))
    # 100 -- outside the modulator API list, refused before modulating.
    reject(9, 100, SfdMode.R4Z2, API_LIST_REASON)
    # 128 -- swept across every code x SFD the matrix covered.
    for code in MEASURED_CODE_INDICES:
        for sfd in MEASURED_SFD_MODES:
            reject(code, 128, sfd, "%s | %s" % (phr_self_description_reason(128),
                                               cfo_fit_leak_reason(128)))
    # 256 / 512 -- swept once each, at code 9 / 4z2.
    for n in (256, 512):
        reject(9, n, SfdMode.R4Z2, "%s | %s" % (phr_self_description_reason(n),
                                              cfo_fit_leak_reason(n)))
    # 1024 -- PHR-legal (index 2 advertises 1024) but the CFO fit is biased.
    reject(9, 1024, SfdMode.R4Z2, cfo_fit_leak_reason(1024))
    # 2048 -- advertises 1024 and the CFO fit is biased.
    reject(9, 2048, SfdMode.R4Z2,
           "%s | %s" % (phr_self_description_reason(2048), cfo_fit_leak_reason(2048)))
    # A code outside the API range; the code check fires first, but the row is
    # carried so a lookup for it quotes the measurement rather than nothing.
    reject(CODE_INDEX_MAX + 1, 64, SfdMode.R4Z2, CODE_INDEX_REASON)
    # The `bogus_sfd` row cannot be carried at all: it is not an SfdMode value.

    c.pending_reason = PENDING_MATRIX_REASON
    return c.freeze()


def build_default_capabilities() -> Capabilities:
    """The C++ ``build_default_capabilities()``: the MEASURED whitelist.

    uwb_twr_config.h stopped shipping empty PHY lists once the matrix landed,
    so parity means the default is the measured one.  See
    ``build_unmeasured_capabilities()`` for the pre-matrix state.
    """
    return build_measured_capabilities()


_MEASURED_CAPS: Optional[Capabilities] = None
_UNMEASURED_CAPS: Optional[Capabilities] = None


def capabilities() -> Capabilities:
    """The process-wide measured whitelist (built once, never mutated).

    THE SAME FROZEN INSTANCE every call, on purpose: one table per process, so
    two endpoints validating concurrently cannot disagree about what the
    hardware supports.  It is safe to share precisely because it is read-only
    (R1): every container is a :class:`ReadOnlyList` and every attribute is
    locked, so ``capabilities().channels.append(9)`` raises instead of silently
    widening the whitelist for every later validation.

    To evaluate a config against a DIFFERENT table, derive one with
    ``CapabilitiesBuilder(capabilities())`` and pass it to ``validate()`` /
    ``effective_config()``.  That is a value of your own, not the shared one.
    """
    global _MEASURED_CAPS
    if _MEASURED_CAPS is None:
        _MEASURED_CAPS = build_measured_capabilities()
    return _MEASURED_CAPS


def unmeasured_capabilities() -> Capabilities:
    """The pre-matrix default-deny whitelist, for the refusal-path QA only.

    One frozen process-wide instance, exactly like :func:`capabilities`.
    """
    global _UNMEASURED_CAPS
    if _UNMEASURED_CAPS is None:
        _UNMEASURED_CAPS = build_unmeasured_capabilities()
    return _UNMEASURED_CAPS


def default_deny_capabilities() -> Capabilities:
    """Backwards-compatible alias of :func:`unmeasured_capabilities`.

    The alias deliberately returns the SAME frozen instance rather than a
    second copy: the pre-matrix table exists to exercise the refusal path, and
    two references to one table cannot drift apart.
    """
    return unmeasured_capabilities()




# --- standard tables -------------------------------------------------------

#: IEEE 802.15.4a channel plan (normative centre frequencies).  A STANDARD
#: table, not a capability claim: the whitelist decides which of these this
#: build may actually use.
UWB_CHANNEL_CENTER_FREQUENCY_HZ: Tuple[float, ...] = (
    4835.2e6, 4953.6e6, 5066.4e6, 5179.2e6, 5292.0e6, 6489.6e6, 6614.4e6,
    6739.2e6, 6863.9e6, 6988.8e6, 7112.6e6, 7236.4e6, 7360.1e6, 7483.8e6,
    7606.6e6, 7729.4e6, 7852.2e6,
)


def uwb_channel_center_frequency_hz(channel: int) -> float:
    """Normative 802.15.4a channel-plan centre frequency, 0.0 if out of range."""
    if channel < 0 or channel > 16:
        return 0.0
    return UWB_CHANNEL_CENTER_FREQUENCY_HZ[channel]


# ===========================================================================
# 7. The typed configuration
# ===========================================================================


# ---------------------------------------------------------------------------
# THE FRAME PROFILE -- an enumeration, never a free-form name (M0.1, R3)
# ---------------------------------------------------------------------------
# Mirrors ``enum class FrameProfileId`` in uwb_twr_frame.h.  Phase 1 implements
# exactly one.  Selecting it by enum is what makes "an arbitrary non-empty
# profile string is an executable geometry" UNREPRESENTABLE rather than merely
# discouraged.
#
# NOT the same concept as the PHY capability profile string
# ("m0-ch5-64sync-4z2" in ``capabilities()``), which names channel / SYNC
# length / SFD, not the MAC PSDU layout.  The two must not be conflated or
# derived from one another.
class FrameProfileId(_StrEnum):
    #: frame v1: 14 B header (version, function code, 16-bit session/seq/PAN/
    #: src/dst/flags), 40-bit little-endian timestamps, standard HRP PHR, FCS
    #: appended by the modulation layer.  On air: Poll 16 / Response 26 /
    #: Final 31 B.
    TWR_V1 = "frame_v1"


#: The complete set, so a validator can enumerate what it may offer.
ALL_FRAME_PROFILE_IDS: Tuple[FrameProfileId, ...] = (FrameProfileId.TWR_V1,)


def frame_profile_id_is_supported(v: FrameProfileId) -> bool:
    """True only for an implemented layout.

    A value cast in from an int, or a future enumerator added without
    handling, is refused -- never defaulted to frame v1.
    """
    return v is FrameProfileId.TWR_V1


# ---------------------------------------------------------------------------
# The claim
# ---------------------------------------------------------------------------
class GeometryField(_StrEnum):
    """The five comparable geometry fields, in a fixed report order."""

    MAC_HEADER_BYTES = "mac_header_bytes"
    TIMESTAMP_BYTES = "timestamp_bytes"
    MAC_FOOTER_BYTES = "mac_footer_bytes"
    MAC_FCS_BYTES = "mac_fcs_bytes"
    PHR_BYTES = "phr_bytes"


#: The report order both languages use, so a rejection lists the fields the
#: same way round.
GEOMETRY_FIELD_ORDER: Tuple[GeometryField, ...] = (
    GeometryField.MAC_HEADER_BYTES,
    GeometryField.TIMESTAMP_BYTES,
    GeometryField.MAC_FOOTER_BYTES,
    GeometryField.MAC_FCS_BYTES,
    GeometryField.PHR_BYTES,
)


class FcsOwner(_StrEnum):
    """Who appends the FCS.  Exactly one layer may (REQ-API-01 帧格式).

    Distinct from :class:`FcsAppender`, which is the configuration layer's own
    enumeration; a configuration maps onto this one.
    """

    PHY_LAYER = "phy"
    MAC_LAYER = "mac"


@dataclass(frozen=True)
class FrameGeometryClaim:
    """What a caller BELIEVES the frame layout to be.

    Same field names as :class:`FrameGeometry` on purpose: the config states
    its claim in that type and the frame layer checks it.  A DISTINCT type from
    the authority below is also deliberate -- the two can never be swapped for
    one another by accident.
    """

    mac_header_bytes: int = 0
    timestamp_bytes: int = 0
    mac_footer_bytes: int = 0
    mac_fcs_bytes: int = 0
    phr_bytes: int = 0

    def any_set(self) -> bool:
        return (self.mac_header_bytes != 0 or self.timestamp_bytes != 0 or
                self.mac_footer_bytes != 0 or self.mac_fcs_bytes != 0 or
                self.phr_bytes != 0)

    def field(self, f: GeometryField) -> int:
        return getattr(self, str(f))

    def items(self) -> List[Tuple[GeometryField, int]]:
        return [(f, self.field(f)) for f in GEOMETRY_FIELD_ORDER]


@dataclass(frozen=True)
class FrameProfileGeometry:
    """THE AUTHORITY: the layout the frame codec builds.

    Every number comes from the module's frozen codec constants; there is no
    second ``header + n * timestamp`` sum anywhere in this file.  ``frame_v1``
    is the only member, so the accessors read their value rather than being
    constants of their own -- a future profile with a footer has ONE place to
    say so, instead of each caller inventing a third geometry.
    """

    id: FrameProfileId = FrameProfileId.TWR_V1
    fcs_owner: FcsOwner = FcsOwner.PHY_LAYER

    def mac_header_bytes(self) -> int:
        return FRAME_HEADER_BYTES

    def timestamp_bytes(self) -> int:
        return FRAME_TIMESTAMP_BYTES

    def mac_footer_bytes(self) -> int:
        return 0

    def mac_fcs_bytes(self) -> int:
        return (FRAME_FCS_BYTES if self.fcs_owner is FcsOwner.MAC_LAYER else 0)

    def fcs_bytes_on_air(self) -> int:
        return FRAME_FCS_BYTES

    def phr_bytes(self) -> int:
        return PHR_STANDARD_INFO_BYTES

    def phr_coded_bits(self) -> int:
        return PHR_STANDARD_CODED_BITS

    def mac_payload_bytes(self, t: FrameType) -> int:
        """MAC PSDU of one frame as THIS profile builds it.

        The single place a frame length is computed:
        ``FRAME_HEADER_BYTES + n * FRAME_TIMESTAMP_BYTES + footer`` plus the
        FCS bytes this profile reserves INSIDE the MAC PSDU.  For the
        executable frame-v1 profile that reservation is 0, so this is exactly
        14 / 24 / 29 B.
        """
        return (self.mac_header_bytes()
                + frame_type_timestamp_count(t) * self.timestamp_bytes()
                + self.mac_footer_bytes()
                + self.mac_fcs_bytes())

    def on_air_bytes(self, t: FrameType) -> int:
        """On-air PSDU: the MAC payload plus the 2 FCS bytes -- 16/26/31 B.

        INVARIANT under ``fcs_owner``: moving who appends the FCS does not
        change a single sample on the air, it only changes which layer is
        responsible for it.
        """
        return self.mac_payload_bytes(t) - self.mac_fcs_bytes() \
            + self.fcs_bytes_on_air()

    def executable(self) -> bool:
        """True only for a layout this codec can actually build.

        The MAC-appends variant is DESCRIBABLE but not encodable here.
        """
        return self.fcs_owner is FcsOwner.PHY_LAYER

    def claim(self) -> FrameGeometryClaim:
        """The claim that agrees with this authority.

        Checking it back must report no mismatch; that self-consistency is
        what the R3 regression test relies on.
        """
        return FrameGeometryClaim(
            mac_header_bytes=self.mac_header_bytes(),
            timestamp_bytes=self.timestamp_bytes(),
            mac_footer_bytes=self.mac_footer_bytes(),
            mac_fcs_bytes=self.mac_fcs_bytes(),
            phr_bytes=self.phr_bytes())


#: The description-only variant: same layout, but the MAC reserves and appends
#: the 2 FCS bytes itself.  Provided so a configuration can STATE that layout
#: and be told it is not executable here, rather than having the number
#: silently accepted.
def frame_geometry_mac_appends_fcs(
        pid: FrameProfileId = FrameProfileId.TWR_V1) -> FrameProfileGeometry:
    return FrameProfileGeometry(id=pid, fcs_owner=FcsOwner.MAC_LAYER)


#: The executable authority for a profile id, or ``None`` when the id names no
#: implemented layout.  Never a silent fallback to frame v1.
def frame_geometry_for(pid: FrameProfileId) -> Optional[FrameProfileGeometry]:
    if not frame_profile_id_is_supported(pid):
        return None
    return FrameProfileGeometry(id=pid, fcs_owner=FcsOwner.PHY_LAYER)


@dataclass(frozen=True)
class GeometryMismatch:
    field: GeometryField
    expected: int
    actual: int


@dataclass(frozen=True)
class GeometryCheckResult:
    id: FrameProfileId
    authority: FrameProfileGeometry
    claimed: FrameGeometryClaim
    #: False when the authority is describeable but not encodable by this codec
    #: (the MAC-appends variant).
    fcs_owner_ok: bool
    #: The mismatching fields, in ``GEOMETRY_FIELD_ORDER``.
    mismatches: Tuple[GeometryMismatch, ...]

    def ok(self) -> bool:
        return self.fcs_owner_ok and not self.mismatches

    def first_mismatch(self) -> Optional[GeometryMismatch]:
        return self.mismatches[0] if self.mismatches else None

    def summary(self) -> str:
        """One line naming every field that disagrees.

        ``"mac_header_bytes: expected 14, got 7; phr_bytes: expected 2, got 12"``
        """
        parts: List[str] = []
        for m in self.mismatches:
            parts.append("%s: expected %d, got %d"
                         % (m.field, m.expected, m.actual))
        if not self.fcs_owner_ok:
            if parts:
                parts.append("")
            parts.append("fcs_owner: expected %s, got %s (this codec only "
                         "encodes the PHY-appends form)"
                         % (FcsOwner.PHY_LAYER, self.authority.fcs_owner))
        return "; ".join(parts)


#: Checks a caller-supplied claim against the authority, PER FIELD.  This is
#: the function a configuration validator calls: it never decides geometry, it
#: only reports where the claim diverges from the one source of truth, so a
#: 7-byte header is reported as ``mac_header_bytes: expected 14, got 7``
#: rather than quietly accepted with a 9-byte budget (M0 review R3).
def frame_geometry_check(claimed: FrameGeometryClaim,
                         authority: FrameProfileGeometry) -> GeometryCheckResult:
    mismatches: List[GeometryMismatch] = []
    for f in GEOMETRY_FIELD_ORDER:
        want = authority.claim().field(f)
        got = claimed.field(f)
        if want != got:
            mismatches.append(GeometryMismatch(f, want, got))
    return GeometryCheckResult(id=authority.id, authority=authority,
                               claimed=claimed,
                               fcs_owner_ok=authority.executable(),
                               mismatches=tuple(mismatches))


@dataclass
class FrameGeometry:
    """A CLAIM about the versioned TWR MAC frame layout (REQ-PROTO-06).

    NOT a second source of truth for it.  M0 review R3: this struct used to
    carry a pre-codec "7-byte header" geometry (FCF(2) + sequence(1) + PAN(2) +
    address(2)) with 9/19/24-byte budgets while the frame codec emitted a
    14-byte header and 16/26/31-byte on-air frames.  Both suites passed because
    nothing compared the two.

    Now the codec owns every one of these numbers
    (:class:`FrameProfileGeometry`) and this struct is only what a caller
    BELIEVES.  :func:`frame_geometry_claim` turns it into the codec's
    :class:`FrameGeometryClaim`, :func:`frame_geometry_check` reports every
    field that disagrees, and the validator turns each disagreement into its
    own rejection naming that exact field.  The fields stay here so the claim
    is visible in the canonical field listing, in the config hash and in the
    JSON -- an operator must be able to see what was claimed -- but they never
    decide anything.
    """

    #: MAC header bytes the caller believes the frame has.  frame v1: 14
    #: (version / function code / 16-bit session / seq / PAN / src / dst /
    #: flags), from FRAME_HEADER_BYTES.
    mac_header_bytes: int = 0
    #: bytes per timestamp field; 5 for the 40-bit frame-v1 timestamp.
    timestamp_bytes: int = 0
    #: trailing address / profile trailer; frame v1 has none.
    mac_footer_bytes: int = 0
    #: FCS bytes reserved INSIDE the MAC PSDU; 0 when the PHY appends it.
    mac_fcs_bytes: int = 0
    #: PHR INFORMATION bytes the caller believes are carried; frame v1: 2
    #: (13 information bits, SEC-DED coded to 19 bits over 21 symbols).  This
    #: is the PHR's INFORMATION size, not its on-air duration.
    phr_bytes: int = 0

    def any_set(self) -> bool:
        return (self.mac_header_bytes != 0 or self.timestamp_bytes != 0 or
                self.mac_footer_bytes != 0 or self.mac_fcs_bytes != 0 or
                self.phr_bytes != 0)

    def to_text(self) -> str:
        """One line, in the codec's own field order so the two read alike."""
        return ("mac_header_bytes=%d timestamp_bytes=%d mac_footer_bytes=%d "
                "mac_fcs_bytes=%d phr_bytes=%d"
                % (self.mac_header_bytes, self.timestamp_bytes,
                   self.mac_footer_bytes, self.mac_fcs_bytes, self.phr_bytes))

    def to_claim(self) -> FrameGeometryClaim:
        return FrameGeometryClaim(
            mac_header_bytes=self.mac_header_bytes,
            timestamp_bytes=self.timestamp_bytes,
            mac_footer_bytes=self.mac_footer_bytes,
            mac_fcs_bytes=self.mac_fcs_bytes,
            phr_bytes=self.phr_bytes)


#: The claim, in the codec's own claim type.  Same field names on purpose.
def frame_geometry_claim(g: FrameGeometry) -> FrameGeometryClaim:
    return g.to_claim()


#: The geometry the CODEC says, in the config's own struct.  This is how a
#: caller (and every test fixture) states a claim without keeping a private
#: copy of the numbers: fill the config from the authority, then let the
#: validator prove the two agree.
def frame_geometry_from_authority(
        g: FrameProfileGeometry) -> FrameGeometry:
    return FrameGeometry(
        mac_header_bytes=g.mac_header_bytes(),
        timestamp_bytes=g.timestamp_bytes(),
        mac_footer_bytes=g.mac_footer_bytes(),
        mac_fcs_bytes=g.mac_fcs_bytes(),
        phr_bytes=g.phr_bytes())


#: The claim of the executable authority for a profile id, or ``None`` when
#: the id names no implemented layout.  Never a silent fallback to frame v1.
def frame_geometry_of_profile(
        pid: FrameProfileId) -> Optional[FrameGeometry]:
    g = frame_geometry_for(pid)
    if g is None:
        return None
    return frame_geometry_from_authority(g)


def frame_psdu_bytes(t: Any, fcs: Any, legacy_fcs: Any = None) -> int:
    """MAC PSDU bytes a frame of type ``t`` occupies.

    INCLUDING the FCS when the MAC appends it and EXCLUDING it when the PHY
    does (exactly one layer appends the FCS, REQ-API-01 帧格式).

    A DELEGATION to the codec authority, which is the only place a frame length
    is computed.  M0 re-summed ``header + n * timestamp + footer`` from the
    config's own geometry claim, which is how a 7-byte header produced a
    9-byte Poll and understated the frame-duration budget.

    RETAINED CALL SHAPE.  ``frame_psdu_bytes(geometry, type, fcs)`` still
    compiles: the geometry argument is DELIBERATELY IGNORED, because a claim
    does not decide a length, and keeping the parameter means an existing call
    site keeps working while losing the ability to pass a number of its own.
    New code should use the two-argument form.
    """
    if isinstance(t, FrameGeometry):  # the old three-argument shape
        t, fcs = fcs, legacy_fcs
    owner = (FcsOwner.MAC_LAYER if fcs is FcsAppender.MAC
             else FcsOwner.PHY_LAYER)
    g = FrameProfileGeometry(id=FrameProfileId.TWR_V1, fcs_owner=owner)
    return g.mac_payload_bytes(t)


def frame_bytes_on_air(t: Any, fcs: Any, legacy_fcs: Any = None,
                       legacy_fcs_bytes: Any = None) -> int:
    """Transmitted bytes, with the FCS counted EXACTLY once -- 16/26/31.

    Invariant under which layer appends the FCS, which is exactly the property
    a frame-duration budget should have.  ``fcs_bytes`` from the old
    four-argument shape is ignored for the same reason as above: the FCS size
    is the codec's, not the config's.
    """
    if isinstance(t, FrameGeometry):  # the old four-argument shape
        t, fcs = fcs, legacy_fcs
    g = frame_geometry_for(FrameProfileId.TWR_V1)
    assert g is not None  # frame_v1 is the only implemented profile
    return g.on_air_bytes(t)


# ---------------------------------------------------------------------------
# Session id: the local -> wire contract (mirrors uwb_twr_frame.h)
# ---------------------------------------------------------------------------
# The wire session field of frame v1 is 2 bytes, so the wire session space is
# 16 bits and that is the ENTIRE space.  Everything below follows from those
# two facts; none of it is a policy choice the codec could have made differently.
SESSION_ID_WIRE_BITS = 16
SESSION_ID_WIRE_MAX = 0xFFFF
#: A locally chosen session id of 0 must not reach the air: the configuration
#: layer uses it as its "not stated" marker.  The codec itself does not refuse
#: it -- a frame is a frame -- so this is a caller rule, stated here so both
#: sides quote the same constant instead of inventing one.
SESSION_ID_RESERVED_LOCAL = 0
#: ~sqrt(2**16): the birthday bound of a 16-bit wire field.  No local->wire
#: mapping can raise it; a wider (or keyed) session field needs a different
#: frame version the peer must also understand.
SESSION_ID_BIRTHDAY_SESSIONS_50PCT = 256.0


class SessionIdError(_StrEnum):
    NONE = "none"
    #: local id > SESSION_ID_WIRE_MAX: refused, never truncated
    OUT_OF_WIRE_RANGE = "session_id_out_of_wire_range"


def session_id_error_to_exchange_status(e: SessionIdError) -> ExchangeStatus:
    """Map a session-id narrowing failure onto the frozen ExchangeStatus."""
    if e is SessionIdError.NONE:
        return ExchangeStatus.OK
    if e is SessionIdError.OUT_OF_WIRE_RANGE:
        # A configuration that cannot be represented is rejected before the
        # radio starts; it is not a peer or a decode problem.
        return ExchangeStatus.CONFIG_REJECTED
    return ExchangeStatus.INTERNAL_ERROR


def session_id_fits_wire(local_session_id: int) -> bool:
    """True when the local id is representable on the wire, bit for bit."""
    return 0 <= local_session_id <= SESSION_ID_WIRE_MAX


#: THE MAPPING RULE (frame v1):
#:
#:     wire_session_id == local_session_id, bit for bit, 16 bits,
#:                       little-endian at the session offset
#:     a local id outside [0, 0xFFFF] is REFUSED
#:
#: There is deliberately no folding, hashing, low-word extraction or derived
#: value.  A lossy local->wire map would make two different local sessions
#: produce byte-identical session fields, and a frame match would then accept
#: the wrong session's reply as this one's (REQ-PROTO-01) -- a wrong-peer frame
#: silently folded into a range.  A refusal is visible and attributable; a
#: collision is neither.  So: in range -> exact, out of range -> explicit error.
def session_id_to_wire(local_session_id: int) -> Tuple[int, str, SessionIdError]:
    """``(wire_id, error_text, error)``.

    On failure the wire id is 0, never left holding a plausible-looking value:
    a caller that ignores the error must not transmit a truncated id that some
    other session is already using.
    """
    if not session_id_fits_wire(local_session_id):
        return (0,
                "%s: local session id %d does not fit the %d-bit wire field at "
                "the session offset; refused, not truncated to %d"
                % (SessionIdError.OUT_OF_WIRE_RANGE, local_session_id,
                   SESSION_ID_WIRE_BITS, local_session_id & SESSION_ID_WIRE_MAX),
                SessionIdError.OUT_OF_WIRE_RANGE)
    return (local_session_id, "", SessionIdError.NONE)


def wire_session_id_collides(a: int, b: int) -> bool:
    """True only for two in-range ids that are the SAME integer.

    Because an out-of-range id is refused instead of narrowed, the mapping is
    injective over everything that can reach the air; the ``fits`` test is what
    makes that statement true rather than false for a rejected id (which
    collides with nothing, because it never goes on the air).
    """
    return session_id_fits_wire(a) and session_id_fits_wire(b) and a == b


@dataclass
class SessionConfig:
    protocol: Protocol = Protocol.SS
    role: Role = Role.INITIATOR
    local_address: int = 0
    peer_address: int = 0
    pan_id: int = 0
    #: LOCAL session id.  The wire field is 16 bits, so the mapping is
    #: identity-or-refuse (:func:`session_id_to_wire`), never a fold or a mask.
    #: The type stays a plain ``int`` DELIBERATELY rather than narrowing to 16
    #: bits: narrowing would make the local id space and the wire id space
    #: indistinguishable, so an operator could no longer express the id they
    #: intended and would never learn it does not fit.  Keeping the wider space
    #: makes the refusal a VISIBLE, TESTABLE, ATTRIBUTABLE configuration error
    #: instead of an unreachable code path.  0 is the frame profile's "no
    #: session" marker.
    session_id: int = 0
    exchange_id: int = 0
    #: Frame sequence number and its own wrap modulus, INDEPENDENT of the
    #: session id (帧序号回绕与会话 ID 独立).
    sequence: int = 0
    sequence_modulus: int = 256
    measurement_count: int = 1
    measurement_interval: Duration = field(default_factory=Duration)
    max_attempts_per_exchange: int = 1
    retry_backoff: Duration = field(default_factory=Duration)
    #: Phase 1 allows exactly one in-flight exchange per endpoint.
    max_in_flight_exchanges: int = MAX_IN_FLIGHT_EXCHANGES
    #: A peer frame with a mismatching PAN/address is WRONG_PEER, not a match.
    require_pan_match: bool = True
    require_address_match: bool = True


@dataclass
class PhyConfig:
    channel: int = 5
    center_frequency_hz: float = 0.0   # 0 = not stated; the plan decides
    tx_preamble_code: int = 0         # 0 = "not stated" -> rejected
    rx_preamble_code: int = 0
    preamble_symbols: int = 0         # SYNC repetitions; must be a PreambleLength
    prf_class: PrfClass = PrfClass.BPRF64
    #: PAYLOAD data rate, measured: 6.81 Mb/s is the only rate that round
    #: trips through this modulator/demodulator pair.
    data_rate: DataRate = DataRate.R6P8M
    #: PHR rate -- a SEPARATE quantity from ``data_rate`` (M0.1, R2).  It used
    #: to be a DataRate required to EQUAL ``data_rate``, which had the rule
    #: exactly backwards: this modulator sends a 0.85 Mb/s PHR (21 symbols of
    #: 512 chips) ahead of a 6.81 Mb/s payload (8 chips per burst of 64 per
    #: symbol).  The two axes are now separately typed and separately
    #: validated, and neither is derived from the other.
    phr_rate: PhrRate = PhrRate.STANDARD_850K


@dataclass
class FrameFormatConfig:
    #: WHICH MAC LAYOUT.  A FrameProfileId, not a free-form string: an
    #: arbitrary non-empty profile name must not be able to act as an
    #: executable geometry (M0 review R3).  It selects the authority that the
    #: ``geometry`` claim below is checked against, and it is REQUIRED in the
    #: JSON schema (twr-config/2).
    frame_profile: FrameProfileId = FrameProfileId.TWR_V1
    sfd_mode: SfdMode = SfdMode.R4Z2
    sfd_symbols: int = 0
    #: Timeout after which a received frame is declared SFD-missing.  Zero
    #: means "disabled"; non-zero must be in a device-tick domain.
    sfd_timeout: TimedField = field(default_factory=TimedField)
    #: PHR presence/form.  The PHR RATE is ``phy.phr_rate``; the two are
    #: separate axes and neither is derived from the other.
    phr_mode: PhrMode = PhrMode.STANDARD
    ranging_bit: bool = True

    # THREE DISTINCT CONCEPTS, never conflated:
    #  (1) upper-layer bytes handed to the MAC, stated as a CLAIM about the
    #      layout that the codec then checks field by field,
    geometry: FrameGeometry = field(default_factory=FrameGeometry)
    mac_psdu_bytes: int = 0
    mac_psdu_includes_fcs: bool = False
    fcs_append: FcsAppender = FcsAppender.PHY
    fcs_bytes: int = 2                # 16-bit CRC for this frame profile
    #: 1 = the application payload the caller wants to carry; the MAC header,
    #: the timestamps and the FCS are NOT counted here.
    application_payload_bytes: int = 0

    #: STS: phase-2 only.  Present so the interface can grow, rejected now.
    sts_mode: StsMode = StsMode.OFF
    sts_length_symbols: int = 0


@dataclass
class TransmitConfig:
    port: int = 0                     # physical TX channel index in the device
    #: The THREE distinct power concepts (REQ-API-01 发射, AGENTS rule 7):
    #:  gain_db is the UHD/USRP TX gain setting in dB;
    gain_db: Optional[float] = None
    #:  iq_amplitude is digital scaling of the SC16 waveform, (0, 1];
    iq_amplitude: Optional[float] = None
    #:  calibrated_tx_power_dbm is the power MEASURED at the antenna plane.
    calibrated_tx_power_dbm: Optional[float] = None
    power_policy: TxPowerPolicy = TxPowerPolicy.LEAVE_UNTOUCHED
    pulse_shaping: PulseShaping = PulseShaping.EXISTING_HRP
    #: A vendor "power word" is a FOURTH, separately named field.  It may only
    #: be set together with the backend capability id that defines its bits.
    vendor_power_word: Optional[int] = None
    vendor_power_word_backend: str = ""


@dataclass
class ReceiveConfig:
    port: int = 0                     # physical RX channel index in the device
    gain_db: Optional[float] = None   # RX gain in dB
    agc: AgcMode = AgcMode.MANUAL
    bandwidth_hz: Optional[float] = None   # RF bandwidth != sample rate
    detection_threshold: float = 0.0      # energy / coarse gate, linear
    correlation_threshold: float = 0.0    # fine preamble correlation gate
    first_path_threshold: float = 0.0     # first-path quality gate
    first_path_index: int = 0             # first usable CIR index
    first_path_window: int = 0            # CIR taps kept for the search
    #: A vendor PAC-like value is ONLY meaningful together with the backend
    #: capability that defines it AND the software step the backend applies.
    #: Mapping a PAC value onto an arbitrary software detection step is
    #: forbidden (REQ-API-01 接收).
    vendor_pac_value: Optional[int] = None
    vendor_pac_backend: str = ""
    vendor_pac_applied_step: Optional[float] = None


@dataclass
class RadioReadback:
    present: bool = False
    sample_rate_hz: float = 0.0
    center_freq_hz: float = 0.0
    tx_channel: int = 0
    rx_channel: int = 0
    mpm_string: str = ""      # e.g. "CG400" / "X410"
    fpga_image: str = ""
    uhd_version: str = ""
    clock_source: str = ""
    time_source: str = ""


@dataclass
class EndpointBinding:
    """One logical endpoint sharing the physical device.

    Phase 1: two of these (A = TX0/RX0, B = TX1/RX1) on one X410.
    """

    id: str = ""                    # "A" / "B"
    role: Role = Role.INITIATOR
    tx_channel: int = 0
    rx_channel: int = 0
    native_sample_rate_hz: float = 0.0
    occupies_resources: bool = True


@dataclass
class RadioConfig:
    device_args: str = ""
    tx_channel: int = 0
    rx_channel: int = 0
    native_sample_rate_hz: float = 0.0
    clock_source: str = ""          # "" = leave untouched
    time_source: str = ""           # "" = leave untouched
    fpga_image: str = ""            # "" = build default
    dpdk_config: str = ""           # "" = no DPDK
    #: The other endpoint(s) of the same session, so the config is
    #: self-describing and the validator can reject channel conflicts.
    peers: List[EndpointBinding] = field(default_factory=list)
    #: Filled at startup.  A readback that differs from the request is a
    #: STARTUP FAILURE (REQ-API-01 无线设备: 读回不符启动失败).
    readback: RadioReadback = field(default_factory=RadioReadback)
    #: A dry-run/offline session may set it false; a real radio must not.
    require_readback: bool = True


@dataclass
class PerMessageTiming:
    #: When the first Poll of a measurement is transmitted.
    poll_start: TimedField = field(default_factory=TimedField)
    #: Responder: RX Poll RMARKER -> Response TX RMARKER.  THE PROTOCOL REPLY
    #: DELAY.
    poll_to_response: TimedField = field(default_factory=TimedField)
    #: Initiator: RX Response RMARKER -> Final TX RMARKER (DS only).  A
    #: different thing from poll_to_response.
    response_to_final: TimedField = field(default_factory=TimedField)
    #: Report: phase 1 has no Report frame, so this must stay zero.
    final_to_report: TimedField = field(default_factory=TimedField)
    #: After the END OF OUR OWN TRANSMITTED FRAME, delay before RX is armed.
    #: THE POST-TX RX-ENABLE DELAY -- a completely different number from the
    #: reply delay and from the RX timeout.
    post_tx_rx_enable: TimedField = field(default_factory=TimedField)
    #: Measured minimum UHD timed-command lead time.  The operator must supply
    #: it; there is NO default (REQ-GR-04).  At the time of writing it is
    #: UNMEASURED, which is why the shipped test config's value is a QA
    #: placeholder and is labelled as one.
    min_tx_lead_time: TimedField = field(default_factory=TimedField)


@dataclass
class TimeoutConfig:
    #: Per-frame RX windows, armed at RX ENABLE on the device clock.
    poll_rx_window: TimedField = field(default_factory=TimedField)
    response_rx_window: TimedField = field(default_factory=TimedField)
    final_rx_window: TimedField = field(default_factory=TimedField)
    report_rx_window: TimedField = field(default_factory=TimedField)
    #: How long to wait for the expected RX frame, measured FROM RX ENABLE.
    rx_timeout: TimedField = field(default_factory=TimedField)
    #: Whole-exchange deadline on the HOST MONOTONIC CLOCK.
    exchange_timeout: TimedField = field(default_factory=TimedField)
    #: Gap between a failed attempt and its retry, host monotonic.
    retry_interval: TimedField = field(default_factory=TimedField)


@dataclass
class CalibrationRecord:
    """Versioned by device / channel / rate / gain / profile.

    ``valid_until_monotonic_ns == 0`` means no expiry.
    """

    calibration_id: str = ""
    device_serial: str = ""
    channel: int = 0
    native_sample_rate_hz: float = 0.0
    profile_version: str = ""
    gain_db: Optional[float] = None   # the gain the constants were taken at
    valid_until_monotonic_ns: int = 0


@dataclass
class TimestampCalibrationConfig:
    #: Link / antenna / cable delays in integer NANOSECONDS (named type, not
    #: a vendor tick).
    tx_link_delay: Duration = field(default_factory=Duration)
    rx_link_delay: Duration = field(default_factory=Duration)
    antenna_delay: Duration = field(default_factory=Duration)
    cable_delay: Duration = field(default_factory=Duration)
    #: The SAME quantity in DEVICE TICKS -- a separate field, never added to
    #: the nanosecond fields above.
    tx_link_delay_native_ticks: int = 0
    rx_link_delay_native_ticks: int = 0
    native_sample_rate_hz: float = 0.0
    #: Which unit the *_native_ticks fields and the Durations refer to.
    link_delay_unit: TimeUnit = TimeUnit.NANOSECONDS
    first_path_algorithm: FirstPathAlgorithm = FirstPathAlgorithm.LEADING_EDGE
    cfo_compensation: CompensationFlag = CompensationFlag.OFF
    sfo_compensation: CompensationFlag = CompensationFlag.OFF
    calibration_id: str = ""
    record: CalibrationRecord = field(default_factory=CalibrationRecord)
    #: REQ-CAL-01: a calibration is applicable EXACTLY ONCE.
    applied_count: int = 0
    #: A calibration is mandatory for a validated absolute range claim.
    calibration_required: bool = True


@dataclass
class DiagnosticsConfig:
    cir_capture_enabled: bool = False
    cir_capture_max_bytes: int = 0
    cir_capture_stride: int = 0
    short_iq_enabled: bool = False
    short_iq_max_bytes: int = 0
    short_iq_stride: int = 0
    raw_frame_dump: bool = False
    raw_frame_max_bytes: int = 0
    result_output_path: str = ""       # "" = no file output
    result_queue_capacity: int = 0
    event_queue_capacity: int = 0
    stats_cadence: TimedField = field(default_factory=TimedField)
    #: Diagnostic I/O must never run on the realtime thread (REQ-API-01
    #: 诊断).  The flag exists so a violation is a CONFIG error, not a
    #: surprise discovered during a soak.
    io_on_realtime_thread: bool = False


@dataclass
class ConfigMeta:
    schema_version: str = SCHEMA_VERSION
    profile_version: str = ""
    calibration_version: str = ""
    #: Optional operator note carried into every result.
    label: str = ""


@dataclass
class TwrConfig:
    meta: ConfigMeta = field(default_factory=ConfigMeta)
    session: SessionConfig = field(default_factory=SessionConfig)
    phy: PhyConfig = field(default_factory=PhyConfig)
    frame: FrameFormatConfig = field(default_factory=FrameFormatConfig)
    tx: TransmitConfig = field(default_factory=TransmitConfig)
    rx: ReceiveConfig = field(default_factory=ReceiveConfig)
    radio: RadioConfig = field(default_factory=RadioConfig)
    timing: PerMessageTiming = field(default_factory=PerMessageTiming)
    timeouts: TimeoutConfig = field(default_factory=TimeoutConfig)
    calibration: TimestampCalibrationConfig = \
        field(default_factory=TimestampCalibrationConfig)
    diagnostics: DiagnosticsConfig = field(default_factory=DiagnosticsConfig)


# ===========================================================================
# 8. Canonical field listing (requested vs effective diff + config hash)
# ===========================================================================


class FieldRecord:
    """A flattened "path=value" record."""

    __slots__ = ("path", "value")

    def __init__(self, path: str, value: str):
        self.path = path
        self.value = value

    def __eq__(self, o: object) -> bool:
        return (isinstance(o, FieldRecord) and o.path == self.path
                and o.value == self.value)

    def __hash__(self) -> int:
        return hash((self.path, self.value))

    def __repr__(self) -> str:
        return "FieldRecord(%r, %r)" % (self.path, self.value)


class FieldSink:
    """Ordered "path=value" writer."""

    def __init__(self) -> None:
        self.records: List[FieldRecord] = []

    def add(self, path: str, v: str) -> None:
        self.records.append(FieldRecord(path, v))

    def i64(self, path: str, v: int) -> None:
        self.add(path, int_to_text(int(v)))

    def boolean(self, path: str, v: bool) -> None:
        self.add(path, bool_to_text(v))

    def double(self, path: str, v: float) -> None:
        self.add(path, double_to_text(v))

    def dur(self, path: str, d: Duration) -> None:
        self.add(path, duration_to_text(d))

    def enum(self, path: str, v: Any) -> None:
        self.add(path, str(v) if v is not None else "invalid")

    def opt_double(self, path: str, o: Optional[float]) -> None:
        self.add(path, double_to_text(o) if o is not None else "null")

    def opt_i64(self, path: str, o: Optional[int]) -> None:
        self.add(path, int_to_text(int(o)) if o is not None else "null")

    def opt_u32(self, path: str, o: Optional[int]) -> None:
        self.add(path, int_to_text(int(o)) if o is not None else "null")

    def opt_str(self, path: str, o: Optional[str]) -> None:
        self.add(path, o if o is not None else "null")

    def opt_marker(self, path: str, o: Optional[TimestampMarker]) -> None:
        self.add(path, str(o) if o is not None else "null")

    def timed(self, path: str, f: TimedField) -> None:
        self.add(path, f.to_text())


def _collect_timed(s: FieldSink, p: str, f: TimedField) -> None:
    s.add(p + ".ns", int_to_text(f.value.nanos()))
    s.enum(p + ".domain", f.domain)
    s.enum(p + ".reference", f.reference)
    s.opt_marker(p + ".marker", f.marker)
    s.double(p + ".quantisation_hz", f.required_quantisation_hz)
    s.i64(p + ".max_quantisation_error_ns", f.max_quantisation_error_ns)
    s.add(p + ".note", f.note)


def _collect_fields(s: FieldSink, v: Any, p: str) -> None:
    """Ordered, per-group field collection.

    A single dispatching function rather than an overload set: the group ORDER
    written here is part of the canonical text and therefore of config_hash, so
    it must live in exactly one place.
    """
    if isinstance(v, ConfigMeta):
        s.add(p + ".schema_version", v.schema_version)
        s.add(p + ".profile_version", v.profile_version)
        s.add(p + ".calibration_version", v.calibration_version)
        s.add(p + ".label", v.label)

    elif isinstance(v, SessionConfig):
        s.enum(p + ".protocol", v.protocol)
        s.enum(p + ".role", v.role)
        s.i64(p + ".local_address", v.local_address)
        s.i64(p + ".peer_address", v.peer_address)
        s.i64(p + ".pan_id", v.pan_id)
        s.i64(p + ".session_id", v.session_id)
        s.i64(p + ".exchange_id", v.exchange_id)
        s.i64(p + ".sequence", v.sequence)
        s.i64(p + ".sequence_modulus", v.sequence_modulus)
        s.i64(p + ".measurement_count", v.measurement_count)
        s.dur(p + ".measurement_interval", v.measurement_interval)
        s.i64(p + ".max_attempts_per_exchange", v.max_attempts_per_exchange)
        s.dur(p + ".retry_backoff", v.retry_backoff)
        s.i64(p + ".max_in_flight_exchanges", v.max_in_flight_exchanges)
        s.boolean(p + ".require_pan_match", v.require_pan_match)
        s.boolean(p + ".require_address_match", v.require_address_match)

    elif isinstance(v, PhyConfig):
        s.i64(p + ".channel", v.channel)
        s.double(p + ".center_frequency_hz", v.center_frequency_hz)
        s.i64(p + ".tx_preamble_code", v.tx_preamble_code)
        s.i64(p + ".rx_preamble_code", v.rx_preamble_code)
        s.i64(p + ".preamble_symbols", v.preamble_symbols)
        s.enum(p + ".prf_class", v.prf_class)
        s.enum(p + ".data_rate", v.data_rate)
        s.enum(p + ".phr_rate", v.phr_rate)

    elif isinstance(v, FrameGeometry):
        s.i64(p + ".mac_header_bytes", v.mac_header_bytes)
        s.i64(p + ".timestamp_bytes", v.timestamp_bytes)
        s.i64(p + ".mac_footer_bytes", v.mac_footer_bytes)
        s.i64(p + ".mac_fcs_bytes", v.mac_fcs_bytes)
        s.i64(p + ".phr_bytes", v.phr_bytes)

    elif isinstance(v, FrameFormatConfig):
        # frame_profile comes FIRST, exactly as C++ collect_fields orders it,
        # so the canonical text -- and therefore config_hash -- is identical on
        # both sides.
        s.enum(p + ".frame_profile", v.frame_profile)
        s.enum(p + ".sfd_mode", v.sfd_mode)
        s.i64(p + ".sfd_symbols", v.sfd_symbols)
        _collect_timed(s, p + ".sfd_timeout", v.sfd_timeout)
        s.enum(p + ".phr_mode", v.phr_mode)
        s.boolean(p + ".ranging_bit", v.ranging_bit)
        _collect_fields(s, v.geometry, p + ".geometry")
        s.i64(p + ".mac_psdu_bytes", v.mac_psdu_bytes)
        s.boolean(p + ".mac_psdu_includes_fcs", v.mac_psdu_includes_fcs)
        s.enum(p + ".fcs_append", v.fcs_append)
        s.i64(p + ".fcs_bytes", v.fcs_bytes)
        s.i64(p + ".application_payload_bytes", v.application_payload_bytes)
        s.enum(p + ".sts_mode", v.sts_mode)
        s.i64(p + ".sts_length_symbols", v.sts_length_symbols)

    elif isinstance(v, TransmitConfig):
        s.i64(p + ".port", v.port)
        s.opt_double(p + ".gain_db", v.gain_db)
        s.opt_double(p + ".iq_amplitude", v.iq_amplitude)
        s.opt_double(p + ".calibrated_tx_power_dbm", v.calibrated_tx_power_dbm)
        s.enum(p + ".power_policy", v.power_policy)
        s.enum(p + ".pulse_shaping", v.pulse_shaping)
        s.opt_u32(p + ".vendor_power_word", v.vendor_power_word)
        s.add(p + ".vendor_power_word_backend", v.vendor_power_word_backend)

    elif isinstance(v, ReceiveConfig):
        s.i64(p + ".port", v.port)
        s.opt_double(p + ".gain_db", v.gain_db)
        s.enum(p + ".agc", v.agc)
        s.opt_double(p + ".bandwidth_hz", v.bandwidth_hz)
        s.double(p + ".detection_threshold", v.detection_threshold)
        s.double(p + ".correlation_threshold", v.correlation_threshold)
        s.double(p + ".first_path_threshold", v.first_path_threshold)
        s.i64(p + ".first_path_index", v.first_path_index)
        s.i64(p + ".first_path_window", v.first_path_window)
        s.opt_u32(p + ".vendor_pac_value", v.vendor_pac_value)
        s.add(p + ".vendor_pac_backend", v.vendor_pac_backend)
        s.opt_double(p + ".vendor_pac_applied_step", v.vendor_pac_applied_step)

    elif isinstance(v, EndpointBinding):
        s.add(p + ".id", v.id)
        s.enum(p + ".role", v.role)
        s.i64(p + ".tx_channel", v.tx_channel)
        s.i64(p + ".rx_channel", v.rx_channel)
        s.double(p + ".native_sample_rate_hz", v.native_sample_rate_hz)
        s.boolean(p + ".occupies_resources", v.occupies_resources)

    elif isinstance(v, RadioReadback):
        s.boolean(p + ".present", v.present)
        s.double(p + ".sample_rate_hz", v.sample_rate_hz)
        s.double(p + ".center_freq_hz", v.center_freq_hz)
        s.i64(p + ".tx_channel", v.tx_channel)
        s.i64(p + ".rx_channel", v.rx_channel)
        s.add(p + ".mpm_string", v.mpm_string)
        s.add(p + ".fpga_image", v.fpga_image)
        s.add(p + ".uhd_version", v.uhd_version)
        s.add(p + ".clock_source", v.clock_source)
        s.add(p + ".time_source", v.time_source)

    elif isinstance(v, RadioConfig):
        s.add(p + ".device_args", v.device_args)
        s.i64(p + ".tx_channel", v.tx_channel)
        s.i64(p + ".rx_channel", v.rx_channel)
        s.double(p + ".native_sample_rate_hz", v.native_sample_rate_hz)
        s.add(p + ".clock_source", v.clock_source)
        s.add(p + ".time_source", v.time_source)
        s.add(p + ".fpga_image", v.fpga_image)
        s.add(p + ".dpdk_config", v.dpdk_config)
        for i, peer in enumerate(v.peers):
            _collect_fields(s, peer, "%s.peers[%d]" % (p, i))
        _collect_fields(s, v.readback, p + ".readback")
        s.boolean(p + ".require_readback", v.require_readback)

    elif isinstance(v, PerMessageTiming):
        _collect_timed(s, p + ".poll_start", v.poll_start)
        _collect_timed(s, p + ".poll_to_response", v.poll_to_response)
        _collect_timed(s, p + ".response_to_final", v.response_to_final)
        _collect_timed(s, p + ".final_to_report", v.final_to_report)
        _collect_timed(s, p + ".post_tx_rx_enable", v.post_tx_rx_enable)
        _collect_timed(s, p + ".min_tx_lead_time", v.min_tx_lead_time)

    elif isinstance(v, TimeoutConfig):
        _collect_timed(s, p + ".poll_rx_window", v.poll_rx_window)
        _collect_timed(s, p + ".response_rx_window", v.response_rx_window)
        _collect_timed(s, p + ".final_rx_window", v.final_rx_window)
        _collect_timed(s, p + ".report_rx_window", v.report_rx_window)
        _collect_timed(s, p + ".rx_timeout", v.rx_timeout)
        _collect_timed(s, p + ".exchange_timeout", v.exchange_timeout)
        _collect_timed(s, p + ".retry_interval", v.retry_interval)

    elif isinstance(v, CalibrationRecord):
        s.add(p + ".calibration_id", v.calibration_id)
        s.add(p + ".device_serial", v.device_serial)
        s.i64(p + ".channel", v.channel)
        s.double(p + ".native_sample_rate_hz", v.native_sample_rate_hz)
        s.add(p + ".profile_version", v.profile_version)
        s.opt_double(p + ".gain_db", v.gain_db)
        s.i64(p + ".valid_until_monotonic_ns", v.valid_until_monotonic_ns)

    elif isinstance(v, TimestampCalibrationConfig):
        s.dur(p + ".tx_link_delay", v.tx_link_delay)
        s.dur(p + ".rx_link_delay", v.rx_link_delay)
        s.dur(p + ".antenna_delay", v.antenna_delay)
        s.dur(p + ".cable_delay", v.cable_delay)
        s.i64(p + ".tx_link_delay_native_ticks", v.tx_link_delay_native_ticks)
        s.i64(p + ".rx_link_delay_native_ticks", v.rx_link_delay_native_ticks)
        s.double(p + ".native_sample_rate_hz", v.native_sample_rate_hz)
        s.enum(p + ".link_delay_unit", v.link_delay_unit)
        s.enum(p + ".first_path_algorithm", v.first_path_algorithm)
        s.enum(p + ".cfo_compensation", v.cfo_compensation)
        s.enum(p + ".sfo_compensation", v.sfo_compensation)
        s.add(p + ".calibration_id", v.calibration_id)
        _collect_fields(s, v.record, p + ".record")
        s.i64(p + ".applied_count", v.applied_count)
        s.boolean(p + ".calibration_required", v.calibration_required)

    elif isinstance(v, DiagnosticsConfig):
        s.boolean(p + ".cir_capture_enabled", v.cir_capture_enabled)
        s.i64(p + ".cir_capture_max_bytes", v.cir_capture_max_bytes)
        s.i64(p + ".cir_capture_stride", v.cir_capture_stride)
        s.boolean(p + ".short_iq_enabled", v.short_iq_enabled)
        s.i64(p + ".short_iq_max_bytes", v.short_iq_max_bytes)
        s.i64(p + ".short_iq_stride", v.short_iq_stride)
        s.boolean(p + ".raw_frame_dump", v.raw_frame_dump)
        s.i64(p + ".raw_frame_max_bytes", v.raw_frame_max_bytes)
        s.add(p + ".result_output_path", v.result_output_path)
        s.i64(p + ".result_queue_capacity", v.result_queue_capacity)
        s.i64(p + ".event_queue_capacity", v.event_queue_capacity)
        _collect_timed(s, p + ".stats_cadence", v.stats_cadence)
        s.boolean(p + ".io_on_realtime_thread", v.io_on_realtime_thread)

    elif isinstance(v, TwrConfig):
        _collect_fields(s, v.meta, p + "meta")
        _collect_fields(s, v.session, p + "session")
        _collect_fields(s, v.phy, p + "phy")
        _collect_fields(s, v.frame, p + "frame")
        _collect_fields(s, v.tx, p + "tx")
        _collect_fields(s, v.rx, p + "rx")
        _collect_fields(s, v.radio, p + "radio")
        _collect_fields(s, v.timing, p + "timing")
        _collect_fields(s, v.timeouts, p + "timeouts")
        _collect_fields(s, v.calibration, p + "calibration")
        _collect_fields(s, v.diagnostics, p + "diagnostics")

    else:
        raise TypeError("no field collector for %r" % type(v).__name__)


def flatten_fields(cfg: TwrConfig) -> List[FieldRecord]:
    """Ordered "path=value" listing.

    Stable across runs and across languages, so it can be hashed and diffed.
    """
    s = FieldSink()
    _collect_fields(s, cfg, "")
    return s.records


def canonical_text(fields: Sequence[FieldRecord]) -> str:
    """``path=value\\n`` lines; the exact text config_hash digests."""
    return "".join("%s=%s\n" % (f.path, f.value) for f in fields)


def config_hash(cfg: TwrConfig) -> str:
    """FNV-1a 64 over the canonical text, formatted as the C++ side formats it.

    Not a security hash: an identifier that appears in every result so a config
    can be tied to the run that used it (REQ-OUT-01).
    """
    data = canonical_text(flatten_fields(cfg)).encode("utf-8")
    h = 1469598103934665603
    for byte in data:
        h ^= byte
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return "fnv1a64:%016x" % h


class FieldChange:
    """One requested-vs-effective difference."""

    __slots__ = ("path", "requested", "effective", "note")

    def __init__(self, path: str, requested: str, effective: str, note: str = ""):
        self.path = path
        self.requested = requested
        self.effective = effective
        self.note = note

    def adjusted(self) -> bool:
        return self.requested != self.effective

    def to_dict(self) -> Dict[str, str]:
        return {"path": self.path, "requested": self.requested,
                "effective": self.effective, "note": self.note}

    def __repr__(self) -> str:
        return "FieldChange(%r, %r, %r, %r)" % (self.path, self.requested,
                                                self.effective, self.note)


def diff_fields(requested: TwrConfig, effective: TwrConfig) -> List[FieldChange]:
    """The per-field differences between a requested and an effective config."""
    a = flatten_fields(requested)
    b = flatten_fields(effective)
    out: List[FieldChange] = []
    n = min(len(a), len(b))
    for i in range(n):
        if a[i].path != b[i].path or a[i].value == b[i].value:
            continue
        out.append(FieldChange(a[i].path, a[i].value, b[i].value))
    return out


# ===========================================================================
# 9. The validator
# ===========================================================================
#
# Every check appends a specific violation.  There is no early return: a
# config that is wrong in many places is reported in full so the operator
# fixes them in one pass.  There is also NO "fill in the blanks" path: a
# missing value is a rejection, never a silent radar / QM35 default
# (REQ-SCOPE-01).


class ConfigValidator:
    """Runs every check and reports ALL violations, not just the first.

    There is no early return and no "fill in the blanks" path: a config that
    is wrong in many places is reported in full so the operator fixes them in
    one pass, and a missing value is a rejection rather than a silent radar /
    QM35 default (REQ-SCOPE-01).  Prefer the module-level ``validate()``.
    """

    def __init__(self, caps: Capabilities):
        self.caps = caps
        self.report = ValidationReport()

    # -- helpers ---------------------------------------------------------
    def _rej(self, f: str, r: ConfigReason, s: ExchangeStatus, m: str,
             req: str) -> None:
        self.report.add_rejection(f, r, s, m, req)

    def _cfg_rej(self, f: str, r: ConfigReason, m: str,
                 req: str = "REQ-API-01") -> None:
        self._rej(f, r, ExchangeStatus.CONFIG_REJECTED, m, req)

    def _unsup(self, f: str, m: str, r: ConfigReason = ConfigReason.UNSUPPORTED,
               req: str = "REQ-SCOPE-01") -> None:
        self._rej(f, r, ExchangeStatus.UNSUPPORTED, m, req)

    def _finite(self, f: str, v: float, req: str = "REQ-API-01") -> None:
        if not _is_finite(v):
            self._cfg_rej(f, ConfigReason.NOT_FINITE,
                         "must be finite (NaN/Inf rejected)", req)

    def _finite_opt(self, f: str, o: Optional[float]) -> None:
        if o is not None and not _is_finite(o):
            self._cfg_rej(f, ConfigReason.NOT_FINITE,
                          "must be finite when present (NaN/Inf rejected)")

    def _finite_timed(self, f: str, t: TimedField) -> None:
        self._finite(f + ".quantisation_hz", t.required_quantisation_hz)
        if t.value.negative():
            self._cfg_rej(f, ConfigReason.NEGATIVE_VALUE,
                          "a delay or window may not be negative")

    # -- sections --------------------------------------------------------
    def run(self, c: TwrConfig) -> ValidationReport:
        self.report = ValidationReport()
        self.check_meta(c)
        self.check_session(c)
        self.check_phy(c)
        self.check_frame(c)
        self.check_tx(c)
        self.check_rx(c)
        self.check_radio(c)
        self.check_timing(c)
        self.check_timeouts(c)
        self.check_calibration(c)
        self.check_diagnostics(c)
        self.check_frame_lengths(c)
        return self.report

    def check_meta(self, c: TwrConfig) -> None:
        # Two INDEPENDENT checks, not an if/elif: an empty version is both "not
        # stated" and "not the version this build implements", and reporting
        # only the first would tell an operator to fill in a field when the
        # field is already there with the wrong value.
        if c.meta.schema_version == "":
            self._cfg_rej("meta.schema_version", ConfigReason.EMPTY_VALUE,
                          "schema_version is required")
        if c.meta.schema_version != self.caps.schema_version:
            # A version this build KNOWS how to migrate from gets the migration
            # message; one it has never heard of gets the generic one.  Neither
            # is a silent misread: the v1 and v2 schemas disagree about what a
            # PHR rate is, so reading one as the other would accept a config
            # that transmits something the author never asked for.
            if c.meta.schema_version in LEGACY_SCHEMA_VERSIONS:
                self._unsup("meta.schema_version",
                            "this build implements %s, config asks for '%s': "
                            "%s" % (self.caps.schema_version,
                                    c.meta.schema_version,
                                    SCHEMA_V1_MIGRATION_REASON))
            else:
                self._unsup("meta.schema_version",
                            "this build implements %s, config asks for '%s'" % (
                                self.caps.schema_version, c.meta.schema_version))
        if c.meta.profile_version == "":
            self._cfg_rej("meta.profile_version", ConfigReason.EMPTY_VALUE,
                          "profile_version is required (REQ-OUT-01 "
                          "calibration/profile/config hash)")

    def check_session(self, c: TwrConfig) -> None:
        s = c.session
        if s.local_address == s.peer_address:
            self._cfg_rej("session.peer_address", ConfigReason.FIELD_CONFLICT,
                          "local and peer address are identical (would make "
                          "every frame a match)")
        if s.local_address in (0x0000, 0xFFFF):
            self._cfg_rej("session.local_address", ConfigReason.FIELD_CONFLICT,
                          "0x0000 and 0xffff are reserved/invalid local "
                          "addresses")
        if s.peer_address in (0x0000, 0xFFFF):
            self._cfg_rej("session.peer_address", ConfigReason.FIELD_CONFLICT,
                          "0x0000 and 0xffff are reserved/invalid peer addresses")
        if s.pan_id in (0x0000, 0xFFFF):
            self._cfg_rej("session.pan_id", ConfigReason.EMPTY_VALUE,
                          "PAN 0x0000/0xffff is not usable")
        if s.session_id == 0:
            self._cfg_rej("session.session_id", ConfigReason.ZERO_VALUE,
                          "0 is the frame profile's 'no session' marker")
        else:
            # The wire session field of frame v1 is 16 bits, so the local ->
            # wire mapping is identity-or-refuse: in range -> exact, out of
            # range -> an explicit error.  It is NOT folded, masked or hashed,
            # because a lossy map makes two different local sessions produce a
            # byte-identical session field and the frame match would then
            # accept the wrong session's reply as this one's (REQ-PROTO-01).
            _wire, sid_err_text, sid_err = session_id_to_wire(s.session_id)
            if sid_err is not SessionIdError.NONE:
                self._rej("session.session_id", ConfigReason.OUT_OF_RANGE,
                          session_id_error_to_exchange_status(sid_err),
                          sid_err_text, "REQ-PROTO-01")
        if s.sequence_modulus == 0:
            self._cfg_rej("session.sequence_modulus", ConfigReason.ZERO_VALUE,
                          "wrap modulus is zero")
        elif s.sequence_modulus not in (4, 16, 64, 256):
            self._cfg_rej("session.sequence_modulus", ConfigReason.OUT_OF_RANGE,
                          "must be 4, 16, 64 or 256 (2/4/6/8 sequence bits)")
        if s.sequence >= s.sequence_modulus:
            self._cfg_rej("session.sequence", ConfigReason.INDEX_OUT_OF_RANGE,
                          "sequence does not fit the wrap modulus")
        if s.measurement_count == 0 or s.measurement_count > MAX_MEASUREMENT_COUNT:
            self._cfg_rej("session.measurement_count", ConfigReason.OUT_OF_RANGE,
                          "must be 1..%d" % MAX_MEASUREMENT_COUNT)
        if s.measurement_interval.negative():
            self._cfg_rej("session.measurement_interval",
                          ConfigReason.NEGATIVE_VALUE, "negative interval")
        if (s.max_attempts_per_exchange == 0 or
                s.max_attempts_per_exchange > MAX_ATTEMPTS_PER_EXCHANGE):
            self._cfg_rej("session.max_attempts_per_exchange",
                          ConfigReason.OUT_OF_RANGE,
                          "must be 1..%d" % MAX_ATTEMPTS_PER_EXCHANGE)
        if s.retry_backoff.negative():
            self._cfg_rej("session.retry_backoff", ConfigReason.NEGATIVE_VALUE,
                          "negative backoff")
        if s.max_attempts_per_exchange > 1:
            if s.retry_backoff.is_zero():
                self._cfg_rej("session.retry_backoff", ConfigReason.FIELD_CONFLICT,
                              "a retry policy with more than one attempt needs "
                              "a non-zero backoff")
        elif not s.retry_backoff.is_zero():
            self._cfg_rej("session.retry_backoff", ConfigReason.FIELD_CONFLICT,
                          "retry_backoff is set but max_attempts_per_exchange "
                          "is 1")
        if s.max_in_flight_exchanges != MAX_IN_FLIGHT_EXCHANGES:
            self._unsup(
                "session.max_in_flight_exchanges",
                "phase 1 allows at most %d in-flight exchange per endpoint; "
                "requested %d" % (MAX_IN_FLIGHT_EXCHANGES,
                                  s.max_in_flight_exchanges),
                ConfigReason.IN_FLIGHT_NOT_SUPPORTED, "REQ-API-01")

    def check_phy(self, c: TwrConfig) -> None:
        p = c.phy
        if p.channel > 16 or p.channel < 0:
            self._cfg_rej("phy.channel", ConfigReason.INDEX_OUT_OF_RANGE,
                          "channel must be 0..16")
        elif not self.caps.channel_supported(p.channel):
            self._unsup(
                "phy.channel",
                "channel %d is not in this build's capability whitelist; the "
                "phase-1 common profile is channel 5 (REQ-PHY-01/03) and other "
                "channels are per-channel acceptance items" % p.channel)

        # Channel vs centre frequency: a conflict is rejected, never ignored.
        self._finite("phy.center_frequency_hz", p.center_frequency_hz,
                     "REQ-PHY-01")
        if _is_finite(p.center_frequency_hz) and p.center_frequency_hz > 0.0:
            plan = uwb_channel_center_frequency_hz(p.channel)
            if plan <= 0.0:
                self._cfg_rej("phy.center_frequency_hz",
                              ConfigReason.INDEX_OUT_OF_RANGE,
                              "no channel plan entry for this channel")
            elif abs(p.center_frequency_hz - plan) > CHANNEL_FREQUENCY_TOLERANCE_HZ:
                self._cfg_rej(
                    "phy.center_frequency_hz",
                    ConfigReason.CHANNEL_FREQUENCY_MISMATCH,
                    "channel %d is %s Hz, config states %s Hz" % (
                        p.channel, double_to_text(plan),
                        double_to_text(p.center_frequency_hz)),
                    "REQ-PHY-01")

        # code / PRF / channel / profile are cross-validated together.
        for path, code in (("phy.tx_preamble_code", p.tx_preamble_code),
                           ("phy.rx_preamble_code", p.rx_preamble_code)):
            if code == 0:
                self._cfg_rej(path, ConfigReason.ZERO_VALUE,
                              "preamble code is required")
            elif not self.caps.code_index_supported(code):
                self._unsup(
                    path,
                    "preamble code %d is not in the capability whitelist "
                    "(%d..%d)" % (code, CODE_INDEX_MIN, CODE_INDEX_MAX))
        if (p.tx_preamble_code != 0 and p.rx_preamble_code != 0 and
                p.tx_preamble_code != p.rx_preamble_code):
            self._cfg_rej(
                "phy.rx_preamble_code", ConfigReason.FIELD_CONFLICT,
                "TX and RX preamble codes differ (%d vs %d); one profile uses "
                "one code" % (p.tx_preamble_code, p.rx_preamble_code),
                "REQ-PHY-01")
        if not self.caps.prf_class_supported(p.prf_class):
            self._unsup("phy.prf_class",
                        "PRF class %s is not in the capability whitelist"
                        % p.prf_class)
        # The PHR rate is validated on its OWN axis.  It is NOT compared to the
        # payload data rate: the PHR is a different modulation at a different
        # rate, and the data-rate field INSIDE the PHR describes the payload,
        # not the PHR's own transmission.  M0's `phr_rate == data_rate` rule
        # therefore accepted the 6.81 Mb/s PHR this modulator cannot produce
        # and rejected the 0.85 Mb/s PHR it does produce.
        if not phr_rate_is_implemented(p.phr_rate):
            self._unsup(
                "phy.phr_rate",
                "PHR rate %s is not implemented: %s It is NOT compared with "
                "phy.data_rate (%s): the PHR and the payload are separate "
                "modulations, and the data-rate field carried inside the PHR "
                "describes the payload"
                % (p.phr_rate, phr_rate_unsupported_reason(p.phr_rate),
                   p.data_rate),
                ConfigReason.UNSUPPORTED, "REQ-PHY-02")

        # data rate: only the one the modulator can produce is enabled.
        if not self.caps.data_rate_supported(p.data_rate):
            reason = DATA_RATE_REJECT_REASON
            if not self.caps.phy_matrix and self.caps.pending_reason:
                # No measured matrix at all: name the pending measurement too,
                # so the operator is told where the verdict will come from.
                reason += "; " + self.caps.pending_reason
            self._unsup(
                "phy.data_rate",
                "data rate %s is not in the capability whitelist: %s" % (
                    p.data_rate, reason),
                ConfigReason.UNSUPPORTED, "REQ-PHY-01")

        # preamble length (SYNC repetitions).
        if p.preamble_symbols == 0:
            self._cfg_rej("phy.preamble_symbols", ConfigReason.ZERO_VALUE,
                          "preamble length is required")
        else:
            # GATE 1: the length must be a member of PreambleLength, i.e. one
            # this repository's demodulator / CFO path can actually run.  The
            # reason names THAT limit and never a chip's capability: the
            # Qorvo parts' own API accepts 64 through 2048, so "the hardware
            # cannot do 128" would be a false statement about hardware this
            # project has never spoken to.
            if preamble_length_from_symbols(p.preamble_symbols) is None:
                self._unsup("phy.preamble_symbols",
                            preamble_length_unsupported_reason(
                                p.preamble_symbols),
                            ConfigReason.UNSUPPORTED, "REQ-PHY-01")
            # GATE 2: the length must also be MEASURED for this (native rate,
            # code, SFD, PSDU, ranging) combination.  Separate from gate 1 on
            # purpose: a length can be implemented and still unmeasured.
            if not self.caps.sync_repetitions_supported(p.preamble_symbols):
                # M0.1 R7: state the SCOPE of the refusal.  For the lengths
                # Qorvo parts do support, this build's decoder / CFO path is
                # the limit -- that is not a hardware claim, and the note
                # carries the vendor citation so the message and the CSV agree.
                note = (" [%s]" % preamble_length_reject_note(
                    p.preamble_symbols)
                    if preamble_length_supported_by_vendor(p.preamble_symbols)
                    else "")
                self._unsup(
                    "phy.preamble_symbols",
                    "preamble length %d is not in the capability whitelist: "
                    "%s%s" % (p.preamble_symbols,
                              self.caps.pending_reason
                              or self._sync_reason(p.preamble_symbols), note),
                    ConfigReason.UNSUPPORTED, "REQ-PHY-01")

    def _sync_reason(self, n: int) -> str:
        """Why a preamble length is not in the whitelist.

        Prefers the MEASURED cell the matrix recorded for that length, so the
        message is greppable against the CSV; falls back to the derived
        structural analysis for a length the matrix never swept, and to the
        pending-matrix text when even that has nothing to say.
        """
        for row in self.caps.phy_matrix_rejections:
            if row.sync_repetitions == n:
                return row.reason
        reasons = sync_repetition_reasons(n)
        if reasons:
            return "; ".join(reasons)
        return self.caps.pending_reason

    def check_frame(self, c: TwrConfig) -> None:
        f = c.frame

        # ---- which layout? ------------------------------------------------
        # The profile is an ENUM, so "an arbitrary non-empty profile string"
        # is not expressible.  An id this build has no implementation for is
        # refused with its name, never defaulted to frame v1.
        if not frame_profile_id_is_supported(f.frame_profile):
            self._unsup(
                "frame.frame_profile",
                "frame profile %s is not implemented: only the ids named by "
                "ALL_FRAME_PROFILE_IDS have a codec, a geometry authority and "
                "an encoder in this build. A free-form profile name is never "
                "an executable geometry" % f.frame_profile,
                ConfigReason.UNSUPPORTED, "REQ-PROTO-06")
        # The authority this config is checked against.  It comes from the
        # enum; it is never read out of the config.
        authority = frame_geometry_for(f.frame_profile)

        if not self.caps.sfd_mode_supported(f.sfd_mode):
            look = self.caps.lookup_phy(
                c.radio.native_sample_rate_hz, c.phy.tx_preamble_code,
                c.phy.preamble_symbols, f.sfd_mode, f.mac_psdu_bytes,
                f.ranging_bit)
            self._unsup("frame.sfd_mode",
                        "SFD mode %s is not in the capability whitelist: %s"
                        % (f.sfd_mode, look.reason),
                        ConfigReason.UNSUPPORTED, "REQ-PHY-01")
        # The raw SFD length is a RESTATEMENT of the mode, never an
        # independent number: "4z2" and 4 symbols cannot both be stated.
        if f.sfd_symbols == 0:
            self._cfg_rej("frame.sfd_symbols", ConfigReason.ZERO_VALUE,
                          "SFD length is required")
        elif f.sfd_symbols != sfd_mode_symbols(f.sfd_mode):
            self._cfg_rej(
                "frame.sfd_symbols", ConfigReason.FIELD_CONFLICT,
                "SFD mode %s is %d symbols, config states %d; the length is "
                "derived from the mode, not chosen beside it" % (
                    f.sfd_mode, sfd_mode_symbols(f.sfd_mode), f.sfd_symbols),
                "REQ-PHY-01")

        # PHR presence/form.  Each unimplemented member carries its own reason
        # (phr_mode_unsupported_reason); M0 caught only `none` and let
        # `extended` through as a valid profile.
        if f.phr_mode is not PhrMode.STANDARD:
            self._unsup(
                "frame.phr_mode",
                "PHR mode %s is not implemented: %s"
                % (f.phr_mode, phr_mode_unsupported_reason(f.phr_mode)),
                ConfigReason.OUT_OF_SCOPE if f.phr_mode is PhrMode.NONE
                else ConfigReason.UNSUPPORTED,
                "REQ-PHY-01")
        if self.caps.ranging_bit_required and not f.ranging_bit:
            self._cfg_rej(
                "frame.ranging_bit", ConfigReason.FIELD_CONFLICT,
                "a TWR MAC frame must set the ranging bit; ranging_bit=false is "
                "not a valid TWR profile", "REQ-PROTO-05")
        if not self.caps.ranging_bit_required and f.ranging_bit:
            self._cfg_rej("frame.ranging_bit", ConfigReason.FIELD_CONFLICT,
                          "this build's whitelist carries no ranging frame",
                          "REQ-PROTO-05")

        self._finite_timed("frame.sfd_timeout", f.sfd_timeout)
        self._validate_timed_field("frame.sfd_timeout", f.sfd_timeout, False,
                                   TimeReferenceEvent.RX_ENABLE, True)

        # Exactly one layer appends the FCS, and WHICH one is not free.
        # A config that names the MAC as the appender is DESCRIBABLE -- the
        # geometry authority has a variant for it and the on-air length does
        # not change -- but it is not ENCODABLE here.  M0 accepted such a
        # configuration and then reported a "MAC PSDU" no encoder can produce,
        # which is the R3 class of defect one layer down.
        if f.fcs_append == FcsAppender.NONE:
            self._cfg_rej("frame.fcs_append", ConfigReason.FIELD_CONFLICT,
                          "no layer appends the FCS; exactly one of mac/phy must")
        elif f.fcs_append is FcsAppender.MAC and authority is not None:
            self._unsup(
                "frame.fcs_append",
                "fcs_append=mac asks for a MAC that builds and appends the FCS "
                "itself. The geometry is DESCRIBABLE and the on-air length is "
                "unchanged, but this codec only encodes the PHY-appends form: "
                "FrameProfileGeometry.executable() is false for the MAC-appends "
                "variant",
                ConfigReason.UNSUPPORTED, "REQ-API-01")
        if f.fcs_bytes == 0:
            self._cfg_rej("frame.fcs_bytes", ConfigReason.ZERO_VALUE,
                          "FCS length is required")
        elif authority is not None and f.fcs_bytes != authority.fcs_bytes_on_air():
            self._unsup(
                "frame.fcs_bytes",
                "frame profile %s has a %d-byte FCS (IEEE 802.15.4), the config "
                "states %d bytes" % (f.frame_profile,
                                     authority.fcs_bytes_on_air(),
                                     f.fcs_bytes),
                ConfigReason.UNSUPPORTED, "REQ-API-01")
        declared_includes = (f.fcs_append is FcsAppender.MAC)
        if f.mac_psdu_includes_fcs != declared_includes:
            self._cfg_rej(
                "frame.mac_psdu_includes_fcs", ConfigReason.FIELD_CONFLICT,
                "fcs_append=%s but mac_psdu_includes_fcs=%s; exactly one layer "
                "appends the FCS" % (f.fcs_append,
                                     bool_to_text(f.mac_psdu_includes_fcs)),
                "REQ-API-01")

        # ---- the geometry CLAIM, field by field, against the codec -------
        if not f.geometry.any_set():
            self._cfg_rej("frame.geometry", ConfigReason.EMPTY_VALUE,
                          "the frame profile geometry must be stated "
                          "explicitly; it is never assumed. Fill it with "
                          "frame_geometry_of_profile(frame.frame_profile) so "
                          "the claim comes from the codec rather than from a "
                          "literal")
        elif authority is not None:
            # A claim that disagrees with the codec is a REJECTION PER FIELD,
            # not a silently-corrected value: an operator who believes the
            # header is 7 bytes must be told it is 14, not handed a working
            # config that says something else.
            #
            # The authority the claim is checked against follows the FCS owner
            # the config NAMES, not the one it should name: a MAC-appends
            # layout is described by frame_geometry_mac_appends_fcs(), which
            # expects the 2 FCS bytes inside the MAC PSDU.  That way a
            # MAC-appends claim that forgets to reserve them is caught as the
            # field disagreement it is, and the separate `fcs_append`
            # rejection above says the layout is not encodable here at all.
            named_authority = (
                frame_geometry_mac_appends_fcs(f.frame_profile)
                if f.fcs_append is FcsAppender.MAC else authority)
            geo = frame_geometry_check(frame_geometry_claim(f.geometry),
                                       named_authority)
            for m in geo.mismatches:
                self._cfg_rej(
                    "frame.geometry.%s" % m.field, ConfigReason.FIELD_CONFLICT,
                    "the frame codec is the geometry authority: profile %s "
                    "with FCS appended by %s has %s = %d, the config claims "
                    "%d. A claim does not decide the layout: fill the geometry "
                    "from frame_geometry_of_profile() so there is one source"
                    % (f.frame_profile, named_authority.fcs_owner, m.field,
                       m.expected, m.actual),
                    "REQ-PROTO-06")

        # STS: out of scope for phase 1, explicitly.
        if f.sts_mode != StsMode.OFF:
            self._unsup("frame.sts_mode",
                        "STS mode %s: %s" % (f.sts_mode,
                                             self.caps.unsupported_sts_reason),
                        ConfigReason.OUT_OF_SCOPE, "REQ-SCOPE-04")
        if f.sts_length_symbols != 0:
            self._cfg_rej("frame.sts_length_symbols",
                          ConfigReason.FIELD_CONFLICT,
                          "sts_mode is off but an STS length is requested")

    def check_tx(self, c: TwrConfig) -> None:
        t = c.tx
        if t.port < 0 or t.port >= MAX_PHYSICAL_CHANNELS:
            self._cfg_rej("tx.port", ConfigReason.INDEX_OUT_OF_RANGE,
                          "physical TX channel index too large")
        self._finite_opt("tx.gain_db", t.gain_db)
        self._finite_opt("tx.iq_amplitude", t.iq_amplitude)
        self._finite_opt("tx.calibrated_tx_power_dbm", t.calibrated_tx_power_dbm)

        if t.gain_db is not None and (t.gain_db < 0.0 or t.gain_db > 120.0):
            self._cfg_rej("tx.gain_db", ConfigReason.OUT_OF_RANGE,
                          "TX gain must be within 0..120 dB")
        if t.iq_amplitude is not None and (t.iq_amplitude <= 0.0 or
                                           t.iq_amplitude > 1.0):
            self._cfg_rej("tx.iq_amplitude", ConfigReason.OUT_OF_RANGE,
                          "digital IQ amplitude must be within (0, 1] for a "
                          "full-scale SC16 stream")
        if t.calibrated_tx_power_dbm is not None and (
                t.calibrated_tx_power_dbm < -30.0 or
                t.calibrated_tx_power_dbm > 30.0):
            self._cfg_rej("tx.calibrated_tx_power_dbm", ConfigReason.OUT_OF_RANGE,
                          "calibrated antenna-plane power must be within "
                          "-30..30 dBm")

        # Exactly one power authority, and it must be the field that is set.
        present = sum(1 for v in (t.gain_db, t.iq_amplitude,
                                  t.calibrated_tx_power_dbm) if v is not None)
        authority_present = (
            t.power_policy == TxPowerPolicy.LEAVE_UNTOUCHED or
            (t.power_policy == TxPowerPolicy.MANUAL_GAIN_DB and
             t.gain_db is not None) or
            (t.power_policy == TxPowerPolicy.IQ_AMPLITUDE and
             t.iq_amplitude is not None) or
            (t.power_policy == TxPowerPolicy.CALIBRATED_DBM and
             t.calibrated_tx_power_dbm is not None))
        if not authority_present:
            self._cfg_rej(
                "tx.power_policy", ConfigReason.FIELD_CONFLICT,
                "power_policy=%s but the field it names is not set (gain dB, "
                "IQ amplitude and calibrated dBm are three separate fields)"
                % t.power_policy)
        if t.power_policy == TxPowerPolicy.LEAVE_UNTOUCHED and present != 0:
            self._cfg_rej("tx.power_policy", ConfigReason.FIELD_CONFLICT,
                          "power_policy is leave_untouched but %d power field(s) "
                          "are set" % present)
        if (t.power_policy == TxPowerPolicy.CALIBRATED_DBM and
                t.calibrated_tx_power_dbm is not None and t.gain_db is None):
            self._cfg_rej("tx.gain_db", ConfigReason.FIELD_CONFLICT,
                          "a calibrated dBm is only meaningful together with "
                          "the gain it was measured at")

        # A vendor power word is a fourth, separately named field.
        if t.vendor_power_word is not None:
            if t.vendor_power_word_backend == "":
                self._cfg_rej(
                    "tx.vendor_power_word", ConfigReason.CALIBRATION_MISMATCH,
                    "a vendor power word is meaningless without the backend "
                    "capability id that defines its bits")
        elif t.vendor_power_word_backend != "":
            self._cfg_rej("tx.vendor_power_word_backend",
                          ConfigReason.FIELD_CONFLICT,
                          "a backend capability id is set but no vendor power "
                          "word is")

    def check_rx(self, c: TwrConfig) -> None:
        r = c.rx
        if r.port < 0 or r.port >= MAX_PHYSICAL_CHANNELS:
            self._cfg_rej("rx.port", ConfigReason.INDEX_OUT_OF_RANGE,
                          "physical RX channel index too large")
        self._finite_opt("rx.gain_db", r.gain_db)
        self._finite_opt("rx.bandwidth_hz", r.bandwidth_hz)
        self._finite("rx.detection_threshold", r.detection_threshold)
        self._finite("rx.correlation_threshold", r.correlation_threshold)
        self._finite("rx.first_path_threshold", r.first_path_threshold)
        self._finite_opt("rx.vendor_pac_applied_step", r.vendor_pac_applied_step)

        if r.gain_db is not None and (r.gain_db < 0.0 or r.gain_db > 120.0):
            self._cfg_rej("rx.gain_db", ConfigReason.OUT_OF_RANGE,
                          "RX gain must be within 0..120 dB")
        if r.agc == AgcMode.MANUAL and r.gain_db is None:
            self._cfg_rej("rx.gain_db", ConfigReason.FIELD_CONFLICT,
                          "agc=manual needs an explicit RX gain in dB")
        if r.bandwidth_hz is not None and r.bandwidth_hz <= 0.0:
            self._cfg_rej("rx.bandwidth_hz", ConfigReason.OUT_OF_RANGE,
                          "RF bandwidth must be > 0; it is a different quantity "
                          "from the sample rate (REQ-PHY-02)")
        if r.bandwidth_hz is not None and r.bandwidth_hz > 1.0e9:
            self._cfg_rej("rx.bandwidth_hz", ConfigReason.OUT_OF_RANGE,
                          "RF bandwidth is implausible")
        for path, v in (("rx.detection_threshold", r.detection_threshold),
                        ("rx.correlation_threshold", r.correlation_threshold),
                        ("rx.first_path_threshold", r.first_path_threshold)):
            if v < 0.0 or v > 1.0:
                self._cfg_rej(path, ConfigReason.OUT_OF_RANGE,
                              "must be within 0..1")
        if r.correlation_threshold < r.detection_threshold:
            self._cfg_rej(
                "rx.correlation_threshold", ConfigReason.FIELD_CONFLICT,
                "the fine correlation gate must not be looser than the coarse "
                "detection gate")
        if r.first_path_window == 0:
            self._cfg_rej("rx.first_path_window", ConfigReason.ZERO_VALUE,
                          "the first-path search window must be at least one tap")
        elif r.first_path_index >= r.first_path_window:
            self._cfg_rej("rx.first_path_index", ConfigReason.INDEX_OUT_OF_RANGE,
                          "first usable CIR index lies outside the search window")

        if r.vendor_pac_value is not None:
            if r.vendor_pac_backend == "":
                self._cfg_rej(
                    "rx.vendor_pac_value", ConfigReason.CALIBRATION_MISMATCH,
                    "a vendor PAC value is meaningless without the backend "
                    "capability id that defines it")
            if r.vendor_pac_applied_step is None:
                self._cfg_rej(
                    "rx.vendor_pac_applied_step",
                    ConfigReason.CALIBRATION_MISMATCH,
                    "a vendor PAC value may not be mapped onto an arbitrary "
                    "software detection step; the backend must state the step "
                    "it applies")
            elif r.vendor_pac_applied_step <= 0.0:
                self._cfg_rej("rx.vendor_pac_applied_step",
                              ConfigReason.OUT_OF_RANGE,
                              "the applied software step must be > 0")
        else:
            if r.vendor_pac_backend != "":
                self._cfg_rej("rx.vendor_pac_backend", ConfigReason.FIELD_CONFLICT,
                              "a backend capability id is set but no vendor PAC "
                              "value is")
            if r.vendor_pac_applied_step is not None:
                self._cfg_rej("rx.vendor_pac_applied_step",
                              ConfigReason.FIELD_CONFLICT,
                              "a software step is declared without a vendor PAC "
                              "value")

    def check_radio(self, c: TwrConfig) -> None:
        r = c.radio
        if r.tx_channel < 0 or r.tx_channel >= MAX_PHYSICAL_CHANNELS:
            self._cfg_rej("radio.tx_channel", ConfigReason.INDEX_OUT_OF_RANGE,
                          "TX channel index too large")
        if r.rx_channel < 0 or r.rx_channel >= MAX_PHYSICAL_CHANNELS:
            self._cfg_rej("radio.rx_channel", ConfigReason.INDEX_OUT_OF_RANGE,
                          "RX channel index too large")
        self._finite("radio.native_sample_rate_hz", r.native_sample_rate_hz,
                     "REQ-PHY-02")
        if (_is_finite(r.native_sample_rate_hz) and
                r.native_sample_rate_hz > 0.0 and
                not self.caps.native_rate_supported(r.native_sample_rate_hz)):
            self._unsup(
                "radio.native_sample_rate_hz",
                "native sample rate %s is not supported; this build accepts "
                "737280000 or 491520000" % double_to_text(
                    r.native_sample_rate_hz),
                ConfigReason.UNSUPPORTED, "REQ-PHY-02")

        if len(r.peers) > MAX_PEERS_PER_CONFIG:
            self._cfg_rej("radio.peers", ConfigReason.OVER_CAPACITY,
                          "at most %d peer endpoints per config"
                          % MAX_PEERS_PER_CONFIG)

        # Resource conflicts on the same physical channel.
        for i, p in enumerate(r.peers):
            pf = "radio.peers[%d]" % i
            if p.id == "":
                self._cfg_rej(pf + ".id", ConfigReason.EMPTY_VALUE,
                              "endpoint id is required")
            if p.tx_channel < 0 or p.tx_channel >= MAX_PHYSICAL_CHANNELS:
                self._cfg_rej(pf + ".tx_channel", ConfigReason.INDEX_OUT_OF_RANGE,
                              "TX channel too large")
            if p.rx_channel < 0 or p.rx_channel >= MAX_PHYSICAL_CHANNELS:
                self._cfg_rej(pf + ".rx_channel", ConfigReason.INDEX_OUT_OF_RANGE,
                              "RX channel too large")
            self._finite(pf + ".native_sample_rate_hz", p.native_sample_rate_hz)
            if not p.occupies_resources:
                continue
            if p.tx_channel == r.tx_channel:
                self._cfg_rej(
                    pf + ".tx_channel", ConfigReason.DUPLICATE_RESOURCE,
                    "physical TX channel %d is already claimed by this endpoint; "
                    "one device resource, one owner" % p.tx_channel,
                    "REQ-BASE-01")
            if p.rx_channel == r.rx_channel:
                self._cfg_rej(
                    pf + ".rx_channel", ConfigReason.DUPLICATE_RESOURCE,
                    "physical RX channel %d is already claimed by this endpoint; "
                    "one device resource, one owner" % p.rx_channel,
                    "REQ-BASE-01")
            if p.role == c.session.role:
                self._cfg_rej(
                    pf + ".role", ConfigReason.FIELD_CONFLICT,
                    "both endpoints claim role %s; a two-endpoint session needs "
                    "one initiator and one responder" % p.role, "REQ-PROTO-01")
            if (p.native_sample_rate_hz > 0.0 and
                    r.native_sample_rate_hz > 0.0 and
                    not rate_matches_strict(p.native_sample_rate_hz,
                                            r.native_sample_rate_hz)):
                self._cfg_rej(
                    pf + ".native_sample_rate_hz", ConfigReason.FIELD_CONFLICT,
                    "endpoints sharing one device must share the native sample "
                    "rate (%s vs %s)" % (
                        double_to_text(r.native_sample_rate_hz),
                        double_to_text(p.native_sample_rate_hz)),
                    "REQ-PHY-02")
            for j in range(i):
                q = r.peers[j]
                if not q.occupies_resources:
                    continue
                if q.id == p.id:
                    self._cfg_rej(pf + ".id", ConfigReason.DUPLICATE_RESOURCE,
                                  "duplicate endpoint id")
                if q.tx_channel == p.tx_channel:
                    self._cfg_rej(
                        pf + ".tx_channel", ConfigReason.DUPLICATE_RESOURCE,
                        "physical TX channel %d claimed by two endpoints"
                        % p.tx_channel, "REQ-BASE-01")
                if q.rx_channel == p.rx_channel:
                    self._cfg_rej(
                        pf + ".rx_channel", ConfigReason.DUPLICATE_RESOURCE,
                        "physical RX channel %d claimed by two endpoints"
                        % p.rx_channel, "REQ-BASE-01")
                if q.role == p.role:
                    self._cfg_rej(
                        pf + ".role", ConfigReason.FIELD_CONFLICT,
                        "two peers claim the same role %s" % p.role,
                        "REQ-PROTO-01")

        # Readback: a mismatch is a STARTUP FAILURE, never a soft warning.
        if r.require_readback and not r.readback.present:
            self._rej("radio.readback.present", ConfigReason.CALIBRATION_MISSING,
                      ExchangeStatus.CALIBRATION_MISSING,
                      "require_readback is set but no hardware readback was "
                      "recorded", "REQ-PHY-02")
        if r.readback.present:
            self._finite("radio.readback.sample_rate_hz",
                         r.readback.sample_rate_hz, "REQ-PHY-02")
            self._finite("radio.readback.center_freq_hz",
                         r.readback.center_freq_hz, "REQ-PHY-01")
            if not self.caps.native_rate_supported(r.readback.sample_rate_hz):
                self._cfg_rej(
                    "radio.readback.sample_rate_hz", ConfigReason.OUT_OF_RANGE,
                    "readback native rate %s Hz is not a supported rate"
                    % double_to_text(r.readback.sample_rate_hz))
            elif (r.native_sample_rate_hz > 0.0 and
                  not rate_matches_strict(r.native_sample_rate_hz,
                                          r.readback.sample_rate_hz)):
                self._cfg_rej(
                    "radio.readback.sample_rate_hz", ConfigReason.FIELD_CONFLICT,
                    "readback native rate %s Hz differs from the requested %s Hz"
                    % (double_to_text(r.readback.sample_rate_hz),
                       double_to_text(r.native_sample_rate_hz)), "REQ-PHY-02")
            if r.readback.tx_channel != r.tx_channel:
                self._cfg_rej("radio.readback.tx_channel",
                              ConfigReason.FIELD_CONFLICT,
                              "readback TX channel differs from the requested one",
                              "REQ-PHY-02")
            if r.readback.rx_channel != r.rx_channel:
                self._cfg_rej("radio.readback.rx_channel",
                              ConfigReason.FIELD_CONFLICT,
                              "readback RX channel differs from the requested one",
                              "REQ-PHY-02")
            if (c.phy.center_frequency_hz > 0.0 and
                    r.readback.center_freq_hz > 0.0 and
                    abs(c.phy.center_frequency_hz - r.readback.center_freq_hz) >
                    CHANNEL_FREQUENCY_TOLERANCE_HZ):
                self._cfg_rej(
                    "radio.readback.center_freq_hz",
                    ConfigReason.CHANNEL_FREQUENCY_MISMATCH,
                    "readback centre frequency %s Hz differs from the requested "
                    "%s Hz" % (double_to_text(r.readback.center_freq_hz),
                               double_to_text(c.phy.center_frequency_hz)),
                    "REQ-PHY-01")
            if (r.clock_source != "" and r.readback.clock_source != "" and
                    r.clock_source != r.readback.clock_source):
                self._cfg_rej("radio.readback.clock_source",
                              ConfigReason.FIELD_CONFLICT,
                              "readback clock source differs from the requested one",
                              "REQ-PHY-02")
            if (r.time_source != "" and r.readback.time_source != "" and
                    r.time_source != r.readback.time_source):
                self._cfg_rej("radio.readback.time_source",
                              ConfigReason.FIELD_CONFLICT,
                              "readback time source differs from the requested one",
                              "REQ-PHY-02")

    # -- the generic timed-field contract --------------------------------
    def _validate_timed_field(self, f: str, t: TimedField, required: bool,
                              expect_ref: TimeReferenceEvent,
                              require_marker: bool) -> None:
        if t.domain == TimeDomain.UNSPECIFIED:
            self._cfg_rej(f + ".domain", ConfigReason.EMPTY_VALUE,
                          "the clock domain must be stated (device_ticks or "
                          "monotonic_host)", "REQ-API-02")
            return
        if t.value.negative():
            self._cfg_rej(f + ".ns", ConfigReason.NEGATIVE_VALUE,
                          "may not be negative", "REQ-API-02")
        if required and t.value.is_zero():
            self._cfg_rej(f + ".ns", ConfigReason.ZERO_VALUE, "must be > 0",
                          "REQ-API-02")
        if (expect_ref != TimeReferenceEvent.HOST_MONOTONIC and
                t.reference != expect_ref):
            self._cfg_rej(f + ".reference", ConfigReason.FIELD_CONFLICT,
                          "must be %s, is %s" % (expect_ref, t.reference),
                          "REQ-API-02")
        if t.domain == TimeDomain.DEVICE_TICKS:
            if require_marker and t.marker is None:
                self._cfg_rej(
                    f + ".marker", ConfigReason.EMPTY_VALUE,
                    "a device-tick field must name the RF marker it is measured "
                    "at (REQ-TIME-02)", "REQ-API-02")
            if (t.marker is not None and
                    t.marker == TimestampMarker.UHD_RX_FIRST_IQ_SAMPLE and
                    expect_ref == TimeReferenceEvent.FRAME_TAIL):
                self._cfg_rej(f + ".marker", ConfigReason.FIELD_CONFLICT,
                              "the first RX IQ sample is not the end of a frame",
                              "REQ-TIME-02")
        else:  # MonotonicHost
            if t.marker is not None:
                self._cfg_rej(
                    f + ".marker", ConfigReason.FIELD_CONFLICT,
                    "a host monotonic field has no RF marker; a marker here would "
                    "invite a cross-domain subtraction", "REQ-TIME-01")
            if t.required_quantisation_hz != 1.0e9:
                self._cfg_rej(
                    f + ".quantisation_hz", ConfigReason.QUANTISATION_UNSUPPORTED,
                    "a host monotonic field must be quantised at 1e9 (integer "
                    "nanoseconds)", "REQ-TIME-01")
        if (not _is_finite(t.required_quantisation_hz) or
                t.required_quantisation_hz <= 0.0):
            self._cfg_rej(f + ".quantisation_hz",
                          ConfigReason.QUANTISATION_UNSUPPORTED,
                          "the required quantisation rate must be finite and > 0")
            return
        if not self.caps.quantisation_supported(t.required_quantisation_hz):
            self._cfg_rej(
                f + ".quantisation_hz", ConfigReason.QUANTISATION_UNSUPPORTED,
                "quantisation %s Hz is not a rate this build can express "
                "(allowed: 1e9 ns, %s work rate, the native device rates)" % (
                    double_to_text(t.required_quantisation_hz),
                    double_to_text(self.caps.work_sample_rate_hz)))
            return
        if t.max_quantisation_error_ns < 0:
            self._cfg_rej(f + ".max_quantisation_error_ns",
                          ConfigReason.OUT_OF_RANGE,
                          "the tolerated quantisation error may not be negative")
        # The precision gate.  A 1 ns budget is exactly what 1 GHz, the
        # 998.4 MS/s work rate and 737.28 MS/s offer; the 491.52 MS/s CG400 tick
        # is 2.0345 ns, so a 1 ns budget is INSUFFICIENT there and 2 (or 3) is
        # required -- the config is rejected rather than silently rounded.
        if not t.value.representable_at(t.required_quantisation_hz,
                                        t.max_quantisation_error_ns):
            self._cfg_rej(
                f + ".ns", ConfigReason.TIMING_PRECISION_INSUFFICIENT,
                "a device tick at %s Hz (%.4f ns period) cannot express this "
                "value with <= %d ns of error" % (
                    double_to_text(t.required_quantisation_hz),
                    1.0e9 / t.required_quantisation_hz,
                    t.max_quantisation_error_ns),
                "REQ-API-01")
        ticks, back = quantise_duration(t.value, t.required_quantisation_hz)
        if ticks is None or back is None:
            self._cfg_rej(
                f + ".ns", ConfigReason.TIMING_PRECISION_INSUFFICIENT,
                "the value does not fit the device tick range at %s Hz"
                % double_to_text(t.required_quantisation_hz), "REQ-API-01")

    def check_timing(self, c: TwrConfig) -> None:
        t = c.timing
        ss = c.session.protocol == Protocol.SS
        initiator = c.session.role == Role.INITIATOR

        for name in ("poll_start", "poll_to_response", "response_to_final",
                     "final_to_report", "post_tx_rx_enable", "min_tx_lead_time"):
            self._finite_timed("timing." + name, getattr(t, name))

        # poll_start: absolute, on the device clock, at the Poll TX RMARKER.
        # Only the initiator transmits a Poll, so a responder's poll_start must
        # be zero rather than silently reusing the initiator's value.
        self._validate_timed_field("timing.poll_start", t.poll_start, initiator,
                                   TimeReferenceEvent.POLL_TX_RMARKER, True)
        if not initiator and not t.poll_start.value.is_zero():
            self._cfg_rej(
                "timing.poll_start.ns", ConfigReason.FIELD_CONFLICT,
                "the responder transmits no Poll, so the Poll start time must be "
                "zero", "REQ-API-02")

        # The three distinct quantities.
        if initiator:
            if not t.poll_to_response.value.is_zero():
                self._cfg_rej(
                    "timing.poll_to_response", ConfigReason.FIELD_CONFLICT,
                    "the initiator never sends a Response, so this reply delay "
                    "must be zero", "REQ-API-02")
            if (t.poll_to_response.reference !=
                    TimeReferenceEvent.POLL_RX_RMARKER):
                self._cfg_rej("timing.poll_to_response.reference",
                              ConfigReason.FIELD_CONFLICT,
                              "must be poll_rx_rmarker", "REQ-API-02")
            if ss:
                if not t.response_to_final.value.is_zero():
                    self._cfg_rej(
                        "timing.response_to_final", ConfigReason.FIELD_CONFLICT,
                        "SS-TWR has no Final message, so this delay must be zero",
                        "REQ-API-02")
            else:
                self._validate_timed_field(
                    "timing.response_to_final", t.response_to_final, True,
                    TimeReferenceEvent.RESPONSE_RX_RMARKER, True)
        else:
            if not t.response_to_final.value.is_zero():
                self._cfg_rej(
                    "timing.response_to_final", ConfigReason.FIELD_CONFLICT,
                    "the responder never sends a Final, so this delay must be "
                    "zero", "REQ-API-02")
            if t.response_to_final.reference != TimeReferenceEvent.FINAL_TX_RMARKER:
                self._cfg_rej("timing.response_to_final.reference",
                              ConfigReason.FIELD_CONFLICT,
                              "must be final_tx_rmarker", "REQ-API-02")
            self._validate_timed_field(
                "timing.poll_to_response", t.poll_to_response, True,
                TimeReferenceEvent.POLL_RX_RMARKER, True)

        # post-TX RX enable: a device-tick delay from the end of OUR OWN frame.
        self._validate_timed_field(
            "timing.post_tx_rx_enable", t.post_tx_rx_enable, True,
            TimeReferenceEvent.FRAME_TAIL, True)
        self._validate_timed_field(
            "timing.min_tx_lead_time", t.min_tx_lead_time, True,
            TimeReferenceEvent.RESPONSE_TX_RMARKER, True)

        # Report is reserved in phase 1.
        if not t.final_to_report.value.is_zero():
            self._unsup(
                "timing.final_to_report",
                "FrameType::REPORT is reserved in phase 1; a Report message "
                "would invalidate the three-message interop claim (REQ-PROTO-05)",
                ConfigReason.OUT_OF_SCOPE, "REQ-PROTO-05")

        # REQ-GR-04 feasibility: the reply delay must cover the device lead
        # time.  There is no default lead time -- the operator must supply the
        # measured value, and a delay shorter than it is rejected instead of
        # being sent late.
        reply = t.response_to_final if initiator else t.poll_to_response
        if (reply.value.negative() or t.min_tx_lead_time.value.negative() or
                reply.value.is_zero()):
            return  # the structural violation was already reported above
        if reply.value > t.min_tx_lead_time.value:
            return  # the budget is feasible
        self._rej(
            "timing.response_to_final.ns" if initiator else
            "timing.poll_to_response.ns",
            ConfigReason.TIMING_BUDGET_INFEASIBLE, ExchangeStatus.DEADLINE_MISSED,
            "reply delay %s does not cover the measured minimum TX lead time "
            "%s (a scheduled TX whose deadline cannot be met must fail, not be "
            "sent late)" % (duration_to_text(reply.value),
                            duration_to_text(t.min_tx_lead_time.value)),
            "REQ-GR-04")

    def check_timeouts(self, c: TwrConfig) -> None:
        t = c.timeouts
        ss = c.session.protocol == Protocol.SS
        initiator = c.session.role == Role.INITIATOR

        for name in ("poll_rx_window", "response_rx_window", "final_rx_window",
                     "report_rx_window", "rx_timeout", "exchange_timeout",
                     "retry_interval"):
            self._finite_timed("timeouts." + name, getattr(t, name))

        # The RX timeout is measured FROM RX ENABLE, on the device clock.
        self._validate_timed_field("timeouts.rx_timeout", t.rx_timeout, True,
                                   TimeReferenceEvent.RX_ENABLE, True)

        # The whole-exchange deadline and the retry gap live on the host
        # monotonic clock and must carry NO RF marker (REQ-TIME-01).
        self._validate_timed_field("timeouts.exchange_timeout",
                                   t.exchange_timeout, True,
                                   TimeReferenceEvent.HOST_MONOTONIC, False)
        self._validate_timed_field("timeouts.retry_interval", t.retry_interval,
                                   False, TimeReferenceEvent.HOST_MONOTONIC,
                                   False)

        # Per-frame RX windows, armed at RX enable.  Only the frames this role
        # actually waits for may be non-zero.
        self._validate_timed_field("timeouts.poll_rx_window", t.poll_rx_window,
                                   not initiator, TimeReferenceEvent.RX_ENABLE,
                                   True)
        self._validate_timed_field("timeouts.response_rx_window",
                                   t.response_rx_window, initiator,
                                   TimeReferenceEvent.RX_ENABLE, True)
        if initiator and not t.poll_rx_window.value.is_zero():
            self._cfg_rej(
                "timeouts.poll_rx_window.ns", ConfigReason.FIELD_CONFLICT,
                "the initiator never receives a Poll, so its Poll RX window must "
                "be zero", "REQ-API-02")
        if not initiator and not t.response_rx_window.value.is_zero():
            self._cfg_rej(
                "timeouts.response_rx_window.ns", ConfigReason.FIELD_CONFLICT,
                "the responder never receives a Response, so its Response RX "
                "window must be zero", "REQ-API-02")
        if ss:
            if not t.final_rx_window.value.is_zero():
                self._cfg_rej(
                    "timeouts.final_rx_window.ns", ConfigReason.FIELD_CONFLICT,
                    "SS-TWR has no Final message, so the Final RX window must be "
                    "zero", "REQ-API-02")
        else:
            self._validate_timed_field("timeouts.final_rx_window",
                                       t.final_rx_window, not initiator,
                                       TimeReferenceEvent.RX_ENABLE, True)
        if not t.report_rx_window.value.is_zero():
            self._unsup("timeouts.report_rx_window",
                        "FrameType::REPORT is reserved in phase 1 (REQ-PROTO-05)",
                        ConfigReason.OUT_OF_SCOPE, "REQ-PROTO-05")

        # Every active window must fit inside the RX timeout measured from the
        # same RX enable event, otherwise the timeout fires before the window
        # closes.
        for path, f in (("timeouts.poll_rx_window", t.poll_rx_window),
                        ("timeouts.response_rx_window", t.response_rx_window),
                        ("timeouts.final_rx_window", t.final_rx_window)):
            if f.value.is_zero():
                continue
            if f.value > t.rx_timeout.value:
                self._cfg_rej(
                    path + ".ns", ConfigReason.TIMING_ORDER_VIOLATION,
                    "the RX window %s is longer than the RX timeout %s measured "
                    "from the same RX enable" % (duration_to_text(f.value),
                                                 duration_to_text(
                                                     t.rx_timeout.value)),
                    "REQ-API-02")

        # The whole-exchange deadline must cover the chain this endpoint runs.
        chain_ns = t.rx_timeout.value.nanos()
        if not initiator:
            chain_ns += c.timing.poll_to_response.value.nanos()
        if not ss and initiator:
            chain_ns += c.timing.response_to_final.value.nanos()
        if chain_ns > 0 and t.exchange_timeout.value.nanos() < chain_ns:
            self._cfg_rej(
                "timeouts.exchange_timeout.ns", ConfigReason.TIMING_ORDER_VIOLATION,
                "the whole-exchange deadline %s is shorter than the chain this "
                "endpoint must run (%s)" % (
                    duration_to_text(t.exchange_timeout.value),
                    duration_to_text(Duration(chain_ns))),
                "REQ-API-02")

        # Consecutive measurements may not overlap: at most one exchange is in
        # flight, so the interval must be at least one whole exchange.
        if (c.session.measurement_count > 1 and
                c.session.measurement_interval.nanos() <
                t.exchange_timeout.value.nanos()):
            self._cfg_rej(
                "session.measurement_interval.ns",
                ConfigReason.TIMING_ORDER_VIOLATION,
                "at most one exchange is in flight, so the measurement interval "
                "must be at least one whole-exchange timeout", "REQ-API-01")

    def check_calibration(self, c: TwrConfig) -> None:
        k = c.calibration
        if (k.tx_link_delay.negative() or k.rx_link_delay.negative() or
                k.antenna_delay.negative() or k.cable_delay.negative()):
            self._cfg_rej("calibration.tx_link_delay", ConfigReason.NEGATIVE_VALUE,
                          "a link/antenna/cable delay may not be negative",
                          "REQ-CAL-01")
        self._finite("calibration.native_sample_rate_hz",
                     k.native_sample_rate_hz, "REQ-PHY-02")
        self._finite_opt("calibration.record.gain_db", k.record.gain_db)
        if (k.native_sample_rate_hz > 0.0 and
                not is_allowed_native_rate(k.native_sample_rate_hz)):
            self._cfg_rej(
                "calibration.native_sample_rate_hz", ConfigReason.OUT_OF_RANGE,
                "calibration is versioned by sample rate; this build only "
                "calibrates at 737280000 or 491520000", "REQ-CAL-01")
        if k.tx_link_delay_native_ticks < 0 or k.rx_link_delay_native_ticks < 0:
            self._cfg_rej("calibration.tx_link_delay_native_ticks",
                          ConfigReason.NEGATIVE_VALUE,
                          "a native-tick delay may not be negative", "REQ-CAL-01")
        if k.link_delay_unit == TimeUnit.SECONDS:
            self._cfg_rej(
                "calibration.link_delay_unit", ConfigReason.UNIT_MISMATCH,
                "the delay fields are integer nanoseconds; use "
                "link_delay_unit=ns or native_ticks and state the seconds value "
                "elsewhere", "REQ-TIME-01")
        if k.link_delay_unit == TimeUnit.NATIVE_TICKS and \
                k.native_sample_rate_hz <= 0.0:
            self._cfg_rej("calibration.native_sample_rate_hz",
                          ConfigReason.UNIT_MISMATCH,
                          "native_ticks needs the native sample rate it refers to",
                          "REQ-TIME-01")
        if (k.link_delay_unit == TimeUnit.NATIVE_TICKS and
                k.tx_link_delay_native_ticks == 0 and
                k.rx_link_delay_native_ticks == 0):
            self._cfg_rej("calibration.link_delay_unit",
                          ConfigReason.FIELD_CONFLICT,
                          "link_delay_unit=native_ticks but no native-tick delay "
                          "is stated", "REQ-TIME-01")

        if k.applied_count != 0:
            self._rej(
                "calibration.applied_count",
                ConfigReason.CALIBRATION_ALREADY_APPLIED,
                ExchangeStatus.CALIBRATION_MISSING,
                "a calibration is applicable exactly once; this one has already "
                "been applied %d time(s)" % k.applied_count, "REQ-CAL-01")

        if not k.calibration_required:
            return
        if k.calibration_id == "":
            self._rej("calibration.calibration_id",
                      ConfigReason.CALIBRATION_MISSING,
                      ExchangeStatus.CALIBRATION_MISSING,
                      "a calibration id is required; an uncalibrated absolute "
                      "range may not be claimed", "REQ-CAL-01")
            return
        if k.record.calibration_id != k.calibration_id:
            self._cfg_rej("calibration.record.calibration_id",
                          ConfigReason.FIELD_CONFLICT,
                          "the applicability record names a different calibration id",
                          "REQ-CAL-01")
        if k.record.channel != c.phy.channel:
            self._cfg_rej(
                "calibration.record.channel", ConfigReason.CALIBRATION_MISMATCH,
                "calibration channel %d does not match phy.channel %d" % (
                    k.record.channel, c.phy.channel), "REQ-CAL-01")
        # M0.1: the applicability rule below is guarded by `> 0.0`, and a NaN
        # fails that comparison, so a non-finite value here used to skip the
        # whole validator.  The record is compared against a rate, so it must be
        # finite whether or not the match is ever attempted.  Matches the C++
        # guard added in uwb_twr_config.h (found by the parity corpus).
        self._finite("calibration.record.native_sample_rate_hz",
                     k.record.native_sample_rate_hz)
        if (k.record.native_sample_rate_hz > 0.0 and
                c.radio.native_sample_rate_hz > 0.0 and
                not rate_matches_strict(k.record.native_sample_rate_hz,
                                        c.radio.native_sample_rate_hz)):
            self._cfg_rej("calibration.record.native_sample_rate_hz",
                          ConfigReason.CALIBRATION_MISMATCH,
                          "calibration sample rate does not match the radio "
                          "native rate", "REQ-CAL-01")
        if (k.record.profile_version != "" and c.meta.profile_version != "" and
                k.record.profile_version != c.meta.profile_version):
            self._cfg_rej(
                "calibration.record.profile_version",
                ConfigReason.CALIBRATION_MISMATCH,
                "calibration profile_version '%s' does not match the session "
                "profile_version '%s'" % (k.record.profile_version,
                                          c.meta.profile_version), "REQ-CAL-01")
        if (c.meta.calibration_version != "" and
                k.calibration_id != c.meta.calibration_version):
            self._cfg_rej("meta.calibration_version",
                          ConfigReason.CALIBRATION_MISMATCH,
                          "meta.calibration_version does not name the calibration "
                          "that is loaded", "REQ-OUT-01")
        if (c.tx.gain_db is not None and k.record.gain_db is not None and
                k.record.gain_db != c.tx.gain_db):
            self._cfg_rej("calibration.record.gain_db",
                          ConfigReason.CALIBRATION_MISMATCH,
                          "the calibration was taken at a different TX gain",
                          "REQ-CAL-01")

    def check_diagnostics(self, c: TwrConfig) -> None:
        d = c.diagnostics
        buffers = (
            ("diagnostics.cir_capture_enabled",
             "diagnostics.cir_capture_max_bytes",
             "diagnostics.cir_capture_stride", d.cir_capture_enabled,
             d.cir_capture_max_bytes, d.cir_capture_stride, True),
            ("diagnostics.short_iq_enabled", "diagnostics.short_iq_max_bytes",
             "diagnostics.short_iq_stride", d.short_iq_enabled,
             d.short_iq_max_bytes, d.short_iq_stride, True),
            ("diagnostics.raw_frame_dump", "diagnostics.raw_frame_max_bytes",
             None, d.raw_frame_dump, d.raw_frame_max_bytes, 0, False),
        )
        for _on, bytes_path, stride_path, enabled, nbytes, stride, has_stride in \
                buffers:
            if nbytes > MAX_DIAGNOSTIC_BYTES:
                self._cfg_rej(bytes_path, ConfigReason.OVER_CAPACITY,
                              "diagnostic buffer above the %d byte bound "
                              "(bounded memory, REQ-GR-03)" % MAX_DIAGNOSTIC_BYTES)
            if enabled and nbytes == 0:
                self._cfg_rej(bytes_path, ConfigReason.ZERO_VALUE,
                              "the diagnostic is enabled but no byte budget was "
                              "stated")
            if not enabled and nbytes != 0:
                self._cfg_rej(bytes_path, ConfigReason.FIELD_CONFLICT,
                              "a byte budget is stated but the diagnostic is "
                              "disabled")
            if has_stride and enabled and stride == 0:
                self._cfg_rej(stride_path, ConfigReason.ZERO_VALUE,
                              "a diagnostic stride of 0 would capture nothing")
            if has_stride and not enabled and stride != 0:
                self._cfg_rej(stride_path, ConfigReason.FIELD_CONFLICT,
                              "a diagnostic stride is stated but the diagnostic "
                              "is disabled")
        if d.result_queue_capacity == 0:
            self._cfg_rej("diagnostics.result_queue_capacity",
                          ConfigReason.ZERO_VALUE,
                          "the result queue must hold at least one entry")
        elif d.result_queue_capacity > MAX_QUEUE_ENTRIES:
            self._cfg_rej("diagnostics.result_queue_capacity",
                          ConfigReason.OVER_CAPACITY,
                          "above the bounded queue limit")
        if d.event_queue_capacity == 0:
            self._cfg_rej("diagnostics.event_queue_capacity",
                          ConfigReason.ZERO_VALUE,
                          "the event queue must hold at least one entry")
        elif d.event_queue_capacity > MAX_QUEUE_ENTRIES:
            self._cfg_rej("diagnostics.event_queue_capacity",
                          ConfigReason.OVER_CAPACITY,
                          "above the bounded queue limit")
        if d.result_output_path != "" and len(d.result_output_path) > 4096:
            self._cfg_rej("diagnostics.result_output_path",
                          ConfigReason.OVER_CAPACITY,
                          "absurdly long output path")
        if d.io_on_realtime_thread:
            self._cfg_rej("diagnostics.io_on_realtime_thread",
                          ConfigReason.FIELD_CONFLICT,
                          "diagnostic I/O must not run on the realtime "
                          "processing thread", "REQ-GR-02")
        self._finite_timed("diagnostics.stats_cadence", d.stats_cadence)
        self._validate_timed_field("diagnostics.stats_cadence", d.stats_cadence,
                                   False, TimeReferenceEvent.HOST_MONOTONIC,
                                   False)

    def check_frame_lengths(self, c: TwrConfig) -> None:
        """Poll / Response / Final must each fit the negotiated PSDU.

        Every length below comes from the CODEC, through ``frame_psdu_bytes()``
        -> ``FrameProfileGeometry.mac_payload_bytes()``.  The config's geometry
        is a claim, already checked field by field in ``check_frame()``, and is
        used here ONLY to describe the arithmetic back to the operator.
        """
        f = c.frame
        if f.mac_psdu_bytes == 0:
            return  # already reported as a missing value elsewhere
        cap = self.caps.max_psdu_bytes()
        if cap != 0 and f.mac_psdu_bytes > cap:
            self._rej("frame.mac_psdu_bytes", ConfigReason.FRAME_LENGTH_OVERFLOW,
                      ExchangeStatus.UNSUPPORTED,
                      "negotiated PSDU %d bytes exceeds the capability maximum "
                      "%d bytes" % (f.mac_psdu_bytes, cap), "REQ-PHY-01")

        authority = frame_geometry_for(f.frame_profile)

        for t in (FrameType.POLL, FrameType.RESPONSE, FrameType.FINAL):
            # `mac_psdu_bytes` is the negotiated MAC PSDU and, when the PHY
            # appends the FCS, EXCLUDES it (see the fcs_append /
            # mac_psdu_includes_fcs cross-check above).  So the comparison
            # below must be against the MAC bytes only.  The 127-byte IEEE
            # limit is separately checked against the ON-AIR length.
            need = frame_psdu_bytes(t, f.fcs_append)
            on_air = (authority.on_air_bytes(t) if authority is not None
                      else need + (f.fcs_bytes
                                   if f.fcs_append is FcsAppender.PHY else 0))
            path = "frame.mac_psdu_bytes[%s]" % t
            if need == 0:
                continue  # the profile itself was already reported as unknown
            if need > f.mac_psdu_bytes:
                self._rej(
                    path, ConfigReason.FRAME_LENGTH_OVERFLOW,
                    ExchangeStatus.UNSUPPORTED,
                    "the %s frame needs %d MAC PSDU bytes (as frame profile %s "
                    "builds it: header %d + %dx%d timestamps + footer %d%s) but "
                    "only %d are negotiated" % (
                        t, need, f.frame_profile,
                        authority.mac_header_bytes() if authority else 0,
                        frame_type_timestamp_count(t),
                        authority.timestamp_bytes() if authority else 0,
                        authority.mac_footer_bytes() if authority else 0,
                        (" + FCS %d" % f.fcs_bytes)
                        if f.fcs_append is FcsAppender.MAC
                        else "; the PHY appends the FCS separately",
                        f.mac_psdu_bytes), "REQ-PHY-01")
            # The IEEE 127-byte limit applies to the ON-AIR PSDU, i.e. with
            # the FCS the PHY appends, so this is the check that uses
            # `on_air`.
            if cap != 0 and on_air > cap:
                self._rej(
                    path, ConfigReason.FRAME_LENGTH_OVERFLOW,
                    ExchangeStatus.UNSUPPORTED,
                    "the %s frame needs %d on-air PSDU bytes (MAC %d + FCS %d) "
                    "and does not fit the %d-byte PSDU maximum" % (
                        t, on_air, need,
                        authority.fcs_bytes_on_air() if authority else f.fcs_bytes,
                        cap), "REQ-PHY-01")

        # The application payload is a separate concept and must fit inside
        # the MAC PSDU after the header/timestamps/FCS are removed.
        if f.geometry.any_set() and f.application_payload_bytes > f.mac_psdu_bytes:
            self._rej("frame.application_payload_bytes",
                      ConfigReason.FRAME_LENGTH_OVERFLOW,
                      ExchangeStatus.UNSUPPORTED,
                      "the application payload (%d bytes) does not fit the MAC "
                      "PSDU (%d bytes)" % (f.application_payload_bytes,
                                           f.mac_psdu_bytes), "REQ-API-01")

        # The joint PHY capability lookup, now that every axis is known.
        look = self.caps.lookup_phy(c.radio.native_sample_rate_hz,
                                    c.phy.tx_preamble_code,
                                    c.phy.preamble_symbols, f.sfd_mode,
                                    f.mac_psdu_bytes, f.ranging_bit)
        if not look.allowed:
            self._rej(
                "phy.preamble_symbols", ConfigReason.UNSUPPORTED,
                ExchangeStatus.UNSUPPORTED,
                "the (native_rate, code, sync, sfd, psdu, ranging) combination is "
                "not in the capability whitelist [%s]: %s" % (look.status,
                                                              look.reason),
                "REQ-PHY-01")


def validate(cfg: TwrConfig, caps: Optional[Capabilities] = None) \
        -> ValidationReport:
    """The ONE validator every entry point uses (REQ-API-01).

    ``caps`` defaults to the process-wide measured whitelist.  The returned
    report is the authority: ``ok()`` is never the only thing a caller learns,
    because each violation carries a field path, a ``ConfigReason``, an
    ``ExchangeStatus``, a human message and a requirement tag.
    """
    return ConfigValidator(caps if caps is not None else capabilities()).run(cfg)


# ===========================================================================
# 10. The frozen effective config
# ===========================================================================


@dataclass
class EffectiveConfig:
    """A frozen, self-describing snapshot (REQ-API-01).

    It is a VALUE: an exchange in flight holds a copy, so a later change to
    the requested config cannot reach it (REQ-API-03).

    VALUE SEMANTICS (M0.1, defect R1)
    ----------------------------------
    :func:`effective_config` builds ``requested``, ``effective`` and ``readback``
    as three INDEPENDENT deep copies, taken once from the caller's config.  No
    attribute here aliases the object the caller passed in, and no two of them
    alias each other -- not the top-level dataclasses, not a nested
    ``radio.peers`` list, not an element of it.

    Why this matters, concretely: before the fix, ``e.effective IS cfg``, so
    ``cfg.phy.channel = 9`` after the call left ``e.ok`` True and
    ``e.config_hash`` naming the config that had actually been validated, while
    the live config no longer validated.  Any later use of the snapshot would
    then have been driven by a config nobody ever checked.  A validated config
    is a value or it is nothing.

    The cost is one deep copy per snapshot, which is a configure-time and
    exchange-time cost only.  See the module docstring for where copying is and
    is not allowed to happen.
    """

    ok: bool = False
    validation: ValidationReport = field(default_factory=ValidationReport)

    schema_version: str = ""
    profile_version: str = ""
    calibration_version: str = ""
    config_hash: str = ""

    requested: TwrConfig = field(default_factory=TwrConfig)
    effective: TwrConfig = field(default_factory=TwrConfig)
    changes: List[FieldChange] = field(default_factory=list)

    readback: RadioReadback = field(default_factory=RadioReadback)

    #: Quantised timing actually handed to the device.  The REQUESTED delay and
    #: the EFFECTIVE (tick-quantised) delay are both available so REQ-GR-04
    #: can report the margin.
    poll_to_response_ticks: int = 0
    response_to_final_ticks: int = 0
    post_tx_rx_enable_ticks: int = 0
    poll_start_ticks: int = 0
    poll_to_response_effective: Duration = field(default_factory=Duration)
    response_to_final_effective: Duration = field(default_factory=Duration)
    post_tx_rx_enable_effective: Duration = field(default_factory=Duration)
    poll_start_effective: Duration = field(default_factory=Duration)
    tick_rate_hz: float = 0.0

    #: Frame budget, per frame type, as transmitted (FCS exactly once).  These
    #: are the CODEC's numbers, not a re-summation of the config's geometry
    #: claim: see :func:`effective_config`.
    poll_bytes: int = 0
    response_bytes: int = 0
    final_bytes: int = 0
    max_psdu_bytes: int = 0
    max_timestamp_count: int = 0
    #: The PHR of this profile, recorded so no consumer recomputes it: the
    #: number of INFORMATION bytes and the number of transmitted coded bits.
    phr_bytes: int = 0
    phr_coded_bits: int = 0


def copy_config(value: Any) -> Any:
    """A VALUE copy of a config object, recursively.

    The same helper copies a whole ``TwrConfig``, a single nested dataclass such
    as ``radio.readback``, and an ``EffectiveConfig``; all three are plain
    dataclasses and enums, and ``deepcopy`` is the only one mechanism that is
    correct for all of them.

    This is the copy :func:`effective_config` and :class:`TwrConfigSnapshot`
    use, exposed so a caller can take the same copy themselves instead of
    reaching into a snapshot.  It is a ``deepcopy``, not a ``dataclasses.
    replace``: a shallow copy would still share ``radio.peers`` and every
    element of it, which is precisely the aliasing R1 is about.

    WHERE COPYING BELONGS (M0.1, binding on M2/M3)
    ----------------------------------------------
    Copying is a CONFIGURE/EXCHANGE-BOUNDARY cost and nothing else:

      * ALLOWED: once when a config is read from JSON or built by the operator;
        once inside ``effective_config()``; once inside ``TwrConfigSnapshot``;
        once per ``set_overrides()``.  These all run at startup or at an
        exchange boundary, where a heap allocation is free.
      * FORBIDDEN: any per-sample or per-IQ path, and any handler that runs on
        a stream.  M2's work-grid / native-rate conversion and M3's timed-TX
        controller must NOT copy a config per packet: they hold the one
        effective value they were given and read it.  A per-packet copy would
        also be a per-packet allocation, which is exactly what the realtime
        budget forbids.
      * If a future hot path needs an immutable view, it must use a borrowed
        reference held for the lifetime of the work, or a fixed-size POD
        struct on the C++ side -- never a per-iteration ``deepcopy``.

    The cost is linear in the number of config fields (a few hundred objects,
    all small) and is paid a handful of times per session.  It is not on any
    path where its cost is observable, and it is the price of never having to
    ask "is this snapshot still true?".
    """
    return _copy.deepcopy(value)


def effective_config(cfg: TwrConfig,
                     caps: Optional[Capabilities] = None) -> EffectiveConfig:
    """Build the frozen snapshot.

    On failure ``ok`` is False, ``validation`` holds every reason, and
    ``effective`` is a value-initialised (NOT a default-substituted) config:
    there is no way to accidentally use it.

    ISOLATION (M0.1, defect R1): the result owns deep copies.  ``requested``,
    ``effective`` and ``readback`` are three independent values, none of them
    is the caller's ``cfg``, and mutating the caller's config afterwards --
    directly, or through a list element, or through the readback -- cannot move
    the snapshot, its hash, its ``ok`` flag or its quantised numbers.  The one
    legal way to change a validated config is ``TwrConfigSnapshot.
    set_overrides()`` at an exchange boundary.
    """
    if caps is None:
        caps = capabilities()
    out = EffectiveConfig()
    # ONE read of the caller's config, immediately turned into a private value.
    # Everything below is computed from that copy, so a caller mutating `cfg`
    # concurrently cannot make the report describe one config and the hash
    # another.  Validating a value rather than a live object is the whole point
    # (REQ-API-01: a validated config is immutable for the run that validated it).
    src = copy_config(cfg)
    out.requested = src
    out.validation = validate(src, caps)
    out.ok = out.validation.ok()
    out.schema_version = caps.schema_version
    out.profile_version = src.meta.profile_version
    out.calibration_version = src.meta.calibration_version
    if not out.ok:
        # A rejected config produces no effective values at all.  The hash of
        # the REQUESTED config is still recorded so a failure can be tied to
        # the exact input that caused it (REQ-OUT-01).
        out.effective = TwrConfig()
        out.config_hash = config_hash(src)
        out.changes = []
        return out

    # Effective == requested plus the materialisations the validator proved
    # legal: nothing is invented, everything is recorded.  It is a SEPARATE
    # copy, so a later edit to `requested` (by anyone) cannot reach it.
    out.effective = copy_config(src)
    out.config_hash = config_hash(out.effective)
    out.readback = copy_config(src.radio.readback)
    out.changes = diff_fields(out.requested, out.effective)
    out.max_psdu_bytes = caps.max_psdu_bytes()
    out.max_timestamp_count = frame_type_timestamp_count(FrameType.FINAL)

    rate = (src.radio.native_sample_rate_hz
            if caps.native_rate_supported(src.radio.native_sample_rate_hz)
            else caps.native_rates_hz[0])
    out.tick_rate_hz = rate

    # `src` stays the private copy for the whole derivation: the quantised
    # numbers and the frame budget below must describe the SAME config the
    # validation report describes, not whatever the caller holds now.
    items = (
        (src.timing.poll_start, "poll_start_ticks", "poll_start_effective"),
        (src.timing.poll_to_response, "poll_to_response_ticks",
         "poll_to_response_effective"),
        (src.timing.response_to_final, "response_to_final_ticks",
         "response_to_final_effective"),
        (src.timing.post_tx_rx_enable, "post_tx_rx_enable_ticks",
         "post_tx_rx_enable_effective"),
    )
    for field, ticks_attr, eff_attr in items:
        if field.domain != TimeDomain.DEVICE_TICKS:
            continue
        ticks, back = quantise_duration(field.value,
                                        field.required_quantisation_hz)
        setattr(out, ticks_attr, ticks if ticks is not None else 0)
        setattr(out, eff_attr, back if back is not None else Duration())
        if back is None or back == field.value:
            continue
        # Record the quantisation as an explicit requested/effective
        # difference; the device gets the tick value, the operator sees both.
        out.changes.append(FieldChange(
            "timing.quantised_delay" if field.value.nanos() != 0 else "timing.delay",
            duration_to_text(field.value), duration_to_text(back),
            "quantised at %s Hz (ticks=%d)" % (
                double_to_text(field.required_quantisation_hz), ticks or 0)))

    # The frame budget is the CODEC's, computed once through the authority.
    # M0 re-summed the config's own geometry claim here, which is how a 7-byte
    # header produced a 9-byte Poll and understated the on-air length; and it
    # took the FCS size from a raw config field, so a wrong fcs_bytes silently
    # changed the answer.  `on_air_bytes()` is invariant under which layer
    # appends the FCS, which is exactly the property the budget should have.
    authority = frame_geometry_for(src.frame.frame_profile)
    if authority is not None:
        out.poll_bytes = authority.on_air_bytes(FrameType.POLL)
        out.response_bytes = authority.on_air_bytes(FrameType.RESPONSE)
        out.final_bytes = authority.on_air_bytes(FrameType.FINAL)
        # The PHR is present in every frame of this profile; recording its
        # information size here means a consumer never has to recompute it.
        out.phr_bytes = authority.phr_bytes()
        out.phr_coded_bits = authority.phr_coded_bits()
    return out


# ===========================================================================
# 11. In-flight gate and per-exchange overrides
# ===========================================================================


class ExchangeGate:
    """Enforces "同端初版一次仅一个在途 exchange" at RUN time.

    A second concurrent request is QUEUE_FULL, never a silently dropped or
    queued-forever exchange.
    """

    def __init__(self, max_in_flight: int = MAX_IN_FLIGHT_EXCHANGES):
        self.max_in_flight = max_in_flight if max_in_flight else 1
        self.in_flight_count = 0

    def try_begin(self) -> ValidationReport:
        r = ValidationReport()
        if self.in_flight_count >= self.max_in_flight:
            r.add(ConfigViolation(
                field="session.max_in_flight_exchanges",
                reason=ConfigReason.EXCHANGE_ALREADY_IN_FLIGHT,
                status=ExchangeStatus.QUEUE_FULL,
                message="an exchange is already in flight; at most %d may be "
                        "outstanding" % self.max_in_flight,
                requirement="REQ-API-01"))
            # A REFUSED request must not consume a slot, or a run of refusals
            # would deadlock the endpoint.
            return r
        self.in_flight_count += 1
        return r

    def end(self) -> None:
        if self.in_flight_count > 0:
            self.in_flight_count -= 1

    def in_flight(self) -> int:
        return self.in_flight_count


@dataclass
class PerMessageOverrides:
    """Only the values a single exchange may re-negotiate (REQ-API-03).

    Anything that affects synchronisation (PHY, frequency, gain, addresses,
    calibration) is deliberately ABSENT: it may only change at a session
    boundary, and the snapshot an exchange already holds is immutable.  There
    is no field here for a preamble code, a channel, a gain or a calibration
    id, and adding one would be an API bug rather than a feature.
    """

    #: 0 = "not overridden"; the validator then requires the base value.
    measurement_count: int = 0
    measurement_interval_ns: int = 0
    poll_to_response_ns: int = 0
    response_to_final_ns: int = 0
    post_tx_rx_enable_ns: int = 0
    rx_timeout_ns: int = 0
    exchange_timeout_ns: int = 0

    #: The complete set of overridable field names, for a test that asserts the
    #: absence of any PHY / frequency / gain / address / calibration field.
    OVERRIDABLE_FIELDS = (
        "measurement_count", "measurement_interval_ns", "poll_to_response_ns",
        "response_to_final_ns", "post_tx_rx_enable_ns", "rx_timeout_ns",
        "exchange_timeout_ns",
    )

    def any(self) -> bool:
        return any(getattr(self, f) != 0 for f in self.OVERRIDABLE_FIELDS)


class TwrConfigSnapshot:
    """The frozen snapshot plus the exchange gate.

    Copying it is the only way to hand an immutable config to a worker; there
    is no mutable accessor.

    ISOLATION MECHANISM (M0.1, defect R1 -- chosen: COPY ON WRITE-OUT)
    ------------------------------------------------------------------
    Two copies, both ``deepcopy``:

      1. The CONSTRUCTOR deep-copies the ``EffectiveConfig`` it is given, so the
         caller's object is the caller's: editing it afterwards cannot reach
         this snapshot, and neither can editing the snapshot's private copy.
      2. ``get()`` returns a FRESH DEEP COPY on every call.  There is no
         reference to the internal config in the public surface at all: no
         attribute, no cache, no iterator, and ``get()`` twice never returns the
         same object.  A caller can do anything it likes to what ``get()``
         returns -- read it, keep it, mutate it, throw it away -- and the
         snapshot is unchanged.

    Why a copy per ``get()`` rather than a read-only proxy (a frozen
    ``EffectiveConfig`` / ``__slots__`` view): a proxy would have to be total.
    ``TwrConfig`` is a tree of mutable dataclasses with a ``List[EndpointBinding]``
    inside it and a ``ValidationReport`` with a ``List[ConfigViolation]``, and a
    proxy that only blocks attribute assignment still lets ``cfg.radio.peers[0]
    .tx_channel = 1`` through.  Making every node of the tree read-only is a
    larger and more error-prone change to 12 dataclasses, and it would make
    ``set_overrides()`` -- which legitimately rewrites a private copy -- awkward
    to express.  A copy is total by construction: there is no in-place edit that
    can escape it, because the caller and the snapshot hold different objects.

    PERFORMANCE CONSEQUENCE, stated plainly: ``get()`` is O(config size) in time
    and allocations -- a few hundred small objects, a few hundred microseconds
    on an ordinary host, and NOT a constant.  That is affordable once per
    exchange and once per result record, and that is where it belongs.  It would
    NOT be affordable per packet inside a controller's RX or TX loop, and it
    must NEVER appear on a per-sample or per-IQ path; see :func:`copy_config`
    for the rule that binds M2/M3.  A caller that needs the config on a hot
    path must hold the value ``get()`` returned for the lifetime of the work
    rather than re-fetching it.  If that ever proves too expensive, the fix is a
    fixed-size POD struct on the C++ side (which M1 owns), not a weaker Python
    guarantee.

    ``caps`` is exposed as a FROZEN table: the process-wide whitelist must not
    be widenable through a snapshot reference, and it is deep-copied here so
    that even a table a caller built and still holds cannot move underneath a
    running snapshot.
    """

    def __init__(self, effective: Optional[EffectiveConfig] = None,
                 caps: Optional[Capabilities] = None):
        self._caps = (caps if caps is not None else capabilities()).frozen_copy()
        self._effective = (copy_config(effective) if effective is not None
                           else EffectiveConfig())
        self._gate = ExchangeGate()

    @property
    def caps(self) -> Capabilities:
        """The read-only capability table this snapshot validates against."""
        return self._caps

    def get(self) -> EffectiveConfig:
        """A fresh deep copy of the effective config; the snapshot is not it.

        See the class docstring: this is a COPY, every call, on purpose.  It is
        an exchange-boundary accessor.
        """
        return copy_config(self._effective)

    def ok(self) -> bool:
        return self._effective.ok

    def in_flight(self) -> int:
        return self._gate.in_flight()

    def begin_exchange(self) -> ValidationReport:
        return self._gate.try_begin()

    def end_exchange(self) -> None:
        self._gate.end()

    def set_overrides(self, o: PerMessageOverrides) \
            -> Tuple[ValidationReport, "TwrConfigSnapshot"]:
        """REQ-API-03: overrides take effect at an exchange BOUNDARY.

        Returns ``(report, next_snapshot)``.  A call while an exchange is in
        flight is rejected -- the running exchange keeps the snapshot it started
        with, and the returned snapshot is not valid.
        """
        r = ValidationReport()
        if self._gate.in_flight() > 0:
            r.add(ConfigViolation(
                field="session", reason=ConfigReason.MID_EXCHANGE_MUTATION,
                status=ExchangeStatus.CONFIG_REJECTED,
                message="an exchange is in flight; overrides may only be applied "
                        "at an exchange boundary and the running exchange keeps "
                        "its immutable snapshot",
                requirement="REQ-API-03"))
            return (r, TwrConfigSnapshot(EffectiveConfig(), self._caps))
        if not self._effective.ok:
            r.add(ConfigViolation(
                field="", reason=ConfigReason.OUT_OF_SCOPE,
                status=ExchangeStatus.CONFIG_REJECTED,
                message="the base config is not valid; overrides cannot be "
                        "applied to it", requirement="REQ-API-03"))
            return (r, TwrConfigSnapshot(EffectiveConfig(), self._caps))

        nxt = copy_config(self._effective.effective)
        if o.measurement_count != 0:
            nxt.session.measurement_count = (o.measurement_count if
                                             o.measurement_count >= 0 else 0)
        if o.measurement_interval_ns != 0:
            nxt.session.measurement_interval = Duration(o.measurement_interval_ns)
        if o.poll_to_response_ns != 0:
            nxt.timing.poll_to_response.value = Duration(o.poll_to_response_ns)
        if o.response_to_final_ns != 0:
            nxt.timing.response_to_final.value = Duration(o.response_to_final_ns)
        if o.post_tx_rx_enable_ns != 0:
            nxt.timing.post_tx_rx_enable.value = Duration(o.post_tx_rx_enable_ns)
        if o.rx_timeout_ns != 0:
            nxt.timeouts.rx_timeout.value = Duration(o.rx_timeout_ns)
        if o.exchange_timeout_ns != 0:
            nxt.timeouts.exchange_timeout.value = Duration(o.exchange_timeout_ns)

        out_eff = effective_config(nxt, self._caps)
        if not out_eff.ok:
            return (out_eff.validation, TwrConfigSnapshot(out_eff, self._caps))
        # The requested side stays the ORIGINAL request, so the record shows
        # what was asked for and what the exchange actually ran with.  It is a
        # COPY of the base snapshot's requested config: two snapshots must never
        # share one TwrConfig, or editing one would edit the other.
        out_eff.requested = copy_config(self._effective.requested)
        out_eff.changes = diff_fields(out_eff.requested, out_eff.effective)
        # The snapshot is built LAST, from the finished EffectiveConfig: its
        # constructor takes a private copy, so anything patched after the fact
        # would be silently lost.
        return (r, TwrConfigSnapshot(out_eff, self._caps))


# ===========================================================================
# 12. Calibration: applicable exactly once
# ===========================================================================


def apply_calibration_once(cal: TimestampCalibrationConfig,
                           now_monotonic_ns: int = 0) \
        -> Tuple[ExchangeStatus, str]:
    """Apply the configured calibration to a measurement.

    Idempotence is ENFORCED, not assumed: a second application is rejected so
    a range can never be corrected twice.  ``now_monotonic_ns`` comes from the
    host monotonic clock and is only used for the expiry check.  Returns
    ``(status, why)``.
    """
    if cal.calibration_id == "":
        return (ExchangeStatus.CALIBRATION_MISSING,
                "no calibration id: an uncalibrated absolute range may not be "
                "claimed")
    if cal.applied_count != 0:
        return (ExchangeStatus.CALIBRATION_MISSING,
                "calibration already applied %d time(s); a calibration is "
                "applicable exactly once" % cal.applied_count)
    if (cal.record.valid_until_monotonic_ns != 0 and now_monotonic_ns != 0 and
            now_monotonic_ns > cal.record.valid_until_monotonic_ns):
        return (ExchangeStatus.CALIBRATION_EXPIRED,
                "calibration %s expired at %d" % (
                    cal.calibration_id, cal.record.valid_until_monotonic_ns))
    cal.applied_count = 1
    return (ExchangeStatus.OK, "calibration %s applied once" % cal.calibration_id)


# ===========================================================================
# 13. JSON import / export
# ===========================================================================
#
# SCHEMA NOTES (REQ-OUT-01), identical to the C++ side:
#   * Every value is emitted, including ``null`` for an absent optional field,
#     so ``from_json(to_json(c)) == c`` is exact.
#   * 64-bit integers are emitted UNQUOTED when |v| <= 2**53-1 and as a
#     QUOTED DECIMAL STRING otherwise.  The reader accepts both forms, so a
#     timestamp beyond the JSON safe range survives a round trip.
#   * Doubles are emitted with %.17g, which round-trips every finite IEEE 754
#     double.  NaN / +Inf / -Inf have no JSON representation and are REJECTED
#     on export, so a corrupt config can never be serialised into a file that a
#     later run would parse as a valid one.
#   * Durations are emitted as a plain integer number of NANOSECONDS under a
#     ``_ns`` key; native-tick fields carry ``_native_ticks``; the calibration
#     additionally carries an explicit ``link_delay_unit``, so ns, seconds and
#     native ticks can never be confused.
#   * An unknown key is an error, and a missing key is an error.  Neither is
#     ever silently ignored.


class ConfigJsonError(Exception):
    """Raised when a config cannot be represented in JSON (REQ-OUT-01)."""

    def __init__(self, message: str):
        super().__init__(message)
        self.message = message


class _JsonFloat(float):
    """Marker type for a literal that WAS written with a '.' or an exponent."""


def _reject_constant(name: str) -> None:
    raise ValueError("JSON constant not allowed: %s" % name)


def parse_int64_strict(s: str) -> Optional[int]:
    """Strict decimal-int64 parse; ``None`` for anything else.

    A decimal fraction, an exponent, hex, or surrounding whitespace is a
    REJECTION, never a silent truncation.
    """
    if not isinstance(s, str) or s == "":
        return None
    i = 1 if s[0] == "-" else 0
    if i >= len(s):
        return None
    for ch in s[i:]:
        if ch < "0" or ch > "9":
            return None
    try:
        v = int(s)
    except ValueError:
        return None
    if v < INT64_MIN or v > INT64_MAX:
        return None
    return v


def _json_int(v: int) -> Any:
    """Unquoted when JSON-safe, a quoted decimal string otherwise."""
    if -JSON_MAX_SAFE_INTEGER <= v <= JSON_MAX_SAFE_INTEGER:
        return int(v)
    return int_to_text(int(v))


def _to_dict_timed(t: TimedField) -> Dict[str, Any]:
    return {
        "ns": _json_int(t.value.nanos()),
        "domain": str(t.domain),
        "reference": str(t.reference),
        "marker": str(t.marker) if t.marker is not None else None,
        "quantisation_hz": _finite_or_fail(t.required_quantisation_hz,
                                           "timed quantisation_hz"),
        "max_quantisation_error_ns": _json_int(int(t.max_quantisation_error_ns)),
        "note": t.note,
    }


def _finite_or_fail(v: float, what: str) -> float:
    if not _is_finite(v):
        raise ConfigJsonError("field %s is not finite and cannot be written as "
                              "JSON" % what)
    return float(v)


def to_json_dict(cfg: TwrConfig) -> Dict[str, Any]:
    """The config as a plain dict with every field present.

    Raises ``ConfigJsonError`` on a non-finite double, so a corrupt config can
    never be serialised into a file a later run would read as valid.
    """
    return {
        "meta": {
            "schema_version": cfg.meta.schema_version,
            "profile_version": cfg.meta.profile_version,
            "calibration_version": cfg.meta.calibration_version,
            "label": cfg.meta.label,
        },
        "session": {
            "protocol": str(cfg.session.protocol),
            "role": str(cfg.session.role),
            "local_address": _json_int(cfg.session.local_address),
            "peer_address": _json_int(cfg.session.peer_address),
            "pan_id": _json_int(cfg.session.pan_id),
            "session_id": _json_int(cfg.session.session_id),
            "exchange_id": _json_int(cfg.session.exchange_id),
            "sequence": _json_int(cfg.session.sequence),
            "sequence_modulus": _json_int(cfg.session.sequence_modulus),
            "measurement_count": _json_int(cfg.session.measurement_count),
            "measurement_interval_ns": _json_int(
                cfg.session.measurement_interval.nanos()),
            "max_attempts_per_exchange": _json_int(
                cfg.session.max_attempts_per_exchange),
            "retry_backoff_ns": _json_int(cfg.session.retry_backoff.nanos()),
            "max_in_flight_exchanges": _json_int(
                cfg.session.max_in_flight_exchanges),
            "require_pan_match": bool(cfg.session.require_pan_match),
            "require_address_match": bool(cfg.session.require_address_match),
        },
        "phy": {
            "channel": _json_int(cfg.phy.channel),
            "center_frequency_hz": _finite_or_fail(cfg.phy.center_frequency_hz,
                                                   "phy.center_frequency_hz"),
            "tx_preamble_code": _json_int(cfg.phy.tx_preamble_code),
            "rx_preamble_code": _json_int(cfg.phy.rx_preamble_code),
            "preamble_symbols": _json_int(cfg.phy.preamble_symbols),
            "prf_class": str(cfg.phy.prf_class),
            "data_rate": str(cfg.phy.data_rate),
            # PHR rate, in the PHR-RATE spelling ("850k" / "same_as_data"), NOT
            # the payload-rate spelling: the two axes have separate value
            # domains and a document that used "6p8m" here was describing a PHR
            # this build cannot transmit.
            "phr_rate": str(cfg.phy.phr_rate),
        },
        "frame": {
            "frame_profile": str(cfg.frame.frame_profile),
            "sfd_mode": str(cfg.frame.sfd_mode),
            "sfd_symbols": _json_int(cfg.frame.sfd_symbols),
            "sfd_timeout": _to_dict_timed(cfg.frame.sfd_timeout),
            "phr_mode": str(cfg.frame.phr_mode),
            "ranging_bit": bool(cfg.frame.ranging_bit),
            "geometry": {
                "mac_header_bytes": _json_int(cfg.frame.geometry.mac_header_bytes),
                "timestamp_bytes": _json_int(cfg.frame.geometry.timestamp_bytes),
                "mac_footer_bytes": _json_int(cfg.frame.geometry.mac_footer_bytes),
                "mac_fcs_bytes": _json_int(cfg.frame.geometry.mac_fcs_bytes),
                "phr_bytes": _json_int(cfg.frame.geometry.phr_bytes),
            },
            "mac_psdu_bytes": _json_int(cfg.frame.mac_psdu_bytes),
            "mac_psdu_includes_fcs": bool(cfg.frame.mac_psdu_includes_fcs),
            "fcs_append": str(cfg.frame.fcs_append),
            "fcs_bytes": _json_int(cfg.frame.fcs_bytes),
            "application_payload_bytes": _json_int(
                cfg.frame.application_payload_bytes),
            "sts_mode": str(cfg.frame.sts_mode),
            "sts_length_symbols": _json_int(cfg.frame.sts_length_symbols),
        },
        "tx": {
            "port": _json_int(cfg.tx.port),
            "gain_db": (None if cfg.tx.gain_db is None else
                        _finite_or_fail(cfg.tx.gain_db, "tx.gain_db")),
            "iq_amplitude": (None if cfg.tx.iq_amplitude is None else
                             _finite_or_fail(cfg.tx.iq_amplitude,
                                             "tx.iq_amplitude")),
            "calibrated_tx_power_dbm": (
                None if cfg.tx.calibrated_tx_power_dbm is None else
                _finite_or_fail(cfg.tx.calibrated_tx_power_dbm,
                                "tx.calibrated_tx_power_dbm")),
            "power_policy": str(cfg.tx.power_policy),
            "pulse_shaping": str(cfg.tx.pulse_shaping),
            "vendor_power_word": (None if cfg.tx.vendor_power_word is None else
                                  _json_int(cfg.tx.vendor_power_word)),
            "vendor_power_word_backend": cfg.tx.vendor_power_word_backend,
        },
        "rx": {
            "port": _json_int(cfg.rx.port),
            "gain_db": (None if cfg.rx.gain_db is None else
                        _finite_or_fail(cfg.rx.gain_db, "rx.gain_db")),
            "agc": str(cfg.rx.agc),
            "bandwidth_hz": (None if cfg.rx.bandwidth_hz is None else
                             _finite_or_fail(cfg.rx.bandwidth_hz,
                                             "rx.bandwidth_hz")),
            "detection_threshold": _finite_or_fail(
                cfg.rx.detection_threshold, "rx.detection_threshold"),
            "correlation_threshold": _finite_or_fail(
                cfg.rx.correlation_threshold, "rx.correlation_threshold"),
            "first_path_threshold": _finite_or_fail(
                cfg.rx.first_path_threshold, "rx.first_path_threshold"),
            "first_path_index": _json_int(cfg.rx.first_path_index),
            "first_path_window": _json_int(cfg.rx.first_path_window),
            "vendor_pac_value": (None if cfg.rx.vendor_pac_value is None else
                                 _json_int(cfg.rx.vendor_pac_value)),
            "vendor_pac_backend": cfg.rx.vendor_pac_backend,
            "vendor_pac_applied_step": (
                None if cfg.rx.vendor_pac_applied_step is None else
                _finite_or_fail(cfg.rx.vendor_pac_applied_step,
                                "rx.vendor_pac_applied_step")),
        },
        "radio": {
            "device_args": cfg.radio.device_args,
            "tx_channel": _json_int(cfg.radio.tx_channel),
            "rx_channel": _json_int(cfg.radio.rx_channel),
            "native_sample_rate_hz": _finite_or_fail(
                cfg.radio.native_sample_rate_hz, "radio.native_sample_rate_hz"),
            "clock_source": cfg.radio.clock_source,
            "time_source": cfg.radio.time_source,
            "fpga_image": cfg.radio.fpga_image,
            "dpdk_config": cfg.radio.dpdk_config,
            "peers": [{
                "id": p.id,
                "role": str(p.role),
                "tx_channel": _json_int(p.tx_channel),
                "rx_channel": _json_int(p.rx_channel),
                "native_sample_rate_hz": _finite_or_fail(
                    p.native_sample_rate_hz, "radio.peers[].native_sample_rate_hz"),
                "occupies_resources": bool(p.occupies_resources),
            } for p in cfg.radio.peers],
            "readback": {
                "present": bool(cfg.radio.readback.present),
                "sample_rate_hz": _finite_or_fail(
                    cfg.radio.readback.sample_rate_hz,
                    "radio.readback.sample_rate_hz"),
                "center_freq_hz": _finite_or_fail(
                    cfg.radio.readback.center_freq_hz,
                    "radio.readback.center_freq_hz"),
                "tx_channel": _json_int(cfg.radio.readback.tx_channel),
                "rx_channel": _json_int(cfg.radio.readback.rx_channel),
                "mpm_string": cfg.radio.readback.mpm_string,
                "fpga_image": cfg.radio.readback.fpga_image,
                "uhd_version": cfg.radio.readback.uhd_version,
                "clock_source": cfg.radio.readback.clock_source,
                "time_source": cfg.radio.readback.time_source,
            },
            "require_readback": bool(cfg.radio.require_readback),
        },
        "timing": {name: _to_dict_timed(getattr(cfg.timing, name)) for name in (
            "poll_start", "poll_to_response", "response_to_final",
            "final_to_report", "post_tx_rx_enable", "min_tx_lead_time")},
        "timeouts": {name: _to_dict_timed(getattr(cfg.timeouts, name)) for name in (
            "poll_rx_window", "response_rx_window", "final_rx_window",
            "report_rx_window", "rx_timeout", "exchange_timeout",
            "retry_interval")},
        "calibration": {
            "tx_link_delay_ns": _json_int(cfg.calibration.tx_link_delay.nanos()),
            "rx_link_delay_ns": _json_int(cfg.calibration.rx_link_delay.nanos()),
            "antenna_delay_ns": _json_int(cfg.calibration.antenna_delay.nanos()),
            "cable_delay_ns": _json_int(cfg.calibration.cable_delay.nanos()),
            "tx_link_delay_native_ticks": _json_int(
                cfg.calibration.tx_link_delay_native_ticks),
            "rx_link_delay_native_ticks": _json_int(
                cfg.calibration.rx_link_delay_native_ticks),
            "native_sample_rate_hz": _finite_or_fail(
                cfg.calibration.native_sample_rate_hz,
                "calibration.native_sample_rate_hz"),
            "link_delay_unit": str(cfg.calibration.link_delay_unit),
            "first_path_algorithm": str(cfg.calibration.first_path_algorithm),
            "cfo_compensation": str(cfg.calibration.cfo_compensation),
            "sfo_compensation": str(cfg.calibration.sfo_compensation),
            "calibration_id": cfg.calibration.calibration_id,
            "record": {
                "calibration_id": cfg.calibration.record.calibration_id,
                "device_serial": cfg.calibration.record.device_serial,
                "channel": _json_int(cfg.calibration.record.channel),
                "native_sample_rate_hz": _finite_or_fail(
                    cfg.calibration.record.native_sample_rate_hz,
                    "calibration.record.native_sample_rate_hz"),
                "profile_version": cfg.calibration.record.profile_version,
                "gain_db": (None if cfg.calibration.record.gain_db is None else
                            _finite_or_fail(cfg.calibration.record.gain_db,
                                            "calibration.record.gain_db")),
                "valid_until_monotonic_ns": _json_int(
                    cfg.calibration.record.valid_until_monotonic_ns),
            },
            "applied_count": _json_int(cfg.calibration.applied_count),
            "calibration_required": bool(cfg.calibration.calibration_required),
        },
        "diagnostics": {
            "cir_capture_enabled": bool(cfg.diagnostics.cir_capture_enabled),
            "cir_capture_max_bytes": _json_int(cfg.diagnostics.cir_capture_max_bytes),
            "cir_capture_stride": _json_int(cfg.diagnostics.cir_capture_stride),
            "short_iq_enabled": bool(cfg.diagnostics.short_iq_enabled),
            "short_iq_max_bytes": _json_int(cfg.diagnostics.short_iq_max_bytes),
            "short_iq_stride": _json_int(cfg.diagnostics.short_iq_stride),
            "raw_frame_dump": bool(cfg.diagnostics.raw_frame_dump),
            "raw_frame_max_bytes": _json_int(cfg.diagnostics.raw_frame_max_bytes),
            "result_output_path": cfg.diagnostics.result_output_path,
            "result_queue_capacity": _json_int(
                cfg.diagnostics.result_queue_capacity),
            "event_queue_capacity": _json_int(
                cfg.diagnostics.event_queue_capacity),
            "stats_cadence": _to_dict_timed(cfg.diagnostics.stats_cadence),
            "io_on_realtime_thread": bool(cfg.diagnostics.io_on_realtime_thread),
        },
    }


class _ConfigJsonReader:
    """Strict reader: every failure is a machine-readable violation.

    Never a silently skipped key, never a default.
    """

    def __init__(self) -> None:
        self.report = ValidationReport()

    def bad(self, field: str, reason: ConfigReason, msg: str,
            status: ExchangeStatus = ExchangeStatus.CONFIG_REJECTED) -> None:
        self.report.add_rejection(field, reason, status, msg, "REQ-API-01")

    # -- navigation ------------------------------------------------------
    def group(self, path: str, parent: Optional[Dict[str, Any]]) \
            -> Optional[Dict[str, Any]]:
        if parent is None:
            return None
        g = parent.get(path)
        if g is None:
            self.bad(path, ConfigReason.MISSING_KEY, "missing required group")
            return None
        if not isinstance(g, dict):
            self.bad(path, ConfigReason.TYPE_MISMATCH,
                     "expected an object, got %s" % _json_type_name(g))
            return None
        return g

    def key_of(self, o: Optional[Dict[str, Any]], path: str, key: str,
               required: bool = True) -> Any:
        if o is None:
            return None
        if key not in o:
            if required:
                self.bad("%s.%s" % (path, key), ConfigReason.MISSING_KEY,
                         "missing required key")
            return None
        return o[key]

    # -- typed getters ---------------------------------------------------
    def get_str(self, o: Optional[Dict[str, Any]], path: str, key: str) -> str:
        v = self.key_of(o, path, key)
        if v is None:
            return ""
        if v is None or v == "":
            return ""
        if not isinstance(v, str):
            self.bad("%s.%s" % (path, key), ConfigReason.TYPE_MISMATCH,
                     "expected a string, got %s" % _json_type_name(v))
            return ""
        return v

    def get_bool(self, o: Optional[Dict[str, Any]], path: str, key: str) -> bool:
        v = self.key_of(o, path, key)
        if v is None:
            return False
        if not isinstance(v, bool):
            self.bad("%s.%s" % (path, key), ConfigReason.TYPE_MISMATCH,
                     "expected a bool, got %s" % _json_type_name(v))
            return False
        return v

    def get_i64(self, o: Optional[Dict[str, Any]], path: str, key: str) -> int:
        v = self.key_of(o, path, key)
        if v is None:
            return 0
        p = "%s.%s" % (path, key)
        if isinstance(v, str):
            # The decimal-string form used for values beyond 2**53.
            parsed = parse_int64_strict(v)
            if parsed is None:
                self.bad(p, ConfigReason.INTEGER_PRECISION_LOSS,
                         "string is not a decimal int64: '%s'" % v)
                return 0
            return parsed
        if isinstance(v, bool) or not isinstance(v, int):
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "expected an integer, got %s (a decimal fraction or "
                     "exponent is never truncated silently)"
                     % _json_type_name(v))
            return 0
        if v < INT64_MIN or v > INT64_MAX:
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "integer literal is outside the int64 range")
            return 0
        return v

    def get_double(self, o: Optional[Dict[str, Any]], path: str, key: str) -> float:
        v = self.key_of(o, path, key)
        if v is None:
            return 0.0
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            self.bad("%s.%s" % (path, key), ConfigReason.TYPE_MISMATCH,
                     "expected a number, got %s" % _json_type_name(v))
            return 0.0
        return float(v)

    def get_dur(self, o: Optional[Dict[str, Any]], path: str, key: str) -> Duration:
        return Duration(self.get_i64(o, path, key))

    def get_opt_i64(self, o: Optional[Dict[str, Any]], path: str,
                    key: str) -> Optional[int]:
        v = self.key_of(o, path, key, required=False)
        if v is None:
            return None
        p = "%s.%s" % (path, key)
        if isinstance(v, str):
            parsed = parse_int64_strict(v)
            if parsed is None:
                self.bad(p, ConfigReason.INTEGER_PRECISION_LOSS,
                         "string is not a decimal int64: '%s'" % v)
                return None
            return parsed
        if isinstance(v, bool) or not isinstance(v, int):
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "expected an integer or null, got %s" % _json_type_name(v))
            return None
        if v < INT64_MIN or v > INT64_MAX:
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "integer literal is outside the int64 range")
            return None
        return v

    def get_opt_u32(self, o: Optional[Dict[str, Any]], path: str,
                    key: str) -> Optional[int]:
        v = self.get_opt_i64(o, path, key)
        if v is None:
            return None
        if v < 0 or v > 0xFFFFFFFF:
            self.bad("%s.%s" % (path, key), ConfigReason.OUT_OF_RANGE,
                     "outside the uint32 range")
            return None
        return v

    def get_opt_double(self, o: Optional[Dict[str, Any]], path: str,
                       key: str) -> Optional[float]:
        v = self.key_of(o, path, key, required=False)
        if v is None:
            return None
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            self.bad("%s.%s" % (path, key), ConfigReason.TYPE_MISMATCH,
                     "expected a number or null, got %s" % _json_type_name(v))
            return None
        return float(v)

    def get_enum(self, o: Optional[Dict[str, Any]], path: str, key: str,
                 enum_cls: Any) -> Any:
        """Read a string enum.

        On ANY failure -- key absent, wrong type, unknown spelling -- the value
        left in the field is the enumeration's ZERO member, exactly what C++
        ``static_cast<E>(0)`` produces.  That is deliberate: it keeps the two
        implementations reporting the same downstream violation, which is what
        the parity corpus compares.

        It is NOT a silent default, and the reason is that this method always
        records a machine-readable violation in the report.  The returned
        config is therefore only usable when the report is empty, and the
        report is what every entry point hands back.  Substituting a
        "sensible" member instead would make the two implementations disagree
        about a document they both reject.
        """
        zero = next(iter(enum_cls))
        if o is None or key not in o:
            self.key_of(o, path, key)          # records the missing key
            return zero
        v = o[key]
        p = "%s.%s" % (path, key)
        if v is None:
            # A present-but-null enum is a TYPE error, not an absent key: C++
            # get_enum sees Type::Null, which is not Type::String, and says so.
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "expected a string enum, got null")
            return zero
        if not isinstance(v, str):
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "expected a string enum, got %s" % _json_type_name(v))
            return zero
        out = enum_cls.from_string(v)
        if out is None:
            self.bad(p, ConfigReason.UNKNOWN_ENUM_VALUE,
                     "'%s' is not a known value of %s" % (v, enum_cls.__name__))
            return zero
        return out

    def get_timed(self, o: Optional[Dict[str, Any]], path: str,
                  key: str) -> TimedField:
        t = TimedField()
        p = "%s.%s" % (path, key)
        v = self.key_of(o, path, key)
        if v is None:
            return t
        if not isinstance(v, dict):
            self.bad(p, ConfigReason.TYPE_MISMATCH,
                     "expected an object, got %s" % _json_type_name(v))
            return t
        t.value = Duration(self.get_i64(v, p, "ns"))
        t.domain = self.get_enum(v, p, "domain", TimeDomain)
        t.reference = self.get_enum(v, p, "reference", TimeReferenceEvent)
        if "marker" in v:
            mk = v["marker"]
            if mk is None:
                t.marker = None
            elif isinstance(mk, str):
                t.marker = TimestampMarker.from_string(mk)
                if t.marker is None:
                    self.bad(p + ".marker", ConfigReason.UNKNOWN_ENUM_VALUE,
                             "'%s' is not a known timestamp marker" % mk)
            else:
                self.bad(p + ".marker", ConfigReason.TYPE_MISMATCH,
                         "expected a marker name or null, got %s"
                         % _json_type_name(mk))
        t.required_quantisation_hz = self.get_double(v, p, "quantisation_hz")
        t.max_quantisation_error_ns = self.get_i64(v, p,
                                                   "max_quantisation_error_ns")
        t.note = self.get_str(v, p, "note")
        return t

    def reject_unknown_keys(self, o: Optional[Dict[str, Any]], path: str,
                            known: Iterable[str]) -> None:
        if not isinstance(o, dict):
            return
        known_set = set(known)
        for k in o:
            if k not in known_set:
                self.bad("%s.%s" % (path, k), ConfigReason.UNKNOWN_KEY,
                         "key is not part of schema %s" % SCHEMA_VERSION)


def _u8(v: int) -> int:
    return v & 0xFF


def _u16(v: int) -> int:
    return v & 0xFFFF


def _u32(v: int) -> int:
    return v & 0xFFFFFFFF


def _u64(v: int) -> int:
    return v & 0xFFFFFFFFFFFFFFFF


def _json_type_name(v: Any) -> str:
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, _JsonFloat):
        return "number"
    if isinstance(v, int):
        return "integer"
    if isinstance(v, float):
        return "number"
    if isinstance(v, str):
        return "string"
    if isinstance(v, list):
        return "array"
    if isinstance(v, dict):
        return "object"
    return "invalid"


_META_KEYS = ("schema_version", "profile_version", "calibration_version", "label")


# The UNSIGNED-WIDTH NARROWING, mirrored from C++.
# ---------------------------------------------------------------------------
# The C++ reader assigns with ``static_cast<uint16_t>(get_i64(...))`` and so on.
# A JSON number outside the field's width is therefore WRAPPED there, not
# refused, and a wrapped value is what the validator then sees.  Python has no
# implicit narrowing, so the reader does it explicitly at the same sites.
#
# It matters: ``"local_address": -1`` is 65535 in C++, which trips the
# "0xffff is a reserved address" rule; a Python that kept -1 would store a value
# no C++ build can hold, would hash differently, and would accept a document the
# authority rejects.  The parity corpus pins the difference (``negative_
# local_address_is_a_negative_index``).
#
# The wrap is a property of the FIELD WIDTH, not a licence to invent a value:
# every check that could reject an out-of-range magnitude still runs afterwards
# on the narrowed number, and the int64 range check in ``get_i64`` still
# refuses a literal outside int64 outright.

_SESSION_KEYS = (
    "protocol", "role", "local_address", "peer_address", "pan_id", "session_id",
    "exchange_id", "sequence", "sequence_modulus", "measurement_count",
    "measurement_interval_ns", "max_attempts_per_exchange", "retry_backoff_ns",
    "max_in_flight_exchanges", "require_pan_match", "require_address_match")
_PHY_KEYS = (
    "channel", "center_frequency_hz", "tx_preamble_code", "rx_preamble_code",
    "preamble_symbols", "prf_class", "data_rate", "phr_rate")
_FRAME_KEYS = (
    "frame_profile",
    "sfd_mode", "sfd_symbols", "sfd_timeout", "phr_mode", "ranging_bit",
    "geometry", "mac_psdu_bytes", "mac_psdu_includes_fcs", "fcs_append",
    "fcs_bytes", "application_payload_bytes", "sts_mode", "sts_length_symbols")
_GEOMETRY_KEYS = ("mac_header_bytes", "timestamp_bytes", "mac_footer_bytes",
                  "mac_fcs_bytes", "phr_bytes")
_TX_KEYS = ("port", "gain_db", "iq_amplitude", "calibrated_tx_power_dbm",
            "power_policy", "pulse_shaping", "vendor_power_word",
            "vendor_power_word_backend")
_RX_KEYS = ("port", "gain_db", "agc", "bandwidth_hz", "detection_threshold",
            "correlation_threshold", "first_path_threshold", "first_path_index",
            "first_path_window", "vendor_pac_value", "vendor_pac_backend",
            "vendor_pac_applied_step")
_RADIO_KEYS = ("device_args", "tx_channel", "rx_channel", "native_sample_rate_hz",
               "clock_source", "time_source", "fpga_image", "dpdk_config",
               "peers", "readback", "require_readback")
_PEER_KEYS = ("id", "role", "tx_channel", "rx_channel", "native_sample_rate_hz",
              "occupies_resources")
_READBACK_KEYS = ("present", "sample_rate_hz", "center_freq_hz", "tx_channel",
                  "rx_channel", "mpm_string", "fpga_image", "uhd_version",
                  "clock_source", "time_source")
_TIMING_KEYS = ("poll_start", "poll_to_response", "response_to_final",
                "final_to_report", "post_tx_rx_enable", "min_tx_lead_time")
_TIMEOUT_KEYS = ("poll_rx_window", "response_rx_window", "final_rx_window",
                 "report_rx_window", "rx_timeout", "exchange_timeout",
                 "retry_interval")
_CALIBRATION_KEYS = (
    "tx_link_delay_ns", "rx_link_delay_ns", "antenna_delay_ns", "cable_delay_ns",
    "tx_link_delay_native_ticks", "rx_link_delay_native_ticks",
    "native_sample_rate_hz", "link_delay_unit", "first_path_algorithm",
    "cfo_compensation", "sfo_compensation", "calibration_id", "record",
    "applied_count", "calibration_required")
_RECORD_KEYS = ("calibration_id", "device_serial", "channel",
                "native_sample_rate_hz", "profile_version", "gain_db",
                "valid_until_monotonic_ns")
_DIAGNOSTICS_KEYS = (
    "cir_capture_enabled", "cir_capture_max_bytes", "cir_capture_stride",
    "short_iq_enabled", "short_iq_max_bytes", "short_iq_stride",
    "raw_frame_dump", "raw_frame_max_bytes", "result_output_path",
    "result_queue_capacity", "event_queue_capacity", "stats_cadence",
    "io_on_realtime_thread")


def from_json_dict(root: Any) -> Tuple[TwrConfig, ValidationReport]:
    """Read a parsed document.  Returns ``(config, report)``."""
    rd = _ConfigJsonReader()
    if not isinstance(root, dict):
        rd.bad("", ConfigReason.TYPE_MISMATCH,
               "the config document must be a JSON object")
        return (TwrConfig(), rd.report)

    c = TwrConfig()
    g = rd.group("meta", root)
    if g is not None:
        rd.reject_unknown_keys(g, "meta", _META_KEYS)
        c.meta.schema_version = rd.get_str(g, "meta", "schema_version")
        c.meta.profile_version = rd.get_str(g, "meta", "profile_version")
        c.meta.calibration_version = rd.get_str(g, "meta", "calibration_version")
        c.meta.label = rd.get_str(g, "meta", "label")

    g = rd.group("session", root)
    if g is not None:
        rd.reject_unknown_keys(g, "session", _SESSION_KEYS)
        s = c.session
        s.protocol = rd.get_enum(g, "session", "protocol", Protocol)
        s.role = rd.get_enum(g, "session", "role", Role)
        s.local_address = _u16(rd.get_i64(g, "session", "local_address"))
        s.peer_address = _u16(rd.get_i64(g, "session", "peer_address"))
        s.pan_id = _u16(rd.get_i64(g, "session", "pan_id"))
        s.session_id = _u32(rd.get_i64(g, "session", "session_id"))
        s.exchange_id = _u32(rd.get_i64(g, "session", "exchange_id"))
        s.sequence = _u8(rd.get_i64(g, "session", "sequence"))
        s.sequence_modulus = _u16(rd.get_i64(g, "session", "sequence_modulus"))
        s.measurement_count = _u32(rd.get_i64(g, "session", "measurement_count"))
        s.measurement_interval = rd.get_dur(g, "session", "measurement_interval_ns")
        s.max_attempts_per_exchange = _u32(rd.get_i64(
            g, "session", "max_attempts_per_exchange"))
        s.retry_backoff = rd.get_dur(g, "session", "retry_backoff_ns")
        s.max_in_flight_exchanges = _u32(rd.get_i64(
            g, "session", "max_in_flight_exchanges"))
        s.require_pan_match = rd.get_bool(g, "session", "require_pan_match")
        s.require_address_match = rd.get_bool(g, "session", "require_address_match")

    g = rd.group("phy", root)
    if g is not None:
        rd.reject_unknown_keys(g, "phy", _PHY_KEYS)
        p = c.phy
        p.channel = _u8(rd.get_i64(g, "phy", "channel"))
        p.center_frequency_hz = rd.get_double(g, "phy", "center_frequency_hz")
        p.tx_preamble_code = _u8(rd.get_i64(g, "phy", "tx_preamble_code"))
        p.rx_preamble_code = _u8(rd.get_i64(g, "phy", "rx_preamble_code"))
        p.preamble_symbols = _u16(rd.get_i64(g, "phy", "preamble_symbols"))
        p.prf_class = rd.get_enum(g, "phy", "prf_class", PrfClass)
        p.data_rate = rd.get_enum(g, "phy", "data_rate", DataRate)
        # PHR rate, read in the PHR-RATE domain.  A payload-rate name such as
        # "6p8m" is now an unknown_enum_value here, which is the point: the
        # two axes no longer share a value domain.
        p.phr_rate = rd.get_enum(g, "phy", "phr_rate", PhrRate)

    g = rd.group("frame", root)
    if g is not None:
        rd.reject_unknown_keys(g, "frame", _FRAME_KEYS)
        f = c.frame
        f.frame_profile = rd.get_enum(g, "frame", "frame_profile", FrameProfileId)
        f.sfd_mode = rd.get_enum(g, "frame", "sfd_mode", SfdMode)
        f.sfd_symbols = _u16(rd.get_i64(g, "frame", "sfd_symbols"))
        f.sfd_timeout = rd.get_timed(g, "frame", "sfd_timeout")
        f.phr_mode = rd.get_enum(g, "frame", "phr_mode", PhrMode)
        f.ranging_bit = rd.get_bool(g, "frame", "ranging_bit")
        geo = g.get("geometry")
        if geo is None:
            rd.bad("frame.geometry", ConfigReason.MISSING_KEY,
                   "missing required group")
        elif not isinstance(geo, dict):
            rd.bad("frame.geometry", ConfigReason.TYPE_MISMATCH, "expected an object")
        else:
            rd.reject_unknown_keys(geo, "frame.geometry", _GEOMETRY_KEYS)
            gp = "frame.geometry"
            f.geometry.mac_header_bytes = _u16(rd.get_i64(geo, gp,
                                                             "mac_header_bytes"))
            f.geometry.timestamp_bytes = _u16(rd.get_i64(geo, gp,
                                                            "timestamp_bytes"))
            f.geometry.mac_footer_bytes = _u16(rd.get_i64(geo, gp,
                                                            "mac_footer_bytes"))
            f.geometry.mac_fcs_bytes = _u16(rd.get_i64(geo, gp,
                                                          "mac_fcs_bytes"))
            f.geometry.phr_bytes = _u16(rd.get_i64(geo, gp, "phr_bytes"))
        f.mac_psdu_bytes = _u16(rd.get_i64(g, "frame", "mac_psdu_bytes"))
        f.mac_psdu_includes_fcs = rd.get_bool(g, "frame", "mac_psdu_includes_fcs")
        f.fcs_append = rd.get_enum(g, "frame", "fcs_append", FcsAppender)
        f.fcs_bytes = _u16(rd.get_i64(g, "frame", "fcs_bytes"))
        f.application_payload_bytes = _u16(rd.get_i64(
            g, "frame", "application_payload_bytes"))
        f.sts_mode = rd.get_enum(g, "frame", "sts_mode", StsMode)
        f.sts_length_symbols = _u16(rd.get_i64(g, "frame", "sts_length_symbols"))

    g = rd.group("tx", root)
    if g is not None:
        rd.reject_unknown_keys(g, "tx", _TX_KEYS)
        t = c.tx
        t.port = _u8(rd.get_i64(g, "tx", "port"))
        t.gain_db = rd.get_opt_double(g, "tx", "gain_db")
        t.iq_amplitude = rd.get_opt_double(g, "tx", "iq_amplitude")
        t.calibrated_tx_power_dbm = rd.get_opt_double(g, "tx",
                                                      "calibrated_tx_power_dbm")
        t.power_policy = rd.get_enum(g, "tx", "power_policy", TxPowerPolicy)
        t.pulse_shaping = rd.get_enum(g, "tx", "pulse_shaping", PulseShaping)
        t.vendor_power_word = rd.get_opt_u32(g, "tx", "vendor_power_word")
        t.vendor_power_word_backend = rd.get_str(g, "tx",
                                                 "vendor_power_word_backend")

    g = rd.group("rx", root)
    if g is not None:
        rd.reject_unknown_keys(g, "rx", _RX_KEYS)
        r = c.rx
        r.port = _u8(rd.get_i64(g, "rx", "port"))
        r.gain_db = rd.get_opt_double(g, "rx", "gain_db")
        r.agc = rd.get_enum(g, "rx", "agc", AgcMode)
        r.bandwidth_hz = rd.get_opt_double(g, "rx", "bandwidth_hz")
        r.detection_threshold = rd.get_double(g, "rx", "detection_threshold")
        r.correlation_threshold = rd.get_double(g, "rx", "correlation_threshold")
        r.first_path_threshold = rd.get_double(g, "rx", "first_path_threshold")
        r.first_path_index = _u16(rd.get_i64(g, "rx", "first_path_index"))
        r.first_path_window = _u16(rd.get_i64(g, "rx", "first_path_window"))
        r.vendor_pac_value = rd.get_opt_u32(g, "rx", "vendor_pac_value")
        r.vendor_pac_backend = rd.get_str(g, "rx", "vendor_pac_backend")
        r.vendor_pac_applied_step = rd.get_opt_double(g, "rx",
                                                      "vendor_pac_applied_step")

    g = rd.group("radio", root)
    if g is not None:
        rd.reject_unknown_keys(g, "radio", _RADIO_KEYS)
        r = c.radio
        r.device_args = rd.get_str(g, "radio", "device_args")
        r.tx_channel = _u8(rd.get_i64(g, "radio", "tx_channel"))
        r.rx_channel = _u8(rd.get_i64(g, "radio", "rx_channel"))
        r.native_sample_rate_hz = rd.get_double(g, "radio", "native_sample_rate_hz")
        r.clock_source = rd.get_str(g, "radio", "clock_source")
        r.time_source = rd.get_str(g, "radio", "time_source")
        r.fpga_image = rd.get_str(g, "radio", "fpga_image")
        r.dpdk_config = rd.get_str(g, "radio", "dpdk_config")
        peers = g.get("peers")
        if peers is None:
            rd.bad("radio.peers", ConfigReason.MISSING_KEY, "missing required key")
        elif not isinstance(peers, list):
            rd.bad("radio.peers", ConfigReason.TYPE_MISMATCH, "expected an array")
        else:
            for i, e in enumerate(peers):
                pp = "radio.peers[%d]" % i
                if not isinstance(e, dict):
                    rd.bad(pp, ConfigReason.TYPE_MISMATCH, "expected an object")
                    continue
                rd.reject_unknown_keys(e, pp, _PEER_KEYS)
                b = EndpointBinding()
                b.id = rd.get_str(e, pp, "id")
                b.role = rd.get_enum(e, pp, "role", Role)
                b.tx_channel = _u8(rd.get_i64(e, pp, "tx_channel"))
                b.rx_channel = _u8(rd.get_i64(e, pp, "rx_channel"))
                b.native_sample_rate_hz = rd.get_double(e, pp,
                                                        "native_sample_rate_hz")
                b.occupies_resources = rd.get_bool(e, pp, "occupies_resources")
                r.peers.append(b)
        rb = g.get("readback")
        if rb is None:
            rd.bad("radio.readback", ConfigReason.MISSING_KEY,
                   "missing required group")
        elif not isinstance(rb, dict):
            rd.bad("radio.readback", ConfigReason.TYPE_MISMATCH,
                   "expected an object")
        else:
            rp = "radio.readback"
            rd.reject_unknown_keys(rb, rp, _READBACK_KEYS)
            r.readback.present = rd.get_bool(rb, rp, "present")
            r.readback.sample_rate_hz = rd.get_double(rb, rp, "sample_rate_hz")
            r.readback.center_freq_hz = rd.get_double(rb, rp, "center_freq_hz")
            r.readback.tx_channel = rd.get_i64(rb, rp, "tx_channel")
            r.readback.rx_channel = rd.get_i64(rb, rp, "rx_channel")
            r.readback.mpm_string = rd.get_str(rb, rp, "mpm_string")
            r.readback.fpga_image = rd.get_str(rb, rp, "fpga_image")
            r.readback.uhd_version = rd.get_str(rb, rp, "uhd_version")
            r.readback.clock_source = rd.get_str(rb, rp, "clock_source")
            r.readback.time_source = rd.get_str(rb, rp, "time_source")
        r.require_readback = rd.get_bool(g, "radio", "require_readback")

    g = rd.group("timing", root)
    if g is not None:
        rd.reject_unknown_keys(g, "timing", _TIMING_KEYS)
        for name in _TIMING_KEYS:
            setattr(c.timing, name, rd.get_timed(g, "timing", name))

    g = rd.group("timeouts", root)
    if g is not None:
        rd.reject_unknown_keys(g, "timeouts", _TIMEOUT_KEYS)
        for name in _TIMEOUT_KEYS:
            setattr(c.timeouts, name, rd.get_timed(g, "timeouts", name))

    g = rd.group("calibration", root)
    if g is not None:
        rd.reject_unknown_keys(g, "calibration", _CALIBRATION_KEYS)
        k = c.calibration
        k.tx_link_delay = rd.get_dur(g, "calibration", "tx_link_delay_ns")
        k.rx_link_delay = rd.get_dur(g, "calibration", "rx_link_delay_ns")
        k.antenna_delay = rd.get_dur(g, "calibration", "antenna_delay_ns")
        k.cable_delay = rd.get_dur(g, "calibration", "cable_delay_ns")
        k.tx_link_delay_native_ticks = rd.get_i64(
            g, "calibration", "tx_link_delay_native_ticks")
        k.rx_link_delay_native_ticks = rd.get_i64(
            g, "calibration", "rx_link_delay_native_ticks")
        k.native_sample_rate_hz = rd.get_double(g, "calibration",
                                                "native_sample_rate_hz")
        k.link_delay_unit = rd.get_enum(g, "calibration", "link_delay_unit",
                                        TimeUnit)
        k.first_path_algorithm = rd.get_enum(g, "calibration",
                                             "first_path_algorithm",
                                             FirstPathAlgorithm)
        k.cfo_compensation = rd.get_enum(g, "calibration", "cfo_compensation",
                                         CompensationFlag)
        k.sfo_compensation = rd.get_enum(g, "calibration", "sfo_compensation",
                                         CompensationFlag)
        k.calibration_id = rd.get_str(g, "calibration", "calibration_id")
        rec = g.get("record")
        if rec is None:
            rd.bad("calibration.record", ConfigReason.MISSING_KEY,
                   "missing required group")
        elif not isinstance(rec, dict):
            rd.bad("calibration.record", ConfigReason.TYPE_MISMATCH,
                   "expected an object")
        else:
            rp = "calibration.record"
            rd.reject_unknown_keys(rec, rp, _RECORD_KEYS)
            k.record.calibration_id = rd.get_str(rec, rp, "calibration_id")
            k.record.device_serial = rd.get_str(rec, rp, "device_serial")
            k.record.channel = _u8(rd.get_i64(rec, rp, "channel"))
            k.record.native_sample_rate_hz = rd.get_double(
                rec, rp, "native_sample_rate_hz")
            k.record.profile_version = rd.get_str(rec, rp, "profile_version")
            k.record.gain_db = rd.get_opt_double(rec, rp, "gain_db")
            k.record.valid_until_monotonic_ns = rd.get_i64(
                rec, rp, "valid_until_monotonic_ns")
        k.applied_count = _u32(rd.get_i64(g, "calibration", "applied_count"))
        k.calibration_required = rd.get_bool(g, "calibration",
                                             "calibration_required")

    g = rd.group("diagnostics", root)
    if g is not None:
        rd.reject_unknown_keys(g, "diagnostics", _DIAGNOSTICS_KEYS)
        d = c.diagnostics
        d.cir_capture_enabled = rd.get_bool(g, "diagnostics", "cir_capture_enabled")
        d.cir_capture_max_bytes = _u64(rd.get_i64(
            g, "diagnostics", "cir_capture_max_bytes"))
        d.cir_capture_stride = _u32(rd.get_i64(g, "diagnostics", "cir_capture_stride"))
        d.short_iq_enabled = rd.get_bool(g, "diagnostics", "short_iq_enabled")
        d.short_iq_max_bytes = _u64(rd.get_i64(g, "diagnostics", "short_iq_max_bytes"))
        d.short_iq_stride = _u32(rd.get_i64(g, "diagnostics", "short_iq_stride"))
        d.raw_frame_dump = rd.get_bool(g, "diagnostics", "raw_frame_dump")
        d.raw_frame_max_bytes = _u64(rd.get_i64(g, "diagnostics", "raw_frame_max_bytes"))
        d.result_output_path = rd.get_str(g, "diagnostics", "result_output_path")
        d.result_queue_capacity = _u32(rd.get_i64(
            g, "diagnostics", "result_queue_capacity"))
        d.event_queue_capacity = _u32(rd.get_i64(
            g, "diagnostics", "event_queue_capacity"))
        d.stats_cadence = rd.get_timed(g, "diagnostics", "stats_cadence")
        d.io_on_realtime_thread = rd.get_bool(g, "diagnostics",
                                              "io_on_realtime_thread")

    # Even a document that failed to import is handed back so the caller can
    # see which groups were readable; the report is the authority.
    return (c, rd.report)


def to_json_string(cfg: TwrConfig, pretty: bool = True) -> str:
    """Serialise a config.  Raises ``ConfigJsonError`` on a non-finite double."""
    return json.dumps(to_json_dict(cfg), indent=2 if pretty else None,
                      ensure_ascii=True, allow_nan=False) + ("\n" if pretty else "")


def effective_to_json_dict(e: EffectiveConfig) -> Dict[str, Any]:
    """The snapshot as a document carrying requested, effective and reasons."""
    d: Dict[str, Any] = {
        "ok": bool(e.ok),
        "schema_version": e.schema_version,
        "profile_version": e.profile_version,
        "calibration_version": e.calibration_version,
        "config_hash": e.config_hash,
    }
    d.update(to_json_dict(e.effective))
    d["changes"] = [ch.to_dict() for ch in e.changes]
    d["violations"] = [{
        "field": v.field, "reason": str(v.reason), "status": str(v.status),
        "message": v.message, "requirement": v.requirement,
    } for v in e.validation.violations]
    d["quantised"] = {
        "tick_rate_hz": _finite_or_fail(e.tick_rate_hz, "quantised.tick_rate_hz"),
        "poll_start_ticks": _json_int(e.poll_start_ticks),
        "poll_to_response_ticks": _json_int(e.poll_to_response_ticks),
        "response_to_final_ticks": _json_int(e.response_to_final_ticks),
        "post_tx_rx_enable_ticks": _json_int(e.post_tx_rx_enable_ticks),
        "poll_to_response_effective_ns": _json_int(
            e.poll_to_response_effective.nanos()),
        "response_to_final_effective_ns": _json_int(
            e.response_to_final_effective.nanos()),
        "post_tx_rx_enable_effective_ns": _json_int(
            e.post_tx_rx_enable_effective.nanos()),
    }
    d["frame_budget"] = {
        "poll_bytes": _json_int(e.poll_bytes),
        "response_bytes": _json_int(e.response_bytes),
        "final_bytes": _json_int(e.final_bytes),
        "max_psdu_bytes": _json_int(e.max_psdu_bytes),
        "phr_bytes": _json_int(e.phr_bytes),
        "phr_coded_bits": _json_int(e.phr_coded_bits),
    }
    return d


def from_json_string(text: str) -> Tuple[TwrConfig, ValidationReport]:
    """Parse a config document.  Returns ``(config, report)``.

    A malformed document is a ``malformed_json`` violation, not an exception,
    so a bad config file produces the same machine-readable report as a bad
    value.
    """
    rd = _ConfigJsonReader()
    if not isinstance(text, str):
        rd.bad("", ConfigReason.MALFORMED_JSON, "config document must be text")
        return (TwrConfig(), rd.report)
    try:
        root = json.loads(text, parse_float=_JsonFloat, parse_constant=_reject_constant)
    except RecursionError:
        rd.bad("", ConfigReason.MALFORMED_JSON, "json nesting too deep")
        return (TwrConfig(), rd.report)
    except (ValueError, TypeError) as exc:
        rd.bad("", ConfigReason.MALFORMED_JSON, str(exc))
        return (TwrConfig(), rd.report)
    return from_json_dict(root)


# ===========================================================================
# 11. The shared differential corpus (M0.1, plan 2.A item 4)
# ===========================================================================
#
# C++ is the runtime authority of record.  A retained pure-Python validator is
# only defensible if it is pinned to the SAME profile data and compared ITEM BY
# ITEM against the SAME JSON corpus: accepted/rejected, the reason, and the
# effective values.  Comparing enums, or grepping source strings, does not count
# -- and neither does "the Python suite is green".  M0 shipped two
# implementations that had drifted badly: a 7-byte frame header against the
# codec's 14, a PHR rate typed as a payload data rate, a session id that did
# not fit the wire field, an accepted `phr_mode=extended`, and a
# `twr-config/1` schema that had become wrong.  Both suites were green.
#
# The corpus lives at testdata/twr/config_parity_corpus.json and is read by
# BOTH implementations:
#
#   * this module's loader, driven by gr-uwb/apps/test_twr_config.py, and
#   * gr-uwb/lib/qa_uwb_twr_config_parity.cc, which parses the same file with
#     the C++ header's own JSON parser and runs the same cases.
#
# SHAPE
# ------
#   {
#     "corpus_version": "twr-config-parity-corpus/1",
#     "config_schema_version": "twr-config/2",
#     "note": "...",
#     "non_finite_fields": [ "phy.center_frequency_hz", ... ],
#     "base": { "ss_initiator": {...}, "ss_responder": {...},
#               "ds_initiator": {...}, "ds_responder": {...} },
#     "cases": [
#       {"name": "...", "note": "...",
#        "base": "ss_initiator",
#        "patch": {"phy.channel": 9, "frame.frame_profile": "frame_v2"},
#        "non_finite_fields": ["rx.detection_threshold"],
#        "expect": {"accepted": false,
#                   "violations": ["phy.channel|unsupported|unsupported", ...],
#                   "import_ok": true,
#                   "import_violations": [...],
#                   "config_hash": "fnv1a64:...",
#                   "effective": {"poll_bytes": 16, ...}}}, ...]
#   }
#
# A patch is a map of dotted path -> value applied to a deep copy of the named
# base document; a null value DELETES the key, which is how the missing-key
# cases are written.  The paths are the SAME dotted field paths the validator
# reports, so a case reads as the sentence it tests.
#
# A violation is the triple "field|ConfigReason|ExchangeStatus" -- the
# machine-readable contract (REQ-ERR-01).  The human MESSAGE is deliberately
# NOT compared: the two implementations are allowed to word a rejection
# differently (the module docstring lists the divergences, because Python quotes
# the measured CSV cell where C++ quotes a summary), and pinning prose would turn
# a contract test into a string comparison.
#
# The config_hash IS compared, and it is the strongest single check in the
# suite: it is an fnv1a64 over the canonical "path=value" listing of EVERY
# field, so one wrong spelling, one missing entry in the listing or one field
# emitted in a different order changes it.

#: The corpus's own version, independent of the config schema it exercises.
CORPUS_VERSION = "twr-config-parity-corpus/1"
#: The config schema the corpus is written against.  A loader that disagrees
#: with this is testing the wrong thing, so both sides assert on it.
CORPUS_SCHEMA_VERSION = SCHEMA_VERSION
#: Where the corpus lives, relative to the repository root.
PARITY_CORPUS_PATH = os.path.join("testdata", "twr",
                                  "config_parity_corpus.json")

#: The three non-finite spellings a corpus may use, and the double each is.
_NON_FINITE: Dict[str, float] = {
    "nan": float("nan"),
    "inf": float("inf"),
    "-inf": float("-inf"),
}


def _split_path(path: str) -> List[Any]:
    """``"radio.peers[0].id"`` -> ``["radio", "peers", 0, "id"]``."""
    out: List[Any] = []
    for part in path.split("."):
        if not part:
            raise ValueError("empty path segment in %r" % path)
        if part.endswith("]") and "[" in part:
            head, _, idx = part.partition("[")
            out.append(head)
            out.append(int(idx[:-1]))
        else:
            out.append(part)
    return out


def apply_corpus_patch(base: Dict[str, Any],
                       patch: Dict[str, Any]) -> Dict[str, Any]:
    """A deep copy of ``base`` with ``patch`` applied.

    ``None`` deletes a key.  An unknown PARENT is an ERROR, not a silent no-op:
    a corpus case whose patch does not land would otherwise be testing the base
    configuration and reporting a false pass.  A new LEAF may be added, because
    "this key is not in the schema" is itself a case worth stating.
    """
    doc = _copy.deepcopy(base)
    for path, value in patch.items():
        parts = _split_path(path)
        node: Any = doc
        for part in parts[:-1]:
            if isinstance(part, int):
                if not isinstance(node, list) or part >= len(node):
                    raise KeyError("corpus patch path %r: no index %d"
                                   % (path, part))
                node = node[part]
            else:
                if not isinstance(node, dict) or part not in node:
                    raise KeyError("corpus patch path %r: no key %r"
                                   % (path, part))
                node = node[part]
        last = parts[-1]
        if isinstance(last, int):
            if not isinstance(node, list) or last >= len(node):
                raise KeyError("corpus patch path %r: no index %d"
                               % (path, last))
            node[last] = value
        else:
            if not isinstance(node, dict):
                raise KeyError("corpus patch path %r: parent is not an object"
                               % path)
            if value is None:
                if last not in node:
                    raise KeyError("corpus patch path %r: cannot delete absent key %r"
                                   % (path, last))
                del node[last]
            else:
                node[last] = value
    return doc


def expand_non_finite(value: Any) -> Tuple[Any, List[Tuple[str, float]]]:
    """Split a document into a strict-JSON copy and its non-finite requests.

    Strict JSON has no ``NaN`` / ``Infinity`` literal, and both readers refuse
    one, so a corpus cannot put a non-finite double in a document.  Instead it
    writes the STRING ``"$nan"`` / ``"$inf"`` / ``"$-inf"`` at a field path, and
    both loaders do the same two steps:

      1. take the document apart here -- the sentinel is removed and recorded
         as ``(dotted_path, value)``;
      2. import the remaining, strictly-serialisable document, then ASSIGN the
         non-finite value to the named config field.

    Step 2 is the only way a non-finite double can reach a validator, and it is
    done identically on both sides, so the case is genuinely mirrored.

    Returns ``(strict_copy, [(path, value), ...])``.
    """
    found: List[Tuple[str, float]] = []
    return (_strip_sentinels(value, found, ""), found)


def _strip_sentinels(node: Any, out: List[Tuple[str, float]],
                     prefix: str = "") -> Any:
    """Remove the non-finite sentinels from a document, recording them."""
    if isinstance(node, dict):
        copy: Dict[str, Any] = {}
        for k, v in node.items():
            child = ("%s.%s" % (prefix, k)) if prefix else str(k)
            if isinstance(v, str) and v in _NON_FINITE:
                out.append((child, _NON_FINITE[v]))
                continue
            copy[k] = _strip_sentinels(v, out, child)
        return copy
    if isinstance(node, list):
        items: List[Any] = []
        for i, v in enumerate(node):
            child = "%s[%d]" % (prefix, i)
            if isinstance(v, str) and v in _NON_FINITE:
                out.append((child, _NON_FINITE[v]))
                continue
            items.append(_strip_sentinels(v, out, child))
        return items
    return node


def set_config_double(cfg: TwrConfig, path: str, value: float) -> None:
    """Assign a (possibly non-finite) double to a dotted config field path.

    This is the ONLY way a non-finite double can reach a validator, because
    neither JSON reader produces one: both refuse the ``NaN`` / ``Infinity``
    literals outright.  The path names a real config field; an unknown path is
    an error, so a corpus case cannot silently do nothing.

    ``<timed field>.quantisation_hz`` is the one composite path: a timed field
    is a nested object, so the last segment is looked up on it rather than on
    the group that owns it, and its attribute is ``required_quantisation_hz``
    while its JSON key is ``quantisation_hz``.
    """
    parts = _split_path(path)
    node: Any = cfg
    for part in parts[:-1]:
        if isinstance(part, int):
            node = node[part]
        else:
            node = getattr(node, part)
    last = parts[-1]
    if isinstance(last, int):
        raise TypeError("cannot assign a double to the list element %r" % path)
    if last == "quantisation_hz" and isinstance(node, TimedField):
        node.required_quantisation_hz = float(value)
        return
    if not hasattr(node, last):
        raise AttributeError("no config field %r (path %r)" % (last, path))
    current = getattr(node, last)
    if current is not None and not isinstance(current, (int, float)):
        raise TypeError("config field %r is not a float" % path)
    setattr(node, last, float(value))


def default_parity_corpus_path() -> str:
    """The corpus, found from the repository root above this file.

    ``gr-uwb/python/uwb/twr_config.py`` -> ``<repo>/testdata/twr/...``.  No
    environment variable, no cwd dependence: the corpus is a repository
    artifact and is looked up like one.
    """
    here = os.path.dirname(os.path.abspath(__file__))   # gr-uwb/python/uwb
    repo = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    return os.path.join(repo, PARITY_CORPUS_PATH)


def load_parity_corpus(path: Optional[str] = None) -> Dict[str, Any]:
    """Read and sanity-check the shared corpus.

    Raises rather than returning an empty corpus: a differential test that
    silently found no cases would pass without having compared anything, which
    is the exact failure mode a parity test exists to prevent.
    """
    if path is None:
        path = default_parity_corpus_path()
    with open(path, "r", encoding="utf-8") as handle:
        corpus = json.load(handle)
    for key in ("corpus_version", "config_schema_version", "base", "cases"):
        if key not in corpus:
            raise ValueError("corpus %s has no %r" % (path, key))
    if corpus["corpus_version"] != CORPUS_VERSION:
        raise ValueError("corpus %s is version %r, this loader implements %r"
                         % (path, corpus["corpus_version"], CORPUS_VERSION))
    if corpus["config_schema_version"] != SCHEMA_VERSION:
        raise ValueError(
            "corpus %s targets config schema %r but this build implements %r"
            % (path, corpus["config_schema_version"], SCHEMA_VERSION))
    return corpus


def run_parity_case(case: Dict[str, Any], base: Dict[str, Any],
                    caps: Optional[Capabilities] = None
                    ) -> Dict[str, Any]:
    """Run ONE corpus case and return its observable outcome.

    The returned mapping is exactly what the C++ suite reports for the same
    case, so the two can be compared field for field: the import report, the
    validation report, the effective values and the config hash.
    """
    table = caps if caps is not None else capabilities()
    doc = apply_corpus_patch(base, case.get("patch", {}))
    strict, non_finite = expand_non_finite(doc)
    # A case may ALSO name the float fields it wants poisoned in a
    # `non_finite_fields` list.  That is the portable spelling the C++ loader
    # reads; the in-document "$nan" sentinel above is the same thing written
    # inline, and both expand to the identical assignment.
    for item in case.get("non_finite_fields", []):
        non_finite.append((str(item), _NON_FINITE["nan"]))
    text = json.dumps(strict, allow_nan=False)
    cfg, import_report = from_json_string(text)
    for path, value in non_finite:
        set_config_double(cfg, path, value)

    report = validate(cfg, table)
    eff = effective_config(cfg, table)
    return {
        "import_ok": import_report.ok(),
        "import_violations": _violation_triples(import_report),
        "accepted": report.ok(),
        "violations": _violation_triples(report),
        "effective": _effective_summary(eff),
        "config_hash": eff.config_hash,
    }


def _violation_triples(report: ValidationReport) -> List[str]:
    """``["field|reason|status", ...]`` -- the machine-readable part only.

    The human MESSAGE is deliberately NOT part of the comparison; see the
    section comment above.
    """
    return ["%s|%s|%s" % (v.field, v.reason, v.status) for v in report.violations]


def _effective_summary(e: EffectiveConfig) -> Dict[str, Any]:
    """The effective values both implementations must agree on."""
    return {
        "ok": bool(e.ok),
        "schema_version": e.schema_version,
        "profile_version": e.profile_version,
        "calibration_version": e.calibration_version,
        # A double is rendered the C++ way (%.17g), because the corpus stores
        # the text the C++ side produced.
        "tick_rate_hz": double_to_text(e.tick_rate_hz),
        "poll_bytes": e.poll_bytes,
        "response_bytes": e.response_bytes,
        "final_bytes": e.final_bytes,
        "max_psdu_bytes": e.max_psdu_bytes,
        "max_timestamp_count": e.max_timestamp_count,
        "phr_bytes": e.phr_bytes,
        "phr_coded_bits": e.phr_coded_bits,
        "poll_start_ticks": e.poll_start_ticks,
        "poll_to_response_ticks": e.poll_to_response_ticks,
        "response_to_final_ticks": e.response_to_final_ticks,
        "post_tx_rx_enable_ticks": e.post_tx_rx_enable_ticks,
        "poll_start_effective_ns": e.poll_start_effective.nanos(),
        "poll_to_response_effective_ns": e.poll_to_response_effective.nanos(),
        "response_to_final_effective_ns": e.response_to_final_effective.nanos(),
        "post_tx_rx_enable_effective_ns": e.post_tx_rx_enable_effective.nanos(),
    }


# ===========================================================================
# End of the TWR configuration layer
# ===========================================================================
