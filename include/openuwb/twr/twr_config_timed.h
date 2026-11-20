/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * TWR configuration, split by responsibility (Phase 2).
 * The aggregate entry point is <openuwb/twr/twr_config.h>; these pieces are
 * ordered, not independent: each one includes its predecessor.
 */
#ifndef INCLUDED_OPENUWB_TWR_CONFIG_TIMED_H
#define INCLUDED_OPENUWB_TWR_CONFIG_TIMED_H

#include <openuwb/twr/twr_config_reasons.h>
namespace gr {
namespace uwb {
namespace twr {

// ===========================================================================
// 2. Timed fields (reference event + marker + unit + quantisation)
// ===========================================================================

// A duration that knows what it is measured FROM, in WHICH clock, with which
// RF marker, and to WHAT resolution the device must express it (REQ-API-02).
struct TimedField {
    Duration value{0};
    TimeDomain domain = TimeDomain::Unspecified;
    TimeReferenceEvent reference = TimeReferenceEvent::HostMonotonic;
    // Required iff domain == DeviceTicks; must be ABSENT for MonotonicHost
    // (there is no RF marker on a host monotonic clock).
    Opt<TimestampMarker> marker;
    // Device tick rate the value is quantised to, or 1e9 for integer ns.
    double required_quantisation_hz = 0.0;
    // Largest quantisation error the caller tolerates, in ns.  1 is exact to
    // the nearest tick for every rate this build supports; 0 demands an exact
    // nanosecond and is therefore rejected (TimingPrecisionInsufficient).
    int64_t max_quantisation_error_ns = 1;
    // Free text: how the number was obtained / converted.  REQ-API-02
    // requires the conversion result to be recorded when a delay is
    // expressed "after the end of the frame".
    std::string note;

    bool operator==(const TimedField& o) const
    {
        return value == o.value && domain == o.domain && reference == o.reference &&
               marker == o.marker &&
                   required_quantisation_hz == o.required_quantisation_hz &&
               max_quantisation_error_ns == o.max_quantisation_error_ns &&
                   note == o.note;
    }
    bool operator!=(const TimedField& o) const { return !(*this == o); }

    std::string to_text() const
    {
        std::string s = twr_duration_to_text(value);
        s += "@";
        s += time_domain_to_string(domain);
        s += "/";
        s += time_reference_event_to_string(reference);
        s += "/";
        s += marker.has() ? timestamp_marker_to_string(marker.value()) : "(no marker)";
        s += "/q=";
        s += twr_double_to_text(required_quantisation_hz);
        s += "/e=";
        s += twr_int_to_text(max_quantisation_error_ns);
        s += "ns";
        return s;
    }
};

inline std::string timed_field_to_text(const TimedField& f) { return f.to_text(); }

// Round to the nearest device tick and report whether the value survived the
// trip.  This is the single place a Duration becomes device ticks; adapters
// convert from here, never at the call site (REQ-API-02).
inline int64_t quantise_duration(const Duration& d, double rate_hz, bool& ok)
{
    int64_t ticks = d.ticks_at(rate_hz, ok);
    if (!ok)
        return 0;
    // Guard the reverse direction too: the tick-quantised value must still be
    // a finite, representable Duration.
    Duration back;
    if (!Duration::from_ticks(ticks, rate_hz, back)) {
        ok = false;
        return 0;
    }
    return ticks;
}

} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_OPENUWB_TWR_CONFIG_TIMED_H */
