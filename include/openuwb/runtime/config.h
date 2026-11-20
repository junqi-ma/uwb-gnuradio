/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unified configuration vocabulary for the runtime domains.
 *
 * Phase 2 scope: this header reserves the shared shape and the naming
 * convention only.  It deliberately carries no fields beyond the kind and a
 * profile name, because every real field is still owned by a concrete config
 * that has been validated and tested (LiveRxConfig for the live receiver,
 * twr::TwrConfig for ranging, the radar cores for sensing).  Adding speculative
 * fields here would create a second, weaker configuration surface and invite
 * exactly the "two validators" bug the refactor is meant to remove.
 *
 * Rule for the next phase: a domain config either IS one of these structs (and
 * then uses the domain's existing validator), or it converts to one explicitly.
 * Never parse into a ConfigCommon alone.
 */
#ifndef INCLUDED_OPENUWB_RUNTIME_CONFIG_H
#define INCLUDED_OPENUWB_RUNTIME_CONFIG_H

#include <cstdint>
#include <string>

namespace openuwb {
namespace runtime {

/// Which domain a configuration belongs to.
enum class ConfigKind : std::uint8_t {
    Rx = 0,
    Tx = 1,
    Twr = 2,
    Radar = 3,
};

constexpr const char* config_kind_name(ConfigKind kind) noexcept
{
    switch (kind) {
    case ConfigKind::Rx:
        return "rx";
    case ConfigKind::Tx:
        return "tx";
    case ConfigKind::Twr:
        return "twr";
    case ConfigKind::Radar:
        return "radar";
    }
    return "unknown";
}

/// Fields every domain config shares.  Keep this minimal on purpose; see the
/// file comment for why it is not a base class with speculative members.
struct ConfigCommon {
    ConfigKind kind = ConfigKind::Rx;
    /// Opaque profile/selector name as understood by that domain, e.g. a PHY
    /// profile id or a ranging scenario.  Empty means "domain default".
    std::string profile;
};

/// Receive configuration.
/// TODO(openuwb-arch-phase3): carry the fields that today live in
/// apps/live_rx_config.h (LiveRxConfig) once the app grows a second consumer.
struct RxConfig {
    ConfigCommon common{ ConfigKind::Rx, {} };
};

/// Transmit configuration.  Reserved: the native chain has no TX app yet.
struct TxConfig {
    ConfigCommon common{ ConfigKind::Tx, {} };
};

/// Two-way-ranging configuration.  The live type is twr::TwrConfig; this is
/// the domain selector used by schedulers, not a replacement for it.
struct TwrConfig {
    ConfigCommon common{ ConfigKind::Twr, {} };
};

/// Radar / sensing configuration.
struct RadarConfig {
    ConfigCommon common{ ConfigKind::Radar, {} };
};

} // namespace runtime
} // namespace openuwb

#endif /* INCLUDED_OPENUWB_RUNTIME_CONFIG_H */
