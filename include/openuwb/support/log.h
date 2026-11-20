/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Minimal header-only diagnostic logger for the native runtime.
 *
 * SCOPE: apps / runtime only.  Never include this header from a pure DSP
 * translation unit (uwb_core); those sources must stay I/O-free and report
 * diagnostics through their result structs instead.
 *
 * Dependency policy: C++17 standard library only.  Everything is `inline`
 * (or a function-local static), so the header is safe in any number of TUs.
 *
 * Level selection order:
 *   1. UWB_LOG_LEVEL       (pre-existing name; wins when both are set)
 *   2. OPENUWB_LOG_LEVEL   (new/global name, used as a fallback)
 *   3. Info                (built-in default)
 * The environment is read exactly once, on first use; no init() call is
 * required.  set_level() / get_level() override and query it afterwards.
 *
 * Sink contract: the sink runs while the logger mutex is held.  A sink must
 * therefore never log recursively and must never block.
 */

#ifndef INCLUDED_GR_UWB_LOG_H
#define INCLUDED_GR_UWB_LOG_H

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace openuwb {
namespace log {

enum class Level { Off = 0, Error = 1, Warn = 2, Info = 3, Debug = 4 };

/// Case-insensitive level parser.  Accepts "off", "error", "warn", "warning",
/// "info" and "debug"; anything else (including an empty string) yields Info.
/// Pure: performs no I/O and touches no global state, so it is unit-testable.
inline Level parse_level(std::string_view text)
{
    std::string lowered;
    lowered.reserve(text.size());
    for (char c : text) {
        if (c >= 'A' && c <= 'Z') {
            lowered.push_back(static_cast<char>(c - 'A' + 'a'));
        } else {
            lowered.push_back(c);
        }
    }

    if (lowered == "off") {
        return Level::Off;
    }
    if (lowered == "error") {
        return Level::Error;
    }
    if (lowered == "warn" || lowered == "warning") {
        return Level::Warn;
    }
    if (lowered == "info") {
        return Level::Info;
    }
    if (lowered == "debug") {
        return Level::Debug;
    }
    return Level::Info;
}

/// Read the level from the environment (UWB_LOG_LEVEL first, then
/// OPENUWB_LOG_LEVEL), falling back to Info.  The logger calls this exactly
/// once, through a function-local static, so later environment changes do not
/// affect the process (use set_level() for runtime changes).
inline Level level_from_env()
{
    if (const char* v = std::getenv("UWB_LOG_LEVEL"); v != nullptr && v[0] != '\0') {
        return parse_level(v);
    }
    if (const char* v = std::getenv("OPENUWB_LOG_LEVEL"); v != nullptr && v[0] != '\0') {
        return parse_level(v);
    }
    return Level::Info;
}

namespace detail {

/// Process-wide level.  Thread-safe first-use initialisation from the
/// environment; relaxed atomics afterwards (only the level itself is shared
/// state, and a stale read merely delays a level change by one message).
inline std::atomic<int>& level_storage() noexcept
{
    static std::atomic<int> level = [] {
        return static_cast<int>(level_from_env());
    }();
    return level;
}

inline std::mutex& log_mutex()
{
    static std::mutex mutex;
    return mutex;
}

inline std::function<void(std::string_view)>& sink_storage()
{
    static std::function<void(std::string_view)> sink;
    return sink;
}

inline std::string_view level_name(Level level) noexcept
{
    switch (level) {
    case Level::Off:
        return "OFF";
    case Level::Error:
        return "ERR";
    case Level::Warn:
        return "WARN";
    case Level::Info:
        return "INFO";
    case Level::Debug:
        return "DEBUG";
    }
    return "INFO";
}

/// Default sink: one fwrite() of the complete line.  stderr is unbuffered, so
/// the line reaches the terminal as a single write even when other threads are
/// writing concurrently.
inline void default_sink(std::string_view line)
{
    std::fwrite(line.data(), 1, line.size(), stderr);
}

} // namespace detail

using SinkFn = std::function<void(std::string_view)>;

/// Replace the output sink.  Passing an empty function restores the default
/// sink (one unbuffered fwrite of the whole line to stderr).
///
/// The sink is invoked while the logger mutex is held: it must not log
/// recursively and must not block.
inline void set_sink(SinkFn sink)
{
    std::lock_guard<std::mutex> lock(detail::log_mutex());
    detail::sink_storage() = std::move(sink);
}

inline void set_level(Level level) noexcept
{
    detail::level_storage().store(static_cast<int>(level),
                                  std::memory_order_relaxed);
}

inline Level get_level() noexcept
{
    return static_cast<Level>(
        detail::level_storage().load(std::memory_order_relaxed));
}

/// True when a message at `level` would be emitted.  Single relaxed load; Off
/// is never enabled (neither as the current level nor as the queried level).
inline bool enabled(Level level) noexcept
{
    const int current = detail::level_storage().load(std::memory_order_relaxed);
    if (level == Level::Off || current == static_cast<int>(Level::Off)) {
        return false;
    }
    return static_cast<int>(level) <= current;
}

/// Format and write one line: "[WARN][tag] message\n".  The caller's message
/// needs no trailing newline.  No-op when the level is filtered out.
/// Serialised through a function-local mutex.
inline void emit(Level level, std::string_view tag, std::string_view message)
{
    if (!enabled(level)) {
        return;
    }

    std::string line;
    line.reserve(tag.size() + message.size() + 16);
    line.push_back('[');
    line += detail::level_name(level);
    line += "][";
    line += tag;
    line += "] ";
    line += message;
    line.push_back('\n');

    std::lock_guard<std::mutex> lock(detail::log_mutex());
    SinkFn& sink = detail::sink_storage();
    if (sink) {
        sink(line);
    } else {
        detail::default_sink(line);
    }
}

} // namespace log
} // namespace openuwb

/// ostream-style logging macros with strict lazy evaluation: when the level is
/// filtered out, no argument expression is evaluated at all.
#define UWB_LOG_ERR(tag, ...)                                                  \
    do {                                                                       \
        if (::openuwb::log::enabled(::openuwb::log::Level::Error)) {           \
            std::ostringstream uwb_log_oss;                                    \
            uwb_log_oss << __VA_ARGS__;                                        \
            ::openuwb::log::emit(::openuwb::log::Level::Error, (tag),          \
                                 uwb_log_oss.str());                           \
        }                                                                      \
    } while (false)

#define UWB_LOG_WARN(tag, ...)                                                 \
    do {                                                                       \
        if (::openuwb::log::enabled(::openuwb::log::Level::Warn)) {            \
            std::ostringstream uwb_log_oss;                                    \
            uwb_log_oss << __VA_ARGS__;                                        \
            ::openuwb::log::emit(::openuwb::log::Level::Warn, (tag),           \
                                 uwb_log_oss.str());                           \
        }                                                                      \
    } while (false)

#define UWB_LOG_INFO(tag, ...)                                                 \
    do {                                                                       \
        if (::openuwb::log::enabled(::openuwb::log::Level::Info)) {            \
            std::ostringstream uwb_log_oss;                                    \
            uwb_log_oss << __VA_ARGS__;                                        \
            ::openuwb::log::emit(::openuwb::log::Level::Info, (tag),           \
                                 uwb_log_oss.str());                           \
        }                                                                      \
    } while (false)

#define UWB_LOG_DEBUG(tag, ...)                                                \
    do {                                                                       \
        if (::openuwb::log::enabled(::openuwb::log::Level::Debug)) {           \
            std::ostringstream uwb_log_oss;                                    \
            uwb_log_oss << __VA_ARGS__;                                        \
            ::openuwb::log::emit(::openuwb::log::Level::Debug, (tag),          \
                                 uwb_log_oss.str());                           \
        }                                                                      \
    } while (false)

/* Thin aliases for the OPENUWB_* spelling. */
#define OPENUWB_LOG_ERR(tag, ...) UWB_LOG_ERR(tag, __VA_ARGS__)
#define OPENUWB_LOG_WARN(tag, ...) UWB_LOG_WARN(tag, __VA_ARGS__)
#define OPENUWB_LOG_INFO(tag, ...) UWB_LOG_INFO(tag, __VA_ARGS__)
#define OPENUWB_LOG_DEBUG(tag, ...) UWB_LOG_DEBUG(tag, __VA_ARGS__)

#endif /* INCLUDED_GR_UWB_LOG_H */
