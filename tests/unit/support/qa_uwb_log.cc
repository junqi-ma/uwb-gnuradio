/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Unit tests for the header-only logger (openuwb::log).
 *
 * Every assertion drives the logger through a capturing sink, so the test run
 * never writes diagnostics to stderr (the one exception is exercised with fd 2
 * redirected to /dev/null on purpose, to prove set_sink({}) really detaches the
 * previous sink).
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_log.h>

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

namespace ulog = ::openuwb::log;
using ulog::Level;

/// Thread-safe collection of captured lines.
struct Captured {
    std::mutex mutex;
    std::vector<std::string> lines;

    std::size_t size()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lines.size();
    }

    std::string at(std::size_t index)
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lines.at(index);
    }

    std::vector<std::string> snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lines;
    }
};

/// Restores the process sink and level after a case, so case order cannot
/// matter.  The sink is restored to the default one, which every case replaces
/// with its own capture sink before emitting anything.
struct LogStateGuard {
    Level saved_level = ulog::get_level();

    ~LogStateGuard()
    {
        ulog::set_sink({});
        ulog::set_level(saved_level);
    }
};

void capture_into(const std::shared_ptr<Captured>& sink)
{
    ulog::set_sink([sink](std::string_view line) {
        std::lock_guard<std::mutex> lock(sink->mutex);
        sink->lines.emplace_back(line);
    });
}

} // namespace

// Runs first on purpose: the logger must have picked up the environment on its
// very first use, without any init() call and before any set_level() here.
BOOST_AUTO_TEST_CASE(test_initial_level_from_environment)
{
    const Level initial = ulog::get_level();
    const bool env_unset = std::getenv("UWB_LOG_LEVEL") == nullptr &&
                           std::getenv("OPENUWB_LOG_LEVEL") == nullptr;

    if (env_unset) {
        // Built-in default.
        BOOST_CHECK(initial == Level::Info);
    } else {
        // Whatever the environment says, the initial value must be its parse.
        BOOST_CHECK(initial == ulog::level_from_env());
    }

    // The documented default survives an unset/blank environment.
    BOOST_CHECK(ulog::level_from_env() == Level::Info ||
                std::getenv("UWB_LOG_LEVEL") != nullptr ||
                std::getenv("OPENUWB_LOG_LEVEL") != nullptr);
}

BOOST_AUTO_TEST_CASE(test_level_enum_values)
{
    BOOST_CHECK_EQUAL(static_cast<int>(Level::Off), 0);
    BOOST_CHECK_EQUAL(static_cast<int>(Level::Error), 1);
    BOOST_CHECK_EQUAL(static_cast<int>(Level::Warn), 2);
    BOOST_CHECK_EQUAL(static_cast<int>(Level::Info), 3);
    BOOST_CHECK_EQUAL(static_cast<int>(Level::Debug), 4);
}

BOOST_AUTO_TEST_CASE(test_parse_level_spellings)
{
    // Documented spellings, case-insensitive.
    BOOST_CHECK(ulog::parse_level("off") == Level::Off);
    BOOST_CHECK(ulog::parse_level("OFF") == Level::Off);
    BOOST_CHECK(ulog::parse_level("Off") == Level::Off);
    BOOST_CHECK(ulog::parse_level("error") == Level::Error);
    BOOST_CHECK(ulog::parse_level("ERROR") == Level::Error);
    BOOST_CHECK(ulog::parse_level("Error") == Level::Error);
    BOOST_CHECK(ulog::parse_level("warn") == Level::Warn);
    BOOST_CHECK(ulog::parse_level("WARN") == Level::Warn);
    BOOST_CHECK(ulog::parse_level("warning") == Level::Warn);
    BOOST_CHECK(ulog::parse_level("Warning") == Level::Warn);
    BOOST_CHECK(ulog::parse_level("WARNING") == Level::Warn);
    BOOST_CHECK(ulog::parse_level("info") == Level::Info);
    BOOST_CHECK(ulog::parse_level("INFO") == Level::Info);
    BOOST_CHECK(ulog::parse_level("Info") == Level::Info);
    BOOST_CHECK(ulog::parse_level("debug") == Level::Debug);
    BOOST_CHECK(ulog::parse_level("DEBUG") == Level::Debug);
    BOOST_CHECK(ulog::parse_level("Debug") == Level::Debug);

    // Anything else falls back to Info.
    BOOST_CHECK(ulog::parse_level("") == Level::Info);
    BOOST_CHECK(ulog::parse_level("garbage") == Level::Info);
    BOOST_CHECK(ulog::parse_level("warnings") == Level::Info);
    BOOST_CHECK(ulog::parse_level(" info") == Level::Info);
    BOOST_CHECK(ulog::parse_level("3") == Level::Info);
}

BOOST_AUTO_TEST_CASE(test_level_filtering)
{
    LogStateGuard guard;
    auto sink = std::make_shared<Captured>();
    capture_into(sink);

    // Off: nothing at all is enabled, including Error.
    ulog::set_level(Level::Off);
    BOOST_CHECK(!ulog::enabled(Level::Off));
    BOOST_CHECK(!ulog::enabled(Level::Error));
    BOOST_CHECK(!ulog::enabled(Level::Warn));
    BOOST_CHECK(!ulog::enabled(Level::Info));
    BOOST_CHECK(!ulog::enabled(Level::Debug));

    // Error: only Error.
    ulog::set_level(Level::Error);
    BOOST_CHECK(ulog::enabled(Level::Error));
    BOOST_CHECK(!ulog::enabled(Level::Warn));
    BOOST_CHECK(!ulog::enabled(Level::Info));
    BOOST_CHECK(!ulog::enabled(Level::Debug));

    // Warn: Error and Warn.
    ulog::set_level(Level::Warn);
    BOOST_CHECK(ulog::enabled(Level::Error));
    BOOST_CHECK(ulog::enabled(Level::Warn));
    BOOST_CHECK(!ulog::enabled(Level::Info));
    BOOST_CHECK(!ulog::enabled(Level::Debug));

    // Info (the documented default): Error, Warn, Info.
    ulog::set_level(Level::Info);
    BOOST_CHECK(ulog::enabled(Level::Error));
    BOOST_CHECK(ulog::enabled(Level::Warn));
    BOOST_CHECK(ulog::enabled(Level::Info));
    BOOST_CHECK(!ulog::enabled(Level::Debug));

    // Debug: everything (Off is never a message level).
    ulog::set_level(Level::Debug);
    BOOST_CHECK(ulog::enabled(Level::Error));
    BOOST_CHECK(ulog::enabled(Level::Warn));
    BOOST_CHECK(ulog::enabled(Level::Info));
    BOOST_CHECK(ulog::enabled(Level::Debug));
    BOOST_CHECK(!ulog::enabled(Level::Off));

    // The same filtering must hold for actual emissions.
    ulog::set_level(Level::Info);
    UWB_LOG_ERR("filter", "e");
    UWB_LOG_WARN("filter", "w");
    UWB_LOG_INFO("filter", "i");
    UWB_LOG_DEBUG("filter", "d");
    BOOST_CHECK_EQUAL(sink->size(), std::size_t(3));
    BOOST_CHECK_EQUAL(sink->at(0), std::string("[ERR][filter] e\n"));
    BOOST_CHECK_EQUAL(sink->at(1), std::string("[WARN][filter] w\n"));
    BOOST_CHECK_EQUAL(sink->at(2), std::string("[INFO][filter] i\n"));
}

BOOST_AUTO_TEST_CASE(test_lazy_evaluation)
{
    LogStateGuard guard;
    auto sink = std::make_shared<Captured>();
    capture_into(sink);

    int counter = 0;

    // Filtered out: the argument expression must not run at all.
    ulog::set_level(Level::Error);
    UWB_LOG_WARN("lazy", "counter=" << ++counter);
    UWB_LOG_INFO("lazy", "counter=" << ++counter);
    UWB_LOG_DEBUG("lazy", "counter=" << ++counter);
    BOOST_CHECK_EQUAL(counter, 0);
    BOOST_CHECK_EQUAL(sink->size(), std::size_t(0));

    // Enabled: exactly one evaluation, at the level boundary.
    ulog::set_level(Level::Info);
    UWB_LOG_INFO("lazy", "counter=" << ++counter);
    BOOST_CHECK_EQUAL(counter, 1);
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(1));
    BOOST_CHECK_EQUAL(sink->at(0), std::string("[INFO][lazy] counter=1\n"));

    // Enabled Error while at Info also evaluates its argument.
    UWB_LOG_ERR("lazy", "counter=" << ++counter);
    BOOST_CHECK_EQUAL(counter, 2);

    // And a filtered Debug again evaluates nothing.
    UWB_LOG_DEBUG("lazy", "counter=" << ++counter);
    BOOST_CHECK_EQUAL(counter, 2);
}

BOOST_AUTO_TEST_CASE(test_sink_line_format)
{
    LogStateGuard guard;
    auto sink = std::make_shared<Captured>();
    capture_into(sink);
    ulog::set_level(Level::Debug);

    // Exact byte layout, including the single trailing newline.
    UWB_LOG_WARN("tag", "msg");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(1));
    const std::string warn_line = sink->at(0);
    BOOST_CHECK_EQUAL(warn_line, std::string("[WARN][tag] msg\n"));
    BOOST_CHECK(warn_line.size() >= 2);
    BOOST_CHECK_EQUAL(warn_line.back(), '\n');
    BOOST_CHECK(warn_line.find('\n') == warn_line.size() - 1); // exactly one, at the end

    // Upper-case level names.
    UWB_LOG_ERR("tag", "msg");
    UWB_LOG_INFO("tag", "msg");
    UWB_LOG_DEBUG("tag", "msg");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(4));
    BOOST_CHECK_EQUAL(sink->at(1), std::string("[ERR][tag] msg\n"));
    BOOST_CHECK_EQUAL(sink->at(2), std::string("[INFO][tag] msg\n"));
    BOOST_CHECK_EQUAL(sink->at(3), std::string("[DEBUG][tag] msg\n"));

    // ostream-style formatting: spaces, integers, floating point and several
    // << pieces all land in the message body verbatim.
    UWB_LOG_INFO("fmt", "n=" << 42 << " pi=" << 3.5 << " hello world");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(5));
    BOOST_CHECK_EQUAL(sink->at(4),
                      std::string("[INFO][fmt] n=42 pi=3.5 hello world\n"));

    // A message built from several pieces keeps every piece.
    UWB_LOG_WARN("fmt", "a=" << 1 << ", b=" << 2);
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(6));
    BOOST_CHECK_EQUAL(sink->at(5), std::string("[WARN][fmt] a=1, b=2\n"));

    // Empty message: prefix, one space, newline.
    UWB_LOG_INFO("fmt", "");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(7));
    BOOST_CHECK_EQUAL(sink->at(6), std::string("[INFO][fmt] \n"));

    // A tag supplied as std::string is used as-is.
    const std::string tag = "dynamic_tag";
    UWB_LOG_INFO(tag, "x");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(8));
    BOOST_CHECK_EQUAL(sink->at(7), std::string("[INFO][dynamic_tag] x\n"));
}

BOOST_AUTO_TEST_CASE(test_empty_sink_restores_default)
{
    LogStateGuard guard;
    ulog::set_level(Level::Info);

    auto first = std::make_shared<Captured>();
    capture_into(first);
    UWB_LOG_INFO("sink", "one");
    BOOST_REQUIRE_EQUAL(first->size(), std::size_t(1));

    // Drop the capturing sink.  The next message must go to the default sink
    // (stderr); redirect fd 2 to /dev/null so the test stays silent while still
    // proving that the old sink is gone.
    ulog::set_sink({});

    const int saved_fd = ::dup(STDERR_FILENO);
    BOOST_REQUIRE(saved_fd >= 0);
    const int null_fd = ::open("/dev/null", O_WRONLY);
    BOOST_REQUIRE(null_fd >= 0);
    BOOST_REQUIRE(::dup2(null_fd, STDERR_FILENO) >= 0);

    UWB_LOG_INFO("sink", "two");
    std::fflush(stderr);

    BOOST_REQUIRE(::dup2(saved_fd, STDERR_FILENO) >= 0);
    ::close(saved_fd);
    ::close(null_fd);

    // The default sink produced output, but the removed capture sink saw
    // nothing.
    BOOST_CHECK_EQUAL(first->size(), std::size_t(1));

    // The logger remains usable: a newly installed sink receives new lines.
    auto second = std::make_shared<Captured>();
    capture_into(second);
    UWB_LOG_INFO("sink", "three");
    BOOST_REQUIRE_EQUAL(second->size(), std::size_t(1));
    BOOST_CHECK_EQUAL(second->at(0), std::string("[INFO][sink] three\n"));
    BOOST_CHECK_EQUAL(first->size(), std::size_t(1));
}

BOOST_AUTO_TEST_CASE(test_filtered_emit_does_not_call_sink)
{
    LogStateGuard guard;
    auto sink = std::make_shared<Captured>();
    capture_into(sink);

    ulog::set_level(Level::Error);

    // Both the macro path and the direct emit() path must be silent.
    int counter = 0;
    UWB_LOG_INFO("quiet", "counter=" << ++counter);
    UWB_LOG_DEBUG("quiet", "counter=" << ++counter);
    ulog::emit(Level::Info, "quiet", "direct call");
    ulog::emit(Level::Off, "quiet", "off");
    BOOST_CHECK_EQUAL(counter, 0);
    BOOST_CHECK_EQUAL(sink->size(), std::size_t(0));

    // The one enabled level does reach the sink.
    UWB_LOG_ERR("quiet", "visible");
    BOOST_REQUIRE_EQUAL(sink->size(), std::size_t(1));
    BOOST_CHECK_EQUAL(sink->at(0), std::string("[ERR][quiet] visible\n"));
}

BOOST_AUTO_TEST_CASE(test_thread_safety)
{
    LogStateGuard guard;
    auto sink = std::make_shared<Captured>();
    capture_into(sink);
    ulog::set_level(Level::Debug);

    constexpr int kThreads = 8;
    constexpr int kPerThread = 200;

    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([t] {
            for (int i = 0; i < kPerThread; ++i) {
                UWB_LOG_WARN("mt", "thread " << t << " msg " << i);
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }

    const std::vector<std::string> lines = sink->snapshot();
    BOOST_REQUIRE_EQUAL(lines.size(), std::size_t(kThreads * kPerThread));

    std::unordered_set<std::string> seen;
    for (const std::string& line : lines) {
        // Complete, single line: prefix, no embedded newline, newline last.
        BOOST_REQUIRE(line.rfind("[WARN][mt] ", 0) == 0);
        BOOST_REQUIRE(!line.empty());
        BOOST_CHECK_EQUAL(line.back(), '\n');
        BOOST_CHECK(line.find('\n') == line.size() - 1);

        std::istringstream in(line.substr(std::string("[WARN][mt] ").size()));
        std::string word_thread;
        std::string word_msg;
        int t = -1;
        int i = -1;
        BOOST_REQUIRE(static_cast<bool>(in >> word_thread >> t >> word_msg >> i));
        BOOST_REQUIRE_EQUAL(word_thread, std::string("thread"));
        BOOST_REQUIRE_EQUAL(word_msg, std::string("msg"));
        BOOST_CHECK(t >= 0 && t < kThreads);
        BOOST_CHECK(i >= 0 && i < kPerThread);
        // No truncation/interleaving: every (thread, message) pair appears once.
        BOOST_CHECK(seen.insert(word_thread + ":" + std::to_string(t) + ":" +
                                std::to_string(i))
                        .second);
    }
    BOOST_CHECK_EQUAL(seen.size(), std::size_t(kThreads * kPerThread));
}
