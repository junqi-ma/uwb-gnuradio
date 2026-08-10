/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Locks production defaults in uwb_defaults.h to the GRC YAML default strings
 * so block factory defaults and GRC UI cannot drift silently.
 */

#include <boost/test/unit_test.hpp>

#include <gnuradio/uwb/uwb_defaults.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace {

std::string read_file(const std::string& path)
{
    std::ifstream in(path);
    BOOST_REQUIRE_MESSAGE(in.good(), "cannot open " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Match YAML default: 'VALUE' on a line for a given parameter id.
// Supports both multi-line (-   id: foo ... default: 'x') and
// single-line compact (- {id: foo, ..., default: 'x'}) forms.
bool yaml_default_equals(const std::string& yaml,
                         const std::string& id,
                         const std::string& expected)
{
    // Compact form: id: name, ... default: 'value'
    {
        std::regex re("id:\\s*" + id +
                      "[^\\n]*default:\\s*['\"]([^'\"]+)['\"]");
        std::smatch m;
        if (std::regex_search(yaml, m, re))
            return m[1].str() == expected;
    }
    // Multi-line: id: name\n ... default: 'value' before next id
    {
        std::regex re("id:\\s*" + id +
                      "\\s*\\n(?:[^\\n]*\\n)*?\\s*default:\\s*['\"]([^'\"]+)['\"]");
        std::smatch m;
        if (std::regex_search(yaml, m, re))
            return m[1].str() == expected;
    }
    return false;
}

bool yaml_has_param(const std::string& yaml, const std::string& id)
{
    std::regex re("id:\\s*" + id + "\\b");
    return std::regex_search(yaml, re);
}

bool yaml_make_contains(const std::string& yaml, const std::string& needle)
{
    return yaml.find(needle) != std::string::npos;
}

// Resolve grc/ path: tests run from build/lib or similar; headers live under
// include/, grc is sibling of lib/.
std::string grc_path(const char* filename)
{
    // CMAKE_SOURCE_DIR is gr-uwb when configured from gr-uwb/
    // Prefer compile-time path if provided.
#ifdef UWB_GRC_DIR
    return std::string(UWB_GRC_DIR) + "/" + filename;
#else
    // Fallbacks relative to common cwd when running ctest from gr-uwb/build.
    const char* candidates[] = {
        "../grc/",
        "../../grc/",
        "grc/",
        "../gr-uwb/grc/",
    };
    for (const char* prefix : candidates) {
        std::string p = std::string(prefix) + filename;
        std::ifstream in(p);
        if (in.good())
            return p;
    }
    return std::string("../grc/") + filename;
#endif
}

} // namespace

BOOST_AUTO_TEST_CASE(test_defaults_production_values)
{
    // Keep the numeric production contract explicit in one place.
    BOOST_CHECK_CLOSE(gr::uwb::defaults::kSampleRateHz, 998400000.0, 1e-12);
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorPreTrigger, size_t(2032));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorCapture, size_t(200000));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorEnergyGateDecimation,
                      size_t(100));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorCoarseDecimation, size_t(4));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorCoarseRepetitions, size_t(1));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kDetectorCoarseMargin, size_t(16));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kScheduledPreGuard, size_t(9984));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kScheduledCapture, size_t(189696));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kScheduledPostGuard, size_t(4096));
    BOOST_CHECK_EQUAL(gr::uwb::defaults::kScheduledPoolSize, size_t(8));
    BOOST_CHECK_CLOSE(gr::uwb::defaults::kScheduledPacketIntervalS, 0.01, 1e-12);
}

BOOST_AUTO_TEST_CASE(test_grc_detector_defaults_match_header)
{
    const std::string yaml = read_file(grc_path("uwb_detector.block.yml"));
    BOOST_CHECK(yaml_default_equals(yaml, "pre_trigger", "2032"));
    BOOST_CHECK(yaml_default_equals(yaml, "capture_samples", "200000"));
    BOOST_CHECK(yaml_default_equals(yaml, "energy_threshold", "0.001"));
    BOOST_CHECK(yaml_default_equals(yaml, "energy_gate_decimation", "100"));
    BOOST_CHECK(yaml_default_equals(yaml, "coarse_decimation", "4"));
    BOOST_CHECK(yaml_default_equals(yaml, "coarse_repetitions", "1"));
    BOOST_CHECK(yaml_default_equals(yaml, "coarse_margin", "16"));
    BOOST_CHECK(yaml_default_equals(yaml, "sample_rate", "998.4e6"));
    BOOST_CHECK(yaml_has_param(yaml, "coarse_repetitions"));
    BOOST_CHECK(yaml_has_param(yaml, "coarse_margin"));
    BOOST_CHECK(yaml_has_param(yaml, "sample_rate"));
    BOOST_CHECK(yaml_make_contains(yaml, "${coarse_repetitions}"));
    BOOST_CHECK(yaml_make_contains(yaml, "${coarse_margin}"));
    BOOST_CHECK(yaml_make_contains(yaml, "${sample_rate}"));
    // Both Python and C++ make templates must wire the new knobs.
    BOOST_CHECK(yaml.find("make_from_file(${template_file}") != std::string::npos);
    BOOST_CHECK(yaml.find("UwbDetector::make_from_file") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(test_grc_detector_sc16_defaults_match_header)
{
    const std::string yaml = read_file(grc_path("uwb_detector_sc16.block.yml"));
    BOOST_CHECK(yaml_default_equals(yaml, "pre_trigger", "2032"));
    BOOST_CHECK(yaml_default_equals(yaml, "capture_samples", "200000"));
    BOOST_CHECK(yaml_default_equals(yaml, "coarse_repetitions", "1"));
    BOOST_CHECK(yaml_default_equals(yaml, "coarse_margin", "16"));
    BOOST_CHECK(yaml_default_equals(yaml, "sample_rate", "998.4e6"));
    BOOST_CHECK(yaml_make_contains(yaml, "${coarse_repetitions}"));
    BOOST_CHECK(yaml_make_contains(yaml, "${coarse_margin}"));
    BOOST_CHECK(yaml_make_contains(yaml, "${sample_rate}"));
    BOOST_CHECK(yaml.find("UwbDetectorSc16::make_from_file") != std::string::npos);
}

BOOST_AUTO_TEST_CASE(test_grc_scheduled_extractor_emit_policy_and_defaults)
{
    const std::string yaml =
        read_file(grc_path("uwb_scheduled_extractor.block.yml"));
    BOOST_CHECK(yaml_default_equals(yaml, "sample_rate", "998.4e6"));
    BOOST_CHECK(yaml_default_equals(yaml, "packet_interval_s", "0.01"));
    BOOST_CHECK(yaml_default_equals(yaml, "pre_guard_samples", "9984"));
    BOOST_CHECK(yaml_default_equals(yaml, "capture_samples", "189696"));
    BOOST_CHECK(yaml_default_equals(yaml, "post_guard_samples", "4096"));
    BOOST_CHECK(yaml_default_equals(yaml, "pool_size", "8"));
    BOOST_CHECK(yaml_has_param(yaml, "emit_policy"));
    BOOST_CHECK(yaml.find("EverySlot") != std::string::npos);
    BOOST_CHECK(yaml.find("VerifiedOnly") != std::string::npos);
    // Must not hard-wire EverySlot only — parameter must drive the make line.
    BOOST_CHECK(yaml_make_contains(yaml, "EmitPolicy.${emit_policy}"));
    BOOST_CHECK(yaml.find("EmitPolicy.EverySlot, ${verification_enabled}") ==
                std::string::npos);
    // C++ template aligned with Python.
    BOOST_CHECK(yaml.find("EmitPolicy::${emit_policy}") != std::string::npos);
}
