/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Checks the reserved runtime config vocabulary: each domain has a distinct
 * kind and a default-constructed config reports the right one.  Phase 2 adds
 * no fields on purpose, so this is a contract test, not a behaviour test.
 */

#include <boost/test/unit_test.hpp>

#include <openuwb/runtime/config.h>

#include <string>

using gr::uwb::runtime::config_kind_name;
using gr::uwb::runtime::ConfigKind;
using gr::uwb::runtime::RadarConfig;
using gr::uwb::runtime::RxConfig;
using gr::uwb::runtime::TwrConfig;
using gr::uwb::runtime::TxConfig;

BOOST_AUTO_TEST_CASE(test_config_kind_names)
{
    BOOST_CHECK_EQUAL(std::string(config_kind_name(ConfigKind::Rx)), "rx");
    BOOST_CHECK_EQUAL(std::string(config_kind_name(ConfigKind::Tx)), "tx");
    BOOST_CHECK_EQUAL(std::string(config_kind_name(ConfigKind::Twr)), "twr");
    BOOST_CHECK_EQUAL(std::string(config_kind_name(ConfigKind::Radar)), "radar");
}

BOOST_AUTO_TEST_CASE(test_default_configs_report_their_domain)
{
    BOOST_CHECK(RxConfig{}.common.kind == ConfigKind::Rx);
    BOOST_CHECK(TxConfig{}.common.kind == ConfigKind::Tx);
    BOOST_CHECK(TwrConfig{}.common.kind == ConfigKind::Twr);
    BOOST_CHECK(RadarConfig{}.common.kind == ConfigKind::Radar);
    // "Empty profile" means "domain default", never a guessed name.
    BOOST_CHECK(RxConfig{}.common.profile.empty());
}
