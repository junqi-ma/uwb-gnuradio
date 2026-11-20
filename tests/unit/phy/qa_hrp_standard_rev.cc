/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Checks the HRP standard-revision vocabulary: distinct enumerators, metadata
 * for every known amendment, and an explicit Unknown that never masquerades as
 * 4a.  There is no behaviour to test by design (Phase 2 reserves the
 * vocabulary only).
 */

#include <boost/test/unit_test.hpp>

#include <openuwb/phy/hrp/common/standard_rev.h>

using openuwb::phy::hrp::HrpStandardInfo;
using openuwb::phy::hrp::HrpStandardRev;
using openuwb::phy::hrp::is_known;
using openuwb::phy::hrp::standard_info;

BOOST_AUTO_TEST_CASE(test_hrp_revisions_are_distinct)
{
    BOOST_CHECK(HrpStandardRev::Unknown != HrpStandardRev::Rev4a);
    BOOST_CHECK(HrpStandardRev::Rev4a != HrpStandardRev::Rev4z);
    BOOST_CHECK(HrpStandardRev::Rev4z != HrpStandardRev::Rev4ab);
    BOOST_CHECK_EQUAL(static_cast<int>(HrpStandardRev::Unknown), 0);
    // Unknown must sort first so a zero-initialised value is never mistaken
    // for a real amendment.
    BOOST_CHECK(static_cast<int>(HrpStandardRev::Rev4a) > 0);
}

BOOST_AUTO_TEST_CASE(test_hrp_metadata_for_known_revisions)
{
    for (HrpStandardRev rev : { HrpStandardRev::Rev4a, HrpStandardRev::Rev4z,
                                HrpStandardRev::Rev4ab }) {
        const HrpStandardInfo info = standard_info(rev);
        BOOST_CHECK(info.rev == rev);
        BOOST_CHECK(!info.name.empty());
        BOOST_CHECK(!info.amendment.empty());
        BOOST_CHECK(is_known(rev));
    }

    // STS is the 4z/4ab ranging feature; 4a predates it.
    BOOST_CHECK(!standard_info(HrpStandardRev::Rev4a).uses_sts);
    BOOST_CHECK(standard_info(HrpStandardRev::Rev4z).uses_sts);
    BOOST_CHECK(standard_info(HrpStandardRev::Rev4ab).uses_sts);
    BOOST_CHECK_EQUAL(standard_info(HrpStandardRev::Rev4z).name, "4z");
}

BOOST_AUTO_TEST_CASE(test_hrp_unknown_is_explicit)
{
    const HrpStandardInfo info = standard_info(HrpStandardRev::Unknown);
    BOOST_CHECK(info.rev == HrpStandardRev::Unknown);
    BOOST_CHECK(info.name.empty());
    BOOST_CHECK(info.amendment.empty());
    BOOST_CHECK(!info.uses_sts);
    BOOST_CHECK(!is_known(HrpStandardRev::Unknown));
}
