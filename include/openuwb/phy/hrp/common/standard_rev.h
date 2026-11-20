/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * HRP standard revisions: the lightweight vocabulary that lets the PHY layer
 * name which 802.15.4 amendment a profile targets.
 *
 * Phase 2 scope: this is a vocabulary and metadata header only.  It contains
 * no behaviour, no profile selection and no code path; the existing DW1000 /
 * DW3000 / QM35825 profiles keep their current behaviour and do not consult
 * this header yet.  When 4z/4ab support is implemented it must arrive with its
 * own profile data and tests, not as an empty code path here.
 *
 * The dependency is one way and trivial: stdlib only.
 */
#ifndef INCLUDED_OPENUWB_PHY_HRP_COMMON_STANDARD_REV_H
#define INCLUDED_OPENUWB_PHY_HRP_COMMON_STANDARD_REV_H

#include <cstdint>
#include <string_view>

namespace openuwb {
namespace phy {
namespace hrp {

/// The HRP amendment a profile belongs to.
///
///   Rev4a  IEEE 802.15.4a-2007  -- the original HRP UWB PHY
///   Rev4z  IEEE 802.15.4z-2020  -- UWB enhancements (STS, ranging)
///   Rev4ab IEEE 802.15.4ab       -- next-generation UWB (in development)
///
/// Unknown is a real value: a caller must never guess a revision, and a
/// profile that does not declare one is reported as Unknown rather than being
/// silently treated as 4a.
enum class HrpStandardRev : std::uint8_t {
    Unknown = 0,
    Rev4a = 1,
    Rev4z = 2,
    Rev4ab = 3,
};

/// Static, human-readable metadata.  Descriptions are intentionally short: the
/// normative text belongs to the standard, not to this header.
struct HrpStandardInfo {
    HrpStandardRev rev;
    std::string_view name;         ///< "4a", "4z", "4ab" (empty for Unknown)
    std::string_view amendment;    ///< e.g. "802.15.4a-2007"
    bool uses_sts;                 ///< secure ranging / STS present
};

constexpr HrpStandardInfo standard_info(HrpStandardRev rev) noexcept
{
    switch (rev) {
    case HrpStandardRev::Rev4a:
        return { rev, "4a", "802.15.4a-2007", false };
    case HrpStandardRev::Rev4z:
        return { rev, "4z", "802.15.4z-2020", true };
    case HrpStandardRev::Rev4ab:
        return { rev, "4ab", "802.15.4ab", true };
    case HrpStandardRev::Unknown:
        break;
    }
    return { HrpStandardRev::Unknown, {}, {}, false };
}

/// True for a known UWB amendment (i.e. everything except Unknown).
constexpr bool is_known(HrpStandardRev rev) noexcept
{
    return rev != HrpStandardRev::Unknown;
}

} // namespace hrp
} // namespace phy
} // namespace openuwb

#endif /* INCLUDED_OPENUWB_PHY_HRP_COMMON_STANDARD_REV_H */
