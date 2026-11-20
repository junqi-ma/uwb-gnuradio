/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Legacy namespace compatibility layer.
 *
 * Phase 3 renamed the implementation namespace from `gr::uwb` to `openuwb`.
 * Public headers under <openuwb/...> now declare `openuwb::...`; this header
 * keeps the old spelling resolvable for downstream sources that have not
 * migrated yet.
 *
 * It is a using-directive, not a set of using-declarations: qualified lookup
 * such as `gr::uwb::twr::TwrConfig` follows the directive inside `gr::uwb` and
 * finds `openuwb::twr::TwrConfig`.  That keeps the alias complete without
 * enumerating every symbol (and without silently drifting when a symbol is
 * added).
 *
 * Do NOT add declarations to `gr::uwb`: a real declaration there would win
 * over the directive and create a second, divergent type with the same name.
 */
#ifndef INCLUDED_GR_UWB_COMPAT_H
#define INCLUDED_GR_UWB_COMPAT_H

// TODO(openuwb-compat-removal): delete this alias once downstream users and
// the remaining forwarding headers are gone (Phase 4 tracks the removal).
//
// The forward declaration matters: a forwarding header includes this file
// BEFORE the real <openuwb/...> header, and a using-directive may only name an
// already-declared namespace.  Declaring it empty here is enough, and later
// declarations merge into the same namespace.
namespace openuwb {}

namespace gr {
namespace uwb {
using namespace ::openuwb;
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GR_UWB_COMPAT_H */
