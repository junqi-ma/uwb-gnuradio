/* -*- c++ -*- */
/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * uwb_twr_test_output.h — where a TWR QA may write its generated artifacts.
 *
 * Why this file exists
 * --------------------
 * M0 review R8 / M0.1 §2.C.2: the TWR QAs used to write their CSV tables
 * straight into the SOURCE tree at `testdata/twr/`.  That is a repository of
 * reviewed, hand-archived golden artifacts, not a scratch directory, so every
 * ordinary `ctest` run silently overwrote it.  The reviewer had to compile
 * and run the timing QA out of tree just to avoid clobbering it, and the
 * default `ldd` resolution meant a regenerated CSV could silently come from a
 * stale /usr/local library.
 *
 * The rule this header encodes:
 *
 *   * DEFAULT  -> write under the BUILD tree
 *                 `${UWB_BUILD_DIR}/test-output/twr` (or
 *                 `${UWB_TEST_OUTPUT_DIR}` when set),
 *                 never into `${CMAKE_SOURCE_DIR}/../testdata/twr`.
 *   * EXPORT   -> writing into the source `testdata/twr` requires an
 *                 explicit, non-empty `UWB_TWR_EXPORT_DIR` pointing there.
 *                 There is no other way to clobber a reviewed artifact.
 *   * OVERRIDE -> `UWB_TWR_TEST_OUTPUT_DIR` wins over the default but is
 *                 still refused if it resolves inside the source testdata
 *                 tree without `UWB_TWR_EXPORT_DIR` being set.
 *
 * A run that cannot get a writable directory reports it and SKIPS the emit
 * case; it never falls back to the source tree.
 *
 * The header is dependency-free (C++ standard library only) and is included
 * by both qa_uwb_twr_phy_matrix.cc and qa_uwb_twr_timing_budget.cc.
 */

#ifndef INCLUDED_GNURADIO_UWB_UWB_TWR_TEST_OUTPUT_H
#define INCLUDED_GNURADIO_UWB_UWB_TWR_TEST_OUTPUT_H

// <cstddef> before <filesystem>/<fstream>: the same libstdc++ ordering
// constraint as in uwb_twr_capability_evidence.h, hit by the same
// external-consumer compile.
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#ifndef UWB_BUILD_DIR
#define UWB_BUILD_DIR ""
#endif

namespace gr {
namespace uwb {
namespace twr {
namespace testout {

// The source-tree artifact directory, as configured at compile time.  The
// check below refuses to write here unless export was requested.
inline std::string
source_twr_dir()
{
    return std::string(UWB_TESTDATA_DIR) + "/twr";
}

inline std::string
env(const char* name)
{
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : std::string();
}

// True when `path` is inside the source `testdata/twr` tree.  Compared on
// lexically normalised absolute paths so `..` cannot smuggle a write back in.
inline bool
is_source_twr_dir(const std::string& path)
{
    if (path.empty())
        return false;
    std::error_code ec;
    const std::filesystem::path a =
        std::filesystem::weakly_canonical(std::filesystem::absolute(path, ec), ec);
    const std::filesystem::path b = std::filesystem::weakly_canonical(
        std::filesystem::absolute(source_twr_dir(), ec), ec);
    if (ec)
        return false;
    const std::string as = a.string(), bs = b.string();
    return as == bs || (as.size() > bs.size() && as.compare(0, bs.size(), bs) == 0 &&
                        as[bs.size()] == '/');
}

// Default build-tree location for generated artifacts.  `build_dir` is the
// build tree this QA was compiled into (UWB_BUILD_DIR); when it is empty the
// system temp dir is used, which is still NOT the source tree.
inline std::string
default_output_dir()
{
    const std::string explicit_dir = env("UWB_TWR_TEST_OUTPUT_DIR");
    if (!explicit_dir.empty())
        return explicit_dir;
    const std::string build = std::string(UWB_BUILD_DIR);
    if (!build.empty())
        return build + "/test-output/twr";
    return (std::filesystem::temp_directory_path() / "uwb_twr_test_output").string();
}

// Resolve the directory a TWR QA may write into, creating it if needed.
// Returns "" when nothing is writable, with `why_out` explaining why -- the
// caller SKIPs the emit case rather than falling back to the source tree.
inline std::string
resolve(std::string& why_out)
{
    std::string dir;
    const std::string export_dir = env("UWB_TWR_EXPORT_DIR");
    if (!export_dir.empty()) {
        dir = export_dir;
    } else {
        dir = default_output_dir();
        if (is_source_twr_dir(dir)) {
            why_out = "refusing_to_write_into_the_source_twr_tree_"
                      "set_UWB_TWR_EXPORT_DIR_to_export_deliberately";
            return std::string();
        }
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec || !std::filesystem::is_directory(dir)) {
        why_out = "output directory is not creatable: " + dir;
        return std::string();
    }
    // Prove writability rather than assume it: a read-only checkout must
    // produce a recorded skip, not a half-written artifact.
    const auto probe = std::filesystem::path(dir) / ".qa_write_probe";
    std::ofstream f(probe, std::ios::app);
    if (!f.good()) {
        why_out = "output directory is not writable: " + dir;
        return std::string();
    }
    f.close();
    std::filesystem::remove(probe, ec);
    why_out = "ok";
    return dir;
}

} // namespace testout
} // namespace twr
} // namespace uwb
} // namespace gr

#endif /* INCLUDED_GNURADIO_UWB_UWB_TWR_TEST_OUTPUT_H */
