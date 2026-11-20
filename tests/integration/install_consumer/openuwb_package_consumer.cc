/*
 * Installed-consumer proof for the PRIMARY openUWB package.
 *
 * Unlike the legacy consumers next to this file, this one uses only the new
 * public tree (<openuwb/...>), the new namespace (openuwb::) and the new
 * CMake package (find_package(OpenUWB)), and links the openuwb:: target
 * spellings.  It must build against an install prefix alone.
 */

#include <openuwb/phy/hrp/rx/demod_pipeline.h>
#include <openuwb/runtime/uhd_sc16_pipeline.h>
#include <openuwb/support/simd.h>
#include <openuwb/twr/twr_config.h>

#include <cstdio>
#include <cstring>

int
main()
{
    std::printf("openUWB installed package consumer (openuwb:: targets)\n");

    const auto& caps = openuwb::twr::capabilities();
    std::printf("  schema=%s profile=%s phy_rows=%zu simd=%s\n",
                caps.schema_version.c_str(),
                caps.profile_version.c_str(),
                caps.phy_matrix.size(),
                openuwb::core::simd::backend_name());

    // Touch one type from each linked layer so the archives are really used:
    // twr_core (capability whitelist, header-only), runtime (pipeline config)
    // and phy_hrp (take the demodulation entry point's address, which forces a
    // real reference into libopenuwb_phy_hrp.a).
    openuwb::twr::TwrConfig twr;
    openuwb::pipeline::PipelineConfig pipeline;
    auto* demod_entry = &openuwb::demod::core::demodulate_one;
    (void)twr;
    (void)pipeline;
    (void)demod_entry;

    const bool ok = caps.schema_version == "twr-config/2"
                    && !caps.phy_matrix.empty()
                    && std::strlen(openuwb::core::simd::backend_name()) > 0;
    std::printf("  result: %s\n", ok ? "ALL OK" : "FAILED");
    return ok ? 0 : 1;
}
