# twr: the standalone, GNU-Radio-free / UHD-free two-way-ranging stack.
#
# openuwb_twr_core is the public protocol core; the fake link is QA transport
# and is NOT installed; the native-rate PHY loop reuses the header-only HRP
# cores, so it links Volk/Threads and needs no GNU Radio runtime.
add_library(openuwb_twr_core STATIC uwb_twr_core.cc)
target_link_libraries(openuwb_twr_core PUBLIC openuwb_phy_hrp)
set_target_properties(openuwb_twr_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(openuwb_twr_fake_link STATIC uwb_twr_fake_link.cc)
target_link_libraries(openuwb_twr_fake_link PUBLIC openuwb_twr_core)

add_library(openuwb_twr_phy STATIC uwb_twr_phy.cc uwb_twr_phy_impairment.cc)
target_link_libraries(openuwb_twr_phy PUBLIC
    openuwb_twr_core openuwb_dsp Volk::volk Threads::Threads)
set_target_properties(openuwb_twr_phy PROPERTIES POSITION_INDEPENDENT_CODE ON)
# uwb_twr_phy.h includes the header-only resampler/demod cores, so it must
# compile them with the same ISA as the rest of the build.
uwb_simd_apply(openuwb_twr_phy)

# Phase 2 compatibility: the Phase 1 target names are still used by apps/ and
# tests/.  ALIAS forwards them to the domain targets.
# TODO(openuwb-compat-removal): switch the consumers and drop these.
add_library(uwb_twr_core ALIAS openuwb_twr_core)
add_library(uwb_twr_fake_link ALIAS openuwb_twr_fake_link)
add_library(uwb_twr_phy ALIAS openuwb_twr_phy)
