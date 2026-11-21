# phy/hrp: the HRP demodulation pipeline implementation.
#
# The demodulation stages used to be header-only; Wave 3 (2026-11-21) moved
# the coarse stage/batch entry points into these translation units so that a
# change to an implementation rebuilds one object instead of every consumer.
add_library(openuwb_phy_hrp STATIC
    uwb_demod_pipeline.cc
    uwb_hrp_mod_core.cc
    uwb_demod_timing.cc
    uwb_demod_cfo_sfd.cc
    uwb_demod_softchips_payload.cc
    uwb_demod_fec.cc)
target_link_libraries(openuwb_phy_hrp PUBLIC openuwb_dsp)
set_target_properties(openuwb_phy_hrp PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_phy_hrp)
