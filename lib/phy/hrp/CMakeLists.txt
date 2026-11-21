# phy/hrp: the HRP demodulation pipeline implementation.
add_library(openuwb_phy_hrp STATIC uwb_demod_pipeline.cc uwb_hrp_mod_core.cc
    uwb_demod_timing.cc uwb_demod_cfo_sfd.cc)
target_link_libraries(openuwb_phy_hrp PUBLIC openuwb_dsp)
set_target_properties(openuwb_phy_hrp PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_phy_hrp)
