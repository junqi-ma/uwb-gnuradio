# sic: preamble / discrete-CIR / full-packet SIC cores.
set(sources
    uwb_sic_unified.cc
    uwb_discrete_cir_estimator.cc
    uwb_discrete_cir_reconstruct.cc
    uwb_preamble_sic_core.cc
    uwb_fullpacket_sic_core.cc)
add_library(openuwb_sic STATIC ${sources})
target_link_libraries(openuwb_sic PUBLIC openuwb_phy_hrp)
set_target_properties(openuwb_sic PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_sic)
