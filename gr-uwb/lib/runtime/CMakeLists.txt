# runtime: the UHD SC16 scheduled pipeline and its sources.
add_library(openuwb_runtime STATIC scheduled_pipeline.cc)
target_link_libraries(openuwb_runtime PUBLIC
    openuwb_phy_hrp openuwb_sic openuwb_sensing openuwb_radio)
set_target_properties(openuwb_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_runtime)
