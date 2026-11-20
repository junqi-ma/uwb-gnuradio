# radio: stream sources and device backends.
set(sources uwb_fixed_tone_cancel_source.cc)
if(UWB_HAVE_UHD)
    list(APPEND sources uwb_uhd_burst_backend.cc)
endif()
add_library(openuwb_radio STATIC ${sources})
target_link_libraries(openuwb_radio PUBLIC openuwb_dsp)
set_target_properties(openuwb_radio PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_radio)
if(UWB_HAVE_UHD)
    target_link_libraries(openuwb_radio PUBLIC UHD::UHD)
    # Public so consumers including uhd_burst_backend.h see the guard.
    target_compile_definitions(openuwb_radio PUBLIC UWB_HAVE_UHD=1)
endif()
