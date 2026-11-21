# dsp: resampler kernels (rational resampler core + transposed kernel) and
# the tone-cancel core.
set(sources
    uwb_detector_core.cc
    uwb_fixed_tone_cancel_core.cc
    uwb_rational_resampler_core.cc
    uwb_resampler_transposed.cc)
# The AVX-512 kernels live in their own translation units and are built only
# when CMake selected the AVX-512 backend; the rest stays on AVX2/FMA.
if(UWB_SIMD_HAVE_AVX512)
    list(APPEND sources
        uwb_rational_resampler_avx512.cc
        uwb_resampler_transposed_avx512.cc
        uwb_demod_sic_avx512.cc)
endif()
add_library(openuwb_dsp STATIC ${sources})
target_link_libraries(openuwb_dsp PUBLIC openuwb_support)
set_target_properties(openuwb_dsp PROPERTIES POSITION_INDEPENDENT_CODE ON)
uwb_simd_apply(openuwb_dsp)
