# support: the baseline-ISA foundation (SIMD runtime probe, kernel backend).
add_library(openuwb_support STATIC
    uwb_simd_runtime.cc
    uwb_kernel_backend.cc
)
target_include_directories(openuwb_support PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(openuwb_support PUBLIC Volk::volk Threads::Threads)
target_compile_features(openuwb_support PUBLIC cxx_std_17)
target_compile_options(openuwb_support PRIVATE -O3)
set_target_properties(openuwb_support PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Deliberately NO uwb_simd_apply(): the capability probe must run (and refuse)
# before any wide instruction is reachable.  See the old simd_probe comment.
