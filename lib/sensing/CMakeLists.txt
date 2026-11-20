# sensing: header-only radar cores (no translation unit yet).
add_library(openuwb_sensing INTERFACE)
target_link_libraries(openuwb_sensing INTERFACE openuwb_phy_hrp)
