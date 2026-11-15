# UHD dependency closure for the installed native uwb package.
#
# Included by uwbConfig.cmake ONLY when the installed uwb::uwb_native was
# built with UHD (the exported uwbTargets.cmake references UHD::UHD).
#
# Resolution order:
#   1. UHD::UHD target from the UHD CMake config (preferred), found with a
#      plain non-REQUIRED find_package(UHD CONFIG QUIET) -- NOT
#      find_dependency(): find_dependency would inherit REQUIRED from the
#      parent find_package(uwb CONFIG REQUIRED) and hard-error (or return)
#      when UHD is missing or CMAKE_DISABLE_FIND_PACKAGE_UHD=TRUE, making the
#      pkg-config fallback below unreachable.
#   2. pkg-config module 'uhd' via its IMPORTED_TARGET interface
#      (PkgConfig::UHD_PC), which carries the linker/include flags and therefore
#      also supports non-system prefixes.
#   3. UHD config/module variables (UHD_LIBRARIES/UHD_INCLUDE_DIRS or
#      UHD_PC_*) as a plain interface fallback.
#   4. If nothing is found: set uwb_FOUND FALSE + uwb_NOT_FOUND_MESSAGE and
#      return WITHOUT creating an empty imported target, so the parent
#      uwbConfig.cmake skips the uwbTargets include and find_package(uwb)
#      reports a proper not-found (fatal only when REQUIRED).

find_package(UHD CONFIG QUIET)
if(NOT TARGET UHD::UHD)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(UHD_PC QUIET IMPORTED_TARGET uhd)
    endif()
endif()

if(NOT TARGET UHD::UHD AND TARGET PkgConfig::UHD_PC)
    add_library(UHD::UHD INTERFACE IMPORTED)
    set_target_properties(UHD::UHD PROPERTIES
        INTERFACE_LINK_LIBRARIES PkgConfig::UHD_PC)
elseif(NOT TARGET UHD::UHD AND (UHD_FOUND OR UHD_PC_FOUND))
    add_library(UHD::UHD INTERFACE IMPORTED)
    if(UHD_INCLUDE_DIRS)
        set_target_properties(UHD::UHD PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${UHD_INCLUDE_DIRS}")
    elseif(UHD_PC_INCLUDE_DIRS)
        set_target_properties(UHD::UHD PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${UHD_PC_INCLUDE_DIRS}")
    endif()
    set_target_properties(UHD::UHD PROPERTIES
        INTERFACE_LINK_LIBRARIES "${UHD_LIBRARIES};${UHD_PC_LIBRARIES}")
endif()

if(NOT TARGET UHD::UHD)
    set(uwb_FOUND FALSE)
    set(uwb_NOT_FOUND_MESSAGE
        "uwb: the installed uwb::uwb_native was built with UHD, but neither "
        "the UHD CMake config (target UHD::UHD or UHD_LIBRARIES) nor "
        "pkg-config module 'uhd' was found (UHD may also be disabled via "
        "CMAKE_DISABLE_FIND_PACKAGE_UHD)")
    set(UWB_UHD_DEP_OK FALSE)
    return()
endif()

set(UWB_UHD_DEP_OK TRUE)
