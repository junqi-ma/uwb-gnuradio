find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_UWB gnuradio-uwb)

FIND_PATH(
    GR_UWB_INCLUDE_DIRS
    NAMES gnuradio/uwb/api.h
    HINTS $ENV{UWB_DIR}/include
        ${PC_UWB_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_UWB_LIBRARIES
    NAMES gnuradio-uwb
    HINTS $ENV{UWB_DIR}/lib
        ${PC_UWB_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-uwbTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_UWB DEFAULT_MSG GR_UWB_LIBRARIES GR_UWB_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_UWB_LIBRARIES GR_UWB_INCLUDE_DIRS)
