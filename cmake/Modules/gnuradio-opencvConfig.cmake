find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_OPENCV gnuradio-opencv)

FIND_PATH(
    GR_OPENCV_INCLUDE_DIRS
    NAMES gnuradio/opencv/api.h
    HINTS $ENV{OPENCV_DIR}/include
        ${PC_OPENCV_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_OPENCV_LIBRARIES
    NAMES gnuradio-opencv
    HINTS $ENV{OPENCV_DIR}/lib
        ${PC_OPENCV_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-opencvTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_OPENCV DEFAULT_MSG GR_OPENCV_LIBRARIES GR_OPENCV_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_OPENCV_LIBRARIES GR_OPENCV_INCLUDE_DIRS)
