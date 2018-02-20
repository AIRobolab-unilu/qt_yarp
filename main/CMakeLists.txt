# Copyright: (C) 2015
# Authors: Ali Paikan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Prepare a device library called "cuddiemod".
# A test executable called "cuddiemoddev" will also be produced.

cmake_minimum_required(VERSION 2.8.9)
project(cuddie)

find_package(YARP 2.3.64.4 REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

#find_package(ICUB REQUIRED)
#list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})


if(NOT WIN32)
    option(CUDDIE_SHARED_LIBRARY "Compile shared libraries rather than static libraries" ON)
    if(CUDDIE_SHARED_LIBRARY)
        set(BUILD_SHARED_LIBS ON)
    endif()
endif()
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "cuddie is always built with dynamic plugins")
include(YarpInstallationHelpers)
yarp_configure_external_installation(cuddie WITH_PLUGINS)


include(AddInstallRPATHSupport)

add_install_rpath_support(LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"                                   # Libraries
                          BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"                                   # Binaries
                                   "${CMAKE_INSTALL_PREFIX}/${CUDDIE_DYNAMIC_PLUGINS_INSTALL_DIR}"     # Plugins
                          USE_LINK_PATH)


include(YarpIDL)

# Everything is configured now, we can start adding source code
#add_subdirectory(manifest)
add_subdirectory(libraries)
add_subdirectory(core)
add_subdirectory(tools)
