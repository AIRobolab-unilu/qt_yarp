# Copyright: (C) 2015 iCub Facility
# Authors: Ali Paikan <ali.paikan@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(herkulexMotionControl TYPE cuddie::dev::herkulexMotionControl INCLUDE herkulexMotionControl.h WRAPPER controlboardwrapper2)

if(ENABLE_cuddiemod_herkulexMotionControl)
get_property(herkulex_INCLUDE_DIRS
                 TARGET herkulex
                 PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                        ${YARP_INCLUDE_DIRS}
                        ${herkulex_INCLUDE_DIRS})

    yarp_add_plugin(herkulexMotionControl herkulexMotionControl.h herkulexMotionControl.cpp)
    target_link_libraries(herkulexMotionControl ${YARP_LIBRARIES} herkulex)
    yarp_install(TARGETS herkulexMotionControl
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CUDDIE_DYNAMIC_PLUGINS_INSTALL_DIR})
    yarp_install(FILES herkulexMotionControl.ini
                 DESTINATION ${CUDDIE_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
