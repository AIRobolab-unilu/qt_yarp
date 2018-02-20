# Copyright: (C) 2016 LuxFutureRobotic
# Authors: Ali Paikan <ali.paikan@gmail.com>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(actionManager TYPE cuddie::dev::ActionManager INCLUDE actionManager.h)

if(ENABLE_cuddiemod_actionManager)

#compile definition file to generate source code into the desired directory
set(IDL_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${IDL_GENERATED_CODE_DIR}/include)

yarp_idl_to_dir(ActionManager.thrift ${IDL_GENERATED_CODE_DIR})


yarp_add_plugin(actionManager
                       actionRecorder.h
                       actionManager.h                       
                       actionRecorder.cpp
                       actionManager.cpp
                       ${IDL_GENERATED_CODE_DIR}/include/ActionManager_IDL.h
                       ${IDL_GENERATED_CODE_DIR}/src/ActionManager_IDL.cpp)

target_link_libraries(actionManager ${YARP_LIBRARIES})
yarp_install(TARGETS actionManager
             COMPONENT Runtime
             LIBRARY DESTINATION ${CUDDIE_DYNAMIC_PLUGINS_INSTALL_DIR})
yarp_install(FILES actionManager.ini
             DESTINATION ${CUDDIE_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
