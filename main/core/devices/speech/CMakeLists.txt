# Copyright: (C) 2016 LuxFutureRobotic
# Authors: Ali Paikan <ali.paikan@gmail.com>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(speech TYPE cuddie::dev::Speech INCLUDE speech.h)

if(ENABLE_cuddiemod_speech)

#compile definition file to generate source code into the desired directory
set(IDL_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${IDL_GENERATED_CODE_DIR}/include
                    ./svoxpico)

yarp_idl_to_dir(speech.thrift ${IDL_GENERATED_CODE_DIR})


file(GLOB svox_source svoxpico/*.c)
file(GLOB svox_header svoxpico/*.h)
source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

yarp_add_plugin(speech ${svox_source}
                       ${svox_header}
                       speech.h
                       speech.cpp
                       ${IDL_GENERATED_CODE_DIR}/include/Speech_IDL.h
                       ${IDL_GENERATED_CODE_DIR}/src/Speech_IDL.cpp)

target_link_libraries(speech ${YARP_LIBRARIES})
yarp_install(TARGETS speech
             COMPONENT Runtime
             LIBRARY DESTINATION ${CUDDIE_DYNAMIC_PLUGINS_INSTALL_DIR})
yarp_install(FILES speech.ini
             DESTINATION ${CUDDIE_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()

