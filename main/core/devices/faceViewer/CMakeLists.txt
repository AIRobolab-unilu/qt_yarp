# Copyright: (C) 2015 iCub Facility
# Authors: Ali Paikan <ali.paikan@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(faceViewer TYPE cuddie::dev::faceViewer INCLUDE faceViewer.h)

if(ENABLE_cuddiemod_faceViewer)

    #compile definition file to generate source code into the desired directory
    set(IDL_GENERATED_CODE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                        ${YARP_INCLUDE_DIRS}
                        ${IDL_GENERATED_CODE_DIR}/include
                        ./common)

    yarp_idl_to_dir(faceViewer.thrift ${IDL_GENERATED_CODE_DIR})


    yarp_add_plugin(faceViewer faceViewer.h
                               faceViewer.cpp
                               ./common/esShader.c
                               ./common/esShapes.c
                               ./common/esTransform.c
                               ./common/esUtil.c
                               ${IDL_GENERATED_CODE_DIR}/include/faceViewer_IDL.h
                               ${IDL_GENERATED_CODE_DIR}/src/faceViewer_IDL.cpp)

    target_link_libraries(faceViewer ${YARP_LIBRARIES} GLESv2 EGL m X11)
    yarp_install(TARGETS faceViewer
                 COMPONENT Runtime
                 LIBRARY DESTINATION ${CUDDIE_DYNAMIC_PLUGINS_INSTALL_DIR})
    yarp_install(FILES faceViewer.ini
                 DESTINATION ${CUDDIE_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()                 
