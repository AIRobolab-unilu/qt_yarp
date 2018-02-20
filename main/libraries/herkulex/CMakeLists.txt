# Copyright (C) 2016 Lux Future Robotic
# Author: Ali Paikan
# email:  ali.paikan@gmail.com

project(herkulex)

set(headers         include/Herkulex.h
                    include/serial.h
                    include/unix.h)

set(sources         src/Herkulex.cpp
                    src/serial.cpp
                    src/unix.cpp)

source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

include_directories(${PROJECT_SOURCE_DIR}/include
                     ${YARP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${headers} ${sources})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} )

# Add install target
yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})

set_property(TARGET ${PROJECT_NAME}
             PROPERTY BUILD_INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                          ${CUDDIE_INCLUDE_DIRS})

