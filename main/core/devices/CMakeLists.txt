include(YarpPlugin)

#########################################
# Prepare plugin library ccuddie
#########################################

yarp_begin_plugin_library(cuddiemod)
    add_subdirectory(herkulexMotor)
    add_subdirectory(faceViewer)
    add_subdirectory(speech)
    add_subdirectory(actionManager)
yarp_end_plugin_library(cuddiemod)

