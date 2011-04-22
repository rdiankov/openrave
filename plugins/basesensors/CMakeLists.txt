###########################################
# basesensors openrave plugin
###########################################
add_library(basesensors SHARED basesensors.cpp basecamera.h  baseflashlidar3d.h  baselaser.h plugindefs.h)
target_link_libraries(basesensors libopenrave)
set_target_properties(basesensors PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basesensors DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
