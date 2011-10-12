###########################################
# baserobots openrave plugin
###########################################
add_library(baserobots SHARED baserobots.cpp collisionmaprobot.h plugindefs.h)
target_link_libraries(baserobots libopenrave)
set_target_properties(baserobots PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS baserobots DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
