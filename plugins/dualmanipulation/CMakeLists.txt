###########################################
# rmanipulation openrave plugin
###########################################
add_library(dualmanipulation SHARED dualmanipulation.cpp dualmanipulation.h    plugindefs.h  dualcommonmanipulation.h )
target_link_libraries(dualmanipulation libopenrave)
set_target_properties(dualmanipulation PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS dualmanipulation DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-dualmanipulation)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-DUALMANIPULATION_DISPLAY_NAME "Plugin for dual arm manipulation" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-dualmanipulation PARENT_SCOPE)
