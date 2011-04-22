###########################################
# logging openrave plugin
###########################################
add_library(logging SHARED logging.cpp loggingproblem.h plugindefs.h)
target_link_libraries(logging libopenrave)
set_target_properties(logging PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS logging DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
