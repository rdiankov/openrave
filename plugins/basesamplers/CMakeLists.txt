###########################################
# basesamplers openrave plugin
###########################################
add_library(basesamplers SHARED basesamplers.cpp configurationjitterer.cpp halton.cpp)
target_link_libraries(basesamplers libopenrave ${LAPACK_LIBRARIES})
set_target_properties(basesamplers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basesamplers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
