###########################################
# configurationcache openrave plugin
###########################################
add_library(configurationcache SHARED cachechecker.cpp configurationcache.cpp configurationcachetree.cpp)
target_link_libraries(configurationcache libopenrave)
set_target_properties(configurationcache PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS configurationcache DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
