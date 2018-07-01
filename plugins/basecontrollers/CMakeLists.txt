###########################################
# basecontrollers openrave plugin
###########################################
add_library(basecontrollers SHARED basecontrollers.cpp redirectcontroller.cpp idealcontroller.cpp idealvelocitycontroller.cpp plugindefs.h)
target_link_libraries(basecontrollers libopenrave)
target_link_libraries(basecontrollers PRIVATE boost_assertion_failed)
set_target_properties(basecontrollers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}" OUTPUT_NAME basecontrollers)
install(TARGETS basecontrollers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
