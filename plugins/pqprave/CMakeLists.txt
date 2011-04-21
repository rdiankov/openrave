###########################################
# pqprave openrave plugin
###########################################
add_subdirectory(pqp)
add_library(pqprave SHARED pqprave.cpp collisionPQP.h plugindefs.h)
target_link_libraries(pqprave libopenrave PQP)
set_target_properties(pqprave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS pqprave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-pqprave)
# don't return the component since cannot distribute this library
