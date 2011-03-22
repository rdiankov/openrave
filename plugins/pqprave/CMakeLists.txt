###########################################
# pqprave openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_subdirectory(pqp)

add_library(pqprave SHARED pqprave.cpp collisionPQP.h plugindefs.h)

target_link_libraries(pqprave libopenrave PQP)
add_dependencies(pqprave libopenrave PQP)
set_target_properties(pqprave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS pqprave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR})
