###########################################
# pqprave openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_subdirectory(pqp)

add_library(pqprave SHARED collisionPQP.cpp pqpravemain.cpp)

target_link_libraries(pqprave libopenrave PQP)
include_directories(pqprave "${CMAKE_SOURCE_DIR}")
add_dependencies(pqprave libopenrave libopenrave_static PQP)
set_target_properties(pqprave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS pqprave DESTINATION share/openrave/plugins )
