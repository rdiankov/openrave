###########################################
# grasper openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(grasper SHARED graspermain.cpp)

target_link_libraries(grasper libopenrave)
include_directories(grasper "${CMAKE_SOURCE_DIR}")
add_dependencies(grasper libopenrave)
set_target_properties(grasper PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                              LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS grasper DESTINATION share/openrave/plugins )
