###########################################
# grasper openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(grasper SHARED graspermain.cpp)

if( QHULL_FOUND )
  target_link_libraries(grasper libopenrave qhull)
  add_definitions(-DQHULL_FOUND)
else()
  target_link_libraries(grasper libopenrave)
endif()

include_directories("${CMAKE_SOURCE_DIR}")
if( ${QHULL_INCLUDE_DIR} )
  include_directories("${CMAKE_SOURCE_DIR}")
endif()

add_dependencies(grasper libopenrave)
set_target_properties(grasper PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                              LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS grasper DESTINATION share/openrave/plugins )
