###########################################
# rplanners openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(rplanners SHARED rplanners.cpp graspgradient.h  plugindefs.h  randomized-astar.h  rplanners.h  rrt.h)

target_link_libraries(rplanners libopenrave)
add_dependencies(rplanners libopenrave)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR})
