###########################################
# rplanners openrave plugin
###########################################
add_library(rplanners SHARED rplanners.cpp plugindefs.h  rplanners.h  rrt.h graspgradient.cpp pathoptimizers.cpp randomized-astar.cpp trajectorytiming.cpp workspacetrajectorytracker.cpp)
target_link_libraries(rplanners libopenrave)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
