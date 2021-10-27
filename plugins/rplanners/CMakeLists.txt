###########################################
# rplanners openrave plugin
###########################################
add_subdirectory(rampoptimizer)
add_subdirectory(ParabolicPathSmooth)
add_library(rplanners SHARED constraintparabolicsmoother.cpp cubicretimer.cpp linearretimer.cpp linearsmoother.cpp mergewaypoints.cpp parabolicretimer.cpp parabolicsmoother.cpp linearshortcutadvanced.cpp randomized-astar.cpp rplanners.h rplanners.cpp rrt.h workspacetrajectorytracker.cpp manipconstraints2.h parabolicretimer2.cpp parabolicsmoother2.cpp)

target_link_libraries(rplanners libopenrave ParabolicPathSmooth rampoptimizer)
target_link_libraries(rplanners PRIVATE boost_assertion_failed)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
