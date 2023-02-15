###########################################
# rplanners openrave plugin
###########################################
add_subdirectory(piecewisepolynomials)
add_subdirectory(rampoptimizer)
add_subdirectory(ParabolicPathSmooth)
add_library(rplanners SHARED constraintparabolicsmoother.cpp cubicretimer.cpp linearretimer.cpp linearsmoother.cpp mergewaypoints.cpp parabolicretimer.cpp parabolicsmoother.cpp linearshortcutadvanced.cpp randomized-astar.cpp rplanners.h rplanners.cpp rrt.h workspacetrajectorytracker.cpp manipconstraints2.h parabolicretimer2.cpp parabolicsmoother2.cpp jerklimitedsmootherbase.h cubicretimer2.cpp cubicsmoother.cpp quinticsmoother.cpp manipconstraints3.h quinticretimer.cpp)

target_link_libraries(rplanners PRIVATE boost_assertion_failed rampoptimizer PUBLIC libopenrave ParabolicPathSmooth piecewisepolynomials)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
