###########################################
# rplanners openrave plugin
###########################################
add_subdirectory(ParabolicPathSmooth)
add_subdirectory(rampoptimizer)
add_library(rplanners SHARED constraintparabolicsmoother.cpp cubicretimer.cpp graspgradient.cpp linearretimer.cpp linearsmoother.cpp mergewaypoints.cpp parabolicretimer.cpp parabolicsmoother.cpp linearshortcutadvanced.cpp randomized-astar.cpp rplanners.h rplanners.cpp rrt.h workspacetrajectorytracker.cpp)

target_link_libraries(rplanners libopenrave ParabolicPathSmooth)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

# python bindings
if(OPT_PYTHON AND PYTHON_EXECUTABLE AND LAPACK_FOUND)
  if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
    if( HAVE_ALL_PYTHON_HEADERS )
      include_directories(${PYTHON_INCLUDE_PATH} ${PYTHON_INCLUDE_DIRS})
      add_library(openravepy_rplanners SHARED openravepy_rplanners.cpp)
      target_link_libraries(openravepy_rplanners openravepy_int rplanners rampoptimizer ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${LAPACK_LIBRARIES})
      set_target_properties(openravepy_rplanners PROPERTIES PREFIX "" COMPILE_FLAGS "")
      if( WIN32 )
        set_target_properties(openravepy_rplanners PROPERTIES SUFFIX ".pyd")
      endif()
      install(TARGETS openravepy_rplanners DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()
  endif()
endif()
