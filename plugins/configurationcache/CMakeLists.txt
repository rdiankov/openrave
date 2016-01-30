###########################################
# configurationcache openrave plugin
###########################################
add_library(configurationcache SHARED cachechecker.cpp configurationcache.cpp configurationcachetree.cpp configurationjitterer.cpp)
target_link_libraries(configurationcache libopenrave ${LAPACK_LIBRARIES})
set_target_properties(configurationcache PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS configurationcache DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

# python bindings
if(OPT_PYTHON AND PYTHON_EXECUTABLE AND LAPACK_FOUND)
  if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
    if( HAVE_ALL_PYTHON_HEADERS )
      include_directories(${PYTHON_INCLUDE_PATH} ${PYTHON_INCLUDE_DIRS})
      add_library(openravepy_configurationcache SHARED openravepy_configurationcache.cpp)
      target_link_libraries(openravepy_configurationcache openravepy_int configurationcache ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${LAPACK_LIBRARIES})
      set_target_properties(openravepy_configurationcache PROPERTIES PREFIX "" COMPILE_FLAGS "")
      if( WIN32 )
        set_target_properties(openravepy_configurationcache PROPERTIES SUFFIX ".pyd")
      endif()
      install(TARGETS openravepy_configurationcache DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
    endif()
  endif()
endif()
