###########################################
# configurationcache openrave plugin
###########################################
add_library(configurationcache SHARED cachechecker.cpp configurationcache.cpp configurationcachetree.cpp configurationjitterer.cpp)
target_link_libraries(configurationcache libopenrave ${LAPACK_LIBRARIES})
target_link_libraries(configurationcache PRIVATE boost_assertion_failed)
set_target_properties(configurationcache PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS configurationcache DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

# python bindings
if( HAVE_ALL_PYTHON_HEADERS )
  # include
  include_directories(${PYTHON_INCLUDE_PATH} ${PYTHON_INCLUDE_DIRS}
    ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS}
  )

  # link
  set(OPENRAVEPY_CONFIGURATIONCACHE_LINKED_LIBRARIES openravepy_int configurationcache ${PYTHON_LIBRARIES} ${Boost_THREAD_LIBRARY} ${LAPACK_LIBRARIES})
  if(NOT USE_PYBIND11_PYTHON_BINDINGS)
    set(OPENRAVEPY_CONFIGURATIONCACHE_LINKED_LIBRARIES ${OPENRAVEPY_CONFIGURATIONCACHE_LINKED_LIBRARIES} ${Boost_PYTHON_LIBRARY})
  endif()

  add_library(openravepy_configurationcache SHARED openravepy_configurationcache.cpp)

  target_link_libraries(openravepy_configurationcache PUBLIC ${OPENRAVEPY_CONFIGURATIONCACHE_LINKED_LIBRARIES} PRIVATE boost_assertion_failed)

  set(OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS "")
  if(USE_PYBIND11_PYTHON_BINDINGS)
    message(STATUS "Build openravepy_configurationcache with pybind11")
    find_package(pybind11 REQUIRED)
    set(OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS} -DUSE_PYBIND11_PYTHON_BINDINGS")
    if(PYBIND11_PYTHON_BINDINGS_DEBUG)
      set(OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS} -g -O0")
    else()
      set(OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS} -g0 -O3")
    endif()
    # pybind11 requires less visibility
    set(OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS} -fvisibility=hidden")
  else()
    message(STATUS "Build openravepy_configurationcache with Boost.Python")
  endif()

  set_target_properties(openravepy_configurationcache
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_CONFIGURATIONCACHE_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy_configurationcache PROPERTIES SUFFIX ".pyd")
  endif()
  install(TARGETS openravepy_configurationcache DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
else()
  message(STATUS "HAVE_ALL_PYTHON_HEADERS = ${HAVE_ALL_PYTHON_HEADERS}; do not build openravepy_configurationcache")
endif()
