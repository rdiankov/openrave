add_library(rampoptimizer SHARED paraboliccommon.h paraboliccommon.cpp ramp.h ramp.cpp interpolator.h interpolator.cpp feasibilitychecker.h feasibilitychecker.cpp parabolicchecker.h parabolicchecker.cpp)
target_link_libraries(rampoptimizer PRIVATE boost_assertion_failed PUBLIC libopenrave)
set_target_properties(rampoptimizer PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

install(TARGETS rampoptimizer DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

add_dependencies(rampoptimizer interfacehashes_target)		

# python bindings
if( OPT_PYTHON )
  # link
  set(OPENRAVEPY_RAMPOPTIMIZER_LINKED_LIBRARIES openravepy_int rampoptimizer ${PYTHON_LIBRARIES} ${Boost_THREAD_LIBRARY} ${LAPACK_LIBRARIES})
  if(NOT USE_PYBIND11_PYTHON_BINDINGS)
    set(OPENRAVEPY_RAMPOPTIMIZER_LINKED_LIBRARIES ${OPENRAVEPY_RAMPOPTIMIZER_LINKED_LIBRARIES} ${Boost_PYTHON_LIBRARY})
  endif()

  add_library(openravepy_rampoptimizer SHARED openravepy_rampoptimizer.cpp)
  target_include_directories(openravepy_rampoptimizer PUBLIC ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS})
  target_link_libraries(openravepy_rampoptimizer
    PRIVATE
      boost_assertion_failed
    PUBLIC
      ${OPENRAVEPY_RAMPOPTIMIZER_LINKED_LIBRARIES}
      Python2::Module
      Python2::NumPy
      pybind11::module
  )

  set(OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS "")
  if(USE_PYBIND11_PYTHON_BINDINGS)
    message(STATUS "Build openravepy_rampoptimizer with pybind11")
    find_package(pybind11 REQUIRED)
    set(OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS} -DUSE_PYBIND11_PYTHON_BINDINGS")
    if(PYBIND11_PYTHON_BINDINGS_DEBUG)
      set(OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS} -g -O0")
    else()
      set(OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS} -g0 -O3")
    endif()
    # pybind11 requires less visibility
    set(OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS} -fvisibility=hidden")
  else()
    message(STATUS "Build openravepy_rampoptimizer with Boost.Python")
  endif()

  set_target_properties(openravepy_rampoptimizer
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy_rampoptimizer PROPERTIES SUFFIX ".pyd")
  endif()
  install(TARGETS openravepy_rampoptimizer DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
else()
  message(STATUS "HAVE_ALL_PYTHON_HEADERS = ${HAVE_ALL_PYTHON_HEADERS}; do not build openravepy_rampoptimizer")
endif()
