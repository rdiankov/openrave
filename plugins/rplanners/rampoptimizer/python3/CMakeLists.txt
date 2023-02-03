  add_library(openravepy3_rampoptimizer SHARED ../openravepy_rampoptimizer.cpp)
  target_include_directories(openravepy3_rampoptimizer PUBLIC ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS})
  target_link_libraries(openravepy3_rampoptimizer
    PRIVATE
      boost_assertion_failed
    PUBLIC
      ${OPENRAVEPY_RAMPOPTIMIZER_LINKED_LIBRARIES}
      Python3::Module
      Python3::NumPy
      pybind11::module
  )

  set_target_properties(openravepy3_rampoptimizer
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_RAMPOPTIMIZER_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy3_rampoptimizer PROPERTIES SUFFIX ".pyd")
  endif()
  set_target_properties(openravepy3_rampoptimizer PROPERTIES OUTPUT_NAME openravepy_rampoptimizer)
  install(TARGETS openravepy3_rampoptimizer DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
