  add_library(openravepy3_piecewisepolynomials SHARED ../openravepy_piecewisepolynomials.cpp)
  target_include_directories(openravepy3_piecewisepolynomials PUBLIC ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS})
  target_link_libraries(openravepy3_piecewisepolynomials
    PRIVATE
      boost_assertion_failed
    PUBLIC
      ${OPENRAVEPY_PIECEWISEPOLYNOMIALS_LINKED_LIBRARIES}
      Python3::Module
      Python3::NumPy
      pybind11::module
    )

  set_target_properties(openravepy3_piecewisepolynomials
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy3_piecewisepolynomials PROPERTIES SUFFIX ".pyd")
  endif()
  set_target_properties(openravepy3_piecewisepolynomials PROPERTIES OUTPUT_NAME openravepy_piecewisepolynomials)
  install(TARGETS openravepy3_piecewisepolynomials DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
