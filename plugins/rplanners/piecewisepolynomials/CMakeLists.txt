find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(piecewisepolynomials STATIC polynomialcommon.h polynomialtrajectory.h polynomialtrajectory.cpp polynomialchecker.h polynomialchecker.cpp interpolatorbase.h cubicinterpolator.h cubicinterpolator.cpp quinticinterpolator.h quinticinterpolator.cpp feasibilitychecker.h generalrecursiveinterpolator.h generalrecursiveinterpolator.cpp)
target_link_libraries(piecewisepolynomials PRIVATE boost_assertion_failed PUBLIC libopenrave)
set_target_properties(piecewisepolynomials PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

install(TARGETS piecewisepolynomials DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

add_dependencies(piecewisepolynomials interfacehashes_target)		

# python bindings
if( OPT_PYTHON )
  # link
  set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_LINKED_LIBRARIES openravepy_int piecewisepolynomials rampoptimizer ${PYTHON_LIBRARIES} ${Boost_THREAD_LIBRARY} ${LAPACK_LIBRARIES})
  if(NOT USE_PYBIND11_PYTHON_BINDINGS)
    set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_LINKED_LIBRARIES ${OPENRAVEPY_PIECEWISEPOLYNOMIALS_LINKED_LIBRARIES} ${Boost_PYTHON_LIBRARY})
  endif()

  add_library(openravepy_piecewisepolynomials SHARED openravepy_piecewisepolynomials.cpp)
  target_include_directories(openravepy_piecewisepolynomials PUBLIC ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS} ${OPENRAVEPY_INCLUDE_LOCAL_DIRS})
  target_link_libraries(openravepy_piecewisepolynomials
    PRIVATE
      boost_assertion_failed
    PUBLIC
      ${OPENRAVEPY_PIECEWISEPOLYNOMIALS_LINKED_LIBRARIES}
      Python2::Module
      Python2::NumPy
      pybind11::module
    )

  set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS "")
  if(USE_PYBIND11_PYTHON_BINDINGS)
    message(STATUS "Build openravepy_piecewisepolynomials with pybind11")
    find_package(pybind11 REQUIRED)
    set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS} -DUSE_PYBIND11_PYTHON_BINDINGS")
    if(PYBIND11_PYTHON_BINDINGS_DEBUG)
      set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS} -g -O0")
    else()
      set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS} -g0 -O3")
    endif()
    # pybind11 requires less visibility
    set(OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS} -fvisibility=hidden")
  else()
    message(STATUS "Build openravepy_piecewisepolynomials with Boost.Python")
  endif()

  set_target_properties(openravepy_piecewisepolynomials
    PROPERTIES PREFIX "" COMPILE_FLAGS "${OPENRAVEPY_PIECEWISEPOLYNOMIALS_COMPILE_FLAGS}"
  )
  if( WIN32 )
    set_target_properties(openravepy_piecewisepolynomials PROPERTIES SUFFIX ".pyd")
  endif()
  install(TARGETS openravepy_piecewisepolynomials DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
else()
  message(STATUS "HAVE_ALL_PYTHON_HEADERS = ${HAVE_ALL_PYTHON_HEADERS}; do not build openravepy_piecewisepolynomials")
endif()
