find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(piecewisepolynomials STATIC polynomialcommon.h polynomialtrajectory.h polynomialtrajectory.cpp polynomialchecker.h polynomialchecker.cpp interpolatorbase.h cubicinterpolator.h cubicinterpolator.cpp quinticinterpolator.h quinticinterpolator.cpp feasibilitychecker.h generalrecursiveinterpolator.h generalrecursiveinterpolator.cpp)
target_link_libraries(piecewisepolynomials PRIVATE boost_assertion_failed PUBLIC rampoptimizer libopenrave)
set_target_properties(piecewisepolynomials PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

install(TARGETS piecewisepolynomials DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})

add_dependencies(piecewisepolynomials interfacehashes_target)		

if (OPT_PYTHON)
    add_library(openravepy_piecewisepolynomials SHARED openravepy_piecewisepolynomials.cpp)
    target_link_libraries(openravepy_piecewisepolynomials PRIVATE boost_assertion_failed PUBLIC piecewisepolynomials openravepy_int openravepy2_meta)
    set_target_properties(openravepy_piecewisepolynomials PROPERTIES PREFIX "")
    install(TARGETS openravepy_piecewisepolynomials DESTINATION ${OPENRAVEPY2_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()

if (OPT_PYTHON3)
    add_library(openravepy3_piecewisepolynomials SHARED openravepy_piecewisepolynomials.cpp)
    target_link_libraries(openravepy3_piecewisepolynomials PRIVATE boost_assertion_failed PUBLIC piecewisepolynomials openravepy3_int openravepy3_meta)
    set_target_properties(openravepy3_piecewisepolynomials PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "python3"
        OUTPUT_NAME "openravepy_piecewisepolynomials"
    )
    install(TARGETS openravepy3_piecewisepolynomials DESTINATION ${OPENRAVEPY3_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
endif()
