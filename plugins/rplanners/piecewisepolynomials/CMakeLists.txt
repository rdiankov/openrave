add_library(piecewisepolynomials STATIC polynomialtrajectory.h polynomialtrajectory.cpp)
set_target_properties(piecewisepolynomials PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")		
add_dependencies(piecewisepolynomials interfacehashes_target)		
