add_library(rampoptimizer STATIC paraboliccommon.h paraboliccommon.cpp ramp.h ramp.cpp interpolator.h interpolator.cpp feasibilitychecker.h feasibilitychecker.cpp parabolicchecker.h parabolicchecker.cpp)
set_target_properties(rampoptimizer PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")		
add_dependencies(rampoptimizer interfacehashes_target)		
