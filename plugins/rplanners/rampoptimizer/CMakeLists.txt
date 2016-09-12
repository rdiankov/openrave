add_library(rampoptimizer STATIC paraboliccommon.h paraboliccommon.cpp ramp.h ramp.cpp parabolicchecker.h parabolicchecker.cpp interpolation.h interpolation.cpp parabolicpath.h parabolicpath.cpp)
set_target_properties(rampoptimizer PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
add_dependencies(rampoptimizer interfacehashes_target)

# add_executable(testramp testramp.cpp)
# target_link_libraries(testramp rampoptimizer libopenrave ${LOG4CXX_LIBRARIES})
# add_executable(testinterpolation testinterpolation.cpp)
# target_link_libraries(testinterpolation rampoptimizer libopenrave ${LOG4CXX_LIBRARIES})