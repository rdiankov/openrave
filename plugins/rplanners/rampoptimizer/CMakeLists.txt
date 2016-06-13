# add_library(rampoptimizer STATIC paraboliccommon.cpp ramp.cpp interpolation.cpp paraboliccommon.h ramp.h interpolation.h)
# set_target_properties(rampoptimizer PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
# add_dependencies(rampoptimizer interfacehashes_target)

# add_executable(testramp testramp.cpp)
# target_link_libraries(testramp rampoptimizer libopenrave ${LOG4CXX_LIBRARIES})


add_library(ParabolicPathSmooth STATIC DynamicPath.cpp paraboliccommon.cpp pramp.cpp pramp.h ppramp.cpp ppramp.h plpramp.cpp plpramp.h ParabolicRamp.cpp Timer.cpp DynamicPath.h ParabolicRamp.h paraboliccommon.h Timer.h ramp.h ramp.cpp interpolation.h interpolation.cpp)
set_target_properties(ParabolicPathSmooth PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
add_dependencies(ParabolicPathSmooth interfacehashes_target)

add_executable(testparabolicramp testparabolicramp.cpp)
target_link_libraries(testparabolicramp ParabolicPathSmooth libopenrave ${LOG4CXX_LIBRARIES})

add_executable(testramp testramp.cpp)
target_link_libraries(testramp ParabolicPathSmooth libopenrave ${LOG4CXX_LIBRARIES})