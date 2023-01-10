add_library(ParabolicPathSmooth STATIC DynamicPath.cpp paraboliccommon.cpp pramp.cpp pramp.h ppramp.cpp ppramp.h plpramp.cpp plpramp.h ParabolicRamp.cpp Timer.cpp DynamicPath.h ParabolicRamp.h paraboliccommon.h Timer.h)
set_target_properties(ParabolicPathSmooth PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

target_link_libraries(ParabolicPathSmooth PRIVATE boost_assertion_failed PUBLIC libopenrave)

add_executable(testparabolicramp testparabolicramp.cpp)
target_link_libraries(testparabolicramp ParabolicPathSmooth libopenrave ${LOG4CXX_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
