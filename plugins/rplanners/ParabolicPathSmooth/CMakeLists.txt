add_library(ParabolicPathSmooth STATIC DynamicPath.cpp ParabolicRamp.cpp Timer.cpp DynamicPath.h ParabolicRamp.h Math.h Timer.h)
set_target_properties(ParabolicPathSmooth PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
add_dependencies(ParabolicPathSmooth interfacehashes_target)
