set(PQP_SRCS Build.cpp
            BV.cpp
            TriDist.cpp
            PQP.cpp)

# build sources, moc'd sources, and rcc'd sources
add_library(PQP STATIC ${PQP_SRCS})
set_target_properties(PQP PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
