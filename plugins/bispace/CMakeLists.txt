###########################################
# bispace openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(bispace SHARED bispacemain.cpp bispace.cpp profiler.cpp)

target_link_libraries(bispace libopenrave)
include_directories(bispace "${CMAKE_SOURCE_DIR}")
add_dependencies(bispace libopenrave libopenrave_static)
set_target_properties(bispace PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                              LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS bispace DESTINATION share/openrave/plugins )
