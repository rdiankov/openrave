###########################################
# logging openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(logging SHARED loggingmain.cpp)

target_link_libraries(logging libopenrave ${PTHREADS_LIBRARY})
include_directories(logging "${CMAKE_SOURCE_DIR}")
add_dependencies(logging libopenrave libopenrave_static)
set_target_properties(logging PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                              LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS logging DESTINATION share/openrave/plugins )
