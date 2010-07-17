###########################################
# logging openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(logging SHARED logging.cpp loggingproblem.h plugindefs.h)

target_link_libraries(logging libopenrave)
include_directories(logging "${CMAKE_SOURCE_DIR}")
add_dependencies(logging libopenrave)
set_target_properties(logging PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                              LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS logging DESTINATION share/openrave/plugins )
