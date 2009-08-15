###########################################
# taskcaging openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(taskcaging SHARED taskcagingmain.cpp taskcagingproblem.cpp)

target_link_libraries(taskcaging libopenrave)
include_directories(taskcaging "${CMAKE_SOURCE_DIR}")
add_dependencies(taskcaging libopenrave libopenrave_static)
set_target_properties(taskcaging PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                            LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS taskcaging DESTINATION share/openrave/plugins )
