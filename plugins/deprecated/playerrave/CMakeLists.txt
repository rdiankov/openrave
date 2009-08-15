###########################################
# playerrave openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(playerrave SHARED playerravemain.cpp)# playerproblem.cpp playercontroller.cpp simpleplayercontroller.cpp)

target_link_libraries(playerrave libopenrave)
include_directories(playerrave "${CMAKE_SOURCE_DIR}")
add_dependencies(playerrave libopenrave libopenrave_static)
set_target_properties(playerrave PROPERTIES COMPILE_FLAGS "-DRAVE_NOPLAYER ${PLUGIN_COMPILE_FLAGS}"
                                 LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS playerrave DESTINATION share/openrave/plugins )
