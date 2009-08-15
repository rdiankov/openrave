###########################################
# basesensors openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basesensors SHARED basecamera.cpp baseflashlidar3d.cpp baselaser.cpp basesensorsmain.cpp)

target_link_libraries(basesensors libopenrave ${PTHREADS_LIBRARY})
include_directories(basesensors "${CMAKE_SOURCE_DIR}")
add_dependencies(basesensors libopenrave libopenrave_static)
set_target_properties(basesensors PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                  LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basesensors DESTINATION share/openrave/plugins )
