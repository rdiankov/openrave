###########################################
# basesensors openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basesensors SHARED basesensorsmain.cpp basecamera.h  baseflashlidar3d.h  baselaser.h plugindefs.h)

target_link_libraries(basesensors libopenrave)
include_directories(basesensors "${CMAKE_SOURCE_DIR}")
add_dependencies(basesensors libopenrave)
set_target_properties(basesensors PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                  LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basesensors DESTINATION share/openrave/plugins )
