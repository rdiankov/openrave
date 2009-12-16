###########################################
# basecontrollers openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basecontrollers SHARED basecontrollersmain.cpp)

target_link_libraries(basecontrollers libopenrave)
include_directories(basecontrollers "${CMAKE_SOURCE_DIR}")
add_dependencies(basecontrollers libopenrave)
set_target_properties(basecontrollers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basecontrollers DESTINATION share/openrave/plugins )
