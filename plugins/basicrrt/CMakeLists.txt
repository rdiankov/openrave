###########################################
# basicrrt openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basicrrt SHARED basicrrtmain.cpp basicrrt.cpp)

target_link_libraries(basicrrt libopenrave)
include_directories(basicrrt "${CMAKE_SOURCE_DIR}")
add_dependencies(basicrrt libopenrave libopenrave_static)
set_target_properties(basicrrt PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                               LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS basicrrt DESTINATION share/openrave/plugins )
