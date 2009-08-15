###########################################
# birrt openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(birrt SHARED birrtmain.cpp birrt.cpp)

target_link_libraries(birrt libopenrave)
include_directories(birrt "${CMAKE_SOURCE_DIR}")
add_dependencies(birrt libopenrave libopenrave_static)
set_target_properties(birrt PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                            LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS birrt DESTINATION share/openrave/plugins )
