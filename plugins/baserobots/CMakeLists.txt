###########################################
# baserobots openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(baserobots SHARED baserobotsmain.cpp genericrobot.h collisionmaprobot.h plugindefs.h)

target_link_libraries(baserobots libopenrave)
include_directories(baserobots "${CMAKE_SOURCE_DIR}")
add_dependencies(baserobots libopenrave)
set_target_properties(baserobots PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                 LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS baserobots DESTINATION share/openrave/plugins )
