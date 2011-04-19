###########################################
# basecontrollers openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(basecontrollers SHARED basecontrollers.cpp basecontrollers.h plugindefs.h)

target_link_libraries(basecontrollers libopenrave)
add_dependencies(basecontrollers libopenrave)
set_target_properties(basecontrollers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}" OUTPUT_NAME basecontrollers)
install(TARGETS basecontrollers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
