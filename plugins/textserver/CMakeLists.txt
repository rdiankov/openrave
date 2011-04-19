###########################################
# textserver openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

add_library(textserver SHARED textserver.cpp textserver.h plugindefs.h)

if( MSVC )
  target_link_libraries(textserver libopenrave imm32 winmm ws2_32)
else()
  target_link_libraries(textserver libopenrave)
endif()

add_dependencies(textserver libopenrave)
set_target_properties(textserver PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS textserver DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-textserver)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-TEXTSERVER_DESCRIPTION "Offers a solid infrastructure of basic planning functionality." PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-TEXTSERVER_DEPENDS ${COMPONENT_PREFIX}base PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-textserver PARENT_SCOPE)
