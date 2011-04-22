###########################################
# textserver openrave plugin
###########################################
add_library(textserver SHARED textserver.cpp textserver.h plugindefs.h)

if( MSVC )
  target_link_libraries(textserver libopenrave imm32 winmm ws2_32)
else()
  target_link_libraries(textserver libopenrave)
endif()

set_target_properties(textserver PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS textserver DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-textserver)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-TEXTSERVER_DISPLAY_NAME "Plugin for a text socket-based server" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-textserver PARENT_SCOPE)
