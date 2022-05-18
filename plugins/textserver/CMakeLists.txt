###########################################
# textserver openrave plugin
###########################################
add_library(textserver SHARED textserver.cpp textserver.h plugindefs.h)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_link_libraries(textserver PRIVATE boost_assertion_failed PUBLIC libopenrave imm32 winmm ws2_32)
else()
  target_link_libraries(textserver PRIVATE boost_assertion_failed PUBLIC libopenrave)
endif()

set_target_properties(textserver PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS textserver DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-textserver)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-TEXTSERVER_DISPLAY_NAME "Plugin for a text socket-based server" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-textserver PARENT_SCOPE)
