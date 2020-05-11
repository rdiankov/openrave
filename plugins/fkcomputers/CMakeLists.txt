##############################################
# forward kinematics computers openrave plugin
##############################################
if( Boost_IOSTREAMS_FOUND )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DBoost_IOSTREAMS_FOUND")
endif()
if( OPT_IKFAST_FLOAT32 )
  set(PLUGIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} -DOPENRAVE_IKFAST_FLOAT32")
endif()

file(GLOB fkcomputers_cppfiles "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(fkcomputers SHARED "${fkcomputers_cppfiles}")

if( Boost_IOSTREAMS_FOUND )
  target_link_libraries(fkcomputers libopenrave ${Boost_IOSTREAMS_LIBRARY} ${LAPACK_LIBRARIES})
else()
  target_link_libraries(fkcomputers libopenrave ${LAPACK_LIBRARIES})
endif()

target_link_libraries(fkcomputers PRIVATE boost_assertion_failed)
set_target_properties(fkcomputers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")

install(TARGETS fkcomputers DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-fkcomputers)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FKCOMPUTERS_DISPLAY_NAME "Plugin interfaces to forward kinematics computing programs" PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FKCOMPUTERS_DEPENDS ${COMPONENT_PREFIX}fkcomputers)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-fkcomputers PARENT_SCOPE)
