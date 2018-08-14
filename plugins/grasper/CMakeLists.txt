###########################################
# grasper openrave plugin
###########################################
if( QHULL_FOUND AND QHULL_INCLUDE_DIR )
  include_directories("${QHULL_INCLUDE_DIR}")
endif()

add_library(grasper SHARED grasper.cpp graspermodule.cpp grasperplanner.cpp  plugindefs.h)

if ( QHULL_FOUND )
  add_definitions(-DQHULL_FOUND)
  if( QHULL_USE_REENTRANT )
    add_definitions(-DQHULL_USE_REENTRANT)
    set(GRASPER_LINK_LIBS libopenrave qhull_r)
  else()
    set(GRASPER_LINK_LIBS libopenrave qhull)
  endif()
else()
  set(GRASPER_LINK_LIBS libopenrave)
endif()

target_link_libraries(grasper PUBLIC ${GRASPER_LINK_LIBS}
                              PRIVATE boost_assertion_failed)

set_target_properties(grasper PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS grasper DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-grasper)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-GRASPER_DISPLAY_NAME "Plugin for basic force-closure grasping" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-grasper PARENT_SCOPE)
