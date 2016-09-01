###########################################
# grasper openrave plugin
###########################################
if( QHULL_FOUND AND QHULL_INCLUDE_DIR )
  include_directories("${QHULL_INCLUDE_DIR}")
endif()

add_library(grasper SHARED grasper.cpp graspermodule.cpp grasperplanner.cpp  plugindefs.h)

if( QHULLCPP_FOUND )
  check_library_exists(qhull_rd qh_new_qhull "" QHULL_REENTRANT_DEBUG_FOUND_LIB)
  if( QHULL_REENTRANT_DEBUG_FOUND_LIB )
    target_link_libraries(grasper libopenrave qhull_rd qhullcpp)
  else()
    target_link_libraries(grasper libopenrave qhull_r qhullcpp)
  endif()
  add_definitions(-DQHULLCPP_FOUND)
else()
  if( QHULL_FOUND )
    target_link_libraries(grasper libopenrave qhull)
    add_definitions(-DQHULL_FOUND)
  else()
    target_link_libraries(grasper libopenrave)
  endif()
endif()

set_target_properties(grasper PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS grasper DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-grasper)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-GRASPER_DISPLAY_NAME "Plugin for basic force-closure grasping" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-grasper PARENT_SCOPE)
