###########################################
# fclrave openrave plugin
###########################################

find_package(libccd REQUIRED)
find_package(fcl REQUIRED)

if(NARROW_COLLISION_CACHING)
  add_definitions(-DNARROW_COLLISION_CACHING)
endif()

if(FCL_USE_STATISTICS)
  add_definitions(-DFCLUSESTATISTICS)
  if(FCL_STATISTICS_DISPLAY_CONTINUOUSLY)
    add_definitions(-DFCL_STATISTICS_DISPLAY_CONTINUOUSLY)
  endif()
endif()

if(FCLRAVE_USE_COLLISION_STATISTICS)
  add_definitions(-DFCLRAVE_COLLISION_OBJECTS_STATISTICS)
endif()

if(FCLRAVE_DEBUG_COLLISION_OBJECTS)
  add_definitions(-DFCLRAVE_DEBUG_COLLISION_OBJECTS)
endif()

if( FCL_SUPPORT_BULK_UPDATE )
    add_definitions(-DFCLRAVE_USE_BULK_UPDATE)
endif()

link_directories(${OPENRAVE_LINK_DIRS})
add_library(fclrave SHARED fclrave.cpp fclcollision.h fclstatistics.h fclspace.h plugindefs.h)
target_link_libraries(fclrave PRIVATE boost_assertion_failed PUBLIC libopenrave fcl::fcl_static libccd::ccd_static)
# ${FCL_CFLAGS_OTHER} is useless as CMAKE_CXX_STANDARD now requires 14
set_target_properties(fclrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
install(TARGETS fclrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-fclrave)

set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FCLRAVE_DISPLAY_NAME "Plugin for Flexible Collision Library (fcl) Collision" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-fclrave PARENT_SCOPE)

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
