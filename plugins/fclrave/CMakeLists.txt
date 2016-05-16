###########################################
# fclrave openrave plugin
###########################################
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(FCL fcl)
else()
  message("PackageConfig is supposed to be installed...")
endif()

if(FCL_FOUND)
  link_directories(${OPENRAVE_LINK_DIRS} ${FCL_LIBRARY_DIRS})
  include_directories(${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
  add_library(fclrave SHARED fclrave.cpp fclcollision.h fclspace.h plugindefs.h)
  target_link_libraries(fclrave libopenrave ${FCL_LIBRARIES})
  if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
    add_definitions("-std=c++11")
  endif()
  set_target_properties(fclrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${FCL_CFLAGS_OTHER} -std=c++11" LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
  install(TARGETS fclrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-fclrave)
else()
  message(STATUS "Could not find FCL. Please install FCL (https://github.com/flexible-collision-library/fcl)")
endif()

set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FCLRAVE_DISPLAY_NAME "Plugin for Flexible Collision Library (fcl) Collision" PARENT_SCOPE)
set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-fclrave PARENT_SCOPE)

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
