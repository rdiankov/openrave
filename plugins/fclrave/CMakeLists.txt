###########################################
# fclrave openrave plugin
###########################################
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(FCL fcl)
else()
  message("PackageConfig is supposed to be installed...")
endif()

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


if( FCL_FOUND )
  STRING( REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" FCL_MINOR_VERSION ${FCL_VERSION})

  if(FCL_MINOR_VERSION GREATER "4")
    set(CMAKE_REQUIRED_INCLUDES ${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
    set(CMAKE_REQUIRED_FLAGS "${PLUGIN_COMPILE_FLAGS} ${FCL_CFLAGS_OTHER} -std=c++11 ${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
    set(CMAKE_REQUIRED_LIBRARIES fcl)
    check_cxx_source_compiles("
      #include <fcl/broadphase/broadphase.h>

      int main() {
        fcl::NaiveCollisionManager manager;
        manager.replaceObject(nullptr, nullptr, false);
        return 0;
      }"
      FCL_HAS_REPLACEOBJECT)

    if( FCL_HAS_REPLACEOBJECT )
      add_definitions(-DFCLRAVE_USE_REPLACEOBJECT)
    endif()


    check_cxx_source_compiles("
      #include <fcl/broadphase/broadphase.h>

      int main() {
        fcl::DynamicAABBTreeCollisionManager manager;
        std::vector<fcl::CollisionObject*> v;
        manager.update(v, false);
        return 0;
      }"
      FCL_SUPPORT_BULK_UPDATE)

    if( FCL_SUPPORT_BULK_UPDATE )
      add_definitions(-DFCLRAVE_USE_BULK_UPDATE)
    endif()

    link_directories(${OPENRAVE_LINK_DIRS} ${FCL_LIBRARY_DIRS})
    include_directories(${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
    add_library(fclrave SHARED fclrave.cpp fclcollision.h fclstatistics.h fclspace.h plugindefs.h)
    target_link_libraries(fclrave libopenrave ${FCL_LIBRARIES})
    target_link_libraries(fclrave PRIVATE boost_assertion_failed)
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
endif()

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
