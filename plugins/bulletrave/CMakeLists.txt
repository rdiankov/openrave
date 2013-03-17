###########################################
# bulletrave openrave plugin
###########################################
if( OPT_BULLET )
  pkg_check_modules(bullet bullet)

  if(bullet_FOUND)
    if( "${bullet_VERSION}" VERSION_GREATER "2.80" )
      message(STATUS "unsupported bullet version ${bullet_VERSION}")
    else()
      include_directories(${bullet_INCLUDE_DIRS})
      link_directories(${OPENRAVE_LINK_DIRS} ${bullet_LIBRARY_DIRS})

      add_library(bulletrave SHARED bulletrave.cpp bulletcollision.cpp bulletphysics.cpp  bulletspace.h  plugindefs.h)

      #message(STATUS "Bullet found ${bullet_INCLUDE_DIRS}, libs: ${bullet_LIBRARIES}, cflags=${bullet_CFLAGS_OTHER}, lflags=${bullet_LDFLAGS_OTHER}, building bulletrave plugin")

      target_link_libraries(bulletrave libopenrave ${bullet_LIBRARIES})

      # for some reason the flags come out as a list, so convert to a string
      set(openrave_bullet_CFLAGS_OTHER)
      foreach(arg ${bullet_CFLAGS_OTHER})
        set(openrave_bullet_CFLAGS_OTHER "${openrave_bullet_CFLAGS_OTHER} ${arg}")
      endforeach()
      set(openrave_bullet_LDFLAGS_OTHER)
      foreach(arg ${bullet_LDFLAGS_OTHER})
        set(openrave_bullet_LDFLAGS_OTHER "${openrave_bullet_LDFLAGS_OTHER} ${arg}")
      endforeach()

      set_target_properties(bulletrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${openrave_bullet_CFLAGS_OTHER}" LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${openrave_bullet_LDFLAGS_OTHER}")

      install(TARGETS bulletrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-bulletrave)
    endif()
  else()
    message(STATUS "Could not find Bullet Physics/Collision engine. Please install ver 2.80 from http://www.bulletphysics.com")
  endif()
  
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-BULLETRAVE_DISPLAY_NAME "Plugin for Bullet Collision/Physics" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-bulletrave PARENT_SCOPE)
endif()
