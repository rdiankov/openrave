###########################################
# bulletrave openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

pkg_check_modules(bullet bullet)

if(bullet_FOUND)
  include_directories(${CMAKE_SOURCE_DIR} ${bullet_INCLUDE_DIRS})
  link_directories(${OPENRAVE_LINK_DIRS} ${bullet_LIBRARY_DIRS})

  add_library(bulletrave SHARED bulletrave.cpp bulletcollision.h  bulletspace.h  plugindefs.h)

  #message(STATUS "Bullet found ${bullet_INCLUDE_DIRS}, libs: ${bullet_LIBRARIES}, cflags=${bullet_CFLAGS_OTHER}, lflags=${bullet_LDFLAGS_OTHER}, building bulletrave plugin")

  target_link_libraries(bulletrave libopenrave ${bullet_LIBRARIES})
  add_dependencies(bulletrave libopenrave)

  # for some reason the flags come out as a list, so convert to a string
  set(openrave_bullet_CFLAGS_OTHER)
  foreach(arg ${bullet_CFLAGS_OTHER})
    set(openrave_bullet_CFLAGS_OTHER "${openrave_bullet_CFLAGS_OTHER} ${arg}")
  endforeach()
  set(openrave_bullet_LDFLAGS_OTHER)
  foreach(arg ${bullet_LDFLAGS_OTHER})
    set(openrave_bullet_LDFLAGS_OTHER "${openrave_bullet_LDFLAGS_OTHER} ${arg}")
  endforeach()

  set_target_properties(bulletrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${openrave_bullet_CFLAGS_OTHER}"
                                LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${openrave_bullet_LDFLAGS_OTHER}")

  install(TARGETS bulletrave DESTINATION share/openrave/plugins )

#  link_directories(${OPENRAVE_LINK_DIRS})
else()
  message(STATUS "Could not find Bullet Physics/Collision engine. Please install ver 2.74+ from http://www.bulletphysics.com")
endif()
