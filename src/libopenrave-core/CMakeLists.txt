check_function_exists(mkstemp HAVE_MKSTEMP)
if( HAVE_MKSTEMP )
  add_definitions("-DHAVE_MKSTEMP")
endif()

set(OPENRAVE_CORE_LIBRARIES ${openrave_libraries})
set(openrave_core_SOURCES openrave-core.cpp environment-core.h openrave-core.h ravep.h xmlreaders.cpp genericcollisionchecker.cpp genericphysicsengine.cpp genericrobot.cpp generictrajectory.cpp)

if( COLLADA_DOM_FOUND )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT ${COLLADA_DOM_CFLAGS_OTHER}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COLLADA_DOM_LDFLAGS_OTHER}")
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_DOM_LIBRARY_DIRS})
  include_directories(${COLLADA_DOM_INCLUDE_DIRS} ${MINIZIP_INCLUDE_DIR})
  set (OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${COLLADA_DOM_LIBRARIES} minizip)
  set(openrave_core_SOURCES ${openrave_core_SOURCES} colladareader.cpp colladawriter.cpp)
endif()

if( ASSIMP_FOUND )
  include_directories(${ASSIMP_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${ASSIMP_LIBRARY_DIRS})
  set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${ASSIMP_LIBRARIES})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_ASSIMP ${ASSIMP_CFLAGS_OTHER}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${ASSIMP_LDFLAGS_OTHER}")
  message(STATUS "Geometry Parsing: Using Open Asset Import Library (assimp)")
endif()

if( IVCON_FOUND )
  message(STATUS "Geometry Parsing: Using ivcon")
  include_directories(${IVCON_INCLUDE_DIR})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_IVCON")
  set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${IVCON_LIBRARY})
endif()

link_directories(${OPENRAVE_LINK_DIRS})

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})

if( COLLADA_DOM_FOUND )
  set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${COLLADA_DOM_LIBRARIES})
endif()
if( IVCON_FOUND )
  set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${IVCON_LIBRARY})
endif()

set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core${OPENRAVE_LIBRARY_SUFFIX}
                                  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_CORE_DLL_EXPORTS -DOPENRAVE_CORE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")

target_link_libraries (libopenrave-core libopenrave ${OPENRAVE_CORE_LIBRARIES})
if( MSVC )
  install(TARGETS libopenrave-core RUNTIME DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base LIBRARY DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}base)
else()
  install(TARGETS libopenrave-core DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}base)
endif()
install(FILES openrave-core.h DESTINATION include/openrave-${OPENRAVE_VERSION_MAJOR}.${OPENRAVE_VERSION_MINOR} COMPONENT ${COMPONENT_PREFIX}dev)

if( OPT_STATIC )
  # visual studio needs static built
  add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

  if( MSVC )
    # static version needs to have different name
    set(LIBOPENRAVE_CORE_NAME libopenrave-core${OPENRAVE_LIBRARY_SUFFIX})
  else()
    set(LIBOPENRAVE_CORE_NAME openrave-core${OPENRAVE_LIBRARY_SUFFIX})
  endif()

  set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_CORE_NAME}
                                                SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                                VERSION ${OPENRAVE_VERSION}
                                                CLEAN_DIRECT_OUTPUT 1
                                                COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} "
                                                LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  
  target_link_libraries (libopenrave-core_static libopenrave_static ${OPENRAVE_CORE_LIBRARIES})
  install(TARGETS libopenrave-core_static DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}dev)
endif()
