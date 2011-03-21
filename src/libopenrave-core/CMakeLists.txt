check_function_exists(mkstemp HAVE_MKSTEMP)
if( HAVE_MKSTEMP )
  add_definitions("-DHAVE_MKSTEMP")
endif()

set(coreLIBS ${openrave_libraries})
set(openrave_core_SOURCES openrave-core.cpp environment-core.h  openrave-core.h ravep.h   xmlreaders.cpp)

if( COLLADA_FOUND )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT ${COLLADA_CFLAGS_OTHER}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COLLADA_LDFLAGS_OTHER}")
  include_directories(${COLLADA_INCLUDE_DIRS} ${MINIZIP_INCLUDE_DIR})
  set (coreLIBS ${coreLIBS} ${COLLADA_LIBRARIES} minizip)
  set(openrave_core_SOURCES ${openrave_core_SOURCES} colladareader.cpp colladawriter.cpp)
endif()

if( ASSIMP_FOUND )
  include_directories(${ASSIMP_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_LIBRARY_DIRS})
  set(coreLIBS ${coreLIBS} ${ASSIMP_LIBRARIES})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_ASSIMP ${ASSIMP_CFLAGS_OTHER}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${ASSIMP_LDFLAGS_OTHER}")
  message(STATUS "Geometry Parsing: Using Open Asset Import Library (assimp)")
elseif( IVCON_FOUND )
  message(STATUS "Geometry Parsing: Using ivcon")
  include_directories(${IVCON_INCLUDE_DIR})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_IVCON")
  set(coreLIBS ${coreLIBS} ${IVCON_LIBRARY})
endif()
# because assimp cannot import IV/VRML files, still need coin3d
if( COIN_LIBRARY_FOUND )
  include_directories(${COIN_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COIN_LINK_DIRS})
  set(coreLIBS ${coreLIBS} ${COIN_LIBRARY})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_COIN3D ${COIN_CXXFLAGS}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")
  message(STATUS "Geometry Parsing: Using Coin3D")
endif()

link_directories(${OPENRAVE_LINK_DIRS})

if( CLOCK_GETTIME_FOUND )
  set(coreLIBS ${coreLIBS} rt)
endif()

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})
add_dependencies(libopenrave-core libopenrave)

if( COLLADA_FOUND )
  add_dependencies(libopenrave-core ${COLLADA_LIBRARIES})
endif()
if( IVCON_FOUND )
  add_dependencies(libopenrave-core ${IVCON_LIBRARY})
endif()

set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION ${OPENRAVE_SOVERSION}
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_CORE_DLL_EXPORTS -DOPENRAVE_CORE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")

target_link_libraries (libopenrave-core libopenrave ${coreLIBS})
if( MSVC )
  install(TARGETS libopenrave-core RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
else()
  install(TARGETS libopenrave-core DESTINATION lib${LIB_SUFFIX})
endif()
install(FILES openrave-core.h DESTINATION include)

if( OPT_STATIC )
  # visual studio needs static built
  add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

  if( MSVC )
    # static version needs to have different name
    set(LIBOPENRAVE_CORE_NAME libopenrave-core-s)
  else()
    set(LIBOPENRAVE_CORE_NAME openrave-core)
  endif()

  if( COLLADA_FOUND )
    add_dependencies(libopenrave-core_static ${COLLADA_LIBRARIES})
  endif()
  if( IVCON_FOUND )
    add_dependencies(libopenrave-core_static ${IVCON_LIBRARY})
  endif()

  set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_CORE_NAME}
                                                SOVERSION ${OPENRAVE_SOVERSION}
                                                VERSION ${OPENRAVE_VERSION}
                                                CLEAN_DIRECT_OUTPUT 1
                                                COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} "
                                                LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  if( MSVC )
    # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
    # this is such an unbelievable hack, that it's ridiculous
    set_target_properties(libopenrave-core_static PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
  endif()

  target_link_libraries (libopenrave-core_static libopenrave_static ${coreLIBS})
  install(TARGETS libopenrave-core_static DESTINATION lib${LIB_SUFFIX})
endif()
