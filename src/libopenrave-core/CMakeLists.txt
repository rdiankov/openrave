check_function_exists(mkstemps HAVE_MKSTEMP)
if( HAVE_MKSTEMP )
  add_definitions("-DHAVE_MKSTEMP")
endif()

set(coreLIBS ${openrave_libraries})

if( COLLADA_FOUND )
  set(LIB_COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT")
  include_directories(${COLLADA_INCLUDE_DIR} ${MINIZIP_INCLUDE_DIR})
  set (coreLIBS ${coreLIBS} ${COLLADA_LIBRARY} minizip)
endif()

if( CLOCK_GETTIME_FOUND )
  set(coreLIBS ${coreLIBS} rt)
endif()

set (openrave_core_SOURCES openrave-core.cpp ivcon.cpp colladareader.h  environment-core.h  openrave-core.h ravep.h   xmlreaders.h colladawriter.h  ivcon.h )

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})
add_dependencies(libopenrave-core libopenrave)

if( COLLADA_FOUND )
  add_dependencies(libopenrave-core ${COLLADA_LIBRARY})
endif()

set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION ${OPENRAVE_SOVERSION}
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DRAVE_CORE_LIBBUILD -DRAVE_CORE_USEDLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")

target_link_libraries (libopenrave-core ${COIN_LIBRARY} libopenrave ${coreLIBS})
if( MSVC )
  install(TARGETS libopenrave-core RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
else()
  install(TARGETS libopenrave-core DESTINATION lib)
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
    add_dependencies(libopenrave-core_static ${COLLADA_LIBRARY})
  endif()

  set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_CORE_NAME}
                                                SOVERSION ${OPENRAVE_SOVERSION}
                                                VERSION ${OPENRAVE_VERSION}
                                                CLEAN_DIRECT_OUTPUT 1
                                                COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                                LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")
  if( MSVC )
    # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
    # this is such an unbelievable hack, that it's disgusting
    set_target_properties(libopenrave-core_static PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
  endif()

  target_link_libraries (libopenrave-core_static ${COIN_LIBRARY} libopenrave_static ${coreLIBS})
  install(TARGETS libopenrave-core_static DESTINATION lib)
endif()
