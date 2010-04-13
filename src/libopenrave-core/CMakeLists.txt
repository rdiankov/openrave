if( MSVC )
  set (coreLIBS imm32 winmm ws2_32)
endif()

if( ZLIB_FOUND AND Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  set(LIB_COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT")
  include_directories(${COLLADA_INCLUDE_DIRS})
  set(USING_COLLADA 1)
  set (coreLIBS ${coreLIBS} collada15reader)
endif()

if( Boost_FILESYSTEM_FOUND )
  set(coreLIBS ${coreLIBS} ${Boost_FILESYSTEM_LIBRARY})
  add_definitions(-DHAVE_BOOST_FILESYSTEM)
endif()

set (openrave_core_SOURCES openrave-core.cpp ivcon.cpp colladareader.h  environment.h  openrave-core.h ravep.h   xmlreaders.h colladawriter.h  ivcon.h  plugindatabase.h  server.h)

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})
add_dependencies(libopenrave-core libopenrave)

if( USING_COLLADA )
  add_dependencies(libopenrave-core collada15reader)
endif()

set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION ${OPENRAVE_SOVERSION}
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DRAVE_CORE_LIBBUILD -DRAVE_CORE_USEDLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")

target_link_libraries (libopenrave-core ${Boost_THREAD_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave ${coreLIBS})
if( MSVC )
  install(TARGETS libopenrave-core RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
else()
  install(TARGETS libopenrave-core DESTINATION lib)
endif()
install(FILES openrave-core.h DESTINATION include)

# visual studio needs static built
add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

if( MSVC )
  # static version needs to have different name
  set(LIBOPENRAVE_CORE_NAME libopenrave-core-s)
else()
  set(LIBOPENRAVE_CORE_NAME openrave-core)
endif()

if( USING_COLLADA )
  add_dependencies(libopenrave-core_static collada15reader)
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

target_link_libraries (libopenrave-core_static ${Boost_THREAD_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave_static ${coreLIBS})
install(TARGETS libopenrave-core_static DESTINATION lib)
