check_function_exists(mkstemp HAVE_MKSTEMP)
if( HAVE_MKSTEMP )
  add_definitions("-DHAVE_MKSTEMP")
endif()

set(OPENRAVE_CORE_LIBRARIES ${openrave_libraries} ${OPENRAVE_CURL_LIBRARIES})
set(OPENRAVE_CORE_STATIC_LIBRARIES ${openrave_static_libraries})
set(openrave_core_SOURCES openrave-core.cpp environment-core.h openrave-core.h ravep.h  xmlreaders-core.cpp genericcollisionchecker.cpp genericphysicsengine.cpp genericrobot.cpp multicontroller.cpp generictrajectory.cpp jsonparser/gpgutils.cpp jsonparser/jsonreader.cpp jsonparser/jsonwriter.cpp jsonparser/jsondownloader.cpp)

if( libpcrecpp_FOUND )
  # pcre for url parsing
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${libpcrecpp_CFLAGS_OTHER}")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${libpcrecpp_LDFLAGS_OTHER}")
  if( TARGET "${libpcrecpp_LIBRARIES}" )
    get_target_property(libpcrecpp_TYPE "${libpcrecpp_LIBRARIES}" TYPE)
  endif()
  if( libpcrecpp_TYPE STREQUAL STATIC_LIBRARY )
    set(OPENRAVE_CORE_STATIC_LIBRARIES ${OPENRAVE_CORE_STATIC_LIBRARIES} ${libpcrecpp_LIBRARIES})
  else()
    set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${libpcrecpp_LIBRARIES})
  endif()
  include_directories(${libpcrecpp_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${libpcrecpp_LIBRARY_DIRS})
  add_definitions(-DHAVE_PCRECPP)
endif()

if( COLLADA_DOM_FOUND )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT ${COLLADA_DOM_CFLAGS_OTHER}")
  if( COLLADA_SUPPORT_WRITE_MEMORY )
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_COLLADA_SUPPORT_WRITE_MEMORY")
  endif()
  
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COLLADA_DOM_LDFLAGS_OTHER}")
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_DOM_LIBRARY_DIRS})
  include_directories(SYSTEM ${COLLADA_DOM_INCLUDE_DIRS})

  if( TARGET "${COLLADA_DOM_LIBRARIES}" )
    get_target_property(COLLADA_DOM_TYPE "${COLLADA_DOM_LIBRARIES}" TYPE)
  endif()
  if( COLLADA_DOM_TYPE STREQUAL STATIC_LIBRARY)
    set (OPENRAVE_CORE_STATIC_LIBRARIES ${OPENRAVE_CORE_STATIC_LIBRARIES} ${COLLADA_DOM_LIBRARIES})
  else()
    set (OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${COLLADA_DOM_LIBRARIES})
  endif()
  message(STATUS "Geometry Parsing: Collada DOM ${COLLADA_DOM_VERSION}")
  set(openrave_core_SOURCES ${openrave_core_SOURCES} colladaparser/colladareader.cpp colladaparser/colladawriter.cpp colladaparser/colladacommon.cpp)
endif()

if( ASSIMP_FOUND )
  include_directories(${ASSIMP_INCLUDE_DIRS})
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${ASSIMP_LIBRARY_DIRS})
  set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${ASSIMP_LIBRARIES})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_ASSIMP ${ASSIMP_CXX_FLAGS} ${ASSIMP_CFLAGS_OTHER}")
  if( IS_ASSIMP_PRE_R896 )
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_ASSIMP_PRE_R896")
  endif()
  if( IS_ASSIMP3 )
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_IS_ASSIMP3")
  endif()
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${ASSIMP_LINK_FLAGS} ${ASSIMP_LDFLAGS_OTHER}")
  message(STATUS "Geometry Parsing: Using Open Asset Import Library ${ASSIMP_VERSION}")

  if( ZLIB_FOUND )
    include_directories(${ZLIB_INCLUDE_DIR})
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_HAS_ZLIB")
    set(OPENRAVE_CORE_LIBRARIES ${OPENRAVE_CORE_LIBRARIES} ${ZLIB_LIBRARIES})
  endif()
  set(openrave_core_SOURCES ${openrave_core_SOURCES}  xfileparser/XFileParser.cpp xfileparser/XFileBindings.cpp)
endif()

if( IVCON_FOUND )
  message(STATUS "Geometry Parsing: Using ivcon")
  include_directories(${IVCON_INCLUDE_DIR})
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_IVCON")
endif()

link_directories(${OPENRAVE_LINK_DIRS})

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})
target_include_directories(libopenrave-core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}-core
                                  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                  VERSION ${OPENRAVE_VERSION}
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_CORE_DLL_EXPORTS -DOPENRAVE_CORE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave-core
  PRIVATE boost_assertion_failed ${IVCON_LIBRARY} ${OPENRAVE_CORE_STATIC_LIBRARIES} openrave_gpgme
  PUBLIC libopenrave ${OPENRAVE_CORE_LIBRARIES}
)
install(TARGETS libopenrave-core
  EXPORT openrave-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
)
install(FILES openrave-core.h DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}dev)

if( OPT_CBINDINGS )
  add_library(libopenrave-core_c SHARED openrave-core_c.cpp)
  add_dependencies(libopenrave-core_c libopenrave-core)
  # remove "${LIBOPENRAVE_LINK_FLAGS} until can figure out symbol visibility issue
  set_target_properties(libopenrave-core_c PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}-core_c 
   SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
    VERSION ${OPENRAVE_VERSION}
    COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_C_CORE_DLL_EXPORTS -DOPENRAVE_C_CORE_DLL -DOPENRAVE_CORE_DLL"
    LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  target_link_libraries (libopenrave-core_c
    PRIVATE boost_assertion_failed
    PUBLIC libopenrave-core
  )
  install(TARGETS libopenrave-core_c
    EXPORT openrave-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
  )
  install(FILES openrave-core_c.h DESTINATION include/${OPENRAVE_INCLUDE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}cbindings-dev)
endif()

if( OPT_STATIC )
  # visual studio needs static built
  add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

  if( MSVC )
    # static version needs to have different name
    set(LIBOPENRAVE_CORE_NAME libopenrave${OPENRAVE_LIBRARY_SUFFIX}-core)
  else()
    set(LIBOPENRAVE_CORE_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}-core)
  endif()

  target_include_directories(libopenrave-core_static PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_CORE_NAME}
                                                SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                                VERSION ${OPENRAVE_VERSION}
                                                COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} "
                                                LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  target_link_libraries(libopenrave-core_static
    PRIVATE boost_assertion_failed ${IVCON_LIBRARY} ${OPENRAVE_CORE_STATIC_LIBRARIES} openrave_gpgme
    PUBLIC libopenrave_static ${OPENRAVE_CORE_LIBRARIES}
  )
  install(TARGETS libopenrave-core_static
    EXPORT openrave-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
  )
endif()
