#add_subdirectory(collada)

set (openrave_core_SOURCES openrave-core.cpp ivcon.cpp)

if( MSVC )
  set (coreLIBS imm32 winmm ws2_32)
endif()
if( HAVE_ZLIB )
  set(coreLIBS ${coreLIBS} z)
endif()

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})

add_dependencies(libopenrave-core libopenrave libopenrave_static)
set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")

target_link_libraries (libopenrave-core ${Boost_THREAD_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave ${coreLIBS})

install(TARGETS libopenrave-core DESTINATION lib)
install(FILES openrave-core.h DESTINATION include)

# visual studio needs static built
add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

add_dependencies(libopenrave-core_static libopenrave libopenrave_static)
set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")
target_link_libraries (libopenrave-core_static ${Boost_THREAD_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave ${coreLIBS})
install(TARGETS libopenrave-core_static DESTINATION lib)
