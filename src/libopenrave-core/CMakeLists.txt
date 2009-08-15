add_subdirectory(collada)

set (openrave_core_SOURCES
      environment.cpp
      ivcon.cpp
      mt19937ar.cpp
      ravep.cpp
      server.cpp
      xmlreaders.cpp)

if( MSVC )
  set (coreLIBS imm32 winmm ws2_32)
endif()
if( HAVE_ZLIB )
  set(coreLIBS ${coreLIBS} z)
endif()

set(CMAKE_REQUIRED_INCLUDES ${PTHREADS_INCLUDE_DIR})
set(CMAKE_REQUIRED_DEFINITIONS "${PTHREADS_DEFINITIONS}")
set(CMAKE_REQUIRED_LIBRARIES ${PTHREADS_LIBRARY})
check_function_exists(pthread_mutex_timedlock HAVE_TIMED_LOCK)
if( HAVE_TIMED_LOCK )
  add_definitions("-DHAVE_TIMED_LOCK")
else()
  message(STATUS "failed to find pthread_mutex_timedlock")
endif()

check_function_exists(clock_gettime HAVE_CLOCK_GETTIME)

if( HAVE_CLOCK_GETTIME )
  add_definitions("-DHAVE_CLOCK_GETTIME")
else()
  check_library_exists(rt clock_gettime "/lib;/usr/lib;/usr/local/lib;/usr/pkg/lib" HAVE_RT_CLOCK_GETTIME)
  if( HAVE_RT_CLOCK_GETTIME )
    set(coreLIBS ${coreLIBS} rt)
    add_definitions("-DHAVE_CLOCK_GETTIME")
  else()
    message(STATUS "failed to find real-time clock function clock_gettime")
  endif()
endif()

add_library(libopenrave-core SHARED ${openrave_core_SOURCES})

add_dependencies(libopenrave-core libopenrave libopenrave_static collada15reader)
set_target_properties(libopenrave-core PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")

target_link_libraries (libopenrave-core ${PTHREADS_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave ${coreLIBS} collada15reader)

install(TARGETS libopenrave-core DESTINATION lib)
install(FILES openrave-core.h DESTINATION include)

# visual studio needs static built
add_library(libopenrave-core_static STATIC ${openrave_core_SOURCES})

add_dependencies(libopenrave-core_static libopenrave libopenrave_static collada15reader)
set_target_properties(libopenrave-core_static PROPERTIES OUTPUT_NAME openrave-core
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${COIN_LINK_FLAGS}")
target_link_libraries (libopenrave-core_static ${PTHREADS_LIBRARY} ${LIBXML2_LIBRARIES} ${COIN_LIBRARY} libopenrave ${coreLIBS} collada15reader)
install(TARGETS libopenrave-core_static DESTINATION lib)
