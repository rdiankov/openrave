if( ZLIB_FOUND AND Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
message(STATUS "Compiling with COLLADA support from sources!")

if( LIBXML2_DEFINITIONS )
  add_definitions(${LIBXML2_DEFINITIONS})
endif()
if( LIBXML2_INCLUDE_DIR )
  include_directories(${LIBXML2_INCLUDE_DIR})
endif()

include_directories(${MINIZIP_INCLUDE_DIR})
include_directories(BEFORE ${libpcrecpp_INCLUDE_DIRS})
link_directories(${libpcrecpp_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/1.5)
file(GLOB dom_files1_4 ${CMAKE_CURRENT_SOURCE_DIR}/src/1.4/dom/*.cpp)
file(GLOB dom_files1_5 ${CMAKE_CURRENT_SOURCE_DIR}/src/1.5/dom/*.cpp)
file(GLOB dae_files ${CMAKE_CURRENT_SOURCE_DIR}/src/dae/*.cpp)
file(GLOB libxmlplugin_files ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/LIBXMLPlugin/*.cpp)
file(GLOB stddatabase_files ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/STLDatabase/*.cpp)
file(GLOB stderrplugin_files ${CMAKE_CURRENT_SOURCE_DIR}/src/modules/stdErrPlugin/*.cpp)

set(COLLADA_LIBS minizip ${libpcrecpp_LIBRARIES} ${ZLIB_LIBRARIES})
set(COLLADA_LIBS ${COLLADA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_library(collada15reader STATIC ${dom_files1_5} ${dae_files} ${libxmlplugin_files} ${stddatabase_files} ${stderrplugin_files})

if( NOT libpcrecpp_FOUND )
  add_dependencies(collada15reader pcre_local)
endif()

add_dependencies(collada15reader libopenrave)
target_link_libraries(collada15reader ${COLLADA_LIBS})
set_target_properties(collada15reader PROPERTIES COMPILE_FLAGS "${libpcrecpp_CFLAGS_OTHERS} -DDOM_INCLUDE_LIBXML -DCOLLADA_DOM_DAEFLOAT_IS64 -DCOLLADA_DOM_SUPPORT150 ${Boost_CFLAGS}"
  LINK_FLAGS "${libpcrecpp_LDFLAGS_OTHERS}")

else()
if( NOT Boost_FILESYSTEM_FOUND )
  message(STATUS "Need to install boost filesystem")
endif()
if( NOT Boost_SYSTEM_FOUND )
  message(STATUS "Need to install boost system")
endif()
if( NOT ZLIB_FOUND )
  message(STATUS "Need to install zlib")
endif()
message(STATUS "No COLLADA support")
endif()

# set the flags
set(COLLADA_DOM_FOUND 1 PARENT_SCOPE)
set(COLLADA_DOM_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
set(COLLADA_DOM_LIBRARY_DIRS PARENT_SCOPE)
set(COLLADA_DOM_LIBRARIES collada15reader PARENT_SCOPE)
set(COLLADA_DOM_CFLAGS_OTHER "-DCOLLADA_DOM_SUPPORT150 -DCOLLADA_DOM_DAEFLOAT_IS64" PARENT_SCOPE)
set(COLLADA_DOM_LDFLAGS_OTHER PARENT_SCOPE)
