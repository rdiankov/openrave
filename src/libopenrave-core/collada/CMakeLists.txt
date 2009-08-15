if( HAVE_ZLIB AND NOT MSVC )
add_definitions(-DOPENRAVE_COLLADA_SUPPORT)

message(STATUS "Compiling with COLLADA support!")

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(EXTRA_COMPILE_FLAGS "-fPIC")
else()
  set(EXTRA_COMPILE_FLAGS "")
endif()

add_subdirectory(minizip)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/minizip/include)

pkg_check_modules(libpcrecpp libpcrecpp)

if( NOT libpcrecpp_FOUND )
  message(STATUS "System pcre not found, using local from sources")
  # include the local pcre
  add_subdirectory(pcre)
  add_definitions(-DHAVE_CONFIG_H)
  add_definitions(-DPCRE_STATIC)
  set(libpcrecpp_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/pcre)
  set(libpcrecpp_LIBRARY_DIRS)
  set(libpcrecpp_LIBRARIES pcre_local)
  set(libpcrecpp_CFLAGS_OTHERS)
  set(libpcrecpp_LDFLAGS_OTHERS)
  #file(GLOB pcre_files_c ${CMAKE_CURRENT_SOURCE_DIR}/pcre/*.c)
  #file(GLOB pcre_files_cc ${CMAKE_CURRENT_SOURCE_DIR}/pcre/*.cpp)
  #set(pcre_files ${pcre_files_c} ${pcre_files_cc})
endif()

include_directories(BEFORE ${libpcrecpp_INCLUDE_DIRS})
link_directories(${libpcrecpp_LIBRARY_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/1.5)
file(GLOB dom_files ${CMAKE_CURRENT_SOURCE_DIR}/1.5/dom/*.cpp)
file(GLOB dae_files ${CMAKE_CURRENT_SOURCE_DIR}/dae/*.cpp)
file(GLOB libxmlplugin_files ${CMAKE_CURRENT_SOURCE_DIR}/modules/LIBXMLPlugin/*.cpp)
file(GLOB stddatabase_files ${CMAKE_CURRENT_SOURCE_DIR}/modules/STLDatabase/*.cpp)
file(GLOB stderrplugin_files ${CMAKE_CURRENT_SOURCE_DIR}/modules/stdErrPlugin/*.cpp)

set(COLLADA_LIBS minizip ${libpcrecpp_LIBRARIES})
if( Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  set(COLLADA_LIBS ${COLLADA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
else()
  message(STATUS "Boost filesystem and system not found, using local from sources")
  add_subdirectory(boost)
  set(COLLADA_LIBS ${COLLADA_LIBS} boost_filesystem_local boost_system_local)
  include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/boost)
  add_definitions(-DBOOST_ALL_NO_LIB) # remove auto-linking
endif()

add_definitions(-DDOM_INCLUDE_LIBXML)

add_library(collada15reader STATIC colladareader.cpp colladawriter.cpp ${dom_files} ${dae_files} ${libxmlplugin_files} ${stddatabase_files} ${stderrplugin_files})

if( Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
else()
  add_dependencies(collada15reader boost_filesystem_local boost_system_local)
endif()

if( NOT libpcrecpp_FOUND )
  add_dependencies(collada15reader pcre_local)
endif()

add_dependencies(collada15reader libopenrave libopenrave_static)
target_link_libraries(collada15reader ${COLLADA_LIBS} z)
set_target_properties(collada15reader PROPERTIES COMPILE_FLAGS "${libpcrecpp_CFLAGS_OTHERS} ${EXTRA_COMPILE_FLAGS}"
                                LINK_FLAGS "${libpcrecpp_LDFLAGS_OTHERS}")

else()
message(STATUS "No COLLADA support, need zlib")
add_library(collada15reader STATIC colladareader.cpp colladawriter.cpp)
add_dependencies(collada15reader libopenrave libopenrave_static)
endif()
