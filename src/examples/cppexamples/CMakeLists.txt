cmake_minimum_required (VERSION 2.4)
project (plugincpp)

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

if( MSVC )
  # have to include the module definition file
  set(PLUGIN_LINK_FLAGS "/DEF:\"${CMAKE_CURRENT_SOURCE_DIR}/openraveplugin.def\"")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(PLUGIN_LINK_FLAGS "")
endif()

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${OPENRAVE_LINK_DIRS})
add_library(plugincpp SHARED plugincpp.cpp)
set_target_properties(plugincpp PROPERTIES COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
set_target_properties(plugincpp PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS} ${PLUGIN_LINK_FLAGS}")
target_link_libraries(plugincpp ${OPENRAVE_LIBRARY})
install(TARGETS plugincpp DESTINATION . )

add_library(customreader SHARED customreader.cpp)
set_target_properties(customreader PROPERTIES COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
set_target_properties(customreader PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS} ${PLUGIN_LINK_FLAGS}")
target_link_libraries(customreader ${OPENRAVE_LIBRARY})
install(TARGETS customreader DESTINATION . )

add_executable(orcollision orcollision.cpp)
set_target_properties(orcollision PROPERTIES COMPILE_FLAGS ${OPENRAVE_CXXFLAGS})
set_target_properties(orcollision PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS} ${PLUGIN_LINK_FLAGS}")
target_link_libraries(orcollision ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orcollision DESTINATION . )
