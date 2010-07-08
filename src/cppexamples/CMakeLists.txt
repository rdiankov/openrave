cmake_minimum_required (VERSION 2.4)
project (plugincpp)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost COMPONENTS iostreams)

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${OPENRAVE_LINK_DIRS} ${Boost_LIBRARY_DIRS})
add_library(plugincpp SHARED plugincpp.cpp)
set_target_properties(plugincpp PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(plugincpp PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(plugincpp ${OPENRAVE_LIBRARY})
install(TARGETS plugincpp DESTINATION . )

add_library(customreader SHARED customreader.cpp)
set_target_properties(customreader PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(customreader PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(customreader ${OPENRAVE_LIBRARY})
install(TARGETS customreader DESTINATION . )

add_executable(orcollision orcollision.cpp)
set_target_properties(orcollision PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orcollision PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orcollision ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orcollision DESTINATION . )

add_executable(orloadviewer orloadviewer.cpp)
set_target_properties(orloadviewer PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orloadviewer PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orloadviewer ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orloadviewer DESTINATION . )

add_executable(ikfastloader ikfastloader.cpp)
set_target_properties(ikfastloader PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(ikfastloader PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(ikfastloader ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY} ${Boost_IOSTREAMS_LIBRARY})
install(TARGETS ikfastloader DESTINATION . )
