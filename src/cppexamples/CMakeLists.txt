cmake_minimum_required (VERSION 2.4)
project (cppexamples)

set(CMAKE_BUILD_TYPE Debug)#RelWithDebInfo)
include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost COMPONENTS iostreams)

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

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
target_link_libraries(ikfastloader ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS ikfastloader DESTINATION . )

add_executable(ortrajectory ortrajectory.cpp)
set_target_properties(ortrajectory PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(ortrajectory PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(ortrajectory ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS ortrajectory DESTINATION . )

add_executable(orplanning_problem orplanning_problem.cpp)
set_target_properties(orplanning_problem PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orplanning_problem PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_problem ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orplanning_problem DESTINATION . )

add_executable(orplanning_planner orplanning_planner.cpp)
set_target_properties(orplanning_planner PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orplanning_planner PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_planner ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orplanning_planner DESTINATION . )

add_executable(orplanning_ik orplanning_ik.cpp)
set_target_properties(orplanning_ik PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orplanning_ik PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_ik ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orplanning_ik DESTINATION . )

