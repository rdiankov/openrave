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

find_package(Boost COMPONENTS iostreams python thread)

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

add_executable(orshowsensors orshowsensors.cpp)
set_target_properties(orshowsensors PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orshowsensors PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orshowsensors ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orshowsensors DESTINATION . )

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

add_executable(ormulticontrol ormulticontrol.cpp)
set_target_properties(ormulticontrol PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(ormulticontrol PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(ormulticontrol ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS ormulticontrol DESTINATION . )

add_executable(orconveyormovement orconveyormovement.cpp)
set_target_properties(orconveyormovement PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orconveyormovement PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orconveyormovement ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orconveyormovement DESTINATION . )

# include python bindings sample
if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
  find_package(PythonLibs)
  if( PYTHONLIBS_FOUND OR PYTHON_LIBRARIES )
    if( PYTHON_EXECUTABLE )
      # get the site-packages directory
      execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print get_python_lib()"
        OUTPUT_VARIABLE _python_sitepackage
        RESULT_VARIABLE _python_failed)
      if( ${_python_failed} EQUAL 0 )
        string(REGEX REPLACE "[\r\n]" "" _python_sitepackage "${_python_sitepackage}")
        set(PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH} ${_python_sitepackage}/numpy/core/include)
      else()
        message(STATUS "failed to get python site-package directory")
      endif()
    endif()

    include_directories(${PYTHON_INCLUDE_PATH} ${OPENRAVE_INCLUDE_DIRS} ${OPENRAVE_CORE_INCLUDE_DIRS})
    add_library(orpythonbinding SHARED orpythonbinding.cpp)
    target_link_libraries(orpythonbinding ${OPENRAVE_LIBRARY} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
    set_target_properties(orpythonbinding PROPERTIES PREFIX "")
    if( WIN32 )
      set_target_properties(orpythonbinding PROPERTIES SUFFIX ".pyd")
    endif()
  endif()
endif()
