cmake_minimum_required (VERSION 2.6.0)
project (cppexamples)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..) # only used for cppexamples!
find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(plugincpp SHARED plugincpp.cpp)
set_target_properties(plugincpp PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(plugincpp PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(plugincpp ${OpenRAVE_LIBRARIES})
install(TARGETS plugincpp DESTINATION . )

add_library(customreader SHARED customreader.cpp)
set_target_properties(customreader PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(customreader PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(customreader ${OpenRAVE_LIBRARIES})
install(TARGETS customreader DESTINATION . )

add_executable(orcollision orcollision.cpp)
set_target_properties(orcollision PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orcollision PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orcollision ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orcollision DESTINATION . )

add_executable(orloadviewer orloadviewer.cpp)
set_target_properties(orloadviewer PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orloadviewer PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orloadviewer ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orloadviewer DESTINATION . )

add_executable(orshowsensors orshowsensors.cpp)
set_target_properties(orshowsensors PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orshowsensors PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orshowsensors ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orshowsensors DESTINATION . )

add_executable(ikfastloader ikfastloader.cpp)
set_target_properties(ikfastloader PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(ikfastloader PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(ikfastloader ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS ikfastloader DESTINATION . )

add_executable(orikfilter orikfilter.cpp)
set_target_properties(orikfilter PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orikfilter PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orikfilter ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orikfilter DESTINATION . )

add_executable(ortrajectory ortrajectory.cpp)
set_target_properties(ortrajectory PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(ortrajectory PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(ortrajectory ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS ortrajectory DESTINATION . )

add_executable(orplanning_module orplanning_module.cpp)
set_target_properties(orplanning_module PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orplanning_module PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_module ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orplanning_module DESTINATION . )

add_executable(orplanning_planner orplanning_planner.cpp)
set_target_properties(orplanning_planner PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orplanning_planner PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_planner ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orplanning_planner DESTINATION . )

add_executable(orplanning_ik orplanning_ik.cpp)
set_target_properties(orplanning_ik PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orplanning_ik PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orplanning_ik ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orplanning_ik DESTINATION . )

add_executable(ormulticontrol ormulticontrol.cpp)
set_target_properties(ormulticontrol PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(ormulticontrol PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(ormulticontrol ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS ormulticontrol DESTINATION . )

add_executable(orconveyormovement orconveyormovement.cpp)
set_target_properties(orconveyormovement PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(orconveyormovement PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orconveyormovement ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})
install(TARGETS orconveyormovement DESTINATION . )

# include python bindings sample
if( Boost_PYTHON_FOUND AND Boost_THREAD_FOUND )
  find_package(PythonLibs)
  if( PYTHONLIBS_FOUND OR PYTHON_LIBRARIES )
    if( PYTHON_EXECUTABLE )
      # get the site-packages directory
      execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_lib; print get_python_lib(1)"
        OUTPUT_VARIABLE _python_sitepackage
        RESULT_VARIABLE _python_failed)
      if( ${_python_failed} EQUAL 0 )
        string(REGEX REPLACE "[\r\n]" "" _python_sitepackage "${_python_sitepackage}")
        set(PYTHON_INCLUDE_PATH ${PYTHON_INCLUDE_PATH} ${_python_sitepackage}/numpy/core/include)
      else()
        message(STATUS "failed to get python site-package directory")
      endif()
    endif()

    include_directories(${PYTHON_INCLUDE_PATH} ${OpenRAVE_INCLUDE_DIRS})
    add_library(orpythonbinding SHARED orpythonbinding.cpp)
    target_link_libraries(orpythonbinding ${OpenRAVE_LIBRARIES} ${PYTHON_LIBRARIES} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY})
    set_target_properties(orpythonbinding PROPERTIES PREFIX "" COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
    if( WIN32 )
      set_target_properties(orpythonbinding PROPERTIES SUFFIX ".pyd")
    endif()
  endif()
endif()

# include OpenCV examples
find_package(OpenCV)
if( OpenCV_FOUND )
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenCV_LIB_DIR})
  add_executable(opencvsaving opencvsaving.cpp)
  set_target_properties(opencvsaving PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
  set_target_properties(opencvsaving PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
  target_link_libraries(opencvsaving ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${OpenCV_LIBS})
  install(TARGETS opencvsaving DESTINATION . )
endif()
