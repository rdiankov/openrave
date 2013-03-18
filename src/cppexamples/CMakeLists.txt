cmake_minimum_required (VERSION 2.6.0)
project (cppexamples)

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

macro(build_openrave_executable name)
  add_executable(${name} ${name}.cpp)
  set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
  set_target_properties(${name} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
  target_link_libraries(${name} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY})
  install(TARGETS ${name} DESTINATION . )
endmacro(build_openrave_executable)

macro(build_openrave_plugin name)
add_library(${name} SHARED ${name}.cpp)
set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}")
set_target_properties(${name} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(${name} ${OpenRAVE_LIBRARIES})
install(TARGETS ${name} DESTINATION . )
endmacro(build_openrave_plugin)

build_openrave_plugin(plugincpp)
build_openrave_plugin(customreader)

build_openrave_executable(orcollision)
build_openrave_executable(orconveyormovement)
build_openrave_executable(orloadviewer)
build_openrave_executable(ikfastloader)
build_openrave_executable(orikfilter)
build_openrave_executable(ormulticontrol)
build_openrave_executable(ormultithreadedplanning)
build_openrave_executable(orpr2turnlever)
build_openrave_executable(orplanning_module)
build_openrave_executable(orplanning_multirobot)
build_openrave_executable(orplanning_planner)
build_openrave_executable(orplanning_door)
build_openrave_executable(orplanning_ik)
build_openrave_executable(orshowsensors)
build_openrave_executable(ortrajectory)

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
  build_openrave_executable(opencvsaving)
  target_link_libraries(opencvsaving ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${OpenCV_LIBS})
endif()

# include qt4 examples
find_package(Qt4 COMPONENTS QtCore QtGui)
if (QT_FOUND)
  include_directories(${QT_INCLUDES})
  build_openrave_executable(orqtcoinviewercustom)
  target_link_libraries(orqtcoinviewercustom ${QT_QTGUI_LIBRARY})
  set_target_properties(orqtcoinviewercustom PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS} ${QT_DEFINITIONS}")
endif()
