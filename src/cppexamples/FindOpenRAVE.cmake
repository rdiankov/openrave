# - Find Open Robotics Automation Virtual Enviornment (OpenRAVE) Installation
# http://www.openrave.org
#
# OpenRAVE provides an environment for testing, developing, and deploying motion planning algorithms
# in real-world robotics applications. The main focus is on simulation and analysis of kinematic and
# geometric information related to motion planning. OpenRAVE’s stand-alone nature allows is to be easily
# integrated into existing robotics systems. An important target application is industrial robotics automation.
#
# Users can set the following variables before calling the module:
#  OPENRAVE_ROOT_DIR - The preferred installation prefix for searching for OpenRAVE. Can be set by the user.
#
# OPENRAVE_FOUND - if OpenRAVE is found
# OPENRAVE_VERSION_STRING - the version found
# OPENRAVE_CXXFLAGS - extra flags
# OPENRAVE_INCLUDE_DIRS - include directories
# OPENRAVE_LIBRARY_DIRS - link directories
# OPENRAVE_LIBRARIES - libraries to link plugins with
# OPENRAVE_CORE_LIBRARIES - libraries to link openrave run-time with
#
# deprecated
#
# OPENRAVE_LINK_DIRS - deprecated
# OPENRAVE_LIBRARY_RELEASE - the relase version
# OPENRAVE_LIBRARY - a default library, with priority debug.
# OPENRAVE_LIBRARY - a default library, with priority debug.
# OPENRAVE_CORE_LIBRARY

#==================================================================================
# Software License Agreement (Lesser GPL)
#
# Copyright (C) 2009-2011 Rosen Diankov
#
# ikfast is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# ikfast is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#==================================================================================

if (NOT OPENRAVE_ROOT_DIR AND NOT $ENV{OPENRAVE_ROOT_DIR} STREQUAL "") 
 set(OPENRAVE_ROOT_DIR $ENV{OPENRAVE_ROOT_DIR})
endif()
if( OPENRAVE_ROOT_DIR )
  file(TO_CMAKE_PATH ${OPENRAVE_ROOT_DIR} OPENRAVE_ROOT_DIR)
endif()

if( WIN32 )
  set(_OPENRAVE_CONFIG "openrave-config.exe")
  if( NOT OPENRAVE_ROOT_DIR )
    get_filename_component(OPENRAVE_VERSION_STRING "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE;]" NAME)
    message(STATUS "OpenRAVE ${OPENRAVE_VERSION_STRING} found in registry")
    set(OPENRAVE_ROOT_DIR)
    if( OPENRAVE_VERSION_STRING )
      get_filename_component(OPENRAVE_ROOT_DIR "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE\\${OPENRAVE_VERSION_STRING};InstallRoot]" ABSOLUTE)
    else( OPENRAVE_VERSION_STRING )
      find_program(_OPENRAVE_EXECUTABLE NAMES openrave.exe DOC "openrave executable")
      if( _OPENRAVE_EXECUTABLE )
        get_filename_component(OPENRAVE_ROOT_DIR "${_OPENRAVE_EXECUTABLE}" PATH) # bin
        get_filename_component(OPENRAVE_ROOT_DIR "${OPENRAVE_ROOT_DIR}" PATH)
      endif( _OPENRAVE_EXECUTABLE )
    endif( OPENRAVE_VERSION_STRING )
  endif( NOT OPENRAVE_ROOT_DIR )
else(WIN32)
  set(_OPENRAVE_CONFIG "openrave-config")
endif( WIN32)

find_program(_OPENRAVE_CONFIG_EXECUTABLE NAMES ${_OPENRAVE_CONFIG} HINTS ${OPENRAVE_ROOT_DIR} ${OPENRAVE_ROOT_DIR}/bin DOC "openrave-config executable")

if(_OPENRAVE_CONFIG_EXECUTABLE)
  set(OPENRAVE_FOUND 1)
  if( NOT WIN32 )
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --version
      OUTPUT_VARIABLE OPENRAVE_VERSION_STRING
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --cflags
      OUTPUT_VARIABLE _openraveconfig_cflags
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --libs
      OUTPUT_VARIABLE _openraveconfig_ldflags
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --cflags-only-I
      OUTPUT_VARIABLE _openraveconfig_includedirs
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCHALL "(^| )-I([./+-_\\a-zA-Z]*)" _openraveconfig_includedirs "${_openraveconfig_includedirs}")
    string(REGEX REPLACE "(^| )-I" "" _openraveconfig_includedirs "${_openraveconfig_includedirs}")
    separate_arguments(_openraveconfig_includedirs)

    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --libs-only-L
      OUTPUT_VARIABLE _openraveconfig_ldflags
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _openraveconfig_ldirs "${_openraveconfig_ldflags}")
    string(REGEX REPLACE "(^| )-L" "" _openraveconfig_ldirs "${_openraveconfig_ldirs}")
    separate_arguments(_openraveconfig_ldirs)

    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --libs-only-l
      OUTPUT_VARIABLE _openraveconfig_libs
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _openraveconfig_libs "${_openraveconfig_libs}")
    string(REGEX REPLACE "(^| )-l" "" _openraveconfig_libs "${_openraveconfig_libs}")

    set( OPENRAVE_CXXFLAGS "${_openraveconfig_cflags}" )
    set( OPENRAVE_LINK_FLAGS "${_openraveconfig_ldflags}" )
    set( OPENRAVE_INCLUDE_DIRS ${_openraveconfig_includedirs})
    set( OPENRAVE_LIBRARY_DIRS ${_openraveconfig_ldirs})
    set( OPENRAVE_LIBRARIES ${_openraveconfig_libs})
    set( OPENRAVE_CORE_LIBRARIES openrave-core)
  else( NOT WIN32 )
    # search for the boost openrave was compiled with
    set(Boost_USE_MULTITHREAD ON)
    set(Boost_USE_STATIC_LIBS OFF)
    set(Boost_USE_STATIC_RUNTIME OFF)
    set(Boost_FIND_VERSION_EXACT 1)
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --boost-version
      OUTPUT_VARIABLE _OPENRAVE_BOOST_VERSION
      RESULT_VARIABLE _openraveconfig_failed OUTPUT_STRIP_TRAILING_WHITESPACE)
    find_package(Boost ${_OPENRAVE_BOOST_VERSION} COMPONENTS thread date_time)
    if(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
      set(OPENRAVE_FOUND 1)
      set( OPENRAVE_CXXFLAGS " -DOPENRAVE_DLL -DOPENRAVE_CORE_DLL -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB /EHc- ")
      set( OPENRAVE_LINK_FLAGS " ")
      set( OPENRAVE_INCLUDE_DIRS "${OPENRAVE_ROOT_DIR}/include" ${Boost_INCLUDE_DIRS})
      set( OPENRAVE_LIBRARY_DIRS "${OPENRAVE_ROOT_DIR}/lib" ${Boost_LIBRARY_DIRS})
      set( OPENRAVE_LIBRARIES openrave libxml2)
      set( OPENRAVE_CORE_LIBRARIES ${OPENRAVE_LIBRARIES} openrave-core ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
    else(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
      message(WARNING "Failed to find Boost ${OPENRAVE_BOOST_VERSION} necessary OpenRAVE")
    endif(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
  endif( NOT WIN32 )

  set( OPENRAVE_LIBRARY ${OPENRAVE_LIBRARIES})
  set( OPENRAVE_CORE_LIBRARY ${OPENRAVE_CORE_LIBRARIES})
  set( OPENRAVE_LINK_DIRS ${OPENRAVE_LIBRARY_DIRS})
else(_OPENRAVE_CONFIG_EXECUTABLE)
  set(OPENRAVE_FOUND NOTFOUND)
endif(_OPENRAVE_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
  OPENRAVE_FOUND
  OPENRAVE_ROOT_DIR
  OPENRAVE_VERSION_STRING
  OPENRAVE_CXXFLAGS
  OPENRAVE_LINK_FLAGS
  OPENRAVE_INCLUDE_DIRS
  OPENRAVE_LIBRARIES
  OPENRAVE_CORE_LIBRARIES
  # deprecated
  OPENRAVE_LINK_DIRS
  OPENRAVE_LIBRARY
  OPENRAVE_CORE_LIBRARY
  OPENRAVE_LIBRARY_RELEASE
  OPENRAVE_LIBRARY_DEBUG
)
