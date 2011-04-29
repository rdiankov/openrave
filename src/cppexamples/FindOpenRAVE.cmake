# - Find Open Robotics Automation Virtual Enviornment (OpenRAVE) Installation
# http://www.openrave.org
#
# OpenRAVE provides an environment for testing, developing, and deploying motion planning algorithms
# in real-world robotics applications. The main focus is on simulation and analysis of kinematic and
# geometric information related to motion planning. OpenRAVE's stand-alone nature allows is to be easily
# integrated into existing robotics systems. An important target application is industrial robotics automation.

#==================================================================================
# Copyright (C) 2009-2011 Rosen Diankov
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distributed this file outside of CMake, substitute the full
#  License text for the above reference.)
#==================================================================================

set(_OpenRAVE_PATHS)
if(NOT OpenRAVE_DIR)
  if( WIN32 )
    # search in the registry
    set(_OpenRAVE_CONFIG_NAME "openrave-config.exe")
    get_filename_component(OpenRAVE_VERSION_STRING "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE;]" NAME)
    message(STATUS "OpenRAVE ${OpenRAVE_VERSION_STRING} found in registry")
    if( OpenRAVE_VERSION_STRING )
      get_filename_component(_OpenRAVE_PATH "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE\\${OpenRAVE_VERSION_STRING};InstallRoot]" ABSOLUTE)
      set(_OpenRAVE_PATHS ${_OpenRAVE_PATHS} ${_OpenRAVE_PATH})
    endif( OpenRAVE_VERSION_STRING )
  else(WIN32)
    set(_OpenRAVE_CONFIG_NAME "openrave-config")
  endif(WIN32)
  # search for the config path
  find_program(_OpenRAVE_CONFIG_EXECUTABLE NAMES ${_OpenRAVE_CONFIG_NAME} DOC "openrave executable")
  if( _OpenRAVE_CONFIG_EXECUTABLE )
    get_filename_component(_OpenRAVE_PATH "${_OpenRAVE_CONFIG_EXECUTABLE}" PATH) # bin
    get_filename_component(_OpenRAVE_PATH "${_OpenRAVE_PATH}" PATH)
    set(_OpenRAVE_PATHS ${_OpenRAVE_PATHS} ${_OpenRAVE_PATH})
  endif( _OpenRAVE_CONFIG_EXECUTABLE )
endif(NOT OpenRAVE_DIR)

find_package(OpenRAVE NO_MODULE PATHS ${_OpenRAVE_PATHS})
