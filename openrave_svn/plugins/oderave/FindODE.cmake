# Try to find Ode
# Once done this will define
#
# ODE_LIBRARY_FOUND - if ode is found
# ODE_CXXFLAGS - extra flags
# ODE_INCLUDE_DIRS - include directories
# ODE_LINK_DIRS - link directories
# ODE_LIBRARY_RELEASE - the relase version
# ODE_LIBRARY_DEBUG - the debug version
# ODE_LIBRARY - a default library, with priority debug.

# use ode-config
find_program(ODE_CONFIG_EXECUTABLE NAMES ode-config DOC "ode-config executable")
mark_as_advanced(ODE_CONFIG_EXECUTABLE)

if(ODE_CONFIG_EXECUTABLE)
  execute_process(
    COMMAND ${ODE_CONFIG_EXECUTABLE} --cflags
    OUTPUT_VARIABLE _odeconfig_cflags
    RESULT_VARIABLE _odeconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _odeconfig_cflags "${_odeconfig_cflags}")
  execute_process(
    COMMAND ${ODE_CONFIG_EXECUTABLE} --libs
    OUTPUT_VARIABLE _odeconfig_lflags
    RESULT_VARIABLE _odeconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _odeconfig_lflags "${_odeconfig_lflags}")
endif()

if(_odeconfig_cflags AND _odeconfig_lflags)
  set(ODE_LIBRARY_FOUND 1)
  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _odeconfig_ldirs "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-L" "" _odeconfig_ldirs "${_odeconfig_ldirs}")

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _odeconfig_libs "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-l" "" _odeconfig_libs "${_odeconfig_libs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _odeconfig_lflags "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _odeconfig_lflags "${_odeconfig_lflags}")
 
  set( ODE_CXXFLAGS "${_odeconfig_cflags} ${_odeconfig_lflags}" )
  set( ODE_INCLUDE_DIRS "")
  set( ODE_LINK_DIRS ${_odeconfig_ldirs})
  set( ODE_LIBRARY ${_odeconfig_libs})
  set( ODE_LIBRARY_RELEASE ${ODE_LIBRARY})
  set( ODE_LIBRARY_DEBUG ${ODE_LIBRARY})

  string(REGEX MATCH "DOUBLE" _odeconfig_double "${_odeconfig_cflags}" )
  if( _odeconfig_double )
    set( ODE_PRECISION "DOUBLE")
  else(_odeconfig_double )
    set( ODE_PRECISION "SINGLE")
  endif(_odeconfig_double )
else()
  # ode include files in local directory
  if( MSVC AND NOT MSVC70 AND NOT MSVC71 AND NOT MSVC80 AND NOT MSVC81 )
  	# must be MSVC90 or later?
  	set(ODE_LIBRARY_FOUND 1)
    set( ODE_CXXFLAGS "-DdDOUBLE")
    set( ODE_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/inc/ode-0.11.1" )
    set( ODE_LINK_DIRS "" )
    set( ODE_LIBRARY optimized ode-vc90-double debug ode-vc90-double)
    set( ODE_LIBRARY_RELEASE "ode-vc90-double")
    set( ODE_LIBRARY_DEBUG "ode-vc90-double")
    set( ODE_PRECISION "DOUBLE")
  elseif( MSVC )
    set(ODE_LIBRARY_FOUND 1)
    set( ODE_CXXFLAGS "")
    set( ODE_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/inc/ode-0.8" )
    set( ODE_LINK_DIRS "" )
    set( ODE_LIBRARY optimized ode debug oded)
    set( ODE_LIBRARY_RELEASE "ode")
    set( ODE_LIBRARY_DEBUG "oded")
    set( ODE_PRECISION "SINGLE")
  else( )
    set(ODE_LIBRARY_FOUND 0)
    MESSAGE("WARNING: Could not find ODE. Please install ODE (http://www.ode.org)")
  endif()  
endif()

MARK_AS_ADVANCED(
    ODE_LIBRARY_FOUND
    ODE_CXXFLAGS
    ODE_INCLUDE_DIRS
    ODE_LINK_DIRS
    ODE_LIBRARY
    ODE_LIBRARY_RELEASE
    ODE_LIBRARY_DEBUG
)
