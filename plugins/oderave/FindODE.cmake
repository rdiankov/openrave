# Try to find Ode
# Once done this will define
#
# ODE_LIBRARY_FOUND - if Ode3d is found
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
  set(ODE_LIBRARY_FOUND 1)

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
else(ODE_CONFIG_EXECUTABLE)
  # ode include files in local directory
  if( MSVC )
    set(ODE_LIBRARY_FOUND 1)
    set( ODE_CXXFLAGS "")
    set( ODE_INCLUDE_DIRS "" )
    set( ODE_LINK_DIRS "" )
    set( ODE_LIBRARY optimized ode debug oded)
    set( ODE_LIBRARY_RELEASE "ode")
    set( ODE_LIBRARY_DEBUG "oded")
    set( ODE_PRECISION "SINGLE")
  else( MSVC )
    set(ODE_LIBRARY_FOUND 0)
    MESSAGE("WARNING: Could not find ODE. Please install ODE (http://www.ode.org)")
  endif( MSVC )  
endif(ODE_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    ODE_LIBRARY_FOUND
    ODE_CXXFLAGS
    ODE_INCLUDE_DIRS
    ODE_LINK_DIRS
    ODE_LIBRARY
    ODE_LIBRARY_RELEASE
    ODE_LIBRARY_DEBUG
)
