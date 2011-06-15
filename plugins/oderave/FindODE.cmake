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
  if( NOT _odeconfig_cflags ) # could be cached
    execute_process(
      COMMAND ${ODE_CONFIG_EXECUTABLE} --cflags
      OUTPUT_VARIABLE _odeconfig_cflags
      RESULT_VARIABLE _odeconfig_failed)
    string(REGEX REPLACE "[\r\n]" " " _odeconfig_cflags "${_odeconfig_cflags}")
  endif()
  if(  NOT _odeconfig_lflags)
    execute_process(
      COMMAND ${ODE_CONFIG_EXECUTABLE} --libs
      OUTPUT_VARIABLE _odeconfig_lflags
      RESULT_VARIABLE _odeconfig_failed)
    string(REGEX REPLACE "[\r\n]" " " _odeconfig_lflags "${_odeconfig_lflags}")
  endif()
endif()

if(_odeconfig_cflags AND _odeconfig_lflags)
  set(ODE_LIBRARY_FOUND 1)
  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _odeconfig_ldirs "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-L" "" _odeconfig_ldirs "${_odeconfig_ldirs}")

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _odeconfig_libs "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-l" "" _odeconfig_libs "${_odeconfig_libs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _odeconfig_lflags "${_odeconfig_lflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _odeconfig_lflags "${_odeconfig_lflags}")
 
  string(REGEX MATCHALL "(^| )-I([./+-_\\a-zA-Z]*)" _odeconfig_includedirs "${_odeconfig_cflags}")
  string(REGEX REPLACE "(^| )-I" "" _odeconfig_includedirs "${_odeconfig_includedirs}")

  set( ODE_CXXFLAGS "${_odeconfig_cflags} ${_odeconfig_lflags}" )
  set( ODE_INCLUDE_DIRS ${_odeconfig_includedirs})
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
  if( MSVC )
    set(ODE_LIBRARY_FOUND 1)
    set( ODE_CXXFLAGS "-DdDOUBLE")
    set( ODE_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/msvc_ode/include" )
    set( ODE_LINK_DIRS "${CMAKE_SOURCE_DIR}/msvc_ode/lib" )
    set( ODE_PRECISION "DOUBLE")
    if( MSVC70 AND MSVC71 AND MSVC80 )
  	  set( ODE_LIBRARY ode_double-vc80-mt-r1798)
    elseif( MSVC90 )
      set( ODE_LIBRARY ode_double-vc90-mt-r1798)
    else()
      set( ODE_LIBRARY ode_double-vc100-mt-r1798)
    endif()
    set( ODE_LIBRARY_RELEASE ${ODE_LIBRARY})
    set( ODE_LIBRARY_DEBUG ${ODE_LIBRARY})
  else()
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
