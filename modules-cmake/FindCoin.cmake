# Try to find Coin3D
# Once done this will define
#
# COIN_LIBRARY_FOUND - if Coin3d is found
# COIN_CXXFLAGS - extra flags
# COIN_LINK_FLAGS - extra flags
# COIN_INCLUDE_DIRS - include directories
# COIN_LINK_DIRS - link directories
# COIN_LIBRARY_RELEASE - the relase version
# COIN_LIBRARY_DEBUG - the debug version
# COIN_LIBRARY - a default library, with priority debug.

# use coin-config
find_program(COIN_CONFIG_EXECUTABLE NAMES coin-config DOC "coin-config executable")
mark_as_advanced(COIN_CONFIG_EXECUTABLE)

if(COIN_CONFIG_EXECUTABLE)
  set(COIN_LIBRARY_FOUND 1)

  execute_process(
    COMMAND ${COIN_CONFIG_EXECUTABLE} --cppflags
    OUTPUT_VARIABLE _coinconfig_cppflags
    RESULT_VARIABLE _coinconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _coinconfig_cppflags "${_coinconfig_cppflags}")
  execute_process(
    COMMAND ${COIN_CONFIG_EXECUTABLE} --includedir
    OUTPUT_VARIABLE _coinconfig_includedir
    RESULT_VARIABLE _coinconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _coinconfig_includedir "${_coinconfig_includedir}")
  execute_process(
    COMMAND ${COIN_CONFIG_EXECUTABLE} --ldflags
    OUTPUT_VARIABLE _coinconfig_ldflags
    RESULT_VARIABLE _coinconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _coinconfig_ldflags "${_coinconfig_ldflags}")
  execute_process(
    COMMAND ${COIN_CONFIG_EXECUTABLE} --libs
    OUTPUT_VARIABLE _coinconfig_libs
    RESULT_VARIABLE _coinconfig_failed)

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _coinconfig_libs "${_coinconfig_libs}")
  string(REGEX REPLACE "(^| )-l" "" _coinconfig_libs "${_coinconfig_libs}")

  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _coinconfig_ldirs "${_coinconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L" "" _coinconfig_ldirs "${_coinconfig_ldirs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _coinconfig_ldflags "${_coinconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _coinconfig_ldflags "${_coinconfig_ldflags}")

  separate_arguments(_coinconfig_includedir)

  set( COIN_CXXFLAGS "${_coinconfig_cppflags}" )
  set( COIN_LINK_FLAGS "${_coinconfig_ldflags}" )
  set( COIN_INCLUDE_DIRS ${_coinconfig_includedir})
  set( COIN_LINK_DIRS ${_coinconfig_ldirs})
  set( COIN_LIBRARY ${_coinconfig_libs})
  set( COIN_LIBRARY_RELEASE ${COIN_LIBRARY})
  set( COIN_LIBRARY_DEBUG ${COIN_LIBRARY})
else(COIN_CONFIG_EXECUTABLE)
  # coin include files in local directory
  if( MSVC )
    set(COIN_LIBRARY_FOUND 1)
    set( COIN_CXXFLAGS "-DCOIN_DLL -DSIMAGE_DLL")
    set( COIN_LINK_FLAGS "")
    set( COIN_INCLUDE_DIRS "")
    set( COIN_LINK_DIRS "" )
    if( MSVC70 OR MSVC71 OR MSVC80 )
      set( COIN_LIBRARY coin3.1-vc80-mt)
    elseif( MSVC90 )
      set( COIN_LIBRARY coin3.1-vc90-mt)
    else() # vc100+
      set( COIN_LIBRARY coin3.1-vc100-mt)
    endif()
    set( COIN_LIBRARY_RELEASE ${COIN_LIBRARY})
    set( COIN_LIBRARY_DEBUG ${COIN_LIBRARY})
  else( MSVC )
    set(COIN_LIBRARY_FOUND 0)
  endif( MSVC )  
endif(COIN_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    COIN_LIBRARY_FOUND
    COIN_CXXFLAGS
    COIN_LINK_FLAGS
    COIN_INCLUDE_DIRS
    COIN_LINK_DIRS
    COIN_LIBRARY
    COIN_LIBRARY_RELEASE
    COIN_LIBRARY_DEBUG
)
