# Try to find OpenRAVE
# Once done this will define
#
# OPENRAVE_FOUND - if Coin3d is found
# OPENRAVE_CXXFLAGS - extra flags
# OPENRAVE_INCLUDE_DIRS - include directories
# OPENRAVE_LINK_DIRS - link directories
# OPENRAVE_LIBRARY_RELEASE - the relase version
# OPENRAVE_LIBRARY_DEBUG - the debug version
# OPENRAVE_LIBRARY - a default library, with priority debug.

# use openrave-config
find_program(OPENRAVE_CONFIG_EXECUTABLE NAMES openrave-config DOC "openrave-config executable")
mark_as_advanced(OPENRAVE_CONFIG_EXECUTABLE)

if(NOT MSVC AND OPENRAVE_CONFIG_EXECUTABLE)
  set(OPENRAVE_FOUND 1)

  execute_process(
    COMMAND ${OPENRAVE_CONFIG_EXECUTABLE} --cflags
    OUTPUT_VARIABLE _openraveconfig_cflags
    RESULT_VARIABLE _openraveconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _openraveconfig_cflags "${_openraveconfig_cflags}")
  execute_process(
    COMMAND ${OPENRAVE_CONFIG_EXECUTABLE} --libs
    OUTPUT_VARIABLE _openraveconfig_ldflags
    RESULT_VARIABLE _openraveconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _openraveconfig_ldflags "${_openraveconfig_ldflags}")

  execute_process(
    COMMAND ${OPENRAVE_CONFIG_EXECUTABLE} --cflags-only-I
    OUTPUT_VARIABLE _openraveconfig_includedirs
    RESULT_VARIABLE _openraveconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _openraveconfig_includedirs "${_openraveconfig_includedirs}")
  string(REGEX MATCHALL "(^| )-I([./+-_\\a-zA-Z]*)" _openraveconfig_includedirs "${_openraveconfig_includedirs}")
  string(REGEX REPLACE "(^| )-I" "" _openraveconfig_includedirs "${_openraveconfig_includedirs}")
  separate_arguments(_openraveconfig_includedirs)

  execute_process(
    COMMAND ${OPENRAVE_CONFIG_EXECUTABLE} --libs-only-L
    OUTPUT_VARIABLE _openraveconfig_ldflags
    RESULT_VARIABLE _openraveconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _openraveconfig_ldflags "${_openraveconfig_ldflags}")
  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _openraveconfig_ldirs "${_openraveconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L" "" _openraveconfig_ldirs "${_openraveconfig_ldirs}")
  separate_arguments(_openraveconfig_ldirs)

  execute_process(
    COMMAND ${OPENRAVE_CONFIG_EXECUTABLE} --libs-only-l
    OUTPUT_VARIABLE _openraveconfig_libs
    RESULT_VARIABLE _openraveconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _openraveconfig_libs "${_openraveconfig_libs}")
  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _openraveconfig_libs "${_openraveconfig_libs}")
  string(REGEX REPLACE "(^| )-l" "" _openraveconfig_libs "${_openraveconfig_libs}")

  set( OPENRAVE_CXXFLAGS "${_openraveconfig_cflags}" )
  set( OPENRAVE_LINK_FLAGS "${_openraveconfig_ldflags}" )
  set( OPENRAVE_INCLUDE_DIRS ${_openraveconfig_includedirs})
  set( OPENRAVE_LINK_DIRS ${_openraveconfig_ldirs})
  set( OPENRAVE_LIBRARY ${_openraveconfig_libs})
  set( OPENRAVE_CORE_LIBRARY openrave-core)
  set( OPENRAVE_LIBRARY_RELEASE ${OPENRAVE_LIBRARY})
  set( OPENRAVE_LIBRARY_DEBUG ${OPENRAVE_LIBRARY})
else(NOT MSVC AND OPENRAVE_CONFIG_EXECUTABLE)
  # openrave include files in local directory
  if( MSVC )
    set(OPENRAVE_FOUND 1)
    set( OPENRAVE_CXXFLAGS " -DRAVE_USEDLL -DRAVE_CORE_USEDLL -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB /EHc- ")
    set( OPENRAVE_LINK_FLAGS " ")
    set( OPENRAVE_INCLUDE_DIRS "c:/program files/openrave/include")
    set( OPENRAVE_LINK_DIRS "c:/program files/openrave/lib" )
    set( OPENRAVE_LIBRARY openrave libxml2)
    if( MSVC70 OR MSVC71 )
      set(OPENRAVE_LIBRARY ${OPENRAVE_LIBRARY} boost_thread-vc71-mt-1_39.lib boost_date_time-vc71-mt-1_39.lib)
    elseif( MSVC80 )
      set(OPENRAVE_LIBRARY ${OPENRAVE_LIBRARY} boost_thread-vc80-mt-1_39.lib boost_date_time-vc80-mt-1_39.lib)
    else()
      set(OPENRAVE_LIBRARY ${OPENRAVE_LIBRARY} boost_thread-vc90-mt-1_39.lib boost_date_time-vc90-mt-1_39.lib)
    endif()

    set( OPENRAVE_CORE_LIBRARY ${OPENRAVE_LIBRARY} openrave-core)
    set( OPENRAVE_LIBRARY_RELEASE ${OPENRAVE_LIBRARY})
    set( OPENRAVE_LIBRARY_DEBUG ${OPENRAVE_LIBRARY})
  else( MSVC )
    set(OPENRAVE_FOUND 0)
  endif( MSVC )
endif(NOT MSVC AND OPENRAVE_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    OPENRAVE_FOUND
    OPENRAVE_CXXFLAGS
    OPENRAVE_LINK_FLAGS
    OPENRAVE_INCLUDE_DIRS
    OPENRAVE_LINK_DIRS
    OPENRAVE_LIBRARY
    OPENRAVE_CORE_LIBRARY
    OPENRAVE_LIBRARY_RELEASE
    OPENRAVE_LIBRARY_DEBUG
)
