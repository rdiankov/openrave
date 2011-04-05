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
    set( _OPENRAVE_ROOT "c:/Program Files/openrave")
    set( OPENRAVE_CXXFLAGS " -DOPENRAVE_DLL -DOPENRAVE_CORE_DLL -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB /EHc- ")
    set( OPENRAVE_LINK_FLAGS " ")
    set( OPENRAVE_INCLUDE_DIRS "${_OPENRAVE_ROOT}/include")
    set( OPENRAVE_LINK_DIRS "${_OPENRAVE_ROOT}/lib" )
    set( OPENRAVE_LIBRARY openrave libxml2)
    find_package(Boost)
    if( Boost_FOUND OR (Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0"))
      set(BOOST_ROOT "${_OPENRAVE_ROOT}")
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
