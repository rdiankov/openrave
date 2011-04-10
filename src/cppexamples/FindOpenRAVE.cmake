# Try to find OpenRAVE
# Once done this will define
#
# OPENRAVE_ROOT - The preferred installation prefix for searching for OpenRAVE. Can be set by the user.
#
# OPENRAVE_FOUND - if OpenRAVE is found
# OPENRAVE_VERSION - the version found
# OPENRAVE_CXXFLAGS - extra flags
# OPENRAVE_INCLUDE_DIRS - include directories
# OPENRAVE_LINK_DIRS - link directories
# OPENRAVE_LIBRARY_RELEASE - the relase version
# OPENRAVE_LIBRARY_DEBUG - the debug version
# OPENRAVE_LIBRARY - a default library, with priority debug.

# If OPENRAVE_ROOT was defined in the environment, use it.
if (NOT OPENRAVE_ROOT AND NOT $ENV{OPENRAVE_ROOT} STREQUAL "")
  set(OPENRAVE_ROOT $ENV{OPENRAVE_ROOT})
endif()
if( OPENRAVE_ROOT )
  file(TO_CMAKE_PATH ${OPENRAVE_ROOT} OPENRAVE_ROOT)
endif()

if( WIN32 )
  set(_OPENRAVE_CONFIG "openrave-config.exe")
  if( NOT OPENRAVE_ROOT )
    get_filename_component(OPENRAVE_VERSION "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE;]" NAME)
    message(STATUS "OpenRAVE ${OPENRAVE_VERSION} found in registry")
    set(OPENRAVE_ROOT)
    if( OPENRAVE_VERSION )
      get_filename_component(OPENRAVE_ROOT "[HKEY_LOCAL_MACHINE\\SOFTWARE\\OpenRAVE\\${OPENRAVE_VERSION};InstallRoot]" ABSOLUTE)
    else( OPENRAVE_VERSION )
      find_program(_OPENRAVE_EXECUTABLE NAMES openrave.exe DOC "openrave executable")
      if( _OPENRAVE_EXECUTABLE )
        get_filename_component(OPENRAVE_ROOT "${_OPENRAVE_EXECUTABLE}" PATH) # bin
        get_filename_component(OPENRAVE_ROOT "${OPENRAVE_ROOT}" PATH)
      endif( _OPENRAVE_EXECUTABLE )
    endif( OPENRAVE_VERSION )
  endif( NOT OPENRAVE_ROOT )
else(WIN32)
  set(_OPENRAVE_CONFIG "openrave-config")
endif( WIN32)

find_program(_OPENRAVE_CONFIG_EXECUTABLE NAMES ${_OPENRAVE_CONFIG} HINTS ${OPENRAVE_ROOT} ${OPENRAVE_ROOT}/bin DOC "openrave-config executable")

if(_OPENRAVE_CONFIG_EXECUTABLE)
  set(OPENRAVE_FOUND 1)
  if( NOT WIN32 )
    execute_process(
      COMMAND ${_OPENRAVE_CONFIG_EXECUTABLE} --version
      OUTPUT_VARIABLE OPENRAVE_VERSION
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
    set( OPENRAVE_LINK_DIRS ${_openraveconfig_ldirs})
    set( OPENRAVE_LIBRARY ${_openraveconfig_libs})
    set( OPENRAVE_CORE_LIBRARY openrave-core)
    set( OPENRAVE_LIBRARY_RELEASE ${OPENRAVE_LIBRARY})
    set( OPENRAVE_LIBRARY_DEBUG ${OPENRAVE_LIBRARY})
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
      set( OPENRAVE_INCLUDE_DIRS "${OPENRAVE_ROOT}/include" ${Boost_INCLUDE_DIRS})
      set( OPENRAVE_LINK_DIRS "${OPENRAVE_ROOT}/lib" ${Boost_LIBRARY_DIRS})
      set( OPENRAVE_LIBRARY openrave libxml2)
      set( OPENRAVE_CORE_LIBRARY ${OPENRAVE_LIBRARY} openrave-core ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
      set( OPENRAVE_LIBRARY_RELEASE ${OPENRAVE_LIBRARY})
      set( OPENRAVE_LIBRARY_DEBUG ${OPENRAVE_LIBRARY})
    else(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
      message(WARNING "Failed to find Boost ${OPENRAVE_BOOST_VERSION} necessary OpenRAVE")
    endif(Boost_VERSION AND NOT "${Boost_VERSION}" STREQUAL "0")
  endif( NOT WIN32 )
else(_OPENRAVE_CONFIG_EXECUTABLE)
  set(OPENRAVE_FOUND NOTFOUND)
endif(_OPENRAVE_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
  OPENRAVE_FOUND
  OPENRAVE_ROOT
  OPENRAVE_VERSION
  OPENRAVE_CXXFLAGS
  OPENRAVE_LINK_FLAGS
  OPENRAVE_INCLUDE_DIRS
  OPENRAVE_LINK_DIRS
  OPENRAVE_LIBRARY
  OPENRAVE_CORE_LIBRARY
  OPENRAVE_LIBRARY_RELEASE
  OPENRAVE_LIBRARY_DEBUG
  OPENRAVE_BOOST_VERSION
  _OPENRAVE_CONFIG_EXECUTABLE
)
