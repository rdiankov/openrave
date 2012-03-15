# Try to find SoQt
# Once done this will define
#
# SOQT_LIBRARY_FOUND - if Soqt3d is found
# SOQT_CXXFLAGS - extra flags
# SOQT_INCLUDE_DIRS - include directories
# SOQT_LINK_DIRS - link directories
# SOQT_LIBRARY_RELEASE - the relase version
# SOQT_LIBRARY_DEBUG - the debug version
# SOQT_LIBRARY - a default library, with priority debug.

# use soqt-config
find_program(SOQT_CONFIG_EXECUTABLE NAMES soqt-config DOC "soqt-config executable")
mark_as_advanced(SOQT_CONFIG_EXECUTABLE)

if(SOQT_CONFIG_EXECUTABLE)
  set(SOQT_LIBRARY_FOUND 1)

  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --cppflags
    OUTPUT_VARIABLE _soqtconfig_cppflags
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_cppflags "${_soqtconfig_cppflags}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --includedir
    OUTPUT_VARIABLE _soqtconfig_includedir
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_includedir "${_soqtconfig_includedir}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --ldflags
    OUTPUT_VARIABLE _soqtconfig_ldflags
    RESULT_VARIABLE _soqtconfig_failed)
  string(REGEX REPLACE "[\r\n]" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")
  execute_process(
    COMMAND ${SOQT_CONFIG_EXECUTABLE} --libs
    OUTPUT_VARIABLE _soqtconfig_libs
    RESULT_VARIABLE _soqtconfig_failed)

  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _soqtconfig_ldirs "${_soqtconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L" "" _soqtconfig_ldirs "${_soqtconfig_ldirs}")
  
  # on Mac OSX -L is in both ldflags and libs:
  # $ soqt-config --libs
  # -lSoQt -L/opt/local/lib -lQtOpenGL -lQtGui -lQtCore -lCoin -lpthread
  # $ soqt-config --ldflags
  # -L/usr/local/lib -Wl,-framework,OpenGL -Wl,-multiply_defined,suppress
  # $ ls /opt/local/lib/ | grep "QtO"
  # libQtOpenGL.4.7.4.dylib
  # libQtOpenGL.4.7.dylib
  # libQtOpenGL.4.dylib
  # libQtOpenGL.dylib
  # libQtOpenGL.la
  # libQtOpenGL.prl
  # $ ls /usr/local/lib/ | grep "QtO"
  # QtOpenGL.framework
  string(REGEX MATCHALL "(^| )-L([./+-_\\a-zA-Z]*)" _soqtconfig_ldirs2 "${_soqtconfig_libs}")
  string(REGEX REPLACE "(^| )-L" "" _soqtconfig_ldirs2 "${_soqtconfig_ldirs2}")

  string(REGEX MATCHALL "(^| )-l([./+-_\\a-zA-Z]*)" _soqtconfig_libs "${_soqtconfig_libs}")
  string(REGEX REPLACE "(^| )-l" "" _soqtconfig_libs "${_soqtconfig_libs}")

  string(REGEX REPLACE "(^| )-l([./+-_\\a-zA-Z]*)" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")
  string(REGEX REPLACE "(^| )-L([./+-_\\a-zA-Z]*)" " " _soqtconfig_ldflags "${_soqtconfig_ldflags}")

  separate_arguments(_soqtconfig_includedir)

  set( SOQT_CXXFLAGS "${_soqtconfig_cppflags}" )
  set( SOQT_LINK_FLAGS "${_soqtconfig_ldflags}" )
  set( SOQT_INCLUDE_DIRS ${_soqtconfig_includedir})
  set( SOQT_LINK_DIRS ${_soqtconfig_ldirs} ${_soqtconfig_ldirs2})
  set( SOQT_LIBRARY ${_soqtconfig_libs})
  set( SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY})
  set( SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY})
else(SOQT_CONFIG_EXECUTABLE)
  # soqt include files in local directory
  if( MSVC )
    set(SOQT_LIBRARY_FOUND 1)
    set( SOQT_CXXFLAGS "-DQT3_SUPPORT -DSOQT_DLL")
    set( SOQT_LINK_FLAGS "")
    set( SOQT_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/msvc_soqt/include")
    set( SOQT_LINK_DIRS "${CMAKE_SOURCE_DIR}/msvc_soqt/lib" )

    if( MSVC70 OR MSVC71 OR MSVC80 )
      set(SOQT_LIBRARY soqt1.5-vc80-mt)
	  elseif(MSVC90)
      set(SOQT_LIBRARY soqt1.5-vc90-mt)
    else() # vc100+
      set(SOQT_LIBRARY soqt1.5-vc100-mt)
    endif()
    set( SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY})
    set( SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY})
  else( MSVC )
    set(SOQT_LIBRARY_FOUND 0)
  endif( MSVC )
endif(SOQT_CONFIG_EXECUTABLE)

MARK_AS_ADVANCED(
    SOQT_LIBRARY_FOUND
    SOQT_CXXFLAGS
    SOQT_LINK_FLAGS
    SOQT_INCLUDE_DIRS
    SOQT_LINK_DIRS
    SOQT_LIBRARY
    SOQT_LIBRARY_RELEASE
    SOQT_LIBRARY_DEBUG
)
