add_subdirectory(openravepy_int)

# add ikfast program
set(IKFAST_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/ikfast/ikfast.py)
if( UNIX OR NOT PYTHON_EXECUTABLE )
  execute_process(
    COMMAND ${IKFAST_EXECUTABLE} --help
    RESULT_VARIABLE _ikfast_success
    ERROR_VARIABLE _ikfast_error
    OUTPUT_VARIABLE _ikfast_output)
else()
  set(_ikfast_success 0)
endif()

if( _ikfast_success MATCHES 0 OR _ikfast_success MATCHES 1 )
  set(IKFAST_FOUND 1)
  message(STATUS "ikfast found (${IKFAST_EXECUTABLE})")
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")

    # untar the resource files
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/ikfast/sympy/__init__.py
      COMMAND ${CMAKE_COMMAND} -E chdir
      ARGS "${CMAKE_CURRENT_SOURCE_DIR}/ikfast" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/ikfast/sympy_0.6.3.tgz"
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ikfast/sympy_0.6.3.tgz)
    add_custom_target(sympy_files ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ikfast/sympy/__init__.py)
    add_dependencies(classhashes_h sympy_files)

    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ikfast/sympy" DESTINATION share/openrave PATTERN ".svn" EXCLUDE)
  else()
    message(WARNING "Could not install sympy needed by ikfast (cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} needs to be >= 2.4.6)")
  endif()
  install(PROGRAMS ${IKFAST_EXECUTABLE} DESTINATION ${OPENRAVEPY_INSTALL_DIR})
else()
  set(IKFAST_FOUND 0)
  message(STATUS "failed to execute ${IKFAST_EXECUTABLE}, return: ${_ikfast_success}")
endif()

# install examples
install(FILES __init__.py metaclass.py DESTINATION ${OPENRAVEPY_INSTALL_DIR})
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples" DESTINATION ${OPENRAVEPY_INSTALL_DIR}  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ PATTERN ".svn" EXCLUDE)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/interfaces" DESTINATION ${OPENRAVEPY_INSTALL_DIR}  FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ PATTERN ".svn" EXCLUDE)
