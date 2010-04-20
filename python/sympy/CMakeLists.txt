if( IKFAST_FOUND )
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")

    # untar the resource files
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/sympy/__init__.py
      COMMAND ${CMAKE_COMMAND} -E chdir
      ARGS "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/sympy_0.6.3.tgz"
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sympy_0.6.3.tgz)
    add_custom_target(sympy_files ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sympy/__init__.py)
    #add_dependencies(openravepy_int sympy_files)

    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/sympy" DESTINATION ${OPENRAVE_PYTHON_INSTALL_DIR} PATTERN ".svn" EXCLUDE)
  else()
    message(WARNING "Could not install sympy needed by ikfast (cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} needs to be >= 2.4.6)")
  endif()
endif()
