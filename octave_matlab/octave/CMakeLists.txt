set(OCTAVE OCTAVE-NOTFOUND)
find_program(OCTAVE NAME "mkoctfile" PATHS)

if(OCTAVE)
  set(MEX_CXXFLAGS "--strip")
  execute_process(COMMAND ${OCTAVE} -p OCTAVE_VERSION OUTPUT_VARIABLE _octave_version OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _octave_failed)

  set(OCTAVE_ORCREATE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orcreate.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORCREATE_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORCREATE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp
    )
  add_custom_target(orcreate_octave ALL DEPENDS ${OCTAVE_ORCREATE_MEX})
  install(FILES ${OCTAVE_ORCREATE_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}octave)

  set(OCTAVE_ORREAD_MEX ${CMAKE_CURRENT_BINARY_DIR}/orread.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORREAD_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORREAD_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orread.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orread.cpp
    )
  add_custom_target(orread_octave ALL DEPENDS ${OCTAVE_ORREAD_MEX})
  add_dependencies(orread_octave orcreate_octave) # used to force mex to be called once at a time
  install(FILES ${OCTAVE_ORREAD_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}octave)

  set(OCTAVE_ORWRITE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orwrite.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORWRITE_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORWRITE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp
    )
  add_custom_target(orwrite_octave ALL DEPENDS ${OCTAVE_ORWRITE_MEX})
  add_dependencies(orwrite_octave orread_octave) # used to force mex to be called once at a time
  install(FILES ${OCTAVE_ORWRITE_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}octave)

  install(FILES ${OCTAVEMATLAB_FILES} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}octave)
  install(DIRECTORY "${OCTAVEMATLAB_FILES_DIR}/examples" DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}octave PATTERN ".svn" EXCLUDE)

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}OCTAVE_DEPENDS ${COMPONENT_PREFIX}base "octave3.2 | octave3.0" PARENT_SCOPE)
  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}OCTAVE_DISPLAY_NAME "Octave ${_octave_version} Bindings" PARENT_SCOPE)
  set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${COMPONENT_PREFIX}octave PARENT_SCOPE)
else()
  message(STATUS "Octave installation not found, is 'mkoctfile' in the system path?")
endif(OCTAVE)
