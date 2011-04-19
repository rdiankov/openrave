set(OCTAVE OCTAVE-NOTFOUND)
find_program(OCTAVE NAME "mkoctfile" PATHS)

if(OCTAVE)
  execute_process(COMMAND ${OCTAVE} "-p OCTAVE_VERSION" OUTPUT_VARIABLE _octave_version OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _octave_failed)
  set(CPACK_COMPONENT_OPENRAVE-OCTAVE_DEPENDS "octave${_octave_version}" PARENT_SCOPE)
  set(CPACK_COMPONENT_OPENRAVE-OCTAVE_DESCRIPTION "OpenRAVE Octave Bindings" PARENT_SCOPE)

  set(OCTAVE_ORCREATE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orcreate.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORCREATE_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORCREATE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp
    )
  add_custom_target(orcreate_octave ALL DEPENDS ${OCTAVE_ORCREATE_MEX})
  install(FILES ${OCTAVE_ORCREATE_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT openrave-octave)

  set(OCTAVE_ORREAD_MEX ${CMAKE_CURRENT_BINARY_DIR}/orread.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORREAD_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORREAD_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orread.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orread.cpp
    )
  add_custom_target(orread_octave ALL DEPENDS ${OCTAVE_ORREAD_MEX})
  add_dependencies(orread_octave orcreate_octave) # used to force mex to be called once at a time
  install(FILES ${OCTAVE_ORREAD_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT openrave-octave)

  set(OCTAVE_ORWRITE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orwrite.mex)
  add_custom_command(
    OUTPUT "${OCTAVE_ORWRITE_MEX}"
    COMMAND "${OCTAVE}"
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORWRITE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp\"
    DEPENDS ${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp
    )
  add_custom_target(orwrite_octave ALL DEPENDS ${OCTAVE_ORWRITE_MEX})
  add_dependencies(orwrite_octave orread_octave) # used to force mex to be called once at a time
  install(FILES ${OCTAVE_ORWRITE_MEX} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT openrave-octave)

  install(DIRECTORY "${OCTAVEMATLAB_FILES_DIR}" DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT openrave-octave FILES_MATCHING PATTERN "*.m")
  install(DIRECTORY "${OCTAVEMATLAB_FILES_DIR}/examples" DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} COMPONENT openrave-octave PATTERN ".svn" EXCLUDE)
  set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} openrave-octave)
else()
  message(STATUS "Octave installation not found, is 'mkoctfile' in the system path?")
endif(OCTAVE)
