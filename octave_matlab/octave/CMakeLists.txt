set(OCTAVE OCTAVE-NOTFOUND)
FIND_PROGRAM(OCTAVE NAME "mkoctfile" PATHS)

if(OCTAVE)

  set(OCTAVE_ORCREATE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orcreate.mex)
  add_custom_command(
    OUTPUT ${OCTAVE_ORCREATE_MEX}
    COMMAND ${OCTAVE}
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORCREATE_MEX}\" \"${MATLAB_FILES_DIR}/orcreate.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orcreate.cpp
    )
  add_custom_target(orcreate_octave ALL DEPENDS ${OCTAVE_ORCREATE_MEX})
  install(FILES ${OCTAVE_ORCREATE_MEX} DESTINATION share/openrave/octave)

  set(OCTAVE_ORREAD_MEX ${CMAKE_CURRENT_BINARY_DIR}/orread.mex)
  add_custom_command(
    OUTPUT ${OCTAVE_ORREAD_MEX}
    COMMAND ${OCTAVE}
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORREAD_MEX}\" \"${MATLAB_FILES_DIR}/orread.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orread.cpp
    )
  add_custom_target(orread_octave ALL DEPENDS ${OCTAVE_ORREAD_MEX})
  install(FILES ${OCTAVE_ORREAD_MEX} DESTINATION share/openrave/octave)

  set(OCTAVE_ORWRITE_MEX ${CMAKE_CURRENT_BINARY_DIR}/orwrite.mex)
  add_custom_command(
    OUTPUT ${OCTAVE_ORWRITE_MEX}
    COMMAND ${OCTAVE}
    ARGS --mex -I${CMAKE_SOURCE_DIR} ${MEX_CXXFLAGS} -o \"${OCTAVE_ORWRITE_MEX}\" \"${MATLAB_FILES_DIR}/orwrite.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orwrite.cpp
    )
  add_custom_target(orwrite_octave ALL DEPENDS ${OCTAVE_ORWRITE_MEX})
  install(FILES ${OCTAVE_ORWRITE_MEX} DESTINATION share/openrave/octave)
else()
message(STATUS "Octave installation not found")
endif(OCTAVE)

install(FILES DESTINATION share/openrave/octave)
