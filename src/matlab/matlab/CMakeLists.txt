#
# generate mex files
#

set(MATLAB MATLAB-NOTFOUND)
if( WIN32 )
  FIND_PROGRAM(MATLAB NAME "mex.bat" PATHS )
else()
  FIND_PROGRAM(MATLAB NAME "mex" PATHS )
endif()

set(MEX_CXXFLAGS)
if( CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX )
  if( NOT WIN32 )
    # check for lapack
    check_library_exists(lapack _init "" HAS_LAPACK_LIB)
    if( HAS_LAPACK_LIB )
      set(MEX_CXXFLAGS "${MEX_LIBS}")
    endif()
  endif()
else()
  if( MSVC )
    set(MEX_CXXFLAGS "ws2_32.lib")
  endif()
endif()

if(MATLAB)

  # check if the mex file is actually matlab (can be confused with latex)
  EXEC_PROGRAM(${MATLAB} ARGS "-version" OUTPUT_VARIABLE MEX_TEST_OUT RETURN_VALUE MEX_TEST_RETURN)
  #message(STATUS "mex result: ${MEX_TEST_OUT}")
  string(REGEX MATCH "MATLAB" IS_MATLAB "${MEX_TEST_OUT}")

  if( IS_MATLAB )
    set(USE_MATLAB 1)
  else()
    set(USE_MATLAB)
  endif()
endif()

if( USE_MATLAB )
  message(STATUS "Matlab detected, compiling matlab mex files")

  if( DARWIN OR APPLE )
    set(MEXEXT "mexmac")
  elseif( UNIX )
    if( IS_X86_64 )
      set(MEXEXT "mexa64")
    else()
      set(MEXEXT "mexglx")
    endif()
  elseif( WIN64 )
    set(MEXEXT "mexw64")
  elseif( WIN32 OR CYGWIN OR WINDOWS )
    set(MEXEXT "mexw32")
  else()
    set(MEXEXT "mex")
  endif()

  set(MATLAB_MEX_OUT ${CMAKE_CURRENT_BINARY_DIR})
  set(MATLAB_ORCREATE_MEX "orcreate.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX}
    COMMAND ${MATLAB}
    ARGS -I\"${MATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORCREATE_MEX}\" \"${MATLAB_FILES_DIR}/orcreate.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orcreate.cpp ${MATLAB_FILES_DIR}/socketconnect.h
    )
  add_custom_target(orcreate_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX})
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX} DESTINATION share/openrave/matlab)

  set(MATLAB_ORREAD_MEX "orread.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX}
    COMMAND ${MATLAB}
    ARGS -I\"${MATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORREAD_MEX}\" \"${MATLAB_FILES_DIR}/orread.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orread.cpp ${MATLAB_FILES_DIR}/socketconnect.h
    )
  add_custom_target(orread_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX})
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX} DESTINATION share/openrave/matlab)

  set(MATLAB_ORWRITE_MEX "orwrite.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX}
    COMMAND ${MATLAB}
    ARGS -I\"${MATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORWRITE_MEX}\" \"${MATLAB_FILES_DIR}/orwrite.cpp\"
    DEPENDS ${MATLAB_FILES_DIR}/orwrite.cpp ${MATLAB_FILES_DIR}/socketconnect.h
    )
  add_custom_target(orwrite_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX})
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX} DESTINATION share/openrave/matlab)
else()
  message(STATUS "MATLAB installation not found")
endif()
