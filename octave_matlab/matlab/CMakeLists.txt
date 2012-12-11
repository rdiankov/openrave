#
# generate mex files
#

set(MATLAB MATLAB-NOTFOUND)
if( WIN32 )
  set(MEX_EXECUTABLE "mex.bat")
else()
  set(MEX_EXECUTABLE "mex")
endif()
find_program(MATLAB NAME ${MEX_EXECUTABLE} PATHS )

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
  exec_program(${MATLAB}ext OUTPUT_VARIABLE MEXEXT)
  message(STATUS "mexext result: ${MEXEXT}")
#  if( DARWIN OR APPLE )
#    set(MEXEXT "mexmac")
#  elseif( UNIX )
#    if( IS_X86_64 )
#      set(MEXEXT "mexa64")
#    else()
#      set(MEXEXT "mexglx")
#    endif()
#  elseif( WIN64 )
#    set(MEXEXT "mexw64")
#  elseif( WIN32 OR CYGWIN OR WINDOWS )
#    set(MEXEXT "mexw32")
#  else()
#    set(MEXEXT "mex")
#  endif()

  set(MATLAB_MEX_OUT ${CMAKE_CURRENT_BINARY_DIR})
  set(MATLAB_ORCREATE_MEX "orcreate.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX}
    COMMAND "${MATLAB}"
    ARGS -I\"${OCTAVEMATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORCREATE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp\"
    DEPENDS "${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp" "${OCTAVEMATLAB_FILES_DIR}/socketconnect.h"
    )
  add_custom_target(orcreate_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX})
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORCREATE_MEX} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)

  set(MATLAB_ORREAD_MEX "orread.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX}
    COMMAND "${MATLAB}"
    ARGS -I\"${OCTAVEMATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORREAD_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orread.cpp\"
    DEPENDS "${OCTAVEMATLAB_FILES_DIR}/orread.cpp" "${OCTAVEMATLAB_FILES_DIR}/socketconnect.h"
    )
  add_custom_target(orread_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX})
  add_dependencies(orread_matlab orcreate_matlab) # used to force mex to be called once at a time
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORREAD_MEX} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)

  set(MATLAB_ORWRITE_MEX "orwrite.${MEXEXT}")
  add_custom_command(
    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX}
    COMMAND "${MATLAB}"
    ARGS -I\"${OCTAVEMATLAB_FILES_DIR}\" ${MEX_CXXFLAGS} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ORWRITE_MEX}\" \"${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp\"
    DEPENDS "${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp" "${OCTAVEMATLAB_FILES_DIR}/socketconnect.h"
    )
  add_custom_target(orwrite_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX})
  add_dependencies(orwrite_matlab orread_matlab) # used to force mex to be called once at a time
  install(FILES ${MATLAB_MEX_OUT}/${MATLAB_ORWRITE_MEX} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
else()
  message(STATUS "MATLAB installation not found, is ${MEX_EXECUTABLE} in the system path?")
endif()

# because MATLAB is non-free, have to distribute the source files
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}MATLAB_DEPENDS ${COMPONENT_PREFIX}base PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}MATLAB_DISPLAY_NAME "Matlab Bindings" PARENT_SCOPE)
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${COMPONENT_PREFIX}matlab PARENT_SCOPE)

install(FILES "${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp" "${OCTAVEMATLAB_FILES_DIR}/socketconnect.h" "${OCTAVEMATLAB_FILES_DIR}/orread.cpp" "${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp" DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
if( WIN32 OR WIN64 )
  install(FILES runmex.bat DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
endif()

install(FILES ${OCTAVEMATLAB_FILES} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
install(DIRECTORY "${OCTAVEMATLAB_FILES_DIR}/examples" DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab PATTERN ".svn" EXCLUDE)
