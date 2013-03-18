cmake_policy(SET CMP0005 NEW)
set(openrave_lib_SOURCES configurationspecification.cpp controller.cpp fparsermulti.h iksolver.cpp interface.cpp kinbody.cpp kinbodygeometry.cpp kinbodyjoint.cpp kinbodylink.cpp  libopenrave.cpp libopenrave.h math.cpp planner.cpp plannerparameters.cpp planningutils.cpp plugindatabase.h robot.cpp robotmanipulator.cpp sensorsystem.cpp trajectory.cpp utils.cpp xmlreaders.cpp ${rave_header_files})

check_function_exists(asinh HAS_ASINH)
check_function_exists(acosh HAS_ACOSH)
check_function_exists(atanh HAS_ATANH)
check_function_exists(exp2 HAS_EXP2)
check_function_exists(log2 HAS_LOG2)
if( HAS_LOG2 )
  add_definitions("-DHAS_LOG2")
endif()

check_include_file(fenv.h HAVE_FENV_H)
if( HAVE_FENV_H )
  add_definitions("-DHAVE_FENV_H")
endif()

link_directories(${OPENRAVE_LINK_DIRS} ${FPARSER_LIBRARY_DIRS})

include_directories(${FPARSER_INCLUDE_DIRS})
add_library(libopenrave SHARED ${openrave_lib_SOURCES})
add_dependencies(libopenrave interfacehashes_target openrave-md5)
if( CRLIBM_FOUND )
  add_dependencies(libopenrave check_libm_accuracy) # forces check to be run before libopenrave is compile
endif()
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}
                                  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS} -DOPENRAVE_DLL_EXPORTS -DOPENRAVE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
target_link_libraries(libopenrave ${openrave_libraries} ${FPARSER_LIBRARIES})
if( MSVC )
  install(TARGETS libopenrave EXPORT openrave-targets RUNTIME DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base LIBRARY DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}base)
else()
  install(TARGETS libopenrave EXPORT openrave-targets DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}base)
endif()
#if( OPENRAVE_LIBRARY_SUFFIX AND NOT WIN32 )
#  # create a symlink to the so for easier use..
#  add_custom_command(TARGET libopenrave POST_BUILD COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE_NAME:libopenrave0.3> $<TARGET_FILE_NAME:libopenrave> )
#  install(FILES libopenrave.so DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}dev)
#endif()

if( OPT_CBINDINGS )
  add_library(libopenrave_c SHARED libopenrave_c.cpp)
  add_dependencies(libopenrave_c libopenrave)
  # remove "${LIBOPENRAVE_LINK_FLAGS} until can figure out symbol visibility issue
  set_target_properties(libopenrave_c PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}_c
    SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
    VERSION ${OPENRAVE_VERSION}
    CLEAN_DIRECT_OUTPUT 1
    COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS} -DOPENRAVE_DLL"
    LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
  target_link_libraries(libopenrave_c libopenrave ${openrave_libraries} ${FPARSER_LIBRARIES})
  if( MSVC )
    install(TARGETS libopenrave_c EXPORT openrave-targets RUNTIME DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base LIBRARY DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}cbindings-base)
  else()
    install(TARGETS libopenrave_c EXPORT openrave-targets DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}cbindings-base)
  endif()
endif()


if( OPT_STATIC )
  # visual studio needs static lib built
  if( MSVC )
    # static version needs to have different name
    set(LIBOPENRAVE_NAME libopenrave${OPENRAVE_LIBRARY_SUFFIX})
  else()
    set(LIBOPENRAVE_NAME openrave${OPENRAVE_LIBRARY_SUFFIX})
  endif()

  add_library(libopenrave_static STATIC ${openrave_lib_SOURCES})
  set_target_properties(libopenrave_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_NAME}
                                           SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                           VERSION ${OPENRAVE_VERSION}
                                           CLEAN_DIRECT_OUTPUT 1
                                           COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS}"
                                           LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
  
  target_link_libraries(libopenrave_static ${openrave_libraries} ${FPARSER_LIBRARIES})
  add_dependencies(libopenrave_static interfacehashes_target openrave-md5)
  if( CRLIBM_FOUND )
    add_dependencies(libopenrave_static check_libm_accuracy) # forces check to be run before libopenrave is compilde
  endif()
  install(TARGETS libopenrave_static EXPORT openrave-targets DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}dev)
endif()
