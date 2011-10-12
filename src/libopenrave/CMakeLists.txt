cmake_policy(SET CMP0005 NEW)
set(openrave_lib_SOURCES controller.cpp iksolver.cpp interface.cpp kinbody.cpp  libopenrave.cpp libopenrave.h math.cpp planner.cpp planningutils.cpp plugindatabase.h robot.cpp sensorsystem.cpp trajectory.cpp ${rave_header_files})

check_function_exists(asinh HAS_ASINH)
check_function_exists(acosh HAS_ACOSH)
check_function_exists(atanh HAS_ATANH)
check_function_exists(exp2 HAS_EXP2)
check_function_exists(log2 HAS_LOG2)
if( HAS_LOG2 )
  add_definitions("-DHAS_LOG2")
endif()

include_directories(${FPARSER_INCLUDE_DIR})
add_library(libopenrave SHARED ${openrave_lib_SOURCES})
add_dependencies(libopenrave interfacehashes_target openrave-md5)
if( CRLIBM_FOUND )
  add_dependencies(libopenrave check_libm_accuracy) # forces check to be run before libopenrave is compile
endif()
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}
                                  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DOPENRAVE_DLL_EXPORTS -DOPENRAVE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave ${openrave_libraries} ${FPARSER_LIBRARY})
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
                                           COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS}"
                                           LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  
  target_link_libraries(libopenrave_static ${openrave_libraries})
  add_dependencies(libopenrave_static interfacehashes_target openrave-md5)
  if( CRLIBM_FOUND )
    add_dependencies(libopenrave_static check_libm_accuracy) # forces check to be run before libopenrave is compilde
  endif()
  install(TARGETS libopenrave_static EXPORT openrave-targets DESTINATION lib${LIB_SUFFIX} COMPONENT ${COMPONENT_PREFIX}dev)
endif()
