set(openrave_lib_SOURCES KinBody.cpp math.cpp libopenrave.cpp Robot.cpp Trajectory.cpp mt19937ar.cpp libopenrave.h  mt19937ar.h plugindatabase.h ${rave_header_files})

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
add_dependencies(libopenrave classhashes_h openrave-md5)
if( OPT_ACCURATEMATH )
  add_dependencies(libopenrave check_libm_accuracy) # forces check to be run before libopenrave is compilde
endif()
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave
                                  SOVERSION ${OPENRAVE_SOVERSION}
                                  VERSION ${OPENRAVE_VERSION}
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DRAVE_LIBBUILD -DRAVE_USEDLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave ${openrave_libraries} ${FPARSER_LIBRARY})
if( MSVC )
  install(TARGETS libopenrave RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
else()
  install(TARGETS libopenrave DESTINATION lib)
endif()

if( OPT_STATIC )
  # visual studio needs static lib built
  if( MSVC )
    # static version needs to have different name
    set(LIBOPENRAVE_NAME libopenrave-s)
  else()
    set(LIBOPENRAVE_NAME openrave)
  endif()

  add_library(libopenrave_static STATIC ${openrave_lib_SOURCES})
  set_target_properties(libopenrave_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_NAME}
                                           SOVERSION ${OPENRAVE_VERSION}
                                           VERSION ${OPENRAVE_VERSION}
                                           CLEAN_DIRECT_OUTPUT 1
                                           COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                           LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
  if( MSVC )
    # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
    # this is such an unbelievable hack, that it's disgusting
    set_target_properties(libopenrave_static PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
  endif()

  target_link_libraries(libopenrave_static ${openrave_libraries})
  add_dependencies(libopenrave_static classhashes_h openrave-md5)
  if( OPT_ACCURATEMATH )
    add_dependencies(libopenrave_static check_libm_accuracy) # forces check to be run before libopenrave is compilde
  endif()
  install(TARGETS libopenrave_static DESTINATION lib)
endif()
