set(openrave_lib_SOURCES KinBody.cpp math.cpp libopenrave.cpp Robot.cpp Trajectory.cpp mt19937ar.cpp libopenrave.h  mt19937ar.h ${rave_header_files})

add_library(libopenrave SHARED ${openrave_lib_SOURCES})
add_dependencies(libopenrave classhashes_h openrave-md5)
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DRAVE_LIBBUILD -DRAVE_USEDLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave ${LIBXML2_LIBRARIES} ${Boost_THREAD_LIBRARY} openrave-md5)
if( MSVC )
  install(TARGETS libopenrave RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
else()
  install(TARGETS libopenrave DESTINATION lib)
endif()

# visual studio needs static lib built
if( MSVC )
  # static version needs to have different name
  set(LIBOPENRAVE_NAME libopenrave-s)
else()
  set(LIBOPENRAVE_NAME openrave)
endif()

add_library(libopenrave_static STATIC ${openrave_lib_SOURCES})
set_target_properties(libopenrave_static PROPERTIES OUTPUT_NAME ${LIBOPENRAVE_NAME}
                                         SOVERSION "${RAVE_CURRENT}"
                                         VERSION "${RAVE_RELEASE}"
                                         CLEAN_DIRECT_OUTPUT 1
                                         COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                         LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
if( MSVC )
  # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
  # this is such an unbelievable hack, that it's disgusting
  set_target_properties(libopenrave_static PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true")
endif()

target_link_libraries(libopenrave_static ${LIBXML2_LIBRARIES} ${Boost_THREAD_LIBRARY} openrave-md5)
add_dependencies(libopenrave_static classhashes_h openrave-md5)
install(TARGETS libopenrave_static DESTINATION lib)
