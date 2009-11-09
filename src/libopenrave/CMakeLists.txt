set(openrave_lib_SOURCES KinBody.cpp math.cpp libopenrave.cpp Robot.cpp Trajectory.cpp mt19937ar.cpp)

add_library(libopenrave SHARED ${openrave_lib_SOURCES})
add_dependencies(libopenrave classhashes_h)
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave
                                  SOVERSION "${RAVE_CURRENT}"
                                  VERSION "${RAVE_RELEASE}"
                                  CLEAN_DIRECT_OUTPUT 1
                                  COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave ${LIBXML2_LIBRARIES} ${Boost_THREAD_LIBRARY} )
install(TARGETS     libopenrave DESTINATION lib)


# visual studio needs static lib built
add_library(libopenrave_static STATIC ${openrave_lib_SOURCES})
set_target_properties(libopenrave_static PROPERTIES OUTPUT_NAME openrave
                                         SOVERSION "${RAVE_CURRENT}"
                                         VERSION "${RAVE_RELEASE}"
                                         CLEAN_DIRECT_OUTPUT 1
                                         COMPILE_FLAGS "${LIB_COMPILE_FLAGS}"
                                         LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS}")
target_link_libraries(libopenrave_static ${LIBXML2_LIBRARIES} ${Boost_THREAD_LIBRARY} )
add_dependencies(libopenrave_static classhashes_h)
install(TARGETS     libopenrave_static DESTINATION lib)
