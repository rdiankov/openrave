# Populates ssources list
set (openrave_SOURCES openrave.cpp)

include_directories(${OPENRAVE_INCLUDE_LOCAL_DIRS})

set(openrave_libraries)
set(openrave_static_libraries)

if( TARGET "${FPARSER_LIBRARIES}" )
  get_target_property(FPARSER_TYPE "${FPARSER_LIBRARIES}" TYPE)
endif()
if( FPARSER_TYPE STREQUAL STATIC_LIBRARY)
  set(openrave_static_libraries ${openrave_static_libraries} ${FPARSER_LIBRARIES})
else()
  set(openrave_libraries ${openrave_libraries} ${FPARSER_LIBRARIES})
endif()

if( TARGET crlibm-native )
  set(openrave_static_libraries ${openrave_static_libraries} crlibm)
  if( CRLIBM_INCLUDE_DIR )
    include_directories(${CRLIBM_INCLUDE_DIR})
  endif()

  # check the accuracy of the current math library with crlibm
  add_executable(check_libm_accuracy-native check_libm_accuracy_main.cpp)
  set_target_properties(check_libm_accuracy-native PROPERTIES COMPILE_FLAGS "${NATIVE_COMPILE_FLAGS}")
  target_link_libraries(check_libm_accuracy-native crlibm-native)
  set(libm_accuracy_results_h "${CMAKE_CURRENT_BINARY_DIR}/libm_accuracy_results.h")
  add_custom_command(TARGET check_libm_accuracy-native POST_BUILD
    COMMAND check_libm_accuracy-native ARGS ${libm_accuracy_results_h}
    COMMENT "Checking accuracy between libm and crlibm")
  add_definitions(-DLIBM_ACCURACY_RESULTS_H=\"${libm_accuracy_results_h}\")
endif()
if( CLOCK_GETTIME_FOUND )
  set(openrave_libraries ${openrave_libraries} rt)
endif()
find_package(Threads)

if( NEED_TRIINDEX  )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DNEED_DTRIINDEX_TYPEDEF")
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS}")
endif()

set(LIBOPENRAVE_LINK_FLAGS "")
if( LINKER_HAS_RDYNAMIC )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -rdynamic")
  set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} -rdynamic")
endif()
if( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # apple has a different meaning on Bsymbolic
  # hidden visibility doesn't work?
else()
  if( LINKER_HAS_BSYMBOLIC )
    set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} -Wl,-Bsymbolic")
  endif()
  if( LINKER_HAS_BSYMBOLIC_FUNCTIONS )
    set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} -Wl,-Bsymbolic-functions")
  endif()
  if( LINKER_HAS_VISIBILITY )
    # not sure whether it is compiler or linkiner flag...
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -fvisibility=hidden")
    set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} -fvisibility=hidden")
  endif()
  if( LINKER_HAS_VISIBILITY_INLINES_HIDDEN )
    # not sure whether it is compiler or linkiner flag...
    set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -fvisibility-inlines-hidden")
    set(LIBOPENRAVE_LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} -fvisibility-inlines-hidden")
  endif()
endif()

if( LOG4CXX_FOUND )
  set(openrave_libraries ${openrave_libraries} ${LOG4CXX_LIBRARIES})
endif()

if(NOT OPENRAVE_DISABLE_ASSERT_HANDLER)
  add_definitions("-DBOOST_ENABLE_ASSERT_HANDLER") # turns segfault into exception
  add_definitions("-DRAPIDJSON_ASSERT=BOOST_ASSERT") # turns segfault into exception
endif()
add_library(boost_assertion_failed STATIC boost_assertion_failed.cpp)
add_dependencies(boost_assertion_failed interfacehashes_target)
add_subdirectory(libopenrave)
add_subdirectory(libopenrave-core)

# because openrave drags in dependencies from libopenrave and libopenrave-core, have to add the correct link-directories
if( COLLADA_DOM_FOUND )
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${COLLADA_DOM_LIBRARY_DIRS})
endif()
if( ASSIMP_FOUND )
  set(OPENRAVE_LINK_DIRS ${OPENRAVE_LINK_DIRS} ${ASSIMP_LIBRARY_DIRS})
endif()

link_directories(${OPENRAVE_LINK_DIRS})

add_executable(openrave ${openrave_SOURCES})
set_target_properties(openrave PROPERTIES COMPILE_FLAGS "${Boost_CFLAGS} -DOPENRAVE_CORE_DLL" OUTPUT_NAME openrave${OPENRAVE_BIN_SUFFIX})

add_dependencies(openrave libopenrave libopenrave-core)

if( MSVC )
  set(SOCKET_LIBS imm32 winmm ws2_32 )
else()
  set(SOCKET_LIBS)
endif()

target_link_libraries(openrave PRIVATE boost_assertion_failed PUBLIC ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} ${SOCKET_LIBS} ${openrave_libraries} libopenrave libopenrave-core)

install(TARGETS openrave DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
if( OPT_BUILD_PACKAGE_DEFAULT AND OPENRAVE_BIN_SUFFIX )
  InstallSymlink(openrave${OPENRAVE_BIN_SUFFIX} ${CMAKE_INSTALL_PREFIX}/bin/openrave)
endif()

# always extract the models since we don't know when models.tgz has been changed
if( EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../models.tgz" )
  message(STATUS "extracting models to ${CMAKE_CURRENT_SOURCE_DIR}")
  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/../models.tgz" WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

if( MSVC )
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cppexamples/runcmake_win.bat.in" "${CMAKE_CURRENT_BINARY_DIR}/cppexamples/runcmake_win.bat" IMMEDIATE @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppexamples/runcmake_win.bat DESTINATION ${OPENRAVE_SHARE_DIR}/cppexamples COMPONENT ${COMPONENT_PREFIX}dev)
endif()

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cppexamples/FindOpenRAVE.cmake" DESTINATION ${OPENRAVE_SHARE_DIR}/cppexamples COMPONENT ${COMPONENT_PREFIX}dev)

if( OPT_INSTALL_3DMODELDATA )
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robots DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
  if( OPT_EXTRA_ROBOTS )
    file(GLOB collada_robot_files ${CMAKE_CURRENT_SOURCE_DIR}/collada_robots/*.zae)
    install(FILES ${collada_robot_files} DESTINATION ${OPENRAVE_SHARE_DIR}/robots COMPONENT ${COMPONENT_PREFIX}data)
  endif()
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cppexamples  DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}dev FILES_MATCHING PATTERN "*.cpp" PATTERN "*.xml" PATTERN "*.h" PATTERN "*.txt" PATTERN ".svn" EXCLUDE)
