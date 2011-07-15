# Populates ssources list
set (openrave_SOURCES openrave.cpp)

include_directories(${OPENRAVE_INCLUDE_DIRS})

set(openrave_libraries ${LIBXML2_LIBRARIES} ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} openrave-md5)
if( Boost_FILESYSTEM_FOUND AND Boost_SYSTEM_FOUND )
  set(openrave_libraries ${openrave_libraries} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
  add_definitions(-DHAVE_BOOST_FILESYSTEM)
endif()
if( CRLIBM_FOUND )
  set(openrave_libraries ${openrave_libraries} ${CRLIBM_LIBRARY})
  if( CRLIBM_INCLUDE_DIR )
    include_directories(${CRLIBM_INCLUDE_DIR})
  endif()
  add_definitions("-DUSE_CRLIBM")

  # check the accuracy of the current math library with crlibm
  add_executable(check_libm_accuracy check_libm_accuracy_main.cpp)
  target_link_libraries(check_libm_accuracy ${CRLIBM_LIBRARY})
  set(libm_accuracy_results_h "${CMAKE_CURRENT_BINARY_DIR}/libm_accuracy_results.h")
  add_custom_command(TARGET check_libm_accuracy POST_BUILD
    COMMAND check_libm_accuracy ARGS ${libm_accuracy_results_h}
    COMMENT "Checking accuracy between libm and crlibm")
  add_definitions(-DLIBM_ACCURACY_RESULTS_H=\"${libm_accuracy_results_h}\")
endif()
if( CLOCK_GETTIME_FOUND )
  set(openrave_libraries ${openrave_libraries} rt)
endif()

set(LIBOPENRAVE_COMPILE_FLAGS "${Boost_CFLAGS}")

if( NEED_TRIINDEX  )
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -DNEED_DTRIINDEX_TYPEDEF")
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(LIBOPENRAVE_COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} -fPIC")
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

add_subdirectory(libopenrave)
add_subdirectory(libopenrave-core)

add_executable(openrave ${openrave_SOURCES})
set_target_properties(openrave PROPERTIES COMPILE_FLAGS "${Boost_CFLAGS} -DOPENRAVE_CORE_DLL" OUTPUT_NAME openrave${OPENRAVE_BIN_SUFFIX})

add_dependencies(openrave libopenrave libopenrave-core)

if( MSVC )
  set(SOCKET_LIBS imm32 winmm ws2_32 )
else()
  set(SOCKET_LIBS)
endif()

target_link_libraries (openrave ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} ${SOCKET_LIBS} libopenrave libopenrave-core)

install(TARGETS openrave DESTINATION bin COMPONENT ${COMPONENT_PREFIX}base)
if( OPT_BUILD_PACKAGE_DEFAULT )
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink openrave${OPENRAVE_BIN_SUFFIX} openrave WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX}/bin)" COMPONENT openrave)
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
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robots DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}data PATTERN ".svn" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cppexamples  DESTINATION ${OPENRAVE_SHARE_DIR} COMPONENT ${COMPONENT_PREFIX}dev FILES_MATCHING PATTERN "*.cpp" PATTERN "*.xml" PATTERN "*.txt" PATTERN ".svn" EXCLUDE)
