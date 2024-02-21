cmake_policy(SET CMP0005 NEW)
set(openrave_lib_SOURCES
  configurationspecification.cpp
  collisionchecker.cpp
  controller.cpp
  environment.cpp
  fparsermulti.h
  iksolver.cpp
  interface.cpp
  kinbody.cpp
  kinbodycollision.cpp
  kinbodygeometry.cpp
  kinbodygrab.cpp
  kinbodyjoint.cpp
  kinbodylink.cpp
  kinbodystatesaver.cpp
  libopenrave.cpp
  libopenrave.h
  openraveexception.cpp
  openravemathextra.cpp
  openravemsgpack.cpp
  planner.cpp
  plannerparameters.cpp
  planningutils.cpp
  plugindatabase.h
  plugindatabase.cpp
  plugindatabase_virtual.cpp
  plugindatabase_static.cpp
  robot.cpp
  robotconnectedbody.cpp
  robotmanipulator.cpp
  sensorsystem.cpp
  trajectory.cpp
  units.cpp
  utils.cpp
  xmlreaders.cpp
  ${rave_header_files})

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

include(StaticPlugins) # Supplies the static_plugins interface library.

include_directories(${FPARSER_INCLUDE_DIRS})
add_library(libopenrave SHARED ${openrave_lib_SOURCES})
add_dependencies(libopenrave interfacehashes_target openrave-md5)
if( TARGET crlibm-native )
  add_dependencies(libopenrave check_libm_accuracy-native) # forces check to be run before libopenrave is compile
endif()
set_target_properties(libopenrave PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}
                                  SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
                                  VERSION ${OPENRAVE_VERSION}
                                  COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS} -DOPENRAVE_DLL_EXPORTS -DOPENRAVE_DLL"
                                  LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
target_compile_definitions(libopenrave PRIVATE "OPENRAVE_STATIC_PLUGINS=${OPENRAVE_STATIC_PLUGINS}")
target_link_libraries(libopenrave
  PRIVATE boost_assertion_failed static_plugins openrave-md5 ${openrave_static_libraries} openrave-msgpack
  PUBLIC LibXml2::LibXml2 Boost::filesystem Boost::thread ${CMAKE_DL_LIBS} ${openrave_libraries}
)
install(TARGETS libopenrave
  EXPORT openrave-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
)

if( OPT_CBINDINGS )
  add_library(libopenrave_c SHARED libopenrave_c.cpp)
  add_dependencies(libopenrave_c libopenrave)
  # remove "${LIBOPENRAVE_LINK_FLAGS} until can figure out symbol visibility issue
  set_target_properties(libopenrave_c PROPERTIES OUTPUT_NAME openrave${OPENRAVE_LIBRARY_SUFFIX}_c
    SOVERSION 0 # always have it 0 since we're including the soversion as part of the library name
    VERSION ${OPENRAVE_VERSION}
    COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS} -DOPENRAVE_DLL -DOPENRAVE_C_DLL -DOPENRAVE_C_DLL_EXPORTS"
    LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
  target_link_libraries(libopenrave_c PRIVATE boost_assertion_failed PUBLIC libopenrave)
  install(TARGETS libopenrave_c
    EXPORT openrave-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
  )
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
                                           COMPILE_FLAGS "${LIBOPENRAVE_COMPILE_FLAGS} ${FPARSER_CXX_FLAGS}"
                                           LINK_FLAGS "${LIBOPENRAVE_LINK_FLAGS} ${FPARSER_LINK_FLAGS}")
  target_compile_definitions(libopenrave_static PRIVATE "OPENRAVE_STATIC_PLUGINS=${OPENRAVE_STATIC_PLUGINS}")
  add_dependencies(libopenrave_static interfacehashes_target openrave-md5)
  if( TARGET crlibm-native )
    add_dependencies(libopenrave_static check_libm_accuracy-native) # forces check to be run before libopenrave is compilde
  endif()
  target_link_libraries(libopenrave_static
    PRIVATE boost_assertion_failed static_plugins openrave-md5 ${openrave_static_libraries} openrave-msgpack
    PUBLIC LibXml2::LibXml2 Boost::filesystem Boost::thread ${openrave_libraries}
  )
  install(TARGETS libopenrave_static
    EXPORT openrave-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${OPENRAVE_INCLUDE_INSTALL_DIR}
  )
endif()
