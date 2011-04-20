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
set_target_properties(openrave PROPERTIES COMPILE_FLAGS "${Boost_CFLAGS} -DOPENRAVE_CORE_DLL")

add_dependencies(openrave libopenrave libopenrave-core)

if( MSVC )
  set(SOCKET_LIBS imm32 winmm ws2_32 )
else()
  set(SOCKET_LIBS)
endif()

target_link_libraries (openrave ${Boost_DATE_TIME_LIBRARY} ${Boost_THREAD_LIBRARY} ${SOCKET_LIBS} libopenrave libopenrave-core)

# always extract the models since we don't know when models.tgz has been changed
#if( NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models/WAM" OR NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/robots/pr2-beta-static.dae")
message(STATUS "extracting models to ${CMAKE_CURRENT_SOURCE_DIR}")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/../models.tgz"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
#endif()

install(TARGETS openrave DESTINATION bin)

if( MSVC )
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cppexamples/runcmake_win.bat.in" "${CMAKE_CURRENT_BINARY_DIR}/cppexamples/runcmake_win.bat" IMMEDIATE @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppexamples/runcmake_win.bat DESTINATION ${OPENRAVE_SHARE_DIR}/cppexamples)
endif()

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cppexamples/FindOpenRAVE.cmake" DESTINATION ${OPENRAVE_SHARE_DIR}/cppexamples)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${OPENRAVE_SHARE_DIR} PATTERN ".svn" EXCLUDE)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robots DESTINATION ${OPENRAVE_SHARE_DIR} PATTERN ".svn" EXCLUDE)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION ${OPENRAVE_SHARE_DIR} PATTERN ".svn" EXCLUDE)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cppexamples  DESTINATION ${OPENRAVE_SHARE_DIR} FILES_MATCHING PATTERN "*.cpp" PATTERN "*.xml" PATTERN "*.txt" PATTERN ".svn" EXCLUDE)
else()
  # manually install
  file(GLOB robot_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.xml)
  install(FILES ${robot_files} DESTINATION ${OPENRAVE_SHARE_DIR}/robots)
  file(GLOB robot_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/schunk_fingers/*.xml)
  install(FILES ${robot_files} DESTINATION ${OPENRAVE_SHARE_DIR}/robots/schunk_fingers)
  
  file(GLOB data_files ${CMAKE_CURRENT_SOURCE_DIR}/data/*.xml)
  install(FILES ${data_files} DESTINATION ${OPENRAVE_SHARE_DIR}/data)
  
  file(GLOB models_axes_files ${CMAKE_CURRENT_SOURCE_DIR}/models/axes/*.iv)
  install(FILES ${models_axes_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/axes)
  file(GLOB models_barrett_files ${CMAKE_CURRENT_SOURCE_DIR}/models/barrett/*.iv)
  install(FILES ${models_barrett_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/barrett)
  file(GLOB models_furniture_files ${CMAKE_CURRENT_SOURCE_DIR}/models/furniture/*.iv)
  install(FILES ${models_furniture_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/furniture)
  file(GLOB models_man1_files ${CMAKE_CURRENT_SOURCE_DIR}/models/man1/*.iv)
  install(FILES ${models_man1_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/man1)
  file(GLOB models_manusarm_files ${CMAKE_CURRENT_SOURCE_DIR}/models/manusarm/*.wrl)
  install(FILES ${models_manusarm_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/manusarm)
  file(GLOB models_objects_files ${CMAKE_CURRENT_SOURCE_DIR}/models/objects/*.iv)
  install(FILES ${models_objects_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/objects)
  file(GLOB models_permma_files ${CMAKE_CURRENT_SOURCE_DIR}/models/permma/*.wrl)
  install(FILES ${models_permma_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/permma)
  file(GLOB models_puma_files ${CMAKE_CURRENT_SOURCE_DIR}/models/puma/*.iv)
  install(FILES ${models_puma_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/puma)
  file(GLOB models_segwayrmp_files ${CMAKE_CURRENT_SOURCE_DIR}/models/segwayrmp/*.iv)
  install(FILES ${models_segwayrmp_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/segwayrmp)
  file(GLOB models_shadow_files ${CMAKE_CURRENT_SOURCE_DIR}/models/shadow/*.iv)
  install(FILES ${models_shadow_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/shadow)
  file(GLOB models_WAM_files ${CMAKE_CURRENT_SOURCE_DIR}/models/WAM/*.iv)
  install(FILES ${models_WAM_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/WAM)
  file(GLOB models_pa10_files ${CMAKE_CURRENT_SOURCE_DIR}/models/pa10/*.iv)
  install(FILES ${models_pa10_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/pa10)
  file(GLOB models_katana_files ${CMAKE_CURRENT_SOURCE_DIR}/models/katana6M180/*.iv)
  install(FILES ${models_katana_files} DESTINATION ${OPENRAVE_SHARE_DIR}/models/katana6M180)
endif()
