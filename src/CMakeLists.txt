# Populates ssources list
set (openrave_SOURCES openrave.cpp)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LIB_COMPILE_FLAGS "-DRAVE_LIBBUILD")

if( NEED_TRIINDEX  )
  set(LIB_COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -DNEED_DTRIINDEX_TYPEDEF")
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(LIB_COMPILE_FLAGS "${LIB_COMPILE_FLAGS} -fPIC")
endif()

add_subdirectory(libopenrave)
add_subdirectory(libopenrave-core)
add_subdirectory(openravepy)

add_executable(openrave ${openrave_SOURCES})

# if( MSVC )
#   # soqt libs not compiled correct so multiply defined symbols
#   set_target_properties(openrave PROPERTIES LINK_FLAGS "/FORCE:MULTIPLE")
# endif()

add_subdirectory(matlab)
add_dependencies(openrave libopenrave libopenrave_static libopenrave-core libopenrave-core_static)


if( MSVC )
  set(SOCKET_LIBS imm32 winmm ws2_32)
else()
  set(SOCKET_LIBS)
endif()

target_link_libraries (openrave ${Boost_THREAD_LIBRARY} libopenrave-core ${SOCKET_LIBS})

# untar the resource files
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/WAM/wam0.iv
  COMMAND ${CMAKE_COMMAND} -E chdir
  ARGS "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_COMMAND} -E tar xzf "${CMAKE_SOURCE_DIR}/models.tgz"
  DEPENDS ${CMAKE_SOURCE_DIR}/models.tgz)
add_custom_target(resource_files ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/WAM/wam0.iv)
add_dependencies(openrave resource_files)

install(TARGETS openrave DESTINATION bin)

# install the resource files
file(GLOB robot_files ${CMAKE_CURRENT_SOURCE_DIR}/robots/*.xml)
install(FILES ${robot_files} DESTINATION share/openrave/robots)

file(GLOB data_files ${CMAKE_CURRENT_SOURCE_DIR}/data/*.xml)
install(FILES ${data_files} DESTINATION share/openrave/data)

# install resources
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" STRGREATER "2.4.3")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION share/openrave)
else()
  # manually install
  file(GLOB models_axes_files ${CMAKE_CURRENT_SOURCE_DIR}/models/axes/*.iv)
  install(FILES ${models_axes_files} DESTINATION share/openrave/models/axes)
  file(GLOB models_barrett_files ${CMAKE_CURRENT_SOURCE_DIR}/models/barrett/*.iv)
  install(FILES ${models_barrett_files} DESTINATION share/openrave/models/barrett)
  file(GLOB models_furniture_files ${CMAKE_CURRENT_SOURCE_DIR}/models/furniture/*.iv)
  install(FILES ${models_furniture_files} DESTINATION share/openrave/models/furniture)
  file(GLOB models_man1_files ${CMAKE_CURRENT_SOURCE_DIR}/models/man1/*.iv)
  install(FILES ${models_man1_files} DESTINATION share/openrave/models/man1)
  file(GLOB models_manusarm_files ${CMAKE_CURRENT_SOURCE_DIR}/models/manusarm/*.wrl)
  install(FILES ${models_manusarm_files} DESTINATION share/openrave/models/manusarm)
  file(GLOB models_objects_files ${CMAKE_CURRENT_SOURCE_DIR}/models/objects/*.iv)
  install(FILES ${models_objects_files} DESTINATION share/openrave/models/objects)
  file(GLOB models_permma_files ${CMAKE_CURRENT_SOURCE_DIR}/models/permma/*.wrl)
  install(FILES ${models_permma_files} DESTINATION share/openrave/models/permma)
  file(GLOB models_puma_files ${CMAKE_CURRENT_SOURCE_DIR}/models/puma/*.iv)
  install(FILES ${models_puma_files} DESTINATION share/openrave/models/puma)
  file(GLOB models_segwayrmp_files ${CMAKE_CURRENT_SOURCE_DIR}/models/segwayrmp/*.iv)
  install(FILES ${models_segwayrmp_files} DESTINATION share/openrave/models/segwayrmp)
  file(GLOB models_shadow_files ${CMAKE_CURRENT_SOURCE_DIR}/models/shadow/*.iv)
  install(FILES ${models_shadow_files} DESTINATION share/openrave/models/shadow)
  file(GLOB models_WAM_files ${CMAKE_CURRENT_SOURCE_DIR}/models/WAM/*.iv)
  install(FILES ${models_WAM_files} DESTINATION share/openrave/models/WAM)
  file(GLOB models_pa10_files ${CMAKE_CURRENT_SOURCE_DIR}/models/pa10/*.iv)
  install(FILES ${models_pa10_files} DESTINATION share/openrave/models/pa10)
  file(GLOB models_katana_files ${CMAKE_CURRENT_SOURCE_DIR}/models/katana6M180/*.iv)
  install(FILES ${models_katana_files} DESTINATION share/openrave/models/katana6M180)
endif()

# install example scripts
file(GLOB example_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.m)
install(FILES ${example_files} DESTINATION share/openrave/examples)
file(GLOB example_files_py ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.py)
install(PROGRAMS ${example_files_py} DESTINATION share/openrave/examples)
file(GLOB example_door_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/doormanipulation/*.m)
install(FILES ${example_door_files} DESTINATION share/openrave/examples/doormanipulation)
file(GLOB example_door2_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/doormanipulation/*.txt)
install(FILES ${example_door2_files} DESTINATION share/openrave/examples/doormanipulation)
file(GLOB example_hanoi_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/hanoi_puzzle/*.m)
install(FILES ${example_hanoi_files} DESTINATION share/openrave/examples/hanoi_puzzle)
file(GLOB example_grasping_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/grasping/*.m)
install(FILES ${example_grasping_files} DESTINATION share/openrave/examples/grasping)
file(GLOB example_grasping2_files ${CMAKE_CURRENT_SOURCE_DIR}/examples/grasping/grasptables/*.mat)
install(FILES ${example_grasping2_files} DESTINATION share/openrave/examples/grasping/grasptables)
set(PLUGINCPP ${CMAKE_CURRENT_SOURCE_DIR}/examples/plugincpp)
install(FILES ${PLUGINCPP}/CMakeLists.txt ${PLUGINCPP}/README ${PLUGINCPP}/plugincpp.cpp ${PLUGINCPP}/customreader.cpp ${PLUGINCPP}/customreader.env.xml ${PLUGINCPP}/FindOpenRAVE.cmake DESTINATION share/openrave/examples/plugincpp)
if( MSVC )
  install(FILES ${CMAKE_SOURCE_DIR}/runcmake_win.bat DESTINATION share/openrave/examples/plugincpp)
  install(FILES ${CMAKE_SOURCE_DIR}/plugins/openraveplugin.def DESTINATION share/openrave/examples/plugincpp)
endif()
