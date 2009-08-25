###########################################
# ikfastsolvers openrave plugin
###########################################
if(APPLE)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -undefined dynamic_lookup")
endif(APPLE)

set(IKTARGETS)

if( IKFAST_FOUND )
  # autogenerate ik solvers for every robot
  set(WAM_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/barrettwam.robot.xml")
  set(WAM_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_barrettwam.h")
  add_custom_command(
    OUTPUT ${WAM_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${WAM_ROBOT}" freeparam Shoulder_Roll manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${WAM_IK_OUTPUT}"
    DEPENDS ${WAM_ROBOT} ${IKFAST_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/robots/wam7.kinbody.xml"
    )
  add_custom_target(wamik ALL DEPENDS ${WAM_IK_OUTPUT})
  add_dependencies(wamik openrave)

  set(PA10_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/pa10.robot.xml")
  set(PA10_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_pa10.h")
  add_custom_command(
    OUTPUT ${PA10_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${PA10_ROBOT}" freeparam S3 manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${PA10_IK_OUTPUT}"
    DEPENDS ${PA10_ROBOT} ${IKFAST_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/robots/pa10.kinbody.xml"
    )
  add_custom_target(pa10ik ALL DEPENDS ${PA10_IK_OUTPUT})
  add_dependencies(pa10ik openrave)

  set(PUMA_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/puma.robot.xml")
  set(PUMA_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_puma.h")
  add_custom_command(
    OUTPUT ${PUMA_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${PUMA_ROBOT}" manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${PUMA_IK_OUTPUT}"
    DEPENDS ${PUMA_ROBOT} ${IKFAST_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/robots/pumaarm.kinbody.xml"
    )
  add_custom_target(pumaik ALL DEPENDS ${PUMA_IK_OUTPUT})
  add_dependencies(pumaik openrave)

  set(MANUS_LEFTARM_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/manusarm_left.robot.xml")
  set(MANUS_LEFTARM_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_manusleftarm.h")
  add_custom_command(
    OUTPUT ${MANUS_LEFTARM_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${MANUS_LEFTARM_ROBOT}" manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${MANUS_LEFTARM_IK_OUTPUT}"
    DEPENDS ${MANUS_LEFTARM_ROBOT} ${IKFAST_EXECUTABLE}
    )
  add_custom_target(manusleftik ALL DEPENDS ${MANUS_LEFTARM_IK_OUTPUT})
  add_dependencies(manusleftik openrave)

  set(MAN1_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/man1.robot.xml")
  set(MAN1_LEFT_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_man1left.h")
  add_custom_command(
    OUTPUT ${MAN1_LEFT_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${MAN1_ROBOT}" freeparam JLShoulder manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${MAN1_LEFT_IK_OUTPUT}"
    DEPENDS ${MAN1_ROBOT} ${IKFAST_EXECUTABLE}
    )
  add_custom_target(man1leftik ALL DEPENDS ${MAN1_LEFT_IK_OUTPUT})
  add_dependencies(man1leftik openrave)

  set(MAN1_RIGHT_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_man1right.h")
  add_custom_command(
    OUTPUT ${MAN1_RIGHT_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${MAN1_ROBOT}" freeparam JRShoulder manipulator 1 ikfast "${IKFAST_EXECUTABLE}" output "${MAN1_RIGHT_IK_OUTPUT}"
    DEPENDS ${MAN1_ROBOT} ${IKFAST_EXECUTABLE}
    )
  add_custom_target(man1rightik ALL DEPENDS ${MAN1_RIGHT_IK_OUTPUT})
  add_dependencies(man1rightik openrave)

  set(KATANA_ROBOT "${CMAKE_SOURCE_DIR}/src/robots/katana.robot.xml")
  set(KATANA_IK_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/ik_katana.h")
  add_custom_command(
    OUTPUT ${KATANA_IK_OUTPUT}
    # need -d 0 to suppress spurious warnings
    COMMAND "${CMAKE_BINARY_DIR}/src/openrave" -d 0 --generateik robot "${KATANA_ROBOT}" manipulator 0 ikfast "${IKFAST_EXECUTABLE}" output "${KATANA_IK_OUTPUT}"
    DEPENDS ${KATANA_ROBOT} ${IKFAST_EXECUTABLE} "${CMAKE_SOURCE_DIR}/src/robots/katana.kinbody.xml"
    )
  add_custom_target(katanaik ALL DEPENDS ${KATANA_IK_OUTPUT})
  add_dependencies(katanaik openrave)

  set(IKTARGETS wamik pa10ik pumaik manusleftik man1leftik man1rightik katanaik)
endif()

add_library(ikfastsolvers SHARED ikfastsolvers.cpp)

target_link_libraries(ikfastsolvers libopenrave)
include_directories(ikfastsolvers "${CMAKE_SOURCE_DIR}")
add_dependencies(ikfastsolvers libopenrave libopenrave_static ${IKTARGETS})
set_target_properties(ikfastsolvers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS ikfastsolvers DESTINATION share/openrave/plugins )
