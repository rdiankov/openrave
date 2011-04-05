set(MATLAB_FILES_DIR ${CMAKE_CURRENT_SOURCE_DIR})
# install all the script files
set(MATLAB_FILES orBodyDestroy.m orEnvCheckCollision.m  orEnvLoadScene.m         orEnvStepSimulation.m orRobotControllerSet.m
orBodyEnable.m          orEnvClose.m           orEnvPlot.m              orRobotGetActiveDOF.m
orBodyGetAABB.m         orEnvCreateKinBody.m   orEnvRayCollision.m      orRobotGetAttachedSensors.m
orBodyGetAABBs.m        orEnvSetOptions.m        orRobotGetDOFLimits.m
orBodyGetDOF.m          orEnvCreateProblem.m   orEnvTriangulate.m       orRobotGetDOFValues.m
orBodyGetJointValues.m  orEnvCreateRobot.m     orEnvWait.m              orRobotGetManipulators.m
orBodyGetLinks.m        orEnvDestroyProblem.m  orRobotSensorGetData.m
orBodyGetTransform.m    orEnvGetBodies.m       orRobotSensorSend.m orBodySetJointTorques.m
orBodySetJointValues.m  orEnvGetBody.m         orProblemSendCommand.m   orRobotSetActiveDOFs.m
orBodySetTransform.m    orEnvGetRobots.m       orRobotSetDOFValues.m
orCommunicator.m        orEnvLoadPlugin.m      orRobotCheckSelfCollision.m 
orRobotControllerSend.m  orRobotStartActiveTrajectory.m orSetCommunication.m )

if(OPT_MATLAB)
  add_subdirectory(matlab)
  install(FILES ${MATLAB_FILES} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR})
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples" DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} PATTERN ".svn" EXCLUDE)
endif()

if(OPT_OCTAVE)
  add_subdirectory(octave)
  install(FILES ${MATLAB_FILES} DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR})
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples" DESTINATION ${OPENRAVE_OCTAVE_INSTALL_DIR} PATTERN ".svn" EXCLUDE)
endif()
