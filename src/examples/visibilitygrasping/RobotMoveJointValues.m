function success = RobotMoveJointValues(robotid, joints, jointinds)
global probs
orRobotSetDOFValues(robotid,joints,jointinds);
pause(0.1); % pause a little to give a chance for controller to start
success = WaitForRobot(robotid);
success = 1;
