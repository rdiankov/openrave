function success = RobotMoveJointValues(robotid, joints, jointinds)
global probs
prevsession = openraveros_getglobalsession();
prevprobs = probs;
setrealsession();
orRobotSetDOFValues(robotid,joints,jointinds);
pause(0.4); % pause a little to give a chance for controller to start
success = WaitForRobot(robotid);
setclonesession(prevsession);
probs = prevprobs;
success = 1;
