% creates the barrett hand robot
function robot = RobotCreateHand(name,robotfilename)

robot.id = orEnvGetBody(name);
robot.filename = robotfilename;

if( robot.id <= 0 )
    robot.id = orEnvCreateRobot(name, robot.filename);
end

manips = orRobotGetManipulators(robot.id);

robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.manip = manips{1};
robot.handjoints = robot.manip.handjoints;
robot.Tgrasp = [robot.manip.Tgrasp;0 0 0 1];

robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
robot.grasp.transform = (8+robot.totaldof)+(1:12);
robot.avoidlinks = [];
