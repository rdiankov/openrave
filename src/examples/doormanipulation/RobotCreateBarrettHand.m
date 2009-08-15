% creates the barrett hand robot
function robot = RobotCreateBarrettHand(name)

robot.id = orEnvGetBody(name);
robot.filename = 'robots/barretthand.robot.xml';

if( robot.id <= 0 )
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.bUseNearestIK = 0;
robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);
robot.hand_type = 'barrett';
robot.preshapejoints = 3;
robot.palmdir = [0 0 1];

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.handjoints = [0 1 2];
robot.open_config = [0.4 0.4 0.4];
robot.closed_config = [1.5 1.5 1.5];


robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
