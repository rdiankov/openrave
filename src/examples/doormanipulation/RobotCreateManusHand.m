% robot = RobotCreatePerMMAHand(name)
function robot = RobotCreateManusHand(name)
global robotsdir

robot.id = orEnvGetBody(name);
robot.filename = 'robots/manushand.robot.xml';

if( robot.id <= 0 )    
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);
robot.hand_type = 'barrett';
robot.preshapejoints = [];
robot.palmdir = [0 1 0];

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.handjoints = [0];
robot.open_config = -0.22;
robot.closed_config = 0.08;


robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
