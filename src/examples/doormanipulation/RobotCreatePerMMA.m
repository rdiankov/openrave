% robot = RobotCreatePerMMA(name, right)
function robot = RobotCreatePerMMA(name, right, onchair)

global robotsdir
robot.id = orEnvGetBody(name);

if( ~exist('onchair','var') )
    onchair = 0;
end

if( ~exist('right','var') )
    right = 0;
end

if( robot.id <= 0 )
    % create a new robot
    robot.filename = 'robots/permma_left.robot.xml';
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.right = right;
robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);

if( onchair )
    offset = 2;
else
    offset = 0;
end

manips = orRobotGetManipulators(robot.id);
if( length(manips) < right+1 )
    error('robot does not have correct manipulator');
end
manip = manips{right+1};

robot.armjoints = manip.armjoints;
if( length(manips{1}.joints) > 0 )
    robot.handoffset = manip.joints(1);
else
    robot.handoffset = robot.armjoints(end)+1; % offset of hand joint indices
end
robot.tgrasp = [manip.Tgrasp; 0 0 0 1];
robot.shoulderid = manip.baselink+1;
robot.wristlinkid = manip.eelink+1;
robot.handoffset = robot.armjoints(end)+1; % offset of hand joint indices

robot.CreateHandFn = @RobotCreateManusHand;
robot.testhandname = 'ManusHand';
