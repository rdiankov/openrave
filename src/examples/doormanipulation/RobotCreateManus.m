% robot = RobotCreateManus(name, right)
function robot = RobotCreateManus(name, right)

robot.id = orEnvGetBody(name);

if( ~exist('right','var') )
    right = 0;
end

if( robot.id <= 0 )
    % create a new robot
    if( right )
        robot.filename = 'robots/manusarm_right.robot.xml';
    else
        robot.filename = 'robots/manusarm_left.robot.xml';
    end
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.right = right;
robot.name = name;
robot.shoulderid = 1;
robot.totaldof = orBodyGetDOF(robot.id);

offset = 0;
robot.armjoints = [0:6]+offset;
robot.handoffset = 6+offset; % offset of hand joint indices
robot.wristlinkid = 7+offset;

robot.tgrasp = eye(4);

robot.CreateHandFn = @RobotCreateManusHand;
robot.testhandname = 'ManusHand';
robot.ikreachability = ''; % ik reachability file
