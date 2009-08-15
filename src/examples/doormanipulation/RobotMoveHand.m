% success = RobotMoveHand(robot, preshape)
%
% moves the robot arm and hand so that the hand can achieve
% a collision free preshape
function [success, goal] = RobotMoveHand(robot, preshape)

global probs

handjoints = robot.handoffset+(0:(length(preshape)-1));
orRobotSetActiveDOFs(robot.id,robot.armjoints);
resp = orProblemSendCommand(['MoveUnsyncJoints planner basicrrt handjoints ' ...
    sprintf('%d ', length(preshape)) ...
    sprintf('%f ', preshape) sprintf('%d ', handjoints)], probs.manip);

if( isempty(resp) )
    success = 0;
    return;
end

success = 1;
resp = sscanf(resp, '%f ');
dowait = resp(1);
time = resp(2)
goal = resp(3:end);

if( dowait )
    WaitForRobot(robot.id);
end

%curvalues = orRobotGetDOFValues(robot.id,handjoints);
%orRobotSetActiveDOFs(robot.id,handjoints);
%orRobotStartActiveTrajectory(robot.id,[curvalues preshape(:)]);
orRobotSetDOFValues(robot.id,preshape(:), handjoints);
WaitForRobot(robot.id);
