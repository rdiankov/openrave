% success = RobotMoveHand(robot, preshape)
%
% moves the robot arm and hand so that the hand can achieve
% a collision free preshape

% Copyright (C) 2008-2010 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
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
