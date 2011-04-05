% RobotGoInitial(robot, home,probid)
%
% plan to move a robot to its home position (default is all 0s)

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
function success = RobotGoInitial(robot, home,probid)

success = 0;

orRobotControllerSend(1,'pause 0');

if( ~exist('home','var') )
    home = zeros([robot.totaldof 1]);
end

s = orProblemSendCommand(['MoveManipulator armvals ' sprintf('%f ', home(robot.armjoints+1))], probid);

if( isempty(s) )
    return;
end

pause(0.5);
WaitForRobot(robot.id);

disp('moving hand');

orRobotSetActiveDOFs(robot.id,0:(robot.totaldof-1));
curvalues = orRobotGetDOFValues(robot.id);
orRobotStartActiveTrajectory(robot.id,[curvalues home(:)]);
WaitForRobot(robot.id);
success = 1;
