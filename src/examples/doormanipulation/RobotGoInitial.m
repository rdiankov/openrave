% RobotGoInitial(robot, home,probid)
%
% plan to move a robot to its home position (default is all 0s)

% Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http:%www.gnu.org/licenses/>.
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
