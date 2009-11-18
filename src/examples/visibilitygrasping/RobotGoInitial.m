% success = RobotGoInitial(robot, home)

% Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function success = RobotGoInitial(robot, home)
global probs robothome

success = 0;
armjoints = robot.manips{robot.activemanip}.armjoints;
handjoints = robot.manips{robot.activemanip}.handjoints;
robotid = robot.id;

if( ~exist('home','var') )
    if( length(robothome) == robot.dof )
        home = robothome;
    else
        disp('robothome not set');
        home = zeros([robot.dof 1]);
    end
end

trajdata = orProblemSendCommand(['MoveManipulator execute 0 outputtraj armvals ' sprintf('%f ', home(armjoints+1))], probs.manip);

if( isempty(trajdata) )
    return;
end

success = StartTrajectory(robotid, trajdata)
if( ~success )
    return;
end

disp('moving hand');
success = RobotMoveJointValues(robotid, home(handjoints+1),handjoints)

%orRobotSetActiveDOFs(robotid,0:(robot.dof-1));
%curvalues = orRobotGetDOFValues(robotid);
%orRobotStartActiveTrajectory(robotid,[curvalues home(:)]);
%WaitForRobot(robotid);
