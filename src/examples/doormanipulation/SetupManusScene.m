% [robot, P, Gfilename, Gcfilename, target, preshape] = SetupManusScene(withhand)
%
% setup manus arm scene

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
%
% Functions to plan with caging grasps. See
% Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
% Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
function [robot, P, Gfilename, Gcfilename, target, preshape] = SetupManusScene(withhand)
    
global probs

if( ~exist('withhand','var') )
    withhand = 0;
end

orEnvLoadScene('data/door.env.xml',1);

preshape = -0.35; 

if(withhand)
    robot = RobotCreateManusHand('ManusHand');
    orBodySetTransform(robot.id,[0.1214   -0.1770    0.9767   -0.3456
    0.5526   -0.8053   -0.2146    1.0439
    0.8245    0.5658         0   -0.1552]);
    orRobotSetDOFValues(robot.id, preshape);
else
    robot = RobotCreatePerMMA('PerMMA',0,1);
    Trobot = [1.0000         0         0   -0.3413;
         0    1.0000         0    0.1700;
         0         0    1.0000   -1.1156];
    orBodySetTransform(robot.id, Trobot(:));
    orRobotSetDOFValues(robot.id, preshape, robot.handoffset+(0:(length(preshape)-1)));
    orRobotSetDOFValues(robot.id, [0   -1.4042    3.0295    1.6864    2.0944 0],robot.armjoints);
end
probs.manip = orEnvCreateProblem('BaseManipulation', robot.name);
probs.cage = orEnvCreateProblem('TaskCaging', robot.name);
Gfilename = fullfile(pwd, '/manushand_graspset.txt');
Gcfilename = fullfile(pwd, '/manushand_graspcontactset.txt');

P = [0:0.04:pi/4];
P = [P; zeros(size(P))];

target.name = 'door';
target.id = orEnvGetBody(target.name);
target.joints = 0;
target.link = 3;
