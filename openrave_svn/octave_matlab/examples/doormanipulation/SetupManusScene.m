% [robot, P, Gfilename, Gcfilename, target, preshape] = SetupManusScene(withhand)
%
% setup manus arm scene

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
