% [robot, P, Gfilename, Gcfilename, target] = SetupWAMScene(withhand, index)

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
function [robot, P, Gfilename, Gcfilename, target, preshape] = SetupWAMScene(withhand, index)
    
global probs

if( ~exist('withhand','var') )
    withhand = 0;
end
if( ~exist('index','var') )
    index = 2;
end

if( withhand )
    orEnvLoadScene('',1);
    orEnvCreateKinBody('Cabinet','data/cabinet1.kinbody.xml');
else
    orEnvLoadScene('data/wam_cabinet.env.xml',1);
end

preshapes = [pi/2 pi/2  0.3294 0;
             pi/2 pi/2  0.3294 0;
             0.5 0.5 0.5 pi;
             0 0 pi/2 0];
preshape = preshapes(index,:);

if( withhand )
    robot = RobotCreateBarrettHand('hand');

    Trobot{1} = [0.8396   -0.0408   -0.5416    0.3785
                0.0331   -0.9915    0.1261    0.0910
               -0.5421   -0.1238   -0.8311    0.4442];
    Trobot{2} =  [0.9985    0.0509    0.0198    0.2724
                0.0271   -0.1468   -0.9888    0.2426
               -0.0474    0.9879   -0.1480    0.1587];
    Trobot{3} = [0.9220   -0.0137   -0.3870    0.3516
               -0.3844   -0.1548   -0.9101    0.2126
               -0.0474    0.9879   -0.1480    0.1404];
    Trobot{4} = [0.1359    0.0114   -0.9907    0.3804
                0.9903   -0.0292    0.1355   -0.0442
               -0.0274   -0.9995   -0.0152    0.1125];
    orBodySetTransform(robot.id, Trobot{index});
    offset = 0;
else
    robot = RobotCreateBarrettWAM('BarrettWAM');
    offset = robot.handoffset;
end

probs.cage = orEnvCreateProblem('TaskCaging', robot.name);
Gfilename = fullfile(pwd,sprintf('wamscene_graspset%d.txt',index));
Gcfilename = fullfile(pwd,sprintf('/wamscene_graspcontactset%d.txt', index));

orRobotSetDOFValues(robot.id, preshape, offset+(0:(length(preshape)-1)));
probs.manip = orEnvCreateProblem('BaseManipulation', robot.name);

target.name = 'Cabinet';
target.id = orEnvGetBody(target.name);
if( index <= 3 )
    target.joints = 0;
    target.link = 2;
    P = [0:0.05:pi/2];
    P = [P; zeros(1,size(P,2))];
else
    target.joints = 1;
    target.link = 3;
    P = [0:0.05:pi/3];
    P = [zeros(1,size(P,2)); P];
end
