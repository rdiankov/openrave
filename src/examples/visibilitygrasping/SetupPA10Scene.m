% [robot, scenedata] = SetupPA10Scene(scene,realrobot,randomize)

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
function [robot, scenedata] = SetupPA10Scene(scene,randomize)
global updir probs robothome
if( ~exist('randomize','var') )
    randomize = [];
end

scenedata = [];

orEnvLoadScene('', 1);
pause(1);
while(1)
    orEnvLoadScene(scene);
    robots = orEnvGetRobots();
    if( length(robots) > 0 )
        break;
    end
    warning('no robots in scene');
end

robot = robots{1};
robot.CreateHandFn = [];%@RobotCreateSchunkHand;
robot.testhandname = 'testhand';

SetupProblems(robot.name);
orBodySetJointValues (robot.id,0.03,7); % move the gripper

orRobotSensorSend(robot.id,0,'power','0');

SwitchModelPatterns = {};
% objind = 1;
% SwitchModelPatterns{objind}.pattern = '^ricebox(\d)+$';
% SwitchModelPatterns{objind}.fatfilename = 'data/riceboxf.kinbody.xml';
% objind = objind + 1;

scenedata.convexfile = fullfile(pwd,'pa10gripper_convex.mat');
scenedata.visibilityfile = fullfile(pwd,'cereal_visibility_pa10.mat');
scenedata.graspsetfile = fullfile(pwd,'cereal_grasps_pa10.mat');
load('pa10_cereal.mat');
save('-ascii',scenedata.convexfile,'gripper_convex');
save('-ascii',scenedata.visibilityfile,'visibility');
save('-ascii',scenedata.graspsetfile,'grasps');

scenedata.targetid = orEnvGetBody ('frootloops');
scenedata.FindTarget = @() FindTarget('^frootloops');
[scenedata.targetname,scenedata.targetid] = scenedata.FindTarget();
scenedata.SwitchModelPatterns = SwitchModelPatterns;
scenedata.dests = [];
scenedata.home = orBodyGetJointValues(robot.id);

%% randomize robot position
if( ~isempty(randomize) )
    Trobot = reshape(orBodyGetTransform(robot.id),[3 4]);    
    while(1)
        Tnew = Trobot;
        Tnew(1:2,4) = Tnew(1:2,4) + [-0.1;-0.5]+rand(2,1).*[0.3;1];
        orBodySetTransform(robot.id, Tnew);
        if( ~orEnvCheckCollision(robot.id) )
            break;
        end
    end
end

tablepattern = '^table$';
tableid = [];
bodies = orEnvGetBodies();
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, tablepattern) )
        tableid = bodies{i}.id;
        break;
    end
end

if( ~isempty(tableid) )
    scenedata.dests = InsertRandomObjects(randomize,tableid,[scenedata.targetid robot.id],scenedata.targetid);
end
