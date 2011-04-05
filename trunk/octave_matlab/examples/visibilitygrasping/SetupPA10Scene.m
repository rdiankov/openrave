% [robot, scenedata] = SetupPA10Scene(scene,realrobot,randomize)

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
robot.activemanip = 1;
robot.manips = orRobotGetManipulators(robot.id);
robot.CreateHandFn = [];%@RobotCreateSchunkHand;
robot.testhandname = 'testhand';
robot.dof = orBodyGetDOF(robot.id);
orRobotSensorSend(robot.id,0,'power','0')

SetupProblems(robot.name);
orBodySetJointValues (robot.id,0.03,7); % move the gripper

orRobotSensorSend(robot.id,0,'power','0');

SwitchModelPatterns = {};
% objind = 1;
% SwitchModelPatterns{objind}.pattern = '^ricebox(\d)+$';
% SwitchModelPatterns{objind}.fatfilename = 'data/riceboxf.kinbody.xml';
% objind = objind + 1;

scenedata.convexfile = fullfile(tempdir(),'pa10gripper_convex.mat');
scenedata.visibilityfile = fullfile(tempdir(),'cereal_visibility_pa10.mat');
scenedata.graspsetfile = fullfile(tempdir(),'cereal_grasps_pa10.mat');
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
