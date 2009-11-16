% [robot, scenedata] = SetupPA10Scene(scene,realrobot,randomize)
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

if( realrobot )
    scenedata.home = orBodyGetJointValues(robot.id);
else
    scenedata.home = orBodyGetJointValues(robot.id);
end

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
bodies = orEnvGetBodies(0,openraveros_BodyInfo().Req_Names());
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, tablepattern) )
        tableid = bodies{i}.id;
        break;
    end
end

if( ~isempty(tableid) )
    scenedata.dests = InsertRandomObjects(randomize,tableid,[scenedata.targetid robot.id],scenedata.targetid);
end
