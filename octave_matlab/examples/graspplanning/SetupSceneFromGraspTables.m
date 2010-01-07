% [robot,scenedata] = SetupSceneFromGraspTables(scenefile,grasptablefile)

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
function [robot,scenedata] = SetupSceneFromGraspTables(scenefile,grasptablefile)
global probs

randomize = 1;
G = load(grasptablefile);

scenedata = [];
scenedata.GraspTable = transpose(G.GraspTable);
scenedata.dests = [];

orEnvLoadScene(scenefile,1);

%% find all the bodies to manipulate
bodies = orEnvGetBodies();
scenedata.targetobjs = {};
for i = 1:length(bodies)
    if( ~isempty(findstr(bodies{i}.filename,G.targetfilename)) )
        scenedata.targetobjs{end+1} = bodies{i};
        if( randomize )
            %% move randomly
            Tbody = reshape(orBodyGetTransform(bodies{i}.id),[3 4]);
            for iter = 1:5
                Tnew = Tbody;
                Tnew(1:2,4) = Tnew(1:2,4) + [-0.1;-0.1]+rand(2,1).*[0.2;0.2];
                orBodySetTransform(bodies{i}.id, Tnew);
                if( ~orEnvCheckCollision(bodies{i}.id) )
                    Tbody = Tnew;
                    break;
                end
            end
            orBodySetTransform(bodies{i}.id, Tbody);
        end
    end
end

%% find the robot
robots = orEnvGetRobots();
robot = robots{1};
robot.manips = orRobotGetManipulators(robot.id);

probs.task = orEnvCreateProblem('TaskManipulation',robot.name);
if( isempty(probs.task) )
    warning('failed to create TaskManipulation problem');
end

probs.manip = orEnvCreateProblem('BaseManipulation',robot.name);
if( isempty(probs.manip) )
    warning('failed to create BaseManipulation problem');
end

robot.grasp = G.robot.grasp;
robot.graspavoidlinks = G.robot.avoidlinks;

robot.activemanip = 1;
robot.manip = robot.manips{robot.activemanip};
orProblemSendCommand(['setactivemanip ' robot.manip.name],probs.manip);

scenedata.home = orBodyGetJointValues(robot.id);

%% randomize robot position
if( randomize )
    Trobot = reshape(orBodyGetTransform(robot.id),[3 4]);    
    while(1)
        Tnew = Trobot;
        Tnew(1:2,4) = Tnew(1:2,4) + [-0.1;-0.1]+rand(2,1).*[0.2;0.2];
        orBodySetTransform(robot.id, Tnew);
        if( ~orEnvCheckCollision(robot.id) )
            break;
        end
    end
end

tablepattern = '^table$';
tableid = [];
for i = 1:length(bodies)
    if( regexp(bodies{i}.name, tablepattern) )
        tableid = bodies{i}.id;
        break;
    end
end

if( isempty(tableid) )
    warning('cannot find talbe to put ojbects on');
end

disp('Searching for destinations');
Tidentity = eye(4);
Ttable = reshape(orBodyGetTransform(tableid),[3 4]);
orBodySetTransform(tableid,Tidentity(1:3,1:4));
ab = orBodyGetAABB(tableid);
orBodySetTransform(tableid,Ttable);

%% limit extents
ab(1,2) = min(ab(1,2),0.2);
ab(2,2) = min(ab(2,2),0.2);

%% table up is assumed to be +z, sample the +y axis of the table
Nx = floor(2*ab(1,2)/0.1);
Ny = floor(2*ab(2,2)/0.1);
X = [];
Y = [];
for x = 0:(Nx-1)
    X = [X 0.5*rand(1,Ny)/(Nx+1) + (x+1)/(Nx+1)];
    Y = [Y 0.5*rand(1,Ny)/(Ny+1) + ([0:(Ny-1)]+0.5)/(Ny+1)];
end

offset = [ab(1,1)-ab(1,2);ab(2,1)-ab(2,2); ab(3,1)+ab(3,2)];
trans = [offset(1)+2*ab(1,2)*X; offset(2)+2*ab(2,2)*Y; repmat(offset(3),size(X))];

targetid = orEnvCreateKinBody('__testbody__',G.targetfilename);
%% all transforms should be relative to the current transformation of the body
Torg = [reshape(orBodyGetTransform(targetid),[3 4]); 0 0 0 1];
scenedata.dests = [];
for i = 1:size(trans,2);
    for roll = 0:pi/2:1.5*pi
        T = Ttable*[RotationMatrixFromAxisAngle([0;0;roll]) trans(:,i); 0 0 0 1]*Torg;
        orBodySetTransform(targetid,T);
        if( ~orEnvCheckCollision(targetid) )
            scenedata.dests = [scenedata.dests T(:)];
        end
    end
    for roll = [pi/2 pi 1.5*pi]
        T = Ttable*[RotationMatrixFromAxisAngle([roll;0;0]) trans(:,i); 0 0 0 1]*Torg;
        orBodySetTransform(targetid,T);
        if( ~orEnvCheckCollision(targetid) )
            scenedata.dests = [scenedata.dests T(:)];
        end
    end
end
orBodyDestroy(targetid);
