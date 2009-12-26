% rundoormanipulation(plannertype, scenetype)
%
% Various robots open doors/cupboards from random positions. The function will
% continuously sample new robot starting states.
% Arguments:
%   plannertype - 0 for relaxed randomized caging planner
%               - 1 for discrete caging planner
%               - 2 for simple constraint planner
% Output:
%   scenetype   - 0 for WAM scene with cupboard
%               - 1 for Manus arm scene with door
%
% Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
% Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.

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
function rundoormanipulation(plannertype, scenetype)

global probs
global autorobot
autorobot = 1;
addopenravepaths_door();

orEnvSetOptions('collision ode');

% use arms for manipulation
timestep = 0.005;
orEnvSetOptions('publishanytime 0');
orEnvSetOptions(sprintf('simulation start %f', timestep)); % make simulation slower

if( ~exist('plannertype','var') )
    plannertype = 0;
end
if( ~exist('scenetype','var') )
    scenetype = 0;
end

switch scenetype
    case 1
        [robot, P, Gfilename, Gcfilename, target, preshape] = SetupManusScene(0);
        Trobot = orBodyGetTransform(robot.id);
        minrange = [-1.1;Trobot(11);-1.4];
        maxrange = [0.75;Trobot(11);-0.6];
    otherwise
        [robot, P, Gfilename, Gcfilename, target, preshape] = SetupWAMScene(0,2);
        Trobot = orBodyGetTransform(robot.id);
        minrange = [-0.3;-0.3;Trobot(12)];
        maxrange = [0.2;0.25;Trobot(12)];
end

home = orBodyGetJointValues(robot.id);

plannertype = 1; %% for now force to discretized version
switch plannertype
    case 1
        disp('using discrete relaxed caging planner (slow, but succeeds often)');
        plannerfn = @TaskConstrainedPlanner;
        targetgoal = P;
    case 2
        disp('using discrete simple constrained planner (fast, but fails often)');
        plannerfn = @SimpleConstrainedPlanner;
        targetgoal = P;
    otherwise
        disp('using randomized relaxed caging planner (fast and succeeds often)');
        plannerfn = @RandomizedRelaxedPlanner;
        % as long as initial and goal are specified, any number of poitns
        % can be specified in the middle
        targetgoal = P(:,1:2:end);
end

while(1)
    disp('searching for random collision-free position');
    Trobot = orBodyGetTransform(robot.id);
    while(1)
        Trobot(10:12) = minrange+rand(3,1).*(maxrange-minrange);
        orBodySetTransform(robot.id,Trobot);

        if( ~orEnvCheckCollision(robot.id) )
            break;
        end
    end

    % go to target preshape
    handjoints = robot.handoffset+(0:(length(preshape)-1));
    orBodySetJointValues(robot.id,preshape,handjoints);

    % pick a start location
    disp('planning for opening a door');
    [success, startgoal, trajname, time, bodytrajname] = plannerfn(targetgoal,Gfilename,Gcfilename,target);

    if( success )
        disp(sprintf('success opening door planning time: %f', time));

        success = orProblemSendCommand(['MoveManipulator armvals ' sprintf('%f ', startgoal(robot.armjoints+1,1))], probs.manip);
        if( isempty(success) )
            disp('failed to plan to handle');
            continue;
        end

        orEnvWait(robot.id);

        % play the trajectory with the target object movement
        orEnvSetOptions('simulation stop');
        orProblemSendCommand(['traj sep ; ' trajname], probs.manip);
        orProblemSendCommand(['bodytraj sep ; target ' target.name ' targettraj ' bodytrajname], probs.cage);
        orEnvSetOptions(sprintf('simulation start %f', timestep));
        orEnvWait(robot.id);

        % go back to initial
        RobotGoInitial(robot,home,probs.manip);
        orBodySetJointValues(target.id,P(:,1));
    else
        disp(sprintf('failed planning time: %f', time));
    end

    disp('-----------------------------------');
end
