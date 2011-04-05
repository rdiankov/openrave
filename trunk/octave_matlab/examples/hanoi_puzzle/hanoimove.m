% success = hanoimove(robot, diskname, radius, srcpeg, destpeg, height)
%
% perform one atomic pick-and-place operation with the arm

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
function success = hanoimove(robot, diskname, radius, srcpeg, destpeg, height)
global probs

orRobotSetDOFValues(robot.id,-0.7, robot.handoffset);
diskid = orEnvGetBody(diskname);

success = 1;
Tdisk = reshape(orBodyGetTransform(diskid),[3 4]);

% a grasp is parameterized by its orientation around the peg and its tilt with respect to the up vector (y-axis)
for ang2 = -pi/2:0.4:(1.5*pi)
    for ang1 = -0.8:0.2:0
        % get the grasp transform given the two angles
        grasps = GetGrasp(Tdisk, radius, [ang1 ang2]);

        for i = 1:length(grasps) % for each of the grasps
            % move the hand to that location
            if( MoveToHandPosition(grasps{i}) )
                % succeeded so grab the disk
                orEnvWait(robot.id);
                orProblemSendCommand('closefingers',probs.manip);
                orEnvWait(robot.id);
                orProblemSendCommand(['grabbody name ' diskname],probs.manip);

                % try to pub the disk in the destination peg
                didputblock = putblock(robot, diskid, srcpeg, destpeg, height);

                if( didputblock )
                    % succeeded so release the disk
                    orEnvWait(robot.id); % wait for robot to complete all trajectories
                    orProblemSendCommand('releaseall',probs.manip);
                    orRobotSetDOFValues(robot.id,-0.7, robot.handoffset);
                    return;
                end
                % open hand and try a different grasp
                orProblemSendCommand('releaseall',probs.manip);
                orRobotSetDOFValues(robot.id,-0.7, robot.handoffset);    
            end
        end
    end
end

success = 0;
disp('failed to find grasp');

% returns the transform of the grasp given its orientation and the location/size of the disk
function T = GetGrasp(Tdisk, radius, angles)

ydir = -Tdisk(:,1:3)*[cos(angles(1))*cos(angles(2)); -sin(angles(1)); cos(angles(1))*sin(angles(2))];
pos = Tdisk(:,4) + radius*Tdisk(:,1:3)*[cos(angles(2)); 0; sin(angles(2))];
xdir = cross(Tdisk(1:3,2),ydir); xdir = xdir ./ norm(xdir);
zdir = cross(xdir,ydir);

T{1} = [xdir ydir zdir pos];
T{2} = [xdir ydir zdir pos] * [rodrigues([0 pi 0]) [0;0;0]; 0 0 0 1];
