% success = hanoimove(robot, diskname, radius, srcpeg, destpeg, height)
%
% perform one atomic pick-and-place operation with the arm

% by Rosen Diankov (rdiankov@cs.cmu.edu)
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
