% success = putblock(robot, diskid, srcpeg, destpeg, height)
%
% Given that the arm is grasping a disk, puts the disk onto another peg
% a peg's axis is always parallel to its local y axis

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

function success = putblock(robot, diskid, srcpeg, destpeg, height)
global probs

srcpegbox = orBodyGetAABB(srcpeg);
destpegbox = orBodyGetAABB(destpeg);

% get all the transformations
success = 1;
L = orBodyGetLinks(robot.id);
Thand = reshape(L(:,robot.wristlinkid), [3 4]);
Tdisk = reshape(orBodyGetTransform(diskid),[3 4]);
Tsrcpeg = reshape(orBodyGetTransform(srcpeg),[3 4]);
Tpeg = reshape(orBodyGetTransform(destpeg),[3 4]);

src_upvec = Tsrcpeg(:,2);
dest_upvec = Tpeg(:,2);

% the IK grasp system is [0 0.175 0] from the center of rotation
Tgrasp = eye(4); Tgrasp(2,4) = 0.175;
Tdiff = inv([Tdisk; 0 0 0 1])*[Thand; 0 0 0 1]*Tgrasp;

% iterate across all possible orientations the destination peg can be in
for ang = -pi:0.3:pi
    % find the dest position
    p = Tpeg(:,4) + height * Tpeg(:,2);
    R = Tpeg(1:3,1:3)*rodrigues([0 ang 0]);
    T = [R p; 0 0 0 1]*Tdiff;
    T = T(1:3,:);

    % check the IK of the destination
    s = orProblemSendCommand(['iktest trans ' sprintf('%f ',T(10:12)) ' rot ' sprintf('%f ',T(1:9))],probs.manip)
    if( isempty(s) )
        % empty so continue
        continue;
    end

    % add two intermediate positions, one right above the source peg
    % and one right above the destination peg
    Tnewhand = Thand*Tgrasp; Tnewhand(:,4) = Tnewhand(:,4) + src_upvec*(max(srcpegbox(:,2))*2.5-0.02);
    % check the IK of the destination
    s = orProblemSendCommand(['iktest trans ' sprintf('%f ',Tnewhand(10:12)) ' rot ' sprintf('%f ',Tnewhand(1:9))],probs.manip)
    if( isempty(s) )       
        % empty so continue
        'Tnewhand invalid'
        continue;
    end

    Tnewhand2 = T; Tnewhand2(:,4) = Tnewhand2(:,4) + dest_upvec*(max(destpegbox(:,2))*2.5-height);
    % check the IK of the destination
    s = orProblemSendCommand(['iktest trans ' sprintf('%f ',Tnewhand2(10:12)) ' rot ' sprintf('%f ',Tnewhand2(1:9))],probs.manip)
    if( isempty(s) )
        % empty so continue
        'Tnewhand2 invalid'
        continue;
    end

    if( ~MoveToHandPosition(Tnewhand) )
        disp('failed to move to position above source peg');
        %orRobotSetDOFValues(robot.id,sscanf(s,'%f '),robot.armjoints);
        continue;
    end
    orEnvWait(robot.id); % wait for robot to complete all trajectories

    if( ~MoveToHandPosition(Tnewhand2) )
        disp('failed to move to position above dest peg')
        %orRobotSetDOFValues(robot.id,sscanf(s,'%f '),robot.armjoints);
        continue;
    end
    orEnvWait(robot.id); % wait for robot to complete all trajectories

    if( ~MoveToHandPosition(T) )
        disp('failed to move to dest peg');
        %orRobotSetDOFValues(robot.id,sscanf(s,'%f '),robot.armjoints);
        continue;
    end
    orEnvWait(robot.id); % wait for robot to complete all trajectories

    return;
end

disp('failed to put block');
success = 0;
