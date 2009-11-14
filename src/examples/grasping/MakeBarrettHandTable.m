% GraspTable = MakeBarrettHandTable(targetfilename, grasptablefilename)
%
% Makes grasp tables for a particular name and targetfilename combination.
% For example: MakeBarrettHandTable('mug1','data/mug1.kinbody.xml')
% 

% Copyright (C) 2008 Rosen Diankov (rdiankov@cs.cmu.edu), Dmitry Berenson
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function GraspTable = MakeBarrettHandTable(targetfilename, grasptablefilename)

global probs
addopenravepaths_grasping();

thresh = 0;

% default parameters
if( ~exist('targetfilename','var') )
    targetfilename = 'data/mug1.kinbody.xml';
end

% extract name from targetfilename
[tdir, name, text] = fileparts(targetfilename);

dots = strfind(name, '.');
if( ~isempty(dots) && dots(1) > 1)
    name = name(1:(dots(1)-1));
end

if( ~exist('grasptablefilename','var') )
    grasptablefilename = sprintf('grasptables/grasp_barrett_%s.mat', name);
end

orEnvLoadScene('',1); % clear the scene

% setup the robot
robot = RobotCreateBarrettHand('TestHand');
probs.grasp = orEnvCreateProblem('GrasperProblem', robot.name);

% 3 preshapes
preshapes = transpose([0.5 0.5 0.5 pi/3;
             0.5 0.5 0.5 0;
             0 0 0 pi/2]);

% setup the target
Target.name = name;
Target.filename = targetfilename;
Target.id = orEnvCreateKinBody(Target.name, Target.filename);

if( Target.id == 0 )
    error(['could not create body ' Target.filename]);
end

orBodySetTransform(Target.id, [0 0 0], [1 0 0 0]); % identity

% start simulating grasps
[GraspTable, GraspStats] = MakeGraspTable(robot,Target,preshapes);

% save the table
GraspTable = GraspTable(find(GraspStats(:,1) > 0),:);
save('-v6',grasptablefilename,'GraspTable','robot','targetfilename');

GraspTableSimple = GraspTable(:,[(end-11):end robot.grasp.joints]);
[d,n,e] = fileparts(grasptablefilename);
save('-ascii',fullfile(d,['simple_' n e],'GraspTableSimple');
