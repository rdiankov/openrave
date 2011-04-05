% GraspTable = MakeHRP2JSKHandTable(targetfilename, grasptablefilename)
%
% Makes grasp tables for a particular name and targetfilename combination.
% For example: MakeBarrettHandTable('mug1','data/mug1.kinbody.xml')
% 

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
function GraspTable = MakeHRP2JSKHandTable(targetfilename, grasptablefilename)

global probs
addopenravepaths_grasping();

thresh = 0;

% default parameters
if( ~exist('targetfilename','var') )
    targetfilename = 'data/box_frootloops.kinbody.xml';
end

% extract name from targetfilename
[tdir, name, text] = fileparts(targetfilename);

dots = strfind(name, '.');
if( ~isempty(dots) && dots(1) > 1)
    name = name(1:(dots(1)-1));
end

if( ~exist('grasptablefilename','var') )
    grasptablefilename = sprintf('grasptables/grasp_hrp2rhandjsk_%s.mat', name);
end

orEnvLoadScene('',1); % clear the scene

% setup the robot
robot = RobotCreateHand('TestHand','robots/hrp2rhandjsk.robot.xml');
robot.avoidlinks{1} = 'RWristCam';
probs.grasp = orEnvCreateProblem('GrasperProblem', robot.name);

preshapes = transpose([1.696]);

% setup the target
Target.name = name;
Target.filename = targetfilename;
Target.id = orEnvCreateKinBody(Target.name, Target.filename);

if( Target.id == 0 )
    error(['could not create body ' Target.filename]);
end

orBodySetTransform(Target.id, [0 0 0], [1 0 0 0]); % identity

standoffs = [0];
rolls = 0:pi/4:7/4*pi;
use_noise = 0;
stepsize = 0.015;
add_spherenorms = 1;

% start simulating grasps
[GraspTable, GraspStats] = MakeGraspTable(robot,Target,preshapes,standoffs,rolls,use_noise,stepsize,add_spherenorms);

% save the table
GraspTable = GraspTable(find(GraspStats(:,1) > 0),:);
save('-v6',grasptablefilename,'GraspTable','robot','targetfilename');

GraspTableSimple = GraspTable(:,[(end-11):end robot.grasp.joints]);
[d,n,e] = fileparts(grasptablefilename);
save('-ascii',fullfile(d,['simple_' n e]),'GraspTableSimple');
