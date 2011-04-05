% data = RunGrasp(robot, grasp, Target, noise, mu, affinedof)
%

% Copyright (C) 2008-2010 Rosen Diankov, Dmitry Berenson
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
function [data,Thand] = RunGrasp(robot, grasp, Target, noise, mu, affinedof)
global probs

data = [];

if( ~exist('affinedof','var') )
    affinedof = 7; % X,Y,Z
end

orBodySetJointValues(robot.id,grasp(robot.grasp.joints));
orRobotSetActiveDOFs(robot.id, robot.handjoints, affinedof);

command_string = ['grasp forceclosure 1 direction ' sprintf('%f ', grasp(robot.grasp.direction)) ' body ' Target.name ' '];

if( exist('noise','var') && noise )
    command_string =  [command_string, ' noise 0.1 0.005 '];
end
if( exist('mu', 'var') && mu > 0 )
    command_string = [command_string, ' friction ', num2str(mu)];
end

command_string = [command_string, ' roll ' num2str(grasp(robot.grasp.roll))];
command_string = [command_string, ' standoff ' num2str(grasp(robot.grasp.standoff))];
command_string = [command_string, ' centeroffset ' sprintf('%f ', grasp(robot.grasp.center))];
%command_string = [command_string, ' collision pqp ' ];
for i = 1:length(robot.avoidlinks)
    command_string = [command_string, 'avoidlink ' robot.avoidlinks{i}];
end

disp(command_string);
data = orProblemSendCommand(command_string, probs.grasp);
orEnvWait(robot.id);
Thand = inv(robot.Tgrasp) * [reshape(orBodyGetTransform(robot.id),[3 4]); 0 0 0 1];

