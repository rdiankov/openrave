% [G, contact] = GenerateGraspSet(robot, setsize, stepsize, targetname,
% targetlink, targetjoints, cagesides)
%
% generate a grasp set for a given target object
% G - 7xn grasp set
% Gcontact - 7xm contact grasp set

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
%
% Functions to plan with caging grasps. See
% Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
% Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
function [G, contact] = GenerateGraspSet(robot, setsize, stepsize, targetname, targetlink, targetjoints, cagesides)
global probs

cagedconfig = 0.1;
exploreprob = 0.2;

cmd = ['graspset step ' num2str(stepsize) ' size ' num2str(setsize) ...
    ' target ' targetname ' ' num2str(targetlink) ...
    ' contactconfigdelta 0.01 cagedconfig ' num2str(cagedconfig) ...
    ' exploreprob ' num2str(exploreprob)];
for i = 1:length(targetjoints)
    cmd = [cmd ' targetjoint ' num2str(targetjoints(i))];
end
if( exist('cagesides','var') )
    cmd = [cmd ' cagesides ' sprintf('%d ', cagesides)]
end

tic;
resp = orProblemSendCommand(cmd,probs.cage);
toc

grasps = sscanf(resp, '%f');
grasps = reshape(grasps, [8 length(grasps)/8]);
G = grasps(1:7,:);
contact = grasps(8,:);
