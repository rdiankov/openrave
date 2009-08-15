% [G, contact] = GenerateGraspSet(robot, setsize, stepsize, targetname,
% targetlink, targetjoints, cagesides)
%
% generate a grasp set for a given target object
% G - 7xn grasp set
% Gcontact - 7xm contact grasp set

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
