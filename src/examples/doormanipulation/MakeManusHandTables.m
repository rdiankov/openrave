% MakeManusHandTables()
%
% Makes Manus Hand caging tables

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
function MakeManusHandTables()

global probs
addopenravepaths_door();

%% setup grasp generation for Manus
orEnvSetOptions('publishanytime 1');
[robot, P, Gfilename, Gcfilename, target, preshape] = SetupManusScene(1);
setsize = 5000;
stepsize = 0.002;
[G, contact] = GenerateGraspSet(robot, setsize, stepsize, target.name, target.link-1, target.joints);
Gcontact = G(:,find(contact==2^2^length(target.joints)-1));
Gpruned = PruneGraspSet(G, 300, 0.03^2); % for visualization
Gcpruned = PruneGraspSet(Gcontact, 100, 0.03^2); % for visualization

Gpruned = Gpruned';
Gcpruned = Gcpruned';
save('-ascii', Gfilename,'Gpruned');
save('-ascii', Gcfilename,'Gcpruned');
