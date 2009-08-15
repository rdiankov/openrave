% MakeBarrettHandTables()
%
% Makes Barrett Hand caging tables

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
function MakeBarrettHandTables()

global probs
addopenravepaths_door();

% grasp generation for barrett hand
orEnvSetOptions('publishanytime 1');

for index = 1:4
    disp(sprintf('index: %d',index));
    [robot, P, Gfilename, Gcfilename, target, preshape] = SetupWAMScene(1,index);
    setsize = 2000;
    stepsize = 0.003;

    [G, contact] = GenerateGraspSet(robot, setsize, stepsize, target.name, target.link-1, target.joints);
    Gcontact = G(:,find(contact==2^2^length(target.joints)-1));
    Gpruned = PruneGraspSet(G, 200, 0.03^2); % for visualization
    Gcpruned = PruneGraspSet(Gcontact, 100, 0.03^2); % for visualization

    Gpruned = Gpruned';
    Gcpruned = Gcpruned';
    save('-ascii', Gfilename,'Gpruned');
    save('-ascii', Gcfilename,'Gcpruned');
end
