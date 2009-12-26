% MakeBarrettHandTables()
%
% Makes Barrett Hand caging tables

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
