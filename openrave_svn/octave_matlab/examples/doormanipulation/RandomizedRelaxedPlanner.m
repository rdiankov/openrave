% [success, startgoal, trajname, time, bodytrajname] =
% RandomizedRelaxedPlanner(targetgoal,G,Gcontact,target, extraopts)
%
% Plans with relaxed task constraints using Randomized A*

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
function [success, startgoal, trajname, time, bodytrajname] = RandomizedRelaxedPlanner(targetgoal,G,Gcontact,target, extraopts)

global probs

if( ~exist('extraopts','var') )
    extraopts = '';
end

traj = [];
success = 0;

curdir = pwd;
trajname = [curdir '/constrainedtraj.txt'];
bodytrajname = [curdir '/bodytraj.txt'];
startgoal = [];

graspthresh = 0.15;
configthresh = 0.5;
weights = [];
plannername = 'RA*';
goalcoeff = 100;
goalthresh = 0.05;
% puma
%distthresh = 0.04;
%sampleradius = 0.04;
%numchildren = 7;
% wam
distthresh = 0.08;
sampleradius = 0.04;
numchildren = 3;
maxiterations = 200;

maxsamples = 1;
maxikiterations = 10;

cmd = ['taskconstraintplan target ' target.name ' targetlink ' num2str(target.link-1) ...
    ' targetjoints ' num2str(length(target.joints)) ' ' sprintf('%d ', target.joints) ...
    ' planner ' plannername ' goalcoeff ' num2str(goalcoeff) ' maxiterations ' sprintf('%d ', maxiterations) ...
    ' goalthresh ' num2str(goalthresh) ' distthresh ' num2str(distthresh) ...
    ' numchildren ' sprintf('%d ', numchildren) ' sampleradius ' num2str(sampleradius) ...
    ' savetraj ' trajname '; savebodytraj ' bodytrajname '; graspthresh ' num2str(graspthresh) ...
    ' maxsamples ' sprintf('%d ', maxsamples) ' maxikiterations ' sprintf('%d ', maxikiterations) ...
    ' configthresh ' num2str(configthresh) ' targettraj ' num2str(size(targetgoal,2)) ' '  ...
    sprintf('%f ', targetgoal(:)) ' graspset ' G '; graspcontactset ' Gcontact ';'];
if( ~isempty(weights) )
    cmd = [cmd ' usegoal ' sprintf('%f ', weights)];
end

%cmd = [cmd ' features '  pwd '/features.txt'];

tic;
resp = orProblemSendCommand(cmd, probs.cage);
time = toc;

if( isempty(resp) )
    return;
end

success = 1;

resp = sscanf(resp, '%f ');
time = resp(1)/1000;
startgoal = reshape(resp(2:end), [(length(resp)-1)/2 2]);
