% [success, startgoal, trajname, time, bodytrajname] = SimpleConstrainedPlanner(targettraj,G,Gcontact,target,extraopts)

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
function [success, startgoal, trajname, time, bodytrajname] = SimpleConstrainedPlanner(targettraj,G,Gcontact,target,extraopts)

global probs

if( ~exist('extraopts','var') )
    extraopts = '';
end

traj = [];
time = 0;
success = 0;
startgoal = [];

curdir = pwd;
trajname = [curdir '/constrainedtraj.txt'];
bodytrajname = [curdir '/bodytraj.txt'];
configthresh = 0.2;

cmd = ['simpleconstraintplan fullcol target ' target.name ' targetlink ' num2str(target.link-1) ...
    ' savetraj ' trajname '; savebodytraj ' bodytrajname  ';' ...
    ' configthresh ' num2str(configthresh) ' targettraj ' num2str(size(targettraj,2)) ...
    ' ' sprintf('%f ', targettraj(:)) ' graspset ' Gcontact ';'];
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
