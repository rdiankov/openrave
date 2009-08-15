% [success, startgoal, trajname, time, bodytrajname] =
% RandomizedRelaxedPlanner(targetgoal,G,Gcontact,target, extraopts)
%
% Plans with relaxed task constraints using Randomized A*

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
