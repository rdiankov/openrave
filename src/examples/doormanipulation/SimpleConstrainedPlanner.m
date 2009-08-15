% [success, startgoal, trajname, time, bodytrajname] = SimpleConstrainedPlanner(targettraj,G,Gcontact,target,extraopts)

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
