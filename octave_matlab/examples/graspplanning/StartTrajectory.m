% success = StartTrajectory(robotid, trajdata)
%
% Starts a trajectory on the real robot and waits for it, in the end sets the new robot values
% in the cloned (current) world.

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
function success = StartTrajectory(robotid,trajdata,timelimit)
global probs

if( isempty(trajdata) )
    success = 1;
    return;
end

if( ~exist('timelimit','var') )
    timelimit = 0;
end

trajsuc = orProblemSendCommand(['traj stream ' trajdata],probs.manip);
if( ~trajsuc )
    disp('trajectory failed');
    success = 0;
    return;
end

disp('waiting for robot');
success = 1;
dowait = 1;
pause(0.3); % pause a little to give a chance for controller to start

tic;
basetime = toc;
while(dowait == 1 & (orEnvWait(robotid, 0.05) == 0) )
    if( timelimit > 0 && toc-basetime > timelimit )
        success = 0;
        break;
    end
end

disp('wait ended');
newjointconfig = orBodyGetJointValues(robotid);
orBodySetJointValues(robotid,newjointconfig);
