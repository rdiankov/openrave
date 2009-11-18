%% success = WaitForRobot(robotid)
%%
%% waits for the robot to complete its trajectory/become idlwe

% Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function success = WaitForRobot(robotid,timelimit)

if( ~exist('timelimit','var') )
    timelimit = 0;
end

disp('waiting for robot');
success = 1;
dowait = 1;
pause(0.5); % always give a little time
tic;

while(dowait == 1 & (orEnvWait(robotid, 1) == 0) )
    pause(0.01);
    if( timelimit > 0 && toc > timelimit )
        success = 0;
        return;
    end
end
