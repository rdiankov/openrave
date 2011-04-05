% if transform_offset is specified, will return with success = 0
% otherwise will wait until the robot is done and will return with success = 1

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
function success = WaitForRobot(robotid)

global autorobot

if( ~exist('autorobot','var') | isempty(autorobot) | autorobot == 0 )
    disp('pausing');
    %pause;
    success = 1;
    return;
end

disp('waiting for robot');
success = 1;
dowait = 1;
pause(0.8); % always give a little time

while(dowait == 1 & (orEnvWait(robotid, 1) == 0) )
    pause(0.01);
end

disp('robot done');
