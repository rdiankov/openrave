% robot = RobotCreateManus(name, right)

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
function robot = RobotCreateManus(name, right)

robot.id = orEnvGetBody(name);

if( ~exist('right','var') )
    right = 0;
end

if( robot.id <= 0 )
    % create a new robot
    if( right )
        robot.filename = 'robots/manusarm_right.robot.xml';
    else
        robot.filename = 'robots/manusarm_left.robot.xml';
    end
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.right = right;
robot.name = name;
robot.shoulderid = 1;
robot.totaldof = orBodyGetDOF(robot.id);

offset = 0;
robot.armjoints = [0:6]+offset;
robot.handoffset = 6+offset; % offset of hand joint indices
robot.wristlinkid = 7+offset;

robot.tgrasp = eye(4);

robot.CreateHandFn = @RobotCreateManusHand;
robot.testhandname = 'ManusHand';
robot.ikreachability = ''; % ik reachability file
