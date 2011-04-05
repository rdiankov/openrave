% robot = RobotCreatePerMMA(name, right)

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
function robot = RobotCreatePerMMA(name, right, onchair)

global robotsdir
robot.id = orEnvGetBody(name);

if( ~exist('onchair','var') )
    onchair = 0;
end

if( ~exist('right','var') )
    right = 0;
end

if( robot.id <= 0 )
    % create a new robot
    robot.filename = 'robots/permma_left.robot.xml';
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.right = right;
robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);

if( onchair )
    offset = 2;
else
    offset = 0;
end

manips = orRobotGetManipulators(robot.id);
if( length(manips) < right+1 )
    error('robot does not have correct manipulator');
end
manip = manips{right+1};

robot.armjoints = manip.armjoints;
if( length(manips{1}.joints) > 0 )
    robot.handoffset = manip.joints(1);
else
    robot.handoffset = robot.armjoints(end)+1; % offset of hand joint indices
end
robot.tgrasp = [manip.Tgrasp; 0 0 0 1];
robot.shoulderid = manip.baselink+1;
robot.wristlinkid = manip.eelink+1;
robot.handoffset = robot.armjoints(end)+1; % offset of hand joint indices

robot.CreateHandFn = @RobotCreateManusHand;
robot.testhandname = 'ManusHand';
