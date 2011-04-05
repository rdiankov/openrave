% robot = RobotCreatePuma(name)
%
% create and intiialize a puma robot object structure

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
function robot = RobotCreatePuma(name)

robot.id = orEnvGetBody(name);

if( robot.id <= 0 )
    % create a new robot
    robot.filename = 'robots/puma.robot.xml';
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.handtype = [];
robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);
robot.armjoints = 0:5;
robot.shoulderid = 1;
robot.handoffset = 6;
robot.wristlinkid = 7;

robot.ikoffset = [0 0.175 0]; %?

robot.CreateHandFn = @RobotCreatePumaHand;
robot.testhandname = 'TestPumaHand';
robot.ikreachability = ''; % ik reachability file

function robot = RobotCreatePumaHand(name)

robot.id = orEnvGetBody(name);
if( robot.id <= 0 )
    robot.id = orEnvCreateRobot(name, 'robots/pumagripper.robot.xml');
end

robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);
robot.hand_type = 'puma';
robot.preshapejoints = [];
robot.palmdir = [0 1 0];

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.handjoints = 0;
robot.open_config = -0.7;
robot.closed_config = 0.1;


robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
