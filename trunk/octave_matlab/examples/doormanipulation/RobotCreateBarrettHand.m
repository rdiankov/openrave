% creates the barrett hand robot

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
function robot = RobotCreateBarrettHand(name)

robot.id = orEnvGetBody(name);
robot.filename = 'robots/barretthand.robot.xml';

if( robot.id <= 0 )
    robot.id = orEnvCreateRobot(name, robot.filename);
end

robot.bUseNearestIK = 0;
robot.name = name;
robot.totaldof = orBodyGetDOF(robot.id);
robot.hand_type = 'barrett';
robot.preshapejoints = 3;
robot.palmdir = [0 0 1];

% joints that will be controlled by the grasper
% any joints not specified belong to the preshape
robot.handjoints = [0 1 2];
robot.open_config = [0.4 0.4 0.4];
robot.closed_config = [1.5 1.5 1.5];


robot.grasp.direction = 1:3;
robot.grasp.center = 4:6;
robot.grasp.roll = 7;
robot.grasp.standoff = 8;
robot.grasp.joints = 8+(1:robot.totaldof);
