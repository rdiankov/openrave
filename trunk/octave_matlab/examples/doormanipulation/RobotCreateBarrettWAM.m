% robot = RobotCreateBarrettWAM(name, use4dofarm)
%
% fills a robot structure with info about the robot, if the robot
% exists in the environment, creates it. otherwise 
% use4dofarm must be specified if robot doesn't exist in the current world

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
function robot = RobotCreateBarrettWAM(name, use4dofarm, filename)

if( ~exist('use4dofarm', 'var') )
    use4dofarm = 0;
end
if( ~exist('mobile', 'var') )
    mobile = 0;
end

robot.id = orEnvGetBody(name);
    
if( robot.id <= 0 )
    % create a new robot
    if( exist('filename','var') )
        robot.filename = filename;
    else
        if( use4dofarm )
            robot.filename = 'robots/barrettwam4.robot.xml';
        else
            robot.filename = 'robots/barrettwam.robot.xml';
        end
    end
    robot.filename
    robot.id = orEnvCreateRobot(name, robot.filename);    
else
    % fill use4dof arm depending on the current robot
    if( orBodyGetDOF(robot.id) == 8 )
        use4dofarm = 1;
    end
end

manips = orRobotGetManipulators(robot.id);

if( length(manips) == 0 )
    error('robot does not have any manipulators');
end

robot.armjoints = manips{1}.armjoints;
if( length(manips{1}.joints) > 0 )
    robot.handoffset = manips{1}.joints(1);
else
    robot.handoffset = robot.armjoints(end)+1; % offset of hand joint indices
end
robot.tgrasp = [manips{1}.Tgrasp; 0 0 0 1];

robot.name = name;
robot.shoulderid = manips{1}.baselink+1;
robot.wristlinkid = manips{1}.eelink+1;
robot.totaldof = orBodyGetDOF(robot.id);

robot.CreateHandFn = @RobotCreateBarrettHand;
robot.testhandname = 'TestHand';
