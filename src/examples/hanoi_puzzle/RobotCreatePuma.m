% robot = RobotCreatePuma(name)
%
% create and intiialize a puma robot object structure

% by Rosen Diankov (rdiankov@cs.cmu.edu)
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
