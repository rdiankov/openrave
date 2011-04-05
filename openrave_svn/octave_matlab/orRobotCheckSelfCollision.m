% dof = orRobotCheckSelfCollision(robotid)
%
% returns the robot's active degrees of freedom used for planning (not necessary corresponding to joints)

function dof = orRobotCheckSelfCollision(robotid)
out = orCommunicator(['robot_checkselfcollision ' num2str(robotid)], 1);
if(strcmp('error',sscanf(out,'%s',1)))
    error(['Unknown robot id: ' num2str(robotid)]);
end

dof = str2double(out);
