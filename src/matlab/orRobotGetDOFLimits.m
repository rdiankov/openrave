% values = orRobotGetDOFLimits(robotid)
%
% Gets the robot's dof limits in a Nx2 vector where N is the DOF, the first column
% is the low limit and the second column is the upper limit

function values = orRobotGetDOFLimits(robotid)

out = orCommunicator(['robot_getlimits ' num2str(robotid)], 1);
if(strcmp('error',sscanf(out,'%s',1)))
    error(['Unknown robot id: ' num2str(robotid)]);
end

values = sscanf(out,'%f');
values = reshape(values(2:end),values(1),2);
