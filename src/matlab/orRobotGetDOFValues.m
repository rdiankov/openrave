% values = orRobotGetDOFValues(robotid, indices)
%
% Gets the robot's dof values in a Nx1 vector where N is the DOF
% robotid - unique id of the robot
% indices [optional]- The indices of the joints whose values should be returned.
%                     If not specified, the active degreees of freedeom set by
%                     orRobotSetActiveDOFs will be used.

function values = orRobotGetDOFValues(robotid, indices)


command_str = ['robot_getdofvalues ' num2str(robotid)];
if( exist('indices', 'var') )
    command_str = [command_str ' ' sprintf('%d ', indices)];
end

out = orCommunicator(command_str, 1);
if(strcmp('error',sscanf(out,'%s',1)))
    error('Error getting DOF Values');
end

values = sscanf(out, '%f');
