% values = orBodyGetJointValues(bodyid, indices)
%
% Gets the body's dof values in a Nx1 vector where N is the DOF
% bodyid - unique id of the robot
% indices [optional]- The indices of the joints whose values should be returned.
%                     If not specified, all joints are returned
function values = orBodyGetJointValues(bodyid, indices)

command_str = ['body_getjoints ' num2str(bodyid)];
if( exist('indices', 'var') )
    command_str = [command_str ' ' sprintf('%d ', indices)];
end

out = orCommunicator(command_str, 1);
% if(strcmp('error',sscanf(out,'%s',1)))
%     error('Error orBodyGetJointValues');
% end

values = sscanf(out,'%f');
