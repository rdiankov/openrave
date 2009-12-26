% orBodySetJointValues(bodyid, values, indices)
%
% Set the raw joint values of a body. If bodyid is a robot, sets the robot's
% joints ignoring its current active degrees of freedom. If a controller on
% the robot is running, this function might not have any effect. Instead
% use orRobotSetDOFValues
% indices [optional] - array specifying the indices to control


function [] = orBodySetJointValues(bodyid, values, indices)

L = max(size(values));
command_str = ['body_setjoints ' num2str(bodyid) ' ' num2str(L) ' ' sprintf('%f ', values)];

if( exist('indices', 'var') )
    command_str = [command_str ' ' sprintf('%d ', indices)];
end

out = orCommunicator(command_str);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active joint values');
%end
