% orBodySetJointTorques(bodyid, values, indices)
%
% Set the raw joint values of a body. If bodyid is a robot, sets the robot's
% joints ignoring its current active degrees of freedom. If a controller on
% the robot is running, this function might not have any effect. Instead
% use orRobotSetDOFTorques
% indices [optional] - array specifying the indices to control

function [] = orBodySetJointTorques(bodyid, values, add)

if( ~exist('add', 'var') )
    add=0;
end

L = max(size(values));
command_str = ['body_setjointtorques ' num2str(bodyid) ' ' num2str(add) ' ' num2str(numel(values)) ' ' sprintf('%f ', values)];
out = orCommunicator(command_str);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active joint values');
%end
