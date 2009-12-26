% orRobotSetDOFValues(robotid, values, indices)
%
% Sets the DOF values of the robot
% robotid - unique id of the robot
% values - the joint values of the robot
% indices [optional] - the indices of the dofs to set of the robot. 
%                      If indices is not specified the active degrees of freedom
%                      set by previous calls to orRobotSetActiveDOFs will be used.
%                      Note that specifying indices will not change the active dofs
%                      of the robot.

function [] = orRobotSetDOFValues(robotid, values, indices)

command_str = ['robot_setdof ' num2str(robotid) ' ' num2str(length(values)) ' ' sprintf('%f ', values)];
if( exist('indices', 'var') )
    command_str = [command_str ' ' sprintf('%d ', indices)];
end

out = orCommunicator(command_str);
%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active DOFs');
%end