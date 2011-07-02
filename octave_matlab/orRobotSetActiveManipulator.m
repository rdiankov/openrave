% orRobotSetActiveManipulator(robotid, manipname)
%
% robotid - unique id of the robot
% manipname - manipulator name

function [] = orRobotSetActiveManipulator(robotid, manipname)

command_str = ['robot_setactivemanipulator ' num2str(robotid) ' ' manipname];
out = orCommunicator(command_str);
%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active DOFs');
%end
