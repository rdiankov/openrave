% out = orRobotSensorSend(robotid, sensorindex, controllercmd,args)
%
% sends a command to a sensor attached to the robot 
% OpenRAVE sends directly to SensorBase::SendCmd,
% SensorBase::SupportsCmd is used to check for command support.
%
% robotid - unique id of the robot
% sensorindex - zero-based index of sensor into robot's attached sensor array
% out - the output of the command
function sout = orRobotSensorSend(robotid, sensorindex, sensorcmd,args)
if( ~exist('args','var') )
    args = [];
end
sout = orCommunicator(['robot_sensorsend ' sprintf('%d ', robotid, sensorindex) ' ' sensorcmd ' ' args], 1);
