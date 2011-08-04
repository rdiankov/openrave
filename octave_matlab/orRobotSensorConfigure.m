% out = orRobotSensorConfigure(robotid, sensorindex, sensorcmd)
%
% sends a command to a sensor attached to the robot
% OpenRAVE sends directly to SensorBase::SendCmd,
% SensorBase::SupportsCmd is used to check for command support.
%
% robotid - unique id of the robot
% sensorindex - zero-based index of sensor into robot's attached sensor array
% out - the output of the command
function sout = orRobotSensorConfigure(robotid, sensorindex, sensorcmd)
if( ~exist('args','var') )
    args = [];
end
sout = orCommunicator(['robot_sensorconfigure ' sprintf('%d ', robotid, sensorindex) ' ' sensorcmd], 1);
