% out = orRobotSensorConfigure(robotid, sensorindex, sensorcmd)
%
% sends a command to a sensor attached to the robot
% OpenRAVE sends directly to SensorBase::Configure,
%
% robotid - unique id of the robot
% sensorindex - zero-based index of sensor into robot's attached sensor array
% sensorcmd - One of the SensorBase::CC_X. specify only the 'X' part of the enum like PowerOn
function sout = orRobotSensorConfigure(robotid, sensorindex, sensorcmd)
if( ~exist('args','var') )
    args = [];
end
sout = orCommunicator(['robot_sensorconfigure ' sprintf('%d ', robotid, sensorindex) ' ' sensorcmd], 1);
