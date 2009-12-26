% success = orRobotControllerSend(robotid, controllercmd)
%
% sends a command to the current controller the robot is connected to.
% OpenRAVE sends directly to ControllerBase::SendCmd,
% ControllerBase::SupportsCmd is also used to check for support.
%
% success - 1 if command was accepted, 0 if not

function success = orRobotControllerSend(robotid, controllercmd)

out = orCommunicator(['robot_controllersend ' num2str(robotid) ' ' controllercmd], 1);

if strcmp('error',sscanf(out,'%s',1))
    %this error can be ok sometimes
    disp(['Error executing command "' controllercmd '" on controller']);
end

success = str2double(out);
