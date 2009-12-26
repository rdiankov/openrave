% success = orRobotControllerSet(robotid, controllername, controllerargs)
%
% Sets a new robot controller and destroys the old.
% controllername - name used to query a controller
% controllerargs [optional] - the arguments to ControllerBase::Init

function success = orRobotControllerSet(robotid, controllername, controllerargs)

if( ~exist('controllerargs','var') )
    controllerargs = [];
end

out = orCommunicator(['robot_controllerset ' num2str(robotid) ' ' controllername ' ' controllerargs], 1);

if strcmp('error',sscanf(out,'%s',1))
    %this error can be ok sometimes
    disp(['Error setting controller']);
    success = 0;
else
    success = 1;
end
