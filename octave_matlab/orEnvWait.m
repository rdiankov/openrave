% success = orEnvWait(robotid, robot_timeout)
%
% wait until all previously sent commands to matlab are finished.
% Since problems are meant to last for a long time orEnvWait waits
% until the problem's main function finishes.
%
% robotid - optional argument. If a robot id is specified, will wait until
% the robot finishes with its trajectory.
%
% robot_timeout (s) - function will return with success set to 0 if robot
% did not finish its commands by robot_timeout ms. If not specified, orEnvWait
% will not return until robot completes.

function success = orEnvWait(robotid, robot_timeout)

command_str = 'wait ';

if( exist('robotid', 'var') )
    command_str = [command_str, num2str(robotid)];
end

if( exist('robot_timeout', 'var') )
    command_str = [command_str, ' ' num2str(robot_timeout)];
end

out = orCommunicator(command_str, 1);

if(strcmp('error',sscanf(out,'%s',1)))
    error('Error waiting for robot');
end

success = str2double(out);