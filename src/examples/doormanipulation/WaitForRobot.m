% if transform_offset is specified, will return with success = 0
% otherwise will wait until the robot is done and will return with success = 1
function success = WaitForRobot(robotid)

global autorobot

if( ~exist('autorobot','var') | isempty(autorobot) | autorobot == 0 )
    disp('pausing');
    %pause;
    success = 1;
    return;
end

disp('waiting for robot');
success = 1;
dowait = 1;
pause(0.8); % always give a little time

while(dowait == 1 & (orEnvWait(robotid, 1) == 0) )
    pause(0.01);
end

disp('robot done');
